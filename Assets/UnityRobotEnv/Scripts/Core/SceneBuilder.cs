using System.Collections.Generic;
using UnityEngine;

namespace UnityRobotEnv.Core
{
    public enum ArenaType
    {
        Table,
        Kitchen,
        Floor,
        CoffeeTable,
        LivingRoom,
        Study
    }

    public enum RobotType
    {
        Panda,
        SO100
    }

    public class SceneBuilder
    {
        private struct PlacedInfo
        {
            public Vector3 pos;
            public float horizontalRadius;
            public float topY;
            public float bottomY;
        }

        private BDDL.BDDLProblem _problem;
        private Dictionary<string, ObjectState> _objectStates;
        private Dictionary<string, ObjectState> _fixtureStates;
        private Dictionary<string, ObjectState> _siteStates;
        private List<string> _objOfInterest;
        private List<PlacedInfo> _placedObjects = new List<PlacedInfo>();

        public BDDL.BDDLProblem Problem => _problem;
        public IReadOnlyDictionary<string, ObjectState> ObjectStates => _objectStates;
        public IReadOnlyDictionary<string, ObjectState> FixtureStates => _fixtureStates;
        public IReadOnlyDictionary<string, ObjectState> SiteStates => _siteStates;
        public List<string> ObjectsOfInterest => _objOfInterest;

        public SceneBuilder(BDDL.BDDLProblem problem)
        {
            _problem = problem;
            _objectStates = new Dictionary<string, ObjectState>();
            _fixtureStates = new Dictionary<string, ObjectState>();
            _siteStates = new Dictionary<string, ObjectState>();
            _objOfInterest = new List<string>(problem.ObjOfInterest);
        }

        public void Build(GameObject arenaRoot, GameObject objectParent)
        {
            LoadFixtures(arenaRoot);
            LoadObjects(objectParent);
            PlaceObjects(objectParent);
            GenerateObjectStateWrappers();
        }

        private void LoadFixtures(GameObject arenaRoot)
        {
            foreach (var kv in _problem.Fixtures)
            {
                string fixtureName = kv.Key;
                string category = kv.Value;

                var fixtureGo = CreatePrimitiveFromCategory(category, fixtureName);
                fixtureGo.transform.SetParent(arenaRoot.transform);
                fixtureGo.name = fixtureName;

                _fixtureStates[fixtureName] = new ObjectState(fixtureName, fixtureGo, isFixture: true);
            }
        }

        private void LoadObjects(GameObject objectParent)
        {
            foreach (var kv in _problem.Objects)
            {
                string objectName = kv.Key;
                string category = kv.Value;

                var objGo = CreatePrimitiveFromCategory(category, objectName);
                objGo.transform.SetParent(objectParent.transform);
                objGo.name = objectName;

                var state = new ObjectState(objectName, objGo, isFixture: false);
                string xmlPath = FindObjectXml(category);
                if (xmlPath != null)
                {
                    var siteData = ObjectLoader.ParsePlacementSites(xmlPath);
                    if (siteData.HasValue)
                    {
                        state.XmlBottomOffset = siteData.Value.BottomOffset;
                        state.XmlTopOffset = siteData.Value.TopOffset;
                        state.XmlHorizontalRadius = siteData.Value.HorizontalRadius;
                        state.HasXmlPlacementParams = true;
                    }
                }
                _objectStates[objectName] = state;
            }
        }

        public void RandomizeObjectPlacements(Transform objectParent = null)
        {
            foreach (var state in _problem.InitialState)
            {
                switch (state.Predicate)
                {
                    case BDDL.InitPredicateType.On:
                        PlaceOn(state, objectParent);
                        break;
                    case BDDL.InitPredicateType.In:
                        PlaceIn(state, objectParent);
                        break;
                }
            }
        }

        private void PlaceObjects(GameObject objectParent)
        {
            _placedObjects.Clear();
            Physics.SyncTransforms();
            RandomizeObjectPlacements(objectParent?.transform);
        }

    private void PlaceOn(BDDL.InitState state, Transform objectParent)
    {
        string objectName = state.ObjectName;
        string locationName = state.LocationName;

        ObjectState objState = GetState(objectName);
        if (objState == null) return;

        var region = _problem.Regions.TryGetValue(locationName, out var r) ? r : null;
        ObjectState targetState = null;

        if (region != null && !string.IsNullOrEmpty(region.Target))
            targetState = GetState(region.Target);
        else if (_fixtureStates.ContainsKey(locationName))
            targetState = _fixtureStates[locationName];
        else if (_objectStates.ContainsKey(locationName))
            targetState = _objectStates[locationName];

        if (targetState != null)
        {
            float surfaceY = GetSurfaceY(targetState.GameObject);

            float pivotToBottom = 0f;
            float horizontalRadius = 0.05f;
            float objTopY = 0.1f;

            if (objState.HasXmlPlacementParams)
            {
                pivotToBottom = objState.XmlBottomOffset;
                horizontalRadius = objState.XmlHorizontalRadius;
                objTopY = objState.XmlTopOffset;
            }
            else
            {
                var myCol = objState.GameObject.GetComponentInChildren<Collider>();
                if (myCol != null)
                {
                    Bounds b = myCol.bounds;
                    pivotToBottom = objState.Position.y - b.min.y;
                    horizontalRadius = Mathf.Max(b.extents.x, b.extents.z);
                    objTopY = b.max.y - objState.Position.y;
                }
            }

            float placeY = surfaceY + pivotToBottom + 0.005f;

            if (region != null)
            {
                Vector3 bestPos = Vector3.zero;
                Quaternion bestRot = Quaternion.identity;

                for (int attempt = 0; attempt < 200; attempt++)
                {
                    float mjX = RegionSampler.SampleX(region);
                    float mjY = RegionSampler.SampleY(region);
                    float mjYaw = RegionSampler.SampleYaw(region);

                    Vector3 absolutePos = CoordinateConverter.TablePosToUnity(mjX, mjY, 0f);
                    bestRot = targetState.Rotation * CoordinateConverter.MuJoCoYawToUnity(mjYaw);
                    bestPos = new Vector3(absolutePos.x, placeY, absolutePos.z);

                    bool hasCollision = false;
                    foreach (var other in _placedObjects)
                    {
                        float dxz = new Vector2(bestPos.x - other.pos.x, bestPos.z - other.pos.z).magnitude;
                        if (dxz < horizontalRadius + other.horizontalRadius)
                        {
                            float myBottom = placeY;
                            float myTop = placeY + objTopY;
                            if (!(myTop <= other.bottomY + 0.002f || myBottom >= other.topY - 0.002f))
                            {
                                hasCollision = true;
                                break;
                            }
                        }
                    }

                    if (!hasCollision) break;
                }

                objState.Position = bestPos;
                objState.Rotation = bestRot;
                _placedObjects.Add(new PlacedInfo { pos = bestPos, horizontalRadius = horizontalRadius, bottomY = placeY, topY = placeY + objTopY });
                Debug.Log($"[PlaceOn] {objectName} → region={locationName} pos=({bestPos.x:F3},{bestPos.y:F3},{bestPos.z:F3}) hRad={horizontalRadius:F3} p2b={pivotToBottom:F3}");
            }
            else
            {
                objState.Position = new Vector3(targetState.Position.x, placeY, targetState.Position.z);
                _placedObjects.Add(new PlacedInfo { pos = objState.Position, horizontalRadius = horizontalRadius, bottomY = placeY, topY = placeY + objTopY });
            }

            Physics.SyncTransforms();

            var rb = objState.GameObject.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }
    }

    private void PlaceIn(BDDL.InitState state, Transform objectParent)
        {
            string objectName = state.ObjectName;
            ObjectState objState = GetState(objectName);
            if (objState == null) return;

            if (_fixtureStates.TryGetValue(state.LocationName, out var fixture))
            {
                float surfaceY = GetSurfaceY(fixture.GameObject);

                float pivotToBottom = 0f;
                if (objState.HasXmlPlacementParams)
                {
                    pivotToBottom = objState.XmlBottomOffset;
                }
                else
                {
                    var myCol = objState.GameObject.GetComponentInChildren<Collider>();
                    if (myCol != null)
                        pivotToBottom = objState.Position.y - myCol.bounds.min.y;
                }

                objState.Position = new Vector3(fixture.Position.x, surfaceY + pivotToBottom + 0.002f, fixture.Position.z);

                Physics.SyncTransforms();
                var rb = objState.GameObject.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.velocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }
        }

        private static float GetSurfaceY(GameObject go)
        {
            if (go == null) return 0f;
            var col = go.GetComponentInChildren<Collider>();
            if (col != null) return col.bounds.max.y;
            return go.transform.position.y;
        }

        private ObjectState GetState(string objectName)
        {
            if (_objectStates.TryGetValue(objectName, out var objState)) return objState;
            if (_fixtureStates.TryGetValue(objectName, out var fixState)) return fixState;
            return null;
        }

        private void GenerateObjectStateWrappers()
        {
            // Already done during LoadFixtures/LoadObjects
        }

        private static readonly string[] MeshSearchDirs = {
            "stable_scanned_objects", "articulated_objects", "turbosquid_objects", "stable_hope_objects"
        };

        private string FindObjectXml(string category)
        {
            string assetsRoot = AssetDatabase.GetAssetPath("");
            foreach (string dir in MeshSearchDirs)
            {
                string xmlPath = System.IO.Path.Combine(assetsRoot, dir, category, category + ".xml");
                if (System.IO.File.Exists(xmlPath)) return xmlPath;

                xmlPath = System.IO.Path.Combine(assetsRoot, dir, category + ".xml");
                if (System.IO.File.Exists(xmlPath)) return xmlPath;
            }
            return null;
        }

        private bool TryLoadMesh(string category, string objectName, GameObject go)
        {
            string xmlPath = FindObjectXml(category);
            if (xmlPath == null) return false;

            try
            {
                ObjectLoader.LoadIntoGameObject(go, xmlPath);
                return true;
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"Failed to load mesh for {category}: {e.Message}");
                return false;
            }
        }

        private GameObject CreatePrimitiveFromCategory(string category, string objectName)
        {
            var go = new GameObject(objectName);

            if (!TryLoadMesh(category, objectName, go))
            {
                // Fallback: procedural primitives for categories without mesh files
                switch (category.ToLowerInvariant())
                {
                    case "table":
                        var table = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        table.transform.SetParent(go.transform);
                        table.transform.localScale = new Vector3(1.2f, 0.05f, 1.0f);
                        table.transform.localPosition = new Vector3(0, -0.025f, 0);

                        var texPath = AssetDatabase.GetAssetPath("textures/martin_novak_wood_table.png");
                        if (System.IO.File.Exists(texPath))
                        {
                            byte[] texData = System.IO.File.ReadAllBytes(texPath);
                            Texture2D tex = new Texture2D(2, 2);
                            tex.LoadImage(texData);
                            Material mat = new Material(Shader.Find("Standard"));
                            mat.mainTexture = tex;
                            table.GetComponent<MeshRenderer>().material = mat;
                        }

                        for (int x = -1; x <= 1; x += 2)
                        {
                            for (int z = -1; z <= 1; z += 2)
                            {
                                var leg = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                                leg.transform.SetParent(go.transform);
                                leg.transform.localScale = new Vector3(0.05f, 0.4375f, 0.05f);
                                leg.transform.localPosition = new Vector3(x * 0.5f, -0.4375f, z * 0.4f);
                            }
                        }
                        break;

                    case "red_block":
                        {
                            var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            cube.transform.SetParent(go.transform);
                            cube.transform.localScale = new Vector3(0.05f, 0.05f, 0.05f);
                            var mat = new Material(Shader.Find("Standard"));
                            mat.color = Color.red;
                            cube.GetComponent<MeshRenderer>().material = mat;
                        }
                        break;
                    case "green_block":
                        {
                            var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            cube.transform.SetParent(go.transform);
                            cube.transform.localScale = new Vector3(0.05f, 0.05f, 0.05f);
                            var mat = new Material(Shader.Find("Standard"));
                            mat.color = Color.green;
                            cube.GetComponent<MeshRenderer>().material = mat;
                        }
                        break;
                    case "blue_block":
                        {
                            var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            cube.transform.SetParent(go.transform);
                            cube.transform.localScale = new Vector3(0.05f, 0.05f, 0.05f);
                            var mat = new Material(Shader.Find("Standard"));
                            mat.color = Color.blue;
                            cube.GetComponent<MeshRenderer>().material = mat;
                        }
                        break;

                    default:
                        var defaultPrimitive = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        defaultPrimitive.transform.SetParent(go.transform);
                        defaultPrimitive.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                        break;
                }
            }

            if (go.GetComponent<Rigidbody>() == null)
            {
                var rb = go.AddComponent<Rigidbody>();
                rb.mass = 0.5f;
                rb.useGravity = true;
            }
            if (go.GetComponentInChildren<Collider>() == null)
            {
                Bounds combined = ComputeMeshBounds(go);
                var col = go.AddComponent<BoxCollider>();
                col.center = combined.center - go.transform.position;
                col.size = combined.size;
            }

            return go;
        }

        private static Bounds ComputeMeshBounds(GameObject go)
        {
            var renderers = go.GetComponentsInChildren<MeshRenderer>();
            if (renderers.Length == 0)
                return new Bounds(Vector3.zero, new Vector3(0.1f, 0.1f, 0.1f));

            Bounds combined = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++)
                combined.Encapsulate(renderers[i].bounds);

            if (combined.size.sqrMagnitude < 0.0001f)
                combined = new Bounds(Vector3.zero, new Vector3(0.1f, 0.1f, 0.1f));

            return combined;
        }
    }
}
