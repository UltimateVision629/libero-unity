using System.Collections.Generic;
using System.IO;
using System.Xml;
using UnityEngine;

namespace LIBERO.Core
{
    public struct RobotBuildResult
    {
        public GameObject Root;
        public ArticulationBody[] Joints;
        public Transform EEFTransform;
        public ArticulationBody LeftFinger;
        public ArticulationBody RightFinger;
        public ArticulationBody GripperJoint;
        public Transform GripSite;
    }

    public static class RobotBuilder
    {
        private class MaterialDef
        {
            public float specular;
            public float shininess;
            public Color rgba;
        }

        public static RobotBuildResult BuildArm(string xmlPath)
        {
            if (!File.Exists(xmlPath))
            {
                Debug.LogError($"Robot XML not found: {xmlPath}");
                return default;
            }

            XmlDocument doc = new XmlDocument();
            doc.Load(xmlPath);
            XmlElement root = doc.DocumentElement;

            string baseDir = Path.GetDirectoryName(xmlPath);
            var materials = ParseMaterials(root);
            var meshFiles = ParseMeshes(root);

            XmlNode worldbody = root.SelectSingleNode("//worldbody");
            if (worldbody == null) return default;

            XmlNode baseBody = worldbody.SelectSingleNode("body");
            if (baseBody == null) return default;

            var result = new RobotBuildResult();
            var jointList = new List<ArticulationBody>();

            result.Root = new GameObject("PandaArm");
            var rootAb = result.Root.AddComponent<ArticulationBody>();
            rootAb.immovable = true;
            rootAb.jointType = ArticulationJointType.FixedJoint;
            rootAb.anchorPosition = Vector3.zero;

            BuildBodyHierarchy(result.Root.transform, baseBody, baseDir, meshFiles, materials,
                result.Root, ref result, ref jointList, isRoot: true, enableArticulation: true);

            result.Joints = jointList.ToArray();

            for (int i = 0; i < jointList.Count - 1; i++)
            {
                var colsA = GetLinkColliders(jointList[i].gameObject);
                var colsB = GetLinkColliders(jointList[i + 1].gameObject);
                foreach (var ca in colsA)
                    foreach (var cb in colsB)
                        if (ca != null && cb != null)
                            Physics.IgnoreCollision(ca, cb);
            }

            Debug.Log($"[RobotBuilder] Built Arm: {jointList.Count} joints, EEF={result.EEFTransform != null}"
                + $" base={result.Root.transform.Find("base")?.position.y:F3}"
                + $" link0={result.Root.transform.Find("base/link0")?.position.y:F3}"
                + $" link1={result.Root.transform.Find("base/link0/link1")?.position.y:F3}");
            return result;
        }

        public static void AttachMount(GameObject robotRoot, string xmlPath)
        {
            if (!File.Exists(xmlPath))
            {
                Debug.LogWarning($"Mount XML not found: {xmlPath}");
                return;
            }

            XmlDocument doc = new XmlDocument();
            doc.Load(xmlPath);
            XmlElement root = doc.DocumentElement;

            string baseDir = Path.GetDirectoryName(xmlPath);
            var materials = ParseMaterials(root);
            var meshFiles = ParseMeshes(root);

            XmlNode worldbody = root.SelectSingleNode("//worldbody");
            if (worldbody == null) return;

            var result = new RobotBuildResult();
            var jointList = new List<ArticulationBody>();

            foreach (XmlNode body in worldbody.SelectNodes("body"))
            {
                if (body.Attributes?["pos"] != null)
                    body.Attributes["pos"].Value = "0 0 0";
                BuildBodyHierarchy(robotRoot.transform, body, baseDir, meshFiles, materials,
                    robotRoot, ref result, ref jointList, isRoot: false, enableArticulation: false);
            }

            Transform mountBase = robotRoot.transform.Find("base");
            if (mountBase != null)
            {
                foreach (var col in mountBase.GetComponentsInChildren<Collider>())
                    col.isTrigger = true;
            }

            Debug.Log($"[RobotBuilder] Built Mount: path={xmlPath}");
        }

        public static void AttachGripper(Transform eef, string xmlPath, ref RobotBuildResult result)
        {
            if (!File.Exists(xmlPath))
            {
                Debug.LogWarning($"Gripper XML not found: {xmlPath}");
                return;
            }

            XmlDocument doc = new XmlDocument();
            doc.Load(xmlPath);
            XmlElement root = doc.DocumentElement;

            string baseDir = Path.GetDirectoryName(xmlPath);
            var materials = ParseMaterials(root);
            var meshFiles = ParseMeshes(root);

            XmlNode worldbody = root.SelectSingleNode("//worldbody");
            if (worldbody == null) return;

            var jointList = new List<ArticulationBody>();

            foreach (XmlNode body in worldbody.SelectNodes("body"))
                BuildBodyHierarchy(eef, body, baseDir, meshFiles, materials,
                    eef.gameObject, ref result, ref jointList, isRoot: false, enableArticulation: false);

            // Find finger articulation bodies
            FindFingerBodies(eef.gameObject, ref result);

            Debug.Log($"[RobotBuilder] Built Gripper: left={result.LeftFinger != null} right={result.RightFinger != null} grip={result.GripSite != null}");
        }

        private static void FindFingerBodies(GameObject go, ref RobotBuildResult result)
        {
            for (int i = 0; i < go.transform.childCount; i++)
            {
                var child = go.transform.GetChild(i);
                if (child.name == "leftfinger")
                {
                    result.LeftFinger = child.GetComponent<ArticulationBody>();
                    var tip = child.Find("finger_joint1_tip");
                    if (tip != null) result.GripSite = tip;
                }
                else if (child.name == "rightfinger")
                {
                    result.RightFinger = child.GetComponent<ArticulationBody>();
                }
                else if (child.name == "eef")
                {
                    if (result.GripSite == null) result.GripSite = child;
                }
                FindFingerBodies(child.gameObject, ref result);
            }
        }

        private static void BuildBodyHierarchy(Transform parent, XmlNode bodyNode, string baseDir,
            Dictionary<string, string> meshFiles, Dictionary<string, MaterialDef> materials,
            GameObject rootGo, ref RobotBuildResult result, ref List<ArticulationBody> jointList,
            bool isRoot, bool enableArticulation)
        {
            string bodyName = GetAttr(bodyNode, "name") ?? "body";
            Vector3 bodyPos = ParsePos(bodyNode);
            Quaternion bodyQuat = ParseQuat(bodyNode);

            var bodyGo = new GameObject(bodyName);
            bodyGo.transform.localPosition = bodyPos;
            bodyGo.transform.localRotation = bodyQuat;
            bodyGo.transform.SetParent(parent, false);

            XmlNode jointNode = bodyNode.SelectSingleNode("joint");

            ArticulationBody ab = null;

            if (enableArticulation)
            {
            if (isRoot)
            {
                bodyGo.transform.SetParent(rootGo.transform);
                ab = bodyGo.AddComponent<ArticulationBody>();
                ab.jointType = ArticulationJointType.FixedJoint;
            }

            if (!isRoot && jointNode != null)
            {
                ab = bodyGo.AddComponent<ArticulationBody>();
                string jointType = GetAttr(jointNode, "type") ?? "revolute";

                if (jointType == "slide")
                {
                    ab.jointType = ArticulationJointType.PrismaticJoint;
                    ab.linearLockX = ArticulationDofLock.LimitedMotion;
                    ab.linearLockY = ArticulationDofLock.LockedMotion;
                    ab.linearLockZ = ArticulationDofLock.LockedMotion;
                    float[] range = ParseRange(jointNode);
                    var drive = ab.xDrive;
                    if (range.Length >= 2) { drive.lowerLimit = range[0]; drive.upperLimit = range[1]; }
                    drive.stiffness = 1000f;
                    drive.damping = 100f;
                    ab.xDrive = drive;
                }
                else
                {
                    ab.jointType = ArticulationJointType.RevoluteJoint;
                    ab.twistLock = ArticulationDofLock.LimitedMotion;
                    ab.anchorPosition = Vector3.zero;

                    string axisStr = GetAttr(jointNode, "axis") ?? "0 0 1";
                    string[] ap = axisStr.Split(' ');
                    float mx = ap.Length >= 1 ? float.Parse(ap[0]) : 0f;
                    float my = ap.Length >= 2 ? float.Parse(ap[1]) : 0f;
                    float mz = ap.Length >= 3 ? float.Parse(ap[2]) : 1f;
                    Vector3 uAxis = new Vector3(-my, mz, mx).normalized;
                    ab.anchorRotation = Quaternion.FromToRotation(Vector3.right, uAxis);

                    float[] range = ParseRange(jointNode);
                    if (range.Length >= 2)
                    {
                        var drive = ab.xDrive;
                        drive.lowerLimit = range[0] * Mathf.Rad2Deg;
                        drive.upperLimit = range[1] * Mathf.Rad2Deg;
                        drive.stiffness = 10000f;
                        drive.damping = 200f;
                        drive.forceLimit = 200f;
                        ab.xDrive = drive;
                    }

                    jointList.Add(ab);
                }
            }
            else if (!isRoot)
            {
                ab = bodyGo.AddComponent<ArticulationBody>();
                ab.jointType = ArticulationJointType.FixedJoint;
            }
            }

            if (bodyName == "right_hand")
                result.EEFTransform = bodyGo.transform;

            if (enableArticulation)
                AlignAnchors(ab, bodyPos);

            // Create visual and collision geoms
            int visIndex = 0;
            foreach (XmlNode geom in bodyNode.SelectNodes("geom"))
            {
                string grp = GetAttr(geom, "group");
                string type = GetAttr(geom, "type");
                string meshName = GetAttr(geom, "mesh");
                string matName = GetAttr(geom, "material");
                string rgba = GetAttr(geom, "rgba");

                if (type == "mesh" && !string.IsNullOrEmpty(meshName))
                {
                    bool isCollision = (grp == "0");
                    if (!isCollision && grp != "1") continue;

                    if (!meshFiles.TryGetValue(meshName, out string meshFile))
                    {
                        string cleanName = meshName.Replace("_stl", "");
                        if (!meshFiles.TryGetValue(cleanName, out meshFile))
                            continue;
                    }

                    string fullPath = Path.Combine(baseDir, meshFile);
                    if (!File.Exists(fullPath)) continue;

                    try
                    {
                        Mesh mesh = MeshFileParser.Load(fullPath, Vector3.one);
                        if (mesh == null) continue;

                        if (isCollision)
                        {
                            Bounds b = mesh.bounds;
                            var colGo = new GameObject(bodyName + "_col_" + visIndex);
                            colGo.transform.SetParent(bodyGo.transform);
                            colGo.transform.localPosition = ParsePos(geom);
                            colGo.transform.localRotation = ParseGeomQuat(geom);
                            var bc = colGo.AddComponent<BoxCollider>();
                            bc.center = b.center - ParsePos(geom);
                            bc.size = b.size;
                        }
                        else
                        {
                            var visGo = new GameObject(bodyName + "_vis_" + visIndex);
                            visGo.transform.SetParent(bodyGo.transform);
                            visGo.transform.localPosition = ParsePos(geom);
                            visGo.transform.localRotation = ParseGeomQuat(geom);
                            visGo.AddComponent<MeshFilter>().sharedMesh = mesh;
                            var mr = visGo.AddComponent<MeshRenderer>();
                            mr.sharedMaterial = CreateMaterial(matName, materials);
                            visIndex++;
                        }
                    }
                    catch (System.Exception e)
                    {
                        Debug.LogWarning($"Failed to load mesh {fullPath}: {e.Message}");
                    }
                }
                else if (type == "box")
                {
                    Vector3 size = ParseSize(geom);
                    var prim = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    prim.name = bodyName + "_box_" + visIndex;
                    prim.transform.SetParent(bodyGo.transform);
                    prim.transform.localPosition = ParsePos(geom);
                    prim.transform.localRotation = ParseGeomQuat(geom);
                    prim.transform.localScale = size;

                    if (!string.IsNullOrEmpty(rgba))
                    {
                        var mat = new Material(Shader.Find("Standard"));
                        mat.color = ParseColor(rgba);
                        prim.GetComponent<MeshRenderer>().sharedMaterial = mat;
                    }

                    if (grp == "0")
                    {
                        prim.GetComponent<MeshRenderer>().enabled = false;
                    }
                    visIndex++;
                }
                else if (type == "cylinder")
                {
                    string sz = GetAttr(geom, "size");
                    if (!string.IsNullOrEmpty(sz))
                    {
                        string[] parts = sz.Split(' ');
                        float radius = float.Parse(parts[0]);
                        float halfHeight = parts.Length > 1 ? float.Parse(parts[1]) : 0.1f;

                        var prim = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                        prim.name = bodyName + "_cyl_" + visIndex;
                        prim.transform.SetParent(bodyGo.transform);
                        prim.transform.localPosition = ParsePos(geom);
                        prim.transform.localRotation = ParseGeomQuat(geom);
                        prim.transform.localScale = new Vector3(radius * 2f, halfHeight, radius * 2f);

                        if (!string.IsNullOrEmpty(rgba))
                        {
                            var mat = new Material(Shader.Find("Standard"));
                            mat.color = ParseColor(rgba);
                            prim.GetComponent<MeshRenderer>().sharedMaterial = mat;
                        }

                        if (grp == "0")
                        {
                            prim.GetComponent<MeshRenderer>().enabled = false;
                        }
                        visIndex++;
                    }
                }
            }

            // Recurse into child bodies
            foreach (XmlNode child in bodyNode.SelectNodes("body"))
            {
                BuildBodyHierarchy(bodyGo.transform, child, baseDir, meshFiles, materials,
                    rootGo, ref result, ref jointList, isRoot: false, enableArticulation: enableArticulation);
            }
        }

        private static List<Collider> GetLinkColliders(GameObject linkObj)
        {
            var cols = new List<Collider>();
            foreach (var c in linkObj.GetComponentsInChildren<Collider>())
            {
                var ab = c.GetComponentInParent<ArticulationBody>();
                if (ab != null && ab.gameObject == linkObj)
                    cols.Add(c);
            }
            return cols;
        }

        private static void AlignAnchors(ArticulationBody childAb, Vector3 localPos)
        {
            if (childAb == null) return;
            childAb.anchorPosition = Vector3.zero;
            childAb.parentAnchorPosition = localPos;
        }

        private static Material CreateMaterial(string matName, Dictionary<string, MaterialDef> materials)
        {
            Material mat = new Material(Shader.Find("Standard"));
            if (!string.IsNullOrEmpty(matName) && materials.TryGetValue(matName, out var def))
            {
                mat.color = def.rgba;
                mat.SetFloat("_Glossiness", def.shininess);
                mat.SetFloat("_SpecularHighlights", def.specular > 0 ? 1f : 0f);
            }
            return mat;
        }

        private static Dictionary<string, MaterialDef> ParseMaterials(XmlElement root)
        {
            var dict = new Dictionary<string, MaterialDef>();
            foreach (XmlNode mat in root.SelectNodes("//asset/material"))
            {
                string name = GetAttr(mat, "name");
                if (string.IsNullOrEmpty(name)) continue;
                dict[name] = new MaterialDef
                {
                    specular = ParseFloat(GetAttr(mat, "specular")),
                    shininess = ParseFloat(GetAttr(mat, "shininess")),
                    rgba = ParseColor(GetAttr(mat, "rgba"))
                };
            }
            return dict;
        }

        private static Dictionary<string, string> ParseMeshes(XmlElement root)
        {
            var dict = new Dictionary<string, string>();
            foreach (XmlNode mesh in root.SelectNodes("//asset/mesh"))
            {
                string name = GetAttr(mesh, "name");
                string file = GetAttr(mesh, "file");
                if (!string.IsNullOrEmpty(name) && !string.IsNullOrEmpty(file))
                    dict[name] = file;
            }
            return dict;
        }

        private static float[] ParseRange(XmlNode joint)
        {
            string r = GetAttr(joint, "range");
            if (string.IsNullOrEmpty(r)) return new float[0];
            string[] parts = r.Split(' ');
            if (parts.Length < 2) return new float[0];
            return new[] { float.Parse(parts[0]), float.Parse(parts[1]) };
        }

        private static Vector3 ParsePos(XmlNode node)
        {
            string pos = GetAttr(node, "pos");
            if (string.IsNullOrEmpty(pos)) return Vector3.zero;
            string[] parts = pos.Split(' ');
            if (parts.Length < 3) return Vector3.zero;
            float mx = float.Parse(parts[0]);
            float my = float.Parse(parts[1]);
            float mz = float.Parse(parts[2]);
            return new Vector3(-my, mz, mx);
        }

        private static Vector3 ParseSize(XmlNode node)
        {
            string s = GetAttr(node, "size");
            if (string.IsNullOrEmpty(s)) return Vector3.one * 0.1f;
            string[] parts = s.Split(' ');
            if (parts.Length < 3) return Vector3.one * 0.1f;
            float sx = float.Parse(parts[0]);
            float sy = float.Parse(parts[1]);
            float sz = float.Parse(parts[2]);
            return new Vector3(sy * 2f, sz * 2f, sx * 2f);
        }

        private static Quaternion ParseQuat(XmlNode node)
        {
            string q = GetAttr(node, "quat");
            if (string.IsNullOrEmpty(q)) return Quaternion.identity;
            string[] parts = q.Split(' ');
            if (parts.Length < 4) return Quaternion.identity;
            float w = float.Parse(parts[0]);
            float x = float.Parse(parts[1]);
            float y = float.Parse(parts[2]);
            float z = float.Parse(parts[3]);
            return new Quaternion(y, -z, -x, w);
        }

        private static Quaternion ParseGeomQuat(XmlNode node) => ParseQuat(node);

        private static Color ParseColor(string rgba)
        {
            if (string.IsNullOrEmpty(rgba)) return Color.white;
            string[] parts = rgba.Split(' ');
            if (parts.Length < 4) return Color.white;
            return new Color(
                float.Parse(parts[0]),
                float.Parse(parts[1]),
                float.Parse(parts[2]),
                float.Parse(parts[3]));
        }

        private static float ParseFloat(string s)
        {
            if (string.IsNullOrEmpty(s)) return 0f;
            return float.TryParse(s, out float val) ? val : 0f;
        }

        private static string GetAttr(XmlNode node, string name)
        {
            return node.Attributes?[name]?.Value;
        }
    }
}
