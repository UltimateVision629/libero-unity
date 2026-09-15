using System.Collections.Generic;
using System.IO;
using System.Xml;
using UnityEngine;

namespace UnityRobotEnv.Core
{
    public struct PlacementSiteData
    {
        public float BottomOffset;
        public float TopOffset;
        public float HorizontalRadius;
    }

    public static class ObjectLoader
    {
        public static void LoadIntoGameObject(GameObject go, string xmlPath)
        {
            if (!File.Exists(xmlPath))
            {
                Debug.LogWarning($"Object XML not found: {xmlPath}");
                return;
            }

            XmlDocument doc = new XmlDocument();
            doc.Load(xmlPath);
            XmlElement root = doc.DocumentElement;

            XmlNodeList meshNodes = root.SelectNodes("//asset/mesh");
            XmlNodeList texNodes = root.SelectNodes("//asset/texture");
            XmlNode worldbody = root.SelectSingleNode("//worldbody");

            string baseDir = Path.GetDirectoryName(xmlPath);

            Material sharedMat = null;
            foreach (XmlNode texNode in texNodes)
            {
                string texFile = GetAttr(texNode, "file");
                if (!string.IsNullOrEmpty(texFile))
                {
                    sharedMat = LoadMaterial(baseDir, texFile);
                    break;
                }
            }

            Dictionary<string, Material> materialCache = new Dictionary<string, Material>();
            foreach (XmlNode matNode in root.SelectNodes("//asset/material"))
            {
                string matName = GetAttr(matNode, "name");
                if (!string.IsNullOrEmpty(matName))
                    materialCache[matName] = sharedMat ?? new Material(Shader.Find("Standard"));
            }

            List<BodyInfo> bodies = ParseBodyHierarchy(worldbody);
            foreach (BodyInfo body in bodies)
            {
                BuildBody(go, body, baseDir, meshNodes, materialCache, sharedMat);
            }

            NormalizeBottom(go);
        }

        public static PlacementSiteData? ParsePlacementSites(string xmlPath)
        {
            if (!File.Exists(xmlPath)) return null;

            XmlDocument doc = new XmlDocument();
            doc.Load(xmlPath);
            XmlElement root = doc.DocumentElement;

            XmlNode bodyNode = root.SelectSingleNode("//worldbody/body");
            if (bodyNode == null) return null;

            XmlNode bottomSite = bodyNode.SelectSingleNode("site[@name='bottom_site']");
            XmlNode topSite = bodyNode.SelectSingleNode("site[@name='top_site']");
            XmlNode hrSite = bodyNode.SelectSingleNode("site[@name='horizontal_radius_site']");

            if (bottomSite == null && hrSite == null) return null;

            PlacementSiteData data = new PlacementSiteData
            {
                BottomOffset = 0.1f,
                TopOffset = 0.1f,
                HorizontalRadius = 0.05f
            };

            if (bottomSite != null)
            {
                string pos = GetAttr(bottomSite, "pos");
                if (!string.IsNullOrEmpty(pos))
                {
                    string[] parts = pos.Split(' ');
                    if (parts.Length >= 3)
                        data.BottomOffset = -float.Parse(parts[2]);
                }
            }

            if (topSite != null)
            {
                string pos = GetAttr(topSite, "pos");
                if (!string.IsNullOrEmpty(pos))
                {
                    string[] parts = pos.Split(' ');
                    if (parts.Length >= 3)
                        data.TopOffset = float.Parse(parts[2]);
                }
            }

            if (hrSite != null)
            {
                string pos = GetAttr(hrSite, "pos");
                if (!string.IsNullOrEmpty(pos))
                {
                    string[] parts = pos.Split(' ');
                    if (parts.Length >= 1)
                        data.HorizontalRadius = float.Parse(parts[0]);
                }
            }

            return data;
        }

        private static void NormalizeBottom(GameObject go)
        {
            var renderers = go.GetComponentsInChildren<MeshRenderer>();
            if (renderers.Length == 0) return;

            Bounds combined = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++)
                combined.Encapsulate(renderers[i].bounds);

            if (combined.size.sqrMagnitude < 0.0001f) return;

            float offsetY = -combined.min.y;
            go.transform.localPosition += new Vector3(0, offsetY, 0);
        }

        private static Material LoadMaterial(string baseDir, string texFile)
        {
            string texPath = Path.Combine(baseDir, texFile);
            if (File.Exists(texPath))
            {
                byte[] texData = File.ReadAllBytes(texPath);
                Texture2D tex = new Texture2D(2, 2);
                tex.LoadImage(texData);
                Material mat = new Material(Shader.Find("Standard"));
                mat.mainTexture = tex;
                return mat;
            }
            return new Material(Shader.Find("Standard"));
        }

        private static List<BodyInfo> ParseBodyHierarchy(XmlNode parent)
        {
            List<BodyInfo> result = new List<BodyInfo>();
            foreach (XmlNode child in parent.ChildNodes)
            {
                if (child.Name != "body") continue;
                BodyInfo body = new BodyInfo
                {
                    name = GetAttr(child, "name"),
                    pos = ParsePos(child),
                    quat = ParseQuat(child),
                    geoms = new List<GeomInfo>(),
                    prims = new List<PrimGeomInfo>(),
                    children = ParseBodyHierarchy(child)
                };
                foreach (XmlNode geom in child.SelectNodes("geom"))
                {
                    string type = GetAttr(geom, "type");
                    string grp = GetAttr(geom, "group");
                    if (type == "mesh")
                    {
                        body.geoms.Add(new GeomInfo
                        {
                            meshName = GetAttr(geom, "mesh"),
                            materialName = GetAttr(geom, "material"),
                            pos = ParsePos(geom),
                            quat = ParseGeomQuat(geom)
                        });
                    }
                    else if ((type == "box" || type == "cylinder") && grp == "1")
                    {
                        body.prims.Add(new PrimGeomInfo
                        {
                            type = type,
                            pos = ParsePos(geom),
                            size = ParseSize(geom),
                            quat = ParseGeomQuat(geom),
                            materialName = GetAttr(geom, "material"),
                            rgba = GetAttr(geom, "rgba")
                        });
                    }
                }
                result.Add(body);
            }
            return result;
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

        private static Quaternion ParseGeomQuat(XmlNode node)
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

        private static void BuildBody(GameObject parent, BodyInfo body, string baseDir,
            XmlNodeList meshNodes, Dictionary<string, Material> matCache, Material defaultMat)
        {
            GameObject bodyGo = new GameObject(body.name ?? "body");
            bodyGo.transform.SetParent(parent.transform);
            bodyGo.transform.localPosition = body.pos;
            bodyGo.transform.localRotation = body.quat;

            foreach (GeomInfo geom in body.geoms)
            {
                Mesh mesh = null;
                string meshFile = null;
                Vector3 meshScale = Vector3.one;

                foreach (XmlNode mNode in meshNodes)
                {
                    if (GetAttr(mNode, "name") == geom.meshName || GetAttr(mNode, "file")?.Contains(geom.meshName) == true)
                    {
                        meshFile = GetAttr(mNode, "file");
                        meshScale = ParseScale(mNode);
                        break;
                    }
                }

                if (string.IsNullOrEmpty(meshFile))
                {
                    foreach (XmlNode mNode in meshNodes)
                    {
                        meshFile = GetAttr(mNode, "file");
                        meshScale = ParseScale(mNode);
                        break;
                    }
                }

                if (!string.IsNullOrEmpty(meshFile))
                {
                    mesh = LoadOrCacheMesh(baseDir, meshFile, meshScale);
                }

                if (mesh != null)
                {
                    GameObject meshGo = new GameObject(geom.meshName ?? "visual");
                    meshGo.transform.SetParent(bodyGo.transform);
                    meshGo.transform.localPosition = geom.pos;
                    meshGo.transform.localRotation = geom.quat;
                    meshGo.AddComponent<MeshFilter>().sharedMesh = mesh;

                    Material mat = defaultMat;
                    if (!string.IsNullOrEmpty(geom.materialName) && matCache.TryGetValue(geom.materialName, out Material cached))
                        mat = cached;
                    meshGo.AddComponent<MeshRenderer>().material = mat;
                }
            }

            foreach (PrimGeomInfo prim in body.prims)
            {
                Material mat = defaultMat;
                if (!string.IsNullOrEmpty(prim.materialName) && matCache.TryGetValue(prim.materialName, out Material cached))
                    mat = cached;
                else if (!string.IsNullOrEmpty(prim.rgba))
                {
                    string[] cparts = prim.rgba.Split(' ');
                    if (cparts.Length >= 4)
                    {
                        mat = new Material(Shader.Find("Standard"));
                        Color c = new Color(float.Parse(cparts[0]), float.Parse(cparts[1]), float.Parse(cparts[2]), float.Parse(cparts[3]));
                        mat.color = c;
                    }
                }

                if (prim.type == "box")
                {
                    var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.transform.SetParent(bodyGo.transform);
                    cube.transform.localPosition = prim.pos;
                    cube.transform.localRotation = prim.quat;
                    cube.transform.localScale = prim.size;
                    cube.GetComponent<MeshRenderer>().material = mat;
                }
                else if (prim.type == "cylinder")
                {
                    var cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    cyl.transform.SetParent(bodyGo.transform);
                    cyl.transform.localPosition = prim.pos;
                    cyl.transform.localRotation = prim.quat;
                    cyl.transform.localScale = prim.size;
                    cyl.GetComponent<MeshRenderer>().material = mat;
                }
            }

            foreach (BodyInfo child in body.children)
                BuildBody(bodyGo, child, baseDir, meshNodes, matCache, defaultMat);
        }

        private static Dictionary<string, Mesh> _meshCache = new Dictionary<string, Mesh>();

        private static Mesh LoadOrCacheMesh(string baseDir, string file, Vector3 scale)
        {
            string fullPath = Path.Combine(baseDir, file).Replace('/', Path.DirectorySeparatorChar);
            string key = $"{fullPath}|{scale}";
            if (_meshCache.TryGetValue(key, out Mesh cached))
            {
                if (cached != null && cached.vertexCount > 0)
                    return cached;
                _meshCache.Remove(key);
            }

            if (File.Exists(fullPath))
            {
                try
                {
                    Mesh mesh = MeshFileParser.Load(fullPath, scale);
                    mesh.hideFlags = HideFlags.DontSave;
                    _meshCache[key] = mesh;
                    return mesh;
                }
                catch (System.Exception e)
                {
                    Debug.LogWarning($"Failed to load mesh {fullPath}: {e.Message}");
                }
            }
            return null;
        }

        private static Vector3 ParseScale(XmlNode node)
        {
            string s = GetAttr(node, "scale");
            if (string.IsNullOrEmpty(s)) return Vector3.one;
            string[] parts = s.Split(' ');
            if (parts.Length < 3) return Vector3.one;
            float sx = float.Parse(parts[0]);
            float sy = float.Parse(parts[1]);
            float sz = float.Parse(parts[2]);
            return new Vector3(sx, sy, sz);
        }

        private static string GetAttr(XmlNode node, string name)
        {
            return node.Attributes?[name]?.Value;
        }

        private class BodyInfo
        {
            public string name;
            public Vector3 pos;
            public Quaternion quat;
            public List<GeomInfo> geoms;
            public List<PrimGeomInfo> prims;
            public List<BodyInfo> children;
        }

        private class GeomInfo
        {
            public string meshName;
            public string materialName;
            public Vector3 pos;
            public Quaternion quat;
        }

        private class PrimGeomInfo
        {
            public string type;
            public Vector3 pos;
            public Vector3 size;
            public Quaternion quat;
            public string materialName;
            public string rgba;
        }
    }
}
