using System.Collections.Generic;
using System.IO;
using System.Xml;
using UnityEngine;

namespace LIBERO.Core
{
    public static class UrdfArmBuilder
    {
        public static RobotBuildResult Build(string urdfPath, string meshBaseDir)
        {
            if (!File.Exists(urdfPath)) return default;

            XmlDocument doc = new XmlDocument();
            doc.Load(urdfPath);
            XmlElement rootNode = doc.DocumentElement;
            if (rootNode == null) return default;

            var linkMap = ParseLinks(rootNode);
            var jointList = new List<ArticulationBody>();

            var root = new GameObject("PandaURDF");
            var rootAb = root.AddComponent<ArticulationBody>();
            rootAb.immovable = true;
            rootAb.jointType = ArticulationJointType.FixedJoint;
            rootAb.anchorPosition = Vector3.zero;

            var result = new RobotBuildResult { Root = root };

            XmlNode firstLink = rootNode.SelectSingleNode("link[@name='panda_link0']");
            if (firstLink == null) return result;

            BuildLink(firstLink, root.transform, linkMap, rootNode, root,
                ref result, ref jointList, meshBaseDir);

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

            Debug.Log($"[UrdfArm] Built: {jointList.Count} joints, "
                + $"EEF={result.EEFTransform != null} "
                + $"L={result.LeftFinger != null} R={result.RightFinger != null}");
            return result;
        }

        private static Dictionary<string, LinkInfo> ParseLinks(XmlElement root)
        {
            var map = new Dictionary<string, LinkInfo>();
            foreach (XmlNode linkNode in root.SelectNodes("link"))
            {
                string name = GetAttr(linkNode, "name");
                if (string.IsNullOrEmpty(name)) continue;

                var info = new LinkInfo();

                foreach (XmlNode visual in linkNode.SelectNodes("visual"))
                {
                    XmlNode mesh = visual.SelectSingleNode("geometry/mesh");
                    if (mesh != null)
                    {
                        string file = GetAttr(mesh, "filename");
                        if (!string.IsNullOrEmpty(file))
                            info.VisualPaths.Add(file);
                    }

                    XmlNode mat = visual.SelectSingleNode("material/color");
                    if (mat != null)
                    {
                        string rgba = GetAttr(mat, "rgba") ?? "1 1 1 1";
                        string[] p = rgba.Split(' ');
                        if (p.Length >= 3)
                            info.Color = new Color(float.Parse(p[0]), float.Parse(p[1]), float.Parse(p[2]), 1f);
                    }

                    XmlNode origin = visual.SelectSingleNode("origin");
                    if (origin != null) info.VisualOffset = ParseRpyXyz(origin);
                }

                foreach (XmlNode collision in linkNode.SelectNodes("collision"))
                {
                    XmlNode mesh = collision.SelectSingleNode("geometry/mesh");
                    if (mesh != null)
                    {
                        string file = GetAttr(mesh, "filename");
                        if (!string.IsNullOrEmpty(file))
                            info.CollisionPaths.Add(file);
                    }

                    XmlNode origin = collision.SelectSingleNode("origin");
                    if (origin != null) info.CollisionOffset = ParseRpyXyz(origin);
                }

                map[name] = info;
            }
            return map;
        }

        private static void BuildLink(XmlNode linkNode, Transform parent,
            Dictionary<string, LinkInfo> linkMap, XmlElement rootNode,
            GameObject rootGo, ref RobotBuildResult result,
            ref List<ArticulationBody> jointList, string meshBaseDir)
        {
            string linkName = GetAttr(linkNode, "name");
            if (string.IsNullOrEmpty(linkName)) return;

            var go = new GameObject(linkName);

            XmlNode jointNode = rootNode.SelectSingleNode(
                $"joint[child/@link='{linkName}']");
            if (jointNode == null)
            {
                // Root link (panda_link0) — no parent joint, add Fixed AB to maintain chain
                go.transform.SetParent(parent, false);
                go.transform.localPosition = Vector3.zero;
                go.transform.localRotation = Quaternion.identity;
                var ab = go.AddComponent<ArticulationBody>();
                ab.jointType = ArticulationJointType.FixedJoint;
                ab.anchorPosition = Vector3.zero;
            }
            else
            {
                XmlNode origin = jointNode.SelectSingleNode("origin");
                Vector3 pos = Vector3.zero;
                Quaternion rot = Quaternion.identity;
                if (origin != null)
                    (pos, rot) = ParseRpyXyz(origin);

                go.transform.localPosition = pos;
                go.transform.localRotation = rot;

                string jointType = GetAttr(jointNode, "type") ?? "fixed";
                ArticulationBody ab = go.AddComponent<ArticulationBody>();

                if (jointType == "revolute")
                {
                    ab.jointType = ArticulationJointType.RevoluteJoint;
                    ab.twistLock = ArticulationDofLock.LimitedMotion;
                    ab.anchorPosition = Vector3.zero;
                    ab.parentAnchorPosition = pos;

                    string axisStr = "0 0 1";
                    XmlNode axisNode = jointNode.SelectSingleNode("axis");
                    if (axisNode != null) axisStr = GetAttr(axisNode, "xyz") ?? "0 0 1";
                    Vector3 uAxis = ParseAxis(axisStr);
                    ab.anchorRotation = Quaternion.FromToRotation(Vector3.right, uAxis);

                    XmlNode limit = jointNode.SelectSingleNode("limit");
                    float lower = ParseFloat(GetAttr(limit, "lower"), -3.14f);
                    float upper = ParseFloat(GetAttr(limit, "upper"), 3.14f);

                    var drive = ab.xDrive;
                    drive.lowerLimit = lower * Mathf.Rad2Deg;
                    drive.upperLimit = upper * Mathf.Rad2Deg;
                    drive.stiffness = 10000f;
                    drive.damping = 200f;
                    drive.forceLimit = 200f;
                    ab.xDrive = drive;

                    jointList.Add(ab);
                }
                else if (jointType == "prismatic")
                {
                    ab.jointType = ArticulationJointType.PrismaticJoint;
                    ab.linearLockX = ArticulationDofLock.LimitedMotion;
                    ab.linearLockY = ArticulationDofLock.LockedMotion;
                    ab.linearLockZ = ArticulationDofLock.LockedMotion;
                    ab.anchorPosition = Vector3.zero;
                    ab.parentAnchorPosition = pos;

                    XmlNode limit = jointNode.SelectSingleNode("limit");
                    float lower = ParseFloat(GetAttr(limit, "lower"), 0f);
                    float upper = ParseFloat(GetAttr(limit, "upper"), 0.04f);

                    var drive = ab.xDrive;
                    drive.lowerLimit = lower;
                    drive.upperLimit = upper;
                    drive.stiffness = 1000f;
                    drive.damping = 100f;
                    ab.xDrive = drive;
                }
                else
                {
                    ab.jointType = ArticulationJointType.FixedJoint;
                    ab.anchorPosition = Vector3.zero;
                    ab.parentAnchorPosition = pos;
                }

                // Track EEF / fingers
                if (linkName == "panda_link8" || linkName == "panda_hand")
                    result.EEFTransform = go.transform;
                if (linkName == "panda_grasptarget")
                    result.GripSite = go.transform;
                if (linkName == "panda_leftfinger")
                    result.LeftFinger = ab;
                if (linkName == "panda_rightfinger")
                    result.RightFinger = ab;
            }

            // Load meshes
            if (linkMap.TryGetValue(linkName, out LinkInfo info))
                LoadLinkMeshes(go, info, meshBaseDir);

            go.transform.SetParent(parent, false);

            // Recurse
            foreach (XmlNode childJoint in rootNode.SelectNodes(
                $"joint[parent/@link='{linkName}']"))
            {
                string childLinkName = GetAttr(childJoint.SelectSingleNode("child"), "link");
                if (!string.IsNullOrEmpty(childLinkName))
                {
                    XmlNode childNode = rootNode.SelectSingleNode(
                        $"link[@name='{childLinkName}']");
                    if (childNode != null)
                        BuildLink(childNode, go.transform, linkMap, rootNode,
                            rootGo, ref result, ref jointList, meshBaseDir);
                }
            }
        }

        private static void LoadLinkMeshes(GameObject go, LinkInfo info, string meshBaseDir)
        {
            foreach (string path in info.VisualPaths)
            {
                string fullPath = ResolveMesh(path, meshBaseDir);
                if (!File.Exists(fullPath))
                {
                    Debug.LogWarning($"[UrdfArm] Visual mesh not found: {fullPath}");
                    continue;
                }
                try
                {
                    Mesh mesh = MeshFileParser.Load(fullPath, Vector3.one);
                    if (mesh == null) continue;
                    var visGo = new GameObject(go.name + "_vis");
                    visGo.transform.SetParent(go.transform, false);
                    visGo.transform.localPosition = info.VisualOffset.pos;
                    visGo.transform.localRotation = info.VisualOffset.rot;
                    visGo.AddComponent<MeshFilter>().sharedMesh = mesh;
                    var mr = visGo.AddComponent<MeshRenderer>();
                    mr.sharedMaterial = new Material(Shader.Find("Standard"));
                    mr.sharedMaterial.color = info.Color;
                }
                catch (System.Exception e)
                {
                    Debug.LogWarning($"Mesh load failed: {fullPath}: {e.Message}");
                }
            }

            foreach (string path in info.CollisionPaths)
            {
                string fullPath = ResolveMesh(path, meshBaseDir);
                if (!File.Exists(fullPath)) continue;
                try
                {
                    Mesh mesh = MeshFileParser.Load(fullPath, Vector3.one);
                    if (mesh == null) continue;
                    Bounds b = mesh.bounds;
                    var colGo = new GameObject(go.name + "_col");
                    colGo.transform.SetParent(go.transform, false);
                    colGo.transform.localPosition = info.CollisionOffset.pos;
                    colGo.transform.localRotation = info.CollisionOffset.rot;
                    var bc = colGo.AddComponent<BoxCollider>();
                    bc.center = b.center;
                    bc.size = b.size;
                }
                catch { }
            }
        }

        private static string ResolveMesh(string urdfFilename, string meshBaseDir)
        {
            // URDF: package://meshes/visual/link0.obj
            // Map to existing robosuite: robots/panda/obj_meshes/link0_vis/link0_vis_0.obj
            string relative = urdfFilename.Replace("package://meshes/", "");
            string fileName = Path.GetFileName(relative); // link0.obj

            if (fileName.Contains("finger"))
                return Path.Combine(meshBaseDir, "obj_meshes", "link7_vis", "link7_vis_0.obj");

            if (fileName.Contains("hand"))
                return Path.Combine(meshBaseDir, "obj_meshes", "link7_vis", "link7_vis_0.obj");

            // Match link0..link7
            for (int i = 0; i <= 7; i++)
            {
                if (fileName.Contains($"link{i}"))
                {
                    // Use first visual mesh from robosuite
                    string subDir = $"link{i}_vis";
                    string firstObj = $"link{i}_vis_0.obj";
                    string candidate = Path.Combine(meshBaseDir, "obj_meshes", subDir, firstObj);
                    if (File.Exists(candidate))
                        return candidate;
                    // Fallback: try all files in subdir
                    string dir = Path.Combine(meshBaseDir, "obj_meshes", subDir);
                    if (Directory.Exists(dir))
                    {
                        var files = Directory.GetFiles(dir, "*.obj");
                        if (files.Length > 0) return files[0];
                    }
                }
            }

            return Path.Combine(meshBaseDir, relative);
        }

        private static (Vector3 pos, Quaternion rot) ParseRpyXyz(XmlNode origin)
        {
            string xyz = GetAttr(origin, "xyz") ?? "0 0 0";
            string rpy = GetAttr(origin, "rpy") ?? "0 0 0";

            string[] xp = xyz.Split(' ');
            float mx = float.Parse(xp[0]), my = float.Parse(xp[1]), mz = float.Parse(xp[2]);
            Vector3 pos = new Vector3(-my, mz, mx);

            string[] rp = rpy.Split(' ');
            float roll = float.Parse(rp[0]), pitch = float.Parse(rp[1]), yaw = float.Parse(rp[2]);
            // URDF rpy (Z-up RH) → Unity Euler (Y-up LH)
            // R_URDF = Rz(yaw) * Ry(pitch) * Rx(roll), Z-up
            // In Unity Y-up: axes map as {X_U=-Y_M, Y_U=Z_M, Z_U=X_M}
            // After axis swap + sign flip for LH → use negated roll/pitch
            Quaternion rot = Quaternion.Euler(
                roll * Mathf.Rad2Deg * -1,
                pitch * Mathf.Rad2Deg * -1,
                yaw * Mathf.Rad2Deg * 1);

            return (pos, rot);
        }

        private static Vector3 ParseAxis(string axisStr)
        {
            string[] p = axisStr.Split(' ');
            float mx = float.Parse(p[0]), my = float.Parse(p[1]), mz = float.Parse(p[2]);
            // URDF axis in Z-up → Unity Y-up
            return new Vector3(-my, mz, mx).normalized;
        }

        private static float ParseFloat(string s, float def = 0f)
        {
            if (string.IsNullOrEmpty(s)) return def;
            return float.TryParse(s, out float v) ? v : def;
        }

        private static string GetAttr(XmlNode node, string name)
        {
            return node?.Attributes?[name]?.Value;
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

        private class LinkInfo
        {
            public List<string> VisualPaths = new List<string>();
            public List<string> CollisionPaths = new List<string>();
            public (Vector3 pos, Quaternion rot) VisualOffset = (Vector3.zero, Quaternion.identity);
            public (Vector3 pos, Quaternion rot) CollisionOffset = (Vector3.zero, Quaternion.identity);
            public Color Color = Color.white;
        }
    }
}
