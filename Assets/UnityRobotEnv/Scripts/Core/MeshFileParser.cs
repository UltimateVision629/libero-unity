using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

namespace UnityRobotEnv.Core
{
    public static class MeshFileParser
    {
        public static Mesh Load(string filePath, Vector3 scale)
        {
            string ext = Path.GetExtension(filePath).ToLowerInvariant();
            if (ext == ".msh") return LoadMsh(filePath, scale);
            if (ext == ".obj") return LoadObj(filePath, scale);
            if (ext == ".stl") return LoadStl(filePath, scale);
            throw new System.NotSupportedException($"Unsupported mesh format: {ext}");
        }

        private static Mesh LoadMsh(string filePath, Vector3 scale)
        {
            byte[] data = File.ReadAllBytes(filePath);
            int offset = 0;

            int nv = ReadInt32(data, ref offset);
            int nn = ReadInt32(data, ref offset);
            int nt = ReadInt32(data, ref offset);
            int nf = ReadInt32(data, ref offset);

            Vector3[] vertices = new Vector3[nv];
            for (int i = 0; i < nv; i++)
            {
                float mx = ReadFloat32(data, ref offset);
                float my = ReadFloat32(data, ref offset);
                float mz = ReadFloat32(data, ref offset);
                vertices[i] = new Vector3(-my * scale.y, mz * scale.z, mx * scale.x);
            }

            Vector3[] normals = new Vector3[nn];
            for (int i = 0; i < nn; i++)
            {
                float nx = ReadFloat32(data, ref offset);
                float ny = ReadFloat32(data, ref offset);
                float nz = ReadFloat32(data, ref offset);
                normals[i] = new Vector3(-ny, nz, nx).normalized;
            }

            Vector2[] uv = new Vector2[nt];
            for (int i = 0; i < nt; i++)
            {
                float u = ReadFloat32(data, ref offset);
                float v = 1f - ReadFloat32(data, ref offset);
                uv[i] = new Vector2(u, v);
            }

            int[] triangles = new int[nf * 3];
            for (int i = 0; i < nf; i++)
            {
                int a = ReadInt32(data, ref offset);
                int b = ReadInt32(data, ref offset);
                int c = ReadInt32(data, ref offset);
                triangles[i * 3] = a;
                triangles[i * 3 + 1] = c;
                triangles[i * 3 + 2] = b;
            }

            Mesh mesh = new Mesh { name = Path.GetFileNameWithoutExtension(filePath), indexFormat = nv > 65535 ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16 };
            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.uv = uv;
            mesh.triangles = triangles;
            mesh.RecalculateBounds();
            return mesh;
        }

        private static Mesh LoadObj(string filePath, Vector3 scale)
        {
            string[] lines = File.ReadAllLines(filePath);

            List<Vector3> rawVerts = new List<Vector3>();
            List<Vector2> rawUv = new List<Vector2>();
            List<Vector3> rawNormals = new List<Vector3>();

            foreach (string line in lines)
            {
                if (line.StartsWith("v ")) { string[] p = line.Split(' '); float mx = float.Parse(p[1]); float my = float.Parse(p[2]); float mz = float.Parse(p[3]); rawVerts.Add(new Vector3(-my * scale.y, mz * scale.z, mx * scale.x)); }
                if (line.StartsWith("vt ")) { string[] p = line.Split(' '); rawUv.Add(new Vector2(float.Parse(p[1]), 1f - float.Parse(p[2]))); }
                if (line.StartsWith("vn ")) { string[] p = line.Split(' '); float nx = float.Parse(p[1]); float ny = float.Parse(p[2]); float nz = float.Parse(p[3]); rawNormals.Add(new Vector3(-ny, nz, nx).normalized); }
            }

            List<Vector3> vertices = new List<Vector3>();
            List<Vector2> uv = new List<Vector2>();
            List<Vector3> normals = new List<Vector3>();
            List<int> triangles = new List<int>();
            Dictionary<string, int> indexMap = new Dictionary<string, int>();

            foreach (string line in lines)
            {
                if (!line.StartsWith("f ")) continue;
                string[] parts = line.Split(' ');
                int[] triIndices = new int[3];
                for (int i = 0; i < 3; i++)
                {
                    string[] comp = parts[i + 1].Split('/');
                    int vi = comp.Length > 0 && int.TryParse(comp[0], out int v) ? v - 1 : 0;
                    int ti = comp.Length > 1 && int.TryParse(comp[1], out int t) ? t - 1 : -1;
                    int ni = comp.Length > 2 && int.TryParse(comp[2], out int n) ? n - 1 : -1;
                    string key = $"{vi}/{ti}/{ni}";
                    if (!indexMap.TryGetValue(key, out int idx))
                    {
                        idx = vertices.Count;
                        indexMap[key] = idx;
                        vertices.Add(vi < rawVerts.Count ? rawVerts[vi] : Vector3.zero);
                        uv.Add(ti >= 0 && ti < rawUv.Count ? rawUv[ti] : Vector2.zero);
                        normals.Add(ni >= 0 && ni < rawNormals.Count ? rawNormals[ni] : Vector3.up);
                    }
                    triIndices[i] = idx;
                }
                triangles.Add(triIndices[0]);
                triangles.Add(triIndices[2]);
                triangles.Add(triIndices[1]);
            }

            Mesh mesh = new Mesh { name = Path.GetFileNameWithoutExtension(filePath), indexFormat = vertices.Count > 65535 ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16 };
            mesh.vertices = vertices.ToArray();
            mesh.uv = uv.ToArray();
            mesh.normals = normals.ToArray();
            mesh.triangles = triangles.ToArray();
            mesh.RecalculateBounds();
            return mesh;
        }

        private static Mesh LoadStl(string filePath, Vector3 scale)
        {
            byte[] data = File.ReadAllBytes(filePath);
            int nf = (int)System.BitConverter.ToUInt32(data, 80);
            int expectedLen = 84 + nf * 50;
            if (data.Length < expectedLen) nf = (data.Length - 84) / 50;
            if (nf <= 0) throw new System.Exception("Empty STL file");

            int vCount = nf * 3;
            Vector3[] vertices = new Vector3[vCount];
            Vector3[] normals = new Vector3[vCount];
            int[] triangles = new int[nf * 3];
            int offset = 84;

            for (int i = 0; i < nf; i++)
            {
                float nx = ReadFloat32(data, ref offset);
                float ny = ReadFloat32(data, ref offset);
                float nz = ReadFloat32(data, ref offset);
                Vector3 n = new Vector3(-ny, nz, nx).normalized;

                int vi0 = i * 3;
                int vi1 = i * 3 + 1;
                int vi2 = i * 3 + 2;

                float mx = ReadFloat32(data, ref offset);
                float my = ReadFloat32(data, ref offset);
                float mz = ReadFloat32(data, ref offset);
                vertices[vi0] = new Vector3(-my * scale.y, mz * scale.z, mx * scale.x);
                normals[vi0] = n;

                mx = ReadFloat32(data, ref offset);
                my = ReadFloat32(data, ref offset);
                mz = ReadFloat32(data, ref offset);
                vertices[vi1] = new Vector3(-my * scale.y, mz * scale.z, mx * scale.x);
                normals[vi1] = n;

                mx = ReadFloat32(data, ref offset);
                my = ReadFloat32(data, ref offset);
                mz = ReadFloat32(data, ref offset);
                vertices[vi2] = new Vector3(-my * scale.y, mz * scale.z, mx * scale.x);
                normals[vi2] = n;

                triangles[vi0] = vi0;
                triangles[vi1] = vi2;
                triangles[vi2] = vi1;

                offset += 2;
            }

            Mesh mesh = new Mesh { name = Path.GetFileNameWithoutExtension(filePath), indexFormat = vCount > 65535 ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16 };
            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.triangles = triangles;
            mesh.RecalculateBounds();
            return mesh;
        }

        private static int ReadInt32(byte[] data, ref int offset)
        {
            int val = System.BitConverter.ToInt32(data, offset);
            offset += 4;
            return val;
        }

        private static float ReadFloat32(byte[] data, ref int offset)
        {
            float val = System.BitConverter.ToSingle(data, offset);
            offset += 4;
            return val;
        }
    }
}
