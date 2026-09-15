using UnityEngine;

namespace UnityRobotEnv.Core
{
    public class AssetDatabase
    {
        private static string _assetRoot;

        public static void SetAssetRoot(string path)
        {
            _assetRoot = path;
        }

        public static string GetAssetPath(string relativePath)
        {
            if (!string.IsNullOrEmpty(_assetRoot))
                return System.IO.Path.Combine(_assetRoot, relativePath);
            return System.IO.Path.Combine(Application.dataPath, "UnityRobotEnv/assets", relativePath);
        }

        public static string GetScenePath(string sceneName)
        {
            return GetAssetPath(System.IO.Path.Combine("scenes", sceneName));
        }

        public static string GetBDDLPath(string suiteName, string bddlFileName)
        {
            return GetAssetPath(System.IO.Path.Combine("bddl_files", suiteName, bddlFileName));
        }
    }
}
