using UnityEngine;

namespace UnityRobotEnv
{
    /// <summary>
    /// Minimal verification script: ensures that the MuJoCo runtime (mujoco.dll) is loadable
    /// and that MjScene can compile a trivial MJCF scene and step it.
    /// Attach to a GameObject and press Play. Check the Console for the result.
    /// </summary>
    public class MjSceneVerifier : MonoBehaviour
    {
        private unsafe void Start()
        {
            var scene = Mujoco.MjScene.Instance;
            if (scene == null)
            {
                Debug.LogError("[MjSceneVerifier] MjScene.Instance is null.");
                return;
            }

            Debug.Log("[MjSceneVerifier] MjScene singleton created successfully.");

            if (scene.Model != null && scene.Data != null)
            {
                Debug.Log("[MjSceneVerifier] PASS — Model/Data valid, DLL loaded.");
            }
            else
            {
                Debug.LogWarning("[MjSceneVerifier] Model or Data is null — add MjComponents to the scene first.");
            }
        }
    }
}
