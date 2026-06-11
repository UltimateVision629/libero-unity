using System;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;

namespace LIBERO.Core
{
    public struct Observation
    {
        public byte[] AgentviewImage;
        public byte[] EyeInHandImage;
        public int ImageWidth;
        public int ImageHeight;

        public float[] JointPositions;   // robot_0 (right arm)
        public float[] EEFPosition;
        public float[] EEFQuaternion;
        public float[] GripperQPos;

        public float[] JointPositions1;  // robot_1 (left arm)
        public float[] EEFPosition1;
        public float[] EEFQuaternion1;
        public float[] GripperQPos1;

        public Dictionary<string, float[]> ObjectPositions;
        public Dictionary<string, float[]> ObjectQuaternions;

        public Dictionary<string, object> ToPythonDict()
        {
            var dict = new Dictionary<string, object>
            {
                ["agentview_image"] = AgentviewImage,
                ["eye_in_hand_image"] = EyeInHandImage,
                ["robot0_joint_pos"] = JointPositions,
                ["robot0_eef_pos"] = EEFPosition,
                ["robot0_eef_quat"] = EEFQuaternion,
                ["robot0_gripper_qpos"] = GripperQPos,
                ["robot1_joint_pos"] = JointPositions1,
                ["robot1_eef_pos"] = EEFPosition1,
                ["robot1_eef_quat"] = EEFQuaternion1,
                ["robot1_gripper_qpos"] = GripperQPos1
            };

            if (ObjectPositions != null)
            {
                foreach (var kv in ObjectPositions)
                    dict[$"{kv.Key}_pos"] = kv.Value;
            }
            if (ObjectQuaternions != null)
            {
                foreach (var kv in ObjectQuaternions)
                    dict[$"{kv.Key}_quat"] = kv.Value;
            }

            return dict;
        }
    }

    public class ObservationCollector : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AutoCreate()
        {
            if (FindObjectOfType<ObservationCollector>() != null) return;
            var go = new GameObject("ObservationCollector");
            DontDestroyOnLoad(go);
            go.AddComponent<ObservationCollector>();
        }

        [Header("Camera Settings")]
        public Camera AgentviewCamera;
        public Camera EyeInHandCamera;
        public int ImageWidth = 128;
        public int ImageHeight = 128;

        [Header("Robot References")]
        public GameObject RobotRoot;
        public ArticulationBody[] JointBodies;
        public RobotArmController Robot0;
        public RobotArmController Robot1;

        private RenderTexture _agentviewRT;
        private RenderTexture _eyeInHandRT;
        private Texture2D _agentviewTex;
        private Texture2D _eyeInHandTex;

        private void Awake()
        {
            _agentviewRT = new RenderTexture(ImageWidth, ImageHeight, 24, RenderTextureFormat.ARGB32);
            _eyeInHandRT = new RenderTexture(ImageWidth, ImageHeight, 24, RenderTextureFormat.ARGB32);
            _agentviewTex = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGB24, false);
            _eyeInHandTex = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGB24, false);

            if (AgentviewCamera == null)
                CreateAgentviewCamera();
        }

        private void CreateAgentviewCamera()
        {
            var camGo = new GameObject("Agentview Camera");
            camGo.transform.SetParent(transform);
            var cam = camGo.AddComponent<Camera>();

            // Read position from MuJoCo XML <site name="agentview_site">
            var site = GameObject.Find("agentview_site");
            if (site != null)
            {
                camGo.transform.position = site.transform.position;
                camGo.transform.rotation = site.transform.rotation;
                Debug.Log($"[ObsCollector] Camera placed at site agentview_site: {site.transform.position}");
            }
            else
            {
                // Fallback hardcoded position
                camGo.transform.position = new Vector3(0f, 0.50f, -0.70f);
                Debug.Log("[ObsCollector] agentview_site not found, using fallback position");
            }

            camGo.transform.LookAt(Vector3.zero);
            cam.nearClipPlane = 0.1f;
            cam.enabled = true;
            AgentviewCamera = cam;
        }

        public Observation Collect(Dictionary<string, ObjectState> objectStates, RobotArmController robot)
        {
            var obs = new Observation
            {
                ImageWidth = ImageWidth,
                ImageHeight = ImageHeight,
                ObjectPositions = new Dictionary<string, float[]>(),
                ObjectQuaternions = new Dictionary<string, float[]>()
            };

            // Capture camera images
            if (AgentviewCamera != null)
            {
                AgentviewCamera.targetTexture = _agentviewRT;
                AgentviewCamera.Render();
                RenderTexture.active = _agentviewRT;
                _agentviewTex.ReadPixels(new Rect(0, 0, ImageWidth, ImageHeight), 0, 0);
                _agentviewTex.Apply();
                obs.AgentviewImage = _agentviewTex.GetRawTextureData();
                RenderTexture.active = null;
                AgentviewCamera.targetTexture = null;
            }

            if (EyeInHandCamera != null)
            {
                EyeInHandCamera.targetTexture = _eyeInHandRT;
                EyeInHandCamera.Render();
                RenderTexture.active = _eyeInHandRT;
                _eyeInHandTex.ReadPixels(new Rect(0, 0, ImageWidth, ImageHeight), 0, 0);
                _eyeInHandTex.Apply();
                obs.EyeInHandImage = _eyeInHandTex.GetRawTextureData();
                RenderTexture.active = null;
                EyeInHandCamera.targetTexture = null;
            }

            // Robot proprioception — use passed robot or public fields, fall back to MuJoCo
            var r0 = robot ?? Robot0;
            var r1 = Robot1;

            if (r0 != null)
            {
                r0.GetJointPositions(out float[] jointPositions);
                r0.GetEEFPose(out Vector3 eefPos, out Quaternion eefQuat);
                r0.GetGripperState(out float[] gripperQPos);

                obs.JointPositions = jointPositions;
                obs.EEFPosition = new float[] { eefPos.x, eefPos.y, eefPos.z };
                obs.EEFQuaternion = new float[] { eefQuat.x, eefQuat.y, eefQuat.z, eefQuat.w };
                obs.GripperQPos = gripperQPos;
            }

            if (r1 != null)
            {
                r1.GetJointPositions(out float[] jointPositions1);
                r1.GetEEFPose(out Vector3 eefPos1, out Quaternion eefQuat1);
                r1.GetGripperState(out float[] gripperQPos1);

                obs.JointPositions1 = jointPositions1;
                obs.EEFPosition1 = new float[] { eefPos1.x, eefPos1.y, eefPos1.z };
                obs.EEFQuaternion1 = new float[] { eefQuat1.x, eefQuat1.y, eefQuat1.z, eefQuat1.w };
                obs.GripperQPos1 = gripperQPos1;
            }

            if (r0 == null && r1 == null)
            {
                var mjJoy = FindObjectOfType<MjJoyConController>();
                if (mjJoy != null)
                {
                    if (mjJoy.TryGetRobotState(0, out float[] j0, out Vector3 p0, out Quaternion q0))
                    {
                        obs.JointPositions = j0;
                        obs.EEFPosition = new float[] { p0.x, p0.y, p0.z };
                        obs.EEFQuaternion = new float[] { q0.x, q0.y, q0.z, q0.w };
                        obs.GripperQPos = new float[] { j0.Length > 0 ? j0[j0.Length - 1] : 0f, 0f };
                        Debug.Log($"[ObsCollector] Robot0 OK: joints={j0.Length}, eef=({p0.x:F3},{p0.y:F3},{p0.z:F3})");
                    }
                    if (mjJoy.TryGetRobotState(1, out float[] j1, out Vector3 p1, out Quaternion q1))
                    {
                        obs.JointPositions1 = j1;
                        obs.EEFPosition1 = new float[] { p1.x, p1.y, p1.z };
                        obs.EEFQuaternion1 = new float[] { q1.x, q1.y, q1.z, q1.w };
                        obs.GripperQPos1 = new float[] { j1.Length > 0 ? j1[j1.Length - 1] : 0f, 0f };
                        Debug.Log($"[ObsCollector] Robot1 OK: joints={j1.Length}, eef=({p1.x:F3},{p1.y:F3},{p1.z:F3})");
                    }
                }
            }

            // Object states
            if (objectStates != null)
            {
                foreach (var kv in objectStates)
                {
                    var pos = kv.Value.Position;
                    var rot = kv.Value.Rotation;
                    obs.ObjectPositions[kv.Key] = new float[] { pos.x, pos.y, pos.z };
                    obs.ObjectQuaternions[kv.Key] = new float[] { rot.x, rot.y, rot.z, rot.w };
                }
            }

            return obs;
        }

        private void OnDestroy()
        {
            if (_agentviewRT != null) _agentviewRT.Release();
            if (_eyeInHandRT != null) _eyeInHandRT.Release();
            if (_agentviewTex != null) Destroy(_agentviewTex);
            if (_eyeInHandTex != null) Destroy(_eyeInHandTex);
        }
    }
}
