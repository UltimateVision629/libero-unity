using System;
using System.Collections.Generic;
using Mujoco;
using UnityEngine;

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

            // Robot proprioception
            if (robot != null)
            {
                robot.GetJointPositions(out float[] jointPositions);
                robot.GetEEFPose(out Vector3 eefPos, out Quaternion eefQuat);
                robot.GetGripperState(out float[] gripperQPos);

                obs.JointPositions = jointPositions;
                obs.EEFPosition = new float[] { eefPos.x, eefPos.y, eefPos.z };
                obs.EEFQuaternion = new float[] { eefQuat.x, eefQuat.y, eefQuat.z, eefQuat.w };
                obs.GripperQPos = gripperQPos;
            }
            else if (MjScene.InstanceExists)
            {
                CollectMuJoCoProprioception(ref obs, "R_", isRobot1: false);
                CollectMuJoCoProprioception(ref obs, "L_", isRobot1: true);
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

        private unsafe void CollectMuJoCoProprioception(ref Observation obs, string prefix, bool isRobot1)
        {
            var model = MjScene.Instance.Model;
            var data = MjScene.Instance.Data;

            string[] jointNames = { "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw" };
            float[] positions = new float[6];
            for (int i = 0; i < 6; i++)
            {
                int jid = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_JOINT, $"{prefix}_{jointNames[i]}");
                positions[i] = jid >= 0 ? (float)data->qpos[jid] : 0f;
            }

            int sid = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_SITE, $"{prefix}_eef_site");
            Vector3 eefPos = Vector3.zero;
            Quaternion eefQuat = Quaternion.identity;
            if (sid >= 0)
            {
                eefPos = MjEngineTool.UnityVector3(data->site_xpos + sid * 3);
                eefQuat = MjEngineTool.UnityQuaternionFromMatrix(data->site_xmat + sid * 9);
            }

            if (isRobot1)
            {
                obs.JointPositions1 = positions;
                obs.EEFPosition1 = new float[] { eefPos.x, eefPos.y, eefPos.z };
                obs.EEFQuaternion1 = new float[] { eefQuat.x, eefQuat.y, eefQuat.z, eefQuat.w };
                obs.GripperQPos1 = new float[] { positions[5], 0f };
            }
            else
            {
                obs.JointPositions = positions;
                obs.EEFPosition = new float[] { eefPos.x, eefPos.y, eefPos.z };
                obs.EEFQuaternion = new float[] { eefQuat.x, eefQuat.y, eefQuat.z, eefQuat.w };
                obs.GripperQPos = new float[] { positions[5], 0f };
            }
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
