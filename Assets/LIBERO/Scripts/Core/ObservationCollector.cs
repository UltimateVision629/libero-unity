using System;
using System.Collections.Generic;
using UnityEngine;

namespace LIBERO.Core
{
    public struct Observation
    {
        public byte[] AgentviewImage;
        public byte[] EyeInHandImage;
        public int ImageWidth;
        public int ImageHeight;

        public float[] JointPositions;
        public float[] EEFPosition;
        public float[] EEFQuaternion;
        public float[] GripperQPos;

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
                ["robot0_gripper_qpos"] = GripperQPos
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
