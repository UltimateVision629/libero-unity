using System.Collections.Generic;
using LIBERO.Networking;
using Mujoco;
using UnityEngine;

namespace LIBERO.Core
{
    public class MjJoyConController : MonoBehaviour
    {
        [Header("References")]
        public JoyConReceiver JoyConInput;

        [Header("Warmup")]
        public int WarmupFrames = 60;

        private Dictionary<string, MjActuator> _lArm = new();
        private Dictionary<string, MjActuator> _rArm = new();
        private static readonly string[] _jointTypes = { "Wrist_Pitch", "Wrist_Roll", "Rotation", "Pitch", "Elbow", "Jaw" };

        private int _warmup;

        private void Awake()
        {
            if (JoyConInput == null)
            {
                JoyConInput = FindObjectOfType<JoyConReceiver>();
                if (JoyConInput == null)
                {
                    var go = new GameObject("JoyConReceiver");
                    JoyConInput = go.AddComponent<JoyConReceiver>();
                }
            }

            MjScene.Instance.postInitEvent += OnSceneReady;
        }

        private void OnSceneReady(object sender, MjStepArgs e)
        {
            FindActuators();
            MjScene.Instance.preUpdateEvent += OnPreUpdate;
        }

        private void FindActuators()
        {
            _lArm.Clear();
            _rArm.Clear();

            foreach (var act in FindObjectsOfType<MjActuator>())
            {
                if (act.MujocoName == null) continue;
                string name = act.MujocoName;

                if (name.StartsWith("L_")) MapActuator(_lArm, name, act);
                else if (name.StartsWith("R_")) MapActuator(_rArm, name, act);
            }

            Debug.Log($"[MjJoyCon] L arm actuators: {_lArm.Count}, R arm: {_rArm.Count}");
        }

        private void MapActuator(Dictionary<string, MjActuator> dict, string name, MjActuator act)
        {
            foreach (var jt in _jointTypes)
                if (name.Contains(jt)) { dict[jt] = act; return; }
        }

        private unsafe void OnPreUpdate(object sender, MjStepArgs e)
        {
            if (JoyConInput == null || !JoyConInput.HasData) return;
            if (_warmup < WarmupFrames) { _warmup++; return; }

            ProcessArm(JoyConInput.GetRobotPose(0), _rArm, 0);
            ProcessArm(JoyConInput.GetRobotPose(1), _lArm, 1);
        }

        private unsafe void ProcessArm(JoyConPose jc, Dictionary<string, MjActuator> arm, int index)
        {
            if (arm.Count == 0) return;

            if (jc.joints != null && jc.joints.Length >= 5)
            {
                SetJoint(arm, "Rotation", jc.joints[0]);
                SetJoint(arm, "Pitch", jc.joints[1]);
                SetJoint(arm, "Elbow", jc.joints[2]);
                SetJoint(arm, "Wrist_Pitch", jc.joints[3]);
                SetJoint(arm, "Wrist_Roll", jc.joints[4]);

                float jaw = jc.joints.Length > 5 ? jc.joints[5] : jc.gripper;
                SetJoint(arm, "Jaw", jaw);

                float[] curJoints = new float[5];
                curJoints[0] = jc.joints[0];
                curJoints[1] = jc.joints[1];
                curJoints[2] = jc.joints[2];
                curJoints[3] = jc.joints[3];
                curJoints[4] = jc.joints[4];
                JoyConInput.SetJointFeedback(index, curJoints);
            }
        }

        private unsafe void SetJoint(Dictionary<string, MjActuator> arm, string jointType, float valueRad)
        {
            if (!arm.TryGetValue(jointType, out var act) || act.Joint == null) return;

            var data = MjScene.Instance.Data;
            var model = MjScene.Instance.Model;

            if (jointType == "Rotation")
            {
                // Rotation actuator broken in MuJoCo 3.2.4 — use qpos teleport (no collision impact)
                int jid = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_JOINT, act.Joint.MujocoName);
                if (jid >= 0)
                {
                    data->qpos[model->jnt_qposadr[jid]] = valueRad;
                    data->qvel[model->jnt_dofadr[jid]] = 0;
                }
            }
            else
            {
                // Force-driven via actuator — respects collision physics
                data->ctrl[act.MujocoId] = valueRad;
            }
            act.Control = valueRad;
        }

        private void OnDestroy()
        {
            if (MjScene.InstanceExists)
            {
                MjScene.Instance.postInitEvent -= OnSceneReady;
                MjScene.Instance.preUpdateEvent -= OnPreUpdate;
            }
        }
    }
}
