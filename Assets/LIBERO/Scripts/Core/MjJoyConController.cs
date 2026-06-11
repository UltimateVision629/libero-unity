using System.Collections.Generic;
using LIBERO.Networking;
using Mujoco;
using UnityEngine;

namespace LIBERO.Core
{
    public class MjJoyConController : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AutoCreate()
        {
            if (FindObjectOfType<MjJoyConController>() != null) return;
            var go = new GameObject("MjJoyConController");
            DontDestroyOnLoad(go);
            go.AddComponent<MjJoyConController>();
        }

        [Header("References")]
        public JoyConReceiver JoyConInput;

        [Header("Warmup")]
        public int WarmupFrames = 60;

        [Header("Rate Limiting")]
        public float MaxJointDelta = 0.3f;   // max single-joint change per step (rad)
        public float MaxEEFRotDelta = 1.0f;  // warning threshold for EEF rotation per step (rad)

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

        private Quaternion _prevEefRot0 = Quaternion.identity;
        private Quaternion _prevEefRot1 = Quaternion.identity;
        private bool _prevEefValid;

        private unsafe void ProcessArm(JoyConPose jc, Dictionary<string, MjActuator> arm, int index)
        {
            if (arm.Count == 0) return;

            if (jc.button == 1)
            {
                ResetArm(arm);
                _prevEefValid = false;
                return;
            }

            if (jc.joints != null && jc.joints.Length >= 5)
            {
                SetJoint(arm, "Rotation", jc.joints[0]);
                SetJoint(arm, "Pitch", jc.joints[1]);
                SetJoint(arm, "Elbow", jc.joints[2]);
                SetJoint(arm, "Wrist_Pitch", jc.joints[3]);
                SetJoint(arm, "Wrist_Roll", jc.joints[4]);

                float jaw = jc.joints.Length > 5 ? jc.joints[5] : jc.gripper;
                SetJoint(arm, "Jaw", jaw);

                // EEF rotation monitor
                string prefix = index == 0 ? "R_" : "L_";
                var siteGo = GameObject.Find($"{prefix}_eef_site");
                if (siteGo != null)
                {
                    Quaternion curRot = siteGo.transform.rotation;
                    ref Quaternion prevRot = ref (index == 0 ? ref _prevEefRot0 : ref _prevEefRot1);
                    if (_prevEefValid)
                    {
                        float angle = Quaternion.Angle(prevRot, curRot) * Mathf.Deg2Rad;
                        if (angle > MaxEEFRotDelta)
                            Debug.LogWarning($"[MjJoyCon] Arm{index} EEF rotation {angle:F2} rad exceeds limit {MaxEEFRotDelta} rad!");
                    }
                    prevRot = curRot;
                    _prevEefValid = true;
                }

                float[] curJoints = new float[5];
                curJoints[0] = jc.joints[0];
                curJoints[1] = jc.joints[1];
                curJoints[2] = jc.joints[2];
                curJoints[3] = jc.joints[3];
                curJoints[4] = jc.joints[4];
                JoyConInput.SetJointFeedback(index, curJoints);
            }
        }

        private unsafe int FindJointId(MjActuator act)
        {
            var model = MjScene.Instance.Model;
            int jid = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_JOINT, act.MujocoName);
            if (jid >= 0) return jid;
            if (act.Joint != null)
            {
                jid = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_JOINT, act.Joint.name);
                if (jid >= 0) return jid;
            }
            return -1;
        }

        private unsafe void SetJoint(Dictionary<string, MjActuator> arm, string jointType, float valueRad)
        {
            if (!arm.TryGetValue(jointType, out var act) || act.Joint == null) return;

            var data = MjScene.Instance.Data;
            var model = MjScene.Instance.Model;

            int jid = FindJointId(act);
            if (jid >= 0)
            {
                float current = (float)data->qpos[model->jnt_qposadr[jid]];
                float delta = valueRad - current;
                float clamped = Mathf.Clamp(delta, -MaxJointDelta, MaxJointDelta);
                valueRad = current + clamped;
            }

            data->ctrl[act.MujocoId] = valueRad;
            act.Control = valueRad;
        }

        private void ResetArm(Dictionary<string, MjActuator> arm)
        {
            // Bypass rate limiting for reset
            if (arm.TryGetValue("Rotation", out var r0) && r0.Joint != null) { unsafe { var d = MjScene.Instance.Data; d->ctrl[r0.MujocoId] = 0f; r0.Control = 0f; } }
            if (arm.TryGetValue("Pitch", out var r1) && r1.Joint != null) { unsafe { var d = MjScene.Instance.Data; d->ctrl[r1.MujocoId] = -3.14f; r1.Control = -3.14f; } }
            if (arm.TryGetValue("Elbow", out var r2) && r2.Joint != null) { unsafe { var d = MjScene.Instance.Data; d->ctrl[r2.MujocoId] = 3.14f; r2.Control = 3.14f; } }
            if (arm.TryGetValue("Wrist_Pitch", out var r3) && r3.Joint != null) { unsafe { var d = MjScene.Instance.Data; d->ctrl[r3.MujocoId] = 0.0f; r3.Control = 0.0f; } }
            if (arm.TryGetValue("Wrist_Roll", out var r4) && r4.Joint != null) { unsafe { var d = MjScene.Instance.Data; d->ctrl[r4.MujocoId] = -1.57f; r4.Control = -1.57f; } }
            if (arm.TryGetValue("Jaw", out var r5) && r5.Joint != null) { unsafe { var d = MjScene.Instance.Data; d->ctrl[r5.MujocoId] = 0.04f; r5.Control = 0.04f; } }
            Debug.Log("[MjJoyCon] Arm reset to home pose");
        }

        public unsafe bool TryGetRobotState(int robotIndex, out float[] joints, out Vector3 eefPos, out Quaternion eefQuat)
        {
            joints = null;
            eefPos = Vector3.zero;
            eefQuat = Quaternion.identity;

            var arm = robotIndex == 0 ? _rArm : _lArm;
            if (arm.Count == 0 || !MjScene.InstanceExists) return false;

            try
            {
                var data = MjScene.Instance.Data;
                joints = new float[6];
                foreach (var kv in arm)
                {
                    var act = kv.Value;
                    if (act == null || act.Joint == null) continue;

                    int idx = System.Array.IndexOf(_jointTypes, kv.Key);
                    if (idx < 0) continue;

                    int jid = FindJointId(act);
                    if (jid >= 0)
                        joints[idx] = (float)data->qpos[MjScene.Instance.Model->jnt_qposadr[jid]];
                }

                string prefix = robotIndex == 0 ? "R_" : "L_";
                var siteGo = GameObject.Find($"{prefix}_eef_site");
                if (siteGo != null)
                {
                    eefPos = siteGo.transform.position;
                    eefQuat = siteGo.transform.rotation;
                }
                else
                {
                    var lastAct = FindLastActuator(arm);
                    if (lastAct != null && lastAct.Joint != null)
                    {
                        eefPos = lastAct.Joint.transform.position;
                        eefQuat = lastAct.Joint.transform.rotation;
                    }
                }

                return true;
            }
            catch (System.Exception ex)
            {
                Debug.LogWarning($"[MjJoyCon] TryGetRobotState({robotIndex}) failed: {ex.Message}");
                return false;
            }
        }

        private MjActuator FindLastActuator(Dictionary<string, MjActuator> arm)
        {
            MjActuator result = null;
            float maxDist = -1f;
            foreach (var kv in arm)
            {
                if (kv.Value?.Joint == null) continue;
                float dist = Vector3.Distance(kv.Value.Joint.transform.position, Vector3.zero);
                if (dist > maxDist) { maxDist = dist; result = kv.Value; }
            }
            return result;
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
