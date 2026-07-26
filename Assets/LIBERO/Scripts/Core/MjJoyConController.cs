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
            // Warmup: hold arms at home pose so they don't droop at qpos=0
            if (_warmup < WarmupFrames)
            {
                if (_warmup == 0)
                {
                    ResetArm(_rArm);
                    ResetArm(_lArm);
                }
                _warmup++;
                return;
            }

            if (JoyConInput == null || !JoyConInput.HasData) return;

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

        // ── Inference: EEF delta → joint delta via finite-difference Jacobian ──

        private unsafe int GetSiteId(string siteName)
        {
            return MujocoLib.mj_name2id(MjScene.Instance.Model, (int)MujocoLib.mjtObj.mjOBJ_SITE, siteName);
        }

        private unsafe Vector3 GetEefPos(int robotIndex)
        {
            string siteName = robotIndex == 0 ? "R_eef_site" : "L_eef_site";
            int sid = GetSiteId(siteName);
            if (sid < 0) return Vector3.zero;
            double* sp = MjScene.Instance.Data->site_xpos;
            return new Vector3((float)sp[sid * 3], (float)sp[sid * 3 + 1], (float)sp[sid * 3 + 2]);
        }

        private unsafe float[] GetCurrentQ(int robotIndex)
        {
            var arm = robotIndex == 0 ? _rArm : _lArm;
            float[] q = new float[5];
            string[] order = { "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll" };
            var model = MjScene.Instance.Model;
            var data = MjScene.Instance.Data;
            for (int i = 0; i < 5; i++)
            {
                if (arm.TryGetValue(order[i], out var act))
                {
                    int jid = model->actuator_trnid[2 * act.MujocoId + 1];
                    if (jid >= 0) q[i] = (float)data->qpos[model->jnt_qposadr[jid]];
                }
            }
            return q;
        }

        private unsafe void SetQpos(int robotIndex, float[] q)
        {
            var arm = robotIndex == 0 ? _rArm : _lArm;
            string[] order = { "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll" };
            var model = MjScene.Instance.Model;
            var data = MjScene.Instance.Data;
            for (int i = 0; i < 5; i++)
            {
                if (arm.TryGetValue(order[i], out var act))
                {
                    int jid = model->actuator_trnid[2 * act.MujocoId + 1];
                    if (jid >= 0) data->qpos[model->jnt_qposadr[jid]] = q[i];
                }
            }
        }

        private unsafe float[][] ComputeJacobian3x5(int robotIndex, float epsilon = 0.005f)
        {
            float[] q0 = GetCurrentQ(robotIndex);
            Vector3 p0 = GetEefPos(robotIndex);
            float[][] J = new float[3][] { new float[5], new float[5], new float[5] };

            for (int col = 0; col < 5; col++)
            {
                float[] qPerturb = (float[])q0.Clone();
                qPerturb[col] += epsilon;
                SetQpos(robotIndex, qPerturb);
                MujocoLib.mj_kinematics(MjScene.Instance.Model, MjScene.Instance.Data);
                Vector3 p1 = GetEefPos(robotIndex);

                J[0][col] = (p1.x - p0.x) / epsilon;
                J[1][col] = (p1.y - p0.y) / epsilon;
                J[2][col] = (p1.z - p0.z) / epsilon;
            }

            SetQpos(robotIndex, q0);
            MujocoLib.mj_kinematics(MjScene.Instance.Model, MjScene.Instance.Data);
            return J;
        }

        private float[] DampLeastSquares3x5(float[][] J, Vector3 dx, float lambda = 0.05f)
        {
            int m = 3, n = 5;
            float[,] A = new float[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                {
                    float s = 0f;
                    for (int k = 0; k < m; k++) s += J[k][i] * J[k][j];
                    A[i, j] = s + ((i == j) ? lambda * lambda : 0f);
                }

            float[] b = new float[n];
            for (int i = 0; i < n; i++)
            {
                float s = 0f;
                for (int k = 0; k < m; k++) s += J[k][i] * dx[k];
                b[i] = s;
            }

            // Gaussian elimination
            for (int col = 0; col < n; col++)
            {
                int maxRow = col;
                float maxVal = Mathf.Abs(A[col, col]);
                for (int row = col + 1; row < n; row++)
                    if (Mathf.Abs(A[row, col]) > maxVal) { maxVal = Mathf.Abs(A[row, col]); maxRow = row; }
                if (maxVal < 1e-10f) continue;
                if (maxRow != col)
                {
                    for (int j = 0; j < n; j++) { float t = A[col, j]; A[col, j] = A[maxRow, j]; A[maxRow, j] = t; }
                    float tb = b[col]; b[col] = b[maxRow]; b[maxRow] = tb;
                }
                float pivot = A[col, col];
                for (int j = col; j < n; j++) A[col, j] /= pivot;
                b[col] /= pivot;
                for (int row = 0; row < n; row++)
                {
                    if (row == col) continue;
                    float factor = A[row, col];
                    for (int j = col; j < n; j++) A[row, j] -= factor * A[col, j];
                    b[row] -= factor * b[col];
                }
            }
            return b;
        }

        private int _applyCount = 0;
        /// <summary>Apply EEF delta [dx,dy,dz,dRx,dRy,dRz,grip] to one arm via MuJoCo.</summary>
        public unsafe void ApplyEefDelta(int robotIndex, float[] action7)
        {
            if (action7 == null || action7.Length < 7) return;
            Vector3 dPos = new Vector3(action7[0], action7[1], action7[2]);

            float[][] J = ComputeJacobian3x5(robotIndex);
            float[] dq = DampLeastSquares3x5(J, dPos, 0.05f);

            // Clamp and apply joint deltas
            var arm = robotIndex == 0 ? _rArm : _lArm;
            string[] order = { "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll" };
            var data = MjScene.Instance.Data;
            var model = MjScene.Instance.Model;

            bool debugMe = _applyCount < 2;
            if (debugMe)
            {
                Debug.Log($"[IK DEBUG #{_applyCount}] dPos=({dPos.x:F6},{dPos.y:F6},{dPos.z:F6})");
                for (int ii = 0; ii < 3; ii++)
                    Debug.Log($"  J row {ii}: [{J[ii][0]:F6}, {J[ii][1]:F6}, {J[ii][2]:F6}, {J[ii][3]:F6}, {J[ii][4]:F6}]");
                for (int ii = 0; ii < 5; ii++)
                    Debug.Log($"  dq[{ii}] = {dq[ii]:F6}");
            }
            {
                foreach (var kv in arm)
                {
                    var act2 = kv.Value;
                    int jid1 = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_JOINT, act2.MujocoName);
                    int jid2 = -1;
                    if (act2.Joint != null)
                        jid2 = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_JOINT, act2.Joint.name);
                    Debug.Log($"[FindJointId] key='{kv.Key}' mujocoName='{act2.MujocoName}' joint.name='{act2.Joint?.name}' jid1={jid1} jid2={jid2}");
                }
            }

            for (int i = 0; i < 5; i++)
            {
                dq[i] = Mathf.Clamp(dq[i], -MaxJointDelta, MaxJointDelta);
                if (arm.TryGetValue(order[i], out var act))
                {
                    // Use actuator_length as current joint position (works for position actuators)
                    float cur = (float)data->actuator_length[act.MujocoId];
                    if (debugMe) Debug.Log($"  {order[i]}: cur={cur:F4} dq={dq[i]:F6} newCtrl={cur+dq[i]:F4} mjId={act.MujocoId}");
                    data->ctrl[act.MujocoId] = cur + dq[i];
                    act.Control = cur + dq[i];
                }
            }

            // Gripper: action 0=open→1=closed. Jaw range [-0.174,1.75].
            // Positive jaw = open, negative/zero = closed. So invert the mapping.
            if (arm.TryGetValue("Jaw", out var jaw) && jaw.Joint != null)
            {
                float jawTarget = Mathf.Lerp(1.75f, -0.174f, Mathf.Clamp01(action7[6]));
                data->ctrl[jaw.MujocoId] = jawTarget;
                jaw.Control = jawTarget;
            }
            _applyCount++;
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
