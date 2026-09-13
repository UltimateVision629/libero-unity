using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
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
        private static readonly string[] _armJointTypes = { "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll" };
        private Dictionary<string, int> _jointPrefixToId = new();
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

            bool exists = MjScene.InstanceExists;
            Debug.Log($"[MjJoyCon] Awake: InstanceExists={exists}");
            if (exists)
            {
                FindActuators();
                Debug.Log($"[MjJoyCon] Awake: jointMap.Count={_jointPrefixToId.Count}");
                if (_jointPrefixToId.Count == 0)
                {
                    Debug.Log("[MjJoyCon] Awake: jointMap empty, subscribing postInitEvent");
                    MjScene.Instance.postInitEvent += OnSceneReady;
                }
                else
                {
                    MjScene.Instance.preUpdateEvent += OnPreUpdate;
                }
            }
            else
            {
                MjScene.Instance.postInitEvent += OnSceneReady;
            }
        }

        private void OnSceneReady(object sender, MjStepArgs e)
        {
            Debug.Log("[MjJoyCon] OnSceneReady called");
            FindActuators();
            Debug.Log($"[MjJoyCon] OnSceneReady: jointMap.Count={_jointPrefixToId.Count}");
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
            BuildJointMap();
        }

        private unsafe void BuildJointMap()
        {
            _jointPrefixToId.Clear();
            var model = MjScene.Instance.Model;
            if ((IntPtr)model == IntPtr.Zero || model->names == null || model->njnt <= 0)
            {
                Debug.LogWarning("[MjJoyCon] BuildJointMap skipped: model not ready");
                return;
            }
            byte* names = (byte*)model->names;

            for (int j = 0; j < model->njnt; j++)
            {
                int adr = model->name_jntadr[j];
                string jname = "";
                for (int k = 0; k < 80; k++)
                {
                    char c = (char)names[adr + k];
                    if (c == '\0') break;
                    jname += c;
                }

                bool matched = false;
                foreach (var side in new[] { "R_", "L_" })
                {
                    if (matched) break;
                    foreach (var joint in _jointTypes)
                    {
                        string key = side + joint;
                        if (jname.StartsWith(key))
                        {
                            _jointPrefixToId[key] = j;
                            matched = true;
                            break;
                        }
                    }
                }
            }
            Debug.Log($"[MjJoyCon] Joint map built: {_jointPrefixToId.Count} entries");
        }

        private void MapActuator(Dictionary<string, MjActuator> dict, string name, MjActuator act)
        {
            foreach (var jt in _jointTypes)
                if (name.Contains(jt)) { dict[jt] = act; return; }
        }

        private unsafe void OnPreUpdate(object sender, MjStepArgs e)
        {
            // Warmup: hold arms at home pose so they don't droop at qpos=0
            // （首帧直接瞬移 home，避免 qpos=0 直伸趴在桌面上的过渡状态）
            if (_warmup < WarmupFrames)
            {
                if (_warmup == 0)
                {
                    HomeArms();
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

        private int FindJointId(MjActuator act)
        {
            // Use prefix-based lookup (mj_name2id broken due to MuJoCo name suffixes)
            foreach (var kv in _jointPrefixToId)
            {
                if (act.MujocoName != null && act.MujocoName.StartsWith(kv.Key))
                    return kv.Value;
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

        /// <summary>Home 关节角（与 ResetArm 的 ctrl 目标一致，SetQpos 顺序见 _armJointTypes）。</summary>
        private static readonly float[] HOME_JOINTS = { 0f, -3.14f, 3.14f, 0f, -1.57f };

        /// <summary>
        /// 双臂瞬移到 home（抓取准备）姿态：SetQpos 直写关节角 + ctrl 归位。
        /// mj_resetData 会把所有关节 qpos 清零——qpos=0 时 SO100 链水平直伸
        /// （部分链节 z≈0 趴在桌面上），只写 ctrl 的话要几十帧才摆到 home，
        /// 期间臂会横扫桌面（可能碰动方块）。直写 qpos 后不存在过渡状态。
        /// </summary>
        private void HomeArms()
        {
            SetQpos(0, HOME_JOINTS);
            SetQpos(1, HOME_JOINTS);
            ResetArm(_rArm);
            ResetArm(_lArm);
        }

        /// <summary>
        /// Scene was reset (mj_resetData): retarget both arms to the home
        /// keyframe immediately and drop the cached Joy-Con pose.  Without
        /// this, MjJoyConController keeps writing the previous episode's last
        /// joints into ctrl and the arms return to the old position after
        /// every reset (the stale-pose window between reset and the next
        /// joint message).
        /// </summary>
        public void OnSceneReset()
        {
            HomeArms();
            if (JoyConInput != null)
                JoyConInput.ClearPose();
            _prevEefValid = false;
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

                // Read EEF directly from MuJoCo site_xpos/site_xmat (GameObject transform may lag)
                eefPos = GetEefPos(robotIndex);
                eefQuat = GetEefQuat(robotIndex);

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

        private int _siteIdR = -1, _siteIdL = -1;
        private unsafe int FindEefSite(string prefix)
        {
            var model = MjScene.Instance.Model;
            byte* names = (byte*)model->names;
            for (int s = 0; s < model->nsite; s++)
            {
                int adr = model->name_siteadr[s];
                string sname = "";
                for (int k = 0; k < 80; k++)
                {
                    char c = (char)names[adr + k];
                    if (c == '\0') break;
                    sname += c;
                }
                if (sname.StartsWith(prefix + "eef_site"))
                    return s;
            }
            return -1;
        }

        private unsafe Vector3 GetEefPos(int robotIndex)
        {
            string prefix = robotIndex == 0 ? "R_" : "L_";
            ref int sidRef = ref (robotIndex == 0 ? ref _siteIdR : ref _siteIdL);
            if (sidRef < 0)
                sidRef = FindEefSite(prefix);
            if (sidRef < 0) return Vector3.zero;
            double* sp = MjScene.Instance.Data->site_xpos;
            return new Vector3((float)sp[sidRef * 3], (float)sp[sidRef * 3 + 1], (float)sp[sidRef * 3 + 2]);
        }

        private unsafe Quaternion GetEefQuat(int robotIndex)
        {
            string prefix = robotIndex == 0 ? "R_" : "L_";
            ref int sidRef = ref (robotIndex == 0 ? ref _siteIdR : ref _siteIdL);
            if (sidRef < 0)
                sidRef = FindEefSite(prefix);
            if (sidRef < 0) return Quaternion.identity;

            // site_xmat: row-major 3x3 rotation matrix (MuJoCo frame).
            // (The plugin binding exposes site_xmat but NOT site_xquat, so
            // convert the matrix to a quaternion here.)
            double* xm = MjScene.Instance.Data->site_xmat;
            int i = sidRef;
            double m00 = xm[i * 9 + 0], m01 = xm[i * 9 + 1], m02 = xm[i * 9 + 2];
            double m10 = xm[i * 9 + 3], m11 = xm[i * 9 + 4], m12 = xm[i * 9 + 5];
            double m20 = xm[i * 9 + 6], m21 = xm[i * 9 + 7], m22 = xm[i * 9 + 8];

            // Matrix → quaternion (w,x,y,z), standard trace-branch method
            double w, x, y, z;
            double tr = m00 + m11 + m22;
            if (tr > 0.0)
            {
                double s = System.Math.Sqrt(tr + 1.0) * 2.0;
                w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                double s = System.Math.Sqrt(1.0 + m00 - m11 - m22) * 2.0;
                w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s;
            }
            else if (m11 > m22)
            {
                double s = System.Math.Sqrt(1.0 + m11 - m00 - m22) * 2.0;
                w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s;
            }
            else
            {
                double s = System.Math.Sqrt(1.0 + m22 - m00 - m11) * 2.0;
                w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s;
            }

            // MuJoCo frame (x,y,z,w) → Unity frame, project convention
            // (AGENTS.md: RH→LH `new Quaternion(y, -z, -x, w)`).  If the
            // observed euler signs are inverted vs the command side, the
            // alternative is `new Quaternion(-y, z, x, w)`.
            return new Quaternion((float)y, (float)(-z), (float)(-x), (float)w);
        }

        private unsafe float[] GetCurrentQ(int robotIndex)
        {
            var arm = robotIndex == 0 ? _rArm : _lArm;
            float[] q = new float[5];
            string prefix = robotIndex == 0 ? "R_" : "L_";
            var model = MjScene.Instance.Model;
            var data = MjScene.Instance.Data;
            for (int i = 0; i < 5; i++)
            {
                string key = prefix + _armJointTypes[i];
                if (_jointPrefixToId.TryGetValue(key, out int jid))
                    q[i] = (float)data->qpos[model->jnt_qposadr[jid]];
            }
            return q;
        }

        private unsafe void SetQpos(int robotIndex, float[] q)
        {
            string prefix = robotIndex == 0 ? "R_" : "L_";
            var model = MjScene.Instance.Model;
            var data = MjScene.Instance.Data;
            for (int i = 0; i < 5; i++)
            {
                string key = prefix + _armJointTypes[i];
                if (_jointPrefixToId.TryGetValue(key, out int jid))
                    data->qpos[model->jnt_qposadr[jid]] = q[i];
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

        /// <summary>Apply EEF delta [dx,dy,dz,dRx,dRy,dRz,grip] to one arm via MuJoCo.</summary>
        public unsafe void ApplyEefDelta(int robotIndex, float[] action7)
        {
            if (action7 == null || action7.Length < 7) return;
            Vector3 dPos = new Vector3(action7[0], action7[1], action7[2]);

            float[][] J = ComputeJacobian3x5(robotIndex);
            float[] dq = DampLeastSquares3x5(J, dPos, 0.05f);

            var arm = robotIndex == 0 ? _rArm : _lArm;
            string[] order = { "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll" };
            var data = MjScene.Instance.Data;

            for (int i = 0; i < 5; i++)
            {
                dq[i] = Mathf.Clamp(dq[i], -MaxJointDelta, MaxJointDelta);
                if (arm.TryGetValue(order[i], out var act))
                {
                    float cur = (float)data->actuator_length[act.MujocoId];
                    data->ctrl[act.MujocoId] = cur + dq[i];
                    act.Control = cur + dq[i];
                }
            }

            // Gripper: action 0=open→1=closed. Jaw range [-0.174,1.75].
            if (arm.TryGetValue("Jaw", out var jaw) && jaw.Joint != null)
            {
                float jawTarget = Mathf.Lerp(1.75f, -0.174f, Mathf.Clamp01(action7[6]));
                data->ctrl[jaw.MujocoId] = jawTarget;
                jaw.Control = jawTarget;
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
