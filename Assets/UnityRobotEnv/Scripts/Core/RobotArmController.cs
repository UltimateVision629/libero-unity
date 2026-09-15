using UnityEngine;

namespace UnityRobotEnv.Core
{
    /// <summary>
    /// Abstract base class for robot arm controllers.
    /// Shared IK servo logic, joint/gripper interface.
    /// Concrete implementations: FrankaPandaController, SO100Controller.
    /// </summary>
    public abstract class RobotArmController : MonoBehaviour
    {
        [Header("Joint References")]
        public ArticulationBody[] Joints;

        [Header("Arm Config")]
        public int ArmJointCount = 7;

        [Header("EEF Reference")]
        public Transform EEFTransform;

        [Header("Gripper (Dual-Finger / Panda-style)")]
        public ArticulationBody LeftFinger;
        public ArticulationBody RightFinger;

        [Header("Gripper (Single-Jaw / Revolute - SO100-style)")]
        public ArticulationBody GripperJoint;

        [Header("Control")]
        public float PositionScale = 0.05f;
        public float GripperScale = 90f;
        public ArticulationBody RootAB { get; set; }

        [Header("IK Control")]
        public bool UseIK = true;
        public IKMode IkSolverMode = IKMode.DLS;
        public float DampingLambda = 0.05f;
        public float IKPosScale = 0.02f;
        public float IKRotScale = 0.3f;

        [Header("IK - Levenberg-Marquardt")]
        public float IKLambda = 0.1f;
        public int IKMaxIterations = 10;
        public float IKTolerance = 0.001f;
        public int IKSearchLimit = 5;
        public bool IKUseJointLimits = false;

        [Header("Joint Limits (radians, lower/upper)")]
        public float[] JointLimitsLower;
        public float[] JointLimitsUpper;

        [Header("Home Pose")]
        public float[] HomePoseDegrees;

        /// <summary>
        /// Override to provide default home pose when HomePoseDegrees is unset.
        /// </summary>
        protected abstract float[] DefaultHomePoseDegrees { get; }

        protected bool _initialized;
        protected ArticulationBody _rootAB;
        protected int _pitchDiagCount;

        protected virtual void Start()
        {
            InitializeJoints();
        }

        public virtual void InitializeJoints()
        {
            if (Joints == null || Joints.Length == 0)
            {
                Debug.LogWarning($"[{GetType().Name}] No joints assigned. Attempting auto-discovery.");
                Joints = GetComponentsInChildren<ArticulationBody>();
            }

            if (Joints != null && Joints.Length > 0)
                _rootAB = RootAB ?? GetComponent<ArticulationBody>();

            _initialized = true;
            Debug.Log($"[{GetType().Name}] Initialized, joints={Joints?.Length ?? 0}, armJoints={ArmJointCount}, rootAB={_rootAB != null}");
        }

        public int TotalJointCount => Joints?.Length ?? 0;

        /// <summary>
        /// Apply RL action (delta position + delta rotation in EEF space).
        /// </summary>
        public virtual void ApplyAction(float[] action)
        {
            if (!_initialized || Joints == null) return;
            if (action == null || action.Length < 6) return;

            if (UseIK && _rootAB != null)
            {
                Vector3 dPos = new Vector3(action[0], action[1], action[2]) * IKPosScale;
                Vector3 dRot = new Vector3(action[3], action[4], action[5]) * IKRotScale;
                float[] dTheta = JacobianSolver.DampedLeastSquares(
                    _rootAB, Joints, 6, dPos, dRot, DampingLambda, 0.25f);

                if (dTheta != null && dTheta.Length == Joints.Length)
                {
                    for (int i = 0; i < Joints.Length; i++)
                    {
                        var drive = Joints[i].xDrive;
                        float deltaDeg = dTheta[i] * Mathf.Rad2Deg;
                        deltaDeg = Mathf.Clamp(deltaDeg, -20f, 20f);
                        float newTarget = Mathf.Clamp(drive.target + deltaDeg, drive.lowerLimit, drive.upperLimit);
                        drive.target = newTarget;
                        Joints[i].xDrive = drive;
                    }
                }
            }
            else if (!UseIK)
            {
                GetJointPositions(out float[] currentJoints);
                int limit = Mathf.Min(currentJoints.Length, action.Length, ArmJointCount);
                for (int i = 0; i < limit; i++)
                    currentJoints[i] += action[i];
                SetJointPositions(currentJoints);
            }

            if (action.Length > ArmJointCount)
                SetGripper(action[ArmJointCount]);
        }

        /// <summary>
        /// Servo towards an absolute target pose using proportional gains.
        /// </summary>
        public void ServoTowardPose(Vector3 targetPos, Quaternion targetRot, float posGain, float rotGain)
        {
            if (!_initialized || Joints == null || _rootAB == null) return;
            if (EEFTransform == null) return;

            Vector3 currentPos = EEFTransform.position;
            Quaternion currentRot = EEFTransform.rotation;

            Vector3 posError = targetPos - currentPos;
            Quaternion rotError = targetRot * Quaternion.Inverse(currentRot);
            rotError.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f) angle -= 360f;
            // Deadzone: skip IK if error is below sensor noise floor
            // (1.5 mm position, 0.2° rotation)
            if (posError.magnitude < 0.0015f && Mathf.Abs(angle) < 0.2f) return;
            Vector3 rotVec = axis * (angle * Mathf.Deg2Rad);

            Vector3 dPos = posError * posGain;
            Vector3 dRot = rotVec * rotGain;

            float[] dTheta = JacobianSolver.DampedLeastSquares(
                _rootAB, Joints, 6, dPos, dRot, DampingLambda, 0.25f);

            if (dTheta != null && dTheta.Length == Joints.Length)
            {
                for (int i = 0; i < Joints.Length; i++)
                {
                    var drive = Joints[i].xDrive;
                    float deltaDeg = dTheta[i] * Mathf.Rad2Deg;
                    deltaDeg = Mathf.Clamp(deltaDeg, -20f, 20f);
                    float newTarget = Mathf.Clamp(drive.target + deltaDeg, drive.lowerLimit, drive.upperLimit);
                    drive.target = newTarget;
                    Joints[i].xDrive = drive;
                }
            }
        }

        /// <summary>
        /// Servo towards an absolute target pose using Levenberg-Marquardt IK.
        /// Uses angle-axis error for exact 6-DOF convergence.
        /// </summary>
        public void ServoTowardPoseLM(Vector3 targetPos, Quaternion targetRot,
            float posGain, float rotGain)
        {
            if (!_initialized || Joints == null || _rootAB == null) return;
            if (EEFTransform == null) return;

            int n = ArmJointCount;
            Matrix4x4 Tep = JacobianSolver.BuildTransform(targetPos, targetRot);

            // Build FK and Jacobian callbacks using Unity's ArticulationBody
            System.Func<float[], Matrix4x4> fkine = (q) =>
            {
                // Apply q to joint drives temporarily and read EEF
                float[] saved = new float[n];
                for (int i = 0; i < n && i < Joints.Length; i++)
                    saved[i] = Joints[i].xDrive.target;
                for (int i = 0; i < n && i < Joints.Length; i++)
                {
                    var drive = Joints[i].xDrive;
                    drive.target = q[i] * Mathf.Rad2Deg;
                    Joints[i].xDrive = drive;
                }
                // Force physics step?  For simplicity, approximate FK
                // by directly reading current EEF (suboptimal but avoids
                // stepping physics mid-frame).
                // Better approach: assume current EEF is close enough
                // and use Unity's built-in Jacobian.
                Matrix4x4 Te = JacobianSolver.BuildTransform(EEFTransform.position, EEFTransform.rotation);
                // Restore
                for (int i = 0; i < n && i < Joints.Length; i++)
                {
                    var drive = Joints[i].xDrive;
                    drive.target = saved[i];
                    Joints[i].xDrive = drive;
                }
                return Te;
            };

            System.Func<float[], float[][]> jacob0 = (q) =>
            {
                // Use Unity's dense Jacobian
                return JacobianSolver.GetDenseJacobian(_rootAB, n);
            };

            System.Func<float[]> getCurrentQ = () =>
            {
                float[] qc = new float[n];
                for (int i = 0; i < n && i < Joints.Length; i++)
                    qc[i] = Joints[i].jointPosition[0]; // radians
                return qc;
            };

            System.Action<float[]> applyQ = (q) =>
            {
                for (int i = 0; i < n && i < Joints.Length; i++)
                {
                    var drive = Joints[i].xDrive;
                    float deg = q[i] * Mathf.Rad2Deg;
                    deg = Mathf.Clamp(deg, drive.lowerLimit, drive.upperLimit);
                    drive.target = deg;
                    Joints[i].xDrive = drive;
                }
            };

            float[][] qlim = null;
            if (IKUseJointLimits && JointLimitsLower != null && JointLimitsUpper != null)
            {
                qlim = new float[2][] { JointLimitsLower, JointLimitsUpper };
            }

            IKSolution sol = JacobianSolver.SolveLM(
                fkine, jacob0, getCurrentQ, applyQ, n, Tep,
                ilimit: IKMaxIterations,
                slimit: IKSearchLimit,
                tol: IKTolerance,
                lambda: IKLambda,
                mode: IkSolverMode,
                qlim: qlim,
                jointLimits: IKUseJointLimits,
                maxAngle: 0.25f);

            if (sol.success)
            {
                applyQ(sol.q);
                // Debug log every 30 frames
                _pitchDiagCount++;
                if (_pitchDiagCount % 30 == 0)
                    Debug.Log($"[LM] iters={sol.iterations} residual={sol.residual:F6}");
            }
        }

        /// <summary>
        /// Apply JoyCon-teleop target pose. Override in subclass to tune gains.
        /// (Legacy, with full orientation IK. Use ServoTowardPositionWithBase for SO100.)
        /// </summary>
        public abstract void ApplyJoyConPose(Vector3 targetPos, Quaternion targetRot);

        /// <summary>
        /// Servo toward absolute EEF pose (position + orientation) with base rotation.
        /// J0 (lockedJointIndex) is set directly to baseTargetDeg (degrees).
        /// Remaining joints are solved via full 6-DOF IK (position + orientation).
        /// </summary>
        public virtual void ServoTowardPoseWithBase(Vector3 targetPos, Quaternion targetRot, float baseTargetDeg,
            int lockedJointIndex, float posGain, float rotGain, float maxAngleRad)
        {
            if (!_initialized || Joints == null || _rootAB == null) return;
            if (EEFTransform == null) return;

            // 1. Drive locked joint (base rotation) directly
            if (lockedJointIndex >= 0 && lockedJointIndex < Joints.Length)
            {
                var baseDrive = Joints[lockedJointIndex].xDrive;
                float clamped = Mathf.Clamp(baseTargetDeg, baseDrive.lowerLimit, baseDrive.upperLimit);
                Joints[lockedJointIndex].xDrive = new ArticulationDrive
                {
                    lowerLimit = baseDrive.lowerLimit,
                    upperLimit = baseDrive.upperLimit,
                    stiffness = baseDrive.stiffness,
                    damping = baseDrive.damping,
                    forceLimit = baseDrive.forceLimit,
                    target = clamped,
                    targetVelocity = baseDrive.targetVelocity
                };
            }

            // 2. Full 6-DOF IK for remaining joints (incl. orientation)
            Vector3 currentPos = EEFTransform.position;
            Quaternion currentRot = EEFTransform.rotation;

            _pitchDiagCount++;
            if (_pitchDiagCount % 60 == 0)
            {
                float pitchDeg = currentRot.eulerAngles.x;
                if (pitchDeg > 180f) pitchDeg -= 360f;
                Debug.Log($"[CS] EEF pitch={pitchDeg:+0.0}° (euler.x)");
            }

            Vector3 posError = targetPos - currentPos;
            Quaternion rotError = targetRot * Quaternion.Inverse(currentRot);
            rotError.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f) angle -= 360f;

            if (posError.magnitude < 0.0015f && Mathf.Abs(angle) < 0.2f) return;

            Vector3 dPos = posError * posGain;
            Vector3 dRot = axis * (angle * Mathf.Deg2Rad) * rotGain;

            float[] dTheta = JacobianSolver.DampedLeastSquaresWithLockedJoint(
                _rootAB, Joints, lockedJointIndex, dPos, dRot, DampingLambda, maxAngleRad);

            if (dTheta != null && dTheta.Length == Joints.Length)
            {
                for (int i = 0; i < Joints.Length; i++)
                {
                    if (i == lockedJointIndex) continue;
                    var drive = Joints[i].xDrive;
                    float deltaDeg = dTheta[i] * Mathf.Rad2Deg;
                    deltaDeg = Mathf.Clamp(deltaDeg, -20f, 20f);
                    float newTarget = Mathf.Clamp(drive.target + deltaDeg, drive.lowerLimit, drive.upperLimit);
                    drive.target = newTarget;
                    Joints[i].xDrive = drive;
                }
            }
        }

        /// <summary>
        /// Servo toward absolute EEF position with base rotation.
        /// J0 (lockedJointIndex) is set directly to baseTargetDeg (degrees).
        /// Remaining joints are solved via position-only IK.
        /// </summary>
        public virtual void ServoTowardPositionWithBase(Vector3 targetPos, float baseTargetDeg,
            int lockedJointIndex, float posGain, float maxAngleRad)
        {
            if (!_initialized || Joints == null || _rootAB == null) return;
            if (EEFTransform == null) return;

            // 1. Drive locked joint (base rotation) directly
            if (lockedJointIndex >= 0 && lockedJointIndex < Joints.Length)
            {
                var baseDrive = Joints[lockedJointIndex].xDrive;
                float clamped = Mathf.Clamp(baseTargetDeg, baseDrive.lowerLimit, baseDrive.upperLimit);
                Joints[lockedJointIndex].xDrive = new ArticulationDrive
                {
                    lowerLimit = baseDrive.lowerLimit,
                    upperLimit = baseDrive.upperLimit,
                    stiffness = baseDrive.stiffness,
                    damping = baseDrive.damping,
                    forceLimit = baseDrive.forceLimit,
                    target = clamped,
                    targetVelocity = baseDrive.targetVelocity
                };
            }

            // 2. Position-only IK for remaining joints
            Vector3 currentPos = EEFTransform.position;
            Vector3 posError = targetPos - currentPos;

            // Deadzone: 1.5 mm
            if (posError.magnitude < 0.0015f) return;

            Vector3 dPos = posError * posGain;

            float[] dTheta = JacobianSolver.DampedLeastSquaresPositionOnly(
                _rootAB, Joints, lockedJointIndex, dPos, DampingLambda, maxAngleRad);

            if (dTheta != null && dTheta.Length == Joints.Length)
            {
                for (int i = 0; i < Joints.Length; i++)
                {
                    if (i == lockedJointIndex) continue;  // already set
                    var drive = Joints[i].xDrive;
                    float deltaDeg = dTheta[i] * Mathf.Rad2Deg;
                    deltaDeg = Mathf.Clamp(deltaDeg, -20f, 20f);
                    float newTarget = Mathf.Clamp(drive.target + deltaDeg, drive.lowerLimit, drive.upperLimit);
                    drive.target = newTarget;
                    Joints[i].xDrive = drive;
                }
            }
        }

        public void GetJointPositions(out float[] positions)
        {
            int count = ArmJointCount;
            positions = new float[count];
            if (Joints != null)
            {
                for (int i = 0; i < Mathf.Min(Joints.Length, count); i++)
                    positions[i] = Joints[i].jointPosition[0];
            }
        }

        public void SetJointPositions(float[] positions)
        {
            if (Joints == null) return;
            int limit = Mathf.Min(Joints.Length, positions.Length, ArmJointCount);


            Debug.Log($"[controller debug]: {Joints.Length} {positions.Length} {ArmJointCount}");  
            Debug.Log($"[controller limit]: {limit}");


            _pitchDiagCount++;
            if (_pitchDiagCount % 30 == 0)
            {
                var curVals = new System.Text.StringBuilder();
                var tgtVals = new System.Text.StringBuilder();
                for (int i = 0; i < limit; i++)
                {
                    curVals.Append($"{Joints[i].jointPosition[0] * Mathf.Rad2Deg:F1}");
                    tgtVals.Append($"{positions[i]:F1}");
                    if (i < limit - 1) { curVals.Append(", "); tgtVals.Append(", "); }
                }
                Debug.Log($"[SJP] limit={limit} cur=[{curVals}] tgt=[{tgtVals}]");
            }

            for (int i = 0; i < limit; i++)
            {
                var drive = Joints[i].xDrive;
                drive.target = positions[i];
                Joints[i].xDrive = drive;
            }
        }

        public void GetEEFPose(out Vector3 position, out Quaternion rotation)
        {
            if (EEFTransform != null)
            {
                position = EEFTransform.position;
                rotation = EEFTransform.rotation;
            }
            else
            {
                position = Vector3.zero;
                rotation = Quaternion.identity;
            }
        }

        public void GetGripperState(out float[] qpos)
        {
            if (GripperJoint != null)
            {
                qpos = new float[1];
                qpos[0] = GripperJoint.jointPosition[0];
            }
            else
            {
                qpos = new float[2];
                if (LeftFinger != null) qpos[0] = LeftFinger.jointPosition[0];
                if (RightFinger != null) qpos[1] = RightFinger.jointPosition[0];
            }
        }

        public virtual void SetGripper(float target)
        {
            if (GripperJoint != null)
            {
                var drive = GripperJoint.xDrive;
                float clamped = Mathf.Clamp(target * GripperScale, drive.lowerLimit, drive.upperLimit);
                drive.target = clamped;
                GripperJoint.xDrive = drive;
            }
            else
            {
                if (LeftFinger != null)
                {
                    var drive = LeftFinger.xDrive;
                    float clamped = Mathf.Clamp(target * GripperScale, drive.lowerLimit, drive.upperLimit);
                    drive.target = clamped;
                    LeftFinger.xDrive = drive;
                }
                if (RightFinger != null)
                {
                    var drive = RightFinger.xDrive;
                    float clamped = Mathf.Clamp(-target * GripperScale, drive.lowerLimit, drive.upperLimit);
                    drive.target = clamped;
                    RightFinger.xDrive = drive;
                }
            }
        }

        public virtual void ResetToHomePose()
        {
            float[] homePose;
            if (HomePoseDegrees != null && HomePoseDegrees.Length >= ArmJointCount)
            {
                homePose = HomePoseDegrees;
            }
            else
            {
                homePose = DefaultHomePoseDegrees;
            }
            SetJointPositions(homePose);
            SetGripper(0.04f);
            Debug.Log($"[{GetType().Name}] Home pose set (degrees)");
        }

        public Vector3 GetRobotStateVector()
        {
            GetJointPositions(out float[] joints);
            GetEEFPose(out Vector3 eefPos, out Quaternion eefQuat);
            GetGripperState(out float[] gripper);

            return new Vector3(eefPos.x, eefPos.y, eefPos.z);
        }
    }
}