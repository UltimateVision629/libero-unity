using UnityEngine;

namespace LIBERO.Core
{
    public class FrankaPandaController : MonoBehaviour
    {
        [Header("Joint References")]
        public ArticulationBody[] Joints;

        [Header("Arm Config")]
        public int ArmJointCount = 7;

        [Header("EEF Reference")]
        public Transform EEFTransform;

        [Header("Gripper (Dual-Finger)")]
        public ArticulationBody LeftFinger;
        public ArticulationBody RightFinger;

        [Header("Gripper (Single-Jaw / Revolute)")]
        public ArticulationBody GripperJoint;

        [Header("Control")]
        public float PositionScale = 0.05f;
        public float GripperScale = 0.1f;
        public ArticulationBody RootAB { get; set; }

        [Header("IK Control")]
        public bool UseIK = true;
        public float DampingLambda = 0.01f;
        public float IKPosScale = 0.02f;
        public float IKRotScale = 0.3f;

        [Header("Home Pose")]
        public float[] HomePoseDegrees;

        private bool _initialized;
        private ArticulationBody _rootAB;

        private void Start()
        {
            InitializeJoints();
        }

        public void InitializeJoints()
        {
            if (Joints == null || Joints.Length == 0)
            {
                Debug.LogWarning("No joints assigned. Attempting auto-discovery.");
                Joints = GetComponentsInChildren<ArticulationBody>();
            }

            if (Joints != null && Joints.Length > 0)
                _rootAB = RootAB ?? GetComponent<ArticulationBody>();

            _initialized = true;
            Debug.Log($"FrankaPandaController initialized, joints={Joints?.Length ?? 0}, armJoints={ArmJointCount}, rootAB={_rootAB != null}");
        }

        public int TotalJointCount => Joints?.Length ?? 0;

        public void ApplyAction(float[] action)
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
                        drive.target += dTheta[i] * Mathf.Rad2Deg;
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

        public void SetGripper(float target)
        {
            if (GripperJoint != null)
            {
                var drive = GripperJoint.xDrive;
                drive.target = target * GripperScale;
                GripperJoint.xDrive = drive;
            }
            else
            {
                if (LeftFinger != null)
                {
                    var drive = LeftFinger.xDrive;
                    drive.target = target * GripperScale;
                    LeftFinger.xDrive = drive;
                }
                if (RightFinger != null)
                {
                    var drive = RightFinger.xDrive;
                    drive.target = target * GripperScale;
                    RightFinger.xDrive = drive;
                }
            }
        }

        public void ResetToHomePose()
        {
            float[] homePose;
            if (HomePoseDegrees != null && HomePoseDegrees.Length >= ArmJointCount)
            {
                homePose = HomePoseDegrees;
            }
            else
            {
                homePose = new float[] { 0, -45f, 0, -135f, 0, 90f, 45f };
            }
            SetJointPositions(homePose);
            SetGripper(0.04f);
            Debug.Log("[FPC] Home pose set (degrees)");
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
