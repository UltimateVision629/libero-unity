using UnityEngine;

namespace LIBERO.Core
{
    public class FrankaPandaController : MonoBehaviour
    {
        [Header("Joint References")]
        public ArticulationBody[] Joints; // 7 joints for Franka Panda

        [Header("EEF Reference")]
        public Transform EEFTransform;

        [Header("Gripper")]
        public ArticulationBody LeftFinger;
        public ArticulationBody RightFinger;

        [Header("Control")]
        public float PositionScale = 0.05f;
        public float GripperScale = 0.1f;

        [Header("IK Control")]
        public bool UseIK = true;
        public float DampingLambda = 0.01f;
        public float IKPosScale = 0.02f;
        public float IKRotScale = 0.3f;

        private bool _initialized;
        private ArticulationBody _rootAB;

        private void Start()
        {
            InitializeJoints();
        }

        public void InitializeJoints()
        {
            if (Joints == null || Joints.Length != 7)
            {
                Debug.LogWarning($"Expected 7 joints, found {Joints?.Length ?? 0}. Attempting auto-discovery.");
                Joints = GetComponentsInChildren<ArticulationBody>();
                if (Joints.Length > 7)
                {
                    var filtered = new ArticulationBody[7];
                    for (int i = 0; i < 7; i++) filtered[i] = Joints[i];
                    Joints = filtered;
                }
            }

            if (Joints != null && Joints.Length > 0)
                _rootAB = GetComponent<ArticulationBody>();

            _initialized = true;
            Debug.Log($"FrankaPandaController initialized, rootAB={_rootAB != null}");
        }

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
                for (int i = 0; i < Mathf.Min(currentJoints.Length, action.Length - 1); i++)
                    currentJoints[i] += action[i] * PositionScale;
                SetJointPositions(currentJoints);
            }

            if (action.Length >= 7)
                SetGripper(action[6]);
        }

        public void GetJointPositions(out float[] positions)
        {
            positions = new float[7];
            if (Joints != null)
            {
                for (int i = 0; i < Mathf.Min(Joints.Length, 7); i++)
                    positions[i] = Joints[i].jointPosition[0];
            }
        }

        public void SetJointPositions(float[] positions)
        {
            if (Joints == null) return;
            for (int i = 0; i < Mathf.Min(Joints.Length, positions.Length); i++)
            {
                var drive = Joints[i].xDrive;
                drive.target = positions[i] * Mathf.Rad2Deg;
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
            qpos = new float[2];
            if (LeftFinger != null) qpos[0] = LeftFinger.jointPosition[0];
            if (RightFinger != null) qpos[1] = RightFinger.jointPosition[0];
        }

        public void SetGripper(float target)
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

        public void ResetToHomePose()
        {
            SetJointPositions(new float[] { 
                0, -0.785f, 0, -2.356f, 0, 1.571f, 0.785f 
            });
            SetGripper(0.04f);
            Debug.Log("[FPC] Home pose set (radians)");
        }

        public Vector3 GetRobotStateVector()
        {
            GetJointPositions(out float[] joints);
            GetEEFPose(out Vector3 eefPos, out Quaternion eefQuat);
            GetGripperState(out float[] gripper);

            return new Vector3(eefPos.x, eefPos.y, eefPos.z); // simplified
        }
    }
}
