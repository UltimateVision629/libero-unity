using UnityEngine;

namespace LIBERO.Core
{
    /// <summary>
    /// Panda-specific controller (7-DOF arm + dual-finger gripper).
    /// Provides Panda joint limits for LM IK and JoyCon teleop.
    /// </summary>
    public class FrankaPandaController : RobotArmController
    {
        protected override float[] DefaultHomePoseDegrees =>
            new float[] { 0, -45f, 0, -135f, 0, 90f, 45f };

        protected override void Start()
        {
            base.Start();

            // Set Franka Panda joint limits (radians) if not set in inspector
            // Standard Panda limits from Franka control interface
            if (JointLimitsLower == null || JointLimitsLower.Length < ArmJointCount)
            {
                JointLimitsLower = new float[]
                {
                    -2.8973f,    // J0: base rotation
                    -1.7628f,    // J1: shoulder
                    -2.8973f,    // J2: elbow
                    -3.0718f,    // J3: forearm
                    -2.8973f,    // J4: wrist
                    -0.0175f,    // J5: wrist-2
                    -2.8973f     // J6: wrist-3
                };
            }
            if (JointLimitsUpper == null || JointLimitsUpper.Length < ArmJointCount)
            {
                JointLimitsUpper = new float[]
                {
                    2.8973f,     // J0: base rotation
                    1.7628f,     // J1: shoulder
                    2.8973f,     // J2: elbow
                    -0.0698f,    // J3: forearm (has negative upper limit!)
                    2.8973f,     // J4: wrist
                    3.7525f,     // J5: wrist-2
                    2.8973f      // J6: wrist-3
                };
            }

            Debug.Log($"[FrankaPandaController] Panda joint limits set: " +
                $"lower=[{string.Join(",", System.Array.ConvertAll(JointLimitsLower, v => $"{v:F2}"))}] " +
                $"upper=[{string.Join(",", System.Array.ConvertAll(JointLimitsUpper, v => $"{v:F2}"))}]");
        }

        public override void ApplyJoyConPose(Vector3 targetPos, Quaternion targetRot)
        {
            // Choose IK mode based on inspector setting
            if (IkSolverMode != IKMode.DLS)
            {
                // Levenberg-Marquardt IK
                ServoTowardPoseLM(targetPos, targetRot, 0.5f, 0.3f);
            }
            else
            {
                // Original DLS
                ServoTowardPose(targetPos, targetRot, 0.5f, 0.3f);
            }
        }
    }
}