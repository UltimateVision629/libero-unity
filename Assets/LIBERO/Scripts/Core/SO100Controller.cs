using UnityEngine;

namespace LIBERO.Core
{
    /// <summary>
    /// SO100-specific controller (5-DOF arm + single revolute jaw gripper).
    /// Provides joint limits for LM IK and JoyCon teleop with low gains.
    /// J0 = base rotation (±180°), J1-J4 = arm joints.
    /// </summary>
    public class SO100Controller : RobotArmController
    {
        protected override float[] DefaultHomePoseDegrees =>
            new float[] { 0, 0, 0, 0, 0 };

        protected override void Start()
        {
            base.Start();

            // Set SO100 joint limits (radians) if not set in inspector
            // J0: base rotation ±180°, J1: shoulder pitch ~-10°..+90°, J2-J4: ±170°
            if (JointLimitsLower == null || JointLimitsLower.Length < ArmJointCount)
            {
                JointLimitsLower = new float[]
                {
                    -Mathf.PI,     // J0: base rotation ±180°
                    -0.17f,        // J1: shoulder ~-10° (prevent self-collision)
                    -2.97f,        // J2: elbow ~-170°
                    -2.97f,        // J3: wrist pitch ~-170°
                    -2.97f         // J4: wrist roll ~-170°
                };
            }
            if (JointLimitsUpper == null || JointLimitsUpper.Length < ArmJointCount)
            {
                JointLimitsUpper = new float[]
                {
                    Mathf.PI,      // J0: base rotation +180°
                    1.57f,         // J1: shoulder +90°
                    0.17f,         // J2: elbow ~+10° (prevent flip-over)
                    2.97f,         // J3: wrist pitch +170°
                    2.97f          // J4: wrist roll +170°
                };
            }

            Debug.Log($"[SO100Controller] Joint limits set: " +
                $"lower=[{string.Join(",", System.Array.ConvertAll(JointLimitsLower, v => $"{v:F2}"))}] " +
                $"upper=[{string.Join(",", System.Array.ConvertAll(JointLimitsUpper, v => $"{v:F2}"))}]");
        }

        public override void ApplyJoyConPose(Vector3 targetPos, Quaternion targetRot)
        {
            // Choose IK mode based on inspector setting
            if (IkSolverMode != IKMode.DLS)
            {
                // Levenberg-Marquardt: uses angle-axis error internally,
                // posGain/rotGain are unused in LM mode (absolute pose target)
                ServoTowardPoseLM(targetPos, targetRot, 0.25f, 0.15f);
            }
            else
            {
                // Original DLS
                ServoTowardPose(targetPos, targetRot, 0.25f, 0.15f);
            }
        }
    }
}