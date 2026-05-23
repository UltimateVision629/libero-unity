using UnityEngine;

namespace LIBERO.Core
{
    /// <summary>
    /// SO100-specific controller (5-DOF arm + single revolute jaw gripper).
    /// Uses lower IK gains for JoyCon teleop to prevent oscillation.
    /// </summary>
    public class SO100Controller : RobotArmController
    {
        protected override float[] DefaultHomePoseDegrees =>
            new float[] { 0, 0, 0, 0, 0 };

        public override void ApplyJoyConPose(Vector3 targetPos, Quaternion targetRot)
        {
            // SO100 requires very low gains — delta is already clamped and
            // EMA-smoothed by the Python bridge. Higher gains cause vibration.
            ServoTowardPose(targetPos, targetRot, 0.25f, 0.15f);
        }
    }
}