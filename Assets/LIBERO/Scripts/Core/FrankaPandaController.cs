using UnityEngine;

namespace LIBERO.Core
{
    /// <summary>
    /// Panda-specific controller (7-DOF arm + dual-finger gripper).
    /// Inherits shared IK servo from RobotArmController.
    /// </summary>
    public class FrankaPandaController : RobotArmController
    {
        protected override float[] DefaultHomePoseDegrees =>
            new float[] { 0, -45f, 0, -135f, 0, 90f, 45f };

        public override void ApplyJoyConPose(Vector3 targetPos, Quaternion targetRot)
        {
            // Panda uses moderate gains — higher stiffness arm tolerates faster tracking
            ServoTowardPose(targetPos, targetRot, 0.5f, 0.3f);
        }
    }
}