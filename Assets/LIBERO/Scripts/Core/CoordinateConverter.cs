using UnityEngine;

namespace LIBERO.Core
{
    public static class CoordinateConverter
    {
        /// <summary>
        /// Convert MuJoCo/robosuite coordinates to Unity coordinates.
        /// MuJoCo: X=forward, Y=left, Z=up
        /// Unity:  X=right,  Y=up,  Z=forward
        /// </summary>
        public static Vector3 MuJoCoToUnity(Vector3 mjPos)
        {
            return new Vector3(-mjPos.y, mjPos.z, mjPos.x);
        }

        public static Vector3 MuJoCoToUnity(float x, float y, float z)
        {
            return new Vector3(-y, z, x);
        }

        /// <summary>
        /// Convert a 2D MuJoCo position (on table plane: X-forward, Y-left) to Unity 3D.
        /// The MuJoCo X becomes Unity Z (forward), MuJoCo Y flips sign to become Unity X (right).
        /// </summary>
        public static Vector3 TablePosToUnity(float mjX, float mjY, float heightOffset = 0f)
        {
            return new Vector3(-mjY, heightOffset, mjX);
        }

        /// <summary>
        /// Convert a MuJoCo yaw rotation (around Z-up axis) to Unity rotation (around Y-up axis).
        /// In MuJoCo, positive yaw is counterclockwise from above (Z-up).
        /// In Unity, positive yaw is clockwise around Y-up.
        /// So: Unity rotation Y = -MuJoCo yaw
        /// </summary>
        public static Quaternion MuJoCoYawToUnity(float mjYawRad)
        {
            return Quaternion.Euler(0, -mjYawRad * Mathf.Rad2Deg, 0);
        }
    }
}
