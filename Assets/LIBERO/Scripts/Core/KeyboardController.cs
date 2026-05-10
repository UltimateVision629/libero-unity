using UnityEngine;

namespace LIBERO.Core
{
    public class KeyboardController : MonoBehaviour
    {
        public FrankaPandaController Robot;
        public float MoveSpeed = 0.5f;   // m/s, EEF speed
        public float RotSpeed = 2.0f;    // rad/s, EEF angular speed

        private float _gripper = 1f;

        void Update()
        {
            if (Robot == null) return;

            float[] action = new float[7];

            // IKPosScale=0.02 divides by 50; compensate so MoveSpeed = true m/s
            float posStep = MoveSpeed * Time.deltaTime / 0.02f;
            float rotStep = RotSpeed * Time.deltaTime / 0.3f;

            if (Input.GetKey(KeyCode.W)) action[2] =  posStep;
            if (Input.GetKey(KeyCode.S)) action[2] = -posStep;
            if (Input.GetKey(KeyCode.A)) action[0] = -posStep;
            if (Input.GetKey(KeyCode.D)) action[0] =  posStep;
            if (Input.GetKey(KeyCode.R)) action[1] =  posStep;
            if (Input.GetKey(KeyCode.F)) action[1] = -posStep;

            if (Input.GetKey(KeyCode.Z)) action[3] = -rotStep;
            if (Input.GetKey(KeyCode.X)) action[3] =  rotStep;
            if (Input.GetKey(KeyCode.T)) action[4] = -rotStep;
            if (Input.GetKey(KeyCode.G)) action[4] =  rotStep;
            if (Input.GetKey(KeyCode.C)) action[5] = -rotStep;
            if (Input.GetKey(KeyCode.V)) action[5] =  rotStep;

            if (Input.GetKeyDown(KeyCode.Space))
                _gripper = _gripper > 0.5f ? 0f : 1f;
            action[6] = _gripper;

            if (Input.GetKeyDown(KeyCode.Q))
                Robot.ResetToHomePose();

            bool hasInput = false;
            for (int i = 0; i < 6; i++) { if (action[i] != 0) hasInput = true; }

            if (hasInput || Input.GetKey(KeyCode.Space))
                Robot.ApplyAction(action);
        }
    }
}
