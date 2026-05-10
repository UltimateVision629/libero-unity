using UnityEngine;

namespace LIBERO.Core
{
    public class KeyboardController : MonoBehaviour
    {
        public FrankaPandaController Robot;
        public float MoveSpeed = 0.5f;
        public float RotSpeed = 2.0f;
        public float JointSpeed = 30f;

        private float _gripper = 1f;
        private bool _jointMode;
        private int _selectedJoint;

        void Update()
        {
            if (Robot == null) return;

            if (Input.GetKeyDown(KeyCode.Tab))
            {
                _jointMode = !_jointMode;
                Robot.UseIK = !_jointMode;
                Debug.Log($"[KBD] Mode: {(_jointMode ? "Joint" : "IK")}");
            }

            if (Input.GetKeyDown(KeyCode.Space))
            {
                _gripper = _gripper > 0.5f ? 0f : 1f;
                Robot.SetGripper(_gripper);
            }

            if (Input.GetKeyDown(KeyCode.Q))
                Robot.ResetToHomePose();

            float dt = Time.deltaTime;

            if (_jointMode)
            {
                for (int i = 0; i <= 6; i++)
                {
                    if (Input.GetKeyDown((KeyCode)((int)KeyCode.Alpha1 + i)))
                    {
                        _selectedJoint = i;
                        Debug.Log($"[KBD] Selected joint {_selectedJoint + 1}");
                    }
                }

                float step = JointSpeed * dt;
                int dir = 0;
                if (Input.GetKey(KeyCode.W)) dir = 1;
                else if (Input.GetKey(KeyCode.S)) dir = -1;

                if (dir != 0)
                {
                    float[] targets = new float[7];
                    for (int i = 0; i < Mathf.Min(Robot.Joints.Length, 7); i++)
                        targets[i] = Robot.Joints[i].xDrive.target;
                    targets[_selectedJoint] += dir * step;
                    Robot.SetJointPositions(targets);
                }
            }
            else
            {
                float[] action = new float[7];

                float posStep = MoveSpeed * dt / 0.02f;
                float rotStep = RotSpeed * dt / 0.3f;

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

                action[6] = _gripper;

                bool hasInput = false;
                for (int i = 0; i < 7; i++) { if (action[i] != 0) hasInput = true; }

                if (hasInput)
                    Robot.ApplyAction(action);
            }
        }
    }
}
