"""
LIBERO Unity Joy-Con Sender (Joint IK Mode)
Reads Nintendo Joy-Con controllers, computes joint angles via lerobot_kinematics IK,
and sends them to Unity via TCP.

Reference: lerobot_joycon_gpos_real.py (lerobot-kinematics/examples/)
Unity uses SetJointPositions() to directly drive ArticulationBody drives.

Joint mapping (6 values per robot):
    [base_yaw, J2, J3, J4, J5, gripper]
    - base_yaw (rad): from Madgwick gyroscope → Unity J0 (base rotation)
    - J2..J5 (rad): 4 arm joints from lerobot_IK
    - gripper: JoyCon trigger state → Unity SetGripper()

Prerequisites:
    pip install joyconrobotics lerobot-kinematics

Start Unity Play mode first (it starts the TCP server),
then run this script: python joycon_sender.py
"""

import json
import socket
import time
import sys
import traceback
import math
import numpy as np

try:
    from joyconrobotics import JoyconRobotics
except ImportError:
    print("ERROR: joyconrobotics not installed.")
    print("  pip install joyconrobotics")
    print("  Or clone from: https://github.com/box2ai-robotics/joycon-robotics")
    sys.exit(1)

try:
    from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot
except ImportError:
    print("ERROR: lerobot_kinematics not installed.")
    print("  pip install lerobot-kinematics")
    sys.exit(1)

TCP_HOST = "127.0.0.1"
TCP_PORT = 5555

JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]

# EEF workspace limits in Joy-Con frame (metres / radians)
# Matching lerobot_joycon_gpos_real.py L27-28 and joycon_bridge.py L40-43
CONTROL_GLIMIT = [
    [0.125, -0.4,  0.046, -3.1, -1.5, -1.5],
    [0.380,  0.4,  0.23,   3.1,  1.5,  1.5],
]

# SO100 FK-derived home XYZ for offset_position_m
# FK for q=[-3.14, 3.14, 0.0, -1.57] on the 4-DOF arm (J2..J5).
# Matching lerobot_joycon_gpos_real.py L30, L33.
_SO100_HOME_XYZ = [0.111, 0.0, 0.098]

# Initial 4-DOF arm joint angles (radians): [J2, J3, J4, J5]
# Matching lerobot_joycon_gpos_real.py L30: init_qpos[1:5]
_INIT_ARM_Q = np.array([-3.14, 3.14, 0.0, -1.57])


class JoyConSender:
    """Wraps two JoyconRobotics instances, runs lerobot_IK, sends joint angles to Unity."""

    def __init__(self):
        self.conn = None
        self.robot = get_robot("so100")

        # Per-arm joint state for IK seeding (4 DOF: J2..J5)
        self.current_arm_q_l = _INIT_ARM_Q.copy()
        self.current_arm_q_r = _INIT_ARM_Q.copy()

    # ── Initialization ───────────────────────────────────────────────

    def init_controllers(self):
        """Connect to both Joy-Con controllers."""
        offset = list(_SO100_HOME_XYZ)

        print("Initializing left Joy-Con ...")
        try:
            self.jc_left = JoyconRobotics(
                device="left",
                horizontal_stick_mode="yaw_diff",
                close_y=True,
                limit_dof=True,
                glimit=CONTROL_GLIMIT,
                offset_position_m=offset,
                common_rad=False,
                lerobot=True,
                pitch_down_double=True,
            )
        except RuntimeError as e:
            print(f"  WARNING: {e}")
            print("  Left Joy-Con not available, continuing with right only.")
            self.jc_left = None
        else:
            print("  Left Joy-Con ready.")

        print("Initializing right Joy-Con ...")
        try:
            self.jc_right = JoyconRobotics(
                device="right",
                horizontal_stick_mode="yaw_diff",
                close_y=True,
                limit_dof=True,
                glimit=CONTROL_GLIMIT,
                offset_position_m=offset,
                common_rad=False,
                lerobot=True,
                pitch_down_double=True,
            )
        except RuntimeError as e:
            print(f"  ERROR: {e}")
            print("  Please connect the right Joy-Con via Bluetooth and try again.")
            sys.exit(1)
        print("  Right Joy-Con ready.")

    def connect_to_unity(self):
        """Connect to Unity's TCP server."""
        print(f"\nConnecting to Unity on {TCP_HOST}:{TCP_PORT} ...")
        sys.stdout.flush()

        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        while True:
            try:
                self.conn.connect((TCP_HOST, TCP_PORT))
                break
            except (ConnectionRefusedError, OSError):
                print("  Waiting for Unity server ... (start Unity Play mode)")
                time.sleep(1.0)

        self.conn.setblocking(False)  # Non-blocking for feedback read
        print("Connected to Unity.")

    # ── IK computation ───────────────────────────────────────────────

    def compute_joints(self, jc, current_arm_q):
        """
        Get Joy-Con control, run lerobot_IK → joint angles.

        Matching lerobot_joycon_gpos_real.py control-to-joints pipeline:
          1. get_control() → [x,y,z,roll,pitch,yaw], gripper, button
          2. Clamp to CONTROL_GLIMIT workspace limits  (L74-75)
          3. pitch = -pitch, roll = roll - pi/2          (L79-80)
          4. y = 0.01 (fixed lateral)                    (L80)
          5. lerobot_IK(current_q, [x,y,z,roll,pitch,0]) → 4 arm joints (L87)
          6. target_qpos = [base_yaw] + 4 arm joints + [gripper]  (L90)

        Returns:
            (joints_array, success_flag)
            joints_array: [base_yaw, J2, J3, J4, J5, gripper] all in radians
        """
        target_pose, gripper_state, btn = jc.get_control()

        # Clamp to workspace limits (matching reference L74-75)
        for i in range(6):
            low, high = CONTROL_GLIMIT[0][i], CONTROL_GLIMIT[1][i]
            if target_pose[i] < low:
                target_pose[i] = low
            elif target_pose[i] > high:
                target_pose[i] = high

        # Extract pose components
        x_r = target_pose[0]
        z_r = target_pose[2]
        _, _, _, roll_r, pitch_r, yaw_r = target_pose
        y_r = 0.01  # Fixed lateral offset (matching reference L80)

        # Apply SO100-specific coordinate transformations (matching reference L79-82)
        pitch_r = -pitch_r
        roll_r = roll_r - math.pi / 2  # Lerobot end-effector rotated 90°

        # Build 6-DOF target: [x, y, z, roll, pitch, yaw]
        right_target_gpos = np.array([x_r, y_r, z_r, roll_r, pitch_r, 0.0])

        # Run Levenberg-Marquardt IK (matching reference L87)
        qpos_inv, ik_success = lerobot_IK(current_arm_q, right_target_gpos, robot=self.robot)

        if ik_success:
            # Combine: [base_yaw] + 4 arm joints + [gripper] (matching reference L90)
            target_qpos = np.concatenate(([yaw_r], qpos_inv[:4], [gripper_state]))
            return target_qpos, True
        else:
            # IK failed — return None so the consumer can reuse last good pose
            return None, False

    # ── Main loop ────────────────────────────────────────────────────

    def run(self):
        """Main loop: Joy-Con → IK → joint angles → Unity."""
        print("\nSender running (Joint IK mode).  Press Ctrl+C to stop.\n")
        print("CONTROLS:")
        print("  Stick up/down    – move end-effector forward / backward")
        print("  Stick left/right – rotate base yaw")
        print("  Joy-Con tilt     – change end-effector orientation")
        print("  L / R trigger    – move end-effector up")
        print("  Stick press down – move end-effector down")
        print("  ZL / ZR          – toggle gripper open / close")
        print()

        diag_count = 0

        while True:
            try:
                msg = {}

                # ── Left Joy-Con → joints → robot_1 ──
                if self.jc_left is not None:
                    joints_l, ok_l = self.compute_joints(self.jc_left, self.current_arm_q_l)
                    if ok_l:
                        self.current_arm_q_l = joints_l[1:5].copy()
                        msg["robot_1"] = {
                            "joints": [float(v) for v in joints_l],
                            "button": 0,
                        }

                # ── Right Joy-Con → joints → robot_0 ──
                joints_r, ok_r = self.compute_joints(self.jc_right, self.current_arm_q_r)
                if ok_r:
                    self.current_arm_q_r = joints_r[1:5].copy()
                    msg["robot_0"] = {
                        "joints": [float(v) for v in joints_r],
                        "button": 0,
                    }

                # ── Diagnostics ──
                diag_count += 1
                if diag_count % 60 == 0:
                    if ok_l: print(f"[J] L joints={[f'{math.degrees(v):+6.1f}°' for v in joints_l]} | gripper={joints_l[-1]:.2f}")
                    if ok_r: print(f"[J] R joints={[f'{math.degrees(v):+6.1f}°' for v in joints_r]} | gripper={joints_r[-1]:.2f}")

                if msg:
                    data = (json.dumps(msg) + "\n").encode("utf-8")
                    self.conn.sendall(data)

                    # ── Read joint feedback from Unity (closed-loop IK seed) ──
                    self._try_read_feedback()

                time.sleep(0.016)  # ~60 Hz

            except BrokenPipeError:
                print("\nUnity disconnected.")
                break
            except KeyboardInterrupt:
                print("\nShutting down.")
                break
            except Exception:
                traceback.print_exc()
                time.sleep(1.0)

    def _try_read_feedback(self):
        """
        Non-blocking read of joint-feedback JSON from Unity.
        Updates current_arm_q_l / current_arm_q_r when available.
        Expected format: {"fb": [[j0..j4], [j0..j4]]}  (radians)
        """
        try:
            buf = b""
            while True:
                try:
                    chunk = self.conn.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                except BlockingIOError:
                    break

            if not buf:
                return

            # Parse complete JSON lines
            for line in buf.decode("utf-8").strip().split("\n"):
                line = line.strip()
                if not line:
                    continue
                try:
                    data = json.loads(line)
                    fb = data.get("fb")
                    if fb and len(fb) >= 2:
                        self.current_arm_q_l = np.array(fb[0][1:5], dtype=np.float64)
                        self.current_arm_q_r = np.array(fb[1][1:5], dtype=np.float64)
                    elif fb and len(fb) >= 1:
                        self.current_arm_q_r = np.array(fb[0][1:5], dtype=np.float64)
                except (json.JSONDecodeError, ValueError):
                    pass
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def close(self):
        if self.conn:
            self.conn.close()


def main():
    sender = JoyConSender()
    try:
        sender.init_controllers()
        sender.connect_to_unity()
        sender.run()
    except KeyboardInterrupt:
        pass
    finally:
        sender.close()
        print("Sender stopped.")


if __name__ == "__main__":
    main()