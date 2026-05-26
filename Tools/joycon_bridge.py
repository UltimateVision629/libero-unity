"""
LIBERO Unity Joy-Con Bridge
Reads Nintendo Joy-Con controllers (left + right) and sends teleop data to Unity via TCP.

Left Joy-Con  → Robot  (SO100_L, table left,  X=-0.2)
Right Joy-Con → Robot2 (SO100_R, table right, X=+0.2)

Sends EEF position [x,y,z], orientation [roll,pitch,yaw] (rad), and base_yaw (rad) to Unity.
Unity handles all IK via JacobianSolver.

Prerequisites:
    pip install joyconrobotics

Start Unity Play mode first (it starts the TCP server),
then run this script: python joycon_bridge.py
"""

import json
import socket
import time
import sys
import traceback
import math
import numpy as np
from collections import deque
from typing import Optional

try:
    from joyconrobotics import JoyconRobotics
except ImportError:
    print("ERROR: joyconrobotics not installed.")
    print("  pip install joyconrobotics")
    print("  Or clone from: https://github.com/box2ai-robotics/joycon-robotics")
    sys.exit(1)

TCP_HOST = "127.0.0.1"
TCP_PORT = 5555

# EEF workspace limits in Joy-Con frame (metres / radians)
CONTROL_GLIMIT = [
    [0.125, -0.4, 0.046, -3.1, -1.5, -1.5],
    [0.380,  0.4,  0.23,  3.1,  1.5,  1.5],
]

# ── SO100 FK-derived home XYZ (copied from lerobot_LK for offset_position_m) ──
# FK for q=[0, -3.14, 3.14, 0, -1.57] on the 4-DOF arm (J2..J5).
# These are standard values that place the EEF in a neutral mid-reach position.
_SO100_HOME_XYZ = [0.244, 0.0, -0.209]


class JoyConBridge:
    """Wraps two JoyconRobotics instances and sends EEF poses to Unity."""

    def __init__(self):
        self.conn: Optional[socket.socket] = None

        # ── Manual X integrator (bypasses direction_vector dependency) ──
        self._x_manual_l: float = _SO100_HOME_XYZ[0]
        self._x_manual_r: float = _SO100_HOME_XYZ[0]

        # ── Base yaw (from Madgwick, transmitted as-is) ────────────────
        self._base_yaw_l: float = 0.0
        self._base_yaw_r: float = 0.0

        # ── Diagnostics ─────────────────────────────────────────────────
        self._diag_count: int = 0

    def init_controllers(self):
        """Connect to both Joy-Con controllers."""
        offset = list(_SO100_HOME_XYZ)  # home XYZ

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
            print(f"  ERROR: {e}")
            print("  Please connect the left Joy-Con via Bluetooth and try again.")
            sys.exit(1)
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

        print(f"Connected to Unity.")

    # Joy-Con stick raw range: 0..4095, center ≈ 2048, min ≈ 768, max ≈ 3328
    _STICK_CENTER: float = 2048.0
    _STICK_HALF_RANGE: float = 1280.0  # ≈ (3328-2048) for full deflection
    _STICK_DEADZONE: float = 0.08      # [-1,+1] equivalent: ~100 raw units

    def _normalize_stick(self, raw: float) -> float:
        """Convert raw Joy-Con stick value (0..4095) to normalised [-1,+1] with deadzone."""
        norm = (raw - self._STICK_CENTER) / self._STICK_HALF_RANGE
        norm = max(-1.0, min(1.0, norm))
        if abs(norm) < self._STICK_DEADZONE:
            return 0.0
        # Rescale so the output starts from 0 at the deadzone edge
        sign = 1.0 if norm > 0 else -1.0
        return sign * (abs(norm) - self._STICK_DEADZONE) / (1.0 - self._STICK_DEADZONE)

    def _get_stick_vertical(self, jc) -> float:
        """Return vertical stick deflection (-1..+1, up-positive)."""
        joycon = jc.joycon
        raw = joycon.get_stick_right_vertical() if joycon.is_right() else joycon.get_stick_left_vertical()
        return self._normalize_stick(raw)

    def _get_stick_horizontal(self, jc) -> float:
        """Return horizontal stick deflection (-1..+1, right-positive)."""
        joycon = jc.joycon
        raw = joycon.get_stick_right_horizontal() if joycon.is_right() else joycon.get_stick_left_horizontal()
        return self._normalize_stick(raw)

    # ── Main loop ─────────────────────────────────────────────────────

    def run(self):
        """Main loop: Joy-Con → EEF poses → Unity (Unity does IK)."""
        print("\nBridge running (Unity IK mode).  Press Ctrl+C to stop.\n")
        print("CONTROLS:")
        print("  Stick up/down    – move end-effector forward / backward (X)")
        print("  Stick left/right – rotate base (J0) left / right")
        print("  Joy-Con tilt     – change end-effector orientation (fixed-base IK)")
        print("  L / R trigger    – move end-effector up (Z)")
        print("  Stick press down – move end-effector down (Z)")
        print("  ZL / ZR          – toggle gripper open / close")
        print("  Home / Capture   – reset pose / recalibrate gyro")
        print("  A button         – next episode")
        print("  Y button         – restart episode\n")

        dt: float = 0.016
        last_time: float = time.perf_counter()

        while True:
            now: float = time.perf_counter()
            dt = now - last_time
            last_time = now

            try:
                # ── Get raw absolute poses from joyconrobotics ──────
                pose_l, grip_l, btn_l = self.jc_left.get_control()
                pose_r, grip_r, btn_r = self.jc_right.get_control()

                # ── Stick integration ───────────────────────────────
                # Stick V:  EEF forward/backward  (X in Joy-Con frame)
                # Stick H:  base rotation  (J0)   (Yaw of base joint)
                # NOTE: horizontal stick sign inverted for intuitive control
                # (stick right → robot turns left, and vice versa)
                stick_v_l = self._get_stick_vertical(self.jc_left)
                stick_v_r = self._get_stick_vertical(self.jc_right)

                SPEED_X = 0.30

                # X position (stick V → X forward)
                self._x_manual_l += stick_v_l * SPEED_X * dt
                self._x_manual_r += stick_v_r * SPEED_X * dt
                # Y is fixed at home (0.0) – lateral motion handled by base rotation
                y_l = _SO100_HOME_XYZ[1]
                y_r = _SO100_HOME_XYZ[1]

                # Z from joyconrobotics pose[2] (uses trigger + stick-press internally)
                z_l = pose_l[2]
                z_r = pose_r[2]

                # ── Build EEF position ──
                eef_pos_l = [self._x_manual_l, y_l, z_l]
                eef_pos_r = [self._x_manual_r, y_r, z_r]

                # ── Use JoyconRobotics Madgwick roll/pitch (matching lerobot_joycon_gpos.py) ──
                # pose[3]=roll, pose[4]=pitch, pose[5]=yaw  (rad, Joy-Con frame)
                roll_l = pose_l[3]
                pitch_l = pose_l[4]
                roll_r = pose_r[3]
                pitch_r = pose_r[4]

                # Apply SO100-specific transformations (matching lerobot_joycon_gpos.py L79-80)
                pitch_l = -pitch_l
                pitch_r = -pitch_r
                roll_l = roll_l - math.pi / 2
                roll_r = roll_r - math.pi / 2

                # Use Madgwick yaw directly as base yaw (matching reference L84,87)
                self._base_yaw_l = pose_l[5]
                self._base_yaw_r = pose_r[5]

                # EEF orientation: roll/pitch only, yaw=0 (J0 handles yaw via base_yaw)
                eef_rot_l = [float(roll_l), float(pitch_l), 0.0]
                eef_rot_r = [float(roll_r), float(pitch_r), 0.0]

                # ── Pitch diagnostics ──────────────────────────────
                self._diag_count += 1
                if self._diag_count % 60 == 0:
                    pl_deg = math.degrees(eef_rot_l[1])
                    pr_deg = math.degrees(eef_rot_r[1])
                    print(
                        f"[PY]  L pitch={eef_rot_l[1]:+.3f} rad ({pl_deg:+6.1f}°)"
                        f" | R pitch={eef_rot_r[1]:+.3f} rad ({pr_deg:+6.1f}°)"
                    )

                # ── Build message ───────────────────────────────────
                msg = {
                    "robot_0": {
                        "pos": [float(v) for v in eef_pos_l],
                        "rot": [float(v) for v in eef_rot_l],
                        "base_yaw": float(self._base_yaw_l),
                        "gripper": float(grip_l),
                        "button": int(btn_l),
                    },
                    "robot_1": {
                        "pos": [float(v) for v in eef_pos_r],
                        "rot": [float(v) for v in eef_rot_r],
                        "base_yaw": float(self._base_yaw_r),
                        "gripper": float(grip_r),
                        "button": int(btn_r),
                    },
                }

                data = (json.dumps(msg) + "\n").encode("utf-8")
                self.conn.sendall(data)
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

    def close(self):
        if self.conn:
            self.conn.close()


def main():
    bridge = JoyConBridge()
    try:
        bridge.init_controllers()
        bridge.connect_to_unity()
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        print("Bridge stopped.")


if __name__ == "__main__":
    main()