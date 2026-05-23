"""
LIBERO Unity Joy-Con Bridge
Reads Nintendo Joy-Con controllers (left + right) and sends teleop data to Unity via TCP.

Left Joy-Con  → Robot  (SO100_L, table left,  X=-0.2)
Right Joy-Con → Robot2 (SO100_R, table right, X=+0.2)

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
from typing import Optional

try:
    from joyconrobotics import JoyconRobotics
except ImportError:
    print("ERROR: joyconrobotics not installed.")
    print("  pip install joyconrobotics")
    print("  Or clone from: https://github.com/box2ai-robotics/joycon-robotics")
    sys.exit(1)


# ── Coordinate convention ───────────────────────────────────────────────
# Joy-Con frame (user-facing):
#   X+ = forward (toward table)    →  Unity Z+
#   Y+ = right                     →  Unity X+
#   Z+ = up                        →  Unity Y+
#
# After the user presses Home to calibrate, the Joy-Con "home" orientation
# should be pointing toward the table.  The bridge does NO coordinate
# remapping here – raw Joy-Con frame values are sent and Unity converts.
# ─────────────────────────────────────────────────────────────────────────

TCP_HOST = "127.0.0.1"
TCP_PORT = 5555

# EEF workspace limits in Joy-Con frame (metres / radians)
# These are the defaults from lerobot-kinematics examples.
CONTROL_GLIMIT = [
    [0.125, -0.4, 0.046, -3.1, -0.75, -1.5],   # lower  [x,y,z,roll,pitch,yaw]
    [0.340, 0.4,  0.23,  2.0,  1.57,  1.5],     # upper
]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class JoyConBridge:
    """Wraps two JoyconRobotics instances and streams data to Unity."""

    def __init__(self):
        self.jc_left: Optional[JoyconRobotics] = None
        self.jc_right: Optional[JoyconRobotics] = None
        self.conn: Optional[socket.socket] = None

        # Delta computational reference: captured on first frame (pos only)
        self._ref_l: Optional[list] = None  # [x,y,z,roll,pitch,yaw]
        self._ref_r: Optional[list] = None

    def init_controllers(self):
        """Connect to both Joy-Con controllers."""
        offset = [0.2, 0.0, 0.5]  # [x, y, z] home offset (metres)

        print("Initializing left Joy-Con ...")
        try:
            self.jc_left = JoyconRobotics(
                device="left",
                horizontal_stick_mode="y",
                close_y=True,
                limit_dof=True,
                glimit=CONTROL_GLIMIT,
                offset_position_m=offset,
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
                horizontal_stick_mode="y",
                close_y=True,
                limit_dof=True,
                glimit=CONTROL_GLIMIT,
                offset_position_m=offset,
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

    # ── Position delta helper ──────────────────────────────────────────

    def _compute_pos_delta(self, raw_pose: list, ref: Optional[list]) -> list:
        """
        Convert absolute Joy-Con position to clamped relative delta.
        Rotation is NOT processed here — sent raw from get_control().

        Returns pos_delta[3]
        """
        # First frame: capture reference and return zero delta
        if ref is None:
            return [0.0, 0.0, 0.0]

        # Position delta: raw position − reference
        pos_delta = [
            raw_pose[i] - ref[i]
            for i in range(3)
        ]

        # Clamp for safety — prevents IK from chasing impossible targets
        max_pos_delta = 1.0  # metres: 1.0 m max displacement

        pos_clamped = [
            clamp(pos_delta[i], -max_pos_delta, max_pos_delta)
            for i in range(3)
        ]

        return pos_clamped

    # ── Main loop ─────────────────────────────────────────────────────

    def run(self):
        """Main loop: read Joy-Cons, send to Unity at ~60 Hz."""
        print("\nBridge running.  Press Ctrl+C to stop.\n")
        print("CONTROLS:")
        print("  Stick up/down    – move end-effector forward / backward (Joy X  direction)")
        print("  Stick left/right – move end-effector left / right (Joy Y direction)")
        print("  L / R trigger    – move end-effector up")
        print("  Stick press down – move end-effector down")
        print("  ZL / ZR          – toggle gripper open / close")
        print("  Home / Capture   – reset pose / recalibrate gyro")
        print("  A button         – next episode")
        print("  Y button         – restart episode\n")

        while True:
            try:
                # ── Get raw absolute poses from joyconrobotics ──────
                pose_l, grip_l, btn_l = self.jc_left.get_control()
                pose_r, grip_r, btn_r = self.jc_right.get_control()

                # ── Capture references on first frame ───────────────
                if self._ref_l is None:
                    self._ref_l = list(pose_l)
                    self._ref_r = list(pose_r)
                    print(f"[ref] Left  pos={pose_l[:3]}  rot={pose_l[3:]}")
                    print(f"[ref] Right pos={pose_r[:3]}  rot={pose_r[3:]}")
                    continue  # skip this frame — next frame starts sending data

                # ── Position: compute clamped delta ─────────────────
                pos_l = self._compute_pos_delta(pose_l, self._ref_l)
                pos_r = self._compute_pos_delta(pose_r, self._ref_r)

                # Fix Y direction: joyconrobotics "y" mode sends stick-right → −Y
                pos_l[1] = -pos_l[1]
                pos_r[1] = -pos_r[1]

                # ── Rotation: use raw absolute [roll, pitch, yaw] ───
                rot_l = [float(pose_l[3]), float(pose_l[4]), float(pose_l[5])]
                rot_r = [float(pose_r[3]), float(pose_r[4]), float(pose_r[5])]

                # ── Build message ───────────────────────────────────
                msg = {
                    "robot_0": {  # Left Joy-Con → Robot (SO100_L)
                        "pos": [float(v) for v in pos_l],
                        "rot": rot_l,
                        "gripper": float(grip_l),
                        "button": int(btn_l),
                    },
                    "robot_1": {  # Right Joy-Con → Robot2 (SO100_R)
                        "pos": [float(v) for v in pos_r],
                        "rot": rot_r,
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