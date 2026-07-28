"""
Replay demo actions using lerobot_IK — EXACT same logic as collect_datasets.py.

Architecture: maintains a virtual Joy-Con "target_pose" state per arm,
accumulates world-frame EEF deltas from the recorded action, and feeds the
reconstructed target_pose through _compute_robot_command() (copied from
collect_datasets.py) for pixel-identical IK → joints.

This eliminates the 4 critical misalignments in the old FK-delta approach:
  1. Base yaw now updates from world-frame orientation decomposition (Z-X-Z)
  2. Euler angles use extrinsic Z-X-Z + the same pitch/roll transforms as collect
  3. Gripper is NOT inverted (raw value, 0=open 1=closed)
  4. target_pose state mirrors Joy-Con output, matching sender-side semantics

Usage: python replay_action_via_ik.py [--traj PATH] [--fps FPS] [--warmup SEC]
"""
import argparse, io, json, math, os, socket, sys, time
from pathlib import Path
import numpy as np

sys.path.insert(0, r"C:\vla\lerobot-kinematics")
from lerobot_kinematics import lerobot_IK, get_robot
from scipy.spatial.transform import Rotation as R


# ── Constants (matching collect_datasets.py) ──────────────────────────────
CONTROL_GLIMIT = [
    [0.125, -0.4, 0.046, -3.1, -1.5, -1.5],
    [0.380, 0.4, 0.23, 3.1, 1.5, 1.5],
]
_SO100_HOME_XYZ = [0.111, 0.0, 0.098]
_INIT_ARM_Q = np.array([-3.14, 3.14, 0.0, -1.57])


# ── Quaternion math (Unity convention: [x, y, z, w]) ──────────────────────

def quat_conjugate(q):
    """Conjugate of unit quaternion [x, y, z, w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(q1, q2):
    """Multiply two quaternions (both [x, y, z, w])."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def quat_to_rotvec(q):
    """Convert unit quaternion [x, y, z, w] to axis-angle vector (axis * angle)."""
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z)
    if norm < 1e-10:
        return np.zeros(3)
    angle = 2.0 * math.atan2(norm, max(min(w, 1.0), -1.0))
    axis = np.array([x, y, z]) / norm
    return axis * angle


def _euler_zxy_to_quat(rx, ry, rz):
    """Convert extrinsic Z-X-Y Euler angles to quaternion [x, y, z, w].

    Matches Unity's Quaternion.Euler(rx, ry, rz): q = q_y(ry) ⊗ q_x(rx) ⊗ q_z(rz)
    """
    hx, hy, hz = rx / 2.0, ry / 2.0, rz / 2.0
    cx, sx = math.cos(hx), math.sin(hx)
    cy, sy = math.cos(hy), math.sin(hy)
    cz, sz = math.cos(hz), math.sin(hz)
    return np.array([
        cy * sx * cz + sy * cx * sz,   # x
        sy * cx * cz - cy * sx * sz,   # y
        cy * cx * sz - sy * sx * cz,   # z
        cy * cx * cz + sy * sx * sz,   # w
    ])


# ── Coordinate transforms (matching _compute_action_label in collect) ─────

def _target_pose_to_world(target_pose):
    """Convert Joy-Con target_pose to world-frame position + quaternion.

    EXACT copy of the math in collect_datasets._compute_action_label() lines 379-399.

    Args:
        target_pose: [x_r, _, z_r, roll_r, pitch_r, yaw_r] — raw Joy-Con values

    Returns:
        (world_pos: np.ndarray[3], world_quat: np.ndarray[4])
    """
    x_r, _, z_r, roll_r, pitch_r, yaw_r = target_pose
    y_r = 0.01  # fixed lateral offset

    # Same orientation transforms as _compute_robot_command
    pitch_t = -pitch_r
    roll_t = roll_r - math.pi / 2

    # World-frame position: rotate base-relative [x_r, y_r, z_r] by base yaw
    cos_y, sin_y = math.cos(yaw_r), math.sin(yaw_r)
    world_x = x_r * cos_y - y_r * sin_y
    world_y = x_r * sin_y + y_r * cos_y
    world_z = z_r
    world_pos = np.array([world_x, world_y, world_z])

    # World-frame orientation: base yaw quat * wrist quat
    # q_wrist = _euler_zxy_to_quat(-pitch_t, 0.0, -(roll_t + π/2))
    #         = _euler_zxy_to_quat(pitch_r, 0.0, -roll_r)   [after substitution]
    #         = Rx(pitch_r) * Rz(-roll_r)
    # q_base  = Rz(yaw_r)
    # world_quat = Rz(yaw_r) * Rx(pitch_r) * Rz(-roll_r)   [extrinsic Z-X-Z]
    q_wrist = _euler_zxy_to_quat(-pitch_t, 0.0, -(roll_t + math.pi / 2))
    q_base = _euler_zxy_to_quat(0.0, 0.0, yaw_r)
    world_quat = quat_multiply(q_base, q_wrist)

    return world_pos, world_quat


def _world_to_target_pose(world_pos, world_quat, prev_target_pose=None):
    """Convert world-frame position + quaternion back to Joy-Con target_pose.

    Inverse of _target_pose_to_world().  The world quaternion is extrinsic Z-X-Z:
        world_quat = Rz(yaw_r) * Rx(pitch_r) * Rz(-roll_r)

    CRITICAL: scipy uses UPPERCASE for extrinsic rotations.
    'ZXZ' (uppercase, extrinsic): R = Rz(euler[0]) * Rx(euler[1]) * Rz(euler[2])
    Our world_quat = Rz(yaw_r) * Rx(pitch_r) * Rz(-roll_r) (extrinsic Z-X-Z):
      euler[0] = yaw_r,  euler[1] = pitch_r,  euler[2] = -roll_r

    Z-X-Z decomposition is NOT unique: (α,β,γ) and (α+π,-β,γ+π) produce the
    same rotation matrix.  We use prev_target_pose to pick the correct branch
    and to wrap angles for temporal continuity.

    Note: scipy has no `extrinsic` keyword arg — case alone controls extrinsic vs intrinsic.

    Args:
        world_pos: np.ndarray[3] — world-frame position
        world_quat: np.ndarray[4] — world-frame quaternion [x, y, z, w]
        prev_target_pose: optional [x_r, _, z_r, roll_r, pitch_r, yaw_r] for branch selection

    Returns:
        target_pose: [x_r, 0.0, z_r, roll_r, pitch_r, yaw_r] — raw Joy-Con format
    """
    world_x, world_y, world_z = world_pos

    # Decompose world quaternion as extrinsic Z-X-Z
    euler = R.from_quat(world_quat).as_euler('ZXZ', degrees=False)
    yaw_candidate = float(euler[0])
    pitch_candidate = float(euler[1])
    roll_candidate = float(-euler[2])  # euler[2] = -roll_r

    # Branch disambiguation: (α, β, γ) vs (α+π, -β, γ+π) produce same rotation.
    # Additionally, when pitch ≈ 0 (gimbal lock), only yaw-roll is determined —
    # scipy forces euler[2]=0 and puts everything in euler[0].
    # We use prev_target_pose to split the total Z rotation between yaw and roll.
    if prev_target_pose is not None:
        prev_roll, prev_pitch, prev_yaw = prev_target_pose[3], prev_target_pose[4], prev_target_pose[5]

        # Normal-case branches: (yaw, pitch, roll) and (yaw+π, -pitch, roll+π)
        branches = [
            (yaw_candidate, pitch_candidate, roll_candidate),
            (yaw_candidate + math.pi, -pitch_candidate, roll_candidate + math.pi),
        ]

        best = None
        best_dist = float('inf')
        for y, p, r in branches:
            for dy in [0, 2*math.pi, -2*math.pi]:
                for dr in [0, 2*math.pi, -2*math.pi]:
                    yy, rr = y + dy, r + dr
                    dist = abs(yy - prev_yaw) + abs(p - prev_pitch) + abs(rr - prev_roll)
                    if dist < best_dist:
                        best_dist = dist
                        best = (yy, p, rr)

        # Gimbal-lock fallback: if the best match is still far from prev (pitch ≈ 0 case),
        # scipy may have forced euler[2]=0.  Only total_z = yaw - roll is well-defined.
        # Split the total Z rotation to keep both yaw and roll close to their prev values.
        yaw_r, pitch_r, roll_r = best
        if abs(pitch_r) < 1e-6 and abs(prev_pitch) < 1e-6:
            total_z = yaw_r - roll_r  # the only well-defined quantity
            prev_diff = prev_yaw - prev_roll
            delta = total_z - prev_diff
            delta_wrapped = (delta + math.pi) % (2*math.pi) - math.pi
            total_z = prev_diff + delta_wrapped
            # Split evenly: keep both yaw and roll close to previous
            yaw_r = prev_yaw + 0.5 * delta_wrapped
            roll_r = prev_roll - 0.5 * delta_wrapped
    else:
        yaw_r, pitch_r, roll_r = yaw_candidate, pitch_candidate, roll_candidate

    # Position: rotate world pos back to base frame
    cos_y, sin_y = math.cos(yaw_r), math.sin(yaw_r)
    x_r = world_x * cos_y + world_y * sin_y
    z_r = world_z
    # y_r = -world_x*sin_y + world_y*cos_y should ≈ 0.01 by construction; not needed

    return [x_r, 0.0, z_r, roll_r, pitch_r, yaw_r]


# ── Robot command (COPIED from collect_datasets.DemoCollector) ────────────

def _compute_robot_command(target_pose, gripper_state, current_arm_q, robot):
    """Robot action processor: Joy-Con target → IK → joint angles.

    EXACT copy of collect_datasets.DemoCollector._compute_robot_command(),
    with warm-start state made explicit via parameters.

    Args:
        target_pose: [x_r, _, z_r, roll_r, pitch_r, yaw_r] — raw Joy-Con values
        gripper_state: raw gripper value (0=open, 1=closed)
        current_arm_q: np.ndarray[4] — IK warm-start (previous IK solution)
        robot: lerobot robot object

    Returns:
        (joint_angles_6d, gripper, new_arm_q) or (None, gripper, current_arm_q) on IK failure
        joint_angles_6d: [yaw, pitch, elbow, wrist_pitch, wrist_roll, gripper]
    """
    # Clamp to workspace limits
    for i in range(6):
        target_pose[i] = max(CONTROL_GLIMIT[0][i], min(CONTROL_GLIMIT[1][i], target_pose[i]))

    x_r, _, z_r, roll_r, pitch_r, yaw_r = target_pose
    y_r = 0.01  # fixed lateral offset

    # Transformations matching lerobot_joycon_gpos_real.py
    pitch_r = -pitch_r
    roll_r = roll_r - math.pi / 2

    right_target_gpos = np.array([x_r, y_r, z_r, roll_r, pitch_r, 0.0])
    qpos_inv, ik_success = lerobot_IK(current_arm_q, right_target_gpos, robot=robot)

    if ik_success:
        target_qpos = np.concatenate(([yaw_r], qpos_inv[:4], [gripper_state]))
        new_arm_q = target_qpos[1:5].copy()  # IK warm-start for next frame
        return target_qpos, gripper_state, new_arm_q
    else:
        return None, gripper_state, current_arm_q


# ── Networking ─────────────────────────────────────────────────────────────

class JoyConClient:
    def __init__(self, host="127.0.0.1", port=5555):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.connect((host, port))

    def send(self, joints_r, grip_r, joints_l, grip_l):
        msg = {
            "robot_0": {"joints": [float(v) for v in joints_r], "gripper": float(grip_r), "button": 0},
            "robot_1": {"joints": [float(v) for v in joints_l], "gripper": float(grip_l), "button": 0},
        }
        self._sock.sendall((json.dumps(msg) + "\n").encode())

    def close(self):
        self._sock.close()


class ObsClient:
    def __init__(self, host="127.0.0.1", port=5556):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.connect((host, port))
        self._buf = b""

    def _send(self, cmd):
        self._sock.sendall((json.dumps(cmd) + "\n").encode())
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("Unity disconnected")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line.decode())

    def reset(self):
        return self._send({"cmd": "reset"})

    def get_obs(self):
        return self._send({"cmd": "get_obs"})

    def close(self):
        self._sock.close()


# ── Home pose ──────────────────────────────────────────────────────────────

def _go_home(joycon, robot):
    """Send both arms to the naturally-bent ready pose via JoyConReceiver.

    Same logic as collect_datasets._go_home(): uses _compute_robot_command
    to get joints, sends via TCP :5555.
    """
    home_target = [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0]

    j_home = [None, None]
    g_home = [0.0, 0.0]
    warm_q = [_INIT_ARM_Q.copy(), _INIT_ARM_Q.copy()]

    for arm_idx in range(2):
        joints, grip, new_q = _compute_robot_command(
            home_target.copy(), 0.0, warm_q[arm_idx], robot)
        if joints is not None:
            j_home[arm_idx] = joints
            g_home[arm_idx] = grip
            warm_q[arm_idx] = new_q

    if j_home[0] is not None and j_home[1] is not None:
        joycon.send(j_home[0], g_home[0], j_home[1], g_home[1])
    time.sleep(0.5)
    print("[Home] Arms sent to home pose.")


# ── Main replay loop ──────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--traj",
        default=r"C:\vla\my\DatasetsCollector\demos\拿起红色方块\episode_0001_拿起红色方块_20260611_200042_success.npz")
    parser.add_argument("--fps", type=float, default=50)
    parser.add_argument("--warmup", type=float, default=3.0)
    args = parser.parse_args()

    data = np.load(args.traj, allow_pickle=True)
    actions = data["action"]
    T = actions.shape[0]
    print(f"[Load] {Path(args.traj).name}  Task: {data.get('language_instruction','?')}  T={T}")

    print(f"\nWarming up {args.warmup:.0f}s — focus Unity! ", end="", flush=True)
    for i in range(int(args.warmup), 0, -1):
        print(f"{i}...", end="", flush=True)
        time.sleep(1)
    print()

    joycon = JoyConClient()
    obs = ObsClient()
    robot = get_robot("so100")
    print("[Connect] JoyConReceiver :5555 + TrainingServer :5556")

    obs.reset()
    time.sleep(1.0)
    _go_home(joycon, robot)
    init = obs.get_obs()
    print(f"[Init] R EEF: {init['robot0_eef_pos']}")

    # ── Per-arm virtual Joy-Con state ──────────────────────────────────
    # target_pose = [x_r, 0.0, z_r, roll_r, pitch_r, yaw_r] — same format as Joy-Con get_control()
    target_pose = [
        [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0],  # right
        [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0],  # left
    ]
    arm_q_warm = [_INIT_ARM_Q.copy(), _INIT_ARM_Q.copy()]  # IK warm-start (matches collect)
    ik_fail_count = [0, 0]

    # Last successful joint command per arm (initialized empty; sent after first IK success)
    last_joints = [None, None]
    last_grip = [0.0, 0.0]

    dt = 1.0 / args.fps
    print(f"\nReplaying {T} steps at {args.fps} Hz via virtual Joy-Con + lerobot_IK...")
    t_start = time.time()
    for t in range(T):
        loop_start = time.perf_counter()
        act = actions[t]

        for arm_idx in range(2):
            base = arm_idx * 7
            dpos = act[base:base+3].astype(float)       # world-frame position delta
            drot = act[base+3:base+6].astype(float)     # world-frame axis-angle rotation delta
            gripper_state = float(act[base+6])           # raw gripper (0=open, 1=closed) — NO inversion

            # ── Step 1: current target_pose → world pose ──────────────
            world_pos, world_quat = _target_pose_to_world(target_pose[arm_idx])

            # ── Step 2: apply world-frame delta ───────────────────────
            world_pos_new = world_pos + dpos

            # Rotation delta: axis-angle → quaternion → apply (world-frame left-multiply)
            if np.linalg.norm(drot) > 1e-10:
                q_delta = R.from_rotvec(drot).as_quat()  # scipy: [x, y, z, w]
                world_quat_new = quat_multiply(q_delta, world_quat)
            else:
                world_quat_new = world_quat

            # ── Step 3: world pose → new target_pose ─────────────────
            # Pass previous target_pose for Z-X-Z branch disambiguation
            target_pose[arm_idx] = _world_to_target_pose(
                world_pos_new, world_quat_new, prev_target_pose=target_pose[arm_idx])

            # ── Step 4: compute robot command (EXACT same as collect) ─
            joints, grip, new_q = _compute_robot_command(
                target_pose[arm_idx].copy(), gripper_state, arm_q_warm[arm_idx], robot)

            if joints is not None:
                arm_q_warm[arm_idx] = new_q
                last_joints[arm_idx] = joints
                last_grip[arm_idx] = grip
                ik_fail_count[arm_idx] = 0
            else:
                ik_fail_count[arm_idx] += 1
                if ik_fail_count[arm_idx] <= 3 or ik_fail_count[arm_idx] % 50 == 0:
                    tp = target_pose[arm_idx]
                    print(f"  [IK FAIL t={t} arm={arm_idx}] x_r={tp[0]:.4f} z_r={tp[2]:.4f} "
                          f"roll_r={tp[3]:.3f} pitch_r={tp[4]:.3f} yaw_r={tp[5]:.3f} "
                          f"(fail #{ik_fail_count[arm_idx]})")
                # Fallback: keep last successful joint command (don't move on IK failure)
                # warm-start is NOT updated — keep the last good solution

        # Send joint commands (use last-known-good on first frames before IK succeeds)
        if last_joints[0] is not None and last_joints[1] is not None:
            joycon.send(last_joints[0], last_grip[0], last_joints[1], last_grip[1])

        if t % 50 == 0:
            print(f"  t={t}/{T}  target_pose_R=[{target_pose[0][0]:.3f},{target_pose[0][2]:.3f},"
                  f"r={target_pose[0][3]:.2f},p={target_pose[0][4]:.2f},y={target_pose[0][5]:.2f}]")

        elapsed_step = time.perf_counter() - loop_start
        if elapsed_step < dt:
            time.sleep(dt - elapsed_step)

    total = time.time() - t_start
    print(f"Done in {total:.1f}s ({int(T)/total:.1f} fps)")

    time.sleep(0.5)
    final = obs.get_obs()
    print(f"\nInit EEF Z: {init['robot0_eef_pos'][2]:.4f}")
    print(f"Final EEF Z: {final['robot0_eef_pos'][2]:.4f}")
    print(f"Init joints: {[f'{j:.3f}' for j in init['robot0_joint_pos']]}")
    print(f"Final joints: {[f'{j:.3f}' for j in final['robot0_joint_pos']]}")
    print(f"IK failures: R={ik_fail_count[0]} L={ik_fail_count[1]} / {T}")
    joycon.close()
    obs.close()


if __name__ == "__main__":
    main()
