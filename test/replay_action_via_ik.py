"""
Replay demo actions using the pluggable IK backend — EXACT same logic as collect_datasets.py.

Architecture: maintains a virtual Joy-Con "target_pose" state per arm,
accumulates world-frame EEF deltas from the recorded action, and feeds the
reconstructed target_pose through the selected IK backend (arm_ik.py) for
joints → JoyConReceiver :5555.

IK backends (--ik-backend):
  lerobot  — C ext lerobot_IK (original; kinematic model ≠ MuJoCo, 30-50cm off)
  mujoco   — MuJoCo Python IK on the SAME model as Unity (default)
  placo    — stub (placo not installed / frame conversion not wired)

Target modes (--ik-target):
  pose  — reconstructed Joy-Con target_pose → solve() (all backends)
  eef   — recorded MuJoCo EEF observation → solve_eef_pos() (mujoco only;
          auto-fallback to pose when observations are stale, e.g. June demos)

Note on fast path (eef): the recorded robot*_eef_pos is in MuJoCo WORLD
coordinates; the arm IK solves in the arm BASE frame, so each frame converts
eef_arm = (rel_y, -rel_x, rel_z) with rel = obs_eef - base_pos (the base is
rotated Rz(+90°) in the XML: world_rel = Rz(90°) · eef_arm).

Usage: python replay_action_via_ik.py [--traj PATH] [--fps FPS] [--warmup SEC]
                                     [--ik-backend {lerobot,mujoco,placo}]
                                     [--ik-target {pose,eef,auto}]
                                     [--dir DIR] [--filter SUBSTR] [--quiet]
"""
import argparse, io, json, math, os, socket, sys, time, traceback, warnings
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))  # arm_ik.py, mujoco_ik.py
from scipy.spatial.transform import Rotation as R

from arm_ik import create_ik, INIT_ARM_Q, CONTROL_GLIMIT, _SO100_HOME_XYZ


# ── Backward-compat aliases (validate_replay_offline.py / run_inference.py) ─
_INIT_ARM_Q = INIT_ARM_Q

# ── MuJoCo arm bases (for the eef fast path: obs world → arm frame) ────────
_RIGHT_BASE = np.array([-0.70, -0.20, -0.075])
_LEFT_BASE = np.array([-0.70, 0.20, -0.075])


# ── Gripper mapping ────────────────────────────────────────────────────────
# Raw gripper (joyconrobotics): 1 = open, 0 = close.
# Unity MjJoyConController.SetJoint() writes the value straight into the
# MuJoCo Jaw position actuator ctrl, whose range is the joint range
# [-0.174, 1.75] rad.  Sending 0/1 directly means "close" puts the jaw at
# 1.0 rad (almost fully OPEN) — the gripper can never close.  Map to radians
# with the same Lerp as TrainingServer.ApplyEefDelta (1.75 → -0.174).
JAW_OPEN_RAD = 1.75
JAW_CLOSE_RAD = -0.174


def gripper_to_jaw(gripper: float) -> float:
    """Map raw gripper (1=open, 0=close) to Jaw actuator radians.

    joyconrobotics: 0=close, 1=open.  Lerp keeps the direction consistent:
      gripper=0 (close) → -0.174 rad (Jaw 闭合位)
      gripper=1 (open)  → 1.75 rad  (Jaw 全开位)
    """
    return JAW_CLOSE_RAD + (JAW_OPEN_RAD - JAW_CLOSE_RAD) * gripper


def _obs_joints_to_arm(obs_joints: np.ndarray) -> np.ndarray:
    """Reorder Unity obs joints [Wrist_Pitch, Wrist_Roll, Rotation, Pitch,
    Elbow, Jaw] → arm order [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll]."""
    return np.array([obs_joints[2], obs_joints[3], obs_joints[4],
                     obs_joints[0], obs_joints[1]], dtype=np.float64)


def _obs_eef_to_arm(obs_eef: np.ndarray, base: np.ndarray) -> np.ndarray:
    """MuJoCo world EEF → arm base frame (base rotated Rz(+90°) in the XML:
    world_rel = Rz(90°)·eef_arm  →  eef_arm = (rel_y, -rel_x, rel_z))."""
    rel = np.asarray(obs_eef, dtype=np.float64) - base
    return np.array([rel[1], -rel[0], rel[2]])


def _eef_obs_valid(obs_eef: np.ndarray, base: np.ndarray) -> bool:
    """Sanity check: recorded EEF is a plausible reach from the arm base
    (June demos have stale/garbage eef — those must use the pose path)."""
    rel = np.asarray(obs_eef, dtype=np.float64) - base
    d = np.linalg.norm(rel)
    return 0.10 < d < 0.65


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

    Inverse of _target_pose_to_world().  The world pose is structurally
        world_pos  = Rz(yaw)  · (x_r, y_local, z_r),   y_local = 0.01
        world_quat = Rz(yaw)  · Rx(pitch) · Rz(-roll)  (extrinsic Z-X-Z)

    The recorded trajectory ALWAYS lies on the "local y = 0.01" cylinder —
    Joy-Con targets are built with y = 0.01 fixed, and the world deltas are
    exact differences of such poses.  So yaw can be solved EXACTLY from the
    position constraint instead of the numerically ill-conditioned Z-X-Z
    quaternion decomposition:

        wy·cosψ - wx·sinψ = r·sin(θ - ψ) = 0.01,   r = hypot(wx,wy), θ = atan2(wy,wx)
        → ψ = θ - asin(0.01/r)  or  θ - π + asin(0.01/r)

    The Z-X-Z decomposition degenerates whenever pitch ≈ 0 (gimbal lock —
    SO100's wrist pitch is naturally ~0 during grasping, which used to smear
    noise into yaw/roll and destroy the position).  The position constraint
    has NO such degeneracy.  pitch/roll are then extracted from
    Rz(-yaw)·world_quat = Rx(pitch)·Rz(-roll), a two-angle decomposition
    that is well-conditioned everywhere (valid away from roll≈π / pitch≈π,
    never reached by SO100 joint limits).

    The old scipy Z-X-Z path is kept only as a fallback for the degenerate
    case world near the origin (r ≈ 0), where yaw is unconstrained by
    position.

    Args:
        world_pos: np.ndarray[3] — world-frame position
        world_quat: np.ndarray[4] — world-frame quaternion [x, y, z, w]
        prev_target_pose: optional [x_r, _, z_r, roll_r, pitch_r, yaw_r]
            for candidate selection (the constraint has two solutions ≈ π apart)

    Returns:
        target_pose: [x_r, 0.0, z_r, roll_r, pitch_r, yaw_r] — raw Joy-Con format
    """
    world_x, world_y, world_z = world_pos
    y_local = 0.01  # Joy-Con cylindrical frame: local y is always 0.01

    # ── yaw from position constraint (well-conditioned, no gimbal lock) ──
    r = math.hypot(world_x, world_y)
    if r > abs(y_local) + 1e-12:
        theta = math.atan2(world_y, world_x)
        delta = math.asin(min(1.0, y_local / r))
        cand1 = theta - delta
        cand2 = theta - math.pi + delta  # ≈ π away; continuity picks the right one
        if prev_target_pose is None:
            yaw_r = cand1
        else:
            prev_yaw = prev_target_pose[5]
            yaw_r = min(cand1, cand2, key=lambda c: abs(
                ((c - prev_yaw + math.pi) % (2 * math.pi)) - math.pi))

        # ── pitch/roll from Rz(-yaw)·world_quat = Rx(pitch)·Rz(-roll) ──
        # q_M = [sin(p/2)cos(r/2), sin(p/2)sin(r/2), -cos(p/2)sin(r/2), cos(p/2)cos(r/2)]
        q_base_inv = _euler_zxy_to_quat(0.0, 0.0, -yaw_r)
        qx, qy, qz, qw = quat_multiply(q_base_inv, world_quat)
        pitch_r = 2.0 * math.atan2(qx, qw)
        roll_r = -2.0 * math.atan2(qz, qw)

        cos_y, sin_y = math.cos(yaw_r), math.sin(yaw_r)
        x_r = world_x * cos_y + world_y * sin_y
        return [x_r, 0.0, world_z, roll_r, pitch_r, yaw_r]

    # ── Degenerate fallback (world near origin): Z-X-Z decomposition ──
    euler = R.from_quat(world_quat).as_euler('ZXZ', degrees=False)
    yaw_candidate = float(euler[0])
    pitch_candidate = float(euler[1])
    roll_candidate = float(-euler[2])  # euler[2] = -roll_r

    # Branch disambiguation: (α, β, γ) vs (α+π, -β, γ+π) produce same rotation.
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

        # Gimbal-lock fallback: pitch ≈ 0 → only total_z = yaw - roll is
        # well-defined.  Split the total Z rotation to keep yaw/roll continuous.
        yaw_r, pitch_r, roll_r = best
        if abs(pitch_r) < 0.05 and abs(prev_pitch) < 0.05:
            total_z = yaw_r - roll_r
            prev_diff = prev_yaw - prev_roll
            delta = total_z - prev_diff
            delta_wrapped = (delta + math.pi) % (2*math.pi) - math.pi
            total_z = prev_diff + delta_wrapped
            yaw_r = prev_yaw + 0.5 * delta_wrapped
            roll_r = prev_roll - 0.5 * delta_wrapped
    else:
        yaw_r, pitch_r, roll_r = yaw_candidate, pitch_candidate, roll_candidate

    cos_y, sin_y = math.cos(yaw_r), math.sin(yaw_r)
    x_r = world_x * cos_y + world_y * sin_y

    return [x_r, 0.0, world_z, roll_r, pitch_r, yaw_r]


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

def _go_home(joycon, ik):
    """Send both arms to the naturally-bent ready pose via JoyConReceiver.

    Same logic as collect_datasets._go_home(): runs IK for a home target pose,
    sends joints via TCP :5555.

    Returns:
        home_warm_q: [q_r, q_l] — the home IK solutions (5-DOF), to be used as
        the IK warm-start of the replay loop (collect_datasets._go_home() keeps
        these in self.current_arm_q_*; dropping them here can converge the
        first replay frame to a different IK branch).
    """
    home_target = [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0]

    j_home = [None, None]
    g_home = [0.0, 0.0]
    warm_q = [INIT_ARM_Q.copy(), INIT_ARM_Q.copy()]

    for arm_idx in range(2):
        ik[arm_idx].reset_warm()  # per-episode warm reset (matches collect)
        joints, new_q = ik[arm_idx].solve(home_target.copy(), 0.0, warm_q[arm_idx])
        if joints is not None:
            # Home pose: jaw closed (-0.174) — same as collect's gripper 0
            j_home[arm_idx] = np.concatenate((joints, [gripper_to_jaw(0.0)]))
            g_home[arm_idx] = 0.0
            warm_q[arm_idx] = new_q

    if j_home[0] is not None and j_home[1] is not None:
        joycon.send(j_home[0], g_home[0], j_home[1], g_home[1])
    time.sleep(0.5)
    print("[Home] Arms sent to home pose.")
    return warm_q


# ── Per-episode helpers ───────────────────────────────────────────────────

def _compute_eef_eligibility(data, ik, ik_target):
    """Per-arm eef fast-path eligibility for ONE .npz (file-specific).

    Requires valid recorded EEF observations (July+ demos; June demos have
    stale zeros) AND the mujoco backend.

    Returns:
        (eef_ok: [bool, bool], obs_eef: [np.ndarray, np.ndarray],
         obs_joints: [np.ndarray, np.ndarray], bases: [np.ndarray, np.ndarray])
    """
    T = data["action"].shape[0]
    obs_eef = [np.asarray(data["robot0_eef_pos"]), np.asarray(data["robot1_eef_pos"])]
    obs_joints = [np.asarray(data["robot0_joint_pos"]), np.asarray(data["robot1_joint_pos"])]
    bases = [_RIGHT_BASE, _LEFT_BASE]
    eef_ok = [False, False]
    if ik_target in ("auto", "eef") and ik[0].name == "mujoco":
        for a in range(2):
            n = obs_eef[a].shape[0]
            if n < T:
                continue
            valid_frac = np.mean([_eef_obs_valid(obs_eef[a][t], bases[a]) for t in range(0, n, 20)])
            joints_valid = not np.all(np.abs(obs_joints[a]) < 1e-6)
            if valid_frac > 0.8 and joints_valid:
                eef_ok[a] = True
    return eef_ok, obs_eef, obs_joints, bases


def _npz_success(data):
    """Read the 'success' flag from a .npz; return None if absent."""
    if "success" in data.files:
        return bool(np.asarray(data["success"]).item())
    return None


def replay_episode(joycon, obs, ik, data, fps, ik_target, name="?", quiet=False):
    """Replay ONE episode through Unity: reset → home → replay loop.

    All per-episode state is local, so repeated calls are clean.
    TCP sockets and IK instances are owned by the CALLER.

    Args:
        name: display name for this episode (e.g. the .npz filename)

    Returns:
        dict with keys: name, task, T, duration, fps, ik_fail, npz_success,
                        ok, error
    """
    actions = data["action"]
    T = actions.shape[0]
    task = str(data.get("language_instruction", "?"))

    if not quiet:
        print(f"[Load] {name}  Task: {task}  T={T}")

    # ── Per-file eef fast-path eligibility ──────────────────────────────
    eef_ok, obs_eef, obs_joints, bases = _compute_eef_eligibility(data, ik, ik_target)
    if any(eef_ok):
        print(f"[IK] eef fast path: R={eef_ok[0]} L={eef_ok[1]} "
              f"(stale obs → pose path)")

    obs.reset()
    time.sleep(1.0)
    arm_q_warm = _go_home(joycon, ik)
    init = obs.get_obs()
    if not quiet:
        print(f"[Init] R EEF: {init['robot0_eef_pos']}")

    # ── Per-arm virtual Joy-Con state ──────────────────────────────────
    home_tp = [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0]
    target_pose = [home_tp.copy(), home_tp.copy()]
    wp_home, wq_home = _target_pose_to_world(home_tp)
    world_pos = [wp_home.copy(), wp_home.copy()]
    world_quat = [wq_home.copy(), wq_home.copy()]
    ik_fail_count = [0, 0]
    last_joints = [None, None]
    last_grip = [0.0, 0.0]

    dt = 1.0 / fps
    if not quiet:
        print(f"\nReplaying {T} steps at {fps} Hz via virtual Joy-Con + {ik[0].name} IK...")
    t_start = time.time()
    for t in range(T):
        loop_start = time.perf_counter()
        act = actions[t]

        for arm_idx in range(2):
            base = arm_idx * 7
            dpos = act[base:base+3].astype(float)
            drot = act[base+3:base+6].astype(float)
            gripper_state = float(act[base+6])

            # ── Step 1: apply world-frame delta to tracked pose ───────
            world_pos[arm_idx] = world_pos[arm_idx] + dpos

            if np.linalg.norm(drot) > 1e-10:
                q_delta = R.from_rotvec(drot).as_quat()
                world_quat[arm_idx] = quat_multiply(q_delta, world_quat[arm_idx])

            # ── Step 2: solve joints via the selected IK backend ──────
            joints5 = None
            if eef_ok[arm_idx] and _eef_obs_valid(obs_eef[arm_idx][t], bases[arm_idx]):
                eef_arm = _obs_eef_to_arm(obs_eef[arm_idx][t], bases[arm_idx])
                q_warm = arm_q_warm[arm_idx]
                if not np.all(np.abs(obs_joints[arm_idx][t]) < 1e-6):
                    q_warm = _obs_joints_to_arm(obs_joints[arm_idx][t])
                joints5 = ik[arm_idx].solve_eef_pos(eef_arm, q_warm)
            else:
                target_pose[arm_idx] = _world_to_target_pose(
                    world_pos[arm_idx], world_quat[arm_idx],
                    prev_target_pose=target_pose[arm_idx])
                joints5, new_q = ik[arm_idx].solve(
                    target_pose[arm_idx].copy(), gripper_state, arm_q_warm[arm_idx])
                if joints5 is not None:
                    arm_q_warm[arm_idx] = new_q

            if joints5 is not None:
                joints = np.concatenate((joints5, [gripper_to_jaw(gripper_state)]))
                last_joints[arm_idx] = joints
                last_grip[arm_idx] = gripper_state
                ik_fail_count[arm_idx] = 0
            else:
                ik_fail_count[arm_idx] += 1
                if ik_fail_count[arm_idx] <= 3 or ik_fail_count[arm_idx] % 50 == 0:
                    tp = target_pose[arm_idx]
                    print(f"  [IK FAIL t={t} arm={arm_idx}] x_r={tp[0]:.4f} z_r={tp[2]:.4f} "
                          f"roll_r={tp[3]:.3f} pitch_r={tp[4]:.3f} yaw_r={tp[5]:.3f} "
                          f"(fail #{ik_fail_count[arm_idx]})")

        if last_joints[0] is not None and last_joints[1] is not None:
            joycon.send(last_joints[0], last_grip[0], last_joints[1], last_grip[1])

        if t % 50 == 0 and not quiet:
            print(f"  t={t}/{T}  target_pose_R=[{target_pose[0][0]:.3f},{target_pose[0][2]:.3f},"
                  f"r={target_pose[0][3]:.2f},p={target_pose[0][4]:.2f},y={target_pose[0][5]:.2f}]")

        elapsed_step = time.perf_counter() - loop_start
        if elapsed_step < dt:
            time.sleep(dt - elapsed_step)

    total = time.time() - t_start
    print(f"Done in {total:.1f}s ({int(T)/total:.1f} fps)")

    time.sleep(0.5)
    final = obs.get_obs()
    if not quiet:
        print(f"\nInit EEF Z: {init['robot0_eef_pos'][2]:.4f}")
        print(f"Final EEF Z: {final['robot0_eef_pos'][2]:.4f}")
        print(f"Init joints: {[f'{j:.3f}' for j in init['robot0_joint_pos']]}")
        print(f"Final joints: {[f'{j:.3f}' for j in final['robot0_joint_pos']]}")
        print(f"IK failures: R={ik_fail_count[0]} L={ik_fail_count[1]} / {T}")

    return {
        "name": name,
        "task": task,
        "T": T,
        "duration": total,
        "fps": int(T) / total,
        "ik_fail": ik_fail_count,
        "npz_success": _npz_success(data),
        "ok": (ik_fail_count[0] == 0 and ik_fail_count[1] == 0
               and last_joints[0] is not None and last_joints[1] is not None),
        "error": None,
    }


# ── Summary table ──────────────────────────────────────────────────────────

def _print_summary(results):
    """Columnar per-file summary + totals."""
    ok_count = sum(1 for r in results if r["ok"])
    npz_ok = sum(1 for r in results if r["npz_success"] is True)
    npz_total = sum(1 for r in results if r["npz_success"] is not None)
    total_time = sum(r["duration"] for r in results if r.get("duration"))

    print(f"\n{'='*95}")
    print(f"[Batch] replay OK {ok_count}/{len(results)}", end="")
    if npz_total:
        print(f"   (npz success {npz_ok}/{npz_total})")
    else:
        print()
    print(f"{'file':<52s} {'T':>5s}  {'dur(s)':>7s}  {'IK_fail R/L':>11s}  {'npz':>4s}  verdict")
    print("-" * 95)
    for r in results:
        dur = f"{r['duration']:.1f}" if r.get("duration") else "-"
        if r.get("ik_fail"):
            ikf = f"{r['ik_fail'][0]}/{r['ik_fail'][1]}"
        else:
            ikf = "-"
        if r["npz_success"] is True:
            ns = "yes"
        elif r["npz_success"] is False:
            ns = "no"
        else:
            ns = "?"
        if r["error"]:
            verdict = "ERR"
        elif r["ok"]:
            verdict = "PASS"
        else:
            verdict = "FAIL"
        notes = f"  ({r['error']})" if r["error"] else ""
        print(f"{r['name']:<52s} {r.get('T',0):>5d}  {dur:>7s}  {ikf:>11s}  {ns:>4s}  {verdict}{notes}")
    print("-" * 95)
    print(f"Totals: replay OK {ok_count}/{len(results)}", end="")
    if npz_total:
        print(f"   npz success {npz_ok}/{npz_total}   total time {total_time:.1f}s")
    else:
        print(f"   total time {total_time:.1f}s")


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--traj",
        default=r"C:\vla\my\DatasetsCollector\demos\new\episode_0001_pick_up_the_red_block_20260728_220541_success.npz")
    parser.add_argument("--fps", type=float, default=50)
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--ik-backend", default="mujoco",
                        choices=["lerobot", "mujoco", "placo"])
    parser.add_argument("--ik-target", default="auto", choices=["auto", "pose", "eef"])
    parser.add_argument("--dir", default=None,
                        help="batch mode: replay ALL *.npz under this dir (recursive)")
    parser.add_argument("--filter", default=None,
                        help="only episodes whose language_instruction contains this substring")
    parser.add_argument("--quiet", action="store_true",
                        help="suppress per-frame/per-episode verbose prints (batch)")
    args = parser.parse_args()

    # ── IK backend (ONE INSTANCE PER ARM — created once, reused across episodes)
    ik = [create_ik(args.ik_backend) for _ in range(2)]
    print(f"[IK] backend={ik[0].name}  target-mode={args.ik_target}")
    if args.ik_target == "eef" and ik[0].name != "mujoco":
        print(f"[WARN] --ik-target eef requires --ik-backend mujoco; "
              f"falling back to pose path")

    # ── Resolve file list ───────────────────────────────────────────────
    files = None
    if args.dir:
        files = sorted(Path(args.dir).rglob("*.npz"))
        if args.filter:
            needle = args.filter.lower()
            keep = []
            for f in files:
                try:
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore")
                        with np.load(f, allow_pickle=True) as d:
                            if needle in str(d.get("language_instruction", "")).lower():
                                keep.append(f)
                except Exception:
                    pass
            files = keep
            print(f"[Batch] filter '{args.filter}': {len(files)} files match")
        if not files:
            raise SystemExit(f"No .npz (matching '{args.filter or '*'}') under {args.dir}")

    # ── Warmup countdown — ONCE ─────────────────────────────────────────
    print(f"\nWarming up {args.warmup:.0f}s — focus Unity! ", end="", flush=True)
    for i in range(int(args.warmup), 0, -1):
        print(f"{i}...", end="", flush=True)
        time.sleep(1)
    print()

    # ── TCP clients — created ONCE, persist across episodes ─────────────
    joycon = JoyConClient()
    obs = ObsClient()
    print("[Connect] JoyConReceiver :5555 + TrainingServer :5556")

    # ── Batch loop ──────────────────────────────────────────────────────
    if files is not None:
        print(f"[Batch] {len(files)} trajectories under {args.dir}")
        results = []
        for f in files:
            print(f"\n{'='*70}\n[{len(results)+1}/{len(files)}] {f.name}\n{'='*70}")
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    with np.load(f, allow_pickle=True) as data:
                        res = replay_episode(joycon, obs, ik, data, args.fps,
                                             args.ik_target, name=f.name,
                                             quiet=args.quiet)
                results.append(res)
            except ConnectionError as e:
                results.append({"name": f.name, "ok": False,
                                "error": f"Unity disconnected: {e}", "T": 0,
                                "duration": 0, "ik_fail": [0, 0],
                                "npz_success": None, "fps": 0, "task": "?"})
                print(f"  ERROR: {results[-1]['error']} — aborting batch")
                break
            except Exception as e:
                traceback.print_exc()
                results.append({"name": f.name, "ok": False, "error": str(e),
                                "T": 0, "duration": 0, "ik_fail": [0, 0],
                                "npz_success": None, "fps": 0, "task": "?"})
                continue
        _print_summary(results)
    else:
        # ── Single-file path ────────────────────────────────────────────
        traj_path = Path(args.traj)
        if traj_path.is_dir():
            raise SystemExit(
                f"'{args.traj}' is a directory, not a .npz file.\n"
                f"Use --dir for batch replay: python replay_action_via_ik.py --dir \"{args.traj}\"")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            with np.load(args.traj, allow_pickle=True) as data:
                replay_episode(joycon, obs, ik, data, args.fps,
                               args.ik_target, name=Path(args.traj).name,
                               quiet=args.quiet)

    joycon.close()
    obs.close()


if __name__ == "__main__":
    main()
