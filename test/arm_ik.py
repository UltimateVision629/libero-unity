"""
Unified IK backends for SO100 — Strategy pattern.

collect / replay / inference share one interface so the IK engine can be
switched with a single --ik-backend flag:

    backend    | solver                          | model consistency w/ MuJoCo
    -----------|---------------------------------|------------------------------
    lerobot    | so100_chain (pure numpy/scipy)  | ❌ 30-50cm off
    mujoco     | MuJoCo FK + scipy least_squares | ✅ same model (4.5e-6 m)
    placo      | placo + URDF (stub)             | ⚠️ URDF ≠ XML

All backends implement `solve(target_pose, gripper, current_q)` with IDENTICAL
input/output semantics so callers are backend-agnostic:

    target_pose: [x_r, _, z_r, roll_r, pitch_r, yaw_r]  (raw Joy-Con values)
    gripper:     raw 0/1 — passed through, NOT mapped (caller maps to Jaw)
    current_q:   5-DOF warm-start [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll]

    returns: (joints_5dof | None, new_warm_q_5dof)
             None joints = IK failure (caller keeps last command)

The Jaw value is appended by the caller:
    replay uses gripper_to_jaw(); collect/inference pass raw 0/1 straight.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

# Local so100_chain (network/scripts/utils) — replaces lerobot_kinematics for
# the Joy-Con → MuJoCo coordinate translation (pure numpy + scipy).
_UTILS_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "network", "scripts"))
if _UTILS_ROOT not in sys.path:
    sys.path.insert(0, _UTILS_ROOT)
from utils.so100_chain import so100_ik  # noqa: E402

# ═══════════════════════════════════════════════════════════════════════════
# Constants (matching collect_datasets.py / replay_action_via_ik.py)
# ═══════════════════════════════════════════════════════════════════════════

CONTROL_GLIMIT = [
    [0.125, -0.4, 0.046, -3.1, -1.5, -1.5],
    [0.380, 0.4, 0.23, 3.1, 1.5, 1.5],
]

# 5-DOF warm-start: [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll]
INIT_ARM_Q = np.array([0.0, -3.14, 3.14, 0.0, -1.57])

_SO100_HOME_XYZ = [0.111, 0.0, 0.098]


def _clamp_target_pose(tp: list[float]) -> list[float]:
    """Clamp Joy-Con target_pose to workspace limits (same as collect)."""
    out = [float(v) for v in tp]
    for i in range(6):
        out[i] = max(CONTROL_GLIMIT[0][i], min(CONTROL_GLIMIT[1][i], out[i]))
    return out


def _target_gpos(tp: list[float]) -> np.ndarray:
    """Joy-Con target_pose → lerobot_IK target [x,y,z,roll,pitch,yaw=0]."""
    x_r, _, z_r, roll_r, pitch_r, yaw_r = tp
    y_r = 0.01  # fixed lateral offset
    pitch_r = -pitch_r
    roll_r = roll_r - math.pi / 2
    return np.array([x_r, y_r, z_r, roll_r, pitch_r, 0.0])


# ═══════════════════════════════════════════════════════════════════════════
# Interface
# ═══════════════════════════════════════════════════════════════════════════

class ArmIK:
    """IK solver interface — one instance services both arms."""

    name = "base"

    def solve(self, target_pose, gripper, current_q):
        """Joy-Con target_pose → (joints_5dof | None, new_warm_q).

        Gripper is passed through unmodified; the caller maps it to the Jaw
        joint (gripper_to_jaw for replay, raw for collect/inference).
        """
        raise NotImplementedError

    def solve_eef_pos(self, eef_pos, current_q):
        """MuJoCo arm-frame EEF position [x,y,z] → 5-DOF joints.

        Fast path — only MuJoCoIK implements this natively; other backends
        raise NotImplementedError.
        """
        raise NotImplementedError

    def reset_warm(self):
        """Reset internal warm-start state (called on episode reset / go_home)."""
        pass


# ═══════════════════════════════════════════════════════════════════════════
# Backend 1: so100_chain — the ORIGINAL lerobot solver (pure numpy/scipy)
# ═══════════════════════════════════════════════════════════════════════════

class LerobotIK(ArmIK):
    """lerobot-model backend (pure-Python so100_chain).  Model ≠ MuJoCo (30-50cm off)."""

    name = "lerobot"

    def __init__(self, robot=None):
        # robot param retained for call-compat; the chain params are now
        # hard-coded in utils.so100_chain (identical to create_so100).
        self.robot = robot

    def solve(self, target_pose, gripper, current_q):
        tp = _clamp_target_pose(target_pose)
        yaw_r = tp[5]
        target_gpos = _target_gpos(tp)
        q4 = np.asarray(current_q[1:5], dtype=np.float64)  # drop Rotation

        qpos_inv, ik_success = so100_ik(q4, target_gpos)
        if not ik_success:
            return None, current_q

        joints = np.concatenate(([yaw_r], qpos_inv[:4]))
        return joints, joints.copy()


# ═══════════════════════════════════════════════════════════════════════════
# Backend 2: MuJoCo Python IK — FK and IK on the SAME model as Unity
# ═══════════════════════════════════════════════════════════════════════════

class MuJoCoIK(ArmIK):
    """MuJoCo-binding backend.  Chain path translates Joy-Con targets via
    so100_ik→MuJoCo FK; the fast path solves directly from a MuJoCo EEF
    position (recorded observation or accumulated world pose).

    IMPORTANT: create ONE instance PER ARM.  The so100 translation tracks
    its own warm-start (matching collect_datasets' per-arm warm), so sharing
    one instance across arms would cross-contaminate the two warms.
    """

    name = "mujoco"

    def __init__(self, robot=None, arm_ik=None):
        # so100_ik is used only as a "coordinate translator" (Joy-Con frame →
        # MuJoCo frame); the final joints come from MuJoCo's own IK.
        self.robot = robot  # retained for call-compat (unused)
        if arm_ik is None:
            from mujoco_ik import MuJoCoArmIK
            arm_ik = MuJoCoArmIK()
        self.arm_ik = arm_ik
        # Per-arm so100 translation warm-start (4-DOF), initialized on first
        # solve from current_q[1:5] — collect_datasets resets to
        # _INIT_ARM_Q[1:5] home
        self._lerobot_warm = None

    def _reset_lerobot_warm(self, current_q):
        self._lerobot_warm = np.asarray(current_q[1:5], dtype=np.float64).copy()

    def reset_warm(self):
        """Forget the tracked translation warm (per-episode reset)."""
        self._lerobot_warm = None

    def solve(self, target_pose, gripper, current_q):
        # 1. so100_ik translates Joy-Con target → joints (any model).
        #    Warm-start the translation with the TRACKED solution sequence
        #    (exactly as collect_datasets does).  Warm-starting it from the
        #    MuJoCo solution drifts the translation — a wrong warm converges
        #    to a different branch that slowly walks away from the recorded
        #    trajectory.
        tp = _clamp_target_pose(target_pose)
        yaw_r = tp[5]
        target_gpos = _target_gpos(tp)
        if self._lerobot_warm is None:
            self._reset_lerobot_warm(current_q)
        qpos_inv, ok = so100_ik(self._lerobot_warm, target_gpos)
        if not ok:
            return None, current_q
        self._lerobot_warm = np.asarray(qpos_inv[:4], dtype=np.float64).copy()

        # 2. where would those joints put the EEF in the MuJoCo model?
        joints_l = np.concatenate(([yaw_r], qpos_inv[:4]))
        eef_target = self.arm_ik.fk(joints_l)

        # 3. MuJoCo IK to that position, warm-started from current_q
        q_m = self.arm_ik.ik(current_q, eef_target)
        return q_m, q_m.copy()

    def solve_eef_pos(self, eef_pos, current_q):
        """Direct MuJoCo IK from an arm-frame EEF position."""
        q_m = self.arm_ik.ik(current_q, np.asarray(eef_pos, dtype=np.float64))
        return q_m


# ═══════════════════════════════════════════════════════════════════════════
# Backend 3: placo + URDF (stub — needs placo installed + frame conversion)
# ═══════════════════════════════════════════════════════════════════════════

class PlacoIK(ArmIK):
    """placo + URDF backend.  Requires `pip install placo`.

    Not yet wired: the Joy-Con target_pose → URDF frame conversion needs the
    same treatment as MuJoCoIK's chain (translate via lerobot_IK → FK on the
    URDF model → placo IK), or a verified URDF that matches MuJoCo exactly.
    """

    name = "placo"

    def __init__(self, urdf_path=None):
        try:
            import placo  # noqa: F401
        except ImportError:
            raise ImportError(
                "'placo' is required for the placo backend. "
                "Install it with: pip install placo")
        self.urdf_path = urdf_path
        raise NotImplementedError(
            "PlacoIK is a stub — URDF frame conversion not implemented yet. "
            "Use backend='mujoco' (same model as Unity).")


# ═══════════════════════════════════════════════════════════════════════════
# Factory
# ═══════════════════════════════════════════════════════════════════════════

def create_ik(backend: str = "mujoco", robot=None) -> ArmIK:
    """Create an IK backend by name.

    Args:
        backend: "lerobot" | "mujoco" | "placo"  (default "mujoco")
        robot: ignored (legacy param, kept for call-compat)
    """
    b = (backend or "mujoco").strip().lower()
    if b == "lerobot":
        return LerobotIK(robot)
    if b == "mujoco":
        return MuJoCoIK(robot)
    if b == "placo":
        return PlacoIK()
    raise ValueError(f"Unknown IK backend: {backend!r} (options: lerobot, mujoco, placo)")


# ═══════════════════════════════════════════════════════════════════════════
# Self-test
# ═══════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=== create_ik self-test ===")
    for backend in ("lerobot", "mujoco"):
        try:
            ik = create_ik(backend)
        except Exception as e:
            print(f"  {backend}: FAILED to create — {e}")
            continue
        print(f"  {backend}: created ({ik.name})")
        # Home target round-trip
        home_tp = [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0]
        q = INIT_ARM_Q.copy()
        joints, q2 = ik.solve(home_tp, 0.0, q)
        if joints is None:
            print(f"  {backend}: home IK FAILED")
        else:
            print(f"  {backend}: home joints = [{joints[0]:.3f}, {joints[1]:.3f}, "
                  f"{joints[2]:.3f}, {joints[3]:.3f}, {joints[4]:.3f}]")
        # A forward reach target
        tp = [0.30, 0.0, 0.07, -0.1, 0.0, -0.2]
        joints, q2 = ik.solve(tp, 0.0, q2)
        if joints is None:
            print(f"  {backend}: reach IK FAILED")
        else:
            print(f"  {backend}: reach joints = [{joints[0]:.3f}, {joints[1]:.3f}, "
                  f"{joints[2]:.3f}, {joints[3]:.3f}, {joints[4]:.3f}]")
    # mujoco fast path
    try:
        ik = create_ik("mujoco")
        q = INIT_ARM_Q.copy()
        j, q2 = ik.solve([0.111, 0.0, 0.098, 0, 0, 0], 0.0, q)
        eef = ik.arm_ik.fk(j)
        j2 = ik.solve_eef_pos(eef, q2)
        eef2 = ik.arm_ik.fk(j2)
        print(f"\n=== MuJoCoIK fast path ===")
        print(f"  fk(j)       = ({eef[0]:.4f}, {eef[1]:.4f}, {eef[2]:.4f})")
        print(f"  fk(ik(eef)) = ({eef2[0]:.4f}, {eef2[1]:.4f}, {eef2[2]:.4f})")
        print(f"  error       = {np.linalg.norm(eef2 - eef):.2e} m")
    except Exception as e:
        print(f"mujoco fast path FAILED: {e}")
