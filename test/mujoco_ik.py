"""
MuJoCo-based FK/IK for SO100 arm — uses the REAL MuJoCo model loaded directly.

Eliminates the lerobot_IK → MuJoCo FK mismatch by computing IK against the
same model that runs in Unity.  The arm kinematic tree (body offsets, joint
axes) in so_arm100.xml is identical to libero_pick_up_red_block.xml.

Uses scipy.optimize.least_squares for IK (5-DOF arm).  The chain path adds
a wrist residual (Wrist_Pitch/Roll targets from the so100 translation) so the
wrist follows the operator's orientation; the fast path stays position-only.
"""
from __future__ import annotations

import math
import os
import numpy as np
import mujoco
from scipy.optimize import least_squares


# Default arm model: project-local copy in network/scripts/utils/so_100.xml
# (was C:\vla\lerobot-kinematics\examples\so_100.xml — external dependency).
def _default_xml_path() -> str:
    local = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "network", "scripts",
        "utils", "so_100.xml"))
    if os.path.exists(local):
        return local
    return r"C:\vla\lerobot-kinematics\examples\so_100.xml"  # legacy fallback


# ═══════════════════════════════════════════════════════════════════════════
# MuJoCo Arm IK
# ═══════════════════════════════════════════════════════════════════════════

class MuJoCoArmIK:
    """Numerical IK on the MuJoCo arm model (so_arm100.xml).

    Joint order (5-DOF arm, no gripper):
        [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll]  (all radians)

    Target frame: the Base body is at the origin.  The EEF target is the
    Fixed_Jaw body position offset by the eef_site local offset [0.01, -0.09, 0].
    """

    # Joint names in MuJoCo model (no L_/R_ prefix — universal arm model)
    ARM_JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]
    EEF_OFFSET = np.array([0.01, -0.09, 0.0], dtype=np.float64)  # in Fixed_Jaw frame

    def __init__(self, xml_path: str = None):
        xml_path = xml_path or _default_xml_path()
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Cache joint qpos addresses
        self._jnt_qposadr = []
        for name in self.ARM_JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in MuJoCo model")
            self._jnt_qposadr.append(self.model.jnt_qposadr[jid])

        # Cache Fixed_Jaw body ID for EEF computation
        self._jaw_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Fixed_Jaw")
        if self._jaw_body_id < 0:
            raise ValueError("Fixed_Jaw body not found")

        # Joint limits from model
        self._jnt_limits = []
        for name in self.ARM_JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self._jnt_limits.append(self.model.jnt_range[jid])

    # ── Forward Kinematics ────────────────────────────────────────────────

    def fk(self, joints_5dof: np.ndarray) -> np.ndarray:
        """Compute EEF position in arm base frame (MuJoCo Base body at origin).

        Args:
            joints_5dof: [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll] (rad)

        Returns:
            EEF position [x, y, z] in base frame (metres)
        """
        joints = np.asarray(joints_5dof, dtype=np.float64)
        for i, adr in enumerate(self._jnt_qposadr):
            self.data.qpos[adr] = float(joints[i])
        mujoco.mj_kinematics(self.model, self.data)

        # Fixed_Jaw body position + eef_site offset
        jaw_pos = self.data.xpos[self._jaw_body_id]
        # The Fixed_Jaw body orientation gives us the local frame rotation
        jaw_rot = self.data.xmat[self._jaw_body_id].reshape(3, 3)
        eef_pos = jaw_pos + jaw_rot @ self.EEF_OFFSET
        return np.array(eef_pos, dtype=np.float64)

    # ── Inverse Kinematics ────────────────────────────────────────────────

    def ik(
        self,
        current_joints_rad: np.ndarray,
        target_eef_pos: np.ndarray,
        max_nfev: int = 40,
        reg: float = 0.3,
        tol: float = 1e-8,
        wrist_target: Optional[np.ndarray] = None,
        wrist_w: float = 0.5,
    ) -> np.ndarray:
        """IK: find arm joints to reach target EEF position (and optionally wrist).

        The 5-DOF arm vs 3-constraint problem has a 2-D null space; a small
        Tikhonov-style regularization term `reg * (q - q0)` keeps consecutive
        solves on the same branch (no wrist/elbow flips between frames) while
        the position error dominates.  Unity's rate limiter (0.3 rad/step)
        cannot follow branch jumps, so smoothness matters as much as accuracy.

        腕部残差（2026-09-01）：纯位置求解会把操作员的 pitch 意图丢弃——
        EEF 位置几乎不随腕部上翘变化，空余自由度被正则锚在 warm-start
        （手腕不跟手）。传入 wrist_target 后 5 DOF 满约束：位置 + 腕角同时解。

        Args:
            current_joints_rad: initial guess [5,] (radians)
            target_eef_pos: desired EEF position [3,] in base frame (metres)
            max_nfev: max FK evaluations per solve
            reg: regularization weight on (q - q0) deviation
            wrist_target: [Wrist_Pitch, Wrist_Roll] 目标（弧度）；None = 纯位置
                （fast path / 回放）
            wrist_w: 腕部残差权重。位置残差单位是米（~0.1），角度是弧度
                （~1-3），0.5 让腕角误差 ~0.1 rad 与 ~5mm 位置误差同级

        Returns:
            Joint angles [5,] (radians) that achieve the target position
        """
        q0 = np.asarray(current_joints_rad, dtype=np.float64).copy()
        target = np.asarray(target_eef_pos, dtype=np.float64)

        # Build joint bounds
        bounds = []
        for lo_hi in self._jnt_limits:
            bounds.append((float(lo_hi[0]), float(lo_hi[1])))

        def cost(q):
            pos = self.fk(q)
            res = [target - pos, reg * (q - q0)]
            if wrist_target is not None:
                res.append(wrist_w * (q[3:5] - wrist_target))
            return np.concatenate(res)

        result = least_squares(
            cost,
            q0,
            bounds=list(zip(*bounds)),
            method='trf',
            max_nfev=max_nfev,
            ftol=tol,
            xtol=tol,
        )
        return result.x


# ═══════════════════════════════════════════════════════════════════════════
# Singleton (created once, cached for the process lifetime)
# ═══════════════════════════════════════════════════════════════════════════

_arm_ik: MuJoCoArmIK | None = None


def get_arm_ik() -> MuJoCoArmIK:
    global _arm_ik
    if _arm_ik is None:
        _arm_ik = MuJoCoArmIK()
    return _arm_ik


# ═══════════════════════════════════════════════════════════════════════════
# Quick test
# ═══════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    arm = MuJoCoArmIK()
    # Test FK at home keyframe qpos from XML
    q_home = np.array([0.0, -1.57079, 1.57079, 0.0, 0.0])
    pos_home = arm.fk(q_home)
    print(f"FK at home keyframe: ({pos_home[0]:.4f}, {pos_home[1]:.4f}, {pos_home[2]:.4f})")

    # Test IK round-trip
    target = np.array([0.15, -0.10, 0.08])
    q = np.array([0.0, -1.0, 1.5, 0.0, -1.5])
    q_ik = arm.ik(q, target)
    pos_ik = arm.fk(q_ik)
    print(f"IK target:  ({target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f})")
    print(f"IK result:  ({pos_ik[0]:.4f}, {pos_ik[1]:.4f}, {pos_ik[2]:.4f})")
    print(f"Error: {np.linalg.norm(pos_ik - target):.4e} m")
    print(f"Joints: [{q_ik[0]:.4f}, {q_ik[1]:.4f}, {q_ik[2]:.4f}, {q_ik[3]:.4f}, {q_ik[4]:.4f}]")

    # Test with observed joints from Unity
    q_obs = np.array([-0.19, -0.71, 1.15, -0.43, -1.77])
    pos_obs = arm.fk(q_obs)
    print(f"\nFK of observed joints: ({pos_obs[0]:.4f}, {pos_obs[1]:.4f}, {pos_obs[2]:.4f})")
    print(f"  (from July28 t=284, rotation/pitch/elbow/wrist_pitch/wrist_roll)")
