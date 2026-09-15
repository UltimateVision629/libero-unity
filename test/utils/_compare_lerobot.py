r"""Compare so100_chain FK/IK against the original lerobot_kinematics model.

要求 lerobot_kinematics 可导入（在 lerobot-kin conda 环境里跑）。
用法（本文件在 libero-unity/test/utils/，2026-09-13 从 network/scripts/utils 搬来）：
    conda activate lerobot-kin
    python libero-unity\test\utils\_compare_lerobot.py
"""
import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.path.insert(0, r"C:\vla\lerobot-kinematics")
# 本文件在 utils/ 包**内部**，所以要把**父目录**（libero-unity/test）加进 sys.path，
# 下面那句 `from utils import so100_chain` 才解析得到这个包。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils import so100_chain
from lerobot_kinematics import lerobot_IK, get_robot

robot = get_robot("so100")

# ── 1. FK equivalence ─────────────────────────────────────────────────────
rng = np.random.default_rng(42)
errs = []
for _ in range(50):
    q = rng.uniform(robot.qlim[0], robot.qlim[1])
    T_mine = so100_chain.so100_fk(q)
    T_ref = robot.fkine(q).A  # numpy 4x4 from ETS FK
    errs.append(np.max(np.abs(T_mine - T_ref)))
print(f"FK max abs diff (50 random q): {max(errs):.2e}  "
      f"(expect ~0 — identical chain params)")

# ── 2. IK equivalence: same target_pose + same warm → same joints ────────
ik_joint_errs, ik_pose_errs, n_ok = [], [], 0
for _ in range(50):
    q_ref = rng.uniform(robot.qlim[0], robot.qlim[1])
    T = so100_chain.so100_fk(q_ref)
    # Full pose from FK (position + rotation) — reachable by construction
    pose = np.concatenate([T[:3, 3], R.from_matrix(T[:3, :3]).as_euler("xyz")])
    warm = q_ref.copy()

    q_orig, ok_orig = lerobot_IK(warm.copy(), pose, robot)
    q_new, ok_new = so100_chain.so100_ik(warm.copy(), pose)

    if ok_orig and ok_new:
        n_ok += 1
        ik_joint_errs.append(np.max(np.abs(q_orig - q_new)))
        T_o = so100_chain.so100_fk(q_orig)
        T_n = so100_chain.so100_fk(q_new)
        ik_pose_errs.append(np.max(np.abs(T_o - T_n)))
    else:
        print(f"  target {_}: ok_orig={ok_orig} ok_new={ok_new}")
print(f"IK both-ok targets: {n_ok}/50")
if ik_joint_errs:
    print(f"IK joint diff (max, {n_ok} targets): {max(ik_joint_errs):.4f} rad")
    print(f"IK pose diff via own FK (max):      {max(ik_pose_errs):.2e}")
