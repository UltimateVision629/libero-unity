"""Verify MuJoCo-backend joints place the EEF at the recorded obs positions.

The recorded robot*_eef_pos is the PHYSICAL MuJoCo position during collection
(= the block position at grasp, since the operator closed the loop).  If
MuJoCo FK of the replayed joints matches the recorded eef, the replay will
grasp.  This is the true end-to-end check the math-only validation can't do.
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))  # arm_ik.py, mujoco_ik.py
from arm_ik import create_ik, INIT_ARM_Q
from replay_action_via_ik import (
    _target_pose_to_world, _world_to_target_pose, _SO100_HOME_XYZ,
    _RIGHT_BASE, _LEFT_BASE, _obs_eef_to_arm, _obs_joints_to_arm,
)
from scipy.spatial.transform import Rotation as R

CASES = [
    (r"C:\vla\my\DatasetsCollector\demos\episode_0001_pick_up_the_red_block_20260728_220541_success.npz", 0),
    (r"C:\vla\my\DatasetsCollector\demos\episode_0001_pick_up_the_red_block_20260728_221549_success.npz", 0),
    (r"C:\vla\my\DatasetsCollector\demos\拿起蓝色方块\episode_0001_拿起蓝色方块_20260611_200524_success.npz", 1),
]

ik = create_ik("mujoco")

for path, arm in CASES:
    data = np.load(path, allow_pickle=True)
    act = data["action"]
    T = act.shape[0]
    obs_eef = data[f"robot{arm}_eef_pos"]
    obs_joints = data[f"robot{arm}_joint_pos"]
    base = _RIGHT_BASE if arm == 0 else _LEFT_BASE
    base_i = arm * 7

    # Does the arm even move?
    dist = np.sum(np.linalg.norm(np.diff(np.cumsum(act[:, base_i:base_i+3], 0), axis=0), axis=1))

    # Tracked warm-start
    home_tp = [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0]
    target_pose = home_tp.copy()
    wp_home, wq_home = _target_pose_to_world(home_tp)
    world_pos = wp_home.copy()
    world_quat = wq_home.copy()
    warm = INIT_ARM_Q.copy()

    # obs valid?
    obs_eef_valid = all(0.10 < np.linalg.norm(obs_eef[t] - base) < 0.65 for t in range(0, T, 20))
    obs_joints_valid = not np.all(np.abs(obs_joints) < 1e-6)

    max_err_chain = 0.0      # |FK(chain joints) - recorded eef| in arm frame
    max_err_fast = 0.0       # |FK(fast-path joints) - recorded eef|
    max_jump_fast = 0.0
    grasp_ok_frame = None

    for t in range(T):
        act_t = act[t]
        dpos = act_t[base_i:base_i+3].astype(float)
        drot = act_t[base_i+3:base_i+6].astype(float)
        grip = float(act_t[base_i+6])

        world_pos = world_pos + dpos
        if np.linalg.norm(drot) > 1e-10:
            qd = R.from_rotvec(drot).as_quat()
            world_quat = np.array([
                qd[0]*world_quat[3] + qd[1]*world_quat[2] - qd[2]*world_quat[1] + qd[3]*world_quat[0],
                qd[1]*world_quat[3] + qd[2]*world_quat[0] - qd[0]*world_quat[2] + qd[3]*world_quat[1],
                qd[2]*world_quat[3] + qd[0]*world_quat[1] - qd[1]*world_quat[0] + qd[3]*world_quat[2],
                qd[3]*world_quat[3] - qd[0]*world_quat[0] - qd[1]*world_quat[1] - qd[2]*world_quat[2],
            ])
        target_pose = _world_to_target_pose(world_pos, world_quat, prev_target_pose=target_pose)

        # Chain path
        j5, warm = ik.solve(target_pose.copy(), grip, warm)
        if j5 is not None:
            eef_chain = ik.arm_ik.fk(j5)
            eef_obs_arm = _obs_eef_to_arm(obs_eef[t], base)
            err = np.linalg.norm(eef_chain - eef_obs_arm)
            max_err_chain = max(max_err_chain, err)

        # Fast path (recorded eef + recorded joints warm-start)
        if obs_eef_valid and obs_joints_valid:
            eef_obs_arm = _obs_eef_to_arm(obs_eef[t], base)
            q_obs = _obs_joints_to_arm(obs_joints[t])
            j5f = ik.solve_eef_pos(eef_obs_arm, q_obs)
            eef_fast = ik.arm_ik.fk(j5f)
            max_err_fast = max(max_err_fast, np.linalg.norm(eef_fast - eef_obs_arm))

        # grasp frame: first grip close
        if grip < 0.5 and grasp_ok_frame is None:
            grasp_ok_frame = t

    name = path.split("\\")[-1][:45]
    print(f"\n=== {name}  arm={arm}  T={T}  moved={dist:.2f}m ===")
    print(f"  obs eef valid: {obs_eef_valid}   obs joints valid: {obs_joints_valid}")
    print(f"  CHAIN path: max |FK(joints) - recorded eef| = {max_err_chain*1000:.1f} mm")
    if obs_eef_valid and obs_joints_valid:
        print(f"  FAST path: max |FK(joints) - recorded eef| = {max_err_fast*1000:.1f} mm")
    if grasp_ok_frame is not None:
        print(f"  first close @ t={grasp_ok_frame}")
        if obs_eef_valid:
            eef_g = _obs_eef_to_arm(obs_eef[grasp_ok_frame], base)
            print(f"    recorded eef(arm frame) @ close: ({eef_g[0]:.4f}, {eef_g[1]:.4f}, {eef_g[2]:.4f})")
            print(f"    block position(arm frame):      (~0.40, ~0.02, ~0.11)  [red R / blue L]")
