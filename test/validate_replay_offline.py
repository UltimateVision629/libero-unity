"""
Offline validation of replay_action_via_ik.py — no TCP, no Unity.

Replays the recorded action deltas through the same virtual-Joy-Con +
lerobot_IK pipeline as replay_action_via_ik.py and reports diagnostics:

  - IK failure rate per arm
  - max per-frame joint jump (branch flips / discontinuity)
  - world-position reconstruction error:  world(t) rebuilt from the
    stored target_pose vs the world(t) accumulated from the action deltas.
    If the forward/inverse transforms are consistent this stays ~1e-6;
    tens of cm means the action semantics don't match the replay model
    (e.g. data from an older collector, or gimbal-lock mis-decomposition).
  - number of frames where the reconstructed target_pose exceeds CONTROL_GLIMIT
  - --roundtrip: random forward/inverse round-trip consistency self-test

Usage (run in the `lerobot-kin` conda env — fknm.pyd is cp311):
  python validate_replay_offline.py [--traj PATH] [--home-warmstart] [--roundtrip] [--debug N]

--home-warmstart: start IK from the home IK solution (as collect_datasets.py
  does in _go_home) instead of _INIT_ARM_Q (current replay behavior).
  Run both ways to compare.
"""
import argparse
import math
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))  # replay_action_via_ik.py

import replay_action_via_ik as replay_mod  # noqa: E402

from replay_action_via_ik import (  # noqa: E402
    CONTROL_GLIMIT,
    _SO100_HOME_XYZ,
    _target_pose_to_world,
    _world_to_target_pose,
    quat_multiply,
    INIT_ARM_Q,
)
from arm_ik import create_ik  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

DEFAULT_TRAJ = r"C:\vla\my\DatasetsCollector\demos\episode_0001_pick_up_the_red_block_20260728_220541_success.npz"


def patch_yimpl():
    """Plan A': carry the implied local y through the round-trip.

    _world_to_target_pose currently discards y_impl = -wx·sinψ + wy·cosψ
    (forces 0.0); _target_pose_to_world / _compute_robot_command then rebuild
    with hardcoded y_r=0.01.  Since Rot(ψ)·(x_r, y_impl) ≡ world for ANY ψ,
    carrying y_impl makes the IK position target exactly equal the tracked
    world position, removing the last position error source (yaw guess
    mismatch under gimbal lock).
    """
    orig_world_to_tp = replay_mod._world_to_target_pose

    def patched_world_to_tp(world_pos, world_quat, prev_target_pose=None):
        r = orig_world_to_tp(world_pos, world_quat,
                             prev_target_pose=prev_target_pose)
        # implied local y for the chosen yaw: y_impl = -wx·sinψ + wy·cosψ
        psi = r[5]
        r[1] = -float(world_pos[0]) * math.sin(psi) + float(world_pos[1]) * math.cos(psi)
        return r

    def patched_tp_to_world(target_pose):
        # y_r from target_pose[1] instead of hardcoded 0.01
        x_r, y_r, z_r, roll_r, pitch_r, yaw_r = target_pose
        pitch_t = -pitch_r
        roll_t = roll_r - math.pi / 2
        cos_y, sin_y = math.cos(yaw_r), math.sin(yaw_r)
        world_x = x_r * cos_y - y_r * sin_y
        world_y = x_r * sin_y + y_r * cos_y
        world_z = z_r
        q_wrist = replay_mod._euler_zxy_to_quat(-pitch_t, 0.0, -(roll_t + math.pi / 2))
        q_base = replay_mod._euler_zxy_to_quat(0.0, 0.0, yaw_r)
        world_quat = quat_multiply(q_base, q_wrist)
        return np.array([world_x, world_y, world_z]), world_quat

    replay_mod._world_to_target_pose = patched_world_to_tp
    replay_mod._target_pose_to_world = patched_tp_to_world


def patch_plan_b():
    """Plan B: replace the Z-X-Z decomposition with a well-conditioned one.

    The recorded trajectory is structurally Rz(yaw)·Rx(pitch)·Rz(-roll) with
    the local y ALWAYS = 0.01 (Joy-Con cylindrical frame).  So:

      yaw   — solved EXACTLY from the position constraint
              -wx·sinψ + wy·cosψ = 0.01   (2 candidates, continuity picks)
              no gimbal lock, no ill-conditioning, independent of quaternion.
      pitch/roll — extracted from Rz(-yaw)·world_quat = Rx(pitch)·Rz(-roll),
              a two-angle decomposition that is well-conditioned everywhere
              (unlike the three-angle Z-X-Z which degenerates at pitch≈0).

    Assumes replay data was recorded with collect_datasets.py (local y=0.01).
    """
    def patched(world_pos, world_quat, prev_target_pose=None):
        wx, wy, wz = (float(v) for v in world_pos)
        y_local = 0.01

        # ── yaw from position constraint ──
        # wy·cosψ - wx·sinψ = y_local
        #   = r·sin(θ-ψ),  r=hypot(wx,wy), θ=atan2(wy,wx)
        #   → ψ = θ - asin(y_local/r)  or  θ - π + asin(y_local/r)
        r = math.hypot(wx, wy)
        if r <= abs(y_local) + 1e-12:   # degenerate: near origin, use ZXZ
            return replay_mod._world_to_target_pose(world_pos, world_quat,
                                                    prev_target_pose)
        theta = math.atan2(wy, wx)
        delta = math.asin(min(1.0, y_local / r))
        cand1 = theta - delta
        cand2 = theta - math.pi + delta
        if prev_target_pose is None:
            yaw_r = cand1
        else:
            prev_yaw = prev_target_pose[5]
            yaw_r = min(cand1, cand2, key=lambda c: abs(
                ((c - prev_yaw + math.pi) % (2 * math.pi)) - math.pi))

        # ── pitch/roll from Rz(-yaw)·world_quat = Rx(pitch)·Rz(-roll) ──
        q_base_inv = replay_mod._euler_zxy_to_quat(0.0, 0.0, -yaw_r)
        qx, qy, qz, qw = quat_multiply(q_base_inv, world_quat)
        # q_M = [sin(p/2)cos(r/2), sin(p/2)sin(r/2), -cos(p/2)sin(r/2), cos(p/2)cos(r/2)]
        pitch_r = 2.0 * math.atan2(qx, qw)   # valid away from roll≈π
        roll_r = -2.0 * math.atan2(qz, qw)   # valid away from pitch≈π

        # Position back to cylindrical frame with the solved yaw
        cos_y, sin_y = math.cos(yaw_r), math.sin(yaw_r)
        x_r = wx * cos_y + wy * sin_y
        return [x_r, y_local, wz, roll_r, pitch_r, yaw_r]

    replay_mod._world_to_target_pose = patched


def patch_gimbal_threshold(threshold):
    """Replace _world_to_target_pose with a version whose gimbal-lock
    fallback triggers for |pitch| < `threshold` instead of 1e-6.

    At small pitch the Z-X-Z decomposition's yaw/roll split is numerically
    ill-conditioned; the fallback keeps yaw continuous and assigns the total
    Z rotation delta between yaw and roll (50/50).
    """
    orig = _world_to_target_pose

    def patched(world_pos, world_quat, prev_target_pose=None):
        world_x, world_y, world_z = world_pos
        euler = R.from_quat(world_quat).as_euler('ZXZ', degrees=False)
        yaw_candidate = float(euler[0])
        pitch_candidate = float(euler[1])
        roll_candidate = float(-euler[2])

        if prev_target_pose is None:
            return [world_x * math.cos(yaw_candidate) + world_y * math.sin(yaw_candidate),
                    0.0, world_z, roll_candidate, pitch_candidate, yaw_candidate]

        prev_roll, prev_pitch, prev_yaw = (prev_target_pose[3], prev_target_pose[4],
                                           prev_target_pose[5])

        # Normal-case branches: (yaw, pitch, roll) and (yaw+π, -pitch, roll+π)
        branches = [
            (yaw_candidate, pitch_candidate, roll_candidate),
            (yaw_candidate + math.pi, -pitch_candidate, roll_candidate + math.pi),
        ]
        best = None
        best_dist = float('inf')
        for y, p, r in branches:
            for dy in [0, 2 * math.pi, -2 * math.pi]:
                for dr in [0, 2 * math.pi, -2 * math.pi]:
                    yy, rr = y + dy, r + dr
                    dist = (abs(yy - prev_yaw) + abs(p - prev_pitch)
                            + abs(rr - prev_roll))
                    if dist < best_dist:
                        best_dist = dist
                        best = (yy, p, rr)

        yaw_r, pitch_r, roll_r = best

        # Gimbal-lock fallback (patched threshold): only total_z = yaw - roll
        # is well-defined. Split the delta 50/50 keeping yaw/roll continuous.
        if abs(pitch_r) < threshold and abs(prev_pitch) < threshold:
            total_z = yaw_r - roll_r
            prev_diff = prev_yaw - prev_roll
            delta = total_z - prev_diff
            delta_wrapped = (delta + math.pi) % (2 * math.pi) - math.pi
            yaw_r = prev_yaw + 0.5 * delta_wrapped
            roll_r = prev_roll - 0.5 * delta_wrapped

        cos_y, sin_y = math.cos(yaw_r), math.sin(yaw_r)
        x_r = world_x * cos_y + world_y * sin_y
        return [x_r, 0.0, world_z, roll_r, pitch_r, yaw_r]

    replay_mod._world_to_target_pose = patched
    return orig


def apply_delta(wp, wq, dpos, drot):
    """World-frame delta application — identical to replay main loop."""
    wp_new = wp + dpos
    if np.linalg.norm(drot) > 1e-10:
        qd = R.from_rotvec(drot).as_quat()  # scipy [x, y, z, w]
        wq_new = quat_multiply(qd, wq)
    else:
        wq_new = wq
    return wp_new, wq_new


def roundtrip_self_test(n_trials=50, seed=1):
    """Random forward (target_pose -> world) then inverse (world -> target_pose)."""
    rng = np.random.default_rng(seed)
    worst_pos, worst_rot = 0.0, 0.0
    for _ in range(n_trials):
        x = rng.uniform(0.15, 0.35)
        z = rng.uniform(0.05, 0.20)
        roll, pitch, yaw = rng.uniform(-1.2, 1.2, size=3)
        tp_in = [x, 0.0, z, roll, pitch, yaw]
        wp, wq = _target_pose_to_world(tp_in)
        tp_out = _world_to_target_pose(wp, wq, prev_target_pose=tp_in)
        wp2, wq2 = _target_pose_to_world(tp_out)
        worst_pos = max(worst_pos, np.linalg.norm(wp2 - wp))
        worst_rot = max(worst_rot, np.linalg.norm(wq2 - wq))
    print(f"[Roundtrip] {n_trials} random poses: worst pos err={worst_pos:.2e} m, "
          f"worst quat err={worst_rot:.2e}")
    return worst_pos


def simulate(actions, ik, warm_from_home, debug_frames=0, dump_tp=None,
             plan_a=True):
    """Replay simulation.

    plan_a=True  — FIXED main loop: world_pos/world_quat accumulate the action
                   deltas in float; target_pose derived from them only as IK
                   input (never rebuilds the world state).
    plan_a=False — legacy loop: world rebuilt from target_pose each frame
                   (pre-fix architecture).

    max_recon_err is measured with the SAME yardstick in both modes:
    | _target_pose_to_world(target_pose) - exact_accumulated_world |
    i.e. how far the IK target position (rebuilt with fixed y_r=0.01)
    drifts from the true accumulated trajectory.
    """
    T = actions.shape[0]
    home_tp = [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0]
    target_pose = [home_tp.copy(), home_tp.copy()]
    wp_home, wq_home = replay_mod._target_pose_to_world(home_tp)
    # plan_a: used as the world state; both modes: exact reference trajectory
    world_pos = [wp_home.copy(), wp_home.copy()]
    world_quat = [wq_home.copy(), wq_home.copy()]
    tp_rows = []  # [t, arm, x, z, roll, pitch, yaw, ik_ok]
    home_target = [_SO100_HOME_XYZ[0], 0.0, _SO100_HOME_XYZ[2], 0.0, 0.0, 0.0]
    warm = [INIT_ARM_Q.copy(), INIT_ARM_Q.copy()]
    if warm_from_home:
        for a in range(2):
            _, nq = ik[a].solve(home_target.copy(), 0.0, warm[a])
            warm[a] = nq

    ik_fail = [0, 0]
    max_dq = [0.0, 0.0]
    max_recon_err = [0.0, 0.0]
    over_limit = 0
    branch_jumps = 0
    prev_j = [None, None]
    first_ik_fail_t = [None, None]

    for t in range(T):
        act = actions[t]
        for a in range(2):
            base = a * 7
            dpos = act[base:base + 3].astype(float)
            drot = act[base + 3:base + 6].astype(float)
            grip = float(act[base + 6])

            if plan_a:
                # Step 1: accumulate delta on tracked world pose (float, no rebuild)
                world_pos[a] = world_pos[a] + dpos
                if np.linalg.norm(drot) > 1e-10:
                    qd = R.from_rotvec(drot).as_quat()
                    world_quat[a] = quat_multiply(qd, world_quat[a])
                # Step 2: world pose -> target_pose (IK input only)
                target_pose[a] = replay_mod._world_to_target_pose(
                    world_pos[a], world_quat[a], prev_target_pose=target_pose[a])
            else:
                # Legacy: world rebuilt from target_pose each frame
                wp, wq = replay_mod._target_pose_to_world(target_pose[a])
                wp_new, wq_new = apply_delta(wp, wq, dpos, drot)
                target_pose[a] = replay_mod._world_to_target_pose(
                    wp_new, wq_new, prev_target_pose=target_pose[a])

            # Same yardstick both modes: IK-target position vs exact trajectory
            wp2, _ = replay_mod._target_pose_to_world(target_pose[a])
            err = np.linalg.norm(wp2 - world_pos[a])
            max_recon_err[a] = max(max_recon_err[a], err)

            # GLIMIT violations on the stored target_pose (collect clamps before
            # computing labels; replay only clamps inside _compute_robot_command)
            for i in (0, 2, 3, 4, 5):
                if (target_pose[a][i] < CONTROL_GLIMIT[0][i] - 1e-9
                        or target_pose[a][i] > CONTROL_GLIMIT[1][i] + 1e-9):
                    over_limit += 1
                    break

            joints5, nq = ik[a].solve(target_pose[a].copy(), grip, warm[a])
            if joints5 is not None:
                if prev_j[a] is not None:
                    dq = float(np.max(np.abs(joints5 - prev_j[a])))
                    max_dq[a] = max(max_dq[a], dq)
                    if dq > 0.3:
                        branch_jumps += 1
                prev_j[a] = joints5.copy()
                warm[a] = nq
            else:
                ik_fail[a] += 1
                if first_ik_fail_t[a] is None:
                    first_ik_fail_t[a] = t

            if debug_frames and t < debug_frames:
                print(f"  t={t} arm={a} dpos={np.round(dpos,5)} drot={np.round(drot,5)} "
                      f"grip={grip}")
                print(f"      tp={[round(v,4) for v in target_pose[a]]} "
                      f"ik_ok={joints is not None}")
            if dump_tp:
                tp_rows.append([t, a, target_pose[a][0], target_pose[a][2],
                                target_pose[a][3], target_pose[a][4], target_pose[a][5],
                                joints is not None])

    if dump_tp:
        np.savetxt(dump_tp, np.array(tp_rows), fmt="%.6f",
                   header="t arm x z roll pitch yaw ik_ok")

    return dict(ik_fail=ik_fail, max_dq=max_dq, max_recon_err=max_recon_err,
                over_limit=over_limit, branch_jumps=branch_jumps,
                first_ik_fail_t=first_ik_fail_t, T=T)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traj", default=DEFAULT_TRAJ)
    ap.add_argument("--dir", default=None,
                    help="batch mode: validate ALL *.npz under this dir "
                         "(recursive), print a summary table")
    ap.add_argument("--ik-backend", default="mujoco",
                    choices=["lerobot", "mujoco", "placo"])
    ap.add_argument("--home-warmstart", action="store_true",
                    help="start IK from home IK solution (collect_datasets behavior)")
    ap.add_argument("--roundtrip", action="store_true", help="run round-trip self-test")
    ap.add_argument("--debug", type=int, default=0, help="print first N frames per arm")
    ap.add_argument("--dump-tp", default=None,
                    help="write per-frame target_pose CSV [t arm x z roll pitch yaw ik_ok]")
    ap.add_argument("--gimbal-threshold", type=float, default=None,
                    help="patch _world_to_target_pose with this gimbal-lock "
                         "pitch threshold (default: 1e-6 as in replay script)")
    ap.add_argument("--legacy", action="store_true",
                    help="use the legacy loop (world rebuilt from target_pose) "
                         "instead of Plan A float accumulation")
    ap.add_argument("--plan-a2", action="store_true",
                    help="additionally carry the implied local y (y_impl) "
                         "through target_pose so the IK position target "
                         "exactly equals the tracked world position")
    ap.add_argument("--plan-b", action="store_true",
                    help="replace Z-X-Z decomposition with position-constrained "
                         "yaw solve + two-angle pitch/roll extraction")
    args = ap.parse_args()

    if args.gimbal_threshold is not None:
        patch_gimbal_threshold(args.gimbal_threshold)
        print(f"[Patch] gimbal-lock threshold -> {args.gimbal_threshold}")
    if args.plan_a2:
        patch_yimpl()
        print("[Patch] y_impl carried through target_pose (Plan A2)")
    if args.plan_b:
        patch_plan_b()
        print("[Patch] Plan B: position-constrained yaw + 2-angle extraction")

    # ── Batch mode: validate every trajectory under a directory ──────────
    if args.dir:
        files = sorted(Path(args.dir).rglob("*.npz"))
        if not files:
            raise SystemExit(f"No .npz found under {args.dir}")
        ik = [create_ik(args.ik_backend) for _ in range(2)]
        print(f"[Batch] {len(files)} trajectories under {args.dir} "
              f"(ik-backend={ik[0].name})")
        hdr = (f"{'file':<52} {'T':>5} {'IK_fail':>8} {'pos_err(m)':>22} "
               f"{'over_lim':>9} {'grip_sw':>7}  verdict")
        print(hdr)
        print("-" * len(hdr))
        n_pass = 0
        for f in files:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                data = np.load(f, allow_pickle=True)
            actions = data["action"]
            T = actions.shape[0]
            # gripper switch count: closing action exists? (raw: 0=close, 1=open)
            grip_sw = int((np.abs(np.diff(actions[:, 6])) > 0.5).sum()
                          + (np.abs(np.diff(actions[:, 13])) > 0.5).sum())
            res = simulate(actions, ik, True, plan_a=not args.legacy)
            ik_ok = res["ik_fail"][0] == 0 and res["ik_fail"][1] == 0
            pos_ok = max(res["max_recon_err"]) < 1e-3
            jump_ok = res["branch_jumps"] == 0
            verdict = "PASS" if (ik_ok and pos_ok and jump_ok) else "FAIL"
            if verdict == "PASS":
                n_pass += 1
            err = f"{res['max_recon_err'][0]:.1e}/{res['max_recon_err'][1]:.1e}"
            print(f"{f.name:<52} {T:>5} {res['ik_fail'][0]}/{res['ik_fail'][1]:<5} "
                  f"{err:>22} {res['over_limit']:>5}/{T*2:<3} {grip_sw:>7}  {verdict}")
        print("-" * len(hdr))
        print(f"[Batch] PASS {n_pass}/{len(files)}")
        return

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        data = np.load(args.traj, allow_pickle=True)
    actions = data["action"]
    T = actions.shape[0]
    print(f"[Load] {Path(args.traj).name}  task={data.get('language_instruction','?')}  T={T}")
    print(f"       action: min={actions.min():.4f} max={actions.max():.4f} "
          f"shape={actions.shape}")
    print(f"       首帧: {actions[0]}")
    print(f"       |delta| 第0-4帧: {np.abs(actions[:5, :6]).max(axis=1)}")

    if args.roundtrip:
        worst = roundtrip_self_test()
        if worst > 1e-3:
            print("  !! Round-trip inconsistency — forward/inverse transforms disagree")

    ik = [create_ik(args.ik_backend) for _ in range(2)]
    print(f"[IK] backend={ik[0].name}")
    tag = "home解起步" if args.home_warmstart else "_INIT_ARM_Q起步"
    res = simulate(actions, ik, args.home_warmstart, debug_frames=args.debug,
                   dump_tp=args.dump_tp, plan_a=not args.legacy)

    print(f"\n=== 模拟回放结果 ({tag}) ===")
    print(f"  IK失败:        R={res['ik_fail'][0]}/{res['T']}  "
          f"L={res['ik_fail'][1]}/{res['T']}")
    if res['first_ik_fail_t'][0] is not None:
        print(f"  R首次IK失败于 t={res['first_ik_fail_t'][0]}  "
              f"L首次IK失败于 t={res['first_ik_fail_t'][1]}")
    print(f"  最大关节帧间跳跃: R={res['max_dq'][0]:.4f}  L={res['max_dq'][1]:.4f} rad"
          f"  (>0.3rad: {res['branch_jumps']}次)")
    print(f"  世界位置重建最大误差: R={res['max_recon_err'][0]:.2e}  "
          f"L={res['max_recon_err'][1]:.2e} m")
    print(f"  target_pose超GLIMIT帧数: {res['over_limit']}/{res['T']*2}")


if __name__ == "__main__":
    main()
