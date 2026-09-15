"""
Benchmark mujoco IK backend per-solve cost — where do the ms go? (v3)

Times the REAL code path per layer on a synthetic teleop trajectory that
mixes constant-velocity pushes, slow wrist waves, and sudden jumps (go-home /
hard stick pushes).  least_squares is probed at MODULE level (so100_chain and
mujoco_ik bind it via `from scipy.optimize import least_squares`, so the
scipy.optimize attribute itself must NOT be patched).

Findings so far (v2): tol 1e-3 vs 1e-8 barely matters (nfev is already 3-5);
the per-solve cost is dominated by fixed per-iteration overhead, and the
so100 translation layer (~1.37ms) is the single largest item — paid by BOTH
backends, with ftol HARDCODED 1e-8 in so100_chain.py.

Usage: python benchmark_ik_perf.py
"""
import os
import sys
import time

import numpy as np

_TEST = os.path.dirname(os.path.abspath(__file__))
# utils/ 就在本目录下（2026-09-13 从 network/scripts/utils 搬来）——
# 这里原先指向 ../../network/scripts，那条跨仓库路径已废弃。
for p in (_TEST,):
    if p not in sys.path:
        sys.path.insert(0, p)

from utils import so100_chain  # noqa: E402
import mujoco_ik  # noqa: E402
from arm_ik import create_ik, INIT_ARM_Q, _clamp_target_pose, _target_gpos  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

# ── Probe least_squares at module level (where callers look it up) ─────────
_ls_log: list[tuple[float, int, int]] = []


def _make_probe():
    def probe(*a, **k):
        t0 = time.perf_counter()
        res = probe._orig(*a, **k)
        _ls_log.append((time.perf_counter() - t0, res.nfev, len(res.x)))
        return res
    return probe


for _mod in (so100_chain, mujoco_ik):
    p = _make_probe()
    p._orig = _mod.least_squares
    _mod.least_squares = p


def _ls_summary(tag: str):
    if not _ls_log:
        print(f"  {tag}: (no least_squares calls)")
        return
    for xlen in sorted({e[2] for e in _ls_log}):
        sub = [e for e in _ls_log if e[2] == xlen]
        ts = np.array([e[0] for e in sub]) * 1e3
        nf = np.array([e[1] for e in sub])
        name = "so100 translation (4-DOF)" if xlen == 4 else f"arm_ik.ik ({xlen}-DOF)"
        # per-iteration overhead: (t_ms / nfev) tells fixed cost per iteration
        per_it = (ts / np.maximum(nf, 1)).mean()
        print(f"  {tag} | {name}: calls={len(ts)}  mean={ts.mean():.3f}ms "
              f"p90={np.percentile(ts,90):.3f}ms  max={ts.max():.3f}ms  "
              f"nfev mean={nf.mean():.1f} p90={np.percentile(nf,90):.0f}  "
              f"~{per_it * 1e3:.0f}us/iteration")
    _ls_log.clear()


# ── Synthetic teleop trajectory ────────────────────────────────────────────

def make_trajectory(n=720, seed=1):
    tp = []
    x, z = 0.111, 0.098
    jump = np.zeros(n, dtype=bool)
    for i in range(n):
        # every 150 frames: hard jump like go-home / fast stick flick
        if i % 150 == 20:
            x = 0.30
            z = 0.055 + 0.05 * ((i // 150) % 2)
            jump[i] = True
        elif i % 150 < 20:
            pass  # hold at jump
        else:
            x += 0.001 if i % 300 < 150 else -0.001
            z += -0.0004 if i % 240 < 120 else 0.0004
        x = min(0.30, max(0.10, x))
        z = min(0.20, max(0.055, z))
        t = i / n
        roll = 0.4 * np.sin(2 * np.pi * t * 2.0) + (1.2 if i % 150 in range(20, 24) else 0.0)
        pitch = 0.3 * np.sin(2 * np.pi * t * 3.0)
        yaw = 0.25 * np.sin(2 * np.pi * t * 1.5)
        tp.append([x, 0.0, z, roll, pitch, yaw])
    return tp, jump


def run_solve_config(tag, backend, tol, traj, jump_mask):
    ik = create_ik(backend)
    current_q = INIT_ARM_Q.copy()
    per_frame, fails, smooth_t, jump_t = [], 0, [], []
    nfev_per_frame = []
    for i, tp in enumerate(traj):
        mark = len(_ls_log)
        t0 = time.perf_counter()
        joints, new_q = ik.solve(tp, 1.0, current_q) if tol is None else \
            ik.solve(tp, 1.0, current_q, tol=tol)
        dt = time.perf_counter() - t0
        nfev_per_frame.append(sum(e[1] for e in _ls_log[mark:]))
        per_frame.append(dt)
        (jump_t if jump_mask[i] else smooth_t).append(dt)
        if joints is None:
            fails += 1
        else:
            current_q = new_q
    ts = np.array(per_frame) * 1e3
    js = np.array(jump_t) * 1e3
    ss = np.array(smooth_t) * 1e3
    print(f"== {tag}: mean={ts.mean():.3f}ms  p50={np.median(ts):.3f}ms  "
          f"p90={np.percentile(ts,90):.3f}ms  max={ts.max():.3f}ms  fails={fails}")
    print(f"   smooth frames (n={len(ss)}): mean={ss.mean():.3f}ms p90={np.percentile(ss,90):.3f}ms  "
          f"| jump frames (n={len(js)}): mean={js.mean():.3f}ms max={js.max():.3f}ms")
    # slowest frames: show their total nfev (both LS layers combined)
    top = np.argsort(per_frame)[-3:][::-1]
    for idx in top:
        print(f"   slowest frame #{idx}: {ts[idx]:.2f}ms  total nfev={nfev_per_frame[idx]}")
    _ls_summary(tag)
    return ts


# ── Proposed-fix experiments ───────────────────────────────────────────────

def so100_ftol_sweep(frames=400, seed=3):
    """Translation layer alone: does loosening ftol save time? (nfev is ~4 —
    expect little change; proves the fix must cut per-iteration cost, not tol)."""
    rng = np.random.default_rng(seed)
    qs = np.zeros((frames, 4))
    qs[0] = np.array([-0.5, 1.2, -0.3, 0.2])
    for i in range(1, frames):
        qs[i] = qs[i - 1] + 0.002 * np.sin(i / 40.0) * rng.uniform(-1, 1, 4)
        qs[i, :2] += 0.0008
    targets = []
    for q in qs:
        T = so100_chain.so100_fk(q)
        targets.append(np.concatenate([T[:3, 3],
                                       R.from_matrix(T[:3, :3]).as_euler("xyz")]))
    # ground-truth displacement per frame (q-space step magnitude)
    step = np.linalg.norm(np.diff(qs, axis=0), axis=1).mean()
    for ftol in (1e-8, 1e-4, 1e-2):
        errs, per_frame, warm, n_fail = [], [], qs[0].copy(), 0
        for tg in targets[:frames]:
            t0 = time.perf_counter()
            q, ok = so100_chain.so100_ik(warm, tg, ftol=ftol)
            per_frame.append(time.perf_counter() - t0)
            if not ok:
                n_fail += 1
                continue
            warm = q
            T = so100_chain.so100_fk(q)
            errs.append(np.linalg.norm(T[:3, 3] - tg[:3]))
        ts = np.array(per_frame) * 1e3
        print(f"  so100_ik ftol={ftol}: mean={ts.mean():.3f}ms  "
              f"eef err mean={np.mean(errs) * 1e3:.3f}mm max={np.max(errs) * 1e3:.3f}mm "
              f"fails={n_fail}  (q step/frame ~{step * 1e3:.2f} mrad)")


def mujoco_maxnfev_sweep(traj, max_nfevs=(40, 12, 6), tol=1e-3, frames=300):
    """MuJoCo arm_ik.ik with smaller max_nfev: time + EEF residual."""
    arm = mujoco_ik.MuJoCoArmIK()
    ik0 = create_ik("mujoco")
    current_q = INIT_ARM_Q.copy()
    warm_lerobot = None
    eef_targets = []
    for tp in traj[:frames]:
        tp = _clamp_target_pose(tp)
        yaw_r = tp[5]
        tg = _target_gpos(tp)
        if warm_lerobot is None:
            warm_lerobot = current_q[1:5].copy()
        qpos_inv, ok = so100_chain.so100_ik(warm_lerobot, tg, tol=1e-3)
        warm_lerobot = qpos_inv[:4].copy()
        eef_targets.append(arm.fk(np.concatenate(([yaw_r], qpos_inv[:4]))))
        j, _ = ik0.solve(tp, 1.0, current_q, tol=tol)
        if j is not None:
            current_q = j
    for mnf in max_nfevs:
        times, resid, warm = [], [], INIT_ARM_Q.copy()
        for et in eef_targets:
            t0 = time.perf_counter()
            q = arm.ik(warm, et, max_nfev=mnf, tol=tol,
                       wrist_target=np.asarray(warm[3:5]))
            times.append(time.perf_counter() - t0)
            warm = q
            resid.append(np.linalg.norm(arm.fk(q) - et))
        ts = np.array(times) * 1e3
        print(f"  arm_ik.ik max_nfev={mnf}: mean={ts.mean():.3f}ms  "
              f"eef resid mean={np.mean(resid) * 1e3:.3f}mm "
              f"p90={np.percentile(resid, 90) * 1e3:.3f}mm  max={np.max(resid) * 1e3:.3f}mm")


def main():
    traj, jump_mask = make_trajectory()
    n = len(traj)
    print(f"Synthetic teleop trajectory: {n} frames "
          f"(constant-velocity pushes + wrist waves + hard jumps every 150 frames)\n")

    run_solve_config("lerobot backend (baseline)", "lerobot", None, traj, jump_mask)
    print()
    run_solve_config("mujoco backend tol=1e-3 (collect --ik-tol default)", "mujoco", 1e-3, traj, jump_mask)
    print()
    run_solve_config("mujoco backend tol=1e-8 (replay/inference default)", "mujoco", 1e-8, traj, jump_mask)

    print("\n-- fix 1 check: loosen translation-layer ftol (expect ~no change; "
          "nfev already ~4) --")
    so100_ftol_sweep()

    print("\n-- fix 2 check: arm_ik.ik smaller max_nfev cap --")
    mujoco_maxnfev_sweep(traj)

    print("\n60Hz frame budget: 16.7ms.  Collect loop also does: fixed 10ms sleep,")
    print("2x get_control(), 2x label math, TCP send + get_obs (image base64).")
    print("Per-arm cost x2 arms = the IK block; the 10ms sleep + get_obs is common")


if __name__ == "__main__":
    main()
