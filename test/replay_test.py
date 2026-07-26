"""
Replay recorded trajectory actions through Unity's HandleStep to verify
the Jacobian IK (ApplyEefDelta) can correctly execute block grasping.

Usage:
  python replay_test.py                          # replay all trajectories
  python replay_test.py --trajectory 000000      # replay a specific one
  python replay_test.py --list                   # list available trajectories
"""

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

import numpy as np


# ── TCP client (same protocol as InferenceClient) ──────────────────────

class StepClient:
    """Minimal TCP client that sends step commands to TrainingServer :5556."""

    def __init__(self, host="127.0.0.1", port=5556):
        self.host = host
        self.port = port
        self._sock = None
        self._buf = b""

    def connect(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[Replay] Connecting to {self.host}:{self.port} ...")
        while True:
            try:
                self._sock.connect((self.host, self.port))
                break
            except (ConnectionRefusedError, OSError):
                print("  Waiting for Unity ...")
                time.sleep(1.0)
        print("  Connected.")

    def close(self):
        if self._sock:
            self._sock.close()

    def _send(self, msg: dict) -> dict:
        data = (json.dumps(msg) + "\n").encode("utf-8")
        self._sock.sendall(data)
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("Unity disconnected")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))

    def reset(self) -> dict:
        return self._send({"cmd": "reset"})

    def step(self, action: list) -> dict:
        return self._send({"cmd": "step", "action": action})

    def get_obs(self) -> dict:
        return self._send({"cmd": "get_obs"})


# ── Success detection ──────────────────────────────────────────────────

def check_grasp_success(initial_obs: dict, final_obs: dict, verbose: bool = True) -> dict:
    """
    Heuristic grasp success check:
      1. Gripper closed: fingers nearly touching (grasping object)
      2. EEF lifted: end-effector Z significantly higher than initial
      3. Optional: block lifted (if object_positions available)

    Returns dict with check results.
    """
    result = {"success": False, "reasons": []}

    # Gripper check — robot0 (right arm) gripper
    grip = np.array(final_obs.get("robot0_gripper_qpos", [0, 0]), dtype=float)
    grip_closed = abs(grip[0] - grip[1]) < 0.05  # fingers nearly touching
    if verbose:
        print(f"  Gripper: {grip} closed={grip_closed}")
    result["gripper_closed"] = grip_closed
    if grip_closed:
        result["reasons"].append("gripper_closed")
    else:
        result["reasons"].append("gripper_open")

    # EEF lift check
    init_eef = np.array(initial_obs.get("robot0_eef_pos", [0, 0, 0]), dtype=float)
    final_eef = np.array(final_obs.get("robot0_eef_pos", [0, 0, 0]), dtype=float)
    eef_dz = final_eef[2] - init_eef[2]
    eef_lifted = eef_dz > 0.03  # lifted 3cm
    if verbose:
        print(f"  EEF initial Z: {init_eef[2]:.4f}  final Z: {final_eef[2]:.4f}  dz: {eef_dz:.4f}  lifted={eef_lifted}")
    result["eef_lifted"] = eef_lifted
    if eef_lifted:
        result["reasons"].append("eef_lifted")
    else:
        result["reasons"].append("eef_not_lifted")

    # Block lift check (if object_positions is available in observation)
    block_lifted = False
    if "object_positions" in final_obs and "object_positions" in initial_obs:
        obj_final = final_obs.get("object_positions", {})
        obj_init = initial_obs.get("object_positions", {})
        for name in obj_final:
            if name in obj_init:
                dz = obj_final[name][2] - obj_init[name][2]
                if verbose:
                    print(f"  {name}: init_z={obj_init[name][2]:.4f} final_z={obj_final[name][2]:.4f} dz={dz:.4f}")
                if dz > 0.03:
                    block_lifted = True
        result["block_lifted"] = block_lifted
        if block_lifted:
            result["reasons"].append("block_lifted")

    # Overall success: gripper closed AND (eef_lifted OR block_lifted)
    result["success"] = grip_closed and (eef_lifted or block_lifted)

    return result


# ── Replay ─────────────────────────────────────────────────────────────

def replay_trajectory(client: StepClient, traj_path: str, delay: float = 0.02,
                      verbose: bool = True) -> dict:
    """Replay one trajectory and return success check result."""
    print(f"\n{'='*70}")
    print(f"[Replay] {Path(traj_path).name}")
    print(f"{'='*70}")

    data = np.load(traj_path, allow_pickle=True)
    T = int(data.get("length", 0))
    if T == 0:
        # Infer T from action keys
        t = 0
        while f"action/{t}" in data:
            t += 1
        T = t
    instruction = str(data.get("language_instruction", "unknown"))
    print(f"  Task: {instruction}")
    print(f"  Frames: {T}")

    # Reset
    resp = client.reset()
    if "error" in resp:
        print(f"  Reset error: {resp['error']}")
        return {"success": False, "error": resp["error"]}
    time.sleep(0.5)

    # Get initial observation
    initial_obs = client.get_obs()
    if verbose:
        print(f"  Initial EEF Z: {initial_obs.get('robot0_eef_pos', [0,0,0])[2]:.4f}")
        if "object_positions" in initial_obs:
            print(f"  Initial objects: {initial_obs['object_positions']}")

    # Replay actions
    print(f"  Replaying {T} actions...")
    t_start = time.time()
    for t in range(T):
        action_key = f"action/{t}"
        if action_key not in data:
            print(f"  Missing {action_key} at t={t}, stopping")
            break
        action = data[action_key].astype(np.float64).tolist()
        resp = client.step(action)

        # Debug first 3 steps: print action and response EEF
        if t < 3 or t >= T - 3:
            a = np.array(action)
            r_dx, r_dy, r_dz = a[0], a[1], a[2]
            l_dx, l_dy, l_dz = a[7], a[8], a[9]
            r_eef = resp.get("obs", {}).get("robot0_eef_pos", [0,0,0])
            l_eef = resp.get("obs", {}).get("robot1_eef_pos", [0,0,0])
            step_n = resp.get("step", -1)
            marker = "<<< LAST" if t == T - 1 else ""
            print(f"  [step {t}] server_step={step_n} "
                  f"R_delta=({r_dx:.6f},{r_dy:.6f},{r_dz:.6f}) R_grip={a[6]:.3f} "
                  f"L_delta=({l_dx:.6f},{l_dy:.6f},{l_dz:.6f}) L_grip={a[13]:.3f} {marker}")
            print(f"           R_eef=({r_eef[0]:.4f},{r_eef[1]:.4f},{r_eef[2]:.4f}) "
                  f"L_eef=({l_eef[0]:.4f},{l_eef[1]:.4f},{l_eef[2]:.4f})")
            if "object_positions" in resp.get("obs", {}):
                print(f"           objects={resp['obs']['object_positions']}")

        if "error" in resp:
            print(f"  Step {t} error: {resp['error']}")
            break

        if verbose and t % 50 == 0:
            # Print mid-replay EEF for drift check
            mid_eef = resp.get("obs", {}).get("robot0_eef_pos", [0,0,0])
            print(f"    t={t}/{T}  R_eef_z={mid_eef[2]:.4f}")

        if delay > 0:
            time.sleep(delay)

    elapsed = time.time() - t_start
    print(f"  Done in {elapsed:.1f}s ({T/elapsed:.1f} steps/s)")

    # Get final observation
    final_obs = client.get_obs()

    # Check success
    result = check_grasp_success(initial_obs, final_obs, verbose=True)
    result["trajectory"] = Path(traj_path).name
    result["task"] = instruction
    result["frames"] = T
    return result


def list_trajectories(traj_dir: str):
    """Print available trajectory files."""
    traj_dir = Path(traj_dir)
    files = sorted(traj_dir.glob("*.npz"))
    print(f"Trajectories in {traj_dir}:")
    for f in files:
        data = np.load(f, allow_pickle=True)
        T = int(data.get("length", len([k for k in data.keys() if k.startswith("action/")])))
        task = str(data.get("language_instruction", "?"))
        print(f"  {f.name}: {T} frames  task={task}")


# ── Main ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Replay recorded actions through Unity HandleStep")
    parser.add_argument("--trajectory", type=str, default=None,
                        help="Specific trajectory file (e.g. trajectory_000000.npz)")
    parser.add_argument("--traj_dir", type=str,
                        default=os.path.join(os.path.dirname(__file__), "..", "..", "datasets", "trajectories"),
                        help="Path to trajectory .npz files")
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--delay", type=float, default=0.02,
                        help="Delay between steps (s). 0 = as fast as possible.")
    parser.add_argument("--list", action="store_true", default=False,
                        help="List available trajectories")
    parser.add_argument("--quiet", action="store_true", default=False,
                        help="Minimal output")
    parser.add_argument("--warmup", type=float, default=3.0,
                        help="Warmup delay before replay (seconds). 0=skip.")
    args = parser.parse_args()

    traj_dir = Path(args.traj_dir)
    if not traj_dir.exists():
        print(f"Error: trajectory directory not found: {traj_dir}")
        sys.exit(1)

    if args.list:
        list_trajectories(str(traj_dir))
        return

    # Determine which trajectories to replay
    if args.trajectory:
        traj_path = traj_dir / args.trajectory
        if not traj_path.exists():
            # Try with .npz suffix
            traj_path = traj_dir / f"{args.trajectory}.npz"
        if not traj_path.exists():
            print(f"Error: trajectory not found: {args.trajectory}")
            sys.exit(1)
        traj_files = [str(traj_path)]
    else:
        traj_files = sorted(str(f) for f in traj_dir.glob("*.npz"))

    if not traj_files:
        print(f"No .npz files found in {traj_dir}")
        sys.exit(1)

    print(f"[Replay] {len(traj_files)} trajectory(s) to replay")
    print(f"[Replay] Server: {args.host}:{args.port}")
    print(f"[Replay] Delay: {args.delay}s per step")

    if args.warmup > 0:
        print(f"\n  ⏳ Warming up {args.warmup:.0f}s — switch to Unity window now!")
        for i in range(int(args.warmup), 0, -1):
            print(f"  {i}...")
            time.sleep(1)
        print("  Starting!\n")

    # Connect
    client = StepClient(args.host, args.port)
    try:
        client.connect()
    except Exception as e:
        print(f"Failed to connect to Unity: {e}")
        print("Make sure Unity is in Play mode with the MuJoCo scene loaded.")
        sys.exit(1)

    # Replay all
    results = []
    for traj_path in traj_files:
        try:
            result = replay_trajectory(client, traj_path, delay=args.delay,
                                       verbose=not args.quiet)
            results.append(result)
        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
            results.append({"trajectory": Path(traj_path).name, "success": False, "error": str(e)})

    # Summary
    print(f"\n{'='*70}")
    print("SUMMARY")
    print(f"{'='*70}")
    n_success = sum(1 for r in results if r.get("success"))
    print(f"{'Trajectory':<35s} {'Frames':>6s}  {'Success':>8s}  {'Notes'}")
    print("-" * 70)
    for r in results:
        notes = ", ".join(r.get("reasons", []))
        if "error" in r:
            notes = f"ERROR: {r['error']}"
        print(f"{r.get('trajectory', '?'):<35s} {r.get('frames', 0):>6d}  "
              f"{'YES' if r.get('success') else 'NO':>8s}  {notes}")
    print(f"\nTotal: {n_success}/{len(results)} successful")

    client.close()


if __name__ == "__main__":
    main()
