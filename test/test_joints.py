"""
Test each degree of freedom: send deltas and check which joints move.
Run Unity first, then: python test_joints.py
"""
import json, socket, time, sys


def connect(host="127.0.0.1", port=5556):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Connecting to {host}:{port}...")
    s.connect((host, port))
    print("Connected.")
    return s


def send(sock, cmd):
    """Send JSON command, return parsed response. Handles any stale data."""
    data = (json.dumps(cmd) + "\n").encode()
    sock.sendall(data)
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("Unity disconnected")
        buf += chunk
    line, rest = buf.split(b"\n", 1)
    # If there's more data, stash it for next call
    if rest:
        # Simple approach: just parse first line, discard rest
        pass
    return json.loads(line.decode())


def main():
    sock = connect()

    # Reset
    send(sock, {"cmd": "reset"})
    print("Warming up 3s — focus Unity window now! ", end="", flush=True)
    for i in range(3, 0, -1):
        print(f"{i}... ", end="", flush=True)
        time.sleep(1)
    print()

    # First observation (bare JSON from get_obs, not wrapped in "obs")
    obs = send(sock, {"cmd": "get_obs"})
    print(f"Obs keys: {sorted(obs.keys())}")
    print(f"Initial R EEF: {obs.get('robot0_eef_pos', 'MISSING')}")
    print(f"Initial R joints: {obs.get('robot0_joint_pos', 'MISSING')}")

    if "robot0_eef_pos" not in obs:
        print("ERROR: observation missing robot0_eef_pos!")
        sock.close()
        return

    joint_names = ["base_yaw", "shoulder_pan", "shoulder_lift",
                   "elbow_flex", "wrist_flex", "wrist_roll"]

    tests = [
        ("+X 5cm",    [0.05, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 1.0]),
        ("-X 5cm",    [-0.05, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 1.0]),
        ("+Y 5cm",    [0, 0.05, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 1.0]),
        ("-Y 5cm",    [0, -0.05, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 1.0]),
        ("+Z 5cm",    [0, 0, 0.05, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 1.0]),
        ("-Z 5cm",    [0, 0, -0.05, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 1.0]),
    ]

    print()
    for name, action in tests:
        # Get state before
        obs = send(sock, {"cmd": "get_obs"})
        prev_eef = obs["robot0_eef_pos"]
        prev_joints = obs["robot0_joint_pos"]

        # Execute
        send(sock, {"cmd": "step", "action": action})

        # Get state after
        obs2 = send(sock, {"cmd": "get_obs"})
        new_eef = obs2["robot0_eef_pos"]
        new_joints = obs2["robot0_joint_pos"]

        deef = [new_eef[i] - prev_eef[i] for i in range(3)]
        dj = [new_joints[i] - prev_joints[i] for i in range(len(prev_joints))]
        moved = sum(1 for d in dj if abs(d) > 0.0001)

        print(f"--- {name} ---")
        print(f"  EEF Δ: ({deef[0]:+.4f}, {deef[1]:+.4f}, {deef[2]:+.4f})")
        print(f"  Joints moved: {moved}/6")
        for i, (n, d) in enumerate(zip(joint_names, dj)):
            m = " ◄" if abs(d) > 0.0001 else ""
            print(f"    {n:>14s}: Δ={d:+.4f}{m}")
        print()

    sock.close()
    print("Done.")


if __name__ == "__main__":
    main()
