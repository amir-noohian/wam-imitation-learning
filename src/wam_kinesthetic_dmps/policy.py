"""Send DMP-generated joint actions to a robot-side UDP listener."""
import argparse
import socket
import time

import numpy as np

from .dmp_model import load_model, rollout
from .protocol import Command, encode_command


def _build_horizon(points, horizon_size=8):
    if len(points) == 0:
        raise ValueError("No policy actions were generated")
    if len(points) <= horizon_size:
        horizon = np.asarray(points, dtype=float)
        if horizon.shape[0] < horizon_size:
            pad = np.repeat(horizon[-1:].copy(), horizon_size - horizon.shape[0], axis=0)
            horizon = np.vstack([horizon, pad])
        return horizon
    return np.asarray(points[:horizon_size], dtype=float)


def generate_actions(model_path, duration=None, sampling_hz=20.0):
    model = load_model(model_path)
    t, q = rollout(model)
    if duration is not None:
        q = q[t <= duration]
        t = t[t <= duration]
    if len(q) == 0:
        raise ValueError("No policy actions were generated")
    step = 1.0 / float(sampling_hz)
    actions = []
    for i, target in enumerate(q):
        actions.append((t[i], np.asarray(target, dtype=float)))
    if len(actions) == 1:
        return actions
    gap = max(0.0, float(actions[1][0] - actions[0][0]))
    if gap == 0.0:
        gap = step
    return actions, gap


def send_actions(model_path, host="127.0.0.1", port=6561, hz=20.0, duration=None, horizon_size=8):
    model = load_model(model_path)
    t, q = rollout(model)
    if duration is not None:
        mask = t <= duration
        t, q = t[mask], q[mask]
    if len(q) == 0:
        raise ValueError("No policy actions were generated")

    horizon_size = max(1, min(int(horizon_size), 8))
    step = 1.0 / max(float(hz), 1e-9)
    seq = 0
    next_due = time.monotonic()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for i in range(len(q)):
            now = time.monotonic_ns()
            window = np.asarray(q[i:i + horizon_size], dtype=float)
            if window.shape[0] < horizon_size:
                pad = np.repeat(window[-1:].copy(), horizon_size - window.shape[0], axis=0)
                window = np.vstack([window, pad]) if window.shape[0] > 0 else np.repeat(q[-1:], horizon_size, axis=0)
            packet = Command(seq, now, np.asarray(window[0], dtype=float), horizon=window)
            sock.sendto(encode_command(packet), (host, port))
            seq += 1
            if i + 1 < len(q):
                remaining = step - (time.monotonic() - next_due)
                if remaining > 0.0:
                    time.sleep(remaining)
                next_due += step


def main():
    parser = argparse.ArgumentParser(description="Send a DMP-generated action horizon to the robot UDP listener")
    parser.add_argument("model", help="Saved JSON DMP model")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6561)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, help="Optional total execution time in seconds")
    parser.add_argument("--horizon", type=int, default=8, help="Number of future joint states in each packet")
    args = parser.parse_args()
    send_actions(args.model, host=args.host, port=args.port, hz=args.hz, duration=args.duration, horizon_size=args.horizon)
    print(f"Sent policy horizons to {args.host}:{args.port} at {args.hz:.1f} Hz")


if __name__ == "__main__":
    main()
