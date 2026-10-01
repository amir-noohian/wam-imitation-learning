"""Command-line tools. None sends robot motion commands."""
import argparse
from pathlib import Path
import socket
import time
import numpy as np
from .protocol import Receiver, State, encode
from .recorder import Recorder, load_demo


def positive(value):
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return result


def record():
    p = argparse.ArgumentParser(description="Record one episode; Ctrl+C stops and saves")
    p.add_argument("output", type=Path)
    p.add_argument("--bind", default="127.0.0.1")
    p.add_argument("--port", type=int, default=6560)
    p.add_argument("--source-ip", help="Accept packets only from this numeric IP")
    p.add_argument("--duration", type=positive, help="Seconds after the first accepted sample")
    p.add_argument("--timeout", type=positive, default=5.0, help="No-data timeout in seconds")
    args = p.parse_args()
    if args.output.exists():
        p.error("Output exists; choose a new episode filename")
    input("Press Enter to start recording (Ctrl+C during recording saves): ")
    receiver = Receiver(args.bind, args.port, args.source_ip)
    recorder = Recorder()
    last_data = time.monotonic()
    start = None
    print(f"Listening on {args.bind}:{args.port}", flush=True)
    try:
        while True:
            now = time.monotonic()
            if start is not None and args.duration and now - start >= args.duration:
                break
            if now - last_data >= args.timeout:
                print("State stream timed out; saving collected samples if any")
                break
            try:
                state = receiver.receive()
            except ValueError:
                recorder.rejected_packets += 1
                continue
            if state is not None and recorder.add(state):
                last_data = time.monotonic()
                if start is None:
                    start = last_data
                    print("Receiving state; recording started", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
    if len(recorder.samples) < 3:
        p.exit(1, "Too few samples; no episode saved\n")
    recorder.save(args.output)
    print(f"Saved {len(recorder.samples)} samples to {args.output}; "
          f"missing={recorder.dropped_packets}, rejected={recorder.rejected_packets}")


def train():
    from .dmp_model import fit, save_model
    p = argparse.ArgumentParser(description="Fit a seven-dimensional joint-space DMP")
    p.add_argument("demo")
    p.add_argument("output")
    p.add_argument("--dt", type=positive, default=0.01)
    p.add_argument("--weights", type=int, default=30)
    p.add_argument("--max-gap", type=positive, default=0.1)
    a = p.parse_args()
    model = fit(a.demo, a.dt, a.weights, a.max_gap)
    save_model(model, a.output)
    print(f"Saved model: {a.output}; duration={model['execution_time']:.3f}s")


def preview():
    from .dmp_model import load_model, rollout
    p = argparse.ArgumentParser(description="Plot demonstration and offline DMP rollout")
    p.add_argument("model")
    p.add_argument("--demo", help="Override the model's source demonstration path")
    p.add_argument("--output", type=Path, required=True, help="PNG or PDF output")
    a = p.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    model = load_model(a.model)
    t, q = load_demo(a.demo or model["source_demo"])
    rt, rq = rollout(model)
    fig, axes = plt.subplots(7, 1, figsize=(10, 14), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t, q[:, j], label="Demonstration")
        ax.plot(rt, rq[:, j], "--", label="DMP")
        ax.set_ylabel(f"J{j+1} (rad)")
        ax.grid(alpha=0.3)
    axes[0].legend()
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.output)
    plt.close(fig)
    reference = np.column_stack([np.interp(t, rt, rq[:, j]) for j in range(7)])
    print("Per-joint RMSE (rad):", np.sqrt(np.mean((q - reference)**2, axis=0)))
    print(f"Saved preview: {a.output}")


def simulate():
    p = argparse.ArgumentParser(description="Send synthetic state packets; no robot required")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=6560)
    p.add_argument("--hz", type=positive, default=500)
    p.add_argument("--duration", type=positive, default=10)
    a = p.parse_args()
    start = time.monotonic()
    seq = 0
    amplitudes = np.linspace(0.05, 0.2, 7)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            while time.monotonic() - start < a.duration:
                t = time.monotonic() - start
                omega = 2 * np.pi / a.duration
                q = amplitudes * (1 - np.cos(omega * t))
                dq = amplitudes * omega * np.sin(omega * t)
                sock.sendto(encode(State(seq, time.monotonic_ns(), q, dq)), (a.host, a.port))
                seq += 1
                time.sleep(max(0, start + seq / a.hz - time.monotonic()))
        except KeyboardInterrupt:
            pass
