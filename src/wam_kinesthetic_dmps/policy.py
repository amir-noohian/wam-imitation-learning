"""Refill a robot-side action buffer from live state and a DMP rollout."""
import argparse
import socket

import numpy as np

from .dmp_model import load_model, rollout
from .protocol import ActionChunk, decode_policy_state, encode_action_chunk


def _receive_latest_state(sock, source_ip, timeout):
    latest = None
    while latest is None:
        data, address = sock.recvfrom(65535)
        if source_ip and address[0] != source_ip:
            continue
        try:
            latest = decode_policy_state(data)
        except ValueError:
            continue
    sock.setblocking(False)
    try:
        while True:
            data, address = sock.recvfrom(65535)
            if source_ip and address[0] != source_ip:
                continue
            try:
                state = decode_policy_state(data)
            except ValueError:
                continue
            if latest is None or state.sequence > latest.sequence:
                latest = state
    except BlockingIOError:
        pass
    finally:
        sock.settimeout(timeout)
    return latest


def send_actions(
    model_path,
    host="127.0.0.1",
    port=6561,
    bind="0.0.0.0",
    state_port=6562,
    source_ip=None,
    buffer_seconds=0.6,
    refill_threshold=0.25,
    state_timeout=3.0,
    duration=None,
    chunk_size=8,
    goal_position=None,
    retarget_forcing=False,
):
    if not np.isfinite([buffer_seconds, refill_threshold, state_timeout]).all():
        raise ValueError("Buffer and timeout settings must be finite")
    if buffer_seconds <= 0 or refill_threshold < 0 or refill_threshold >= buffer_seconds or state_timeout <= 0:
        raise ValueError("Require buffer > refill threshold >= 0 and a positive state timeout")
    chunk_size = max(1, min(int(chunk_size), 8))
    model = load_model(model_path)
    state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    state_socket.bind((bind, state_port))
    state_socket.settimeout(state_timeout)
    command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    actions = None
    sample_period_ns = int(round(float(model["dt"]) * 1e9))
    base_index = None
    next_index = 0
    last_state_sequence = -1
    target_buffer_ns = int(buffer_seconds * 1e9)
    refill_ns = int(refill_threshold * 1e9)
    try:
        print(f"Waiting for WAM state on {bind}:{state_port}", flush=True)
        while True:
            try:
                state = _receive_latest_state(state_socket, source_ip, state_timeout)
            except socket.timeout as exc:
                raise TimeoutError("Timed out waiting for policy state from the WAM") from exc
            if state.sequence <= last_state_sequence:
                continue
            last_state_sequence = state.sequence

            if actions is None:
                if state.remaining_ns > 0:
                    continue
                base_index = state.next_action_index
                t, actions = rollout(
                    model,
                    start_position=state.positions,
                    start_velocity=state.velocities,
                    goal_position=goal_position,
                    retarget_forcing=retarget_forcing,
                )
                if duration is not None:
                    mask = t <= duration
                    actions = actions[mask]
                if len(actions) == 0:
                    raise ValueError("No policy actions were generated")
                mode = "retargeted" if retarget_forcing else "unscaled"
                print(f"DMP initialized from live WAM state ({mode}); {len(actions)} action samples at {1e9 / sample_period_ns:.1f} Hz", flush=True)

            if state.next_action_index < base_index:
                raise RuntimeError("WAM action index moved backwards during the rollout")
            acknowledged = state.next_action_index - base_index
            if acknowledged > len(actions):
                raise RuntimeError("WAM acknowledged an action index beyond this DMP rollout")
            next_index = acknowledged
            estimated_buffer_ns = state.remaining_ns
            if estimated_buffer_ns <= refill_ns:
                while next_index < len(actions) and estimated_buffer_ns < target_buffer_ns:
                    end_index = min(next_index + chunk_size, len(actions))
                    chunk = ActionChunk(
                        base_index + next_index,
                        sample_period_ns,
                        actions[next_index:end_index],
                    )
                    command_socket.sendto(encode_action_chunk(chunk), (host, port))
                    queued_ns = (end_index - next_index) * sample_period_ns
                    next_index = end_index
                    estimated_buffer_ns += queued_ns
                print(
                    f"WAM accepted through action {acknowledged}; "
                    f"buffer ~{state.remaining_ns / 1e9:.3f}s; sent through {next_index}",
                    flush=True,
                )

            if acknowledged >= len(actions):
                print("All DMP action samples are buffered on the WAM.", flush=True)
                return
    finally:
        state_socket.close()
        command_socket.close()


def main():
    parser = argparse.ArgumentParser(description="Stream state-conditioned DMP action chunks to the WAM")
    parser.add_argument("model", help="Saved JSON DMP model")
    parser.add_argument("--host", default="127.0.0.1", help="WAM IPv4 address for action chunks")
    parser.add_argument("--port", type=int, default=6561)
    parser.add_argument("--bind", default="0.0.0.0", help="Local address for WAM state feedback")
    parser.add_argument("--state-port", type=int, default=6562)
    parser.add_argument("--source-ip", help="Accept state packets only from this WAM IPv4 address")
    parser.add_argument("--buffer-seconds", type=float, default=0.6)
    parser.add_argument("--refill-threshold", type=float, default=0.25)
    parser.add_argument("--state-timeout", type=float, default=3.0)
    parser.add_argument("--duration", type=float, help="Optional total execution time in seconds")
    parser.add_argument("--chunk-size", type=int, default=8)
    parser.add_argument("--goal", nargs=7, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
                        help="Optional target joint position in radians; defaults to the trained DMP goal")
    parser.add_argument("--retarget-forcing", action="store_true",
                        help="Scale DMP forcing for the live start and selected goal; suppresses learned forcing on demo joints moving under 0.02 rad")
    args = parser.parse_args()
    send_actions(
        args.model,
        host=args.host,
        port=args.port,
        bind=args.bind,
        state_port=args.state_port,
        source_ip=args.source_ip,
        buffer_seconds=args.buffer_seconds,
        refill_threshold=args.refill_threshold,
        state_timeout=args.state_timeout,
        duration=args.duration,
        chunk_size=args.chunk_size,
        goal_position=args.goal,
        retarget_forcing=args.retarget_forcing,
    )


if __name__ == "__main__":
    main()
