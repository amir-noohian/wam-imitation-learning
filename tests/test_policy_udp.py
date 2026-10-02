import struct
import socket
import threading
import time

import numpy as np
import pytest

from wam_kinesthetic_dmps.dmp_model import fit, load_model, rollout, save_model
from wam_kinesthetic_dmps.policy import send_actions
from wam_kinesthetic_dmps.protocol import (
    ActionChunk,
    Command,
    PolicyState,
    State,
    decode_action_chunk,
    decode_command,
    decode_policy_state,
    encode_action_chunk,
    encode_command,
    encode_policy_state,
)
from wam_kinesthetic_dmps.recorder import Recorder


def test_command_packet_roundtrip():
    cmd = Command(7, 123456789, np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
    packed = encode_command(cmd)
    assert len(packed) == struct.calcsize("<4sIQQI7d")
    got = decode_command(packed)
    assert got.sequence == 7
    assert got.timestamp_ns == 123456789
    np.testing.assert_allclose(got.positions, cmd.positions)
    np.testing.assert_allclose(got.horizon, np.empty((0, 7)))

    with pytest.raises(ValueError):
        decode_command(b"FAIL" + packed[4:])


def test_horizon_packet_roundtrip():
    horizon = np.array([
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
        [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9],
    ])
    cmd = Command(11, 987654321, horizon[0], horizon=horizon)
    packed = encode_command(cmd)
    assert len(packed) == struct.calcsize("<4sIQQI" + "7d" * 8)
    got = decode_command(packed)
    assert got.sequence == 11
    assert got.timestamp_ns == 987654321
    np.testing.assert_allclose(got.positions, horizon[0])
    np.testing.assert_allclose(got.horizon, horizon)


def test_policy_state_packet_roundtrip():
    state = PolicyState(4, 123456789, 250000000, 24, np.arange(7), np.arange(7) / 10)
    got = decode_policy_state(encode_policy_state(state))
    assert got.sequence == state.sequence
    assert got.timestamp_ns == state.timestamp_ns
    assert got.remaining_ns == state.remaining_ns
    assert got.next_action_index == state.next_action_index
    np.testing.assert_allclose(got.positions, state.positions)
    np.testing.assert_allclose(got.velocities, state.velocities)


def test_action_chunk_packet_roundtrip():
    actions = np.arange(21, dtype=float).reshape(3, 7) / 10
    chunk = ActionChunk(16, 10_000_000, actions)
    got = decode_action_chunk(encode_action_chunk(chunk))
    assert got.first_index == chunk.first_index
    assert got.sample_period_ns == chunk.sample_period_ns
    np.testing.assert_allclose(got.actions, actions)


def test_action_chunk_rejects_invalid_size():
    with pytest.raises(ValueError):
        ActionChunk(0, 10_000_000, np.empty((0, 7)))


def test_policy_sender_refills_from_robot_feedback(tmp_path):
    timestamps = 1_000_000_000 + np.arange(21, dtype=np.int64) * 10_000_000
    progress = np.linspace(0.0, 1.0, len(timestamps))
    positions = 0.1 + progress[:, None] * np.arange(1, 8)[None, :] * 0.01
    velocities = np.gradient(positions, 0.01, axis=0)
    recorder = Recorder()
    for sequence, (timestamp, q, dq) in enumerate(zip(timestamps, positions, velocities)):
        recorder.add(State(sequence, int(timestamp), q, dq))
    demo_path = tmp_path / "demo.hdf5"
    model_path = tmp_path / "model.json"
    recorder.save(demo_path)
    save_model(fit(demo_path, weights=5), model_path)

    start_position = np.full(7, 0.05)
    model = load_model(model_path)
    _, expected_actions = rollout(
        model,
        start_position=start_position,
        start_velocity=np.zeros(7),
        retarget_forcing=True,
    )

    state_probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state_probe.bind(("127.0.0.1", 0))
    state_port = state_probe.getsockname()[1]
    state_probe.close()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as command_receiver, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as state_sender:
        command_receiver.bind(("127.0.0.1", 0))
        command_receiver.settimeout(0.005)
        runner_errors = []

        def run_policy():
            try:
                send_actions(
                    model_path,
                    host="127.0.0.1",
                    port=command_receiver.getsockname()[1],
                    bind="127.0.0.1",
                    state_port=state_port,
                    source_ip="127.0.0.1",
                    buffer_seconds=0.08,
                    refill_threshold=0.04,
                    state_timeout=1.0,
                    retarget_forcing=True,
                )
            except Exception as exc:
                runner_errors.append(exc)

        runner = threading.Thread(target=run_policy)
        runner.start()
        base_index = 40
        expected_index = base_index
        received_actions = []
        deadline = time.monotonic() + 5.0
        state_sequence = 0
        while runner.is_alive() and time.monotonic() < deadline:
            state = PolicyState(
                state_sequence,
                time.monotonic_ns(),
                0,
                expected_index,
                start_position,
                np.zeros(7),
            )
            state_sender.sendto(encode_policy_state(state), ("127.0.0.1", state_port))
            state_sequence += 1
            try:
                data, _ = command_receiver.recvfrom(65535)
            except socket.timeout:
                continue
            chunk = decode_action_chunk(data)
            if chunk.first_index != expected_index:
                continue
            received_actions.extend(chunk.actions)
            expected_index += len(chunk.actions)

        runner.join(timeout=1.0)
        assert not runner.is_alive(), "Policy sender did not finish acknowledging the rollout"
        assert not runner_errors
        assert expected_index == base_index + len(expected_actions)
        np.testing.assert_allclose(received_actions, expected_actions)
