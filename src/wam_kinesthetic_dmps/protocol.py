"""Versioned little-endian UDP protocols for robot state and actuation commands."""
from dataclasses import dataclass, field
import socket
import struct

import numpy as np

PACKET = struct.Struct("<4sIQQ14d")
MAGIC = b"WAM7"
COMMAND_MAGIC = b"WAMC"
POLICY_STATE_MAGIC = b"WAPS"
ACTION_CHUNK_MAGIC = b"WAMH"
VERSION = 1
MAX_COMMAND_HORIZON = 8
COMMAND_PACKET = struct.Struct("<4sIQQI7d")
HORIZON_PACKET = struct.Struct(f"<4sIQQI{MAX_COMMAND_HORIZON * 7}d")
POLICY_STATE_PACKET = struct.Struct("<4sIQQQQ14d")
ACTION_CHUNK_PACKET = struct.Struct(f"<4sIQQI{MAX_COMMAND_HORIZON * 7}d")


@dataclass
class State:
    sequence: int
    timestamp_ns: int
    positions: np.ndarray
    velocities: np.ndarray


@dataclass
class Command:
    sequence: int
    timestamp_ns: int
    positions: np.ndarray
    horizon: np.ndarray = field(default_factory=lambda: np.empty((0, 7), dtype=float))

    def __post_init__(self):
        q = np.asarray(self.positions, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError("Expected seven finite target positions")
        horizon = np.asarray(self.horizon, dtype=float) if self.horizon is not None else np.empty((0, 7), dtype=float)
        if horizon.size == 0:
            horizon = np.empty((0, 7), dtype=float)
        elif horizon.ndim == 1:
            if horizon.size != 7:
                raise ValueError("Expected a 7D target or an (N, 7) future horizon")
            horizon = horizon.reshape(1, 7)
        if horizon.ndim != 2 or horizon.shape[1] != 7:
            raise ValueError("Expected the future horizon to have shape (N, 7)")
        if horizon.shape[0] > MAX_COMMAND_HORIZON:
            raise ValueError(f"Future horizon cannot exceed {MAX_COMMAND_HORIZON} points")
        if not np.isfinite(horizon).all():
            raise ValueError("Expected finite future horizon values")
        self.positions = q
        self.horizon = horizon


@dataclass
class PolicyState:
    sequence: int
    timestamp_ns: int
    remaining_ns: int
    next_action_index: int
    positions: np.ndarray
    velocities: np.ndarray


@dataclass
class ActionChunk:
    first_index: int
    sample_period_ns: int
    actions: np.ndarray

    def __post_init__(self):
        actions = np.asarray(self.actions, dtype=float)
        if actions.ndim != 2 or actions.shape[1] != 7:
            raise ValueError("Expected action chunk with shape (N, 7)")
        if not 1 <= actions.shape[0] <= MAX_COMMAND_HORIZON:
            raise ValueError(f"Action chunk must contain 1..{MAX_COMMAND_HORIZON} points")
        if not np.isfinite(actions).all():
            raise ValueError("Expected finite action positions")
        if self.sample_period_ns <= 0:
            raise ValueError("Sample period must be positive")
        self.actions = actions


def encode(state):
    q = np.asarray(state.positions, dtype=float)
    dq = np.asarray(state.velocities, dtype=float)
    if q.shape != (7,) or dq.shape != (7,) or not np.isfinite([q, dq]).all():
        raise ValueError("Expected seven finite positions and velocities")
    return PACKET.pack(MAGIC, VERSION, state.sequence, state.timestamp_ns, *q, *dq)


def decode(data):
    if len(data) != PACKET.size:
        raise ValueError(f"Expected {PACKET.size} bytes, got {len(data)}")
    magic, version, seq, ns, *values = PACKET.unpack(data)
    if magic != MAGIC or version != VERSION:
        raise ValueError("Unrecognized state protocol")
    if not np.isfinite(values).all() or ns == 0:
        raise ValueError("Invalid state values or timestamp")
    return State(seq, ns, np.array(values[:7]), np.array(values[7:]))


def encode_command(command):
    if getattr(command, "horizon", None) is None or len(command.horizon) == 0:
        q = np.asarray(command.positions, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError("Expected seven finite target positions")
        return COMMAND_PACKET.pack(COMMAND_MAGIC, VERSION, command.sequence, command.timestamp_ns, 1, *q)

    horizon = np.asarray(command.horizon, dtype=float)
    if horizon.ndim == 1:
        horizon = horizon.reshape(1, 7)
    if horizon.ndim != 2 or horizon.shape[1] != 7:
        raise ValueError("Expected the future horizon to have shape (N, 7)")
    if horizon.shape[0] > MAX_COMMAND_HORIZON:
        raise ValueError(f"Future horizon cannot exceed {MAX_COMMAND_HORIZON} points")
    if not np.isfinite(horizon).all():
        raise ValueError("Expected finite future horizon values")

    flat = np.zeros(MAX_COMMAND_HORIZON * 7, dtype=float)
    flat[:horizon.size] = horizon.reshape(-1)
    return HORIZON_PACKET.pack(COMMAND_MAGIC, VERSION, command.sequence, command.timestamp_ns, len(horizon), *flat)


def decode_command(data):
    if len(data) == COMMAND_PACKET.size:
        magic, version, seq, ns, horizon_len, *values = COMMAND_PACKET.unpack(data)
        if magic != COMMAND_MAGIC or version != VERSION:
            raise ValueError("Unrecognized action protocol")
        if horizon_len != 1:
            raise ValueError("Unexpected command horizon length")
        positions = np.array(values[:7], dtype=float)
        horizon = np.empty((0, 7), dtype=float)
    elif len(data) == HORIZON_PACKET.size:
        magic, version, seq, ns, horizon_len, *values = HORIZON_PACKET.unpack(data)
        if magic != COMMAND_MAGIC or version != VERSION:
            raise ValueError("Unrecognized action protocol")
        if horizon_len < 1 or horizon_len > MAX_COMMAND_HORIZON:
            raise ValueError("Invalid command horizon length")
        horizon = np.asarray(values[:horizon_len * 7], dtype=float).reshape(horizon_len, 7)
        positions = horizon[0].copy()
    else:
        raise ValueError(f"Expected {COMMAND_PACKET.size} or {HORIZON_PACKET.size} bytes, got {len(data)}")

    if not np.isfinite(values).all() or ns == 0:
        raise ValueError("Invalid command values or timestamp")
    return Command(seq, ns, positions, horizon=horizon)


def encode_policy_state(state):
    q = np.asarray(state.positions, dtype=float)
    dq = np.asarray(state.velocities, dtype=float)
    if q.shape != (7,) or dq.shape != (7,) or not np.isfinite([q, dq]).all():
        raise ValueError("Expected seven finite state positions and velocities")
    if state.timestamp_ns <= 0 or state.remaining_ns < 0:
        raise ValueError("Invalid policy state timestamp or buffer duration")
    return POLICY_STATE_PACKET.pack(
        POLICY_STATE_MAGIC, VERSION, state.sequence, state.timestamp_ns,
        state.remaining_ns, state.next_action_index, *q, *dq,
    )


def decode_policy_state(data):
    if len(data) != POLICY_STATE_PACKET.size:
        raise ValueError(f"Expected {POLICY_STATE_PACKET.size} bytes, got {len(data)}")
    magic, version, sequence, timestamp_ns, remaining_ns, next_action_index, *values = POLICY_STATE_PACKET.unpack(data)
    if magic != POLICY_STATE_MAGIC or version != VERSION:
        raise ValueError("Unrecognized policy-state protocol")
    if timestamp_ns == 0 or not np.isfinite(values).all():
        raise ValueError("Invalid policy state timestamp or values")
    return PolicyState(
        sequence, timestamp_ns, remaining_ns, next_action_index,
        np.asarray(values[:7]), np.asarray(values[7:]),
    )


def encode_action_chunk(chunk):
    actions = np.asarray(chunk.actions, dtype=float)
    if actions.ndim != 2 or actions.shape[1] != 7:
        raise ValueError("Expected action chunk with shape (N, 7)")
    if not 1 <= actions.shape[0] <= MAX_COMMAND_HORIZON:
        raise ValueError(f"Action chunk must contain 1..{MAX_COMMAND_HORIZON} points")
    if not np.isfinite(actions).all() or chunk.sample_period_ns <= 0:
        raise ValueError("Invalid action chunk values or sample period")
    padded = np.zeros((MAX_COMMAND_HORIZON, 7), dtype=float)
    padded[:len(actions)] = actions
    return ACTION_CHUNK_PACKET.pack(
        ACTION_CHUNK_MAGIC, VERSION, chunk.first_index, chunk.sample_period_ns,
        len(actions), *padded.reshape(-1),
    )


def decode_action_chunk(data):
    if len(data) != ACTION_CHUNK_PACKET.size:
        raise ValueError(f"Expected {ACTION_CHUNK_PACKET.size} bytes, got {len(data)}")
    magic, version, first_index, sample_period_ns, count, *values = ACTION_CHUNK_PACKET.unpack(data)
    if magic != ACTION_CHUNK_MAGIC or version != VERSION:
        raise ValueError("Unrecognized action-chunk protocol")
    if not 1 <= count <= MAX_COMMAND_HORIZON or sample_period_ns == 0:
        raise ValueError("Invalid action chunk length or sample period")
    actions = np.asarray(values[:count * 7], dtype=float).reshape(count, 7)
    if not np.isfinite(actions).all():
        raise ValueError("Invalid action chunk positions")
    return ActionChunk(first_index, sample_period_ns, actions)


class Receiver:
    def __init__(self, bind_ip="127.0.0.1", port=6560, source_ip=None):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            self.socket.bind((bind_ip, port))
            self.socket.settimeout(0.2)
        except Exception:
            self.socket.close()
            raise
        self.source_ip = source_ip

    def receive(self):
        try:
            data, address = self.socket.recvfrom(65535)
        except socket.timeout:
            return None
        if self.source_ip and address[0] != self.source_ip:
            return None
        return decode(data)

    def close(self):
        self.socket.close()
