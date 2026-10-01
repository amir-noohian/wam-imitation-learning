import struct

import numpy as np
import pytest

from wam_kinesthetic_dmps.protocol import Command, encode_command, decode_command


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
