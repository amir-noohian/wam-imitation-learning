import socket
import struct
import h5py
import numpy as np
import pytest
from wam_kinesthetic_dmps.protocol import State, Receiver, encode, decode
from wam_kinesthetic_dmps.recorder import Recorder, load_demo
from wam_kinesthetic_dmps.dmp_model import fit, save_model, load_model, rollout


def state(seq, ns):
    return State(seq, ns, np.arange(7, dtype=float), np.zeros(7))


def test_wire_layout_and_validation():
    data = struct.pack("<4sIQQ14d", b"WAM7", 1, 4, 100, *range(14))
    got = decode(data)
    assert len(data) == 136
    assert got.sequence == 4 and got.timestamp_ns == 100
    np.testing.assert_array_equal(got.positions, np.arange(7))
    np.testing.assert_array_equal(got.velocities, np.arange(7, 14))
    assert encode(got) == data
    for invalid in (data[:-1], b"FAIL" + data[4:], data[:4] + struct.pack("<I", 2) + data[8:]):
        with pytest.raises(ValueError):
            decode(invalid)
    with pytest.raises(ValueError):
        encode(State(1, 100, [float("nan")] * 7, [0] * 7))


def test_udp_loopback():
    receiver = Receiver(port=0)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(encode(state(1, 100)), receiver.socket.getsockname())
        got = receiver.receive()
        assert got.sequence == 1
        np.testing.assert_array_equal(got.positions, np.arange(7))
    finally:
        receiver.close()


def test_packet_order_and_exclusive_save(tmp_path):
    recorder = Recorder()
    assert recorder.add(state(1, 100))
    assert recorder.add(state(3, 200))
    assert not recorder.add(state(2, 150))
    assert not recorder.add(state(4, 200))
    assert recorder.add(state(4, 300))
    assert recorder.dropped_packets == 1
    assert recorder.rejected_packets == 2
    path = tmp_path / "demo.hdf5"
    recorder.save(path)
    with h5py.File(path) as f:
        assert f.attrs["dropped_packets"] == 1
    with pytest.raises(FileExistsError):
        recorder.save(path)
    t, q = load_demo(path)
    np.testing.assert_allclose(t, [0, 1e-7, 2e-7])
    assert q.shape == (3, 7)


def test_training_and_model_roundtrip(tmp_path):
    # Irregular source sampling and a large clock epoch test timing preservation.
    t = np.linspace(0, 2, 1001)
    t[1:-1] += 0.0001 * np.sin(np.arange(999))
    amplitude = np.linspace(0.1, 0.4, 7)
    phase = t / 2
    blend = 10 * phase**3 - 15 * phase**4 + 6 * phase**5
    q = blend[:, None] * amplitude
    dq = np.gradient(q, t, axis=0)
    recorder = Recorder()
    for i in range(len(t)):
        recorder.add(State(i, 10**18 + int(t[i] * 1e9), q[i], dq[i]))
    path = tmp_path / "demo.hdf5"
    recorder.save(path)
    model = fit(path)
    assert model["execution_time"] == pytest.approx(2)
    model_path = tmp_path / "model.json"
    save_model(model, model_path)
    rt, rq = rollout(load_model(model_path))
    ot, oq = rollout(model)
    np.testing.assert_array_equal(rt, ot)
    np.testing.assert_allclose(rq, oq, atol=1e-12)
    interpolated = np.column_stack([np.interp(t, rt, rq[:, j]) for j in range(7)])
    assert np.sqrt(np.mean((interpolated - q)**2)) < 0.01
    with pytest.raises(ValueError, match="Sampling gap"):
        fit(path, max_gap=0.0005)
