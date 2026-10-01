"""State-only HDF5 episodes with source timestamps and packet-loss counts."""
from pathlib import Path
import h5py
import numpy as np

JOINT_NAMES = tuple(f"wam_j{i}" for i in range(1, 8))


class Recorder:
    def __init__(self):
        self.samples = []
        self.dropped_packets = 0
        self.rejected_packets = 0

    def add(self, state):
        if self.samples:
            prev = self.samples[-1]
            if state.sequence <= prev.sequence or state.timestamp_ns <= prev.timestamp_ns:
                self.rejected_packets += 1
                return False
            self.dropped_packets += state.sequence - prev.sequence - 1
        self.samples.append(state)
        return True

    def save(self, path):
        if len(self.samples) < 3:
            raise ValueError("Need at least three samples to save an episode")
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with h5py.File(path, "x") as f:
            f.attrs.update(schema_version=1, dof=7, position_units="rad",
                           velocity_units="rad/s", timestamp_source="sender monotonic clock",
                           dropped_packets=self.dropped_packets,
                           rejected_packets=self.rejected_packets)
            f.create_dataset("joint_names", data=np.asarray(JOINT_NAMES, dtype="S"))
            f.create_dataset("timestamp_ns", data=np.array([s.timestamp_ns for s in self.samples], dtype=np.uint64))
            f.create_dataset("sequence", data=np.array([s.sequence for s in self.samples], dtype=np.uint64))
            f.create_dataset("joint_positions", data=np.stack([s.positions for s in self.samples]), compression="gzip")
            f.create_dataset("joint_velocities", data=np.stack([s.velocities for s in self.samples]), compression="gzip")


def load_demo(path):
    with h5py.File(path, "r") as f:
        if f.attrs.get("schema_version") != 1 or f.attrs.get("dof") != 7:
            raise ValueError("Unsupported demonstration schema")
        if tuple(x.decode() for x in f["joint_names"][:]) != JOINT_NAMES:
            raise ValueError("Unexpected joint order")
        ns, q = f["timestamp_ns"][:], f["joint_positions"][:]
    if ns.ndim != 1 or len(ns) < 3 or q.shape != (len(ns), 7) or not np.isfinite(q).all():
        raise ValueError("Expected at least three finite seven-joint samples")
    if np.any(ns[1:] <= ns[:-1]):
        raise ValueError("Timestamps must increase strictly")
    return (ns - ns[0]).astype(float) * 1e-9, q
