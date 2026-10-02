"""Train and restore JSON DMPs using the public movement_primitives API."""
import json
from importlib.metadata import version
from pathlib import Path
import numpy as np
from movement_primitives.dmp import DMP
from .recorder import JOINT_NAMES, load_demo

BOUNDARIES = ("start_y", "start_yd", "start_ydd", "goal_y", "goal_yd", "goal_ydd")


def fit(path, dt=0.01, weights=30, max_gap=0.1):
    if not np.isfinite([dt, max_gap]).all() or dt <= 0 or max_gap <= 0 or weights < 2:
        raise ValueError("dt/max_gap must be positive; weights must be >= 2")
    t, q = load_demo(path)
    if np.max(np.diff(t)) > max_gap:
        raise ValueError("Sampling gap exceeds max_gap; inspect the demonstration")
    if t[-1] < 2 * dt:
        raise ValueError("Demonstration too short for the requested timestep")
    grid = np.linspace(0, t[-1], int(np.ceil(t[-1] / dt)) + 1)
    y = np.column_stack([np.interp(grid, t, q[:, j]) for j in range(7)])
    dmp = DMP(n_dims=7, execution_time=float(t[-1]), dt=dt, n_weights_per_dim=weights)
    dmp.imitate(grid, y)
    model = dict(schema_version=1, joint_names=list(JOINT_NAMES),
                 library_version=version("movement_primitives"),
                 source_demo=str(Path(path).resolve()), execution_time=float(t[-1]),
                 dt=dt, n_weights_per_dim=weights, weights=dmp.get_weights().tolist())
    for name in BOUNDARIES:
        model[name] = np.asarray(getattr(dmp, name)).tolist()
    return model


def save_model(model, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as f:
        json.dump(model, f, indent=2, allow_nan=False)
        f.write("\n")


def load_model(path):
    with open(path) as f:
        model = json.load(f)
    if model.get("schema_version") != 1 or model.get("joint_names") != list(JOINT_NAMES):
        raise ValueError("Unsupported model schema or joint order")
    return model


def rollout(model, start_position=None, start_velocity=None):
    dmp = DMP(n_dims=7, execution_time=model["execution_time"], dt=model["dt"],
              n_weights_per_dim=model["n_weights_per_dim"])
    dmp.set_weights(np.asarray(model["weights"]))
    boundaries = {name: np.asarray(model[name]) for name in BOUNDARIES}
    if start_position is not None:
        start_position = np.asarray(start_position, dtype=float)
        if start_position.shape != (7,) or not np.isfinite(start_position).all():
            raise ValueError("Expected seven finite starting joint positions")
        boundaries["start_y"] = start_position
    if start_velocity is not None:
        start_velocity = np.asarray(start_velocity, dtype=float)
        if start_velocity.shape != (7,) or not np.isfinite(start_velocity).all():
            raise ValueError("Expected seven finite starting joint velocities")
        boundaries["start_yd"] = start_velocity
    dmp.configure(**boundaries)
    t, q = dmp.open_loop()
    if not np.isfinite(q).all():
        raise ValueError("Non-finite DMP trajectory")
    return t, q
