"""Spatially retarget DMP forcing weights between joint-space boundaries."""
import numpy as np


MIN_DEMO_DISPLACEMENT = 0.02
MAX_FORCING_SCALE = 3.0


def scale_forcing_weights(
    weights,
    demo_start,
    demo_goal,
    new_start,
    new_goal,
    min_demo_displacement=MIN_DEMO_DISPLACEMENT,
    max_forcing_scale=MAX_FORCING_SCALE,
):
    weights = np.asarray(weights, dtype=float)
    boundaries = [np.asarray(value, dtype=float) for value in (
        demo_start, demo_goal, new_start, new_goal,
    )]
    if any(value.shape != (7,) or not np.isfinite(value).all() for value in boundaries):
        raise ValueError("Expected finite seven-joint start and goal positions")
    if weights.ndim != 2 or weights.shape[0] != 7 or not np.isfinite(weights).all():
        raise ValueError("Expected finite DMP weights with shape (7, n_weights)")
    if not np.isfinite([min_demo_displacement, max_forcing_scale]).all():
        raise ValueError("Retargeting limits must be finite")
    if min_demo_displacement <= 0 or max_forcing_scale <= 0:
        raise ValueError("Retargeting limits must be positive")

    demo_delta = boundaries[1] - boundaries[0]
    new_delta = boundaries[3] - boundaries[2]
    scales = np.zeros(7, dtype=float)
    movable = np.abs(demo_delta) >= min_demo_displacement
    scales[movable] = np.clip(
        new_delta[movable] / demo_delta[movable],
        -max_forcing_scale,
        max_forcing_scale,
    )
    return weights * scales[:, np.newaxis], scales