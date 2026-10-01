"""Export a time-scaled DMP for controller-side playback; never starts hardware."""
import argparse
from pathlib import Path
import numpy as np
from .dmp_model import load_model, rollout

HEADER = "time_s,j1,j2,j3,j4,j5,j6,j7"


def export_trajectory(model, path, slowdown=2.0):
    if not np.isfinite(slowdown) or slowdown < 1:
        raise ValueError("slowdown must be >= 1 (2 means twice the duration)")
    t, q = rollout(model)
    if len(t) < 3 or q.shape != (len(t), 7) or not np.isfinite(t).all():
        raise ValueError("Invalid DMP rollout")
    if t[0] != 0 or np.any(np.diff(t) <= 0):
        raise ValueError("DMP timestamps must start at zero and increase")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as f:
        np.savetxt(f, np.column_stack((t * slowdown, q)), delimiter=",",
                   header=HEADER, comments="", fmt="%.17g")
    return t[-1] * slowdown, q[0]


def main():
    parser = argparse.ArgumentParser(description="Export DMP replay CSV (does not move the robot)")
    parser.add_argument("model")
    parser.add_argument("output")
    parser.add_argument("--slowdown", type=float, default=2.0)
    args = parser.parse_args()
    duration, start = export_trajectory(load_model(args.model), args.output, args.slowdown)
    print(f"Saved {args.output}; duration={duration:.3f}s, plus controller start blend")
    print("Start posture (rad):", start)
    print("Validate with wam_replay --check before hardware playback.")


if __name__ == "__main__":
    main()
