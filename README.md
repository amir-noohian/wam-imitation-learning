# WAM kinesthetic DMPs

A ROS-free project for **one seven-joint WAM with no gripper**: kinesthetic
recording, DMP fitting, preview, and controller-side replay.

```
WAM + libbarrett                 Python computer
  gravity compensation
  joint-state sampling --UDP--> recorder --> HDF5 --> DMP JSON --> preview
```

Teaching and replay are separate executables. The simulator emits state packets only. No camera, teleoperation, ROS, haptic-handle,
or gripper dependencies are required.

## Python setup

From this directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
pytest -q
```

The initial development checks used `/tmp/wam-dmp-venv`; this temporary environment
is not part of the repository. CLI settings (`--help`) configure addresses, ports,
timeouts, DMP sample interval, and model size.

## Try the complete workflow without a robot

In terminal 1, activate the environment, then run:

```bash
wam-record data/demo_001.hdf5 --duration 5
```

Press Enter to open the receiver and start waiting. In terminal 2, activate the same
environment and run:

```bash
wam-simulate --duration 6
```

Then:

```bash
wam-train data/demo_001.hdf5 data/model_001.json
wam-preview data/model_001.json --output data/preview_001.png
```

The recorder stops after the requested duration following its first sample. Without
`--duration`, Ctrl+C stops and saves. A five-second no-data timeout also stops and
saves any collected samples; use `--timeout` to change it. Fewer than three samples
produce no file. Choose a new filename for every episode: demonstrations and models
are never overwritten. Recordings are buffered in RAM until saved.

## Hardware teaching program

`controller/wam_teach.cpp` is a standalone libbarrett program. Build it **on the WAM
control computer**, where libbarrett, the CAN interface, and the appropriate robot
configuration are available:

```bash
cmake -S controller -B build/controller
cmake --build build/controller
```

The Zeus configuration and its existing zero/gravity calibration are included in
`config/zeus-7dof/`, copied from `wam_teleop/config/leader-zeus-7dof`.
Set up CAN before starting the robot program, then select the configuration in the
same terminal used to launch the controller:

```bash
./scripts/can_init.sh          # peak_pci, can1, 1 Mbit/s; uses sudo
source scripts/setup_zeus.sh  # selects zeus.conf and its zero/gravity data
```

The CAN script defaults to `can1`, matching `bus.port = 1` in `zeus.conf`.
The setup script selects existing calibration; it does not run a new zero-calibration
procedure. See `config/README.md` for file details and interface selection.

Example when the recording computer is at `192.168.1.20`:

```bash
export WAM_STATE_HOST=192.168.1.20
export WAM_STATE_PORT=6560
./build/controller/wam_teach
```

On the recording computer (substitute the controller's actual IP for source-ip):

```bash
wam-record data/demo_001.hdf5 --bind 0.0.0.0 --source-ip 192.168.1.10
```

The libbarrett standard startup handles WAM initialization. The program enables
`wam.gravityCompensate()` without a position-tracking reference. Once the arm is
ready, press Enter in the recorder and guide the arm through the demonstration.
Start and finish at rest. Ctrl+C in **the Python recorder** saves the demonstration;
it does not change the robot's mode. Enter `q` in the C++ terminal to stop publishing,
then shift-idle the arm to let that program exit, as in the existing WAM application.

The C++ program has been compiled and linked successfully against the local
libbarrett installation in `/usr/local`. CMake explicitly selects C++11 because
the installed libbarrett/libconfig headers use exception specifications removed
in C++17. Hardware operation has not been tested; verify robot configuration and
gravity compensation on the WAM before collecting real demonstrations. The nominal 500 Hz publisher is a normal polling loop, not a
hard-real-time sampling guarantee; libbarrett retains the low-level robot control.

## Data and timing

State datagrams use the new protocol documented in `docs/protocol.md`; they are
**not compatible with the old leader/follower policy packet**.

An episode contains `joint_names`, `sequence`, `timestamp_ns`, `joint_positions`
(N x 7, radians), and `joint_velocities` (N x 7, radians/second). Attributes describe
units, schema, packet loss, and rejected packets. No images are required.

The timestamp is generated on the sending computer with a monotonic clock. Training
subtracts the first timestamp before conversion to seconds and resamples positions
onto a uniform grid. No synchronized wall clocks are needed. Large sampling gaps
(default >0.1 s) reject training rather than silently filling a long interruption.
Duplicate/reordered packets are rejected and sequence gaps counted. Restarting the
sender during an episode is unsupported: stop and start a new recording.

Models store seven-dimensional DMP weights, boundary states, execution duration,
sample interval, source filename, and library version in JSON. The default output
interval is 0.01 s (100 Hz), independent of the recording rate. Preview plots all
seven joints and reports reconstruction RMSE. There is no automatic trimming or
smoothing; long stationary periods and measurement noise remain in the demonstration.

The implementation uses the public `DMP` constructor, `imitate`, `configure`,
`get_weights`, `set_weights`, and `open_loop` APIs:
https://dfki-ric.github.io/movement_primitives/_apidoc/movement_primitives.dmp.DMP.html

## Project layout

- `src/wam_kinesthetic_dmps/protocol.py`: packet validation and UDP reception
- `src/wam_kinesthetic_dmps/recorder.py`: episode storage and validation
- `src/wam_kinesthetic_dmps/dmp_model.py`: fitting and JSON model restoration
- `src/wam_kinesthetic_dmps/cli.py`: recording, training, preview, synthetic sender
- `src/wam_kinesthetic_dmps/replay.py`: slowed trajectory export
- `config/zeus-7dof/`: Zeus configuration and zero/gravity calibration
- `scripts/`: configuration selection and CAN setup
- `controller/`: separate libbarrett teaching/replay programs and offline interpolation tests
- `tests/`: packet layout, UDP loopback, sample ordering, storage, model round-trip

## Replay a learned trajectory

Replay preloads a CSV and interpolates it inside libbarrett's execution manager.
It does not stream commands from Python and has no network dependency during motion.
Run `wam_replay` **instead of** `wam_teach`; do not run two robot programs together.
The replay program does not publish recording UDP states in this version.

1. Reinstall the Python package to register the new command:

   ```bash
   python -m pip install -e '.[test]'
   cmake -S controller -B build/controller
   cmake --build build/controller -j2
   ctest --test-dir build/controller --output-on-failure
   ```

2. Export a model with twice its original duration (positions are unchanged):

   ```bash
   wam-export data/zeus_model_001.json data/zeus_replay_001.csv --slowdown 2
   ```

3. Review `config/zeus-7dof/replay.conf`, which contains the user-supplied
   Zeus J1–J7 rotation limits in radians. Review the initial command velocity and
   acceleration caps (0.2 rad/s, 0.5 rad/s²) for your setup.

4. Validate offline; `--check` exits before constructing ProductManager:

   ```bash
   ./build/controller/wam_replay --check \
       data/zeus_replay_001.csv config/zeus-7dof/replay.conf
   ```

   Validation bounds the complete cubic interpolation, including between
   waypoints. If velocity/acceleration bounds fail, export with a larger slowdown
   to a new filename and check again.

5. After validation, with the teaching program closed and CAN already set up:

   ```bash
   source scripts/setup_zeus.sh
   ./build/controller/wam_replay \
       data/zeus_replay_001.csv config/zeus-7dof/replay.conf
   ```

   Follow WAM startup prompts. The program starts in gravity compensation.

   - `h` + Enter: move automatically from the measured posture to the recorded
     start. The arm must initially be stationary (each joint below 0.03 rad/s).
     The approach uses nonblocking `wam.moveTo()` with libbarrett's default
     internal velocity and acceleration values instead of the config-derived
     scalar limits. The endpoints are checked against position limits before
     moving. `start_blend_seconds` applies only to the replay blend.
   - `b` + Enter: return to the configured WAM home posture from
     `wam.getHomePosition()` using nonblocking `wam.moveTo()` with libbarrett's
     default values, then hold there. The arm must still be stationary and the
     same position and tracking checks apply as for `h`.
   - At the start posture: the controller holds position. Let the arm settle,
     then enter `r` + Enter to replay. Replay still checks that each joint is
     within 0.03 rad of its initial waypoint and below 0.03 rad/s. Replay does not
     start automatically after the approach.
   - You can also manually guide near the start in gravity compensation and use
     `r` directly. Replay includes the existing short blend from measured posture.
   - `s` + Enter: stop any motion and return to gravity compensation.
   - `q` + Enter or Ctrl+C: stop tracking, then wait for shift-idle to exit.
   - At replay completion: hold the final position. Use `h` to return to start,
     `b` to go home, or `s`/`q` to stop holding. Commands `h`/`b`/`r` are rejected during active motion.

The approach moves along a straight line in joint space, not in Cartesian space.
Check that its path is clear of the table, objects, and the robot itself; position
limits do not provide collision avoidance. `--check` validates the exported replay
only; approach endpoint checks use the live posture when `h` is pressed.

Position/velocity violations, non-finite state, or a tracking error above 0.15 rad
stop replay. During CSV replay, a 250 ms supervisory-loop timeout freezes the reference in the
libbarrett callback; when the supervisor resumes, it disconnects tracking. These
software checks are not a replacement for the WAM safety module. Stopping returns
to gravity compensation rather than applying a controlled braking trajectory.

The cubic interpolator imposes zero endpoint velocities. Replay settings are
software bounds, not collision checks. The C++ build and offline validation tests
have been exercised; robot replay has **not** been run by the assistant.
