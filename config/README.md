# Zeus seven-joint WAM configuration

Copied from `/home/hela/amir/wam_teleop/config/leader-zeus-7dof`.
The directory is renamed `zeus-7dof`, and `leader.conf` is renamed `zeus.conf`.
Configuration and calibration contents are otherwise unchanged.

- `zeus-7dof/zeus.conf`: entry point; selects CAN bus port **1** (`can1`).
- `zeus-7dof/wam7w.conf`: robot model, gains, limits, and calibration includes.
- `zeus-7dof/calibration_data/wam7w/zerocal.conf`: existing home/zero data.
- `zeus-7dof/calibration_data/wam7w/gravitycal.conf`: existing gravity calibration.

Run `source scripts/setup_zeus.sh` from the project root in the terminal that will
launch the controller. This exports `BARRETT_CONFIG_FILE`; it does not communicate
with the robot or generate new calibration data. The zero file defines the home
posture as `(0, -2, 0, 3.13, 0, 0, 0)` radians. Follow the normal libbarrett startup
prompts for the physical arm; selecting a configuration is not a calibration run.

`scripts/can_init.sh` loads `peak_pci` and brings up `can1` at 1 Mbit/s,
matching `bus.port = 1` in `zeus.conf`. Run it before starting robot programs.
To use a different interface, edit the interface name in the script and the
`bus.port` value in `zeus.conf` to match.
