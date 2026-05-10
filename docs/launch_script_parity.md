# Launch Script Parity

Date: 2026-05-07

## Scope

This note records the shared wrapper arguments for `scripts/run_go2_sim.sh` and `scripts/run_go2_real.sh`.

## Shared Arguments

Both scripts accept:

- `mapping` or `localization` as the first positional mode.
- `--map <path>` for localization mode.
- `--no-rviz` to suppress RViz.
- `--use-lio` to start the optional LIO backend.
- `--no-stvl` to select the non-STVL Nav2 profile path.
- `--nav-profile <id>` for `default_safe`, `narrow_gap_1p0m`, or `forward_static`.
- `--use-route-server` and `--route-graph-file <path>` for route server experiments.
- `--robot-name <name>` where the underlying launch mode uses a robot name or namespace.
- `--distortion-imu-topic <topic>` to choose the IMU used by LiDAR correction.
- `--nav-delay <sec>` to delay navigation startup. Simulation delays SLAM by this amount and starts Nav2 10 seconds later; real hardware delays SLAM and Nav2 together.

## Mode-Specific Defaults

Simulation keeps `--nav-delay` at `20.0` seconds because CHAMP needs spawn and stand-up time before SLAM samples the LiDAR. Real hardware keeps `--nav-delay` at `0.0` seconds to preserve the previous default startup behavior.

Real hardware defaults the raw LiDAR pointcloud input to `/utlidar/cloud` with frame `utlidar_lidar`. `robot_lio` relays the cloud to `/lidar/points` and publishes an identity `lidar_link -> utlidar_lidar` TF alias; the hardware `lidar_link` orientation is the measured Unitree driver frame.

Real hardware defaults the raw LiDAR IMU input to `/utlidar/imu`. The `robot_lio` relay publishes the canonical `/lidar/imu` topic for LIO consumers.

## Mode-Specific Arguments

Simulation-only options are `--world`, `--gui`, `--no-gui`, and `--no-clean`.

Real-only options are `--no-safety`, `--use-safety`, `--lidar-points-in-topic`, `--lidar-points-frame`, `--lidar-imu-in-topic`, `--recovery-ack-key-mask`, `--resume-key-mask`, `--abort-key-mask`, `--record-rosbag`, `--rosbag-dir`, and `--rosbag-topics`.

`--record-rosbag` starts `ros2 bag record` as a background sibling of the launch and traps SIGINT/SIGTERM to flush metadata. The default topic set covers offline perception replay (relayed LiDAR cloud + IMU, processed clouds, body IMU, sport-mode state, odometry, TF, robot description, map, status). The recorder uses `scripts/rosbag_qos_overrides.yaml` so it subscribes to `/tf_static`, `/robot_description`, and `/map` with TRANSIENT_LOCAL durability and captures their latched samples.
