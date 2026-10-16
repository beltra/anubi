# Launch Script Parity

## Scope

This note records the shared wrapper arguments for `scripts/run_go2_sim.sh` and
`scripts/run_go2_real.sh`, and (in "Unitree G1 Wrappers" below) the G1 equivalents
`scripts/run_g1_sim.sh` / `scripts/run_g1_real.sh`.

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

Simulation keeps `--nav-delay` at `20.0` seconds because CHAMP needs spawn and stand-up time before SLAM samples the LiDAR. Real hardware keeps `--nav-delay` at `0.0` seconds.

Real hardware defaults the raw LiDAR pointcloud input to `/utlidar/cloud` with frame `utlidar_lidar`. `robot_lio` relays the cloud to `/lidar/points` and publishes an identity `lidar_link -> utlidar_lidar` TF alias; the hardware `lidar_link` orientation is the measured Unitree driver frame.

Real hardware defaults the raw LiDAR IMU input to `/utlidar/imu`. The `robot_lio` relay publishes the canonical `/lidar/imu` topic for LIO consumers.

## Mode-Specific Arguments

Simulation-only options are `--world`, `--gui`, `--no-gui`, and `--no-clean`.

Real-only options are `--no-safety`, `--use-safety`, `--lidar-points-in-topic`, `--lidar-points-frame`, `--lidar-imu-in-topic`, `--recovery-ack-key-mask`, `--resume-key-mask`, `--abort-key-mask`, `--record-rosbag`, `--rosbag-dir`, and `--rosbag-topics`.

`--record-rosbag` starts `ros2 bag record` as a background sibling of the launch and traps SIGINT/SIGTERM to flush metadata. The default topic set covers offline perception replay (relayed LiDAR cloud + IMU, processed clouds, body IMU, sport-mode state, odometry, TF, robot description, map, status). The recorder uses `scripts/rosbag_qos_overrides.yaml` so it subscribes to `/tf_static`, `/robot_description`, and `/map` with TRANSIENT_LOCAL durability and captures their latched samples.

## Unitree G1 Wrappers

`scripts/run_g1_sim.sh` and `scripts/run_g1_real.sh` mirror the Go2 wrappers for the G1 humanoid. The underlying launches (`g1_sim sim_bringup_g1.launch.py`, `g1_bringup g1_bringup.launch.py`) reuse the robot-agnostic autonomy unchanged; the G1 differences are the kinematic sim base, the head-mounted Mid-360, the selectable locomotion command bridge (AMO or SONIC), and the G1 Nav2 profiles.

Shared with the Go2 wrappers: `mapping`/`localization` positional mode, `--map`, `--no-rviz`, `--no-stvl`, `--nav-delay`, `--distortion-imu-topic`, `--no-clean`.

G1-only shared options (both G1 wrappers): `--locomotion amo|sonic` (default `amo`) selects the low-level controller and with it the Nav2 velocity envelope (`nav2_params_g1.yaml` vs `nav2_params_g1_sonic_*.yaml`); `--sonic-profile slow_walk|walk` (default `slow_walk`) selects the SONIC envelope. In sim these switch ONLY the Nav2 params (the kinematic base is unchanged); on real they additionally start `cmd_vel_to_sonic` (ZMQ PUB :5556) instead of `cmd_vel_to_amo`. See `docs/g1.md`.

`run_g1_sim.sh` (sim-only): `--world`, `--gui`/`--no-gui` (headless is the reliable default on software GL, where the Gazebo GUI client may not render), and `--use-lio` to enable the optional DLIO backend. The `run_g1_sim.sh` wrapper sets `--nav-delay` to `14.0` (covers Gazebo + bridge + TF settle; the bare `sim_bringup_g1.launch.py` arg default is `10.0`). `use_lio` defaults to false: ground-truth odom + EKF is the reliable demo path because DLIO degrades during the kinematic base's motion.

`run_g1_real.sh` (real-only): `--no-livox`, `--no-lio`, `--no-ekf`, `--no-state-bridge`, `--no-nav`, `--no-safety`, `--livox-config <path>`, `--standing-height <m>`, `--net-if <iface>` (DDS NIC), and `--recovery-ack-key-mask`/`--resume-key-mask`/`--abort-key-mask`. `--nav-delay` defaults to `5.0` so DLIO finishes its ~3 s stationary IMU/gravity calibration and the EKF settles before navigation; keep the robot still during it. Real LiDAR input is `/livox/lidar` + `/livox/imu` (rescaled to `/livox/imu_ms2`), relayed by `robot_lio` to canonical `/lidar/points` + `/lidar/imu`. The command tail depends on `--locomotion`: `/cmd_vel_safe -> cmd_vel_to_amo -> AMO WebSocket :8766` (default; the AMO driver runs as a separate process on the nav host) or `/cmd_vel_safe -> cmd_vel_to_sonic -> ZMQ PUB :5556 -> g1_deploy_onnx_ref` on the Jetson (`--locomotion sonic`). Never run both low-level controllers at once (single writer on `rt/lowcmd`). The EKF owns `odom -> base_footprint` exactly as on the Go2. With `--locomotion sonic`, `--sonic-arm-preset none|default|carry` (default `none`, real-only, ignored for amo) makes `cmd_vel_to_sonic` hold a fixed upper-body pose via its optional `upper_body_position` planner field (loco-manipulation; validate hoisted and tune per payload).

`run_g1_sim.sh --world <path>` also derives `world_name:=` from the SDF's `<world name="...">` and forwards it, because the robot spawn (`create -world <name>`) must target that name. (`run_go2_sim.sh` does not forward a world name; the Go2 sim launch has no such argument.)

## Offline Replay Wrappers

`scripts/replay_bag_perception.sh` (Go2) and `scripts/replay_bag_perception_g1.sh` (G1) replay recorded LiDAR bags through minimal tuning stacks (`bag_perception.launch.py` / `bag_perception_g1.launch.py`). Shared options: `--rate`, `--loop`, `--start-offset`, `--no-rviz`, `--save-map <dir>`. Go2-only: `--odom-topic`, `--nav-profile`. G1-only: `--launch-arg <k:=v>` (repeatable passthrough); the G1 bag path defaults to `bags/g1/lidar_record`. Playback whitelists differ by design: the Go2 wrapper plays raw sensors + `/tf_static` + body odometry; the G1 wrapper plays ONLY `/livox/lidar_filtered` + `/livox/imu` and drops even `/tf_static` (the bag's own static tree conflicts with the replay statics). With `--save-map`, the G1 wrapper also records the replay outputs (`/tf`, `/map`, `/scan`, `/lidar_odom`, nav cloud, costmaps) to `<dir>/replay_outputs` for `scripts/perception_metrics.py`.

G1 SLAM config: all G1 entrypoints (`run_g1_sim.sh`, `run_g1_real.sh`, `replay_bag_perception_g1.sh`) default `slam_params` to `slam_toolbox_params_g1.yaml`. The shared `slam_toolbox_params.yaml` stays tuned for the Go2, whose drifting leg odometry needs a loose scan matcher; the G1's DLIO odometry is centimeter-accurate and the loose matcher slid meters along corridors (see the header of the G1 file for the measured evidence and the exact deltas).
