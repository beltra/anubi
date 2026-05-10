# AGENTS.md

## Scope and Truth Sources
- Workspace is ROS 2 Humble + Gazebo Fortress at `/ws/anubi`.
- Prefer executable truth over prose: check `src/**/launch/*.launch.py` and `scripts/*.sh` first.
- Topic/TF contract authority is `docs/topic_map.md`; if topic/TF behavior changes, update it in the same change.
- Launch-wrapper parity decisions are tracked in `docs/launch_script_parity.md`.
- Some README launch argument tables may be stale; trust launch files and wrapper `--help` output.

## Boundaries That Matter
- `src/platform/`: robot-specific adapters (`go2_bringup`, `go2_description`).
- `src/autonomy/`: robot-agnostic autonomy (`robot_nav`, `robot_sim`, `robot_safety`, `robot_lio`).
- `src/core_interfaces/`: shared message/service contracts (`robot_common_interfaces`, `unitree_*`).
- `src/simulation/`: worlds, models, scenario runtime tools.
- `src/vendors/`: vendored third-party code; avoid edits unless task explicitly targets vendor internals.
- Never hand-edit generated artifacts under `build/`, `install/`, or `log/`.

## Canonical Commands
- Full build order: `source /opt/ros/humble/setup.bash` -> `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` -> `source install/setup.bash`.
- Focused build: `colcon build --packages-select <pkg>` or `colcon build --packages-up-to <pkg>`.
- Tests: `colcon test` then `colcon test-result --all`.
- Sim entrypoint: `scripts/run_go2_sim.sh` (default `mapping`; `localization` requires `--map`).
- Real robot entrypoint: `scripts/run_go2_real.sh` (default `mapping`; `localization` requires `--map`; safety is on by default).

## Current Launch Contracts
- Shared wrapper options include `--map`, `--no-rviz`, `--use-lio`, `--no-stvl`, `--nav-profile`, `--use-route-server`, `--route-graph-file`, `--robot-name`, `--distortion-imu-topic`, and `--nav-delay`.
- Sim-only wrapper options include `--world`, `--gui`, `--no-gui`, and `--no-clean`.
- Real-only wrapper options include `--no-safety`, `--use-safety`, `--lidar-points-in-topic`, `--lidar-points-frame`, `--lidar-imu-in-topic`, `--recovery-ack-key-mask`, `--resume-key-mask`, `--abort-key-mask`, `--record-rosbag`, `--rosbag-dir`, and `--rosbag-topics`.
- `scripts/run_go2_sim.sh` keeps `--nav-delay` default `20.0`; SLAM starts after that delay and Nav2 starts 10 seconds later.
- `scripts/run_go2_real.sh` keeps `--nav-delay` default `0.0`; SLAM and Nav2 start together after that delay.

## Runtime Contracts You Must Preserve
- Command chain is `Nav2 /cmd_vel -> collision_monitor -> /cmd_vel_collision_free`.
- If `use_safety:=true`, `velocity_limiter` gates `/cmd_vel_collision_free -> /cmd_vel_safe`, and `go2_hw_bridge` must read `/cmd_vel_safe`.
- `collision_monitor` publishes to `/cmd_vel_collision_free` (from `robot_nav/config/collision_monitor_params.yaml`); do not bypass it.
- With default EKF fusion (`go2_bringup.launch.py`), `go2_hw_bridge` publishes `/odom_raw`; EKF publishes `/odom` and `odom -> base_footprint` TF.
- Canonical real-robot LiDAR relay path is `/utlidar/cloud` -> `/lidar/points` and `/utlidar/imu` -> `/lidar/imu` via `robot_lio`.
- `distortion_imu_topic` defaults to `/imu/data`; set it to `/lidar/imu` only when intentionally comparing LiDAR IMU timing.
- Optional LIO publishes `/lidar_odom`; feed EKF through `/lidar_odom_reliable`, not directly from `/lidar_odom`.

## Launch And Script Gotchas
- ROS setup scripts are not nounset-safe; keep `set +u` while sourcing setup files, then restore `set -u`.
- `run_go2_sim.sh` cleans stale ROS/Gazebo processes by default; pass `--no-clean` to keep existing sessions.
- `sim_bringup.launch.py` intentionally delays startup (`nav_delay_sec`, then Nav2 10s after SLAM); do not remove this without replacing the stabilization rationale.
- `go2_bringup.launch.py` accepts `nav_delay_sec` for parity but defaults to immediate navigation startup on real hardware.
- `nav2.launch.py` parameter selection depends on `use_stvl` + `nav_profile` (`default_safe`, `narrow_gap_1p0m`, or `forward_static`); avoid hardcoding a single params YAML.
- If you add/change launch-started processes, update `scripts/kill_ros_nodes.sh` in the same change.

## Coding Guidelines For This Workspace
- Use ASCII only in new/edited files.
- Keep comments clean and sparse: explain non-obvious integration constraints (TF ownership, topic contracts, hardware quirks), not obvious code flow.
- Use explicit function and variable names that reflect ROS intent (`cmd_vel_in`, `lidar_points_input_topic`, `use_safety`) and keep parameter naming consistent with existing launch/config conventions.
- For booleans, keep `use_*`, `enable_*`, or `start_*` naming; for topics, prefer `*_topic` or `*_input_topic`/`*_output_topic`.
- Preserve safety-critical defaults unless task explicitly changes behavior (watchdog timeouts, cmd gating, TF ownership).

## Documentation Rules
- Update `docs/topic_map.md` whenever topic names, TF ownership, command topics, or sensor relay paths change.
- Update `docs/launch_script_parity.md` whenever wrapper arguments or sim/real defaults change.
- Keep operational docs direct and ROS-specific; avoid marketing language and stale launch snippets.

## Minimum Validation Before Hand-off
- Build only affected packages.
- For launch/topic/TF/safety edits, run one relevant smoke launch or at least `ros2 launch <pkg> <file> --show-args` and check impacted topics/TF against `docs/topic_map.md`.
- For wrapper script edits, run `bash -n` and check the affected `--help` output.
