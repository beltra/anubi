# AGENTS.md

## Scope and Source of Truth
- This is a ROS 2 Humble + Gazebo Fortress `colcon` workspace rooted at `/ws/anubi`.
- Trust executable wiring first: launch files in `src/**/launch/*.launch.py`, then helper scripts in `scripts/*.sh`.
- Topic/TF contract authority is `docs/topic_map.md` (kept current and explicitly marked authoritative).

## Package Boundaries (important)
- `src/platform/`: robot-specific adapters (`go2_bringup`, `go2_description`).
- `src/autonomy/`: robot-agnostic autonomy (`robot_nav`, `robot_sim`, `robot_safety`, `robot_lio`).
- `src/core_interfaces/`: shared msgs/contracts (`robot_common_interfaces`, `unitree_*`).
- `src/simulation/`: sim assets and runtime scenario tools.
- `src/vendors/`: vendored third-party packages; avoid incidental edits.

## Canonical Run Paths
- Sim stack: `scripts/run_go2_sim.sh` (default `mapping`; supports `localization --map ...`).
- Real robot stack: `scripts/run_go2_real.sh` (default `mapping`; supports `--use-safety`, `--use-lio`, custom LiDAR input topics).
- Manual sim entrypoint: `ros2 launch robot_sim sim_bringup.launch.py`.
- Manual hardware entrypoint: `ros2 launch go2_bringup go2_bringup.launch.py`.

## Build/Test Commands (exact)
- Source ROS first, then build:
  - `source /opt/ros/humble/setup.bash`
  - `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
  - `source install/setup.bash`
- Focused build: `colcon build --packages-select <pkg>` or `--packages-up-to <pkg>`.
- Tests: `colcon test` then `colcon test-result --all`.

## High-Signal Runtime Contracts
- Nav command path is `Nav2 /cmd_vel -> collision_monitor -> /cmd_vel_collision_free`.
- If `use_safety:=true`, `velocity_limiter` gates `/cmd_vel_collision_free -> /cmd_vel_safe`; `go2_hw_bridge` switches to `/cmd_vel_safe`.
- If EKF fusion is enabled (default in `go2_bringup.launch.py`), `go2_hw_bridge` publishes `/odom_raw` and EKF owns `/odom` + `odom->base_footprint` TF.
- Real-robot canonical LiDAR inputs default to `/utlidar/cloud` and `/unilidar/imu`, relayed by `robot_lio` to `/lidar/points` and `/lidar/imu`.

## Launch/Process Maintenance Rule
- If you add/change launch-started nodes/processes, update `scripts/kill_ros_nodes.sh` in the same change so cleanup remains complete.

## Launch And Script Gotchas
- ROS setup scripts are not nounset-safe; keep `set +u` while sourcing setup files, then restore `set -u`.
- `run_go2_sim.sh` cleans stale ROS/Gazebo processes by default; pass `--no-clean` to keep existing sessions.
- `sim_bringup.launch.py` intentionally delays startup (`nav_delay_sec`, then Nav2 10s after SLAM); do not remove this without replacing the stabilization rationale.
- `nav2.launch.py` parameter selection depends on `use_stvl` + `nav_profile` (`default_safe` or `narrow_gap_1p0m`); avoid hardcoding a single params YAML.
- If you add/change launch-started processes, update `scripts/kill_ros_nodes.sh` in the same change.

## Coding Guidelines For This Workspace
- Use ASCII only in new/edited files.
- Keep comments clean and sparse: explain non-obvious integration constraints (TF ownership, topic contracts, hardware quirks), not obvious code flow.
- Use explicit function and variable names that reflect ROS intent (`cmd_vel_in`, `lidar_points_input_topic`, `use_safety`) and keep parameter naming consistent with existing launch/config conventions.
- For booleans, keep `use_*`, `enable_*`, or `start_*` naming; for topics, prefer `*_topic` or `*_input_topic`/`*_output_topic`.
- Preserve safety-critical defaults unless task explicitly changes behavior (watchdog timeouts, cmd gating, TF ownership).

## Minimum Validation Before Hand-off
- Build only affected packages.
- For launch/topic/TF/safety edits, run one relevant smoke launch (`scripts/run_go2_sim.sh` or `scripts/run_go2_real.sh`) and check impacted topics/TF against `docs/topic_map.md`.
