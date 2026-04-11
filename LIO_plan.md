# Lightweight LIO Integration Plan for Anubi

## Summary

- Implement a new **odometry-first LIO path** that improves short-horizon odometry reliability and precision without replacing the current SLAM/Nav2 architecture.
- Keep `slam_toolbox` as the `map → odom` authority in phase 1, and add LIO as a new `/lidar_odom` source fused with body odom in the repo’s already-planned pattern from [PROJECT_SPECS.md](/ws/anubi/docs/PROJECT_SPECS.md) and [ekf_fusion_params.yaml](/ws/anubi/src/autonomy/robot_nav/config/ekf_fusion_params.yaml).
- Use **DLIO** as the primary implementation target because it is **MIT-licensed**, explicitly described upstream as a **lightweight** LIO, accepts `sensor_msgs/PointCloud2` plus a **6-axis IMU**, and has a ROS2 branch: <https://github.com/vectr-ucla/direct_lidar_inertial_odometry>.
- Keep an optional phase-2 branch for **ROS2 LIO-SAM** only after phase 1 is stable. It is **BSD-3-Clause** and tested on ROS2 Humble, but it is heavier and requires point `time`, `ring`, and a stronger IMU contract: <https://github.com/westonrobot/lio_sam>.
- Exclude from implementation:
  - **FAST-LIO / CT-LIO / Unitree point_lio_unilidar** because they are GPL-2.0 or ROS1-first, which conflicts with the chosen “avoid all GPL” direction: <https://github.com/hku-mars/FAST_LIO>, <https://github.com/chengwei0427/ct-lio>, <https://github.com/unitreerobotics/point_lio_unilidar>
  - **MOLA** because it is GPLv3: <https://docs.ros.org/en/ros2_packages/rolling/api/mola_lidar_odometry/__README.html>

## Key Changes

- Add a new robot-agnostic package, e.g. `src/autonomy/robot_lio`, that owns:
  - LIO launch/config
  - a thin integration wrapper around DLIO ROS2
  - topic remaps to publish canonical `/lidar_odom`
  - an explicit `publish_tf:=false` default so LIO never competes for `odom → base_*`
- Change Go2 hardware bringup so fusion mode becomes the default LIO-ready contract:
  - `go2_hw_bridge` publishes **body odom on `/odom_raw`**, not `/odom`
  - EKF owns `/odom` and `odom → base_footprint`
  - LIO publishes `/lidar_odom`
  - Nav2 and `slam_toolbox` continue consuming `/odom`
- Add the missing preparatory sensor path for LIO:
  - keep `/lidar/points` for existing SLAM/costmaps
  - add a dedicated LiDAR IMU topic, default `/lidar/imu`
  - use the **LiDAR-integrated IMU** for LIO, not the Go2 body IMU on `/imu/data`, because upstream DLIO explicitly needs LiDAR/IMU time sync and recommends integrated IMU setups
  - preserve original point cloud fields end-to-end; verify that `time` survives the relay path before adopting DLIO
- Add a canonical LiDAR-IMU TF contract:
  - retain `lidar_link` as the canonical cloud frame
  - add a fixed `lidar_imu_link` frame for the LiDAR’s own IMU
  - populate the static transform from Unitree’s published LiDAR-to-IMU translation in `unilidar_sdk` docs/repo: <https://github.com/unitreerobotics/unilidar_sdk>
  - do not overload existing `imu_link`, which currently represents the body IMU
- Keep `robot_nav` robot-agnostic:
  - no LIO-specific Go2 assumptions in `robot_nav`
  - only add launch hooks and topic expectations that any future robot can satisfy
- Add a hardware-first rollout:
  - `use_lio:=false` by default until the sensor contract is verified on the real robot
  - once verified, `use_lio:=true` launches DLIO + EKF fusion
  - keep simulation launch behavior unchanged initially
- Add a phase-2 optional path for full LIO-SLAM:
  - evaluate `westonrobot/lio_sam` only after phase 1
  - if adopted, run it in a separate launch path that replaces `slam_toolbox` rather than competing with it
  - do not mix two `map → odom` publishers in one stack

## Preliminary Modifications Required Before LIO

- Update documentation first because repo truth is drifting:
  - [topic_map.md](/ws/anubi/docs/topic_map.md) still says `/imu/data` is absent on hardware, but [go2_hw_bridge.cpp](/ws/anubi/src/platform/go2_bringup/src/go2_hw_bridge.cpp) already publishes it
  - document `/odom_raw`, `/lidar_odom`, `/lidar/imu`, and TF ownership before wiring LIO
- Verify the real Go2 LiDAR bridge exposes what DLIO needs:
  - point cloud still available as `/lidar/points` after relay
  - LiDAR IMU available from Unitree path such as `/unilidar/imu`
  - timestamps are synchronized enough for LIO
  - the `PointCloud2` still contains usable relative timing; if not, stop phase 1 and add a small relay/adapter that preserves or reconstructs the required timing field
- Add a Go2-specific relay/adapter node if the current `topic_tools relay` path is too thin:
  - input: Unitree raw cloud + LiDAR IMU
  - output: canonical `/lidar/points` + `/lidar/imu`
  - preserve field names, stamps, and frame IDs
- Do not promise simulation parity yet:
  - current sim LiDAR is an instantaneous GPU lidar snapshot and does not clearly model the same LiDAR IMU/time-field contract
  - phase 1 should be hardware-validated first
  - only after hardware success should we decide whether sim needs ring/time/IMU extensions for LIO regression coverage

## Test Plan

- Sensor contract checks on hardware:
  - confirm `/lidar/points` frequency, frame, and field list
  - confirm `/lidar/imu` frequency, frame, and timestamp behavior
  - confirm static TF chain includes `base_link → lidar_link → lidar_imu_link`
- LIO standalone validation:
  - launch DLIO alone and verify stable pose output at idle, during slow indoor walking, and during turns
  - record CPU and memory on Orin NX with `tegrastats`; acceptance target is enough headroom to keep Nav2, SLAM, and safety nodes responsive
  - compare drift and pose smoothness against current body odom on repeated indoor loops
- Fusion validation:
  - EKF inputs are `/odom_raw` and `/lidar_odom`
  - only EKF publishes `/odom` and `odom → base_footprint`
  - no duplicate TF publishers remain
- Navigation regression checks:
  - existing `slam_toolbox` mapping/localization still works
  - Nav2 consumes fused `/odom` without interface changes
  - safety path and joystick override semantics remain unchanged
- Phase-2 gate:
  - only evaluate LIO-SAM if phase-1 metrics are still insufficient
  - require verified `time` + `ring` fields and acceptable Orin NX runtime before starting that branch

## Assumptions And Defaults

- Licensing default: **avoid all GPL-family runtime code**, not just GPL-3.0.
- Phase 1 default: **DLIO odometry-only integration** using ROS2 branch and MIT license.
- Phase 2 default: **optional LIO-SAM evaluation**, not part of the first implementation pass.
- LIO IMU default: use the **LiDAR’s own IMU** on `/lidar/imu`; keep body IMU on `/imu/data` for existing stabilization/fusion consumers.
- TF ownership default:
  - `slam_toolbox` owns `map → odom`
  - EKF owns `odom → base_footprint`
  - URDF/static publishers own sensor extrinsics
  - LIO publishes odometry messages only unless a later dedicated mode explicitly changes that
