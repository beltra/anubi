# Topic Map - Anubi Autonomy Stack

Last updated: 2026-05-06
Branch: `feat/sim2real`

This document is the authoritative reference for all ROS 2 topics in the anubi stack.
"Sim name" and "HW name" columns are filled only when the two differ.

---

## Sensor Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | gazebo_bridge (sim) / `robot_lio` relay of `/utlidar/cloud` (HW) | `distortion_corrector`, `dlio_odom_node` (opt.) | same | same | Frame: `lidar_link`. Raw sensor output; RViz fixed frame `map` applies TF, so a real flat floor should still look flat in `map`. **Not consumed directly by Nav2 or SLAM**. |
| `/lidar/points_undistorted` | `sensor_msgs/PointCloud2` | `distortion_corrector` | `pointcloud_to_laserscan`, Nav2 STVL costmap, `collision_monitor` | same | same | Frame: `lidar_stabilized` when IMU state is available. Single processed cloud: self-filtered, trunk-stabilized, and optionally deskewed on HW. No ground/floor removal. Input to all downstream consumers. |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` | `slam_toolbox`, `obstacle_layer` | same | same | Derived from `/lidar/points_undistorted` horizontal slice (+/-12 cm around equator). |
| `/imu/data` | `sensor_msgs/Imu` | gazebo_bridge (sim) / `go2_hw_bridge` (HW) | EKF (`footprint_to_odom_ekf` in sim, `ekf_filter_node` on HW), `distortion_corrector` | published | published | Body IMU in `imu_link`. Default correction IMU. Set `distortion_imu_topic:=/lidar/imu` to compare LiDAR IMU timing. |
| `/lidar/imu` | `sensor_msgs/Imu` | `robot_lio` relay (sim/HW) | optional LIO backend | same | same | Canonical LiDAR-integrated IMU topic for LIO. |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_publisher_system` (sim only) | `robot_state_publisher`, CHAMP | published | absent | SIM-ONLY. HW uses Sport Mode API; no joint-level state is published on real robot. |

---

## Odometry and Localisation Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/odom_raw` | `nav_msgs/Odometry` | CHAMP odom bridge (sim) / `go2_hw_bridge` (HW) | EKF fusion node | same | same | Raw body odometry input to EKF. |
| `/lidar_odom` | `nav_msgs/Odometry` | optional LIO backend (via `robot_lio`) | `lidar_odom_relay` | optional | optional | Disabled by default (`use_lio:=false`). All-zero covariances (DLIO limitation) — never feed directly to the EKF. |
| `/lidar_odom_reliable` | `nav_msgs/Odometry` | `lidar_odom_relay` | EKF fusion node | optional | optional | `/lidar_odom` with injected diagonal covariances. Active only when `use_lio:=true`. This is the topic the EKF consumes. |
| `/odom` | `nav_msgs/Odometry` | `footprint_to_odom_ekf` (sim) / `ekf_filter_node` (HW) | `slam_toolbox`, Nav2 | same | same | Fused odometry output. frame_id=`odom`, child_frame_id=`base_footprint`. |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | Nav2 global costmap `StaticLayer`, RViz | same | same | Published during mapping mode; used as localization reference in localization mode and by STVL profiles for map-backed global planning. |

---

## TF Transforms

| Transform | Publisher | Sim | HW | Notes |
|-----------|-----------|-----|----|-------|
| `map -> odom` | `slam_toolbox` | same | same | Published by slam_toolbox during active SLAM. Static identity fallback available via `publish_map_to_odom_tf:=true` launch arg (for testing without SLAM). |
| `odom -> base_footprint` | EKF (`footprint_to_odom_ekf` in sim / `ekf_filter_node` on HW) | same | same | EKF owns this TF in both environments. |
| `base_footprint -> base_link` | `static_transform_publisher` (sim) / `go2_hw_bridge` (HW) | same | same | Sim uses fixed standing-height offset; HW includes body roll/pitch plus standing height. |
| `base_link -> lidar_link` | URDF / `robot_state_publisher` | rpy=(0, -0.26180, 0) | rpy=(pi, -0.26180, 0) | xyz=(0.28945, 0, -0.046825). Sim uses a right-side-up Fortress workaround; HW is upside-down with ~15 deg forward pitch. |
| `base_link -> imu_link` | URDF / `robot_state_publisher` | same | same | Identity (co-located with base_link). |
| `base_footprint -> lidar_stabilized` | `lidar_level_frame_publisher` | same | same | Dynamic TF at IMU rate. Gravity-levelled LiDAR frame: tracks physical LiDAR position while holding zero roll/pitch. Used as frame_id for undistorted clouds when gravity_align is enabled. |
| `lidar_link -> lidar_imu_link` | URDF / `robot_state_publisher` (sim) / `robot_lio` static TF (HW) | URDF | robot_lio TF | Unitree UniLidar IMU offset: xyz=(-0.007698, -0.014655, 0.00667). Sim: always in URDF. HW: published by `robot_lio` (always included from `go2_bringup`; absent when using `go2_hardware.launch.py` alone). |

---

## Command Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 `controller_server` | Nav2 `velocity_smoother` | same | same | Nav2 bringup remaps controller output from `cmd_vel` to `cmd_vel_nav`. |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 `velocity_smoother` | `collision_monitor` | same | same | Smoothed Nav2 command. Nav2 Humble uses Twist (not TwistStamped). |
| `/cmd_vel_collision_free` | `geometry_msgs/Twist` | `collision_monitor` | `velocity_limiter` safety arbiter | same | same | collision_monitor output after proximity checks. In real production runs this must not go directly to hardware. |
| `/cmd_vel_manual_collision_free` | `geometry_msgs/Twist` | optional manual source after collision filtering | `velocity_limiter` (disabled by default) | optional | optional | Reserved command-mux input. Must be collision-filtered before enabling. |
| `/cmd_vel_assisted_collision_free` | `geometry_msgs/Twist` | optional assisted teleop source after collision filtering | `velocity_limiter` (disabled by default) | optional | optional | Reserved command-mux input. Must be collision-filtered before enabling. |
| `/cmd_vel_recovery_collision_free` | `geometry_msgs/Twist` | optional recovery source after collision filtering | `velocity_limiter` (disabled by default) | optional | optional | Reserved command-mux input. Must be collision-filtered before enabling. |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | `velocity_limiter` | CHAMP `quadruped_controller` (sim) / `go2_hw_bridge` (HW) | same | same | Final command after watchdog, joystick override, and tilt estop gating. Default real-robot hardware input. |

---

## Hardware-Only Topics (No Sim Equivalent)

| Topic | Type | Publisher | Consumer(s) | Notes |
|-------|------|-----------|-------------|-------|
| `/sportmodestate` | `unitree_go/SportModeState` | Go2 internal bridge | `go2_hw_bridge` | ~50 Hz. Provides position, velocity, IMU, and foot contacts. Source of HW odometry. |
| `/lowstate` | `unitree_go/LowState` | Go2 internal bridge | `go2_hw_bridge` | Provides battery data (`BmsState.soc`). |
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | Go2 internal bridge | `robot_lio` relay | Raw LiDAR pointcloud input on the real robot. Relayed to `/lidar/points`. |
| `/utlidar/imu` | `sensor_msgs/Imu` | Go2 internal bridge | `robot_lio` relay | Raw LiDAR IMU input on the real robot. Relayed to `/lidar/imu` by default. |
| `/api/sport/request` | `unitree_api/Request` | `go2_hw_bridge` | Go2 onboard controller | Sport Mode API command channel. `go2_hw_bridge` publishes `RecoveryStand`, `StandUp`, `ClassicWalk(true)`, plus runtime `Move()` and `StopMove()`. |

---

## Status and Safety Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/robot_status` | `robot_common_interfaces/RobotStatus` | `go2_hw_bridge` | `velocity_limiter` | absent | published | Published at 1 Hz. Contains hardware estop/watchdog flag and battery percentage. In sim, `go2_hw_bridge` is absent; `velocity_limiter` is configured with `status_topic: ""` (watchdog-only mode). |
| `/wirelesscontroller` | `unitree_go/WirelessController` | Go2 internal bridge | `joystick_safety`, `tilt_monitor` recovery ack | absent | published | Hardware controller axes and key bitmask. Any active axis or non-command key triggers `/joystick_override`. Resume and abort key masks are configurable and must be calibrated per controller. |
| `/joystick_override` | `std_msgs/Bool` | `joystick_safety` | `velocity_limiter` | absent | published | True while operator override is latched. Any operator motion cancels Nav2 and holds `/cmd_vel_safe` at zero. Resume clears override and resends the latest `/goal_pose`; abort clears the active goal and leaves Nav2 idle. |
| `/safety/tilt_estop` | `std_msgs/Bool` | `tilt_monitor` | `velocity_limiter` | absent | published | Latched true when body roll/pitch exceeds the configured estop threshold; clears only below clear threshold with calibrated controller ack. |
| `/safety/command_source` | `std_msgs/String` | `velocity_limiter` | operators/diagnostics | same | same | Current final arbiter state: `nav`, `estop`, `tilt_estop`, `joystick_override`, `watchdog`, or `waiting_for_cmd`. |
| `/diagnostics/lidar_correction` | `diagnostic_msgs/DiagnosticArray` | `distortion_corrector` when `publish_diagnostics:=true` | operators / diagnostics tools | opt-in | opt-in | Lightweight counters/timing only: deskew applied, gravity alignment applied, nearest IMU/odom age, processing time, processed/passthrough counts, and self-filter count. Disabled by default. |

---

## Simulation-Only Internal Topics

| Topic | Type | Publisher | Consumer(s) | Notes |
|-------|------|-----------|-------------|-------|
| `/joint_group_effort_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | CHAMP `quadruped_controller` | `ign_ros2_control` effort controller | CHAMP -> ign_ros2_control joint effort command channel. No HW equivalent. |
| `/debug/lidar/points_filtered` | `sensor_msgs/PointCloud2` | `cloud_self_filter` (opt-in) | RViz | Debug only. Enabled via `launch_cloud_filter:=true`. Self-filtered view of `/lidar/points`. |

---

## Navigation Topics (Nav2 Internal)

These follow standard Nav2 Humble topic naming; listed here for completeness.

| Topic | Type | Notes |
|-------|------|-------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 goal input (RViz -> Nav2). |
| `/plan` | `nav_msgs/Path` | Global planner output. |
| `/local_plan` | `nav_msgs/Path` | Local planner trajectory (MPPI by default). |
| `/costmap_raw` | `nav2_msgs/Costmap` | Raw costmap for debugging. |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 controller output before velocity smoothing (see Command Topics above). |
| `/speed_limit` | `nav2_msgs/SpeedLimit` | Optional controller speed-limit input. No default publisher. |

## Optional Topics (route_server, disabled by default)

Active only when `use_route_server:=true` is passed to `nav2.launch.py`.
The safety chain (`/cmd_vel_nav -> velocity_smoother -> /cmd_vel -> /cmd_vel_collision_free -> /cmd_vel_safe`) is unchanged.

| Topic | Type | Notes |
|-------|------|-------|
| `/route` | `nav2_msgs/Route` | Route graph path computed by `route_server`. |
| `/route_graph` | `visualization_msgs/MarkerArray` | Route graph visualization (RViz). |
