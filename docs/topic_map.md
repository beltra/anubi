# Topic Map — Anubi Autonomy Stack

Last updated: 2026-03-29
Branch: `feature/stvl-local-costmap`

This document is the authoritative reference for all ROS 2 topics in the anubi stack.
"Sim name" and "HW name" columns are filled only when the two differ.

---

## Sensor Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | gazebo_bridge (sim) / Unitree L1 LiDAR (HW) | `pointcloud_to_laserscan`, Nav2 costmap layers | same | same | Frame: `lidar_link`. Rate: 10 Hz. |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` | `slam_toolbox`, `obstacle_layer` | same | same | Derived from `/lidar/points` horizontal slice (±10 mm around equator). |
| `/imu/data` | `sensor_msgs/Imu` | gazebo_bridge (sim only) | EKF (`footprint_to_odom_ekf`) | published | absent | SIM-ONLY. HW bridge reads IMU internally from SportModeState; does not republish on a ROS topic. |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_publisher_system` (sim only) | `robot_state_publisher`, CHAMP | published | absent | SIM-ONLY. HW uses Sport Mode API; no joint-level state is published on real robot. |

---

## Odometry and Localisation Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/odom/raw` | `nav_msgs/Odometry` | CHAMP `quadruped_controller` (via Gazebo bridge) | `footprint_to_odom_ekf` | published | absent | SIM-ONLY. CHAMP raw wheel-equivalent odometry before EKF fusion. |
| `/odom` | `nav_msgs/Odometry` | `footprint_to_odom_ekf` (sim) / `go2_hw_bridge` (HW) | `slam_toolbox`, Nav2 | same | same | frame_id=`odom`, child_frame_id=`base_footprint`. Rate: 50 Hz. |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | Nav2 `map_server`, RViz | same | same | Published during mapping mode; used as localization reference in localization mode. |

---

## TF Transforms

| Transform | Publisher | Sim | HW | Notes |
|-----------|-----------|-----|----|-------|
| `map → odom` | `slam_toolbox` | same | same | Published by slam_toolbox during active SLAM. Static identity fallback available via `publish_map_to_odom_tf:=true` launch arg (for testing without SLAM). |
| `odom → base_footprint` | `footprint_to_odom_ekf` (sim) / `go2_hw_bridge` (HW) | same | same | Different publisher, identical transform semantics. |
| `base_footprint → base_link` | `static_transform_publisher` (identity, 0 0 0) | same | same | Nav2 planar convention: base_footprint at ground level, base_link at `standing_height` (0.28 m). |
| `base_link → lidar_link` | URDF / `robot_state_publisher` | same | same | xyz=(0.28945, 0, −0.046825), rpy=(0, 0, 0). |
| `base_link → imu_link` | URDF / `robot_state_publisher` | same | same | Identity (co-located with base_link). |

---

## Command Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 `controller_server` | `velocity_limiter` (if `use_safety:=true`) or locomotion directly | same | same | Nav2 Humble publishes Twist (not TwistStamped). |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | `velocity_limiter` | CHAMP `quadruped_controller` (sim) / `go2_hw_bridge` (HW) | same | same | Output of velocity_limiter safety gate. Only active when `use_safety:=true`. |

---

## Hardware-Only Topics (No Sim Equivalent)

| Topic | Type | Publisher | Consumer(s) | Notes |
|-------|------|-----------|-------------|-------|
| `/sportmodestate` | `unitree_go/SportModeState` | Go2 internal bridge | `go2_hw_bridge` | ~50 Hz. Provides position, velocity, IMU, and foot contacts. Source of HW odometry. |
| `/lowstate` | `unitree_go/LowState` | Go2 internal bridge | `go2_hw_bridge` | Provides battery data (`BmsState.soc`). |
| `/api/sport/request` | `unitree_api/Request` | `go2_hw_bridge` | Go2 onboard controller | Sport Mode API command channel. `go2_hw_bridge` calls `Move()` and `StopMove()`. |

---

## Status and Safety Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/robot_status` | `robot_common_interfaces/RobotStatus` | `go2_hw_bridge` | `velocity_limiter` | same | same | Published at 1 Hz. Contains `estop_active` flag and battery percentage. In sim, `go2_hw_bridge` is absent; `velocity_limiter` is configured with `status_topic: ""` (watchdog-only mode). |

---

## Simulation-Only Internal Topics

| Topic | Type | Publisher | Consumer(s) | Notes |
|-------|------|-----------|-------------|-------|
| `/joint_group_effort_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | CHAMP `quadruped_controller` | `ign_ros2_control` effort controller | CHAMP → ign_ros2_control joint effort command channel. No HW equivalent. |
| `/debug/lidar/points_filtered` | `sensor_msgs/PointCloud2` | `cloud_self_filter` (opt-in) | RViz | Debug only. Enabled via `launch_cloud_filter:=true`. Self-filtered view of `/lidar/points`. |

---

## Navigation Topics (Nav2 Internal)

These follow standard Nav2 Humble topic naming; listed here for completeness.

| Topic | Type | Notes |
|-------|------|-------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 goal input (RViz → Nav2). |
| `/plan` | `nav_msgs/Path` | Global planner output. |
| `/local_plan` | `nav_msgs/Path` | DWB local planner trajectory. |
| `/costmap_raw` | `nav2_msgs/Costmap` | Raw costmap for debugging. |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller output (see Command Topics above). |
