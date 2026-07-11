# Topic Map - Anubi Autonomy Stack

Last updated: 2026-07-03
Branch: `feat/g1` (G1 humanoid variant; off `feat/sim2real`)

This document is the authoritative reference for all ROS 2 topics in the anubi stack.
"Sim name" and "HW name" columns are filled only when the two differ. The main tables
below describe the Go2; the **Unitree G1 Hardware Variant** section near the end gives
the G1-specific publishers/frames (only those differ -- the canonical contract is shared).

---

## Sensor Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | gazebo_bridge (sim) / `robot_lio` relay of `/utlidar/cloud` (HW) | `distortion_corrector`, `dlio_odom_node` (opt.) | frame: `lidar_link` | frame: `utlidar_lidar` by default | Raw sensor output. HW keeps the driver `frame_id` and publishes an identity `lidar_link -> utlidar_lidar` TF alias; the HW `lidar_link` orientation is set to the measured Unitree driver frame. **Not consumed directly by Nav2 or SLAM**. |
| `/lidar/points_undistorted` | `sensor_msgs/PointCloud2` | `distortion_corrector` | `scan_cloud_accumulator` (HW), Nav2 STVL costmap, `collision_monitor` | same | same | Frame: `lidar_stabilized` when IMU state is available. Single processed cloud: self-filtered, trunk-stabilized, and optionally deskewed on HW. No ground/floor removal. |
| `/lidar/points_scan_accumulated` | `sensor_msgs/PointCloud2` | `scan_cloud_accumulator` (HW real bringup) | `pointcloud_to_laserscan` | absent | published | Short rolling window (default 0.20 s / 3 frames) of `/lidar/points_undistorted` frames, each re-projected into the newest sensor pose through `odom` so accumulated points reflect the current pose. Used only to densify `/scan` for the non-repetitive Unitree L1 pattern. Graceful degradation: if older frames' TF lookups fail, the newest is still published, so output rate is locked to the LiDAR input rate. Nav2 STVL and collision monitoring still consume the latest corrected cloud directly. |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` | `slam_toolbox`, `obstacle_layer` | same | same | Sim derives directly from `/lidar/points_undistorted`; HW derives from `/lidar/points_scan_accumulated` to avoid sparse radial scans from single non-repetitive L1 frames. |
| `/imu/data` | `sensor_msgs/Imu` | gazebo_bridge (sim) / `go2_hw_bridge` (HW) | EKF (`footprint_to_odom_ekf` in sim, `ekf_filter_node` on HW), `distortion_corrector` | published | published | Body IMU in `imu_link`. Default correction IMU. Set `distortion_imu_topic:=/lidar/imu` to compare LiDAR IMU timing. |
| `/lidar/imu` | `sensor_msgs/Imu` | `robot_lio` relay (sim/HW) | optional LIO backend | same | same | Canonical LiDAR-integrated IMU topic for LIO. |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_publisher_system` (sim) / `go2_hw_bridge` (HW, decimated from `/lowstate.motor_state`) | `robot_state_publisher`, CHAMP (sim only) | published @ ~200 Hz | published @ ~50 Hz | HW path maps Unitree motor indices (FR=0..2, FL=3..5, RR=6..8, RL=9..11) to CHAMP-style URDF joint names. Decimation is timer-driven; rate set by `joint_states_rate` parameter on `go2_hw_bridge`. RViz uses these to articulate the leg links defined in `go2_description/urdf/leg.xacro`. |

---

## Odometry and Localisation Topics

| Topic | Type | Publisher | Consumer(s) | Sim | HW | Notes |
|-------|------|-----------|-------------|-----|----|-------|
| `/odom_raw` | `nav_msgs/Odometry` | CHAMP odom bridge (sim) / `go2_hw_bridge` (HW) | EKF fusion node | same | same | Raw body odometry input to EKF. |
| `/lidar_odom` | `nav_msgs/Odometry` | optional LIO backend (via `robot_lio`) | `lidar_odom_relay` | optional | optional | Default OFF on the Go2 (`use_lio:=false`); **default ON for the G1 real robot** (`use_lio:=true`, DLIO is its primary odometry). All-zero covariances (DLIO limitation) - never feed directly to the EKF. |
| `/lidar_odom_reliable` | `nav_msgs/Odometry` | `lidar_odom_relay` | EKF fusion node | optional | optional | `/lidar_odom` with injected diagonal covariances. Active only when `use_lio:=true`. This is the topic the EKF consumes. |
| `/odom` | `nav_msgs/Odometry` | `footprint_to_odom_ekf` (sim) / `ekf_filter_node` (HW) | `slam_toolbox`, Nav2, `cmd_vel_to_sonic` (yaw source, `locomotion:=sonic`) | same | same | Fused odometry output. frame_id=`odom`, child_frame_id=`base_footprint`. G1 SONIC bridge reads yaw here to anchor its closed-loop body->world heading conversion. |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | Nav2 global costmap `StaticLayer`, RViz | same | same | Published during mapping mode; used as localization reference in localization mode and by STVL profiles for map-backed global planning. |

---

## TF Transforms

| Transform | Publisher | Sim | HW | Notes |
|-----------|-----------|-----|----|-------|
| `map -> odom` | `slam_toolbox` | same | same | Published by slam_toolbox during active SLAM. Static identity fallback available via `publish_map_to_odom_tf:=true` launch arg (for testing without SLAM). |
| `odom -> base_footprint` | EKF (`footprint_to_odom_ekf` in sim / `ekf_filter_node` on HW) | same | same | EKF owns this TF in both environments. Offline bag replay (`scripts/replay_bag_perception.sh`) is the documented exception: DLIO owns `odom -> base_link` directly because no body odom is available. See ADR 0012. |
| `base_footprint -> base_link` | `static_transform_publisher` (sim) / `go2_hw_bridge` (HW) | same | same | Sim uses fixed standing-height offset; HW includes body roll/pitch plus standing height. |
| `base_link -> lidar_link` | URDF / `robot_state_publisher` | rpy=(0, -0.26180, 0) | rpy=(0, 2.88000, 0) | xyz=(0.28945, 0, -0.046825). Sim uses a right-side-up Fortress workaround; HW orientation is derived from matching `/utlidar/cloud` to `/utlidar/cloud_base`. |
| `lidar_link -> utlidar_lidar` | `robot_lio` static TF (HW default) | absent | identity | Driver frame alias for real `/utlidar/cloud` pointclouds. Identity is valid because HW `lidar_link` matches the measured driver frame. `scripts/run_go2_real.sh --lidar-points-frame <frame>` changes the child frame when the driver reports a different name. |
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
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | Go2 internal bridge | `robot_lio` relay | Raw LiDAR pointcloud input on the real robot. Default frame: `utlidar_lidar`. Relayed to `/lidar/points` with the header preserved. |
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
| `/recovery/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `robot_nav_recovery` BT nodes in `bt_navigator` | operators / diagnostics tools | same | same | Structured Nav2 recovery stage events and terminal TF/localization or recovery-exhaustion diagnostics. |

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
| `/local_costmap/costmap_raw` | `nav2_msgs/Costmap` | Local 2D costmap used by guarded recovery sector checks and behavior collision checking. |
| `/costmap_raw` | `nav2_msgs/Costmap` | Raw costmap for debugging when remapped by a Nav2 component. |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 controller output before velocity smoothing (see Command Topics above). |
| `/speed_limit` | `nav2_msgs/SpeedLimit` | Optional controller speed-limit input. No default publisher. |

## Unitree G1 Hardware Variant

The G1 humanoid reuses every canonical topic/TF in this document; only the robot-specific
PUBLISHERS differ (full detail in `docs/g1_embodiment.md`). The autonomy stack and the
EKF frame ownership (`odom -> base_footprint`) are identical to the Go2.

| Canonical topic / TF | Go2 publisher (HW) | G1 publisher (HW) |
|----------------------|--------------------|-------------------|
| `/lidar/points` | `robot_lio` relay of `/utlidar/cloud` | `robot_lio` relay of `/livox/lidar` (Livox Mid-360 driver, frame `lidar_link`) |
| `/lidar/imu` | `robot_lio` relay of `/utlidar/imu` | `robot_lio` relay of `/livox/imu_ms2` (= `imu_rescale` of `/livox/imu`, g -> m/s^2) |
| `/odom_raw` | `go2_hw_bridge` (SportModeState) | `g1_state_bridge` (DDS `rt/odommodestate`) |
| `/imu/data` | `go2_hw_bridge` (body IMU) | `g1_state_bridge` (DDS `rt/lowstate` pelvis IMU) |
| `/odom` + `odom -> base_footprint` TF | `ekf_filter_node` | `ekf_filter_node` (same robot-agnostic config) |
| `base_footprint -> base_link` TF | `go2_hw_bridge` | `g1_state_bridge` (body roll/pitch + standing height ~0.78 m, calibrate) |
| `base_link -> <sensor>` TF | `base_link -> lidar_link -> utlidar_lidar` (URDF + `robot_lio` alias) | `base_link -> mid360_link` (URDF) + identity `mid360_link -> lidar_link` alias (`g1_bringup`, for the driver cloud `frame_id`) |
| `base_footprint -> lidar_stabilized` TF | `lidar_level_frame_publisher` | `lidar_level_frame_publisher` (same node; G1 Mid-360 extrinsics) |
| `/lidar/points_undistorted` | `distortion_corrector` (`distortion_corrector.yaml`) | `distortion_corrector` (`distortion_corrector_g1.yaml`, G1 self-filter box; shared by sim) |
| `/lidar_odom` | DLIO (use_lio) | DLIO (use_lio, default true; `dlio_params_g1_real.yaml`) |
| `/cmd_vel_safe` consumer | `go2_hw_bridge` (Sport API) | `cmd_vel_to_amo` (AMO WebSocket :8766; caps vx0.5/vy0.4/yaw0.4) or, with `locomotion:=sonic`, `cmd_vel_to_sonic` (SONIC planner ticks, ZMQ PUB :5556; also SUBs `/odom` for closed-loop heading; caps per `sonic_profile`: slow_walk vx0.5/vy0.4/yaw0.4, walk vx0.8/vy0.4/yaw0.5) |

G1 hardware-only inputs (no Go2 equivalent): `/livox/lidar`, `/livox/imu`, `/livox/imu_ms2`,
and the Unitree DDS topics `rt/odommodestate` + `rt/lowstate` (read by `g1_state_bridge`).
The G1 has no Sport API (`/api/sport/request`); commands go to the selected low-level
controller: the AMO driver over WebSocket (`locomotion:=amo`, default) or the SONIC
whole-body controller (`g1_deploy_onnx_ref` on the Jetson) over ZMQ (`locomotion:=sonic`,
see `docs/g1_sonic_locomotion.md`). Exactly one controller may run at a time (single
writer on `rt/lowcmd`). The state side (`g1_state_bridge`, DLIO, EKF) is identical for both.
The G1 does NOT use scan accumulation: `/scan` is derived from single Mid-360 frames
(`accumulate_scan_cloud:=false`), so `/lidar/points_scan_accumulated` is absent on the G1
(unlike the Go2 real bringup); sim and G1-real share this behavior.

## Optional Topics (route_server, disabled by default)

Active only when `use_route_server:=true` is passed to `nav2.launch.py`.
The safety chain (`/cmd_vel_nav -> velocity_smoother -> /cmd_vel -> /cmd_vel_collision_free -> /cmd_vel_safe`) is unchanged.

| Topic | Type | Notes |
|-------|------|-------|
| `/route` | `nav2_msgs/Route` | Route graph path computed by `route_server`. |
| `/route_graph` | `visualization_msgs/MarkerArray` | Route graph visualization (RViz). |
