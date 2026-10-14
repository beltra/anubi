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
| `/lidar/points_undistorted` | `sensor_msgs/PointCloud2` | `distortion_corrector` | `scan_cloud_accumulator` (HW), Nav2 STVL costmap, `collision_monitor` | same | same | Frame: `lidar_stabilized`. Single processed cloud: self-filtered, mount-righted, and optionally deskewed on HW. No ground/floor removal. On the G1 the mount righting is STATIC only (`gravity_align_attenuation: 0.0`): the head is gait-stabilized by the locomotion controller, so injecting pelvis-IMU roll/pitch made the walking floor tilt WORSE (2.76 deg vs 1.75 deg median). Near-field self-return guard `min_radius: 0.55` m on the G1 (was 0.20) removes head/shoulder-band returns at 0.19-0.51 m; this is a documented 0.55 m blind sphere around the sensor that collision_monitor/STVL share. |
| `/lidar/points_registered` | `sensor_msgs/PointCloud2` | `dlio_odom_node` (remapped from `deskewed`) | `scan_cloud_accumulator` (G1) | absent | published | Per-scan cloud placed in `odom` by DLIO's own GICP registration (the same transform that builds the DLIO map), deskewed by DLIO. The G1 mapping scan is derived from it so the painted `/map` inherits DLIO-map-coherent wall geometry; it bypasses the distortion corrector, so the accumulator applies a near-body radius self-filter (0.45 m) instead of the body box. |
| `/lidar/points_scan_accumulated` | `sensor_msgs/PointCloud2` | `scan_cloud_accumulator` (HW real bringup) | `pointcloud_to_laserscan` | absent | published | Short rolling window (default 0.20 s / 3 frames) of `/lidar/points_undistorted` frames, each re-projected into the newest sensor pose through `odom` so accumulated points reflect the current pose. Used only to densify `/scan` for non-repetitive LiDAR patterns (Unitree L1, Livox Mid-360). On the G1 a yaw gate (`max_yaw_rate_rad_s`, 0.60 on G1-real; registered sweeps paint correctly through normal turns, only fast spins are dropped) and a rear-sector filter (`exclude_rear_sector_half_width_deg` 40 / `exclude_rear_sector_max_range` 3.0 m on G1-real, keeps a trailing operator out of the painted map) additionally shape the MAPPING path only. Graceful degradation: if older frames' TF lookups fail, the newest is still published, so output rate is locked to the LiDAR input rate. Nav2 STVL and collision monitoring still consume the latest corrected cloud directly. |
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
| `/lidar_odom_reliable` | `nav_msgs/Odometry` | `lidar_odom_relay` | EKF fusion node | optional | optional | `/lidar_odom` with injected diagonal covariances. Active only when `use_lio:=true`. This is the topic the EKF consumes. The relay's position-jump detector evaluates displacement over >= `nominal_dt` windows (not per message) so 100 Hz IMU-propagated DLIO streams are judged at scan cadence; G1 launches inject tight x/y/yaw covariances so the EKF tracks DLIO without turn lag. |
| `/odom` | `nav_msgs/Odometry` | `footprint_to_odom_ekf` (sim) / `ekf_filter_node` (HW) | `slam_toolbox`, Nav2, `cmd_vel_to_sonic` (yaw source, `locomotion:=sonic`) | same | same | Fused odometry output. frame_id=`odom`, child_frame_id=`base_footprint`. G1 SONIC bridge reads yaw here to anchor its closed-loop body->world heading conversion. |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | Nav2 global costmap `StaticLayer`, RViz | same | same | Published during mapping mode; used as localization reference in localization mode and by STVL profiles for map-backed global planning. |

---

## TF Transforms

| Transform | Publisher | Sim | HW | Notes |
|-----------|-----------|-----|----|-------|
| `map -> odom` | `slam_toolbox` | same | same | Published by slam_toolbox during active SLAM. Static identity fallback available via `publish_map_to_odom_tf:=true` launch arg (for testing without SLAM). |
| `odom -> base_footprint` | EKF (`footprint_to_odom_ekf` in sim / `ekf_filter_node` on HW) | same | same | EKF owns this TF in both environments. Offline bag replay is the documented exception: on the Go2 flow (`scripts/replay_bag_perception.sh`) DLIO owns `odom -> base_link` directly; on the G1 flow (`scripts/replay_bag_perception_g1.sh`) the DLIO estimate owns `odom -> base_footprint` so the odom floor datum matches the live contract; it is rebroadcast from the 100 Hz `/lidar_odom` topic by `odom_to_tf` (DLIO itself sends TF only per scan, ~8 Hz, which left TF consumers up to ~120 ms stale). `odom_to_tf` de-duplicates non-increasing header stamps (`dedup_stamps`, default on): DLIO republishes ~17% of poses under an already-used stamp (scan-match corrections) plus rare zero stamps, which would otherwise be rebroadcast as non-monotonic TF; the node holds each pose one step and emits the LAST of a duplicate-stamp group. The EKF path (`lidar_odom_relay` -> `/lidar_odom_reliable`) does NOT de-duplicate: robot_localization resolves same-stamp measurements itself and de-duplicating there added latency that degraded turn tracking. See ADR 0012. |
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
| `/lidar/points` | `robot_lio` relay of `/utlidar/cloud` | `robot_lio` RESTAMPING relay of `/livox/lidar` (`cloud_restamp_relay`, frame `lidar_link`): header.stamp rewritten to host clock minus one sweep because the Mid-360 timebase drifts ~20 ms/s vs the host; per-point timestamps untouched (DLIO reconciles internally) |
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
G1-real derives the MAPPING scan from DLIO's registered cloud: `/scan` feeds
slam_toolbox only (collision_monitor and STVL consume `/lidar/points_undistorted`
directly), so `g1_bringup` points the scan accumulator at `/lidar/points_registered`
(`scan_cloud_in_fixed_frame:=true`, plain 3-frame concatenation of world-anchored
sweeps, 0.45 m near-body radius filter) with a 0.60 rad/s yaw gate
(`scan_accumulation_max_yaw_rate`, drops only fast spins) whose trip mode is
`single` (`scan_accumulation_yaw_gate_mode`): a gated cycle still emits the
newest registered sweep alone instead of publishing nothing, so `/scan` holds
>= 6 Hz through sustained spins (was 3.6 Hz with 1.5 s gaps) without thickening
the accumulated walls. A rear-sector operator filter (rear +/-40 deg within
3.0 m, mapping path only -- STVL and collision_monitor keep the full unfiltered
view behind the robot) also shapes this path.
Each painted sweep is thus placed by the same GICP registration that builds the DLIO
map, which is what keeps the 2D `/map` walls coherent with the DLIO 3D map. The G1
sim keeps accumulation off (gpu_lidar clouds are dense and instantaneous), so
`/lidar/points_registered` and `/lidar/points_scan_accumulated` exist on G1-real only.

### G1 Isaac Sim Adapter (g1_isaac)

The Isaac Sim variant (`src/simulation/g1_isaac`, full detail in its README)
reuses the canonical contract with these publishers (all on ROS_DOMAIN_ID 42,
sim time from the Isaac host `/clock`):

| Canonical topic / TF | Isaac publisher |
|----------------------|-----------------|
| `/lidar/points`, `/lidar/imu` | `frame_relay_node` of the host `/livox/lidar` + `/livox/imu_raw` (frame -> `lidar_link`; stamps and per-point `time`/`intensity` fields preserved; host IMU already m/s^2, no `imu_rescale`) |
| `/imu/data` | `frame_relay_node` of `/pelvis/imu_raw` (synthesized pelvis IMU; frame `imu_link` by default -- real G1 uses `imu`, team decision pending) |
| `/odom_raw` | `frame_relay_node` of `/odom_gt` (ground-truth pelvis world pose, all-zero covariance; child `base_footprint`) |
| `base_footprint -> base_link` TF | g1_isaac bringup static TF (standing height 0.80; real ~0.78, calibrate) |
| `base_link -> lidar_link` TF | g1_isaac bringup static TF (upright Mid-360 mount, pitch 0.0401) |
| `/odom` + `odom -> base_footprint` TF | `footprint_to_odom_ekf` (g1_isaac_nav.launch.py; absolute x/y/yaw from `/odom_raw`, no DLIO -- the Isaac lidar stream cannot support it) |
| `map -> odom` | `slam_toolbox` (sim params copy `slam_toolbox_params_g1_isaac.yaml`, transform_timeout 2 s -- see file header) |
| `/cmd_vel_safe` consumer | none in sim (no robot_safety): `cmd_vel_to_sonic` reads `/cmd_vel_collision_free` directly (`g1_sonic_sim_params.yaml`) |

## Unitree A2-W Wheeled Variant

The A2-W (Pro) reuses every canonical topic/TF in this document (plan and geometry:
`docs/a2w_embodiment_plan.md`). Differences are its two Hesai JT128 LiDARs, merged into the
single canonical cloud by `robot_nav cloud_merger`, and a diff-drive Nav2 profile (no vy).
The sim base is kinematic (Fortress VelocityControl, legs fixed in the stand pose); the
real adapter is `go2_hw_bridge` on the A2 Sport API topics. The real path is not yet
validated on hardware.

| Canonical topic / TF | A2-W sim publisher (`a2w_sim`) | A2-W real publisher (`a2w_bringup`) |
|----------------------|--------------------------------|-------------------------------------|
| per-sensor clouds | `ros_ign_bridge`: `/lidar/front/points` (frame `front_lidar_link`), `/lidar/rear/points` (frame `rear_lidar_link`) | two JT128 drivers (`front/rear_lidar_points_input_topic`, defaults `/lidar_points_front`, `/lidar_points_rear`; verify on the robot) |
| `/lidar/points` | `cloud_merger` (frame `lidar_link`, stamp of the front sweep, RELIABLE QoS so DLIO's reliable subscription matches) | `cloud_merger` (same node and params) |
| `/odom_raw` | `ros_ign_bridge` of Fortress OdometryPublisher (ground truth) | `go2_hw_bridge` from `/lf/sportmodestate` |
| `/imu/data` | `ros_ign_bridge` of the Gazebo IMU (frame `imu_link`) | `go2_hw_bridge` (SportModeState imu_state) |
| `/odom` + `odom -> base_footprint` TF | `footprint_to_odom_ekf` (odom0 `/lidar_odom_reliable` with use_lio, else `/odom_raw`; imu0 yaw rate) | `ekf_filter_node` (`ekf_fusion_params_lio.yaml` / `ekf_fusion_params.yaml`) |
| `base_footprint -> base_link` TF | static, z 0.497 | `go2_hw_bridge` (body roll/pitch, standing_height 0.497) |
| `base_link -> {front,rear}_lidar_link, lidar_link, imu_link` TF | `robot_state_publisher` (`a2w.urdf.xacro`, fixed legs) | same; plus an optional `<side>_lidar_link -> <driver frame>` alias when the driver frame_id differs |
| `base_footprint -> lidar_stabilized` TF | `lidar_level_frame_publisher` (lidar_link at z 0.0843 in base_link) | same |
| `/lidar/points_undistorted` | `distortion_corrector` (`distortion_corrector_a2w.yaml`; sim overrides: no IMU, no gravity align) | `distortion_corrector` (`distortion_corrector_a2w.yaml`) |
| `/scan` | `pointcloud_to_laserscan` (`scan_from_cloud_a2w.yaml`, target `lidar_stabilized`) | same |
| `/lidar_odom` | DLIO (use_lio, default true; `dlio_params_a2w_sim.yaml`, IMU `/imu/data`) | DLIO (use_lio, default true; `dlio_params_a2w_real.yaml`, IMU `/imu/data`) |
| `/cmd_vel_safe` consumer | `ros_ign_bridge` -> `/model/<robot>/cmd_vel` (VelocityControl) | `go2_hw_bridge` -> SportClient Move on `/api/sport/request` |
| `/cmd_vel_safe` consumer, `locomotion:=policy` | `a2w_policy_controller` -> `/model/<robot>/joint/<joint>/cmd_pos` (12 legs, JointPositionController) and `.../cmd_vel` (4 wheels, JointController), bridged ROS -> Ignition | n/a (the onboard sport controller) |
| `/joint_states` | `locomotion:=policy` only: `ros_ign_bridge` of `/model/<robot>/joint_state` (feeds `robot_state_publisher`); kinematic mode has no movable joints | `go2_hw_bridge` |

`/lidar/imu` is not used on the A2-W: the body IMU is the single inertial reference for
DLIO and the EKF. The Go2 startup gait sequence (ClassicWalk) is disabled in
`a2w_params.yaml`; the operator puts the robot in its wheeled stand mode first.

## Optional Topics (route_server, disabled by default)

Active only when `use_route_server:=true` is passed to `nav2.launch.py`.
The safety chain (`/cmd_vel_nav -> velocity_smoother -> /cmd_vel -> /cmd_vel_collision_free -> /cmd_vel_safe`) is unchanged.

| Topic | Type | Notes |
|-------|------|-------|
| `/route` | `nav2_msgs/Route` | Route graph path computed by `route_server`. |
| `/route_graph` | `visualization_msgs/MarkerArray` | Route graph visualization (RViz). |
