# EKF Integration Plan (robot_localization)

Purpose: add lidar + body odometry fusion safely, without breaking existing Nav2/SLAM behavior.

## 1) Target Runtime Topology

```text
[robot adapter] body odom  -> /odom_raw
[robot adapter] imu        -> /imu/data
[lidar odom node]          -> /lidar_odom  (optional phase-1)
[EKF]                      -> /odom (+ TF odom->base_footprint)
[slam_toolbox]             -> TF map->odom
[Nav2]                     -> consumes /odom and TF
```

Current repository scaffolding added:
- `src/robot_nav/launch/ekf.launch.py`
- `src/robot_nav/config/ekf_fusion_params.yaml`

## 2) Required Topic/TF Changes

## 2.1 Required inputs

1. `/odom_raw` (`nav_msgs/Odometry`)  
   Source: robot adapter (`go2_hw_bridge` / future `d1_hw_bridge`)

2. `/imu/data` (`sensor_msgs/Imu`)  
   Source: hardware bridge or IMU adapter node

3. `/lidar_odom` (`nav_msgs/Odometry`, optional in first pass)

## 2.2 Required outputs

1. `/odom` (fused odometry)  
   EKF output remapped from `/odometry/filtered`

2. TF `odom->base_footprint`  
   EKF should own this when fusion is active

## 2.3 TF ownership rules

- `map->odom`: slam_toolbox
- `odom->base_footprint`: EKF
- `base_footprint->base_link->sensor frames`: URDF via robot_state_publisher

## 3) Node Responsibilities

- Robot adapter node should publish **raw** odom only.
- Robot adapter should support `publish_tf=false` when EKF is active.
- EKF should publish fused odom and odom TF.
- Lidar odom node should not publish map TF; only odom-relative odometry topic.

## 4) Go2 vs D1 Config Split

Recommended files:

- Common:
  - `robot_nav/config/common/nav2_common.yaml`
  - `robot_nav/config/common/slam_common.yaml`
  - `robot_nav/config/common/ekf_common.yaml`

- Go2 overlay:
  - `robot_nav/config/go2/nav2_overlay.yaml`
  - `robot_nav/config/go2/ekf_overlay.yaml`
  - `go2_bringup/config/go2_params.yaml`

- D1 overlay:
  - `robot_nav/config/d1/nav2_overlay.yaml`
  - `robot_nav/config/d1/ekf_overlay.yaml`
  - `d1_bringup/config/d1_params.yaml`

Overlay content should include:
- footprint polygon
- velocity/acceleration limits
- odom/imu covariance
- lidar scan slicing/filtering parameters
- topic remaps if native SDK names differ

## 5) Step-by-Step Integration Sequence

## Phase 1: Body odom + IMU fusion

1. Set adapter output to `/odom_raw`.
2. Ensure adapter can disable raw TF (`publish_tf=false`).
3. Publish IMU on `/imu/data` with realistic covariance.
4. Launch EKF:
   - `ros2 launch robot_nav ekf.launch.py output_odom_topic:=/odom`
5. Keep lidar odom disabled initially.

## Phase 2: Add lidar odometry

1. Add one lidar odom node (scan matcher / ICP / NDT / LIO adapter).
2. Publish its output to `/lidar_odom` in odom frame.
3. Enable `odom1` in EKF config.
4. Tune covariance balance (`odom0`, `imu0`, `odom1`) with bag replay.

## Phase 3: D1 rollout

1. Implement `d1_hw_bridge` with same external topic contract.
2. Provide D1 overlay configs.
3. Reuse same EKF launch with D1 params file override.

## 6) Validation Procedure (rosbag + RViz)

## 6.1 Record (or replay) topics

Record:
- `/odom_raw`
- `/lidar_odom`
- `/odometry/filtered` (or `/odom` if remapped)
- `/imu/data`
- `/tf`, `/tf_static`
- `/scan` and/or `/unilidar/cloud`
- `/cmd_vel`

Example:
```bash
ros2 bag record /odom_raw /lidar_odom /odometry/filtered /imu/data /tf /tf_static /scan /unilidar/cloud /cmd_vel
```

## 6.2 TF sanity checks

1. Confirm only one `odom->base_footprint` publisher exists.
2. Confirm `map->odom` comes from slam_toolbox only.
3. Confirm no TF jumps on EKF startup.

Example:
```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

## 6.3 Trajectory comparison

In RViz:
- Show three paths: raw body odom, lidar odom, fused odom.
- Verify fused path reduces drift without introducing lag/oscillation.

## 6.4 Acceptance criteria

- Nav2 plans and controls normally with fused `/odom`.
- No duplicate TF warnings.
- EKF diagnostics stable (no frequent rejected measurements).
- Fused odom smooth and consistent during turns and lateral motion.

## 7) Known Pre-Integration Blockers

1. Raw odom covariance currently missing in Go2 adapter.
2. Real bringup does not yet enforce `/imu/data` normalization.
3. D1 adapter package is not present.
4. `d1_sim` is still a stub.

