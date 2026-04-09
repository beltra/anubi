# robot_nav

Robot-agnostic Nav2 configuration and launch files.
**No robot-specific code is in this package.**

---

## Architecture

```
/lidar/points (PointCloud2)
    ├──► distortion_corrector ──► /lidar/points_undistorted
    │          ├──► pointcloud_to_laserscan ──► /scan (LaserScan)
    │          │          └──► slam_toolbox ──► /map + TF(map→odom)
    │          └──► Nav2 local/global costmaps (obstacle detection)

/odom + TF(odom→base_footprint) ──► slam_toolbox + Nav2
Nav2 /cmd_vel ──► [go2_hw_bridge] ──► Go2 hardware
```

## Robot Physical Dimensions (Unitree Go2)

| Dimension | Trunk only | Full robot (with legs) |
|---|---|---|
| Length (X) | 0.440 m | **0.700 m** |
| Width (Y) | 0.120 m | **0.310 m** |
| Height (Z) | 0.100 m | 0.400 m (standing) |
| Standing height (ground to body centre) | — | **0.280 m** |
| LiDAR height above ground | — | **0.233 m** |
| Circumscribed radius | — | **0.383 m** |

The URDF `base_link` collision box models the trunk only (440 × 120 × 100 mm).
The Nav2 footprint parameter uses the FULL robot dimensions including legs.

## Packages Included

| Package | Licence | Role |
|---------|---------|------|
| slam_toolbox | LGPL-2.1 | 2D SLAM: map + map→odom TF |
| pointcloud_to_laserscan | BSD-3-Clause | UniLidar PointCloud2 → /scan |
| robot_nav `distortion_corrector.py` | Apache-2.0 | PointCloud2 deskew using `/odom` + optional `/imu/data` |
| nav2_bringup + components | Apache-2.0 | Navigation stack |

**slam_toolbox LGPL-2.1** — compatible with industrial/commercial projects under
dynamic linking (the default in ROS 2 / colcon workspaces).

---

## Launch Files

| File | Purpose |
|------|---------|
| `slam.launch.py` | distortion_corrector + pointcloud_to_laserscan + slam_toolbox |
| `nav2.launch.py` | Nav2 navigation nodes |
| `ekf.launch.py` | Optional robot_localization EKF (not enabled by default) |

## Config Files

| File | Purpose |
|------|---------|
| `config/slam_toolbox_params.yaml` | SLAM algorithm tuning |
| `config/scan_from_cloud.yaml` | PointCloud2 → LaserScan conversion params |
| `config/distortion_corrector.yaml` | LiDAR deskew / undistortion tuning |
| `config/nav2_params.yaml` | DWB controller, NavFn planner, costmaps |
| `config/ekf_fusion_params.yaml` | Optional EKF template for `/odom_raw` + IMU + lidar odom fusion |

---

## SLAM Modes

### Mapping (default)
```bash
ros2 launch go2_bringup go2_bringup.launch.py slam_mode:=mapping
# Drive around to build map, then save:
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/path/to/map'}"
```

### Localization (after mapping)
```bash
ros2 launch go2_bringup go2_bringup.launch.py \
    slam_mode:=localization \
    map:=/path/to/map
```

---

## Costmap Configuration

- **Local costmap**: PointCloud2 from `/lidar/points_undistorted`, rolling window 4×4 m
- **Global costmap**: PointCloud2 from `/lidar/points_undistorted` + static map layer, rolling window 30×30 m
- Obstacle height filter: 0.05 m – 2.0 m
- Footprint: polygon 0.70×0.31 m (full standing footprint including legs)
- Inflation radius: 0.58 m (local), 0.68 m (global)

## 3D Obstacle Avoidance (STVL)

### What was changed
- local_costmap now uses spatio_temporal_voxel_layer instead of obstacle_layer
- The L1 LiDAR 3D point cloud is first deskewed to `/lidar/points_undistorted`, then feeds the voxel buffer
- No persistent 3D map is built; voxels decay after voxel_decay seconds (default 3.0 s)
- The global_costmap is unchanged (2D, static + inflation only)

### Launch arguments

| Argument | Default | Effect |
|---|---|---|
| `use_stvl` | `true` | Load STVL local costmap config (3D obstacle avoidance) |
| `use_stvl:=false` | — | Revert to 2D obstacle_layer (nav2_params_2d.yaml) |

### Tuning notes
- voxel_decay: how many seconds before an unseen obstacle disappears (default 3.0)
- voxel_size: voxel edge length in metres (default 0.05 m = 5 cm)
- max_obstacle_height: maximum z of points to mark (default 2.0 m)
- min_obstacle_height: minimum z of points to mark (default 0.05 m - filters ground returns)
- publish_voxel_map: set true to visualize the 3D voxel grid in RViz2

## Key Tuning Points

| Parameter | File | Value | When to change |
|-----------|------|-------|----------------|
| `max_laser_range` | `slam_toolbox_params.yaml` | 30.0 m | Large spaces → increase |
| `resolution` | `slam_toolbox_params.yaml` | 0.05 m | Precision vs memory trade-off |
| `max_vel_x` | `nav2_params.yaml` | 1.0 m/s | Increase when space allows |
| `inflation_radius` | `nav2_params.yaml` | 0.58 m (local), 0.68 m (global) | Reduce if paths too narrow |
| `min_height` / `max_height` | `scan_from_cloud.yaml` | −0.01 m / +0.01 m | Adjust for LiDAR tilt or mounting height |
| `min_obstacle_height` / `max_obstacle_height` | `nav2_params.yaml` | 0.05 m / 2.0 m | Adjust costmap height filter |

## DWB Local Planner — Tuning Notes

The DWB critic weights are set for a slow indoor quadruped.  The values below
were chosen to prevent a failure mode where the robot hangs near its goal:

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `trans_stopped_velocity` | 0.05 m/s | Threshold below which robot is considered "stopped translating". Must be well below `min_vel_x` to avoid premature RotateToGoal activation. |
| `RotateToGoal.scale` | 16.0 | Half the weight of PathDist/PathAlign (32.0). Prevents heading-alignment from dominating and deadlocking trajectories near the goal. |
| `BaseObstacle.scale` | 0.2 | Enough weight to keep robot out of occupied cells without overwhelming directional critics. |
| `failure_tolerance` | 0.5 s | 10 control cycles (at 20 Hz) before recovery is triggered. Provides margin for momentary plan failures without premature recovery cascades. |

**Recovery hang budget** (using the default Nav2 BT): if failure_tolerance fires,
the BT runs up to 6 retries of: ClearCostmap + Spin(1.57 rad) + **Wait(5 s)** +
BackUp(0.3 m). Worst case: ~90 s before navigation aborts.  The fixes above
reduce the frequency of recovery triggering near the goal.

## Simulation Stability Tuning (2026-04-09)

Recent simulation regressions showed frequent falls when Nav2 generated lateral
or high angular commands faster than the gait controller could track.

Applied tuning changes:
- DWB switched to non-holonomic commanding (`holonomic: false`, `vy_samples: 1`)
- Reduced DWB limits to `max_vel_x: 0.75`, `max_vel_theta: 0.6`
- Reduced DWB accel/decel limits to smoother values (`acc_lim_x: 0.6`, `acc_lim_theta: 0.8`)
- Reduced behavior server spin dynamics (`max_rotational_vel: 0.6`, `rotational_acc_lim: 0.8`)
- Aligned `velocity_smoother` limits with DWB (`[0.75, 0.0, 0.6]`)
- Reduced CHAMP gait caps in sim (`max_linear_velocity_x: 0.9`, `max_angular_velocity_z: 0.7`, no lateral)

Changed files:
- `go2_sim/config/gait.yaml`
- `robot_nav/config/nav2_params.yaml`
- `robot_nav/config/nav2_params_2d.yaml`
- `robot_nav/config/nav2_params_navfn.yaml`

Rationale:
- Keep Nav2 command envelope inside stable sim gait limits.
- Avoid lateral strafing commands that are a common fall trigger for this setup.
- Limit rotational transients to reduce foot slip and tipping during tight maneuvers.

## Speed Profile Update (2026-04-09)

To better match observed real Go2 pace while keeping simulation stable, Nav2 was
updated from an ultra-conservative profile to a moderate-fast quasi-holonomic one.

Updated command envelope:
- DWB: `max_vel_x: 1.2`, `max_vel_y: 0.2`, `max_vel_theta: 1.0`
- DWB: `holonomic: true` with bounded lateral sampling (`vy_samples: 6`)
- velocity_smoother aligned to the same limits and accel/decel envelope
- behavior_server spin limits raised to `max_rotational_vel: 1.0`
- CHAMP gait caps in sim raised to `1.4 m/s` forward, `0.25 m/s` lateral, `1.0 rad/s` yaw

Rationale:
- Go2 is not strictly differential-drive; moderate lateral motion is valid.
- Prior non-holonomic + low-speed profile was safer but unrealistically slow.
- This profile increases throughput while keeping bounded transients.

## Slip Mitigation Notes (2026-04-09)

Online patterns checked:
- CHAMP docs and robot configs emphasize that large `stance_duration` increases step displacement.
- Community quadruped setups commonly require careful foot friction + inertia/contact tuning in Gazebo.
- Unitree Go2 URDF notes mention collision-model adjustments are often required for simulation stability.

What was changed in this repo:
- Reduced step-length tendency while keeping speed: lower `stance_duration` and `swing_height` in CHAMP gait.
- Increased foot/ground friction and disabled unnecessary leg self-collision in sim URDF.
- Tightened accel/transient limits in Nav2 + velocity_smoother while keeping a fast top-speed envelope.

URDF feet validation:
- Foot kinematic chain (`*_lower_leg_link` -> fixed `*_FOOT`) and nominal foot geometry are structurally consistent.
- Main issue was contact parameterization (friction/self-collision), not foot-link topology.

---

## Adding a New Robot

To keep this package robot-agnostic, new robots must preserve the nav-stack
contract below. If these frame/topic names are kept stable, the shared files
`config/nav2_params.yaml`, `config/slam_toolbox_params.yaml`, and
`config/scan_from_cloud.yaml` do not need changes.

### Required Frames

- `base_footprint` — ground projection, `z=0` at contact
- `base_link` — torso center at standing height
- `lidar_stabilized` — gravity-leveled LiDAR frame for 2D scan extraction
- `lidar_link` — LiDAR optical center, horizontal mount (`rpy="0 0 0"`)
- `imu_link` — IMU frame
- `odom` — odometry origin (published by robot bridge)
- `map` — SLAM map origin (published by `slam_toolbox`)

### Required Topics

- `/lidar/points` — LiDAR `sensor_msgs/PointCloud2` (or remapped to this)
- `/lidar/points_undistorted` — deskewed LiDAR `sensor_msgs/PointCloud2`
- `/scan` — 2D `sensor_msgs/LaserScan` from `pointcloud_to_laserscan`
- `/odom` — `nav_msgs/Odometry`
- `/cmd_vel` — `geometry_msgs/Twist` command input

### D1 Ultra Integration Pattern

1. Create `src/d1_description/urdf/d1.urdf.xacro` with the required frame names.
2. Create `src/d1_bringup/` with `robot_interface.launch.py` that remaps native
   odometry/velocity topics to `/odom` and `/cmd_vel`.
3. If the LiDAR topic differs, remap it to `/lidar/points` in launch, or add
   a `d1_scan_from_cloud.yaml` and keep outward nav topics unchanged.
4. Keep shared nav files unchanged:
   `config/nav2_params.yaml`, `config/slam_toolbox_params.yaml`,
   `config/scan_from_cloud.yaml`.
