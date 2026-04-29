# Dynamic Obstacle Pipeline

This branch adds an opt-in predictive LiDAR dynamic obstacle pipeline for the Go2 Nav2 stack.
The runtime pipeline now launches C++ executables from `robot_nav`; the older
Python scripts remain in `src/autonomy/robot_nav/scripts/` as reference.

## Nodes

Launch with `robot_nav/launch/dynamic_obstacles.launch.py`.

| Node | Input | Output | Purpose |
|---|---|---|---|
| `dynamic_obstacle_preprocessor` | `/lidar/points_undistorted` | `/dynamic_obstacles/points_world` | Transforms LiDAR points into the fixed `map` frame at the cloud timestamp, removes floor-height returns, and voxel-downsamples. |
| `dynamic_cluster_detector` | `/dynamic_obstacles/points_world`, `/map`, TF | `/dynamic_obstacles/clusters` | Clusters LiDAR points, rejects mapped static structure and shelf-like fragments, masks LiDAR shadow cones behind confirmed movers, and emits moving clusters with world-frame velocity estimates. |
| `dynamic_obstacle_tracker` | `/dynamic_obstacles/clusters` | `/dynamic_obstacles/tracks` | Maintains persistent tracks, uses gated EKF-style correction, predicts straight-line future poses by default, and drops stale internal state when the cluster stream times out. |
| `dynamic_obstacle_costmap_bridge` | `/dynamic_obstacles/tracks` | `/dynamic_obstacles/predicted_occupancy`, `/dynamic_obstacles/markers` | Converts predicted paths into a Nav2 `PointCloud2` marking source and RViz markers. |

## Nav2 Hook

The default STVL Nav2 profiles add `/dynamic_obstacles/predicted_occupancy` as a marking-only observation source in the local costmap. STVL temporal decay removes stale predicted points, and the bridge watchdog publishes an empty cloud plus delete markers when tracker data stops.

Because the detector works on `/dynamic_obstacles/points_world` in `map`, cluster speed is estimated after ego-motion compensation by TF: static obstacles stay near zero speed while moving obstacles keep their world speed independent from the robot linear velocity. The detector still applies ego-motion rejection thresholds to suppress residual artifacts caused by TF jitter, stale fallback transforms, or pure viewpoint rotation.

RViz can display:

- `/dynamic_obstacles/markers` as `MarkerArray` for boxes, IDs, speeds, and predicted paths.
- `/dynamic_obstacles/predicted_occupancy` as `PointCloud2` for the exact costmap marking cloud.

## Run In Simulation

```bash
scripts/run_go2_sim.sh mapping --dynamic-obstacles --nav-profile forward_static --rviz
```

The default direct simulation launch now uses `sim_worlds/worlds/industrial_dynamic.world`:

```bash
ros2 launch robot_sim sim_bringup.launch.py use_dynamic_obstacles:=true use_rviz:=true
```

The world is a 20 x 15 m primitive SDF warehouse with perimeter walls, two long shelf rows, a 4 x 4 m clear spawn area centred at `(0, 0)`, a primary navigation aisle along +X, and a dedicated south forklift aisle. It ships with `robot_nav/maps/industrial_dynamic.yaml` and `.pgm`; the map includes only static walls, racks, pallets, and fixed barriers.

| Validation scenario | World actor |
|---|---|
| Straight crossing | `worker_straight_crossing` crosses the +X corridor at x=4 m, loops in about 11.5 s, and starts after 1.0 s. |
| Curved approach / CTRV | `worker_curved_approach` follows a curved arc from the north aisle toward the main corridor, loops in 16 s, and starts after 4.5 s. |
| Multi-target visibility | `worker_straight_crossing`, `worker_curved_approach`, and `worker_diagonal_open_area` have overlapping windows with staggered delays. |
| Track deletion / non-collision pass | `worker_diagonal_open_area` diagonally crosses the open spawn area over about 9 m and loops in 19 s. |
| Larger, faster obstacle | `forklift_aisle_patrol` uses a 2.8 x 1.35 m rigid primitive body and patrols the south aisle in 27.4 s. |

To trigger robot-in-motion validation automatically after startup:

```bash
ros2 launch robot_sim sim_bringup.launch.py \
  use_dynamic_obstacles:=true \
  auto_nav_goal:=true \
  auto_nav_goal_delay_sec:=38.0
```

## Run On Hardware

```bash
scripts/run_go2_real.sh mapping --dynamic-obstacles --use-safety
```

The hardware pipeline reuses the existing LiDAR contract:

- Raw cloud: `/utlidar/cloud`
- Canonical cloud: `/lidar/points`
- Processed cloud: `/lidar/points_undistorted`

The pipeline does not modify the LiDAR driver. Remap with existing `run_go2_real.sh --lidar-points-in-topic <topic>` if the raw driver topic differs.

## Parameters

Simulation parameters: `src/autonomy/robot_nav/config/dynamic_obstacles_sim.yaml`

Hardware parameters: `src/autonomy/robot_nav/config/dynamic_obstacles_hw.yaml`

Important tuning groups:

- Preprocessor: `fixed_frame`, `min_z`, `max_z`, `voxel_size`, `max_points`, `max_latest_tf_age_sec`
- Detector: `cluster_tolerance`, `min_cluster_points`, `static_inflation_radius`, `min_dynamic_speed`, `max_dynamic_speed`, `min_dynamic_cluster_long_axis`, `min_dynamic_hits`, `min_consecutive_dynamic_hits`, `static_shape_min_length`, `wall_sliver_min_length`, `ego_velocity_rejection`, `ego_speed_ratio_threshold`, `ego_artifact_min_speed_ratio`, `ego_artifact_max_speed_ratio`, `ego_artifact_direction_cosine`, `shadow_management_enabled`, `shadow_persistence_sec`, `shadow_angle_margin_rad`, `shadow_range_padding`
- Tracker: `confirmation_hits`, `max_misses`, `max_publish_misses`, `min_publish_confidence`, `gating_threshold`, `prediction_horizon_sec`, `straight_line_prediction`
- Bridge: `watchdog_timeout_sec`, `occupancy_resolution`, `footprint_padding`, `uncertainty_scale`, `max_prediction_padding`, `min_track_confidence`, `max_points`

Hardware defaults use lower point budgets and shorter prediction horizons than simulation to reduce Jetson Orin 8GB load.
