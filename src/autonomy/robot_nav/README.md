# robot_nav

Robot-agnostic navigation package for Nav2 + slam_toolbox.

## Data flow

```
/lidar/points (PointCloud2)
  -> distortion_corrector
  -> /lidar/points_undistorted
  -> pointcloud_to_laserscan
  -> /scan (LaserScan)
  -> slam_toolbox
  -> /map + TF(map->odom)

/odom + TF(odom->base_footprint) -> slam_toolbox + Nav2
Nav2 /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> collision_monitor -> /cmd_vel_collision_free
```

## Launch files

| File | Purpose |
|---|---|
| `launch/slam.launch.py` | Distortion correction, cloud-to-scan conversion, slam_toolbox |
| `launch/nav2.launch.py` | Nav2 stack + collision monitor |
| `launch/ekf.launch.py` | Optional EKF |
| `launch/nav_realrobot.launch.py` | Real robot orchestration (EKF + SLAM + Nav2) |

## Main config files

| File | Purpose |
|---|---|
| `config/nav2_params.yaml` | Default STVL profile (Smac lattice planner + MPPI omni) |
| `config/nav2_params_narrow_gap_1p0m.yaml` | Narrow-gap STVL profile |
| `config/nav2_params_forward_static.yaml` | Comparison profile: SmacPlanner2D + DiffDrive MPPI + map-backed global costmap |
| `config/nav2_params_2d.yaml` | 2D obstacle-layer profile (NavFn + DWB) |
| `config/nav2_params_navfn.yaml` | STVL + NavFn profile |
| `config/slam_toolbox_params.yaml` | SLAM tuning |
| `config/scan_from_cloud.yaml` | PointCloud2 -> LaserScan conversion |
| `config/distortion_corrector.yaml` | Point cloud deskew and gravity alignment |
| `config/collision_monitor_params.yaml` | Collision monitor zones and sources |

## SLAM modes

### Mapping

```bash
ros2 launch go2_bringup go2_bringup.launch.py slam_mode:=mapping
```

Save map:

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/path/to/map'}"
```

### Localization

```bash
ros2 launch go2_bringup go2_bringup.launch.py \
  slam_mode:=localization \
  map:=/path/to/map
```

## Nav2 profile selection

`launch/nav2.launch.py` selects params with:

| `use_stvl` | `nav_profile` | YAML used |
|---|---|---|
| `true` | `default_safe` (default) | `nav2_params.yaml` |
| `true` | `narrow_gap_1p0m` | `nav2_params_narrow_gap_1p0m.yaml` |
| `true` | `forward_static` | `nav2_params_forward_static.yaml` |
| `false` | *(any)* | `nav2_params_2d.yaml` |

`forward_static` is an opt-in comparison profile (SmacPlanner2D + DiffDrive MPPI + map-backed global
costmap). It does not replace `default_safe`.

An explicit `params_file:=...` argument overrides profile selection when non-empty.

### Optional route_server

`nav2.launch.py`, `sim_bringup.launch.py`, and `go2_bringup.launch.py` accept
`use_route_server:=true` to start the `route_server` scaffold with its own lifecycle manager.
Off by default. Pass `route_graph_file:=<path>` for route graph experiments.

The convenience scripts expose the same controls:

```bash
scripts/run_go2_sim.sh mapping --nav-profile forward_static
scripts/run_go2_real.sh mapping --nav-profile forward_static
scripts/run_go2_sim.sh mapping --use-route-server --route-graph-file /opt/ros/humble/share/nav2_route/graphs/sample_graph.geojson
```

## Integration contract

Required frames:

- `map`
- `odom`
- `base_footprint`
- `base_link`
- `lidar_link`
- `lidar_stabilized`

Required topics:

- `/lidar/points`
- `/lidar/points_undistorted`
- `/scan`
- `/odom`
- `/cmd_vel_nav`
- `/cmd_vel`
