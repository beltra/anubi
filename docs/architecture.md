# Architecture

## Packages

```
src/
  platform/          robot adapters
    go2_bringup        go2_hw_bridge (Sport API <-> ROS), real-robot launch
    go2_description    Go2 URDF
    g1_bringup         g1_state_bridge, cmd_vel_to_amo / cmd_vel_to_sonic, real-robot launch
    g1_description     G1 URDF + Mid-360 mount
    g1_locomotion      AMO walking policy driver (separate process)
  autonomy/          robot-agnostic
    robot_nav          Nav2, slam_toolbox, EKF, LiDAR processing; configs and launch
    robot_nav_recovery Nav2 recovery plugins for legged robots
    robot_safety       velocity_limiter, joystick_safety, tilt_monitor
    robot_lio          LiDAR relays, DLIO integration, lidar_odom_relay
    robot_sim          Go2 sim bringup (sim + SLAM + Nav2 + RViz)
  core_interfaces/   messages (robot_common_interfaces, unitree_api, unitree_go)
  simulation/        go2_sim, g1_sim, g1_isaac, d1_sim (stub), sensor_models, sim_worlds, sim_scenarios
  vendors/           CHAMP, DLIO, livox_ros_driver2 (third-party, keep edits minimal)
```

Code specific to one robot goes in `platform/` and `simulation/`. Anything under
`autonomy/` has to work unchanged on every robot. A new robot needs an adapter that
publishes the canonical topics below and consumes `/cmd_vel_safe`.

## Data flow

```
LiDAR ---> robot_lio relay ---> /lidar/points ---> distortion_corrector ---> /lidar/points_undistorted
                                                                               |-> STVL costmap, collision_monitor
                                                                               '-> pointcloud_to_laserscan -> /scan -> slam_toolbox

robot adapter ---> /odom_raw, /imu/data ---> EKF ---> /odom + TF odom->base_footprint
(DLIO, optional) ---> /lidar_odom ---> lidar_odom_relay ---> /lidar_odom_reliable ---> EKF

Nav2 /cmd_vel ---> collision_monitor ---> /cmd_vel_collision_free ---> velocity_limiter ---> /cmd_vel_safe ---> robot adapter
```

## TF ownership

| Transform | Owner |
|---|---|
| `map -> odom` | slam_toolbox |
| `odom -> base_footprint` | EKF (sim and real) |
| `base_footprint -> base_link` | robot adapter (body roll/pitch + standing height), static in sim |
| `base_footprint -> lidar_stabilized` | `lidar_level_frame_publisher` (gravity-leveled LiDAR frame) |
| `base_link -> sensors` | URDF via robot_state_publisher |

Each transform has exactly one publisher, so don't add a second one.

The full topic and frame contract is in [topic_map.md](topic_map.md). If you change a
topic or a TF owner, update that file in the same commit.

## Design decisions

- Sim and real use the same topic names, frames and wrapper options wherever they
  behave the same. Where hardware needs something different, it's an explicit
  wrapper option.
- `collision_monitor` only checks for collisions. On a real robot every command then
  goes through `velocity_limiter` (watchdog, joystick override, tilt e-stop) before the
  adapter. `--no-safety` skips that and is meant for bench tests.
- The EKF owns odometry. In live runs DLIO never publishes TF; its output reaches the
  EKF through `lidar_odom_relay`, which adds the covariances DLIO leaves at zero.
- SLAM, Nav2 and the collision monitor all read one cloud, `/lidar/points_undistorted`.
  It is self-filtered, mount-corrected and published in `lidar_stabilized`. When the
  IMU or odometry it depends on goes stale, the corrector drops the cloud instead of
  passing it through raw.
- The Unitree L1 and Livox Mid-360 scan in non-repetitive patterns, and one frame is
  too sparse for a 2D scan. On hardware, `scan_cloud_accumulator` combines a short
  window of frames before `pointcloud_to_laserscan`. STVL and the collision monitor
  still read the latest cloud directly.
- `nav2.launch.py` picks its params YAML from `use_stvl` and `nav_profile`, so don't
  hardcode one file.
- Bag replay is the one exception to TF ownership. The replay launches
  (`bag_perception*.launch.py`) have no body odometry, so the EKF can't run and DLIO
  publishes the odom TF itself. Live runs keep the EKF as the owner.
- Validation tools (MPPI benchmarks in `run_mppi_suite.sh`, scenario managers, replay)
  are opt-in and never change the default bringup.

## Maintenance rules

- If you add or rename a launched node, update `scripts/kill_ros_nodes.sh` in the same
  commit.
- If you change a wrapper option or default, update
  [launch_script_parity.md](launch_script_parity.md).
