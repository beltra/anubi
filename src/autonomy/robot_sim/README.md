# robot_sim

Gazebo simulation bringup package for the Go2 autonomy stack.

---

## Overview

`sim_bringup.launch.py` starts the maintained Fortress + CHAMP simulation:

```
go2_sim/sim_champ.launch.py  (Ignition Gazebo + ros_ign_bridge + CHAMP + EKFs)
    +  distortion_corrector  (/lidar/points -> /lidar/points_undistorted)
    +  slam.launch.py        (pointcloud_to_laserscan + slam_toolbox, use_sim_time=true)
    +  nav2.launch.py        (Nav2 navigation stack, use_sim_time=true)
    +  RViz2                 (optional, reuses robot_nav/rviz/nav2.rviz)
```

`go2_hw_bridge` is **not** started. Simulation provides the same navigation-facing
topics (`/lidar/points`, `/scan`, `/odom`, `/cmd_vel`) through CHAMP + SLAM/Nav2.

TF chain in simulation:

```
map ─(slam_toolbox)─► odom ─(CHAMP EKF chain)─► base_footprint
                                                  ├─(lidar_level_frame_publisher)─► lidar_stabilized
                                                  └─(URDF)─► base_link
                                                                  ├──► lidar_link
                                                                  └──► imu_link
```

All Nav2 configuration is shared with the real robot (`robot_nav/`).
Simulation uses the same processed LiDAR topic contract as hardware. The only
mount difference is the Fortress gpu_lidar workaround: sim uses a right-side-up
`lidar_link` with `mount_rotation_mode:=direct`, while hardware uses the real
upside-down mount with `mount_rotation_mode:=inverse`.

---

## Quick Start

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash

# Full sim — industrial dynamic world (default), 2D map localization, headless
ros2 launch robot_sim sim_bringup.launch.py

# Warehouse world, slam_toolbox localization with a saved posegraph, Gazebo GUI + RViz enabled
ros2 launch robot_sim sim_bringup.launch.py \
    world:=$(ros2 pkg prefix sim_worlds)/share/sim_worlds/worlds/warehouse.world \
    world_name:=warehouse \
    slam_mode:=localization \
    map:=/ws/anubi/src/autonomy/robot_nav/maps/my_map \
    gui:=true \
    use_rviz:=true

# Dynamic-obstacle validation, with an optional Nav2 goal sent after startup
ros2 launch robot_sim sim_bringup.launch.py \
    use_dynamic_obstacles:=true \
    auto_nav_goal:=true \
    auto_nav_goal_delay_sec:=38.0

# Nav2-only restart entry point is not provided in the current tree.
```

---

## Launch Arguments

### `sim_bringup.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `industrial_dynamic.world` | Path to Gazebo world file |
| `world_name` | `industrial_dynamic` | Gazebo world name used by spawn and bridge topic paths |
| `robot_name` | `go2` | Spawned robot entity name |
| `slam_mode` | `localization` | `mapping` or `localization` |
| `map` | `industrial_dynamic.yaml` | Path to Nav2 `.yaml` map or slam_toolbox `.posegraph` file in localization mode |
| `use_rviz` | `false` | Launch RViz2 |
| `slam_params` | robot_nav default | slam_toolbox params YAML override |
| `world_init_z` | `0.375` m | Robot Z spawn height for CHAMP standing startup |
| `gui` | `false` | Show Gazebo gzclient GUI |
| `use_dynamic_obstacles` | `false` | Start the predictive dynamic obstacle pipeline |
| `auto_nav_goal` | `false` | Send a NavigateToPose goal after startup |
| `auto_nav_goal_delay_sec` | `38.0` | Delay before the optional validation goal |

## Obstacle Management

```bash
# Spawn a crate at (2, 1)
ros2 run sim_scenarios spawn_obstacle \
    --name box_1 --model obstacle_box --x 2.0 --y 1.0 --z 0.5

# Delete it
ros2 run sim_scenarios spawn_obstacle --delete box_1

# Load a pre-defined scenario (crowded_room or narrow_corridor)
ros2 run sim_scenarios scenario_manager \
    --ros-args -p scenario_file:=crowded_room
```

---

## Known Limitations

- Fortress + CHAMP is the maintained simulation path.
- AgiBot D1 Ultra model (`simulation/d1_sim/`) is a stub — chassis dimensions are placeholders.
    Launch with `robot:=d1` is supported once `simulation/d1_sim/urdf/d1_sim.urdf.xacro` is completed.
- `config/sim_params.yaml` is intentionally empty — `use_sim_time=true` is set via
  launch argument, and no per-node simulation overrides are needed.
