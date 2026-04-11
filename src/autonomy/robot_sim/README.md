# robot_sim

Gazebo simulation bringup package for the Go2 autonomy stack.

---

## Overview

`sim_bringup.launch.py` starts the maintained Fortress + CHAMP simulation:

```
go2_sim/sim_champ.launch.py  (Ignition Gazebo + ros_ign_bridge + CHAMP + EKFs)
    +  slam.launch.py        (pointcloud_to_laserscan + slam_toolbox, use_sim_time=true)
    +  nav2.launch.py        (Nav2 navigation stack, use_sim_time=true)
    +  RViz2                 (optional, reuses robot_nav/rviz/nav2.rviz)
```

`go2_hw_bridge` is **not** started. Simulation provides the same navigation-facing
topics (`/lidar/points`, `/scan`, `/odom`, `/cmd_vel`) through CHAMP + SLAM/Nav2.

TF chain in simulation (identical to real robot):

```
map ─(slam_toolbox)─► odom ─(CHAMP EKF chain)─► base_footprint
                                                  └─(URDF)─► base_link
                                                                  ├──► lidar_link
                                                                  └──► imu_link
```

All Nav2 configuration is shared with the real robot (`robot_nav/`).

---

## Quick Start

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash

# Full sim — indoor office world (default), mapping mode, Gazebo GUI + RViz
ros2 launch robot_sim sim_bringup.launch.py

# Warehouse world, localization with a saved map, no GUI
ros2 launch robot_sim sim_bringup.launch.py \
    world:=$(ros2 pkg prefix sim_worlds)/share/sim_worlds/worlds/warehouse.world \
    slam_mode:=localization \
    map:=/ws/anubi/src/robot_nav/maps/my_map \
    gui:=false

# Nav2-only restart entry point is not provided in the current tree.
```

---

## Launch Arguments

### `sim_bringup.launch.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `indoor_office.world` | Path to Gazebo world file |
| `robot` | `go2` | Robot model: `go2` or `d1` |
| `slam_mode` | `mapping` | `mapping` or `localization` |
| `map` | `''` | Path to `.posegraph` file (localization mode only) |
| `use_rviz` | `true` | Launch RViz2 |
| `slam_params` | robot_nav default | slam_toolbox params YAML override |
| `nav_params` | robot_nav default | Nav2 params YAML override |
| `spawn_z` | `0.05` m | Robot Z spawn height (slight ground clearance) |
| `gui` | `true` | Show Gazebo gzclient GUI |

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
- AgiBot D1 Ultra model (`d1_sim/`) is a stub — chassis dimensions are placeholders.
  Launch with `robot:=d1` is supported once `d1_sim/urdf/d1_sim.urdf.xacro` is completed.
- `config/sim_params.yaml` is intentionally empty — `use_sim_time=true` is set via
  launch argument, and no per-node simulation overrides are needed.
