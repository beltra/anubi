# Anubi

ROS 2 navigation stack for Unitree legged robots. It runs on the Go2 quadruped and the
G1 humanoid, in Gazebo and on real hardware. The autonomy packages don't know which
robot they're driving: each robot plugs in through a small adapter that publishes the
same topics and TF frames.

## Stack

| Layer | What runs |
|---|---|
| Middleware | ROS 2 Humble, Gazebo Fortress |
| Localization | slam_toolbox (2D mapping + localization), robot_localization EKF, optional DLIO LiDAR-inertial odometry |
| Perception | LiDAR deskew/self-filter/leveling (`distortion_corrector`), `pointcloud_to_laserscan`, STVL voxel costmap |
| Navigation | Nav2 with MPPI, custom recovery behaviors for legged robots |
| Safety | Nav2 `collision_monitor`, then a `velocity_limiter` with a command watchdog, joystick override and tilt e-stop |
| Robot adapters | Go2: Sport Mode API bridge. G1: DDS state bridge plus the AMO or SONIC walking controller |

Command path:

```
Nav2 /cmd_vel -> collision_monitor -> /cmd_vel_collision_free
              -> velocity_limiter  -> /cmd_vel_safe -> robot adapter
```

## Quick start (simulation)

Ubuntu 22.04 with ROS 2 Humble installed. The scripts expect the workspace at
`/ws/anubi`.

```bash
sudo mkdir -p /ws && sudo chown $USER /ws
git clone https://github.com/beltra/anubi.git /ws/anubi
cd /ws/anubi

source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -y -r
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

scripts/run_go2_sim.sh            # Go2 in an office world, mapping mode
scripts/run_g1_sim.sh             # or the G1
```

Wait for RViz, then click **2D Goal Pose** and drag a goal onto the map.

## Docs

- [Getting started](docs/getting_started.md): install, build, Docker, first sim and robot runs
- [Usage](docs/usage.md): mapping, localization, saving maps, wrapper options, safety controls
- [Architecture](docs/architecture.md): packages, data flow, TF ownership, design decisions
- [Unitree G1](docs/g1.md): G1 specifics, AMO and SONIC controllers
- [Topic map](docs/topic_map.md): full topic and TF reference
- [Wrapper parity](docs/launch_script_parity.md): which wrapper options exist in sim vs. real

## Status

The Go2 is the reference platform and runs in both sim and on hardware. G1 support is
newer: it navigates in Gazebo, and its hardware path is still being tuned. Do the first
G1 runs on a hoist, and that goes double for SONIC. The Isaac Sim adapter (`g1_isaac`)
is experimental.

## License

AGPL-3.0-only, see [LICENSE](LICENSE). Copyright (C) 2026 beltra.

Third-party code keeps its own license:

- `src/vendors/`: CHAMP (BSD), DLIO (MIT), livox_ros_driver2 (MIT, bundles RapidJSON)
- `src/core_interfaces/unitree_api`, `unitree_go`: Unitree ROS 2 message definitions
- `src/platform/g1_locomotion`: Apache-2.0, derived from AMO (UCSD) and Psi0 (PSI-lab, USC).
  `amo_driver.py` also contains unlicensed portions by Lorenzo Ortolani, pending
  permission; see its NOTICE.
- `src/platform/*_description/meshes`: Unitree robot meshes
