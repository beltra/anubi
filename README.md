# Anubi - Go2 Nav2 Autonomy Stack

Minimal, clean, production-oriented ROS 2 Humble autonomy stack for the **Unitree Go2**
quadruped robot.  Navigation uses **Nav2**; localisation uses **slam_toolbox** (2D SLAM).
Everything is self-contained: no upstream workspace needs to be sourced at runtime.

---

## Repository Layout

```
src/
├── platform/
│   ├── go2_bringup/          # Go2 hardware adapter (go2_hw_bridge) + launch files
│   └── go2_description/      # URDF/xacro: body + sensor frames
├── core_interfaces/
│   ├── robot_common_interfaces/  # Shared message types and robot adapter contract
│   ├── unitree_api/              # Unitree Go2 API request/response messages
│   └── unitree_go/               # Go2 hardware messages (SportModeState, LowState, ...)
├── autonomy/
│   ├── robot_nav/            # Robot-agnostic Nav2 + slam_toolbox config/launch
│   ├── robot_lio/            # Robot-agnostic LIO hooks (/lidar/imu, /lidar_odom)
│   ├── robot_safety/         # velocity_limiter safety gate and obstacle tracking
│   └── robot_sim/            # Simulation bringup launch (sim_bringup.launch.py)
├── simulation/
│   ├── go2_sim/              # Go2 Gazebo simulation URDF (planar_move + L1 LiDAR + IMU)
│   ├── d1_sim/               # AgiBot D1 Ultra simulation stub (TODO(agibot))
│   ├── sensor_models/        # Reusable Gazebo sensor xacro macros (L1 LiDAR)
│   ├── sim_worlds/           # World files + SDF obstacle models
│   └── sim_scenarios/        # Runtime obstacle spawn/remove CLI + scenario manager
└── vendors/
    ├── champ/                # CHAMP quadruped locomotion library (third-party, vendored)
    ├── champ_base/
    ├── champ_msgs/
    └── direct_lidar_inertial_odometry/
```

### TF tree (full chain)

```
map --(slam_toolbox)--> odom --(EKF)--> base_footprint --(lidar_level_frame_publisher)--> lidar_stabilized
                                             |
                                             +--(URDF)--> base_link
                                                              +--> lidar_link
                                                              +--> imu_link
```

### Data flow

```
Go2 hardware
  |
  +-- /sportmodestate  [SportModeState, ~50 Hz]
  |       +--> go2_hw_bridge --> /odom_raw
  |                                +--> EKF --> /odom + TF(odom -> base_footprint)
  |
  +-- /utlidar/cloud + /unilidar/imu
          +--> robot_lio relay --> /lidar/points + /lidar/imu
                                      |
                                      +--> distortion_corrector (C++, sim and HW)
                                               +--> /lidar/points_undistorted  [frame: lidar_stabilized]
                                                        +--> pointcloud_to_laserscan --> /scan --> slam_toolbox --> /map
                                                        +--> Nav2 STVL costmap + collision_monitor

Nav2 --> /cmd_vel --> collision_monitor --> /cmd_vel_collision_free
    +--> [velocity_limiter safety arbiter] --> /cmd_vel_safe
    +--> go2_hw_bridge --> /api/sport/request --> Go2 hardware

/wirelesscontroller --> joystick_safety --> /joystick_override
/imu/data --> tilt_monitor --> /safety/tilt_estop
```

---

## Dependencies

All packages are in this workspace; no upstream workspace needed at runtime.

| Dependency | Install |
|---|---|
| ROS 2 Humble base | `apt install ros-humble-ros-base` |
| Nav2 | `apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| slam_toolbox (LGPL-2.1) | `apt install ros-humble-slam-toolbox` |
| pointcloud_to_laserscan (BSD-3-Clause) | `apt install ros-humble-pointcloud-to-laserscan` |
| robot_state_publisher, xacro | `apt install ros-humble-robot-state-publisher ros-humble-xacro` |
| nlohmann/json (MIT) | `apt install nlohmann-json3-dev` |
| rviz2 | `apt install ros-humble-rviz2` |

> **Assumption A1:** The Go2 robot's internal ROS2 bridge is running and publishing:
> - `/sportmodestate` -- `unitree_go/SportModeState` (proprioceptive state, ~50 Hz)
> - `/lowstate` -- `unitree_go/LowState` (low-level state; used for battery data)
> - `/utlidar/cloud` -- `sensor_msgs/PointCloud2` (3D LiDAR, relayed to `/lidar/points`)
> - `/unilidar/imu` -- `sensor_msgs/Imu` (LiDAR IMU, relayed to `/lidar/imu`)
>
> `/lowstate` is used only for battery reporting in `RobotStatus`.  If the bridge
> does not publish it, navigation still works but `battery_soc` and `battery_voltage`
> in `/robot_status` will read 0.
>
> **Assumption A2:** The Go2 bridge subscribes to `/api/sport/request` for motion commands.
>
> **Assumption A3:** `slam_toolbox` is LGPL-2.1.  Under dynamic linking (colcon default),
> this is compatible with proprietary application code.

### SLAM Choice Rationale

**slam_toolbox** (LGPL-2.1) was selected over alternatives because:
- Native integration with Nav2 lifecycle and map server
- Supports online mapping **and** map-serialise-then-localise workflow
- Async mode decouples scan processing from the Nav2 control loop
- Loop closure for long-session reliability in industrial environments
- Actively maintained as part of the Nav2 ecosystem
- LGPL-2.1 is compatible with industrial deployment via dynamic linking

---

## How to Build

```bash
# Install system dependencies
sudo apt install \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-state-publisher ros-humble-xacro \
    ros-humble-rviz2 \
    nlohmann-json3-dev

# Build
source /opt/ros/humble/setup.bash
cd /ws/anubi
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

---

## How to Run on a Real Go2

### Essential scripts (recommended)

You can use these wrappers instead of typing long `ros2 launch` commands:

```bash
# Simulation (mapping mode)
/ws/anubi/scripts/run_go2_sim.sh

# Simulation (localization mode)
/ws/anubi/scripts/run_go2_sim.sh localization --map /ws/anubi/src/autonomy/robot_nav/maps/my_map

# Real robot (mapping mode, with LiDAR relay /utlidar/cloud -> /lidar/points)
/ws/anubi/scripts/run_go2_real.sh

# Real robot (localization mode; safety gate is on by default)
/ws/anubi/scripts/run_go2_real.sh localization --map /ws/anubi/src/autonomy/robot_nav/maps/my_map
```

Script options:

- `run_go2_sim.sh --help`
- `run_go2_real.sh --help`

### Step 1 -- Start the Go2 internal bridge on the robot

The Unitree Go2 internal software must publish:
- `/sportmodestate`  (`unitree_go/SportModeState`)
- `/utlidar/cloud`  (`sensor_msgs/PointCloud2`)
- `/utlidar/imu`  (`sensor_msgs/Imu`)

And subscribe to:
- `/api/sport/request`  (`unitree_api/Request`)

Refer to the Unitree Go2 ROS 2 SDK documentation for activating the bridge.
`robot_lio` relays the LiDAR topics into the stack as `/lidar/points` and
`/lidar/imu`.

### Step 2 -- Map the environment (first time only)

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash

# Launch in mapping mode (default)
ros2 launch go2_bringup go2_bringup.launch.py slam_mode:=mapping

# Drive the robot around manually to build the map, then save it:
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/ws/anubi/src/autonomy/robot_nav/maps/my_map'}"
```

### Step 3 -- Navigate with a saved map

```bash
ros2 launch go2_bringup go2_bringup.launch.py \
    slam_mode:=localization \
    map:=/ws/anubi/src/autonomy/robot_nav/maps/my_map
```

### Step 4 -- Send a goal in RViz

1. RViz opens automatically (set `rviz:=false` to disable).
2. Click **"2D Goal Pose"** in the toolbar.
3. Click + drag on the map to set position and orientation.
4. The robot plans a path and drives autonomously.

### Optional -- Mapping mode without goal (SLAM only, no navigation)

```bash
ros2 launch go2_bringup go2_bringup.launch.py slam_mode:=mapping rviz:=true
```

---

## How to Port to AgiBot D1 Ultra

1. Create `agibot_d1_bringup` package, mirroring `go2_bringup`.
2. Implement `d1_hw_bridge.cpp`:
   - Subscribe to the D1 Ultra state topic -> publish `/odom` + TF `odom -> base_footprint`
   - Subscribe to `/cmd_vel` -> convert to D1 Ultra motion API
3. Create `agibot_d1_description` with the same frame names: `base_footprint`, `base_link`, `lidar_link`, `imu_link`.
4. Create `agibot_d1_bringup/config/d1_params.yaml` with D1-specific topic names.
5. Use `go2_bringup/launch/go2_bringup.launch.py` as a template.
6. **Zero changes** required in `robot_nav`, `slam_toolbox_params.yaml`, or `robot_common_interfaces`.

---

## How to Run in Gazebo Simulation

The simulation layer is fully implemented. **No hardware required.**
`robot_sim/sim_bringup.launch.py` starts the Fortress CHAMP simulation stack
(`go2_sim/sim_champ.launch.py`) plus SLAM/Nav2.

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash

# Full simulation -- office world (default), mapping mode, headless by default
ros2 launch robot_sim sim_bringup.launch.py

# Warehouse world, localization with a saved map, Gazebo GUI + RViz enabled
ros2 launch robot_sim sim_bringup.launch.py \
    world:=$(ros2 pkg prefix sim_worlds)/share/sim_worlds/worlds/warehouse.world \
    slam_mode:=localization \
    map:=/ws/anubi/src/autonomy/robot_nav/maps/my_map \
    gui:=true \
    use_rviz:=true
```

### Runtime obstacle management

```bash
# Spawn a crate at (2, 1)
ros2 run sim_scenarios spawn_obstacle \
    --name box_1 --model obstacle_box --x 2.0 --y 1.0 --z 0.5

# Delete it
ros2 run sim_scenarios spawn_obstacle --delete box_1

# Load a pre-defined scenario (crowded_room or narrow_corridor)
ros2 run sim_scenarios scenario_manager \
    --ros-args -p scenario_file:=crowded_room

# Nav2-only restart entry point is not provided in the current tree.
```

### Known simulation limits

- Gazebo Fortress + CHAMP is the maintained simulation path.
- AgiBot D1 Ultra sim (`simulation/d1_sim/`) is a stub; chassis dimensions are placeholders.

---

## Key Design Decisions

| Decision | Rationale |
|---|---|
| `go2_hw_bridge` single node for both odom and cmd_vel | Minimal Go2-specific surface area |
| `SportModeState` -> `odom/TF` (not Point-LIO) | Self-contained; Go2's internal state estimator is accurate short-term; slam_toolbox corrects long-term drift |
| `pointcloud_to_laserscan` -> slam_toolbox | slam_toolbox needs 2D scan; Nav2 costmaps still use full 3D cloud |
| `odom -> base_footprint` (not base_link) | Standard Nav2 2D convention; slam_toolbox tracks `base_footprint` |
| cmd_vel watchdog (0.5 s) | Stops robot if Nav2 crashes; prevents runaway |
| nlohmann/json (system package, MIT) | No vendoring; standard Go2 SDK serialisation format |
| slam_toolbox LGPL-2.1 | Standard Nav2 SLAM; LGPL OK under dynamic linking |

---

## TODO Markers in Code

- `TODO(agibot)` -- changes for AgiBot D1 Ultra porting (implement `simulation/d1_sim/urdf/d1_sim.urdf.xacro` and `agibot_d1_bringup`)
- `TODO(sim)` -- Gazebo plugin stubs remaining in `go2_description/urdf/go2.urdf.xacro` (hardware URDF); the simulation itself is fully working via `go2_sim/`
- `TODO(safety)` -- calibrate the real controller START key mask for tilt and navigation recovery
- `TODO(localization)` -- where to wire in map-based localization improvements

---

## Project Improvement Roadmap

Ordered by current value to the project. Difficulty is a rough implementation estimate: S = small, M = medium, L = large, XL = multi-week or cross-cutting.

| Priority | Difficulty | Area | Improvement | What it should bring |
|---|---:|---|---|---|
| 1 | M | Safety | Make real-robot safety gating the production default | Enable `use_safety` by default in the real-robot path, keep an explicit opt-out for bench tests, and verify that `go2_hw_bridge` only reads `/cmd_vel_safe` in production runs. This reduces the chance of bypassing the watchdog and future safety checks during field use. |
| 2 | M | Safety | Add joystick override and autonomous-goal cancellation | Implement the `/wirelesscontroller -> /joystick_override` path described in `docs/PROJECT_SPECS.md`, cancel active Nav2 goals when the operator touches the controller, and hold zero velocity until the controller is neutral for a configurable grace period. This gives the human operator hard priority over autonomy. |
| 3 | M | Safety | Implement IMU tilt and fall detection | Add the planned `tilt_monitor` node using `/imu/data`, publish a latched safety/estop state, and latch zero commands until recovery is acknowledged. This protects the robot and environment when the body pitch/roll exceeds safe walking assumptions. |
| 4 | M | Safety | Add sensor, TF, and command-chain health monitors | Create diagnostics for stale `/lidar/points_undistorted`, `/scan`, `/odom`, `/tf`, `/cmd_vel`, and `/cmd_vel_safe`, with clear WARN/ERROR states. This makes silent perception or localization failures visible before they become motion failures. |
| 5 | M | Safety | Validate collision monitor zones against real geometry | Re-measure the Go2 footprint, leg sweep, stopping distance, LiDAR blind zones, and collision monitor polygons in both sim and hardware. This turns the current hand-tuned stop/slow zones into documented safety envelopes. |
| 6 | L | Safety | Add safety regression scenarios in simulation | Extend `sim_scenarios` with repeatable tests for sudden obstacles, blocked corridors, stale sensor topics, map jumps, and operator override. This gives safety behavior a measurable pass/fail loop instead of relying on manual RViz inspection. |
| 7 | M | Safety | Add low-battery and hardware-state policies | Use `/robot_status` battery and hardware fields to slow down, reject new goals, or stop safely under configurable thresholds. This turns status reporting into operational behavior. |
| 8 | L | Safety | Build a formal command multiplexer | Introduce a clear arbiter for Nav2, joystick/manual, assisted teleop, recovery, and emergency stop sources before the final safety gate. This makes future motion sources easier to add without accidentally bypassing collision or watchdog logic. |
| 9 | M | Safety | Add speed-limit publishers for zones and degraded state | Feed Nav2's `/speed_limit` from route metadata, narrow spaces, low localization confidence, or safety degradation. This allows smoother risk-aware motion instead of only stop/slow/normal behavior. |
| 10 | M | Validation | Add launch smoke tests for sim and real contracts | Use `launch_testing` or scripted smoke tests to start core launch files and assert required topics, lifecycle nodes, and TF links from `docs/topic_map.md`. This catches broken launch wiring early. |
| 11 | L | Validation | Add CI for build, unit tests, launch checks, and lint | Create GitHub Actions or another CI runner for `colcon build`, `colcon test`, Python linting, C++ linting, and non-GUI launch checks. This turns the workspace into a repeatable engineering artifact instead of a local-only stack. |
| 12 | M | Validation | Expand unit tests for custom nodes | Add tests for `velocity_limiter`, `lidar_odom_relay`, `distortion_corrector`, `lidar_level_frame_publisher`, `scenario_manager`, and `go2_hw_bridge` parameter behavior. This protects the custom integration logic that ROS packages will not test for you. |
| 13 | M | Validation | Add a topic-map contract checker | Write a script that compares launch/config references against `docs/topic_map.md` and flags undocumented topic or TF changes. This reinforces the repo's stated source of truth. |
| 14 | M | Validation | Add rosbag-based replay tests | Record short sim and real bags for odom, IMU, LiDAR, TF, and commands, then replay them through perception/localization/safety nodes. This enables deterministic debugging of failures seen on the robot. |
| 15 | M | Localization | Harden localization mode and map lifecycle | Clarify when to use slam_toolbox localization versus AMCL/map_server, add validation for `.posegraph`, `.data`, `.yaml`, and `.pgm` assets, and document migration between mapping and localization runs. This reduces map workflow confusion. |
| 16 | L | Localization | Calibrate and validate LIO fusion | Turn optional DLIO into a measured feature: tune covariances, test degeneration cases, log reliability scale, and decide when EKF should consume or ignore `/lidar_odom_reliable`. This can improve odometry while avoiding over-trusting bad scan matches. |
| 17 | M | Perception | Finish dynamic obstacle tracking | Replace the placeholder `dynamic_obstacle_tracker.py` with clustering, velocity estimation, and projected obstacle points for local costmaps. This improves behavior around moving people, forklifts, and carts. |
| 18 | M | Perception | Add point cloud quality metrics | Publish counts/rates for raw, self-filtered, deskewed, passthrough, and scan-converted clouds. This makes LiDAR degradation, self-filter mistakes, and simulation/hardware differences much easier to diagnose. |
| 19 | M | Navigation | Benchmark and select Nav2 profiles systematically | Extend `run_mppi_suite.sh` into a profile comparison suite for `default_safe`, `narrow_gap_1p0m`, `forward_static`, and 2D fallback profiles. This makes planner/controller tuning evidence-based. |
| 20 | L | Navigation | Complete route-server workflows | Add route graph authoring examples, loader validation, RViz visualization, and scenario tests for `use_route_server:=true`. This brings the project closer to industrial site navigation over preferred lanes and checkpoints. |
| 21 | L | Navigation | Add waypoint missions and task execution | Build first-class waypoint/mission files for patrols, inspections, waits, and route constraints. This moves the stack from single-goal demos toward repeatable industrial workflows. |
| 22 | L | Navigation | Add semantic zones and keepout/speed maps | Support keepout areas, preferred lanes, restricted aisles, slow zones, and temporary closures from map overlays or route metadata. This gives operators control over how the robot behaves in real facilities. |
| 23 | XL | Features | Implement full D1 Ultra support | Complete `simulation/d1_sim`, add `agibot_d1_bringup`, and prove the same `robot_nav`, `robot_lio`, and `robot_safety` contracts work unchanged. This validates the robot-agnostic architecture. |
| 24 | L | Features | Add multi-robot namespace support end to end | Finish namespacing for launch files, topics, TF prefixes, Gazebo entities, scenario tools, and RViz configs. This enables multiple robots in the same simulation or facility. |
| 25 | M | Features | Add a map and mission asset manager | Standardize where saved maps, posegraphs, PCD files, route graphs, scenarios, and benchmark reports live, with metadata for date, robot, sensor setup, and environment. This prevents important operational assets from becoming anonymous files. |
| 26 | M | Ease of Use | Add a preflight checker | Provide a script that verifies ROS environment, build availability, required packages, map files, Gazebo services, Unitree topics, LiDAR relay inputs, and expected TF publishers before launching. This shortens the path from "nothing moves" to the actual missing dependency. |
| 27 | M | Ease of Use | Add reproducible dependency setup | Add `rosdep` instructions or files, optional Docker/devcontainer support, and explicit Gazebo Fortress / `ros_gz` / `ros_ign` dependency installation steps. This makes onboarding less dependent on tribal setup knowledge. |
| 28 | S | Ease of Use | Align READMEs with current launch files | Audit stale package READMEs, especially command topics, odometry ownership, dependency lists, and safety defaults, against launch files and `docs/topic_map.md`. This keeps documentation from becoming a source of operational mistakes. |
| 29 | S | Ease of Use | Improve troubleshooting docs | Add short checklists for missing map, stale TF, blank RViz, no `/scan`, no `/cmd_vel_safe`, Gazebo service bridge issues, and localization startup failures. This saves time during field debugging. |
| 30 | M | Ease of Use | Add operator dashboards | Create RViz panels, rqt perspectives, or a lightweight web dashboard for safety state, battery, localization mode, current goal, command chain, LiDAR health, and map save actions. This makes the system easier to supervise without watching many terminals. |
| 31 | M | SW Engineering | Refactor shared Nav2 config overlays | Reduce duplicated YAML across Nav2 profiles by introducing shared base config plus profile overlays where ROS launch tooling allows it. This lowers the cost of keeping safety-critical parameters consistent. |
| 32 | M | SW Engineering | Add typed parameter validation | Validate important parameters at node startup: topics are non-empty, speed limits are positive, covariance arrays have expected lengths, frame IDs are valid, and mode strings are known. This catches misconfiguration before launch side effects appear. |
| 33 | M | SW Engineering | Improve lifecycle and shutdown discipline | Bring custom nodes closer to managed lifecycle behavior where useful, and ensure wrapper scripts shut down spawned processes cleanly without overly broad kills. This improves reliability during repeated sim and field runs. |
| 34 | M | SW Engineering | Add structured logging and metrics | Standardize log messages, expose counters for watchdog events, stale clouds, collision monitor stops, LIO reliability degradation, and goal outcomes. This makes regression analysis and field reports far more useful. |
| 35 | M | SW Engineering | Create hardware-in-the-loop validation checklists | Document and automate the safest possible real-robot validation sequence: static checks, wheels/legs unloaded where applicable, low-speed first command, safety stop, mapping, localization, then autonomous goal. This reduces risk when changes leave simulation. |
| 36 | M | SW Engineering | Add release and configuration versioning | Version maps, Nav2 profiles, robot parameters, route graphs, and known-good launch combinations. This makes it possible to know what was tested on a given robot and environment. |
| 37 | S | SW Engineering | Remove or isolate placeholders | Either implement, hide, or clearly quarantine placeholder executables such as `dynamic_obstacle_tracker.py` so they cannot be launched accidentally. This keeps the installed surface area honest. |

## License

Copyright (C) 2026 beltra <gbeve2002@gmail.com>

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU Affero General Public License, version 3, as published by
the Free Software Foundation. It is distributed WITHOUT ANY WARRANTY; see
[LICENSE](LICENSE) for the full text.

Third-party code keeps its own license and is not covered by the above:

- `src/vendors/` — CHAMP (BSD), DLIO (MIT), livox_ros_driver2 (MIT, bundles RapidJSON); see the license file in each directory.
- `src/core_interfaces/unitree_api`, `src/core_interfaces/unitree_go` — Unitree ROS 2 message definitions.
- `src/platform/g1_locomotion` — Apache-2.0, derived from AMO (UCSD) and Psi0 (PSI-lab, USC); see its NOTICE.
- `src/platform/g1_description/meshes`, `src/platform/go2_description/meshes` — Unitree robot meshes.
