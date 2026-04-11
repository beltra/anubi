# Repository Structure (Fortress-only)

This repository is organized as a modular ROS 2 workspace focused on Gazebo Fortress simulation and sim-to-real alignment.

## Core layers

- `src/platform/go2_bringup` and `src/platform/go2_description`: real-robot bridge and hardware-aligned Go2 description.
- `src/core_interfaces/robot_common_interfaces`: shared message and interface contracts.
- `src/core_interfaces/unitree_api`, `src/core_interfaces/unitree_go`: Unitree API/state contracts.
- `src/autonomy/robot_sim`: top-level simulation bringup orchestration (sim + SLAM + Nav2 + RViz).
- `src/autonomy/robot_nav`: robot-agnostic navigation and localization configuration.
- `src/autonomy/robot_safety`: watchdog and velocity safety controls.
- `src/autonomy/robot_lio`: robot-agnostic LIO integration hooks.
- `src/simulation/go2_sim`: maintained Go2 simulation model and launch integration for Fortress.
- `src/simulation/d1_sim`: D1 simulation stub kept as a template for future implementation.
- `src/simulation/sensor_models`: reusable sensor xacro macros shared across robot simulation packages.
- `src/simulation/sim_worlds`: Fortress world files and reusable SDF obstacle models.
- `src/simulation/sim_scenarios`: runtime scenario and obstacle management utilities.
- `src/vendors/*`: vendored third-party dependencies.

## Current simulation contract

- Primary simulation target is Gazebo Fortress.
- Go2 is the active reference implementation.
- D1 remains a non-production stub with stable frame naming to preserve integration compatibility.
- Scenario spawning uses world-scoped Fortress ROS GZ services via `ros_gz_interfaces`.

## Possible future improvements

- Implement `d1_sim` as a full Fortress model by reusing the same launch and sensor contracts used by Go2.
- Add automated CI smoke tests for `robot_sim` launch and key topic availability.
- Add multi-robot namespace support in bringup to run parallel simulation instances with the same modular stack.
