# Context Map

This repository has multiple feature contexts because several active branches
carry distinct autonomy features. Each context below records domain language
and relationships; implementation contracts remain in `docs/topic_map.md`,
`docs/launch_script_parity.md`, launch files, and wrapper scripts.

## Contexts

- [Workspace Architecture](./docs/contexts/workspace-architecture/CONTEXT.md) - package ownership boundaries for the modular ROS 2 workspace.
- [Sim-Real Navigation](./docs/contexts/sim-real-navigation/CONTEXT.md) - shared robot operation, command, and safety language for simulation and real Go2 bringup.
- [LiDAR Processing](./docs/contexts/lidar-processing/CONTEXT.md) - raw and corrected cloud semantics, gravity alignment, and the corrected-topic contract.
- [LIO Fusion](./docs/contexts/lio-fusion/CONTEXT.md) - optional lidar odometry and EKF ownership language.
- [Nav2 Recovery](./docs/contexts/nav2-recovery/CONTEXT.md) - quadruped-aware recovery stages and operator-visible recovery diagnostics.
- [Navigation Profiles](./docs/contexts/navigation-profiles/CONTEXT.md) - Nav2 profile selection, STVL, MPPI, and route-server experiments.
- [Dynamic Obstacles](./docs/contexts/dynamic-obstacles/CONTEXT.md) - branch feature for moving-obstacle tracking and predicted occupancy.
- [Simulation Validation](./docs/contexts/sim-validation/CONTEXT.md) - benchmark and regression scenario language for sim validation branches.

## Branch Origins

- `feature/stvl-local-costmap`: STVL local costmap and early pointcloud processing.
- `feat/gravity-align-mppi-benchmark`: gravity-aligned LiDAR frame, MPPI profiles, and benchmark runner.
- `feat/lio`: optional DLIO integration and reliable lidar odometry relay.
- `feat/navRecovery`: quadruped-aware Nav2 recovery package and BT wiring.
- `feat/sim2real`: current integrated sim-real safety, wrapper parity, launch contracts, and real Go2 boot flow.
- `feat/dynObstacles`: moving-obstacle detection, tracking, and predicted occupancy branch.
- `feat/simTest`: simulation safety regression and CI branch.

## Relationships

- **Sim-Real Navigation** consumes **LiDAR Processing**, **LIO Fusion**, **Navigation Profiles**, and **Nav2 Recovery** through shared topics and launch arguments.
- **LiDAR Processing** provides corrected perception input to **Navigation Profiles**, **Dynamic Obstacles**, and SLAM.
- **LIO Fusion** may improve odometry for **Sim-Real Navigation**, but the EKF remains the owner of `odom -> base_footprint`.
- **Nav2 Recovery** and **Navigation Profiles** must preserve the **Sim-Real Navigation** command chain.
- **Dynamic Obstacles** is branch-scoped until merged, but its predicted occupancy is already referenced by the default Nav2 profile.
- **Simulation Validation** exercises the other contexts without becoming part of default bringup.
