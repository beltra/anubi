# Dynamic Obstacles C++ Rewrite Design

## Goal

Replace the current Python dynamic obstacle pipeline in `robot_nav` with a C++ implementation that preserves the existing ROS topics, message types, parameters, launch behavior, and default Nav2 profile compatibility.

## Scope

This rewrite covers the four runtime nodes in `src/autonomy/robot_nav/scripts/`:

- `dynamic_obstacle_preprocessor.py`
- `dynamic_cluster_detector.py`
- `dynamic_obstacle_tracker.py`
- `dynamic_obstacle_costmap_bridge.py`

The Python implementation remains in the tree as the behavioral reference and fallback during development.

## External Contract

The rewritten pipeline must preserve:

- node names
- topic names
- message types
- parameter names and semantics
- launch arguments and launch include behavior
- existing sim and hardware YAML parameter files
- Nav2 consumption of `/dynamic_obstacles/predicted_occupancy`
- RViz consumption of `/dynamic_obstacles/markers`

## Runtime Behavior

The C++ rewrite keeps the current four-stage pipeline:

1. Preprocessor
   - input: `/lidar/points_undistorted`
   - transform into fixed frame
   - height filter
   - voxel downsample
   - publish `/dynamic_obstacles/points_world`

2. Cluster detector
   - subtract occupied cells from `/map`
   - cluster remaining points
   - associate clusters frame to frame
   - reject ego-motion artifacts
   - manage shadow cones
   - publish `DynamicObstacleClusterArray`

3. Tracker
   - maintain persistent tracks
   - perform CTRV prediction and position correction
   - publish predicted future poses in `DynamicObstacleTrackArray`

4. Costmap bridge
   - rasterize current and predicted footprints into a `PointCloud2`
   - publish RViz markers
   - clear outputs on timeout

## Implementation Shape

Use a shared C++ utility layer for:

- PointCloud2 parsing and creation
- quaternion and yaw helpers
- transform math
- voxel filtering and footprint generation
- Euclidean clustering
- shadow cone logic
- CTRV state prediction and Jacobian

Keep the node classes package-local and direct. No extra framework or lifecycle conversion is needed for this pass.

## Planned File-Level Changes

Create:

- `src/autonomy/robot_nav/include/robot_nav/dynamic_obstacles/common.hpp`
- `src/autonomy/robot_nav/src/dynamic_obstacles_common.cpp`
- `src/autonomy/robot_nav/src/dynamic_obstacle_preprocessor.cpp`
- `src/autonomy/robot_nav/src/dynamic_cluster_detector.cpp`
- `src/autonomy/robot_nav/src/dynamic_obstacle_tracker.cpp`
- `src/autonomy/robot_nav/src/dynamic_obstacle_costmap_bridge.cpp`
- `src/autonomy/robot_nav/test/test_dynamic_obstacles_cpp.cpp`

Modify:

- `src/autonomy/robot_nav/CMakeLists.txt`
- `src/autonomy/robot_nav/package.xml`
- `src/autonomy/robot_nav/launch/dynamic_obstacles.launch.py`
- `src/autonomy/robot_nav/README.md`
- `DYNAMIC_OBSTACLES.md`
- `docs/topic_map.md`

Keep as reference:

- `src/autonomy/robot_nav/robot_nav/dynamic_obstacles.py`
- `src/autonomy/robot_nav/scripts/dynamic_obstacle_preprocessor.py`
- `src/autonomy/robot_nav/scripts/dynamic_cluster_detector.py`
- `src/autonomy/robot_nav/scripts/dynamic_obstacle_tracker.py`
- `src/autonomy/robot_nav/scripts/dynamic_obstacle_costmap_bridge.py`

## Validation

Validation should cover:

- `colcon build` for the affected packages
- unit tests for shared math and geometry behavior
- launch smoke test for `dynamic_obstacles.launch.py`
- simulation smoke test through the default Nav2 profile when possible
- message and topic parity against the Python reference behavior

