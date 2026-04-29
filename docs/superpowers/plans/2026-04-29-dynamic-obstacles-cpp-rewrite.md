# Dynamic Obstacles C++ Rewrite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Python dynamic obstacle runtime pipeline with C++ executables while preserving the current ROS interface and launch behavior.

**Architecture:** Add one shared C++ utility library for point-cloud parsing, geometry helpers, clustering, shadow-cone filtering, and CTRV tracking math. Build four focused ROS2 executables on top of that library and switch `dynamic_obstacles.launch.py` to those executables, while leaving the Python implementation in place as a reference.

**Tech Stack:** ROS 2 Humble, `rclcpp`, `tf2_ros`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `robot_common_interfaces`, C++17, Eigen3, GTest, existing Python reference tests.

---

### Task 1: Add shared C++ dynamic-obstacle utilities

**Files:**
- Create: `src/autonomy/robot_nav/include/robot_nav/dynamic_obstacles/common.hpp`
- Create: `src/autonomy/robot_nav/src/dynamic_obstacles_common.cpp`

- [ ] Define the shared data structures and helper declarations needed by all four nodes.
- [ ] Implement PointCloud2 read/write helpers, quaternion and yaw helpers, point transforms, voxel filtering, height filtering, footprint generation, clustering, shadow-cone masking, and CTRV math.
- [ ] Keep helper APIs narrow enough that the node files own the ROS behavior and the utility file owns reusable math and point-cloud operations.

### Task 2: Rewrite the preprocessor in C++

**Files:**
- Create: `src/autonomy/robot_nav/src/dynamic_obstacle_preprocessor.cpp`

- [ ] Recreate the existing parameter set and defaults.
- [ ] Recreate TF lookup behavior, including latest-transform fallback and throttled warnings.
- [ ] Publish `/dynamic_obstacles/points_world` with the same frame and topic contract as the Python node.

### Task 3: Rewrite the cluster detector in C++

**Files:**
- Create: `src/autonomy/robot_nav/src/dynamic_cluster_detector.cpp`

- [ ] Recreate map occupancy filtering, out-of-map handling, Euclidean clustering, cluster-shape extraction, frame-to-frame association, ego-motion rejection, and shadow-cone logic.
- [ ] Preserve detector message fields, dynamic promotion thresholds, and empty-output behavior when a required static map is missing.

### Task 4: Rewrite the tracker in C++

**Files:**
- Create: `src/autonomy/robot_nav/src/dynamic_obstacle_tracker.cpp`

- [ ] Recreate the current parameter set, timer-driven republishing, gated cluster-to-track assignment, CTRV prediction, correction, confidence tracking, and stale-input timeout behavior.
- [ ] Preserve track message layout and prediction horizon behavior.

### Task 5: Rewrite the costmap bridge in C++

**Files:**
- Create: `src/autonomy/robot_nav/src/dynamic_obstacle_costmap_bridge.cpp`

- [ ] Recreate occupancy footprint rasterization, uncertainty padding, marker generation, watchdog clearing, and timeout handling.
- [ ] Preserve the current marker namespaces and output topics used by RViz and Nav2.

### Task 6: Wire the package to build and launch the C++ nodes

**Files:**
- Modify: `src/autonomy/robot_nav/CMakeLists.txt`
- Modify: `src/autonomy/robot_nav/package.xml`
- Modify: `src/autonomy/robot_nav/launch/dynamic_obstacles.launch.py`

- [ ] Add the shared utility library and four executables to CMake.
- [ ] Add any C++ build dependencies required by the new implementation.
- [ ] Switch `dynamic_obstacles.launch.py` to the new executables without changing node names, launch arguments, or parameter-file behavior.
- [ ] Leave the Python scripts installed or present as reference unless the build wiring makes that unnecessary.

### Task 7: Add focused C++ verification and keep the reference checks

**Files:**
- Create: `src/autonomy/robot_nav/test/test_dynamic_obstacles_cpp.cpp`
- Keep: `src/autonomy/robot_nav/test/test_dynamic_obstacles.py`

- [ ] Add focused GTests for helper behavior that is easy to compare against the Python implementation: yaw round-trip, oriented footprint extents, shadow-cone masking, ego-motion velocity matching, and publishability logic.
- [ ] Keep the Python helper tests so the original reference behavior remains easy to inspect during parity checks.

### Task 8: Validate build, launch, and documentation consistency

**Files:**
- Modify: `src/autonomy/robot_nav/README.md`
- Modify: `DYNAMIC_OBSTACLES.md`
- Modify: `docs/topic_map.md`

- [ ] Build the affected packages with `colcon build`.
- [ ] Run the dynamic-obstacle unit tests.
- [ ] Run a launch smoke test for `robot_nav/launch/dynamic_obstacles.launch.py`.
- [ ] Attempt a simulation smoke test with the default Nav2 profile and dynamic obstacles enabled.
- [ ] Update docs only where they still mention the Python runtime or otherwise disagree with the final launch behavior.
