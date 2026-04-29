# Dynamic Obstacles Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a LiDAR-based dynamic obstacle tracking pipeline that predicts near-future obstacle occupancy for Nav2 and visualizes tracks in RViz.

**Architecture:** Reuse the existing `/lidar/points_undistorted` output from `distortion_corrector` so SLAM and current costmaps remain unchanged. Add topic-decoupled nodes in `robot_nav`: preprocessing, dynamic cluster detection, EKF tracking/prediction, and a Nav2/RViz bridge. Nav2 consumes predicted occupancy as an additional `PointCloud2` observation source, matching the existing STVL/VoxelLayer pattern.

**Tech Stack:** ROS 2 Humble, `rclpy`, NumPy, `tf2_ros`, `sensor_msgs/PointCloud2`, custom `robot_common_interfaces` messages, Nav2 STVL/VoxelLayer, Gazebo Fortress SDF actors, RViz `visualization_msgs/MarkerArray`.

---

### Task 1: Message Contracts

**Files:**
- Create: `src/core_interfaces/robot_common_interfaces/msg/DynamicObstacleCluster.msg`
- Create: `src/core_interfaces/robot_common_interfaces/msg/DynamicObstacleClusterArray.msg`
- Create: `src/core_interfaces/robot_common_interfaces/msg/DynamicObstacleTrack.msg`
- Create: `src/core_interfaces/robot_common_interfaces/msg/DynamicObstacleTrackArray.msg`
- Modify: `src/core_interfaces/robot_common_interfaces/CMakeLists.txt`
- Modify: `src/core_interfaces/robot_common_interfaces/package.xml`

- [ ] Add cluster messages with ID, centroid, size, yaw, point count, and dynamic flag.
- [ ] Add track messages with ID, pose, velocity, size, yaw rate, confidence, predicted poses, predicted stamps, and uncertainty radii.
- [ ] Register messages with `rosidl_generate_interfaces`.
- [ ] Build `robot_common_interfaces`.

### Task 2: Core Pipeline Nodes

**Files:**
- Create: `src/autonomy/robot_nav/robot_nav/dynamic_obstacles.py`
- Create: `src/autonomy/robot_nav/scripts/dynamic_obstacle_preprocessor.py`
- Create: `src/autonomy/robot_nav/scripts/dynamic_cluster_detector.py`
- Create: `src/autonomy/robot_nav/scripts/dynamic_obstacle_tracker.py`
- Create: `src/autonomy/robot_nav/scripts/dynamic_obstacle_costmap_bridge.py`
- Modify: `src/autonomy/robot_nav/CMakeLists.txt`
- Modify: `src/autonomy/robot_nav/package.xml`

- [ ] Implement NumPy point cloud extraction, voxel filtering, ground filtering, 2D clustering, CTRV/CV prediction, marker creation, and predicted occupancy point generation.
- [ ] Implement preprocessor: transform cloud into fixed frame, ground filter, voxel downsample, publish `/dynamic_obstacles/points_world`.
- [ ] Implement detector: subtract static occupancy map, cluster foreground points, classify moving clusters, publish `/dynamic_obstacles/clusters`.
- [ ] Implement tracker: lifecycle-managed EKF-like tracks, distance gating, CTRV prediction, publish `/dynamic_obstacles/tracks`.
- [ ] Implement bridge: publish predicted occupancy cloud `/dynamic_obstacles/predicted_occupancy` and RViz markers `/dynamic_obstacles/markers`, with watchdog clearing.
- [ ] Install scripts and dependencies.

### Task 3: Launch And Parameters

**Files:**
- Create: `src/autonomy/robot_nav/config/dynamic_obstacles_sim.yaml`
- Create: `src/autonomy/robot_nav/config/dynamic_obstacles_hw.yaml`
- Create: `src/autonomy/robot_nav/launch/dynamic_obstacles.launch.py`
- Modify: `src/autonomy/robot_sim/launch/sim_bringup.launch.py`
- Modify: `src/platform/go2_bringup/launch/go2_bringup.launch.py`
- Modify: `scripts/run_go2_sim.sh`
- Modify: `scripts/run_go2_real.sh`
- Modify: `scripts/kill_ros_nodes.sh`

- [ ] Add shared launch for the four nodes, with parameter-file override and `use_sim_time`.
- [ ] Add simulation flag `use_dynamic_obstacles`.
- [ ] Add hardware flag `use_dynamic_obstacles`.
- [ ] Keep defaults disabled unless explicitly requested, preserving existing behavior.
- [ ] Update cleanup script for new node names.

### Task 4: Nav2 Costmap Wiring

**Files:**
- Modify: `src/autonomy/robot_nav/config/nav2_params.yaml`
- Modify: `src/autonomy/robot_nav/config/nav2_params_forward_static.yaml`
- Modify: `src/autonomy/robot_nav/config/nav2_params_narrow_gap_1p0m.yaml`
- Modify: `src/autonomy/robot_nav/config/nav2_params_2d.yaml`

- [ ] Add `/dynamic_obstacles/predicted_occupancy` QoS overrides.
- [ ] Add predicted cloud as a marking-only observation source in local and global point-cloud costmap layers.
- [ ] Use short observation persistence so watchdog silence naturally reverts to static behavior.

### Task 5: Simulation Actor And Docs

**Files:**
- Modify: `src/simulation/sim_worlds/worlds/indoor_office.world`
- Modify: `docs/topic_map.md`
- Create: `DYNAMIC_OBSTACLES.md`

- [ ] Add Gazebo Fortress SDF actors that cross, pass straight, and curve near typical robot paths.
- [ ] Document nodes, topics, parameters, simulation run command, hardware run command, and RViz markers.
- [ ] Update topic map with new pipeline topics.

### Task 6: Verification

**Files:**
- Workspace build/test output only.

- [ ] Run `source /opt/ros/humble/setup.bash && colcon build --packages-up-to robot_nav robot_sim go2_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`.
- [ ] Run focused Python syntax checks on new scripts.
- [ ] Report any smoke launch limitation if Gazebo/Nav2 cannot be run in this environment.
