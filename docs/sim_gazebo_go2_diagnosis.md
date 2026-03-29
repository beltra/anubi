# Gazebo Go2 Simulation Diagnosis

## Scope

This document records the diagnosis and fix for the Gazebo Classic + ROS 2 + Nav2 simulation regression introduced by the newer detailed Go2 model.

Repository state:

- Failing revision inspected at `HEAD` (`06cb9ec`, `Improved Gazebo Go2 Model`)
- Last known working sim model compared against `2ffb9a1`

## Symptoms Reproduced

The following were reproduced from the repository launch entry point:

- Gazebo launched and the new Go2 visual model spawned correctly.
- Raw simulated LiDAR data contained very short near-body returns.
- Nav2 perceived the robot as blocked by itself in the simulation obstacle pipeline.
- The robot tipped badly when commanded to move.
- Navigation behavior looked directionally wrong because the body left the planar regime while Nav2 still planned in 2D.

Launch entry point used:

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
export ROS_LOG_DIR=/ws/anubi/log/runtime
export ROS_HOME=/ws/anubi/.ros
export GAZEBO_LOG_PATH=/ws/anubi/log/gazebo
export ROS2CLI_NO_DAEMON=1
ros2 launch robot_sim sim_bringup.launch.py gui:=false use_rviz:=false
```

## Simulation Pipeline Map

### Motion path

Gazebo physics -> `go2_sim.urdf.xacro` -> `libgazebo_ros_planar_move.so` -> `/cmd_vel` -> model motion -> `/odom` + `odom -> base_footprint`

### Perception path

Gazebo GPU LiDAR (`/unilidar/cloud`) -> `cloud_self_filter.py` -> `/unilidar/cloud_filtered` -> Nav2 voxel layers -> planner/controller -> `/cmd_vel`

Gazebo GPU LiDAR (`/unilidar/cloud`) -> `pointcloud_to_laserscan` -> `/scan` -> SLAM / Nav2 scan obstacle layers -> planner/controller -> `/cmd_vel`

## Root Cause 1: Self-obstacle Detection

### What was actually wrong

The regression was not caused by a bad Nav2 footprint polygon. The footprint was still reasonable. The real issue was that the simulation costmaps were consuming the raw Gazebo pointcloud from the detailed model through voxel layers, while the new model generated many near-body and grazing ground returns around the sensor/body.

That created a sim-only self-observation path:

`/unilidar/cloud` -> Nav2 voxel layers -> robot body marked as obstacle

At the same time, `/scan` also contained short near returns, but the scan pipeline alone was not what poisoned the local costmap. The failure was specific to feeding the raw 3D sim cloud into the voxel layers.

### Evidence

Raw scan sample from the failing/final sim still showed near returns from `lidar_link`:

- `/scan` ranges included values around `0.0509 m`, `0.0550 m`, `0.0675 m`, `0.0988 m`, `0.1347 m`

Final local costmap check on the fixed build with voxel layers enabled:

- `local_costmap_final cells=6400 width=80 height=80 center_window_nonzero=0`

During a goal on the fixed build:

- `local_costmap_goal cells=6400 width=80 height=80 center_window_nonzero=0`
- `/navigate_to_pose` to `(0.40, 0.00)` returned `SUCCEEDED`
- `/unilidar/cloud_filtered` remained the voxel source while `/scan` stayed unchanged

### Files responsible

- `src/robot_nav/config/nav2_params.yaml` was the shared baseline
- `src/robot_sim/config/nav2_sim_params.yaml` now carries the simulation-specific fix
- `src/robot_sim/scripts/cloud_self_filter.py`
- `src/robot_sim/CMakeLists.txt`
- `src/robot_sim/package.xml`
- `src/robot_sim/launch/sim_bringup.launch.py`
- `src/robot_sim/launch/sim_nav.launch.py`

### Fix

Created a sim-specific filtered pointcloud path for the voxel layers and switched the simulation launches to it.

Changes:

- kept local voxel layer
- kept global voxel layer
- changed the simulation voxel sources from `/unilidar/cloud` to `/unilidar/cloud_filtered`
- added `cloud_self_filter.py` to remove:
  - points inside the robot body envelope
  - very near self-points around the lidar
  - grazing ground returns below a world-height gate
- fixed `z_voxels` from `20` to `16` because Nav2 Humble's voxel layer rejects values above `16`
- kept scan-based `obstacle_layer`
- kept footprint and frame conventions unchanged

### Why this works

The voxel layers themselves were not fundamentally broken. The bad input was the raw Gazebo sim cloud. Filtering out self-points and near-ground grazing hits keeps 3D obstacle marking available while removing the exact data that made the robot block itself.

## Root Cause 2: Tipping in Gazebo

### What was actually wrong

Two coupled model issues caused the tipping:

1. The new detailed model replaced the old planar-safe collision abstraction with a full fixed-joint detailed body/leg collision tree. Gazebo lumped that into one dynamic rigid body, which is incompatible with the repo's planar-move locomotion approximation.
2. After the first hybrid-model repair, the physics frame was still `45 mm` too high (`base_z=0.380` instead of the old stable `0.335`). That kept the COM too high above the flat contact proxy, so the robot still pitched under acceleration even though it no longer exploded at spawn.

### Evidence

Broken intermediate model under goal:

- `/odom.pose.position.z` rose to about `0.275 m`
- `/odom.pose.orientation` left the planar regime
- `gz model -m go2 -i` showed the body tilted with large non-planar quaternion terms

Final fixed model at idle:

- immediate `/odom` sample: `z ~= -1.0e-06`
- 30 s idle `/odom` sample: `z ~= -9.7e-07`
- final Gazebo inertial pose reported COM at `z = 0.329648`, matching the old stable planar model instead of the raised intermediate model (`~0.3746`)

Final fixed model during goal:

- after goal, `/odom.pose.position.x ~= 0.095`
- `/odom.pose.position.z ~= -1.0e-06`
- quaternion stayed planar (`x` and `y` near zero)

### Files responsible

- `src/go2_sim/urdf/go2_sim.urdf.xacro`

### Fix

Replaced the failing detailed-physics model with a hybrid model:

- keep detailed Go2 visuals
- keep fixed frame names
- restore simplified torso collision
- restore old flat contact proxy collision
- remove detailed leg collisions from physics

Then restored the old stable physics height:

- `base_z` back to `0.335`
- detailed visuals and sensor mounts shifted upward with `visual_z_offset=0.045`

### Why this works

The repository does not simulate legged dynamics; it simulates a planar robot driven by `gazebo_ros_planar_move`. The correct fix is therefore to preserve detailed appearance while restoring the old planar collision and COM assumptions that Nav2 and the sim controller were designed for.

## Root Cause 3: Wrong-direction Motion

### What was actually wrong

I did not find a REP-103 frame/sign bug in:

- `base_link`
- `base_footprint`
- `odom`
- `map`
- LiDAR frame orientation
- `gazebo_ros_planar_move` axis mapping

The apparent wrong-direction behavior was a consequence of the bad physics state, not a separate sign inversion:

- once the body tipped, the visual facing direction no longer matched the 2D planning assumption
- Nav2 still issued correct planar commands, but the unstable model made the robot appear to move the wrong way

### Evidence

Final fixed build:

- navigation goal `(0.40, 0.00)` succeeded
- odometry advanced in +X from near zero to about `0.095 m`
- `z` stayed near zero during motion
- a direct positive `angular.z` probe increased odom quaternion `z` from `0.4002537` to `0.4507935`

### Files responsible

- `src/go2_sim/urdf/go2_sim.urdf.xacro`

### Fix

No frame renaming or sign flip was required. Fixing the planar physics model fixed the wrong-direction symptom at the correct layer.

## Old vs New Model Comparison

Focused differences between the last working sim model (`2ffb9a1`) and the failing detailed model:

- old model used one trunk collision plus one flat contact proxy
- old model used `base_link_height_from_ground = 0.335`
- old model did not include detailed colliding legs in physics
- failing model introduced detailed fixed-joint leg/body collisions and a different mass/collision layout
- failing model therefore violated the assumptions of the planar locomotion abstraction

The final repair intentionally returns only the physics behavior to the old sim contract while preserving the new detailed appearance.

## Files Changed

- `src/go2_sim/urdf/go2_sim.urdf.xacro`
- `src/robot_sim/config/nav2_sim_params.yaml`
- `src/robot_nav/config/nav2_params.yaml`
- `src/robot_sim/launch/sim_bringup.launch.py`
- `src/robot_sim/launch/sim_nav.launch.py`
- `src/robot_sim/scripts/cloud_self_filter.py`
- `src/robot_sim/CMakeLists.txt`
- `src/robot_sim/package.xml`

## Verification Commands Run

Build and model validation:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
xacro src/go2_sim/urdf/go2_sim.urdf.xacro >/tmp/go2_sim_postfix.urdf
check_urdf /tmp/go2_sim_postfix.urdf
```

Idle stability:

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
export ROS2CLI_NO_DAEMON=1
timeout 6 ros2 topic echo /odom --once | sed -n '1,80p'
sleep 30
timeout 6 ros2 topic echo /odom --once | sed -n '1,80p'
```

Model state:

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
export ROS2CLI_NO_DAEMON=1
gz model -m go2 -i | sed -n '1,60p'
```

Self-obstacle verification:

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
export ROS2CLI_NO_DAEMON=1
timeout 12 ros2 topic echo /local_costmap/costmap --once --full-length > /tmp/local_costmap_final.yaml
python3 -c "from pathlib import Path; import re; s=Path('/tmp/local_costmap_final.yaml').read_text(); w=int(re.search(r'width: (\\d+)', s).group(1)); h=int(re.search(r'height: (\\d+)', s).group(1)); data=list(map(int,re.findall(r'^-\\s*(-?\\d+)\\s*$', s.split('data:\\n',1)[1], re.M))); cx=w//2; cy=h//2; vals=[data[y*w+x] for y in range(max(0,cy-3), min(h,cy+4)) for x in range(max(0,cx-3), min(w,cx+4))]; print(sum(v>0 for v in vals))"
```

Navigation goal:

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
export ROS2CLI_NO_DAEMON=1
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.40, y: 0.00, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

Turning sign:

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
export ROS2CLI_NO_DAEMON=1
timeout 6 ros2 topic echo /odom --once --field pose.pose.orientation.z
timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout 6 ros2 topic echo /odom --once --field pose.pose.orientation.z
```

## Final Status

| Check | Result | Evidence |
|---|---|---|
| Gazebo launch | PASS | Full sim launch reached active Nav2 lifecycle |
| Idle stability | PASS | 30 s idle with `z ~= 0` and no tipping |
| Forward direction | PASS | Goal to `(0.40, 0.00)` advanced odom +X |
| Turning direction | PASS | Positive `angular.z` increased odom yaw term |
| Self-obstacle detection | PASS | Local costmap center window stayed zero |
| Goal following | PASS | `navigate_to_pose` returned `SUCCEEDED` |

## Remaining Caveats

- The simulated LiDAR still produces some near-body and near-ground returns. That is acceptable as long as the obstacle pipeline does not self-mark the robot, which the final filtered-voxel costmap configuration now avoids.
- Controller loop timing warnings remain (`Control loop missed its desired rate of 20 Hz`), but they no longer prevent stable goal completion.
