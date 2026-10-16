# Usage

All runs go through four wrapper scripts. Each one takes `mapping` (the default) or
`localization` as its first argument, and `--help` lists every option.

| Script | Robot |
|---|---|
| `scripts/run_go2_sim.sh` | Go2 in Gazebo |
| `scripts/run_go2_real.sh` | Go2 hardware |
| `scripts/run_g1_sim.sh` | G1 in Gazebo |
| `scripts/run_g1_real.sh` | G1 hardware (see [g1.md](g1.md)) |

Each script is a thin wrapper around a launch file (`robot_sim sim_bringup.launch.py`,
`go2_bringup go2_bringup.launch.py`, and the G1 equivalents). Run
`ros2 launch <pkg> <file> --show-args` to see the launch arguments directly.

## Map, then localize

1. Start in mapping mode and walk the robot around. You can send Nav2 goals into open
   space or drive it with the controller.

   ```bash
   scripts/run_go2_sim.sh
   ```

2. While it's still running, save the map from a second terminal:

   ```bash
   scripts/save_current_maps.sh --output-dir ~/maps/office
   ```

   This writes the slam_toolbox posegraph (`slam_map.posegraph` + `.data`), a Nav2
   occupancy map (`nav2_map.yaml` + image) and, when DLIO is running, a 3D PCD.

3. Next time, start in localization mode on that map:

   ```bash
   scripts/run_go2_sim.sh localization --map ~/maps/office/slam_map
   ```

   `--map` accepts the posegraph path with or without the `.posegraph` extension.

## Sending goals

In RViz, click **2D Goal Pose** and drag. From the command line:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

## Common options

| Option | What it does |
|---|---|
| `--no-rviz` | Run without RViz |
| `--nav-profile <id>` | Nav2 tuning: `default_safe` (default), `narrow_gap_1p0m`, `forward_static` |
| `--no-stvl` | Use a plain 2D obstacle layer instead of the STVL voxel layer |
| `--use-lio` | Run DLIO LiDAR-inertial odometry and fuse it in the EKF. G1 real has it on by default; turn it off with `--no-lio` |
| `--nav-delay <sec>` | Wait before starting SLAM/Nav2 |
| `--world <path>` | Sim only. Worlds are in `src/simulation/sim_worlds/worlds/` |
| `--gui` / `--no-gui` | Sim only. Show or hide the Gazebo client |
| `--no-safety` | Real only, bench tests only. Bypasses `velocity_limiter` |
| `--record-rosbag` | Go2 real only. Records perception topics to `bags/` |

The [parity table](launch_script_parity.md) shows which options each script supports.

## Simulation obstacles

```bash
# drop a box at (2, 1), then remove it
ros2 run sim_scenarios spawn_obstacle --name box_1 --model obstacle_box --x 2.0 --y 1.0 --z 0.5
ros2 run sim_scenarios spawn_obstacle --delete box_1

# load a scenario: crowded_room, narrow_corridor, empty
ros2 run sim_scenarios scenario_manager --ros-args -p scenario_file:=crowded_room
```

## Safety controls

On hardware, the last thing a command passes through before the robot is
`velocity_limiter`. It sends zero velocity when any of these is true:

- No fresh command has arrived for 0.5 s, for example because Nav2 died (watchdog).
- You touched a stick or a non-command key on the Unitree controller (joystick
  override). This cancels the active Nav2 goal, and the override stays latched until
  you press the resume key with the sticks centered. Resume re-sends the last goal;
  the abort key drops it and leaves Nav2 idle.
- Body roll or pitch went past 30 deg (tilt e-stop). To clear it, get the robot back
  below 15 deg and press the recovery key.

The key masks depend on the controller. To find yours, run:

```bash
ros2 run robot_safety wireless_key_inspector
```

Then pass them with `--resume-key-mask`, `--abort-key-mask` and
`--recovery-ack-key-mask`.

If the robot ignores goals, check which source the limiter is currently passing:

```bash
ros2 topic echo /safety/command_source   # nav | joystick_override | tilt_estop | watchdog | ...
```

A value stuck at `joystick_override` means the resume key hasn't been pressed.

## Offline replay

You can tune perception on recorded bags without the robot:

```bash
scripts/replay_bag_perception.sh bags/go2_real/<bag>        # Go2
scripts/replay_bag_perception_g1.sh bags/g1/<bag>           # G1
```

## Cleanup

```bash
scripts/kill_ros_nodes.sh
```

This kills every node the stack starts. It also clears stale FastDDS shared memory.
