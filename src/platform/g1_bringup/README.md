# g1_bringup

Unitree **G1** real-robot bring-up. The G1 counterpart of `go2_bringup`. This is the
**hardware** bring-up (not simulation; for the sim see `g1_sim` / `run_g1_sim.sh`).

Unlike `go2_bringup` (which speaks the Unitree Go2 Sport API directly from a C++
`go2_hw_bridge`), the G1 walks under a learned locomotion policy that runs as its
**own process**; `g1_bringup` produces velocity commands from Nav2 and forwards them,
it does NOT run the locomotion controller itself. Two controllers are supported,
selected by the `locomotion` launch arg (never run both at once -- single writer on
`rt/lowcmd`):

- `locomotion:=amo` (default): the **AMO** policy in the separate `g1_locomotion`
  package, commanded over a plain WebSocket (`ws://127.0.0.1:8766`).
- `locomotion:=sonic`: the **SONIC** whole-body controller (GR00T
  `g1_deploy_onnx_ref` on a Jetson), commanded via `cmd_vel_to_sonic`'s ZMQ PUB
  socket (:5556). See `docs/g1_sonic_locomotion.md` for setup + bringup.

The robot-agnostic autonomy (`robot_nav`, `robot_safety`, `robot_lio`) is reused
UNCHANGED; only the G1 adapters differ. Where the Go2 uses `go2_hw_bridge` for the
state and command halves, the G1 uses `g1_state_bridge` (DDS state -> ROS) for STATE and
`cmd_vel_to_amo` / `cmd_vel_to_sonic` (Nav2 velocity -> controller) for COMMAND. The
Go2 wheel/utlidar is replaced by the head-mounted **Livox Mid-360** (livox driver +
`imu_rescale` + DLIO). The STATE side is identical for both controllers.

---

## What this launch starts

`launch/g1_bringup.launch.py` wires the full stack; each block is gated behind a boolean
arg so the file is testable piecewise.

| Step | Node(s) | Purpose |
|------|---------|---------|
| 1 | `robot_state_publisher` + `joint_state_publisher` | G1 URDF (`g1_description/g1.urdf.xacro`) -> TF chain `base_link..mid360_link` |
| 2 | `g1_state_bridge` | DDS body state -> `/odom_raw` + `/imu/data`; owns `base_footprint -> base_link` (body roll/pitch + standing height). `[use_state_bridge]` |
| 3 | `lidar_level_frame_publisher` | gravity-levelled `base_footprint -> lidar_stabilized` (IMU-rate) |
| 4 | `velocity_limiter` (robot_safety) | `/cmd_vel_collision_free` -> `/cmd_vel_safe` `[use_safety]` |
| 5 | `ekf_node` (robot_localization) | fuses DLIO (+ body odom) -> `/odom` and owns `odom -> base_footprint` `[use_ekf_fusion]` |
| 6 | `lidar_odom_relay` | `/lidar_odom` -> `/lidar_odom_reliable` (injects covariances for the EKF) `[use_lio]` |
| 7 | `livox_ros_driver2` + `imu_rescale` | Mid-360 cloud `/livox/lidar`; IMU `/livox/imu` (g) -> `/livox/imu_ms2` (m/s^2) `[use_livox]` |
| 8 | `robot_lio` (`lio.launch.py`) | relays `/livox/lidar -> /lidar/points`, `/livox/imu_ms2 -> /lidar/imu`; runs `dlio_odom_node` -> `/lidar_odom` `[use_lio]` |
| 9 | `slam.launch.py` | `distortion_corrector` (G1 self-filter) `/lidar/points -> /lidar/points_undistorted`; `pointcloud_to_laserscan -> /scan`; `slam_toolbox` (owns `map -> odom`) |
| 10 | `nav2.launch.py` | Nav2 + `collision_monitor` (G1 STVL profiles; AMO or SONIC velocity envelope) `[use_nav]` |
| 11 | `cmd_vel_to_amo` OR `cmd_vel_to_sonic` | `/cmd_vel_safe` -> AMO WebSocket :8766, or SONIC ZMQ PUB :5556 `[use_nav, locomotion]` |
| 12 | RViz2 | optional `[rviz]` |

### Command chain

```
Nav2 /cmd_vel
  -> collision_monitor   /cmd_vel_collision_free
  -> velocity_limiter     /cmd_vel_safe          (robot_safety)
then, by locomotion:
  amo:   cmd_vel_to_amo    ws://127.0.0.1:8766 {vx, vy, yaw}   (this package)
         -> AMO driver (g1_locomotion amo_driver, SEPARATE PROCESS) -> robot
  sonic: cmd_vel_to_sonic  ZMQ PUB tcp://*:5556 planner ticks  (this package)
         -> g1_deploy_onnx_ref (SONIC controller, Jetson, SEPARATE PROCESS) -> robot
```

### Frame contract (identical to the Go2; see `docs/topic_map.md`)

```
map ->(slam_toolbox)-> odom ->(EKF)-> base_footprint
base_footprint ->(g1_state_bridge: body roll/pitch + standing height)-> base_link(==pelvis)
base_footprint ->(lidar_level_frame_publisher, IMU-rate)-> lidar_stabilized
base_link ->(URDF / RSP)-> ... -> mid360_link / imu
```

Odometry: **DLIO is the real odometry source** (`use_lio:=true` default). DLIO publishes
`/lidar_odom` (NOT a TF); `lidar_odom_relay` injects covariances; the EKF fuses it (and
the AMO body odometry `/odom_raw` when available) into a planar `odom -> base_footprint`,
exactly as on the Go2. Nav2 `robot_base_frame = base_footprint`; the nav cloud is
`/lidar/points_undistorted`. Velocity caps depend on the controller: AMO
`vx<=0.5, vy<=0.4, wz<=0.4` (in-distribution range; `nav2_params_g1` +
`cmd_vel_to_amo` + the AMO driver's clamp), SONIC per `sonic_profile`
(`slow_walk` equals the AMO envelope, `walk` is `vx<=0.8, wz<=0.5`;
`nav2_params_g1_sonic_*` + `cmd_vel_to_sonic`, which is the FINAL clamp --
SONIC itself does not limit yaw or ZMQ speed).

---

## Build (real robot)

The vendored **`livox_ros_driver2`** ships `package_ROS2.xml` / `package_ROS1.xml` and
**gitignores the generated `package.xml`**, so colcon cannot discover it until its build
script generates one. Run that script ONCE (it copies `package_ROS2.xml -> package.xml`
and `launch_ROS2/ -> launch/`), then build the workspace:

```
# 1. Generate the livox package.xml (needs the Livox SDK installed; real image only)
src/vendors/livox_ros_driver2/build.sh humble

# 2. Build the G1 real stack
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to g1_bringup g1_locomotion
source install/setup.bash
```

Without step 1, `--packages-up-to g1_bringup` cannot satisfy the `livox_ros_driver2`
dependency and the default `use_livox:=true` launch fails at runtime
(`ros2 run livox_ros_driver2 ...` not found). The non-rosdep Python deps
(`torch`+CUDA, `unitree_sdk2py`, `websockets`, CycloneDDS) must also be present; see
`g1_locomotion/README.md` and `docker/Dockerfile.real`.

---

## Hardware bring-up ORDER

The AMO driver is a **separate process** from this launch.

1. **Power + DDS up.** Power the G1, confirm it is on the network, bring up the correct
   `ROS_DOMAIN_ID` on the compute host, and verify the Livox Mid-360 is reachable (IPs in
   `MID360_config.json`). Pass the robot's NIC with `--net-if <iface>`.

2. **Start the AMO locomotion driver** (`g1_locomotion`, its own process):
   - First a **dry run** to confirm wiring without commanding the legs:
     `ros2 run g1_locomotion amo_driver --observe_only`
   - Then **live** (the robot holds a standing balance and accepts WebSocket velocity
     commands on port 8766): `ros2 run g1_locomotion amo_driver`

3. **Start this bring-up** (state bridge + Livox + DLIO + EKF + Nav2 + the cmd_vel->AMO
   bridge), e.g. via the wrapper:
   ```
   scripts/run_g1_real.sh --net-if eth0 --standing-height 0.78
   ```
   **Hold the robot STILL for the first few seconds** (`--nav-delay`, default 5 s): DLIO
   calibrates its IMU and estimates gravity over the first ~3 s, and SLAM/Nav2 only start
   after the delay. Moving during init makes the gravity/bias estimate diverge.

4. **Send a Nav2 goal** (RViz "2D Goal Pose" or a `nav2_msgs` action) once TF is healthy
   (`/lidar/points` flowing, `/lidar_odom` from DLIO into the EKF `/odom`, `/scan`
   populated, Nav2 lifecycle active).

> The AMO driver runs as its OWN process; start it (step 2) BEFORE this launch so the
> `cmd_vel_to_amo` WebSocket client has a server to connect to.

---

## Launch arguments (subset; see the launch docstring for all)

| Arg | Default | Description |
|-----|---------|-------------|
| `use_sim_time` | `false` | Hardware bring-up; keep false |
| `use_livox` | `true` | Start Livox Mid-360 driver + `imu_rescale` |
| `use_lio` | `true` | Run DLIO (the real LiDAR-inertial odometry source) |
| `use_ekf_fusion` | `true` | Run the EKF; it owns `odom -> base_footprint` |
| `use_state_bridge` | `true` | DDS body state -> `/odom_raw` + `/imu/data` + body TF |
| `use_nav` | `true` | Start Nav2 + `collision_monitor` + the locomotion bridge |
| `locomotion` | `amo` | Low-level controller: `amo` (ws :8766) or `sonic` (ZMQ :5556) |
| `sonic_profile` | `slow_walk` | SONIC envelope: `slow_walk` (vx 0.5, wz 0.4) or `walk` (vx 0.8, wz 0.5) |
| `use_stvl` | `true` | 3D STVL local costmap |
| `use_safety` | `true` | `velocity_limiter` gates `/cmd_vel_collision_free -> /cmd_vel_safe` |
| `rviz` | `true` | Open RViz2 |
| `nav_delay_sec` | `5.0` | Delay before SLAM/Nav2 start (lets DLIO calibrate) |
| `standing_height` | `0.78` | G1 pelvis(base_link) height [m]; forwarded to URDF + state bridge + leveling. **Calibrate on HW.** |
| `net_if` | `""` | DDS NIC the G1 is reachable on (e.g. `eth0`) |
| `livox_config` | `MID360_config.json` | Livox config (host/lidar IPs + `roll:180` mount) |
| `distortion_imu_topic` | `/imu/data` | IMU topic for cloud leveling + `lidar_stabilized` |

Bring up perception/odometry only (no Nav2, no commands):

```
scripts/run_g1_real.sh --no-nav --no-rviz
```

---

## Python nodes

| Executable | Script | Topics |
|------------|--------|--------|
| `g1_state_bridge` | `scripts/g1_state_bridge_node.py` | Unitree DDS state -> `/odom_raw` + `/imu/data` + `base_footprint->base_link` TF |
| `cmd_vel_to_amo` | `scripts/cmd_vel_to_amo_node.py` | `/cmd_vel_safe` -> WebSocket `{vx, vy, yaw}` |
| `cmd_vel_to_sonic` | `scripts/cmd_vel_to_sonic_node.py` | `/cmd_vel_safe` -> SONIC planner ticks (ZMQ PUB :5556; needs `python3-zmq`) |
| `imu_rescale` | `scripts/imu_rescale_node.py` | `/livox/imu` (g) -> `/livox/imu_ms2` (m/s^2) |

All are ROS-runnable (`ros2 run g1_bringup <exe>`); parameters come from
`config/g1_params.yaml` (`cmd_vel_to_sonic`: from the profile files
`config/g1_sonic_slow_walk_params.yaml` / `config/g1_sonic_walk_params.yaml`).

---

## TF ownership (do not duplicate)

`g1_description/g1.urdf.xacro` provides `base_link`, `lidar_stabilized` and `mid360_link`
under `base_link`. To keep single ownership of every edge:
- `slam_toolbox` owns `map -> odom`.
- the **EKF** owns `odom -> base_footprint` (DLIO publishes `/lidar_odom`, NOT a TF).
- `g1_state_bridge` owns `base_footprint -> base_link` (a static fallback is published
  only when `use_state_bridge:=false`, for rosbag replay / bench tests).
- `lidar_level_frame_publisher` owns `base_footprint -> lidar_stabilized`.
