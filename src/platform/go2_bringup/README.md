# go2_bringup

Go2-specific hardware adapter.  The **only** package in the stack that speaks the
Unitree Go2 protocol.

---

## `go2_hw_bridge` node

The central hardware bridge.  Single node that handles both directions:

### State → ROS direction
- Subscribes to `/sportmodestate` (`unitree_go/SportModeState`, ~50 Hz)
- Subscribes to `/lowstate` (`unitree_go/LowState`) — for battery state (`bms_state.soc`, `power_v`)
- Publishes `/odom` (`nav_msgs/Odometry`)
- Publishes `/imu/data` (`sensor_msgs/Imu`)
- Publishes TF: `odom → base_footprint`  (planar x, y, yaw)
- Publishes TF: `base_footprint → base_link`  (standing height + roll/pitch from IMU)
- Publishes `/robot_status` (`robot_common_interfaces/RobotStatus`, 1 Hz)

### Command → hardware direction
- Subscribes to `/cmd_vel` (`geometry_msgs/Twist`) from Nav2
- Clamps velocities to configured limits
- Calls `SportClient::Move(vx, vy, vyaw)` → publishes `unitree_api/Request`
  on `/api/sport/request` (consumed by Go2 onboard controller)
- Startup sequence (default enabled): `StandUp` once, then `ClassicWalk(true)`
- **Watchdog**: sends `StopMove` if no `/cmd_vel` arrives within `cmd_timeout` seconds

### Parameters (`config/go2_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `state_topic` | `/sportmodestate` | Go2 sport mode state (pose, velocity, IMU) |
| `lowstate_topic` | `/lowstate` | Go2 low-level state (battery: `bms_state.soc`, `power_v`) |
| `cmd_vel_topic` | `/cmd_vel` | Nav2 velocity command |
| `sport_req_topic` | `/api/sport/request` | Go2 sport API endpoint |
| `odom_topic` | `/odom` | Odometry topic published by bridge (`/odom_raw` when fusion is enabled) |
| `imu_topic` | `/imu/data` | IMU topic published by bridge for fusion / deskew |
| `odom_frame` | `odom` | Odometry frame |
| `base_frame` | `base_footprint` | Robot base frame (2D footprint) |
| `base_link_frame` | `base_link` | Body frame receiving roll/pitch TF |
| `imu_frame` | `imu_link` | Frame stamped on `/imu/data` |
| `publish_tf` | `true` | Publish TF `odom → base_footprint` (disable when EKF owns this TF) |
| `publish_body_tf` | `true` | Publish TF `base_footprint → base_link` |
| `standing_height` | `0.28` m | Z offset used in `base_footprint → base_link` |
| `cmd_timeout` | `0.5` s | Stop if no cmd_vel within this time |
| `enable_startup_motion_sequence` | `true` | Send startup `StandUp` then `ClassicWalk(true)` on hardware bringup |
| `startup_standup_delay_sec` | `1.0` s | Delay from first state sample to `StandUp` |
| `startup_classic_walk_delay_sec` | `3.0` s | Delay after `StandUp` before `ClassicWalk(true)` |
| `max_linear_vel` | `1.0` m/s | Forward/backward limit |
| `max_lateral_vel` | `0.3` m/s | Lateral limit |
| `max_angular_vel` | `1.5` rad/s | Rotation limit |

---

## `SportClient` library

`include/go2_bringup/sport_client.hpp` + `src/sport_client.cpp`

C++ class that encodes Unitree sport mode commands as JSON-parameterised
`unitree_api/Request` messages.  Uses **nlohmann/json** (MIT, system package).

The full command list is in the header.  Key commands used by navigation:
- `Move(req, vx, vy, vyaw)` — velocity command
- `StopMove(req)` — immediate stop
- `StandUp(req)` / `StandDown(req)` — posture control
- `RecoveryStand(req)` — recover from fall

---

## Launch Files

| File | Purpose |
|------|---------|
| `go2_bringup.launch.py` | **Master entry point** — full stack |
| `go2_hardware.launch.py` | Hardware only (no SLAM/Nav2) |
| `go2_nav_rviz.launch.py` | Nav2 + RViz (assumes hw + SLAM already running) |

---

## Porting to AgiBot D1 Ultra

Create `agibot_d1_bringup` with `d1_hw_bridge.cpp` following the same pattern.
Use `go2_hw_bridge.cpp` as the implementation template.

Zero changes required outside `agibot_d1_bringup`.
