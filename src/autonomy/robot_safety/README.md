# robot_safety

Safety gating layer between collision-checked autonomy commands and the robot.

---

## `velocity_limiter` node

Sits between `/cmd_vel_collision_free` and `go2_hw_bridge`.
Subscriptions: command input, `/robot_status`, `/joystick_override`, and `/safety/tilt_estop`.
Publication: `/cmd_vel_safe` (`geometry_msgs/Twist`).

**Gating rules (first match wins):**
1. Hardware estop, tilt estop, or joystick override active: publish zero velocity
2. No command received within `cmd_timeout` seconds: publish zero velocity
3. Otherwise: forward the collision-free Nav2 command

**Integration:** Enabled by default in `go2_bringup.launch.py` with `use_safety:=true`.
The launch file automatically redirects `go2_hw_bridge` to read `/cmd_vel_safe`.

```
Nav2 /cmd_vel -> collision_monitor -> /cmd_vel_collision_free -> velocity_limiter -> /cmd_vel_safe
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cmd_timeout` | `0.5` s | Watchdog: publish zero velocity if no `/cmd_vel` within this time |
| `publish_rate` | `20.0` Hz | Output `/cmd_vel_safe` publish rate |
| `cmd_vel_in` | `/cmd_vel_collision_free` | Input topic after collision monitor |
| `cmd_vel_out` | `/cmd_vel_safe` | Output topic (to `go2_hw_bridge`) |
| `status_topic` | `/robot_status` | `RobotStatus` input topic |
| `joystick_override_topic` | `/joystick_override` | Operator override input |
| `tilt_estop_topic` | `/safety/tilt_estop` | Tilt estop input |
| `enable_manual_source` | `false` | Enable optional collision-free manual command input |
| `enable_assisted_source` | `false` | Enable optional collision-free assisted teleop input |
| `enable_recovery_source` | `false` | Enable optional collision-free recovery command input |

---

## `joystick_safety` node

Subscribes to `/wirelesscontroller`, publishes `/joystick_override`, and cancels active Nav2 goals when any controller axis or key is active. It remembers the latest `/goal_pose`; pressing the calibrated START key with neutral sticks clears override and resends that goal through `navigate_to_pose`, forcing Nav2 to compute a new path from the current pose. Override also clears without resuming after the controller is neutral for `neutral_hold_secs`.

---

## `tilt_monitor` node

Subscribes to `/imu/data` and publishes latched `/safety/tilt_estop`. Defaults are warn at 20 deg, estop at 30 deg for 0.25 s, and allow recovery below 15 deg after the calibrated controller START key is pressed.

Use `ros2 run robot_safety wireless_key_inspector` on the real robot to find the START key bit, then launch with `recovery_ack_key_mask:=<mask>` or `scripts/run_go2_real.sh --recovery-ack-key-mask <mask>`.

Parameters are set via `safety.launch.py` launch arguments, not `config/safety_params.yaml`.

---

## Launch

| File | Purpose |
|------|---------|
| `launch/safety.launch.py` | Start safety arbiter, joystick override, and tilt monitor |

---

## Real Robot Note

Safety is enabled by default in `scripts/run_go2_real.sh`. Use `--no-safety` only for unloaded bench tests.
