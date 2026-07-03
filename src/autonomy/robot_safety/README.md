ros2 run robot_safety wireless_key_inspector# robot_safety

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

Subscribes to `/wirelesscontroller`, publishes `/joystick_override`, and cancels active Nav2 goals when any controller axis or non-command key is active. It remembers the latest `/goal_pose`; pressing the configured resume key with neutral sticks clears override and resends that goal through `navigate_to_pose`, forcing Nav2 to compute a new path from the current pose. Pressing the configured abort key cancels Nav2, holds zero velocity briefly, then clears override and leaves navigation idle.

### Override latch is by design

Once `/joystick_override` flips True for any reason (axis above `joystick_deadzone`, or a non-command key pressed), the node holds it True until the operator presses the resume key. Watchdog timeout, sticks returning to neutral, and the controller going offline do **not** clear override. This is intentional: the operator must explicitly acknowledge re-handoff to autonomy.

**Operational symptom of forgetting the resume press:**
- Nav2 publishes `/cmd_vel_nav` and `/plan` normally (controller_server is happy)
- `/safety/command_source` stays `joystick_override`
- `/cmd_vel_safe` stays zero
- Robot does not move

**Verification from a bag (or live):**
```
ros2 topic echo /safety/command_source
ros2 topic echo /joystick_override
```
If `command_source` is `joystick_override` while `/cmd_vel_nav` is non-zero, the resume key has not been pressed. Press the configured `resume_key_mask` on the controller (default `0x0010`) once with neutral sticks. `/joystick_override` flips to `false`, the last `/goal_pose` is automatically re-sent through `navigate_to_pose`, and `command_source` transitions to `nav`.

This is the most common reason "robot does not reliably move on goal" during real-robot mapping runs; the default rosbag topic set in `scripts/run_go2_real.sh` includes the full cmd_vel chain plus `/joystick_override` and `/safety/command_source` so this state is reproducible offline.

---

## `tilt_monitor` node

Subscribes to `/imu/data` and publishes latched `/safety/tilt_estop`. Defaults are warn at 20 deg, estop at 30 deg for 0.25 s, and allow recovery below 15 deg after the calibrated controller START key is pressed.

Use `ros2 run robot_safety wireless_key_inspector` on the real robot to find the controller bitmasks, then launch with:

- `recovery_ack_key_mask:=<mask>` for tilt-estop recovery
- `resume_key_mask:=<mask>` for Nav2 resume and replan
- `abort_key_mask:=<mask>` for controlled Nav2 abort

Parameters are set via `safety.launch.py` launch arguments, not `config/safety_params.yaml`.

---

## Launch

| File | Purpose |
|------|---------|
| `launch/safety.launch.py` | Start safety arbiter, joystick override, and tilt monitor |

---

## Real Robot Note

Safety is enabled by default in `scripts/run_go2_real.sh`. Use `--no-safety` only for unloaded bench tests.
