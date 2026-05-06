# Real Go2 Navigation Port

Date: 2026-05-06

## Summary

This port makes the real Go2 Nav2 stack operational with the final command chain:

`Nav2 -> velocity_smoother -> collision_monitor -> velocity_limiter -> go2_hw_bridge -> /api/sport/request`

The Go2-specific bridge remains the only publisher of Unitree Sport API commands. The safety layer owns operator preemption and goal recovery. Real-robot LiDAR IMU input now defaults to `/utlidar/imu`.

## What Changed

- `go2_hw_bridge` now runs `RecoveryStand -> StandUp -> ClassicWalk(true)` at boot and once again when the first Nav2 goal is accepted or executing.
- `go2_hw_bridge` now sends `StopMove` for zero-velocity commands and watchdog stop, and it suppresses nonzero motion commands while a posture sequence is active.
- `joystick_safety` now treats resume and abort as dedicated controller commands instead of generic override keys.
- Joystick override is latched by operator input and no longer clears automatically on neutral sticks.
- Resume resends the last `/goal_pose` through `navigate_to_pose`, forcing Nav2 to replan from the current pose.
- Abort cancels Nav2, holds zero output briefly, then clears override and leaves navigation idle.
- `scripts/run_go2_real.sh` adds `--resume-key-mask` and `--abort-key-mask`.
- `scripts/run_go2_real.sh` accepts the shared `--nav-delay`, `--nav-profile`, `--use-lio`, `--no-stvl`, `--use-route-server`, `--route-graph-file`, `--robot-name`, and `--distortion-imu-topic` options used by the simulation wrapper where the option applies to real hardware.

## Safety Behavior

- Any stick motion or non-command key press immediately cancels Nav2 and publishes `/joystick_override=true`.
- While `/joystick_override=true`, `velocity_limiter` publishes zero velocity on `/cmd_vel_safe`.
- Tilt-estop behavior is unchanged and still uses `recovery_ack_key_mask`.
- Resume and abort controller masks default to `0` and must be calibrated on the real controller.

## Operation

1. Calibrate button masks with:
   `ros2 run robot_safety wireless_key_inspector`
2. Launch the real stack with calibrated masks:
   `scripts/run_go2_real.sh mapping --resume-key-mask <mask> --abort-key-mask <mask> --recovery-ack-key-mask <mask>`
3. Set goals from RViz on `/goal_pose`.
4. Move the controller to suspend autonomy immediately.
5. Press the resume button with neutral sticks to replan to the previous goal.
6. Press the abort button with neutral sticks to cancel the goal and keep Nav2 idle.

## Topics To Check On Hardware

- Sensors: `/sportmodestate`, `/lowstate`, `/wirelesscontroller`, `/utlidar/cloud`, `/utlidar/imu`, `/lidar/points`, `/lidar/imu`, `/lidar/points_undistorted`, `/scan`, `/imu/data`, `/odom_raw`, `/odom`
- Control: `/goal_pose`, `/navigate_to_pose/_action/status`, `/cmd_vel`, `/cmd_vel_collision_free`, `/cmd_vel_safe`, `/api/sport/request`
- Safety: `/joystick_override`, `/safety/tilt_estop`, `/safety/command_source`, `/robot_status`
