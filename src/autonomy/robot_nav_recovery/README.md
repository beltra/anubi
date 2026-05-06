# robot_nav_recovery

`robot_nav_recovery` provides Go2-friendly Nav2 recovery plugins used by
`robot_nav` in both sim and real robot modes.

The package keeps the default command safety chain intact. Recovery behaviors
publish to Nav2's normal behavior-server `cmd_vel` output, which is still
checked by `collision_monitor` and the optional `velocity_limiter`.

## Recovery stages

1. Clear the local costmap around the robot and replan.
2. Select the least-obstructed local costmap sector and rotate toward it with
   continuous collision checking.
3. Back up a short distance only when the rear sector is clear.
4. Step laterally with non-zero `linear.y` only when the side sector is clear.
5. Clear a broader local window and retry the primary planner.
6. Publish a terminal diagnostic and abort.

TF or localization faults are reported as terminal diagnostics in v1. The tree
does not attempt automatic relocalization.

## Tuning parameters

Parameters are set under `bt_navigator.ros__parameters.recovery` and
`behavior_server.ros__parameters`.

- `recovery.costmap_topic`: local costmap topic used by BT checks.
- `recovery.diagnostics_topic`: diagnostic topic for recovery events.
- `recovery.sector_count`: number of angular bins around the robot.
- `recovery.inner_radius_m`: cells closer than this radius are ignored.
- `recovery.outer_radius_m`: maximum radius used for sector scoring.
- `recovery.occupied_cost_threshold`: cost at or above this value is blocked.
- `recovery.unknown_is_occupied`: treat unknown cells as blocked.
- `recovery.blocked_fraction_threshold`: occupied fraction that blocks a sector.
- `recovery.stuck_cmd_speed_threshold_mps`: command threshold for stuck checks.
- `recovery.stuck_odom_speed_threshold_mps`: odometry threshold for stuck checks.
- `recovery.stuck_duration_s`: duration before proactive recovery triggers.
- `guarded_backup.*`: acceleration, deceleration, and minimum speed limits.
- `guarded_lateral_step.*`: acceleration, deceleration, and minimum speed limits.

The local costmap is the v1 obstacle source. Future versions can add pointcloud
or semantic obstacle checks behind the same BT nodes without changing the tree.
