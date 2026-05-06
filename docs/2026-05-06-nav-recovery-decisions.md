# 2026-05-06 Nav2 recovery decisions

Feature: quadruped-aware Nav2 recovery for the Go2.

## Decisions

- Added `src/autonomy/robot_nav_recovery` instead of putting recovery logic in
  `robot_nav`; `robot_nav` remains the launch/config owner while the new
  package owns loadable Nav2 plugins.
- Reused existing Nav2 action types (`Spin`, `BackUp`, and `DriveOnHeading`) so
  v1 does not add new ROS interfaces or wrapper launch paths.
- Added `EscalatingRecovery` as a BT control node so each successful recovery
  stage returns to Nav2 for a fresh planner/controller retry while preserving the
  next stage across failed retries.
- Used the local 2D costmap as the only obstacle source in v1. Sector analysis
  is shared by `SelectRecoveryHeading` and `CheckSectorClear`.
- Kept stock Nav2 behavior plugins registered because other trees or ad hoc
  operator overrides can still reference them.
- Treated TF/localization faults as terminal diagnostics. Automatic
  relocalization is intentionally out of scope for v1.

## Operational notes

- The command chain remains Nav2 -> velocity_smoother -> `/cmd_vel` ->
  `collision_monitor` -> `/cmd_vel_collision_free` -> optional
  `velocity_limiter`.
- Lateral recovery publishes non-zero `linear.y` through the same behavior
  server command path and depends on the active controller/locomotion profile
  allowing lateral motion.
- Tuning parameters are duplicated across the active Nav2 profiles so profile
  selection cannot silently fall back to the old generic recovery behavior.
