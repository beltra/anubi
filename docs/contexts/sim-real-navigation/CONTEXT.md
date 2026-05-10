# Sim-Real Navigation

This context defines the shared navigation and safety language used by both Gazebo simulation and the real Go2.
It exists to keep simulation behavior, real hardware behavior, and wrapper arguments aligned.

## Language

**Sim-Real Contract**:
The shared set of topics, frames, launch arguments, and safety semantics that must match between simulation and real hardware.
_Avoid_: Sim setup, hardware setup

**Command Chain**:
The ordered velocity path from Nav2 output to the final robot command.
_Avoid_: Cmd vel path, control flow

**Collision-Free Command**:
A velocity command that has passed `collision_monitor` but has not necessarily passed final safety arbitration.
_Avoid_: Safe command

**Safe Command**:
The final velocity command after watchdog, joystick override, tilt estop, and source arbitration.
_Avoid_: Collision-free command

**Operator Override**:
A latched operator safety state that cancels autonomous navigation and holds final velocity at zero.
_Avoid_: Manual mode, joystick input

**Resume Command**:
A calibrated controller command that clears operator override and resends the latest navigation goal for replanning.
_Avoid_: Continue button

**Abort Command**:
A calibrated controller command that cancels navigation and leaves Nav2 idle after a controlled stop.
_Avoid_: Stop button

## Relationships

- The **Command Chain** produces exactly one **Safe Command** for the robot adapter.
- A **Collision-Free Command** must still pass the safety arbiter before reaching real hardware.
- An **Operator Override** cancels active navigation before a **Resume Command** can restart goal pursuit.
- A **Resume Command** replans to the latest goal; an **Abort Command** discards that active goal.
- The **Sim-Real Contract** is documented by `docs/topic_map.md` and wrapper `--help` output.

## Example Dialogue

> **Dev:** "Can `collision_monitor` output go straight to `go2_hw_bridge`?"
> **Domain expert:** "No. That is only the **Collision-Free Command**; real hardware must receive the **Safe Command** after final safety arbitration."

## Flagged Ambiguities

- "safe command" was previously used loosely for collision-filtered output; resolved: **Safe Command** means `/cmd_vel_safe`, not `/cmd_vel_collision_free`.
