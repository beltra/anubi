# Nav2 Recovery

This context defines the recovery language for quadruped-aware Nav2 behavior.
It exists because recovery decisions must account for legged motion, local clearance, and operator diagnostics.

## Language

**Recovery Stage**:
One ordered attempt to escape a planning or control failure before retrying the primary task.
_Avoid_: Recovery action

**Guarded Recovery Behavior**:
A Nav2 behavior that checks local clearance while publishing through the normal command path.
_Avoid_: Custom behavior

**Escape Sector**:
A local-costmap angular region scored for rotation, backup, or lateral step clearance.
_Avoid_: Direction bucket

**Lateral Step**:
A recovery motion using non-zero sideways velocity when the side sector is clear.
_Avoid_: Strafe

**Terminal Diagnostic**:
A recovery event that reports why autonomy cannot continue safely.
_Avoid_: Failure log

## Relationships

- A **Recovery Stage** returns control to Nav2 for a fresh planner or controller retry.
- A **Guarded Recovery Behavior** must preserve the shared command chain.
- An **Escape Sector** can authorize rotation, backup, or a **Lateral Step**.
- A **Terminal Diagnostic** ends v1 recovery for TF, localization, or exhausted-recovery faults.

## Example Dialogue

> **Dev:** "Should the robot try relocalization if recovery fails?"
> **Domain expert:** "Not in v1. TF and localization faults produce a **Terminal Diagnostic** instead of automatic relocalization."

## Flagged Ambiguities

- "recovery" can refer to the BT tree, a behavior plugin, or an individual stage; use **Recovery Stage** or **Guarded Recovery Behavior** when that distinction matters.
