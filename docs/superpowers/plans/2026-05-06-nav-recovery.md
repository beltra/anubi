# Quadruped Nav2 Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a quadruped-aware Nav2 recovery system for the Go2 that replaces the current generic recovery subtree with staged, sensor-aware recovery behavior suitable for both sim and hardware.

**Architecture:** Keep the integration additive to the existing autonomy stack. The current `robot_nav` package continues to own Nav2 launch, behavior tree selection, and navigation parameter profiles, while a new recovery-focused ROS 2 package under `src/autonomy/` owns the custom recovery plugins and any recovery-specific BT nodes. Internal file layout, class split, and test file naming are intentionally left to the implementing agent, who should follow the local ROS 2/Nav2 patterns they find at implementation time.

**Tech Stack:** ROS 2 Humble, Nav2, BehaviorTree.CPP / Nav2 BT plugins, pluginlib, costmap-based obstacle reasoning, `diagnostic_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`.

---

## Repo Fit Check

This plan is still coherent with the current repo as of 2026-05-06.

- `robot_nav` is still the correct integration owner for Nav2 launch, behavior-tree selection, parameter profiles, and docs.
- The current Nav2 launch entrypoint still overrides the default BT XML paths at launch time, so the implementation must update that integration point and not rely only on YAML changes.
- The active navigation profiles still expose `bt_navigator.plugin_lib_names` and `behavior_server.behavior_plugins`, so the custom recovery system can be added without restructuring the stack.
- The runtime command chain still goes through `collision_monitor` and optional `velocity_limiter`; recovery motions must preserve that chain and must not bypass it.
- Nav2 Humble in this workspace still does not expose structured planner/controller failure codes into the BT layer, so recovery branching must infer planner-vs-controller context from the failing subtree or from local tracking nodes rather than from upstream action result codes.
- The wrapper scripts and sim/real top-level launch flows do not need structural changes for v1 unless the implementing agent discovers a real integration gap while wiring the new package into the existing Nav2 launch path.

## Design Constraints To Preserve

- Keep the change additive where possible. Existing navigation modes, profiles, and non-recovery behavior should keep working.
- Support both sim and real robot from the same recovery design, with runtime parameter tuning instead of divergent logic trees.
- Use the projected 2D local costmap as the primary obstacle source for v1 recovery decisions.
- Leave internal package file structure open. The implementing agent should choose the package layout, header/source split, and test organization based on the existing ROS 2 and Nav2 conventions already in the repo.
- Leave room for direct point-cloud or richer localization recovery later, but do not require it for the first implementation.

## Required Runtime Capabilities

### 1. Recovery plugin package

- [ ] Create a new ROS 2 package under `src/autonomy/` dedicated to quadruped recovery integration.
- [ ] Export custom Nav2 behavior plugins through pluginlib so they can be loaded by `behavior_server`.
- [ ] Build and export any custom BT nodes needed by `bt_navigator`.
- [ ] Declare the dependencies needed for Nav2 behavior plugins, BT plugins, costmap access, action interfaces, and diagnostics.
- [ ] Document package-level tuning knobs and operational assumptions in the package README or equivalent local documentation.

### 2. Custom recovery behaviors

- [ ] Implement a behavior that analyzes the local costmap around the robot, bins occupancy into configurable sectors, and returns the least-obstructed escape heading.
- [ ] Implement a rotation behavior that rotates toward a target heading while continuously checking collision along the sweep and aborting if the rotation becomes unsafe.
- [ ] Implement a lateral step behavior that commands non-zero `linear.y`, closes the motion loop against odometry, and validates lateral clearance before and during execution.
- [ ] Keep all recovery motions within the safe limits already implied by the current Go2 locomotion and navigation configs.

### 3. Recovery-specific BT nodes

- [ ] Add BT nodes or wrappers that let the tree:
  - classify planner-vs-controller recovery context,
  - query whether a named or parameterized sector is clear,
  - detect proactive stuck conditions from commanded motion versus observed odometry,
  - publish structured recovery diagnostics,
  - invoke the custom recovery behaviors from the tree.
- [ ] Do not depend on upstream Nav2 error-code plumbing for planner/controller branching; use local context tracking instead.
- [ ] Treat TF/localization faults as a generic terminal diagnostic path rather than a fully automated localization recovery stage in v1.

### 4. Recovery tree behavior

- [ ] Replace the current RoundRobin-based recovery subtree with an escalating recovery sequence that preserves progress across retries.
- [ ] Apply the new recovery logic to both single-goal and through-poses navigation trees so the behavior is consistent across goal-driven flows.
- [ ] Keep the stage order:
  1. local clearing and replan,
  2. escape-heading selection followed by guarded rotation,
  3. guarded short backup,
  4. guarded lateral step,
  5. broader costmap clearing and replan with the primary planner,
  6. terminal diagnostic and abort.
- [ ] Run stuck detection in parallel with nominal navigation so recovery can trigger before the action server fully times out.
- [ ] Keep planner-focused and controller-focused recovery distinct, but do not add a fallback planner in this version.

### 5. Nav2 integration

- [ ] Update the existing Nav2 integration points in `robot_nav` so the custom BT library is discoverable and the custom behaviors are registered with `behavior_server`.
- [ ] Update every navigation profile that is actively used in this repo, not just the default one, so profile-specific runs do not silently keep the old recovery behavior.
- [ ] Update the BT selection wiring so the new recovery tree is actually used at runtime.
- [ ] Add a configurable diagnostics topic name and recovery tuning parameters in the current Nav2 config surfaces.
- [ ] Keep stock Nav2 behaviors registered if they are still used elsewhere.

## Implementation Order

### Task 1: Establish the extension surface

- [ ] Create the new recovery package scaffold under `src/autonomy/`.
- [ ] Add build/export wiring for custom Nav2 behavior plugins, custom BT nodes, and any custom recovery actions if that is the cleanest bridge between BT and `behavior_server`.
- [ ] Make the package buildable in isolation before adding behavior logic.

### Task 2: Build shared recovery primitives

- [ ] Implement the reusable costmap-sector analysis logic.
- [ ] Implement shared diagnostics publishing helpers or a small recovery event utility if useful.
- [ ] Implement a reusable motion monitoring utility for stuck detection and motion completion checks.

### Task 3: Implement custom recovery behaviors

- [ ] Add the escape-heading behavior.
- [ ] Add the guarded rotation behavior.
- [ ] Add the guarded lateral-step behavior.
- [ ] Verify these behaviors obey the existing command and safety pipeline assumptions.

### Task 4: Implement BT-side recovery logic

- [ ] Add the BT-side wrappers/conditions needed for planner-vs-controller context tracking, sector checks, stuck detection, diagnostics, and invoking the custom behaviors.
- [ ] Make sure the BT nodes can be loaded by `bt_navigator` through the existing plugin library mechanism used in the repo.

### Task 5: Replace the current recovery trees

- [ ] Introduce new BT XML definitions for both navigation entrypoints.
- [ ] Swap recovery flow from RoundRobin to escalating memory-preserving behavior.
- [ ] Wire planner-context and controller-context branches to the correct recovery stages.

### Task 6: Wire the existing Nav2 stack to use the new system

- [ ] Update the current Nav2 launch/config integration points in `robot_nav`.
- [ ] Register the new BT plugin library and custom behaviors across the active profiles.
- [ ] Add or update parameter documentation for recovery tuning.

### Task 7: Validate and document

- [ ] Add focused automated tests for sector analysis, sector-clear logic, stuck detection, and BT plugin loading.
- [ ] Run at least one sim smoke validation against a blocked-controller scenario and one planner-failure scenario.
- [ ] Update the relevant navigation docs so future operators understand the recovery stages and tuning surface.

## Acceptance Criteria

- Recovery no longer uses blind spin as a default stage.
- Recovery does not perform blind backup when the rear sector is occupied.
- Lateral motion is available as an explicit recovery stage.
- Planner and controller failures follow different recovery paths.
- Stuck detection can trigger recovery before the full Nav2 action timeout path.
- The new recovery components load through the existing Nav2 stack without breaking current sim or hardware bringup.
- The default Nav2 command safety chain remains intact.

## Open Design Defaults Already Chosen

- Planner-vs-controller branching will be inferred locally in the recovery BT integration, not by extending upstream Nav2 result messages.
- TF/localization faults will publish a structured diagnostic and terminate rather than attempt an automated relocalization behavior in v1.
- Stage 5 will clear broader costmaps and retry the primary planner; it will not switch to a secondary planner in this version.
- The implementing agent chooses the internal package structure, the exact file names inside the new package, and the exact test layout, as long as the work remains idiomatic for this repo and keeps the integration points above intact.

## Coding rules
Follow these coding rules without exception:
- ASCII only — no Unicode, emoji, or non-ASCII comments
- Functions do one thing; keep them short and obviously correct
- No duplication — shared logic lives in one place and is imported/included everywhere it is used
- Parameters are never hardcoded in source; they belong in config files with clear names and inline comments explaining units and valid ranges
- Every non-obvious decision has a single-line comment explaining why, not what
- Remove dead code, commented-out blocks, and unused parameters entirely — do not leave them as comments
- Launch files and config files have a header comment stating their purpose and which robot mode they apply to (sim / real / both)

Do not over-engineer. The target is the smallest correct change, not the most elegant abstraction.
