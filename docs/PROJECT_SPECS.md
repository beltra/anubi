# Anubi Project Specification & AI Agent Guidelines

This document defines the goals, architecture, constraints, and coding style for the **anubi** project, with a focus on keeping **simulation and real hardware coherent**, avoiding **duplicated or misleading configuration**, and enabling **AI agents** to safely extend the system.

It is written primarily for automated agents (code assistants, CI bots, etc.) but is equally useful for human contributors.

---

## 1. High-Level Overview

Anubi is a ROS 2–based navigation stack for **legged robots**, currently targeting:

- **Unitree Go2** (primary hardware)
- Future support for **D1 Ultra** and other robots with similar sensing and control capabilities

The stack provides:

- Real‑robot integration via the **Go2 Sport Mode API**
- **Gazebo + CHAMP** simulation that mirrors real-robot behavior
- A **Nav2**-based navigation pipeline (global + local planning, costmaps, recoveries)
- **SLAM / localization** (e.g., slam_toolbox)
- A roadmap toward **odometry fusion** (body state + lidar + IMU)

Key design principles:

- **Sim–real coherence**: same topics and frames, or explicit documented remaps.
- **No duplicated or unclear configuration**: one canonical definition of each parameter.
- **Minimal but accurate documentation**: comments only where they add real value.
- **Robot‑agnostic core**: navigation and fusion should not be hard‑wired to Go2.

---

## 2. Robots and Hardware

### 2.1 Primary robot: Unitree Go2

- Motion control via **Sport Mode API** (topic `/api/sport/request`).
- Gaits such as **ClassicWalk** engaged at startup for stable navigation.
- Odometry derived from `SportModeState` and exposed on `/odom` (with correct frames and covariances).
- Wireless joystick controller topic (e.g., `/wirelesscontroller`) used for **safety override** and manual control.

### 2.2 LiDAR

- Internal LiDAR (e.g. L1) is **mounted on the robot** and its data:
  - Is produced by the internal Go2 computer (not an external PC driver).
  - Is bridged to ROS 2 as a point cloud and/or scan topic.
- The mount may be **tilted**; URDF `lidar_link` RPY and/or driver behavior decide whether a static transform is needed.
- The same LiDAR pipeline must be used consistently in:
  - Real hardware bringup
  - Simulation (Gazebo + CHAMP)

### 2.3 Future robots (e.g. D1 Ultra)

- Must provide:
  - A **body odometry** topic
  - An **IMU** topic (desired)
  - A **LiDAR or scan** topic
  - A TF tree compatible with Nav2 (map → odom → base_footprint → base_link → sensors)
- Anubi’s **core nav & fusion layers** should remain unchanged; only robot adapters / configs should vary.

---

## 3. System Architecture

At a high level:

- **Hardware layer**
  - Robot-specific bridge (e.g., `go2_hw_bridge`) handling:
    - Sport API calls (Move, StopMove, gait control, ClassicWalk, etc.)
    - Reading robot state (e.g., SportModeState) and publishing `/odom`
    - Startup sequences (e.g., RecoveryStand → StandUp → ClassicWalk)
  - Safety nodes (joystick override, velocity limiter)

- **Simulation layer**
  - Gazebo robot description + plugins
  - **CHAMP** for quadruped locomotion in sim
  - Simulation-specific launch files that **mirror hardware topics and frames**

- **Sensing layer**
  - LiDAR pointclouds and/or scans
  - `pointcloud_to_laserscan` (when needed)
  - Preprocessing filters (height limits, self‑filtering, etc.)

- **Localization & mapping**
  - `slam_toolbox` (mapping + localization), fed by `/scan`
  - Planned or existing `robot_localization` EKF to fuse:
    - Body odom
    - IMU
    - (Later) lidar odom / scan-matching odom

- **Navigation**
  - Nav2 stack:
    - Global planner (e.g., Smac Hybrid-A*)
    - Local planner (ideally MPPI)
    - Local & global costmaps with voxel or obstacle layers
    - Recovery behaviors

---

## 4. Simulation with CHAMP

Simulation must remain tightly **aligned with real hardware**:

- **Robot description**
  - CHAMP URDF/xacro must match the real robot geometry and frames used in the nav stack.
  - Joint names in CHAMP configs must match URDF joint names.

- **Topics**
  - Simulation should publish the same high‑level topics that the real robot uses, including:
    - `/odom`
    - `/tf` and `/tf_static`
    - `/joint_states`
    - `/scan` (or a remapped LiDAR pointcloud)
    - `/cmd_vel` as the command interface
  - If the internal implementation needs different topic names, use **explicit remappings** in launch files and add a short comment explaining them.

- **Controllers**
  - In sim, CHAMP / ros2_control should replicate how commands influence the base (e.g., `cmd_vel` dynamics).
  - The simulated odom source should follow the **same frame conventions** as hardware.

- **Consistency checks for agents**
  - Every topic used by Nav2, SLAM, or safety nodes must exist in **both**
    - sim bringup, and
    - hardware bringup,
    - or be clearly documented as sim-only or hardware-only.
  - No simulation-only topic name should be silently different from the hardware name.

---

## 5. Odometry & Fusion Strategy

Current and target design:

- **Body odometry**
  - On Go2, `/odom` is produced from `SportModeState` (body state).
  - Frames:
    - `frame_id`: `"odom"`
    - `child_frame_id`: `"base_footprint"`
  - Covariances should be **non-zero and realistic** (no “perfect odom”).

- **LiDAR use**
  - LiDAR is used at minimum for mapping and costmaps (`/scan` or voxel layer input).
  - Future: support **LiDAR-assisted odom** (ICP/NDT/scan-matching) and fusion with body + IMU.

- **Fusion (robot_localization) — target pattern**
  - Raw sources:
    - `/odom_raw` (body odom)
    - `/imu/data`
    - `/lidar_odom` (later, if added)
  - EKF/UKF node produces:
    - `/odometry/filtered` and/or remapped `/odom`
    - Owns `odom → base_*` TF.
  - Nav2 consumes the **fused** odom.

Agents should preserve or move toward this separation:

- **Do not** hard‑wire Nav2 directly to raw body odom if fusion is introduced.
- Ensure there is a clean place to insert new odom sources without breaking the existing pipeline.

---

## 6. Topic & TF Conventions

To keep sim and hardware aligned:

- **Core topics**
  - `/odom`             — odometry (body or fused, depending on integration level)
  - `/scan`             — 2D LiDAR scan for SLAM & costmaps
  - `/joint_states`     — joint positions for `robot_state_publisher`
  - `/cmd_vel`          — high-level velocity commands
  - `/api/sport/request` — Go2 Sport Mode API requests
  - `/sportmodestate`   — state feedback from Go2
  - `/wirelesscontroller` — Unitree joystick input (Go2 hardware)
  - `/joystick_override` — safety flag (navigation must obey)

- **Frames**
  - `map`              — global frame (from SLAM or localization)
  - `odom`             — local frame (from odom or EKF)
  - `base_footprint`   — ground-projected base
  - `base_link`        — robot base frame
  - `lidar_link`       — LiDAR mount frame (from URDF)
  - `imu_link`         — IMU frame

Rules for agents:

- If sim and hardware use different names for the **same conceptual topic**, fix this by:
  - Renaming in sim to match hardware, or
  - Adding explicit remappings (with a short explanation).
- Do not create new core topic names without a strong reason; stick to the above conventions.
- Ensure there is exactly **one TF publisher** for each transform (e.g., do not duplicate odom broadcasters).

---

## 7. Navigation Stack Peculiarities

- **Local planner**
  - MPPI (or similarly trajectory-based) local planner is preferred for Go2’s gait dynamics.
  - Parameters must obey Go2’s kinematic limits (e.g., no lateral velocity if treated as non-holonomic).

- **Global planner**
  - Smac Hybrid-A* (or equivalent) is preferred to respect turning radius and reduce infeasible paths.

- **Costmaps**
  - Use a **polygonal footprint** representing the Go2 body + leg reach;
    avoid simplistic circular footprints.
  - Prefer a **voxel layer** for obstacles when using the pointcloud, with:
    - Proper `min_obstacle_height` and `max_obstacle_height`
    - Correct LiDAR height and tilt assumptions
  - Inflation radius must reflect the robot’s safety buffer.

- **SLAM / localization**
  - `slam_toolbox` in async mapping mode for exploration.
  - Optionally a separate localization mode when using a saved map.

Agents modifying nav configs must:

- Keep **sim** and **hardware** versions in sync where appropriate.
- Move truly shared parameters into shared config files.
- Avoid duplicating identical parameter blocks across multiple YAMLs.

---

## 8. Safety & Joystick Behavior

Safety is a **hard requirement**, not a convenience:

- **Joystick override**
  - Node subscribes to `/wirelesscontroller`.
  - When any axis or button is active:
    - Publish `/joystick_override = True`.
    - Cancel active Nav2 goals.
    - Ensure navigation (and any autonomous motion) stops.
  - When joystick returns to neutral for N seconds:
    - Publish `/joystick_override = False`.
    - Allow navigation to resume.

- **Velocity limiter**
  - `velocity_limiter_node` (or equivalent) must:
    - Obey `/joystick_override` (publish zero velocity while override is true).
    - Apply hardware-appropriate speed limits and (optionally) gait-aware scaling.

Agents must **never** remove this safety path and must preserve:
- Topic names for the safety interface.
- The semantics that **operator input always wins** over autonomous planning.

---

## 9. Coding Style & Documentation Rules

These rules apply to all languages in the repo (C++, Python, YAML, XML/launch, xacro):

### 9.1 Code style

- Prefer **clear, small functions** over long monolithic ones.
- Use **descriptive names** for nodes, parameters, and topics; avoid cryptic abbreviations.
- Don’t hard‑code magic numbers; use named constants or parameters when values matter.
- Keep error handling explicit: log meaningful errors and warnings.

### 9.2 Comments & docs

- Comments must be:
  - **Accurate** — describe what the code does today.
  - **Necessary** — explain non-obvious logic, hardware quirks, or design rationale.
- Do **not**:
  - Re-state what is obvious from the code (`# increment i` above `i++`).
  - Leave large commented-out blocks with no explanation.
  - Leave TODOs that are already completed or obviously obsolete.
- YAML / parameter comments must:
  - Correctly describe the parameter and its impact on the robot.
  - Not contradict the actual value (e.g., saying “radius 0.3 m” with `0.45` in code is wrong).

### 9.3 Configuration duplication

- A parameter should have **one canonical definition**.
- If the same parameter and value are repeated in multiple files:
  - Consolidate into a shared config and import/overlay where needed.
- If the same parameter has different values in multiple places:
  - This must be deliberate and documented (e.g., “sim vs hardware override”).
  - Otherwise treat as a bug.

### 9.4 Sim vs hardware separation

- Keep simulation-specific configurations in **sim-only** folders/files.
- Keep hardware-specific configurations in **hw-only** folders/files.
- Shared configuration (e.g., Nav2 defaults) should live in a common place and be overlaid as necessary.

---

## 10. Expectations for AI Agents

When you (the AI agent) modify or extend this project, you MUST:

1. **Preserve sim–real coherence**
   - Before changing a topic or frame, search the whole repo to see where it is used.
   - Update both sim and hardware paths, or add a documented remapping.
   - Do not introduce silent divergences.

2. **Avoid duplication**
   - Search before adding new parameters or nodes; reuse when possible.
   - If you see identical blocks in multiple files, consider refactoring.

3. **Maintain safety behavior**
   - Never remove or bypass joystick override and safety gates.
   - If you add new motion sources, ensure they respect the existing safety signals.

4. **Respect style & docs rules**
   - Only add comments where they add clarity.
   - Update existing comments if you change the behavior.
   - Keep README and docs aligned with the actual code.

5. **Design for future robots**
   - When adding new features, think about whether they can be parameterized:
     - Robot dimensions
     - Topic names
     - Frames
   - Prefer adding abstraction layers (adapters, config overlays) over hard‑coding Go2‑only assumptions.

6. **Validate pipelines**
   - For any significant change, explicitly check:
     - Odometry flow: robot/CHAMP/Go2 → /odom → Nav2
     - LiDAR flow: LiDAR/cloud → preprocessing → /scan → SLAM & costmaps
     - Command flow: Nav2 → /cmd_vel → velocity limiter → robot/CHAMP
     - Safety flow: joystick → override → Nav2 + command gating

7. **Keep changes minimal but complete**
   - Prefer the smallest robust change that fully fixes an issue.
   - Avoid partial changes that leave sim and hardware out of sync.