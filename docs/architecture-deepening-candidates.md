# Architecture Deepening Candidates

> Source: `/improve-codebase-architecture` skill, run on branch `feat/sim2real`, 2026-05-07.
> Each candidate below is shaped as a Linear-ready issue: title, files, problem, proposed solution, benefits, priority, labels. The aim is **deepening** — turning shallow modules into deep ones so the interface is small relative to the implementation, which improves locality, leverage, and testability. Cleanup-only items (orphaned files, dead code) are listed separately at the end.

## Vocabulary

Domain terms come from `CONTEXT-MAP.md` and `docs/contexts/*/CONTEXT.md` (Corrected Cloud, Gravity-Aligned LiDAR Frame, Navigation Profile, Sim-Real Navigation, LIO Fusion, Quadruped Recovery, Platform Adapter, Autonomy Package). Architecture terms (Module, Interface, Seam, Adapter, Depth, Leverage, Locality) come from the skill's LANGUAGE.md.

---

# Package: `robot_nav`

Central autonomy package — Nav2 wiring, distortion correction, gravity-aligned LiDAR frame, EKF launch, profile selection.

## RN-1. Extract a Corrected Cloud Pipeline module from `distortion_corrector`

- **Files:** [src/autonomy/robot_nav/src/distortion_corrector.cpp:626-780](src/autonomy/robot_nav/src/distortion_corrector.cpp#L626-L780)
- **Problem.** `cloud_callback` is 150+ lines that interleave pure geometry (per-point deskew, gravity rotation, self-filter bbox, timestamp resolution, azimuth fallback) with PointCloud2 field iteration and ROS topic IO. The pure logic — given a pose buffer, IMU sample, and cloud, return a corrected cloud — has no seam; you cannot exercise the deskew without spinning a node. Failures hide behind `pcl_ros::transformPointCloud` glue. Deletion test: removing the callback also removes the geometry — currently a one-call site for a real concept.
- **Proposed solution.** Introduce a `CorrectedCloudPipeline` with one entry point: `correct(cloud, pose_buffer, imu_sample, policy) -> Cloud | Drop`. Internally it composes deskew, gravity alignment, and self-filter. The ROS node shrinks to subscribe-route-publish and choosing a `Drop` vs. passthrough policy (per ADR-0004's Missing-State Drop).
- **Benefits.**
  - *Locality:* geometry, timestamp inference, and Drop policy in one place — currently split across the callback, helper struct, and a stale Python reference.
  - *Leverage:* pipeline runs in benchmarks, regression tests, and offline bag replays without ROS.
  - *Tests:* synthetic IMU + fixture cloud replace "spin a node and observe a topic."
- **Priority:** High. **Labels:** `package:robot_nav`, `area:lidar-processing`, `friction:untestable-callback`.
- **ADR alignment:** ADR-0004 (corrected cloud), ADR-0010 (lidar driver frame), ADR-0011 (lidar orientation/IMU). No conflict.

## RN-2. Promote LiDAR geometry to a shared library both C++ and Python consume

- **Files:** [src/autonomy/robot_nav/src/distortion_corrector.cpp:62-115](src/autonomy/robot_nav/src/distortion_corrector.cpp#L62-L115), [src/autonomy/robot_nav/src/lidar_level_frame_publisher.cpp:15-40](src/autonomy/robot_nav/src/lidar_level_frame_publisher.cpp#L15-L40), [src/autonomy/robot_nav/robot_nav/pointcloud_utils.py](src/autonomy/robot_nav/robot_nav/pointcloud_utils.py)
- **Problem.** `roll_pitch_from_imu` and `rotation_from_rpy` are written three times — twice in C++ private namespaces and once in Python. The Python copy is treated as a "reference" but is never the source of truth. If the quaternion convention drifts (axis remap, pitch sign), nodes silently diverge.
- **Proposed solution.** Public C++ header + thin Python module under `robot_nav::geometry` exposing the same primitives. `distortion_corrector` and `lidar_level_frame_publisher` consume the C++ side; debug scripts (`cloud_self_filter.py`) consume the Python side.
- **Benefits.**
  - *Locality:* gravity-alignment math has one home — the Gravity-Aligned LiDAR Frame context now has three implementations.
  - *Leverage:* same primitives serve production node, diagnostic publisher, and Python debug filter.
  - *Tests:* gtest cases on the header replace integration-test-the-whole-node.
- **Priority:** High. **Labels:** `package:robot_nav`, `area:lidar-processing`, `friction:duplication`.

## RN-3. First-class `Navigation Profile` registry

- **Files:** [src/autonomy/robot_nav/launch/nav2.launch.py:107-117](src/autonomy/robot_nav/launch/nav2.launch.py#L107-L117), `config/nav2_params*.yaml`
- **Problem.** ADR-0006 says profiles are selected by launch contract — fine. But the actual profile *registry* is a hardcoded `_profile_map` dict resolved by `OpaqueFunction`. Wrappers (`run_go2_sim.sh`, `run_go2_real.sh`) duplicate the list. `nav2_params_navfn.yaml` is orphaned — strong evidence the registry isn't load-bearing.
- **Proposed solution.** Declarative Navigation Profile registry — single file (YAML or Python data module) mapping `profile_name -> { params_file, requires_stvl, description }`. `nav2.launch.py` reads it; wrappers and `--nav-profile` validation read it; tooling can list profiles. Respects ADR-0006 (selection still happens at launch).
- **Benefits.**
  - *Locality:* adding a profile = one file edit, not three.
  - *Leverage:* validation, `--help` output, orphan detection share the same source.
  - *Tests:* one test asserts every registered profile's YAML exists and parses.
- **Priority:** Medium. **Labels:** `package:robot_nav`, `area:navigation-profiles`, `friction:hidden-registry`.
- **ADR alignment:** ADR-0006. No conflict; reinforces the contract.

## RN-4. EKF/Nav2 params-file mutual exclusion as a launch contract

- **Files:** [src/autonomy/robot_nav/launch/ekf.launch.py:4-10](src/autonomy/robot_nav/launch/ekf.launch.py#L4-L10), [src/autonomy/robot_nav/launch/nav_realrobot.launch.py:69](src/autonomy/robot_nav/launch/nav_realrobot.launch.py#L69)
- **Problem.** Composing `ekf.launch.py` and `nav2.launch.py` requires the caller to pass `params_file:=""` to one of them — otherwise EKF params leak into Nav2. Today this is enforced by a docstring. A future caller will violate it silently.
- **Proposed solution.** Either (a) split EKF's params namespace so passing both is safe, or (b) `nav_realrobot.launch.py` owns the composition explicitly with a launch-time validator that errors when both `params_file` args are non-empty.
- **Benefits.**
  - *Locality:* composition contract becomes machinery, not folklore.
  - *Leverage:* future composites inherit the same protection.
  - *Tests:* a launch test that asserts the failure mode.
- **Priority:** Medium. **Labels:** `package:robot_nav`, `area:launch-contracts`, `friction:undocumented-coupling`.

## RN-5. Topic contract as a generated header (not magic strings in 8 files)

- **Files:** [src/autonomy/robot_nav/src/distortion_corrector.cpp:248](src/autonomy/robot_nav/src/distortion_corrector.cpp#L248), `config/distortion_corrector.yaml`, four `nav2_params*.yaml`, `config/collision_monitor_params.yaml`, `slam.launch.py`, `docs/topic_map.md`
- **Problem.** `/lidar/points_undistorted` and `/lidar/points` appear as string literals in 8+ places. The Corrected Cloud contract has no module — `topic_map.md` is the de-facto source of truth but isn't load-bearing in code.
- **Proposed solution.** Generated header (or `core_interfaces`-owned constants module) deriving canonical topic strings from one registry. YAMLs use launch-time substitutions; C++ includes the header.
- **Benefits.** *Locality:* one edit to rename a topic. *Leverage:* every consumer inherits. *Tests:* doc-vs-code consistency check.
- **Priority:** Low. **Labels:** `package:robot_nav`, `area:topic-contract`, `friction:magic-strings`.
- **Caveat.** Risk of being shallow (a constants header is just a list of strings) unless paired with contract validation against `topic_map.md`.

---

# Package: `robot_safety`

Five Python nodes implementing the Safety Arbiter / Velocity Limiter / Tilt Monitor / Operator Override chain (ADR-0003).

## RS-1. **Wireless Joystick Decoder** module — collapse triple-duplicated key-bit logic

- **Files:** [src/autonomy/robot_safety/scripts/joystick_safety_node.py:28-32,55-56,59-70](src/autonomy/robot_safety/scripts/joystick_safety_node.py#L28-L70), [src/autonomy/robot_safety/scripts/tilt_monitor_node.py:15-19,22-28,55-56,114-123](src/autonomy/robot_safety/scripts/tilt_monitor_node.py#L15-L123), [src/autonomy/robot_safety/scripts/wireless_key_inspector.py:22-40](src/autonomy/robot_safety/scripts/wireless_key_inspector.py#L22-L40)
- **Problem.** `parse_key_mask`, `pressed_keys = current & ~previous`, and the `resume_requested` / `abort_requested` / `should_clear_tilt_estop` predicates are written three times across three nodes. If the joystick protocol changes (different bit order, longer mask), three files must be updated in lockstep. The implicit interface "what does this controller mean" has no module.
- **Proposed solution.** Single `wireless_keys` Python module exposing `parse_key_mask`, `pressed_keys`, and predicate constructors (`resume_predicate(mask)`, `abort_predicate(mask)`). All three nodes import it; each node owns only its node-specific wiring.
- **Benefits.** *Locality:* one place to change protocol logic. *Leverage:* a fourth diagnostic node would import, not re-derive. *Tests:* the existing pure-function tests in `test/test_safety_nodes.py` already point at this — formalize the module so the test imports become natural.
- **Priority:** Medium. **Labels:** `package:robot_safety`, `friction:duplication`.

## RS-2. Extract latch state machines from timer/IMU callbacks

- **Files:** [src/autonomy/robot_safety/scripts/velocity_limiter_node.py:144-165](src/autonomy/robot_safety/scripts/velocity_limiter_node.py#L144-L165), [src/autonomy/robot_safety/scripts/joystick_safety_node.py:173-214](src/autonomy/robot_safety/scripts/joystick_safety_node.py#L173-L214), [src/autonomy/robot_safety/scripts/tilt_monitor_node.py:84-112](src/autonomy/robot_safety/scripts/tilt_monitor_node.py#L84-L112)
- **Problem.** Each node hides a real state machine inside its timer/callback: freshness tracking + source selection (velocity_limiter), six-variable override/resume/abort latch (joystick_safety), warn-and-estop hysteresis with two thresholds (tilt_monitor). Tests exercise pure helpers but the *transitions* — the bit that goes wrong in production — are reachable only by spinning a node. That's exactly the friction the skill calls out: pure logic extracted for testability without locality, while the real bugs hide in the wiring.
- **Proposed solution.** A small state-machine class per node with `step(now, inputs) -> (state, outputs, transitions)`. The ROS callback becomes "collect inputs, call `step`, dispatch outputs." No new abstractions across nodes — each machine stays node-local.
- **Benefits.** *Locality:* the state machine has one home, not five member variables across two methods. *Leverage:* the same machine drives unit tests, sim scenarios, and bag replays. *Tests:* transition tables become the test surface.
- **Priority:** High (safety-critical logic, currently untested in the failure modes). **Labels:** `package:robot_safety`, `friction:untestable-state-machine`.

## RS-3. Source priority as data, not loop order

- **Files:** [src/autonomy/robot_safety/scripts/velocity_limiter_node.py:21-33](src/autonomy/robot_safety/scripts/velocity_limiter_node.py#L21-L33)
- **Problem.** `select_command_source` iterates `['recovery', 'assisted_teleop', 'manual', 'nav']`. This priority order — a load-bearing safety contract — is enforced by a Python list literal, not declared anywhere referenced in ADR-0003 or `topic_map.md`. Adding a fifth source means editing a list.
- **Proposed solution.** Move the priority list to a parameter (or to the Safety Arbiter context module). Document it in `docs/topic_map.md`.
- **Benefits.** *Leverage:* the priority becomes inspectable from launch args and docs. *Tests:* the order is asserted against the doc.
- **Priority:** Low (works today, but invisible to operators).
- **ADR alignment:** Worth mentioning in ADR-0003 as a normative constraint.

## RS-4. Tilt monitor must respect `distortion_imu_topic` contract

- **Files:** [src/autonomy/robot_safety/launch/safety.launch.py:96](src/autonomy/robot_safety/launch/safety.launch.py#L96)
- **Problem.** `tilt_monitor` IMU topic is hardcoded to `/imu/data`. The launch contract in `topic_map.md` and `AGENTS.md` says `distortion_imu_topic` may be `/lidar/imu` for LIO-aware testing. Tilt monitor will read the wrong IMU silently in that mode.
- **Proposed solution.** Expose `imu_topic` as a launch arg defaulting to `/imu/data`; have the wrappers route the same value used for distortion correction.
- **Benefits.** *Locality:* one IMU choice, applied uniformly. **Priority:** Low. **Labels:** `package:robot_safety`, `friction:hardcoded-topic`.

## RS-5. Drop placeholder `dynamic_obstacle_tracker.py`

- **Files:** [src/autonomy/robot_safety/scripts/dynamic_obstacle_tracker.py](src/autonomy/robot_safety/scripts/dynamic_obstacle_tracker.py)
- **Problem.** 17-line script that raises `NotImplementedError`. Belongs to the `feat/dynObstacles` branch, not safety.
- **Proposed solution.** Delete (or move to that branch's package). Cleanup, not deepening — included for completeness.
- **Priority:** Low. **Labels:** `cleanup`, `package:robot_safety`.

---

# Package: `robot_lio`

Optional LIO wrapper around DLIO — relays raw Unitree topics, optionally launches DLIO, publishes frame aliases.

## RL-1. **Reliable Lidar Odometry** as a first-class module — currently split across two packages

- **Files:** [src/autonomy/robot_lio/launch/lio.launch.py:144-163](src/autonomy/robot_lio/launch/lio.launch.py#L144-L163), [src/autonomy/robot_nav/scripts/lidar_odom_relay.py:39-167](src/autonomy/robot_nav/scripts/lidar_odom_relay.py#L39-L167)
- **Problem.** The `LIO Fusion` context defines "Reliable Lidar Odometry" as the concept: covariance injection, EMA smoothing, jump detection, freshness penalty. The implementation is split: `robot_lio` publishes raw `/lidar_odom`; `robot_nav/scripts/lidar_odom_relay.py` (a different package) builds the reliability heuristics on top. A developer asking "where does Reliable Lidar Odometry live?" must search two packages. The concept has no module boundary.
- **Proposed solution.** Move `lidar_odom_relay.py` into `robot_lio` (its natural owner per ADR-0005) and rename it to `reliable_lidar_odometry_node`. Extract the pure quality heuristics (jump detection, EMA, covariance scaling) into a `reliable_odom` Python module that can be tested without ROS.
- **Benefits.** *Locality:* the LIO-fusion concept has one home. *Leverage:* the reliability layer is testable with synthetic odom traces. *Tests:* deterministic fixtures replace "spin DLIO + EKF and watch for drift."
- **Priority:** High. **Labels:** `package:robot_lio`, `area:lio-fusion`, `friction:concept-split-across-packages`.
- **ADR alignment:** ADR-0005. No conflict; reinforces.

## RL-2. Frame alias ownership: URDF vs. launch-time static TFs

- **Files:** [src/autonomy/robot_lio/launch/lio.launch.py:90-140](src/autonomy/robot_lio/launch/lio.launch.py#L90-L140), [src/platform/go2_bringup/launch/go2_bringup.launch.py:255-269](src/platform/go2_bringup/launch/go2_bringup.launch.py#L255-L269), `src/platform/go2_description/urdf/go2.urdf.xacro`
- **Problem.** The `lidar_link → utlidar_lidar` alias and the `lidar_link → lidar_imu_link` IMU offset are each defined in two places: URDF (sim path) and a launch-time static TF (real path). A comment in `go2_bringup.launch.py` says "lidar_link->lidar_imu_link is in go2.urdf.xacro; do not republish" — yet real bringup *does* republish it via launch args. This is folklore-as-contract.
- **Proposed solution.** Choose one owner per frame. ADR-0010 already names the contract; either remove the runtime static TFs (URDF wins always) or remove the URDF entries (launch wins on real, identity on sim). A small ros2 launch test that asserts each TF has exactly one publisher.
- **Benefits.** *Locality:* one source of truth per transform. *Tests:* a TF-uniqueness check.
- **Priority:** Medium. **Labels:** `package:robot_lio`, `area:tf-ownership`, `friction:dual-publisher`.
- **ADR alignment:** ADR-0010, ADR-0011.

## RL-3. EKF config switching is hidden in a `PythonExpression`

- **Files:** [src/platform/go2_bringup/launch/go2_bringup.launch.py:212-214](src/platform/go2_bringup/launch/go2_bringup.launch.py#L212-L214)
- **Problem.** Setting `use_lio:=true` silently swaps `ekf_fusion_params.yaml` for `ekf_fusion_params_lio.yaml` (a fundamentally different fusion strategy: velocity-only wheel odom + DLIO position vs. absolute position + IMU). The switch is a `PythonExpression` inline in launch — invisible to `--show-args`, not surfaced in any doc.
- **Proposed solution.** Promote the EKF mode to a named profile (analogous to RN-3 above) — `ekf_profile: base | lio` — declared in the EKF launch with explicit logging at startup. Same data-driven pattern as Navigation Profiles.
- **Benefits.** *Locality:* mode selection visible in one declarative file. *Leverage:* future modes (LIO + GPS, etc.) follow the same pattern.
- **Priority:** Medium. **Labels:** `package:robot_lio`, `area:lio-fusion`, `friction:hidden-mode-switch`.

## RL-4. DLIO extrinsics format leaks across the autonomy boundary

- **Files:** [src/autonomy/robot_lio/config/dlio_params.yaml:44-54](src/autonomy/robot_lio/config/dlio_params.yaml#L44-L54), `src/autonomy/robot_lio/config/dlio_params_sim.yaml`
- **Problem.** DLIO encodes lidar/IMU extrinsics as 3×3 rotation matrices. The rest of the codebase uses Euler `rpy`. Updating mount calibration means measuring (per ADR-0011), computing the matrix, editing the YAML, *and* updating URDF separately. No validation that the two agree.
- **Proposed solution.** A small build-time generator that takes the URDF rpy values and emits the DLIO matrix YAML — single source (URDF) of truth.
- **Benefits.** *Locality:* one place to change mount calibration. *Tests:* generator output diffed against committed YAML in CI.
- **Priority:** Low (works today, painful when it breaks).
- **Caveat.** Vendor coupling — DLIO format is what it is; this is a wrapper, not a change to DLIO.

---

# Package: `robot_nav_recovery`

Quadruped-aware Nav2 recovery — three behavior plugins + eight BT nodes + pure utilities.

## RR-1. **Sector-aware BT base** — `SelectRecoveryHeading` and `CheckSectorClear` are 90% the same node

- **Files:** [src/autonomy/robot_nav_recovery/src/recovery_bt_nodes.cpp:131-301](src/autonomy/robot_nav_recovery/src/recovery_bt_nodes.cpp#L131-L301)
- **Problem.** Both nodes subscribe to the same costmap, do the same TF lookup, load the same config, and call the same pure analyzer. Only the output differs (heading vs. boolean). The duplication is *systematic* — adding a third sector consumer (e.g., `SelectSidleDirection`) doubles the footprint again.
- **Proposed solution.** A `SectorAwareBTNode` base class owning the costmap subscription, TF lookup, and config caching. Subclasses provide `interpret(SectorReport) -> Output`.
- **Benefits.** *Locality:* costmap/TF wiring in one place. *Leverage:* new sector consumers cost ~30 lines. *Tests:* the base wiring tested once.
- **Priority:** High. **Labels:** `package:robot_nav_recovery`, `friction:duplication`.

## RR-2. **Axis-aligned recovery base** — `GuardedBackUp` and `GuardedLateralStep` are line-for-line clones

- **Files:** [src/autonomy/robot_nav_recovery/src/recovery_behaviors.cpp:228-446](src/autonomy/robot_nav_recovery/src/recovery_behaviors.cpp#L228-L446)
- **Problem.** Identical except for goal validation, command axis, and distance interpretation. Adding another aligned recovery (e.g., diagonal) means another full clone.
- **Proposed solution.** Templated `AxisAlignedGuardedRecovery<Axis>` (or runtime axis member) — collision-checking + cycle update logic in one class.
- **Benefits.** *Locality:* one cycle-update method. *Tests:* parameterized over axes.
- **Priority:** High. **Labels:** `package:robot_nav_recovery`, `friction:duplication`.

## RR-3. Robot-agnostic diagnostic identity (autonomy-boundary fix)

- **Files:** [src/autonomy/robot_nav_recovery/src/recovery_bt_nodes.cpp:495-496](src/autonomy/robot_nav_recovery/src/recovery_bt_nodes.cpp#L495-L496)
- **Problem.** `RecoveryDiagnostics::tick` hardcodes `name = "go2_nav_recovery"` and `hardware_id = "go2"`. Per ADR-0001, autonomy packages must not embed Go2-specific identity.
- **Proposed solution.** Parameterize `diagnostic_name` and `hardware_id` via node parameters; let `go2_bringup` (the platform adapter) inject Go2's identity. Default placeholder: `unknown`.
- **Benefits.** *Locality:* robot identity belongs to the platform adapter. *Leverage:* the package becomes reusable on a non-Go2 quadruped.
- **Priority:** Medium. **Labels:** `package:robot_nav_recovery`, `friction:autonomy-boundary`, `adr:0001`.

## RR-4. Recovery escalation as data, not BT topology

- **Files:** [src/autonomy/robot_nav_recovery/src/recovery_bt_nodes.cpp:466-588](src/autonomy/robot_nav_recovery/src/recovery_bt_nodes.cpp#L466-L588), `nav2_params.yaml` (recovery section)
- **Problem.** "What recovery stages exist, in what order, with what timeouts and diagnostic strings" is split between (a) BT XML tree topology, (b) the `recovery.*` parameter namespace, and (c) string literals in `RecoveryDiagnostics` invocations. There is no single source of truth.
- **Proposed solution.** A declarative recovery-escalation registry (YAML under `config/`) listing stages with their thresholds, timeouts, and diagnostic templates. `EscalatingRecovery` reads the registry; the BT tree references stages by name.
- **Benefits.** *Locality:* one file describes the full escalation policy. *Leverage:* tuning a stage = one edit. *Tests:* a registry-vs-BT consistency check.
- **Priority:** Medium. **Labels:** `package:robot_nav_recovery`, `friction:scattered-policy`.
- **ADR alignment:** ADR-0007 (worth a clarification amendment).

## RR-5. BT integration tests

- **Files:** [src/autonomy/robot_nav_recovery/test/](src/autonomy/robot_nav_recovery/test/)
- **Problem.** Tests cover pure utilities and plugin loading. No test exercises BT execution, behavior cycle steps, or `RecoveryDiagnostics` emission. The bugs that matter — "recovery escalation skipped a stage" — are unreachable through current tests.
- **Proposed solution.** A small `bt_test_harness` that ticks a synthetic tree against a mocked costmap + odometry. Borrow patterns from `nav2_behavior_tree`'s test utilities.
- **Benefits.** *Tests:* the BT-execution surface becomes assertable.
- **Priority:** Medium. **Labels:** `package:robot_nav_recovery`, `friction:test-gap`.

---

# Package: `robot_sim`

Simulation bringup wrapper — composes `go2_sim`, `robot_nav`, `robot_lio` into a sim stack.

## RSI-1. Multi-robot namespace not propagated to SLAM/Nav2

- **Files:** [src/autonomy/robot_sim/launch/sim_bringup.launch.py:66,117,212-238](src/autonomy/robot_sim/launch/sim_bringup.launch.py#L66-L238)
- **Problem.** `robot_name` is declared and passed to `sim_champ.launch.py` but **not** to `slam.launch.py` or `nav2.launch.py`. Real bringup propagates it correctly. Multi-robot sim will silently cross-talk topics.
- **Proposed solution.** Pass `robot_name` (or `namespace`) through every sub-launch include; add a launch-test that two bringups in different namespaces produce disjoint topic sets.
- **Benefits.** *Locality:* namespace contract is uniform. *Leverage:* any future multi-robot sim "just works."
- **Priority:** High (latent bug). **Labels:** `package:robot_sim`, `friction:silent-namespace-leak`.

## RSI-2. Distortion-corrector input not parameterizable in sim

- **Files:** [src/autonomy/robot_sim/launch/sim_bringup.launch.py:172-197](src/autonomy/robot_sim/launch/sim_bringup.launch.py#L172-L197)
- **Problem.** Sim hardcodes `/lidar/points` as input; real bringup exposes `lidar_points_input_topic`. Sim/real parity (ADR-0002) is broken at this seam — testing alternate sensor models in sim requires editing launch code.
- **Proposed solution.** Add `lidar_points_input_topic` and `lidar_imu_input_topic` args mirroring the real-side wrapper.
- **Benefits.** *Locality:* the topic contract is uniform across sim and real.
- **Priority:** Medium. **Labels:** `package:robot_sim`, `area:sim-real-parity`, `adr:0002`.

## RSI-3. Wrapper/launch arg parity gap

- **Files:** [scripts/run_go2_sim.sh:173-184](scripts/run_go2_sim.sh#L173-L184), [src/autonomy/robot_sim/launch/sim_bringup.launch.py](src/autonomy/robot_sim/launch/sim_bringup.launch.py)
- **Problem.** Wrapper exposes ~11 args; launch declares ~20. `--launch-cloud-filter`, `--slam-params`, `--rviz-config`, `--publish-map-to-odom-tf` are reachable only by bypassing the wrapper. `docs/launch_script_parity.md` should make this match a contract.
- **Proposed solution.** Either (a) add wrappers for every launch arg, or (b) declare a curated subset as the wrapper-supported surface in `launch_script_parity.md` and add a test asserting the rest are unset by default.
- **Benefits.** *Locality:* one place lists what the wrapper supports.
- **Priority:** Medium. **Labels:** `package:robot_sim`, `area:wrapper-parity`.

## RSI-4. LIO timing is heuristic, not validated

- **Files:** [src/autonomy/robot_sim/launch/sim_bringup.launch.py:131-168](src/autonomy/robot_sim/launch/sim_bringup.launch.py#L131-L168)
- **Problem.** `lio_start_before_nav_sec=4.0` is a guess; if LIO takes longer to converge, EKF fuses stale odometry. No code-level "is LIO ready" check.
- **Proposed solution.** Either (a) wait on a `/lidar_odom_reliable` first-message check before starting Nav2, or (b) keep the timer but log convergence so operators can spot the gap.
- **Benefits.** *Locality:* readiness is a checked condition, not a timing assumption.
- **Priority:** Low. **Labels:** `package:robot_sim`, `area:startup-sequencing`.

## RSI-5. Validation seam not actually opt-in (no seam at all)

- **Files:** [src/autonomy/robot_sim/package.xml:38](src/autonomy/robot_sim/package.xml#L38), `src/autonomy/robot_sim/launch/sim_bringup.launch.py`
- **Problem.** ADR-0009 says validation is opt-in; `package.xml` declares a dep on `sim_scenarios`, but `sim_bringup.launch.py` has no opt-in launch arg to start a scenario manager. The "seam" is "operator runs two terminals."
- **Proposed solution.** Add `start_scenario:=<scenario_name>` arg that includes `sim_scenarios` launch when set; defaults to none.
- **Benefits.** *Leverage:* repeatable validation runs from one command.
- **Priority:** Low. **Labels:** `package:robot_sim`, `area:validation`, `adr:0009`.

---

# Package: `go2_bringup`

Go2 platform adapter — Sport Mode API bridge, EKF launch, the orchestration entrypoint for real bringup.

## GB-1. Encapsulate Sport Mode API behind the `SportClient` adapter (hide the enum constants)

- **Files:** [src/platform/go2_bringup/include/go2_bringup/sport_client.hpp:23-44](src/platform/go2_bringup/include/go2_bringup/sport_client.hpp#L23-L44)
- **Problem.** 22 `SPORT_API_*` integer constants live at package scope in the public header alongside the `SportClient` class. Anything that includes the header sees the raw API IDs and can build messages by hand, bypassing the adapter. The interface "what can the Go2 do?" is duplicated as a constant list and a method list.
- **Proposed solution.** Move the API IDs into the `.cpp` (anonymous namespace) so they are private to `SportClient`. Expose them only through the typed methods. If the client needs API-version negotiation later, a single `capabilities()` method gives a real seam.
- **Benefits.** *Locality:* Sport Mode knowledge contained in one class. *Leverage:* a future API revision changes one file. *Tests:* the existing JSON-encoding tests already point at the right surface.
- **Priority:** Medium. **Labels:** `package:go2_bringup`, `friction:leaky-encapsulation`.

## GB-2. **Command-Chain Adapter** — make the bridge's input topic a first-class concept

- **Files:** [src/platform/go2_bringup/launch/go2_bringup.launch.py:140-142,163](src/platform/go2_bringup/launch/go2_bringup.launch.py#L140-L163), [src/platform/go2_bringup/src/go2_hw_bridge.cpp:108,150](src/platform/go2_bringup/src/go2_hw_bridge.cpp#L108-L150)
- **Problem.** The bridge subscribes to `/cmd_vel_safe` or `/cmd_vel_collision_free` based on a launch parameter. A reader of `go2_hw_bridge.cpp` cannot tell *which* command topic the bridge consumes — the choice is "magic in launch." This is a Platform Adapter ↔ Cmd Chain coupling expressed as an opaque param.
- **Proposed solution.** Two options to compare in grilling:
  1. Promote the bridge's input role to a named param `cmd_chain_terminal_topic` with documented enum (`safe`, `collision_free`) and log it loudly at startup.
  2. Or: have the bridge always read `/cmd_vel_safe`, and let the safety arbiter run as identity passthrough when `use_safety:=false`. Removes the conditional entirely.
- **Benefits.** *Locality:* the cmd-chain terminus is a declared contract. *Leverage:* future cmd-chain extensions (recovery overlay, manual override) compose without launch tricks.
- **Priority:** High. **Labels:** `package:go2_bringup`, `area:cmd-chain`, `adr:0003`.

## GB-3. Decouple bridge from Nav2 internals (goal-accepted motion sequence)

- **Files:** [src/platform/go2_bringup/src/go2_hw_bridge.cpp:154,366-385](src/platform/go2_bringup/src/go2_hw_bridge.cpp#L154-L385)
- **Problem.** The bridge subscribes to `/navigate_to_pose/_action/status` directly to trigger a startup motion sequence (RecoveryStand → StandUp → ClassicWalk). This means the Platform Adapter knows about a Nav2 internal topic — exactly what ADR-0001 forbids.
- **Proposed solution.** Introduce a small `RobotReadyTrigger` topic (e.g., `/platform/ready_request` of type `std_msgs/Trigger` or a dedicated msg). Whoever needs the robot ready (Nav2 BT, a recovery node, an operator UI) publishes it. The bridge owns "how to get ready" — autonomy owns "when to ask."
- **Benefits.** *Locality:* readiness logic in one place; trigger source independent. *Leverage:* a non-Nav2 caller (sim scenario, benchmark) can request readiness without using Nav2's action machinery. *Tests:* unit-testable trigger flow.
- **Priority:** Medium. **Labels:** `package:go2_bringup`, `area:autonomy-boundary`, `adr:0001`.

## GB-4. Resolve the dual-watchdog overlap (bridge + velocity_limiter)

- **Files:** [src/platform/go2_bringup/src/go2_hw_bridge.cpp:387-406](src/platform/go2_bringup/src/go2_hw_bridge.cpp#L387-L406), [src/autonomy/robot_safety/scripts/velocity_limiter_node.py:144-165](src/autonomy/robot_safety/scripts/velocity_limiter_node.py#L144-L165)
- **Problem.** Both nodes implement a 0.5s watchdog. They cover different failure modes (bridge: cmd_vel never arrives → Sport Mode `StopMove`; velocity_limiter: any source stale → publish zero). Overlap is acceptable defense-in-depth, but the *contract* — "bridge watchdog is the last line, safety arbiter is the upstream gate" — is folklore, not encoded.
- **Proposed solution.** Document the contract in ADR-0003 (already mentions the chain; extend it with watchdog responsibility split). Optionally: bridge watchdog timeout *strictly greater* than safety arbiter's, with a runtime check that errors at startup if they're equal/inverted.
- **Benefits.** *Locality:* watchdog responsibilities documented in one ADR. *Tests:* a startup assertion catches misconfiguration.
- **Priority:** Medium. **Labels:** `package:go2_bringup`, `area:safety-chain`, `adr:0003`.

## GB-5. `go2_hardware.launch.py` default contradicts ADR-0003

- **Files:** [src/platform/go2_bringup/launch/go2_hardware.launch.py:35](src/platform/go2_bringup/launch/go2_hardware.launch.py#L35)
- **Problem.** Default `cmd_vel_topic` is `/cmd_vel_collision_free`, bypassing `velocity_limiter`. This is a test-fixture launch but the docstring claims it "keeps the safety path." That's misleading and a footgun.
- **Proposed solution.** Either rename to `go2_hardware_unsafe.launch.py` and require `--allow-unsafe`, or default to `/cmd_vel_safe` and document the bypass as opt-in.
- **Benefits.** *Locality:* the safety contract is uniform across launches.
- **Priority:** Medium. **Labels:** `package:go2_bringup`, `friction:safety-default`, `adr:0003`.

## GB-6. Integration tests for the bridge node behavior

- **Files:** [src/platform/go2_bringup/test/](src/platform/go2_bringup/test/)
- **Problem.** Only `SportClient` JSON encoding is tested. Watchdog, motion sequence, parameter-driven topic routing, and the EKF / non-EKF TF split are completely untested.
- **Proposed solution.** Launch-test harness with mock SportModeState + cmd_vel publishers asserting topic outputs and TF emissions. Borrow patterns from `nav2_lifecycle_manager` tests.
- **Priority:** Medium. **Labels:** `package:go2_bringup`, `friction:test-gap`.

---

# Package: `go2_description`

URDF/xacro for the Go2. Mostly description data, but xacro has logic and the package is the right home for shared geometry constants.

## GD-1. **Sensor Geometry Module** — single source of truth for LiDAR mount, IMU offset, standing height

- **Files:** [src/platform/go2_description/urdf/go2.urdf.xacro:47-56,169-173](src/platform/go2_description/urdf/go2.urdf.xacro#L47-L173), `src/autonomy/robot_nav/config/distortion_corrector.yaml`, `src/autonomy/robot_lio/config/dlio_params.yaml`, `src/autonomy/robot_lio/launch/lio.launch.py`, `src/autonomy/robot_nav/scripts/cloud_self_filter.py`, `src/platform/go2_bringup/config/go2_params.yaml`, `src/platform/go2_bringup/launch/go2_bringup.launch.py`
- **Problem.** LiDAR mount orientation (`lidar_p=2.88` rad) appears in **8 locations** with different sign conventions across packages — URDF, distortion_corrector YAML, two DLIO YAMLs, sim_bringup, cloud_self_filter, go2_bringup launch, go2_hardware launch. Standing height (`0.28 m`) and LiDAR XYZ position (`0.28945, 0, -0.046825`) similarly triplicated. Re-calibrating mount means editing 5+ files with no validation.
- **Proposed solution.** Treat `go2_description` as the canonical Sensor Geometry Module. Expose constants (mount xyz, mount rpy, standing height, IMU offset) as ros2 parameters loaded from a single YAML (`config/go2_geometry.yaml`), generated at build time from xacro properties. Every consumer reads from the parameter server, not from local YAML literals. The xacro stays the source of truth.
- **Benefits.** *Locality:* one mount calibration, eight consumers. *Leverage:* swapping the LiDAR (e.g., to a different L1 mount kit) is one xacro edit. *Tests:* a CI check that asserts no consumer YAML hardcodes a mount value.
- **Priority:** High (latent calibration footgun). **Labels:** `package:go2_description`, `area:geometry`, `adr:0010`, `adr:0011`.
- **ADR alignment:** ADR-0010 / ADR-0011. Reinforces both — currently the ADRs document a parity intent that the code does not enforce.

## GD-2. Sim/real LiDAR orientation switch as a `xacro:arg`

- **Files:** [src/platform/go2_description/urdf/go2.urdf.xacro:54-56](src/platform/go2_description/urdf/go2.urdf.xacro#L54-L56), `src/simulation/go2_sim/urdf/go2_sim.urdf.xacro`
- **Problem.** Real HW URDF hardcodes `lidar_p=2.88` (physics convention). Sim has a separate URDF in `go2_sim` that uses a different orientation as a Fortress-rendering workaround. No xacro arg switches between them; sim_bringup overrides the value in launch. The two URDFs drift.
- **Proposed solution.** A single xacro file with `<xacro:arg name="use_sim" default="false"/>` selecting the orientation. `go2_sim` consumes the same xacro with the arg flipped.
- **Benefits.** *Locality:* one xacro file for both modes. *Tests:* a doc-vs-xacro check that ADR-0010 / ADR-0011 values match.
- **Priority:** Medium. **Labels:** `package:go2_description`, `area:sim-real-parity`, `adr:0002`.

## GD-3. Cleanup: missing trunk mesh referenced in xacro comments

- **Files:** [src/platform/go2_description/urdf/go2.urdf.xacro:65-98](src/platform/go2_description/urdf/go2.urdf.xacro#L65-L98)
- **Problem.** Comments say "Replace with mesh when available" referencing `meshes/trunk.dae` which does not exist. Placeholder boxes are used.
- **Proposed solution.** Either ship the mesh or remove the comment.
- **Priority:** Low. **Labels:** `cleanup`, `package:go2_description`.

---

# Package: `go2_sim`

Gazebo Fortress assets + bringup for the Go2 simulation.

## GS-1. Bridge YAML as a separate config (not inline launch arguments)

- **Files:** [src/simulation/go2_sim/launch/sim_champ.launch.py:313-343](src/simulation/go2_sim/launch/sim_champ.launch.py#L313-L343)
- **Problem.** All `ros_ign_bridge` topic mappings (`/unilidar/cloud/points → /lidar/points`, `/model/go2/odometry → /odom_raw`, etc.) are inlined as launch args to `parameter_bridge`. The bridge contract — sim's view of the topic_map — has no module. Comparing against `topic_map.md` is a manual-eyeball job.
- **Proposed solution.** Move bridge mappings to `config/ros_gz_bridge.yaml` consumed by `parameter_bridge --bridge-config`. Add a CI check comparing the YAML against `topic_map.md`.
- **Benefits.** *Locality:* sim's topic translation in one file. *Leverage:* easy to mirror for D1 or Harmonic.
- **Priority:** Medium. **Labels:** `package:go2_sim`, `area:sim-real-parity`.

## GS-2. Replace controller-spawner timer magic with a wait-for-controller predicate

- **Files:** [src/simulation/go2_sim/launch/sim_champ.launch.py:361-385](src/simulation/go2_sim/launch/sim_champ.launch.py#L361-L385)
- **Problem.** Spawners delayed by 10s and 15s for Gazebo init. Brittle on slow machines / Fortress patches.
- **Proposed solution.** Use `ros2 control list_controllers` polling (or `controller_manager` service readiness) before spawning. There's no real "deepening" here — but the timer is the kind of magic the skill targets when it hides a real readiness concept.
- **Priority:** Low. **Labels:** `package:go2_sim`, `friction:timer-magic`.

## GS-3. `velocity_limiter` instantiated inside sim launch — diverging from real path

- **Files:** [src/simulation/go2_sim/launch/sim_champ.launch.py:345-358](src/simulation/go2_sim/launch/sim_champ.launch.py#L345-L358)
- **Problem.** Sim launch instantiates `velocity_limiter` directly with `status_topic:=""`. Real launch goes through `safety.launch.py` which exposes more knobs. If `velocity_limiter`'s param interface evolves, sim and real silently diverge.
- **Proposed solution.** Sim should `IncludeLaunchDescription` the same `safety.launch.py` (with sim-appropriate args) instead of instantiating the node directly.
- **Benefits.** *Locality:* one launch file owns the safety chain wiring. *Leverage:* changes to safety chain propagate to both modes.
- **Priority:** Medium. **Labels:** `package:go2_sim`, `area:sim-real-parity`, `adr:0002`.

---

# Package: `sim_scenarios`

Python package — scenario manager, obstacle spawner, MPPI benchmark.

## SC-1. **Scenario interface** — separate the simulator-specific dispatch from the scenario semantics

- **Files:** [src/simulation/sim_scenarios/sim_scenarios/scenario_manager_node.py:76-80](src/simulation/sim_scenarios/sim_scenarios/scenario_manager_node.py#L76-L80), [src/simulation/sim_scenarios/sim_scenarios/benchmark_mppi.py:179-205](src/simulation/sim_scenarios/sim_scenarios/benchmark_mppi.py#L179-L205)
- **Problem.** Scenarios YAMLs are declarative but the loader hardwires Gazebo service paths (`/world/{name}/create`, `/world/{name}/set_pose`). Porting to Harmonic / a different simulator means rewriting the manager. There's a real concept here ("Scenario") that has no module — only a script that calls Gazebo.
- **Proposed solution.** Introduce a `ScenarioBackend` interface with one Gazebo Fortress adapter (today). The `Scenario` itself becomes a typed dataclass loaded from YAML; the manager node composes `Scenario × Backend`. ADR-0009 already calls this out as opt-in — this is the right time to give it a seam.
- **Benefits.** *Locality:* scenario semantics separated from simulator API. *Leverage:* one adapter = hypothetical seam (today); two = real seam. Worth doing now if Harmonic / D1-sim is foreseeable; otherwise mark hypothetical.
- **Priority:** Medium (only if multi-simulator support is on the roadmap; otherwise Low).
- **Labels:** `package:sim_scenarios`, `friction:simulator-coupling`, `adr:0009`.

## SC-2. **Benchmark Run** as a composable API, not a CLI script

- **Files:** [src/simulation/sim_scenarios/sim_scenarios/benchmark_mppi.py:341-342](src/simulation/sim_scenarios/sim_scenarios/benchmark_mppi.py#L341-L342)
- **Problem.** Running a benchmark on a custom scenario requires three terminals + polling `/tmp/` for JSON. There's no Python class that says "given a scenario and a config, run the benchmark and return results." Reuse from a CI script or a notebook is hard.
- **Proposed solution.** Extract `BenchmarkRunner` class with `run(scenario, config) -> Result`. CLI becomes a thin entry point.
- **Benefits.** *Leverage:* benchmark can be driven from CI, from `pytest`, or from a notebook. *Tests:* `Result` becomes the assertion surface.
- **Priority:** Medium. **Labels:** `package:sim_scenarios`, `friction:non-composable`.

## SC-3. Shared `_euler_to_quat` utility (pull duplicated quaternion code into one module)

- **Files:** [src/simulation/sim_scenarios/sim_scenarios/scenario_manager_node.py:270-283](src/simulation/sim_scenarios/sim_scenarios/scenario_manager_node.py#L270-L283), [src/simulation/sim_scenarios/sim_scenarios/spawn_obstacle.py:39-51](src/simulation/sim_scenarios/sim_scenarios/spawn_obstacle.py#L39-L51)
- **Problem.** Quaternion conversion duplicated across two scripts in the same package.
- **Proposed solution.** Single `sim_scenarios.geometry` module.
- **Priority:** Low. **Labels:** `package:sim_scenarios`, `friction:duplication`.

## SC-4. World-name contract between `go2_sim` and `sim_scenarios`

- **Files:** [src/simulation/sim_scenarios/sim_scenarios/scenario_manager_node.py:65](src/simulation/sim_scenarios/sim_scenarios/scenario_manager_node.py#L65), [src/simulation/go2_sim/launch/sim_champ.launch.py:83-86](src/simulation/go2_sim/launch/sim_champ.launch.py#L83-L86)
- **Problem.** `go2_sim` defaults `world_name=default`; `sim_scenarios` defaults `world_name=indoor_office`. If they disagree, scenario loading silently fails. The contract is "share the same string."
- **Proposed solution.** A common ros2 parameter (`/sim/world_name`) set by `go2_sim` at startup; `sim_scenarios` reads it. No more silent disagreement.
- **Benefits.** *Locality:* world identity has one owner. *Tests:* assertion on startup if missing.
- **Priority:** Medium. **Labels:** `package:sim_scenarios`, `area:sim-contract`.

## SC-5. Tests

- **Files:** `src/simulation/sim_scenarios/test/`
- **Problem.** Empty test directory. Spin detection (`benchmark_mppi.py:105-127`) and scenario YAML parsing are untested.
- **Proposed solution.** Unit tests on the pure logic (spin classifier, YAML parser).
- **Priority:** Low. **Labels:** `package:sim_scenarios`, `friction:test-gap`.

---

# Data-only packages (no deepening candidates)

The following packages contain assets only — no Python/C++ logic to deepen. Listed here for completeness:

- **`d1_sim`** — URDF only for the AgiBot D1.
- **`sim_worlds`** — Gazebo world files and models (no logic).
- **`sensor_models`** — SDF sensor model includes.
- **`robot_common_interfaces` / `unitree_api` / `unitree_go`** — ROS message/service definitions only. The only deepening question for interface packages is "is the contract right?" — which belongs to ADR review, not architectural refactor.

# Vendors (out of scope)

- **`champ`, `champ_base`, `champ_msgs`** — vendored quadruped controller; do not edit incidentally.
- **`direct_lidar_inertial_odometry`** — vendored DLIO; wrap in `robot_lio`, do not modify.

---

# Cross-cutting themes

A handful of friction patterns appear across many packages. Each candidate above addresses one instance; the table below names the pattern and links examples.

| Theme | Examples | Architectural form |
| --- | --- | --- |
| **Geometry / topic / frame constants duplicated across N files** | RN-2, RN-5, RL-2, GD-1, RS-4 | A real concept (Gravity-Aligned LiDAR Frame, Corrected Cloud, Sensor Geometry, IMU contract) without a module. |
| **State machines hidden in callbacks** | RN-1, RS-2, RR-2 | Pure logic extracted for testability without locality — exactly the failure mode the skill flags. |
| **Concept split across packages** | RL-1 (Reliable Lidar Odometry), RR-4 (Recovery Escalation), GS-3 (Safety Chain in sim) | The interface "this concept" has no owner; readers search 2+ packages. |
| **Hidden mode switch in launch** | RN-3 (nav profile), RL-3 (EKF mode), GB-2 (cmd-chain terminal) | A first-class concept hidden in `OpaqueFunction` / `PythonExpression` — invisible to `--show-args` and to operators. |
| **Sim/real divergence at unintentional seams** | RSI-2, GS-3, GD-2, GS-1 | ADR-0002 calls for parity but launches diverge silently; bridges, safety wiring, and URDF orientation drift. |
| **Folklore-as-contract** | RN-4 (EKF params leak), GB-4 (dual watchdog ordering), RS-3 (priority order), GB-3 (Nav2 internals coupling) | Invariants documented in comments / docstrings, not enforced by code or ADR. |
| **Test gaps on the integration surface** | RN-1, RR-5, GB-6, SC-5 | Pure helpers tested; the wiring that actually fails in production is unreachable. |

# Top-priority backlog (suggested Linear sprint slice)

If we pick a tight sprint, these have the best ratio of leverage to risk:

1. **GD-1** — Sensor Geometry Module (8-file constant duplication; calibration footgun).
2. **RL-1** — Reliable Lidar Odometry as a first-class concept (concept split across packages).
3. **RN-1** — Corrected Cloud Pipeline extraction (untestable 150-line callback).
4. **RR-1 + RR-2** — Recovery duplication cleanup (deletes ~200 lines, makes adding stages cheap).
5. **GB-2** — Command-Chain terminal as a declared contract (high-leverage safety clarification).
6. **RS-2** — Safety state machines extracted from callbacks (safety-critical, currently untested in failure modes).

# Cleanup / deletions (not deepening, but should land alongside)

- `src/autonomy/robot_nav/scripts/distortion_corrector.py` — dead placeholder.
- `src/autonomy/robot_nav/config/nav2_params_navfn.yaml` — orphaned profile.
- `src/autonomy/robot_nav/config/ekf_fusion_params_lio.yaml` — never wired (would be wired by RL-3).
- `src/autonomy/robot_safety/scripts/dynamic_obstacle_tracker.py` — `NotImplementedError` placeholder.
- `src/platform/go2_description/urdf/` mesh comments — fix or remove.

---

_End of inventory. Suggested next step: pick one high-priority candidate (GD-1, RL-1, RN-1, or RR-1+2 are strong starting points) and run a grilling session to design the deepened interface._
