# Mid-360 + Nav2 Reference Designs: Survey and Tuning Reference

Purpose: a tuning reference for the Livox Mid-360 -> LIO -> Nav2 pipeline in this
workspace (G1 primary, Go2 where noted), grounded in a verified survey of
published open-source designs. Every external claim below was checked against
the actual source/config files of the cited repo (multi-vote adversarial
verification, July 2026). Where the survey and this workspace disagree, the
divergence is stated explicitly so an agent can decide whether it is a tuning
lead or an intentional local decision.

Authority note: this document is a reference, not a contract. Runtime contracts
remain `docs/topic_map.md`; profile selection remains ADR 0006; local pipeline
decisions remain ADRs 0004/0005/0008/0010/0011.

---

## 1. The three architecture families found in the wild

No single canonical Mid-360 + Nav2 reference design exists. Credible designs
split into three families:

1. **Full 3D LIO + ground segmentation + Nav2** (pb_rm_simulation,
   FAST_LIO_ROS2-based stacks). FAST-LIO2 or Point-LIO produces odometry; a
   separate ground-segmentation node (linefit_ground_segmentation) strips the
   floor before the cloud reaches the costmap; custom TF glue satisfies
   REP-105 because no FAST-LIO port publishes `map->odom` / `odom->base_link`.
2. **2D projection** (arpa-byte/Go2_where_r_u). `livox_ros_driver2` ->
   `pointcloud_to_laserscan` -> slam_toolbox, odometry from the robot's own
   SDK bridge. No LIO at all. Viable when 2D mapping suffices.
3. **STVL-centric costmap** (linorobot2 STVL variants, Nav2 STVL tutorial).
   PointCloud2 fed directly into `spatio_temporal_voxel_layer`; per-source
   height clipping + time decay replace an explicit ground filter.

**This workspace is a hybrid of all three**: DLIO (family 1, but DLIO instead
of FAST-LIO and no explicit ground segmentation), pointcloud_to_laserscan into
slam_toolbox for the map (family 2), and STVL for local obstacles (family 3),
tied together by a gravity-leveled corrected cloud (`lidar_stabilized`,
ADR 0004) that substitutes for ground segmentation via height gating.

---

## 2. SLAM / odometry: how reference designs did it

### 2.1 LIO backend choice

| Design | LIO | Rate | Notes |
|---|---|---|---|
| pb_rm_simulation (RoboMaster sentry; most complete Mid-360+Nav2 reference) | FAST-LIO2 or Point-LIO (selectable) | ~10 Hz / 100+ Hz | Point-LIO described as friendlier to Nav2's control loop, at higher CPU cost |
| Ericsii/FAST_LIO_ROS2 (standard Humble port) | FAST-LIO2 | ~10 Hz | Its `config/mid360.yaml` is the ecosystem LIO baseline (Section 5.1) |
| deepglint/FAST_LIO_LOCALIZATION_HUMANOID (G1/Go2) | FAST-LIO2 + map matching vs offline .pcd | - | Localization only, no Nav2 costmap layer |
| Go2_where_r_u | none (SDK odom + slam_toolbox 2D) | - | FAST-LIO listed in README but never launched |
| **This workspace** | **DLIO** (vendored) | ~10 Hz (Mid-360 frame rate) | `robot_lio/config/dlio_params_g1_real.yaml` |

No surveyed design used DLIO; the ecosystem default is FAST-LIO2. This is not
a reason to switch: DLIO gives the same class of odometry, and the local
config already carries quadruped-specific fixes no FAST-LIO reference has
(accel-bias caps for heel strikes, `Kab: 0.85`, `abias_max: 1.0`).

### 2.2 TF wiring (the universal integration problem)

Verified: FAST_LIO_ROS2 broadcasts exactly one TF, `camera_init->body`,
hardcoded in `laserMapping.cpp` (not parameterized, not remappable), and
publishes `/Odometry` in those same frames. **No FAST-LIO port satisfies
REP-105 out of the box.** Every Nav2 integration adds one of:

- an EKF fusing LIO odometry as a topic input (what this workspace does:
  DLIO `publish_tf: false` -> `/lidar_odom` -> covariance-injecting relay
  `/lidar_odom_reliable` -> `robot_localization` EKF owns
  `odom->base_footprint`; ADR 0005);
- source-level frame edits (pb_rm_simulation's Point-LIO fork does
  `lookupTransform(livox_frame, base_link)` in `laserMapping.cpp` and
  broadcasts child `base_link`, with the body->LiDAR extrinsic user-measured
  in `measurement_params_real.yaml`);
- a standalone frame republisher node.

The local EKF-fusion pattern is the most robust of the three (covariance
injection, staleness/jump downscaling in `lidar_odom_relay.py`) and matches
what the ecosystem converges on. Keep it.

### 2.3 map->odom

Surveyed designs either let slam_toolbox own `map->odom` (2D projection
family, and this workspace) or run FAST-LIO's map frame as a de-facto global
frame with localization-vs-prior-map matching (deepglint). No design let the
LIO publish `map->odom` directly into Nav2.

---

## 3. Point cloud -> costmap: how obstacles were detected

### 3.1 Ground handling, three approaches

| Approach | Who | Critical parameter | Failure mode |
|---|---|---|---|
| Explicit ground segmentation (linefit) | pb_rm_simulation | `sensor_height` (measured LiDAR-to-ground, e.g. 0.59 m) | Mis-measured height -> floor marked as obstacles. Upstream linefit README: "you have to adjust the sensor_height to your robot specifications, otherwise, it will not work." |
| Height clipping per STVL source | linorobot2 STVL, Nav2 tutorial | `min_obstacle_height` / `max_obstacle_height` | Fixed z gate fails if the cloud frame tilts with the body |
| Gravity-leveled frame + height gate | **this workspace** (ADR 0004) | `min_obstacle_height: 0.25` in `lidar_stabilized`; scan band [-0.95, +0.20] | Leveling residual (0.7 deg median / 2.6 deg p90 while walking) tilts far floor returns above the gate; mitigated by bounding `obstacle_range` to 5.0 m |

The local approach is a superset of the ecosystem's height-clipping approach:
`lidar_stabilized` makes the z gate valid under body pitch/roll, which the
static-clip designs (wheeled robots) never had to solve. The pb_rm lesson
transfers directly though: **the sensor-height / floor-offset calibration is
the number-one false-obstacle knob.** Locally that is the observed floor at
odom z ~= +0.08 while walking (DLIO z drift + sensor height calibration),
which is why `min_obstacle_height` is 0.25 and not the tutorial's 0.0.

### 3.2 Obstacle layer choice

Verified from the STVL README (humble branch) and Nav2 docs:

- **STVL** is the maintainer-recommended path for 3D lidars: OpenVDB-backed,
  consumes PointCloud2 directly, models 3D lidars with an hourglass-shaped
  clearing frustum (`model_type: 1`; the 6-plane frustum `model_type: 0` is
  for depth cameras). Explicitly NOT recommended for planar 2D lidars.
- **Nav2 voxel_layer** (used in local `nav2_params_2d.yaml` non-STVL profile)
  has a hard vertical limit: max 16 z-voxels (bit-packed 32-bit column),
  defaults `z_voxels 10`, `z_resolution 0.2`, `origin_z 0.0` (2.0 m
  coverage). Finer z resolution trades away vertical range. This constraint
  is a main reason quadruped designs favor STVL.
- **pointcloud_to_laserscan** into an obstacle layer appears only in the 2D
  projection family; here it feeds slam_toolbox and collision_monitor, not
  the costmap. That matches the ecosystem.

### 3.3 The recurring STVL pattern (verified against config files)

Nav2 STVL tutorial (reference parameterization): plugins
`["static_layer", "stvl_layer"]` in BOTH local and global costmaps;
`voxel_decay 15` linear, `voxel_size 0.05`, `mark_threshold 0`,
`combination_method 1`, `transform_tolerance 0.2`,
`update_footprint_enabled true`; per source: `obstacle_range 3.0`,
`min/max_obstacle_height 0.0/2.0`, frustum `min_z 0.1 / max_z 7.0`,
`decay_acceleration 15.0`. Caveat: the tutorial sensor is a RealSense
(`model_type 0`); a Mid-360 must use `model_type 1` and lidar FOV values, and
the frustum/decay_acceleration numbers are depth-camera starting points.

linorobot2 STVL variant (`navigation_3d.yaml`, values read from the file):
local plugins `[stvl_layer, inflation_layer]` with `voxel_decay 5.0` linear;
global plugins `[static_layer, obstacle_layer, stvl_layer, inflation_layer]`
with `decay_model -1` (persistent); both `voxel_size 0.05`,
`obstacle_range 3.0`, `mark_threshold 0`, `observation_persistence 0.0`,
`model_type 1`. Pattern: **aggressive short decay locally, persistent or long
decay globally.**

STVL decay facts worth knowing (verified in code):
- `decay_model`: 0 = linear, 1 = exponential; any other value = persistent
  (README documents -1). Compiled-in default `voxel_decay` is -1.0 (never
  decays) — the 15-20 s figures are tutorial examples, not defaults.
- The exponential model computes time-until-decay as `voxel_decay*exp(-dt)`,
  which never reaches zero; without per-source `decay_acceleration`, voxels
  then never expire. **This workspace hit exactly that bug** (G1: monotonic
  0 -> 21k lethal cells from floor smear + a walking person, bag
  `g1_planner_tune_20260710_175531`) and fixed it with linear decay 3.0 s —
  independently consistent with linorobot2's linear 5.0 s local choice.

---

## 4. Verified recurring tuning values

### 4.1 LIO baseline for the Mid-360 (FAST-LIO `config/mid360.yaml`, identical in hku-mars upstream and the Ericsii Humble port)

| Parameter | Ecosystem baseline | This workspace (DLIO G1 real) | Comment |
|---|---|---|---|
| Input topics | `/livox/lidar`, `/livox/imu` (CustomMsg) | `/lidar/points`, `/lidar/imu` (PointCloud2, relayed) | Local driver runs `xfer_format 0` (PointXYZRTLT per-point stamps) so DLIO deskews without CustomMsg |
| Min-range crop ("blind") | 0.5 m | corrector self-filter box + `min_radius 0.20` | Local crop is tighter; body box filter covers the difference. If self-hits appear (arms), widen the box before touching min_radius |
| Downsample voxel | `filter_size_surf/map 0.5` | `voxelFilter/res 0.25` | Finer local voxel is deliberate (indoor scale); baseline 0.5 targets 100 m outdoor `det_range` |
| Timestamp unit | 3 (nanoseconds, CustomMsg offsets) | per-point float stamps via PointXYZRTLT | Equivalent intent: per-point time for deskew |
| point_filter_num | 3 (keep 1 of 3 points) | n/a (voxel filter instead) | Both are input decimation |
| Factory LiDAR-IMU extrinsic | T = [-0.011, -0.02329, 0.04412], identity R | inside-unit extrinsic implicit; body->sensor extrinsic in DLIO yaml | The factory IMU-in-LiDAR offset matters if you ever feed `/lidar/imu` to a consumer that assumes IMU at the cloud origin |
| Gravity/IMU init | FAST-LIO estimates on start | `approximateGravity true`, `calibration/time 3.0 s` stationary | Reason for `run_g1_real.sh --nav-delay 5.0`: robot must stand still through DLIO calibration |

### 4.2 STVL / costmap values

| Parameter | Ecosystem recurring value | This workspace (G1 profile) | Verdict |
|---|---|---|---|
| voxel_size | 0.05 m (universal) | 0.05 | match |
| costmap resolution | 0.05 m | 0.05 | match |
| local voxel_decay | 5-15 s linear (tutorial 15, linorobot2 5) | 3.0 linear (Go2 default_safe: 2.0 exponential) | G1 at the aggressive end; justified by ~8-10 Hz re-confirmation from the non-repetitive pattern. Go2 default_safe still runs decay_model 1 (exponential) — same model that never expired voxels on the G1; it has not shown the runaway there, but if lethal smear appears on Go2, switch it to linear before touching anything else |
| global decay | 15-45 s or persistent | global costmap is static+inflation (no STVL) in G1 profile | local divergence, intentional: slam_toolbox map carries persistent structure |
| obstacle_range | 3.0 m (recurring) | 3.0 (G1, adopted 2026-07-11: leveling wobble scales with range and smeared the costmap while walking), 8.0 (Go2 profiles) | match on G1 |
| raytrace_range | > obstacle_range (tutorial) | 10.0 | match in spirit |
| mark_threshold | 0 (universal) | 0 | match |
| min/max_obstacle_height | 0.0 / 2.0 (wheeled, static clip) | 0.25 / 2.3 in `lidar_stabilized` | Local is correct for a walking biped with floor at odom z ~= +0.08 |
| model_type | 1 (hourglass) for 3D lidar — universal in lidar designs | 1 + `vertical_fov_angle 1.03` / `horizontal_fov_angle 6.29` / `decay_acceleration 5.0` (G1 `lidar_mark`, adopted 2026-07-11 after bag validation: ~65% fewer in-motion lethal cells) | match |
| voxel filter min points | not used in surveyed designs | `voxel_min_points 2` (G1), 1 (Go2) | Local addition that compensates for single-point ghosts in sparse non-repetitive frames; keep |
| inflation | tutorial-era defaults vary; linorobot2 keeps Nav2 defaults | 0.35 m / scale 3.0 (G1), 0.30 / 3.0 (Go2 default) | No ecosystem consensus; local values are footprint-derived and documented inline |

### 4.3 Controller

The surveyed Mid-360 designs are thin on controller tuning: pb_rm (omni
sentry) and linorobot2 use DWB-era defaults; none publish MPPI tuning for a
legged platform. **There is no external reference better than the local MPPI
configs** — the Jetson-compute lessons in `nav2_params.yaml` (batch 1200 not
2000, `wz_std 0.4` to stop angular sign-flips, VelocityDeadbandCritic because
the Go2 only steps decisively above ~25 cm/s) and the AMO/SONIC velocity caps
in the G1 profiles are ahead of anything found. Treat local MPPI values as
authoritative; do not import controller settings from wheeled references.

### 4.4 Driver (livox_ros_driver2)

| Parameter | Ecosystem | This workspace | Comment |
|---|---|---|---|
| xfer_format | 1 (CustomMsg) for FAST-LIO designs | 0 (PointCloud2 PointXYZRTLT) | Local choice is correct for DLIO (auto-detects LIVOX type from per-point stamps) and keeps the topic standard for Nav2/RViz |
| publish_freq | 10.0 Hz (universal) | 10.0 | match |
| Cloud QoS | best_effort sensor data | best_effort into STVL | match |
| extrinsic_parameter | rarely used | `roll: 180` in `MID360_config.json` (sensor physically inverted in G1 head) | See pitfall P1 below — this rights the cloud only, NOT the IMU |

---

## 5. Verified pitfalls (Mid-360-specific)

**P1 — Inverted-mount IMU (G1).** The Mid-360 in the Unitree G1 head is
mounted upside down. Stock `livox_ros_driver2` applies
`extrinsic_parameter` only to the point cloud (`lddc.cpp` publishes IMU raw,
no rotation; confirmed against driver source and issue #139). Any consumer of
`/livox/imu` therefore sees an inverted IMU. deepglint's
FAST_LIO_LOCALIZATION_HUMANOID ships a modified driver that rotates both.
**Local handling**: DLIO's IMU extrinsic includes `Rx(180)`
(`dlio_params_g1_real.yaml`), and `/imu/data` (body IMU), not the LiDAR IMU,
feeds the distortion corrector — so the stack is safe as wired. The trap is
future work: anything new that consumes `/lidar/imu` (`/livox/imu_ms2`
rescaled) must apply the 180-degree roll itself or the deepglint driver must
be adopted. Same class of trap as ADR 0011's Go2 LiDAR-IMU warning.

**P2 — Sensor-height / floor-offset miscalibration** is the top false-obstacle
cause across all families (Section 3.1). Locally: recheck `STANDING_HEIGHT`
(0.78) and the observed odom-z floor offset whenever the mount, gait, or DLIO
extrinsics change; `min_obstacle_height 0.25` encodes the current +0.08
offset plus tilt margin.

**P3 — STVL exponential decay never expires voxels** without per-source
`decay_acceleration` (Section 3.3). Already fixed on G1 (linear 3.0); Go2
default_safe still carries `decay_model 1`.

**P4 — Non-repetitive scan pattern**: any single 100 ms frame is sparse and
non-uniform. Consequences and local mitigations: (a) a 2D slice from one
frame is too thin for slam_toolbox — hence `scan_cloud_accumulator` (3
frames / 0.20 s, yaw gate 0.30 rad/s) feeding pointcloud_to_laserscan only
(ADR 0011); (b) single-point ghosts — hence `voxel_min_points 2`; (c) an
obstacle is re-confirmed only every ~2-3 sweeps at a given voxel, which sets
the floor on usable voxel_decay (do not go below ~1.5-2 s marking decay).
FAST-LIO handles the pattern natively (`scan_line 4` is a formality); DLIO
likewise — the pattern is a costmap/scan problem, not a LIO problem.

**P5 — LIO deskew needs real per-point timestamps.** Real driver provides
them (both formats); Gazebo gpu_lidar does not — deskew must stay off in sim
(`dlio_params_g1_sim.yaml` already does; the corrector's sim override too).
Enabling deskew on stamp-less clouds corrupts geometry while moving.

**P6 — Do not trust search-visible repos blindly.** Verified negatives:
CMU `autonomy_stack_go2` uses the built-in L1 (not Mid-360) and the CMU
planner stack (not Nav2); `Unitree-Go2-Robot/go2_robot` uses a Hesai lidar
and its SLAM/Nav2 are unimplemented despite a checked checklist in the
README. Neither is a Mid-360+Nav2 reference. Also refuted in verification:
Go2_where_r_u's published height-clip values (0-3 vote — do not cite), and a
claim that voxel_layer defaults are `obstacle_max_range 2.5` /
`raytrace_max_range 3.0` (0-3 — check Humble source before assuming).

---

## 6. Which repo to consult per subsystem

| Subsystem | Reference | What to take from it |
|---|---|---|
| LIO config baseline for Mid-360 | Ericsii/FAST_LIO_ROS2 (`config/mid360.yaml`), hku-mars/FAST_LIO | blind radius, filter sizes, timestamp handling, factory IMU extrinsic |
| Full Mid-360 -> LIO -> ground-seg -> Nav2 chain | LihanChen2004/pb_rm_simulation (+ their Point-LIO fork) | base_link TF modeling, `measurement_params_real.yaml` extrinsic pattern, linefit `sensor_height` discipline |
| Ground segmentation (if ever added) | lorenwel/linefit_ground_segmentation | `sensor_height`, slope/fit-error params |
| STVL semantics and tuning envelope | SteveMacenski/spatio_temporal_voxel_layer (humble), Nav2 STVL tutorial | decay model semantics, model_type/frustum, per-source schema |
| Concrete STVL YAML cross-check | mich-pest/ros2_navigation_stvl (`linorobot2_navigation/config/navigation_3d.yaml`) | local-fast/global-persistent decay pattern (sim AGV — generic reference only) |
| G1 inverted-mount driver fix | deepglint/FAST_LIO_LOCALIZATION_HUMANOID | modified livox_ros_driver2 rotating cloud AND IMU; prior-map localization pattern |
| Minimal 2D path sanity check | arpa-byte/Go2_where_r_u | launch topology only; its tuning values failed verification |
| Controller (MPPI on legged/Jetson) | **this repo** (`nav2_params.yaml`, `nav2_params_g1*.yaml` inline comments) | no better external reference exists |

---

## 7. Tuning checklist for an agent working this stack

Ordered by likelihood of being the actual problem:

1. **False obstacles on the floor**: check odom-z floor offset vs
   `min_obstacle_height` (P2), then leveling residual vs `obstacle_range`
   (Section 3.1), then STVL decay model (P3). Do not raise
   `min_obstacle_height` past ~0.30 without accepting blindness to low
   obstacles (the /scan band bottom is ~0.29 m already).
2. **Lethal cells that never clear**: decay model must be linear (P3); then
   consider `model_type 1` + `decay_acceleration` for frustum clearing
   before shortening `voxel_decay` below ~2 s (P4c).
3. **Obstacles flicker / disappear between sweeps**: voxel_decay too short
   for the non-repetitive re-confirmation rate (P4c), or
   `voxel_min_points` too high for the range.
4. **Odometry jumps or z drift**: DLIO side — stationary calibration window
   respected (`--nav-delay`), accel-bias caps (`Kab`, `abias_max`), then the
   relay's covariance scaling thresholds in `lidar_odom_relay.py`. Never
   bypass `/lidar_odom_reliable` (ADR 0005).
5. **New IMU consumer misbehaves**: inverted LiDAR IMU (P1).
6. **Sim-only geometry smear while moving**: deskew enabled on stamp-less
   sim clouds (P5).
7. **Controller oscillation / low rate on Jetson**: MPPI batch/std values —
   local comments in `nav2_params.yaml` are the reference (Section 4.3).

---

Sources: all repo/tutorial claims verified against primary files July 2026 by
multi-agent adversarial review (99 agents; refuted claims noted inline in
Section 5/P6). Local values read from workspace configs on branch `feat/g1`
at the same date.
