# G1 Perception Tuning Plan (offline bag replay + multi-world sim)

Goal: tune the G1 perception stack so it produces a CORRECT and SAFE 2D map and
obstacle costmap in which the robot can walk, using the recorded G1 LiDAR bag as the
primary evidence, then confirm the tuned configs in the Gazebo Fortress sim across
multiple worlds.

Status: SECOND PASS (2026-07-11) on the live-stack bag
`bags/g1_planner_tune_20260710_175531` (maps + evidence in
`maps/g1_map_compare/`, tooling `scripts/compare_slam_dlio_maps.py` +
`scripts/dlio_pcd_to_nav2_map.py`). Outcomes now in the live configs:
(a) G1 slam_toolbox runs with `use_scan_matching: false` -- DLIO is the pose
source and any 2D matching measurably snapped (10-24 deg) or drifted (8.6 deg)
against it; (b) STVL decay switched to linear 3.0 s: the exponential model
with no per-source `decay_acceleration` NEVER expires voxels (lethal cells
grew monotonically, 0 -> 21k over one bag); (c) STVL `min_obstacle_height`
0.25 / `obstacle_range` 5.0 -- the floor sits at odom z ~= +0.08 while
walking and gait leveling residual tilts far floor returns above the old
0.12 gate; (d) nav-cloud deskew ON (turning sweeps 6.4 -> 4.9 cm wall error);
(e) mapping scan accumulated + yaw-gated at 0.20 rad/s on G1-real
(`/scan` feeds slam only; safety consumers read the raw cloud).

Status: IMPLEMENTED (2026-07-10). The harness exists
(`bag_perception_g1.launch.py` + `scripts/replay_bag_perception_g1.sh` +
`*_g1_bag.yaml` configs), Stage 1 gates pass on the bag, and Stage 2 passed in
all three worlds. Two findings amended the design during implementation (see
ADR 0012, G1 section): (a) DLIO tracks `base_footprint`, not `base_link`, and
the bag's `/tf_static` is dropped, so the replay's odom floor datum matches the
live STVL height contract; (b) a floor-plane fit measured the standing sensor
attitude 7.2 deg off the nominal URDF mount -- the calibrated
`sensor_mount_rpy` landed in the LIVE `distortion_corrector_g1.yaml` (the only
live-config value this tuning pass changed; all other shared values validated
as-is). Section 2's motion figures came from the bag's onboard `map ->
base_link` estimate and proved wrong: polar wall-profile matching shows the
robot stepped in place, made a partial rotation excursion, and returned to its
start pose (the onboard 16.2 m / 185 deg trace is estimator wander).

---

## 1. Scope and definition of done

- Deliverable: tuned values in the LIVE shared G1 configs
  (`distortion_corrector_g1.yaml`, `scan_from_cloud_g1.yaml`, `nav2_params_g1.yaml`,
  `slam_toolbox_params.yaml`), so the same tuning ships to real hardware and sim.
- Validated in two stages:
  1. Offline replay of `bags/g1/lidar_record` through a new G1 replay harness.
  2. G1 Gazebo sim across three worlds (functional + regression bar).
- "Correct" = the `/map` occupancy grid matches the recorded room geometry (walls
  closed and straight, no ghosting/double-walls from odometry drift, floor not painted
  as obstacle). "Safe" = the costmap marks real obstacles with footprint-appropriate
  inflation, with no self-return ring, no ground false-positives, and no single-point
  phantom obstacles, while free space stays walkable.

Out of scope: deep DLIO algorithm retuning (see decision 7); changing the live TF
ownership contract outside the ADR-0012 replay exception; hardware bring-up.

---

## 2. The bag (measured facts)

`bags/g1/lidar_record/lidar_record_0.db3`, 64.9 s, four topics:

| topic | type | count | rate | notes |
|-------|------|------:|-----:|-------|
| `/livox/lidar_filtered` | PointCloud2 | 648 | ~10 Hz | frame `livox_frame`; fields x,y,z,intensity,tag,line,**timestamp (f64)** |
| `/livox/imu` | Imu | 12983 | ~200 Hz | frame `livox_frame`; accel in **g** and **inverted** (z ~ -1.0) |
| `/tf` | TFMessage | 47253 | - | `map->base_link` @ ~10 Hz + full articulated joint tree + `pelvis->imu_link` @ ~700 Hz |
| `/tf_static` | TFMessage | 72 | - | full G1 URDF static tree incl. `mid360_link->livox_frame`, `base_link->base_footprint` |

Motion: the robot walked ~16.2 m of path over a ~4.0 x 1.75 m footprint with ~185 deg
of yaw change (gyro up to 1.74 rad/s) - a real trajectory and a real map to build.

What the bag does NOT contain: `/odom_raw`, `/odom`, body `/imu/data`, the canonical
`/lidar/points` / `/lidar/imu`, `/scan`, or `/map`. So it cannot feed the existing
Go2-shaped replay flow (`replay_bag_perception.sh` + `bag_perception.launch.py`), which
expects `/lidar/points` + `/imu/data` + `/odom_raw|/odom` and expands the Go2 URDF.

---

## 3. Findings that shape the approach

1. No body odometry and no body IMU - only the LiDAR cloud, the LiDAR IMU, and the
   bag's own `map->base_link` (Unitree onboard estimate, which wanders ~1.15 m in z).
   Decision: run DLIO fresh as the odometry (ADR-0012 replay TF-owner override).

2. The cloud is still INVERTED. In `livox_frame`, 63% of returns are at z > +0.5 and
   the median z is +0.89 (floor above the sensor) - the MID360_config `roll:180`
   righting that the LIVE driver applies was NOT applied in this recording. Therefore
   the bag-scoped extrinsics/mount must fold in `Rx(180)`; the live upright-cloud
   configs cannot be reused verbatim for the replay.

3. Per-point `timestamp` is valid absolute Livox nanoseconds (~1.7835e18, per-scan
   span 0.1003 s). DLIO auto-detects `SensorType::LIVOX` from this field and deskews;
   `computeTimeOffset` works. (Minor: f64 quantizes ~256 ns absolute, negligible vs a
   0.1 s scan.)

4. `/livox/lidar_filtered` is range/outlier-filtered raw geometry (still in
   `livox_frame`, all raw fields present) - NOT self-filtered in the body frame, so the
   `distortion_corrector` self-filter still applies. This frame's nearest return is
   ~0.95 m; the driver pre-filter may blank the near field (a data-side blind spot for
   close-obstacle checks - verify across frames).

5. `imu_rescale` only scales g->m/s^2 (it does not de-invert); inversion is handled in
   DLIO's `baselink2imu` extrinsic. That live IMU extrinsic (`Ry(0.04)*Rx(180)`) is
   exactly the cloud extrinsic the bag needs, because the bag cloud is inverted the
   same way the live IMU is.

---

## 4. Fixed decisions

| # | Decision | Choice | Rationale |
|---|----------|--------|-----------|
| 1 | Deliverable / done | Offline tune on this bag, THEN G1 sim validation | Tuning must reach hardware; sim is the generalization check |
| 2 | Odometry backbone | DLIO fresh - owns `odom->base_link`; slam_toolbox owns `map->odom` | Bag has no body odom; matches the live G1-real localizer (DLIO+EKF) |
| 3 | Harness form | New parallel `bag_perception_g1.launch.py` + wrapper | Go2 launch hardcodes Go2 URDF/joint bridge/`/odom` path; parallel matches the G1 config convention |
| 4 | Leveling | Disabled for replay - `gravity_align:false`, static upright `lidar_stabilized` | No body IMU; accept a few deg of gait tilt for simplicity |
| 5 | Where tuning lands | Live shared G1 configs | Only way the sim stage tests what ships |
| 6 | Acceptance bar | Visual + quantitative + walkability | Reproducible headless gates + a human RViz pass |
| 7 | DLIO scope | Held ~fixed - bag-required extrinsic/flag overrides only | Don't perturb odometry proven on hardware; drift is a flagged finding, not a retune |
| 8 | Sim bar | Functional + regression, across MULTIPLE worlds | Confirm walkable navigation and no regression, not just a smoke launch |

---

## 5. Stage 1 - offline replay harness

### 5.1 Playback whitelist

Play only: `/livox/lidar_filtered`, `/livox/imu`, `/tf_static`. DROP the bag's `/tf`
(its `map->base_link` and joint tree) so there is exactly one fresh publisher per
transform. Play with `--clock` and `use_sim_time:=true`.

### 5.2 Nodes (bag scaffolding - NOT shipped to hardware)

- `topic_tools relay`: `/livox/lidar_filtered -> /lidar/points`.
- `imu_rescale` (`g1_bringup`): `/livox/imu -> /lidar/imu` (g->m/s^2).
- `robot_state_publisher` (`g1_description`): owns `/robot_description` and URDF
  `/tf_static`, including `base_link->base_footprint` and `base_link->...->livox_frame`.
- `joint_state_publisher` at the fixed standing pose (RViz link animation only).
- static `base_footprint -> lidar_stabilized` (upright, at sensor height) - replaces
  `lidar_level_frame_publisher`, which needs a body IMU we do not have.
- DLIO (`direct_lidar_inertial_odometry`) with the bag config below; owns
  `odom -> base_link`.
- `distortion_corrector` (bag config) -> `cloud_accumulator` ->
  `pointcloud_to_laserscan` (`scan_from_cloud_g1.yaml`) -> `/scan`.
- `slam_toolbox` (async) owns `map -> odom`, publishes `/map`.
- Nav2 `controller_server` + `planner_server` host the local/global costmaps
  (STVL marks off `/lidar/points_undistorted`); optional RViz.

### 5.3 Resulting TF tree

```
map --(slam_toolbox)--> odom --(DLIO)--> base_link --(URDF)--> base_footprint
base_footprint --(static, upright)--> lidar_stabilized   (2D scan frame)
base_link --(URDF)--> ... -> mid360_link -> livox_frame   (cloud frame)
```

Note: `base_footprint` is a rigid URDF child of the DLIO-tracked `base_link`, so it
tilts with body pitch/roll (no EKF planarizer in replay). Accepted under decision 4.

### 5.4 New bag-scoped configs (scaffolding, do NOT ship)

`dlio_params_g1_bag.yaml` = `dlio_params_g1_real.yaml` plus:

```yaml
use_sim_time: true
publish_tf: true
map/waitUntilMove: false
# Bag cloud is INVERTED (roll:180 not applied at record time), so the cloud
# extrinsic gets the same Rx(180) the live IMU extrinsic already has:
extrinsics/baselink2lidar/R: [ 0.99919457, 0.0, -0.04012671,
                               0.0,       -1.0,  0.0,
                              -0.04012671, 0.0, -0.99919457]   # Ry(0.04014)*Rx(180)
# baselink2imu unchanged from g1_real (already inverted); baselink2lidar/t unchanged.
```

`distortion_corrector_g1_bag.yaml` = `distortion_corrector_g1.yaml` plus:

```yaml
gravity_align: false            # no body IMU; static lidar_stabilized instead
sensor_mount_rpy: [3.14159265, 0.04014, 0.0]   # add roll=pi to right the inverted cloud
# deskew stays OFF (DLIO deskews its own copy); self-filter box unchanged - it is the
# TUNED live value and must match nav2/scan.
```

Both `sensor_mount_rpy` and `baselink2lidar/R` are VERIFY-then-trust: confirm the
righted cloud has the floor below the sensor and the self-filter removes the body box,
before reading any costmap result.

### 5.5 Tuning surface (values land in the LIVE shared G1 configs)

- `distortion_corrector_g1.yaml`: self-filter box (`robot_min/max_x/y/z`, `min_radius`).
- `scan_from_cloud_g1.yaml`: band `min_height`/`max_height`, `range_min`, `range_max`.
- `nav2_params_g1.yaml` STVL `lidar_mark`: `min_obstacle_height`, `max_obstacle_height`,
  `voxel_min_points`, `obstacle_range`, `raytrace_range`, `voxel_decay`; `inflation_radius`.
- `slam_toolbox_params.yaml`: resolution and loop-closure settings if the loop smears.

### 5.6 Method

Iterate: run the replay (record `/tf /map /scan /odom`, save the map on exit) -> run
`scripts/perception_metrics.py` on the recording -> inspect the saved map + RViz ->
adjust the live config -> repeat. Keep a before/after metrics diff per change.

### 5.7 Acceptance gates (Stage 1)

Quantitative (headless, from the recorded replay bag + saved map):
- DLIO does not diverge over the loop (trajectory continuous; `/map` occupied-cell count
  stabilizes rather than growing monotonically = no smear).
- TF present at rate: `map->odom` (slam), `odom->base_link` (DLIO), `base_link->base_footprint`.
- `/scan` finite-range coverage reported and stable; floor excluded.
- No self-return ring: zero marked cells inside the footprint at the robot pose.
- No phantom obstacles: single-point returns rejected (`voxel_min_points >= 2`).
Walkability:
- The recorded trajectory lies in non-lethal costmap cells (with footprint clearance).
- A global plan (SmacPlanner2D) solves between two free poses across the map.
Visual (human): RViz pass on `/map`, local/global costmap, `/scan`, cloud.

---

## 6. Stage 2 - multi-world sim validation

Run `scripts/run_g1_sim.sh mapping --world <w>` with the tuned LIVE configs across
three worlds, each probing a different failure mode:

| world | probes |
|-------|--------|
| `sim_worlds/worlds/navigation_empty.world` | pure false-positive test: NO self-ring, NO floor marks, NO phantom obstacles in an empty scene |
| `sim_worlds/worlds/indoor_office.world` (default) | clutter + corridors: walls/furniture marked, walkable corridor preserved |
| `sim_worlds/worlds/warehouse.world` | scale + tall obstacles: range, tall-obstacle marking, map scale |

Pass bar (functional + regression) per world:
1. Map builds cleanly with walls/obstacles; no self-return ring or floor marks.
2. One Nav2 goal plans and the G1 drives to it without the costmap blocking free space
   or clipping obstacles (walkable).
3. No regression vs current sim behavior (map/costmap no worse than pre-tuning).

Reminder: the "disable leveling" and inverted-cloud handling are replay-only
scaffolding and are NOT in the shipped configs; sim keeps `gravity_align` off (kinematic
base) and real keeps it on, so Stage 2 re-tests exactly the self-filter box, scan band,
STVL heights, `voxel_min_points`, decay, and inflation that landed in the live configs.

---

## 7. Files to add / change

Add (scaffolding):
- `src/autonomy/robot_nav/launch/bag_perception_g1.launch.py`
- `scripts/replay_bag_perception_g1.sh` (or a `--robot g1` branch in the existing wrapper)
- `src/autonomy/robot_lio/config/dlio_params_g1_bag.yaml`
- `src/autonomy/robot_nav/config/distortion_corrector_g1_bag.yaml`

Edit (the tuning deliverable - live shared G1 configs):
- `src/autonomy/robot_nav/config/distortion_corrector_g1.yaml`
- `src/autonomy/robot_nav/config/scan_from_cloud_g1.yaml`
- `src/autonomy/robot_nav/config/nav2_params_g1.yaml`
- `src/autonomy/robot_nav/config/slam_toolbox_params.yaml`

Maintenance (required by workspace rules):
- `scripts/kill_ros_nodes.sh` - add any new launch-started nodes/processes.
- `docs/topic_map.md` - if any topic/frame/relay path changes.
- `docs/launch_script_parity.md` - if the new wrapper adds options.
- ADR note for the inverted-cloud replay handling (extends ADR-0012), since the
  bag-scoped `Rx(180)` extrinsic/mount is a documented deviation from the live
  upright-cloud contract.

---

## 8. Risks and caveats

- Single 65 s bag, one room -> overfit risk. Multi-world sim (Stage 2) is the
  generalization guard; do not over-tighten numeric thresholds to this bag.
- Inverted-cloud handling (`Rx(180)`) is verify-then-trust; a wrong sign silently
  mis-rotates the self-filter box and the whole scan. Gate on the righted-cloud check
  before reading costmap results.
- Near-field (~<0.95 m) may be pre-filtered in the recording -> close-obstacle safety
  cannot be fully validated from this bag; call it out and lean on sim.
- `base_footprint` tilts with `base_link` during gait (no EKF planarizer) -> a few deg
  of scan/marking tilt; accepted under decision 4.
- `dynamic_prediction` STVL observation source (`/dynamic_obstacles/predicted_occupancy`)
  stays silent in replay (no dynamic pipeline) - expected, not a fault.
- Editing `nav2_params_g1.yaml` touches safety-relevant costmap params; every change is
  justified against the bag/sim evidence and preserves the footprint/velocity contract.
```
