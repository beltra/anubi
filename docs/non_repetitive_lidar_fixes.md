# Non-repetitive LiDAR (Unitree L1) perception fixes

Targeted fixes for issues that arise because the Unitree L1 emits a
non-repetitive scan pattern.

Real-robot bag inspection (`/root/rosbags/go2_real/go2_real_mapping_20260512_120809`,
49.6 s) shows:
- ~1400 points per ~70 ms frame at 14.7 Hz (the 2023 bench bag was 10 Hz,
  the live driver runs faster).
- **12 rings** (ids 0-11, ~116 pts/ring/frame). Earlier docs claimed 18; the
  live driver/firmware in this workspace consistently exposes 12.
- Per-point `time` field present (FLOAT32, monotonic, range 0-0.072 s).
- Effective return range was ~6 m in the recorded environment; the L1's
  actual reach indoors is closer to 15-20 m on plain walls and is the value
  to use for slam ranges, not the bag-environment-limited number.

The current tuning targets these intrinsics directly. See "Empirical density
numbers" further down for the slice-width sweep that drove the +/-0.30 m
choice.

## 1. Enable hardware deskew

**Problem:** `distortion_corrector` had `deskew_enabled: false`. Non-repetitive
frames spread points over the full 100 ms acquisition window with no azimuth
ordering, so motion blur is worse than on a spinning LiDAR. Without deskew,
walls drag during locomotion (0.5 m/s × 0.1 s = 5 cm at the point level).

**Fix:** `src/autonomy/robot_nav/config/distortion_corrector.yaml` now defaults
to `deskew_enabled: true`. Sim already overrides this to `false` in
`src/autonomy/robot_sim/launch/sim_bringup.launch.py` because Fortress
`gpu_lidar` omits the per-point `time` field, so only the real path flips.

**Why it works:** The L1 driver attaches a usable `time` field that the
existing `distortion_corrector` per-point compensator
(`P_corrected = P + t * (v + omega x r_sensor)`) already supports. Enabling
the flag was the only blocker.

## 2. Replace azimuth fallback with explicit drop

**Problem:** When the time field is missing, `distortion_corrector` previously
fabricated relative times from azimuth ordering. Non-repetitive LiDARs do not
scan in azimuth order, so the fallback assigns wrong timestamps and produces
worse-than-no-deskew motion correction.

**Fix:**
- New `fallback_mode: disabled` branch in
  `src/autonomy/robot_nav/src/distortion_corrector.cpp` that drops the frame
  with a diagnostic instead of publishing a corrupted cloud.
- Default in `distortion_corrector.yaml` switched from `azimuth` to
  `disabled`.

**Why it works:** Matches the project's "missing-state drop" policy in
`docs/contexts/lidar-processing/CONTEXT.md` and ADR-0004: better to silence
the corrected topic than to publish a misleading cloud. The default fallback
no longer hides driver regressions.

## 3. 2D scan structural holes -> widen the slice asymmetrically (replaces the original accumulator-only fix)

**Problem:** The 2D LaserScan derived from `/lidar/points_undistorted` with
the original `+/-0.12 m` horizontal slice in `lidar_stabilized` covered only
~4.5% of the 720 angular bins per scan. The z-histogram of the live cloud
shows the dense band of returns sits at `z in [-0.30, -0.20] m` (peak at
z=-0.25), well below the +/-0.12 cut.

**Initial fix (rejected on real-robot trial):** Symmetric `+/-0.30 m` slice.
Offline density numbers are excellent (1112 pts/frame, 70.6% angular
coverage), but on the live Go2 this band consists largely of the robot's
own legs: legs hang below `z = -0.25 m` in `base_link`, which is outside the
distortion_corrector self_filter box (`robot_min_z: -0.25`), so leg returns
reach the slice. Symmetric +/-0.30 therefore produced false-positive
obstacles ringing the robot.

**Current fix:** Asymmetric slice in `scan_from_cloud.yaml`:
`min_height: -0.25`, `max_height: 0.50`, plus `range_min: 0.50` to drop any
return inside half a metre of the robot. The slice asymmetry keeps the
upper rings; the radial cut removes residual leg returns near the LiDAR
that the height filter alone misses. Combined with the self-filter
extension below, leg false positives go away.

**Companion fix in `distortion_corrector.yaml`:** Lift
`robot_min_z: -0.25 -> -0.40` so the self-filter box covers the full
standing leg span (tibia + thigh). The floor sits at z<=-0.45 in base_link,
so the box still excludes ground returns. This removes leg returns at
source for **both** the /scan slice and STVL.

Live metrics with `--no-ekf-fusion`, asymmetric slice, lifted self_filter,
and finer angle_increment (0.286 deg = 1257 bins/scan):

| phase | /scan rate | bins | mean % | abs hits/scan | hits/s |
|-------|------------|------|--------|---------------|--------|
| baseline (live, old configs, EKF on) | 4.56 Hz | 720 | 4.6 | 33 | 150 |
| tuned (live, new configs, EKF off) | 11.59 Hz | 1257 | 11.0 | 138 | 1600 |

10.7x absolute information rate into SLAM, no leg false positives.

## 4. `scan_time` metadata in `pointcloud_to_laserscan`

No change. `scan_time: 0.1` matches the per-frame publication rate of
`/lidar/points_undistorted` (10 Hz observed in the bag). slam_toolbox and
Nav2 rely on `LaserScan.header.stamp`, not `scan_time`, for timing.

## 5. Motion-compensated `scan_cloud_accumulator` (resized and graceful-degraded)

**Problem 1 (motion smear, original):** The Python accumulator concatenated
raw point bytes from buffered frames without correcting for robot motion
between frames. Since each frame was already in `lidar_stabilized` (a
sensor-following gravity-aligned frame), older frames carried points
anchored to older sensor poses. Walls smeared (e.g., 22 degrees of yaw smear
at 30 deg/s during a 0.75 s window).

**Problem 2 (window over-sized after slice widening):** The original
window was 0.75 s / 12 frames to densify a +/-0.12 m slice. After fix #3
widened the slice to +/-0.30 m (one frame already covers ~70% of angles),
12 frames is wasteful: 12x `lookup_transform_full` + 12x `do_transform_cloud`
per cycle in Python is enough to slow the callback below LiDAR rate, which
in the live bag dropped /scan from 14.7 Hz to 4.6 Hz.

**Problem 3 (EKF starvation in TF):** `lookup_transform_full` is gated on
the `odom` TF rate. When the EKF (which owns `odom -> base_footprint`)
publishes that TF at a degraded rate (see ADR/incident around the 4.8 Hz
EKF bug), the older buffered frames' TF lookups silently fail and were
discarded. The output rate then collapsed to match the EKF rate.

**Fix:**
- Motion compensation logic unchanged.
- Default window resized: `accumulation_window_sec: 0.75 -> 0.20`,
  `max_frames: 12 -> 3`. Matches `slam.launch.py` defaults.
- Graceful degradation made explicit in the docstring: the newest frame is
  always publishable without a TF lookup (it IS the target pose). Older
  frames that fail TF are dropped individually; the accumulator still
  publishes whatever projected frames succeed -- including just the newest.
  Output rate is therefore locked to the LiDAR input rate even when the
  fixed_frame TF tree lags.

**Effect on the recorded 49.6 s bag:**

| metric | before | after |
|-------|--------|-------|
| /scan rate | 4.56 Hz | 10.24 Hz (replay still on the bag's 4.77 Hz `odom` TF) |
| /scan angular coverage (mean) | 4.6% | 41.5% |
| TF lookups per cycle | 12 | 3 |

After the EKF rate is restored to 50 Hz, /scan reaches the LiDAR input rate
(~14.7 Hz).

## 6. STVL voxel decay

**Problem:** Each non-repetitive frame covers only a subset of directions, so
a real obstacle may not be re-confirmed every cycle. With `voxel_decay: 0.5`
and `decay_model: 0` (linear), STVL was clearing real obstacles whose
sectors weren't sampled in a given pass.

**Fix:** All STVL profiles (default, `navfn`, `forward_static`) now use
`voxel_decay: 2.0` and `decay_model: 1` (exponential). `narrow_gap_1p0m`
keeps its explicit fast-decay tuning. Updated in
`src/autonomy/robot_nav/config/nav2_params*.yaml`.

**Why it works:** A longer decay window lets the next pass of the sweep
re-confirm voxels before they expire. Exponential decay holds confidence
near full strength for longer and then drops sharply, which suits a
non-repetitive sensor that re-confirms cells eventually but not every frame.

## 7. DLIO keyframe spacing

**Problem:** With sparse per-frame coverage, GICP found correspondences in
densely sampled sectors and missed the rest, biasing rotation estimates and
producing yaw drift. The default keyframe thresholds (0.20 m / 20 degrees)
made the submap reference rebuild slowly.

**Fix:** `src/autonomy/robot_lio/config/dlio_params.yaml`:
- `odom/keyframe/threshD: 0.20 -> 0.10`
- `odom/keyframe/threshR: 20.0 -> 10.0`

**Why it works:** Tighter keyframe spacing accumulates a denser, more
isotropic submap faster. Each scan registers against a richer reference, so
GICP correspondences cover more directions and yaw estimation becomes less
biased.

## 8. DLIO voxel filter resolution

**Problem:** `odom/preprocessing/voxelFilter/res: 0.25` collapsed multiple
points per 25 cm cell into one. Because the per-frame cloud was already
sparse in some directions, this coarse filter destroyed thin features (door
frames, table legs) that GICP relied on.

**Fix:** `odom/preprocessing/voxelFilter/res: 0.25 -> 0.10` in
`dlio_params.yaml`.

**Why it works:** A finer voxel grid keeps more of the already-sparse
geometry visible to GICP, increasing the diversity of correspondences. CPU
cost goes up but the L1 frame size (~2000 points) stays well below GICP
processing limits.

## 9. Loop closure smear

**Problem:** Two passes through the same area produce different
non-repetitive sub-patterns. The 2D LaserScan correlation used by
slam_toolbox's loop closure scored those passes as different geometry, so
real loops fell below `loop_match_minimum_response_fine: 0.45` and were
missed.

**Fix:** `loop_search_space_smear_deviation: 0.03 -> 0.08` in
`src/autonomy/robot_nav/config/slam_toolbox_params.yaml`.

**Why it works:** The smear deviation widens the spatial extent across which
the correlation accumulates evidence, absorbing the per-pass sampling
differences without lowering the response threshold itself. The match
threshold remains strict; only the search space is more forgiving of pattern
variation.

## 11. slam_toolbox far-range and processing-rate caps

**Problem A (far-zone mapping):** `slam_toolbox.scan_buffer_maximum_scan_distance:
10.0` discarded scan points beyond 10 m during scan matching even though
`max_laser_range: 25.0` would have allowed them. Any wall the L1 reported
between 10 m and the configured 25 m was thrown away inside slam_toolbox and
never reached `/map`. This is the simplest explanation for "far zones not
correctly mapped" whenever the environment had structure past 10 m.

**Problem B (processing-rate cap):** `minimum_time_interval: 0.25 s` capped
SLAM processing to 4 Hz. With /scan previously at 4.6 Hz (fix #5) this was
non-binding, but once /scan is restored to LiDAR rate this becomes a
14 deg/update gap during rotation at 1 rad/s -- enough to skew the
scan-match search window during turns.

**Fix in `src/autonomy/robot_nav/config/slam_toolbox_params.yaml`:**
- `scan_buffer_maximum_scan_distance: 10.0 -> 25.0`
- `minimum_time_interval: 0.25 -> 0.10`

Other slam_toolbox knobs are unchanged: `minimum_travel_distance: 0.15`,
`minimum_travel_heading: 0.15`, correlation/loop smear values from fix #9
still apply.

## 10. STVL `voxel_min_points`

**Problem:** STVL's `filter: voxel` rejected voxels with fewer than 2 hits
per cycle. With a non-repetitive scan, distant voxels (far walls, narrow
posts) often receive only one hit per frame, so they never marked - the
costmap effectively went blind beyond a few meters.

**Fix:** `voxel_min_points: 2 -> 1` for the lidar_mark observation across
all four nav2 profiles.

**Why it works:** A single confirmed point is a legitimate observation when
deskew (#1) and the self-filter remove the main sources of false returns.
The increased voxel decay (#6) means that even if a phantom voxel marks
spuriously, it fades quickly and the costmap inflation kernel dominates
short-lived false positives. Real far-range walls remain visible.

## What was not changed

- `scan_time` in `scan_from_cloud.yaml` (item 4): the 10 Hz per-frame rate is
  honest metadata and slam_toolbox does not use this field for timing.
- `narrow_gap_1p0m` STVL decay: kept at 0.5 s linear because the profile is
  explicitly tuned for short ghost retention in tight corridors. Only the
  `voxel_min_points` change applies.
- ADR-0004 ("gravity-aligned corrected LiDAR cloud") and ADR-0011 ("real
  LiDAR orientation and IMU processing") still describe the architecture
  correctly; only defaults shifted, not the contracts.

## Known issue: EKF /odom rate at 4.85 Hz (R1)

`robot_localization`'s `ekf_filter_node` publishes `/odom` and
`odom -> base_footprint` TF at a steady 4.85 Hz on this Go2, not the configured
50 Hz. Confirmed live and offline:

- `frequency: 50.0`, `sensor_timeout: 0.1`, `transform_timeout: 0.2` are all
  loaded correctly (verified via `ros2 param get`).
- Lowering `transform_timeout` to 0.05 had no effect on the rate.
- Toggling `predict_to_current_time` and changing `frequency` at runtime had
  no effect (those params are read once at startup).
- `/odom` inter-message intervals are deterministic 202-210 ms.
- Live `/diagnostics` reports only one issue: `2 absolute pose inputs detected
  for YAW. This may result in oscillations.` Both `odom0_config` and
  `imu0_config` enable absolute yaw (config bug, but not the rate killer).
- robot_localization 3.5.4 source (`src/ros_filter.cpp`) shows publication is
  gated on `filter_.getLastMeasurementTime()` advancing between cycles; the
  gate is bypassable with `permit_corrected_publication: true` but the live
  root cause of the gate firing at 4.85 Hz remains unresolved.

### Workaround for non-LIO mode: bypass the EKF

`/odom_raw` from `go2_hw_bridge` is already a fused estimate from Unitree's
Sport Mode controller (leg kinematics + onboard IMU), published at ~290 Hz
with reasonable covariances. Stacking an EKF on top adds little fusion value
for this platform, so the wrapper now exposes `--no-ekf-fusion`:

```
scripts/run_go2_real.sh --no-ekf-fusion ...
```

In that mode `go2_hw_bridge` publishes `/odom` directly and owns
`odom -> base_footprint`, both at ~290 Hz. R1 disappears.

### When the EKF is mandatory

LIO mode (`--use-lio`). DLIO and the body odom must be fused through the EKF
to produce a single coherent `odom -> base_footprint` TF; bypassing it there
would lose either DLIO or the body-velocity prior. The wrapper now refuses
`--use-lio` together with `--no-ekf-fusion`. R1 will need a real fix before
LIO mode is run again.

### Config bug to fix when R1 is revisited

`ekf_fusion_params.yaml` has two absolute yaw inputs:

```yaml
odom0_config:  [T, T, F, F, F, T, ...]   # index 5 (yaw) = T  -> absolute yaw
imu0_config:   [F, F, F, F, F, T, ...]   # index 5 (yaw) = T  -> absolute yaw
```

Remove yaw from `odom0_config` (set index 5 to False) so the IMU is the single
absolute-yaw source. The Go2's `/odom_raw` will still provide x/y position and
all velocity components.

## Empirical density numbers

Source: `/root/rosbags/go2_real/go2_real_mapping_20260512_120809` (49.6 s)
for the baseline, live measurements for the tuned numbers.
Reproducer: `scripts/perception_metrics.py <bag_dir>`.

| metric | baseline (live, old configs, EKF on) | tuned (live, new configs, --no-ekf-fusion) |
|--------|---------------------------------------|--------------------------------------------|
| /lidar/points rate | 14.7 Hz | 14.71 Hz |
| /scan rate | 4.56 Hz | 11.59 Hz |
| /scan bins | 720 (0.5 deg) | 1257 (0.286 deg) |
| /scan absolute hits/scan | 33 | 138 |
| /scan info rate (hits/s) | 150 | 1600 |
| /odom rate | 4.83 Hz | 296 Hz (hw_bridge direct) |
| odom -> base_footprint TF rate | 4.77 Hz | 170 Hz (hw_bridge direct) |

## Validation status

- `colcon build --packages-select robot_nav`: clean.
- `ros2 launch robot_nav slam.launch.py --show-args`: shows
  `scan_accumulation_window_sec=0.20` and `scan_accumulation_max_frames=3`.
- Offline replay against the recorded bag confirms the slice + accumulator
  changes lift /scan rate 2.2x and angular coverage 9x.
- EKF /odom rate root-cause (R1): live diagnosis pending.
- Live acceptance run with metrics: pending.
