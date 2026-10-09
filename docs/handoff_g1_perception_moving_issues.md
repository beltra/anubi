# Prompt: G1 perception — fix the two remaining WHILE-MOVING issues

## RESOLUTION (2026-07-11, third pass) — both issues addressed

Everything below this section is the original handoff, kept for reference.
Outcome, all replay-measured on `bags/g1_planner_tune_20260710_175531`
(details in `docs/g1_perception_tuning_plan.md` third-pass status; images
`maps/g1_map_compare/report/22..29_*.png`):

- Issue 2 root cause (live): the EKF's `odom->base_footprint` yaw lagged
  DLIO ~140 ms (up to 23 deg during turns), so STVL marks rotated with the
  robot. Mechanism: `lidar_odom_relay`'s per-message position-jump detector
  misfires at the G1's 100 Hz DLIO rate (covariance pinned at 100x for 5%
  of the live run, all during motion) plus loose DLIO trust. Fixed:
  windowed jump detector (robot-agnostic, Go2 semantics unchanged) + tight
  G1 relay covariances. Offline EKF re-run of the bag's own odometry:
  turn yaw error p90 9.0 -> 2.5 deg, lag 140 -> 20 ms, no outages.
- The user's "mostly pink costmap when moving" (seen in an RViz replay of
  runM) does NOT reproduce in current configs: reconstructing the full
  5 Hz costmap+updates stream shows max 8% colored cells, still or moving.
  It matches the pre-fix exponential-decay flood. Re-judge in RViz.
- Issue 1: slam insertion gates 0.15 (was 0.25), `map_update_interval`
  0.25, accumulator yaw gate 0.60 (was 0.20; registered sweeps paint fine
  during turns — walls IMPROVED with the gate relaxed). Visible map
  changes 14 -> 31 per bag, median gap 1.5 -> 0.9 s, max 13 -> 8 s,
  coverage 55 -> 70%, walls 92.7 -> 90.6% matched (user-approved trade).
- H3 shipped as option (c): rear-sector filter in `cloud_accumulator`
  (rear +/-40 deg within 3.0 m — measured operator band; the 0.5-1.0 m
  prior estimate was too near, real spread reached 2.8 m), MAP PATH ONLY.
  Costmap/collision still see behind the robot.
- H2 verified: mount pitch matches the Relo02/Navigation G1 stack exactly
  (0.04014 rad). The Mid-360 internal lidar->IMU lever arm is now folded
  into DLIO `baselink2imu/t` (g1_real + g1_bag_live configs): worst
  turn-time DLIO correction steps -28%, coherence unchanged.
- The "everything rotates around the robot" RViz appearance was the
  nav2.rviz TopDownOrtho `Target Frame: base_footprint` (camera yaw-locked
  to the robot); now `map`. The remaining while-moving costmap mass is
  real: 75% of lethal within 0.3 m of DLIO structure at the fast turn
  (99% still), the rest is inflation + the 3 s decay trail
  (report/30_local_costmap_5hz_frames.png).
- Remaining user actions: (1) RViz pass on
  `scripts/replay_bag_perception_g1.sh bags/g1_planner_tune_20260710_175531`
  (no overrides — defaults now mirror shipped configs); (2) record a new
  live bag (`run_g1_real.sh --record-rosbag`) and re-run the EKF-vs-DLIO
  yaw analysis to confirm the turn-lag fix on hardware.

---

Copy everything below into a new session.

---

You are continuing a G1 (Unitree humanoid, head-mounted Livox Mid-360) SLAM +
perception tuning effort in the ROS 2 Humble workspace `/ws/anubi`. A previous
session got the slam_toolbox map to 93% wall coherence with the DLIO 3D map
(median 5 cm) and a self-healing local costmap, all validated by offline
replay of `bags/g1_planner_tune_20260710_175531` (143 s real-G1 walk in a big
room with furniture, an operator walking behind the robot the whole time).
Everything that was tuned is SHIPPED in the live configs. Two issues remain,
both visible only WHILE THE ROBOT IS MOVING (everything is fine when still).
Do not regress the shipped wins while fixing them.

## Read these first (in order)

1. `docs/g1_perception_tuning_plan.md` — status section documents every
   shipped change and its measured justification.
2. `docs/mid360_nav2_reference_designs.md` — verified ecosystem survey;
   its Section 7 checklist and pitfalls P1-P6 are grounded.
3. `docs/topic_map.md` — authoritative topic/TF contract, including the new
   `/lidar/points_registered` mapping-scan path.
4. `maps/g1_map_compare/report/` — numbered images of the whole experiment
   trail (00 = broken live map, 20/21 = final validated state).
5. External cross-reference (user-suggested):
   https://github.com/Relo02/Navigation/tree/feat/local-planner-port —
   another Unitree G1 + Mid-360 + DLIO stack (Jetson Orin Nano; Open3D voxel
   map + terrain-slope costmap + replanning A*, no Nav2). Use it to
   cross-check against THIS robot's geometry: their DLIO config/extrinsics
   and livox_ros_driver2 tuning for the same G1 head mount directly answer
   the "are the R|t matrices right" question (H2), and their voxel-map
   column-carving + dynamic obstacle tracking are alternative treatments of
   the moving-smear and operator problems (H1/H3). Verify claims against
   their actual config files before importing numbers.
5. Memory: the dev container loses apt packages on reset (see memory file
   `env-missing-apt-packages`); images the user should see go under
   `maps/<task>/report/` inside the repo.

## Current architecture (shipped, validated in replay)

- DLIO is the odometry; G1 slam_toolbox runs `use_scan_matching: false`
  (`slam_toolbox_params_g1.yaml`) — it rasterizes, never corrects.
- The mapping `/scan` comes from DLIO's GICP-registered cloud:
  `dlio deskewed -> /lidar/points_registered` (remap in
  `robot_lio/launch/lio.launch.py`) -> `cloud_accumulator`
  (`scan_cloud_in_fixed_frame:=true`, 3-frame concat, 0.45 m near-body radius
  filter, 0.20 rad/s yaw gate) -> `pointcloud_to_laserscan` -> slam_toolbox.
  `/scan` feeds slam ONLY; collision_monitor and STVL consume
  `/lidar/points_undistorted` directly.
- STVL `lidar_mark` (`nav2_params_g1.yaml`): linear decay 3.0 s (exponential
  NEVER expires without decay_acceleration — do not revert), 3D-lidar
  hourglass frustum `model_type: 1` + `decay_acceleration: 5.0`,
  `obstacle_range: 3.0`, `min_obstacle_height: 0.25` (floor sits at odom
  z ~= +0.08 while walking).
- Nav-cloud deskew ON in `distortion_corrector_g1.yaml` (sim overrides off).
- `map/waitUntilMove: false` in `dlio_params_g1_real.yaml`.

## Replay harness + metrics (use these; they are the evidence loop)

- `scripts/replay_bag_perception_g1.sh bags/g1_planner_tune_20260710_175531
  --no-rviz --save-map maps/g1_map_compare/<run>` replays the bag through the
  shipped configs (override any config via `--launch-arg`); add RViz by
  dropping `--no-rviz`. Saves slam map + posegraph + DLIO PCD and records
  outputs for analysis. ALWAYS check the ROS graph is empty first
  (`scripts/kill_ros_nodes.sh`; a leftover sim/replay silently corrupts runs
  with duplicate publishers and clock jumps — this burned one run already).
- `scripts/compare_slam_dlio_maps.py --map-dir <run>` — slam-vs-DLIO wall
  coherence (matched %, chamfer) + overlay images. Final state to beat:
  matched 92.7%, median 0.05 m.
- `scripts/perception_metrics.py <run>/replay_outputs` — topic rates, TF
  rates, /scan coverage.
- Session scratch scripts (recreate if gone, they are small): per-scan yaw
  error vs DLIO (grid-search best rotation per /scan), costmap smear
  (lethal-cell chamfer to DLIO reference split still/moving), analyze_run
  (map->odom peaks, time-synced walkability, lethal profile).
- IMPORTANT pgrep trap: a background wait loop whose own command line
  contains the pattern (`pgrep -f replay_bag_perception_g1.sh`) can match
  itself and hang forever. Wait on file existence or poll differently.

## Issue 1 — the 2D map updates slowly; robot "jumps inside the map"

User observation (RViz): while the robot walks, `/map` visibly updates only
every few seconds, so the robot appears to jump within the map; DLIO looks
smooth. Also: "in map frame the DLIO map rotates" (see Issue 2 — likely the
same root cause).

Known contributing settings to investigate, in order:

1. slam_toolbox insertion gating: `minimum_travel_distance/heading/time
   0.25/0.25/0.25` + the 0.20 rad/s yaw gate mean NO map paint during turns
   and sparse paint at 0.5 m/s walk. `map_update_interval: 0.5` further
   throttles re-render. Try: lower `minimum_time_interval`, lower
   `map_update_interval` (0.2?), and consider whether the yaw gate can be
   relaxed now that scans are GICP-registered (the gate predates the
   registered-cloud path; registered sweeps during turns may be fine to
   paint — MEASURE with the per-scan yaw-error script on a gate-disabled
   run before deciding).
2. The registered-scan rate: /scan ran at 7.2 Hz in replay (972/135 s) vs
   8.2 Hz cloud rate — the gate ate ~12%. Check nothing else starves it
   (accumulator TF waits, scan_queue_size 50 in slam params).
3. Note the "jump" may be partly an RViz artifact of map re-render cadence,
   but treat it as real until measured: log /map update timestamps vs robot
   displacement between updates.

## Issue 2 — while moving, the costmap "rotates with the robot" and floods
lethal; the DLIO map cloud "rotates" in map frame

This was NOT reproduced in replay (replay: 0 lethal robot cells, in-motion
lethal chamfer p90 17 cm). The user sees it in RViz — establish FIRST whether
they were watching a replay session or the LIVE robot, because the leading
hypothesis only exists live:

- **H1 (live-only, most likely): EKF-vs-DLIO frame divergence.** In replay,
  DLIO owns `odom->base_footprint` (ADR 0012 override) and everything is
  self-consistent. LIVE, the EKF (`robot_localization`, fed by
  `/lidar_odom_reliable` + AMO body odom `/odom_raw`) owns that TF while the
  registered cloud and DLIO's internal pose live in DLIO's own frame. Any
  yaw lag/offset between EKF output and DLIO pose makes: (a) the DLIO map
  cloud appear to rotate in map/odom frame, (b) STVL marks (placed via EKF
  TF from a cloud stamped in DLIO's frame... actually the corrector cloud is
  sensor-frame; STVL transforms via EKF TF at cloud stamp — if EKF yaw lags
  the true yaw during turns, marks rotate with the robot and smear lethal).
  Measure live or from a NEW live bag: record `/lidar_odom`, `/odom`, `/tf`,
  compute yaw(EKF) - yaw(DLIO) vs time, especially during turns; check EKF
  update rate and lag (see `docs/ekf_rate_diag.md` for prior EKF rate work,
  `ekf_fusion_params_lio.yaml` for the config). If confirmed, candidate
  fixes: EKF yaw gain/lag tuning, feeding the corrector/STVL path
  DLIO-consistent TF, or gating STVL marking on yaw rate like the map path.
- **H2: leveling/extrinsic error (user's suggestion — verify the R|t
  matrices).** Check the chain end-to-end with a floor+wall plane-fit script
  on a live bag: `MID360_config.json` extrinsic (roll 180 rights the cloud),
  `sensor_mount_rpy [0, 0.04014, 0]` + `mount_rotation_mode: direct` in
  `distortion_corrector_g1.yaml`, DLIO `baselink2lidar/R` (upright) vs
  `baselink2imu/R` (Rx(180) — driver never rights the IMU, pitfall P1 in the
  reference doc), `standing_height 0.78` vs reality (sensor measured ~1.21 m
  above floor in the bag, nominal 1.24 — 3 cm off already noted).
  Prior measurements: leveling residual 0.7 deg median / 2.6 deg p90 / 5.7
  deg max during fast turns (lidar_level_frame_publisher, IMU-driven).
  The 5.7 deg worst case IS enough to flood lethal at range while turning if
  it coincides with H1.
- **H3 (user's suggestion): filter the robot's back sector to exclude the
  operator.** The operator walks 0.5-1.0 m behind (measured azimuth
  [150,210] deg in base frame, ~100 pts/sweep at torso height) and gets
  painted into BOTH the map and costmap. Options: (a) azimuth sector filter
  in the distortion corrector self-filter (base-frame sector, e.g.
  rear +/-40 deg within 1.5 m) — affects costmap AND scan; (b) same sector
  filter only in cloud_accumulator (map only); (c) leave the costmap
  marking (a real obstacle behind the robot is safety-relevant!) and filter
  only the MAP. Recommend (c) unless the user wants the costmap blind
  behind. This is a design choice — ask the user before shipping.

## Constraints

- Do not regress: slam-DLIO coherence >= 92% matched / 0.05 m median on the
  bag; 0 lethal robot cells time-synced; lethal profile must stay
  self-healing (never monotonic).
- Safety path untouched unless justified: collision_monitor and STVL consume
  the ungated `/lidar/points_undistorted`; keep it that way unless evidence
  says otherwise.
- Per CLAUDE.md: update `docs/topic_map.md` + `scripts/kill_ros_nodes.sh` on
  any topic/node change; ASCII only; build only affected packages; validate
  with the replay harness before declaring anything fixed.
- Save all images the user should judge into `maps/g1_map_compare/report/`
  (numbered, self-describing names).
- The user judges final quality in RViz (`replay_bag_perception_g1.sh`
  without `--no-rviz`); do not call the work finished until they confirm.
- Ask the user about design choices you cannot ground in measurements
  (operator filtering scope, map latency vs robustness tradeoffs), and
  ground every change in a measured before/after on the bag. If live-only
  hypotheses (H1) need data the bag cannot provide, ask the user to record a
  new bag with `run_g1_real.sh --record-rosbag` including `/odom`,
  `/lidar_odom`, `/tf`, `/livox/lidar`, `/livox/imu`, `/imu/data`.
