# Independent verification: G1 SLAM + perception stack (2026-07-12 state)

You are an independent verification session in the ROS 2 Humble workspace
`/ws/anubi`. A previous session diagnosed and shipped fixes to the Unitree G1
(head-mounted Livox Mid-360) perception stack. Your job is to VERIFY, not to
extend: re-measure every claim below from raw data with your own scripts,
judge each PASS / FAIL / INCONCLUSIVE with your own numbers, quantify the
margin of improvement, and hunt for defects the previous session missed.
Treat the repo docs (`docs/g1_perception_tuning_plan.md`,
`docs/handoff_g1_perception_moving_issues.md`) as CLAIMS to check, not as
truth. Do not reuse the previous session's analysis scripts even where they
survive; write your own so errors do not propagate. Where your number
disagrees with a claimed number, say so plainly and investigate which
measurement is wrong before assigning a verdict.

## Ground facts (mechanics you should not waste time rediscovering)

- Evidence bag: `bags/g1_planner_tune_20260710_175531` (143 s, real G1 walk
  in a furnished room, an operator walking 0.5-1.0 m behind the robot the
  whole time, two fast-turn windows around t=32-38 s and t=46-53 s relative
  to bag start). It contains the full live stack output including `/tf`,
  `/odom`, `/lidar_odom`, `/imu/data`, `/livox/lidar`, `/livox/imu`, and the
  live `/map` of the (older, pre-fix) configs.
- Replay harness: `scripts/replay_bag_perception_g1.sh <bag> --no-rviz
  --save-map maps/g1_verify/<run>` replays through the CURRENT shipped
  configs; `--launch-arg k:=v` overrides any launch argument. ALWAYS run
  `scripts/kill_ros_nodes.sh` first and confirm the ROS graph is empty; a
  leftover session silently corrupts runs. One replay takes ~4 min.
- The shipped state is UNCOMMITTED work on branch `feat/g1`; `git diff` shows
  the exact changed surface. To reproduce the PRE-fix behavior do NOT edit
  files: pass `--launch-arg restamp_lidar_points:=false` (disables the clock
  fix; the old plain relay runs instead).
- Existing comparison tool (you may use it, but spot-check its math once):
  `scripts/compare_slam_dlio_maps.py --map-dir <run>` rasterizes the DLIO 3D
  map (z 0.29-1.44 m slice) onto the slam map grid and reports matched% and
  chamfer both directions.
- Nav2 G1 footprint is 0.40 m (x, fore-aft) x 0.50 m (y) + 0.03 padding;
  grid resolution 0.05 m; STVL marks z 0.25-2.3 m from
  `/lidar/points_undistorted`; the mapping `/scan` band is z 0.29-1.44 m
  from DLIO's registered cloud. The local costmap publishes full grids at
  5 Hz (`always_send_full_costmap: true`); note that rolling-window
  `costmap_updates` patches carry NO origin, so never composite them.
- rosbag2 sqlite: `messages.timestamp` is RECEIVE time; header stamps are a
  different thing entirely on this robot (see C1). The bag records with
  original QoS; recorded `/livox/lidar` publishes ~one sweep AFTER its own
  header stamp.
- Container may lose apt ROS packages on reset; if `ros2 launch` dies with
  "package not found", install the list in the memory note
  (`ros-humble-slam-toolbox`, `ros-humble-spatio-temporal-voxel-layer`,
  `ros-humble-pointcloud-to-laserscan`, `ros-humble-topic-tools`,
  `ros-humble-robot-localization`, nav2 binaries, etc.).
- Methodological traps that burned the previous session -- avoid them:
  (a) a background wait loop whose command line contains the pattern it
  pgreps can match itself; (b) an rclpy TF probe that does BLOCKING
  `lookup_transform(timeout=...)` inside a single-threaded spin starves its
  own TF listener and reports false failures -- use instant lookups or a
  spin thread; (c) the distortion corrector preserves point ORDER and count
  input->output, which enables per-point correction-field forensics by
  joining input/output sweeps on identical header stamps; (d) run-to-run
  noise on wall-matched% is about +/-1 percentage point -- never judge a
  1-point delta from single runs, repeat the run.

## Claims to verify (previous session's numbers in brackets)

C1. Clock domains: `/livox/lidar` header stamps drift against receive time
    from about -0.15 s to -2.9 s over the bag (~ -20 ms/s), while
    `/livox/imu`, `/imu/data`, `/lidar_odom` stay at ~0. This is the root
    cause of the shipped restamp fix. Verify per-topic stamp-minus-receive
    over time from the bag itself.

C2. Distortion correction + alignment (the user-visible fix). With the
    shipped stack, per-sweep clouds on `/lidar/points_undistorted`,
    transformed at their header stamp via the replay TF chain, sit ON the
    DLIO map walls even during the 1.2-1.4 rad/s turns [claimed residual
    median ~0.03-0.05 m at best fit; pre-fix runs showed a rigid
    turn-direction yaw offset saturating +/-12-14 deg with residuals up to
    0.4 m]. Run one shipped replay and one `restamp_lidar_points:=false`
    replay and measure both. Also verify deskew is ACTIVE in the shipped
    replay and was silently DISABLED pre-fix once drift exceeded the
    corrector's 0.25 s IMU timeout (the corrector supports
    `publish_diagnostics`; or prove it via input/output correction-field
    forensics per trap (c) -- recovered angular velocity should match
    `/imu/data` at the sweep time).

C3. DLIO mapping correctness. Claimed: slam 2D map matches the DLIO 3D map
    slice at 91.1% within 0.15 m, median 0.05 m, mean 0.097 m (shipped,
    runT5-equivalent). Beyond reproducing that: judge DLIO itself --
    plane-fit RMS of large wall segments in `dlio_map.pcd` (double-wall or
    smear means odometry drift), floor-plane flatness and its z-level vs
    the odom datum, trajectory continuity (per-frame displacement steps of
    `/lidar_odom`, worst turn-time steps), and start/end self-consistency
    (the robot returns near its start; check first-vs-last wall geometry
    around the start pose). DLIO input must be time-monotonic: confirm the
    restamp relay preserves per-point-minus-header relative times exactly.

C4. Coverage recalibration claim. Under the shipped (honest-clock) stack,
    DLIO-slice coverage by the slam map reads ~65%, vs 70-72% pre-fix; the
    previous session claims the OLD number was inflated by mis-projected
    paint (broken-clock band slicing), and that relaxing the accumulator
    yaw gate 0.60 -> 1.0 recovers nothing (65.2%) while dropping walls to
    89.1%. Verify with your own runs, and adjudicate the interpretation:
    are the pre-fix "extra" painted cells real structure or mis-paint?
    (E.g. attribute pre-fix-only occupied cells against the full DLIO
    cloud and against floor z.) This claim decides whether a spec bar gets
    recalibrated; be adversarial.

C5. Obstacle false positives. On a shipped replay, decompose ALL lethal
    cells of the 5 Hz local costmap stream: fraction within 0.30 m of DLIO
    structure (z 0.20-2.3) split still vs moving [claimed still ~99%,
    moving 93-95%, p90 ~0.2 m], operator-attributable fraction (near the
    robot's trailing path), floor-attributable fraction (cells whose only
    supporting returns are below z 0.25), single-point phantoms, and
    self/arm returns (lethal within 0.6 m of the robot with body-frame
    positions consistent with arm swing). Footprint-lethal states claimed
    1/553 transient (1-2 cells at the padded footprint boundary, all
    traceable to 1-3 s-old stale marks, not fresh returns) -- verify count
    AND attribution.

C6. Floor filtering. Verify the floor never enters (a) the `/scan` used for
    mapping (no occupied cells in the final `/map` attributable only to
    floor returns; the traversed-footprint swath has 0 occupied cells) and
    (b) the costmap (no lethal cells whose supporting cloud returns are all
    below z 0.25 in odom). Measure the actual floor z distribution in
    `/lidar/points_undistorted` while walking (claimed odom z ~ +0.08 with
    gait-dependent spread) and report the margin to `min_obstacle_height
    0.25` at the 3.0 m marking range including observed leveling residual.

C7. Gravity alignment. The corrector outputs in `lidar_stabilized`
    (roll/pitch-leveled, yaw-follows-body). Fit the floor plane per sweep
    on the OUTPUT cloud: the normal should be +z within a small residual
    [prior sessions claimed ~0.7 deg median / ~2.6 deg p90 while walking,
    worst ~5.7 deg in fast turns -- re-measure; pre-fix the leveling was
    silently OFF, so pre-fix sweeps should show the raw body tilt instead].
    Also verify the `base_footprint -> lidar_stabilized` TF has yaw == 0
    and roll/pitch consistent with `/imu/data` at the same instants.

C8. EKF turn-yaw fidelity (live config; affects the real robot, not the
    replay). Offline A/B: play `/odom_raw` + `/lidar_odom` + `/tf_static`
    from the bag on an isolated ROS_DOMAIN_ID, run `lidar_odom_relay.py`
    with the G1 covariances from `g1_bringup.launch.py` and
    `ekf_node` with `config/ekf_fusion_params_lio.yaml` (remap
    `odometry/filtered` -> `/odom`, add a static
    `base_footprint -> base_link` identity: the bag's dynamic one is not
    played). Claimed with shipped params: yaw(EKF)-yaw(DLIO) while turning
    p90 2.5 deg / max 13.2 deg, cross-correlation lag +20 ms; claimed
    PRE-fix (git HEAD relay/EKF params) p90 ~8.8 deg / max 23 deg / lag
    ~140 ms. Verify both, plus EKF output rate ~50 Hz and position error.

C9. Mid-360 internal IMU lever arm in DLIO. Claimed: adding the
    [11.0, 23.29, -44.12] mm housing-frame offset (folded through the
    mount) cut worst turn-time DLIO correction steps ~28% with unchanged
    map coherence. Verify by replaying with a param-override copy of
    `dlio_params_g1_bag_live.yaml` that reverts `baselink2imu/t` to equal
    `baselink2lidar/t`, and compare per-frame `/lidar_odom` step-speed
    tails during turns.

C10. Timeliness and self-healing. On the shipped replay: `/scan` >= 6 Hz;
     odom->base_footprint TF ~100 Hz with max gap <= 0.2 s; local costmap
     full grids ~4-5 Hz with max gap <= 1 s; lethal-cell count time series
     decays after transients (fit slope excluding the first 20 s; claimed
     ~ -5 to -11 cells/s, never monotonic growth); robot displacement
     between visible `/map` content changes while moving [claimed p95
     ~0.9-1.25 m with the residual gaps coinciding with >0.6 rad/s spins
     and the startup-calibration window -- verify the coincidence, since it
     justified declaring a spec gate unreachable].

## Deliverables

1. `maps/g1_verify/report/` -- numbered, self-describing images for every
   visual claim (cloud-vs-walls during turns before/after, floor-plane
   tilt time series, lethal decomposition, clock-domain plot).
2. A verdict table in `docs/g1_perception_verification_report.md`: one row
   per claim -- claimed number, YOUR number, verdict
   (CONFIRMED / REFUTED / PARTIAL / INCONCLUSIVE), margin of improvement
   where before/after applies, and one-line evidence pointer.
3. A "new findings" section: any defect, false positive source, or metric
   the claims missed. Look actively; do not stop at confirming.
4. Do NOT modify shipped configs, launches, or scripts (scratch analysis
   scripts and `--launch-arg` overrides only). If you find a real defect,
   document it with evidence and a proposed fix; the fix ships in a later
   session.

Budget guidance: the before/after pairs (C2, C4, C8) each need two runs;
everything else shares those recordings. Plan the run matrix first so no
replay is wasted; record any topic you need at replay time (the harness
records /tf, /map, /scan, /lidar_odom, clouds, and local costmap full
grids by default).
