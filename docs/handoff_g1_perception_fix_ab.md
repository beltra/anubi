# Prompt: G1 perception -- A/B-test and ship fixes for the verified defects

Copy everything below into a new session.

---

You are a fix-and-verify session in the ROS 2 Humble workspace `/ws/anubi`
(branch `feat/g1`, uncommitted working tree). An independent verification
session (2026-07-12) re-measured every claim of the G1 perception stack and
found real defects. Your job: A/B-test candidate fixes for each defect,
ship the winning configuration, and VERIFY the result with the quantitative
methods below. You work autonomously to a goal: DO NOT stop, summarize, or
hand back until every issue F1-F7 is either (a) fixed and verified against
its numeric target with repeat runs, or (b) proven unreachable by exhausted
A/B + external research, with the evidence written down and a justified
revised target. "I tried my proposal and it did not work" is a midpoint,
not an endpoint: when a proposed technique misses its target, fan out
research subagents (codebase, ecosystem repos, web) for alternative
techniques, rank them, and test the next one. Use subagents/workflows for
parallel analysis; keep replays serialized.

Read first (authoritative, in order):
1. `docs/g1_perception_verification_report.md` -- verdict table, defect
   list, and the measurement conventions this prompt's targets use.
2. `maps/g1_verify/` -- the verification runs (R1_shipped, R2_prefix,
   R3_yawgate, R4_leverarm, R5_shipped2, C8_*) and `report/` images/JSONs.
   R1/R5 are your shipped-baseline references.
3. `docs/g1_perception_tuning_plan.md`, `docs/topic_map.md` -- context and
   contracts. Treat pre-verification claims in the plan doc as corrected by
   the report wherever they disagree.

## Ground facts (do not rediscover)

- Evidence bag: `bags/g1_planner_tune_20260710_175531` (143.5 s live G1,
  operator trailing 0.5-2.8 m, stepped-yaw turn windows t=31.6-38.0 and
  45.5-53.2 s; peaks 1.29 rad/s but |wz|>0.8 rad/s holds only ~12%/2% of
  each window).
- Replay: `scripts/replay_bag_perception_g1.sh <bag> --no-rviz --save-map
  maps/g1_fix_ab/<run>` (~3.5 min/run). ALWAYS `scripts/kill_ros_nodes.sh`
  first and confirm `ros2 node list` is empty. `--launch-arg k:=v`
  overrides any launch arg; A/B via `--launch-arg` + scratch param-file
  copies ONLY until a winner is chosen, then ship it in the real configs.
- The recorder saves /tf /tf_static /map /scan /lidar_odom /lidar/points
  /lidar/points_undistorted /local_costmap/costmap /global_costmap/costmap
  with sim-time receive stamps (= bag time).
- Mid-360 clock truth (report C1): cloud header stamps tick a fixed
  ~100 ms/sweep in the device timebase while true delivery runs
  5.9-10.6 Hz; offset wanders -0.15..-2.94 s, NON-monotonic. The DATA is
  fresh at arrival (proven); only stamps lie. The shipped
  `cloud_restamp_relay` writes header = now()-0.10 s and shifts per-point
  f64-ns timestamps by the same delta (exact to 1 ULP = 256 ns). Per-point
  times are natively non-monotonic within sweeps (Livox interleave) and,
  after restamping, overlap sweep boundaries by up to 25.6 ms -- DLIO
  tolerates this; never assume sorted point times.
- The corrector (`robot_nav/src/distortion_corrector.cpp`) preserves point
  order and count input->output (join forensics on identical stamps); its
  `imu_timeout_sec` is DEAD logic for stale stamps (nearest-buffered-sample
  test, 400-msg ~8 s buffer, never fires inside the buffered past).
- Map-metric run noise: per-run compare (`scripts/compare_slam_dlio_maps.py`)
  is +/-2.1-2.4 pts between identical-config runs. NEVER decide an A/B on
  a per-run delta under ~3 pts from single runs. Use the controlled method:
  rasterize both arms' slam maps onto ONE common odom grid and score
  against ONE fixed reference slice (R1's dlio_map.pcd) -- measured noise
  0.3 pts. For non-map metrics, still run >=2 replays per arm for any
  margin comparable to its spread.
- Offline node harness traps (cost a contaminated run last time): exec node
  BINARIES directly (`/opt/ros/humble/lib/<pkg>/<node>`, `python3
  <installed script>`), never `ros2 run` (kill orphans the child; it
  contaminates the next run on the same domain); pre-flight
  `pgrep -af '[e]kf_node|...'` (bracketed so pgrep cannot match itself);
  `ros2 bag play --clock 200` for anything timing-sensitive (default 40 Hz
  /clock quantizes sim-time timers). /lidar_odom carries 17.2% duplicate
  header stamps + 6 zero stamps -- dt-based analyses must handle dt==0 and
  zero stamps explicitly or they silently drop the largest corrections.
- Never trust a leftover analysis script (including the verification
  session's) without spot-checking one number it produces.

## Issues, proposed techniques, and numeric targets

Baselines below are R1_shipped/R5_shipped2 values from the report. Every
target comes with its measurement method in the VERIFICATION section.

### F1 (highest impact): gravity_align injects pelvis attitude
The G1 locomotion controller already gait-stabilizes the head; rotating the
cloud by pelvis-IMU roll/pitch INJECTS attitude. Baseline floor-normal tilt
of `/lidar/points_undistorted` in its own frame: walking median 2.76 deg /
p90 8.24 / turn max 10.6; ~3.3 deg constant during the initial standstill.
Try in order:
 (a) `gravity_align: false` in `distortion_corrector_g1.yaml` (keep deskew
     ON, keep static mount righting). Verification predicted 1.74/4.80 deg
     walking median/p90, turn max 2.61 -- confirm by measurement, not by
     the prediction.
 (b) If (a) hurts some other gate: attenuated injection (scale roll/pitch
     by k in [0,1], A/B k=0.3/0.5) or low-pass-only leveling (remove stance
     lean, pass gait dynamics).
 (c) Level from the Livox internal IMU (head-mounted, already in the bag as
     /livox/imu) instead of the pelvis /imu/data.
Decide the standstill question explicitly: fit the gravity direction from
the Livox accelerometer during the still segments and determine whether the
~3.3 deg start-segment tilt is a real body lean the output SHOULD remove or
a leveling-chain bias. Related sub-defect F1b: the output frame label is
~1.1 deg off the recorded lidar_stabilized TF chain even when still (node
levels from its own IMU sample, TF from lidar_level_frame_publisher whose
ROTATION is identity by design) -- make one source of truth; if (a) wins,
lidar_stabilized must equal the exact static mount transform.
TARGET: walking median <= 1.8 deg, p90 <= 5.0 deg, turn-window max
<= 3.5 deg, still-segment residual <= 1.0 deg after the stance-lean
decision; frame-label-vs-TF disagreement <= 0.3 deg.

### F2: residual ~0.3 s content-vs-stamp lag in restamped clouds
Baseline: turn-window identity-placement residual median 0.103 m / best-fit
yaw median 9 deg; a single +0.30 s TF-lookup shift repairs it to 0.058 m
(so the content is ~0.3 s older than the restamped header claims; the fixed
`restamp_offset_sec 0.10` underestimates sweep age at ~8 Hz delivery).
Try in order:
 (a) Rate-adaptive offset in `cloud_restamp_relay.py`: offset = EMA of the
     measured inter-arrival interval (+ small transport constant), instead
     of the 0.10 constant.
 (b) Span-anchored: header = now() - (last_point_ts - first_point_ts),
     using the cloud's own per-point span.
 (c) Cheap probe first: constant 0.30 (this validates the mechanism in one
     run but is wrong when delivery recovers to 10 Hz -- do not ship a
     constant unless (a)/(b) fail and the bag says delivery is stable).
Also measure how much of the residual lives in DLIO itself (report finding:
odom->base_footprint TF yaw-rate during oscillations correlates with
neither the gyro nor DLIO's own twist). If a significant share is DLIO-side,
A/B `odom_to_tf` fed by the per-scan (non-propagated) pose, or DLIO
settings, before blaming the relay.
TARGET: turn-window identity residual median <= 0.065 m, p90 <= 0.14 m;
best-fit yaw median <= 3 deg; per-sweep best-TF-shift distribution centered
within +/-0.05 s of zero (search grid must extend at least +/-0.7 s -- do
not clip the grid; that artifact created a false claim once already).

### F3: near-field self-band lethal marks (both footprint-lethal events)
356 mark events with all supports at z 1.0-1.4 m captured 0.19-0.51 m from
the sensor; caused both footprint-lethal grids (t=52.9 s with 24% FRESH
supports, t=54.8 s stale-only). The corrector self-filter (`min_radius
0.20`, body box top z 0.55 in pelvis frame) does not cover the head band.
Try: raise `min_radius` toward ~0.55, and/or extend the self-filter box to
cover the head region. SAFETY CHECK REQUIRED: quantify what genuinely close
REAL obstacles would be lost -- verify the trailing operator's legs/torso
(0.5-2.8 m) still mark, and report the new blind radius explicitly.
TARGET: footprint-lethal grids 0/560; near-field self-band events < 10
(from 356); operator still marked (lethal cells on the operator trail
within 1 pt of baseline).

### F4: transient floor-lift marking
Two sweeps at t=29.7-29.85 s read flat floor 2.4-3.6 m ahead at z 0.20-0.40
(transient 5-6 deg residual) -> ~99 open-floor lethal cells until decay.
Expected to be fixed by F1; verify separately, do not assume.
TARGET: 0 open-floor lethal-cell clusters attributable to floor returns
lifted above 0.25 m (scan the whole run with the C6 method, not just
t=29.7 s).

### F5: ~2 pt real coverage regression vs pre-fix
Controlled common-grid coverage: shipped 68.7-69.0 vs pre-fix 70.9; the
shipped stack loses 178 real-structure slice cells (gains 132). Find WHERE
the lost paint went (attribute the 178 cells: which sweeps painted them in
R2 -- were those sweeps yaw-gated, rear-sector-filtered, starved by F6, or
z-band-shifted by the DLIO floor tilt of 1.47 deg?), then fix the actual
cause. Do NOT re-try yaw gate 1.0 (verified: -3.0 pts). F6's fix is a
likely contributor -- re-measure F5 after F6.
TARGET: controlled coverage >= 70.0 against the same fixed reference with
walls within 1.0 pt of baseline (90.9 controlled), both arms scored on the
common grid, 2 runs.

### F6: /scan starves during yaw-gated spins
Sustained 3.6 Hz (5 s window) with 1.50/1.10/0.85 s gaps exactly during the
turns. The sweeps are GICP-registered, so gated sweeps are not garbage.
Try: in `cloud_accumulator.py`, when the yaw gate trips, emit the current
single registered sweep (no concat) instead of dropping the cycle; or gate
only the accumulation depth (3->1) by yaw rate.
TARGET: /scan 5 s-window min rate >= 6 Hz, max gap <= 0.5 s, with walls
and coverage within noise of baseline (controlled method), and the C5
lethal decomposition not worse.

### F7: /lidar_odom stream hygiene
17.2% duplicate header stamps (scan-match correction republications, one
stamp 55x) and 6 zero-stamp messages; `odom_to_tf` rebroadcasts them.
Fix at the consumer edge (skip non-increasing stamps in odom_to_tf.py and
lidar_odom_relay.py, keeping the LAST pose of a duplicate group -- it is
the corrected one) unless you find a clean DLIO-side switch. Confirm the
EKF path is unaffected (C8 harness below).
TARGET: 0 non-increasing-stamp TF broadcasts; odom->base_footprint unique-
stamp rate ~100 Hz preserved as message rate; C8 gates unchanged.

Also fix while you are there (no A/B needed): the stale docstring in
`bag_perception_g1.launch.py` (yaw-gate default is 0.60, not 0.20), and add
a defensive max-age-vs-now() check to the corrector's IMU matching so the
F2 class of failure can never silently return (log throttled warning when
matched sample is older than ~0.5 s vs now).

## VERIFICATION methods (use these; they were validated this week)

Recreate the analyzers from these specs (do not trust leftovers; the
verification scratch under /tmp may be gone):

1. Floor-normal tilt (F1/F4): per-sweep robust plane fit on
   `/lidar/points_undistorted` IN ITS OWN FRAME: candidates z in
   [-1.45,-1.05], horizontal range < 4 m, iterative LS with 2.5-sigma
   rejection, >= 200 inliers; report walking (|v|>0.2 from /lidar_odom
   twist, smoothed +/-0.25 s) median/p90 and turn-window max. Standstill
   sanity: plane RMS should stay ~6-9 mm; if RMS grows your fix broke
   deskew.
2. Placement residual (F2): transform every undistorted sweep in
   t=30-56 s (plus every 5th elsewhere) into odom at its header stamp via
   the run's OWN recorded TF (slerp between bracketing odom->base_footprint
   samples; NEVER interpolate across map->odom steps); filter z 0.3-1.5 m,
   range 0.5-6 m; score = median 2D NN distance to the run's own
   dlio_map.pcd z-band (cKDTree). Note the metric floor: DLIO map self-
   spacing ~0.033 m. Best-fit yaw via UNCLIPPED grid search about the robot
   position. TF-shift sweep: repeat the identity scoring with the TF lookup
   shifted by deltas in [-0.7,+0.7] s; the argmin distribution must center
   at 0 after the fix.
3. Deskew-active forensics (any change touching the corrector or relay):
   join /lidar/points to /lidar/points_undistorted per sweep on identical
   header stamps (order/count preserved), remove the static leveling part,
   fit the implied per-sweep angular rate, correlate against /imu/data
   gyro-z at capture: corr must stay >= +0.99. If it correlates with the
   gyro at ANY OTHER time, you reintroduced stale matching.
4. Costmap decomposition (F3/F4/F6 regression gate): decode all
   /local_costmap/costmap grids (lethal == 100); matched = lethal cell
   within 0.30 m (2D) of dlio_map.pcd points z 0.20-2.3 (NOT the full
   cloud -- a floor-contaminated reference reproduces the old inflated
   numbers); split still/moving by |v|<0.1 and |w|<0.2. Baselines to not
   regress: still 96.5%, moving 85.2%, p90-moving 0.548 m, footprint-lethal
   2/560 (target 0 after F3). Footprint test: padded 0.46 x 0.56 m
   rectangle oriented at the TF pose at grid stamp.
5. Map A/B (F5/F6): per-run `scripts/compare_slam_dlio_maps.py` for eyes
   only; DECIDE on the controlled common-grid method (both arms' slam maps
   rasterized into one odom grid via each run's final map->odom, scored
   against R1's dlio_map.pcd slice; noise 0.3 pts). Two runs per arm
   minimum when a margin is < 3 pts.
6. Floor filtering (F4 gate): final /map occupied cells with no DLIO
   structure (z 0.25-2.3) within 0.15 m must have non-floor supports at
   the 0.15 m/all-range convention (baseline 0); traversed-swath occupied
   cells = 0; lethal cells with all supports z < 0.25 = 0.
7. EKF gate (F7 and any relay change): offline A/B harness on an isolated
   ROS_DOMAIN_ID -- play /odom_raw /lidar_odom /tf_static with --clock 200,
   run the installed lidar_odom_relay.py (direct exec) with the shipped G1
   covariances + robot_localization ekf_node (`-r __node:=ekf_filter_node`,
   params `ekf_fusion_params_lio.yaml`, remap odometry/filtered -> /odom,
   static base_footprint->base_link identity), record /odom
   /lidar_odom_reliable /tf. Gates: turning (|w|>0.3) yaw-err p90 <= 3.0
   deg, max <= 16 deg, relay pinned-fraction 0%, EKF ~50 Hz, pos err p90
   <= 10 mm.
8. Timeliness (F6): /scan windowed min rate and max gap; TF and costmap
   rates; lethal-count decay (no 10 s window strictly increasing).
9. Visual: one RViz pass per shipped candidate under Xvfb (Xvfb :5 +
   `import -window root` every 10 s during the replay; view target frame
   `map`) and eyeball turn windows vs baseline screenshots in
   `maps/g1_verify/R1_shipped/rviz_shots/`.

## Research escalation (when a proposal misses its target)

Fan out subagents BEFORE burning more replays blindly: (a) a code-reading
agent over the relevant node (corrector / accumulator / relay / DLIO
config) enumerating every parameter and code path that touches the failing
metric; (b) an ecosystem agent researching how other Mid-360 + humanoid /
DLIO stacks solve it (livox_ros_driver2 timestamp modes and publish_freq,
PTP/gPTP, Unitree G1 community stacks such as Relo02/Navigation, DLIO
issue tracker, STVL docs) -- verify claims against actual config files
before importing numbers; (c) a devil's-advocate agent asked to refute
your current hypothesis with the recorded data. Synthesize, pick the next
technique, and A/B it. Repeat until the target is met or you can PROVE
(with data, not fatigue) the target is unreachable on this bag -- in that
case write the proof into the final report and propose the revised target.

## Shipping and reporting rules

- Ship winners into the real configs/scripts (this is a fix session, not
  a report-only session). Keep every A/B loser in the final writeup with
  its numbers. Per repo rules: update `docs/topic_map.md` and
  `scripts/kill_ros_nodes.sh` on any topic/node change; ASCII only; build
  only affected packages; keep sim parity (the sim already runs
  gravity_align off -- check nothing else diverges).
- Preserve the safety path semantics: collision_monitor/STVL keep consuming
  the ungated `/lidar/points_undistorted`; F3 changes the self-filter, so
  document the new blind radius explicitly.
- Final deliverables: `docs/g1_perception_fix_ab_report.md` (one section
  per issue: baseline, arms tested, numbers, winner, verification results
  incl. repeats), images in `maps/g1_fix_ab/report/` (numbered,
  self-describing), and updated `docs/g1_perception_tuning_plan.md` status.
- End state definition (your stop condition): all F1-F7 targets green with
  repeat-run evidence, all regression gates (methods 3-8) green on the
  final shipped configuration IN ONE final confirmation replay pair (two
  identical runs of the final config), report written. The live robot
  still needs a new bag for hardware confirmation -- state that as the
  single remaining user action; everything replay-verifiable must be
  verified by YOU before you stop.
