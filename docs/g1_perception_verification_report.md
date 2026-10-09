# G1 Perception Verification Report

Independent re-measurement (2026-07-12) of the claims shipped on `feat/g1`
(uncommitted working tree) for the Unitree G1 / Livox Mid-360 perception
stack. Every number below was re-derived from raw data with fresh scripts;
the previous session's docs and scratch tooling were treated as claims under
test, and the four heaviest analyses were additionally audited by independent
verifier passes that re-derived the decisive numbers (all four audits:
no corrections required).

- Evidence bag: `bags/g1_planner_tune_20260710_175531` (143.5 s live G1 walk,
  operator trailing 0.5-2.8 m, fast turns t=32-38 s and 46-53 s).
- Evidence images: `maps/g1_verify/report/NN_*.png` (referenced per claim).
- Replay runs: `maps/g1_verify/R*` (see run matrix). One replay = full bag
  through the CURRENT shipped configs via `scripts/replay_bag_perception_g1.sh`.

## Run matrix

| Run | Config delta vs shipped | Used by |
|---|---|---|
| R1_shipped | none (restamp ON); RViz under Xvfb, screenshots | C2-C7, C10 |
| R2_prefix | `restamp_lidar_points:=false` (pre-fix clocks); RViz | C1 visual, C2, C4, C7 |
| R3_yawgate | `scan_accumulation_max_yaw_rate:=1.0` | C4 |
| R4_leverarm | DLIO `baselink2imu/t` reverted to `baselink2lidar/t` (scratch param copy) | C9 |
| R5_shipped2 | none (repeat) | run-to-run noise (C3, C4, C9) |
| C8_shipped / C8_prefix | offline EKF graph, isolated domain, `--clock 200`; shipped vs git-HEAD relay+covariances | C8 |

## Verdict table

Verdicts: CONFIRMED / REFUTED / PARTIAL (headline and substance split) /
INCONCLUSIVE. "Their number is wrong because ..." notes identify the
previous session's measurement error where my number disagreed.

| # | Claimed | Measured | Verdict | Margin (pre-fix -> shipped) | Evidence |
|---|---|---|---|---|---|
| C1 | cloud stamps drift -0.15 -> -2.9 s over the bag (~ -20 ms/s); all other topics ~0 | -0.153 s start, -2.94 s min, others ~0; 98.2% of sweeps outside the corrector's 0.25 s window; but NON-monotonic: plunge to -2.94 s in first ~7 s, +21 ms/s recovery, second plunge t=126-136 s; data proven fresh at arrival (DLIO-vs-body-IMU xcorr lag ~0) | PARTIAL (root cause CONFIRMED, drift-shape description wrong) | n/a | 01, `report/c1_clock_domains.json` |
| C2 | shipped: undistorted sweeps sit on DLIO walls even in 1.2-1.4 rad/s turns (best-fit residual median 0.03-0.05 m); pre-fix: rigid turn-direction yaw offset saturating +/-12-14 deg, residuals to 0.4 m; deskew active shipped / silently disabled pre-fix | turn-window (t=30-56 s, n=106) identity-residual median 0.103 m (R1) vs 0.236 m (R2), p90 0.199 vs 0.352, max 0.254 vs 0.412 [claim's "up to 0.4 m" OK]; best-fit residual median 0.039-0.050 m (R1) [claim 0.03-0.05 OK, but that is the fit, not placement]; the pre-fix offset DOES follow the stale-TF prediction (corr +0.955), but the claimed "+/-12-14 deg saturation" is the PRIOR SESSION'S +/-16 deg clipped grid search piling real offsets at the clip edge - true pre-fix offsets reach +/-85 deg (median 30 deg, 79% beyond 16 deg); worst identical sweep: 0.412 m / -84.2 deg (R2) vs 0.090 m / -6.0 deg (R1); deskew ACTIVE in R1 (recovered rate vs gyro at capture corr +0.999); pre-fix deskew NOT disabled - it consumed 1.9-2.9 s stale IMU (corr +0.998, gain 0.99 vs gyro at the broken stamp; -0.089 at capture) | PARTIAL (placement fix CONFIRMED 2.3x in turns; saturation figure and disable-mechanism are their measurement artifacts) | turn-window identity residual -56% (0.236 -> 0.103 m), yaw offset -70% (30 -> 9 deg median) | 02-05, 19 |
| C3 | slam-vs-DLIO walls 91.1% <=0.15 m, median 0.05 m, mean 0.097 m | 90.9% (repo tool) / 91.2% (independent recompute; 0.36 pp float-boundary artifact at exactly 3 cells), median 0.050, mean 0.104 (R5 repeat: 88.8%, 0.114) - claim reproduced within run noise; DLIO healthy: wall RMS 4.9-6.2 cm at the stationary-sensor baseline, no double walls; loop check clean (structure), robot itself ended 0.84 m from start. Caveat: with the slam matcher OFF the 2D map is rasterized from DLIO-registered scans, so this metric is partly pipeline self-consistency, not independent truth | CONFIRMED | n/a | 06-08 |
| C4 | shipped coverage ~65% vs 70-72% pre-fix; old number inflated by broken-clock mis-paint; yaw gate 0.60->1.0 recovers nothing (65.2%) and drops walls to 89.1% | per-run coverage: R1 66.1 / R5 68.3 / R2 69.0 / R3 64.9; controlled common-grid (noise 0.3 pt): shipped 68.7-69.0, pre-fix 70.9 (+2.0 real), yawgate 65.8 (-3.0); attribution: 92.7% of pre-fix-only paint sits ON real DLIO structure (81.3% under the conservative beyond-1-cell variant; phantom 5.3% and symmetric between runs) | PARTIAL (yaw-gate numbers CONFIRMED; 5-7 pt drop and the mis-paint interpretation REFUTED) | coverage: -2.0 pts (a real, small regression on real structure) | 09-10 |
| C5 | lethal cells near structure: still ~99%, moving 93-95%, p90 moving ~0.2 m; footprint-lethal 1/553, stale marks only | still 96.5%, moving 85.2%, p90 moving 0.548 m (verifier-confirmed); their numbers reproduce ONLY with a floor-contaminated reference (99.9/99.5) or by reading the overall p90 (0.234 m) as "moving" - their measurement error, not mine; footprint-lethal 2/560 (t=52.9 s: 24% FRESH supports; t=54.8 s: stale-only), both single cells at the padded boundary from near-field self-band returns | PARTIAL (qualitative story holds; all four headline numbers optimistic) | vs pre-fix (my A/B): lethal within 0.30 m of structure 83.6% -> 91.4%, p90 0.52 -> 0.23 m | 11-13, 19, `report/costmap_flood_ab.json` |
| C6 | floor never enters mapping /scan -> /map (0 floor-only cells, 0 occupied in traversed swath) nor costmap (0 all-below-0.25 lethals); floor z ~ +0.08 while walking | all four zeros CONFIRMED (3,072 map cells, 12,918 local + 4,227 global lethal cells); floor z median +0.038 m (claim's +0.08 overstates ~2x), p90 0.151; margin to 0.25 m at 2-3 m: 0.054 m = 1.03 deg leveling headroom (definition-censored at the z<0.20 cap) | CONFIRMED (floor-z characterization overstated) | n/a | 14 |
| C7 | corrector output leveled: floor normal 0.7 deg median / 2.6 p90 walking, 5.7 max in turns; pre-fix leveling silently OFF (raw body tilt); TF yaw == 0 | R1 walking 2.76 / 8.24 deg, turn max 10.6 - 4x the claim (their 0.7 matches the ALL-sweeps median 0.86, i.e. their walking mask was broken); R2 leveling RAN (stale-matched, 1179/1179 sweeps) and is BETTER walking (2.10 med, turn max 3.07): gravity_align INJECTS pelvis attitude into an already gait-stabilized head; TF yaw 0.000000 deg (7177 samples) but rotation is IDENTITY by design - the TF never represents leveling | REFUTED (except TF-yaw sub-claim) | NEGATIVE: shipped fix makes leveling injection fresher, walking tilt 2.10 -> 2.76 deg | 15 |
| C8 | EKF vs DLIO turning: shipped p90 2.5 / max 13.2 deg, lag +20 ms; pre-fix p90 ~8.8 / max 23 / lag ~140 ms; rate ~50 Hz | pre-fix p90 9.08 / max 23.1 deg, cov pinned 5.0% of frames; shipped p90 2.78 / max 15.2 deg, pinned 0%, rate 49.97 Hz, max gap 40 ms, pos err p90 23.8 -> 7.7 mm; lag NOT reproduced as group delay: turn-window xcorr +20 -> +10 ms (the pre-fix defect is amplitude attenuation from pinning, not a time shift) | CONFIRMED (lag framing PARTIAL) | turn yaw p90 -69%, max -34%, pos err p90 -68% | 16, `report/c8_results.json` |
| C9 | Mid-360 internal lidar->IMU lever arm in DLIO cut worst turn-time correction steps ~28%, coherence unchanged | turn-window tails ON vs OFF: -3.6% (p99) / +2.7% (p99.9) / +1.2% (top50), inside the 10-25% same-config run spread - the claimed 28% was single-run MAX-statistic noise; the turn-specific metric points the OTHER way: same-stamp correction jumps p99 53/43 mm (ON) vs 35 mm (OFF); coherence unchanged (90.9/88.8 vs 91.4%) | PARTIAL (benefit REFUTED; no-harm CONFIRMED) | ~0% (none measurable) | 17 |
| C10 | /scan >= 6 Hz; TF ~100 Hz gap <= 0.2 s; costmap 4-5 Hz gap <= 1 s; lethal decay -5..-11 cells/s never monotonic; map-change displacement p95 0.9-1.25 m with residual gaps coinciding with >0.6 rad/s spins + startup | /scan mean 7.98 Hz but 3.60 Hz sustained min with 1.50 s gap during yaw-gated spins; TF 100.3 Hz gap 0.120 s OK (83.1 Hz unique stamps); costmap 3.90 Hz gap 0.30 s OK; decay stronger than claimed (post-spike -41..-181, slow tail -8.2 cells/s; 0/258 windows monotonic); map-change p95 0.54-0.76 m (BETTER than claimed; their 0.9-1.25 not reproducible with any defensible estimator); coincidence rationale REFUTED - residual gaps happen in straight walking over already-mapped ground | PARTIAL (timeliness substance holds; /scan guarantee and the spec-gate coincidence rationale fail) | n/a | 18 |

## New findings (defects and corrections the claims missed)

Ranked by operational impact. None of these are fixed in this session (the
task forbids shipping changes); proposed fixes are directions, each grounded
in the evidence cited.

1. **`gravity_align` actively degrades the G1 nav cloud while walking**
   (image 15). The locomotion controller already gait-stabilizes the head;
   rotating the cloud by pelvis-IMU roll/pitch INJECTS attitude (correlation
   +0.76 with the injection model, -0.77 against the unleveled-body model).
   Simulated `gravity_align: false` + static mount righting on the same
   sweeps: walking tilt 1.74/4.80 deg median/p90 and turn max 2.61 deg vs
   shipped 2.76/8.24/10.6. Proposed fix: disable gravity_align on the G1
   (keep deskew), or level from a head-mounted attitude source.
2. **The pre-fix failure mechanism is mischaracterized in the docs**: broken
   device stamps did NOT disable deskew/leveling via the 0.25 s timeout. The
   corrector's IMU history (400 msgs ~ 8 s at 50 Hz) still contains samples
   around the stale stamp, so deskew ran with ~2 s-old IMU data - exact
   fingerprint: R2's recovered deskew rate correlates +0.998 (gain 0.99)
   with the gyro AT THE BROKEN STAMP and -0.089 with the gyro at capture
   (image 05); spot values: sweep t=31.7 s applied +0.26 rad/s == gyro at
   t=29.2 s (true rate -0.83). `imu_timeout_sec` is dead logic for
   late/stale stamps - nearest_msg() always finds a buffered sample within
   ~10 ms of ANY stamp inside the buffered past. Any future "keep original
   stamps, grow buffers" approach reintroduces this silently; a max-age
   check against now() would not.
2b. **The shipped stack retains a ~0.3 s content-vs-stamp lag in turn
   transients** (image 02/03 R1 panels): header-stamp placement misaligns
   turn-transient sweeps by median 0.103 m / up to 35 deg, and a single
   global +0.30 s TF-lookup shift repairs most of it (median 0.105 ->
   0.058 m; per-sweep best shift median +0.30 s). Consistent with the fixed
   `restamp_offset_sec 0.10` underestimating the true sweep-start age when
   delivery runs below 10 Hz (~121 ms spacing + relay/corrector latency).
   Candidate fix: derive the restamp offset from the measured inter-sweep
   delivery interval, or stamp from the per-point timestamp span instead of
   a constant.
3. **Near-field self-band lethal marks** (images 12, 13): 356 unmatched mark
   events (3,542 lethal instances) first appear <= 0.6 m from the robot with
   ALL supports at z 1.0-1.4 m captured 0.19-0.51 m from the sensor. Both
   footprint-lethal events (2/560 grids, one with 24% fresh supports) come
   from this population. The corrector's `min_radius 0.20` / body box does
   not cover the head-adjacent band. Proposed fix: raise `min_radius` or
   extend the self-filter box upward; re-check with a bag where no operator
   is nearby.
4. **Transient floor-lift marking** (image 14): at t=29.7-29.85 s two
   consecutive sweeps read flat floor 2.4-3.6 m ahead at z 0.20-0.40 m
   (transient ~5-6 deg leveling/deskew residual vs the 1.03 deg steady-state
   headroom), STVL marked ~99 open-floor lethal cells; decay cleared them in
   0.6-8 s and none reached the final map. Same root cause as finding 1.
5. **The coverage spec-bar recalibration (70-72% -> ~65%) is not supported**
   (images 09, 10). Pre-fix extra paint is real structure (92.7% strict,
   81.3% conservative; phantom paint ~5% and symmetric between runs). The
   shipped stack has a real ~2 pt coverage regression on real obstacles
   (178 slice cells lost / 132 gained, controlled reference). Recalibrate
   the bar to what shipped actually does (~67-69%) only while stating the
   regression, or recover the 2 pts; do not attribute it to mis-paint.
6. **Run-to-run noise on the map metrics is ~2.1-2.4 pts** (R1 vs R5), about
   double the +/-1 pt the docs assume; most of it is DLIO cloud sampling
   (~6.7k-point PCDs). Single-run headline figures (91.1%, the C9 "28%")
   need repeats; the C9 claim specifically was single-run MAX-statistic
   noise (25% spread on that statistic between identical-config runs).
7. **/lidar_odom stream hygiene**: 17.2% of messages carry duplicate header
   stamps (2,477 republications, one stamp repeated 55x, plus 6 zero-stamp
   messages); unique-stamp TF rate is 83 Hz, not 100. Duplicate-stamp pose
   republications ARE the scan-match corrections (up to 91 mm) - any dt-based
   analysis that drops dt==0 pairs silently discards exactly the largest
   corrections (this bit the previous session's C9 measurement).
8. **/scan starves during yaw-gated spins**: sustained 3.60 Hz (5 s window)
   with gaps of 1.50 / 1.10 / 0.85 s during the fast turns. The ">= 6 Hz"
   spec line only holds as a run mean. If turn-time paint matters, the yaw
   gate is the binding constraint, not slam cadence.
9. **The "unreachable spec gate" rationale for map cadence is wrong but the
   metric is fine** (image 18): measured p95 displacement between visible
   /map changes is 0.54-0.76 m (better than the claimed 0.9-1.25 m band,
   which no defensible estimator reproduces), and the residual gaps occur
   during straight-line walking over already-mapped ground - not during
   spins or startup (0/2 spin-dominated; 5/7 zero-overlap).
10. **Restamped per-point times overlap across sweep boundaries** (C3): each
    sweep is shifted by its own header delta, so 210/1178 consecutive sweep
    pairs overlap in per-point time (median 0.5 ms, max 25.6 ms; original
    stream max 0.47 ms). Additionally, per-point times are natively
    non-monotonic WITHIN every Mid-360 sweep (line interleave, steps to
    -461 us), and the float64 absolute-ns field quantizes to 256 ns (the
    relay's shift is exact to 1 ULP - preservation claim CONFIRMED). DLIO
    tolerates all of this; document that consumers must not assume sorted
    per-point times.
11. **DLIO map floor is tilted 1.47 deg laterally** (normal
    [0.003, 0.025, 0.9997]): floor z runs -1.9 cm at y=+2..5 to +18 cm at
    y=-8..-5 (20 cm over 10 m), datum +4.8 cm at the odom origin. Any
    z-band slicing (scan band, STVL heights, coverage slices) inherits this
    tilt; it also caps how well "floor at odom z=0" holds away from start.
12. **Mid-360 stamping mechanism** (image 01): stamps advance almost exactly
    100.0 ms per delivered sweep regardless of the true delivery rate
    (5.9-10.6 Hz measured; exactly-200 ms stamp gaps mark device-side sweep
    drops). The offset is therefore an integral of the rate deficit, not a
    fixed-rate clock drift. Cross-correlation of live DLIO yaw-rate vs the
    body IMU in receive time shows lag ~0 through the worst offset windows:
    the DATA is fresh at arrival, only stamps lie -> restamp-at-arrival is
    architecturally correct, and PTP alone would not fix this (the stamping
    logic, not the clock, is the problem).
13. **DLIO pose/twist mutual inconsistency during yaw oscillations** (C2):
    /lidar_odom twist wz tracks the gyro (corr +0.98), but the yaw rate of
    the odom->base_footprint TF derived from the same odometry correlates
    with neither - part of the shipped turn-transient placement residual
    lives inside DLIO's pose output, not in the corrector.
14. **The evidence bag's "fast turns" are stepped yaw oscillations, not
    sustained 1.2-1.4 rad/s turns**: smoothed |wz|>0.8 rad/s holds for only
    12% / 2% of the two windows, net heading change ~22 deg per window.
    Claims phrased against "1.2-1.4 rad/s turns" rest on ~7 sweeps.
15. **Corrector output frame labeling is ~1.1 deg off the TF chain** even on
    still sweeps in both runs (whole-sweep rigid fit vs recorded
    lidar_link->lidar_stabilized): the node levels from its own matched IMU
    sample while the TF comes from lidar_level_frame_publisher, and the two
    disagree systematically - consumers treating the frame label as exact
    inherit the offset (relates to finding 1).
16. Smaller items: `bag_perception_g1.launch.py` docstring still says the
    yaw-gate default is 0.20 "as live" (actual default and live value 0.60);
    the "mostly pink costmap" impression is quantified at max 7.3% (R1) /
    8.0% (R2) colored cells - the RViz look is zoom plus smear placement
    (image 19); robot ended 0.84 m / -8.2 deg from its start pose (loop
    checks must compare structure, not poses); local costmap value encoding
    verified {0, 68-92, 99, 100}; first C8 attempt was invalidated by
    orphaned `ros2 run` children surviving kill - the A/B harness now
    direct-execs node binaries and pre-flight-checks the domain.

## Method notes

- Replays were serialized with `scripts/kill_ros_nodes.sh` + empty-graph
  checks between runs; replay outputs recorded with sim time so all
  recordings share the original bag time base.
- The pre-fix state was reproduced via `restamp_lidar_points:=false` only
  (no file edits); C9 via a scratch copy of `dlio_params_g1_bag_live.yaml`;
  C8 via an isolated-domain offline graph (git-HEAD relay extracted with
  `git show`).
- All heavy per-claim analyses ran as independent agents with their own
  scripts; C2-C5 were additionally audited by skeptic agents that re-derived
  the decisive numbers from scratch (all confirmed, minor-issue findings
  documented in their scratch dirs).
- Scratch analysis code lives under the session scratchpad
  (`analysis/`, `c8/`, `agent_c*/`, `verify_C*/`); nothing under `src/`,
  `scripts/`, or configs was modified.
