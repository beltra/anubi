# R1 — EKF /odom rate at 4.85 Hz diagnosis plan

Open follow-up from the 2026-05-12 perception tuning round. The
`robot_localization` EKF (non-LIO config) on this Go2 publishes `/odom` and
`odom -> base_footprint` TF at a deterministic ~4.85 Hz instead of the
configured 50 Hz. The current production workaround is
`scripts/run_go2_real.sh --no-ekf-fusion`, which has `go2_hw_bridge` publish
`/odom` directly at ~290 Hz. This sidestep is fine for non-LIO operation
but blocks LIO mode (which needs the EKF as the DLIO + body-odom fusion
point).

This document captures what we already know, ranks the remaining
hypotheses, and lays out the test order to nail the root cause.

## What we already know

Verified live and offline against
`/root/rosbags/go2_real/go2_real_mapping_20260512_120809` (49.6 s):

- `frequency: 50.0`, `sensor_timeout: 0.1`, `transform_timeout: 0.2` all
  load correctly (verified with `ros2 param get`).
- Lowering `transform_timeout` from 0.2 to 0.05 had **no** effect on the
  rate. TF lookup is not the gate.
- `/odom_raw` is published at 290 Hz, `/imu/data` at 290 Hz. Inputs are
  healthy.
- `/odom` is published at 4.85 Hz with inter-message intervals of
  **0.001 s min, 0.208 s max** — bursty, not steady. The filter publishes
  per accepted measurement, not per timer tick.
- `odom -> base_footprint` TF rate matches the `/odom` rate (4.77 Hz live,
  4.83 Hz topic), so the publication itself is throttled, not subscriber-
  side filtering.
- `/diagnostics` consistently reports
  `"Frequency too low. Actual frequency 4.82 Hz, Min acceptable 43.2 Hz."`
- The yaw double-source config bug was real and has been fixed
  (commit `83e30d5`), but did **not** change the rate.
- Offline replay of `/odom_raw + /imu/data + /tf + /tf_static` into a
  fresh `ekf_node` reproduces the bug (1.17 Hz output). It is **not** a
  live-system contention issue.
- `/diagnostics` in offline replay also reports
  `"imu0_pose_timestamp = The imu0_pose message has a timestamp before
  that of the previous message received, this message will be ignored"`
  intermittently, plus the same for `odom0_pose`/`odom0_twist`/`imu0_twist`.
- Bag duplicates in /imu/data and /odom_raw: 208 / 14216 and 208 / 14233
  respectively (~1.5%). Strict-greater-than comparisons reject duplicates.

Together these point at **message rejection inside the filter** being the
gate. `predict_to_current_time` defaults to `false` in Humble, so when
most measurements are rejected the publish rate collapses to the rate of
accepted updates.

## Hypotheses, ranked

### High likelihood

1. **`imu0_remove_gravitational_acceleration: true` is wrong for the Go2**
   Sport Mode's onboard IMU bridge may already deliver linear acceleration
   with gravity removed. If we then subtract gravity *again* the resulting
   acceleration is essentially `-g` at rest — an unphysical state shock
   that the filter's outlier check rejects, in a tight loop.
   **Why we think it:** the rejection-style diagnostic in the live trace,
   plus the per-measurement publication pattern.
   **Cost:** 1 param toggle + restart.

2. **`predict_to_current_time: false` (default) + sparse accepted updates**
   With Humble's default, the EKF publishes only when a measurement is
   accepted. Anything else (timestamp rejected, covariance singular,
   quaternion non-unit, gravity rejection) silently drops without
   publishing. Hypothesis 1 explains *why* most are rejected; this
   explains *why* the output rate drops to the accepted-rate.
   **Test value:** flipping `predict_to_current_time` to `true` should
   pin output at 50 Hz regardless of accepted measurements. If that
   happens, the upstream rejection rate is the real bug to chase.

3. **Mass timestamp rejection from clustered duplicates**
   208 duplicate header stamps over 14000 samples is 1.5%. If duplicates
   are clustered (e.g., go2_hw_bridge re-publishes when /sportmodestate
   arrives but uses the same wall-clock stamp for adjacent IMU + odom
   pairs), bursts of rejections could starve cycles. Need histogram of
   duplicate inter-arrival in the bag.

### Medium likelihood

4. **IMU orientation quaternion not unit-norm**
   `validateOrientation()` rejects if `|norm - 1.0|` exceeds tolerance.
   Go2 IMU drivers occasionally publish 0.999-norm quaternions.
   **Test:** echo /imu/data, compute `qx² + qy² + qz² + qw²` over a few
   samples.

5. **Singular IMU `linear_acceleration_covariance`**
   Bag's first sample shows healthy `[0.02, 0.02, 0.02]` on diag, but
   later samples might differ. A 0 on diagonal causes Cholesky failure
   inside the EKF update step and silent rejection.

6. **`print_diagnostics: true` overhead under single-threaded executor**
   robot_localization's diagnostic updater runs on each cycle. At
   2 x 290 Hz inputs the single-threaded executor may stall queueing
   sensor callbacks behind diagnostic processing.

7. **Outlier rejection thresholds firing**
   `_pose_rejection_threshold` and `_twist_rejection_threshold` default
   to `numeric_limits<double>::max()` (no rejection), but if any of the
   per-sensor thresholds are overridden to a Mahalanobis distance the
   filter consistently exceeds, every message gets rejected.

### Lower likelihood

8. **QoS mismatch (RELIABLE pub, BEST_EFFORT sub) under Cyclone DDS load**
   Compatibility is met but high-rate dual-topic best-effort
   subscriptions sometimes behave oddly in Humble + cyclonedds.

9. **Sensor stamps slightly in the future relative to filter time**
   Would cause buffering. Possible if any clock-source jitters; unlikely
   on a single machine.

10. **two_d_mode + gravity removal numerical interaction**
    `two_d_mode: true` projects z/roll/pitch to zero. The gravity-removal
    step that depends on those projected-out components may produce
    invalid intermediate values. Related to hypothesis 1.

## Test order

Run on the live Go2 with `--use-ekf-fusion` (default), keeping the rest
of the stack identical. After each test, measure `/odom` rate with the
recipe at the bottom.

| # | Step | Cost | What it tells us |
|---|------|------|------------------|
| 1 | Echo /imu/data once; check `linear_acceleration.z` at rest (~9.81 means raw, ~0 means already gravity-removed) and quaternion norm (`qx²+qy²+qz²+qw²`) | 1 min | Direct evidence for hypotheses 1, 4, 8. The IMU's actual data tells us which branch we're on. |
| 2 | `ros2 param set /ekf_filter_node imu0_remove_gravitational_acceleration false` and re-measure | 1 min | If /odom jumps to ≥40 Hz: hypothesis 1 confirmed, root cause found. |
| 3 | If 2 doesn't fix: `ros2 param set /ekf_filter_node predict_to_current_time true` and re-measure | 1 min | If /odom jumps to 50 Hz but measurement-accepted-rate is still 5 Hz, hypothesis 2 confirmed and we have a workaround; the underlying message rejection is still a bug to chase. |
| 4 | If 3 doesn't fix: `ros2 param set /ekf_filter_node print_diagnostics false` and re-measure | 1 min | Eliminates hypothesis 6. |
| 5 | Inspect duplicate distribution in the bag: are the 208 duplicates clustered in time, or roughly uniform? | 5 min | Eliminates or confirms hypothesis 3. |
| 6 | Read `robot_localization 3.5.4` source: `RosFilter::enqueueMeasurement()`, `RosFilter::integrateMeasurements()` and any rejection log calls. Match what we see in `/diagnostics`. | 30 min | Most conclusive route if 1–5 don't pin it. |

Most steps are non-destructive runtime param toggles. None requires a code
change.

## Measurement recipe

After each param change, give the filter ~10 s to stabilize, then:

```bash
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
ros2 topic hz /odom
# or for finer detail:
ros2 topic delay /odom
# /diagnostics shows the filter's internal frequency tracker:
ros2 topic echo /diagnostics --once
```

## Don't forget

- `ekf_fusion_params_lio.yaml` does **not** trip the bug because it
  disables `imu0` entirely and uses `/odom_raw` for linear velocity only.
  Any fix here should be re-verified against the LIO config to make sure
  we don't regress it.
- `permit_corrected_publication` is a related knob worth knowing about;
  it would let late measurements still produce a publish even if they
  arrived out of order. Not a fix on its own.
- After R1 is fixed, the `--no-ekf-fusion` flag in
  `scripts/run_go2_real.sh` should stay (useful for bench testing without
  the EKF), but the LIO/non-EKF compatibility check should be revisited.

## Pointers

- Wrapper bypass: `scripts/run_go2_real.sh --no-ekf-fusion`
- Config files: `src/autonomy/robot_nav/config/ekf_fusion_params.yaml`
  (non-LIO), `ekf_fusion_params_lio.yaml` (LIO)
- Launch: `src/autonomy/robot_nav/launch/ekf.launch.py` and
  `src/platform/go2_bringup/launch/go2_bringup.launch.py` (use_ekf_fusion
  arg)
- Bag for offline reproduction:
  `/root/rosbags/go2_real/go2_real_mapping_20260512_120809`
- Related commit: `83e30d5` (yaw double-source config fix; independent of
  the rate bug but cleans the filter for the day this is revisited).
