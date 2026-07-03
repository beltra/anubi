# G1 + SONIC Whole-Body Controller: First Setup and Real-Robot Bringup

This guide covers running the anubi navigation stack on the Unitree G1 with the
NVIDIA SONIC whole-body locomotion policy as the low-level controller
(`--locomotion sonic`), as an alternative to the default AMO policy. It is the
end-to-end procedure for a FIRST deployment: one-time setup of both machines,
sim validation, and the supervised hardware bringup.

Deployment layer reference: `talos-robotics-ai/sonic-g1-locomanipulation`
(called "the sonic repo" below), which sits on top of
`NVlabs/GR00T-WholeBodyControl` ("GR00T"). Its `docs/setup.md` and
`docs/bringup.md` are authoritative for the Jetson side; this guide summarizes
them and adds everything anubi-specific.

## 1. Architecture and topology

Two machines, both on the robot's ethernet segment (`192.168.123.x`):

- **Nav PC** (this workspace): Nav2 + SLAM + DLIO + EKF + safety chain +
  `cmd_vel_to_sonic` (the ZMQ bridge node in `g1_bringup`).
- **Jetson Orin Nano**: only the SONIC real-time controller
  (`g1_deploy_onnx_ref`: policy + target-velocity planner, 50 Hz whole-body
  command over Unitree DDS).

```
 Nav PC                                                Jetson                   robot
 Nav2 /cmd_vel -> collision_monitor -> velocity_limiter
   -> /cmd_vel_safe -> cmd_vel_to_sonic ----ZMQ tcp----> g1_deploy_onnx_ref --DDS--> rt/lowcmd
                       (binds *:5556)      :5556         (policy + planner)  <-DDS-- rt/lowstate
 g1_state_bridge <------------------ DDS rt/lowstate + rt/odommodestate ------------ robot
```

Key contracts:

- The command chain up to `/cmd_vel_safe` is IDENTICAL to the AMO variant
  (`docs/topic_map.md`); only the last hop differs: `cmd_vel_to_sonic`
  translates Twist into SONIC planner ticks (world-frame movement/facing
  direction + speed) over ZMQ instead of `{vx, vy, yaw}` over WebSocket.
- The STATE side is UNCHANGED: `g1_state_bridge` keeps reading `rt/lowstate` /
  `rt/odommodestate` (read-only) for `/odom_raw`, `/imu/data` and the body TF;
  DLIO remains the primary odometry. Both controllers require the stock Unitree
  high-level/sport service to be released, so the state sources are the same.
- `cmd_vel_to_sonic` BINDS the ZMQ PUB socket on the nav PC; the Jetson
  controller CONNECTS to it. This is inverted from what the sonic repo's
  single-host scripts assume (see 2.1, step F).
- **Single-writer rule: never run `amo_driver` and `g1_deploy_onnx_ref` at the
  same time.** Two writers on `rt/lowcmd` is dangerous. Switching controller
  means stopping one process, verifying it is dead, then starting the other.

Velocity envelopes (Nav2 + bridge caps are matched per profile, and are the
ONLY yaw limits in the sonic chain -- SONIC itself does not clamp yaw or the
ZMQ speed input):

| Profile (`--sonic-profile`) | Mode | vx fwd | vx rev | vy | wz | Files |
|---|---|---|---|---|---|---|
| `slow_walk` (default) | SLOW_WALK (1) | 0.5 | 0.2 | 0.4 | 0.4 | `nav2_params_g1_sonic_slow.yaml` + `g1_sonic_slow_walk_params.yaml` |
| `walk` | WALK (2) | 0.8 | 0.3 | 0.4 | 0.5 | `nav2_params_g1_sonic_walk.yaml` + `g1_sonic_walk_params.yaml` |

The caps are REALIZED m/s: the SONIC planner reaches ~0.85x of the commanded
speed, and the bridge compensates with `speed_gain: 1.18` applied AFTER the
clamp. `slow_walk` deliberately equals the AMO envelope so the first hardware
runs isolate the controller swap from behavior tuning. Use `walk` only after
`slow_walk` is validated; its 0.8 m/s exceeds what the collision monitor's
static SlowZone/StopZone polygons were sized for (the time-based ApproachZone
scales automatically) -- revisit `collision_monitor_params_g1.yaml` before
running `walk` in cluttered spaces.

## 2. One-time setup

### 2.1 Jetson (SONIC controller)

Target: Jetson Orin Nano 8 GB, JetPack/L4T, aarch64. Follow the sonic repo
`docs/setup.md`; summarized:

A. Clone GR00T at the pinned commit and apply the sonic repo patch
   (RT thread pinning, stale-command hold, TensorRT workspace cap):

```bash
git clone https://github.com/NVlabs/GR00T-WholeBodyControl.git "$HOME/GR00T-WholeBodyControl"
cd "$HOME/GR00T-WholeBodyControl"
git checkout 0e35637
git apply --ignore-whitespace /path/to/sonic-g1-locomanipulation/patches/gr00t-sonic-deploy.patch
```

B. Install the deploy toolchain (`gear_sonic_deploy/scripts/install_deps.sh`)
   and the MuJoCo sim venv (`install_scripts/install_mujoco_sim.sh`).

C. Install TensorRT **10.7** for aarch64 and point `TENSORRT_ROOT` at it
   (`libnvinfer.so.10` must resolve from `$TENSORRT_ROOT/lib`; do NOT link an
   older system TensorRT).

D. Download the models (`python3 download_from_hf.py`): policy encoder/decoder
   ONNX + `planner/target_vel/V2/planner_sonic.onnx`. The `.trt` engines are
   built on first launch (~5 min); keep them cached.

E. Build the deploy binary (`scripts/build.sh`) and grant real-time limits
   once (`sudo scripts/setup_rt_limits.sh`, then re-login and verify
   `ulimit -r` prints 99 in the launching shell/tmux).

F. **Split-topology change (required for anubi):** the sonic repo's
   `start_deploy_real.sh` hardcodes `--zmq-host localhost`; with the bridge on
   the nav PC the controller must connect there instead. Make the host
   env-overridable in the sonic repo:

```bash
# in scripts/start_deploy_real.sh (and start_deploy_sim.sh if used this way):
#   --zmq-host localhost            ->   --zmq-host "${SONIC_ZMQ_HOST:-localhost}"
SONIC_ZMQ_HOST=<nav-pc-ip> scripts/start_deploy_real.sh
```

G. Validate the Jetson side standalone with the sonic repo's sim bringup
   (`docs/bringup.md` section 1: MuJoCo sim on DDS loopback + controller +
   `--demo forward`). A clean pass proves toolchain, models, engines and RT
   limits before anything touches the robot or the nav PC.

### 2.2 Nav PC (anubi stack)

A. Build the workspace as usual (see `.claude/CLAUDE.md` build commands). The
   only new runtime dependency is **pyzmq** for `cmd_vel_to_sonic`
   (`apt install python3-zmq`, or `pip install pyzmq`; already added to
   `docker/Dockerfile.real`).

B. Network: the nav PC needs (a) a NIC on the robot segment `192.168.123.x`
   for the Unitree DDS state (`g1_state_bridge --net-if <iface>`) and the
   Livox Mid-360, and (b) TCP reachability FROM the Jetson to the nav PC on
   port 5556 (the Jetson connects out; open the port if a firewall is active).

C. DDS separation (recommended): the Unitree SDK and the SONIC controller use
   raw DDS domain 0 for `rt/*` regardless of ROS settings. Set a nonzero
   `ROS_DOMAIN_ID` for the ROS 2 stack on both machines' shells if the ROS
   graph must stay clear of the robot's DDS traffic. `g1_state_bridge` is
   unaffected (it initializes the Unitree channel factory on domain 0
   explicitly).

D. Smoke-check the bridge alone (no robot needed):

```bash
ros2 run g1_bringup cmd_vel_to_sonic --ros-args -p cmd_vel_topic:=/cmd_vel_safe
# separate shell:
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```

The node must log its caps/mode and bind without error. (The planner ticks go
nowhere until the Jetson controller connects.)

## 3. Validate in sim first (always)

Two independent sim checks, in this order:

1. **SONIC loopback sim on the Jetson** (sonic repo `docs/bringup.md` section
   1): validates policy + planner + controller timing. Optionally point the
   anubi bridge at it: run `start_sim.sh` + `start_deploy_sim.sh` with
   `SONIC_ZMQ_HOST=<nav-pc-ip>`, then drive from the nav PC with
   `ros2 topic pub` as above -- this exercises the REAL wire path
   (nav PC ZMQ -> Jetson controller) end to end with zero robot risk.
2. **anubi Gazebo sim with the SONIC Nav2 envelope** (nav PC or dev machine):

```bash
scripts/run_g1_sim.sh --locomotion sonic                      # slow_walk envelope
scripts/run_g1_sim.sh --locomotion sonic --sonic-profile walk # walk envelope
```

   The sim base stays the kinematic VelocityControl ghost (no gait is
   simulated); this validates Nav2 tuning, costmaps and the safety chain
   against the sonic velocity caps before hardware.

## 4. Real-robot bringup

> Follow the sonic repo `docs/bringup.md` safety framing: robot on a
> hoist/gantry for first tests, verified E-stop in hand, a human supervising at
> all times. The physical E-stop is the primary safety; every software watchdog
> is a backstop.

### 4.1 Safety gates (all true before any motion)

- G1 powered, hoisted, clear of people; E-stop verified and in hand.
- Jetson-robot ethernet up (`ping 192.168.123.161` from the Jetson).
- Nav PC on the robot segment; Jetson can reach nav PC port 5556.
- **Unitree's stock high-level/sport service stopped/released** (per your
  standard Unitree procedure) so only one controller writes `rt/lowcmd`.
- **`amo_driver` is NOT running anywhere** (`pgrep -f amo_driver` empty).
- Exactly one `g1_deploy_onnx_ref` running (after step 4.2).

### 4.2 Start order

1. **Jetson -- controller:**

```bash
tmux new-session -d -s rdeploy \
  "SONIC_ZMQ_HOST=<nav-pc-ip> scripts/start_deploy_real.sh > /tmp/sonic_deploy_real.log 2>&1"
until grep -qE "Init Done|out of memory" /tmp/sonic_deploy_real.log; do sleep 2; done
grep "\[RT\]" /tmp/sonic_deploy_real.log   # expect FIFO+pin for all 4 workers
```

   It must not keep printing `LowState is not available` (robot state absent)
   and must not print `out of memory` (drop page caches per the sonic repo
   troubleshooting and relaunch).

2. **Nav PC -- sensors + odometry only, no commands yet:**

```bash
scripts/run_g1_real.sh --no-nav --net-if <robot-iface>
```

   Keep the robot still for the first ~5 s (DLIO IMU/gravity calibration).
   Verify against `docs/topic_map.md`: `/odom_raw`, `/imu/data`,
   `/lidar/points_undistorted`, `/lidar_odom`, `/odom`, TF chain complete.

3. **Nav PC -- full stack with SONIC (robot still hoisted):**

```bash
scripts/kill_ros_nodes.sh
scripts/run_g1_real.sh --locomotion sonic --net-if <robot-iface>
```

   The bridge re-sends the (idempotent) start command every second, so the
   bridge and controller may start in either order; the controller enters
   planner mode as soon as it connects. On the hoist, send a short forward
   goal or a manual twist and confirm stepping, then stop (kill the goal; the
   bridge watchdog idles the gait within ~0.3 s of silent cmd_vel, the SONIC
   planner within ~1 s of lost ticks).

### 4.3 Feet-down progression (pause for sign-off between stages)

Mirrors the sonic repo progression, driven through the nav stack:

1. Hoisted idle -- stable suspended stand, no `[Safety]` lines, motor temps ok.
2. Hoisted slow forward/turn via short Nav2 goals (slow_walk profile).
3. Hoisted stop behavior: cancel a goal mid-motion; verify watchdog idle.
4. **Sign-off.** Feet down, idle only.
5. Feet-down short goals in open space at `slow_walk`; verify clean starts,
   stops, in-place turns, and that the collision monitor SlowZone/StopZone
   trigger as expected when approaching an obstacle.
6. Mapping runs, then localization runs (`run_g1_real.sh localization --map ...`).
7. Only after all of the above is repeatable: `--sonic-profile walk`, first in
   open space; re-verify stopping distances against the collision zones.

### 4.4 Stop / E-stop

- Physical E-stop: primary, always in hand.
- Kill commands only: stop the Nav2 goal, or `Ctrl-C` the nav stack -- the
  bridge idles the gait (and on shutdown sends the SONIC stop command).
- Kill the controller: `pkill -9 -f g1_deploy_onnx_ref` on the Jetson (the
  patched writer soft-holds joints at reduced stiffness on command loss --
  compliant, not rigid; the robot can sag).

## 5. Runtime behavior and tuning

- **Watchdog layering** (fastest first): `cmd_vel_to_sonic` `cmd_timeout: 0.3`
  (cmd_vel silence -> IDLE) -> SONIC planner ~1 s (tick loss -> idle) ->
  patched 500 Hz writer stale-hold (`SONIC_MOTOR_COMMAND_STALE_MS`). The
  upstream `velocity_limiter` watchdog (0.5 s) additionally zeroes
  `/cmd_vel_safe` if Nav2 goes silent.
- **Open-loop heading:** SONIC has no yaw-rate input; the bridge integrates
  `angular.z` into a facing direction. Nav2 closes the loop through
  localization. A bridge restart mid-run resets the integrated heading and
  causes a brief heading transient until Nav2 corrects -- avoid restarting the
  bridge while a goal is active.
- **speed_gain:** 1.18 compensates the planner's ~0.85x speed realization and
  is applied AFTER the caps (caps are realized m/s). Set it to 1.0 to fall
  back to uncorrected upstream behavior; Nav2 still navigates, just slower
  than commanded.
- **Changing envelopes:** always change the Nav2 profile and the bridge caps
  together (they ship as matched file pairs; the launch selects both from
  `--sonic-profile`). Never widen the bridge caps alone.
- **Upper-body / carry poses:** not ported to `cmd_vel_to_sonic`. For
  loco-manipulation (fixed arm/wrist pose while walking), run the sonic repo's
  own `cmd_vel_to_sonic.py --ros2 --cmd-vel-topic /cmd_vel_safe --arm-preset
  carry ...` ON the Jetson with the controller pointed back at `localhost`
  (unset `SONIC_ZMQ_HOST`). The upstream bridge then subscribes to
  `/cmd_vel_safe` over ROS 2 DDS, so the Jetson needs ROS 2 Humble + the same
  `ROS_DOMAIN_ID` as the nav PC. The anubi `cmd_vel_to_sonic` keeps running but
  binds unused in this setup (no subscriber connects to it), which is harmless.
  See the sonic repo `docs/bringup.md` section 4 for pose tables and
  hoist-first validation rules.

## 6. Troubleshooting

| Symptom | Check / fix |
|---|---|
| Controller hangs at `LowState is not available` | Robot state absent: robot off/E-stopped, wrong `SONIC_DDS_IFACE`, or ethernet down. |
| Controller `Init Done` but robot ignores goals | Jetson cannot reach nav PC :5556 (firewall/route); check the bridge log shows the bind and `ss -tnp \| grep 5556` on the nav PC shows the Jetson connection. |
| Robot walks ~15% slower than commanded | `speed_gain` left at 1.0 -- expected upstream behavior; set 1.18. |
| Robot will not turn | Planner ticks missing facing updates: verify the anubi bridge is the one connected (not a stale upstream bridge in `--frame body`). |
| Gait stutters between commands | `cmd_timeout` below the actual cmd_vel period; chain publishes at 20 Hz, keep >= 0.3 s. |
| `CUDA initialization failure (out of memory)` on the Jetson | Unified-memory pressure: `sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"`, relaunch (sonic repo troubleshooting). |
| Periodic `soft-holding` / `MotorCommand stale` | State logging left on (`SONIC_ENABLE_LOGS`), or Jetson memory/disk pressure. Never enable logging on hardware. |
| Two controllers fighting / erratic motion | `amo_driver` or the stock sport service still alive alongside SONIC. Stop everything; enforce the single-writer rule. |
| `/odom_raw` silent under SONIC | `rt/odommodestate` availability with the sport service released is a hardware-validation item (same as AMO); if absent, run `--no-lio` is NOT the fix -- DLIO still provides odometry, but check the EKF config expectations in `docs/topic_map.md`. |

## 7. File map (what `--locomotion sonic` touches)

| File | Role |
|---|---|
| `src/platform/g1_bringup/scripts/cmd_vel_to_sonic_node.py` | Twist -> SONIC planner-tick ZMQ bridge (binds :5556; caps, yaw clamp, watchdog, speed_gain, idempotent start) |
| `src/platform/g1_bringup/config/g1_sonic_slow_walk_params.yaml` | Bridge profile: SLOW_WALK mode + slow_walk caps |
| `src/platform/g1_bringup/config/g1_sonic_walk_params.yaml` | Bridge profile: WALK mode + walk caps |
| `src/autonomy/robot_nav/config/nav2_params_g1_sonic_slow.yaml` | Nav2 envelope matching slow_walk |
| `src/autonomy/robot_nav/config/nav2_params_g1_sonic_walk.yaml` | Nav2 envelope matching walk |
| `src/platform/g1_bringup/launch/g1_bringup.launch.py` | `locomotion` / `sonic_profile` selection (AMO files untouched) |
| `src/simulation/g1_sim/launch/sim_bringup_g1.launch.py` | Same selection for the sim (envelope only) |
| `scripts/run_g1_real.sh` / `scripts/run_g1_sim.sh` | `--locomotion` / `--sonic-profile` wrapper options |

The AMO configuration (`g1_params.yaml`, `nav2_params_g1.yaml`, `cmd_vel_to_amo`,
`amo_driver`) is intentionally untouched by all of the above.
