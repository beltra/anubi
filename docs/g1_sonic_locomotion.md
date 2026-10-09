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

**Already set through section 2 once?** Jump to section 8 (day-of quickstart) for
the return-visit checklist. First time on this hardware: do sections 2-4 in order.

## 1. Architecture and topology

Two machines, both on the robot's ethernet segment (`192.168.123.x`):

- **Nav PC** (this workspace): Nav2 + SLAM + DLIO + EKF + safety chain +
  `cmd_vel_to_sonic` (the ZMQ bridge node in `g1_bringup`). It MUST sit on the
  wired robot LAN `192.168.123.x` -- that is where the Livox UDP stream and the
  Unitree DDS state live. A laptop qualifies if it is dual-homed: ethernet into
  the robot switch AND wifi for internet (see section 2.3).
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
  direction + speed) over ZMQ instead of `{vx, vy, yaw}` over WebSocket. Unlike
  the AMO bridge it also SUBSCRIBES `/odom` (the EKF-fused body pose) to anchor
  the body->world heading conversion closed-loop on measured yaw (section 5).
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

**Deployment topology -- on-board Jetson vs tethered nav PC.** anubi ships the
nav-PC/Jetson SPLIT above by design: with SONIC, keep the Nav2 stack on an
ethernet-tethered laptop and run only `g1_deploy_onnx_ref` on the Jetson. The
confirmed-working reference (`/home/beltra/unitree`) instead runs everything on-board
the Orin Nano, but only because it replaced Nav2 with a much lighter planner (CUDA
voxel map + Python VFH/Pure-Pursuit). anubi's Nav2 stack (MPPI controller + STVL voxel
layer + EKF + slam_toolbox) is several times heavier and would oversubscribe the 6-core
Orin Nano while SONIC needs 3 clean cores -- exactly the CPU-contention that starves
LowState and drops the robot (section 4.5). So do NOT co-locate anubi's Nav2 stack with
SONIC on the Jetson without first swapping the planner; that is a separate effort. The
AMO variant is lighter on the ROS side (the policy runs in its own GPU-bound container)
and is more plausible on-board, but validate core headroom first.

Because the nav stack (and therefore the Livox driver) runs on the nav PC in the split,
`MID360_config.json` `host_net_info` must be the **nav PC's** IP on the robot LAN
(anubi default `192.168.123.224`). The reference file uses `192.168.123.99` because it
runs the driver on-board the Jetson. The LiDAR's own IP (`192.168.123.120`) and the
`roll:180` mount are identical in both and are robot-side facts -- do not change them.
Set `host_net_info` to whatever machine actually runs `livox_ros_driver2`.

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

A. Provision the nav PC. The base is the same navigation runtime as the sim
   (ROS 2 Humble + Nav2 + `slam_toolbox` + `robot_localization` + STVL +
   `pointcloud_to_laserscan` + DLIO); either `apt`-install those or use the
   `anubi-g1-real:humble` image (`docker/Dockerfile.real`, run with
   `--network host` so Unitree DDS and the Livox UDP stream reach the
   container). On top of that base the G1 hardware layer adds FOUR things the
   sim image does NOT have -- provision all four (the sim dev container has only
   pyzmq of these):

   1. **pyzmq** (`cmd_vel_to_sonic`): `apt install python3-zmq` or
      `pip install pyzmq`.
   2. **unitree_sdk2py + Cyclone DDS** (`g1_state_bridge`, and the read-only DDS
      transport that `g1_locomotion` exposes and the bridge imports):
      `pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git`.
      Its `cyclonedds` build needs a Cyclone DDS C library; if pip cannot find
      one, build Cyclone DDS from source and export `CYCLONEDDS_HOME` before the
      pip install (see the unitree_sdk2_python README). TWO gotchas this pip
      install leaves behind, both of which crash the stack at runtime, not install
      time:
      - **Missing CRC lib.** The pip-from-git install does NOT ship the prebuilt
        `unitree_sdk2py/utils/lib/crc_*.so` (not declared as package_data), so
        `g1_state_bridge` dies at startup with
        `OSError: .../unitree_sdk2py/utils/lib/crc_amd64.so: cannot open shared
        object file`. Restore it: copy the repo's `utils/lib/` into the installed
        package, or compile the vendored `crc32_core`
        (`tools/unitree_ros2/example/.../common/motor_crc.cpp`) to
        `crc_amd64.so` (x86_64) / `crc_aarch64.so` (arm64) and drop it in
        `<site-packages>/unitree_sdk2py/utils/lib/`. `observe_only` still
        constructs a `CRC()`, so the state bridge needs this even though it never
        signs a `LowCmd`.
      - **numpy gets bumped to 2.x.** This pip step pulls `numpy>=2` (+opencv),
        whose ABI break makes ROS 2 Humble C-extension nodes fail to import.
        Re-pin afterward: `pip install "numpy<2"` (the `Dockerfile.real` image
        pins `numpy<2` for exactly this reason).
   3. **Livox-SDK2** (needed to COMPILE *and* RUN `livox_ros_driver2`):
      `git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git`, then
      `cmake -S Livox-SDK2 -B Livox-SDK2/build && cmake --build Livox-SDK2/build -j &&
      cmake --install Livox-SDK2/build && ldconfig`. The driver `dlopen`s
      `liblivox_lidar_sdk_shared.so` at RUNTIME (it is not a link-time `NEEDED`
      entry, so `ldd` on the node does not show it), and aborts with
      `Could not load library ... liblivox_lidar_sdk_shared.so` if the SDK is not
      installed / in `ldconfig`. Keep the SDK installed on the robot, not just on
      the build host.
   4. **livox_ros_driver2 package.xml** (the vendored driver gitignores it, so
      colcon cannot see the package until it is generated). Generate it WITHOUT
      the destructive helper: `src/vendors/livox_ros_driver2/build.sh` first does
      `rm -rf ../../build ../../devel ../../install` and then a full NON-symlink
      `colcon build` of the whole workspace -- fine on a fresh checkout, but it
      wipes an existing tree and drops this repo's standard build flags. On an
      already-built tree do the two file copies it does, by hand:
      `cd src/vendors/livox_ros_driver2 && cp -f package_ROS2.xml package.xml &&
      cp -rf launch_ROS2/ launch/`, then build normally.

   Build up to the real bringup package (the livox driver needs its two extra
   CMake defines):

   ```
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install \
     --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
     --packages-up-to g1_bringup
   source install/setup.bash
   ```

   Verify the hardware layer is present:

   ```
   python3 -c "import unitree_sdk2py"     # DDS state-bridge dependency imports
   ldconfig -p | grep -i livox            # liblivox installed
   ros2 pkg executables livox_ros_driver2 # Livox driver built
   ros2 pkg executables g1_bringup        # lists g1_state_bridge, cmd_vel_to_sonic, ...
   ```

   `g1_locomotion`'s AMO weights and `torch`/CUDA are only needed to RUN the AMO
   driver; the SONIC path never runs it, but `g1_locomotion` must still BUILD --
   `g1_state_bridge` imports its DDS transport (`G1DDSInterface`), so
   `--packages-up-to g1_bringup` pulls it in.

B. Network (full detail in section 2.3): the nav PC needs (a) a WIRED NIC on the
   robot segment `192.168.123.x` for the Unitree DDS state
   (`g1_state_bridge --net-if <iface>`) and the Livox Mid-360, and (b) TCP
   reachability FROM the Jetson to the nav PC on port 5556 (the Jetson connects
   out; open the port if a firewall is active).

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

### 2.3 Networking and safe remote access

Rule of thumb: **every robot DATA path stays on the wired LAN; wifi is only a
human SSH / remote-desktop path.**

What travels where:

- **Livox Mid-360** -> UDP on `192.168.123.x`. The nav PC must be on that segment.
- **Unitree DDS `rt/lowstate` / `rt/odommodestate`** -> domain-0 DDS on the same
  segment (state bridge + SONIC controller). Wired.
- **Nav -> SONIC velocity ticks** -> ZMQ from `cmd_vel_to_sonic` (nav PC, binds
  `:5556`) to the Jetson (connects out). Keep this on the wired LAN too.
- **Your SSH to the Jetson** -> control plane only; wifi is fine.

Dual-homed nav-PC laptop (recommended): plug ethernet (or a USB-ethernet adapter)
into the robot switch so the laptop gets a `192.168.123.x` address, and keep wifi
for internet. `--net-if` MUST name that WIRED interface (never the wifi one) -- it
is the NIC `g1_state_bridge` binds Unitree DDS on. Find it with `ip -br addr` (the
interface holding a `192.168.123.x` address).

**Exactly ONE IPv4 on the wired NIC (critical for Unitree DDS).** The wired NIC
must carry a single clean static `192.168.123.224/24` (the Livox host IP,
`MID360_config.json`) and NOTHING else -- no leftover DHCP/APIPA `169.254.x`, no
duplicate/`/0`-prefixed address. Unitree's CycloneDDS binds and announces ONE
locator per interface; with a second address present it can pick the wrong one and
then `g1_state_bridge` receives ZERO `rt/lowstate` / `rt/odommodestate` -- it logs
`Waiting for first lowstate...`, times out after 10 s, and publishes no `/imu/data`
or `/odom_raw` -- **even though `ping 192.168.123.161` is 0% loss and the robot's
RTPS multicast is arriving at the NIC.** This failure looks like a dead robot but
is purely a host addressing problem. Verify the NIC has only the one address
(`ip -br addr show <iface>`) before bringup; a network manager may re-add an APIPA
or a bad prefix, so re-check after any LAN reconfiguration.

Is reaching the Jetson over wifi safe? Yes, for SSH -- but understand what a wifi
drop does and does NOT do. The real-time locomotion loop (SONIC policy + the
patched writer <-> robot over `rt/lowcmd` / `rt/lowstate`) runs entirely on the
Jetson and the wired LAN; your wifi link is NOT in that loop, and the physical
E-stop is independent, so a wifi drop cannot make the robot fall or bolt
uncontrolled. But in the recommended wired-command topology a wifi/SSH drop also
does NOT stop the robot: SONIC keeps running (it is a DETACHED `tmux` session on
the Jetson) and the nav stack keeps commanding it over the WIRED ZMQ path, so the
robot continues executing its current Nav2 goal even though your SSH terminal is
gone. Losing the remote connection is not a safe auto-stop -- to stop, cancel the
goal or `Ctrl-C` the nav stack on the laptop (both local, unaffected by wifi), or
hit the physical E-stop. The watchdog-idle behavior (`cmd_vel_to_sonic` 0.3 s /
SONIC ~1 s -> IDLE, section 5) fires only when the COMMAND stream stops -- which is
exactly why that stream stays on the wired LAN and never on wifi: over wifi, every
stall would trip those watchdogs into a stuttering gait.

Quick reachability check (from the nav-PC laptop):

```
ping -c1 192.168.123.161     # the robot
ping -c1 <jetson-ip>         # the SONIC Jetson
ip -br addr                  # confirm a 192.168.123.x on the WIRED iface
# after the stack is up, confirm the Jetson connected to the bridge:
ss -tnp | grep 5556
```

SSH in to start SONIC: `ssh <user>@<jetson-host>` (reachable over the LAN or over
wifi once the laptop is dual-homed).

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

1. **Jetson -- controller (pinned):** start it with CPU pinning so its LowState
   DDS threads cannot be starved into a fall (section 4.5). Use the
   `start_sonic_pinned.sh` wrapper (copy it from this repo onto the Jetson, or run
   the inline equivalent below):

```bash
# wrapper: sets SONIC_CPU_*, wraps in `taskset -c 2-5`, waits for Init Done, and
# VERIFIES every thread (incl. DDS recvUC/dq.*) landed on the RT cores.
TMUX=1 SONIC_ZMQ_HOST=<nav-pc-ip> scripts/start_sonic_pinned.sh

# inline equivalent (if the wrapper is not on the Jetson):
sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"
pkill -9 -f "[g]1_deploy_onnx_ref" 2>/dev/null || true
tmux new-session -d -s rdeploy \
  "GR00T_DIR=\$HOME/GR00T-WholeBodyControl SONIC_ZMQ_HOST=<nav-pc-ip> \
   SONIC_CPU_MAIN=3 SONIC_CPU_INPUT=3 SONIC_CPU_PLANNER=3 SONIC_CPU_WRITER=4 SONIC_CPU_CONTROL=5 \
   taskset -c 2-5 scripts/start_deploy_real.sh > /tmp/sonic_deploy_real.log 2>&1"
until grep -qE "Init Done|out of memory" /tmp/sonic_deploy_real.log; do sleep 2; done
grep "\[RT\]" /tmp/sonic_deploy_real.log            # expect FIFO+pin for all 4 workers
ps -L -o tid,psr,comm -p "$(pgrep -f '[g]1_deploy_onnx_ref')"  # every PSR in 2-5
```

   It must not keep printing `LowState is not available` (robot state absent)
   and must not print `out of memory` (drop page caches per the sonic repo
   troubleshooting and relaunch). The `taskset -c 2-5` is the load-bearing part:
   `SONIC_CPU_*` alone leaves the controller's MAIN and CycloneDDS LowState threads
   floating on the general cores, where they can be starved -> fall (section 4.5).

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

### 4.5 CPU pinning -- prevent LowState starvation (falls)

The SONIC controller pins its four RT worker threads via `SONIC_CPU_*`, but its
MAIN thread and the CycloneDDS worker threads (`recvUC`, `recv`, `dq.builtins`,
`rlsnr`) that actually deliver `rt/lowstate` at ~500 Hz are **not** pinned by those
variables. If they float onto a busy core, a scheduling stall (measured p99 5.6 ms
/ max 15 ms under load, vs 72 us on a clean core) drops 2-7 LowState frames and the
controller trips:

```
[ERROR] Lost LowState data connection from robot!
[ERROR] Safety check failed, stopping control.
```

-> it stops writing `rt/lowcmd` -> **the robot falls.** This is CPU contention, not
a cable fault (`ping 192.168.123.161` stays 0% loss).

**The fix -- confine the WHOLE controller (RT loop + main + DDS/LowState threads) to
its own cores** with an outer `taskset`, which is exactly what
`scripts/start_sonic_pinned.sh` does (and then verifies). The outer
`taskset -c 2-5` is the load-bearing part; `SONIC_CPU_*` alone leaves the unpinned
threads floating. **Never start SONIC without it.**

anubi runs a **split topology** (the nav stack is on the nav PC, not the Jetson), so
the Jetson runs only `g1_deploy_onnx_ref` and there is no co-resident perception load
to fight -- but JetPack housekeeping and the DDS pool still share the general cores,
so the same rule holds: keep the controller on cores 2-5 and leave 0,1 for the OS.

`start_sonic_pinned.sh` also applies best-effort `jetson_clocks` (max clocks), which
keeps the RT cores off the 729 MHz idle step so the 500 Hz writer/control loops never
wait on a frequency ramp. It is a no-op if `jetson_clocks` is absent or sudo is denied.

**Fully on-board alternative (reference topology).** The confirmed-working reference
deploy (`/home/beltra/unitree`) runs nav AND SONIC on one Jetson Orin Nano with a
measured **3/3 core split**: perception+nav on cores 0,1,2 (`taskset -c 0,1,2`,
`OMP_NUM_THREADS=2` to cap DLIO's OpenMP) and SONIC on cores 3,4,5 (`SONIC_CORES=3-5`,
`SONIC_CPU_MAIN/INPUT/PLANNER=3 WRITER=4 CONTROL=5`). That split was measured strictly
better than 2/4 -- identical LowState health (~5 ms p50, 0 losses) while the nav side
dropped from ~94% to ~80% per core -- but it ONLY fits because the reference planner is
featherweight (CUDA voxel map + a pure-Python VFH/Pure-Pursuit follower). anubi's Nav2
stack (MPPI + STVL + EKF + slam_toolbox) is several times heavier and will not co-reside
with SONIC on 3 cores without swapping the planner; that is why anubi keeps the
nav-PC/Jetson split. See "Deployment topology" note below.

Verify on a running controller (every thread PSR should be within 2-5):

```bash
ps -L -o tid,psr,comm -p "$(pgrep -f '[g]1_deploy_onnx_ref')"
```

Optional hardening (needs a reboot): hard-isolate the controller cores so kernel
housekeeping stays off them, shaving rare tail-jitter. Add to the Jetson boot args
(`/boot/extlinux/extlinux.conf`): `isolcpus=2-5 nohz_full=2-5 rcu_nocbs=2-5`, reboot,
and confirm `cat /sys/devices/system/cpu/isolated` -> `2-5`. Pinning alone already
gave 0 LowState losses in the source deployment, so this is a "last 5%" layer.

## 5. Runtime behavior and tuning

- **Watchdog layering** (fastest first): `cmd_vel_to_sonic` `cmd_timeout: 0.3`
  (cmd_vel silence -> IDLE) -> SONIC planner ~1 s (tick loss -> idle) ->
  patched 500 Hz writer stale-hold (`SONIC_MOTOR_COMMAND_STALE_MS`). The
  upstream `velocity_limiter` watchdog (0.5 s) additionally zeroes
  `/cmd_vel_safe` if Nav2 goes silent.
- **Closed-loop heading:** SONIC has no yaw-rate input; it steers toward a
  world-frame `facing` direction. The bridge anchors that frame on the MEASURED
  odometry yaw (`odom_topic`, default `/odom`) rather than integrating `angular.z`
  open loop, so the body->world conversion does not drift over long runs and
  survives a bridge restart (it re-anchors on the current measured yaw instead of
  snapping back to a reset integral). Turning is injected as
  `face = dth + wz * facing_lookahead_sec`, so `facing_lookahead_sec` MUST be > 0
  or the robot never turns; it converges back to the measured heading as Nav2
  shrinks `wz`. If `/odom` is unavailable at startup the bridge holds SONIC's start
  pose (IDLE) until the first odometry message arrives.
- **Idle heading hold (drift vs real rotation):** while idle (no translation, no
  turn) the held facing is LATCHED, but the anchor is advanced by
  `_update_hold_anchor`: SLOW measured-yaw change (odom/DLIO drift, capped at
  `hold_drift_absorb_rad_s: 0.01` rad/s) is absorbed into the frame anchor so the
  torso does not rotate to chase it (most visible on a hoist), while anything FASTER
  (real rotation -- a hoist swing, balance steps) flows into the held facing so the
  bridge frame stays aligned with SONIC's internal world frame. The naive "re-anchor
  every tick" absorbs real rotation too, which skews the two frames and makes the NEXT
  facing command start its heading correction in the WRONG direction -- this is the
  wrong-way-turn fix from the confirmed-working reference.
- **Replan-storm guard (LowState safety):** SONIC replans its ~28 ms planner model on
  ANY change of movement/facing/speed, and the bridge recomputes those from measured
  yaw every tick, so yaw noise alone would drive a 30 Hz replan storm that saturates
  the planner core and starves the DDS LowState threads (past the 500 ms LowState-absent
  safety the robot collapses -- the same failure as unpinned threads, section 4.5). The
  SENT vectors are deadbanded against the last sent value (`movement_deadband_rad`,
  `facing_deadband_rad`, `speed_deadband`) so a steady walk emits a byte-identical
  vector (0 replans) while the 30 Hz message still flows as SONIC's input heartbeat.
- **Start-settle gate:** after the `start={1,planner=1}` handshake the bridge waits
  `start_settle_sec: 1.0` BEFORE subscribing to cmd_vel/odom, so no velocity can reach
  the controller until its soft-hold -> planner-driven control switch has settled. This
  is separate from the `--nav-delay` DLIO settle in the launch.
- **speed_gain:** 1.18 compensates the planner's ~0.85x speed realization and
  is applied AFTER the caps (caps are realized m/s). Set it to 1.0 to fall
  back to uncorrected upstream behavior; Nav2 still navigates, just slower
  than commanded.
- **Changing envelopes:** always change the Nav2 profile and the bridge caps
  together (they ship as matched file pairs; the launch selects both from
  `--sonic-profile`). Never widen the bridge caps alone.
- **Upper-body / carry poses:** the anubi `cmd_vel_to_sonic` bridge can hold a
  fixed upper-body pose while the legs walk under Nav2. Select it with
  `run_g1_real.sh --locomotion sonic --sonic-arm-preset carry` (or
  `sonic_arm_preset:=carry` on the launch; also `default`, or `none` = arms swing
  with the gait, the default). The bridge then appends the optional 17-DOF
  `upper_body_position` (+ zero velocity) to every planner tick, which the deploy
  binary uses to REPLACE the arm/wrist joints. The `carry` preset (arms forward,
  elbows bent ~70 deg) is copied from the sonic reference bridge; the joint values
  live in `UB_PRESETS` in `cmd_vel_to_sonic_node.py`. This is loco-manipulation:
  validate hoisted and tune the preset per payload before feet-down use (the node
  logs a warning while any non-`none` preset is active). For custom per-joint poses
  or presets not in `UB_PRESETS`, the sonic repo's own
  `cmd_vel_to_sonic.py --arm-preset ... --set joint=rad` can still be run ON the
  Jetson (controller pointed back at `localhost`, `SONIC_ZMQ_HOST` unset; it then
  subscribes to `/cmd_vel_safe` over ROS 2 DDS, so the Jetson needs ROS 2 Humble +
  the nav PC's `ROS_DOMAIN_ID`). See the sonic repo `docs/bringup.md` section 4 for
  pose tables and hoist-first validation rules.

## 6. Troubleshooting

| Symptom | Check / fix |
|---|---|
| Controller hangs at `LowState is not available` | Robot state absent: robot off/E-stopped, wrong `SONIC_DDS_IFACE`, or ethernet down. |
| Controller `Init Done` but robot ignores goals | Jetson cannot reach nav PC :5556 (firewall/route); check the bridge log shows the bind and `ss -tnp \| grep 5556` on the nav PC shows the Jetson connection. |
| Controller **dies mid-run** on `Lost LowState data connection` -> **robot falls** | LowState DDS thread was **starved**, not a cable fault (`ping 192.168.123.161` still 0% loss). The controller was started WITHOUT the outer `taskset` -- restart it with `start_sonic_pinned.sh` and verify every thread PSR is in 2-5 (section 4.5). |
| Robot walks ~15% slower than commanded | `speed_gain` left at 1.0 -- expected upstream behavior; set 1.18. |
| Robot will not turn | `facing_lookahead_sec` is 0 (the closed-loop `facing` then equals the measured heading every tick -- set it > 0, default 0.4). Also confirm the bridge is receiving `odom_topic` (`ros2 topic hz /odom`): with no measured yaw it holds the start pose. |
| Robot slowly rotates in place while idle | Stale build without the idle heading-hold, or `odom_topic` yaw is drifting hard: the bridge latches heading while idle, so confirm the bridge is the updated one and that `/odom` yaw is sane. |
| Gait stutters between commands | `cmd_timeout` below the actual cmd_vel period; chain publishes at 20 Hz, keep >= 0.3 s. |
| `CUDA initialization failure (out of memory)` on the Jetson | Unified-memory pressure: `sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"`, relaunch (sonic repo troubleshooting). |
| Periodic `soft-holding` / `MotorCommand stale` | State logging left on (`SONIC_ENABLE_LOGS`), or Jetson memory/disk pressure. Never enable logging on hardware. |
| Two controllers fighting / erratic motion | `amo_driver` or the stock sport service still alive alongside SONIC. Stop everything; enforce the single-writer rule. |
| `/odom_raw` silent under SONIC | `rt/odommodestate` availability with the sport service released is a hardware-validation item (same as AMO); if absent, run `--no-lio` is NOT the fix -- DLIO still provides odometry, but check the EKF config expectations in `docs/topic_map.md`. |
| `g1_state_bridge` dies at startup on `OSError: .../unitree_sdk2py/utils/lib/crc_amd64.so: cannot open shared object file` | The pip-from-git `unitree_sdk2py` did not ship the prebuilt CRC lib. Restore `unitree_sdk2py/utils/lib/crc_amd64.so` (section 2.2 A.2): copy the repo's `utils/lib/`, or compile the vendored `crc32_core` to that path. |
| `g1_state_bridge` connects then `Waiting for first lowstate...` -> 10 s timeout -> no `/imu/data` or `/odom_raw` (0 msgs) | Robot state not being received. First confirm the robot IS emitting: `tcpdump -ni <iface> host 192.168.123.161 and udp` should show RTPS to `239.255.0.1`. If packets arrive but the bridge sees nothing, the wired NIC has a second address (stale `169.254.x` or a `/0`-prefixed dup) and CycloneDDS bound the wrong locator -- reduce the NIC to a single clean `192.168.123.224/24` (section 2.3) and relaunch. |
| `livox_ros_driver2_node` aborts: `Could not load library ... liblivox_lidar_sdk_shared.so` | Livox-SDK2 shared lib not installed at RUNTIME (the driver `dlopen`s it). Build/install Livox-SDK2 and `ldconfig` (section 2.2 A.3); the `ldd` on the node will not show this dependency because it is dlopen'd. |

## 7. File map (what `--locomotion sonic` touches)

| File | Role |
|---|---|
| `src/platform/g1_bringup/scripts/cmd_vel_to_sonic_node.py` | Twist -> SONIC planner-tick ZMQ bridge (binds :5556; caps, yaw clamp, watchdog, speed_gain, idempotent start; closed-loop body->world via `odom_topic` yaw + `facing_lookahead_sec`; idle heading-hold; optional `arm_preset` fixed upper-body pose via `UB_PRESETS`) |
| `scripts/start_sonic_pinned.sh` | JETSON-side helper: start `g1_deploy_onnx_ref` with `SONIC_CPU_*` + outer `taskset` so the DDS LowState threads can't be starved into a fall; verifies thread confinement (section 4.5) |
| `src/platform/g1_bringup/config/g1_sonic_slow_walk_params.yaml` | Bridge profile: SLOW_WALK mode + slow_walk caps |
| `src/platform/g1_bringup/config/g1_sonic_walk_params.yaml` | Bridge profile: WALK mode + walk caps |
| `src/autonomy/robot_nav/config/nav2_params_g1_sonic_slow.yaml` | Nav2 envelope matching slow_walk |
| `src/autonomy/robot_nav/config/nav2_params_g1_sonic_walk.yaml` | Nav2 envelope matching walk |
| `src/platform/g1_bringup/launch/g1_bringup.launch.py` | `locomotion` / `sonic_profile` / `sonic_arm_preset` selection (AMO files untouched) |
| `src/simulation/g1_sim/launch/sim_bringup_g1.launch.py` | Same selection for the sim (envelope only) |
| `scripts/run_g1_real.sh` / `scripts/run_g1_sim.sh` | `--locomotion` / `--sonic-profile` wrapper options (`--sonic-arm-preset` is real-only) |

The AMO configuration (`g1_params.yaml`, `nav2_params_g1.yaml`, `cmd_vel_to_amo`,
`amo_driver`) is intentionally untouched by all of the above.

## 8. Day-of quickstart (repeat visit)

Use this ONLY after section 2 one-time setup is done on BOTH machines. It does NOT
replace the section 4.1 safety gates or the section 4.3 feet-down sign-off
progression -- it sequences them for a return visit.

0. **Power + safety.** Power the G1; put it on the hoist/gantry; verify the
   physical E-stop and keep it in hand (section 4 framing). Robot clear of people.

1. **Nav-PC laptop on the LAN.** Confirm it is dual-homed: `ping -c1
   192.168.123.161` (robot) and `ping -c1 <jetson-ip>` both succeed, and
   `ip -br addr` shows a `192.168.123.x` on the WIRED interface. Note that iface
   name -- it is your `--net-if` (section 2.3).

2. **Start SONIC on the Jetson (SSH).**

   ```
   ssh <user>@<jetson-host>
   # pinned start (section 4.5): confines all threads incl. DDS LowState to cores 2-5.
   TMUX=1 SONIC_ZMQ_HOST=192.168.123.224 scripts/start_sonic_pinned.sh
   # (the wrapper waits for Init Done, prints the [RT] lines, and verifies pinning)
   ```

   Require: `Init Done`, `[RT]` for all workers, the wrapper's "all threads confined
   to cores 2-5" OK line, NOT repeating `LowState is not available`, NOT `out of
   memory`. Enforce the single-writer rule: `pgrep -f amo_driver` empty, and the
   stock Unitree sport/high-level service released. If the wrapper is not on the
   Jetson, use the inline pinned command in section 4.2 step 1.

3. **Nav PC: sensors + odometry only (no commands yet).**

   ```
   scripts/run_g1_real.sh --no-nav --net-if enx00e04cd247cc
   ```

   Keep the robot still ~5 s (DLIO IMU/gravity calibration). Check against
   `docs/topic_map.md`: `/odom_raw`, `/imu/data`, `/lidar/points_undistorted`,
   `/lidar_odom`, `/odom`, and a complete TF chain.

4. **Nav PC: full stack with SONIC (robot still hoisted).**

   ```
   scripts/kill_ros_nodes.sh
   scripts/run_g1_real.sh --locomotion sonic --sonic-arm-preset default --net-if enx00e04cd247cc
   # known map instead of mapping:
   # scripts/run_g1_real.sh localization --map /ws/anubi/maps/<name>.posegraph \
   #   --locomotion sonic --net-if enx00e04cd247cc
   ```

   Confirm the Jetson connected to the bridge: `ss -tnp | grep 5556` on the nav
   PC shows the connection.

5. **Test -- follow section 4.3.** Hoisted short goals first, watchdog-idle check,
   sign-off, THEN feet down. Do not skip the sign-off on a first feet-down run or
   after any config change. Start on `slow_walk`; only move to `--sonic-profile
   walk` once slow_walk is validated (section 1).

6. **Shutdown (section 4.4).** Stop the Nav2 goal or `Ctrl-C` the nav stack (the
   bridge sends the SONIC stop on exit); then `pkill -9 -f g1_deploy_onnx_ref` on
   the Jetson. The physical E-stop is always the primary stop.
