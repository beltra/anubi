# G1 Nav2 Port - Honest Status and Remaining Work

Branch: `feat/g1` (off `origin/feat/sim2real`). Audience: a fresh session picking this up.
This is a candid handoff: what is verified working, what is broken, and the decisions
and tasks still open to reach "sim + real bringup fully working".

## TL;DR

The SIM perception + standing + Nav2 stack WORKS end to end (verified: full Nav2 stack
activates and the robot drives to a goal on the default ground-truth odometry). The
REAL-robot bringup (`g1_bringup`) has been REWIRED to mirror `go2_bringup` exactly,
with an EKF owning `odom -> base_footprint` (the user asked the G1 to share the Go2's
EKF). Remaining DLIO-in-sim degradation during the kinematic base's motion is a known
sim-only limitation (use_lio default is now false). The real path is correct-by-
construction and builds/parses/structurally-runs; its sensor/DDS bring-up needs the
real robot to fully validate (see "Real robot" below).

### Sim bring-up gotcha discovered this session (IMPORTANT)

The container's PID 1 does not reap orphaned processes, so every `ros2 launch` that is
killed leaves DEFUNCT (zombie) nodes. After a few hundred accumulate, FastDDS discovery
and Nav2 lifecycle bring-up start TIMING OUT (`collision_monitor` stuck inactive,
`ros2 lifecycle get` returns "Node not found" for a live node, `change_state` service
response timeouts). `kill_ros_nodes.sh` CANNOT reap zombies. The fix is to RESTART THE
CONTAINER (`docker restart anubi_g1`), which reaps them all; nav then activates cleanly
and the robot navigates. The kill script now warns when zombies exceed 50.

## Environment / how to run (sim)

- Docker container `anubi_g1`, image `anubi-g1-sim:humble`, workspace mounted at `/ws/anubi`.
- Host has NO native ROS; everything runs in the container. GPU is an NVIDIA L40S
  (gpu_lidar renders via EGL). Display is NoMachine `:1001` (software GL / llvmpipe).
- ALWAYS clean up before relaunch: `bash scripts/kill_ros_nodes.sh`
  (it now also reaps `/dev/shm/fastrtps_*` - see SHM note below).
- Launch (headless server, reliable spawn):
  `ros2 launch g1_sim sim_bringup_g1.launch.py gui:=false use_rviz:=false nav_delay_sec:=16.0`
  then a separate `rviz2 -d <robot_nav>/rviz/nav2.rviz --ros-args -p use_sim_time:=true`.
- `gui:=true` makes the Gazebo `create` spawn time out (server starved rendering the
  meshes on software GL). Keep the server headless. The Gazebo GUI client
  (`ign gazebo -g`) does NOT render on `:1001` (ogre and ogre2 both fail
  "egl: failed to create dri2 screen"); use RViz for visualization.
- Build closure: `colcon build --symlink-install --packages-up-to g1_sim robot_nav_recovery robot_lio`
  (skip `go2_description`: pre-existing broken rviz install on sim2real, not our fault).

### CRITICAL: /dev/shm exhaustion (was the cause of flaky Nav2 bringup)

`/dev/shm` in the container is only 64 MB (Docker default). FastDDS leaves shared-memory
segments after `kill -9`; across many restarts they accumulated (saw 177 stale files)
until SHM transport could no longer allocate a buffer for the large LiDAR PointCloud2
(`RTPS_TRANSPORT_SHM ... open_and_lock_file failed`). DDS then fell back to slow UDP and
Nav2 lifecycle activations timed out intermittently (controller/collision_monitor stuck
inactive). FIX applied: `kill_ros_nodes.sh` now `rm -f /dev/shm/fastrtps_*`. With a clean
pool the full stack activates ("Managed nodes are active"). CycloneDDS is NOT installed
(offline) - consider installing it for big-cloud robustness.

## VERIFIED WORKING (sim)

1. Standing body. `g1_body_standing.urdf` is ONE rigid link (`base_link`) with the official
   G1 meshes FK-posed at the AMO standing stance. It holds z=0.80 idle AND during motion,
   no fall/tip/sink. The recipe (in `sim_g1.launch.py` spawn-SDF generation):
   - gravity ON + a single footprint box `<collision>` (0.40x0.50x0.90, sole at base
     z=-0.80 = floor at the 0.80 m spawn height) so VelocityControl (a vehicle-style
     planar controller that does NOT hold z) has ground contact for z.
   - LOW center of mass (`g1_body_standing.urdf` inertial origin at base z=-0.6, world
     ~0.20 m) so the top-heavy body cannot tip under VelocityControl accelerations.
   - Things that did NOT work and why: no-collision ghost -> free-falls (VC needs contact
     for z); `<kinematic>true>` -> breaks VC motion; `<gravity>false>` -> coasts on
     residual / contact tunnels during motion; high CoM + collision -> tips during motion.
2. Realistic perception. Mid-360 stays at the EXACT real mount (pelvis->mid360
   xyz=(-0.00368,0.00003,0.46018) rpy=(0,0.04014,0), inside the head). The gpu_lidar sees
   the room (horizontal r~11.6 m) and the body realistically occludes the downward beams;
   only the head/sensor-pod mesh is hidden from the lidar (the real Mid-360 does not detect
   its own housing). Mechanism: `<visibility_mask>4294967293` on the sensor (survives
   URDF->SDF) + `<visibility_flags>2` injected onto ONLY the head_link visual at spawn
   (URDF->SDF drops per-visual flags, so `sim_g1.launch.py` generates the SDF via
   `ign sdf -p` and injects it). Fortress gpu_lidar DOES honor this - earlier failures
   were a stale 2nd `ign gazebo` server, not the mechanism.
3. Self-filter ON in sim (mirrors real). `distortion_corrector` removes the body box
   (base frame x[-0.18,0.25] y[-0.26,0.26] z[-0.90,0.55], min_radius 0.20); filtered cloud
   is clean from ~0.45 m. Use the SAME box on the real robot in `g1_bringup`.
4. /scan. `pointcloud_to_laserscan` -> near-full ring. NEW G1-specific config
   `robot_nav/config/scan_from_cloud_g1.yaml` (separate from the Go2's): TALL band
   knees..head (lidar_stabilized z -0.95..+0.20, world ~0.31..1.46 m), floor (z=-1.26)
   excluded. Wired into `sim_bringup_g1.launch.py` slam include.
5. Nav2 stack activates reliably (after the SHM fix) and the robot drives to goals when
   using the GROUND-TRUTH odometry (use_lio:=false): verified reaching forward goals.
6. DLIO runs: `/lidar_odom` at 100 Hz, no crash. Tracks ground truth within ~0.1 m AFTER
   setting `pointcloud/deskew: false` (the Gazebo cloud has no per-point timestamps, so
   deskew corrupted the moving cloud - this was the first "wrong when moving" cause). IMU
   placement/extrinsics VERIFIED correct (at rest lidar IMU accel ~(-0.38,0,9.82) = gravity
   rotated by the 0.04 mount pitch; DLIO yaw matches GT yaw).

## BROKEN / OPEN (sim)

1. DLIO-PRIMARY navigation still fails in motion. With use_lio:=true the EKF is driven by
   DLIO only (ground-truth /odom_raw dropped). Symptoms: robot drives the wrong direction
   and the planner aborts with "Starting point in lethal space" (the robot's own cell is
   marked lethal). Best current hypothesis (NOT the IMU mount - that is verified fine):
   the KINEMATIC VelocityControl base produces unrealistic IMU during motion (velocity is
   set directly, so velocity steps become acceleration spikes), which DLIO's inertial
   prediction cannot handle -> DLIO jumps during motion -> obstacles get marked at wrong
   positions / smear onto the robot footprint -> lethal. This is the inherent tension: the
   one intentionally-unrealistic thing (movement) corrupts DLIO's IMU input. The talos
   reference avoids it by running the real AMO gait (realistic IMU) or pinning the robot,
   NOT a VelocityControl base.
2. Gazebo GUI not viewable on `:1001` (software GL). RViz only.

## DECISION NEEDED: DLIO in sim (pick one next session)

- A. Ground-truth odom in sim (use_lio:=false). Works, navigates accurately. DLIO validated
     on the real robot only. Simplest reliable demo. (Recommended for a working sim demo.)
- B. Keep DLIO but make the kinematic IMU smooth: hard-limit velocity_smoother acceleration
     so VC velocity (hence IMU accel) is smooth, and/or down-weight IMU / raise DLIO process
     noise. Then re-test the lethal-space issue (also add costmap footprint clearing).
- C. Fuse DLIO + ground-truth in the EKF (the real g1_bringup also fuses body odom + DLIO).
     EKF smooths DLIO jumps; ground truth dominates so it is robust but DLIO is not really
     "the" odometry.
- D. Drive the sim with the real AMO gait instead of the kinematic base (realistic IMU).
     Most realistic, but AMO-in-sim was explicitly deferred as hard.

NOTE (RESOLVED): decision A was taken -- `sim_bringup_g1.launch.py` and `run_g1_sim.sh`
now default `use_lio:=false` (Gazebo ground-truth odom + EKF, the reliable demo path);
DLIO in sim is opt-in via `--use-lio`. `maybe_create_ekf` in `sim_g1.launch.py` drops
/odom_raw only when use_lio:=true.

## REMAINING WORK to "sim + real fully working"

Sim:
- [ ] Resolve the DLIO decision above and get reliable multi-goal navigation.
- [ ] (if keeping DLIO) fix lethal-space via IMU/velocity smoothing + costmap clearing/inflation.

Done this session (after the status doc was first written):
- [x] `scripts/run_g1_real.sh` - wraps g1_bringup with its current arg set (use_livox/
      use_dlio/use_nav/use_stvl/use_rviz/livox_config/standing_height). bash -n + --help OK.
      Full wrapper parity (--map/--nav-profile/--no-safety/key masks) comes with the
      g1_bringup rewire below.
- [x] `docs/g1_embodiment.md` - how the G1 is embodied (model, Mid-360, sim ghost +
      standing mechanics, real AMO path, sim/real parity table).
- [x] ASCII sweep of all new/edited files (em-dashes + box-drawing chars stripped from the
      launch files + dlio config; no `--` inside XML comments; xacro still parses).
- [x] sim default reverted to use_lio:=false (working ground-truth) so `ros2 launch` and
      `run_g1_sim.sh` agree; DLIO is opt-in via --use-lio.

Real robot - REWIRED to mirror go2_bringup (EKF, like the user asked):
- [x] `platform/g1_bringup/launch/g1_bringup.launch.py` rewritten to be structurally
      identical to `go2_bringup.launch.py`. Wiring: RSP(g1.urdf.xacro, now rooted at
      base_link) + `g1_state_bridge` (DDS rt/odommodestate + rt/lowstate -> /odom_raw +
      /imu/data + base_footprint->base_link TF) + EKF (`ekf_fusion_params(_lio).yaml`,
      owns odom->base_footprint) + lidar_level_frame_publisher + Livox driver +
      imu_rescale + robot_lio (canonical /livox->/lidar relays + DLIO `dlio_params_g1_real.yaml`,
      deskew ON, baselink2imu R_x(180), approximateGravity) + lidar_odom_relay + slam
      (distortion_corrector `distortion_corrector_g1.yaml` self-filter + gravity_align +
      p2l + slam_toolbox) + Nav2 (nav2_params_g1 + collision_monitor_params_g1) +
      velocity_limiter + cmd_vel_to_amo. Builds + parses + structurally runs (all
      non-hardware nodes activate). `/g1/points` is gone; canonical /lidar/points(_undistorted).
- [x] New: `g1_state_bridge` node (reuses g1_locomotion's G1DDSInterface read-only),
      `dlio_params_g1_real.yaml`, `distortion_corrector_g1.yaml`. `g1.urdf.xacro` rooted at
      base_link (base_footprint + lidar_stabilized owned by the bringup, like go2.urdf.xacro).
- [x] `run_g1_real.sh` full parity (mapping/localization, --map, --no-lio/-ekf/-safety/
      -state-bridge/-nav/-stvl/-livox, --net-if, --nav-delay, key masks).
- [x] `kill_ros_nodes.sh` gains the g1 real patterns + the zombie warning.
- [x] Docs: g1_embodiment.md (real EKF path), topic_map.md, launch_script_parity.md updated.

Needs the real robot to finish validating (correctly-declared deps absent in the sim container):
- [ ] HW smoke: `livox_ros_driver2` needs the Livox SDK (so g1_bringup only fully builds on
      the real image), `topic_tools` (the /livox->/lidar relays), and `unitree_sdk2py` + the
      robot (g1_state_bridge DDS). Confirm rt/odommodestate is published under AMO low-level
      control; if not, the EKF still runs on DLIO alone (use_lio default true).
- [ ] Calibrate on HW: standing_height (0.78), the DLIO Mid-360 extrinsics (t + 0.04 pitch),
      and the g1_state_bridge body-odom frame/sign conventions against measured TF.
- [ ] Verify the AMO driver loop (g1_locomotion amo_driver WS :8766, cmd_vel_to_amo).

## Deployment-readiness audit fixes (applied)

A full real-robot readiness audit was run and the following fixes applied (code-complete;
the locomotion-safety items still need HARDWARE validation):

- Velocity caps reconciled to the AMO in-distribution range `vx 0.5 / vy 0.4 / yaw 0.4`
  across `amo_driver` (CAP_*), `cmd_vel_to_amo`, and `g1_params.yaml` (were 0.6/0.3 --
  vx 0.6 was out-of-distribution). Now consistent with `nav2_params_g1`.
- AMO driver safety (HW-validate): command-staleness deadman zeros velocity if the
  cmd_vel_to_amo WS stream stops; soft damping pose (kp=0, kd=KD) on shutdown AND on
  tilt/stale-state abort (new `robot_dds_io.request_damping`); CUDA pre-flight assert in
  `run_control` before `start_publisher` (CPU would crash mid-loop after motor streaming).
- `mid360_link -> lidar_link` identity alias TF in `g1_bringup` so the Livox cloud
  `frame_id` resolves in TF (RViz / parity with sim); no nav consumer depended on it.
- Scan accumulation DISABLED on the real G1 (`accumulate_scan_cloud:=false`), matching
  the sim (single Mid-360 frames). Re-enable if `/scan` is too sparse on hardware.
- Sim now loads the SHARED `distortion_corrector_g1.yaml` (self-filter box / sensor
  geometry single-sourced) with only the sim-only leveling flags overridden inline.
- `collision_monitor_params_g1.yaml` `use_sim_time:false` (was true; launch overrides it,
  but the file default was a foot-gun).
- Build/env: `docker/Dockerfile.real` added (UNVALIDATED starting point -- CUDA torch +
  Livox/Unitree SDKs); livox `build.sh humble` pre-step documented (the vendored package
  gitignores its `package.xml`, so colcon cannot discover it until generated).

Still open for HW: validate damp/deadman/CUDA-assert on the robot; calibrate
`standing_height` (0.78) + DLIO extrinsics + state-bridge sign conventions; validate
`Dockerfile.real` on the target arch (x86+dGPU vs Jetson arm64).

## Files created/edited this session

- `simulation/g1_sim/launch/sim_g1.launch.py` - spawn-SDF generation (head-visibility
  injection, footprint collision, low-CoM relies on urdf); EKF DLIO-primary when use_lio.
- `simulation/g1_sim/launch/sim_bringup_g1.launch.py` - G1 scan_params; use_lio default false.
- `simulation/g1_sim/urdf/g1_sim.urdf.xacro` - visibility_mask 4294967293; comments.
- `simulation/g1_sim/urdf/g1_body_standing.urdf` - inertial CoM to base z=-0.6; removed
  the (URDF-dropped) per-visual visibility_flags.
- `simulation/sensor_models/urdf/mid360_lidar.urdf.xacro` - 480x64 samples (was 360x40
  too sparse / 720x96 too heavy); visibility doc.
- `autonomy/robot_nav/config/scan_from_cloud_g1.yaml` - NEW tall G1 scan band.
- `autonomy/robot_lio/config/dlio_params_g1_sim.yaml` - deskew OFF (sim), voxel 0.18.
- `scripts/kill_ros_nodes.sh` - grep instead of rg (container-portable), /dev/shm cleanup,
  g1 launch patterns.
