# G1 Embodiment

How the Unitree G1 humanoid is embodied in this workspace: the robot model, the
head-mounted Livox Mid-360, the frame/topic contract, and the two runtimes (Gazebo
Fortress simulation and real hardware) that share that contract. The autonomy stack
(`robot_nav`, `robot_safety`, `robot_lio`) is robot-agnostic and is reused UNCHANGED;
only the G1-specific model, sensor placement, locomotion, and a handful of config
profiles differ from the Go2.

For current verified-vs-broken status and remaining work, see
`docs/g1_porting_status.md`. For the authoritative topic/TF contract see
`docs/topic_map.md`.

## 1. What "embodiment" means here

The G1 is embodied at three layers, all converging on the SAME canonical frame/topic
contract so the autonomy stack does not know or care whether it is driving a Go2, a
simulated G1, or a real G1:

1. Kinematics / frames - the G1 URDF (`g1_description`) and the standing pose.
2. Perception - the head-mounted Livox Mid-360 (cloud + IMU) and how its returns are
   conditioned (deskew, self-filter, 2D scan slice).
3. Locomotion - how Nav2 velocity commands become motion: the AMO walking policy on
   hardware, and a kinematic velocity-controlled base in simulation.

The one INTENTIONALLY unrealistic thing is locomotion in simulation: the AMO gait is
hard to simulate, so the sim drives a velocity-controlled rigid "ghost" base instead.
Everything in the perception path is meant to match hardware.

## 2. Frame contract

```
map --(SLAM, both)--> odom
odom --(EKF, both)--> base_footprint
base_footprint --(g1_state_bridge: body roll/pitch + standing height, real;
                  static ~0.80 m, sim)--> base_link  (== pelvis)
base_link --> lidar_link (Mid-360, ~0.46 m up, inside the head) , imu_link
base_footprint --(gravity-levelled, IMU-rate)--> lidar_stabilized   (2D scan frame)
```

- The EKF owns `odom -> base_footprint` in BOTH sim and real (same as the Go2). The
  deliberate difference is only the EKF's odometry INPUT: ground-truth body odom in
  sim (DLIO optional), DLIO + AMO body odometry on hardware. DLIO never publishes a
  TF; it feeds the EKF via `/lidar_odom -> lidar_odom_relay -> /lidar_odom_reliable`.
- `robot_base_frame` for Nav2 is `base_footprint`.
- The Mid-360 sits at the REAL mount on `base_link`/pelvis:
  `xyz = (-0.00368, 0.00003, 0.46018) m`, `rpy = (0, 0.04014, 0) rad` (a small forward
  pitch), i.e. inside the head. It is NOT moved for the sim.
- Sensor frame naming: the URDF sensor link is `mid360_link`. The SIM gpu_lidar and the
  REAL Livox driver both stamp their cloud `frame_id = lidar_link`; on the real robot
  `g1_bringup` publishes an identity `mid360_link -> lidar_link` alias so that cloud
  frame resolves in TF (the sim gets it from the URDF). No nav consumer relies on the
  cloud's TF frame (DLIO + `distortion_corrector` use explicit extrinsics, and the nav
  cloud is re-stamped to `lidar_stabilized`); the alias is for RViz / TF completeness.
- `lidar_stabilized` is a gravity-levelled frame co-located with the sensor, published
  by `robot_nav`'s `lidar_level_frame_publisher`. The 2D scan slice is taken in it.

## 3. The robot model

`g1_description` carries the official Unitree G1 URDF + meshes. The thin xacro wrapper
keeps `pelvis` as the root and adds `base_link`/`base_footprint` (ground projection at
the nominal standing height) plus `lidar_stabilized`, to satisfy the Nav2 frame
contract. The Mid-360 macro lives in `simulation/sensor_models/urdf/mid360_lidar.urdf.xacro`.

### Standing pose

The robot is posed in the AMO `default_dof_pos` standing stance. The leg/arm/waist joint
angles come from the AMO policy's nominal pose; the FK of that pose places the feet on
the floor at a pelvis height of ~0.80 m, and the Mid-360 at ~1.26 m above the floor.
(The SIM spawns at this 0.80 m; the REAL bringup defaults `standing_height:=0.78` and is
meant to be calibrated on hardware, so the real Mid-360 sits ~1.24 m up. The 2 cm delta
is a calibration item, not a contract difference.)

## 4. Simulation embodiment (Gazebo Fortress)

Package: `simulation/g1_sim`. Launch: `g1_sim/launch/sim_bringup_g1.launch.py`
(wrapped by `scripts/run_g1_sim.sh`). Mirrors `go2_sim` + `robot_sim` but with a
KINEMATIC base instead of CHAMP + leg physics.

### 4.1 Single rigid "ghost" body

`g1_sim/urdf/g1_body_standing.urdf` is generated offline: the whole 29-DoF robot is
collapsed into ONE rigid link `base_link` whose visuals are every body mesh
forward-kinematically posed at the standing stance. Being one link, it cannot fall apart
or sag. It carries no mesh collisions.

The Fortress `VelocityControl` plugin drives it directly from the final safe command:
ROS `/cmd_vel_safe` is bridged to Ignition `/model/g1/cmd_vel`. This is the analogue of
how `go2_sim` uses CHAMP - the velocity-controlled base stands in for the AMO gait.

### 4.2 Standing mechanics (why it does not fall)

`VelocityControl` is a vehicle-style PLANAR controller: it drives x / y / yaw but never
holds z. So the sim model is held up by physics contact, NOT a kinematic hack. The spawn
SDF (generated at launch by `sim_g1.launch.py`) injects onto `base_link`:

- one box `<collision>` sized to the Nav2 footprint, whose sole sits at base `z = -0.80`
  (the floor when `base_link` spawns at the 0.80 m standing height), so the body rests on
  the ground plane and `VelocityControl` slides it; gravity stays ON so the contact is
  stable during motion (it cannot tunnel);
- a LOW centre of mass (the standing URDF inertial origin is at base `z = -0.6`, world
  ~0.20 m) so the top-heavy body is bottom-heavy at the contact and cannot tip under
  command accelerations.

(Alternatives that FAILED, for the record: no collision -> free-falls; `<kinematic>true`
-> breaks `VelocityControl`; `<gravity>false` -> coasts on residual velocity / contact
tunnels; high CoM + collision -> tips during motion.)

### 4.3 Perception realism

Goal: the sim perception matches hardware as closely as possible.

- The gpu_lidar is at the REAL Mid-360 mount and SEES the body, so the torso, shoulders,
  arms and legs realistically OCCLUDE the downward beams. The horizontal beams clear the
  body and see the room (~11.6 m in the test world).
- Only the head/sensor-pod mesh is hidden from the lidar - the real Mid-360 emits through
  its own window and never returns its own housing. Mechanism: the sensor gets
  `<visibility_mask>4294967293` (this survives URDF->SDF conversion) and the spawn step
  injects `<visibility_flags>2` onto ONLY the `head_link` visual (per-visual flags are
  dropped by URDF->SDF, so `sim_g1.launch.py` runs `ign sdf -p` and regex-injects it).
- The resulting self-returns from the rest of the body are removed by the SAME self-filter
  used on hardware: `robot_nav`'s `distortion_corrector` with a G1 body box
  (base frame `x[-0.18,0.25] y[-0.26,0.26] z[-0.90,0.55]`, `min_radius 0.20`),
  producing `/lidar/points_undistorted`.
- The 2D scan uses a G1-specific `pointcloud_to_laserscan` config
  (`robot_nav/config/scan_from_cloud_g1.yaml`): a TALL band from ~knee height up to just
  above the sensor (separate from the Go2's thin slice, because the Mid-360 is ~1.26 m
  up), with the floor excluded.

### 4.4 Sim odometry

Two options (see `docs/g1_porting_status.md` for the open decision):

- Default / working: the Gazebo OdometryPublisher provides ground-truth body odom
  (`/odom_raw`), fused with the IMU by the EKF (`odom -> base_footprint`); `slam_toolbox`
  owns `map -> odom`.
- DLIO (`use_lio:=true`): `robot_lio` runs DLIO on the Mid-360 cloud + IMU as the
  odometry, matching hardware. NOTE: DLIO tracks well at rest but degrades during the
  KINEMATIC base's motion (velocity steps become IMU acceleration spikes), which is the
  open item - on hardware the AMO gait produces realistic IMU and this does not apply.

### 4.5 Bridges (Ignition <-> ROS)

`/clock`, `/imu/data`, `/lidar/imu`; `/livox/lidar/points -> /lidar/points` (frame
`lidar_link`); `/model/g1/odometry -> /odom_raw`; ROS `/cmd_vel_safe -> /model/g1/cmd_vel`.
The Ignition `/tf` is intentionally NOT bridged - EKF, static TFs and RSP own the ROS TF
tree.

## 5. Real-robot embodiment

Package: `platform/g1_bringup`. Launch: `g1_bringup/launch/g1_bringup.launch.py`
(wrapped by `scripts/run_g1_real.sh`). Structurally IDENTICAL to `go2_bringup`: the
robot-agnostic autonomy (`robot_nav`, `robot_safety`, `robot_lio`) is reused unchanged,
and only the robot-specific adapters differ. Where the Go2 uses `go2_hw_bridge` (Sport
API), the G1 splits that role: `g1_state_bridge` (DDS state -> ROS) for the STATE half
and `cmd_vel_to_amo` (velocity -> AMO WebSocket) for the COMMAND half.

- `robot_state_publisher` (G1 URDF, rooted at `base_link`) + `joint_state_publisher`.
- `g1_state_bridge`: reads the Unitree DDS state (`rt/odommodestate` body odometry +
  `rt/lowstate` pelvis IMU, reusing `g1_locomotion`'s DDS transport read-only) and
  publishes `/odom_raw`, `/imu/data`, and the `base_footprint -> base_link` TF (body
  roll/pitch + standing height). This is the G1 analogue of `go2_hw_bridge`'s state half.
- EKF (`robot_localization`): fuses `/odom_raw` (+ DLIO `/lidar_odom_reliable`) into a
  planar `odom -> base_footprint`, using the SAME robot-agnostic config the Go2 uses
  (`ekf_fusion_params.yaml` / `ekf_fusion_params_lio.yaml`). This is the EKF the G1
  shares with the Go2.
- Livox Mid-360 driver (`livox_ros_driver2`) in PointCloud2 mode -> `/livox/lidar`
  (per-point timestamps so DLIO deskews) + `/livox/imu` (accel in g). The Mid-360 is
  physically inverted on the head; `MID360_config.json` applies `roll:180` so the
  PUBLISHED cloud is upright. `imu_rescale` converts the Livox IMU to m/s^2.
- `robot_lio` relays `/livox/lidar -> /lidar/points` and `/livox/imu_ms2 -> /lidar/imu`,
  and runs DLIO. On hardware DLIO's IMU is realistic so deskew is ON and the inverted
  IMU is corrected with `baselink2imu = R_x(180)` (`dlio_params_g1_real.yaml`). DLIO
  publishes `/lidar_odom` (NOT a TF); the EKF is the single owner of `odom -> base_footprint`.
- The same `distortion_corrector` self-filter (`distortion_corrector_g1.yaml`, with
  gravity_align ON for the walking robot), `lidar_level_frame_publisher`,
  `pointcloud_to_laserscan` (G1 scan config), and Nav2 G1 profiles as the sim.
- `robot_safety` `velocity_limiter` gates `/cmd_vel_collision_free -> /cmd_vel_safe`.

Real-robot bring-up validation needs hardware: the Livox SDK (`livox_ros_driver2`),
`topic_tools` (the `/livox -> /lidar` relays), and `unitree_sdk2py` + the robot
(`g1_state_bridge` DDS). All are correctly declared package dependencies. Keep the
robot STILL for the first few seconds so DLIO completes its IMU/gravity calibration
(see `run_g1_real.sh --nav-delay`, default 5 s).

### 5.1 Locomotion: the AMO bridge

Nav2 produces a body-frame velocity. The chain is:

```
Nav2 /cmd_vel -> collision_monitor /cmd_vel_collision_free
              -> velocity_limiter   /cmd_vel_safe
              -> cmd_vel_to_amo      ws://127.0.0.1:8766  {vx, vy, yaw}
              -> AMO driver (g1_locomotion amo_driver, SEPARATE process) -> robot
```

`g1_locomotion` is the vendored, self-contained AMO walking policy (assets copied out of
Psi0; anubi does not depend on Psi0 at runtime). It runs as its own process, exposes a
WebSocket server on :8766, and turns `{vx, vy, yaw}` into a 29-DoF leg/waist gait at
250 Hz (arms held at the default pose). Velocity caps: `vx <= 0.5`, `vy <= 0.4`,
`yaw <= 0.4 rad/s` -- the AMO policy's in-distribution range, enforced identically by
Nav2 (`nav2_params_g1`), `cmd_vel_to_amo`, and the AMO driver's final clamp. (Nav2's
MPPI is `DiffDrive`, so it turns-then-walks and does not command `vy`; the lateral cap
is kept for the policy's strafe capability / Omni readiness.) This is what makes the G1
a holonomic-ish mobile base from Nav2's point of view, without Nav2 knowing anything
about legs.

### 5.2 Locomotion alternative: the SONIC whole-body controller

With `locomotion:=sonic` (`run_g1_real.sh --locomotion sonic`), the last hop is replaced
by `cmd_vel_to_sonic`, which turns the Twist into SONIC planner ticks (world-frame
movement/facing direction + speed) over a ZMQ PUB socket (:5556 on the nav host). The
SONIC controller (`g1_deploy_onnx_ref` from GR00T-WholeBodyControl, deployed on a Jetson
Orin Nano via the sonic-g1-locomanipulation repo) connects to that socket and writes
`rt/lowcmd` at 50 Hz. Everything upstream of the bridge -- and the whole STATE side
(`g1_state_bridge`, DLIO, EKF) -- is identical to the AMO variant. Two matched
config pairs (`sonic_profile`): `slow_walk` (SLOW_WALK mode, AMO-equal envelope,
first-bringup default) and `walk` (WALK mode, vx 0.8 / wz 0.5). The AMO configs are
never modified by the sonic path. NEVER run `amo_driver` and `g1_deploy_onnx_ref`
simultaneously (single writer on `rt/lowcmd`). Full first-setup + bringup procedure:
`docs/g1_sonic_locomotion.md`.

## 6. Sim/real parity summary

| Aspect                | Simulation                              | Real robot                          |
|-----------------------|-----------------------------------------|-------------------------------------|
| Body                  | 1 rigid ghost link, footprint collision | Articulated G1, AMO or SONIC gait   |
| Motion from /cmd_vel  | VelocityControl kinematic base          | AMO policy (ws :8766) or SONIC (ZMQ :5556) |
| Mid-360 mount         | Real pose, gpu_lidar (sees body)        | Real pose, Livox driver             |
| Self-returns          | distortion_corrector self-filter        | distortion_corrector self-filter    |
| 2D scan               | scan_from_cloud_g1.yaml                  | scan_from_cloud_g1.yaml             |
| odom->base_footprint  | EKF                                     | EKF                                 |
| EKF odometry input    | ground-truth /odom_raw (DLIO optional)  | DLIO + AMO body odom /odom_raw      |
| base_footprint->base_link | static (no tilt)                    | g1_state_bridge (body roll/pitch)   |
| Nav2 profiles         | nav2_params_g1[_sonic_*] + collision_monitor_g1 | nav2_params_g1[_sonic_*] + collision_monitor_g1 |

The deliberate difference is the body/motion row and the EKF's odometry INPUT (no
ground truth on hardware); the EKF ownership, perception, and Nav2 rows are identical
by design.
