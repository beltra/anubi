# Unitree A2-W (Pro) embodiment plan

Status: plan, under review. Sim is the first target; every interface is chosen so the
real robot needs a bridge + calibration, not a redesign.

## 1. What the robot is (facts and confidence)

| Item | Value | Confidence / source |
| --- | --- | --- |
| Body | A2 quadruped body, 12 leg joints (hip/thigh/calf) + 4 wheel motors at the calf tips | High for legs (official `unitree_ros/robots/a2_description/urdf/a2.urdf`); wheel layout inferred from Go2-W / B2-W URDFs |
| Leg geometry | hip offset x +-0.259, y +-0.075; thigh offset 0.128; thigh 0.275; calf 0.275 | High (official URDF) |
| Mass | ~40 kg (URDF link sum), retailer 37-42 kg | Medium |
| Size | 820 x 440 x 570 mm standing | Medium (retailer) |
| Wheel | ~190 mm tyre (r ~0.095 m), fixed axle along y, no steering | Low (retailer) / inferred |
| LiDARs (Pro) | 2, front and rear. Official URDF frames: `front_lidar_link` at (0.338, 0, 0.081) with optical z axis pointing +x; `rear_lidar_link` at (-0.280, 0, 0.087) with optical z axis pointing -x | High for frames; LiDAR model NOT documented (hemispherical, 360 x ~90 deg per the frame layout and a vendor blog) |
| Control API | `unitree::robot::a2::SportClient` on DDS service `sport`; `Move(vx, vy, vyaw)` = api_id 1008 (same as Go2/B2), state on `rt/lf/sportmodestate` (`unitree_go` SportModeState IDL) | High (unitree_sdk2 `include/unitree/robot/a2/sport`, `example/a2`) |
| Wheeled variants | Go2-W and B2-W examples reuse the legged SportClient; the B2-W example commands `Move(0, 0.3, 0)` | High that the API is the same; lateral execution mechanism unknown |
| Locomotion policy | No public A2-W RL policy or Gazebo controller exists | High |

Consequence: on hardware the onboard sport controller turns a body twist into wheel/leg
motion. The stack never drives wheels directly, so the sim should also take a body twist.

## 2. Sim approach

No CHAMP, no leg physics. Same pattern as the G1 sim (`g1_sim`), which is already proven
in this repo:

- One rigid body: A2 meshes with legs frozen in the standing pose, plus 4 wheel visuals at
  the calf tips. A box collision sized to the footprint rests on the floor.
- Fortress `VelocityControl` slides the body from `/cmd_vel_safe` (ROS) ->
  `/model/<robot>/cmd_vel` (ign). This is the same body-twist contract as `SportClient::Move`.
- `OdometryPublisher` ground truth -> `/odom_raw` -> EKF -> `/odom` + `odom->base_footprint`
  (EKF stays the TF owner, as for Go2/G1). On hardware `/odom_raw` comes from `lf/sportmodestate`.
- Two `gpu_lidar` sensors at the official URDF frames, hemispherical 360 x 90 deg, 10 Hz.
  Sampling is reduced on CPU-only hosts (launch default 64 channels x 180 azimuth per
  sensor; real JT128 is 128 x 900). Keep the channel count high: with the dome axes
  horizontal, the vertical channel spacing sets the angular resolution of `/scan`.
- Physics step 0.004 s (`physics_step_sec`): the 1 ms default ran /clock at ~1 kHz and
  starved Nav2 on a 4-core host.
  Ign topics `/a2w/front_lidar/points`, `/a2w/rear_lidar/points` -> ROS
  `/lidar/front/points`, `/lidar/rear/points` (frames `front_lidar_link`, `rear_lidar_link`).
- Body IMU `/imu/data` (frame `imu_link`).

## 3. Two LiDARs into a single-cloud stack

Every perception consumer (distortion_corrector, pointcloud_to_laserscan, STVL,
collision_monitor, slam_toolbox via `/scan`) assumes one cloud. Add one robot-agnostic node
in `robot_nav`:

- `cloud_merger` (C++): subscribes N clouds, transforms each into a fixed output frame via
  TF (static URDF extrinsics), concatenates xyz(+intensity), publishes `/lidar/points` at the
  rate of the primary input. A stale secondary cloud (> `max_input_age_sec`) is dropped, not
  waited for, so losing the rear LiDAR degrades coverage instead of stopping perception.
- Output frame `lidar_link`: a virtual frame fixed to `base_link` with identity rotation at
  the midpoint height of the two sensors. With identity rotation, `distortion_corrector` needs
  `sensor_mount_rpy: [0,0,0]` and the virtual sensor position, so it is reused unchanged.
- The same node runs on the real robot against the driver topics, so the merge path is
  identical in sim and on hardware.

Downstream stays canonical: `/lidar/points -> /lidar/points_undistorted -> /scan`.

## 4. Navigation

- SLAM / localization: 2D `slam_toolbox` on `/scan` built from the merged cloud
  (mapping and localization modes), same as Go2/G1; slam_toolbox owns `map->odom`.
- Odometry: DLIO on the merged cloud + body IMU feeds the EKF (`use_lio:=true`, default on
  sim and real); `use_lio:=false` falls back to ground truth (sim) or Sport Mode odom (real).
- Nav2: new `nav2_params_a2w.yaml` (Smac2D + MPPI), footprint x -0.52..0.42, y +/-0.29 m (rear knees reach x -0.457),
  `collision_monitor_params_a2w.yaml`, `scan_from_cloud_a2w.yaml`,
  `distortion_corrector_a2w.yaml`, `slam_toolbox_params_a2w.yaml`.
- Command chain unchanged: Nav2 `/cmd_vel` -> collision_monitor -> `/cmd_vel_collision_free`
  -> velocity_limiter -> `/cmd_vel_safe` -> robot.

## 5. Packages and scripts

- `src/platform/a2w_description`: A2 meshes + URDF (BSD-3 from unitree_ros) and a wheeled
  xacro wrapper (frozen standing legs, wheels, lidar frames, imu).
- `src/simulation/a2w_sim`: sim xacro (plugins + sensors), `sim_a2w.launch.py`,
  `sim_bringup_a2w.launch.py`.
- `src/platform/a2w_bringup` (real, untested scaffold): reuse `go2_hw_bridge` with A2 topics
  (`/api/sport/request`, `/lf/sportmodestate`), cloud_merger on the driver topics, same
  safety chain. Lidar driver topics left as launch args until verified on the robot.
- `scripts/run_a2w_sim.sh` (+ `run_a2w_real.sh`), `kill_ros_nodes.sh`, `docs/topic_map.md`,
  `docs/launch_script_parity.md` updated in the same change.

## 6. Validation (in this container, headless)

1. Model: xacro -> SDF, spawn, robot rests on the floor, does not tip under step commands.
2. Sensors: both clouds publish at ~10 Hz; merged cloud contains both hemispheres; TF
   front/rear points line up on a wall seen by both.
3. Motion: commanded vs ground-truth twist (vx, vyaw) within tolerance.
4. Mapping: drive a loop in `indoor_office`, save the map image, check walls are single lines.
5. Navigation: scripted A->B goals in mapping and localization mode; record success,
   time, min obstacle distance, collision_monitor activations.
6. Measure real-time factor with both gpu_lidars + Nav2 on 4 CPUs.

## 7. Open decisions

Tracked in the project thread; defaults are marked there and applied until answered.

## Status (2026-10-14)

Done and validated in sim (RTX laptop, RoboStack env, Gazebo GUI on, RTF ~1.0):
- Model and sensors: lumped sliding base tracks `/cmd_vel_safe` exactly, z stable at 0.497 m.
- `cloud_merger` fuses both JT128 clouds into `/lidar/points` (~0.1 s latency end to end).
- slam_toolbox builds a clean map of `indoor_office` from the merged cloud.
- Nav2 goal suite `6,0,0  6,-3,-1.57  0,3,1.57  -7,0,3.14  0,0,0` (mapping mode, ~33 m per
  lap, two laps per odometry source, clean ROS graph):

  | Odometry | Success | Final xy error vs goal (ground truth) | Odom drift vs ground truth |
  | --- | --- | --- | --- |
  | ground truth (`--no-lio`) | 10/10 | 0.04 - 0.24 m | < 0.01 m |
  | DLIO (default) | 10/10 | 0.06 - 0.27 m | max 0.054 m, max 2.1 deg yaw after ~70 m |

  Each lap takes ~55 s wall. xy error is measured on ground truth; the goal checker uses the
  SLAM estimate, so a few results sit just above the 0.25 m tolerance.
- Real-robot scaffold `a2w_bringup` + `run_a2w_real.sh`: launch wiring checked without
  hardware (nodes start, frame aliases and merger inputs resolve). Not run on the robot.

Root causes fixed:
1. MPPI reverse driving was not a tuning problem. The RoboStack
   `ros-humble-nav2-mppi-controller` 1.1.17 binary (build `_7`) computes wrong controls: in an
   isolated controller_server with a straight path ahead and only PathFollowCritic enabled it
   commands vx ~-0.23 m/s; the same YAML on MPPI 1.1.17 built from source gives +0.86 m/s.
   The env pairs xtensor 0.23.9 with xsimd 14, which cannot even compile the upstream SIMD
   path. Fix: `scripts/build_mppi_conda_overlay.sh` builds MPPI without xsimd into an overlay
   and sources it from a conda activate.d hook. Every robot profile uses MPPI DiffDrive, so
   Go2/G1 sims on RoboStack were affected too. Docker/apt Humble is not affected.
2. DLIO received no clouds: `cloud_merger` published BEST_EFFORT and DLIO subscribes RELIABLE.
   The merger now publishes RELIABLE. This applied to the real path too.
3. `kill_ros_nodes.sh` did not stop Nav2 servers, bridges or robot_state_publisher under
   RoboStack (it matched `/opt/ros/`), so relaunches stacked zombie Nav2 graphs. It now also
   matches `$CONDA_PREFIX/lib/<pkg>/<exe>` and `ros2 launch`.

Behaviour notes:
- With `vx_min: -0.3`, MPPI backs up instead of turning around when the goal is behind the
  robot and has the same heading (e.g. 3 m straight back). Reverse samples are 10-20 % of
  commands on goals that require a heading change. Raise PreferForwardCritic or set
  `vx_min: 0.0` if the robot should never back up more than a few decimetres.
- The 20 Hz MPPI loop missed its rate a few times per lap (batch 2000, scalar build).

Open issues, in order:
1. Visual check of the sim on the PC (`./scripts/run_a2w_sim.sh --gui --no-lio`).
2. Localization mode (`localization --map ...`) goal suite; not run yet.
3. On the robot: JT128 driver topics/frame_id and their alignment with the URDF lidar
   frames, `/lf/sportmodestate` presence, Move acceptance in wheeled mode, remote key masks.
   Check which MPPI binary the robot computer runs (apt is fine, RoboStack needs the overlay).

PC setup (Gabriele's laptop): worktree `/home/beltra/TalOS/anubi-a2w`, micromamba env
`anubi_humble` (`export MAMBA_ROOT_PREFIX=$HOME/micromamba; eval "$(~/micromamba-bin/bin/micromamba shell hook -s bash)"; micromamba activate anubi_humble`),
build with `CXXFLAGS="-include cstdint"` (conda GCC vs Humble generated headers). Run
`scripts/build_mppi_conda_overlay.sh` once per env, then re-activate; check with
`ros2 pkg prefix nav2_mppi_controller` (must point to `mppi_overlay_ws`).
