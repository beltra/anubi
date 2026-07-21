# g1_isaac -- Isaac Sim G1 adapter

Thin relay layer that lets the anubi stack (Nav2 + SLAM + DLIO) consume the
Isaac Sim G1 simulation. The Isaac host (`TalOS/Navigation sim/isaac_dds_host.py`,
separate repo) publishes raw Livox topics and `/clock` on **ROS_DOMAIN_ID 42**;
this package maps them onto the canonical contract in `docs/topic_map.md` and
publishes the static TFs the Isaac bringup owns. No URDF /
`robot_state_publisher` runs against the Isaac host, so the sensor mount TF
lives here.

Status: contract smoke-verified live (robot held) and driven through a full
Nav2 bringup on CT111 (Isaac host + deploy + EKF + slam_toolbox + Nav2 +
collision_monitor + cmd_vel_to_sonic). Not yet wired into a
`scripts/run_*.sh` wrapper; `scripts/kill_ros_nodes.sh` covers these nodes.

## Topic map

| In (Isaac host) | Out (canonical) | Type | Frame out | QoS out | Status |
|---|---|---|---|---|---|
| `/livox/lidar` | `/lidar/points` | `sensor_msgs/PointCloud2` | `lidar_link` | RELIABLE, depth 5 | live |
| `/livox/imu_raw` | `/lidar/imu` | `sensor_msgs/Imu` | `lidar_link` | RELIABLE, depth 5 | live |
| `/pelvis/imu_raw` | `/imu/data` | `sensor_msgs/Imu` | `imu_frame` arg (default `imu_link`) | RELIABLE, depth 5 | live (`enable_body_imu`, default true) |
| `/odom_gt` | `/odom_raw` | `nav_msgs/Odometry` | `odom` / child `base_footprint` | RELIABLE, depth 5 | live (`enable_odom_raw`, default true) |

Relay behavior (`scripts/frame_relay_node.py`): only `header.frame_id` (and,
for odometry, `child_frame_id`) is rewritten. Header stamps pass through
untouched -- the Isaac host stamps from the shared sim `/clock`, so the
host-clock restamping that the real G1 needs (`robot_lio`'s
`cloud_restamp_relay` compensating Mid-360 timebase drift) does not apply
here. The cloud's extra point fields (`intensity`, and the per-point relative
`time` float32 used by the distortion corrector for deskew) are preserved
byte-for-byte. The host IMU is already in m/s^2, so the real-G1
`imu_rescale` (g -> m/s^2) stage is also not needed.

Sensor geometry note: the host cloud arrives in an upright sensor frame with
elevation span -52..+7 deg (downward-biased, righted production convention),
hence the upright, pitch-only mount TF below.

## Static TFs (published here)

| Transform | Value | Notes |
|---|---|---|
| `base_footprint -> base_link` | z = 0.80 (arg `standing_height`) | Fixed standing height, sim convention. `docs/topic_map.md` quotes ~0.78 m for the real G1 -- calibrate. |
| `base_link -> lidar_link` | xyz (-0.00368, 0.00003, 0.46018), rpy (0, 0.0401, 0) | Head Mid-360, mounted UPRIGHT (no roll; cloud already righted). |
| `base_link -> <imu_frame>` | identity | Only with `enable_body_imu:=true`; follows the `imu_frame` arg. |

Not published here (owned elsewhere, per `docs/topic_map.md`): `map -> odom`
(slam_toolbox) and `odom -> base_footprint` (EKF).

## IMU frame discrepancy -- DECISION NEEDED

The anubi **sim** contract puts the body IMU in frame `imu_link`
(`docs/topic_map.md`, `/imu/data` row). The **real G1** publishes frame
`"imu"`. This package deliberately does NOT pick for you: the launch arg
`imu_frame` defaults to `imu_link` (sim convention) and the identity
`base_link -> <imu_frame>` TF follows it. If the team decides Isaac should
mimic real-robot frames for sim2real parity, pass `imu_frame:=imu` -- but do
it explicitly and consistently with the EKF/distortion-corrector configs.

## Host publishers (live)

- **Pelvis (body) IMU** on `/pelvis/imu_raw`: synthesized by
  `livox_zmq_republisher.py --pelvis-imu-topic /pelvis/imu_raw` from the
  host odom stream (orientation + body gyro + gravity-only accelerometer,
  |acc| = 9.8066 -- a minimum model, no motion accel, no noise).
- **Ground-truth odometry** on `/odom_gt`: `isaac_dds_host.py
  --publish-odom-gt` streams the pelvis world pose over ZMQ :5560
  (`[b"odom", json]` @ 50 Hz sim); the republisher publishes it as
  `nav_msgs/Odometry` with all-zero covariance (flag for the EKF -- see
  team decisions below).

DLIO is NOT used against the Isaac host: the RTX lidar annotator's ~3.9 Hz
effective stream with multi-second dead phases makes GICP re-lock aliased
on the warehouse rack geometry (verified twice on the TalOS stack; the real
robot keeps DLIO unchanged on its dense 10 Hz stream).

## Nav stack timing notes (sim-scoped, learned in the Nav2 bringup)

- `config/slam_toolbox_params_g1_isaac.yaml` raises `transform_timeout`
  0.5 -> 2 s: slam_toolbox stamps map->odom at (last scan + timeout), and
  the annotator's scan holes otherwise push every Nav2 robot-pose lookup
  into "extrapolation into the future" (MPPI aborts the goal). Larger
  leads backfire -- see the file header for the measured 60 s and 8 s
  failure modes (tf2 cache interaction). Side effects: Nav2 map-frame
  activation stalls ~2 sim-seconds at startup -- gate bringup on the
  lifecycle manager's "Managed nodes are active", not on action-server
  existence; annotator dead phases longer than ~2 s can abort the active
  goal ("Goal failed") -- re-send it once the lidar stream has been back
  for a few seconds.
- `config/collision_monitor_params_g1_isaac.yaml` raises `source_timeout`
  0.5 -> 1.5 s for the same scan holes; multi-second dead phases still stop
  the robot (intended).
- The bridge profile `config/g1_sonic_sim_params.yaml` caps commanded speed
  at 0.28 m/s: the deploy ticks wall clock against RTF < 1 physics, so
  commanded 0.28 realizes ~0.4 m/s in sim time.

## Running against the Isaac host

The host publishes on domain 42 with sim time; every consumer must match the
domain, RMW, and clock:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash   # after: colcon build --packages-select g1_isaac

ros2 launch g1_isaac g1_isaac_bringup.launch.py
```

All nodes started by this launch run with `use_sim_time:=true` (default);
any downstream stack launched against it (distortion corrector, SLAM, Nav2,
DLIO) must also set `use_sim_time:=true`, since `/clock` comes from Isaac.

Quick checks:

```bash
ros2 topic hz /lidar/points          # ~10 Hz sim time
ros2 topic echo /lidar/points --field header.frame_id --once   # lidar_link
ros2 run tf2_tools view_frames       # base_footprint -> base_link -> lidar_link
```

## Launch arguments

| Arg | Default | Purpose |
|---|---|---|
| `use_sim_time` | `true` | Isaac host is the `/clock` source. |
| `lidar_points_input_topic` / `lidar_points_topic` | `/livox/lidar` / `/lidar/points` | Cloud relay endpoints. |
| `lidar_imu_input_topic` / `lidar_imu_topic` | `/livox/imu_raw` / `/lidar/imu` | LiDAR IMU relay endpoints. |
| `lidar_frame` | `lidar_link` | Output frame for both LiDAR relays and the mount TF child. |
| `enable_body_imu` | `false` | Start the pelvis IMU relay + IMU TF (host support pending). |
| `body_imu_input_topic` / `body_imu_topic` | `/pelvis/imu_raw` / `/imu/data` | Body IMU relay endpoints. |
| `imu_frame` | `imu_link` | Body IMU frame_id -- see discrepancy note above. |
| `enable_odom_raw` | `false` | Start the ground-truth odom relay (host support pending). |
| `odom_input_topic` / `odom_raw_topic` | `/odom_gt` / `/odom_raw` | Odometry relay endpoints. |
| `standing_height` | `0.80` | `base_footprint -> base_link` z offset. |
