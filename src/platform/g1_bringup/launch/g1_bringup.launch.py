"""
g1_bringup.launch.py -- Master entry point for the Unitree G1 REAL-ROBOT autonomy stack.

The G1 counterpart of go2_bringup.launch.py, and structurally identical to it: the
robot-agnostic autonomy (robot_nav, robot_safety, robot_lio) is reused UNCHANGED, and
only the robot-specific adapters differ. Where the Go2 uses go2_hw_bridge (Sport API)
the G1 uses g1_state_bridge (DDS state -> ROS) for the STATE half and cmd_vel_to_amo
(Nav2 velocity -> AMO WebSocket) for the COMMAND half. The Go2's wheel-LiDAR is
replaced by the head-mounted Livox Mid-360 (livox driver + imu_rescale + DLIO).

This is HARDWARE bring-up (not simulation). The low-level locomotion controller
runs as its OWN process and is selected by the `locomotion` argument; this launch
only produces velocity commands for it:
  locomotion:=amo    AMO policy (g1_locomotion amo_driver, WebSocket :8766, this host)
  locomotion:=sonic  SONIC whole-body controller (g1_deploy_onnx_ref on the Jetson;
                     connects to THIS host's cmd_vel_to_sonic ZMQ PUB :5556).
                     See docs/g1_sonic_locomotion.md.
NEVER run both controllers at once: two writers on rt/lowcmd is dangerous.

Starts (mirrors go2_bringup order; each block gated by a boolean arg):
  1. robot_state_publisher (g1.urdf.xacro) + joint_state_publisher
  2. g1_state_bridge   DDS -> /odom_raw + /imu/data + base_footprint->base_link TF
        (static base_footprint->base_link fallback when the state bridge is off)
  3. lidar_level_frame_publisher   base_footprint -> lidar_stabilized (IMU-rate)
  4. robot_safety velocity_limiter   /cmd_vel_collision_free -> /cmd_vel_safe   [use_safety]
  5. EKF (robot_localization)   fuses /odom_raw (+ DLIO) -> /odom + odom->base_footprint
  6. lidar_odom_relay   /lidar_odom -> /lidar_odom_reliable                       [use_lio]
  7. Livox Mid-360 driver + imu_rescale (g -> m/s^2)                              [use_livox]
  8. robot_lio (canonical /livox -> /lidar relays + DLIO)                         [use_lio]
  9. slam.launch.py   distortion_corrector (G1 self-filter) + p2l(/scan) + slam_toolbox
 10. nav2.launch.py   Nav2 + collision_monitor (G1 STVL profiles; AMO or SONIC
       velocity envelope picked by locomotion + sonic_profile)                    [use_nav]
 11. cmd_vel_to_amo   /cmd_vel_safe -> AMO WebSocket :8766           [use_nav, locomotion=amo]
     cmd_vel_to_sonic /cmd_vel_safe -> SONIC ZMQ PUB :5556          [use_nav, locomotion=sonic]
 12. RViz2                                                                        [rviz]

Frame / topic contract (G1) -- identical to the Go2, see docs/topic_map.md
--------------------------------------------------------------------------
  map ->(slam_toolbox)-> odom ->(EKF)-> base_footprint
  base_footprint ->(g1_state_bridge: body roll/pitch + standing height)-> base_link(==pelvis)
  base_footprint ->(lidar_level_frame_publisher, IMU-rate)-> lidar_stabilized
  base_link ->(URDF / RSP)-> ... -> mid360_link ->(identity alias)-> lidar_link
  Nav2 robot_base_frame = base_footprint. Nav cloud = /lidar/points_undistorted.
  (lidar_link is the Livox driver's cloud frame_id; the alias makes it resolve in TF.)

  Odometry: DLIO is the real odometry source (use_lio:=true default). DLIO publishes
  /lidar_odom (NOT a TF); lidar_odom_relay injects covariances; the EKF fuses it (and
  the AMO body odometry /odom_raw when available) into a planar odom->base_footprint,
  exactly as on the Go2 (this is the EKF the user asked the G1 to share with the Go2).

  Command chain:
     Nav2 /cmd_vel -> collision_monitor /cmd_vel_collision_free
                   -> velocity_limiter   /cmd_vel_safe          (robot_safety)
     then, by locomotion:
       amo:   cmd_vel_to_amo    ws://127.0.0.1:8766 {vx, vy, yaw}
              -> AMO driver (g1_locomotion amo_driver, separate process) -> robot
       sonic: cmd_vel_to_sonic  ZMQ PUB tcp://*:5556 (planner ticks)
              -> g1_deploy_onnx_ref (SONIC controller on the Jetson) -> robot

Arguments
---------
use_sim_time   : bool  (default: false)  hardware bring-up; keep false
use_livox      : bool  (default: true)   start the Livox Mid-360 driver + imu_rescale
use_lio        : bool  (default: true)   DLIO LiDAR-inertial odometry (the real odometry)
use_ekf_fusion : bool  (default: true)   fuse odometry via EKF, own odom->base_footprint
use_state_bridge: bool (default: true)   DDS body state -> /odom_raw + /imu/data + body TF
use_nav        : bool  (default: true)   Nav2 + collision_monitor + cmd_vel_to_amo
use_stvl       : bool  (default: true)   3D STVL local costmap
use_safety     : bool  (default: true)   velocity_limiter gates /cmd_vel_safe
rviz           : bool  (default: true)   open RViz2
slam_mode      : str   'mapping' | 'localization'
map            : str   slam_toolbox .posegraph for localization mode
nav_delay_sec  : float SLAM/Nav2 start delay (default 5.0: let DLIO calibrate while still)
livox_config   : str   Livox MID360_config.json (host/lidar IPs + roll:180 mount)
standing_height: str   pelvis(base_link) height [m]; forwarded to URDF + state bridge + leveling
net_if         : str   DDS NIC the G1 is reachable on (e.g. eth0)
locomotion     : str   'amo' | 'sonic'  low-level controller the stack commands (default amo)
sonic_profile  : str   'slow_walk' | 'walk'  SONIC velocity envelope; anything but
                       'walk' falls back to the conservative slow_walk profile
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    g1_bringup_share = get_package_share_directory("g1_bringup")
    g1_desc_share = get_package_share_directory("g1_description")
    robot_nav_share = get_package_share_directory("robot_nav")
    robot_safety_share = get_package_share_directory("robot_safety")
    robot_lio_share = get_package_share_directory("robot_lio")

    g1_params = os.path.join(g1_bringup_share, "config", "g1_params.yaml")
    urdf_file = os.path.join(g1_desc_share, "urdf", "g1.urdf.xacro")
    rviz_config = os.path.join(robot_nav_share, "rviz", "nav2.rviz")

    # G1 Nav2 / collision / scan / distortion / DLIO configs (robot-agnostic stack,
    # G1-specific profiles). The EKF configs are the SAME robot-agnostic files the Go2
    # uses (ekf_fusion_params.yaml / ekf_fusion_params_lio.yaml): canonical topics,
    # base_footprint, two_d_mode -- nothing Go2-specific.
    nav2_params_g1 = os.path.join(robot_nav_share, "config", "nav2_params_g1.yaml")
    # SONIC velocity-envelope variants of the G1 Nav2 profile; the AMO profile
    # above is never modified by the sonic path (parallel configs by design).
    nav2_params_g1_sonic_slow = os.path.join(
        robot_nav_share, "config", "nav2_params_g1_sonic_slow.yaml")
    nav2_params_g1_sonic_walk = os.path.join(
        robot_nav_share, "config", "nav2_params_g1_sonic_walk.yaml")
    # cmd_vel_to_sonic per-profile params (mode + caps matched to the Nav2 files).
    g1_sonic_slow_walk_params = os.path.join(
        g1_bringup_share, "config", "g1_sonic_slow_walk_params.yaml")
    g1_sonic_walk_params = os.path.join(
        g1_bringup_share, "config", "g1_sonic_walk_params.yaml")
    collision_params_g1 = os.path.join(robot_nav_share, "config", "collision_monitor_params_g1.yaml")
    scan_params_g1 = os.path.join(robot_nav_share, "config", "scan_from_cloud_g1.yaml")
    distortion_params_g1 = os.path.join(robot_nav_share, "config", "distortion_corrector_g1.yaml")
    slam_params = os.path.join(robot_nav_share, "config", "slam_toolbox_params.yaml")
    dlio_params_g1_real = os.path.join(robot_lio_share, "config", "dlio_params_g1_real.yaml")

    # livox_ros_driver2 is a real-robot-only C++ package (needs the Livox SDK) and may
    # not be installed off-robot; resolve its default config defensively so this launch
    # still parses (the livox driver node is gated behind use_livox anyway).
    try:
        default_livox_config = os.path.join(
            get_package_share_directory("livox_ros_driver2"),
            "config", "MID360_config.json",
        )
    except Exception:
        default_livox_config = ""

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Hardware bring-up; keep false."),
        DeclareLaunchArgument("use_livox", default_value="true",
                              description="Start the Livox Mid-360 driver + imu_rescale."),
        DeclareLaunchArgument("use_lio", default_value="true",
                              description="DLIO LiDAR-inertial odometry (the real odometry source)."),
        DeclareLaunchArgument("use_ekf_fusion", default_value="true",
                              description="Fuse odometry via EKF; EKF owns odom->base_footprint."),
        DeclareLaunchArgument("use_state_bridge", default_value="true",
                              description="DDS body state -> /odom_raw + /imu/data + base_footprint->base_link TF."),
        DeclareLaunchArgument("use_nav", default_value="true",
                              description="Start Nav2 + collision_monitor + cmd_vel_to_amo."),
        DeclareLaunchArgument("use_stvl", default_value="true",
                              description="3D STVL local costmap."),
        DeclareLaunchArgument("use_safety", default_value="true",
                              description="velocity_limiter gates /cmd_vel_collision_free -> /cmd_vel_safe."),
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz2."),
        DeclareLaunchArgument("robot_name", default_value="",
                              description="Nav2 namespace prefix."),
        DeclareLaunchArgument("slam_mode", default_value="mapping",
                              description="'mapping' or 'localization'."),
        DeclareLaunchArgument("map", default_value="",
                              description="slam_toolbox .posegraph for localization mode."),
        DeclareLaunchArgument("slam_params", default_value=slam_params),
        DeclareLaunchArgument(
            "nav_delay_sec", default_value="5.0",
            description="Delay before SLAM/Nav2 start. Default 5 s lets DLIO finish its "
                        "~3 s stationary IMU/gravity calibration and the EKF settle "
                        "before navigation begins; keep the robot still during it."),
        DeclareLaunchArgument("distortion_imu_topic", default_value="/imu/data",
                              description="IMU topic used by distortion_corrector + lidar leveling."),
        DeclareLaunchArgument("livox_config", default_value=default_livox_config,
                              description="Livox MID360_config.json (host/lidar IPs + roll:180 mount)."),
        DeclareLaunchArgument("standing_height", default_value="0.78",
                              description="G1 pelvis(base_link) height above floor [m]; "
                                          "forwarded to URDF + state bridge + lidar leveling. Calibrate on HW."),
        DeclareLaunchArgument("net_if", default_value="",
                              description="DDS NIC the G1 is reachable on (e.g. eth0); empty = SDK default."),
        DeclareLaunchArgument("recovery_ack_key_mask", default_value="0"),
        DeclareLaunchArgument("resume_key_mask", default_value="0"),
        DeclareLaunchArgument("abort_key_mask", default_value="0"),
        DeclareLaunchArgument(
            "locomotion", default_value="amo",
            description="Low-level locomotion controller: 'amo' (WebSocket :8766) or "
                        "'sonic' (SONIC whole-body controller via ZMQ :5556). Selects "
                        "the matching Nav2 velocity envelope and cmd_vel bridge node. "
                        "NEVER run both controllers at once (rt/lowcmd single writer)."),
        DeclareLaunchArgument(
            "sonic_profile", default_value="slow_walk",
            description="SONIC velocity envelope: 'slow_walk' (vx 0.5, wz 0.4; first "
                        "bringup) or 'walk' (vx 0.8, wz 0.5; validated hardware only). "
                        "Anything but 'walk' falls back to slow_walk. Ignored for amo."),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_livox = LaunchConfiguration("use_livox")
    use_lio = LaunchConfiguration("use_lio")
    use_ekf_fusion = LaunchConfiguration("use_ekf_fusion")
    use_state_bridge = LaunchConfiguration("use_state_bridge")
    use_nav = LaunchConfiguration("use_nav")
    use_stvl = LaunchConfiguration("use_stvl")
    use_safety = LaunchConfiguration("use_safety")
    rviz = LaunchConfiguration("rviz")
    robot_name = LaunchConfiguration("robot_name")
    slam_mode = LaunchConfiguration("slam_mode")
    map_path = LaunchConfiguration("map")
    slam_params_cfg = LaunchConfiguration("slam_params")
    nav_delay_sec = LaunchConfiguration("nav_delay_sec")
    distortion_imu_topic = LaunchConfiguration("distortion_imu_topic")
    livox_config = LaunchConfiguration("livox_config")
    standing_height = LaunchConfiguration("standing_height")
    net_if = LaunchConfiguration("net_if")
    recovery_ack_key_mask = LaunchConfiguration("recovery_ack_key_mask")
    resume_key_mask = LaunchConfiguration("resume_key_mask")
    abort_key_mask = LaunchConfiguration("abort_key_mask")
    locomotion = LaunchConfiguration("locomotion")
    sonic_profile = LaunchConfiguration("sonic_profile")

    # The locomotion bridge consumes the safety-gated topic when safety is on, else
    # the collision_monitor output directly (mirrors go2_bringup's hw_bridge_cmd_topic).
    loco_cmd_topic = PythonExpression(
        ["'/cmd_vel_safe' if '", use_safety, "' == 'true' else '/cmd_vel_collision_free'"]
    )
    # Exactly one cmd_vel bridge runs; both are additionally gated by use_nav.
    start_cmd_vel_to_amo = PythonExpression(
        ["'", use_nav, "' == 'true' and '", locomotion, "' != 'sonic'"]
    )
    start_cmd_vel_to_sonic = PythonExpression(
        ["'", use_nav, "' == 'true' and '", locomotion, "' == 'sonic'"]
    )
    # Nav2 velocity envelope follows the selected controller. An unknown
    # sonic_profile intentionally degrades to the conservative slow_walk files.
    nav2_params_file = PythonExpression([
        "('", nav2_params_g1_sonic_walk, "' if '", sonic_profile, "' == 'walk' else '",
        nav2_params_g1_sonic_slow, "') if '", locomotion, "' == 'sonic' else '",
        nav2_params_g1, "'",
    ])
    sonic_bridge_params = PythonExpression([
        "'", g1_sonic_walk_params, "' if '", sonic_profile, "' == 'walk' else '",
        g1_sonic_slow_walk_params, "'",
    ])
    # EKF config: LIO mode (DLIO + body-odom velocity) vs body-odom + IMU mode.
    ekf_params = PythonExpression([
        "'", os.path.join(robot_nav_share, "config", "ekf_fusion_params_lio.yaml"),
        "' if '", use_lio, "' == 'true' else '",
        os.path.join(robot_nav_share, "config", "ekf_fusion_params.yaml"), "'",
    ])

    # ---- 1. robot_state_publisher + joint_state_publisher ----
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file, " standing_height:=", standing_height]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[g1_params, {
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )
    # No joint encoders are bridged to ROS, so publish nominal (zero) joint states to
    # keep the URDF chain (base_link..mid360_link) well-defined for TF/RViz. The nav
    # cloud does not depend on this (distortion_corrector uses explicit sensor geometry).
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---- 2. g1_state_bridge (DDS body state) ----
    # publish_tf:=false -> EKF owns odom->base_footprint. publish_body_tf:=true ->
    # this node owns base_footprint->base_link (body roll/pitch + standing height).
    g1_state_bridge = Node(
        package="g1_bringup",
        executable="g1_state_bridge",
        name="g1_state_bridge",
        output="screen",
        parameters=[g1_params, {
            "use_sim_time": use_sim_time,
            "net_if": net_if,
            "standing_height": standing_height,
            "publish_tf": False,
            "publish_body_tf": True,
        }],
        condition=IfCondition(use_state_bridge),
    )
    # Fallback static base_footprint->base_link (no body tilt) for rosbag replay or
    # bench tests where the state bridge is off. Real runs use the bridge instead.
    base_footprint_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_to_base_link_static_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x", "0", "--y", "0", "--z", standing_height,
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_footprint", "--child-frame-id", "base_link",
        ],
        condition=UnlessCondition(use_state_bridge),
    )

    # ---- 2b. mid360_link -> lidar_link cloud-frame alias ----
    # The Livox driver stamps its cloud with frame_id "lidar_link" (matching the
    # sim's gpu_lidar frame), but the URDF sensor link is "mid360_link". No nav
    # consumer does a TF lookup on the cloud frame (DLIO + distortion_corrector
    # use explicit extrinsics, and the nav cloud is re-stamped to lidar_stabilized),
    # but without this identity alias RViz cannot display the raw /livox/lidar and
    # the cloud frame is absent from TF. Publish an identity mid360_link->lidar_link
    # so the driver frame resolves on real exactly as it does in sim.
    mid360_to_lidar_link_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mid360_to_lidar_link_alias_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "mid360_link", "--child-frame-id", "lidar_link",
        ],
    )

    # ---- 3. lidar_level_frame_publisher: base_footprint -> lidar_stabilized ----
    lidar_level_frame_publisher = Node(
        package="robot_nav",
        executable="lidar_level_frame_publisher",
        name="lidar_level_frame_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "imu_topic": distortion_imu_topic,
            "parent_frame": "base_footprint",
            "child_frame": "lidar_stabilized",
            "lidar_x_in_base": -0.00368,
            "lidar_y_in_base": 0.00003,
            "lidar_z_in_base": 0.46018,
            "standing_height": ParameterValue(standing_height, value_type=float),
        }],
    )

    # ---- 4. robot_safety velocity_limiter ----
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_safety_share, "launch", "safety.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "cmd_vel_in": "/cmd_vel_collision_free",
            "cmd_vel_out": "/cmd_vel_safe",
            "nav_namespace": robot_name,
            "recovery_ack_key_mask": recovery_ack_key_mask,
            "resume_key_mask": resume_key_mask,
            "abort_key_mask": abort_key_mask,
        }.items(),
        condition=IfCondition(use_safety),
    )

    # ---- 5. EKF: owns /odom + odom->base_footprint ----
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "ekf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "output_odom_topic": "/odom",
            "params_file": ekf_params,
        }.items(),
        condition=IfCondition(use_ekf_fusion),
    )

    # ---- 6. lidar_odom_relay (DLIO covariance injection) ----
    lidar_odom_relay = Node(
        package="robot_nav",
        executable="lidar_odom_relay.py",
        name="lidar_odom_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/lidar_odom",
            "output_topic": "/lidar_odom_reliable",
            "pose_cov": [0.03, 0.03, 0.05, 0.03, 0.03, 0.02],
            "twist_cov": [0.02, 0.02, 0.05, 0.02, 0.02, 0.02],
            "max_pos_speed": 1.5,
            "max_scale": 100.0,
        }],
        condition=IfCondition(use_lio),
    )

    # ---- 7. Livox Mid-360 driver + imu_rescale ----
    # PointCloud2 mode (xfer_format=0): PointXYZRTLT with per-point timestamps so DLIO
    # auto-detects LIVOX and deskews. MID360_config.json applies roll:180 to right the
    # inverted mount's cloud; the IMU stays inverted (corrected by the DLIO extrinsic).
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format": 0,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "lidar_link",
            "user_config_path": livox_config,
            "cmdline_input_bd_code": "livox0000000001",
        }],
        condition=IfCondition(use_livox),
    )
    imu_rescale = Node(
        package="g1_bringup",
        executable="imu_rescale",
        name="imu_rescale",
        output="screen",
        parameters=[g1_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_livox),
    )

    # ---- 8. robot_lio: canonical /livox -> /lidar relays + DLIO ----
    # The relays (/livox/lidar -> /lidar/points, /livox/imu_ms2 -> /lidar/imu) ALWAYS
    # run so distortion_corrector has /lidar/points; DLIO itself is gated by use_lio.
    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_lio_share, "launch", "lio.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_lio": use_lio,
            "start_lidar_points_relay": "true",
            "lidar_points_input_topic": "/livox/lidar",
            "lidar_points_frame": "lidar_link",
            "publish_lidar_points_frame_tf": "false",
            "start_lidar_imu_relay": "true",
            "lidar_imu_input_topic": "/livox/imu_ms2",
            # Mid-360 IMU TF is handled by the DLIO extrinsic, not a static TF here.
            "publish_lidar_imu_tf": "false",
            "dlio_publish_tf": "false",
            "dlio_params_file": dlio_params_g1_real,
        }.items(),
    )

    # ---- 9. SLAM: distortion_corrector (G1 self-filter) + p2l(/scan) + slam_toolbox ----
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_mode": slam_mode,
            "slam_params": slam_params_cfg,
            "map": map_path,
            "distortion_params": distortion_params_g1,
            "distortion_imu_topic": distortion_imu_topic,
            "scan_params": scan_params_g1,
            # Scan accumulation disabled on the G1: /scan is taken from single
            # Mid-360 frames, matching the sim path (which has no accumulator), so
            # sim and real share the same 2D scan behavior. If the scan proves too
            # sparse on hardware, re-enable accumulate_scan_cloud.
            "accumulate_scan_cloud": "false",
        }.items(),
    )

    # ---- 10. Nav2 + collision_monitor (G1 STVL profiles) ----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": robot_name,
            "use_stvl": use_stvl,
            "params_file": nav2_params_file,
            "collision_monitor_params": collision_params_g1,
        }.items(),
        condition=IfCondition(use_nav),
    )

    # ---- 11. locomotion bridge: exactly one of cmd_vel_to_amo / cmd_vel_to_sonic ----
    cmd_vel_to_amo = Node(
        package="g1_bringup",
        executable="cmd_vel_to_amo",
        name="cmd_vel_to_amo",
        output="screen",
        parameters=[g1_params, {
            "use_sim_time": use_sim_time,
            "cmd_vel_topic": loco_cmd_topic,
        }],
        condition=IfCondition(start_cmd_vel_to_amo),
    )
    # SONIC bridge binds ZMQ PUB :5556; g1_deploy_onnx_ref on the Jetson connects
    # to this host (see docs/g1_sonic_locomotion.md for the Jetson-side flags).
    cmd_vel_to_sonic = Node(
        package="g1_bringup",
        executable="cmd_vel_to_sonic",
        name="cmd_vel_to_sonic",
        output="screen",
        parameters=[sonic_bridge_params, {
            "use_sim_time": use_sim_time,
            "cmd_vel_topic": loco_cmd_topic,
        }],
        condition=IfCondition(start_cmd_vel_to_sonic),
    )

    # ---- 12. RViz2 ----
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(rviz),
    )

    # SLAM + Nav2 start after nav_delay_sec so DLIO calibrates and the EKF settles
    # while the robot stands still (mirrors go2_bringup's delayed_nav_stack).
    delayed_nav_stack = TimerAction(
        period=nav_delay_sec,
        actions=[slam, nav2, cmd_vel_to_amo, cmd_vel_to_sonic],
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        joint_state_publisher,
        g1_state_bridge,
        base_footprint_to_base_link_tf,
        mid360_to_lidar_link_alias_tf,
        lidar_level_frame_publisher,
        safety,
        ekf,
        lidar_odom_relay,
        livox_driver,
        imu_rescale,
        lio,
        delayed_nav_stack,
        rviz2,
    ])
