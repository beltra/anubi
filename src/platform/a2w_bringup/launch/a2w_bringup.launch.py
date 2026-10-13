"""
a2w_bringup.launch.py -- Master entry point for the Unitree A2-W (Pro) REAL-ROBOT stack.

Real-robot counterpart of a2w_sim/sim_bringup_a2w.launch.py. The perception and
navigation chain is the one validated in sim; only the sources differ:
  sim                               real
  Gazebo VelocityControl         -> go2_hw_bridge -> SportClient Move (/api/sport/request)
  Gazebo OdometryPublisher       -> go2_hw_bridge from /lf/sportmodestate -> /odom_raw
  Gazebo IMU /imu/data           -> go2_hw_bridge SportModeState.imu_state -> /imu/data
  gpu_lidar x2                   -> Hesai JT128 driver x2 (started outside this launch)

Starts:
  1. robot_state_publisher  (a2w.urdf.xacro, fixed legs in the stand pose)
  2. go2_hw_bridge          (A2 Sport API adapter, a2w_params.yaml)
  3. lidar frame aliases    (driver frame_id -> front/rear_lidar_link, optional)
  4. cloud_merger           (front + rear clouds -> /lidar/points, frame lidar_link)
  5. lidar_level_frame_publisher
  6. robot_safety           (velocity_limiter /cmd_vel_collision_free -> /cmd_vel_safe)
  7. EKF + lidar_odom_relay + DLIO (use_lio, default true)
  8. slam.launch.py         (distortion_corrector_a2w + scan_from_cloud_a2w + slam_toolbox)
  9. nav2.launch.py         (nav2_params_a2w.yaml + collision_monitor_params_a2w.yaml)
 10. RViz2                  (optional)

Not verified on hardware. Before the first run, check on the robot:
  - the JT128 driver topics and frame_id (front/rear_lidar_points_input_topic,
    front/rear_lidar_points_frame); the upstream A2 URDF lidar frames have z on the
    dome axis, and lidar_frame_alias_yaw rotates the driver frame about that axis
  - that /lf/sportmodestate is published and Move is accepted in the wheeled mode
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

# Kept in lockstep with a2w_description/urdf/a2w.urdf.xacro (lidar_link) and
# a2w_params.yaml (standing_height).
STANDING_HEIGHT_M = 0.497
LIDAR_Z_IN_BASE_M = 0.0843


def generate_launch_description():
    a2w_bringup_share = get_package_share_directory("a2w_bringup")
    a2w_description_share = get_package_share_directory("a2w_description")
    robot_nav_share = get_package_share_directory("robot_nav")
    robot_safety_share = get_package_share_directory("robot_safety")

    a2w_params = os.path.join(a2w_bringup_share, "config", "a2w_params.yaml")
    urdf_file = os.path.join(a2w_description_share, "urdf", "a2w.urdf.xacro")
    default_slam = os.path.join(robot_nav_share, "config", "slam_toolbox_params_a2w.yaml")
    rviz_config = os.path.join(robot_nav_share, "rviz", "nav2.rviz")
    ekf_params_base = os.path.join(robot_nav_share, "config", "ekf_fusion_params.yaml")
    ekf_params_lio = os.path.join(robot_nav_share, "config", "ekf_fusion_params_lio.yaml")
    nav2_params = os.path.join(robot_nav_share, "config", "nav2_params_a2w.yaml")
    collision_params = os.path.join(robot_nav_share, "config", "collision_monitor_params_a2w.yaml")
    scan_params = os.path.join(robot_nav_share, "config", "scan_from_cloud_a2w.yaml")
    distortion_params = os.path.join(robot_nav_share, "config", "distortion_corrector_a2w.yaml")

    args = [
        DeclareLaunchArgument("use_stvl", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value="",
                              description="slam_toolbox map file (.posegraph) for localization mode"),
        DeclareLaunchArgument("robot_name", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "nav_delay_sec", default_value="5.0",
            description="Delay before SLAM and Nav2 start; lets DLIO finish its stationary "
                        "IMU/gravity calibration (robot must stand still)",
        ),
        DeclareLaunchArgument("slam_params", default_value=default_slam),
        DeclareLaunchArgument("slam_mode", default_value="mapping",
                              description="'mapping' or 'localization'"),
        DeclareLaunchArgument("use_ekf_fusion", default_value="true",
                              description="Fuse /odom_raw via EKF and publish /odom"),
        DeclareLaunchArgument("use_lio", default_value="true",
                              description="DLIO on the merged cloud + body IMU feeds the EKF"),
        DeclareLaunchArgument("front_lidar_points_input_topic", default_value="/lidar_points_front",
                              description="Front JT128 driver cloud topic"),
        DeclareLaunchArgument("rear_lidar_points_input_topic", default_value="/lidar_points_rear",
                              description="Rear JT128 driver cloud topic"),
        DeclareLaunchArgument(
            "front_lidar_points_frame", default_value="front_lidar_link",
            description="frame_id stamped by the front driver; aliased to front_lidar_link if different",
        ),
        DeclareLaunchArgument(
            "rear_lidar_points_frame", default_value="rear_lidar_link",
            description="frame_id stamped by the rear driver; aliased to rear_lidar_link if different",
        ),
        DeclareLaunchArgument(
            "lidar_frame_alias_yaw", default_value="0.0",
            description="Rotation [rad] about the dome axis from the URDF lidar frame to the "
                        "driver frame (alias TF only)",
        ),
        DeclareLaunchArgument(
            "distortion_imu_topic", default_value="/imu/data",
            description="IMU topic used by distortion_corrector",
        ),
        DeclareLaunchArgument("use_safety", default_value="true",
                              description="Enable velocity_limiter; go2_hw_bridge reads /cmd_vel_safe"),
        DeclareLaunchArgument("recovery_ack_key_mask", default_value="0"),
        DeclareLaunchArgument("resume_key_mask", default_value="0"),
        DeclareLaunchArgument("abort_key_mask", default_value="0"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_lio = LaunchConfiguration("use_lio")
    use_ekf_fusion = LaunchConfiguration("use_ekf_fusion")
    use_safety = LaunchConfiguration("use_safety")
    robot_name = LaunchConfiguration("robot_name")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file, " fixed_legs:=true"]), value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            a2w_params,
            {"robot_description": robot_description, "use_sim_time": use_sim_time},
        ],
    )

    a2w_hw_bridge = Node(
        package="go2_bringup",
        executable="go2_hw_bridge",
        name="go2_hw_bridge",
        output="screen",
        parameters=[a2w_params, {
            "use_sim_time": use_sim_time,
            "cmd_vel_topic": PythonExpression(
                ["'/cmd_vel_safe' if '", use_safety, "' == 'true' else '/cmd_vel_collision_free'"]),
            "publish_tf": PythonExpression(["'false' if '", use_ekf_fusion, "' == 'true' else 'true'"]),
            "odom_topic": PythonExpression(["'/odom_raw' if '", use_ekf_fusion, "' == 'true' else '/odom'"]),
        }],
    )

    def make_lidar_aliases(context):
        yaw = LaunchConfiguration("lidar_frame_alias_yaw").perform(context)
        nodes = []
        for side in ("front", "rear"):
            urdf_frame = f"{side}_lidar_link"
            driver_frame = LaunchConfiguration(f"{side}_lidar_points_frame").perform(context)
            if driver_frame == urdf_frame:
                continue
            nodes.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{side}_lidar_frame_alias_tf",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", yaw,
                    "--frame-id", urdf_frame, "--child-frame-id", driver_frame,
                ],
            ))
        return nodes

    lidar_aliases = OpaqueFunction(function=make_lidar_aliases)

    def make_cloud_merger(context):
        # Resolved here so input_topics stays a string array, not one joined string.
        input_topics = [
            LaunchConfiguration("front_lidar_points_input_topic").perform(context),
            LaunchConfiguration("rear_lidar_points_input_topic").perform(context),
        ]
        return [Node(
            package="robot_nav",
            executable="cloud_merger",
            name="cloud_merger",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topics": input_topics,
                "output_topic": "/lidar/points",
                "output_frame": "lidar_link",
                "max_input_age_sec": 0.15,
            }],
        )]

    cloud_merger = OpaqueFunction(function=make_cloud_merger)

    lidar_level_frame_publisher = Node(
        package="robot_nav",
        executable="lidar_level_frame_publisher",
        name="lidar_level_frame_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "imu_topic": "/imu/data",
            "parent_frame": "base_footprint",
            "child_frame": "lidar_stabilized",
            "lidar_x_in_base": 0.0,
            "lidar_y_in_base": 0.0,
            "lidar_z_in_base": LIDAR_Z_IN_BASE_M,
            "standing_height": STANDING_HEIGHT_M,
        }],
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_safety_share, "launch", "safety.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "cmd_vel_in": "/cmd_vel_collision_free",
            "cmd_vel_out": "/cmd_vel_safe",
            "nav_namespace": robot_name,
            "recovery_ack_key_mask": LaunchConfiguration("recovery_ack_key_mask"),
            "resume_key_mask": LaunchConfiguration("resume_key_mask"),
            "abort_key_mask": LaunchConfiguration("abort_key_mask"),
        }.items(),
        condition=IfCondition(use_safety),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_nav_share, "launch", "ekf.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "output_odom_topic": "/odom",
            "params_file": PythonExpression([
                f"'{ekf_params_lio}' if '", use_lio, f"' == 'true' else '{ekf_params_base}'"
            ]),
        }.items(),
        condition=IfCondition(use_ekf_fusion),
    )

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

    def maybe_include_lio(context):
        if use_lio.perform(context).lower() != "true":
            return []
        try:
            robot_lio_share = get_package_share_directory("robot_lio")
        except PackageNotFoundError:
            raise RuntimeError("use_lio:=true but package 'robot_lio' was not found.")
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_lio_share, "launch", "lio.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_lio": use_lio,
                # cloud_merger already publishes the canonical /lidar/points.
                "start_lidar_points_relay": "false",
                "start_lidar_imu_relay": "false",
                "publish_lidar_imu_tf": "false",
                # Body IMU is the single inertial reference (one per robot, not per JT128).
                "lidar_imu_topic": "/imu/data",
                "dlio_params_file": os.path.join(
                    robot_lio_share, "config", "dlio_params_a2w_real.yaml"),
            }.items(),
        )]

    lio = OpaqueFunction(function=maybe_include_lio)

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_nav_share, "launch", "slam.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params": LaunchConfiguration("slam_params"),
            "slam_mode": LaunchConfiguration("slam_mode"),
            "map": LaunchConfiguration("map"),
            "distortion_params": distortion_params,
            "distortion_imu_topic": LaunchConfiguration("distortion_imu_topic"),
            "scan_params": scan_params,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_nav_share, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": robot_name,
            "use_stvl": LaunchConfiguration("use_stvl"),
            "params_file": nav2_params,
            "collision_monitor_params": collision_params,
        }.items(),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    delayed_nav_stack = TimerAction(
        period=LaunchConfiguration("nav_delay_sec"),
        actions=[slam, nav2],
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        a2w_hw_bridge,
        lidar_aliases,
        cloud_merger,
        lidar_level_frame_publisher,
        safety,
        ekf,
        lidar_odom_relay,
        lio,
        delayed_nav_stack,
        rviz2,
    ])
