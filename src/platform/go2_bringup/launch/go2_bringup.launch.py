"""
go2_bringup.launch.py
Master entry point for the Go2 autonomy stack.

Starts (in order):
  1. robot_state_publisher  (URDF + static TF from joints)
  2. go2_hw_bridge          (SportModeState -> odom/TF,  cmd_vel -> Sport API)
  3. velocity_limiter       (optional safety gate; enabled by use_safety:=true)
  4. slam.launch.py         (pointcloud_to_laserscan + slam_toolbox)
  5. nav2.launch.py         (full Nav2 navigation stack)
  6. RViz2                  (optional)

Arguments
---------
use_sim_time    : bool   (default: false)
map             : str    Path to slam_toolbox serialised map (.posegraph) for
                         localisation-only mode. Empty = mapping mode (default).
robot_name      : str    Robot namespace prefix  (default: '')
rviz            : bool   Launch RViz2 (default: true)
params_file     : str    Override Nav2 params file
slam_params     : str    Override slam_toolbox params file
slam_mode       : str    'mapping' or 'localization' (default: 'mapping')
use_safety      : bool   Enable velocity_limiter safety gate (default: false)
                         When true: go2_hw_bridge reads /cmd_vel_safe instead of /cmd_vel.

LiDAR topic contract (real robot)
----------------------------------
The Go2 internal ROS 2 bridge publishes raw point cloud on /utlidar/cloud.
robot_lio (included from go2_bringup) relays it to /lidar/points and provides
the distortion_corrector pipeline that publishes /lidar/points_undistorted.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    go2_bringup_share     = get_package_share_directory("go2_bringup")
    go2_description_share = get_package_share_directory("go2_description")
    robot_nav_share       = get_package_share_directory("robot_nav")
    robot_safety_share    = get_package_share_directory("robot_safety")

    go2_params      = os.path.join(go2_bringup_share,    "config", "go2_params.yaml")
    default_nav     = os.path.join(robot_nav_share,      "config", "nav2_params.yaml")
    default_slam    = os.path.join(robot_nav_share,      "config", "slam_toolbox_params.yaml")
    urdf_file       = os.path.join(go2_description_share,"urdf",   "go2.urdf.xacro")
    rviz_config     = os.path.join(robot_nav_share,      "rviz",   "nav2.rviz")

    # Arguments
    args = [
        DeclareLaunchArgument(
            "use_stvl",
            default_value="true",
            description="Use SpatioTemporalVoxelLayer for 3D obstacle avoidance"
                        " in the local costmap. Set false to revert to 2D obstacle layer.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map",          default_value="",
                              description="slam_toolbox map file (.posegraph) for"
                                          " localisation mode; empty = mapping mode"),
        DeclareLaunchArgument("robot_name",   default_value=""),
        DeclareLaunchArgument("rviz",         default_value="true"),
        DeclareLaunchArgument("params_file",  default_value=default_nav),
        DeclareLaunchArgument("slam_params",  default_value=default_slam),
        DeclareLaunchArgument("slam_mode",    default_value="mapping",
                              description="'mapping' or 'localization'"),
        DeclareLaunchArgument("use_ekf_fusion", default_value="true",
                      description="Use EKF to fuse /odom_raw and publish /odom"),
        DeclareLaunchArgument("use_lio", default_value="false",
                      description="Enable optional LIO odometry backend"),
        DeclareLaunchArgument("lidar_points_input_topic", default_value="/utlidar/cloud",
                      description="Raw LiDAR pointcloud input topic for canonical relay"),
        DeclareLaunchArgument("lidar_imu_input_topic", default_value="/unilidar/imu",
                      description="Raw LiDAR IMU input topic for canonical relay"),
        DeclareLaunchArgument("use_safety",   default_value="false",
                              description="Enable velocity_limiter safety gate; "
                                          "go2_hw_bridge will read /cmd_vel_safe"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_stvl     = LaunchConfiguration("use_stvl")
    map_path     = LaunchConfiguration("map")
    robot_name   = LaunchConfiguration("robot_name")
    rviz         = LaunchConfiguration("rviz")
    slam_params  = LaunchConfiguration("slam_params")
    slam_mode    = LaunchConfiguration("slam_mode")
    use_ekf_fusion = LaunchConfiguration("use_ekf_fusion")
    use_lio = LaunchConfiguration("use_lio")
    lidar_points_input_topic = LaunchConfiguration("lidar_points_input_topic")
    lidar_imu_input_topic = LaunchConfiguration("lidar_imu_input_topic")
    use_safety   = LaunchConfiguration("use_safety")

    # Collision Monitor always sits in the command path for industrial safety.
    # When use_safety=true, velocity_limiter adds a watchdog / estop gate after it.
    hw_bridge_cmd_topic = PythonExpression(
        ["'/cmd_vel_safe' if '", use_safety, "' == 'true' else '/cmd_vel_collision_free'"]
    )

    # 1. robot_state_publisher
    robot_description = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            go2_params,
            {"robot_description": robot_description, "use_sim_time": use_sim_time},
        ],
    )

    # 2. go2_hw_bridge
    # Converts SportModeState -> odom + TF(odom->base_footprint)
    # Converts cmd_vel -> /api/sport/request
    go2_hw_bridge = Node(
        package="go2_bringup",
        executable="go2_hw_bridge",
        name="go2_hw_bridge",
        output="screen",
        parameters=[go2_params, {
            "use_sim_time":  use_sim_time,
            "cmd_vel_topic": hw_bridge_cmd_topic,
            "publish_tf": PythonExpression(["'false' if '", use_ekf_fusion, "' == 'true' else 'true'"]),
            # Without EKF, the bridge must publish on /odom so SLAM/Nav2 see odometry.
            # With EKF enabled, keep /odom_raw so the EKF can consume it and publish /odom.
            "odom_topic": PythonExpression(["'/odom_raw' if '", use_ekf_fusion, "' == 'true' else '/odom'"]),
        }],
    )
    lidar_stabilized_tf = Node(
        package="robot_nav",
        executable="imu_stabilized_tf.py",
        name="imu_stabilized_tf",
        output="screen",
        parameters=[{
            "use_sim_time":    use_sim_time,
            "imu_topic":       "/imu/data",
            "parent_frame":    "base_footprint",
            "child_frame":     "lidar_stabilized",
            "lidar_x_in_base":  0.28945,   # from go2_description/urdf/go2.urdf.xacro
            "lidar_y_in_base":  0.0,
            "lidar_z_in_base": -0.046825,
            "standing_height":  0.28,
        }],
    )

    # 3. Safety gate (optional)
    # velocity_limiter sits between Nav2 /cmd_vel and go2_hw_bridge.
    # Only launched when use_safety:=true.
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_safety_share, "launch", "safety.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "cmd_vel_in": "/cmd_vel_collision_free",
            "cmd_vel_out": "/cmd_vel_safe",
        }.items(),
        condition=IfCondition(use_safety),
    )

    # 3b. Optional odometry fusion (default ON)
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "ekf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "output_odom_topic": "/odom",
        }.items(),
        condition=IfCondition(use_ekf_fusion),
    )

    # 3c. Canonical LIO sensor contract + optional backend
    def maybe_include_lio(context):
        use_lio_enabled = use_lio.perform(context).lower() == "true"
        try:
            robot_lio_share = get_package_share_directory("robot_lio")
        except PackageNotFoundError:
            if use_lio_enabled:
                raise RuntimeError(
                    "use_lio:=true but package 'robot_lio' was not found in the workspace/install."
                )
            return [
                LogInfo(
                    msg=(
                        "package 'robot_lio' not found; skipping internal LiDAR relay/LIO include."
                    )
                )
            ]

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_lio_share, "launch", "lio.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_lio": use_lio,
                    "start_lidar_points_relay": "true",
                    "start_lidar_imu_relay": "true",
                    "lidar_points_input_topic": lidar_points_input_topic,
                    "lidar_imu_input_topic": lidar_imu_input_topic,
                    # lidar_link->lidar_imu_link is defined in go2.urdf.xacro and
                    # published by robot_state_publisher - do not duplicate it.
                    "publish_lidar_imu_tf": "false",
                }.items(),
            )
        ]

    lio = OpaqueFunction(function=maybe_include_lio)

    # 4. SLAM (slam_toolbox + pointcloud_to_laserscan)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params":  slam_params,
            "slam_mode":    slam_mode,
            "map":          map_path,
        }.items(),
    )

    # 5. Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace":    robot_name,
            "use_stvl":     use_stvl,
            "params_file":  LaunchConfiguration("params_file"),
        }.items(),
    )

    # 6. RViz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        go2_hw_bridge,
        lidar_stabilized_tf,
        safety,
        ekf,
        lio,
        slam,
        nav2,
        rviz2,
    ])
