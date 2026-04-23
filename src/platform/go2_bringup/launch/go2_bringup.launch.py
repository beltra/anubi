"""
go2_bringup.launch.py -- Master entry point for the Go2 autonomy stack.

Starts (in order):
  1. robot_state_publisher  (URDF + static TF from joints)
  2. go2_hw_bridge          (SportModeState -> odom/TF, cmd_vel -> Sport API)
  3. velocity_limiter       (optional safety gate; enabled by use_safety:=true)
  4. EKF                    (optional odom fusion; enabled by use_ekf_fusion:=true)
  5. robot_lio              (canonical LiDAR relay + optional DLIO backend)
  6. slam.launch.py         (pointcloud_to_laserscan + slam_toolbox)
  7. nav2.launch.py         (full Nav2 navigation stack)
  8. RViz2                  (optional)

LiDAR topic contract: the Go2 internal ROS 2 bridge publishes raw point cloud on
/utlidar/cloud. robot_lio relays it to /lidar/points and distortion_corrector
publishes /lidar/points_undistorted for SLAM and costmaps.

Arguments
---------
use_sim_time    : bool   (default: false)
map             : str    slam_toolbox .posegraph file for localization mode; empty = mapping
robot_name      : str    Nav2 namespace prefix (default: '')
rviz            : bool   Launch RViz2 (default: true)
params_file     : str    Override Nav2 params file
slam_params     : str    Override slam_toolbox params file
slam_mode       : str    'mapping' or 'localization' (default: 'mapping')
use_safety      : bool   Enable velocity_limiter safety gate (default: false)
use_ekf_fusion  : bool   EKF odom fusion (default: true)
use_lio         : bool   Enable DLIO LiDAR-inertial odometry (default: false)
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
    ekf_params_base = os.path.join(robot_nav_share,      "config", "ekf_fusion_params.yaml")
    ekf_params_lio  = os.path.join(robot_nav_share,      "config", "ekf_fusion_params_lio.yaml")

    args = [
        DeclareLaunchArgument(
            "use_stvl",
            default_value="true",
            description="Use SpatioTemporalVoxelLayer in the local costmap",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map",          default_value="",
                              description="slam_toolbox map file (.posegraph) for localization mode"),
        DeclareLaunchArgument("robot_name",   default_value=""),
        DeclareLaunchArgument("rviz",         default_value="true"),
        DeclareLaunchArgument("params_file",  default_value=default_nav),
        DeclareLaunchArgument("slam_params",  default_value=default_slam),
        DeclareLaunchArgument("slam_mode",    default_value="mapping",
                              description="'mapping' or 'localization'"),
        DeclareLaunchArgument("use_ekf_fusion", default_value="true",
                              description="Fuse /odom_raw via EKF and publish /odom"),
        DeclareLaunchArgument("use_lio", default_value="false",
                              description="Enable DLIO LiDAR-inertial odometry backend"),
        DeclareLaunchArgument("lidar_points_input_topic", default_value="/utlidar/cloud",
                              description="Raw LiDAR pointcloud topic to relay to /lidar/points"),
        DeclareLaunchArgument("lidar_imu_input_topic", default_value="/unilidar/imu",
                              description="Raw LiDAR IMU topic to relay to /lidar/imu"),
        DeclareLaunchArgument("use_safety",   default_value="false",
                              description="Enable velocity_limiter; go2_hw_bridge reads /cmd_vel_safe"),
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

    hw_bridge_cmd_topic = PythonExpression(
        ["'/cmd_vel_safe' if '", use_safety, "' == 'true' else '/cmd_vel_collision_free'"]
    )

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

    go2_hw_bridge = Node(
        package="go2_bringup",
        executable="go2_hw_bridge",
        name="go2_hw_bridge",
        output="screen",
        parameters=[go2_params, {
            "use_sim_time":  use_sim_time,
            "cmd_vel_topic": hw_bridge_cmd_topic,
            # Without EKF: bridge publishes /odom directly.
            # With EKF: bridge publishes /odom_raw and EKF owns /odom.
            "publish_tf": PythonExpression(["'false' if '", use_ekf_fusion, "' == 'true' else 'true'"]),
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
            "lidar_x_in_base":  0.28945,
            "lidar_y_in_base":  0.0,
            "lidar_z_in_base": -0.046825,
            "standing_height":  0.28,
        }],
    )

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

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "ekf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "output_odom_topic": "/odom",
            # LIO mode uses a separate EKF config: velocity-only wheel odom + DLIO position.
            # Non-LIO mode uses the base config: Sport Mode absolute position + body IMU.
            "params_file": PythonExpression([
                f"'{ekf_params_lio}' if '", use_lio, f"' == 'true' else '{ekf_params_base}'"
            ]),
        }.items(),
        condition=IfCondition(use_ekf_fusion),
    )

    # DLIO publishes all-zero covariances; this relay injects diagonal covariances
    # so the EKF applies a proper Kalman gain instead of hard-overwriting state.
    # Only needed (and only running) when use_lio:=true.
    lidar_odom_relay = Node(
        package="robot_nav",
        executable="lidar_odom_relay.py",
        name="lidar_odom_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic":  "/lidar_odom",
            "output_topic": "/lidar_odom_reliable",
            # pose diagonal [x, y, z, roll, pitch, yaw] in m^2 / rad^2
            "pose_cov":  [0.005, 0.005, 0.05, 0.005, 0.005, 0.002],
            # twist diagonal [vx, vy, vz, vroll, vpitch, vyaw]
            "twist_cov": [0.01, 0.01, 0.05, 0.005, 0.005, 0.005],
        }],
        condition=IfCondition(use_lio),
    )

    def maybe_include_lio(context):
        use_lio_enabled = use_lio.perform(context).lower() == "true"
        try:
            robot_lio_share = get_package_share_directory("robot_lio")
        except PackageNotFoundError:
            if use_lio_enabled:
                raise RuntimeError(
                    "use_lio:=true but package 'robot_lio' was not found."
                )
            return [LogInfo(msg="package 'robot_lio' not found; skipping LiDAR relay/LIO include.")]

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
                    # lidar_link->lidar_imu_link is in go2.urdf.xacro; do not republish
                    "publish_lidar_imu_tf": "false",
                }.items(),
            )
        ]

    lio = OpaqueFunction(function=maybe_include_lio)

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
        lidar_odom_relay,
        lio,
        slam,
        nav2,
        rviz2,
    ])
