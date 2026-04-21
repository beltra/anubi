"""
sim_bringup.launch.py
Full Gazebo Fortress + CHAMP + Nav2 simulation bringup for Go2.

What this launch file starts
-----------------------------
  1. go2_sim/sim_champ.launch.py
       - Ignition Gazebo (Fortress)
       - ros_ign bridges
       - robot_state_publisher
       - CHAMP controller + EKF odometry (odom -> base_footprint)
  2. cloud_self_filter.py
       - /lidar/points -> /lidar/points_undistorted for SLAM / costmaps in sim
       - optional debug cloud on /debug/lidar/points_filtered
  3. robot_nav/slam.launch.py (optional via start_nav:=true)
  4. robot_nav/nav2.launch.py (optional via start_nav:=true)
  5. RViz2 (optional)

Key integration rule
--------------------
sim_champ.launch.py publishes a fallback static map->odom transform by default.
For SLAM/Nav2 bringup we disable that fallback so slam_toolbox is the sole
publisher of map->odom.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_sim_share = get_package_share_directory("go2_sim")
    sim_worlds_share = get_package_share_directory("sim_worlds")
    robot_nav_share = get_package_share_directory("robot_nav")

    default_world = os.path.join(sim_worlds_share, "worlds", "indoor_office.world")
    default_ros_control = os.path.join(go2_sim_share, "config", "ros_control.yaml")
    default_slam_params = os.path.join(robot_nav_share, "config", "slam_toolbox_params.yaml")
    default_rviz_config = os.path.join(robot_nav_share, "rviz", "nav2.rviz")

    args = [
        DeclareLaunchArgument(
            "use_stvl",
            default_value="true",
            description="Use SpatioTemporalVoxelLayer for 3D obstacle avoidance"
                        " in the local costmap. Set false to revert to 2D obstacle layer.",
        ),
        DeclareLaunchArgument(
            "nav_profile",
            default_value="default_safe",
            description="Nav2 profile: default_safe or narrow_gap_1p0m",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument(
            "world_name",
            default_value="indoor_office",
            description="Gazebo world name used in transport topic paths",
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("robot_name", default_value="go2"),
        DeclareLaunchArgument("ros_control_file", default_value=default_ros_control),
        DeclareLaunchArgument("world_init_x", default_value="0.0"),
        DeclareLaunchArgument("world_init_y", default_value="0.0"),
        DeclareLaunchArgument("world_init_z", default_value="0.375"),
        DeclareLaunchArgument("world_init_heading", default_value="0.0"),
        DeclareLaunchArgument("publish_map_to_odom_tf", default_value="false"),
        DeclareLaunchArgument("use_lio", default_value="false"),
        DeclareLaunchArgument(
            "lio_start_before_nav_sec",
            default_value="4.0",
            description=(
                "Start DLIO this many seconds before nav_delay_sec. This keeps "
                "LIO after CHAMP startup while still starting before the nav stack."
            ),
        ),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument(
            "nav_delay_sec",
            default_value="20.0",
            description=(
                "Delay before starting SLAM. Must exceed the CHAMP controller "
                "spawn and stand-up time so perception starts from the settled "
                "standing pose."
            ),
        ),
        DeclareLaunchArgument("slam_mode", default_value="mapping"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("slam_params", default_value=default_slam_params),
        DeclareLaunchArgument("launch_cloud_filter", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
    ]

    start_nav = LaunchConfiguration("start_nav")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_stvl = LaunchConfiguration("use_stvl")
    nav_profile = LaunchConfiguration("nav_profile")
    use_lio = LaunchConfiguration("use_lio")

    champ_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_sim_share, "launch", "sim_champ.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_lio": use_lio,
            "rviz": "false",
            "robot_name": LaunchConfiguration("robot_name"),
            "ros_control_file": LaunchConfiguration("ros_control_file"),
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "gui": LaunchConfiguration("gui"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "publish_map_to_odom_tf": LaunchConfiguration("publish_map_to_odom_tf"),
        }.items(),
    )

    def maybe_include_lio(context):
        if use_lio.perform(context).lower() != "true":
            return []

        nav_delay_sec = float(LaunchConfiguration("nav_delay_sec").perform(context))
        lio_start_before_nav_sec = float(
            LaunchConfiguration("lio_start_before_nav_sec").perform(context)
        )
        lio_start_delay_sec = max(0.0, nav_delay_sec - lio_start_before_nav_sec)

        try:
            robot_lio_share = get_package_share_directory("robot_lio")
        except PackageNotFoundError:
            raise RuntimeError(
                "use_lio:=true but package 'robot_lio' was not found in the workspace/install."
            )

        dlio_sim_params = os.path.join(robot_lio_share, "config", "dlio_params_sim.yaml")

        return [
            TimerAction(
                period=lio_start_delay_sec,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(robot_lio_share, "launch", "lio.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "use_lio": use_lio,
                            "start_lidar_points_relay": "false",
                            # Simulation already publishes /lidar/imu from lidar_imu_link.
                            "start_lidar_imu_relay": "false",
                            "publish_lidar_imu_tf": "false",
                            # Simulation uses a different LiDAR mount rotation from the
                            # hardware URDF; DLIO must consume matching extrinsics.
                            "dlio_params_file": dlio_sim_params,
                        }.items(),
                    )
                ],
            )
        ]

    lio = OpaqueFunction(function=maybe_include_lio)

    # Self-body filter — simulation only.
    # The real Unitree L1 has a hardware minimum range of 0.8 m. The Go2's body
    # and legs are always within ~0.4 m of the sensor, so hardware prevents
    # self-body returns entirely. Gazebo uses range_min=0.1 m for simulation
    # stability, so body returns appear and must be removed in software.
    nav_cloud_filter = Node(
        package="robot_nav",
        executable="cloud_self_filter.py",
        name="nav_cloud_filter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/lidar/points",
            "output_topic": "/lidar/points_undistorted",
            # Sim sensor is mounted right-side-up (rpy=0,-0.262,0) to work
            # around a Fortress gpu_lidar rendering bug that drops most rays
            # when the parent link has Rx(π) roll.  The real L1 uses
            # sensor_mount_rpy=[π,-0.262,0] (see distortion_corrector.yaml).
            "sensor_mount_rpy": [0.0, -0.26179938779915, 0.0],
            "gravity_align": True,
            "gravity_align_frame": "lidar_stabilized",
        }],
        condition=IfCondition(start_nav),
    )

    debug_cloud_filter = Node(
        package="robot_nav",
        executable="cloud_self_filter.py",
        name="cloud_self_filter_debug",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/lidar/points",
            "output_topic": "/debug/lidar/points_filtered",
        }],
        condition=IfCondition(LaunchConfiguration("launch_cloud_filter")),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "slam.launch.py")
        ),
        condition=IfCondition(start_nav),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_mode": LaunchConfiguration("slam_mode"),
            "slam_params": LaunchConfiguration("slam_params"),
            "map": LaunchConfiguration("map"),
            # Gazebo produces instantaneous LiDAR snapshots, so bypass deskew
            # and use the self-filtered nav cloud directly in simulation.
            "bypass_distortion_corrector": "true",
            "scan_cloud_topic": "/lidar/points_undistorted",
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "nav2.launch.py")
        ),
        condition=IfCondition(start_nav),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_stvl": use_stvl,
            "nav_profile": nav_profile,
        }.items(),
    )

    # SLAM starts only after CHAMP has had time to spawn both controllers
    # (20 s + 30 s in sim_champ.launch.py) and settle into its standing pose.
    # Starting scan processing before then pollutes SLAM and the downstream
    # obstacle stack with observations from the collapsed/spawning posture.
    # Nav2 starts 10 s after SLAM so slam_toolbox can publish map→odom first.
    # Total Nav2 start offset = nav_delay_sec + 10 s.
    start_nav_stack = TimerAction(
        period=LaunchConfiguration("nav_delay_sec"),
        actions=[
            slam,
            TimerAction(period=10.0, actions=[nav2]),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(args + [
        champ_sim,
        lio,
        nav_cloud_filter,
        debug_cloud_filter,
        start_nav_stack,
        rviz,
    ])
