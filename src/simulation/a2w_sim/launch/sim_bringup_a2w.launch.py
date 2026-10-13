"""
sim_bringup_a2w.launch.py - Full Gazebo Fortress + Nav2 simulation bringup for the
Unitree A2-W (Pro) wheeled quadruped.

Wheeled analogue of g1_sim/sim_bringup_g1.launch.py. Starts:
  1. a2w_sim/sim_a2w.launch.py   (Gazebo, sliding base, bridges, cloud_merger, EKF, TF)
  2. distortion_corrector        (/lidar/points -> /lidar/points_undistorted)
  3. robot_lio (DLIO)            (use_lio:=true, default: /lidar_odom -> EKF)
  4. robot_nav/slam.launch.py    (slam_toolbox owns map->odom)
  5. robot_nav/nav2.launch.py    (nav2_params_a2w.yaml + collision_monitor_params_a2w.yaml)
  6. RViz2                       (optional)

The shared autonomy packages are reused unchanged; the A2-W inputs are the sim
model, cloud_merger, and the *_a2w.yaml configs.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    a2w_sim_share = get_package_share_directory("a2w_sim")
    sim_worlds_share = get_package_share_directory("sim_worlds")
    robot_nav_share = get_package_share_directory("robot_nav")

    default_world = os.path.join(sim_worlds_share, "worlds", "indoor_office.world")
    default_slam_params = os.path.join(robot_nav_share, "config", "slam_toolbox_params_a2w.yaml")
    default_rviz_config = os.path.join(robot_nav_share, "rviz", "nav2.rviz")
    nav2_params = os.path.join(robot_nav_share, "config", "nav2_params_a2w.yaml")
    collision_params = os.path.join(robot_nav_share, "config", "collision_monitor_params_a2w.yaml")
    scan_params = os.path.join(robot_nav_share, "config", "scan_from_cloud_a2w.yaml")
    # Shared with the real bringup so the self-filter box and sensor geometry
    # cannot drift between sim and real.
    distortion_params = os.path.join(robot_nav_share, "config", "distortion_corrector_a2w.yaml")

    args = [
        DeclareLaunchArgument("use_stvl", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("world_name", default_value="indoor_office"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("robot_name", default_value="a2w"),
        DeclareLaunchArgument("world_init_x", default_value="0.0"),
        DeclareLaunchArgument("world_init_y", default_value="0.0"),
        DeclareLaunchArgument("world_init_z", default_value="0.497"),
        DeclareLaunchArgument("world_init_heading", default_value="0.0"),
        DeclareLaunchArgument("publish_map_to_odom_tf", default_value="false"),
        DeclareLaunchArgument("lidar_channels", default_value="64"),
        DeclareLaunchArgument("lidar_h_samples", default_value="180"),
        DeclareLaunchArgument("physics_step_sec", default_value="0.004"),
        DeclareLaunchArgument(
            "use_lio", default_value="true",
            description="DLIO on the merged cloud + body IMU drives the EKF (as on the "
                        "real robot). false = Gazebo ground-truth odometry + EKF.",
        ),
        DeclareLaunchArgument("lio_start_before_nav_sec", default_value="4.0"),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument(
            "nav_delay_sec", default_value="10.0",
            description="Delay before SLAM starts; covers Gazebo + bridge + TF settle and "
                        "DLIO's stationary IMU calibration. Nav2 starts 10 s after SLAM.",
        ),
        DeclareLaunchArgument("slam_mode", default_value="mapping"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("slam_params", default_value=default_slam_params),
        DeclareLaunchArgument("distortion_imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
    ]

    start_nav = LaunchConfiguration("start_nav")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_lio = LaunchConfiguration("use_lio")

    a2w_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(a2w_sim_share, "launch", "sim_a2w.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_lio": use_lio,
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "gui": LaunchConfiguration("gui"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "lidar_channels": LaunchConfiguration("lidar_channels"),
            "lidar_h_samples": LaunchConfiguration("lidar_h_samples"),
            "physics_step_sec": LaunchConfiguration("physics_step_sec"),
            "publish_map_to_odom_tf": LaunchConfiguration("publish_map_to_odom_tf"),
        }.items(),
    )

    def maybe_include_lio(context):
        if use_lio.perform(context).lower() != "true":
            return []
        nav_delay_sec = float(LaunchConfiguration("nav_delay_sec").perform(context))
        lead_sec = float(LaunchConfiguration("lio_start_before_nav_sec").perform(context))
        try:
            robot_lio_share = get_package_share_directory("robot_lio")
        except PackageNotFoundError:
            raise RuntimeError("use_lio:=true but package 'robot_lio' was not found.")
        return [TimerAction(
            period=max(0.0, nav_delay_sec - lead_sec),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_lio_share, "launch", "lio.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_lio": use_lio,
                    "start_lidar_points_relay": "false",
                    "start_lidar_imu_relay": "false",
                    "publish_lidar_imu_tf": "false",
                    # Two JT128 IMUs would need their own fusion; the body IMU is
                    # the single inertial reference, as in the EKF.
                    "lidar_imu_topic": "/imu/data",
                    "dlio_params_file": os.path.join(
                        robot_lio_share, "config", "dlio_params_a2w_sim.yaml"),
                }.items(),
            )],
        )]

    lio = OpaqueFunction(function=maybe_include_lio)

    # The sliding sim base never tilts and its IMU cannot sense gravity reliably
    # (VelocityControl is kinematic), so leveling is turned off here; the real
    # robot keeps it on through the shared YAML.
    lidar_cloud_processor = Node(
        package="robot_nav",
        executable="distortion_corrector",
        name="distortion_corrector",
        output="screen",
        parameters=[
            distortion_params,
            {
                "use_sim_time": use_sim_time,
                "imu_topic": LaunchConfiguration("distortion_imu_topic"),
                "use_imu": False,
                "use_3d_distortion_correction": False,
                "gravity_align": False,
                "deskew_enabled": False,
            },
        ],
        condition=IfCondition(start_nav),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_nav_share, "launch", "slam.launch.py")),
        condition=IfCondition(start_nav),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_mode": LaunchConfiguration("slam_mode"),
            "slam_params": LaunchConfiguration("slam_params"),
            "map": LaunchConfiguration("map"),
            "bypass_distortion_corrector": "true",
            "scan_cloud_topic": "/lidar/points_undistorted",
            "scan_params": scan_params,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_nav_share, "launch", "nav2.launch.py")),
        condition=IfCondition(start_nav),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_stvl": LaunchConfiguration("use_stvl"),
            "params_file": nav2_params,
            "collision_monitor_params": collision_params,
        }.items(),
    )

    start_nav_stack = TimerAction(
        period=LaunchConfiguration("nav_delay_sec"),
        actions=[slam, TimerAction(period=10.0, actions=[nav2])],
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

    return LaunchDescription(args + [a2w_sim, lio, lidar_cloud_processor, start_nav_stack, rviz])
