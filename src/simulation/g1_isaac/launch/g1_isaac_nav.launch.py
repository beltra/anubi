"""
g1_isaac_nav.launch.py -- anubi autonomy stack (EKF + SLAM + Nav2) against the
Isaac Sim G1 adapter.

Companion to g1_isaac_bringup.launch.py (which must be running: it provides the
canonical /lidar/points, /lidar/imu, /imu/data, /odom_raw relays and the static
base_footprint -> base_link -> lidar_link TFs). This launch adds the
robot-agnostic autonomy on top, mirroring sim_bringup_g1.launch.py (the Gazebo
G1 sim) node for node:

  /odom_raw + /imu/data -> footprint_to_odom_ekf -> /odom + odom->base_footprint
  /imu/data -> lidar_level_frame_publisher -> base_footprint->lidar_stabilized
  /lidar/points -> distortion_corrector (sim overrides) -> /lidar/points_undistorted
     -> pointcloud_to_laserscan -> /scan -> slam_toolbox (map->odom, /map)
  Nav2 (nav2_params_g1_sonic_slow.yaml, STVL on /lidar/points_undistorted)
     + collision_monitor (sim-scoped source_timeout, see config)

Sim specifics:
  - distortion corrector: IMU leveling / 3D correction / gravity alignment /
    deskew all OFF (same overrides as the Gazebo sim): Isaac RTX clouds are
    single-pose snapshots, and the SONIC-stabilized head makes the static mount
    righting sufficient. The shared distortion_corrector_g1.yaml still supplies
    the body-box self-filter and sensor geometry.
  - EKF: the ground-truth /odom_raw variant of sim_g1.launch.py's
    footprint_to_odom_ekf (absolute x/y/yaw + velocities from /odom_raw, vyaw
    from /imu/data).
  - The cmd_vel_to_sonic bridge is deliberately NOT started here: it starts the
    SONIC policy the moment the node comes up, and its graceful shutdown sends
    the SONIC stop command (stopping the whole deploy). The test harness manages
    its lifecycle separately (pause with SIGKILL only), using
    config/g1_sonic_sim_params.yaml.

Arguments
---------
use_sim_time    : bool  (default: true)   Isaac host is the /clock source
slam_mode       : str   'mapping' (default) | 'localization'
map             : str   .posegraph for localization mode
nav_delay_sec   : float delay before SLAM starts (default 8.0)
nav2_after_slam_sec : float Nav2 start delay after SLAM (default 10.0)
standing_height : float base_footprint->base_link z, must match the bringup (0.80)
use_stvl        : bool  (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    g1_isaac_share = get_package_share_directory("g1_isaac")
    robot_nav_share = get_package_share_directory("robot_nav")

    # Sim-scoped copy: transform_timeout 60 s future-dates map->odom past the
    # RTX annotator's multi-second scan dead phases (see the file header).
    slam_params = os.path.join(g1_isaac_share, "config", "slam_toolbox_params_g1_isaac.yaml")
    # Sim-scoped copy: floor margin raised + range cut so gait-pitch floor
    # returns are not painted as obstacles (see the file header).
    scan_params = os.path.join(g1_isaac_share, "config", "scan_from_cloud_g1_isaac.yaml")
    distortion_params = os.path.join(robot_nav_share, "config", "distortion_corrector_g1.yaml")
    nav2_params = os.path.join(robot_nav_share, "config", "nav2_params_g1_sonic_slow.yaml")
    collision_params = os.path.join(
        g1_isaac_share, "config", "collision_monitor_params_g1_isaac.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_mode", default_value="mapping"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("nav_delay_sec", default_value="8.0"),
        DeclareLaunchArgument("nav2_after_slam_sec", default_value="10.0"),
        DeclareLaunchArgument("standing_height", default_value="0.80"),
        DeclareLaunchArgument("use_stvl", default_value="true"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    standing_height = LaunchConfiguration("standing_height")

    # EKF on ground-truth body odometry (sim_g1.launch.py's non-LIO variant):
    # absolute x/y/yaw + vx/vy/vyaw from /odom_raw, vyaw from /imu/data.
    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"base_link_frame": "base_footprint"},
            {"odom_frame": "odom"},
            {"world_frame": "odom"},
            {"publish_tf": True},
            {"frequency": 50.0},
            {"two_d_mode": True},
            {"odom0": "/odom_raw"},
            {"odom0_config": [True, True, False,
                              False, False, True,
                              True, True, False,
                              False, False, True,
                              False, False, False]},
            {"odom0_differential": False},
            {"odom0_relative": False},
            {"odom0_queue_size": 10},
            {"imu0": "imu/data"},
            {"imu0_config": [False, False, False,
                             False, False, False,
                             False, False, False,
                             False, False, True,
                             False, False, False]},
            {"imu0_remove_gravitational_acceleration": True},
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # Gravity-levelled LiDAR frame (target frame of the /scan slice). The Isaac
    # mount values match g1_isaac_bringup.launch.py's base_link -> lidar_link TF.
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
            "lidar_x_in_base": -0.00368,
            "lidar_y_in_base": 0.00003,
            "lidar_z_in_base": 0.46018,
            "standing_height": ParameterValue(standing_height, value_type=float),
        }],
    )

    # Shared G1 body box / sensor geometry; sim leveling overrides as in the
    # Gazebo sim (single-pose RTX snapshots, no pelvis-IMU leveling).
    distortion_corrector = Node(
        package="robot_nav",
        executable="distortion_corrector",
        name="distortion_corrector",
        output="screen",
        parameters=[
            distortion_params,
            {
                "use_sim_time": use_sim_time,
                "use_imu": False,
                "use_3d_distortion_correction": False,
                "gravity_align": False,
                "deskew_enabled": False,
            },
        ],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_mode": LaunchConfiguration("slam_mode"),
            "map": LaunchConfiguration("map"),
            "slam_params": slam_params,
            "scan_params": scan_params,
            # Corrector runs above with sim overrides; slam.launch.py must not
            # start a second one.
            "bypass_distortion_corrector": "true",
            "scan_cloud_topic": "/lidar/points_undistorted",
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_stvl": LaunchConfiguration("use_stvl"),
            "params_file": nav2_params,
            "collision_monitor_params": collision_params,
        }.items(),
    )

    start_nav_stack = TimerAction(
        period=LaunchConfiguration("nav_delay_sec"),
        actions=[
            slam,
            TimerAction(
                period=LaunchConfiguration("nav2_after_slam_sec"),
                actions=[nav2],
            ),
        ],
    )

    return LaunchDescription(args + [
        footprint_to_odom_ekf,
        lidar_level_frame_publisher,
        distortion_corrector,
        start_nav_stack,
    ])
