"""
nav_realrobot.launch.py
Full navigation stack launch for a real robot.

Starts (in order):
  1. ekf.launch.py     — robot_localization EKF: fuses raw odometry + IMU
                         → publishes /odom and odom→base_footprint TF
  2. slam.launch.py    — distortion_corrector + pointcloud_to_laserscan + slam_toolbox
                         → publishes /map, /scan, and map→odom TF
  3. nav2.launch.py    — Nav2 navigation stack (planner, controller, bt, etc.)

Operator MUST launch the following SEPARATELY before calling this file:
  • go2_bringup real-robot bringup (go2_bringup package)
      Provides: hardware drivers, raw odometry, IMU, joint states, base TF tree
  • A /lidar/points republisher that remaps /utlidar/cloud → /lidar/points
      Required so slam.launch.py (and STVL local costmap) receive a
      canonically named PointCloud2 on /lidar/points
  • velocity_limiter (from robot_safety or go2_bringup)
      Applies safety speed caps before commands reach the hardware

This file must NOT be used alongside sim_champ.launch.py or any Gazebo/Ignition
simulation bringup: it includes ekf.launch.py (real-robot EKF), which would
conflict with the inline EKF spawned by sim_champ.launch.py.

Arguments
---------
use_sim_time : bool   (default: false) — must remain false for real-robot use
slam_mode    : str    'mapping' (default) | 'localization'
map          : str    Path to .posegraph file (required if slam_mode:=localization)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    robot_nav_share = get_package_share_directory("robot_nav")

    args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock — must be false for real-robot use",
        ),
        DeclareLaunchArgument(
            "slam_mode",
            default_value="mapping",
            description="SLAM mode: 'mapping' or 'localization'",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Path to map file (required if slam_mode:=localization)",
        ),
    ]

    slam_mode = LaunchConfiguration("slam_mode")
    map_path  = LaunchConfiguration("map")

    # ── 1. EKF odometry fusion ────────────────────────────────────────────────
    # Real-robot only: fuses raw wheel odometry + IMU into a filtered /odom.
    # use_sim_time is hard-coded false — this launch file targets real hardware.
    # Do NOT combine with sim_champ.launch.py (it has its own inline EKF).
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "ekf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    # ── 2. SLAM pipeline ──────────────────────────────────────────────────────
    # distortion_corrector deskews /lidar/points → /lidar/points_undistorted.
    # pointcloud_to_laserscan then converts /lidar/points_undistorted → /scan.
    # slam_toolbox runs in mapping or localization mode depending on slam_mode.
    # use_sim_time is hard-coded false — real-robot only.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "slam_mode":    slam_mode,
            "map":          map_path,
        }.items(),
    )

    # ── 3. Nav2 navigation stack ──────────────────────────────────────────────
    # Delegates to nav2_bringup/navigation_launch.py.
    # Localisation (map→odom) is provided by slam_toolbox above.
    # Odometry (odom→base_footprint) is provided by the EKF above.
    # use_sim_time is hard-coded false — real-robot only.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription(args + [
        ekf,
        slam,
        nav2,
    ])
