"""
nav_realrobot.launch.py -- Full navigation stack for the real robot.

Starts: ekf.launch.py -> slam.launch.py -> nav2.launch.py

Prerequisites (launched separately):
  - go2_bringup: hardware drivers, raw odom, IMU, joint states, base TF tree
  - LiDAR relay: /utlidar/cloud -> /lidar/points
  - velocity_limiter (from robot_safety or go2_bringup)

Arguments
---------
use_sim_time : bool   (default: false)
slam_mode    : str    'mapping' (default) | 'localization'
map          : str    Path to .posegraph file (required for localization mode)
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
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_mode", default_value="mapping",
                              description="'mapping' or 'localization'"),
        DeclareLaunchArgument("map", default_value="",
                              description="Path to map file (required if slam_mode:=localization)"),
    ]

    slam_mode = LaunchConfiguration("slam_mode")
    map_path  = LaunchConfiguration("map")

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "ekf.launch.py")
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "slam_mode":    slam_mode,
            "map":          map_path,
            "accumulate_scan_cloud": "true",
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            # Keep ekf.launch.py's params_file argument from leaking into Nav2.
            "params_file": "",
        }.items(),
    )

    return LaunchDescription(args + [ekf, slam, nav2])
