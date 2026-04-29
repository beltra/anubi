"""
dynamic_obstacles.launch.py -- Predictive LiDAR dynamic obstacle pipeline.

Topics:
  /lidar/points_undistorted -> preprocessor -> /dynamic_obstacles/points_world
  /dynamic_obstacles/points_world -> detector -> /dynamic_obstacles/clusters
  /dynamic_obstacles/clusters -> tracker -> /dynamic_obstacles/tracks
  /dynamic_obstacles/tracks -> bridge -> /dynamic_obstacles/predicted_occupancy
                                      and /dynamic_obstacles/markers for RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_nav_share = get_package_share_directory("robot_nav")
    default_params = os.path.join(robot_nav_share, "config", "dynamic_obstacles_sim.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("dynamic_obstacles_params_file", default_value=default_params),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("dynamic_obstacles_params_file")
    common_params = [params_file, {"use_sim_time": use_sim_time}]

    preprocessor = Node(
        package="robot_nav",
        executable="dynamic_obstacle_preprocessor",
        name="dynamic_obstacle_preprocessor",
        output="screen",
        parameters=common_params,
    )
    detector = Node(
        package="robot_nav",
        executable="dynamic_cluster_detector",
        name="dynamic_cluster_detector",
        output="screen",
        parameters=common_params,
    )
    tracker = Node(
        package="robot_nav",
        executable="dynamic_obstacle_tracker",
        name="dynamic_obstacle_tracker",
        output="screen",
        parameters=common_params,
    )
    bridge = Node(
        package="robot_nav",
        executable="dynamic_obstacle_costmap_bridge",
        name="dynamic_obstacle_costmap_bridge",
        output="screen",
        parameters=common_params,
    )

    return LaunchDescription(args + [preprocessor, detector, tracker, bridge])
