"""
ekf.launch.py — Optional robot_localization EKF for odometry fusion.

REAL-ROBOT ONLY. Do not include alongside sim_champ.launch.py, which already
spawns its own EKF (footprint_to_odom_ekf) that owns /odom and odom->base_footprint.

LIO frame contract: when use_lio:=true, DLIO publishes /lidar_odom with
child_frame_id: base_link. This EKF is the mandatory mediator that re-publishes
/odom with child_frame_id: base_footprint. Never feed /lidar_odom directly to
Nav2 or slam_toolbox.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_nav_share = get_package_share_directory("robot_nav")
    default_params = os.path.join(robot_nav_share, "config", "ekf_fusion_params.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "output_odom_topic",
            default_value="/odometry/filtered",
            description="EKF output topic (remap to /odom for Nav2)",
        ),
    ]

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("odometry/filtered", LaunchConfiguration("output_odom_topic")),
        ],
    )

    return LaunchDescription(args + [ekf])
