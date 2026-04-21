"""
ekf.launch.py
Optional robot_localization EKF launch for odometry fusion.

This launch is not included by default in go2_bringup/robot_sim bringups.
Use it explicitly when raw odometry, IMU, and optional lidar odometry topics
are available and TF ownership is configured to avoid duplicates.

REAL-ROBOT ONLY - do NOT include this launch alongside sim_champ.launch.py.
sim_champ.launch.py already spawns its own EKF node (footprint_to_odom_ekf)
that publishes /odom and the odom->base_footprint TF. Launching ekf.launch.py
at the same time would create two competing /odom publishers and two
odom->base_footprint TF broadcasters, causing TF conflicts and unpredictable
odometry in Nav2. This file is intended for real-robot bringup only.

LIO frame contract: when use_lio:=true, the EKF consumes /lidar_odom from DLIO.
DLIO's dlio_params.yaml sets frames/baselink=base_link, so /lidar_odom has
child_frame_id: base_link, not base_footprint. The EKF is the mandatory
mediator: it fuses /lidar_odom and re-publishes /odom with child_frame_id:
base_footprint. Never bypass the EKF by feeding /lidar_odom directly to Nav2
or slam_toolbox.
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
            description="EKF odometry output topic (remap to /odom for Nav2 consumption)",
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
