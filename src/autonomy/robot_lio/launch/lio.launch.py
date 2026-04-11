"""
lio.launch.py
Robot-agnostic LIO integration wrapper.

Responsibilities:
  1. Normalize sensor topics to canonical names:
       - /lidar/points
       - /lidar/imu
  2. Publish static lidar_link -> lidar_imu_link transform.
  3. Optionally start DLIO ROS2 odometry and expose /lidar_odom.

By default use_lio:=false, so this launch acts as a topic/TF contract provider
without changing active odometry ownership.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dlio_default_params = os.path.join(
        get_package_share_directory("robot_lio"),
        "config",
        "dlio_params.yaml",
    )

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_lio", default_value="false"),
        DeclareLaunchArgument("start_lidar_points_relay", default_value="false"),
        DeclareLaunchArgument("start_lidar_imu_relay", default_value="false"),
        DeclareLaunchArgument("lidar_points_input_topic", default_value="/lidar/points"),
        DeclareLaunchArgument("lidar_points_topic", default_value="/lidar/points"),
        DeclareLaunchArgument("lidar_imu_input_topic", default_value="/lidar/imu"),
        DeclareLaunchArgument("lidar_imu_topic", default_value="/lidar/imu"),
        DeclareLaunchArgument("lidar_odom_topic", default_value="/lidar_odom"),
        DeclareLaunchArgument("dlio_publish_tf", default_value="false"),
        DeclareLaunchArgument("publish_lidar_imu_tf", default_value="true"),
        DeclareLaunchArgument("lidar_frame", default_value="lidar_link"),
        DeclareLaunchArgument("lidar_imu_frame", default_value="lidar_imu_link"),
        # Unitree UniLidar SDK: IMU origin in LiDAR frame (meters).
        DeclareLaunchArgument("lidar_imu_x", default_value="-0.007698"),
        DeclareLaunchArgument("lidar_imu_y", default_value="-0.014655"),
        DeclareLaunchArgument("lidar_imu_z", default_value="0.00667"),
        DeclareLaunchArgument("lidar_imu_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_imu_yaw", default_value="0.0"),
        DeclareLaunchArgument("dlio_params_file", default_value=dlio_default_params),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    lidar_points_input_topic = LaunchConfiguration("lidar_points_input_topic")
    lidar_points_topic = LaunchConfiguration("lidar_points_topic")
    lidar_imu_input_topic = LaunchConfiguration("lidar_imu_input_topic")
    lidar_imu_topic = LaunchConfiguration("lidar_imu_topic")
    lidar_odom_topic = LaunchConfiguration("lidar_odom_topic")

    pointcloud_relay = Node(
        package="topic_tools",
        executable="relay",
        name="lidar_points_relay",
        output="screen",
        arguments=[lidar_points_input_topic, lidar_points_topic],
        condition=IfCondition(LaunchConfiguration("start_lidar_points_relay")),
    )

    imu_relay = Node(
        package="topic_tools",
        executable="relay",
        name="lidar_imu_relay",
        output="screen",
        arguments=[lidar_imu_input_topic, lidar_imu_topic],
        condition=IfCondition(LaunchConfiguration("start_lidar_imu_relay")),
    )

    lidar_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_to_lidar_imu_tf",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("lidar_imu_x"),
            "--y", LaunchConfiguration("lidar_imu_y"),
            "--z", LaunchConfiguration("lidar_imu_z"),
            "--roll", LaunchConfiguration("lidar_imu_roll"),
            "--pitch", LaunchConfiguration("lidar_imu_pitch"),
            "--yaw", LaunchConfiguration("lidar_imu_yaw"),
            "--frame-id", LaunchConfiguration("lidar_frame"),
            "--child-frame-id", LaunchConfiguration("lidar_imu_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_lidar_imu_tf")),
    )

    # Keep DLIO isolated from Nav2 TF ownership by publishing odometry on
    # /lidar_odom and disabling TF unless explicitly requested.
    dlio_odom_node = Node(
        package="direct_lidar_inertial_odometry",
        executable="dlio_odom_node",
        name="dlio_odom_node",
        output="screen",
        parameters=[
            LaunchConfiguration("dlio_params_file"),
            {"use_sim_time": use_sim_time},
            {"publish_tf": LaunchConfiguration("dlio_publish_tf")},
        ],
        remappings=[
            ("pointcloud", lidar_points_topic),
            ("imu", lidar_imu_topic),
            ("odom", lidar_odom_topic),
        ],
        condition=IfCondition(LaunchConfiguration("use_lio")),
    )

    return LaunchDescription(args + [
        pointcloud_relay,
        imu_relay,
        lidar_imu_tf,
        dlio_odom_node,
    ])
