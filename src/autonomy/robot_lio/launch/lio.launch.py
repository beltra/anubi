"""
lio.launch.py -- Robot-agnostic LIO integration wrapper.

Normalizes sensor topics to canonical names (/lidar/points, /lidar/imu),
publishes optional sensor-frame compatibility TFs,
and optionally starts the DLIO odometry node (/lidar_odom).

By default use_lio:=false, so this launch acts as a topic/TF relay only.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    vendor_dlio_pkg = get_package_share_directory("direct_lidar_inertial_odometry")
    dlio_default_params = os.path.join(
        get_package_share_directory("robot_lio"),
        "config",
        "dlio_params.yaml",
    )
    dlio_base_params = os.path.join(vendor_dlio_pkg, "cfg", "dlio.yaml")
    dlio_upstream_params = os.path.join(vendor_dlio_pkg, "cfg", "params.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_lio", default_value="false"),
        DeclareLaunchArgument(
            "start_dlio_map_node",
            default_value="true",
            description="Start DLIO map node to expose /save_pcd map export service",
        ),
        DeclareLaunchArgument("start_lidar_points_relay", default_value="false"),
        DeclareLaunchArgument("start_lidar_imu_relay", default_value="false"),
        DeclareLaunchArgument("lidar_points_input_topic", default_value="/lidar/points"),
        DeclareLaunchArgument("lidar_points_topic", default_value="/lidar/points"),
        DeclareLaunchArgument("lidar_points_frame", default_value="lidar_link"),
        DeclareLaunchArgument(
            "publish_lidar_points_frame_tf",
            default_value="false",
            description="Publish identity lidar_frame -> lidar_points_frame TF for driver frame aliases",
        ),
        DeclareLaunchArgument("lidar_imu_input_topic", default_value="/lidar/imu"),
        DeclareLaunchArgument("lidar_imu_topic", default_value="/lidar/imu"),
        DeclareLaunchArgument("lidar_odom_topic", default_value="/lidar_odom"),
        DeclareLaunchArgument(
            "dlio_keyframes_topic",
            default_value="/kf_cloud",
            description="Keyframe cloud topic consumed by dlio_map_node",
        ),
        DeclareLaunchArgument("dlio_publish_tf", default_value="false"),
        DeclareLaunchArgument("publish_lidar_imu_tf", default_value="true"),
        DeclareLaunchArgument("lidar_frame", default_value="lidar_link"),
        DeclareLaunchArgument("lidar_imu_frame", default_value="lidar_imu_link"),
        # UniLidar SDK IMU offset within the LiDAR frame [m]
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
    dlio_keyframes_topic = LaunchConfiguration("dlio_keyframes_topic")

    pointcloud_relay = Node(
        package="topic_tools",
        executable="relay",
        name="lidar_points_relay",
        output="screen",
        arguments=[lidar_points_input_topic, lidar_points_topic],
        condition=IfCondition(LaunchConfiguration("start_lidar_points_relay")),
    )

    lidar_points_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_points_frame_alias_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", LaunchConfiguration("lidar_frame"),
            "--child-frame-id", LaunchConfiguration("lidar_points_frame"),
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("publish_lidar_points_frame_tf"),
                "' == 'true' and '", LaunchConfiguration("lidar_points_frame"),
                "' != '' and '", LaunchConfiguration("lidar_points_frame"),
                "' != '", LaunchConfiguration("lidar_frame"), "'",
            ])
        ),
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

    # DLIO publishes odometry on /lidar_odom; TF publication is disabled to keep
    # EKF as the single owner of odom->base_footprint.
    dlio_odom_node = Node(
        package="direct_lidar_inertial_odometry",
        executable="dlio_odom_node",
        name="dlio_odom_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            dlio_base_params,
            dlio_upstream_params,
            LaunchConfiguration("dlio_params_file"),
            {"use_sim_time": use_sim_time},
            {"publish_tf": LaunchConfiguration("dlio_publish_tf")},
        ],
        remappings=[
            ("pointcloud", lidar_points_topic),
            ("imu", lidar_imu_topic),
            ("odom", lidar_odom_topic),
            # GICP-registered per-scan cloud in the odom frame: each scan is
            # placed by the same registration that builds the DLIO map, so
            # consumers (G1 mapping scan) inherit DLIO-map-coherent geometry.
            ("deskewed", "/lidar/points_registered"),
        ],
        condition=IfCondition(LaunchConfiguration("use_lio")),
    )

    dlio_map_node = Node(
        package="direct_lidar_inertial_odometry",
        executable="dlio_map_node",
        name="dlio_map_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            dlio_base_params,
            dlio_upstream_params,
            LaunchConfiguration("dlio_params_file"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("keyframes", dlio_keyframes_topic),
            # dlio_map_node's "map" output is a PointCloud2; without this
            # remap it collides with slam_toolbox's /map OccupancyGrid
            # whenever both run together.
            ("map", "/dlio/map_cloud"),
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("use_lio"), "' == 'true' and '",
                LaunchConfiguration("start_dlio_map_node"),
                "' == 'true'",
            ])
        ),
    )

    return LaunchDescription(args + [
        pointcloud_relay,
        lidar_points_frame_tf,
        imu_relay,
        lidar_imu_tf,
        dlio_odom_node,
        dlio_map_node,
    ])
