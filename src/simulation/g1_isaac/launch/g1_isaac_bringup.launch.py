"""
g1_isaac_bringup.launch.py -- Isaac Sim G1 adapter bringup.

Maps the Isaac host topics (isaac_dds_host.py, ROS_DOMAIN_ID 42, sim time)
onto the canonical anubi contract from docs/topic_map.md:

  /livox/lidar   -> /lidar/points  (frame -> lidar_link, RELIABLE)
  /livox/imu_raw -> /lidar/imu     (frame -> lidar_link, RELIABLE)

and publishes the static TFs this bringup owns. No URDF /
robot_state_publisher runs against the Isaac host, so the sensor mount TF
lives here instead of in a description package. The Mid-360 is mounted
UPRIGHT (no roll): the host cloud is already in the righted production
convention (elevation span -52..+7 deg).

Host-fed relays (host publishers live: isaac_dds_host.py --publish-odom-gt +
livox_zmq_republisher --pelvis-imu-topic /pelvis/imu_raw):
  enable_body_imu : /pelvis/imu_raw -> /imu/data   (default true)
  enable_odom_raw : /odom_gt        -> /odom_raw   (default true)

Everything runs with use_sim_time:=true by default; the Isaac host is the
/clock source.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # Live Isaac host inputs.
        DeclareLaunchArgument("lidar_points_input_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("lidar_points_topic", default_value="/lidar/points"),
        DeclareLaunchArgument("lidar_imu_input_topic", default_value="/livox/imu_raw"),
        DeclareLaunchArgument("lidar_imu_topic", default_value="/lidar/imu"),
        DeclareLaunchArgument("lidar_frame", default_value="lidar_link"),
        # Host support live: livox_zmq_republisher --pelvis-imu-topic
        # /pelvis/imu_raw synthesizes the pelvis IMU from the host odom stream.
        DeclareLaunchArgument(
            "enable_body_imu",
            default_value="true",
            description="Relay the pelvis IMU to /imu/data",
        ),
        DeclareLaunchArgument("body_imu_input_topic", default_value="/pelvis/imu_raw"),
        DeclareLaunchArgument("body_imu_topic", default_value="/imu/data"),
        # DELIBERATELY exposed, do not fold into a constant: the sim contract
        # uses imu_link (docs/topic_map.md) but the real G1 publishes frame
        # "imu"; the team has not yet decided which one Isaac should mimic.
        DeclareLaunchArgument(
            "imu_frame",
            default_value="imu_link",
            description="Body IMU frame_id: imu_link (anubi sim convention) or imu (real G1)",
        ),
        # Host support live: isaac_dds_host.py --publish-odom-gt streams the
        # pelvis pose over ZMQ; the republisher publishes it on /odom_gt.
        DeclareLaunchArgument(
            "enable_odom_raw",
            default_value="true",
            description="Relay ground-truth odometry to /odom_raw",
        ),
        DeclareLaunchArgument("odom_input_topic", default_value="/odom_gt"),
        DeclareLaunchArgument("odom_raw_topic", default_value="/odom_raw"),
        # Standing pelvis height for base_footprint -> base_link. NOTE:
        # docs/topic_map.md quotes ~0.78 m for the real G1 (g1_state_bridge);
        # 0.80 is the Isaac model's standing height. Calibrate if they diverge.
        DeclareLaunchArgument("standing_height", default_value="0.80"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    lidar_points_relay = Node(
        package="g1_isaac",
        executable="frame_relay_node.py",
        name="lidar_points_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": LaunchConfiguration("lidar_points_input_topic"),
            "output_topic": LaunchConfiguration("lidar_points_topic"),
            "message_type": "pointcloud2",
            "frame_id": LaunchConfiguration("lidar_frame"),
        }],
    )

    lidar_imu_relay = Node(
        package="g1_isaac",
        executable="frame_relay_node.py",
        name="lidar_imu_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": LaunchConfiguration("lidar_imu_input_topic"),
            "output_topic": LaunchConfiguration("lidar_imu_topic"),
            "message_type": "imu",
            "frame_id": LaunchConfiguration("lidar_frame"),
        }],
    )

    body_imu_relay = Node(
        package="g1_isaac",
        executable="frame_relay_node.py",
        name="body_imu_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": LaunchConfiguration("body_imu_input_topic"),
            "output_topic": LaunchConfiguration("body_imu_topic"),
            "message_type": "imu",
            "frame_id": LaunchConfiguration("imu_frame"),
        }],
        condition=IfCondition(LaunchConfiguration("enable_body_imu")),
    )

    odom_raw_relay = Node(
        package="g1_isaac",
        executable="frame_relay_node.py",
        name="odom_raw_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": LaunchConfiguration("odom_input_topic"),
            "output_topic": LaunchConfiguration("odom_raw_topic"),
            "message_type": "odometry",
            "frame_id": "odom",
            "child_frame_id": "base_footprint",
        }],
        condition=IfCondition(LaunchConfiguration("enable_odom_raw")),
    )

    # Fixed standing-height offset, matching the sim convention in
    # docs/topic_map.md (the EKF owns odom -> base_footprint; this bringup
    # must not publish that transform).
    footprint_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="footprint_to_base_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", LaunchConfiguration("standing_height"),
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Head-mounted Mid-360 extrinsics, UPRIGHT (pitch-only): the Isaac host
    # cloud is already righted, so no roll term belongs here.
    base_to_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        output="screen",
        arguments=[
            "--x", "-0.00368",
            "--y", "0.00003",
            "--z", "0.46018",
            "--roll", "0.0",
            "--pitch", "0.0401",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", LaunchConfiguration("lidar_frame"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Identity body-IMU frame, co-located with base_link per the sim
    # contract. Published only with the body IMU relay so the frame does not
    # dangle while the host topic is pending; follows the imu_frame choice.
    base_to_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", LaunchConfiguration("imu_frame"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("enable_body_imu")),
    )

    return LaunchDescription(args + [
        lidar_points_relay,
        lidar_imu_relay,
        body_imu_relay,
        odom_raw_relay,
        footprint_to_base_tf,
        base_to_lidar_tf,
        base_to_imu_tf,
    ])
