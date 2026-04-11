"""
go2_hardware.launch.py
Hardware-only launch: robot description + hw bridge.
No SLAM, no Nav2, no RViz.
Useful for testing the hardware interface in isolation.

NOTE: The Go2 robot's internal ROS2 bridge must be running and publishing:
  /sportmodestate  [unitree_go/SportModeState]
before this launch is started.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    go2_bringup_share     = get_package_share_directory("go2_bringup")
    go2_description_share = get_package_share_directory("go2_description")

    go2_params = os.path.join(go2_bringup_share,    "config", "go2_params.yaml")
    urdf_file  = os.path.join(go2_description_share,"urdf",   "go2.urdf.xacro")
    lidar_x = "0.28945"
    lidar_y = "0.0"
    lidar_z = "0.233175"  # standing_height + lidar_z from go2_description

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel_collision_free",
            description="Command topic consumed by go2_hw_bridge. Keep the safety path by default.",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    # FIX(agent5): Removed "use_gazebo:=" arg — go2_description URDF no longer
    # accepts use_gazebo; all Gazebo content lives in go2_sim only.
    robot_description = Command(["xacro ", urdf_file])

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                go2_params,
                {"robot_description": robot_description, "use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="go2_bringup",
            executable="go2_hw_bridge",
            name="go2_hw_bridge",
            output="screen",
            parameters=[go2_params, {"use_sim_time": use_sim_time, "cmd_vel_topic": cmd_vel_topic}],
        ),
        Node(
            package="robot_nav",
            executable="imu_stabilized_tf.py",
            name="imu_stabilized_tf",
            output="screen",
            parameters=[{
                "use_sim_time":    use_sim_time,
                "imu_topic":       "/imu/data",
                "parent_frame":    "base_footprint",
                "child_frame":     "lidar_stabilized",
                "lidar_x_in_base":  0.28945,   # from go2_description/urdf/go2.urdf.xacro
                "lidar_y_in_base":  0.0,
                "lidar_z_in_base": -0.046825,
                "standing_height":  0.28,
            }],
        ),
    ]

    return LaunchDescription(args + nodes)
