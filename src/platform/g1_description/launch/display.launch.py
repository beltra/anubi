"""Display launch for the Unitree G1 description.

Brings up the full TF tree from g1.urdf.xacro so the model can be inspected in
RViz in isolation:

  * robot_state_publisher  — publishes TF from the URDF (xacro-expanded).
  * joint_state_publisher  — publishes default (zero) joint states so the TF
                             tree is complete even without a live robot. The
                             real-robot / sim pipeline replaces this with the
                             actual joint-state source.
  * rviz2 (optional)       — visualization, toggled by the ``rviz`` arg.

Launch args
-----------
  standing_height : nominal pelvis height above the floor (m). MUST be
                    calibrated on hardware. Default 0.78.
  use_sim_time    : use /clock from a simulator. Default false.
  rviz            : start RViz2. Default true.

Run inside the ROS 2 Humble environment:
    ros2 launch g1_description display.launch.py
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("g1_description")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "g1.urdf.xacro"])

    standing_height = LaunchConfiguration("standing_height")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    standing_height_arg = DeclareLaunchArgument(
        "standing_height",
        default_value="0.78",
        description="Nominal G1 pelvis height above the floor (m). "
        "Calibrate on hardware.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (/clock) time.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2.",
    )

    # Expand the xacro wrapper into the robot_description URDF string, threading
    # the standing_height arg through to the xacro.
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ",
            xacro_file,
            " standing_height:=", standing_height,
        ]),
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Non-GUI joint_state_publisher publishes default joint states (all zeros)
    # so robot_state_publisher can complete the TF tree without a live robot.
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        standing_height_arg,
        use_sim_time_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
