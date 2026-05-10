"""
safety.launch.py
Launches the final velocity arbiter and safety signal nodes.

Intended to be included from go2_bringup.launch.py with use_safety:=true.
When active, go2_hw_bridge receives /cmd_vel_safe instead of /cmd_vel_collision_free.

Arguments
---------
use_sim_time           bool   (default: false)
cmd_timeout            float  Watchdog timeout in seconds  (default: 0.5)
publish_rate           float  Output publish rate in Hz    (default: 20.0)
cmd_vel_in             str    Input command topic          (default: /cmd_vel_collision_free)
cmd_vel_out            str    Output command topic         (default: /cmd_vel_safe)
status_topic           str    RobotStatus topic            (default: /robot_status)
nav_namespace          str    Nav2 namespace prefix        (default: '')
start_joystick_safety  bool   Launch joystick override node (default: true)
start_tilt_monitor     bool   Launch tilt estop node       (default: true)
recovery_ack_key_mask  int    Controller key mask for tilt recovery
resume_key_mask        int    Controller key mask for Nav2 resume and replan
abort_key_mask         int    Controller key mask for controlled Nav2 abort
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        DeclareLaunchArgument('cmd_timeout',   default_value='0.5',
                              description='Watchdog: zero velocity after this silence (s)'),
        DeclareLaunchArgument('publish_rate',  default_value='20.0',
                              description='Output /cmd_vel_safe publish rate (Hz)'),
        DeclareLaunchArgument('cmd_vel_in',    default_value='/cmd_vel_collision_free'),
        DeclareLaunchArgument('cmd_vel_out',   default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('status_topic',  default_value='/robot_status'),
        DeclareLaunchArgument('nav_namespace', default_value=''),
        DeclareLaunchArgument('joystick_topic', default_value='/wirelesscontroller'),
        DeclareLaunchArgument('joystick_override_topic', default_value='/joystick_override'),
        DeclareLaunchArgument('tilt_estop_topic', default_value='/safety/tilt_estop'),
        DeclareLaunchArgument('command_source_topic', default_value='/safety/command_source'),
        DeclareLaunchArgument('start_joystick_safety', default_value='true'),
        DeclareLaunchArgument('start_tilt_monitor', default_value='true'),
        DeclareLaunchArgument('recovery_ack_key_mask', default_value='0'),
        DeclareLaunchArgument('resume_key_mask', default_value='0'),
        DeclareLaunchArgument('abort_key_mask', default_value='0'),
    ]

    velocity_limiter = Node(
        package='robot_safety',
        executable='velocity_limiter',
        name='velocity_limiter',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_timeout':  LaunchConfiguration('cmd_timeout'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'cmd_vel_in':   LaunchConfiguration('cmd_vel_in'),
            'cmd_vel_out':  LaunchConfiguration('cmd_vel_out'),
            'status_topic': LaunchConfiguration('status_topic'),
            'joystick_override_topic': LaunchConfiguration('joystick_override_topic'),
            'tilt_estop_topic': LaunchConfiguration('tilt_estop_topic'),
            'command_source_topic': LaunchConfiguration('command_source_topic'),
        }],
    )

    joystick_safety = Node(
        package='robot_safety',
        executable='joystick_safety',
        name='joystick_safety',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_joystick_safety')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'joystick_topic': LaunchConfiguration('joystick_topic'),
            'override_topic': LaunchConfiguration('joystick_override_topic'),
            'resume_key_mask': LaunchConfiguration('resume_key_mask'),
            'abort_key_mask': LaunchConfiguration('abort_key_mask'),
            'nav_namespace': LaunchConfiguration('nav_namespace'),
        }],
    )

    tilt_monitor = Node(
        package='robot_safety',
        executable='tilt_monitor',
        name='tilt_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_tilt_monitor')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_topic': '/imu/data',
            'joystick_topic': LaunchConfiguration('joystick_topic'),
            'tilt_estop_topic': LaunchConfiguration('tilt_estop_topic'),
            'recovery_ack_key_mask': LaunchConfiguration('recovery_ack_key_mask'),
        }],
    )

    return LaunchDescription(args + [joystick_safety, tilt_monitor, velocity_limiter])
