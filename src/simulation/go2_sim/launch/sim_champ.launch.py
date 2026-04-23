"""
sim_champ.launch.py — Gazebo Fortress + CHAMP simulation for the Unitree Go2.

Topic bridges (Ignition -> ROS):
  /clock, /imu/data, /lidar/imu, /joint_states,
  /unilidar/cloud/points -> /lidar/points,
  /model/go2/odometry -> /odom_raw
ROS -> Ignition:
  /joint_group_effort_controller/joint_trajectory

NOTE: The Ignition /tf topic is intentionally NOT bridged.
  Bridging it would create a TF cycle (base_link gets two parents) that breaks
  pointcloud_to_laserscan and slam_toolbox. The full TF tree is provided by
  EKF (odom->base_footprint), static transforms, and RSP.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_lio = LaunchConfiguration("use_lio")
    standing_height_m = 0.28
    lidar_x_m = 0.28945
    lidar_y_m = 0.0
    lidar_z_in_base_m = -0.046825

    go2_sim_dir = get_package_share_directory("go2_sim")

    ign_resource_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[os.path.join(go2_sim_dir, "..")]
    )
    gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[os.path.join(go2_sim_dir, "..")]
    )

    joints_config    = os.path.join(go2_sim_dir, "config", "joints.yaml")
    links_config     = os.path.join(go2_sim_dir, "config", "links.yaml")
    gait_config      = os.path.join(go2_sim_dir, "config", "gait.yaml")
    ros_control_file = os.path.join(go2_sim_dir, "config", "ros_control.yaml")
    urdf_path        = os.path.join(go2_sim_dir, "urdf", "go2_sim.urdf.xacro")
    world_path       = os.path.join(go2_sim_dir, "worlds", "default.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false",
    )
    declare_use_lio = DeclareLaunchArgument(
        "use_lio",
        default_value="false",
        description="Enable EKF fusion with optional /lidar_odom input",
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2",
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file", default_value=ros_control_file,
    )
    declare_world = DeclareLaunchArgument(
        "world", default_value=world_path,
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name", default_value="default",
        description="Gazebo world name; must match <world name=\"...\"> in the SDF",
    )
    declare_gui = DeclareLaunchArgument("gui", default_value="true")
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument(
        "world_init_z", default_value="0.375",
        description="Spawn height (0.375 m = nominal standing height + clearance)",
    )
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_publish_map_to_odom_tf = DeclareLaunchArgument(
        "publish_map_to_odom_tf",
        default_value="true",
        description="Publish static map->odom identity TF. Set false when SLAM owns map->odom.",
    )
    declare_sim_partition = DeclareLaunchArgument(
        "sim_partition",
        default_value=f"go2_sim_{os.getpid()}",
        description="Gazebo transport partition — unique default avoids collisions from stale processes.",
    )

    ign_partition = SetEnvironmentVariable(
        name="IGN_PARTITION",
        value=LaunchConfiguration("sim_partition"),
    )
    gz_partition = SetEnvironmentVariable(
        name="GZ_PARTITION",
        value=LaunchConfiguration("sim_partition"),
    )

    robot_description_content = ParameterValue(
        Command([
            "xacro ", urdf_path,
            " robot_controllers:=", LaunchConfiguration("ros_control_file"),
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": robot_description_content},
            {"hardware_connected": False},
            {"close_loop_odom": True},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel_safe")],
    )

    def maybe_create_ekf(context):
        ekf_params = [
            {"use_sim_time": use_sim_time},
            {"base_link_frame": "base_footprint"},
            {"odom_frame": "odom"},
            {"world_frame": "odom"},
            {"publish_tf": True},
            {"frequency": 50.0},
            {"two_d_mode": True},
            {"odom0": "/odom_raw"},
            {"odom0_config": [True, True, False,
                              False, False, True,
                              True, True, False,
                              False, False, True,
                              False, False, False]},
            {"odom0_differential": False},
            {"odom0_relative": False},
            {"imu0": "imu/data"},
            {"imu0_config": [False, False, False,
                             False, False, False,
                             False, False, False,
                             False, False, True,
                             False, False, False]},
            {"imu0_remove_gravitational_acceleration": True},
        ]

        if use_lio.perform(context).lower() == "true":
            ekf_params.extend([
                {"odom1": "/lidar_odom"},
                {"odom1_config": [True, True, False,
                                  False, False, True,
                                  True, True, False,
                                  False, False, True,
                                  False, False, False]},
                {"odom1_differential": False},
                {"odom1_relative": False},
                {"odom1_queue_size": 10},
            ])

        return [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="footprint_to_odom_ekf",
                output="screen",
                parameters=ekf_params,
                remappings=[("odometry/filtered", "odom")],
            )
        ]

    footprint_to_odom_ekf = OpaqueFunction(function=maybe_create_ekf)

    # Static base_footprint->base_link: sim uses fixed z-only (no roll/pitch) because
    # CHAMP two_d_mode doesn't model body tilt. Does not affect Nav2 or SLAM accuracy
    # with gravity-aligned point clouds.
    base_footprint_to_base_link_tf = Node(
        package="tf2_ros",
        name="base_footprint_to_base_link_tf",
        executable="static_transform_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x", "0", "--y", "0", "--z", str(standing_height_m),
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_footprint", "--child-frame-id", "base_link",
        ],
    )
    lidar_stabilized_tf = Node(
        package="robot_nav",
        executable="imu_stabilized_tf.py",
        name="imu_stabilized_tf",
        output="screen",
        parameters=[{
            "use_sim_time":    use_sim_time,
            "imu_topic":       "/imu/data",
            "parent_frame":    "base_footprint",
            "child_frame":     "lidar_stabilized",
            "lidar_x_in_base":  lidar_x_m,
            "lidar_y_in_base":  lidar_y_m,
            "lidar_z_in_base":  lidar_z_in_base_m,
            "standing_height":  standing_height_m,
        }],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        name="map_to_odom_tf",
        executable="static_transform_publisher",
        condition=IfCondition(LaunchConfiguration("publish_map_to_odom_tf")),
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")

    ign_gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r"],
            "on_exit_shutdown": "true",
        }.items(),
    )
    ign_gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py")
        ),
        condition=UnlessCondition(LaunchConfiguration("gui")),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r -s"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gazebo_spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        arguments=[
            "-world", LaunchConfiguration("world_name"),
            "-name", LaunchConfiguration("robot_name"),
            "-topic", "robot_description",
            "-x", LaunchConfiguration("world_init_x"),
            "-y", LaunchConfiguration("world_init_y"),
            "-z", LaunchConfiguration("world_init_z"),
            "-Y", LaunchConfiguration("world_init_heading"),
        ],
    )

    gazebo_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/lidar/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # /tf is intentionally NOT bridged — see module docstring
            [
                "/world/",
                LaunchConfiguration("world_name"),
                "/model/go2/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            "/unilidar/cloud/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/model/go2/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/joint_group_effort_controller/joint_trajectory"
            "@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory",
        ],
        remappings=[
            ([
                "/world/",
                LaunchConfiguration("world_name"),
                "/model/go2/joint_state",
            ], "/joint_states"),
            ("/model/go2/odometry", "/odom_raw"),
            ("/unilidar/cloud/points", "/lidar/points"),
        ],
    )

    velocity_limiter = Node(
        package="robot_safety",
        executable="velocity_limiter",
        name="velocity_limiter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "cmd_timeout": 0.5,
            "publish_rate": 20.0,
            "cmd_vel_in": "/cmd_vel_collision_free",
            "cmd_vel_out": "/cmd_vel_safe",
            "status_topic": "",  # no RobotStatus in simulation; watchdog-only gating
        }],
    )

    # Controller spawners: delayed for Gazebo + ign_ros2_control init (~15-20 s on Fortress)
    controller_spawner_js = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=["--controller-manager-timeout", "120", "joint_states_controller"],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    controller_spawner_effort = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=["--controller-manager-timeout", "120", "joint_group_effort_controller"],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        ign_resource_path,
        gz_resource_path,
        declare_use_sim_time,
        declare_rviz,
        declare_use_lio,
        declare_robot_name,
        declare_ros_control_file,
        declare_world,
        declare_world_name,
        declare_gui,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,
        declare_publish_map_to_odom_tf,
        declare_sim_partition,
        ign_partition,
        gz_partition,
        ign_gazebo_gui,
        ign_gazebo_headless,
        robot_state_publisher,
        gazebo_spawn_robot,
        gazebo_bridge,
        velocity_limiter,
        quadruped_controller_node,
        footprint_to_odom_ekf,
        base_footprint_to_base_link_tf,
        lidar_stabilized_tf,
        map_to_odom_tf,
        controller_spawner_js,
        controller_spawner_effort,
        rviz2,
    ])
