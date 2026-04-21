"""
sim_champ.launch.py
Fortress (Ignition Gazebo 6) + CHAMP simulation launch file for the Unitree Go2.

Source: khaledgabr77/unitree_go2_ros2 / unitree_go2_sim/launch/unitree_go2_launch.py
Port changes:
  Fortress packages only (ros_ign_gazebo, ros_ign_bridge, ign_ros2_control)
  Ignition message namespace only (ignition.msgs.*)

Topic bridge (all required by nav stack):
  /clock              ignition -> ROS   rosgraph_msgs/Clock
  /imu/data           ignition -> ROS   sensor_msgs/Imu
  /lidar/imu          ignition -> ROS   sensor_msgs/Imu
  /joint_states       ignition -> ROS   sensor_msgs/JointState
  /unilidar/cloud/points ignition -> ROS   sensor_msgs/PointCloud2  (remapped to /lidar/points)
  /model/go2/odometry ignition -> ROS   nav_msgs/Odometry (remapped to /odom_raw for EKF)
  /joint_group_effort_controller/joint_trajectory
                      ROS -> ignition   trajectory_msgs/JointTrajectory

NOTE: The Ignition /tf topic is intentionally NOT bridged.
  ignition-gazebo-odometry-publisher-system publishes an odom->base_link transform
  on the Ignition /tf topic by default.  Bridging it would give base_link two
  parents in the ROS TF tree (base_footprint via static TF and odom via the bridge),
  causing a TF cycle that breaks pointcloud_to_laserscan and slam_toolbox lookups.
  The full TF tree is provided by EKF (odom->base_footprint), static transforms, and RSP.

Usage:
  ros2 launch go2_sim sim_champ.launch.py
  ros2 launch go2_sim sim_champ.launch.py gui:=false rviz:=false
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
    lidar_z_in_base_m = -0.046825   # lidar_link z in base_link frame (from go2_sim.urdf.xacro)

    go2_sim_dir = get_package_share_directory("go2_sim")

    # Resource path setup
    # Add the package's share directory to the Ignition Gazebo resource path.
    # This allows Gazebo to find meshes via 'model://go2_sim/...' or 'package://go2_sim/...'.
    # We use both IGN_GAZEBO_RESOURCE_PATH (Fortress) and GZ_SIM_RESOURCE_PATH (Harmonic).
    ign_resource_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[os.path.join(go2_sim_dir, "..")]
    )
    gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[os.path.join(go2_sim_dir, "..")]
    )

    # Config file paths
    joints_config    = os.path.join(go2_sim_dir, "config", "joints.yaml")
    links_config     = os.path.join(go2_sim_dir, "config", "links.yaml")
    gait_config      = os.path.join(go2_sim_dir, "config", "gait.yaml")
    ros_control_file = os.path.join(go2_sim_dir, "config", "ros_control.yaml")
    urdf_path        = os.path.join(go2_sim_dir, "urdf", "go2_sim.urdf.xacro")
    world_path       = os.path.join(go2_sim_dir, "worlds", "default.sdf")

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Launch RViz2 (no config bundled; add -d <config> manually)",
    )
    declare_use_lio = DeclareLaunchArgument(
        "use_lio",
        default_value="false",
        description="Enable EKF fusion with optional /lidar_odom input",
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name in Gazebo"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file", default_value=ros_control_file,
        description="Path to ros_control YAML",
    )
    declare_world = DeclareLaunchArgument(
        "world", default_value=world_path,
        description="Path to Gazebo world SDF",
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name", default_value="default",
        description=(
            "Gazebo world name used in transport topic paths. "
            "Set this to the <world name=\"...\"> value from the SDF."
        ),
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Launch Gazebo GUI"
    )
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
        description=(
            "Publish static map->odom identity transform. "
            "Set false when SLAM/localization owns map->odom."
        ),
    )
    declare_sim_partition = DeclareLaunchArgument(
        "sim_partition",
        default_value=f"go2_sim_{os.getpid()}",
        description=(
            "Gazebo transport partition for this launch. "
            "Unique default avoids scene/topic collisions from stale sim processes."
        ),
    )

    # Keep all Gazebo participants isolated to one launch instance.
    # Setting both names covers Fortress (IGN_*) and newer Gazebo (GZ_*).
    ign_partition = SetEnvironmentVariable(
        name="IGN_PARTITION",
        value=LaunchConfiguration("sim_partition"),
    )
    gz_partition = SetEnvironmentVariable(
        name="GZ_PARTITION",
        value=LaunchConfiguration("sim_partition"),
    )

    # Robot description (processed at launch time by xacro)
    robot_description_content = ParameterValue(
        Command([
            "xacro ", urdf_path,
            " robot_controllers:=", LaunchConfiguration("ros_control_file"),
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # Nodes

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # CHAMP quadruped locomotion controller
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

    # EKF: base_footprint -> odom (global odometry, publishes /odom + TF)
    # When use_lio:=true, fuse /lidar_odom as odom1 to improve drift behavior.
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
            # In simulation /odom_raw is already world-referenced odometry from
            # Ignition's odometry publisher. Fuse its planar pose directly so
            # /odom follows the robot's actual motion instead of dead-reckoning
            # only from twist.
            {"odom0_config": [True, True, False,
                              False, False, True,
                              True, True, False,
                              False, False, True,
                              False, False, False]},
            {"odom0_differential": False},
            {"odom0_relative": False},
            {"imu0": "imu/data"},
            # Keep IMU as a yaw-rate stabilizer instead of a second pose source.
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
                # Fuse DLIO as an absolute pose source; differential mode can
                # amplify startup instability when calibration is active.
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

    # Static transform: base_footprint -> base_link
    # The sim URDF (go2_sim.urdf.xacro) has base_link as its root, so Nav2/SLAM need
    # an explicit ground-projection frame bridge. This must preserve the nominal
    # body-center height used everywhere else in the stack; an identity transform
    # incorrectly places lidar_link below the ground plane in the TF tree.
    #
    # SIM/REAL ASYMMETRY: On the real Go2, go2_hw_bridge publishes a dynamic
    # base_footprint->base_link TF that reflects actual IMU roll and pitch from
    # SportModeState.  Here in simulation we use a fixed z-only static TF
    # (no roll/pitch component) because CHAMP operates in two_d_mode and the
    # EKF does not model body tilt.  This means RViz shows the simulated robot
    # as perfectly level even when climbing ramps; it does not affect Nav2
    # costmap or SLAM accuracy with gravity-aligned point clouds.
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
            "lidar_x_in_base":  lidar_x_m,           # 0.28945 from go2_sim.urdf.xacro
            "lidar_y_in_base":  lidar_y_m,           # 0.0
            "lidar_z_in_base":  lidar_z_in_base_m,   # -0.046825 from go2_sim.urdf.xacro
            "standing_height":  standing_height_m,   # 0.28
        }],
    )

    # Static transform: map -> odom (identity; nav stack expects this until SLAM publishes)
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

    # RViz2 (optional; no bundled config - pass -d <config> via extra_args if needed)
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Gazebo Fortress (ign gazebo)
    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")

    ign_gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                " -r",  # start unpaused
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )
    ign_gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py")
        ),
        condition=UnlessCondition(LaunchConfiguration("gui")),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                " -r -s",  # start unpaused, server-only (no GUI)
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot in Ignition Gazebo
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

    # ros_ign_bridge: topic bridges between Ignition and ROS 2
    #
    # Bridge direction syntax (Fortress):
    #   topic@ros_type[ignition_type   - ignition -> ROS  (unidirectional)
    #   topic@ros_type]ignition_type   - ROS -> ignition  (unidirectional)
    #   topic@ros_type@ignition_type   - bidirectional
    #
    # Message namespace: ignition.msgs.*  (NOT gz.msgs.* which is Harmonic)
    #
    # Required topics for nav stack:
    #   /joint_states   -> robot_state_publisher, CHAMP
    #   /imu/data       -> EKF
    #   /lidar/imu      -> LIO (LiDAR-integrated IMU)
    #   /lidar/points   -> voxel_layer, pointcloud_to_laserscan  (bridged from /unilidar/cloud/points)
    #   /odom_raw       -> EKF footprint_to_odom
    #   /clock          -> use_sim_time
    #   /joint_group_effort_controller/joint_trajectory -> effort controller
    #   NOTE: /tf is NOT bridged (see module docstring)
    gazebo_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            # Ignition -> ROS
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/lidar/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # NOTE: /tf is intentionally absent - see module docstring for rationale.
            [
                "/world/",
                LaunchConfiguration("world_name"),
                "/model/go2/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            "/unilidar/cloud/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/model/go2/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # ROS -> Ignition
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

    # Safety gate in simulation: Nav2 /cmd_vel -> /cmd_vel_safe
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
            # Simulation has no RobotStatus publisher; run watchdog-only gating.
            "status_topic": "",
        }],
    )

    # Controller spawners (delayed to allow Gazebo + ign_ros2_control init)
    # controller_manager is brought up by ign_ros2_control-system plugin in the URDF.
    # Delays are empirical: Fortress + ign_ros2_control typically takes 15-20 s from spawn
    # to ready.  The spawner's --controller-manager-timeout 120 provides an additional
    # safety margin if init is slow on the host machine.
    controller_spawner_js = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_states_controller",
                ],
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
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_group_effort_controller",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    # Launch description
    return LaunchDescription([
        # Resource path environment variables
        ign_resource_path,
        gz_resource_path,

        # Arguments
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

        # Gazebo + robot spawn
        ign_partition,
        gz_partition,
        ign_gazebo_gui,
        ign_gazebo_headless,
        robot_state_publisher,
        gazebo_spawn_robot,
        gazebo_bridge,
        velocity_limiter,

        # CHAMP locomotion stack
        quadruped_controller_node,

        # Odometry / TF
        footprint_to_odom_ekf,
        base_footprint_to_base_link_tf,
        lidar_stabilized_tf,
        map_to_odom_tf,

        # Controller lifecycle
        controller_spawner_js,
        controller_spawner_effort,

        # Visualisation
        rviz2,
    ])
