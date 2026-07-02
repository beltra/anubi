"""
sim_g1.launch.py - Gazebo Fortress kinematic-base simulation for the Unitree G1.

Humanoid analogue of go2_sim/sim_champ.launch.py. The G1 walking gait (AMO) is
NOT simulated; a Fortress VelocityControl plugin drives a collision-less ghost
base directly from /cmd_vel_safe, so the full Nav2 perception + planning loop can
be verified without leg physics. Everything downstream is identical to the Go2
because the sim publishes the SAME canonical topics/frames (docs/topic_map.md).

Topic bridges (Ignition -> ROS):
  /clock, /imu/data, /lidar/imu,
  /livox/lidar/points -> /lidar/points   (frame: lidar_link)
  /model/<robot>/odometry -> /odom_raw
ROS -> Ignition:
  /cmd_vel_safe -> /model/<robot>/cmd_vel   (VelocityControl)

NOTE: The Ignition /tf topic is intentionally NOT bridged - EKF (odom->base_footprint),
static TFs, and RSP own the ROS TF tree (mirrors sim_champ.launch.py).
"""

import os
import re
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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
    robot_name = LaunchConfiguration("robot_name")

    # G1 nominal standing geometry. 0.80 m seats the AMO standing-pose feet on the
    # floor (FK foot origin is -0.779 m from the pelvis; +~0.02 m clears the soles).
    standing_height_m = 0.80
    # Mid-360 (lidar_link) offset in base_link, from the official URDF chain.
    lidar_x_m = -0.00368
    lidar_y_m = 0.00003
    lidar_z_in_base_m = 0.46018  # real pelvis->mid360 mount height

    g1_sim_dir = get_package_share_directory("g1_sim")
    g1_desc_dir = get_package_share_directory("g1_description")
    sim_worlds_dir = get_package_share_directory("sim_worlds")
    urdf_path = os.path.join(g1_sim_dir, "urdf", "g1_sim.urdf.xacro")
    default_world = os.path.join(sim_worlds_dir, "worlds", "indoor_office.world")

    # Resource path so Gazebo resolves model://sim_worlds/... and the robot's
    # package://g1_description/meshes/... STL files. Each package's share-parent
    # is added so "package://<pkg>/..." resolves to "<share>/<pkg>/...".
    resource_dirs = os.pathsep.join([
        os.path.join(g1_sim_dir, ".."),
        os.path.join(g1_desc_dir, ".."),
    ])
    ign_resource_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=[resource_dirs]
    )
    gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=[resource_dirs]
    )

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "use_lio", default_value="false",
            description="Enable EKF fusion with optional /lidar_odom input",
        ),
        DeclareLaunchArgument("robot_name", default_value="g1"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument(
            "world_name", default_value="indoor_office",
            description="Gazebo world name; must match <world name=\"...\"> in the SDF",
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world_init_x", default_value="0.0"),
        DeclareLaunchArgument("world_init_y", default_value="0.0"),
        DeclareLaunchArgument(
            "world_init_z", default_value=str(standing_height_m),
            description="Spawn height = nominal pelvis standing height (base_link).",
        ),
        DeclareLaunchArgument("world_init_heading", default_value="0.0"),
        DeclareLaunchArgument(
            "publish_map_to_odom_tf", default_value="true",
            description="Publish static map->odom identity TF. Set false when SLAM owns map->odom.",
        ),
        DeclareLaunchArgument(
            "sim_partition", default_value="g1sim",
            description="Gazebo transport partition. Fixed so a GUI client can attach to a "
                        "headless server with 'IGN_PARTITION=g1sim ign gazebo -g'. Override "
                        "per run (e.g. g1sim_$$) to isolate concurrent sims.",
        ),
    ]

    ign_partition = SetEnvironmentVariable(
        name="IGN_PARTITION", value=LaunchConfiguration("sim_partition")
    )
    gz_partition = SetEnvironmentVariable(
        name="GZ_PARTITION", value=LaunchConfiguration("sim_partition")
    )

    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_path, " robot_name:=", robot_name]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 50.0},
        ],
    )

    # -- Gazebo Fortress ------------------------------------------------------
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

    # Gazebo is spawned from a GENERATED SDF (not "-topic robot_description") so the
    # head/sensor-pod visual can be tagged invisible to the gpu_lidar. The URDF-to-SDF
    # conversion drops per-visual <visibility_flags>, so the flag is injected here; the
    # gpu_lidar's <visibility_mask> (set in the Mid-360 xacro) does survive conversion.
    # RViz/TF still use the full URDF via robot_state_publisher, so the head stays in
    # the visualized model and the GUI camera (default mask) renders it normally.
    def make_spawn_sdf(context):
        name = robot_name.perform(context)
        urdf_xml = subprocess.check_output(
            ["xacro", urdf_path, "robot_name:=" + name], text=True
        )
        urdf_tmp = os.path.join(tempfile.gettempdir(), "g1_sim_" + name + ".urdf")
        with open(urdf_tmp, "w") as fh:
            fh.write(urdf_xml)
        sdf_xml = subprocess.check_output(
            ["ign", "sdf", "-p", urdf_tmp], text=True, stderr=subprocess.DEVNULL
        )
        # Keep only the SDF document (drop any leading tool banner line).
        sdf_xml = sdf_xml[sdf_xml.index("<sdf"):]
        # Tag ONLY the head/sensor-pod visual (the one mesh that encloses the Mid-360
        # optical center) with visibility_flags=2 so the gpu_lidar, whose mask clears
        # bit 1, cannot see its own housing. Every other body visual keeps the default
        # 0xFFFFFFFF and is still seen, giving realistic self-occlusion + self-returns.
        sdf_xml, n_sub = re.subn(
            r"(<visual\b[^>]*>(?:(?!</visual>).)*?head_link\.STL(?:(?!</visual>).)*?)(</visual>)",
            r"\1<visibility_flags>2</visibility_flags>\2",
            sdf_xml, count=1, flags=re.DOTALL,
        )
        if n_sub != 1:
            raise RuntimeError(
                "g1_sim spawn: expected exactly one head_link visual to tag, got "
                + str(n_sub)
            )
        # The FK body meshes carry NO collisions. The Fortress VelocityControl plugin is
        # a vehicle-style planar controller: it drives x/y/yaw and relies on GROUND
        # CONTACT to hold z (it never controls z itself). So give base_link one box
        # <collision> sized to the Nav2 footprint whose sole sits at base z=-0.80, i.e.
        # exactly the floor when base_link spawns at the 0.80 m standing height: the body
        # rests on the ground plane and VelocityControl slides it, with z pinned by
        # contact. Gravity stays ON (weight keeps the contact stable during motion, so it
        # cannot tunnel), and tipping is prevented by the LOW center of mass set in
        # g1_body_standing.urdf (inertial origin at base z=-0.6, world ~0.20 m): a
        # bottom-heavy body on a 0.40x0.50 footprint cannot topple under VelocityControl
        # accelerations. The box matches the planner footprint, so physical clearance
        # equals what Nav2 assumes. URDF to SDF emits no <collision>, so inject one.
        collision = (
            "<collision name='base_footprint_collision'>"
            "<pose>0.025 0 -0.35 0 0 0</pose>"
            "<geometry><box><size>0.40 0.50 0.90</size></box></geometry>"
            "</collision>"
        )
        sdf_xml, n_col = re.subn(
            r"(<link\s+name=['\"]base_link['\"]\s*>)",
            r"\1" + collision,
            sdf_xml, count=1,
        )
        if n_col != 1:
            raise RuntimeError("g1_sim spawn: could not add base_link collision")
        sdf_tmp = os.path.join(tempfile.gettempdir(), "g1_sim_" + name + ".sdf")
        with open(sdf_tmp, "w") as fh:
            fh.write(sdf_xml)
        return [Node(
            package="ros_ign_gazebo",
            executable="create",
            output="screen",
            arguments=[
                "-world", LaunchConfiguration("world_name").perform(context),
                "-name", name,
                "-file", sdf_tmp,
                "-x", LaunchConfiguration("world_init_x").perform(context),
                "-y", LaunchConfiguration("world_init_y").perform(context),
                "-z", LaunchConfiguration("world_init_z").perform(context),
                "-Y", LaunchConfiguration("world_init_heading").perform(context),
            ],
        )]

    gazebo_spawn_robot = OpaqueFunction(function=make_spawn_sdf)

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
            # /tf intentionally NOT bridged - see module docstring.
            "/livox/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            ["/model/", robot_name, "/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"],
            # ROS -> Ignition: final safe command drives the kinematic base.
            ["/model/", robot_name, "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"],
        ],
        remappings=[
            ("/livox/lidar/points", "/lidar/points"),
            (["/model/", robot_name, "/odometry"], "/odom_raw"),
            (["/model/", robot_name, "/cmd_vel"], "/cmd_vel_safe"),
        ],
    )

    # -- Odometry / TF (identical pattern to sim_champ) -----------------------
    def maybe_create_ekf(context):
        lio = use_lio.perform(context).lower() == "true"
        # With use_lio, the Mid-360 DLIO estimate (/lidar_odom -> relay ->
        # /lidar_odom_reliable) IS the odometry, exactly as on the real robot, and the
        # Gazebo ground-truth OdometryPublisher (/odom_raw) is dropped so DLIO is
        # actually exercised end to end. Without use_lio the EKF runs on the perfect
        # ground-truth /odom_raw, which is simpler but does not test the LIO pipeline.
        primary_odom = "/lidar_odom_reliable" if lio else "/odom_raw"
        ekf_params = [
            {"use_sim_time": use_sim_time},
            {"base_link_frame": "base_footprint"},
            {"odom_frame": "odom"},
            {"world_frame": "odom"},
            {"publish_tf": True},
            {"frequency": 50.0},
            {"two_d_mode": True},
            {"odom0": primary_odom},
            {"odom0_config": [True, True, False,
                              False, False, True,
                              True, True, False,
                              False, False, True,
                              False, False, False]},
            {"odom0_differential": False},
            {"odom0_relative": False},
            {"odom0_queue_size": 10},
            {"imu0": "imu/data"},
            {"imu0_config": [False, False, False,
                             False, False, False,
                             False, False, False,
                             False, False, True,
                             False, False, False]},
            {"imu0_remove_gravitational_acceleration": True},
        ]
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

    lidar_odom_relay = Node(
        package="robot_nav",
        executable="lidar_odom_relay.py",
        name="lidar_odom_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/lidar_odom",
            "output_topic": "/lidar_odom_reliable",
            "pose_cov": [0.03, 0.03, 0.05, 0.03, 0.03, 0.02],
            "twist_cov": [0.02, 0.02, 0.05, 0.02, 0.02, 0.02],
            "max_pos_speed": 1.5,
            "max_scale": 100.0,
        }],
        condition=IfCondition(use_lio),
    )

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

    lidar_level_frame_publisher = Node(
        package="robot_nav",
        executable="lidar_level_frame_publisher",
        name="lidar_level_frame_publisher",
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

    # Safety arbiter tail: /cmd_vel_collision_free -> /cmd_vel_safe (watchdog-only in sim).
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
            "status_topic": "",
        }],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(args + [
        ign_resource_path,
        gz_resource_path,
        ign_partition,
        gz_partition,
        ign_gazebo_gui,
        ign_gazebo_headless,
        robot_state_publisher,
        gazebo_spawn_robot,
        gazebo_bridge,
        velocity_limiter,
        footprint_to_odom_ekf,
        lidar_odom_relay,
        base_footprint_to_base_link_tf,
        lidar_level_frame_publisher,
        map_to_odom_tf,
        rviz2,
    ])
