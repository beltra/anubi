"""
sim_a2w.launch.py - Gazebo Fortress wheeled-base simulation for the Unitree A2-W (Pro).

Wheeled analogue of g1_sim/sim_g1.launch.py. No gait and no wheel dynamics: the
legs are frozen in the standing pose and a Fortress VelocityControl plugin slides
the lumped body from /cmd_vel_safe, the same body-twist contract as the real
robot's SportClient::Move. Everything downstream publishes the canonical topics
and frames (docs/topic_map.md, A2-W section).

Topic bridges (Ignition -> ROS):
  /clock, /imu/data
  /a2w/front_lidar/points -> /lidar/front/points   (frame: front_lidar_link)
  /a2w/rear_lidar/points  -> /lidar/rear/points    (frame: rear_lidar_link)
  /model/<robot>/odometry -> /odom_raw
ROS -> Ignition:
  /cmd_vel_safe -> /model/<robot>/cmd_vel   (VelocityControl)

cloud_merger fuses the two clouds into /lidar/points (frame lidar_link), the single
cloud the shared perception chain consumes; the real bringup runs the same node.

The Ignition /tf topic is NOT bridged: the EKF owns odom->base_footprint, and the
static TFs + robot_state_publisher own the rest.
"""

import os
import re
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

# Standing geometry, kept in lockstep with a2w_description/urdf/a2w.urdf.xacro:
# base_link height above the floor = calf-tip depth + wheel radius.
STANDING_HEIGHT_M = 0.497
# Virtual merged-cloud frame lidar_link in base_link (identity rotation).
LIDAR_X_M = 0.0
LIDAR_Y_M = 0.0
LIDAR_Z_IN_BASE_M = 0.0843


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_lio = LaunchConfiguration("use_lio")
    robot_name = LaunchConfiguration("robot_name")

    a2w_sim_dir = get_package_share_directory("a2w_sim")
    a2w_desc_dir = get_package_share_directory("a2w_description")
    sim_worlds_dir = get_package_share_directory("sim_worlds")
    urdf_path = os.path.join(a2w_sim_dir, "urdf", "a2w_sim.urdf.xacro")
    default_world = os.path.join(sim_worlds_dir, "worlds", "indoor_office.world")

    # Each package's share-parent so model://<pkg>/... and package://<pkg>/... resolve.
    resource_dirs = os.pathsep.join([
        os.path.join(a2w_sim_dir, ".."),
        os.path.join(a2w_desc_dir, ".."),
    ])

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "use_lio", default_value="false",
            description="Drive the EKF from DLIO (/lidar_odom_reliable) instead of "
                        "Gazebo ground-truth odometry",
        ),
        DeclareLaunchArgument("robot_name", default_value="a2w"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument(
            "world_name", default_value="indoor_office",
            description="Gazebo world name; must match <world name=\"...\"> in the SDF",
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world_init_x", default_value="0.0"),
        DeclareLaunchArgument("world_init_y", default_value="0.0"),
        DeclareLaunchArgument("world_init_z", default_value=str(STANDING_HEIGHT_M)),
        DeclareLaunchArgument("world_init_heading", default_value="0.0"),
        DeclareLaunchArgument(
            "lidar_channels", default_value="64",
            description="Simulated JT128 channels per sensor (real: 128). Also sets the "
                        "horizontal /scan resolution, since the dome axes are horizontal.",
        ),
        DeclareLaunchArgument(
            "lidar_h_samples", default_value="180",
            description="Simulated JT128 azimuth samples per sweep (real: 900).",
        ),
        DeclareLaunchArgument(
            "physics_step_sec", default_value="0.004",
            description="Gazebo physics step for the world copy this launch runs; sets "
                        "the /clock rate (1/step).",
        ),
        DeclareLaunchArgument(
            "publish_map_to_odom_tf", default_value="true",
            description="Publish static map->odom identity TF. Set false when SLAM owns map->odom.",
        ),
        DeclareLaunchArgument(
            "sim_partition", default_value="a2wsim",
            description="Gazebo transport partition; attach a GUI with "
                        "'IGN_PARTITION=a2wsim ign gazebo -g'.",
        ),
    ]

    env = [
        AppendEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=[resource_dirs]),
        AppendEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=[resource_dirs]),
        SetEnvironmentVariable(name="IGN_PARTITION", value=LaunchConfiguration("sim_partition")),
        SetEnvironmentVariable(name="GZ_PARTITION", value=LaunchConfiguration("sim_partition")),
    ]

    xacro_cmd = [
        "xacro ", urdf_path,
        " robot_name:=", robot_name,
        " lidar_channels:=", LaunchConfiguration("lidar_channels"),
        " lidar_h_samples:=", LaunchConfiguration("lidar_h_samples"),
    ]
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)},
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 50.0},
        ],
    )

    # The sliding base needs no 1 kHz physics. Every ROS node with use_sim_time
    # wakes on each /clock tick, so the world's 1 ms step (1000 Hz /clock) costs
    # more CPU than the whole nav stack; run a copy of the world at
    # physics_step_sec instead. Real-time factor is unchanged.
    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")
    ign_gazebo_launch = os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py")

    def make_gazebo(context):
        world_src = LaunchConfiguration("world").perform(context)
        step = float(LaunchConfiguration("physics_step_sec").perform(context))
        with open(world_src) as fh:
            world_xml = fh.read()
        world_xml, n_step = re.subn(
            r"<max_step_size>[^<]*</max_step_size>",
            "<max_step_size>%g</max_step_size>" % step, world_xml)
        world_xml = re.sub(
            r"<real_time_update_rate>[^<]*</real_time_update_rate>",
            "<real_time_update_rate>%g</real_time_update_rate>" % (1.0 / step), world_xml)
        if n_step == 0:
            world_xml = re.sub(
                r"(<world\b[^>]*>)",
                r"\1<physics name='a2w_step' type='ode'><max_step_size>%g</max_step_size>"
                r"<real_time_factor>1</real_time_factor>"
                r"<real_time_update_rate>%g</real_time_update_rate></physics>" % (step, 1.0 / step),
                world_xml, count=1)
        world_tmp = os.path.join(
            tempfile.gettempdir(), "a2w_world_" + os.path.basename(world_src))
        with open(world_tmp, "w") as fh:
            fh.write(world_xml)
        flags = " -r" if LaunchConfiguration("gui").perform(context).lower() == "true" else " -r -s"
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ign_gazebo_launch),
            launch_arguments={"gz_args": world_tmp + flags, "on_exit_shutdown": "true"}.items(),
        )]

    ign_gazebo = OpaqueFunction(function=make_gazebo)

    # Spawn from a generated SDF rather than "-topic robot_description": the
    # URDF->SDF conversion cannot express visibility flags, a single support
    # collision, or a lowered centre of mass, so they are injected here. RViz/TF
    # keep the full URDF through robot_state_publisher.
    def make_spawn_sdf(context):
        name = robot_name.perform(context)
        urdf_xml = subprocess.check_output(
            ["xacro", urdf_path, "robot_name:=" + name,
             "lidar_channels:=" + LaunchConfiguration("lidar_channels").perform(context),
             "lidar_h_samples:=" + LaunchConfiguration("lidar_h_samples").perform(context)],
            text=True,
        )
        urdf_tmp = os.path.join(tempfile.gettempdir(), "a2w_sim_" + name + ".urdf")
        with open(urdf_tmp, "w") as fh:
            fh.write(urdf_xml)
        sdf_xml = subprocess.check_output(
            ["ign", "sdf", "-p", urdf_tmp], text=True, stderr=subprocess.DEVNULL
        )
        sdf_xml = sdf_xml[sdf_xml.index("<sdf"):]

        # The body shell mesh encloses both JT128 origins: hide it from the lidars
        # (their visibility_mask clears bit 1). Legs and wheels stay visible.
        sdf_xml, n_vis = re.subn(
            r"(<visual\b[^>]*>(?:(?!</visual>).)*?/base_link\.STL(?:(?!</visual>).)*?)(</visual>)",
            r"\1<visibility_flags>2</visibility_flags>\2",
            sdf_xml, count=1, flags=re.DOTALL,
        )
        if n_vis != 1:
            raise RuntimeError("a2w_sim spawn: expected one base_link.STL visual, got " + str(n_vis))

        # VelocityControl drives x/y/yaw only and relies on ground contact for z.
        # Replace the lumped leg/wheel collisions with ONE box matching the Nav2
        # footprint whose sole is the floor at the standing height, so physical
        # clearance equals what the planner assumes and nothing snags while
        # sliding. Wheel contact dynamics are deliberately not modelled.
        sdf_xml, n_col = re.subn(r"<collision\b.*?</collision>", "", sdf_xml, flags=re.DOTALL)
        if n_col == 0:
            raise RuntimeError("a2w_sim spawn: no collisions found to replace")
        support_h = STANDING_HEIGHT_M + 0.10
        support = (
            "<collision name='base_support_collision'>"
            "<pose>-0.05 0 {z:.4f} 0 0 0</pose>"
            "<geometry><box><size>0.94 0.58 {h:.4f}</size></box></geometry>"
            "<surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>"
            "</collision>"
        ).format(z=0.10 - support_h / 2.0, h=support_h)

        # Lower the lumped centre of mass so velocity steps cannot tip the slider.
        sdf_xml, n_inertial = re.subn(
            r"(<link\s+name=['\"]base_link['\"]\s*>\s*<inertial>\s*<pose>)[^<]*(</pose>)",
            r"\g<1>0 0 -0.40 0 0 0\2",
            sdf_xml, count=1,
        )
        if n_inertial != 1:
            raise RuntimeError("a2w_sim spawn: could not set base_link inertial pose")
        sdf_xml, n_link = re.subn(
            r"(<link\s+name=['\"]base_link['\"]\s*>)", r"\1" + support, sdf_xml, count=1
        )
        if n_link != 1:
            raise RuntimeError("a2w_sim spawn: could not add base_link support collision")

        sdf_tmp = os.path.join(tempfile.gettempdir(), "a2w_sim_" + name + ".sdf")
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
            "/a2w/front_lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/a2w/rear_lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            ["/model/", robot_name, "/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"],
            ["/model/", robot_name, "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"],
        ],
        remappings=[
            ("/a2w/front_lidar/points", "/lidar/front/points"),
            ("/a2w/rear_lidar/points", "/lidar/rear/points"),
            (["/model/", robot_name, "/odometry"], "/odom_raw"),
            (["/model/", robot_name, "/cmd_vel"], "/cmd_vel_safe"),
        ],
    )

    cloud_merger = Node(
        package="robot_nav",
        executable="cloud_merger",
        name="cloud_merger",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topics": ["/lidar/front/points", "/lidar/rear/points"],
            "output_topic": "/lidar/points",
            "output_frame": "lidar_link",
            "max_input_age_sec": 0.15,
        }],
    )

    def make_ekf(context):
        lio = use_lio.perform(context).lower() == "true"
        # With use_lio the DLIO estimate IS the odometry, as on the real robot,
        # and ground truth is not fused, so DLIO is exercised end to end.
        primary_odom = "/lidar_odom_reliable" if lio else "/odom_raw"
        return [Node(
            package="robot_localization",
            executable="ekf_node",
            name="footprint_to_odom_ekf",
            output="screen",
            parameters=[
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
            ],
            remappings=[("odometry/filtered", "odom")],
        )]

    footprint_to_odom_ekf = OpaqueFunction(function=make_ekf)

    lidar_odom_relay = Node(
        package="robot_nav",
        executable="lidar_odom_relay.py",
        name="lidar_odom_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/lidar_odom",
            "output_topic": "/lidar_odom_reliable",
            "pose_cov": [0.01, 0.01, 0.05, 0.03, 0.03, 0.004],
            "twist_cov": [0.02, 0.02, 0.05, 0.02, 0.02, 0.02],
            "max_pos_speed": 2.0,
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
            "--x", "0", "--y", "0", "--z", str(STANDING_HEIGHT_M),
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
            "use_sim_time": use_sim_time,
            "imu_topic": "/imu/data",
            "parent_frame": "base_footprint",
            "child_frame": "lidar_stabilized",
            "lidar_x_in_base": LIDAR_X_M,
            "lidar_y_in_base": LIDAR_Y_M,
            "lidar_z_in_base": LIDAR_Z_IN_BASE_M,
            "standing_height": STANDING_HEIGHT_M,
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

    return LaunchDescription(args + env + [
        ign_gazebo,
        robot_state_publisher,
        gazebo_spawn_robot,
        gazebo_bridge,
        cloud_merger,
        velocity_limiter,
        footprint_to_odom_ekf,
        lidar_odom_relay,
        base_footprint_to_base_link_tf,
        lidar_level_frame_publisher,
        map_to_odom_tf,
    ])
