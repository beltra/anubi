"""
bag_perception.launch.py -- minimal offline replay stack for tuning
slam_toolbox and Nav2/STVL against recorded Go2 sensor data.

The wrapper script (scripts/replay_bag_perception.sh) plays the bag with a
strict topic whitelist (raw sensors + /tf_static only). All dynamic TFs
and every derived perception/SLAM output are produced fresh inside this
launch, so there is exactly one publisher per topic and per transform.

What runs
---------
1. `robot_state_publisher` owns /robot_description (fresh xacro expansion
   of go2_description/urdf/go2.urdf.xacro) and all URDF-derived /tf_static
   frames including the leg links. This supersedes the bag's recorded
   /robot_description and /tf_static, which is why the wrapper drops both
   from the playback whitelist.
2. `lowstate_joint_state_publisher.py` (go2_bringup) consumes the bag's
   /lowstate and publishes /joint_states at ~50 Hz so robot_state_publisher
   can animate the leg links in RViz.
3. `odom_to_tf` re-broadcasts the bag's /odom topic as the
   `odom -> base_footprint` transform.
4. A static `base_footprint -> base_link` (z = standing_height) stands in
   for the body-height TF that go2_hw_bridge would publish live.
5. A static `lidar_link -> utlidar_lidar` (identity) replaces the alias
   that robot_lio publishes live; the bag's /lidar/points keeps the
   `utlidar_lidar` frame_id.
6. `lidar_level_frame_publisher` publishes the dynamic gravity-aligned
   `base_footprint -> lidar_stabilized` TF from /imu/data.
7. `distortion_corrector` consumes /lidar/points + /imu/data, publishes
   /lidar/points_undistorted.
8. `cloud_accumulator` rolls /lidar/points_undistorted into a short window
   on /lidar/points_scan_accumulated.
9. `pointcloud_to_laserscan` slices the accumulated cloud into /scan.
10. `async_slam_toolbox_node` owns `map -> odom` and publishes /map.
11. Nav2's `controller_server` + `planner_server` host the local and global
    costmaps; STVL marks live off the fresh /lidar/points_undistorted.
12. RViz (optional).

Nothing in the bag's /tf or /tf_static is consumed -- those streams are
dropped at playback by the wrapper script. URDF static frames now come
from robot_state_publisher so the bag's stale /robot_description (recorded
before the legs were added) does not race with the active URDF.

Arguments
---------
slam_params      : slam_toolbox YAML
distortion_params: distortion_corrector YAML
scan_params      : pointcloud_to_laserscan YAML
nav_params       : Nav2 YAML (default: nav2_params.yaml from robot_nav)
odom_topic       : where the bag's odometry arrives (default: /odom)
standing_height  : z offset for base_footprint -> base_link (default: 0.28)
nav_profile      : Nav2 profile (default: default_safe)  -- only used if
                   nav_params is empty.
use_rviz         : start RViz (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_nav_params(context):
    """If nav_params is empty, derive it from nav_profile."""
    nav_params = LaunchConfiguration("nav_params").perform(context)
    if nav_params:
        return [SetLaunchConfiguration("nav_params", nav_params)]

    share = get_package_share_directory("robot_nav")
    profile = LaunchConfiguration("nav_profile").perform(context)
    options = {
        "default_safe":    "nav2_params.yaml",
        "narrow_gap_1p0m": "nav2_params_narrow_gap_1p0m.yaml",
        "forward_static":  "nav2_params_forward_static.yaml",
    }
    chosen = options.get(profile, "nav2_params.yaml")
    return [SetLaunchConfiguration("nav_params", os.path.join(share, "config", chosen))]


def generate_launch_description():
    share = get_package_share_directory("robot_nav")

    default_slam_params       = os.path.join(share, "config", "slam_toolbox_params.yaml")
    # The bench-bag variant (distortion_corrector_bag.yaml) sets
    # sensor_mount_rpy to identity, gravity_align=false, filter_self=false --
    # all wrong for Go2 mapping bags. Use the production config so the
    # replayed cloud matches what the live system produced.
    default_distortion_params = os.path.join(share, "config", "distortion_corrector.yaml")
    default_scan_params       = os.path.join(share, "config", "scan_from_cloud.yaml")
    default_rviz_config       = os.path.join(share, "rviz", "nav2.rviz")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_params",       default_value=default_slam_params),
        DeclareLaunchArgument("distortion_params", default_value=default_distortion_params),
        DeclareLaunchArgument("scan_params",       default_value=default_scan_params),
        DeclareLaunchArgument("nav_params",        default_value=""),
        DeclareLaunchArgument("nav_profile",       default_value="default_safe"),
        DeclareLaunchArgument("odom_topic",        default_value="/odom"),
        DeclareLaunchArgument("standing_height",   default_value="0.28"),
        DeclareLaunchArgument("use_rviz",          default_value="true"),
        DeclareLaunchArgument("rviz_config",       default_value=default_rviz_config),
    ]

    # Fresh URDF -> robot_state_publisher avoids replaying the bag's stale
    # /robot_description (recorded before the legs were added to go2.urdf.xacro)
    # and lets RViz animate the leg links from /joint_states.
    default_urdf_xacro = PathJoinSubstitution([
        FindPackageShare("go2_description"), "urdf", "go2.urdf.xacro",
    ])

    args.append(DeclareLaunchArgument(
        "urdf_xacro", default_value=default_urdf_xacro,
        description="Robot description xacro for robot_state_publisher",
    ))

    use_sim_time      = LaunchConfiguration("use_sim_time")
    slam_params       = LaunchConfiguration("slam_params")
    distortion_params = LaunchConfiguration("distortion_params")
    scan_params       = LaunchConfiguration("scan_params")
    odom_topic        = LaunchConfiguration("odom_topic")
    standing_height   = LaunchConfiguration("standing_height")
    use_rviz          = LaunchConfiguration("use_rviz")
    rviz_config       = LaunchConfiguration("rviz_config")

    sim_time_params = {"use_sim_time": use_sim_time}

    # --- Robot description + joint states ---------------------------------
    # robot_state_publisher owns /robot_description and broadcasts the URDF's
    # static frames (base_link <-> trunk, lidar_link, imu_link, lidar_imu_link)
    # plus the revolute leg transforms driven by /joint_states.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": Command(["xacro ", LaunchConfiguration("urdf_xacro")]),
        }],
    )

    # /lowstate -> /joint_states bridge (go2_hw_bridge is intentionally absent
    # in replay; this is the joint-state slice only).
    lowstate_joint_states = Node(
        package="go2_bringup",
        executable="lowstate_joint_state_publisher.py",
        name="lowstate_joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- TF: odom -> base_footprint (from bag /odom topic) ----------------
    odom_to_tf = Node(
        package="robot_nav",
        executable="odom_to_tf.py",
        name="bag_replay_odom_to_tf",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "odom_topic": odom_topic,
            "parent_frame": "odom",
            "child_frame": "base_footprint",
        }],
    )

    # --- TF: base_footprint -> base_link (static body-height stand-in) ----
    base_footprint_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bag_replay_base_footprint_to_base_link",
        output="screen",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", standing_height,
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
    )

    # --- TF: lidar_link -> utlidar_lidar (identity alias) -----------------
    # Live, robot_lio publishes this so the Unitree driver frame on /lidar/points
    # resolves into the URDF chain. The bag's /lidar/points keeps that
    # frame_id, so the alias must also exist on replay.
    lidar_points_frame_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bag_replay_lidar_points_frame_alias",
        output="screen",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "lidar_link",
            "--child-frame-id", "utlidar_lidar",
        ],
    )

    # --- TF: base_footprint -> lidar_stabilized (gravity-aligned, IMU) ----
    # Use the node's defaults (Go2 URDF lidar offset + 0.28 m standing height).
    # Overriding these to zero would put the gravity-aligned lidar frame at
    # the body origin and silently misalign every downstream consumer
    # (STVL marks, /scan slice, slam_toolbox scan registration).
    lidar_level_frame = Node(
        package="robot_nav",
        executable="lidar_level_frame_publisher",
        name="lidar_level_frame_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "imu_topic": "/imu/data",
            "parent_frame": "base_footprint",
            "child_frame": "lidar_stabilized",
        }],
    )

    # --- Perception: distortion correct -> accumulate -> laserscan --------
    distortion_corrector = Node(
        package="robot_nav",
        executable="distortion_corrector",
        name="distortion_corrector",
        output="screen",
        parameters=[
            distortion_params,
            sim_time_params,
            {"imu_topic": "/imu/data"},
        ],
    )

    cloud_accumulator = Node(
        package="robot_nav",
        executable="cloud_accumulator.py",
        name="scan_cloud_accumulator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/lidar/points_undistorted",
            "output_topic": "/lidar/points_scan_accumulated",
            "fixed_frame": "odom",
            "accumulation_window_sec": 0.20,
            "max_frames": 3,
        }],
    )

    pc_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            scan_params,
            sim_time_params,
        ],
        remappings=[
            ("cloud_in", "/lidar/points_scan_accumulated"),
            ("scan",     "/scan"),
        ],
    )

    # --- SLAM: slam_toolbox owns map -> odom and /map ---------------------
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            sim_time_params,
        ],
    )

    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "save_map_timeout": 5.0,
            "free_thresh_default": 0.25,
            "occupied_thresh_default": 0.65,
            "map_subscribe_transient_local": True,
        }],
    )

    # slam_toolbox manages its own lifecycle; only map_saver needs activation.
    slam_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_saver"],
        }],
    )

    # --- Nav2 costmaps (just enough for STVL tuning) ----------------------
    # IncludeLaunchDescription of nav2.launch.py would also drag in
    # bt_navigator, behavior_server, collision_monitor, etc. -- noise for a
    # tuning flow. We launch only the two costmap-hosting servers.
    nav_params = LaunchConfiguration("nav_params")

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav_params, sim_time_params],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav_params, sim_time_params],
    )

    nav_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["controller_server", "planner_server"],
        }],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[sim_time_params],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        OpaqueFunction(function=_resolve_nav_params),
        robot_state_publisher,
        lowstate_joint_states,
        odom_to_tf,
        base_footprint_to_base_link,
        lidar_points_frame_alias,
        lidar_level_frame,
        distortion_corrector,
        cloud_accumulator,
        pc_to_scan,
        slam_toolbox,
        map_saver,
        slam_lifecycle_manager,
        controller_server,
        planner_server,
        nav_lifecycle_manager,
        rviz2,
    ])
