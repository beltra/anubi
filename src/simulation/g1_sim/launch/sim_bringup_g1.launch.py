"""
sim_bringup_g1.launch.py - Full Gazebo Fortress + Nav2 simulation bringup for the
Unitree G1 humanoid.

Humanoid analogue of robot_sim/sim_bringup.launch.py. Starts:
  1. g1_sim/sim_g1.launch.py        (Gazebo, kinematic base, bridges, EKF, TF)
  2. distortion_corrector           (/lidar/points -> /lidar/points_undistorted)
  3. (optional) robot_lio           (DLIO backend, off by default)
  4. robot_nav/slam.launch.py       (optional via start_nav:=true)
  5. robot_nav/nav2.launch.py       (G1 profile nav2_params_g1.yaml, or the SONIC
                                     envelope variants via locomotion:=sonic)
  6. RViz2                          (optional)

The autonomy stack (robot_nav, robot_lio, robot_safety) is reused UNCHANGED - the
only G1-specific inputs are the kinematic sim model, the Mid-360 mount rpy fed to
distortion_corrector, and the nav2_params_g1*.yaml profile (footprint + velocity caps).

locomotion / sonic_profile select ONLY the Nav2 velocity envelope here: the sim
base is a kinematic VelocityControl ghost either way (no gait is simulated), so
these args exist to exercise the SONIC Nav2 profiles before hardware runs.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    g1_sim_share = get_package_share_directory("g1_sim")
    sim_worlds_share = get_package_share_directory("sim_worlds")
    robot_nav_share = get_package_share_directory("robot_nav")

    default_world = os.path.join(sim_worlds_share, "worlds", "indoor_office.world")
    # G1-specific slam_toolbox tuning (trusts DLIO/ground-truth odometry; the shared
    # slam_toolbox_params.yaml stays loose for the Go2's drifting leg odometry).
    default_slam_params = os.path.join(robot_nav_share, "config", "slam_toolbox_params_g1.yaml")
    default_rviz_config = os.path.join(robot_nav_share, "rviz", "nav2.rviz")
    g1_nav2_params = os.path.join(robot_nav_share, "config", "nav2_params_g1.yaml")
    # SONIC velocity-envelope variants (same files g1_bringup.launch.py uses on
    # the real robot), selected by locomotion:=sonic + sonic_profile.
    g1_nav2_params_sonic_slow = os.path.join(
        robot_nav_share, "config", "nav2_params_g1_sonic_slow.yaml"
    )
    g1_nav2_params_sonic_walk = os.path.join(
        robot_nav_share, "config", "nav2_params_g1_sonic_walk.yaml"
    )
    g1_collision_params = os.path.join(
        robot_nav_share, "config", "collision_monitor_params_g1.yaml"
    )
    # G1-specific pointcloud_to_laserscan band (tall: knees..head), separate from the
    # Go2 scan_from_cloud.yaml because the Mid-360 sits ~1.26 m up inside the head.
    g1_scan_params = os.path.join(robot_nav_share, "config", "scan_from_cloud_g1.yaml")
    # SHARED self-filter / sensor-geometry config (same file the real g1_bringup uses),
    # so the body box and Mid-360 mount are a single source of truth for sim and real.
    # Only the sim-only leveling flags are overridden on the node below.
    g1_distortion_params = os.path.join(
        robot_nav_share, "config", "distortion_corrector_g1.yaml"
    )

    args = [
        DeclareLaunchArgument("use_stvl", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("world_name", default_value="indoor_office"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("robot_name", default_value="g1"),
        DeclareLaunchArgument("world_init_x", default_value="0.0"),
        DeclareLaunchArgument("world_init_y", default_value="0.0"),
        DeclareLaunchArgument("world_init_z", default_value="0.80"),
        DeclareLaunchArgument("world_init_heading", default_value="0.0"),
        DeclareLaunchArgument("publish_map_to_odom_tf", default_value="false"),
        # Odometry: default to the Gazebo ground-truth OdometryPublisher + EKF, which
        # navigates reliably. Pass use_lio:=true to run DLIO (Mid-360 LiDAR-inertial
        # odometry) as on the real robot - it tracks well at rest but currently degrades
        # during the KINEMATIC base's motion (velocity steps -> IMU accel spikes); see
        # docs/g1.md. run_g1_sim.sh exposes this as --use-lio.
        DeclareLaunchArgument("use_lio", default_value="false"),
        DeclareLaunchArgument("lio_start_before_nav_sec", default_value="4.0"),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument(
            "nav_delay_sec", default_value="10.0",
            description=(
                "Delay before SLAM starts. The kinematic base is ready quickly; this only "
                "needs to cover Gazebo + bridge + TF settle time (no stand-up delay)."
            ),
        ),
        DeclareLaunchArgument("slam_mode", default_value="mapping"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("slam_params", default_value=default_slam_params),
        DeclareLaunchArgument("distortion_imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument(
            "locomotion", default_value="amo",
            description="Nav2 velocity envelope to exercise: 'amo' (nav2_params_g1) or "
                        "'sonic' (SONIC profiles). Sim locomotion itself is unchanged."),
        DeclareLaunchArgument(
            "sonic_profile", default_value="slow_walk",
            description="SONIC envelope when locomotion:=sonic: 'slow_walk' or 'walk'; "
                        "anything but 'walk' falls back to slow_walk."),
    ]

    start_nav = LaunchConfiguration("start_nav")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_stvl = LaunchConfiguration("use_stvl")
    use_lio = LaunchConfiguration("use_lio")

    # Same selection logic as g1_bringup.launch.py: an unknown sonic_profile
    # degrades to the conservative slow_walk file.
    nav2_params_file = PythonExpression([
        "('", g1_nav2_params_sonic_walk, "' if '", LaunchConfiguration("sonic_profile"),
        "' == 'walk' else '", g1_nav2_params_sonic_slow, "') if '",
        LaunchConfiguration("locomotion"), "' == 'sonic' else '", g1_nav2_params, "'",
    ])

    g1_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(g1_sim_share, "launch", "sim_g1.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_lio": use_lio,
            "rviz": "false",
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "gui": LaunchConfiguration("gui"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "publish_map_to_odom_tf": LaunchConfiguration("publish_map_to_odom_tf"),
        }.items(),
    )

    def maybe_include_lio(context):
        if use_lio.perform(context).lower() != "true":
            return []
        nav_delay_sec = float(LaunchConfiguration("nav_delay_sec").perform(context))
        lio_start_before_nav_sec = float(
            LaunchConfiguration("lio_start_before_nav_sec").perform(context)
        )
        lio_start_delay_sec = max(0.0, nav_delay_sec - lio_start_before_nav_sec)
        try:
            robot_lio_share = get_package_share_directory("robot_lio")
        except PackageNotFoundError:
            raise RuntimeError("use_lio:=true but package 'robot_lio' was not found.")
        # G1 Mid-360 sim extrinsics (near-identity upright mount).
        dlio_sim_params = os.path.join(robot_lio_share, "config", "dlio_params_g1_sim.yaml")
        return [
            TimerAction(
                period=lio_start_delay_sec,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(robot_lio_share, "launch", "lio.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "use_lio": use_lio,
                            "start_lidar_points_relay": "false",
                            "start_lidar_imu_relay": "false",
                            "publish_lidar_imu_tf": "false",
                            "dlio_params_file": dlio_sim_params,
                        }.items(),
                    )
                ],
            )
        ]

    lio = OpaqueFunction(function=maybe_include_lio)

    # Reuse the autonomy distortion_corrector with the SHARED G1 config
    # (distortion_corrector_g1.yaml): the self-filter body box, sensor geometry,
    # mount rpy and topics come from that single file so they cannot drift between
    # sim and real. Only the leveling flags are overridden here, because the
    # kinematic sim base is always perfectly level and its IMU cannot sense gravity
    # (VelocityControl makes it kinematic), so IMU leveling / 3D correction / gravity
    # alignment are both unnecessary and unreliable in sim. The real robot walks and
    # keeps them ON (see the YAML); the level lidar_stabilized TF handles leveling here.
    lidar_cloud_processor = Node(
        package="robot_nav",
        executable="distortion_corrector",
        name="distortion_corrector",
        output="screen",
        parameters=[
            g1_distortion_params,
            {
                "use_sim_time": use_sim_time,
                "imu_topic": LaunchConfiguration("distortion_imu_topic"),
                "use_imu": False,
                "use_3d_distortion_correction": False,
                "gravity_align": False,
                # gpu_lidar clouds are instantaneous (no spin): the shared
                # config's deskew would fall back to azimuth timing and warp
                # the cloud whenever the base yaws.
                "deskew_enabled": False,
            },
        ],
        condition=IfCondition(start_nav),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "slam.launch.py")
        ),
        condition=IfCondition(start_nav),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_mode": LaunchConfiguration("slam_mode"),
            "slam_params": LaunchConfiguration("slam_params"),
            "map": LaunchConfiguration("map"),
            "bypass_distortion_corrector": "true",
            "scan_cloud_topic": "/lidar/points_undistorted",
            "scan_params": g1_scan_params,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "nav2.launch.py")
        ),
        condition=IfCondition(start_nav),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_stvl": use_stvl,
            # Explicit G1 profile (footprint + velocity caps) overrides nav_profile;
            # AMO or SONIC envelope picked by locomotion + sonic_profile.
            "params_file": nav2_params_file,
            # G1 collision monitor: approach + slowdown + stop polygons.
            "collision_monitor_params": g1_collision_params,
        }.items(),
    )

    start_nav_stack = TimerAction(
        period=LaunchConfiguration("nav_delay_sec"),
        actions=[
            slam,
            TimerAction(period=10.0, actions=[nav2]),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(args + [
        g1_sim,
        lio,
        lidar_cloud_processor,
        start_nav_stack,
        rviz,
    ])
