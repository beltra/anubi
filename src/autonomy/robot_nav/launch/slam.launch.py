"""
slam.launch.py -- 2D SLAM pipeline using slam_toolbox.

Starts: distortion_corrector -> pointcloud_to_laserscan -> slam_toolbox (mapping or localization)
        + map_saver + lifecycle_manager_localization

Arguments
---------
use_sim_time                : bool   (default: false)
slam_params                 : str    Path to slam_toolbox YAML
scan_params                 : str    Path to pointcloud_to_laserscan YAML
distortion_params           : str    Path to distortion_corrector YAML
slam_mode                   : str    'mapping' (default) | 'localization'
map                         : str    Path to .posegraph file (localization mode)
autostart                   : bool   (default: true)
bypass_distortion_corrector : bool   Skip deskew (use in simulation)
distortion_imu_topic       : str    IMU topic used for deskew/gravity alignment
accumulate_scan_cloud       : bool   Accumulate non-repetitive LiDAR frames before /scan
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _validate_localization_inputs(context):
    """Fail fast if localization mode is requested without a valid map file."""
    mode = LaunchConfiguration("slam_mode").perform(context).strip()
    map_file = LaunchConfiguration("map").perform(context).strip()

    if mode == "localization":
        if not map_file:
            raise RuntimeError(
                "slam_mode:=localization requires map:=<path/to/map.posegraph>."
            )
        # slam_toolbox expects the base path WITHOUT .posegraph extension.
        # Accept both forms; always check for the .posegraph file.
        posegraph = map_file if map_file.endswith(".posegraph") else map_file + ".posegraph"
        if not os.path.isfile(posegraph):
            raise RuntimeError(
                f"Localization map file not found: '{posegraph}'. "
                "Provide a valid slam_toolbox .posegraph file path via map:=..."
            )

    return []


def generate_launch_description():

    robot_nav_share = get_package_share_directory("robot_nav")

    default_slam_params  = os.path.join(robot_nav_share, "config", "slam_toolbox_params.yaml")
    default_scan_params  = os.path.join(robot_nav_share, "config", "scan_from_cloud.yaml")
    default_distortion_params = os.path.join(
        robot_nav_share, "config", "distortion_corrector.yaml"
    )

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_params",  default_value=default_slam_params),
        DeclareLaunchArgument("scan_params",  default_value=default_scan_params),
        DeclareLaunchArgument("distortion_params", default_value=default_distortion_params),
        DeclareLaunchArgument("slam_mode",    default_value="mapping",
                              description="'mapping' or 'localization'"),
        DeclareLaunchArgument("map",          default_value="",
                              description="Path to .posegraph file for localization mode"),
        DeclareLaunchArgument("autostart",    default_value="true"),
        DeclareLaunchArgument(
            "bypass_distortion_corrector", default_value="false",
            description="Skip deskew (use in simulation where scans are instantaneous)",
        ),
        DeclareLaunchArgument(
            "distortion_imu_topic", default_value="/imu/data",
            description="IMU topic used by distortion_corrector",
        ),
        DeclareLaunchArgument(
            "scan_cloud_topic", default_value="/lidar/points_undistorted",
            description="PointCloud2 topic consumed by pointcloud_to_laserscan",
        ),
        DeclareLaunchArgument(
            "accumulate_scan_cloud", default_value="false",
            description="Accumulate short non-repetitive LiDAR windows before /scan",
        ),
        DeclareLaunchArgument(
            "scan_accumulated_cloud_topic",
            default_value="/lidar/points_scan_accumulated",
            description="Accumulated PointCloud2 topic consumed by pointcloud_to_laserscan",
        ),
        # Defaults match cloud_accumulator.py: a 3-frame / 0.20 s window suits
        # the widened +/-0.30 m /scan slice. The old 0.75 s / 12-frame window
        # was sized for the much sparser +/-0.12 m slice.
        DeclareLaunchArgument("scan_accumulation_window_sec", default_value="0.20"),
        DeclareLaunchArgument("scan_accumulation_max_frames", default_value="3"),
        # Yaw gate for the mapping scan: drop sweeps captured above this yaw
        # rate (rad/s) so rotation-smeared sweeps never paint the map. 0.0
        # disables the gate. Only meaningful for the accumulated (mapping)
        # path; safety consumers must keep using the ungated cloud.
        DeclareLaunchArgument("scan_accumulation_max_yaw_rate", default_value="0.0"),
        # scan_cloud_topic already expressed in odom (e.g. DLIO's registered
        # /lidar/points_registered): accumulate by concatenation and apply the
        # near-body radius filter (that cloud bypasses the distortion
        # corrector's body-box self-filter).
        DeclareLaunchArgument("scan_cloud_in_fixed_frame", default_value="false"),
        DeclareLaunchArgument("scan_self_filter_radius", default_value="0.0"),
        # Skip slam_toolbox itself (mapping AND localization nodes) while
        # still running the perception pipeline that produces /scan. Used by
        # bag-replay flows where the bag already carries /map and the
        # `map -> odom` TF, and a second slam_toolbox would create a
        # competing TF publisher.
        DeclareLaunchArgument("start_slam_toolbox", default_value="true"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params  = LaunchConfiguration("slam_params")
    scan_params  = LaunchConfiguration("scan_params")
    distortion_params = LaunchConfiguration("distortion_params")
    slam_mode    = LaunchConfiguration("slam_mode")
    map_path     = LaunchConfiguration("map")
    autostart    = LaunchConfiguration("autostart")
    bypass_distortion_corrector = LaunchConfiguration("bypass_distortion_corrector")
    distortion_imu_topic = LaunchConfiguration("distortion_imu_topic")
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic")
    accumulate_scan_cloud = LaunchConfiguration("accumulate_scan_cloud")
    scan_accumulated_cloud_topic = LaunchConfiguration("scan_accumulated_cloud_topic")
    scan_accumulation_window_sec = LaunchConfiguration("scan_accumulation_window_sec")
    scan_accumulation_max_frames = LaunchConfiguration("scan_accumulation_max_frames")
    start_slam_toolbox = LaunchConfiguration("start_slam_toolbox")
    scan_input_topic = PythonExpression([
        "'", scan_accumulated_cloud_topic, "' if '", accumulate_scan_cloud,
        "' == 'true' else '", scan_cloud_topic, "'",
    ])

    is_mapping       = PythonExpression([
        "'", slam_mode, "' == 'mapping' and '", start_slam_toolbox, "' == 'true'",
    ])
    is_localization  = PythonExpression([
        "'", slam_mode, "' == 'localization' and '", start_slam_toolbox, "' == 'true'",
    ])

    distortion_corrector = Node(
        package="robot_nav",
        executable="distortion_corrector",
        name="distortion_corrector",
        output="screen",
        parameters=[
            distortion_params,
            {
                "use_sim_time": use_sim_time,
                "imu_topic": distortion_imu_topic,
            },
        ],
        condition=UnlessCondition(bypass_distortion_corrector),
    )

    scan_cloud_accumulator = Node(
        package="robot_nav",
        executable="cloud_accumulator.py",
        name="scan_cloud_accumulator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": scan_cloud_topic,
            "output_topic": scan_accumulated_cloud_topic,
            "accumulation_window_sec": scan_accumulation_window_sec,
            "max_frames": scan_accumulation_max_frames,
            "max_yaw_rate_rad_s": LaunchConfiguration("scan_accumulation_max_yaw_rate"),
            "input_cloud_in_fixed_frame": LaunchConfiguration("scan_cloud_in_fixed_frame"),
            "self_filter_radius": LaunchConfiguration("scan_self_filter_radius"),
        }],
        condition=IfCondition(accumulate_scan_cloud),
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            scan_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cloud_in", scan_input_topic),
            ("scan",     "/scan"),
        ],
    )

    slam_mapping = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(is_mapping),
    )

    # slam_toolbox appends .posegraph itself; strip the extension so either
    # "my_map" or "my_map.posegraph" from the caller works correctly.
    map_file_name = PythonExpression(["'", map_path, "'.removesuffix('.posegraph')"])

    slam_localization = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim_time,
                "map_file_name": map_file_name,
            },
        ],
        condition=IfCondition(is_localization),
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

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["map_saver"],
        }],
    )

    return LaunchDescription(args + [
        OpaqueFunction(function=_validate_localization_inputs),
        distortion_corrector,
        scan_cloud_accumulator,
        pointcloud_to_laserscan,
        slam_mapping,
        slam_localization,
        map_saver,
        lifecycle_manager_localization,
    ])
