"""
slam.launch.py — 2D SLAM pipeline using slam_toolbox.

Starts: distortion_corrector -> pointcloud_to_laserscan -> slam_toolbox (mapping or
        posegraph localization) OR map_server + AMCL when localization map is a
        Nav2 YAML occupancy map.

Arguments
---------
use_sim_time                : bool   (default: false)
slam_params                 : str    Path to slam_toolbox YAML
scan_params                 : str    Path to pointcloud_to_laserscan YAML
distortion_params           : str    Path to distortion_corrector YAML
slam_mode                   : str    'mapping' (default) | 'localization'
map                         : str    Path to .posegraph or Nav2 .yaml map (localization mode)
localization_params         : str    Nav2 YAML containing amcl/map_server params
autostart                   : bool   (default: true)
bypass_distortion_corrector : bool   Skip deskew (use in simulation)
distortion_imu_topic       : str    IMU topic used for deskew/gravity alignment
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
        if map_file.endswith((".yaml", ".yml")):
            if not os.path.isfile(map_file):
                raise RuntimeError(
                    f"Localization occupancy map YAML not found: '{map_file}'."
                )
            return []

        # slam_toolbox expects the base path WITHOUT .posegraph extension. Accept
        # both forms; always check for the .posegraph file.
        posegraph = map_file if map_file.endswith(".posegraph") else map_file + ".posegraph"
        if not os.path.isfile(posegraph):
            raise RuntimeError(
                f"Localization map file not found: '{posegraph}'. "
                "Provide a valid slam_toolbox .posegraph or Nav2 .yaml map via map:=..."
            )

    return []


def generate_launch_description():

    robot_nav_share = get_package_share_directory("robot_nav")

    default_slam_params  = os.path.join(robot_nav_share, "config", "slam_toolbox_params.yaml")
    default_scan_params  = os.path.join(robot_nav_share, "config", "scan_from_cloud.yaml")
    default_distortion_params = os.path.join(
        robot_nav_share, "config", "distortion_corrector.yaml"
    )
    default_localization_params = os.path.join(robot_nav_share, "config", "nav2_params.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_params",  default_value=default_slam_params),
        DeclareLaunchArgument("scan_params",  default_value=default_scan_params),
        DeclareLaunchArgument("distortion_params", default_value=default_distortion_params),
        DeclareLaunchArgument("slam_mode",    default_value="mapping",
                              description="'mapping' or 'localization'"),
        DeclareLaunchArgument("map",          default_value="",
                              description="Path to .posegraph or Nav2 .yaml map for localization mode"),
        DeclareLaunchArgument(
            "localization_params",
            default_value=default_localization_params,
            description="Nav2 params YAML used by AMCL/map_server for .yaml map localization",
        ),
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
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params  = LaunchConfiguration("slam_params")
    scan_params  = LaunchConfiguration("scan_params")
    distortion_params = LaunchConfiguration("distortion_params")
    slam_mode    = LaunchConfiguration("slam_mode")
    map_path     = LaunchConfiguration("map")
    localization_params = LaunchConfiguration("localization_params")
    autostart    = LaunchConfiguration("autostart")
    bypass_distortion_corrector = LaunchConfiguration("bypass_distortion_corrector")
    distortion_imu_topic = LaunchConfiguration("distortion_imu_topic")
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic")

    is_mapping       = PythonExpression(["'", slam_mode, "' == 'mapping'"])
    is_localization  = PythonExpression(["'", slam_mode, "' == 'localization'"])
    is_yaml_localization = PythonExpression([
        "'",
        slam_mode,
        "' == 'localization' and '",
        map_path,
        "'.endswith(('.yaml', '.yml'))",
    ])
    is_posegraph_localization = PythonExpression([
        "'",
        slam_mode,
        "' == 'localization' and not '",
        map_path,
        "'.endswith(('.yaml', '.yml'))",
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
            ("cloud_in", scan_cloud_topic),
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
        condition=IfCondition(is_posegraph_localization),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            localization_params,
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_path,
            },
        ],
        condition=IfCondition(is_yaml_localization),
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            localization_params,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(is_yaml_localization),
    )

    lifecycle_manager_amcl = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_amcl",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["map_server", "amcl"],
        }],
        condition=IfCondition(is_yaml_localization),
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
        pointcloud_to_laserscan,
        slam_mapping,
        slam_localization,
        map_server,
        amcl,
        lifecycle_manager_amcl,
        map_saver,
        lifecycle_manager_localization,
    ])
