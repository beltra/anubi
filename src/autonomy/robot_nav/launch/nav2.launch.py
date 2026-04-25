"""
nav2.launch.py -- Robot-agnostic Nav2 launch file.

Starts the Nav2 navigation stack via nav2_bringup.
Localization (map->odom) is provided by slam_toolbox (slam.launch.py).
Odometry (odom->base_footprint) is provided by the robot adapter.

Arguments
---------
use_sim_time        : bool   (default: false)
params_file         : str    Explicit Nav2 YAML override; if empty, nav_profile drives selection
nav_profile         : str    Predefined Nav2 profile (default_safe|narrow_gap_1p0m|forward_static)
use_stvl            : bool   Use SpatioTemporalVoxelLayer (default: true)
namespace           : str    Robot namespace prefix (default: '')
use_route_server    : bool   Launch route_server scaffold (default: false)
route_graph_file    : str    Path to route graph file (default: '')
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    robot_nav_share    = get_package_share_directory("robot_nav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    _params_stvl         = os.path.join(robot_nav_share, "config", "nav2_params.yaml")
    _params_stvl_narrow  = os.path.join(robot_nav_share, "config", "nav2_params_narrow_gap_1p0m.yaml")
    _params_2d           = os.path.join(robot_nav_share, "config", "nav2_params_2d.yaml")
    _params_fwd_static   = os.path.join(robot_nav_share, "config", "nav2_params_forward_static.yaml")
    _params_collision    = os.path.join(robot_nav_share, "config", "collision_monitor_params.yaml")
    _bt_nav_to_pose      = os.path.join(robot_nav_share, "behavior_trees", "navigate_to_pose_industrial_safety.xml")
    _bt_nav_through      = os.path.join(robot_nav_share, "behavior_trees", "navigate_through_poses_industrial_safety.xml")
    _lattice_file        = os.path.join(robot_nav_share, "config", "lattice_omni_5.json")

    # Profile -> YAML mapping resolved at launch time via OpaqueFunction.
    _profile_map = {
        "default_safe":    _params_stvl,
        "narrow_gap_1p0m": _params_stvl_narrow,
        "forward_static":  _params_fwd_static,
    }

    args = [
        DeclareLaunchArgument(
            "use_stvl",
            default_value="true",
            description="Use SpatioTemporalVoxelLayer in the local costmap",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "nav_profile",
            default_value="default_safe",
            description="Navigation profile: default_safe | narrow_gap_1p0m | forward_static",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value="",
            description="Explicit Nav2 params YAML path; overrides nav_profile when non-empty",
        ),
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="Robot namespace for multi-robot setups",
        ),
        DeclareLaunchArgument(
            "collision_monitor_params",
            default_value=_params_collision,
            description="Collision monitor params YAML",
        ),
        DeclareLaunchArgument(
            "lattice_filepath",
            default_value=_lattice_file,
            description="Absolute path to Smac lattice primitive file",
        ),
        DeclareLaunchArgument(
            "use_route_server",
            default_value="false",
            description="Launch optional route_server scaffold (requires a route graph file)",
        ),
        DeclareLaunchArgument(
            "route_graph_file",
            default_value="",
            description="Path to route graph GeoJSON file (required when use_route_server:=true)",
        ),
    ]

    def resolve_params_file(context):
        explicit = context.launch_configurations.get("params_file", "")
        if explicit:
            return []  # caller provided an explicit path; keep it
        use_stvl_val  = context.launch_configurations.get("use_stvl", "true")
        nav_profile_val = context.launch_configurations.get("nav_profile", "default_safe")
        if use_stvl_val.lower() != "true":
            path = _params_2d
        else:
            path = _profile_map.get(nav_profile_val, _params_stvl)
        return [SetLaunchConfiguration("params_file", path)]

    use_sim_time              = LaunchConfiguration("use_sim_time")
    params_file               = LaunchConfiguration("params_file")
    namespace                 = LaunchConfiguration("namespace")
    collision_monitor_params  = LaunchConfiguration("collision_monitor_params")
    lattice_filepath          = LaunchConfiguration("lattice_filepath")
    use_route_server          = LaunchConfiguration("use_route_server")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            "lattice_filepath":                   lattice_filepath,
            "graph_filepath":                     LaunchConfiguration("route_graph_file"),
            "use_sim_time":                       use_sim_time,
            "default_nav_to_pose_bt_xml":         _bt_nav_to_pose,
            "default_nav_through_poses_bt_xml":   _bt_nav_through,
        },
        convert_types=True,
    )
    configured_collision_monitor_params = ParameterFile(
        collision_monitor_params,
        allow_substs=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file":  configured_params,
            "namespace":    namespace,
            "autostart":    "true",
        }.items(),
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            configured_collision_monitor_params,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=remappings,
    )
    collision_monitor_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["collision_monitor"]},
        ],
    )

    route_server = Node(
        package="nav2_route",
        executable="route_server",
        name="route_server",
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=remappings,
        condition=IfCondition(use_route_server),
    )
    route_server_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_route_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["route_server"]},
        ],
        condition=IfCondition(use_route_server),
    )

    return LaunchDescription(args + [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        OpaqueFunction(function=resolve_params_file),
        nav2_bringup,
        collision_monitor,
        collision_monitor_lifecycle,
        route_server,
        route_server_lifecycle,
    ])
