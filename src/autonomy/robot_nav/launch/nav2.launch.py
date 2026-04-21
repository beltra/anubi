"""
nav2.launch.py
Robot-agnostic Nav2 launch file.

Starts the Nav2 navigation nodes via nav2_bringup.
Localization (map->odom) is provided by slam_toolbox, started separately via slam.launch.py.
Odometry (odom->base_footprint) is provided by the robot adapter.

Arguments
---------
use_sim_time : bool   (default: false)
params_file  : str    Path to Nav2 YAML params (default: robot_nav default)
nav_profile  : str    Predefined Nav2 profile (default_safe|narrow_gap_1p0m)
namespace    : str    Robot namespace prefix (default: '')
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    robot_nav_share    = get_package_share_directory("robot_nav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_nav2_params_stvl = os.path.join(robot_nav_share, "config", "nav2_params.yaml")
    default_nav2_params_stvl_narrow = os.path.join(
        robot_nav_share, "config", "nav2_params_narrow_gap_1p0m.yaml"
    )
    default_nav2_params_2d = os.path.join(robot_nav_share, "config", "nav2_params_2d.yaml")
    default_collision_monitor_params = os.path.join(
        robot_nav_share, "config", "collision_monitor_params.yaml"
    )
    default_nav_to_pose_bt = os.path.join(
        robot_nav_share, "behavior_trees", "navigate_to_pose_industrial_safety.xml"
    )
    default_nav_through_poses_bt = os.path.join(
        robot_nav_share, "behavior_trees", "navigate_through_poses_industrial_safety.xml"
    )
    default_lattice_file = os.path.join(robot_nav_share, "config", "lattice_omni_5.json")

    declare_use_stvl = DeclareLaunchArgument(
        "use_stvl",
        default_value="true",
        description="Use SpatioTemporalVoxelLayer for 3D obstacle avoidance"
                    " in the local costmap. Set false to revert to 2D obstacle layer.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock",
    )
    declare_nav2_params_stvl = DeclareLaunchArgument(
        "nav2_params_stvl",
        default_value=default_nav2_params_stvl,
        description="Default STVL Nav2 params YAML",
    )
    declare_nav2_params_stvl_narrow = DeclareLaunchArgument(
        "nav2_params_stvl_narrow",
        default_value=default_nav2_params_stvl_narrow,
        description="STVL narrow-gap Nav2 params YAML (target 1.0 m passages)",
    )
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params",
        default_value=default_nav2_params_2d,
        description="Fallback 2D Nav2 params YAML",
    )
    declare_nav_profile = DeclareLaunchArgument(
        "nav_profile",
        default_value="default_safe",
        description="Navigation profile: default_safe or narrow_gap_1p0m",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PythonExpression([
            '"', LaunchConfiguration("nav2_params_stvl_narrow"), '"',
            ' if ("true" == "', LaunchConfiguration("use_stvl"),
            '" and "narrow_gap_1p0m" == "', LaunchConfiguration("nav_profile"), '")',
            ' else ("', LaunchConfiguration("nav2_params_stvl"), '"',
            ' if "true" == "', LaunchConfiguration("use_stvl"), '"',
            ' else "', LaunchConfiguration("nav2_params"), '")',
        ]),
        description="Full path to Nav2 params YAML",
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="",
        description="Robot namespace for multi-robot setups",
    )
    declare_collision_monitor_params = DeclareLaunchArgument(
        "collision_monitor_params",
        default_value=default_collision_monitor_params,
        description="Collision monitor params YAML",
    )
    declare_lattice_filepath = DeclareLaunchArgument(
        "lattice_filepath",
        default_value=default_lattice_file,
        description="Absolute path to Smac lattice primitive file",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file  = LaunchConfiguration("params_file")
    namespace    = LaunchConfiguration("namespace")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params")
    lattice_filepath = LaunchConfiguration("lattice_filepath")

    # SmacPlannerLattice expects an absolute lattice file path.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            "lattice_filepath": lattice_filepath,
            "use_sim_time": use_sim_time,
            "default_nav_to_pose_bt_xml": default_nav_to_pose_bt,
            "default_nav_through_poses_bt_xml": default_nav_through_poses_bt,
        },
        convert_types=True,
    )
    configured_collision_monitor_params = ParameterFile(
        RewrittenYaml(
            source_file=collision_monitor_params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Delegate node composition and lifecycle handling to nav2_bringup.
    # slam_toolbox is started separately via slam.launch.py.
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
        parameters=[configured_collision_monitor_params],
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

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        declare_use_stvl,
        declare_use_sim_time,
        declare_nav2_params_stvl,
        declare_nav2_params_stvl_narrow,
        declare_nav2_params,
        declare_nav_profile,
        declare_params_file,
        declare_namespace,
        declare_collision_monitor_params,
        declare_lattice_filepath,
        nav2_bringup,
        collision_monitor,
        collision_monitor_lifecycle,
    ])
