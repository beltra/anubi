"""
nav2.launch.py
Robot-agnostic Nav2 launch file.

Starts the Nav2 navigation nodes via nav2_bringup.
Localisation (map→odom) is provided by slam_toolbox, started separately via slam.launch.py.
Odometry (odom→base_footprint) is provided by go2_hw_bridge (or equivalent adapter).

Arguments
---------
use_sim_time : bool   (default: false)
params_file  : str    Path to Nav2 YAML params (default: robot_nav default)
namespace    : str    Robot namespace prefix (default: '')
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    robot_nav_share    = get_package_share_directory("robot_nav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_nav2_params_stvl = os.path.join(robot_nav_share, "config", "nav2_params.yaml")
    default_nav2_params_2d = os.path.join(robot_nav_share, "config", "nav2_params_2d.yaml")

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
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params",
        default_value=default_nav2_params_2d,
        description="Fallback 2D Nav2 params YAML",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PythonExpression([
            '"', LaunchConfiguration("nav2_params_stvl"), '"',
            ' if "true" == "', LaunchConfiguration("use_stvl"), '"',
            ' else "', LaunchConfiguration("nav2_params"), '"',
        ]),
        description="Full path to Nav2 params YAML",
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="",
        description="Robot namespace for multi-robot setups",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file  = LaunchConfiguration("params_file")
    namespace    = LaunchConfiguration("namespace")

    # Delegate to nav2_bringup for standard node composition and lifecycle management.
    # slam=false: slam_toolbox is started separately via slam.launch.py.
    # map is not passed: map_server is not in the lifecycle node list (slam_toolbox
    # provides /map directly).
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file":  params_file,
            "namespace":    namespace,
            "autostart":    "true",
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        declare_use_stvl,
        declare_use_sim_time,
        declare_nav2_params_stvl,
        declare_nav2_params,
        declare_params_file,
        declare_namespace,
        nav2_bringup,
    ])
