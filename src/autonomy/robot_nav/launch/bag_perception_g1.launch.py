"""
bag_perception_g1.launch.py -- minimal offline replay stack for tuning the G1
perception pipeline (distortion corrector self-filter, /scan band, slam_toolbox,
Nav2/STVL) against the recorded G1 LiDAR bag (bags/g1/lidar_record).

The G1 counterpart of bag_perception.launch.py (Go2). The wrapper script
(scripts/replay_bag_perception_g1.sh) plays the bag with a strict topic
whitelist: /livox/lidar_filtered + /livox/imu ONLY. Unlike the Go2 flow, even
the bag's /tf_static is dropped: it carries an IDENTITY base_link ->
base_footprint (base_footprint co-located with the pelvis, from the Unitree
onboard stack), which would both fight this launch's floor-level
base_footprint and shift the odom z-datum by the standing height -- and the
STVL obstacle heights being tuned here are odom-frame z values that must mean
the same thing as on the live stack (floor at odom z=0). Every TF is
regenerated fresh inside this launch.

What runs
---------
1. topic_tools relay + imu_rescale: /livox/lidar_filtered -> /lidar/points and
   /livox/imu (accel in g) -> /lidar/imu (m/s^2) -- the canonical sensor names
   (via robot_lio lio.launch.py for the cloud, matching live G1 wiring).
2. robot_state_publisher (g1.urdf.xacro) + joint_state_publisher (zeros): owns
   /robot_description and the URDF chain base_link -> ... -> mid360_link at
   the fixed standing pose.
3. Static base_footprint -> base_link (z = standing_height): floor-level
   base_footprint, standing in for g1_state_bridge/EKF.
4. Static mid360_link -> livox_frame (roll = pi): the bag cloud's frame. The
   recording was made WITHOUT the live driver's MID360_config.json roll:180
   righting, so the cloud is inverted; this transform (measured from the bag's
   own /tf_static) encodes the physical inverted mount so TF consumers
   (pointcloud_to_laserscan, STVL) right the cloud correctly.
5. Static base_footprint -> lidar_stabilized (upright, at sensor height):
   replaces lidar_level_frame_publisher, which needs the body IMU the bag does
   not have. Accepts a few degrees of gait tilt (plan decision 4).
6. DLIO (via robot_lio lio.launch.py) with dlio_params_g1_bag.yaml: fresh
   LiDAR-inertial odometry, publish_tf on -- owns odom -> base_footprint
   (Replay TF Owner Override, ADR 0012; frames/baselink is base_footprint so
   the floor lands at odom z=0, see the config header).
7. slam.launch.py: distortion_corrector (distortion_corrector_g1_bag.yaml)
   -> pointcloud_to_laserscan (scan_from_cloud_g1.yaml) -> /scan
   -> slam_toolbox (owns map -> odom, publishes /map) + map_saver.
   Scan accumulation + the 0.20 rad/s mapping yaw gate are ON by default,
   matching live G1 bringup.
8. Nav2 controller_server + planner_server hosting the local/global costmaps
   (STVL marks off /lidar/points_undistorted). Just enough for costmap tuning;
   bt_navigator/behaviors/collision_monitor are noise for this flow.
9. RViz (optional).

Resulting TF tree
-----------------
  map --(slam_toolbox)--> odom --(DLIO)--> base_footprint
  base_footprint --(static, z=standing_height)--> base_link --(URDF)--> ...
      --> mid360_link --(static, roll=pi)--> livox_frame     (cloud frame)
  base_footprint --(static, upright at sensor height)--> lidar_stabilized

Live-stack recordings (bags recorded by run_g1_real.sh --record-rosbag) carry the
UPRIGHT driver cloud on /livox/lidar plus the body IMU /imu/data. For those,
the wrapper selects the live-stack profile: lidar_input_topic:=/livox/lidar,
use_body_imu:=true (lidar_level_frame_publisher replaces the static upright
lidar_stabilized, and the LIVE distortion_corrector_g1.yaml runs with its
IMU-driven gravity alignment), and the upright-cloud DLIO config
(dlio_params_g1_bag_live.yaml).

Arguments
---------
slam_params       : slam_toolbox YAML (default: live shared slam_toolbox_params.yaml)
distortion_params : distortion_corrector YAML (default: distortion_corrector_g1_bag.yaml)
scan_params       : pointcloud_to_laserscan YAML (default: live scan_from_cloud_g1.yaml)
nav_params        : Nav2 YAML (default: live nav2_params_g1.yaml)
dlio_params       : DLIO YAML (default: dlio_params_g1_bag.yaml)
lidar_input_topic : bag cloud topic relayed to /lidar/points (default: /livox/lidar_filtered)
use_body_imu      : body IMU /imu/data is in the bag: dynamic gait leveling via
                    lidar_level_frame_publisher instead of the static upright
                    lidar_stabilized TF (default: false)
distortion_imu_topic : IMU for the distortion corrector (default: /lidar/imu;
                    the wrapper passes /imu/data in live-stack mode)
standing_height   : base_footprint -> base_link z offset (default: 0.78)
accumulate_scan_cloud : accumulate LiDAR frames before /scan (default: true, as live)
scan_accumulation_max_yaw_rate : drop mapping sweeps above this yaw rate
                    [rad/s] (default: 0.20, as live; 0 disables)
use_rviz          : start RViz (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Mid-360 position in base_link(pelvis), from the official URDF; must match
# sensor_*_in_base in the distortion corrector config and the DLIO extrinsics.
SENSOR_X_IN_BASE = "-0.00368"
SENSOR_Y_IN_BASE = "0.00003"
SENSOR_Z_IN_BASE = "0.46018"


def generate_launch_description():
    robot_nav_share = get_package_share_directory("robot_nav")
    robot_lio_share = get_package_share_directory("robot_lio")
    g1_desc_share = get_package_share_directory("g1_description")

    default_slam_params = os.path.join(robot_nav_share, "config", "slam_toolbox_params_g1.yaml")
    default_distortion_params = os.path.join(
        robot_nav_share, "config", "distortion_corrector_g1_bag.yaml")
    default_scan_params = os.path.join(robot_nav_share, "config", "scan_from_cloud_g1.yaml")
    default_nav_params = os.path.join(robot_nav_share, "config", "nav2_params_g1.yaml")
    default_dlio_params = os.path.join(robot_lio_share, "config", "dlio_params_g1_bag.yaml")
    default_rviz_config = os.path.join(robot_nav_share, "rviz", "nav2.rviz")
    urdf_file = os.path.join(g1_desc_share, "urdf", "g1.urdf.xacro")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_params", default_value=default_slam_params),
        DeclareLaunchArgument("distortion_params", default_value=default_distortion_params),
        DeclareLaunchArgument("scan_params", default_value=default_scan_params),
        DeclareLaunchArgument("nav_params", default_value=default_nav_params),
        DeclareLaunchArgument("dlio_params", default_value=default_dlio_params),
        DeclareLaunchArgument("lidar_input_topic", default_value="/livox/lidar_filtered"),
        DeclareLaunchArgument("use_body_imu", default_value="false"),
        DeclareLaunchArgument("distortion_imu_topic", default_value="/lidar/imu"),
        DeclareLaunchArgument("standing_height", default_value="0.78"),
        DeclareLaunchArgument("accumulate_scan_cloud", default_value="true"),
        DeclareLaunchArgument("scan_accumulation_max_yaw_rate", default_value="0.20"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    standing_height = LaunchConfiguration("standing_height")
    sim_time_params = {"use_sim_time": use_sim_time}

    # lidar_stabilized sits at the sensor position above the floor: standing
    # height plus the pelvis -> sensor offset, identity (upright) rotation.
    lidar_stabilized_z = PythonExpression([standing_height, " + ", SENSOR_Z_IN_BASE])

    # --- Robot description + joint states (fixed standing pose) -----------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": ParameterValue(
                Command(["xacro ", urdf_file, " standing_height:=", standing_height]),
                value_type=str,
            ),
        }],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[sim_time_params],
    )

    # --- Static TFs ---------------------------------------------------------
    base_footprint_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bag_replay_base_footprint_to_base_link",
        output="screen",
        parameters=[sim_time_params],
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", standing_height,
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
    )

    # Inverted-mount cloud frame: value measured from the bag's own /tf_static
    # (whose playback is dropped -- see module docstring).
    mid360_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bag_replay_mid360_to_livox_frame",
        output="screen",
        parameters=[sim_time_params],
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "3.14159265", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "mid360_link",
            "--child-frame-id", "livox_frame",
        ],
    )

    # Live-driver cloud frame alias (upright recordings stamp lidar_link),
    # identical to the alias g1_bringup publishes on hardware.
    mid360_to_lidar_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bag_replay_mid360_to_lidar_link",
        output="screen",
        parameters=[sim_time_params],
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "mid360_link",
            "--child-frame-id", "lidar_link",
        ],
    )

    use_body_imu = LaunchConfiguration("use_body_imu")

    lidar_stabilized_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bag_replay_lidar_stabilized_static",
        output="screen",
        parameters=[sim_time_params],
        arguments=[
            "--x", SENSOR_X_IN_BASE, "--y", SENSOR_Y_IN_BASE, "--z", lidar_stabilized_z,
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "lidar_stabilized",
        ],
        condition=UnlessCondition(use_body_imu),
    )

    # With a recorded body IMU, run the live gait-leveling node instead of the
    # static upright frame (same parameters as g1_bringup.launch.py).
    lidar_level_frame_publisher = Node(
        package="robot_nav",
        executable="lidar_level_frame_publisher",
        name="lidar_level_frame_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "imu_topic": LaunchConfiguration("distortion_imu_topic"),
            "parent_frame": "base_footprint",
            "child_frame": "lidar_stabilized",
            "lidar_x_in_base": float(SENSOR_X_IN_BASE),
            "lidar_y_in_base": float(SENSOR_Y_IN_BASE),
            "lidar_z_in_base": float(SENSOR_Z_IN_BASE),
            "standing_height": ParameterValue(standing_height, value_type=float),
        }],
        condition=IfCondition(use_body_imu),
    )

    # --- Sensor relays + DLIO (robot_lio) ----------------------------------
    # /livox/lidar_filtered -> /lidar/points relay + DLIO with publish_tf ON
    # (owns odom -> base_footprint; Replay TF Owner Override, ADR 0012).
    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_lio_share, "launch", "lio.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_lio": "true",
            "start_lidar_points_relay": "true",
            "lidar_points_input_topic": LaunchConfiguration("lidar_input_topic"),
            "publish_lidar_points_frame_tf": "false",
            "start_lidar_imu_relay": "false",
            "publish_lidar_imu_tf": "false",
            "dlio_publish_tf": "true",
            "dlio_params_file": LaunchConfiguration("dlio_params"),
        }.items(),
    )

    # /livox/imu (accel in g) -> /lidar/imu (m/s^2). Live G1 rescales to
    # /livox/imu_ms2 and relays; here the rescale output IS the canonical topic.
    imu_rescale = Node(
        package="g1_bringup",
        executable="imu_rescale",
        name="imu_rescale",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/livox/imu",
            "output_topic": "/lidar/imu",
        }],
    )

    # --- Perception + SLAM (same wiring as live g1_bringup) ----------------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_share, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_mode": "mapping",
            "slam_params": LaunchConfiguration("slam_params"),
            "distortion_params": LaunchConfiguration("distortion_params"),
            "distortion_imu_topic": LaunchConfiguration("distortion_imu_topic"),
            "scan_params": LaunchConfiguration("scan_params"),
            "accumulate_scan_cloud": LaunchConfiguration("accumulate_scan_cloud"),
            "scan_accumulation_max_yaw_rate": LaunchConfiguration(
                "scan_accumulation_max_yaw_rate"),
            # Mapping scan from DLIO's registered cloud, as live G1 bringup.
            "scan_cloud_topic": "/lidar/points_registered",
            "scan_cloud_in_fixed_frame": "true",
            "scan_self_filter_radius": "0.45",
        }.items(),
    )

    # --- Nav2 costmaps (just enough for STVL tuning) ------------------------
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
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[sim_time_params],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        joint_state_publisher,
        base_footprint_to_base_link,
        mid360_to_livox_frame,
        mid360_to_lidar_link,
        lidar_stabilized_static,
        lidar_level_frame_publisher,
        lio,
        imu_rescale,
        slam,
        controller_server,
        planner_server,
        nav_lifecycle_manager,
        rviz2,
    ])
