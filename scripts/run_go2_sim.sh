#!/usr/bin/env bash
set -euo pipefail

MODE="mapping"
MAP_PATH=""
WORLD_PATH=""
GUI="true"
RVIZ="true"
USE_STVL="true"
NAV_PROFILE="default_safe"
USE_ROUTE_SERVER="false"
ROUTE_GRAPH_FILE=""
NAV_DELAY_SEC="20.0"
USE_LIO="false"
ROBOT_NAME=""
DISTORTION_IMU_TOPIC="/imu/data"
CLEAN_EXISTING="true"

usage() {
  cat <<'EOF'
Usage:
  run_go2_sim.sh [mapping|localization] [options]

Options:
  --map <path>         Path to slam_toolbox map (.posegraph), required in localization mode
  --world <path>       Override world file (default from sim_bringup.launch.py)
  --gui                Start Gazebo with GUI
  --rviz               Start RViz
  --no-gui             Start Gazebo without GUI (overrides default: start GUI)
  --no-rviz            Start without RViz (overrides default: start RViz)
  --no-stvl            Disable SpatioTemporalVoxelLayer in Nav2 local costmap
  --use-lio            Enable optional LIO backend (/lidar_odom)
  --no-clean           Do not clean pre-existing ROS/Gazebo processes before launch
  --nav-profile <id>   Nav2 profile: default_safe | narrow_gap_1p0m | forward_static
  --use-route-server   Launch optional Nav2 route_server scaffold
  --route-graph-file <path>
                       Route graph GeoJSON file for route_server experiments
  --robot-name <name>  Optional simulation robot/entity name
  --distortion-imu-topic <topic>
                       IMU topic used for LiDAR correction (default: /imu/data; try /lidar/imu)
  --nav-delay <sec>    Delay before SLAM starts; Nav2 starts 10 s later (default: 20.0)
  -h, --help           Show this help

Examples:
  run_go2_sim.sh
  run_go2_sim.sh mapping --world /ws/anubi/src/simulation/sim_worlds/worlds/warehouse.world
  run_go2_sim.sh localization --map /ws/anubi/src/autonomy/robot_nav/maps/my_map
EOF
}

if [[ "${1:-}" == "mapping" || "${1:-}" == "localization" ]]; then
  MODE="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      [[ $# -ge 2 ]] || { echo "Missing value for --map" >&2; usage; exit 1; }
      MAP_PATH="$2"
      shift 2
      ;;
    --world)
      [[ $# -ge 2 ]] || { echo "Missing value for --world" >&2; usage; exit 1; }
      WORLD_PATH="$2"
      shift 2
      ;;
    --no-gui)
      GUI="false"
      shift
      ;;
    --gui)
      GUI="true"
      shift
      ;;
    --no-rviz)
      RVIZ="false"
      shift
      ;;
    --rviz)
      RVIZ="true"
      shift
      ;;
    --no-stvl)
      USE_STVL="false"
      shift
      ;;
    --use-lio)
      USE_LIO="true"
      shift
      ;;
    --no-clean)
      CLEAN_EXISTING="false"
      shift
      ;;
    --nav-profile)
      [[ $# -ge 2 ]] || { echo "Missing value for --nav-profile" >&2; usage; exit 1; }
      NAV_PROFILE="$2"
      shift 2
      ;;
    --use-route-server)
      USE_ROUTE_SERVER="true"
      shift
      ;;
    --route-graph-file)
      [[ $# -ge 2 ]] || { echo "Missing value for --route-graph-file" >&2; usage; exit 1; }
      ROUTE_GRAPH_FILE="$2"
      shift 2
      ;;
    --robot-name)
      [[ $# -ge 2 ]] || { echo "Missing value for --robot-name" >&2; usage; exit 1; }
      ROBOT_NAME="$2"
      shift 2
      ;;
    --distortion-imu-topic)
      [[ $# -ge 2 ]] || { echo "Missing value for --distortion-imu-topic" >&2; usage; exit 1; }
      DISTORTION_IMU_TOPIC="$2"
      shift 2
      ;;
    --nav-delay)
      [[ $# -ge 2 ]] || { echo "Missing value for --nav-delay" >&2; usage; exit 1; }
      NAV_DELAY_SEC="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ "$NAV_PROFILE" != "default_safe" && "$NAV_PROFILE" != "narrow_gap_1p0m" && "$NAV_PROFILE" != "forward_static" ]]; then
  echo "Invalid --nav-profile: $NAV_PROFILE (expected default_safe, narrow_gap_1p0m, or forward_static)" >&2
  exit 1
fi

if [[ "$USE_ROUTE_SERVER" == "true" && -z "$ROUTE_GRAPH_FILE" ]]; then
  echo "--use-route-server requires --route-graph-file <path>." >&2
  exit 1
fi
if [[ "$USE_ROUTE_SERVER" == "true" && ! -f "$ROUTE_GRAPH_FILE" ]]; then
  echo "Route graph file not found: $ROUTE_GRAPH_FILE" >&2
  exit 1
fi

if [[ "$MODE" == "localization" ]]; then
  if [[ -z "$MAP_PATH" ]]; then
    echo "In localization mode you must provide --map <path>." >&2
    exit 1
  fi
  if [[ ! -f "$MAP_PATH" && ! -f "${MAP_PATH}.posegraph" ]]; then
    echo "Map not found: $MAP_PATH (also checked ${MAP_PATH}.posegraph)." >&2
    exit 1
  fi
fi

if [[ "$CLEAN_EXISTING" == "true" ]]; then
  echo "Cleaning stale ROS/Gazebo processes before launch..."
  bash "$(cd "$(dirname "$0")" && pwd)/kill_ros_nodes.sh" || true
fi

# ROS setup scripts are not fully nounset-safe; source them with +u.
set +u
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
set -u

CMD=(
  ros2 launch robot_sim sim_bringup.launch.py
  slam_mode:="$MODE"
  gui:="$GUI"
  use_rviz:="$RVIZ"
  use_stvl:="$USE_STVL"
  use_lio:="$USE_LIO"
  nav_profile:="$NAV_PROFILE"
  use_route_server:="$USE_ROUTE_SERVER"
  nav_delay_sec:="$NAV_DELAY_SEC"
  distortion_imu_topic:="$DISTORTION_IMU_TOPIC"
)

if [[ -n "$ROUTE_GRAPH_FILE" ]]; then
  CMD+=(route_graph_file:="$ROUTE_GRAPH_FILE")
fi

if [[ -n "$MAP_PATH" ]]; then
  CMD+=(map:="$MAP_PATH")
fi

if [[ -n "$WORLD_PATH" ]]; then
  CMD+=(world:="$WORLD_PATH")
fi

if [[ -n "$ROBOT_NAME" ]]; then
  CMD+=(robot_name:="$ROBOT_NAME")
fi

echo "Starting GO2 simulation stack..."
printf 'Command:'
printf ' %q' "${CMD[@]}"
echo

exec "${CMD[@]}"
