#!/usr/bin/env bash
set -euo pipefail

MODE="mapping"
MAP_PATH=""
RVIZ="true"
USE_SAFETY="false"
USE_STVL="true"
USE_LIO="false"
NAV_PROFILE="default_safe"
USE_ROUTE_SERVER="false"
ROUTE_GRAPH_FILE=""
ROBOT_NAME=""
LIDAR_POINTS_IN_TOPIC="/utlidar/cloud"
LIDAR_IMU_IN_TOPIC="/unilidar/imu"

usage() {
  cat <<'EOF'
Usage:
  run_go2_real.sh [mapping|localization] [options]

Options:
  --map <path>               Path to slam_toolbox map (.posegraph), required in localization mode
  --no-rviz                  Start without RViz
  --use-safety               Enable safety gate (velocity_limiter)
  --use-lio                  Enable optional LIO backend (/lidar_odom)
  --no-stvl                  Disable SpatioTemporalVoxelLayer in Nav2 local costmap
  --nav-profile <id>         Nav2 profile: default_safe | narrow_gap_1p0m | forward_static
  --use-route-server         Launch optional Nav2 route_server scaffold
  --route-graph-file <path>  Route graph GeoJSON file for route_server experiments
  --robot-name <name>        Optional Nav2 namespace
  --lidar-points-in-topic <topic>
                              Raw pointcloud topic relayed to /lidar/points (default: /utlidar/cloud)
  --lidar-imu-in-topic <topic>
                              Raw LiDAR IMU topic relayed to /lidar/imu (default: /unilidar/imu)
  -h, --help                 Show this help

Examples:
  run_go2_real.sh
  run_go2_real.sh mapping --use-safety
  run_go2_real.sh localization --map /ws/anubi/src/autonomy/robot_nav/maps/my_map --use-safety
  run_go2_real.sh mapping --nav-profile forward_static
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
    --no-rviz)
      RVIZ="false"
      shift
      ;;
    --use-safety)
      USE_SAFETY="true"
      shift
      ;;
    --use-lio)
      USE_LIO="true"
      shift
      ;;
    --no-stvl)
      USE_STVL="false"
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
    --lidar-points-in-topic)
      [[ $# -ge 2 ]] || { echo "Missing value for --lidar-points-in-topic" >&2; usage; exit 1; }
      LIDAR_POINTS_IN_TOPIC="$2"
      shift 2
      ;;
    --lidar-imu-in-topic)
      [[ $# -ge 2 ]] || { echo "Missing value for --lidar-imu-in-topic" >&2; usage; exit 1; }
      LIDAR_IMU_IN_TOPIC="$2"
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

# ROS setup scripts are not fully nounset-safe; source them with +u.
set +u
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
set -u

CMD=(
  ros2 launch go2_bringup go2_bringup.launch.py
  slam_mode:="$MODE"
  rviz:="$RVIZ"
  use_safety:="$USE_SAFETY"
  use_stvl:="$USE_STVL"
  use_lio:="$USE_LIO"
  nav_profile:="$NAV_PROFILE"
  use_route_server:="$USE_ROUTE_SERVER"
  lidar_points_input_topic:="$LIDAR_POINTS_IN_TOPIC"
  lidar_imu_input_topic:="$LIDAR_IMU_IN_TOPIC"
)

if [[ -n "$ROUTE_GRAPH_FILE" ]]; then
  CMD+=(route_graph_file:="$ROUTE_GRAPH_FILE")
fi

if [[ -n "$MAP_PATH" ]]; then
  CMD+=(map:="$MAP_PATH")
fi

if [[ -n "$ROBOT_NAME" ]]; then
  CMD+=(robot_name:="$ROBOT_NAME")
fi

echo "Starting GO2 real robot stack..."
printf 'Command:'
printf ' %q' "${CMD[@]}"
echo

"${CMD[@]}"
