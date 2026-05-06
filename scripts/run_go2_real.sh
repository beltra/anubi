#!/usr/bin/env bash
set -euo pipefail

MODE="mapping"
MAP_PATH=""
RVIZ="true"
USE_SAFETY="true"
USE_STVL="true"
USE_LIO="false"
NAV_PROFILE="default_safe"
USE_ROUTE_SERVER="false"
ROUTE_GRAPH_FILE=""
ROBOT_NAME=""
LIDAR_POINTS_IN_TOPIC="/utlidar/cloud"
LIDAR_IMU_IN_TOPIC="/utlidar/imu"
DISTORTION_IMU_TOPIC="/imu/data"
RECOVERY_ACK_KEY_MASK="0"
RESUME_KEY_MASK="0"
ABORT_KEY_MASK="0"

usage() {
  cat <<'EOF'
Usage:
  run_go2_real.sh [mapping|localization] [options]

Options:
  --map <path>               Path to slam_toolbox map (.posegraph), required in localization mode
  --no-rviz                  Start without RViz
  --no-safety                Disable safety arbiter for bench tests only
  --use-lio                  Enable optional LIO backend (/lidar_odom)
  --no-stvl                  Disable SpatioTemporalVoxelLayer in Nav2 local costmap
  --nav-profile <id>         Nav2 profile: default_safe | narrow_gap_1p0m | forward_static
  --use-route-server         Launch optional Nav2 route_server scaffold
  --route-graph-file <path>  Route graph GeoJSON file for route_server experiments
  --robot-name <name>        Optional Nav2 namespace
  --lidar-points-in-topic <topic>
                              Raw pointcloud topic relayed to /lidar/points (default: /utlidar/cloud)
  --lidar-imu-in-topic <topic>
                              Raw LiDAR IMU topic relayed to /lidar/imu (default: /utlidar/imu)
  --distortion-imu-topic <topic>
                              IMU topic used for LiDAR correction (default: /imu/data; try /lidar/imu)
  --recovery-ack-key-mask <mask>
                              Controller START key mask for tilt recovery (hex or decimal)
  --resume-key-mask <mask>
                              Controller key mask for Nav2 resume and replanning
  --abort-key-mask <mask>
                              Controller key mask for controlled Nav2 abort
  -h, --help                 Show this help

Examples:
  run_go2_real.sh
  run_go2_real.sh localization --map /ws/anubi/src/autonomy/robot_nav/maps/my_map
  run_go2_real.sh mapping --recovery-ack-key-mask 0x0001
  run_go2_real.sh mapping --no-safety
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
    --no-safety)
      USE_SAFETY="false"
      shift
      ;;
    --use-safety)
      echo "--use-safety is now the default; use --no-safety only for bench tests." >&2
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
    --distortion-imu-topic)
      [[ $# -ge 2 ]] || { echo "Missing value for --distortion-imu-topic" >&2; usage; exit 1; }
      DISTORTION_IMU_TOPIC="$2"
      shift 2
      ;;
    --recovery-ack-key-mask)
      [[ $# -ge 2 ]] || { echo "Missing value for --recovery-ack-key-mask" >&2; usage; exit 1; }
      RECOVERY_ACK_KEY_MASK="$2"
      shift 2
      ;;
    --resume-key-mask)
      [[ $# -ge 2 ]] || { echo "Missing value for --resume-key-mask" >&2; usage; exit 1; }
      RESUME_KEY_MASK="$2"
      shift 2
      ;;
    --abort-key-mask)
      [[ $# -ge 2 ]] || { echo "Missing value for --abort-key-mask" >&2; usage; exit 1; }
      ABORT_KEY_MASK="$2"
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
  distortion_imu_topic:="$DISTORTION_IMU_TOPIC"
  recovery_ack_key_mask:="$RECOVERY_ACK_KEY_MASK"
  resume_key_mask:="$RESUME_KEY_MASK"
  abort_key_mask:="$ABORT_KEY_MASK"
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
if [[ "$USE_SAFETY" != "true" ]]; then
  echo "WARNING: safety arbiter disabled. Use only for unloaded bench tests." >&2
fi
printf 'Command:'
printf ' %q' "${CMD[@]}"
echo

"${CMD[@]}"
