#!/usr/bin/env bash
set -euo pipefail

MODE="mapping"
MAP_PATH=""
RVIZ="true"
USE_SAFETY="false"
USE_STVL="true"
ROBOT_NAME=""
LIDAR_IN_TOPIC="/utlidar/cloud"
START_LIDAR_RELAY="true"
RELAY_PID=""

usage() {
  cat <<'EOF'
Usage:
  run_go2_real.sh [mapping|localization] [options]

Options:
  --map <path>               Path to slam_toolbox map (.posegraph), required in localization mode
  --no-rviz                  Start without RViz
  --use-safety               Enable safety gate (velocity_limiter)
  --no-stvl                  Disable SpatioTemporalVoxelLayer in Nav2 local costmap
  --robot-name <name>        Optional Nav2 namespace
  --lidar-in-topic <topic>   Input pointcloud topic for relay to /lidar/points (default: /utlidar/cloud)
  --skip-lidar-relay         Do not start topic relay (use only if /lidar/points already exists)
  -h, --help                 Show this help

Examples:
  run_go2_real.sh
  run_go2_real.sh mapping --use-safety
  run_go2_real.sh localization --map /ws/anubi/src/robot_nav/maps/my_map --use-safety
EOF
}

cleanup() {
  if [[ -n "$RELAY_PID" ]] && kill -0 "$RELAY_PID" 2>/dev/null; then
    kill "$RELAY_PID" 2>/dev/null || true
    wait "$RELAY_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

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
    --no-stvl)
      USE_STVL="false"
      shift
      ;;
    --robot-name)
      [[ $# -ge 2 ]] || { echo "Missing value for --robot-name" >&2; usage; exit 1; }
      ROBOT_NAME="$2"
      shift 2
      ;;
    --lidar-in-topic)
      [[ $# -ge 2 ]] || { echo "Missing value for --lidar-in-topic" >&2; usage; exit 1; }
      LIDAR_IN_TOPIC="$2"
      shift 2
      ;;
    --skip-lidar-relay)
      START_LIDAR_RELAY="false"
      shift
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

if [[ "$START_LIDAR_RELAY" == "true" ]]; then
  echo "Starting LiDAR relay: ${LIDAR_IN_TOPIC} -> /lidar/points"
  ros2 run topic_tools relay "$LIDAR_IN_TOPIC" /lidar/points >/tmp/go2_lidar_relay.log 2>&1 &
  RELAY_PID=$!
  sleep 1
  if ! kill -0 "$RELAY_PID" 2>/dev/null; then
    echo "Failed to start LiDAR relay. Check /tmp/go2_lidar_relay.log" >&2
    exit 1
  fi
fi

CMD=(
  ros2 launch go2_bringup go2_bringup.launch.py
  slam_mode:="$MODE"
  rviz:="$RVIZ"
  use_safety:="$USE_SAFETY"
  use_stvl:="$USE_STVL"
)

if [[ -n "$MAP_PATH" ]]; then
  CMD+=(map:="$MAP_PATH")
fi

if [[ -n "$ROBOT_NAME" ]]; then
  CMD+=(robot_name:="$ROBOT_NAME")
fi

echo "Starting GO2 real robot stack..."
printf 'Command: %q ' "${CMD[@]}"
echo

"${CMD[@]}"
