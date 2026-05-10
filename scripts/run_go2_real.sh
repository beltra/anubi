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
NAV_DELAY_SEC="0.0"
ROBOT_NAME=""
LIDAR_POINTS_IN_TOPIC="/utlidar/cloud"
LIDAR_POINTS_FRAME="utlidar_lidar"
LIDAR_IMU_IN_TOPIC="/utlidar/imu"
DISTORTION_IMU_TOPIC="/imu/data"
RECOVERY_ACK_KEY_MASK="0x0010"
RESUME_KEY_MASK="0x0010"
ABORT_KEY_MASK="0x0020"
RECORD_ROSBAG="false"
ROSBAG_DIR="${HOME}/rosbags/go2_real"
ROSBAG_TOPICS_OVERRIDE=""
ROSBAG_QOS_OVERRIDES_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rosbag_qos_overrides.yaml"

# Curated default topic set for offline perception replay.
# Skips raw /utlidar/{cloud,imu} because robot_lio relays them identically to
# /lidar/points and /lidar/imu, which is what the perception stack consumes.
# Latched topics (/tf_static, /robot_description, /map) rely on the QoS overrides
# YAML so the recorder subscribes with TRANSIENT_LOCAL durability.
DEFAULT_ROSBAG_TOPICS=(
  /lidar/points
  /lidar/imu
  /lidar/points_undistorted
  /lidar/points_scan_accumulated
  /scan
  /imu/data
  /sportmodestate
  /lowstate
  /odom_raw
  /odom
  /lidar_odom
  /lidar_odom_reliable
  /joint_states
  /tf
  /tf_static
  /robot_description
  /map
  /robot_status
  /wirelesscontroller
)

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
  --nav-delay <sec>          Delay before SLAM and Nav2 start (default: 0.0)
  --robot-name <name>        Optional Nav2 namespace
  --lidar-points-in-topic <topic>
                              Raw pointcloud topic relayed to /lidar/points (default: /utlidar/cloud)
  --lidar-points-frame <frame>
                              Raw pointcloud frame_id to alias to lidar_link (default: utlidar_lidar)
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
  --record-rosbag            Record a rosbag of perception-relevant topics in the
                              background. Output goes under --rosbag-dir as
                              go2_real_<mode>_<timestamp>. LiDAR pointclouds at
                              the L1 native rate use ~50-100 MB/min of disk.
  --rosbag-dir <path>        Output directory for rosbag recordings
                              (default: $HOME/rosbags/go2_real)
  --rosbag-topics "<list>"   Space-separated topic list to record instead of the
                              curated perception default. QoS overrides for
                              /tf_static, /robot_description, /map still apply.
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
    --nav-delay)
      [[ $# -ge 2 ]] || { echo "Missing value for --nav-delay" >&2; usage; exit 1; }
      NAV_DELAY_SEC="$2"
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
    --lidar-points-frame)
      [[ $# -ge 2 ]] || { echo "Missing value for --lidar-points-frame" >&2; usage; exit 1; }
      LIDAR_POINTS_FRAME="$2"
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
    --record-rosbag)
      RECORD_ROSBAG="true"
      shift
      ;;
    --rosbag-dir)
      [[ $# -ge 2 ]] || { echo "Missing value for --rosbag-dir" >&2; usage; exit 1; }
      ROSBAG_DIR="$2"
      shift 2
      ;;
    --rosbag-topics)
      [[ $# -ge 2 ]] || { echo "Missing value for --rosbag-topics" >&2; usage; exit 1; }
      ROSBAG_TOPICS_OVERRIDE="$2"
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
  nav_delay_sec:="$NAV_DELAY_SEC"
  lidar_points_input_topic:="$LIDAR_POINTS_IN_TOPIC"
  lidar_points_frame:="$LIDAR_POINTS_FRAME"
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

ROSBAG_PID=""

cleanup_rosbag() {
  if [[ -n "$ROSBAG_PID" ]] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
    echo "Stopping rosbag recorder (pid=$ROSBAG_PID) to flush metadata..."
    kill -INT "$ROSBAG_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$ROSBAG_PID" 2>/dev/null || break
      sleep 1
    done
    kill -9 "$ROSBAG_PID" 2>/dev/null || true
  fi
}

if [[ "$RECORD_ROSBAG" == "true" ]]; then
  if [[ -n "$ROSBAG_TOPICS_OVERRIDE" ]]; then
    # shellcheck disable=SC2206
    ROSBAG_TOPIC_LIST=( $ROSBAG_TOPICS_OVERRIDE )
  else
    ROSBAG_TOPIC_LIST=( "${DEFAULT_ROSBAG_TOPICS[@]}" )
  fi
  if [[ ! -f "$ROSBAG_QOS_OVERRIDES_PATH" ]]; then
    echo "Missing rosbag QoS overrides file: $ROSBAG_QOS_OVERRIDES_PATH" >&2
    exit 1
  fi
  mkdir -p "$ROSBAG_DIR"
  ROSBAG_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
  ROSBAG_OUTPUT="${ROSBAG_DIR}/go2_real_${MODE}_${ROSBAG_TIMESTAMP}"
  echo "Recording rosbag to: $ROSBAG_OUTPUT"
  echo "Topics (${#ROSBAG_TOPIC_LIST[@]}): ${ROSBAG_TOPIC_LIST[*]}"
  trap cleanup_rosbag EXIT INT TERM
  ros2 bag record \
    --output "$ROSBAG_OUTPUT" \
    --qos-profile-overrides-path "$ROSBAG_QOS_OVERRIDES_PATH" \
    "${ROSBAG_TOPIC_LIST[@]}" &
  ROSBAG_PID=$!
fi

echo "Starting GO2 real robot stack..."
if [[ "$USE_SAFETY" != "true" ]]; then
  echo "WARNING: safety arbiter disabled. Use only for unloaded bench tests." >&2
fi
printf 'Command:'
printf ' %q' "${CMD[@]}"
echo

"${CMD[@]}"
