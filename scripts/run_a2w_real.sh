#!/usr/bin/env bash
# Launch the Unitree A2-W (Pro) REAL-ROBOT autonomy stack (hardware bring-up).
# Wraps a2w_bringup.launch.py: go2_hw_bridge on the A2 Sport API topics
# (/lf/sportmodestate in, SportClient Move on /api/sport/request out) + cloud_merger
# (front + rear JT128 -> /lidar/points) + EKF + DLIO + slam_toolbox + Nav2 (A2-W
# configs) + robot_safety velocity_limiter.
#
# Before launching: start both Hesai JT128 drivers, put the robot in its wheeled stand
# mode with the remote, and keep it still during --nav-delay so DLIO can calibrate.
# Not yet validated on hardware: verify the LiDAR topics/frames first (--help).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

MODE="mapping"
MAP_PATH=""
RVIZ="true"
USE_STVL="true"
USE_LIO="true"
USE_SAFETY="true"
ROBOT_NAME=""
NAV_DELAY_SEC="5.0"
DISTORTION_IMU_TOPIC="/imu/data"
FRONT_LIDAR_TOPIC="/lidar_points_front"
REAR_LIDAR_TOPIC="/lidar_points_rear"
FRONT_LIDAR_FRAME="front_lidar_link"
REAR_LIDAR_FRAME="rear_lidar_link"
LIDAR_FRAME_ALIAS_YAW="0.0"
# Same unitree_go/WirelessController layout as run_go2_real.sh; not yet checked on the
# A2 remote. With 0 a latched tilt E-STOP can never be acknowledged.
RECOVERY_ACK_KEY_MASK="0x0010"
RESUME_KEY_MASK="0x0010"
ABORT_KEY_MASK="0x0020"
CLEAN_EXISTING="true"

usage() {
  cat <<'USAGE'
Usage:
  run_a2w_real.sh [mapping|localization] [options]

Options:
  --map <path>                 slam_toolbox .posegraph map (required in localization mode)
  --no-rviz                    Start without RViz
  --no-stvl                    Disable SpatioTemporalVoxelLayer in the local costmap
  --use-lio                    DLIO drives the EKF (default)
  --no-lio                     EKF uses Sport Mode odometry + body IMU only
  --use-safety                 Run velocity_limiter; the bridge reads /cmd_vel_safe (default)
  --no-safety                  Bridge reads /cmd_vel_collision_free directly
  --robot-name <name>          Nav2 namespace prefix (default: none)
  --nav-delay <sec>            Delay before SLAM + Nav2 start (default: 5.0)
  --distortion-imu-topic <t>   IMU topic for LiDAR correction (default: /imu/data)
  --front-lidar-topic <t>      Front JT128 driver cloud (default: /lidar_points_front)
  --rear-lidar-topic <t>       Rear JT128 driver cloud (default: /lidar_points_rear)
  --front-lidar-frame <f>      frame_id of the front driver cloud (default: front_lidar_link)
  --rear-lidar-frame <f>       frame_id of the rear driver cloud (default: rear_lidar_link)
  --lidar-frame-alias-yaw <r>  Yaw [rad] about the dome axis from the URDF lidar frame to a
                               differing driver frame (default: 0.0)
  --recovery-ack-key-mask <m>  Controller key mask for tilt E-STOP recovery (default: 0x0010)
  --resume-key-mask <m>        Controller key mask for Nav2 resume (default: 0x0010)
  --abort-key-mask <m>         Controller key mask for Nav2 abort (default: 0x0020)
  --no-clean                   Do not kill pre-existing ROS processes before launch
  -h, --help                   Show this help

Examples:
  run_a2w_real.sh
  run_a2w_real.sh --front-lidar-topic /front/lidar_points --front-lidar-frame hesai_front \
                  --rear-lidar-topic /rear/lidar_points --rear-lidar-frame hesai_rear
  run_a2w_real.sh localization --map src/autonomy/robot_nav/maps/my_map
USAGE
}

need_value() { [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 1; }; }

if [[ "${1:-}" == "mapping" || "${1:-}" == "localization" ]]; then
  MODE="$1"; shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) need_value "$@"; MAP_PATH="$2"; shift 2;;
    --no-rviz) RVIZ="false"; shift;;
    --no-stvl) USE_STVL="false"; shift;;
    --use-lio) USE_LIO="true"; shift;;
    --no-lio) USE_LIO="false"; shift;;
    --use-safety) USE_SAFETY="true"; shift;;
    --no-safety) USE_SAFETY="false"; shift;;
    --robot-name) need_value "$@"; ROBOT_NAME="$2"; shift 2;;
    --nav-delay) need_value "$@"; NAV_DELAY_SEC="$2"; shift 2;;
    --distortion-imu-topic) need_value "$@"; DISTORTION_IMU_TOPIC="$2"; shift 2;;
    --front-lidar-topic) need_value "$@"; FRONT_LIDAR_TOPIC="$2"; shift 2;;
    --rear-lidar-topic) need_value "$@"; REAR_LIDAR_TOPIC="$2"; shift 2;;
    --front-lidar-frame) need_value "$@"; FRONT_LIDAR_FRAME="$2"; shift 2;;
    --rear-lidar-frame) need_value "$@"; REAR_LIDAR_FRAME="$2"; shift 2;;
    --lidar-frame-alias-yaw) need_value "$@"; LIDAR_FRAME_ALIAS_YAW="$2"; shift 2;;
    --recovery-ack-key-mask) need_value "$@"; RECOVERY_ACK_KEY_MASK="$2"; shift 2;;
    --resume-key-mask) need_value "$@"; RESUME_KEY_MASK="$2"; shift 2;;
    --abort-key-mask) need_value "$@"; ABORT_KEY_MASK="$2"; shift 2;;
    --no-clean) CLEAN_EXISTING="false"; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1;;
  esac
done

if [[ "$MODE" == "localization" ]]; then
  if [[ -z "$MAP_PATH" ]]; then
    echo "In localization mode you must provide --map <path>." >&2; exit 1
  fi
  if [[ ! -f "$MAP_PATH" && ! -f "${MAP_PATH}.posegraph" ]]; then
    echo "Map not found: $MAP_PATH (also checked ${MAP_PATH}.posegraph)." >&2; exit 1
  fi
fi

if [[ "$USE_SAFETY" == "false" ]]; then
  echo "WARNING: --no-safety: velocity_limiter is off; the bridge reads /cmd_vel_collision_free."
fi

if [[ "$CLEAN_EXISTING" == "true" ]]; then
  echo "Cleaning stale ROS processes before launch..."
  bash "$SCRIPT_DIR/kill_ros_nodes.sh" || true
fi

set +u
if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/humble/setup.bash
fi
source "$WS_ROOT/install/setup.bash"
set -u

CMD=(
  ros2 launch a2w_bringup a2w_bringup.launch.py
  slam_mode:="$MODE"
  rviz:="$RVIZ"
  use_stvl:="$USE_STVL"
  use_lio:="$USE_LIO"
  use_safety:="$USE_SAFETY"
  robot_name:="$ROBOT_NAME"
  nav_delay_sec:="$NAV_DELAY_SEC"
  distortion_imu_topic:="$DISTORTION_IMU_TOPIC"
  front_lidar_points_input_topic:="$FRONT_LIDAR_TOPIC"
  rear_lidar_points_input_topic:="$REAR_LIDAR_TOPIC"
  front_lidar_points_frame:="$FRONT_LIDAR_FRAME"
  rear_lidar_points_frame:="$REAR_LIDAR_FRAME"
  lidar_frame_alias_yaw:="$LIDAR_FRAME_ALIAS_YAW"
  recovery_ack_key_mask:="$RECOVERY_ACK_KEY_MASK"
  resume_key_mask:="$RESUME_KEY_MASK"
  abort_key_mask:="$ABORT_KEY_MASK"
)
[[ -n "$MAP_PATH" ]] && CMD+=(map:="$MAP_PATH")

echo "Starting A2-W real-robot stack..."
printf 'Command:'; printf ' %q' "${CMD[@]}"; echo
exec "${CMD[@]}"
