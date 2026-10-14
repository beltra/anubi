#!/usr/bin/env bash
# Launch the Unitree A2-W (Pro) Gazebo Fortress + Nav2 simulation.
#
# Wheeled analogue of run_g1_sim.sh. By default the articulated legs and wheels are
# driven from /cmd_vel_safe by a trained wheel-legged policy (a2w_policy_controller);
# --locomotion kinematic slides a rigid VelocityControl base instead. The two JT128 LiDARs are merged by cloud_merger into /lidar/points.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

MODE="mapping"
MAP_PATH=""
WORLD_PATH=""
GUI="false"
RVIZ="true"
USE_STVL="true"
USE_LIO="true"     # DLIO on the merged cloud + body IMU, as on the real robot
NAV_DELAY_SEC="10.0"
DISTORTION_IMU_TOPIC="/imu/data"
CLEAN_EXISTING="true"
LIDAR_CHANNELS=""
LIDAR_H_SAMPLES=""
PHYSICS_STEP_SEC=""
LOCOMOTION="policy"
POLICY_PATH=""

usage() {
  cat <<'USAGE'
Usage:
  run_a2w_sim.sh [mapping|localization] [options]

Options:
  --map <path>         slam_toolbox map (.posegraph), required in localization mode
  --world <path>       Override world file (default: indoor_office.world)
  --gui                Start the Gazebo GUI
  --no-gui             Headless Gazebo server (default)
  --rviz               Start RViz (default)
  --no-rviz            Start without RViz
  --no-stvl            Disable SpatioTemporalVoxelLayer in the Nav2 local costmap
  --use-lio            DLIO drives the EKF (default)
  --no-lio             Gazebo ground-truth odometry drives the EKF instead of DLIO
  --no-clean           Do not kill pre-existing ROS/Gazebo processes before launch
  --nav-delay <sec>    Delay before SLAM starts; Nav2 starts 10 s later (default: 10.0)
  --distortion-imu-topic <topic>
                       IMU topic used for LiDAR correction (default: /imu/data)
  --lidar-channels <n> Simulated JT128 channels per sensor (launch default: 64; real: 128)
  --lidar-h-samples <n>
                       Simulated azimuth samples per sensor (launch default: 180; real: 900)
  --physics-step <sec> Gazebo max_step_size (launch default: 0.002 policy, 0.004 kinematic)
  --locomotion <policy|kinematic>
                       policy: articulated legs and wheels driven by a2w_policy_controller
                       with the bundled a2w_flat_v1 policy (default). kinematic: rigid
                       sliding base
  --policy <file.npz>  Other actor weights (implies --locomotion policy)
  --no-policy          Articulated model, but the controller only holds the stance and
                       skid-steers the wheels
  -h, --help           Show this help

Examples:
  run_a2w_sim.sh
  run_a2w_sim.sh --gui --no-lio
  run_a2w_sim.sh --lidar-channels 128 --lidar-h-samples 900   # GPU host
  run_a2w_sim.sh --gui --no-lio --locomotion kinematic          # sliding base
  run_a2w_sim.sh --gui --no-lio --no-policy                     # stance + skid-steer
  run_a2w_sim.sh --gui --no-lio --policy a2w_flat.npz           # another trained policy
  run_a2w_sim.sh localization --map src/autonomy/robot_nav/maps/my_map
USAGE
}

need_value() { [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 1; }; }

if [[ "${1:-}" == "mapping" || "${1:-}" == "localization" ]]; then
  MODE="$1"; shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)   need_value "$@"; MAP_PATH="$2"; shift 2;;
    --world) need_value "$@"; WORLD_PATH="$2"; shift 2;;
    --gui)    GUI="true"; shift;;
    --no-gui) GUI="false"; shift;;
    --rviz)    RVIZ="true"; shift;;
    --no-rviz) RVIZ="false"; shift;;
    --no-stvl) USE_STVL="false"; shift;;
    --use-lio) USE_LIO="true"; shift;;
    --no-lio)  USE_LIO="false"; shift;;
    --no-clean) CLEAN_EXISTING="false"; shift;;
    --nav-delay) need_value "$@"; NAV_DELAY_SEC="$2"; shift 2;;
    --distortion-imu-topic) need_value "$@"; DISTORTION_IMU_TOPIC="$2"; shift 2;;
    --lidar-channels) need_value "$@"; LIDAR_CHANNELS="$2"; shift 2;;
    --lidar-h-samples) need_value "$@"; LIDAR_H_SAMPLES="$2"; shift 2;;
    --physics-step) need_value "$@"; PHYSICS_STEP_SEC="$2"; shift 2;;
    --locomotion) need_value "$@"; LOCOMOTION="$2"; shift 2;;
    --policy) need_value "$@"; POLICY_PATH="$2"; LOCOMOTION="policy"; shift 2;;
    --no-policy) POLICY_PATH="none"; LOCOMOTION="policy"; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1;;
  esac
done

if [[ "$LOCOMOTION" != "kinematic" && "$LOCOMOTION" != "policy" ]]; then
  echo "--locomotion must be kinematic or policy (got: $LOCOMOTION)." >&2; exit 1
fi
if [[ -n "$POLICY_PATH" && "$POLICY_PATH" != "none" ]]; then
  if [[ ! -f "$POLICY_PATH" ]]; then
    echo "Policy file not found: $POLICY_PATH" >&2; exit 1
  fi
  POLICY_PATH="$(cd "$(dirname "$POLICY_PATH")" && pwd)/$(basename "$POLICY_PATH")"
fi

if [[ "$MODE" == "localization" ]]; then
  if [[ -z "$MAP_PATH" ]]; then
    echo "In localization mode you must provide --map <path>." >&2; exit 1
  fi
  if [[ ! -f "$MAP_PATH" && ! -f "${MAP_PATH}.posegraph" ]]; then
    echo "Map not found: $MAP_PATH (also checked ${MAP_PATH}.posegraph)." >&2; exit 1
  fi
fi

if [[ "$CLEAN_EXISTING" == "true" ]]; then
  echo "Cleaning stale ROS/Gazebo processes (and /dev/shm) before launch..."
  bash "$SCRIPT_DIR/kill_ros_nodes.sh" || true
fi

# ROS setup scripts are not fully nounset-safe; source them with +u. An already
# sourced ROS environment (e.g. an activated RoboStack env) is kept.
set +u
if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/humble/setup.bash
fi
source "$WS_ROOT/install/setup.bash"
set -u

CMD=(
  ros2 launch a2w_sim sim_bringup_a2w.launch.py
  slam_mode:="$MODE"
  gui:="$GUI"
  use_rviz:="$RVIZ"
  use_stvl:="$USE_STVL"
  use_lio:="$USE_LIO"
  nav_delay_sec:="$NAV_DELAY_SEC"
  distortion_imu_topic:="$DISTORTION_IMU_TOPIC"
  locomotion:="$LOCOMOTION"
)
[[ -n "$POLICY_PATH" ]] && CMD+=(policy_path:="$POLICY_PATH")
[[ -n "$MAP_PATH" ]] && CMD+=(map:="$MAP_PATH")
[[ -n "$LIDAR_CHANNELS" ]] && CMD+=(lidar_channels:="$LIDAR_CHANNELS")
[[ -n "$LIDAR_H_SAMPLES" ]] && CMD+=(lidar_h_samples:="$LIDAR_H_SAMPLES")
[[ -n "$PHYSICS_STEP_SEC" ]] && CMD+=(physics_step_sec:="$PHYSICS_STEP_SEC")
if [[ -n "$WORLD_PATH" ]]; then
  CMD+=(world:="$WORLD_PATH")
  # The spawn (`create -world <name>`) must target the SDF's <world name>.
  WORLD_NAME="$(grep -oPm1 '<world name="\K[^"]+' "$WORLD_PATH" || true)"
  [[ -n "$WORLD_NAME" ]] && CMD+=(world_name:="$WORLD_NAME")
fi

echo "Starting A2-W simulation stack..."
printf 'Command:'; printf ' %q' "${CMD[@]}"; echo
exec "${CMD[@]}"
