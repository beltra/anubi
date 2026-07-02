#!/usr/bin/env bash
# Launch the Unitree G1 Gazebo Fortress + Nav2 simulation.
#
# Humanoid analogue of run_go2_sim.sh. The G1 walking gait (AMO) is NOT simulated; a
# kinematic VelocityControl base stands in for it so the full Nav2 perception + planning
# + control loop can be exercised. Run INSIDE the anubi-g1-sim:humble container.
#
# Visualization note: the Gazebo GUI does not render on the NoMachine software-GL display
# and gui:=true also starves the server so the spawn `create` call times out. The reliable
# setup is a HEADLESS Gazebo server (default) plus RViz, so --gui is off by default and
# RViz is on by default. Override with --gui / --no-rviz if you have a real GL display.
set -euo pipefail

MODE="mapping"
MAP_PATH=""
WORLD_PATH=""
GUI="false"        # headless Gazebo server by default (reliable spawn); visualize in RViz
RVIZ="true"
USE_STVL="true"
USE_LIO="false"    # default: Gazebo ground-truth odom + EKF (reliable). --use-lio = DLIO.
NAV_DELAY_SEC="14.0"
DISTORTION_IMU_TOPIC="/imu/data"
CLEAN_EXISTING="true"

usage() {
  cat <<'EOF'
Usage:
  run_g1_sim.sh [mapping|localization] [options]

Options:
  --map <path>         slam_toolbox map (.posegraph), required in localization mode
  --world <path>       Override world file (default: indoor_office.world)
  --gui                Start Gazebo with its GUI (experimental: software GL on a NoMachine
                       display is slow and can time out the model spawn; prefer RViz)
  --no-gui             Headless Gazebo server (default; visualize in RViz)
  --rviz               Start RViz (default)
  --no-rviz            Start without RViz
  --no-stvl            Disable SpatioTemporalVoxelLayer in the Nav2 local costmap
  --use-lio            Use DLIO (Mid-360 LiDAR-inertial odometry) as the odometry, like the
                       real robot. NOTE: the kinematic sim base feeds DLIO unrealistic IMU
                       during motion, so DLIO tracks well at rest/slow but can drift while
                       moving; the default (ground-truth odom) is the reliable sim path.
  --no-clean           Do not kill pre-existing ROS/Gazebo processes before launch
  --nav-delay <sec>    Delay before SLAM starts; Nav2 starts 10 s later (default: 14.0)
  --distortion-imu-topic <topic>
                       IMU topic used for LiDAR correction (default: /imu/data)
  -h, --help           Show this help

Examples:
  run_g1_sim.sh
  run_g1_sim.sh --use-lio
  run_g1_sim.sh mapping --world /ws/anubi/src/simulation/sim_worlds/worlds/warehouse.world
  run_g1_sim.sh localization --map /ws/anubi/src/autonomy/robot_nav/maps/my_map
EOF
}

if [[ "${1:-}" == "mapping" || "${1:-}" == "localization" ]]; then
  MODE="$1"; shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)   [[ $# -ge 2 ]] || { echo "Missing value for --map" >&2; exit 1; }; MAP_PATH="$2"; shift 2;;
    --world) [[ $# -ge 2 ]] || { echo "Missing value for --world" >&2; exit 1; }; WORLD_PATH="$2"; shift 2;;
    --gui)    GUI="true"; shift;;
    --no-gui) GUI="false"; shift;;
    --rviz)    RVIZ="true"; shift;;
    --no-rviz) RVIZ="false"; shift;;
    --no-stvl) USE_STVL="false"; shift;;
    --use-lio) USE_LIO="true"; shift;;
    --no-clean) CLEAN_EXISTING="false"; shift;;
    --nav-delay) [[ $# -ge 2 ]] || { echo "Missing value for --nav-delay" >&2; exit 1; }; NAV_DELAY_SEC="$2"; shift 2;;
    --distortion-imu-topic) [[ $# -ge 2 ]] || { echo "Missing value for --distortion-imu-topic" >&2; exit 1; }; DISTORTION_IMU_TOPIC="$2"; shift 2;;
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

if [[ "$GUI" == "true" ]]; then
  echo "WARNING: --gui starts the Gazebo GUI. On a software-GL display this is slow and"
  echo "         the model spawn can time out. If the robot never appears, rerun headless."
fi

if [[ "$CLEAN_EXISTING" == "true" ]]; then
  # kill_ros_nodes.sh also reaps /dev/shm/fastrtps_* segments; on the small (64 MB) Docker
  # /dev/shm those otherwise pile up across restarts and break DDS / Nav2 bring-up.
  echo "Cleaning stale ROS/Gazebo processes (and /dev/shm) before launch..."
  bash "$(cd "$(dirname "$0")" && pwd)/kill_ros_nodes.sh" || true
fi

# ROS setup scripts are not fully nounset-safe; source them with +u.
set +u
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
set -u

CMD=(
  ros2 launch g1_sim sim_bringup_g1.launch.py
  slam_mode:="$MODE"
  gui:="$GUI"
  use_rviz:="$RVIZ"
  use_stvl:="$USE_STVL"
  use_lio:="$USE_LIO"
  nav_delay_sec:="$NAV_DELAY_SEC"
  distortion_imu_topic:="$DISTORTION_IMU_TOPIC"
)
[[ -n "$MAP_PATH"   ]] && CMD+=(map:="$MAP_PATH")
[[ -n "$WORLD_PATH" ]] && CMD+=(world:="$WORLD_PATH")

echo "Starting G1 simulation stack..."
printf 'Command:'; printf ' %q' "${CMD[@]}"; echo
exec "${CMD[@]}"
