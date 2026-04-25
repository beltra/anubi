#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stamp="$(date +%Y-%m-%d_%H-%M-%S)"

RESULTS_DIR="${RESULTS_DIR:-${workspace_root}/log/safety_regression_${stamp}}"
WORLD_NAME="${WORLD_NAME:-indoor_office}"
START_SIM="${START_SIM:-true}"
SYNTHETIC_ONLY="${SYNTHETIC_ONLY:-false}"
SIM_PID=""
BRIDGE_STARTED="false"
SCENARIO_MANAGER_STARTED="false"

usage() {
  cat <<'EOF'
Usage:
  run_sim_safety_regressions.sh [options]

Options:
  --synthetic-only   Run deterministic report checks without a live ROS graph
  --reuse-sim        Do not start sim; use the currently running ROS/Gazebo graph
  --results-dir DIR  Output directory for JSON reports
  -h, --help         Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --synthetic-only)
      SYNTHETIC_ONLY="true"
      shift
      ;;
    --reuse-sim)
      START_SIM="false"
      shift
      ;;
    --results-dir)
      [[ $# -ge 2 ]] || { echo "Missing value for --results-dir" >&2; exit 1; }
      RESULTS_DIR="$2"
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

cleanup() {
  if [[ "${SCENARIO_MANAGER_STARTED}" == "true" ]]; then
    pkill -f "ros2 run sim_scenarios scenario_manager" >/dev/null 2>&1 || true
  fi
  if [[ "${BRIDGE_STARTED}" == "true" ]]; then
    pkill -f "ros2 run ros_gz_bridge parameter_bridge /world/${WORLD_NAME}/create" >/dev/null 2>&1 || true
  fi
  if [[ -n "${SIM_PID}" ]]; then
    kill "${SIM_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

set +u
source /opt/ros/humble/setup.bash
source "${workspace_root}/install/setup.bash"
set -u

mkdir -p "${RESULTS_DIR}"

run_runner() {
  local report="$1"
  shift
  ros2 run sim_scenarios safety_regression_runner \
    --manifest safety_regression_cases.yaml \
    --report-file "${report}" \
    "$@"
}

if [[ "${SYNTHETIC_ONLY}" == "true" ]]; then
  run_runner "${RESULTS_DIR}/synthetic.json" --synthetic
  echo "Safety regression reports: ${RESULTS_DIR}"
  exit 0
fi

wait_for_topic() {
  local topic="$1"
  local timeout="$2"
  local start
  start="$(date +%s)"
  while (( "$(date +%s)" - start < timeout )); do
    if ros2 topic list 2>/dev/null | rg -q "^${topic}$"; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${topic}" >&2
  return 1
}

detect_partition_from_ign() {
  local ign_pid
  ign_pid="$(pgrep -f 'ign gazebo server' | head -n 1 || true)"
  [[ -n "${ign_pid}" ]] || return 1

  local part
  part="$(tr '\0' '\n' < "/proc/${ign_pid}/environ" | awk -F= '/^(GZ_PARTITION|IGN_PARTITION)=/ {print $2; exit}')"
  [[ -n "${part}" ]] || return 1

  export GZ_PARTITION="${part}"
  export IGN_PARTITION="${part}"
}

ensure_world_service_bridge() {
  if ros2 service list 2>/dev/null | rg -q "^/world/${WORLD_NAME}/create$"; then
    return 0
  fi
  detect_partition_from_ign || return 0
  ros2 run ros_gz_bridge parameter_bridge \
    "/world/${WORLD_NAME}/create@ros_gz_interfaces/srv/SpawnEntity" \
    "/world/${WORLD_NAME}/remove@ros_gz_interfaces/srv/DeleteEntity" \
    >/tmp/anubi_safety_world_bridge.log 2>&1 &
  BRIDGE_STARTED="true"
  sleep 2
}

ensure_scenario_manager() {
  if ros2 node list 2>/dev/null | rg -q '^/scenario_manager$'; then
    return 0
  fi
  detect_partition_from_ign || true
  ros2 run sim_scenarios scenario_manager --ros-args -p world_name:="${WORLD_NAME}" \
    >/tmp/anubi_safety_scenario_manager.log 2>&1 &
  SCENARIO_MANAGER_STARTED="true"
  sleep 2
}

load_scenario() {
  local scenario="$1"
  ensure_world_service_bridge || true
  ensure_scenario_manager || true
  ros2 param set /scenario_manager scenario_file "${scenario}" >/dev/null 2>&1 || true
  ros2 service call /sim/load_scenario std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
}

if [[ "${START_SIM}" == "true" ]] && ! ros2 topic list 2>/dev/null | rg -q '^/diagnostics$'; then
  "${workspace_root}/scripts/run_go2_sim.sh" mapping --no-gui --no-rviz --nav-delay 5.0 \
    >/tmp/anubi_safety_sim.log 2>&1 &
  SIM_PID="$!"
fi

wait_for_topic /diagnostics 90
wait_for_topic /cmd_vel_safe 90

rc=0
load_scenario safety_sudden_obstacle
run_runner "${RESULTS_DIR}/sudden_obstacle.json" --case sudden_obstacle --timeout 30 || rc=1

load_scenario safety_blocked_corridor
run_runner "${RESULTS_DIR}/blocked_corridor.json" --case blocked_corridor --timeout 30 || rc=1

run_runner "${RESULTS_DIR}/operator_override.json" --case operator_override --timeout 30 || rc=1
run_runner "${RESULTS_DIR}/stale_and_map_synthetic.json" --synthetic --case stale_sensor --case map_jump || rc=1

echo "Safety regression reports: ${RESULTS_DIR}"
exit "${rc}"
