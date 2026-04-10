#!/usr/bin/env bash
set -euo pipefail

# Runs the MPPI benchmark suite for default_safe scenarios and stores JSON reports.
# Optional environment variables:
#   RESULTS_DIR        Output directory (default: /ws/anubi/log/mppi_suite_<timestamp>)
#   GOALS_FILE         Benchmark goals manifest (default: benchmark_goals_default_safe.yaml)
#   WORLD_NAME         Gazebo world name for set_pose service (default: indoor_office)
#   ROBOT_ENTITY_NAME  Gazebo model name for robot reset (default: go2)
#   ENABLE_POSE_RESET  true|false (default: true)
#   BRIDGE_WORLD_SERVICES true|false (default: true)
#   AUTO_START_SCENARIO_MANAGER true|false (default: true)

set +u
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
set -u

stamp="$(date +%Y-%m-%d_%H-%M-%S)"
RESULTS_DIR="${RESULTS_DIR:-/ws/anubi/log/mppi_suite_${stamp}}"
GOALS_FILE="${GOALS_FILE:-benchmark_goals_default_safe.yaml}"
WORLD_NAME="${WORLD_NAME:-indoor_office}"
ROBOT_ENTITY_NAME="${ROBOT_ENTITY_NAME:-go2}"
ENABLE_POSE_RESET="${ENABLE_POSE_RESET:-true}"
BRIDGE_WORLD_SERVICES="${BRIDGE_WORLD_SERVICES:-true}"
AUTO_START_SCENARIO_MANAGER="${AUTO_START_SCENARIO_MANAGER:-true}"

BRIDGE_TERM_ID=""
SCENARIO_TERM_ID=""

cleanup() {
  if [[ -n "${SCENARIO_TERM_ID}" ]]; then
    pkill -f "ros2 run sim_scenarios scenario_manager" >/dev/null 2>&1 || true
  fi
  if [[ -n "${BRIDGE_TERM_ID}" ]]; then
    pkill -f "ros2 run ros_gz_bridge parameter_bridge /world/${WORLD_NAME}/create@ros_gz_interfaces/srv/SpawnEntity" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

detect_partition_from_ign() {
  local ign_pid
  ign_pid="$(pgrep -f 'ign gazebo server' | head -n 1 || true)"
  if [[ -z "${ign_pid}" ]]; then
    return 1
  fi

  local part
  part="$(tr '\0' '\n' < "/proc/${ign_pid}/environ" | awk -F= '/^(GZ_PARTITION|IGN_PARTITION)=/ {print $2; exit}')"
  if [[ -z "${part}" ]]; then
    return 1
  fi

  export GZ_PARTITION="${part}"
  export IGN_PARTITION="${part}"
  return 0
}

ensure_world_service_bridge() {
  if [[ "${BRIDGE_WORLD_SERVICES}" != "true" ]]; then
    return 0
  fi

  if ros2 service list 2>/dev/null | rg -q "^/world/${WORLD_NAME}/create$" && \
     ros2 service list 2>/dev/null | rg -q "^/world/${WORLD_NAME}/set_pose$"; then
    return 0
  fi

  if ! detect_partition_from_ign; then
    echo "[suite] warning: unable to detect Gazebo partition; cannot auto-bridge world services" >&2
    return 1
  fi

  echo "[suite] starting world service bridge for /world/${WORLD_NAME} (partition=${GZ_PARTITION})"
  ros2 run ros_gz_bridge parameter_bridge \
    "/world/${WORLD_NAME}/create@ros_gz_interfaces/srv/SpawnEntity" \
    "/world/${WORLD_NAME}/remove@ros_gz_interfaces/srv/DeleteEntity" \
    "/world/${WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose" \
    >/tmp/mppi_world_bridge.log 2>&1 &
  BRIDGE_TERM_ID="bridge_started"

  sleep 2
  if ! ros2 service list 2>/dev/null | rg -q "^/world/${WORLD_NAME}/create$"; then
    echo "[suite] warning: world service bridge did not expose /world/${WORLD_NAME}/create" >&2
    return 1
  fi
  return 0
}

mkdir -p "${RESULTS_DIR}"

scenarios=(crowded_room narrow_corridor)

load_scenario_if_available() {
  local scenario="$1"
  if ros2 node list 2>/dev/null | rg -q '^/scenario_manager$'; then
    ros2 param set /scenario_manager scenario_file "${scenario}" >/dev/null || true
    ros2 service call /sim/load_scenario std_srvs/srv/Trigger "{}" >/dev/null || true
  fi
}

ensure_scenario_manager() {
  if [[ "${AUTO_START_SCENARIO_MANAGER}" != "true" ]]; then
    return 0
  fi

  if ros2 node list 2>/dev/null | rg -q '^/scenario_manager$'; then
    return 0
  fi

  if ! detect_partition_from_ign; then
    echo "[suite] warning: unable to detect Gazebo partition; scenario_manager may fail" >&2
  fi

  echo "[suite] starting scenario_manager (world_name=${WORLD_NAME})"
  ros2 run sim_scenarios scenario_manager --ros-args -p world_name:="${WORLD_NAME}" \
    >/tmp/mppi_scenario_manager.log 2>&1 &
  SCENARIO_TERM_ID="scenario_started"
  sleep 2
}

echo "MPPI suite output: ${RESULTS_DIR}"
ensure_world_service_bridge || true
ensure_scenario_manager || true
rc=0

for scenario in "${scenarios[@]}"; do
  echo "[suite] scenario=${scenario}"
  load_scenario_if_available "${scenario}"

  report="${RESULTS_DIR}/${scenario}.json"
  if ! ros2 run sim_scenarios benchmark_mppi --ros-args \
    -p goals_file:="${GOALS_FILE}" \
    -p scenario:="${scenario}" \
    -p report_file:="${report}" \
    -p world_name:="${WORLD_NAME}" \
    -p robot_entity_name:="${ROBOT_ENTITY_NAME}" \
    -p enable_pose_reset:="${ENABLE_POSE_RESET}"; then
    echo "[suite] scenario ${scenario} failed" >&2
    rc=1
  fi
done

echo "[suite] reports generated in ${RESULTS_DIR}"

# Create a tiny human-readable summary.
python3 - <<'PY' "${RESULTS_DIR}"
import json
import os
import sys

results_dir = sys.argv[1]
rows = []
for name in sorted(os.listdir(results_dir)):
    if not name.endswith('.json'):
        continue
    path = os.path.join(results_dir, name)
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception:
        continue
    s = data.get('summary', {})
    rows.append((
        data.get('scenario', os.path.splitext(name)[0]),
        s.get('goals', 0),
        s.get('navigate_success', 0),
        s.get('spin_episodes_total', 0),
        s.get('compute_path_max_s', 0.0),
    ))

if not rows:
    print('[suite] no JSON reports found')
    sys.exit(0)

print('[suite] summary')
print('scenario, goals, nav_ok, spins_total, compute_path_max_s')
for r in rows:
    print(f'{r[0]}, {r[1]}, {r[2]}, {r[3]}, {r[4]}')
PY

exit "${rc}"
