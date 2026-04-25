#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  source "${workspace_root}/install/setup.bash"
fi
set -u

check_launch_args() {
  local package="$1"
  local launch_file="$2"
  echo "[launch-contract] ${package} ${launch_file}"
  ros2 launch "${package}" "${launch_file}" --show-args >/tmp/anubi_launch_contract_${package}_${launch_file}.log
}

check_script_help() {
  local script="$1"
  echo "[launch-contract] ${script} --help"
  "${workspace_root}/${script}" --help >/tmp/anubi_launch_contract_$(basename "${script}").log
}

check_launch_args robot_sim sim_bringup.launch.py
check_launch_args go2_sim sim_champ.launch.py
check_launch_args robot_nav nav2.launch.py
check_launch_args robot_safety safety.launch.py
check_launch_args go2_bringup go2_bringup.launch.py

check_script_help scripts/run_go2_sim.sh
check_script_help scripts/run_go2_real.sh

ANUBI_WORKSPACE_ROOT="${workspace_root}" python3 - <<'PY'
import os
from pathlib import Path

root = Path(os.environ["ANUBI_WORKSPACE_ROOT"])
topic_map = (root / "docs" / "topic_map.md").read_text(encoding="utf-8")
required = [
    "/diagnostics",
    "/cmd_vel_collision_free",
    "/cmd_vel_safe",
    "map -> odom",
    "odom -> base_footprint",
]
missing = [item for item in required if item not in topic_map]
if missing:
    raise SystemExit(f"topic_map.md missing contract entries: {', '.join(missing)}")
PY

echo "[launch-contract] OK"
