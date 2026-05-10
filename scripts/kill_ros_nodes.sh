#!/usr/bin/env bash
set -euo pipefail

self=$$
parent=$PPID
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

declare -a pids=()

add_pids() {
  local pid
  while IFS= read -r pid; do
    [ -n "$pid" ] && pids+=("$pid")
  done
}

for name in \
  roscore rosmaster roslaunch rosout ros2 rviz2 rqt \
  ign ignition gazebo gz gzserver gzclient \
  gzserver-fortress gzclient-fortress \
  gazebo gazebo-fortress \
  parameter_bridge create \
  robot_state_publisher static_transform_publisher \
  pointcloud_to_laserscan_node cloud_accumulator.py velocity_limiter joystick_safety tilt_monitor wireless_key_inspector \
  quadruped_controller_node state_estimation_node \
  ekf_node spawner \
  champ champ_base champ_msgs champ_gazebo champ_utils champ_localization champ_navigation champ_teleop champ_description; do
  add_pids < <(pgrep -x "$name" || true)
done

add_pids < <(pgrep -f '/opt/ros/' || true)
add_pids < <(pgrep -f "${workspace_root}/install/" || true)
add_pids < <(pgrep -f "${workspace_root}/build/" || true)

add_pids < <(pgrep -f 'gazebo.*fortress' || true)
add_pids < <(pgrep -f 'gzserver.*fortress' || true)
add_pids < <(pgrep -f 'gzclient.*fortress' || true)
add_pids < <(pgrep -f 'ign gazebo' || true)
add_pids < <(pgrep -f 'ignition-gazebo' || true)
add_pids < <(pgrep -f 'gz sim' || true)

add_pids < <(pgrep -f 'sim_champ.launch.py' || true)
add_pids < <(pgrep -f 'ros_ign_gazebo' || true)
add_pids < <(pgrep -f 'ros_ign_bridge' || true)
add_pids < <(pgrep -f 'controller_manager/spawner' || true)
add_pids < <(pgrep -f 'pointcloud_to_laserscan' || true)
add_pids < <(pgrep -f 'robot_nav/lib/robot_nav/cloud_accumulator.py' || true)
add_pids < <(pgrep -f 'robot_nav/lib/robot_nav/distortion_corrector' || true)
add_pids < <(pgrep -f 'velocity_limiter' || true)
add_pids < <(pgrep -f 'joystick_safety' || true)
add_pids < <(pgrep -f 'tilt_monitor' || true)
add_pids < <(pgrep -f 'wireless_key_inspector' || true)
add_pids < <(pgrep -f 'route_server' || true)
add_pids < <(pgrep -f 'ros2 bag' || true)
add_pids < <(pgrep -f 'rosbag2_transport' || true)

add_pids < <(pgrep -f 'champ' || true)

pids=$(
  printf '%s\n' "${pids[@]:-}" \
  | rg -v '^\s*$' \
  | sort -u \
  | rg -v "^(${self}|${parent})$" \
  || true
)

if [ -z "${pids:-}" ]; then
  echo "No ROS processes found."
  exit 0
fi

echo "Stopping ROS processes:"
printf '%s\n' "$pids"

while IFS= read -r pid; do
  kill "$pid" 2>/dev/null || true
done <<< "$pids"

sleep 1

while IFS= read -r pid; do
  if kill -0 "$pid" 2>/dev/null; then
    kill -9 "$pid" 2>/dev/null || true
  fi
done <<< "$pids"

echo "Done."
