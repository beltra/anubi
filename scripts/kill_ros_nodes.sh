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
add_pids < <(pgrep -f 'sim_bringup_g1.launch.py' || true)
add_pids < <(pgrep -f 'sim_g1.launch.py' || true)
add_pids < <(pgrep -f 'go2_bringup.launch.py' || true)
add_pids < <(pgrep -f 'g1_bringup.launch.py' || true)
# G1 real-robot adapters / sensors / odometry
add_pids < <(pgrep -f 'g1_state_bridge' || true)
add_pids < <(pgrep -f 'cmd_vel_to_amo' || true)
add_pids < <(pgrep -f 'cmd_vel_to_sonic' || true)
add_pids < <(pgrep -f 'imu_rescale' || true)
add_pids < <(pgrep -f 'livox_ros_driver2' || true)
add_pids < <(pgrep -f 'dlio_odom_node' || true)
add_pids < <(pgrep -f 'dlio_map_node' || true)
add_pids < <(pgrep -f 'lidar_odom_relay' || true)
add_pids < <(pgrep -f 'collision_monitor' || true)
add_pids < <(pgrep -f 'slam_toolbox' || true)
add_pids < <(pgrep -f 'ign gazebo -g' || true)
add_pids < <(pgrep -f 'lidar_level_frame_publisher' || true)
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

# Use grep (always present) rather than rg so this runs identically on the host and
# inside the sim container.
pids=$(
  printf '%s\n' "${pids[@]:-}" \
  | grep -vE '^[[:space:]]*$' \
  | sort -u \
  | grep -vE "^(${self}|${parent})$" \
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

# FastDDS leaves shared-memory segments and locks in /dev/shm after kill -9. On a small
# /dev/shm (the 64 MB Docker default) these accumulate across restarts until SHM
# transport can no longer allocate a buffer for the large LiDAR PointCloud2; DDS then
# falls back to slow UDP and Nav2 lifecycle bringup starts timing out. Reap them so each
# restart starts from a clean shared-memory pool.
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

# Zombie (defunct) ROS nodes cannot be reaped by this script: their parent launch
# already exited, so they are orphaned to PID 1, and the container's PID 1 (a plain
# entrypoint, not an init/reaper) never wait()s on them. They accumulate across
# launches and, once there are hundreds, the DDS discovery / Nav2 lifecycle bring-up
# starts timing out (collision_monitor stuck inactive, "Node not found" for live
# nodes). The only fix is to restart the container, which reaps them all. Warn loudly.
zombies=$(ps -eo stat 2>/dev/null | grep -c '^Z' || true)
if [ "${zombies:-0}" -ge 50 ]; then
  echo "WARNING: ${zombies} zombie (defunct) processes present; this script cannot reap them."
  echo "         They will eventually break DDS discovery / Nav2 lifecycle bring-up."
  echo "         Restart the container to clear them:  docker restart <container>"
fi

echo "Done."
