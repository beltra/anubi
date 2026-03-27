#!/usr/bin/env bash
set -euo pipefail

# Gracefully stop ROS1 nodes first when rosnode is available.
if command -v rosnode >/dev/null 2>&1; then
  rosnode kill -a >/tmp/rosnode_kill_all.log 2>&1 || true
fi

# Collect ROS-related processes and avoid killing this script shell.
self=$$
parent=$PPID

declare -a pids=()

add_pids() {
  local pid
  while IFS= read -r pid; do
    [ -n "$pid" ] && pids+=("$pid")
  done
}

# Common ROS/Gazebo process names.
for name in roscore rosmaster roslaunch rosout ros2 rviz2 rqt gzserver gzclient; do
  add_pids < <(pgrep -x "$name" || true)
done

# Catch ROS-installed executables that may not use common names.
add_pids < <(pgrep -f '/opt/ros/' || true)

# Unique + filter out this script shell and parent.
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

# Try TERM first.
while IFS= read -r pid; do
  kill "$pid" 2>/dev/null || true
done <<< "$pids"

sleep 1

# Force kill any remaining.
while IFS= read -r pid; do
  if kill -0 "$pid" 2>/dev/null; then
    kill -9 "$pid" 2>/dev/null || true
  fi
done <<< "$pids"

echo "Done."
