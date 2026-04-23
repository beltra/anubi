#!/usr/bin/env bash
set -euo pipefail

MAP_INPUT=""
SEARCH_DIR="/tmp"
RVIZ_CONFIG="/ws/anubi/src/autonomy/robot_nav/rviz/nav2.rviz"
START_RVIZ="true"

usage() {
  cat <<'EOF'
Usage:
  view_nav2_map.sh [options]

Starts Nav2 map_server with a saved map YAML and opens RViz.
If --map is not provided, the script loads the latest map YAML from --search-dir.

Options:
  --map <path>          Map YAML file, or directory containing one
  --search-dir <dir>    Directory used to find latest map (default: /tmp)
  --rviz-config <path>  RViz config file (default: robot_nav nav2.rviz)
  --no-rviz             Start map_server only (no RViz)
  -h, --help            Show this help

Examples:
  view_nav2_map.sh --map /tmp/go2_maps_20260423_144217/nav2_map.yaml
  view_nav2_map.sh --map /tmp/go2_maps_20260423_144217
  view_nav2_map.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      [[ $# -ge 2 ]] || { echo "Missing value for --map" >&2; usage; exit 1; }
      MAP_INPUT="$2"
      shift 2
      ;;
    --search-dir)
      [[ $# -ge 2 ]] || { echo "Missing value for --search-dir" >&2; usage; exit 1; }
      SEARCH_DIR="$2"
      shift 2
      ;;
    --rviz-config)
      [[ $# -ge 2 ]] || { echo "Missing value for --rviz-config" >&2; usage; exit 1; }
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    --no-rviz)
      START_RVIZ="false"
      shift
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

set +u
source /opt/ros/humble/setup.bash
if [[ -f /ws/anubi/install/setup.bash ]]; then
  source /ws/anubi/install/setup.bash
fi
set -u

is_map_yaml() {
  local f="$1"
  [[ -f "$f" ]] || return 1
  grep -q '^image:' "$f" && grep -q '^resolution:' "$f"
}

resolve_map_yaml() {
  local input="$1"

  if [[ -n "$input" ]]; then
    if [[ -d "$input" ]]; then
      local candidate
      candidate="$(find "$input" -maxdepth 1 -type f -name '*.yaml' | head -n 1 || true)"
      if [[ -z "$candidate" ]]; then
        echo "No YAML map found in directory: $input" >&2
        exit 1
      fi
      if ! is_map_yaml "$candidate"; then
        echo "YAML found but does not look like a Nav2 map metadata file: $candidate" >&2
        exit 1
      fi
      echo "$candidate"
      return
    fi

    if ! is_map_yaml "$input"; then
      echo "Invalid map YAML: $input" >&2
      exit 1
    fi

    echo "$input"
    return
  fi

  if [[ ! -d "$SEARCH_DIR" ]]; then
    echo "Search dir does not exist: $SEARCH_DIR" >&2
    exit 1
  fi

  local latest
  latest="$({
    find "$SEARCH_DIR" -type f -name '*.yaml' 2>/dev/null
    find /ws/anubi/src/autonomy/robot_nav/maps -type f -name '*.yaml' 2>/dev/null
  } | while read -r f; do
    if is_map_yaml "$f"; then
      printf '%s\t%s\n' "$(stat -c %Y "$f")" "$f"
    fi
  done | sort -nr | head -n 1 | cut -f2- || true)"

  if [[ -z "$latest" ]]; then
    echo "No map YAML found. Provide one with --map <path>." >&2
    exit 1
  fi

  echo "$latest"
}

MAP_YAML="$(resolve_map_yaml "$MAP_INPUT")"

echo "Using map: $MAP_YAML"

MAP_SERVER_NODE="map_server_viewer"
MAP_SERVER_FQN="/$MAP_SERVER_NODE"

ros2 run nav2_map_server map_server \
  --ros-args \
  -r __node:="$MAP_SERVER_NODE" \
  -p yaml_filename:="$MAP_YAML" \
  -p topic_name:=/map \
  -p frame_id:=map \
  >/tmp/view_nav2_map_map_server.log 2>&1 &
MAP_SERVER_PID="$!"

cleanup() {
  kill "$MAP_SERVER_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# Wait for map_server lifecycle endpoint.
for _ in $(seq 1 10); do
  if ros2 lifecycle get "$MAP_SERVER_FQN" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

STATE_LINE="$(ros2 lifecycle get "$MAP_SERVER_FQN" 2>/dev/null || true)"

if echo "$STATE_LINE" | grep -qi "unconfigured"; then
  ros2 lifecycle set "$MAP_SERVER_FQN" configure >/dev/null
  ros2 lifecycle set "$MAP_SERVER_FQN" activate >/dev/null
elif echo "$STATE_LINE" | grep -qi "inactive"; then
  ros2 lifecycle set "$MAP_SERVER_FQN" activate >/dev/null
elif echo "$STATE_LINE" | grep -qi "active"; then
  :
else
  # Fallback for unexpected state text: attempt normal transition path.
  ros2 lifecycle set "$MAP_SERVER_FQN" configure >/dev/null || true
  ros2 lifecycle set "$MAP_SERVER_FQN" activate >/dev/null
fi

if [[ "$START_RVIZ" != "true" ]]; then
  wait "$MAP_SERVER_PID"
  exit 0
fi

if [[ -f "$RVIZ_CONFIG" ]]; then
  exec rviz2 -d "$RVIZ_CONFIG"
fi

exec rviz2