#!/usr/bin/env bash
set -euo pipefail

PCD_INPUT=""
SEARCH_DIR="/tmp"
FRAME_ID="map"
PUBLISH_RATE="1.0"
RVIZ_CONFIG="/ws/anubi/src/autonomy/robot_nav/rviz/dlio_map.rviz"
START_RVIZ="true"

usage() {
  cat <<'EOF'
Usage:
  view_dlio_map.sh [options]

Publishes a DLIO PCD map on /cloud_pcd and optionally opens RViz.
If --pcd is not provided, the script loads the latest dlio_map.pcd from --search-dir.

Options:
  --pcd <path>          Path to .pcd file, or directory containing one
  --search-dir <dir>    Directory to search latest dlio_map.pcd (default: /tmp)
  --frame-id <frame>    Frame id for published cloud (default: map)
  --rate <hz>           Publish rate for pcd_to_pointcloud (default: 1.0)
  --rviz-config <path>  RViz config file (default: robot_nav dlio_map.rviz)
  --no-rviz             Publish only, do not open RViz
  -h, --help            Show this help

Examples:
  view_dlio_map.sh --pcd /tmp/go2_maps_20260423_144217/dlio_map.pcd
  view_dlio_map.sh --pcd /tmp/go2_maps_20260423_144217
  view_dlio_map.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pcd)
      [[ $# -ge 2 ]] || { echo "Missing value for --pcd" >&2; usage; exit 1; }
      PCD_INPUT="$2"
      shift 2
      ;;
    --search-dir)
      [[ $# -ge 2 ]] || { echo "Missing value for --search-dir" >&2; usage; exit 1; }
      SEARCH_DIR="$2"
      shift 2
      ;;
    --frame-id)
      [[ $# -ge 2 ]] || { echo "Missing value for --frame-id" >&2; usage; exit 1; }
      FRAME_ID="$2"
      shift 2
      ;;
    --rate)
      [[ $# -ge 2 ]] || { echo "Missing value for --rate" >&2; usage; exit 1; }
      PUBLISH_RATE="$2"
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

resolve_pcd_file() {
  local input="$1"

  if [[ -n "$input" ]]; then
    if [[ -d "$input" ]]; then
      local candidate
      candidate="$(find "$input" -maxdepth 1 -type f -name '*.pcd' | head -n 1 || true)"
      if [[ -z "$candidate" ]]; then
        echo "No PCD found in directory: $input" >&2
        exit 1
      fi
      echo "$candidate"
      return
    fi

    [[ -f "$input" ]] || { echo "PCD not found: $input" >&2; exit 1; }
    echo "$input"
    return
  fi

  [[ -d "$SEARCH_DIR" ]] || { echo "Search dir does not exist: $SEARCH_DIR" >&2; exit 1; }

  local latest
  latest="$(find "$SEARCH_DIR" -type f -name 'dlio_map.pcd' -printf '%T@\t%p\n' 2>/dev/null | sort -nr | head -n 1 | cut -f2- || true)"
  if [[ -z "$latest" ]]; then
    echo "No dlio_map.pcd found. Provide one with --pcd <path>." >&2
    exit 1
  fi
  echo "$latest"
}

PCD_FILE="$(resolve_pcd_file "$PCD_INPUT")"
echo "Using PCD: $PCD_FILE"

ros2 run pcl_ros pcd_to_pointcloud \
  --ros-args \
  -p file_name:="$PCD_FILE" \
  -p publish_rate:="$PUBLISH_RATE" \
  -p tf_frame:="$FRAME_ID" \
  >/tmp/view_dlio_map_publisher.log 2>&1 &
PCD_PUB_PID="$!"

cleanup() {
  kill "$PCD_PUB_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "Publishing /cloud_pcd in frame $FRAME_ID"

if [[ "$START_RVIZ" != "true" ]]; then
  wait "$PCD_PUB_PID"
  exit 0
fi

if [[ -f "$RVIZ_CONFIG" ]]; then
  exec rviz2 -d "$RVIZ_CONFIG"
fi

exec rviz2