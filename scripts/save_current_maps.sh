#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR=""
MAP_BASENAME="nav2_map"
SLAM_BASENAME="slam_map"
DLIO_LEAF_SIZE="0.10"
TIMEOUT_SEC="8"
DLIO_KEYFRAMES_TOPIC="/kf_cloud"
AUTO_START_DLIO_MAP="true"
SKIP_DLIO="false"
SKIP_NAV2="false"
TEMP_DLIO_MAP_PID=""

usage() {
  cat <<'EOF'
Usage:
  save_current_maps.sh [options]

Save current maps from running ROS graph:
  1) DLIO 3D map (PCD) via SavePCD service
  2) Nav2 occupancy map (YAML + image) via map_saver_cli
  3) slam_toolbox posegraph (.posegraph + .data) via serialize_map

Options:
  --output-dir <path>        Output directory (default: /tmp/go2_maps_<timestamp>)
  --map-basename <name>      Base name for Nav2 occupancy map files (default: nav2_map)
  --slam-basename <name>     Base name for slam_toolbox posegraph (default: slam_map)
  --dlio-leaf-size <meters>  Voxel leaf size for DLIO PCD save (default: 0.10)
  --dlio-keyframes-topic <t> Keyframe topic consumed by dlio_map_node (default: /kf_cloud)
  --no-autostart-dlio-map    Do not auto-start temporary dlio_map_node if save_pcd is missing
  --timeout <sec>            Service wait timeout (default: 8)
  --skip-dlio                Skip DLIO 3D map save
  --skip-nav2                Skip Nav2/slam map saves
  -h, --help                 Show this help

Examples:
  save_current_maps.sh
  save_current_maps.sh --output-dir /ws/anubi/src/autonomy/robot_nav/maps/session_01
  save_current_maps.sh --dlio-keyframes-topic /dlio/odom_node/pointcloud/keyframe
  save_current_maps.sh --dlio-leaf-size 0.05 --map-basename office_a
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      [[ $# -ge 2 ]] || { echo "Missing value for --output-dir" >&2; usage; exit 1; }
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --map-basename)
      [[ $# -ge 2 ]] || { echo "Missing value for --map-basename" >&2; usage; exit 1; }
      MAP_BASENAME="$2"
      shift 2
      ;;
    --slam-basename)
      [[ $# -ge 2 ]] || { echo "Missing value for --slam-basename" >&2; usage; exit 1; }
      SLAM_BASENAME="$2"
      shift 2
      ;;
    --dlio-leaf-size)
      [[ $# -ge 2 ]] || { echo "Missing value for --dlio-leaf-size" >&2; usage; exit 1; }
      DLIO_LEAF_SIZE="$2"
      shift 2
      ;;
    --dlio-keyframes-topic)
      [[ $# -ge 2 ]] || { echo "Missing value for --dlio-keyframes-topic" >&2; usage; exit 1; }
      DLIO_KEYFRAMES_TOPIC="$2"
      shift 2
      ;;
    --no-autostart-dlio-map)
      AUTO_START_DLIO_MAP="false"
      shift
      ;;
    --timeout)
      [[ $# -ge 2 ]] || { echo "Missing value for --timeout" >&2; usage; exit 1; }
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --skip-dlio)
      SKIP_DLIO="true"
      shift
      ;;
    --skip-nav2)
      SKIP_NAV2="true"
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

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="/tmp/go2_maps_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "$OUTPUT_DIR"

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Required command not found: $cmd" >&2
    exit 1
  fi
}

bootstrap_ros_env_if_needed() {
  # Always source workspace overlay when available: ros2 can be present from
  # /opt/ros only, but custom interfaces (e.g. SavePCD) live in install/.
  set +u
  if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "/ws/anubi/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source /ws/anubi/install/setup.bash
  fi
  set -u
}

bootstrap_ros_env_if_needed
require_cmd ros2

run_with_timeout() {
  if command -v timeout >/dev/null 2>&1; then
    timeout "${TIMEOUT_SEC}s" "$@"
  else
    "$@"
  fi
}

# --- Daemon-free helpers ---------------------------------------------------
# The ros2 CLI daemon hangs on this workspace's Humble + rmw_cyclonedds_cpp
# combination, so we discover services/topics with --no-daemon and call
# services through rclpy instead of `ros2 service call`. These helpers work
# whether or not a daemon is also running.

ros2_service_list_no_daemon() {
  ros2 service list --no-daemon --spin-time 2 2>/dev/null
}

ros2_topic_list_no_daemon() {
  ros2 topic list --no-daemon --spin-time 2 2>/dev/null
}

# Call a ROS 2 service through rclpy. Exits 0 on success, non-zero otherwise.
#   $1: service name (e.g. /save_pcd)
#   $2: fully qualified service type (e.g. direct_lidar_inertial_odometry/srv/SavePCD)
#   $3: YAML request body (e.g. "{leaf_size: 0.1, save_path: '/tmp/out'}")
#   $4: wait timeout in seconds (default 30)
#   $5: response wait timeout in seconds (default 60)
call_ros2_service_rclpy() {
  local svc="$1"
  local pkg_type="$2"
  local body="$3"
  local wait_sec="${4:-30}"
  local resp_sec="${5:-60}"

  python3 - "$svc" "$pkg_type" "$body" "$wait_sec" "$resp_sec" <<'PY'
import sys
import time
from importlib import import_module

import yaml

import rclpy
from rclpy.node import Node

svc, pkg_type, body, wait_sec, resp_sec = sys.argv[1:6]
wait_sec = float(wait_sec)
resp_sec = float(resp_sec)

pkg, mid, srv_name = pkg_type.split("/", 2)
if mid != "srv":
    print(f"unexpected service type: {pkg_type}", file=sys.stderr)
    sys.exit(64)
SrvType = getattr(import_module(f"{pkg}.srv"), srv_name)
fields = yaml.safe_load(body) or {}

rclpy.init()
node = Node("save_current_maps_client")
client = node.create_client(SrvType, svc)
deadline = time.time() + wait_sec
while time.time() < deadline:
    if client.service_is_ready():
        break
    rclpy.spin_once(node, timeout_sec=0.2)
else:
    print(f"{svc}: not available after {wait_sec:.0f}s", file=sys.stderr)
    rclpy.shutdown()
    sys.exit(2)

req = SrvType.Request()
for k, v in fields.items():
    setattr(req, k, v)

future = client.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=resp_sec)
resp = future.result()
if resp is None:
    print(f"{svc}: no response within {resp_sec:.0f}s", file=sys.stderr)
    rclpy.shutdown()
    sys.exit(3)

print(f"{svc} response: {resp}")
success_attr = getattr(resp, "success", None)
rclpy.shutdown()
# Honor an explicit success flag if the service has one; otherwise treat a
# returned response as success.
sys.exit(0 if (success_attr is True or success_attr is None) else 4)
PY
}

echo "Saving maps into: $OUTPUT_DIR"

cleanup() {
  if [[ -n "$TEMP_DLIO_MAP_PID" ]]; then
    kill "$TEMP_DLIO_MAP_PID" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

start_temporary_dlio_map_node() {
  if [[ "$AUTO_START_DLIO_MAP" != "true" ]]; then
    return 1
  fi

  echo "[DLIO] save_pcd not found, starting temporary dlio_map_node..."
  ros2 run direct_lidar_inertial_odometry dlio_map_node \
    --ros-args -r keyframes:="$DLIO_KEYFRAMES_TOPIC" \
    >"$OUTPUT_DIR/dlio_map_node.log" 2>&1 &
  TEMP_DLIO_MAP_PID="$!"

  local i
  for i in $(seq 1 "$TIMEOUT_SEC"); do
    if ros2_service_list_no_daemon | grep -E '(^|/)save_pcd$' >/dev/null 2>&1; then
      echo "[DLIO] temporary dlio_map_node is ready"
      return 0
    fi
    sleep 1
  done

  echo "[WARN] temporary dlio_map_node did not expose save_pcd within timeout."
  return 1
}

save_dlio_map() {
  local save_pcd_service
  save_pcd_service="$(ros2_service_list_no_daemon | grep -E '(^|/)save_pcd$' | head -n 1 || true)"

  if [[ -z "$save_pcd_service" ]]; then
    start_temporary_dlio_map_node || {
      echo "[WARN] No save_pcd service found."
      echo "       Start DLIO map node (direct_lidar_inertial_odometry dlio_map_node) and retry."
      return 1
    }
    save_pcd_service="$(ros2_service_list_no_daemon | grep -E '(^|/)save_pcd$' | head -n 1 || true)"
  fi

  # dlio_map_node always advertises this exact type, so we skip a daemon-
  # backed `ros2 service type` lookup (which hangs on cyclonedds builds).
  local save_pcd_type="direct_lidar_inertial_odometry/srv/SavePCD"

  echo "[DLIO] Using service: $save_pcd_service"
  echo "[DLIO] Service type: $save_pcd_type"
  if ! call_ros2_service_rclpy "$save_pcd_service" "$save_pcd_type" \
      "{leaf_size: $DLIO_LEAF_SIZE, save_path: '$OUTPUT_DIR'}" \
      "$TIMEOUT_SEC" 60; then
    echo "[WARN] save_pcd service call failed or timed out."
    return 1
  fi

  if [[ -f "$OUTPUT_DIR/dlio_map.pcd" ]]; then
    echo "[DLIO] Saved: $OUTPUT_DIR/dlio_map.pcd"
  else
    echo "[WARN] DLIO save service returned, but dlio_map.pcd was not found in output dir."
    return 1
  fi

  return 0
}

save_nav2_and_slam_maps() {
  local map_topic="/map"
  local nav2_prefix="$OUTPUT_DIR/$MAP_BASENAME"
  local slam_prefix="$OUTPUT_DIR/$SLAM_BASENAME"
  local had_error=0

  if ! ros2_topic_list_no_daemon | grep -Fx "$map_topic" >/dev/null 2>&1; then
    echo "[WARN] Topic $map_topic not found. Nav2 map save skipped."
    return 1
  fi

  echo "[NAV2] Saving occupancy map from $map_topic to ${nav2_prefix}.yaml/.pgm"
  if ! ros2 run nav2_map_server map_saver_cli \
      -f "$nav2_prefix" \
      --ros-args -p save_map_timeout:=5.0 -p map_subscribe_transient_local:=true; then
    echo "[WARN] map_saver_cli failed."
    had_error=1
  fi

  if [[ -f "${nav2_prefix}.yaml" ]]; then
    echo "[NAV2] Saved: ${nav2_prefix}.yaml"
  else
    echo "[WARN] Nav2 map saver finished, but ${nav2_prefix}.yaml not found."
    had_error=1
  fi

  local serialize_service="/slam_toolbox/serialize_map"
  if ros2_service_list_no_daemon | grep -Fx "$serialize_service" >/dev/null 2>&1; then
    echo "[SLAM] Saving posegraph to ${slam_prefix}.posegraph/.data"
    if ! call_ros2_service_rclpy "$serialize_service" \
        "slam_toolbox/srv/SerializePoseGraph" \
        "{filename: '$slam_prefix'}" \
        "$TIMEOUT_SEC" 60; then
      echo "[WARN] serialize_map service call failed or timed out."
      had_error=1
    fi

    if [[ -f "${slam_prefix}.posegraph" ]]; then
      echo "[SLAM] Saved: ${slam_prefix}.posegraph"
    else
      echo "[WARN] serialize_map completed, but ${slam_prefix}.posegraph not found."
      had_error=1
    fi
  else
    echo "[WARN] Service $serialize_service not found. slam_toolbox posegraph not saved."
    echo "       Localization in this repo expects a .posegraph map."
    had_error=1
  fi

  if [[ "$had_error" -eq 0 ]]; then
    return 0
  fi

  return 1
}

DLIO_OK=0
NAV2_OK=0

if [[ "$SKIP_DLIO" != "true" ]]; then
  save_dlio_map && DLIO_OK=1 || true
fi

if [[ "$SKIP_NAV2" != "true" ]]; then
  save_nav2_and_slam_maps && NAV2_OK=1 || true
fi

echo
echo "Save summary:"
if [[ "$SKIP_DLIO" == "true" ]]; then
  echo "  DLIO 3D map: skipped"
elif [[ "$DLIO_OK" -eq 1 ]]; then
  echo "  DLIO 3D map: ok ($OUTPUT_DIR/dlio_map.pcd)"
else
  echo "  DLIO 3D map: failed or unavailable"
fi

if [[ "$SKIP_NAV2" == "true" ]]; then
  echo "  Nav2/slam map: skipped"
elif [[ "$NAV2_OK" -eq 1 ]]; then
  echo "  Nav2/slam map: done (check $OUTPUT_DIR)"
else
  echo "  Nav2/slam map: failed or unavailable"
fi

echo "Output directory: $OUTPUT_DIR"