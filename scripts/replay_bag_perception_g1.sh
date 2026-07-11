#!/usr/bin/env bash
# Replay the recorded G1 LiDAR bag through the G1 tuning stack.
#
# Purpose
# -------
# Tune the G1 perception pipeline (distortion corrector self-filter, /scan
# band, slam_toolbox, Nav2/STVL) against recorded real-G1 data with zero
# double-publisher noise. The bag is played with a strict whitelist; every
# TF and every perception output is regenerated fresh inside
# bag_perception_g1.launch.py.
#
# Two bag profiles are auto-detected from metadata.yaml:
#
#   legacy-inverted (bags/g1/lidar_record): plays /livox/lidar_filtered +
#   /livox/imu. The cloud is INVERTED (pre-driver-righting recording); static
#   upright lidar_stabilized, no gait leveling.
#
#   live-stack (run_g1_real.sh --record-rosbag bags, e.g. bags/g1_planner_tune_*):
#   plays /livox/lidar + /livox/imu + /imu/data. The cloud is the UPRIGHT live
#   driver output and the body IMU drives the real gait-leveling chain
#   (lidar_level_frame_publisher + the LIVE distortion_corrector_g1.yaml).
#
# Everything else is dropped -- INCLUDING /tf and /tf_static and every
# recorded derived output (/scan, /map, /odom, /lidar_odom, /lidar/points_*):
# each would compete with the fresh publisher this launch starts. The bag's
# /tf_static (where present) also carries a base_link -> base_footprint that
# would fight the replay's floor-level base_footprint and shift the odom
# z-datum the STVL heights are tuned in.
#
# Usage:
#   replay_bag_perception_g1.sh [bag_path] [options]
#
#   bag_path defaults to bags/g1/lidar_record.
#
# Options:
#   --rate <hz>           Playback rate (default: 1.0).
#   --loop                Loop the bag.
#   --start-offset <s>    Start playback offset in seconds.
#   --no-rviz             Do not start RViz.
#   --save-map <dir>      On exit, save Nav2 occupancy map + slam_toolbox
#                         posegraph + DLIO 3D map (dlio_map.pcd, via the
#                         replay's own dlio_map_node); also record the replay
#                         outputs (/tf, /map, /scan, /lidar_odom, costmaps,
#                         nav cloud) to <dir>/replay_outputs for
#                         perception_metrics.py.
#   --launch-arg <k:=v>   Extra argument forwarded to the launch (repeatable),
#                         e.g. --launch-arg distortion_params:=/path/to.yaml
#   -h, --help            Show this help.

set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BAG_INPUT=""
RATE="1.0"
LOOP="false"
START_OFFSET=""
USE_RVIZ="true"
SAVE_MAP_DIR=""
LAUNCH_DELAY_SEC="6"
EXTRA_LAUNCH_ARGS=()

usage() {
  sed -n '2,/^set -euo pipefail$/p' "$0" | sed -n '/^# /p' | sed 's/^# \?//'
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate)         RATE="$2"; shift 2 ;;
    --loop)         LOOP="true"; shift ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    --no-rviz)      USE_RVIZ="false"; shift ;;
    --save-map)     SAVE_MAP_DIR="$2"; shift 2 ;;
    --launch-arg)   EXTRA_LAUNCH_ARGS+=("$2"); shift 2 ;;
    -h|--help)      usage ;;
    -*)             echo "Unknown option: $1" >&2; usage ;;
    *)
      if [[ -z "$BAG_INPUT" ]]; then BAG_INPUT="$1"
      else echo "Unexpected positional: $1" >&2; exit 1
      fi
      shift
      ;;
  esac
done

[[ -n "$BAG_INPUT" ]] || BAG_INPUT="${WS_ROOT}/bags/g1/lidar_record"
[[ -e "$BAG_INPUT" ]] || { echo "Bag path not found: $BAG_INPUT" >&2; exit 1; }
if [[ -d "$BAG_INPUT" && -f "$BAG_INPUT/metadata.yaml" ]]; then
  BAG_DIR="$BAG_INPUT"
else
  echo "Expected a ROS2 bag directory (with metadata.yaml). Got: $BAG_INPUT" >&2
  exit 1
fi

# Profile detection: live-stack bags carry the upright driver cloud
# (/livox/lidar) plus the body IMU (/imu/data); the legacy inverted recording
# carries /livox/lidar_filtered only.
PROFILE=""
PROFILE_LAUNCH_ARGS=()
if grep -q "name: /livox/lidar$" "$BAG_DIR/metadata.yaml" \
   && grep -q "name: /imu/data$" "$BAG_DIR/metadata.yaml"; then
  PROFILE="live-stack"
  PLAY_TOPICS=(/livox/lidar /livox/imu /imu/data)
  PROFILE_LAUNCH_ARGS=(
    lidar_input_topic:=/livox/lidar
    use_body_imu:=true
    distortion_imu_topic:=/imu/data
    distortion_params:="${WS_ROOT}/install/robot_nav/share/robot_nav/config/distortion_corrector_g1.yaml"
    dlio_params:="${WS_ROOT}/install/robot_lio/share/robot_lio/config/dlio_params_g1_bag_live.yaml"
  )
elif grep -q "name: /livox/lidar_filtered$" "$BAG_DIR/metadata.yaml"; then
  PROFILE="legacy-inverted"
  PLAY_TOPICS=(/livox/lidar_filtered /livox/imu)
else
  echo "Bag has neither /livox/lidar (+/imu/data) nor /livox/lidar_filtered; cannot replay." >&2
  exit 1
fi

for t in "${PLAY_TOPICS[@]}"; do
  if ! grep -q "name: ${t}$" "$BAG_DIR/metadata.yaml"; then
    echo "Bag is missing required topic ${t}; the G1 replay needs the Livox cloud + IMU." >&2
    exit 1
  fi
done

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  source "${WS_ROOT}/install/setup.bash"
else
  echo "Workspace not built: ${WS_ROOT}/install/setup.bash missing." >&2
  echo "Run: colcon build --symlink-install --packages-up-to robot_nav robot_lio g1_bringup" >&2
  exit 1
fi
set -u

cat <<EOF
--- G1 replay plan ---
  bag         : ${BAG_DIR}
  profile     : ${PROFILE}
  played      : ${PLAY_TOPICS[*]}
  dropped     : everything else (incl. /tf AND /tf_static -- see header)
  rate        : ${RATE}  loop: ${LOOP}  start_offset: ${START_OFFSET:-0}
  rviz        : ${USE_RVIZ}
  save dir    : ${SAVE_MAP_DIR:-<none>}
  extra args  : ${EXTRA_LAUNCH_ARGS[*]:-<none>}
EOF

LAUNCH_PID=""
BAG_PID=""
RECORD_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ -n "$RECORD_PID" ]] && kill -0 "$RECORD_PID" 2>/dev/null; then
    echo "Stopping replay-output recorder (pid=$RECORD_PID)"
    kill -INT "-$RECORD_PID" 2>/dev/null || kill -INT "$RECORD_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$RECORD_PID" 2>/dev/null || break
      sleep 1
    done
    kill -9 "$RECORD_PID" 2>/dev/null || true
  fi

  if [[ -n "$SAVE_MAP_DIR" ]]; then
    echo "Saving maps to: $SAVE_MAP_DIR"
    mkdir -p "$SAVE_MAP_DIR"
    if [[ -x "${WS_ROOT}/scripts/save_current_maps.sh" ]]; then
      # The replay launch runs its own dlio_map_node (lio.launch.py default),
      # so /save_pcd is live and the DLIO 3D map is saved alongside the
      # slam_toolbox occupancy map for coherence comparison.
      "${WS_ROOT}/scripts/save_current_maps.sh" \
        --output-dir "$SAVE_MAP_DIR" \
        --no-autostart-dlio-map || true
    else
      echo "save_current_maps.sh not found or not executable; skipping." >&2
    fi
  fi

  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "-$BAG_PID" 2>/dev/null || kill -INT "$BAG_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5; do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 1; done
    kill -9 "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8; do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -9 "$LAUNCH_PID" 2>/dev/null || true
  fi
  exit "$exit_code"
}
trap cleanup EXIT INT TERM

LAUNCH_LOG="/tmp/replay_bag_perception_g1_launch.log"
echo "Launching G1 tuning stack (logs: $LAUNCH_LOG)"
setsid ros2 launch robot_nav bag_perception_g1.launch.py \
  use_sim_time:=true \
  use_rviz:="$USE_RVIZ" \
  "${PROFILE_LAUNCH_ARGS[@]:+${PROFILE_LAUNCH_ARGS[@]}}" \
  "${EXTRA_LAUNCH_ARGS[@]:+${EXTRA_LAUNCH_ARGS[@]}}" \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "Launch pid=$LAUNCH_PID"

echo "Waiting ${LAUNCH_DELAY_SEC}s for nodes to come up..."
sleep "$LAUNCH_DELAY_SEC"

if [[ -n "$SAVE_MAP_DIR" ]]; then
  mkdir -p "$SAVE_MAP_DIR"
  RECORD_BAG_DIR="${SAVE_MAP_DIR}/replay_outputs"
  rm -rf "$RECORD_BAG_DIR"
  QOS_OVERRIDES="${WS_ROOT}/scripts/rosbag_qos_overrides.yaml"
  QOS_ARGS=()
  [[ -f "$QOS_OVERRIDES" ]] && QOS_ARGS=(--qos-profile-overrides-path "$QOS_OVERRIDES")
  echo "Recording replay outputs to: $RECORD_BAG_DIR"
  setsid ros2 bag record \
    --use-sim-time \
    --output "$RECORD_BAG_DIR" \
    "${QOS_ARGS[@]}" \
    /tf /tf_static /map /scan /lidar_odom \
    /lidar/points /lidar/points_undistorted \
    /local_costmap/costmap /global_costmap/costmap \
    > /tmp/replay_bag_perception_g1_record.log 2>&1 &
  RECORD_PID=$!
  echo "Replay-output recorder pid=$RECORD_PID"
fi

# --topics whitelist is the whole reason this script works: the bag's /tf
# (Unitree map->base_link + joint tree) and /tf_static (identity base_link ->
# base_footprint) must never reach the fresh replay graph.
PLAY_ARGS=(
  "$BAG_DIR"
  --clock
  --rate "$RATE"
  --topics "${PLAY_TOPICS[@]}"
)
[[ "$LOOP" == "true" ]] && PLAY_ARGS+=(--loop)
[[ -n "$START_OFFSET" ]] && PLAY_ARGS+=(--start-offset "$START_OFFSET")

echo "Playing: ros2 bag play ${PLAY_ARGS[*]}"
setsid ros2 bag play "${PLAY_ARGS[@]}" &
BAG_PID=$!

while kill -0 "$BAG_PID" 2>/dev/null && kill -0 "$LAUNCH_PID" 2>/dev/null; do
  sleep 1
done

if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "Launch exited unexpectedly. Tail of $LAUNCH_LOG:" >&2
  tail -n 40 "$LAUNCH_LOG" >&2 || true
fi
