#!/usr/bin/env bash
# Replay a Go2 rosbag through the minimal tuning stack.
#
# Purpose
# -------
# Tune slam_toolbox and Nav2/STVL against recorded real-robot data with
# zero double-publisher noise. The bag is played with a strict whitelist
# (raw sensors + /tf_static); every dynamic TF and every perception output
# is regenerated fresh inside bag_perception.launch.py.
#
# The whitelist policy is the whole point of this script: the bag also
# contains /scan, /map, /tf, /odom (EKF output) and /lidar/points_*
# derivatives from the live run. Replaying any of them creates a competing
# publisher and makes the live stack's map and costmap jump around.
#
# Topics actually played from the bag:
#   /lidar/points   (raw lidar PointCloud2)
#   /lidar/imu      (lidar-IMU; some bags may not have this)
#   /imu/data       (body IMU)
#   /odom_raw       (preferred) OR /odom (fallback, from go2_hw_bridge or EKF)
#   /tf_static      (URDF + driver static frames)
#
# Everything else in the bag is silently dropped.
#
# Usage:
#   replay_bag_perception.sh <bag_path> [options]
#
# Options:
#   --rate <hz>           Playback rate (default: 1.0).
#   --loop                Loop the bag.
#   --start-offset <s>    Start playback offset in seconds.
#   --no-rviz             Do not start RViz.
#   --save-map <dir>      On exit, save Nav2 occupancy map +
#                         slam_toolbox posegraph + replay odometry bag.
#   --odom-topic <t>      Override the odom topic name. Default: /odom_raw
#                         if the bag has it, /odom otherwise.
#   --nav-profile <n>     Nav2 profile (default: default_safe).
#   -h, --help            Show this help.

set -euo pipefail

BAG_INPUT=""
RATE="1.0"
LOOP="false"
START_OFFSET=""
USE_RVIZ="true"
SAVE_MAP_DIR=""
ODOM_TOPIC=""
NAV_PROFILE="default_safe"
LAUNCH_DELAY_SEC="6"

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
    --odom-topic)   ODOM_TOPIC="$2"; shift 2 ;;
    --nav-profile)  NAV_PROFILE="$2"; shift 2 ;;
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

[[ -n "$BAG_INPUT" ]] || { echo "Missing bag path." >&2; exit 1; }
[[ -e "$BAG_INPUT" ]] || { echo "Bag path not found: $BAG_INPUT" >&2; exit 1; }
if [[ -d "$BAG_INPUT" && -f "$BAG_INPUT/metadata.yaml" ]]; then
  BAG_DIR="$BAG_INPUT"
else
  echo "Expected a ROS2 bag directory (with metadata.yaml). Got: $BAG_INPUT" >&2
  exit 1
fi

# Resolve odom topic from the bag's metadata if the user did not specify one.
if [[ -z "$ODOM_TOPIC" ]]; then
  if grep -q "name: /odom_raw$" "$BAG_DIR/metadata.yaml"; then
    ODOM_TOPIC="/odom_raw"
  elif grep -q "name: /odom$" "$BAG_DIR/metadata.yaml"; then
    ODOM_TOPIC="/odom"
  else
    echo "Bag carries neither /odom_raw nor /odom; tuning replay needs body odometry." >&2
    exit 1
  fi
fi

# Whitelist of bag topics that the live stack should actually receive.
# Anything else in the bag stays in the file and is silently ignored at
# playback because ros2 bag play --topics only emits the listed ones.
PLAY_TOPICS=(
  /lidar/points
  /lidar/imu
  /imu/data
  /tf_static
  "$ODOM_TOPIC"
)

# Drop topics not actually in the bag (e.g. /lidar/imu on some recordings)
# so ros2 bag play does not error out.
FILTERED_TOPICS=()
for t in "${PLAY_TOPICS[@]}"; do
  if grep -q "name: ${t}$" "$BAG_DIR/metadata.yaml"; then
    FILTERED_TOPICS+=("$t")
  else
    echo "Note: ${t} not in bag, skipping."
  fi
done
PLAY_TOPICS=("${FILTERED_TOPICS[@]}")

set +u
source /opt/ros/humble/setup.bash
if [[ -f /ws/anubi/install/setup.bash ]]; then
  source /ws/anubi/install/setup.bash
else
  echo "Workspace not built: /ws/anubi/install/setup.bash missing." >&2
  echo "Run: colcon build --symlink-install --packages-up-to robot_nav" >&2
  exit 1
fi
set -u

cat <<EOF
--- replay plan ---
  bag         : ${BAG_DIR}
  odom topic  : ${ODOM_TOPIC}
  played      : ${PLAY_TOPICS[*]}
  dropped     : everything else (incl. /tf, /map, /scan, /lidar/points_*)
  rate        : ${RATE}  loop: ${LOOP}  start_offset: ${START_OFFSET:-0}
  rviz        : ${USE_RVIZ}  nav profile: ${NAV_PROFILE}
  save dir    : ${SAVE_MAP_DIR:-<none>}
EOF

LAUNCH_PID=""
BAG_PID=""
ODOM_BAG_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ -n "$ODOM_BAG_PID" ]] && kill -0 "$ODOM_BAG_PID" 2>/dev/null; then
    echo "Stopping odometry recorder (pid=$ODOM_BAG_PID)"
    kill -INT "-$ODOM_BAG_PID" 2>/dev/null || kill -INT "$ODOM_BAG_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$ODOM_BAG_PID" 2>/dev/null || break
      sleep 1
    done
    kill -9 "$ODOM_BAG_PID" 2>/dev/null || true
  fi

  if [[ -n "$SAVE_MAP_DIR" ]]; then
    echo "Saving maps to: $SAVE_MAP_DIR"
    mkdir -p "$SAVE_MAP_DIR"
    if [[ -x /ws/anubi/scripts/save_current_maps.sh ]]; then
      /ws/anubi/scripts/save_current_maps.sh \
        --output-dir "$SAVE_MAP_DIR" \
        --skip-dlio || true
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

LAUNCH_LOG="/tmp/replay_bag_perception_launch.log"
echo "Launching tuning stack (logs: $LAUNCH_LOG)"
setsid ros2 launch robot_nav bag_perception.launch.py \
  use_sim_time:=true \
  odom_topic:="$ODOM_TOPIC" \
  nav_profile:="$NAV_PROFILE" \
  use_rviz:="$USE_RVIZ" \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "Launch pid=$LAUNCH_PID"

echo "Waiting ${LAUNCH_DELAY_SEC}s for nodes to come up..."
sleep "$LAUNCH_DELAY_SEC"

# Start an odometry-only recorder if the user asked to save maps.
if [[ -n "$SAVE_MAP_DIR" ]]; then
  mkdir -p "$SAVE_MAP_DIR"
  ODOM_BAG_DIR="${SAVE_MAP_DIR}/replay_odometry"
  rm -rf "$ODOM_BAG_DIR"
  QOS_OVERRIDES="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rosbag_qos_overrides.yaml"
  QOS_ARGS=()
  [[ -f "$QOS_OVERRIDES" ]] && QOS_ARGS=(--qos-profile-overrides-path "$QOS_OVERRIDES")
  echo "Recording replay odometry to: $ODOM_BAG_DIR"
  setsid ros2 bag record \
    --use-sim-time \
    --output "$ODOM_BAG_DIR" \
    "${QOS_ARGS[@]}" \
    /tf /tf_static /map /odom /scan \
    > /tmp/replay_bag_perception_odom_record.log 2>&1 &
  ODOM_BAG_PID=$!
  echo "Odometry recorder pid=$ODOM_BAG_PID"
fi

# Build playback args. `--topics` whitelist is the whole reason this script
# works: without it, the bag's /tf, /map, /scan, /odom (EKF), and the
# /lidar/points_* derivatives would all reach the live graph and compete
# with the fresh nodes spawned by bag_perception.launch.py.
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
