#!/usr/bin/env bash
# Launch the Unitree G1 REAL-ROBOT autonomy stack (hardware bring-up).
# Humanoid analogue of run_go2_real.sh. Wraps g1_bringup.launch.py, which mirrors
# go2_bringup: g1_state_bridge (DDS body state -> /odom_raw + /imu/data + body TF)
#   + EKF (odom -> base_footprint) + Livox Mid-360 driver + imu_rescale + robot_lio
#   (canonical relays + DLIO) + slam (distortion_corrector self-filter + p2l + slam_toolbox)
#   + Nav2 (G1 STVL configs) + robot_safety velocity_limiter + the locomotion bridge
#   (cmd_vel_to_amo ws:8766, or cmd_vel_to_sonic ZMQ:5556 with --locomotion sonic).
#
# IMPORTANT: the low-level locomotion controller runs as its OWN process and must
# already be up; this stack only produces velocity commands for it:
#   --locomotion amo   (default) g1_locomotion amo_driver on this host (ws :8766)
#   --locomotion sonic SONIC g1_deploy_onnx_ref on the Jetson, connecting to this
#                      host's ZMQ :5556 (see docs/g1.md)
# NEVER run both controllers at once (two writers on rt/lowcmd is dangerous).
# Keep the robot STILL for the first few seconds so DLIO completes its IMU/gravity
# calibration (see --nav-delay).
#
# BUILD NOTE: the vendored livox_ros_driver2 gitignores its package.xml, so colcon
# cannot discover it until you run `src/vendors/livox_ros_driver2/build.sh humble`
# ONCE before building the workspace (see g1_bringup/README.md "Build").
set -euo pipefail

MODE="mapping"
MAP_PATH=""
USE_LIVOX="true"
USE_LIO="true"
USE_EKF="true"
USE_STATE_BRIDGE="true"
USE_NAV="true"
USE_STVL="true"
USE_SAFETY="true"
RVIZ="true"
LIVOX_CONFIG=""
STANDING_HEIGHT="0.78"
NAV_DELAY_SEC="5.0"
DISTORTION_IMU_TOPIC="/imu/data"
NET_IF=""
# Controller key masks (same unitree_go/WirelessController layout as the Go2, so
# match run_go2_real.sh). Leaving these at 0 disables controller acknowledgement:
# a latched tilt E-STOP could then never be cleared and Nav2 resume/abort would be
# inert. Override per-controller if the button map differs.
RECOVERY_ACK_KEY_MASK="0x0010"
RESUME_KEY_MASK="0x0010"
ABORT_KEY_MASK="0x0020"
CLEAN_EXISTING="true"
LOCOMOTION="amo"
SONIC_PROFILE="slow_walk"
SONIC_ARM_PRESET="none"

usage() {
  cat <<'EOF'
Usage:
  run_g1_real.sh [mapping|localization] [options]

Hardware bring-up for the Unitree G1. Start the low-level locomotion controller
SEPARATELY first; this stack only commands it:
  amo (default): g1_locomotion amo_driver on this host (WebSocket :8766)
  sonic:         SONIC g1_deploy_onnx_ref on the Jetson (connects to this host's
                 cmd_vel_to_sonic ZMQ PUB :5556); see docs/g1.md
NEVER run both controllers at once (rt/lowcmd single-writer rule).
Keep the robot STILL during the initial --nav-delay so DLIO can calibrate.

Options:
  --map <path>           slam_toolbox .posegraph map (required in localization mode)
  --locomotion <name>    Low-level controller: amo (default) or sonic
  --sonic-profile <name> SONIC velocity envelope: slow_walk (default; vx 0.5, wz 0.4)
                         or walk (vx 0.8, wz 0.5; only after slow_walk is validated)
  --sonic-arm-preset <name>
                         SONIC fixed upper-body pose held while walking: none
                         (default; arms swing with the gait), default (nominal
                         standing arms), or carry (two-handed carry). Loco-
                         manipulation; validate hoisted and tune per payload.
  --no-livox             Do NOT start the Livox Mid-360 driver + imu_rescale
                         (e.g. when replaying a rosbag of /livox/lidar + /livox/imu)
  --no-lio               Do NOT run DLIO (odometry then relies on the AMO body odom only)
  --no-ekf               Do NOT run the EKF (advanced; leaves odom->base_footprint unowned)
  --no-state-bridge      Do NOT run the DDS state bridge (use a static base_footprint->base_link;
                         e.g. rosbag replay or bench tests without the robot)
  --no-nav               Do NOT start Nav2 + collision_monitor + the locomotion bridge
  --no-stvl              Disable SpatioTemporalVoxelLayer in the Nav2 local costmap
  --no-safety            Disable the velocity_limiter (the bridge reads /cmd_vel_collision_free)
  --no-rviz              Start without RViz
  --nav-delay <sec>      Delay before SLAM/Nav2 start (default 5.0; lets DLIO calibrate)
  --livox-config <path>  Livox MID360_config.json (host/lidar IPs + roll:180 mount)
  --standing-height <m>  G1 pelvis(base_link) height above floor (default 0.78; calibrate)
  --net-if <iface>       DDS NIC the G1 is reachable on (e.g. eth0)
  --distortion-imu-topic <topic>
                         IMU topic for cloud leveling + lidar_stabilized (default /imu/data)
  --recovery-ack-key-mask <int>  Controller key mask for tilt-estop recovery ack
  --resume-key-mask <int>        Controller key mask for Nav2 resume/replan
  --abort-key-mask <int>         Controller key mask for controlled Nav2 abort
  --no-clean             Do not kill pre-existing ROS processes before launch
  -h, --help             Show this help

Examples:
  run_g1_real.sh                                   # mapping, full stack, AMO
  run_g1_real.sh localization --map /ws/anubi/maps/office.posegraph
  run_g1_real.sh --no-nav                          # sensors + DLIO + EKF only, no commands
  run_g1_real.sh --net-if eth0
  run_g1_real.sh --locomotion sonic                # SONIC controller, slow_walk envelope
  run_g1_real.sh --locomotion sonic --sonic-profile walk
  run_g1_real.sh --locomotion sonic --sonic-arm-preset carry   # hold two-handed carry pose
EOF
}

if [[ "${1:-}" == "mapping" || "${1:-}" == "localization" ]]; then
  MODE="$1"; shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) [[ $# -ge 2 ]] || { echo "Missing value for --map" >&2; exit 1; }; MAP_PATH="$2"; shift 2;;
    --no-livox) USE_LIVOX="false"; shift;;
    --no-lio)   USE_LIO="false"; shift;;
    --no-ekf)   USE_EKF="false"; shift;;
    --no-state-bridge) USE_STATE_BRIDGE="false"; shift;;
    --no-nav)   USE_NAV="false"; shift;;
    --no-stvl)  USE_STVL="false"; shift;;
    --no-safety) USE_SAFETY="false"; shift;;
    --no-rviz)  RVIZ="false"; shift;;
    --nav-delay) [[ $# -ge 2 ]] || { echo "Missing value for --nav-delay" >&2; exit 1; }; NAV_DELAY_SEC="$2"; shift 2;;
    --livox-config) [[ $# -ge 2 ]] || { echo "Missing value for --livox-config" >&2; exit 1; }; LIVOX_CONFIG="$2"; shift 2;;
    --standing-height) [[ $# -ge 2 ]] || { echo "Missing value for --standing-height" >&2; exit 1; }; STANDING_HEIGHT="$2"; shift 2;;
    --net-if) [[ $# -ge 2 ]] || { echo "Missing value for --net-if" >&2; exit 1; }; NET_IF="$2"; shift 2;;
    --distortion-imu-topic) [[ $# -ge 2 ]] || { echo "Missing value" >&2; exit 1; }; DISTORTION_IMU_TOPIC="$2"; shift 2;;
    --recovery-ack-key-mask) [[ $# -ge 2 ]] || { echo "Missing value" >&2; exit 1; }; RECOVERY_ACK_KEY_MASK="$2"; shift 2;;
    --resume-key-mask) [[ $# -ge 2 ]] || { echo "Missing value" >&2; exit 1; }; RESUME_KEY_MASK="$2"; shift 2;;
    --abort-key-mask) [[ $# -ge 2 ]] || { echo "Missing value" >&2; exit 1; }; ABORT_KEY_MASK="$2"; shift 2;;
    --no-clean) CLEAN_EXISTING="false"; shift;;
    --locomotion) [[ $# -ge 2 ]] || { echo "Missing value for --locomotion" >&2; exit 1; }; LOCOMOTION="$2"; shift 2;;
    --sonic-profile) [[ $# -ge 2 ]] || { echo "Missing value for --sonic-profile" >&2; exit 1; }; SONIC_PROFILE="$2"; shift 2;;
    --sonic-arm-preset) [[ $# -ge 2 ]] || { echo "Missing value for --sonic-arm-preset" >&2; exit 1; }; SONIC_ARM_PRESET="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1;;
  esac
done

if [[ "$LOCOMOTION" != "amo" && "$LOCOMOTION" != "sonic" ]]; then
  echo "Invalid --locomotion '$LOCOMOTION' (expected amo or sonic)." >&2; exit 1
fi
if [[ "$SONIC_PROFILE" != "slow_walk" && "$SONIC_PROFILE" != "walk" ]]; then
  echo "Invalid --sonic-profile '$SONIC_PROFILE' (expected slow_walk or walk)." >&2; exit 1
fi
if [[ "$SONIC_ARM_PRESET" != "none" && "$SONIC_ARM_PRESET" != "default" && "$SONIC_ARM_PRESET" != "carry" ]]; then
  echo "Invalid --sonic-arm-preset '$SONIC_ARM_PRESET' (expected none, default, or carry)." >&2; exit 1
fi

if [[ "$MODE" == "localization" && -z "$MAP_PATH" ]]; then
  echo "In localization mode you must provide --map <path>." >&2; exit 1
fi

if [[ "$CLEAN_EXISTING" == "true" ]]; then
  echo "Cleaning stale ROS processes before launch..."
  bash "$(cd "$(dirname "$0")" && pwd)/kill_ros_nodes.sh" || true
fi

# ROS setup scripts are not fully nounset-safe; source them with +u.
set +u
source /opt/ros/humble/setup.bash
source /ws/anubi/install/setup.bash
set -u

CMD=(
  ros2 launch g1_bringup g1_bringup.launch.py
  use_sim_time:=false
  slam_mode:="$MODE"
  use_livox:="$USE_LIVOX"
  use_lio:="$USE_LIO"
  use_ekf_fusion:="$USE_EKF"
  use_state_bridge:="$USE_STATE_BRIDGE"
  use_nav:="$USE_NAV"
  use_stvl:="$USE_STVL"
  use_safety:="$USE_SAFETY"
  rviz:="$RVIZ"
  nav_delay_sec:="$NAV_DELAY_SEC"
  standing_height:="$STANDING_HEIGHT"
  distortion_imu_topic:="$DISTORTION_IMU_TOPIC"
  recovery_ack_key_mask:="$RECOVERY_ACK_KEY_MASK"
  resume_key_mask:="$RESUME_KEY_MASK"
  abort_key_mask:="$ABORT_KEY_MASK"
  locomotion:="$LOCOMOTION"
  sonic_profile:="$SONIC_PROFILE"
  sonic_arm_preset:="$SONIC_ARM_PRESET"
)
[[ -n "$MAP_PATH"     ]] && CMD+=(map:="$MAP_PATH")
[[ -n "$LIVOX_CONFIG" ]] && CMD+=(livox_config:="$LIVOX_CONFIG")
[[ -n "$NET_IF"       ]] && CMD+=(net_if:="$NET_IF")

echo "Starting G1 real-robot stack..."
if [[ "$LOCOMOTION" == "sonic" ]]; then
  echo "NOTE: ensure the SONIC controller (g1_deploy_onnx_ref on the Jetson) is running and"
  echo "      pointed at this host's ZMQ :5556, and the AMO driver is NOT running."
  echo "      SONIC profile: ${SONIC_PROFILE}, arm preset: ${SONIC_ARM_PRESET}. See docs/g1.md."
  [[ "$SONIC_ARM_PRESET" != "none" ]] && echo "      NOTE: arm preset '${SONIC_ARM_PRESET}' pins the upper body (loco-manipulation); validate hoisted, tune per payload."
else
  echo "NOTE: ensure the AMO locomotion driver (g1_locomotion amo_driver, ws://127.0.0.1:8766) is already running."
fi
echo "NOTE: keep the robot STILL for the first ${NAV_DELAY_SEC}s so DLIO can calibrate."
printf 'Command:'; printf ' %q' "${CMD[@]}"; echo
exec "${CMD[@]}"
