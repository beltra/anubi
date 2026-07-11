#!/usr/bin/env bash
# Start the SONIC controller (g1_deploy_onnx_ref) on the JETSON with validated CPU
# pinning, so its CycloneDDS LowState receive threads can never be starved into a fall.
#
# WHERE THIS RUNS: on the Jetson Orin (the SONIC controller host), NOT the nav PC.
# In anubi's split topology the Jetson runs ONLY g1_deploy_onnx_ref; the whole nav
# stack (Nav2 + DLIO + EKF + safety + cmd_vel_to_sonic) runs on a separate nav PC.
# The controller must therefore connect OUT to the nav PC's ZMQ bridge, so
# SONIC_ZMQ_HOST=<nav-pc-ip> is REQUIRED (the sonic repo default `localhost` is wrong
# for the split -- see docs/g1_sonic_locomotion.md section 2.1 step F).
#
# WHY PINNING MATTERS (measured on the real G1):
#   The controller's four RT worker threads are pinned by SONIC_CPU_* -- but its
#   MAIN thread and the CycloneDDS receive threads (recvUC / recv / dq.* / rlsnr)
#   that actually deliver rt/lowstate at 500 Hz are NOT. If those float onto a busy
#   core they suffer 5-15 ms scheduling stalls (vs ~72 us on a clean core), miss
#   LowState frames, and trip: "Lost LowState data connection" -> safety-stop ->
#   the robot FALLS. This is not a cable fault (ping stays 0% loss); it is CPU
#   contention. The ONLY thing that confines EVERY controller thread (incl. those
#   DDS threads) to the RT cores is the outer `taskset -c ${SONIC_CORES}`. Starting
#   SONIC without it is the #1 fall cause. Even though the nav stack is off-board
#   here, JetPack housekeeping and the DDS pool still share the general cores --
#   keep the controller on its own cores and keep cores 0,1 for the OS.
#
# SAFETY: SONIC drives the motors. Robot on a hoist + hardware E-stop in hand for the
# first runs. Exactly one g1_deploy_onnx_ref may write rt/lowcmd (this script kills any
# stale one first); Unitree's stock high-level/sport service must be released, and
# amo_driver must NOT be running (single-writer rule, docs section 1).
#
# Usage (on the Jetson):
#   SONIC_ZMQ_HOST=192.168.123.224 scripts/start_sonic_pinned.sh          # foreground
#   TMUX=1 SONIC_ZMQ_HOST=192.168.123.224 scripts/start_sonic_pinned.sh   # detached tmux 'rdeploy'
# Env overrides:
#   SONIC_ZMQ_HOST (REQUIRED)         nav-PC IP the controller connects to (:5556)
#   SONIC_DIR   (default ~/sonic-g1-locomanipulation)   sonic repo (holds the deploy script)
#   GR00T_DIR   (default ~/GR00T-WholeBodyControl)       GR00T checkout the deploy needs
#   SONIC_CORES (default 2-5)   SONIC_DEPLOY (default scripts/start_deploy_real.sh)
#   SONIC_LOG   (default /tmp/sonic_deploy_real.log)
set -uo pipefail

SONIC_DIR="${SONIC_DIR:-$HOME/sonic-g1-locomanipulation}"
GR00T_DIR="${GR00T_DIR:-$HOME/GR00T-WholeBodyControl}"
SONIC_CORES="${SONIC_CORES:-2-5}"
SONIC_DEPLOY="${SONIC_DEPLOY:-scripts/start_deploy_real.sh}"
LOG="${SONIC_LOG:-/tmp/sonic_deploy_real.log}"

if [[ -z "${SONIC_ZMQ_HOST:-}" ]]; then
  echo "!! SONIC_ZMQ_HOST is required: the nav-PC IP the controller connects to on :5556." >&2
  echo "   e.g. SONIC_ZMQ_HOST=192.168.123.224 $0" >&2
  exit 1
fi
[[ -d "$SONIC_DIR" ]] || { echo "!! SONIC_DIR not found: $SONIC_DIR" >&2; exit 1; }
cd "$SONIC_DIR"
[[ -f "$SONIC_DEPLOY" ]] || { echo "!! deploy script not found: $SONIC_DIR/$SONIC_DEPLOY" >&2; exit 1; }

# Per-boot prep: free the unified memory TensorRT needs; ensure exactly one controller.
sudo -n bash -c "sync; echo 3 > /proc/sys/vm/drop_caches" 2>/dev/null || \
  echo ">> note: could not drop_caches (needs sudo) -- continuing" >&2
pkill -9 -f "[g]1_deploy_onnx_ref" 2>/dev/null || true

# GR00T_DIR and SONIC_ZMQ_HOST MUST be passed INTO the launched command: a detached
# tmux/non-interactive shell does not inherit ad-hoc exports or ~/.bashrc, so the
# deploy script would fall back to its wrong defaults (GR00T path, --zmq-host localhost).
# SONIC_CPU_* map each RT worker to a core; taskset -c ${SONIC_CORES} confines ALL
# threads -- including the unpinned DDS LowState receive threads -- to the RT cores.
CMD="GR00T_DIR=${GR00T_DIR} SONIC_ZMQ_HOST=${SONIC_ZMQ_HOST} \
SONIC_CPU_MAIN=3 SONIC_CPU_INPUT=3 SONIC_CPU_PLANNER=3 SONIC_CPU_WRITER=4 SONIC_CPU_CONTROL=5 \
taskset -c ${SONIC_CORES} ${SONIC_DEPLOY}"

echo ">> starting SONIC pinned to cores ${SONIC_CORES} (connecting to nav PC ${SONIC_ZMQ_HOST}:5556); log -> ${LOG}"
if [[ "${TMUX:-0}" == "1" ]]; then
  tmux kill-session -t rdeploy 2>/dev/null || true
  tmux new-session -d -s rdeploy "$CMD > ${LOG} 2>&1"
  echo ">> detached tmux 'rdeploy'. Waiting for Init Done ..."
  for _ in $(seq 1 25); do grep -qE "Init Done|out of memory|LowState is not available" "$LOG" 2>/dev/null && break; sleep 2; done
else
  # Foreground: run in background just long enough to verify pinning, then re-attach output.
  bash -c "$CMD" > "$LOG" 2>&1 &
  BGPID=$!
  echo ">> pid $BGPID; waiting for Init Done ..."
  for _ in $(seq 1 25); do grep -qE "Init Done|out of memory|LowState is not available" "$LOG" 2>/dev/null && break; sleep 2; done
fi

# -- Verify the pinning actually took (fail loudly if a thread escaped to 0,1) --
PID=$(pgrep -f "[g]1_deploy_onnx_ref" | head -1)
if [[ -n "$PID" ]] && grep -q "Init Done" "$LOG" 2>/dev/null; then
  bad=0
  for t in /proc/$PID/task/*; do
    allowed=$(awk '/Cpus_allowed_list/{print $2}' "$t/status" 2>/dev/null)
    case "$allowed" in ""|2|3|4|5|2-5|2,3,4,5|3-5|3,4,5|4,5|3,4) : ;; *) echo "!! thread $(cat "$t/comm" 2>/dev/null) allowed on $allowed (expected within ${SONIC_CORES})"; bad=1 ;; esac
  done
  if [[ $bad -eq 0 ]]; then
    echo ">> OK: all controller threads (incl. LowState recvUC/dq) confined to cores ${SONIC_CORES}."
  else
    echo "!! WARNING: some controller threads are NOT confined to ${SONIC_CORES} -- LowState fall risk!" >&2
  fi
  grep -E "\[RT\]" "$LOG" 2>/dev/null | tail -4
else
  echo "!! SONIC did not reach 'Init Done' -- check ${LOG}:" >&2; tail -8 "$LOG" >&2
  exit 1
fi

# Foreground mode: stream the controller log so this terminal owns its lifetime.
[[ "${TMUX:-0}" == "1" ]] || { echo ">> streaming ${LOG} (Ctrl-C stops the controller) ..."; wait ${BGPID:-} 2>/dev/null || tail -f "$LOG"; }
