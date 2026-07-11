# Handoff: SONIC controller intermittently aborts at startup with "Lost LowState data connection"

Status: **root cause CONFIRMED. Fix was applied and verified on 2026-07-03, then
REVERTED the same day**: the operator reported the policy became "very aggressive
and jittery" during the post-fix gantry session, and asked for a full revert
pending investigation. See section 0 for what was tried, measured, and reverted.
Audience: a future session that revisits this startup abort.

## 0. Fix attempt record (2026-07-03, applied then reverted)

- The startup-grace fix from section 7 was applied (constant
  `STARTUP_SAFETY_GRACE{5000}` ms, bounded retry in `WAIT_FOR_CONTROL`),
  rebuilt via the sonic repo `scripts/build.sh`, and verified:
  **pre-fix baseline 3 aborts / 5 launches; post-fix 10 / 10 stable**, with the
  grace window absorbing the race in 6 of 10 launches (recovery ~15 stale ticks,
  never near the 5 s bound). The in-CONTROL 500 ms check was left strict.
- During the subsequent gantry run the operator reported the walking policy was
  "very aggressive and jittery" and requested a revert. Everything from that
  session was reverted on the Jetson: source restored (grace fix removed),
  binary rebuilt from the restored source, the regenerated
  `gr00t-sonic-deploy.patch` restored from backup, and the sonic repo script
  fixes (`start_deploy_real.sh` `SONIC_ZMQ_HOST` override, `env.sh` GR00T_DIR
  default, `build.sh` nounset guard) checked out to their committed state.
- Forensic copies kept on the Jetson in `/tmp`:
  `g1_deploy_onnx_ref.cpp.with_grace_fix`, `g1_deploy_onnx_ref.bin.with_grace_fix`,
  `gr00t-sonic-deploy.patch` (with-fix regen) alongside the `.bak`, plus the
  trial logs `sonic_base_*.log` / `sonic_postfix_*.log`.
- Jitter causality was NOT established. Notes for the next investigation:
  - The grace change only executes in the startup failure branch; the in-CONTROL
    path is byte-identical code. All other modified GR00T sources predate the
    original Jul 2 binary, so the rebuild introduced no foreign changes.
  - Candidate explanations: (a) the newly working cross-host ZMQ command path
    (`SONIC_ZMQ_HOST` -> nav PC) delivered real Nav2 cmd_vel to the robot for
    the first time -- command-stream/tuning jitter would look exactly like this; (b) a
    selection effect: pre-fix, launches on a starvation-prone system died at
    startup; post-fix they proceed to CONTROL where episodic sub-500 ms
    LowState lag degrades the policy (watch `Loop timing - LowState age` for
    spikes); (c) unrelated operator-side changes (other sessions were active on
    the Jetson concurrently).
- Keyboard note: with `--input-type zmq_manager` the `]` start key is NOT
  honoured; start/stop arrive as ZMQ `command` messages (`{start, stop,
  planner}`) on port 5556 (see `cmd_vel_to_sonic_node.py`).

---

## 1. TL;DR

On the Jetson Orin Nano, launching the SONIC low-level controller
(`g1_deploy_onnx_ref`, started via `~/groot/sim_run/start_deploy_real.sh`)
**intermittently** aborts right after init:

```
Init Done
[ERROR] Lost LowState data connection from robot!
[ERROR] Safety check failed, cannot start control.
[DEBUG] Stopping G1Deploy...
```

It is a **scheduling race**, not a network/robot fault. The startup safety
gate demands a LowState sample fresher than **500 ms**, but the timestamp is
stamped by an **unpinned, default-priority DDS callback thread** that gets
starved by ~13 s of TensorRT/CUDA initialisation. When the first
`WAIT_FOR_CONTROL` safety check fires (immediately after `Init Done`), the last
callback is sometimes >500 ms old, so it hard-aborts. Robot packets keep
arriving at the NIC the whole time (~6,600 pkts/s) — the wire and robot are
healthy; only the app-side callback thread is late.

**Recommended fix:** give the *startup* safety gate (`WAIT_FOR_CONTROL`) a
bounded grace (retry ~3–5 s) instead of hard-aborting on the first failure.
Leave the in-`CONTROL` 500 ms check strict. Small C++ change + rebuild.

---

## 2. Symptom / how to reproduce

- Failure is **intermittent**: some launches pass and enter control fine, some
  abort as above. Same everything (engines cached, DDS pinned, sport service
  released).
- Occurs **only at the INIT -> WAIT_FOR_CONTROL transition** (right after
  `Init Done`). Once the robot is actually running, it stays healthy — the
  in-`CONTROL` safety check (same 500 ms) does not trip in steady state.
- Reproduce by launching several times (see safe procedure in §6). Expect a
  mix of PASS and FAIL. To get a baseline failure rate, launch ~10 times.

---

## 3. Root cause (confirmed, with source citations)

Source file on the Jetson:
`/home/dev/groot/GR00T-WholeBodyControl/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp`

The chain:

1. **Threshold.** `static constexpr std::chrono::milliseconds LOW_STATE_ABSENT_THRESHOLD{500};` (line ~298).

2. **The gate.** `CheckSafety()` (line ~2887):
   ```cpp
   auto now = std::chrono::steady_clock::now();
   if (now - low_state_data.timestamp > LOW_STATE_ABSENT_THRESHOLD) {
     std::cout << "[ERROR] Lost LowState data connection from robot!" << std::endl;
     return false;
   }
   ```
   Note: it checks *when the app last received a LowState callback*, NOT whether
   packets are arriving at the NIC.

3. **Who stamps the timestamp.** `low_state_data.timestamp` is set inside
   `LowStateHandler()` (line ~2676) when it calls `low_state_buffer_.SetData(...)`.
   `LowStateHandler` is a **DDS subscriber callback**, registered at line ~2322:
   ```cpp
   lowstate_subscriber_->InitChannel(std::bind(&G1Deploy::LowStateHandler, this, _1), 1);
   ```
   This callback runs on the **Unitree SDK's own DDS dispatch thread**, which is
   **not** one of the pinned RT threads. The pinned RT threads are
   `Input` (CPU 2, FIFO 60 — runs `G1Deploy::Input`, the ZMQ input loop, line ~2640/~3539),
   `control` (CPU 5, FIFO 80), `command_writer` (CPU 4, FIFO 90),
   `planner` (CPU 3, FIFO 40). The DDS lowstate thread is default-scheduled and
   floats on the non-RT cores (0, 1) alongside the TensorRT/CUDA init worker
   threads.
   > VERIFY (minor): confirm the exact thread by reading the SDK's
   > `channel_subscriber.hpp` under
   > `gear_sonic_deploy/thirdparty/unitree_sdk2/`. The `InitChannel(handler, N)`
   > pattern spawns an SDK-managed subscriber/dispatch thread; it is clearly not
   > the pinned `Input` RT thread (that thread runs `G1Deploy::Input`).

4. **The transition has no freshness gate.** `InitControl()` (line ~2852) only
   requires that *some* LowState exists (`if (!ls) return false;`), runs a blend
   to the default pose over `duration_`, prints `Init Done`, and sets
   `program_state_ = ProgramState::WAIT_FOR_CONTROL`.

5. **First WAIT_FOR_CONTROL tick hard-aborts.** State machine in `Control()`
   (around line ~3951):
   ```cpp
   case ProgramState::WAIT_FOR_CONTROL:
     if (!CheckSafety()) {
       std::cout << "[ERROR] Safety check failed, cannot start control." << std::endl;
       operator_state.stop = true;   // <-- immediate hard abort on first failure
       break;
     }
   ```

**Net:** during the ~13 s of engine init the unpinned DDS thread is starved; if
its last callback landed >500 ms before that first `WAIT_FOR_CONTROL` tick, the
gate aborts. Pure timing race -> intermittent.

The **in-CONTROL** check is a separate `CheckSafety()` call (line ~3978,
"[ERROR] Safety check failed, stopping control.") — that one must stay strict.

---

## 4. Evidence collected (this session)

- **Wire is healthy across the failure window.** With no `tcpdump`/`unitree_sdk2py`
  available, sampled `/sys/class/net/enP8p1s0/statistics/rx_packets` at 1 Hz
  during launches: steady **~6,600 pkts/s** straight through `Init Done`. Brief
  dips to ~4,600–5,200 only coincide with deploy teardown/relaunch between
  attempts (DDS participant leaving/rejoining), not with a safety-gate failure.
  => rules out robot-side stop, cable/link intermittency, and DDS-interface
  misbinding.
- **DDS interface is correctly pinned.** The binary takes the NIC as its first
  positional arg, defaulting to `enP8p1s0` (`"${SONIC_DDS_IFACE:-enP8p1s0}"`),
  which is the robot-LAN NIC. Not a wifi-discovery problem.
- **Intermittent.** Clean tmux launches this session reached `Init Done` and
  PASSED safety (went to control); the user's launches sometimes FAIL. Engines
  are cached (repeat runs), so it is not first-build starvation specifically —
  cached-engine deserialise + CUDA context init still costs ~13 s.
- Log timeline of one launch: planner/engine work spans ~t+0 to ~t+13 s
  ("Initialize Engine..." -> "TensorRT planner model loaded"), RT threads pin at
  ~t+13 s, `G1Deploy object created`, then `Init Done` ~2.8 s later.

Ruled out earlier (via user Q&A + inspection): first-build starvation only,
wifi DDS discovery, stock sport service still running — all not the cause.

---

## 5. Environment & access (how to reach everything)

- **Jetson SSH:** `dev@192.168.123.99` (robot-LAN NIC) — also reachable at
  `dev@10.251.101.176` (wifi). **Password: `dev`.** `sudo` works **with**
  password `dev` (NOT passwordless). Non-interactive: use
  `sshpass -p dev ssh -o StrictHostKeyChecking=accept-new dev@192.168.123.99 ...`.
- **Interfaces on the Jetson:** `enP8p1s0` = 192.168.123.99 (robot LAN, this is
  the DDS/`SONIC_DDS_IFACE` NIC), `wlP1p1s0` = 10.251.101.176 (wifi / SSH).
  Robot is at **192.168.123.161**.
- **Not installed on the Jetson:** `tcpdump`, `unitree_sdk2py`. Use
  `/proc/net/dev` or `/sys/class/net/enP8p1s0/statistics/rx_packets` for a
  zero-install wire probe.
- **Launch script the user actually runs:** `/home/dev/groot/sim_run/start_deploy_real.sh`
  (CPU pins MAIN=0/INPUT=2/PLANNER=3/WRITER=4/CONTROL=5 match the log;
  `SONIC_MOTOR_COMMAND_STALE_MS=100`; supports `SONIC_ZMQ_HOST` override). It
  `cd`s to `~/groot/GR00T-WholeBodyControl/gear_sonic_deploy` and execs
  `target/release/g1_deploy_onnx_ref "${SONIC_DDS_IFACE:-enP8p1s0}" ...`.
  (There is also `/home/dev/groot/sonic-g1-locomotion/scripts/start_deploy_real.sh`,
  the sonic-repo canonical one, which hardcodes `--zmq-host localhost`.)
- **Deploy source tree:** `/home/dev/groot/GR00T-WholeBodyControl/gear_sonic_deploy/`
  (main file `src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp`; build via
  `scripts/build.sh` in that dir — confirm exact build invocation before running).
- **Durable fix location:** the sonic repo patch
  `/home/dev/groot/sonic-g1-locomotion/patches/gr00t-sonic-deploy.patch`
  (this is where anubi's `docs/g1_sonic_locomotion.md` says GR00T is patched).
  For a quick test, edit the source directly and rebuild; then fold the change
  into the patch.
- This workspace (`mintPC`, `/ws/anubi`) is **not** on the robot LAN; drive the
  Jetson over SSH. `~/.ssh` is permission-restricted here (can't read local ssh
  config), and the Bash tool sandbox may block `ping`/`ip` — don't infer
  reachability from those.

---

## 6. CRITICAL gotchas (do not relearn these the hard way)

1. **Never feed the deploy binary stdin from an SSH heredoc/pipe.** Its
   `InputInterface`/`ZMQManager` reads the keyboard from stdin; a piped/heredoc
   stdin is consumed as phantom keystrokes (`[ZMQManager] EMERGENCY STOP (O/o)`,
   `[Compliance] ...`, `Max close ratio adjusted` — the many `o`s in shell text
   trigger emergency stops). That makes the deploy exit for the WRONG reason and
   contaminates the run. **Always launch it in a detached `tmux` session** (real
   tty stdin), exactly as the user does:
   ```bash
   tmux new-session -d -s diag "bash ~/groot/sim_run/start_deploy_real.sh > /tmp/log 2>&1"
   ```
2. **Set the Bash-tool `timeout` explicitly.** It defaults to 120 s and will
   SIGTERM your SSH mid-run, orphaning tmux/deploy processes on the Jetson.
   After any long op, reconnect and clean up.
3. **Launching = energising the hoisted robot.** If safety PASSES, the deploy
   enters `WAIT_FOR_CONTROL` and PD-holds the default pose (motors active); if a
   ZMQ `start` arrives it walks. Keep diagnostic runs short (stop within ~3 s of
   `Init Done`). Always stop with `pkill -INT -f g1_deploy_onnx_ref; sleep 2;
   pkill -9 -f g1_deploy_onnx_ref` and confirm `pgrep -f g1_deploy_onnx_ref` is
   empty. Robot is on a gantry, supervised; the user has authorised launching.
4. **Enforce single-writer.** Never run `amo_driver` alongside `g1_deploy_onnx_ref`
   (`pgrep -f amo_driver` must be empty). Stock Unitree sport/high-level service
   must be released.

### Safe reproduction snippet (tty stdin, RX probe, auto-stop)

Run this as ONE remote script (via `sshpass ... ssh ... bash -l <<'EOF' ... EOF`),
with the Bash-tool `timeout` set to e.g. 180000 ms:

```bash
IF=enP8p1s0
cat > /tmp/run_deploy_ts.sh <<'INNER'
#!/bin/bash
bash "$HOME/groot/sim_run/start_deploy_real.sh" 2>&1 | while IFS= read -r l; do printf '%.3f %s\n' "$(date +%s.%N)" "$l"; done
INNER
chmod +x /tmp/run_deploy_ts.sh
( prev=$(cat /sys/class/net/$IF/statistics/rx_packets)
  for i in $(seq 1 120); do sleep 1; cur=$(cat /sys/class/net/$IF/statistics/rx_packets)
    printf '%.3f RX %s\n' "$(date +%s.%N)" "$((cur-prev))"; prev=$cur; done ) > /tmp/rx.log 2>&1 &
RXPID=$!
tmux new-session -d -s diag "bash /tmp/run_deploy_ts.sh > /tmp/sonic_clean.log 2>&1"
INIT_T=0
for i in $(seq 1 90); do
  sleep 1
  grep -qE 'Lost LowState|Safety check failed' /tmp/sonic_clean.log && { echo "FAIL at +${i}s"; sleep 1; break; }
  [ $INIT_T = 0 ] && grep -q 'Init Done' /tmp/sonic_clean.log && INIT_T=$i
  [ $INIT_T != 0 ] && [ $((i-INIT_T)) -ge 3 ] && { echo "PASS (stopping)"; break; }
done
pkill -INT -f g1_deploy_onnx_ref; sleep 2; tmux kill-session -t diag 2>/dev/null; pkill -9 -f g1_deploy_onnx_ref
kill $RXPID 2>/dev/null
pgrep -f g1_deploy_onnx_ref || echo confirmed_stopped
```

---

## 7. Recommended fix (do this)

Make the **startup** gate tolerant of the post-init transient; keep the
in-control gate strict.

Edit `g1_deploy_onnx_ref.cpp`, `WAIT_FOR_CONTROL` case (anchor on the string
`"[ERROR] Safety check failed, cannot start control."`). Add a bounded startup
grace so we only abort if the stream is dead for the whole window, not on the
first stale sample:

```cpp
case ProgramState::WAIT_FOR_CONTROL: {
  if (!CheckSafety()) {
    // The LowState freshness timestamp is stamped by an unpinned DDS callback
    // thread that can be starved for >500ms during TensorRT/CUDA init. Once init
    // is done the stream refreshes within ~2ms, so allow a bounded settle window
    // here instead of aborting on the first stale sample. The in-CONTROL
    // CheckSafety() stays strict.
    if (!wait_for_control_since_) wait_for_control_since_ = std::chrono::steady_clock::now();
    if (std::chrono::steady_clock::now() - *wait_for_control_since_ < STARTUP_SAFETY_GRACE) {
      break;  // keep PD-holding the ready pose; re-check next tick
    }
    std::cout << "[ERROR] Safety check failed, cannot start control." << std::endl;
    operator_state.stop = true;
    break;
  }
  wait_for_control_since_.reset();  // healthy again
  ...
```

Add the members/const near the other config:
```cpp
static constexpr std::chrono::milliseconds STARTUP_SAFETY_GRACE{5000};
std::optional<std::chrono::steady_clock::time_point> wait_for_control_since_;
```
(Optionally make the grace env-overridable, e.g. `SONIC_STARTUP_SAFETY_GRACE_MS`.)

Why it's safe: in `WAIT_FOR_CONTROL` the robot is only PD-holding the default
pose (not walking); extending that hold a few seconds while the DDS thread
settles is benign. Walking still requires `operator_state.start` AND a passing
`CheckSafety()`. Real disconnects during control still abort instantly (separate
strict check at line ~3978, unchanged).

**Fold the change into** `sonic-g1-locomotion/patches/gr00t-sonic-deploy.patch`
so it survives a clean GR00T checkout (per `docs/g1_sonic_locomotion.md` §2.1).

## 8. Alternatives / complements (optional)

- **Config-only, no rebuild (try first for a quick mitigation):** raise the
  CycloneDDS receive-thread scheduling via `CYCLONEDDS_URI` `<Threads>` so the
  DDS path is not starved during init. Lower confidence it fully covers the
  Unitree SDK dispatch thread — test empirically. (Unknown whether the C++ SDK
  honours `CYCLONEDDS_URI` thread config here; verify.)
- **Address the starvation at the source (root-est):** pin/raise priority of the
  SDK's DDS dispatch thread. Harder — the thread is created inside the SDK.
- **Reduce init contention:** `sudo sh -c "sync; echo 3 > /proc/sys/vm/drop_caches"`
  before launch; `jetson_clocks` (already invoked by the script). Reduces
  frequency, does not eliminate the race.
- **Weakest:** just relaunch until it passes. Confirms intermittency; not a fix.

## 9. Verification plan for the fix

1. Rebuild the deploy binary (confirm the exact build command in
   `gear_sonic_deploy` — likely `scripts/build.sh`).
2. **Baseline (pre-fix):** launch ~10x via §6 snippet; record PASS/FAIL count.
3. **Post-fix:** launch ~10x; expect **0** startup `Lost LowState` aborts, and
   `Init Done` -> control every time.
4. **Regression:** confirm (a) it still transitions to CONTROL when a ZMQ `start`
   arrives, and (b) the in-CONTROL strict check still aborts on a REAL disconnect
   (e.g., unplug the robot NIC mid-run on the gantry, or kill the robot-side
   publisher) — that must still print
   "[ERROR] Safety check failed, stopping control." within ~500 ms.

## 10. Open items to double-check

- Confirm the LowState callback thread identity/priority in the vendored
  `unitree_sdk2` (`gear_sonic_deploy/thirdparty/unitree_sdk2/.../channel_subscriber.hpp`).
- Confirm the deploy build command and where the rebuilt binary lands
  (`target/release/g1_deploy_onnx_ref`).
- Decide fix home: quick edit-in-place to validate, then port into
  `gr00t-sonic-deploy.patch`.
- If you also want the config-only mitigation, determine whether the C++ Unitree
  SDK path honours `CYCLONEDDS_URI` on this build.
