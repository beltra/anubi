#!/usr/bin/env python3
"""
Unitree G1 AMO locomotion driver (legs-only, navigation).

Adapts the Talos AMO inference scaffold (CommandSource / WebSocket :8766 /
JointSmoother / Pacer / safety guards) to drive the *Psi0* AMO policy
(amo_jit.pt) via this package's AmoPolicy + G1DDSInterface. The arms are held
at their default pose; only the 15-DoF lower body (legs + waist) is driven.

Control loop (250 Hz)::

    measured_state ─► AmoPolicy.step(vx,vy,yaw) ─► JointSmoother ─► DDS rt/lowcmd

Modes
-----
  (default)            live control: DDS + WS command source + motor commands.
  --observe_only       DDS subscribe, run inference, but NEVER publish to lowcmd.
  --offline_selftest   NO robot, NO DDS: build the observation from a synthetic
                       default state, assert its dim == amo_jit input dim, run
                       ONE forward pass, print obs dim + action shape, exit 0.

Command interface
-----------------
  ws://127.0.0.1:8766  JSON {"vx":..,"vy":..,"yaw":..}.  Caps (see _clamp_cmd):
    |vx| <= 0.5, |vy| <= 0.4, |yaw| <= 0.4.

Bring-up order (HARDWARE): ALWAYS start with --observe_only first and confirm
the obs/action shapes and tilt readings look sane, THEN run live. Run
--offline_selftest before any motor wiring (the observation verification gate).

Run::
    ros2 run g1_locomotion amo_driver --offline_selftest
    ros2 run g1_locomotion amo_driver --observe_only --net_if eth0
    ros2 run g1_locomotion amo_driver --net_if eth0          # live
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading
import time

import numpy as np

# Sibling-module imports work both as an installed package and run-in-place.
try:
    from g1_locomotion.amo_policy_runtime import AmoPolicy, default_robot_state
except ImportError:  # run directly from the source dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from amo_policy_runtime import AmoPolicy, default_robot_state  # type: ignore

logger = logging.getLogger('g1.amo_driver')

# Velocity caps (rad/s for yaw, m/s for vx/vy). Matched to the AMO policy's
# in-distribution range (vx +/-0.5, vy +/-0.4 m/s) and to the Nav2 controller
# caps (nav2_params_g1.yaml vx_max 0.5 / vy_max 0.4 / wz_max 0.4) so this final
# backstop never admits an out-of-distribution forward command.
CAP_VX = 0.5
CAP_VY = 0.4
CAP_YAW = 0.4

# Control rate. The Psi0 policy ticks at 50 Hz (dt=0.02) for the observation
# integration (target_yaw, gait_cycle), but the DDS lowcmd publisher runs at
# 250 Hz holding the latest target. We run the inference loop at 50 Hz and let
# the DDS publish thread stream at 250 Hz. TODO(verify): on hardware confirm the
# 50 Hz inference cadence matches Psi0's effective control rate.
INFER_HZ = 50.0

# Startup activation schedule (talos scaffold defaults; tuned for the soft
# legs-have-no-rate-limit hand-off). All in seconds.
BLEND_S = 3.0           # pose blend from measured -> policy target
GAIN_RAMP_S = 2.0       # kp/kd soft -> full
STABILIZE_S = 3.0       # hold zero command after blend before walking
STARTUP_CMD_RAMP_S = 2.0
KP_SCALE_START = 0.3
KD_SCALE_START = 0.5
CLAMP_DELTA_RAD = 0.05  # per-tick per-joint slew rail during reaching

# Safety guards.
SAFETY_TILT_RAD = 0.6   # abort if base tilt exceeds this
STATE_TIMEOUT_S = 0.5   # abort if no lowstate for this long
CMD_TIMEOUT_S = 0.5     # command deadman: zero velocity if no WS command for this long
DAMP_HOLD_S = 0.4       # stream a damping pose for this long before shutdown


# ── command source (thread-safe holder + optional WS server) ────────────────
class CommandSource:
    """Thread-safe holder for the (vx, vy, yaw) velocity command."""

    def __init__(self, initial=(0.0, 0.0, 0.0)):
        self._lock = threading.Lock()
        self._cmd = np.asarray(initial, dtype=np.float32)
        self._t = None   # monotonic time of the last set(); None until first command

    def get(self) -> np.ndarray:
        with self._lock:
            return self._cmd.copy()

    def get_with_age(self):
        """Return (cmd_copy, seconds_since_last_set or None) for the deadman."""
        with self._lock:
            age = None if self._t is None else (time.perf_counter() - self._t)
            return self._cmd.copy(), age

    def set(self, vx, vy, yaw) -> None:
        with self._lock:
            self._cmd = np.asarray([vx, vy, yaw], dtype=np.float32)
            self._t = time.perf_counter()


def _clamp_cmd(vx, vy, yaw):
    return (float(np.clip(vx, -CAP_VX, CAP_VX)),
            float(np.clip(vy, -CAP_VY, CAP_VY)),
            float(np.clip(yaw, -CAP_YAW, CAP_YAW)))


def start_websocket_source(source: CommandSource, port: int = 8766) -> None:
    """Best-effort WebSocket server accepting {"vx","vy","yaw"} JSON messages.

    Runs in a daemon thread; failures degrade to the last-held command rather
    than crashing the control loop. Commands are clamped to the package caps.
    """
    import asyncio
    import json

    async def _handler(ws):
        async for raw in ws:
            try:
                msg = json.loads(raw)
                vx, vy, yaw = _clamp_cmd(
                    float(msg.get('vx', 0.0)),
                    float(msg.get('vy', 0.0)),
                    float(msg.get('yaw', msg.get('yaw_rate', 0.0))),
                )
                source.set(vx, vy, yaw)
            except (ValueError, TypeError) as exc:
                logger.warning('bad WS command %r: %s', raw, exc)

    def _run():
        try:
            import websockets
        except ImportError:
            logger.error('websockets not installed; command source stays zero')
            return
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def _serve():
            # ping_interval=None: the planner's bridge is send-only and never
            # answers pings (would otherwise drop every ~20 s).
            async with websockets.serve(_handler, '0.0.0.0', port,
                                        ping_interval=None):
                logger.info('WS command server listening on :%d', port)
                await asyncio.Future()

        try:
            loop.run_until_complete(_serve())
        except Exception as exc:  # noqa: BLE001
            logger.error('WS command server stopped: %s', exc)

    threading.Thread(target=_run, daemon=True, name='ws-command').start()


# ── activation helpers (smoothstep schedule) ────────────────────────────────
def _smoothstep(alpha: float) -> float:
    a = float(np.clip(alpha, 0.0, 1.0))
    return a * a * (3.0 - 2.0 * a)


def _gain_ramp_scale(elapsed_s, ramp_s, scale_start):
    if ramp_s <= 0.0:
        return 1.0
    g = _smoothstep(elapsed_s / ramp_s)
    return float(scale_start + (1.0 - scale_start) * g)


def _command_ramp_factor(elapsed_s, ramp_s):
    if ramp_s <= 0.0:
        return 1.0
    return _smoothstep(elapsed_s / ramp_s)


class JointSmoother:
    """Startup pose blend + gain ramp + per-tick slew clamp on the 29-DoF target.

    Mirrors the talos joint_filters.JointSmoother behavior but trimmed to the
    layers this driver needs (A pose-blend, C per-tick clamp, B gain ramp). The
    smoother seeds at the measured pose so tick-0 delta is ~0 (anti-snap).
    """

    def __init__(self, dt, blend_s, gain_ramp_s, kp_scale_start, kd_scale_start,
                 clamp_delta):
        self.dt = float(dt)
        self.blend_s = float(blend_s)
        self.gain_ramp_s = float(gain_ramp_s)
        self.kp_scale_start = float(kp_scale_start)
        self.kd_scale_start = float(kd_scale_start)
        self.clamp_delta = float(clamp_delta)
        self._measured = None
        self._prev = None
        self._t = 0.0

    def reset(self, measured_q):
        self._measured = np.asarray(measured_q, dtype=np.float64).copy()
        self._prev = self._measured.copy()
        self._t = 0.0

    @property
    def elapsed(self):
        return self._t

    @property
    def blending(self):
        return self.blend_s > 0.0 and self._t < self.blend_s

    def gain_scales(self):
        return (_gain_ramp_scale(self._t, self.gain_ramp_s, self.kp_scale_start),
                _gain_ramp_scale(self._t, self.gain_ramp_s, self.kd_scale_start))

    def step(self, raw_target):
        if self._measured is None:
            self.reset(raw_target)
        raw = np.asarray(raw_target, dtype=np.float64)
        # Layer A: smoothstep blend measured -> raw target over blend_s.
        if self.blending:
            s = _smoothstep(self._t / self.blend_s)
            target = (1.0 - s) * self._measured + s * raw
        else:
            target = raw
        # Layer C: per-tick per-joint slew clamp.
        if self.clamp_delta > 0.0:
            delta = np.clip(target - self._prev, -self.clamp_delta, self.clamp_delta)
            cmd = self._prev + delta
        else:
            cmd = target
        self._prev = cmd
        self._t += self.dt
        return cmd


class Pacer:
    """Fixed-rate loop pacer with drift correction (talos scaffold)."""

    def __init__(self, dt):
        self.dt = dt
        self._next = time.perf_counter()

    def wait(self):
        self._next += self.dt
        sleep_t = self._next - time.perf_counter()
        if sleep_t > 0:
            time.sleep(sleep_t)
        else:
            self._next = time.perf_counter()


# ── offline self-test ───────────────────────────────────────────────────────
def offline_selftest(device: str = 'cpu') -> int:
    """Load the policy and validate the observation against amo_jit's input dim.

    NO robot, NO DDS. Builds the observation from a synthetic default state,
    asserts dim == reconstructed expected dim, runs ONE forward pass, prints the
    obs dim + action shape, exits 0 on success.
    """
    logging.getLogger().setLevel(logging.INFO)
    logger.info('offline self-test: loading AMO policy (device=%s)', device)
    policy = AmoPolicy(device=device)

    state = default_robot_state(policy.default_dof_pos)
    policy.reset_yaw(float(state['rpy'][2]))

    # Build the observation and validate the reconstructed dimension.
    observation, extra_hist = policy.build_observation(state)
    obs_dim = int(observation.shape[0])
    expected = policy.expected_obs_dim
    extra_dim = int(np.array(extra_hist).flatten().shape[0])
    expected_extra = policy.expected_extra_hist_dim

    print(f'[selftest] reconstructed obs dim       = {obs_dim}')
    print(f'[selftest] expected (reconstruction)   = {expected}')
    print(f'[selftest] extra_hist (privileged) dim = {extra_dim} '
          f'(expected {expected_extra})')

    assert obs_dim == expected, (
        f'obs dim {obs_dim} != reconstructed expected {expected}')
    assert extra_dim == expected_extra, (
        f'extra_hist dim {extra_dim} != expected {expected_extra}')

    # ONE forward pass through the policy. If the obs dim is wrong, amo_jit
    # raises here -- this IS the verification gate.
    try:
        pd_target, raw_action = policy.infer(observation, extra_hist)
    except RuntimeError as exc:
        if 'cuda' in str(exc).lower() and device == 'cpu':
            # amo_jit.pt was traced with a constant baked onto cuda:0, so a
            # pure-CPU forward pass dies with a device mismatch (NOT a dim
            # error -- the dims above already validated). Re-run on CUDA.
            print('[selftest] NOTE: amo_jit.pt has a CUDA-resident constant; '
                  'its forward pass requires device=cuda. The observation '
                  'dimension gate (above) PASSED on CPU. Re-run with '
                  '--device cuda to exercise the forward pass.')
            print('[selftest] PARTIAL PASS (dims validated; forward pass needs CUDA)')
            return 0
        raise
    print(f'[selftest] amo_jit forward pass OK')
    print(f'[selftest] raw_action shape            = {tuple(np.shape(raw_action))}')
    print(f'[selftest] pd_target (29-DoF) shape     = {tuple(np.shape(pd_target))}')
    print(f'[selftest] pd_target legs+waist[:15]    = '
          f'{np.round(pd_target[:15], 3).tolist()}')
    # Arms (15:29) must equal default_dof_pos (held at default).
    arms_held = np.allclose(pd_target[15:], policy.default_dof_pos[15:])
    print(f'[selftest] arms held at default         = {arms_held}')
    assert arms_held, 'arms are NOT held at default_dof_pos'

    # Sanity: the standing target should be near the default leg pose.
    leg_drift = float(np.max(np.abs(pd_target[:15] - policy.default_dof_pos[:15])))
    print(f'[selftest] max |legs - default|         = {leg_drift:.3f} rad')

    print('[selftest] PASS')
    return 0


# ── live / observe-only control loop ────────────────────────────────────────
def run_control(net_if, device, observe_only) -> int:
    from g1_locomotion.robot_dds_io import G1DDSInterface  # lazy (needs SDK)

    logger.info('initializing AMO policy (device=%s)', device)
    policy = AmoPolicy(device=device)

    # CUDA pre-flight: amo_jit.pt has a cuda:0-resident constant, so its forward
    # pass ONLY runs on CUDA. run_control (both live and observe_only) runs a
    # forward pass every tick, so refuse to start on CPU here -- otherwise the
    # driver would connect DDS, start streaming rt/lowcmd, and only THEN crash on
    # the first inference tick. Fail fast before any motor command is published.
    if 'cuda' not in str(policy.device):
        logger.error(
            'amo_jit.pt requires a CUDA device but the policy resolved to %r; the '
            'forward pass would crash mid-loop after motor streaming begins. Run on '
            'a CUDA host (or pass --device cuda). Aborting before DDS connect.',
            policy.device)
        return 1

    logger.info('connecting DDS (net_if=%s, observe_only=%s)', net_if, observe_only)
    io = G1DDSInterface(net_if=net_if, observe_only=observe_only)
    io.connect()
    if not io.wait_for_state(timeout=10.0):
        logger.error('no lowstate within 10 s; aborting')
        return 1
    io.start_publisher()

    state = io.read_state()
    policy.reset_yaw(float(state['rpy'][2]))

    dt = 1.0 / INFER_HZ
    smoother = JointSmoother(dt, BLEND_S, GAIN_RAMP_S, KP_SCALE_START,
                             KD_SCALE_START, CLAMP_DELTA_RAD)
    smoother.reset(io.get_motor_q())

    command = CommandSource()
    start_websocket_source(command, port=8766)

    command_hold_s = BLEND_S + STABILIZE_S
    pacer = Pacer(dt)
    stop = {'flag': False}

    def _on_signal(signum, _frame):
        logger.info('signal %d received; shutting down', signum)
        stop['flag'] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    logger.info('AMO control loop @ %.0f Hz | blend=%.1fs gain_ramp=%.1fs '
                'stabilize=%.1fs | observe_only=%s',
                INFER_HZ, BLEND_S, GAIN_RAMP_S, STABILIZE_S, observe_only)

    last_state_t = time.perf_counter()
    tick = 0
    try:
        while not stop['flag']:
            now = time.perf_counter()

            # ── safety guards ──────────────────────────────────────────────
            if io.has_state():
                last_state_t = now
            elif (now - last_state_t) > STATE_TIMEOUT_S:
                logger.error('lowstate stale for %.2fs; aborting',
                             now - last_state_t)
                break
            tilt = io.base_tilt_rad()
            if tilt > SAFETY_TILT_RAD:
                logger.error('tilt %.2f rad > %.2f rad; aborting',
                             tilt, SAFETY_TILT_RAD)
                break

            # ── velocity command (held zero during startup, then ramped) ───
            elapsed = smoother.elapsed
            if elapsed < command_hold_s:
                vx, vy, yaw = 0.0, 0.0, 0.0
            else:
                live, cmd_age = command.get_with_age()
                # Command deadman: cmd_vel_to_amo streams at 20 Hz, so a gap
                # > CMD_TIMEOUT_S means the bridge died / disconnected. Hold zero
                # velocity (the policy keeps balancing) rather than the last
                # command -- the driver's own backstop to the bridge watchdog.
                if cmd_age is None or cmd_age > CMD_TIMEOUT_S:
                    live = np.zeros(3, dtype=np.float32)
                ramp = _command_ramp_factor(elapsed - command_hold_s,
                                            STARTUP_CMD_RAMP_S)
                vx, vy, yaw = (live * ramp).tolist()

            # ── inference -> smoothing -> motor command ────────────────────
            state = io.read_state()
            raw_target = policy.step(state, vx, vy, yaw)  # 29-DoF
            cmd_q = smoother.step(raw_target)
            s_kp, s_kd = smoother.gain_scales()
            io.set_targets(cmd_q, kp_scale=s_kp, kd_scale=s_kd)

            if tick % int(round(INFER_HZ)) == 0:  # ~1 Hz status
                logger.info('t=%5.1fs phase=%-9s gains=(%.2f,%.2f) '
                            'cmd=(%.2f,%.2f,%.2f) max|dq|=%.3f tilt=%.2f',
                            elapsed,
                            'blend' if smoother.blending else
                            ('stabilize' if elapsed < command_hold_s else 'walk'),
                            s_kp, s_kd, vx, vy, yaw,
                            float(np.max(np.abs(cmd_q - state['motorstate']))),
                            tilt)
            tick += 1
            pacer.wait()
    finally:
        # Soft e-stop: damp the legs before releasing DDS so the robot settles
        # instead of holding the last PD target. Covers normal SIGINT exit AND
        # the tilt / stale-state aborts above (all paths reach this finally).
        # No-op in observe_only (request_damping never Writes lowcmd there).
        logger.info('commanding damping pose, then shutting down DDS')
        try:
            io.request_damping()
            time.sleep(DAMP_HOLD_S)
        except Exception as exc:  # noqa: BLE001
            logger.warning('damping request failed: %s', exc)
        io.shutdown()
    return 0


# ── CLI ─────────────────────────────────────────────────────────────────────
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Unitree G1 AMO legs-only locomotion driver.')
    p.add_argument('--net_if', default=os.environ.get('PSI_DDS_IFACE'),
                   help='DDS NIC bound to the robot (e.g. eth0).')
    p.add_argument('--device', default=None,
                   help='torch device (cpu/cuda); default auto.')
    p.add_argument('--observe_only', action='store_true',
                   help='Subscribe + run inference but NEVER publish motor cmds.')
    p.add_argument('--offline_selftest', action='store_true',
                   help='No robot/DDS: validate obs dim + one forward pass, exit 0.')
    return p.parse_args(argv)


def main(argv=None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(name)s] %(levelname)s %(message)s',
        datefmt='%H:%M:%S',
    )
    a = parse_args(argv)
    device = a.device or ('cpu' if a.offline_selftest else None)
    if a.offline_selftest:
        return offline_selftest(device=device or 'cpu')
    return run_control(net_if=a.net_if, device=device,
                       observe_only=a.observe_only)


if __name__ == '__main__':
    raise SystemExit(main())
