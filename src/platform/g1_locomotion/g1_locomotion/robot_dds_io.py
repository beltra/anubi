"""
Unitree G1 DDS I/O for the AMO locomotion driver (legs-only, no teleop).

Extracted and stripped from Psi0/real/teleop/teleop/robot_control/robot_body.py
(BaseBodyController / G1_29_BodyController). Removed: all arm-IK / Pinocchio /
hands / teleop / camera / VLA / data-writer code. What remains is the minimal
DDS transport:

  - subscribe rt/lowstate   (LowState_  : 35-motor q/dq/tau_est + IMU + pad)
  - subscribe rt/odommodestate (SportModeState_ : base position/velocity/rpy/quat)
  - publish   rt/lowcmd     (LowCmd_    : 29-DoF PD targets) @ 250 Hz

unitree_sdk2py is imported LAZILY (inside connect()) so this module imports on a
host without the SDK -- the offline self-test never touches DDS.

Motor index map (29 used of 35), Psi0 robot_arm_joints.G1_29_JointIndex:
    0-5   left  leg  (hip p/r/y, knee, ankle p/r)
    6-11  right leg  (hip p/r/y, knee, ankle p/r)
    12-14 waist (yaw, roll, pitch)
    15-21 left  arm  (shoulder p/r/y, elbow, wrist r/p/y)
    22-28 right arm  (shoulder p/r/y, elbow, wrist r/p/y)
    29-34 unused

PD gains (kp/kd), Psi0 robot_body.py:_setup_motor_params (the active arrays):
    kp = [150,150,150,300,80,20, 150,150,150,300,80,20, 400,400,400,
          50,50,50,50,30,30,30, 50,50,50,50,30,30,30]
    kd = [2,2,2,4,2,1, 2,2,2,4,2,1, 15,15,15,
          7.5,7.5,7.5,7.5,6,6,6, 7.5,7.5,7.5,7.5,6,6,6]
"""

from __future__ import annotations

import os
import threading
import time

import numpy as np

# DDS topic names -- Psi0 robot_body.py:27-29.
TOPIC_LOWCMD = 'rt/lowcmd'
TOPIC_LOWSTATE = 'rt/lowstate'
TOPIC_ODOM = 'rt/odommodestate'

G1_29_NUM_MOTORS = 35   # robot_body.py:30 (35 slots, 29 used)
G1_NUM_USED = 29

# PD gains, robot_body.py:201-222 (the un-commented arrays).
KP = np.array([
    150, 150, 150, 300, 80, 20,
    150, 150, 150, 300, 80, 20,
    400, 400, 400,
    50, 50, 50, 50, 30, 30, 30,
    50, 50, 50, 50, 30, 30, 30,
], dtype=np.float64)
KD = np.array([
    2, 2, 2, 4, 2, 1,
    2, 2, 2, 4, 2, 1,
    15, 15, 15,
    7.5, 7.5, 7.5, 7.5, 6, 6, 6,
    7.5, 7.5, 7.5, 7.5, 6, 6, 6,
], dtype=np.float64)

CONTROL_HZ = 250.0          # robot_body.py:98 (control_dt = 1/250)
CONTROL_DT = 1.0 / CONTROL_HZ


def quat_to_euler(quat) -> np.ndarray:
    """wxyz quaternion -> [roll, pitch, yaw]. Port of master_whole_body.quatToEuler."""
    qw, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    euler = np.zeros(3)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    euler[0] = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    if np.abs(sinp) >= 1:
        euler[1] = np.copysign(np.pi / 2, sinp)
    else:
        euler[1] = np.arcsin(sinp)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    euler[2] = np.arctan2(siny_cosp, cosy_cosp)
    return euler


class _DataBuffer:
    """Thread-safe single-slot buffer (robot_body.py:55-66).

    set() stamps a monotonic arrival time so consumers can tell a *fresh*
    sample from the last one that keeps sitting in the buffer after the DDS
    stream dies. The freshness guards (state/odom staleness aborts) depend on
    this: without it, get() alone cannot distinguish "still streaming" from
    "frozen on the last sample".
    """

    def __init__(self):
        self._data = None
        self._t = None
        self._lock = threading.Lock()

    def get(self):
        with self._lock:
            return self._data

    def get_with_age(self):
        """Return (data, seconds_since_last_set) or (None, None) if never set."""
        with self._lock:
            age = None if self._t is None else (time.perf_counter() - self._t)
            return self._data, age

    def set(self, data):
        with self._lock:
            self._data = data
            self._t = time.perf_counter()


class G1DDSInterface:
    """Minimal Unitree G1 DDS transport for AMO locomotion.

    Usage::

        io = G1DDSInterface(net_if="eth0")
        io.connect()                 # raises if unitree_sdk2py / robot absent
        io.wait_for_state()          # block until first lowstate
        io.start_publisher()         # 250 Hz publish thread, holds measured pose
        ...
        io.set_targets(pd_target_29, kp_scale=1.0, kd_scale=1.0)  # per control tick
        state = io.read_state()      # dict for AmoPolicy.step()
        ...
        io.shutdown()

    With ``observe_only=True`` the publish thread runs but never Writes to
    rt/lowcmd (subscribe-only dry run).
    """

    def __init__(self, net_if: str | None = None, observe_only: bool = False):
        self.net_if = net_if if net_if is not None else os.environ.get('PSI_DDS_IFACE')
        self.observe_only = observe_only

        self._lowstate_buf = _DataBuffer()
        self._odom_buf = _DataBuffer()
        self._stop = threading.Event()
        self._ctrl_lock = threading.Lock()

        # Command targets (29-DoF). None until set; the publish thread holds the
        # measured pose until the first set_targets() call. robot_body.py:89-92.
        self._q_target = None
        self._tauff_target = np.zeros(G1_NUM_USED)
        self._kp_scale = 1.0
        self._kd_scale = 1.0
        self._damping = False   # when True the publish loop streams a damping pose

        # SDK handles (filled in connect()).
        self._torch = None
        self._sdk = None
        self._publisher = None
        self._lowstate_sub = None
        self._odom_sub = None
        self._crc = None
        self._msg = None
        self._LowState = None
        self._subscribe_thread = None
        self._publish_thread = None
        self._connected = False

    # -----------------------------------------------------------------------
    def connect(self) -> None:
        """Lazily import unitree_sdk2py and bring up the DDS channels.

        Raises ImportError if the SDK is missing, or RuntimeError on init
        failure. NOT called by the offline self-test.
        """
        # --- lazy imports (robot_body.py:6-14) ---
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_._SportModeState_ import (
            SportModeState_,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        self._LowState = LowState_

        # Honor net_if so DDS binds the right NIC on multi-NIC hosts
        # (robot_body.py:104-111).
        if self.net_if:
            ChannelFactoryInitialize(0, self.net_if)
        else:
            ChannelFactoryInitialize(0)

        self._publisher = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)
        self._publisher.Init()

        self._lowstate_sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._lowstate_sub.Init()

        self._odom_sub = ChannelSubscriber(TOPIC_ODOM, SportModeState_)
        self._odom_sub.Init(self._odom_callback, 1)

        self._crc = CRC()
        self._msg = unitree_hg_msg_dds__LowCmd_()
        self._msg.mode_pr = 0  # robot_body.py:136

        # Subscribe thread (robot_body.py:122-125).
        self._subscribe_thread = threading.Thread(
            target=self._subscribe_loop, daemon=True)
        self._subscribe_thread.start()
        self._connected = True

    # -----------------------------------------------------------------------
    def _odom_callback(self, msg):
        self._odom_buf.set(msg)

    def _subscribe_loop(self):
        """Poll rt/lowstate and copy q/dq/tau_est + IMU. robot_body.py:234-250."""
        while not self._stop.is_set():
            msg = self._lowstate_sub.Read()
            if msg is not None:
                self._lowstate_buf.set(msg)
            time.sleep(0.002)

    # -----------------------------------------------------------------------
    def wait_for_state(self, timeout: float | None = None) -> bool:
        """Block until a lowstate arrives. robot_body.py:127-131."""
        t0 = time.time()
        while self._lowstate_buf.get() is None:
            if timeout is not None and (time.time() - t0) > timeout:
                return False
            time.sleep(0.01)
        # Initialize the mode_machine and seed targets to the measured pose so
        # the publisher holds position (robot_body.py:137-142). Read from the
        # buffer the subscribe thread fills, not a second Read() on the shared
        # subscriber: that Read() can race the subscribe thread and return None.
        self._msg.mode_machine = self._lowstate_buf.get().mode_machine
        self._q_target = self.get_motor_q()
        return True

    # -----------------------------------------------------------------------
    def start_publisher(self) -> None:
        """Configure motor params and launch the 250 Hz publish thread.

        robot_body.py:144-152 / 227-231 (gains) and :260-300 (loop).
        """
        # Seed every used motor cmd with measured q + nominal gains.
        all_q = self.get_motor_q()
        for i in range(G1_NUM_USED):
            self._msg.motor_cmd[i].mode = 1
            self._msg.motor_cmd[i].kp = float(KP[i])
            self._msg.motor_cmd[i].kd = float(KD[i])
            self._msg.motor_cmd[i].q = float(all_q[i])
            self._msg.motor_cmd[i].dq = 0.0
            self._msg.motor_cmd[i].tau = 0.0
        self._publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True)
        self._publish_thread.start()

    def _publish_loop(self):
        """250 Hz: write the 29-DoF PD target to rt/lowcmd. robot_body.py:260-300.

        Unlike the Psi0 controller there is NO per-joint velocity clip here:
        rate limiting / startup blending is done upstream by the driver's
        JointSmoother before set_targets() is called.
        """
        while not self._stop.is_set():
            start = time.time()
            with self._ctrl_lock:
                q_target = self._q_target
                tauff = self._tauff_target
                kp_s = self._kp_scale
                kd_s = self._kd_scale
                damping = self._damping
            if damping:
                # Soft e-stop: pure velocity damping. kp=0 (no position target,
                # so q is irrelevant) + kd retained so the joints resist motion
                # and the robot settles instead of holding its stance.
                for i in range(G1_NUM_USED):
                    self._msg.motor_cmd[i].q = 0.0
                    self._msg.motor_cmd[i].dq = 0.0
                    self._msg.motor_cmd[i].tau = 0.0
                    self._msg.motor_cmd[i].kp = 0.0
                    self._msg.motor_cmd[i].kd = float(KD[i])
                if not self.observe_only:
                    self._msg.crc = self._crc.Crc(self._msg)
                    self._publisher.Write(self._msg)
            elif q_target is not None:
                for i in range(G1_NUM_USED):
                    self._msg.motor_cmd[i].q = float(q_target[i])
                    self._msg.motor_cmd[i].dq = 0.0
                    self._msg.motor_cmd[i].tau = float(tauff[i])
                    self._msg.motor_cmd[i].kp = float(KP[i] * kp_s)
                    self._msg.motor_cmd[i].kd = float(KD[i] * kd_s)
                if not self.observe_only:
                    self._msg.crc = self._crc.Crc(self._msg)
                    self._publisher.Write(self._msg)
            elapsed = time.time() - start
            time.sleep(max(0.0, CONTROL_DT - elapsed))

    # -----------------------------------------------------------------------
    def set_targets(self, pd_target_29, kp_scale: float = 1.0,
                    kd_scale: float = 1.0, tauff_29=None) -> None:
        """Update the 29-DoF PD position target the publisher streams."""
        with self._ctrl_lock:
            self._q_target = np.asarray(pd_target_29, dtype=np.float64).copy()
            if tauff_29 is not None:
                self._tauff_target = np.asarray(tauff_29, dtype=np.float64).copy()
            self._kp_scale = float(kp_scale)
            self._kd_scale = float(kd_scale)

    # -----------------------------------------------------------------------
    def get_motor_q(self) -> np.ndarray:
        """29-DoF measured joint position. robot_body.py:315-319."""
        ls = self._lowstate_buf.get()
        return np.array([ls.motor_state[i].q for i in range(G1_NUM_USED)],
                        dtype=np.float64)

    def get_motor_dq(self) -> np.ndarray:
        """29-DoF measured joint velocity. robot_body.py:321-328."""
        ls = self._lowstate_buf.get()
        return np.array([ls.motor_state[i].dq for i in range(G1_NUM_USED)],
                        dtype=np.float64)

    def get_imu(self):
        """Raw IMU sub-message (quaternion wxyz / gyroscope / rpy). robot_body.py:337-339.

        Reads the buffer the subscribe thread fills (same source as get_motor_q),
        NOT a direct Read() on the shared subscriber: a second Read() races the
        subscribe thread and can return None, crashing every per-tick caller.
        """
        return self._lowstate_buf.get().imu_state

    def get_odom(self):
        """Base odometry dict from rt/odommodestate. robot_body.py:172-182."""
        state = self._odom_buf.get()
        if not state:
            return None
        return {
            'position': np.array(state.position),
            'velocity': np.array(state.velocity),
            'orientation_rpy': np.array(state.imu_state.rpy),
            'orientation_quaternion': np.array(state.imu_state.quaternion),
        }

    # -----------------------------------------------------------------------
    def read_state(self) -> dict:
        """Assemble the robot-state dict that AmoPolicy.step() consumes.

        Uses the LowState IMU for orientation/ang-vel (the same source Psi0
        feeds the policy: master_whole_body.py:610-623). Quaternion is wxyz.
        """
        imu = self.get_imu()
        quat = np.array(imu.quaternion, dtype=np.float32)        # wxyz
        rpy = quat_to_euler(quat).astype(np.float32)
        ang_vel = np.array(imu.gyroscope, dtype=np.float32)
        return {
            'motorstate': self.get_motor_q().astype(np.float32),
            'velstate': self.get_motor_dq().astype(np.float32),
            'ang_vel': ang_vel,
            'rpy': rpy,
            'quat': quat,
        }

    # -----------------------------------------------------------------------
    def base_tilt_rad(self) -> float:
        """Gravity tilt angle (rad) from vertical, for the safety guard.

        angle between the body +Z axis and world up. 0 = perfectly upright.
        """
        st = self.read_state()
        roll, pitch = float(st['rpy'][0]), float(st['rpy'][1])
        # cos(tilt) = cos(roll)*cos(pitch) for a ZYX rpy; clamp for acos domain.
        c = np.clip(np.cos(roll) * np.cos(pitch), -1.0, 1.0)
        return float(np.arccos(c))

    def has_state(self) -> bool:
        """True once any lowstate has ever arrived (NOT a freshness check).

        This latches True for the buffer's lifetime, so it cannot detect a dead
        DDS stream -- use state_age() for the staleness guard.
        """
        return self._lowstate_buf.get() is not None

    def state_age(self) -> float | None:
        """Seconds since the last *fresh* lowstate, or None if none has arrived.

        The subscribe thread only refreshes the buffer on a genuinely new sample,
        so a frozen/dead rt/lowstate stream makes this grow without bound. This is
        the signal the AMO driver's stale-state abort must use (has_state() cannot
        see staleness because it never un-latches)."""
        _, age = self._lowstate_buf.get_with_age()
        return age

    def odom_age(self) -> float | None:
        """Seconds since the last *fresh* rt/odommodestate, or None if none yet."""
        _, age = self._odom_buf.get_with_age()
        return age

    # -----------------------------------------------------------------------
    def request_damping(self) -> None:
        """Switch the publish loop to a soft damping pose (kp=0, kd=KD).

        The 250 Hz publisher then streams a pure-damping command (joints resist
        velocity, no position target) instead of the last PD target, so the
        robot settles rather than holding its stance. The AMO driver calls this
        on shutdown and on safety abort, then sleeps briefly so a few damping
        frames go out before shutdown(). No-op in observe_only (never Writes).
        Idempotent and thread-safe.
        """
        with self._ctrl_lock:
            self._damping = True

    # -----------------------------------------------------------------------
    def shutdown(self) -> None:
        """Stop the publish/subscribe threads and release DDS.

        Call request_damping() first (and allow a few 250 Hz frames to stream,
        e.g. via a short sleep) for a soft e-stop; the AMO driver does this on
        both normal shutdown and safety abort. Once the threads are stopped no
        further lowcmd is written, so the robot is left holding whatever was
        last commanded (the damping pose). The hardware damp combo remains the
        ultimate e-stop (Psi0 master_whole_body.py:570-576).
        """
        self._stop.set()
        for t in (self._publish_thread, self._subscribe_thread):
            if t is not None:
                t.join(timeout=1.0)
