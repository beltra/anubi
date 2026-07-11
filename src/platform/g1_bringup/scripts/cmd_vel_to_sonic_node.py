#!/usr/bin/env python3
"""Bridge a ROS 2 /cmd_vel (geometry_msgs/Twist) -> the SONIC whole-body controller.

This is the LAST ROS hop of the G1 command chain when locomotion:=sonic. The SONIC
controller (g1_deploy_onnx_ref from GR00T-WholeBodyControl, deployed via the
sonic-g1-locomanipulation repo) is NOT a ROS 2 node: it runs on the Jetson next to
the robot, reads per-tick locomotion targets from a ZMQ PUB/SUB socket, and writes
rt/lowcmd over the Unitree DDS at 50 Hz. This node closes the gap:

    Nav2 /cmd_vel
      -> collision_monitor   /cmd_vel_collision_free
      -> velocity_limiter     /cmd_vel_safe        (robot_safety)
      -> THIS NODE            ZMQ PUB tcp://*:5556 (topics `command` + `planner`)
      -> g1_deploy_onnx_ref (SONIC controller, Jetson)  -> rt/lowcmd -> robot

This node BINDS the ZMQ socket; the SONIC controller connects to it
(`--zmq-host <this host> --zmq-port <zmq_port>` on g1_deploy_onnx_ref).

Wire format (must match gear_sonic_deploy ZMQPackedMessageSubscriber):
    message = topic_bytes + header(1280 B null-padded JSON) + little-endian payload
Header field order must match the payload concatenation order.
`planner` fields: mode:i32, movement:f32[3], facing:f32[3], speed:f32, height:f32.
movement/facing are WORLD-frame direction unit vectors (Z ignored); speed -1 and
height -1 select the mode defaults.

Heading model (CLOSED LOOP): the SONIC planner has no yaw-rate input; it steers
toward `facing`. Turning therefore happens ONLY through `facing`, and a body-frame
command alone can never make SONIC turn. This node anchors the world frame on the
MEASURED odometry yaw (odom_topic, default /odom, the EKF-fused body pose that also
owns odom->base_footprint) rather than integrating angular.z open loop:

    dth      = wrap(yaw_meas - yaw0)          # heading vs SONIC's start frame
    movement = R(dth) . [vx, vy]              # body velocity -> world direction
    face     = dth + wz * facing_lookahead    # inject the turn intent AHEAD of truth
    facing   = [cos(face), sin(face)]

yaw0 is the odom yaw latched when this node starts commanding (SONIC defines its own
world frame at policy start as facing=[1,0,0]); anchoring relative makes the odom and
SONIC frames agree. facing_lookahead_sec is REQUIRED (>0): without it `facing` == the
current heading and the robot never turns; it converges back to dth as the measured
yaw catches up and Nav2 shrinks wz. This is the drift-free replacement for the stock
bridge's open-loop `theta += wz*dt`, whose integrated heading accumulates error and
resets on a bridge restart.

Idle heading hold: with no translation and no turn command the world frame is LATCHED
(facing pinned, yaw0 re-anchored every tick) so measured-yaw drift is ABSORBED instead
of turning the torso to chase it. Without this the robot slowly rotates to follow odom
yaw drift while just standing (worst on a hoist, where the yaw estimate wanders --
"not staying still until commanded a goal"). Reactivation is seamless: dth resumes
from the held angle when a real command arrives.

Velocity semantics: the caps below are REALIZED m/s. The SONIC target-velocity
planner realizes ~0.85x of the commanded speed, so the Twist is clamped to the
caps FIRST and multiplied by speed_gain (~1.18) AFTER, right before the wire.
This keeps the nav-visible envelope equal to the caps while the robot actually
reaches them. Upstream's cmd_vel_to_sonic.py clamps after the gain, which
silently cancels the gain at top speed; this node deliberately does not.

Yaw clamp: nothing in the SONIC stack limits yaw rate on the ZMQ path (the
per-mode clamps live only in its keyboard handler), so max_yaw_rate here and
Nav2's wz_max are the ONLY yaw limits in the sonic chain. Keep them matched.

Watchdog layering: if no cmd_vel arrives within cmd_timeout this node commands
IDLE (planner ticks keep flowing); the SONIC controller separately falls back if
planner messages stop for ~1 s, and its 500 Hz writer soft-holds on stale motor
commands. This node's watchdog is the first and fastest of the three.

The `command` start message is re-sent periodically: the deploy applies start
only when not already started (level-triggered, idempotent), so re-sending makes
the bridge robust to controller restarts and to the controller connecting late
(ZMQ PUB/SUB drops messages published before the subscriber joins).

Run standalone (parameters otherwise come from config/g1_sonic_*_params.yaml):

    ros2 run g1_bringup cmd_vel_to_sonic --ros-args \\
        -p zmq_port:=5556 -p cmd_vel_topic:=/cmd_vel_safe -p locomotion_mode:=1
"""
from __future__ import annotations

import json
import math
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ZMQPackedMessageSubscriber constant: JSON header is padded to exactly this size.
HEADER_SIZE = 1280

# gear_sonic_deploy LocomotionMode values used for velocity-driven navigation.
MODE_IDLE = 0
MODE_SLOW_WALK = 1
MODE_WALK = 2

# Below this magnitude a translation/yaw command counts as "no command" (idle).
_EPS = 1e-3

# Optional 17-DOF upper-body target the deploy binary accepts in the planner
# message (g1_deploy_onnx_ref input_command.hpp upper_body_position). Sending it
# every tick pins the waist/arms/wrists to a fixed pose while the lower body keeps
# walking under planner control -- i.e. carry an object without the arms swinging.
# Wire order is the hardware joint order the deploy expects
# (policy_parameters.hpp upper_body_joint_isaaclab_order_in_mujoco_index): waist
# yaw/roll/pitch, then L/R interleaved shoulder-pitch, shoulder-roll, shoulder-yaw,
# elbow, wrist-roll, wrist-pitch, wrist-yaw.
UPPER_BODY_JOINTS = [
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "right_shoulder_pitch",
    "left_shoulder_roll", "right_shoulder_roll",
    "left_shoulder_yaw", "right_shoulder_yaw",
    "left_elbow", "right_elbow",
    "left_wrist_roll", "right_wrist_roll",
    "left_wrist_pitch", "right_wrist_pitch",
    "left_wrist_yaw", "right_wrist_yaw",
]

# Radian targets in UPPER_BODY_JOINTS order. "default" is the policy's nominal
# standing arms; "carry" brings the arms forward, elbows bent ~70 deg, forearms
# level -- a stable two-handed carry. Tune per payload; validate on a hoist first.
UB_PRESETS = {
    "default": [
        0.0, 0.0, 0.0,
        0.2, 0.2, 0.2, -0.2, 0.0, 0.0,
        0.6, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ],
    "carry": [
        0.0, 0.0, 0.0,
        -0.4, -0.4, 0.25, -0.25, 0.0, 0.0,
        1.2, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ],
}


def _wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quat(q) -> float:
    """Yaw (rad) from a geometry_msgs/Quaternion, without a tf2 dependency."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _upper_body_target(preset):
    """Return the 17-DOF upper-body vector for `preset`, or None to leave the arms
    under gait control. Accepts none/off/'' (-> None) or a UB_PRESETS key."""
    key = (preset or "none").strip().lower()
    if key in ("", "none", "off"):
        return None
    if key not in UB_PRESETS:
        raise ValueError(
            f"unknown arm_preset '{preset}'. valid: none, {', '.join(sorted(UB_PRESETS))}")
    return list(UB_PRESETS[key])


def _packed_header(fields: list[dict]) -> bytes:
    header = {"v": 1, "endian": "le", "count": 1, "fields": fields}
    raw = json.dumps(header).encode("utf-8")
    if len(raw) > HEADER_SIZE:
        raise ValueError("ZMQ header exceeds HEADER_SIZE")
    return raw + b"\x00" * (HEADER_SIZE - len(raw))


class CmdVelToSonic(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_sonic")
        # ZMQ PUB endpoint this node binds; the SONIC controller connects here.
        self.declare_parameter("zmq_bind_host", "*")
        self.declare_parameter("zmq_port", 5556)
        # By default consume the safety-gated topic, NOT raw Nav2 /cmd_vel.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        # Measured yaw source for the closed-loop world-frame conversion. Default
        # /odom: the EKF-fused body pose (owns odom->base_footprint, present in sim
        # and real). Set to /lidar_odom to anchor directly on DLIO instead.
        self.declare_parameter("odom_topic", "/odom")
        # Planner tick rate. The SONIC planner runs at 50 Hz and treats a ~1 s
        # planner-message gap as command loss; >= 10 Hz required, 30 Hz default.
        self.declare_parameter("rate_hz", 30.0)
        # LocomotionMode sent while moving: 1 = SLOW_WALK (0.2-0.8 m/s band),
        # 2 = WALK. Must stay consistent with the velocity caps below.
        self.declare_parameter("locomotion_mode", MODE_SLOW_WALK)
        # G1 velocity caps in REALIZED [m/s, m/s, m/s, rad/s]. Final ROS-side
        # clamp before the robot; must match the nav2_params_g1_sonic_* profile
        # so this hop never widens the Nav2 envelope. max_reverse_vel is a
        # positive magnitude (vx is clamped to [-max_reverse_vel, max_forward_vel]).
        self.declare_parameter("max_forward_vel", 0.5)
        self.declare_parameter("max_reverse_vel", 0.2)
        self.declare_parameter("max_lateral_vel", 0.4)
        self.declare_parameter("max_yaw_rate", 0.4)
        # Commanded-to-wire speed multiplier compensating the planner's ~0.85x
        # speed realization (1.18 ~= 1/0.85). Applied AFTER the caps.
        self.declare_parameter("speed_gain", 1.18)
        # Turn-intent look-ahead: `facing` leads the measured heading by wz * this.
        # REQUIRED (>0) -- with 0 the closed-loop `facing` equals the current
        # heading every tick and the robot never turns.
        self.declare_parameter("facing_lookahead_sec", 0.4)
        # Watchdog: command IDLE if no cmd_vel within this many seconds. The
        # chain publishes at 20 Hz, so 0.3 s rides out jitter without coasting.
        self.declare_parameter("cmd_timeout", 0.3)
        # Period for re-sending the (idempotent) start/planner-mode command.
        self.declare_parameter("start_resend_sec", 1.0)
        # Fixed upper-body pose held while walking: 'none' (default; arms swing with
        # the gait), 'default' (nominal standing arms), or 'carry' (two-handed carry).
        self.declare_parameter("arm_preset", "none")

        bind_host = self.get_parameter("zmq_bind_host").value
        port = int(self.get_parameter("zmq_port").value)
        topic = self.get_parameter("cmd_vel_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        rate = max(10.0, float(self.get_parameter("rate_hz").value))
        self._mode = int(self.get_parameter("locomotion_mode").value)
        self._vx_fwd = float(self.get_parameter("max_forward_vel").value)
        self._vx_rev = abs(float(self.get_parameter("max_reverse_vel").value))
        self._vy_max = float(self.get_parameter("max_lateral_vel").value)
        self._wz_max = float(self.get_parameter("max_yaw_rate").value)
        self._speed_gain = float(self.get_parameter("speed_gain").value)
        self._lookahead = float(self.get_parameter("facing_lookahead_sec").value)
        self._timeout = float(self.get_parameter("cmd_timeout").value)
        start_resend = float(self.get_parameter("start_resend_sec").value)
        # None -> arms follow the gait; a 17-DOF vector -> pinned every tick. Raises
        # on an unknown preset so a typo fails at startup, not silently on hardware.
        self._arm_preset = (self.get_parameter("arm_preset").value or "none").strip().lower()
        self._upper_body = _upper_body_target(self._arm_preset)

        # pyzmq is the one extra runtime dep of this node (rosdep: python3-zmq).
        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError(
                "cmd_vel_to_sonic requires pyzmq (apt: python3-zmq)") from exc
        self._zmq_ctx = zmq.Context.instance()
        self._sock = self._zmq_ctx.socket(zmq.PUB)
        self._sock.bind(f"tcp://{bind_host}:{port}")

        self._dt = 1.0 / rate
        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_t = None      # monotonic ROS time of the last received Twist
        self._yaw = None             # latest measured odom yaw [rad]
        self._yaw0 = None            # anchor: odom yaw at control start (SONIC frame origin)
        self._need_anchor = True     # latch yaw0 on the first tick with a yaw available
        self._idle_face = None       # latched heading held while idle [rad]

        self.create_subscription(Twist, topic, self._on_twist, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_timer(self._dt, self._tick)
        self.create_timer(max(0.1, start_resend), self._send_start)
        self._send_start()

        self.get_logger().info(
            f"bridging {topic} (+ yaw from {odom_topic}) -> ZMQ PUB "
            f"tcp://{bind_host}:{port} at {rate:.0f} Hz "
            f"(mode={self._mode}, caps vx=[{-self._vx_rev},{self._vx_fwd}] "
            f"vy={self._vy_max} wz={self._wz_max}, speed_gain={self._speed_gain}, "
            f"lookahead={self._lookahead:.2f}s, cmd_timeout={self._timeout}s, "
            f"arm_preset={self._arm_preset})")
        if self._lookahead <= 0.0:
            self.get_logger().warn(
                "facing_lookahead_sec <= 0: the closed-loop `facing` equals the "
                "measured heading every tick, so the robot will NOT turn. Set > 0.")
        if self._upper_body is not None:
            self.get_logger().warn(
                f"holding fixed upper-body pose '{self._arm_preset}' (loco-manipulation): "
                "arms/wrists are pinned while walking. Validate on a hoist and tune per "
                "payload before feet-down use.")

    # ---- ZMQ wire helpers ------------------------------------------------------

    def _send_command(self, start: bool, stop: bool, planner: bool) -> None:
        fields = [
            {"name": "start", "dtype": "u8", "shape": [1]},
            {"name": "stop", "dtype": "u8", "shape": [1]},
            {"name": "planner", "dtype": "u8", "shape": [1]},
        ]
        payload = struct.pack("BBB", int(start), int(stop), int(planner))
        self._sock.send(b"command" + _packed_header(fields) + payload)

    def _send_planner(self, mode: int, movement, facing, speed: float,
                      height: float = -1.0, upper_body=None) -> None:
        fields = [
            {"name": "mode", "dtype": "i32", "shape": [1]},
            {"name": "movement", "dtype": "f32", "shape": [3]},
            {"name": "facing", "dtype": "f32", "shape": [3]},
            {"name": "speed", "dtype": "f32", "shape": [1]},
            {"name": "height", "dtype": "f32", "shape": [1]},
        ]
        payload = struct.pack("<i", int(mode))
        payload += struct.pack("<fff", *[float(v) for v in movement])
        payload += struct.pack("<fff", *[float(v) for v in facing])
        payload += struct.pack("<ff", float(speed), float(height))
        # Optional fixed upper-body target: 17-DOF position + zero velocity. Header
        # field order must match the payload order; the deploy slices buffers by
        # field order and enables upper-body control when upper_body_position exists.
        if upper_body is not None:
            fields.append({"name": "upper_body_position", "dtype": "f32", "shape": [17]})
            fields.append({"name": "upper_body_velocity", "dtype": "f32", "shape": [17]})
            payload += struct.pack("<17f", *[float(v) for v in upper_body])
            payload += struct.pack("<17f", *([0.0] * 17))
        self._sock.send(b"planner" + _packed_header(fields) + payload)

    def _send_start(self) -> None:
        # Idempotent on the deploy side (start applies only when not started);
        # periodic re-send covers controller restarts and PUB/SUB slow joiners.
        self._send_command(start=True, stop=False, planner=True)

    def _facing_hold(self):
        """`facing` unit vector that HOLDS heading. Uses the latched idle heading
        when known, else falls back to SONIC's start heading before any odom/anchor
        is available (world frame not yet built)."""
        if self._idle_face is not None:
            return [math.cos(self._idle_face), math.sin(self._idle_face), 0.0]
        return [1.0, 0.0, 0.0]

    def _send_idle(self) -> None:
        self._send_planner(MODE_IDLE, [0.0, 0.0, 0.0], self._facing_hold(),
                           speed=-1.0, upper_body=self._upper_body)

    # ---- ROS callbacks -----------------------------------------------------------

    @staticmethod
    def _clip(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(value)))

    def _on_twist(self, msg: Twist) -> None:
        self._cmd = (
            self._clip(msg.linear.x, -self._vx_rev, self._vx_fwd),
            self._clip(msg.linear.y, -self._vy_max, self._vy_max),
            self._clip(msg.angular.z, -self._wz_max, self._wz_max),
        )
        self._last_cmd_t = self.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def _tick(self) -> None:
        vx, vy, wz = self._cmd
        if self._timeout > 0.0:
            if self._last_cmd_t is None:
                vx, vy, wz = 0.0, 0.0, 0.0
            else:
                age = (self.get_clock().now() - self._last_cmd_t).nanoseconds * 1e-9
                if age > self._timeout:
                    vx, vy, wz = 0.0, 0.0, 0.0

        # No measured yaw yet -> cannot build the world frame; hold the start pose.
        if self._yaw is None:
            self._send_idle()
            return
        if self._need_anchor:
            self._yaw0 = self._yaw
            self._need_anchor = False
            self.get_logger().info(
                f"anchored SONIC world frame at yaw0={self._yaw0:.3f} rad")

        dth = _wrap(self._yaw - self._yaw0)
        speed_mag = math.hypot(vx, vy)

        # No motion command -> HOLD heading. Latch `facing` on entering idle and
        # re-anchor yaw0 each tick so measured-yaw drift is ABSORBED instead of
        # turning the torso to chase it. Reactivation resumes from the held angle.
        if speed_mag <= _EPS and abs(wz) <= _EPS:
            if self._idle_face is None:
                self._idle_face = dth
            self._yaw0 = _wrap(self._yaw - self._idle_face)
            facing = [math.cos(self._idle_face), math.sin(self._idle_face), 0.0]
            self._send_planner(MODE_IDLE, [0.0, 0.0, 0.0], facing,
                               speed=-1.0, upper_body=self._upper_body)
            return

        # Active command -> resume MEASURED-yaw tracking.
        self._idle_face = None
        c, s = math.cos(dth), math.sin(dth)
        # Body-frame velocity rotated into SONIC's world frame by the MEASURED heading.
        dir_x = vx * c - vy * s
        dir_y = vx * s + vy * c
        if speed_mag > _EPS:
            norm = math.hypot(dir_x, dir_y)
            movement = [dir_x / norm, dir_y / norm, 0.0]
            speed = speed_mag * self._speed_gain
        else:
            # Turn in place: zero movement, mode kept so the gait keeps stepping.
            movement = [0.0, 0.0, 0.0]
            speed = -1.0
        # `facing` leads the measured heading by the commanded turn rate so the
        # policy actually turns; converges to dth as wz -> 0.
        face = dth + wz * self._lookahead
        facing = [math.cos(face), math.sin(face), 0.0]
        self._send_planner(self._mode, movement, facing, speed=speed,
                           upper_body=self._upper_body)

    def destroy_node(self):
        # Leave the robot standing in place: IDLE tick, then release control. The
        # short sleeps let the PUB socket flush each message before it is closed
        # (close discards anything still queued), so the stop actually reaches the
        # controller.
        try:
            self._send_idle()
            time.sleep(0.1)
            self._send_command(start=False, stop=True, planner=True)
            time.sleep(0.1)
            self._sock.close(0)
        except Exception:
            pass
        super().destroy_node()


def main(argv=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=argv)
    node = CmdVelToSonic()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT or SIGTERM (kill_ros_nodes.sh): fall through to destroy_node,
        # which sends the SONIC stop command before closing the socket.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
