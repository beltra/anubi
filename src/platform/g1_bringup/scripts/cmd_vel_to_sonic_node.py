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

Heading model: the SONIC planner has no yaw-rate input; it steers toward `facing`.
This node integrates angular.z into a heading angle theta (open loop) and sends
facing=[cos theta, sin theta]; body-frame (vx, vy) is rotated by theta into the
world frame for `movement`. Nav2 closes the loop through localization, so the
integrated heading never needs to be globally right -- but it does mean a bridge
restart mid-run resets theta and produces a heading transient until Nav2 corrects.

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

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# ZMQPackedMessageSubscriber constant: JSON header is padded to exactly this size.
HEADER_SIZE = 1280

# gear_sonic_deploy LocomotionMode values used for velocity-driven navigation.
MODE_IDLE = 0
MODE_SLOW_WALK = 1
MODE_WALK = 2


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
        # Watchdog: command IDLE if no cmd_vel within this many seconds. The
        # chain publishes at 20 Hz, so 0.3 s rides out jitter without coasting.
        self.declare_parameter("cmd_timeout", 0.3)
        # Period for re-sending the (idempotent) start/planner-mode command.
        self.declare_parameter("start_resend_sec", 1.0)

        bind_host = self.get_parameter("zmq_bind_host").value
        port = int(self.get_parameter("zmq_port").value)
        topic = self.get_parameter("cmd_vel_topic").value
        rate = max(10.0, float(self.get_parameter("rate_hz").value))
        self._mode = int(self.get_parameter("locomotion_mode").value)
        self._vx_fwd = float(self.get_parameter("max_forward_vel").value)
        self._vx_rev = abs(float(self.get_parameter("max_reverse_vel").value))
        self._vy_max = float(self.get_parameter("max_lateral_vel").value)
        self._wz_max = float(self.get_parameter("max_yaw_rate").value)
        self._speed_gain = float(self.get_parameter("speed_gain").value)
        self._timeout = float(self.get_parameter("cmd_timeout").value)
        start_resend = float(self.get_parameter("start_resend_sec").value)

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
        self._last_cmd_t = None     # monotonic ROS time of the last received Twist
        self._theta = 0.0           # open-loop integrated heading [rad]

        self.create_subscription(Twist, topic, self._on_twist, 10)
        self.create_timer(self._dt, self._tick)
        self.create_timer(max(0.1, start_resend), self._send_start)
        self._send_start()

        self.get_logger().info(
            f"bridging {topic} -> ZMQ PUB tcp://{bind_host}:{port} at {rate:.0f} Hz "
            f"(mode={self._mode}, caps vx=[{-self._vx_rev},{self._vx_fwd}] "
            f"vy={self._vy_max} wz={self._wz_max}, speed_gain={self._speed_gain}, "
            f"cmd_timeout={self._timeout}s)")

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
                      height: float = -1.0) -> None:
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
        self._sock.send(b"planner" + _packed_header(fields) + payload)

    def _send_start(self) -> None:
        # Idempotent on the deploy side (start applies only when not started);
        # periodic re-send covers controller restarts and PUB/SUB slow joiners.
        self._send_command(start=True, stop=False, planner=True)

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

    def _tick(self) -> None:
        vx, vy, wz = self._cmd
        if self._timeout > 0.0:
            if self._last_cmd_t is None:
                vx, vy, wz = 0.0, 0.0, 0.0
            else:
                age = (self.get_clock().now() - self._last_cmd_t).nanoseconds * 1e-9
                if age > self._timeout:
                    vx, vy, wz = 0.0, 0.0, 0.0

        self._theta += wz * self._dt
        theta = self._theta
        facing = [math.cos(theta), math.sin(theta), 0.0]

        speed = math.hypot(vx, vy)
        if speed > 1e-3:
            # Body-frame velocity rotated by the integrated heading: the planner
            # expects a world-frame translation direction aligned with `facing`.
            dir_x = vx * math.cos(theta) - vy * math.sin(theta)
            dir_y = vx * math.sin(theta) + vy * math.cos(theta)
            norm = math.hypot(dir_x, dir_y)
            movement = [dir_x / norm, dir_y / norm, 0.0]
            self._send_planner(self._mode, movement, facing,
                               speed=speed * self._speed_gain)
        elif abs(wz) > 1e-3:
            # Turn in place: zero movement, mode kept so the gait keeps stepping.
            self._send_planner(self._mode, [0.0, 0.0, 0.0], facing, speed=-1.0)
        else:
            self._send_planner(MODE_IDLE, [0.0, 0.0, 0.0], facing, speed=-1.0)

    def destroy_node(self):
        # Leave the robot standing in place: IDLE tick, then release control.
        try:
            self._send_planner(MODE_IDLE, [0.0, 0.0, 0.0],
                               [math.cos(self._theta), math.sin(self._theta), 0.0],
                               speed=-1.0)
            self._send_command(start=False, stop=True, planner=True)
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
