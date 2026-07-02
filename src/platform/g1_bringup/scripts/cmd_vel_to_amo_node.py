#!/usr/bin/env python3
"""Bridge a ROS 2 /cmd_vel (geometry_msgs/Twist) -> the G1 AMO velocity command.

This is the LAST hop in the G1 command chain. The AMO locomotion driver (the
separate `g1_locomotion` package, run as its OWN process) is NOT a ROS 2 node:
it talks to the robot over CycloneDDS and accepts (vx, vy, yaw) velocity commands
on a plain WebSocket server (default ws://127.0.0.1:8766). This node closes the
gap between the two worlds:

    Nav2 /cmd_vel
      -> collision_monitor   /cmd_vel_collision_free
      -> velocity_limiter     /cmd_vel_safe        (robot_safety)
      -> THIS NODE            ws://127.0.0.1:8766 {vx, vy, yaw}
      -> AMO driver (g1_locomotion amo_driver)     -> robot

So by default this node listens on /cmd_vel_safe (the safety-gated topic), clamps
to the G1 velocity caps (vx<=0.5, vy<=0.4, wz<=0.4 -- the AMO policy's
in-distribution range, matching nav2_params_g1 and amo_driver), and forwards the
command as JSON to the AMO WebSocket at a fixed rate.

A minimal, send-only RFC6455 WebSocket client built on the Python stdlib is used
so NO extra Python package (e.g. `websockets`) is needed in the ROS 2 container.

Run standalone (parameters otherwise come from config/g1_params.yaml):

    ros2 run g1_bringup cmd_vel_to_amo --ros-args \\
        -p amo_host:=127.0.0.1 -p amo_port:=8766 -p cmd_vel_topic:=/cmd_vel_safe
"""
from __future__ import annotations

import base64
import json
import os
import socket
import struct
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class _WSClient:
    """Minimal send-only RFC6455 WebSocket client (masked text frames)."""

    def __init__(self, host: str, port: int, path: str = "/"):
        self.host, self.port, self.path = host, int(port), path
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

    def _connect(self) -> None:
        s = socket.create_connection((self.host, self.port), timeout=2.0)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        s.sendall(req.encode())
        resp = s.recv(1024).decode("latin-1", "ignore")
        if "101" not in resp.split("\r\n", 1)[0]:
            s.close()
            raise ConnectionError(f"WS handshake failed: {resp[:80]!r}")
        self._sock = s

    @staticmethod
    def _frame(payload: bytes) -> bytes:
        n = len(payload)
        header = bytes([0x81])                      # FIN + text opcode
        if n < 126:
            header += bytes([0x80 | n])
        elif n < 65536:
            header += bytes([0x80 | 126]) + struct.pack(">H", n)
        else:
            header += bytes([0x80 | 127]) + struct.pack(">Q", n)
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        return header + mask + masked

    def send_json(self, obj: dict) -> bool:
        with self._lock:
            try:
                if self._sock is None:
                    self._connect()
                self._sock.sendall(self._frame(json.dumps(obj).encode()))
                return True
            except (OSError, ConnectionError):
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                self._sock = None
                return False

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None


class CmdVelToAmo(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_amo")
        # WebSocket endpoint of the (non-ROS) AMO locomotion driver.
        self.declare_parameter("amo_host", "127.0.0.1")
        self.declare_parameter("amo_port", 8766)
        # By default consume the safety-gated topic, NOT raw Nav2 /cmd_vel.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        # Fixed send rate to the AMO driver (decoupled from the cmd_vel rate).
        self.declare_parameter("rate_hz", 20.0)
        # G1 velocity caps [m/s, m/s, rad/s]. Final clamp before the robot,
        # matched to the AMO policy in-distribution range (vx 0.5 / vy 0.4) and
        # to nav2_params_g1 + amo_driver so this hop never widens the envelope.
        self.declare_parameter("max_forward_vel", 0.5)
        self.declare_parameter("max_lateral_vel", 0.4)
        self.declare_parameter("max_yaw_rate", 0.4)
        # Watchdog: if no /cmd_vel arrives within this many seconds, command zero
        # velocity. Mirrors the go2_hw_bridge / velocity_limiter cmd_timeout so a
        # silent planner / dropped topic halts the robot rather than coasting.
        self.declare_parameter("cmd_timeout", 0.5)

        host = self.get_parameter("amo_host").value
        port = self.get_parameter("amo_port").value
        topic = self.get_parameter("cmd_vel_topic").value
        rate = float(self.get_parameter("rate_hz").value)
        self._timeout = float(self.get_parameter("cmd_timeout").value)
        self._vmax = (float(self.get_parameter("max_forward_vel").value),
                      float(self.get_parameter("max_lateral_vel").value),
                      float(self.get_parameter("max_yaw_rate").value))

        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_t = None     # monotonic time of the last received Twist
        self._ws = _WSClient(host, port)
        self._connected = False

        self.create_subscription(Twist, topic, self._on_twist, 10)
        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f"bridging {topic} -> ws://{host}:{port} at {rate:.0f} Hz "
            f"(caps vx={self._vmax[0]} vy={self._vmax[1]} yaw={self._vmax[2]}, "
            f"cmd_timeout={self._timeout}s)")

    @staticmethod
    def _clip(v, lim):
        return max(-lim, min(lim, float(v)))

    def _on_twist(self, msg: Twist):
        self._cmd = (
            self._clip(msg.linear.x, self._vmax[0]),
            self._clip(msg.linear.y, self._vmax[1]),
            self._clip(msg.angular.z, self._vmax[2]),
        )
        self._last_cmd_t = self.get_clock().now()

    def _tick(self):
        # Watchdog: zero the command if cmd_vel has gone silent.
        vx, vy, yaw = self._cmd
        if self._timeout > 0.0:
            if self._last_cmd_t is None:
                vx, vy, yaw = 0.0, 0.0, 0.0
            else:
                dt = (self.get_clock().now() - self._last_cmd_t).nanoseconds * 1e-9
                if dt > self._timeout:
                    vx, vy, yaw = 0.0, 0.0, 0.0

        ok = self._ws.send_json({"vx": vx, "vy": vy, "yaw": yaw})
        if ok and not self._connected:
            self._connected = True
            self.get_logger().info("connected to AMO WebSocket")
        elif not ok and self._connected:
            self._connected = False
            self.get_logger().warn("AMO WebSocket dropped; retrying")

    def destroy_node(self):
        self._ws.close()
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = CmdVelToAmo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
