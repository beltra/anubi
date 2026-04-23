#!/usr/bin/env python3
"""Safety gate between Nav2 /cmd_vel and go2_hw_bridge.

Gating rules (first match wins):
  1. E-stop active -> zero velocity
  2. No /cmd_vel within cmd_timeout seconds (watchdog) -> zero velocity
  3. Otherwise -> forward /cmd_vel unchanged

Enable via go2_bringup.launch.py use_safety:=true.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from robot_common_interfaces.msg import RobotStatus


class VelocityLimiter(Node):

    def __init__(self):
        super().__init__('velocity_limiter')

        self.declare_parameter('cmd_timeout',  0.5)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_in',   '/cmd_vel')
        self.declare_parameter('cmd_vel_out',  '/cmd_vel_safe')
        self.declare_parameter('status_topic', '/robot_status')

        self._cmd_timeout  = self.get_parameter('cmd_timeout').value
        publish_rate       = self.get_parameter('publish_rate').value
        cmd_vel_in         = self.get_parameter('cmd_vel_in').value
        cmd_vel_out        = self.get_parameter('cmd_vel_out').value
        status_topic       = self.get_parameter('status_topic').value

        self._pub = self.create_publisher(Twist, cmd_vel_out, 10)
        self._cmd_sub = self.create_subscription(Twist, cmd_vel_in, self._on_cmd_vel, 10)
        self._status_sub = None
        if status_topic:
            self._status_sub = self.create_subscription(
                RobotStatus, status_topic, self._on_status, 10
            )
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self._last_cmd_vel  = Twist()
        self._last_cmd_time = None
        self._estop         = False
        self._prev_estop    = False
        self._prev_watchdog = False

        status_mode = status_topic if status_topic else "disabled"
        self.get_logger().info(
            f'velocity_limiter: {cmd_vel_in} -> {cmd_vel_out}  '
            f'watchdog={self._cmd_timeout}s  rate={publish_rate}Hz  '
            f'status_topic={status_mode}'
        )

    def _on_cmd_vel(self, msg: Twist):
        self._last_cmd_vel  = msg
        self._last_cmd_time = self.get_clock().now()

    def _on_status(self, msg: RobotStatus):
        self._estop = msg.estop_active
        if self._estop and not self._prev_estop:
            self.get_logger().warning('E-STOP active -- blocking all velocity commands')
        elif not self._estop and self._prev_estop:
            self.get_logger().info('E-STOP cleared -- velocity commands resumed')
        self._prev_estop = self._estop

    def _on_timer(self):
        zero = Twist()

        if self._estop:
            self._pub.publish(zero)
            return

        if self._last_cmd_time is None:
            return

        age_sec = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        watchdog_active = age_sec > self._cmd_timeout
        if watchdog_active and not self._prev_watchdog:
            self.get_logger().warning(
                f'velocity_limiter watchdog: no /cmd_vel for {age_sec:.2f}s -- zeroing'
            )
        elif not watchdog_active and self._prev_watchdog:
            self.get_logger().info('velocity_limiter watchdog cleared')
        self._prev_watchdog = watchdog_active

        if watchdog_active:
            self._pub.publish(zero)
            return

        self._pub.publish(self._last_cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityLimiter()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
