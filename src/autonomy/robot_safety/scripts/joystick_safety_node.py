#!/usr/bin/env python3
"""Publish joystick override and cancel Nav2 goals on operator input."""

import math

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from unitree_go.msg import WirelessController


def joystick_active(msg, deadzone):
    axes = [msg.lx, msg.ly, msg.rx, msg.ry]
    return any(math.fabs(axis) > deadzone for axis in axes) or msg.keys != 0


class JoystickSafety(Node):

    def __init__(self):
        super().__init__('joystick_safety')

        self.declare_parameter('joystick_topic', '/wirelesscontroller')
        self.declare_parameter('override_topic', '/joystick_override')
        self.declare_parameter('joystick_deadzone', 0.05)
        self.declare_parameter('neutral_hold_secs', 2.0)
        self.declare_parameter('watchdog_timeout', 1.0)
        self.declare_parameter('cancel_on_override', True)
        self.declare_parameter(
            'cancel_services',
            [
                '/navigate_to_pose/_action/cancel_goal',
                '/navigate_through_poses/_action/cancel_goal',
            ],
        )

        joystick_topic = self.get_parameter('joystick_topic').value
        override_topic = self.get_parameter('override_topic').value
        self._deadzone = float(self.get_parameter('joystick_deadzone').value)
        self._neutral_hold = float(self.get_parameter('neutral_hold_secs').value)
        self._watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)
        self._cancel_on_override = bool(self.get_parameter('cancel_on_override').value)
        cancel_services = list(self.get_parameter('cancel_services').value)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._override_pub = self.create_publisher(Bool, override_topic, latched_qos)
        self._joy_sub = self.create_subscription(
            WirelessController, joystick_topic, self._on_joystick, 10
        )
        self._cancel_clients = [
            (service_name, self.create_client(CancelGoal, service_name))
            for service_name in cancel_services
        ]
        self.create_timer(0.05, self._on_timer)

        self._last_msg = None
        self._last_msg_time = None
        self._neutral_since = self.get_clock().now()
        self._override_active = False
        self._publish_override(False)

        self.get_logger().info(
            f'joystick_safety: {joystick_topic} -> {override_topic} '
            f'deadzone={self._deadzone:.2f} neutral_hold={self._neutral_hold:.1f}s'
        )

    def _on_joystick(self, msg):
        self._last_msg = msg
        self._last_msg_time = self.get_clock().now()

    def _on_timer(self):
        active = self._is_active()
        now = self.get_clock().now()

        if active:
            self._neutral_since = None
            if not self._override_active:
                self._override_active = True
                self._publish_override(True)
                self.get_logger().warning(
                    'JOYSTICK OVERRIDE: autonomous navigation suspended'
                )
                if self._cancel_on_override:
                    self._cancel_nav2_goals()
            return

        if self._neutral_since is None:
            self._neutral_since = now

        neutral_age = (now - self._neutral_since).nanoseconds * 1e-9
        if self._override_active and neutral_age >= self._neutral_hold:
            self._override_active = False
            self._publish_override(False)
            self.get_logger().info('Joystick returned to neutral: Nav2 re-enabled')

    def _is_active(self):
        if self._last_msg is None or self._last_msg_time is None:
            return False

        age = (self.get_clock().now() - self._last_msg_time).nanoseconds * 1e-9
        if age > self._watchdog_timeout:
            return False

        return joystick_active(self._last_msg, self._deadzone)

    def _cancel_nav2_goals(self):
        request = CancelGoal.Request()
        for service_name, client in self._cancel_clients:
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.05):
                client.call_async(request)
            else:
                self.get_logger().warning(
                    f'Nav2 cancel service unavailable: {service_name}'
                )

    def _publish_override(self, active):
        msg = Bool()
        msg.data = bool(active)
        self._override_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickSafety()
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


if __name__ == '__main__':
    main()
