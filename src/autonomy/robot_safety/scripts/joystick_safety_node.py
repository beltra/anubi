#!/usr/bin/env python3
"""Publish joystick override and cancel Nav2 goals on operator input."""

import math

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from unitree_go.msg import WirelessController


def joystick_active(msg, deadzone):
    axes = [msg.lx, msg.ly, msg.rx, msg.ry]
    return any(math.fabs(axis) > deadzone for axis in axes) or msg.keys != 0


def axes_active(msg, deadzone):
    axes = [msg.lx, msg.ly, msg.rx, msg.ry]
    return any(math.fabs(axis) > deadzone for axis in axes)


def pressed_keys(previous_keys, current_keys):
    return current_keys & ~previous_keys


def start_resume_requested(previous_keys, msg, deadzone, start_key_mask):
    return (
        start_key_mask != 0
        and not axes_active(msg, deadzone)
        and (pressed_keys(previous_keys, int(msg.keys)) & start_key_mask) != 0
    )


class JoystickSafety(Node):

    def __init__(self):
        super().__init__('joystick_safety')

        self.declare_parameter('joystick_topic', '/wirelesscontroller')
        self.declare_parameter('override_topic', '/joystick_override')
        self.declare_parameter('joystick_deadzone', 0.05)
        self.declare_parameter('neutral_hold_secs', 2.0)
        self.declare_parameter('watchdog_timeout', 1.0)
        self.declare_parameter('cancel_on_override', True)
        self.declare_parameter('resume_on_start', True)
        self.declare_parameter('start_key_mask', '0')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
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
        self._resume_on_start = bool(self.get_parameter('resume_on_start').value)
        self._start_key_mask = int(str(self.get_parameter('start_key_mask').value), 0)
        goal_pose_topic = self.get_parameter('goal_pose_topic').value
        nav_action_name = self.get_parameter('navigate_to_pose_action').value
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
        self._goal_sub = self.create_subscription(
            PoseStamped, goal_pose_topic, self._on_goal_pose, 10
        )
        self._navigate_client = ActionClient(self, NavigateToPose, nav_action_name)
        self._cancel_clients = [
            (service_name, self.create_client(CancelGoal, service_name))
            for service_name in cancel_services
        ]
        self.create_timer(0.05, self._on_timer)

        self._last_msg = None
        self._last_msg_time = None
        self._neutral_since = self.get_clock().now()
        self._override_active = False
        self._last_keys = 0
        self._resume_requested = False
        self._last_goal_pose = None
        self._goal_interrupted = False
        self._publish_override(False)

        self.get_logger().info(
            f'joystick_safety: {joystick_topic} -> {override_topic} '
            f'deadzone={self._deadzone:.2f} neutral_hold={self._neutral_hold:.1f}s '
            f'start_key_mask=0x{self._start_key_mask:04x}'
        )

    def _on_joystick(self, msg):
        if self._override_active and start_resume_requested(
            self._last_keys,
            msg,
            self._deadzone,
            self._start_key_mask,
        ):
            self._resume_requested = True
        self._last_msg = msg
        self._last_msg_time = self.get_clock().now()
        self._last_keys = int(msg.keys)

    def _on_goal_pose(self, msg):
        self._last_goal_pose = msg

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
                    self._goal_interrupted = self._last_goal_pose is not None
                    self._cancel_nav2_goals()
            return

        if self._neutral_since is None:
            self._neutral_since = now

        neutral_age = (now - self._neutral_since).nanoseconds * 1e-9
        clear_by_hold = neutral_age >= self._neutral_hold
        clear_by_start = self._resume_requested
        if self._override_active and (clear_by_hold or clear_by_start):
            self._override_active = False
            resume = clear_by_start and self._resume_on_start
            self._resume_requested = False
            self._publish_override(False)
            if resume:
                self._resume_navigation()
            else:
                self.get_logger().info('Joystick returned to neutral: Nav2 re-enabled')

    def _is_active(self):
        if self._last_msg is None or self._last_msg_time is None:
            return False

        age = (self.get_clock().now() - self._last_msg_time).nanoseconds * 1e-9
        if age > self._watchdog_timeout:
            return False

        if self._override_active and self._resume_requested:
            return axes_active(self._last_msg, self._deadzone)

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

    def _resume_navigation(self):
        if not self._goal_interrupted or self._last_goal_pose is None:
            self.get_logger().info('Joystick START pressed: Nav2 re-enabled')
            return
        if not self._navigate_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning('navigate_to_pose action unavailable; cannot resume')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._last_goal_pose
        self._navigate_client.send_goal_async(goal)
        self._goal_interrupted = False
        self.get_logger().info('Joystick START pressed: resent last goal for replanning')

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
