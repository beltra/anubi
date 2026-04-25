#!/usr/bin/env python3
"""Latch a tilt estop when body attitude exceeds safe limits."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from unitree_go.msg import WirelessController


def parse_key_mask(value):
    return int(str(value), 0)


def should_clear_tilt_estop(latched, current_tilt, clear_tilt, ack_mask, pressed_keys):
    return (
        latched
        and current_tilt <= clear_tilt
        and ack_mask != 0
        and (pressed_keys & ack_mask) != 0
    )


class TiltMonitor(Node):

    def __init__(self):
        super().__init__('tilt_monitor')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('joystick_topic', '/wirelesscontroller')
        self.declare_parameter('tilt_estop_topic', '/safety/tilt_estop')
        self.declare_parameter('warn_tilt_deg', 20.0)
        self.declare_parameter('estop_tilt_deg', 30.0)
        self.declare_parameter('clear_tilt_deg', 15.0)
        self.declare_parameter('estop_hold_sec', 0.25)
        self.declare_parameter('recovery_ack_key_mask', '0')

        imu_topic = self.get_parameter('imu_topic').value
        joystick_topic = self.get_parameter('joystick_topic').value
        tilt_topic = self.get_parameter('tilt_estop_topic').value
        self._warn_tilt = math.radians(float(self.get_parameter('warn_tilt_deg').value))
        self._estop_tilt = math.radians(float(self.get_parameter('estop_tilt_deg').value))
        self._clear_tilt = math.radians(float(self.get_parameter('clear_tilt_deg').value))
        self._estop_hold = float(self.get_parameter('estop_hold_sec').value)
        ack_mask_value = self.get_parameter('recovery_ack_key_mask').value
        self._recovery_ack_mask = parse_key_mask(ack_mask_value)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._tilt_pub = self.create_publisher(Bool, tilt_topic, latched_qos)
        self._imu_sub = self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self._joy_sub = self.create_subscription(
            WirelessController, joystick_topic, self._on_joystick, 10
        )

        self._over_since = None
        self._latched = False
        self._warn_active = False
        self._current_tilt = 0.0
        self._last_keys = 0
        self._publish_estop(False)

        if self._recovery_ack_mask == 0:
            self.get_logger().warning(
                'recovery_ack_key_mask is 0; tilt estop cannot be cleared from controller'
            )
        self.get_logger().info(
            f'tilt_monitor: imu={imu_topic} estop_topic={tilt_topic} '
            f'warn={math.degrees(self._warn_tilt):.1f}deg '
            f'estop={math.degrees(self._estop_tilt):.1f}deg '
            f'clear={math.degrees(self._clear_tilt):.1f}deg'
        )

    def _on_imu(self, msg):
        roll, pitch = self._roll_pitch(msg.orientation)
        self._current_tilt = max(abs(roll), abs(pitch))
        now = self.get_clock().now()

        if self._current_tilt >= self._warn_tilt and not self._warn_active:
            self._warn_active = True
            self.get_logger().warning(
                f'Body tilt warning: {math.degrees(self._current_tilt):.1f} deg'
            )
        elif self._current_tilt < self._clear_tilt and self._warn_active:
            self._warn_active = False
            self.get_logger().info('Body tilt returned below warning threshold')

        if self._latched:
            return

        if self._current_tilt >= self._estop_tilt:
            if self._over_since is None:
                self._over_since = now
            over_age = (now - self._over_since).nanoseconds * 1e-9
            if over_age >= self._estop_hold:
                self._latched = True
                self._publish_estop(True)
                self.get_logger().error(
                    f'Tilt E-STOP latched at {math.degrees(self._current_tilt):.1f} deg'
                )
        else:
            self._over_since = None

    def _on_joystick(self, msg):
        previous_keys = self._last_keys
        self._last_keys = int(msg.keys)
        pressed = self._last_keys & ~previous_keys
        if should_clear_tilt_estop(
            self._latched,
            self._current_tilt,
            self._clear_tilt,
            self._recovery_ack_mask,
            pressed,
        ):
            self._latched = False
            self._over_since = None
            self._publish_estop(False)
            self.get_logger().info('Tilt E-STOP cleared by controller acknowledgment')

    def _publish_estop(self, active):
        msg = Bool()
        msg.data = bool(active)
        self._tilt_pub.publish(msg)

    @staticmethod
    def _roll_pitch(q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        return roll, pitch


def main(args=None):
    rclpy.init(args=args)
    node = TiltMonitor()
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
