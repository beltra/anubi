#!/usr/bin/env python3
"""Print controller key transitions for recovery_ack_key_mask calibration."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from unitree_go.msg import WirelessController


class WirelessKeyInspector(Node):

    def __init__(self):
        super().__init__('wireless_key_inspector')
        self.declare_parameter('joystick_topic', '/wirelesscontroller')
        joystick_topic = self.get_parameter('joystick_topic').value
        self._last_keys = None
        self._sub = self.create_subscription(
            WirelessController, joystick_topic, self._on_joystick, 10
        )
        self.get_logger().info(f'Listening for key transitions on {joystick_topic}')

    def _on_joystick(self, msg):
        keys = int(msg.keys)
        if self._last_keys is None:
            self._last_keys = keys
            return
        if keys == self._last_keys:
            return

        pressed = keys & ~self._last_keys
        released = self._last_keys & ~keys
        self._last_keys = keys
        if pressed:
            self.get_logger().info(
                f'keys=0x{keys:04x} pressed=0x{pressed:04x}'
            )
        if released:
            self.get_logger().info(
                f'keys=0x{keys:04x} released=0x{released:04x}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WirelessKeyInspector()
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
