#!/usr/bin/env python3
"""Final command arbiter before the robot hardware bridge.

Gating rules (first match wins):
  1. E-stop, tilt estop, or joystick override active -> zero velocity
  2. No input command within cmd_timeout seconds -> zero velocity
  3. Otherwise -> forward the selected command source

Enable via go2_bringup.launch.py use_safety:=true.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from robot_common_interfaces.msg import RobotStatus
from std_msgs.msg import Bool, String


def select_command_source(estop, tilt_estop, joystick_override, source_freshness):
    if estop:
        return 'estop'
    if tilt_estop:
        return 'tilt_estop'
    if joystick_override:
        return 'joystick_override'
    for source in ['recovery', 'assisted_teleop', 'manual', 'nav']:
        if source_freshness.get(source) == 'fresh':
            return source
    if any(state == 'stale' for state in source_freshness.values()):
        return 'watchdog'
    return 'waiting_for_cmd'


class VelocityLimiter(Node):

    def __init__(self):
        super().__init__('velocity_limiter')

        self.declare_parameter('cmd_timeout',  0.5)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_in',   '/cmd_vel')
        self.declare_parameter('cmd_vel_out',  '/cmd_vel_safe')
        self.declare_parameter('status_topic', '/robot_status')
        self.declare_parameter('joystick_override_topic', '/joystick_override')
        self.declare_parameter('tilt_estop_topic', '/safety/tilt_estop')
        self.declare_parameter('command_source_topic', '/safety/command_source')
        self.declare_parameter('enable_manual_source', False)
        self.declare_parameter('manual_cmd_vel_in', '/cmd_vel_manual_collision_free')
        self.declare_parameter('enable_assisted_source', False)
        self.declare_parameter('assisted_cmd_vel_in', '/cmd_vel_assisted_collision_free')
        self.declare_parameter('enable_recovery_source', False)
        self.declare_parameter('recovery_cmd_vel_in', '/cmd_vel_recovery_collision_free')

        self._cmd_timeout  = self.get_parameter('cmd_timeout').value
        publish_rate       = self.get_parameter('publish_rate').value
        cmd_vel_in         = self.get_parameter('cmd_vel_in').value
        cmd_vel_out        = self.get_parameter('cmd_vel_out').value
        status_topic       = self.get_parameter('status_topic').value
        joystick_topic     = self.get_parameter('joystick_override_topic').value
        tilt_topic         = self.get_parameter('tilt_estop_topic').value
        source_topic       = self.get_parameter('command_source_topic').value

        self._pub = self.create_publisher(Twist, cmd_vel_out, 10)
        self._source_pub = self.create_publisher(String, source_topic, 10)
        self._commands = {}
        self._source_subs = [
            self.create_subscription(
                Twist,
                cmd_vel_in,
                lambda msg: self._on_source_cmd('nav', msg),
                10,
            )
        ]
        self._add_optional_source('manual', 'enable_manual_source', 'manual_cmd_vel_in')
        self._add_optional_source(
            'assisted_teleop',
            'enable_assisted_source',
            'assisted_cmd_vel_in',
        )
        self._add_optional_source('recovery', 'enable_recovery_source', 'recovery_cmd_vel_in')
        self._status_sub = None
        if status_topic:
            self._status_sub = self.create_subscription(
                RobotStatus, status_topic, self._on_status, 10
            )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._joystick_sub = self.create_subscription(
            Bool, joystick_topic, self._on_joystick_override, latched_qos
        )
        self._tilt_sub = self.create_subscription(
            Bool, tilt_topic, self._on_tilt_estop, latched_qos
        )
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self._estop         = False
        self._tilt_estop    = False
        self._joystick_override = False
        self._prev_estop    = False
        self._prev_tilt_estop = False
        self._prev_joystick_override = False
        self._prev_watchdog = False
        self._prev_source = None

        status_mode = status_topic if status_topic else "disabled"
        self.get_logger().info(
            f'velocity_limiter: {cmd_vel_in} -> {cmd_vel_out}  '
            f'watchdog={self._cmd_timeout}s  rate={publish_rate}Hz  '
            f'status_topic={status_mode} joystick_topic={joystick_topic} '
            f'tilt_topic={tilt_topic}'
        )

    def _add_optional_source(self, source_name, enabled_param, topic_param):
        if not bool(self.get_parameter(enabled_param).value):
            return
        topic = self.get_parameter(topic_param).value
        self._source_subs.append(
            self.create_subscription(
                Twist,
                topic,
                lambda msg, name=source_name: self._on_source_cmd(name, msg),
                10,
            )
        )
        self.get_logger().info(f'enabled command source {source_name}: {topic}')

    def _on_source_cmd(self, source_name, msg: Twist):
        self._commands[source_name] = (msg, self.get_clock().now())

    def _on_status(self, msg: RobotStatus):
        self._estop = msg.estop_active

    def _on_joystick_override(self, msg: Bool):
        self._joystick_override = bool(msg.data)

    def _on_tilt_estop(self, msg: Bool):
        self._tilt_estop = bool(msg.data)

    def _on_timer(self):
        zero = Twist()

        source_freshness = {}
        for source_name, (_, stamp) in self._commands.items():
            age_sec = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            source_freshness[source_name] = (
                'stale' if age_sec > self._cmd_timeout else 'fresh'
            )

        source = select_command_source(
            self._estop,
            self._tilt_estop,
            self._joystick_override,
            source_freshness,
        )
        cmd = self._commands[source][0] if source in self._commands else zero

        self._log_transitions(source == 'watchdog', source)
        self._pub.publish(cmd)
        self._publish_source(source)

    def _log_transitions(self, watchdog_active, source):
        if self._estop and not self._prev_estop:
            self.get_logger().warning('E-STOP active: blocking velocity commands')
        elif not self._estop and self._prev_estop:
            self.get_logger().info('E-STOP cleared')
        self._prev_estop = self._estop

        if self._tilt_estop and not self._prev_tilt_estop:
            self.get_logger().warning('Tilt E-STOP active: blocking velocity commands')
        elif not self._tilt_estop and self._prev_tilt_estop:
            self.get_logger().info('Tilt E-STOP cleared')
        self._prev_tilt_estop = self._tilt_estop

        if self._joystick_override and not self._prev_joystick_override:
            self.get_logger().warning('Joystick override active: blocking autonomy commands')
        elif not self._joystick_override and self._prev_joystick_override:
            self.get_logger().info('Joystick override cleared')
        self._prev_joystick_override = self._joystick_override

        if watchdog_active and not self._prev_watchdog:
            self.get_logger().warning(
                f'velocity_limiter watchdog: no command for {self._cmd_timeout:.2f}s'
            )
        elif not watchdog_active and self._prev_watchdog:
            self.get_logger().info('velocity_limiter watchdog cleared')
        self._prev_watchdog = watchdog_active

        if source != self._prev_source:
            self.get_logger().info(f'velocity_limiter source: {source}')
            self._prev_source = source

    def _publish_source(self, source):
        msg = String()
        msg.data = source
        self._source_pub.publish(msg)


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
