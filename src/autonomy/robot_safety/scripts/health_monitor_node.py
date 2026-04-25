#!/usr/bin/env python3
"""Publish safety-critical topic and TF health diagnostics."""

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def diagnostic_level(value) -> int:
    if isinstance(value, bytes):
        return int.from_bytes(value, byteorder="little")
    return int(value)


def diagnostic_level_msg(value: int) -> bytes:
    return bytes([int(value)])


OK = diagnostic_level(DiagnosticStatus.OK)
WARN = diagnostic_level(DiagnosticStatus.WARN)
ERROR = diagnostic_level(DiagnosticStatus.ERROR)


@dataclass(frozen=True)
class FreshnessResult:
    level: int
    message: str
    age_sec: Optional[float]


@dataclass(frozen=True)
class MapJumpResult:
    level: int
    message: str
    translation_delta_m: float
    yaw_delta_rad: float


def freshness_status(
    last_seen_sec: Optional[float],
    now_sec: float,
    warn_after_sec: float,
    error_after_sec: float,
) -> FreshnessResult:
    if last_seen_sec is None:
        return FreshnessResult(ERROR, "no messages observed", None)

    age_sec = max(0.0, now_sec - last_seen_sec)
    if age_sec >= error_after_sec:
        return FreshnessResult(ERROR, "stale", age_sec)
    if age_sec >= warn_after_sec:
        return FreshnessResult(WARN, "delayed", age_sec)
    return FreshnessResult(OK, "fresh", age_sec)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def map_jump_status(
    previous_tf,
    current_tf,
    warn_translation_m: float,
    error_translation_m: float,
    warn_yaw_rad: float,
    error_yaw_rad: float,
) -> MapJumpResult:
    if previous_tf is None or current_tf is None:
        return MapJumpResult(OK, "waiting for map transform history", 0.0, 0.0)

    prev_t = previous_tf.transform.translation
    curr_t = current_tf.transform.translation
    dx = curr_t.x - prev_t.x
    dy = curr_t.y - prev_t.y
    dz = curr_t.z - prev_t.z
    translation_delta_m = math.sqrt(dx * dx + dy * dy + dz * dz)

    prev_yaw = yaw_from_quaternion(previous_tf.transform.rotation)
    curr_yaw = yaw_from_quaternion(current_tf.transform.rotation)
    yaw_delta_rad = abs(shortest_angle_delta(curr_yaw, prev_yaw))

    if translation_delta_m >= error_translation_m or yaw_delta_rad >= error_yaw_rad:
        return MapJumpResult(
            ERROR,
            "map transform jump exceeds error threshold",
            translation_delta_m,
            yaw_delta_rad,
        )
    if translation_delta_m >= warn_translation_m or yaw_delta_rad >= warn_yaw_rad:
        return MapJumpResult(
            WARN,
            "map transform jump exceeds warning threshold",
            translation_delta_m,
            yaw_delta_rad,
        )
    return MapJumpResult(OK, "map transform stable", translation_delta_m, yaw_delta_rad)


def parse_tf_pair(value: str) -> tuple[str, str]:
    if "->" in value:
        parent, child = value.split("->", 1)
    elif ":" in value:
        parent, child = value.split(":", 1)
    else:
        raise ValueError(f"invalid TF pair '{value}', expected parent->child")
    parent = parent.strip()
    child = child.strip()
    if not parent or not child:
        raise ValueError(f"invalid TF pair '{value}', expected parent->child")
    return parent, child


class HealthMonitor(Node):

    def __init__(self):
        super().__init__("health_monitor")

        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("sensor_warn_after_sec", 1.0)
        self.declare_parameter("sensor_error_after_sec", 3.0)
        self.declare_parameter("command_warn_after_sec", 0.75)
        self.declare_parameter("command_error_after_sec", 1.5)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("map_jump_warn_translation_m", 0.50)
        self.declare_parameter("map_jump_error_translation_m", 1.00)
        self.declare_parameter("map_jump_warn_yaw_rad", 0.35)
        self.declare_parameter("map_jump_error_yaw_rad", 0.70)
        self.declare_parameter("lidar_topic", "/lidar/points_undistorted")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter(
            "required_tf_pairs",
            [
                "map->odom",
                "odom->base_footprint",
                "base_footprint->base_link",
                "base_link->lidar_link",
                "base_link->imu_link",
                "base_footprint->lidar_stabilized",
            ],
        )

        self._sensor_warn = float(self.get_parameter("sensor_warn_after_sec").value)
        self._sensor_error = float(self.get_parameter("sensor_error_after_sec").value)
        self._command_warn = float(self.get_parameter("command_warn_after_sec").value)
        self._command_error = float(self.get_parameter("command_error_after_sec").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._map_warn_translation = float(
            self.get_parameter("map_jump_warn_translation_m").value
        )
        self._map_error_translation = float(
            self.get_parameter("map_jump_error_translation_m").value
        )
        self._map_warn_yaw = float(self.get_parameter("map_jump_warn_yaw_rad").value)
        self._map_error_yaw = float(self.get_parameter("map_jump_error_yaw_rad").value)

        self._last_seen = {}
        self._topic_specs = [
            (
                "lidar_points_undistorted",
                self.get_parameter("lidar_topic").value,
                PointCloud2,
                "sensor",
            ),
            ("scan", self.get_parameter("scan_topic").value, LaserScan, "sensor"),
            ("odom", self.get_parameter("odom_topic").value, Odometry, "sensor"),
            ("cmd_vel", self.get_parameter("cmd_vel_topic").value, Twist, "command"),
            ("cmd_vel_safe", self.get_parameter("cmd_vel_safe_topic").value, Twist, "command"),
        ]
        self._subs = [
            self.create_subscription(
                msg_type,
                topic,
                lambda _msg, key=key: self._mark_seen(key),
                10,
            )
            for key, topic, msg_type, _group in self._topic_specs
        ]

        self._tf_pairs = [
            parse_tf_pair(value)
            for value in list(self.get_parameter("required_tf_pairs").value)
        ]
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._previous_map_tf = None
        self._latest_map_jump = MapJumpResult(
            OK, "waiting for map transform history", 0.0, 0.0
        )

        diagnostics_topic = self.get_parameter("diagnostics_topic").value
        self._pub = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / publish_rate, self._publish_diagnostics)

        self.get_logger().info(
            f"health_monitor publishing {diagnostics_topic} for "
            f"{len(self._topic_specs)} topics and {len(self._tf_pairs)} TF links"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _mark_seen(self, key: str) -> None:
        self._last_seen[key] = self._now_sec()

    def _topic_status(self, key: str, topic: str, group: str, now_sec: float):
        if group == "command":
            warn_after = self._command_warn
            error_after = self._command_error
        else:
            warn_after = self._sensor_warn
            error_after = self._sensor_error

        result = freshness_status(
            self._last_seen.get(key),
            now_sec,
            warn_after,
            error_after,
        )
        values = [KeyValue(key="topic", value=topic)]
        if result.age_sec is not None:
            values.append(KeyValue(key="age_sec", value=f"{result.age_sec:.3f}"))
        return self._status(
            name=f"anubi/topic/{key}",
            level=result.level,
            message=result.message,
            values=values,
        )

    def _tf_status(self, parent: str, child: str):
        try:
            ok = self._tf_buffer.can_transform(
                parent,
                child,
                Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            return self._status(
                name=f"anubi/tf/{parent}_to_{child}",
                level=ERROR,
                message=str(exc),
                values=[KeyValue(key="parent", value=parent), KeyValue(key="child", value=child)],
            )

        level = OK if ok else ERROR
        message = "available" if ok else "missing"
        return self._status(
            name=f"anubi/tf/{parent}_to_{child}",
            level=level,
            message=message,
            values=[KeyValue(key="parent", value=parent), KeyValue(key="child", value=child)],
        )

    def _update_map_jump_status(self):
        try:
            current = self._tf_buffer.lookup_transform("map", "odom", Time())
        except TransformException:
            return self._status(
                name="anubi/map_jump/map_to_odom",
                level=ERROR,
                message="map->odom transform unavailable",
                values=[],
            )

        result = map_jump_status(
            self._previous_map_tf,
            current,
            self._map_warn_translation,
            self._map_error_translation,
            self._map_warn_yaw,
            self._map_error_yaw,
        )
        self._previous_map_tf = current
        self._latest_map_jump = result

        return self._status(
            name="anubi/map_jump/map_to_odom",
            level=result.level,
            message=result.message,
            values=[
                KeyValue(
                    key="translation_delta_m",
                    value=f"{result.translation_delta_m:.3f}",
                ),
                KeyValue(key="yaw_delta_rad", value=f"{result.yaw_delta_rad:.3f}"),
            ],
        )

    def _publish_diagnostics(self):
        now_sec = self._now_sec()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.extend(
            self._topic_status(key, topic, group, now_sec)
            for key, topic, _msg_type, group in self._topic_specs
        )
        msg.status.extend(self._tf_status(parent, child) for parent, child in self._tf_pairs)
        msg.status.append(self._update_map_jump_status())
        self._pub.publish(msg)

    @staticmethod
    def _status(name: str, level: int, message: str, values):
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = "anubi"
        status.level = diagnostic_level_msg(level)
        status.message = message
        status.values = values
        return status


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
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
