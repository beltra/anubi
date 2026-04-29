#!/usr/bin/env python3
"""Drive the indoor-office validation person along a repeatable Gazebo path."""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class DynamicPersonDriver(Node):
    """Moves the online person mesh model through Gazebo's set_pose service."""

    def __init__(self) -> None:
        super().__init__("dynamic_person_driver")

        self.declare_parameter("world_name", "indoor_office")
        self.declare_parameter("entity_name", "dynamic_office_person")
        self.declare_parameter(
            "path_xy",
            [-5.0, -3.0, -1.2, -3.0, 0.0, -2.0, 0.5, -0.4, 0.5, 2.7, 4.5, 2.7],
        )
        self.declare_parameter("speed_mps", 0.7)
        self.declare_parameter("update_rate_hz", 12.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("start_delay_s", 0.0)

        self._world_name = str(self.get_parameter("world_name").value)
        self._entity_name = str(self.get_parameter("entity_name").value)
        self._path = self._parse_path(self.get_parameter("path_xy").value)
        self._speed_mps = max(0.01, float(self.get_parameter("speed_mps").value))
        self._z = float(self.get_parameter("z").value)
        self._loop = bool(self.get_parameter("loop").value)
        self._start_delay_s = max(0.0, float(self.get_parameter("start_delay_s").value))

        update_rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self._segment_index = 0
        self._segment_progress = 0.0
        self._last_time = self.get_clock().now()
        self._start_time = self._last_time
        self._pending_future = None
        self._warned_waiting = False

        self._client = self.create_client(SetEntityPose, f"/world/{self._world_name}/set_pose")
        self._timer = self.create_timer(1.0 / update_rate_hz, self._tick)

        self.get_logger().info(
            f"Driving Gazebo validation person '{self._entity_name}' "
            f"through {len(self._path)} waypoints at {self._speed_mps:.2f} m/s"
        )

    def _parse_path(self, raw_values) -> List[Tuple[float, float]]:
        values = [float(v) for v in raw_values]
        if len(values) < 4 or len(values) % 2 != 0:
            raise ValueError("path_xy must contain an even number of values for at least 2 points")
        return [(values[i], values[i + 1]) for i in range(0, len(values), 2)]

    def _tick(self) -> None:
        if not self._client.service_is_ready():
            if not self._warned_waiting:
                self.get_logger().warn(f"Waiting for Gazebo service {self._client.srv_name}")
                self._warned_waiting = True
            return

        now = self.get_clock().now()
        elapsed_from_start = (now - self._start_time).nanoseconds / 1e9
        if elapsed_from_start < self._start_delay_s:
            self._last_time = now
            return

        dt = max(0.0, min(0.25, (now - self._last_time).nanoseconds / 1e9))
        self._last_time = now

        pose = self._advance(dt)
        if pose is None:
            return
        x, y, yaw = pose
        self._set_pose(x, y, yaw)

    def _advance(self, dt: float) -> Optional[Tuple[float, float, float]]:
        while True:
            start = self._path[self._segment_index]
            end = self._path[self._segment_index + 1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                if not self._next_segment():
                    return None
                continue

            self._segment_progress += self._speed_mps * dt
            if self._segment_progress <= length:
                ratio = self._segment_progress / length
                x = start[0] + dx * ratio
                y = start[1] + dy * ratio
                return x, y, math.atan2(dy, dx)

            self._segment_progress -= length
            if not self._next_segment():
                return end[0], end[1], math.atan2(dy, dx)

    def _next_segment(self) -> bool:
        self._segment_index += 1
        if self._segment_index < len(self._path) - 1:
            return True
        if not self._loop:
            self._segment_index = len(self._path) - 2
            return False
        self._path.append(self._path[0])
        self._segment_index = len(self._path) - 2
        return True

    def _set_pose(self, x: float, y: float, yaw: float) -> None:
        if self._pending_future is not None and not self._pending_future.done():
            return

        req = SetEntityPose.Request()
        req.entity.name = self._entity_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = self._z
        req.pose.orientation.z = math.sin(yaw * 0.5)
        req.pose.orientation.w = math.cos(yaw * 0.5)
        self._pending_future = self._client.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicPersonDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
