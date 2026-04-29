#!/usr/bin/env python3
"""Transform and thin LiDAR clouds for dynamic obstacle detection."""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from robot_nav.dynamic_obstacles import (
    cloud_from_points,
    filter_height,
    points_from_cloud,
    transform_points,
    transform_to_matrix,
    voxel_downsample,
)


class DynamicObstaclePreprocessor(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_preprocessor")

        self.declare_parameter("input_topic", "/lidar/points_undistorted")
        self.declare_parameter("output_topic", "/dynamic_obstacles/points_world")
        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("min_z", 0.08)
        self.declare_parameter("max_z", 2.0)
        self.declare_parameter("voxel_size", 0.12)
        self.declare_parameter("max_points", 6000)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("fallback_to_latest_tf", False)
        self.declare_parameter("max_latest_tf_age_sec", 0.05)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._fixed_frame = str(self.get_parameter("fixed_frame").value)
        self._min_z = float(self.get_parameter("min_z").value)
        self._max_z = float(self.get_parameter("max_z").value)
        self._voxel_size = float(self.get_parameter("voxel_size").value)
        self._max_points = int(self.get_parameter("max_points").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._fallback_to_latest_tf = bool(
            self.get_parameter("fallback_to_latest_tf").value
        )
        self._max_latest_tf_age = float(self.get_parameter("max_latest_tf_age_sec").value)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2, self._output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, self._input_topic, self._cloud_cb, qos_profile_sensor_data
        )
        self._last_warn_time = 0.0

        self.get_logger().info(
            f"dynamic_obstacle_preprocessor: {self._input_topic} -> "
            f"{self._output_topic} in {self._fixed_frame}"
        )

    def _warn_throttled(self, message: str) -> None:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - self._last_warn_time > 2.0:
            self._last_warn_time = now
            self.get_logger().warning(message)

    def _lookup_transform(self, msg: PointCloud2):
        msg_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            return self._tf_buffer.lookup_transform(
                self._fixed_frame,
                msg.header.frame_id,
                msg_stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            if not self._fallback_to_latest_tf:
                raise
            try:
                latest = self._tf_buffer.lookup_transform(
                    self._fixed_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self._tf_timeout),
                )
                latest_stamp = rclpy.time.Time.from_msg(latest.header.stamp)
                age = abs((msg_stamp - latest_stamp).nanoseconds) * 1.0e-9
                if age <= self._max_latest_tf_age:
                    return latest
            except TransformException:
                raise exc
            raise exc

    def _cloud_cb(self, msg: PointCloud2) -> None:
        try:
            transform = self._lookup_transform(msg)
        except TransformException as exc:
            self._warn_throttled(f"Skipping dynamic obstacle cloud: TF unavailable ({exc})")
            return

        points = points_from_cloud(msg)
        points = transform_points(points, transform_to_matrix(transform))
        points = filter_height(points, self._min_z, self._max_z)
        points = voxel_downsample(points, self._voxel_size)
        if self._max_points > 0 and points.shape[0] > self._max_points:
            step = max(1, points.shape[0] // self._max_points)
            points = points[::step][: self._max_points]

        out_header = Header()
        out_header.stamp = msg.header.stamp
        out_header.frame_id = self._fixed_frame
        self._publisher.publish(cloud_from_points(out_header, points))


def main() -> None:
    rclpy.init()
    node = DynamicObstaclePreprocessor()
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
