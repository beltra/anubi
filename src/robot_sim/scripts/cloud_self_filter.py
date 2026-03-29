#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudSelfFilter(Node):
    """Mask the simulated robot body out of the voxel-layer cloud feed."""

    def __init__(self) -> None:
        super().__init__("cloud_self_filter")

        self.declare_parameter("input_topic", "/lidar/points")
        self.declare_parameter("output_topic", "/lidar/points_filtered")
        self.declare_parameter("lidar_x", 0.25)
        self.declare_parameter("lidar_y", -0.038)
        self.declare_parameter("lidar_z", -0.03)
        self.declare_parameter("base_z", 0.335)
        self.declare_parameter("min_world_z", 0.06)
        self.declare_parameter("robot_min_x", -0.40)
        self.declare_parameter("robot_max_x", 0.38)
        self.declare_parameter("robot_min_y", -0.26)
        self.declare_parameter("robot_max_y", 0.26)
        self.declare_parameter("robot_min_z", -0.42)
        self.declare_parameter("robot_max_z", 0.18)
        self.declare_parameter("min_radius", 0.25)

        self._lidar_x = float(self.get_parameter("lidar_x").value)
        self._lidar_y = float(self.get_parameter("lidar_y").value)
        self._lidar_z = float(self.get_parameter("lidar_z").value)
        self._base_z = float(self.get_parameter("base_z").value)
        self._min_world_z = float(self.get_parameter("min_world_z").value)
        self._robot_min_x = float(self.get_parameter("robot_min_x").value)
        self._robot_max_x = float(self.get_parameter("robot_max_x").value)
        self._robot_min_y = float(self.get_parameter("robot_min_y").value)
        self._robot_max_y = float(self.get_parameter("robot_max_y").value)
        self._robot_min_z = float(self.get_parameter("robot_min_z").value)
        self._robot_max_z = float(self.get_parameter("robot_max_z").value)
        self._min_radius = float(self.get_parameter("min_radius").value)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self._publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, qos_profile_sensor_data
        )

    def _inside_self_box(self, x: float, y: float, z: float) -> bool:
        x_base = x + self._lidar_x
        y_base = y + self._lidar_y
        z_base = z + self._lidar_z

        return (
            self._robot_min_x <= x_base <= self._robot_max_x
            and self._robot_min_y <= y_base <= self._robot_max_y
            and self._robot_min_z <= z_base <= self._robot_max_z
        )

    def _below_ground_gate(self, z: float) -> bool:
        z_base = z + self._lidar_z
        return (self._base_z + z_base) < self._min_world_z

    def _cloud_cb(self, msg: PointCloud2) -> None:
        filtered_points = []

        for point in point_cloud2.read_points(msg, field_names=None, skip_nans=False):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                if math.sqrt(x * x + y * y + z * z) < self._min_radius:
                    continue
                if self._below_ground_gate(z):
                    continue
                if self._inside_self_box(x, y, z):
                    continue

            filtered_points.append(tuple(point))

        filtered = point_cloud2.create_cloud(msg.header, msg.fields, filtered_points)
        filtered.is_dense = msg.is_dense
        self._publisher.publish(filtered)


def main() -> None:
    rclpy.init()
    node = CloudSelfFilter()
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
