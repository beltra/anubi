#!/usr/bin/env python3

import math
from collections import deque
from typing import Deque, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CloudSelfFilter(Node):
    """Mask robot-body and below-floor returns from a point cloud."""

    def __init__(self) -> None:
        super().__init__("cloud_self_filter")

        self.declare_parameter("input_topic", "/lidar/points")
        self.declare_parameter("output_topic", "/lidar/points_filtered")
        self.declare_parameter("lidar_x", 0.28945)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_z", -0.056825)
        self.declare_parameter("base_z", 0.28)
        self.declare_parameter("min_world_z", 0.03)
        self.declare_parameter("robot_min_x", -0.40)
        self.declare_parameter("robot_max_x", 0.38)
        self.declare_parameter("robot_min_y", -0.26)
        self.declare_parameter("robot_max_y", 0.26)
        self.declare_parameter("robot_min_z", -0.42)
        self.declare_parameter("robot_max_z", 0.18)
        self.declare_parameter("min_radius", 0.12)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_timeout_sec", 0.25)

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
        self._imu_timeout_sec = float(self.get_parameter("imu_timeout_sec").value)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)

        self._imu_history: Deque[Imu] = deque(maxlen=50)

        cloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._publisher = self.create_publisher(PointCloud2, output_topic, cloud_qos)
        self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, imu_topic, self._imu_cb, qos_profile_sensor_data
        )

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_history.append(msg)

    def _nearest_imu(self, stamp_sec: float) -> Optional[Imu]:
        if not self._imu_history:
            return None
        best = min(
            self._imu_history,
            key=lambda m: abs(_stamp_to_sec(m.header.stamp) - stamp_sec),
        )
        if abs(_stamp_to_sec(best.header.stamp) - stamp_sec) > self._imu_timeout_sec:
            return None
        return best

    def _world_z_row(self, imu: Optional[Imu]) -> Optional[np.ndarray]:
        """Return the world-frame z row of R_world_base from the IMU orientation quaternion.

        The IMU quaternion encodes the rotation from base_link to world (z-up ENU).
        The third row of the derived rotation matrix, dotted with a point in
        base_link frame and offset by base_z, gives the estimated world-frame z.
        """
        if imu is None:
            return None
        q = imu.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        # Third row of R (body→world) from unit quaternion:
        # R[2] = [2(xz − yw),  2(yz + xw),  1 − 2(x² + y²)]
        return np.array(
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
            dtype=np.float64,
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

    def _below_ground_gate(
        self, x: float, y: float, z: float, R_z_row: Optional[np.ndarray]
    ) -> bool:
        """Return True if the point's estimated world z is below min_world_z.

        The L1 LiDAR joint has rpy="π 0 0" (Rx(π)), so the lidar_link z-axis
        points physically downward. Floor returns therefore arrive at positive z
        in lidar_link frame. The transform to base_link inverts y and z:

            x_b =  x_l + lidar_x
            y_b = -y_l + lidar_y
            z_b = -z_l + lidar_z   ← Rx(π) negates z

        When the IMU orientation is available, the full tilt-corrected world z is
        computed using the body rotation quaternion so that floor returns are
        rejected regardless of pitch and roll during gait. Without IMU data the
        formula falls back to a level-robot static estimate.
        """
        x_b = x + self._lidar_x
        y_b = -y + self._lidar_y
        z_b = -z + self._lidar_z

        if R_z_row is not None:
            world_z = (
                float(R_z_row[0]) * x_b
                + float(R_z_row[1]) * y_b
                + float(R_z_row[2]) * z_b
                + self._base_z
            )
        else:
            world_z = z_b + self._base_z

        return world_z < self._min_world_z

    def _cloud_cb(self, msg: PointCloud2) -> None:
        cloud_stamp_sec = _stamp_to_sec(msg.header.stamp)
        imu = self._nearest_imu(cloud_stamp_sec)
        R_z_row = self._world_z_row(imu)

        filtered_points = []

        for point in point_cloud2.read_points(msg, field_names=None, skip_nans=False):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                if math.sqrt(x * x + y * y + z * z) < self._min_radius:
                    continue
                if self._below_ground_gate(x, y, z, R_z_row):
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
