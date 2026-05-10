#!/usr/bin/env python3
"""
cloud_self_filter.py -- Debug self-body filter for LiDAR PointCloud2.

The real Unitree L1 has a hardware minimum range of 0.8 m; the Go2 body is always
within ~0.4 m of the sensor, so self-body returns never appear on hardware.
Gazebo uses range_min=0.1 m, so body returns may be removed in software.
This script is debug-only; the runtime pipeline uses the C++ distortion_corrector.
"""

import math
from collections import deque
from typing import Deque, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from robot_nav.pointcloud_utils import (
    build_pc2_dtype,
    gravity_align_cloud,
    roll_pitch_from_imu,
    rotation_from_rpy,
    stamp_to_sec,
)


class CloudSelfFilter(Node):
    """Mask robot-body and below-floor returns from a point cloud."""

    def __init__(self) -> None:
        super().__init__("cloud_self_filter")

        self.declare_parameter("input_topic", "/lidar/points")
        self.declare_parameter("output_topic", "/lidar/points_filtered")
        self.declare_parameter("lidar_x", 0.28945)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_z", -0.046825)
        # Real /utlidar/cloud frame; inverse maps raw LiDAR points into base axes.
        self.declare_parameter(
            "sensor_mount_rpy", [0.0, -2.88000012056847, 0.0]
        )
        # Self-body bounding box in base_link frame.
        # z lower bound must not reach the ground plane or floor returns are deleted.
        self.declare_parameter("robot_min_x", -0.40)
        self.declare_parameter("robot_max_x", 0.36)
        self.declare_parameter("robot_min_y", -0.20)
        self.declare_parameter("robot_max_y", 0.20)
        self.declare_parameter("robot_min_z", -0.25)
        self.declare_parameter("robot_max_z", 0.10)
        self.declare_parameter("min_radius", 0.12)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_timeout_sec", 0.25)
        self.declare_parameter("gravity_align", False)
        self.declare_parameter("gravity_align_frame", "lidar_stabilized")

        self._lidar_x = float(self.get_parameter("lidar_x").value)
        self._lidar_y = float(self.get_parameter("lidar_y").value)
        self._lidar_z = float(self.get_parameter("lidar_z").value)
        mount_rpy = list(self.get_parameter("sensor_mount_rpy").value)
        self._R_mount = rotation_from_rpy(mount_rpy[0], mount_rpy[1], mount_rpy[2])
        self._robot_min_x = float(self.get_parameter("robot_min_x").value)
        self._robot_max_x = float(self.get_parameter("robot_max_x").value)
        self._robot_min_y = float(self.get_parameter("robot_min_y").value)
        self._robot_max_y = float(self.get_parameter("robot_max_y").value)
        self._robot_min_z = float(self.get_parameter("robot_min_z").value)
        self._robot_max_z = float(self.get_parameter("robot_max_z").value)
        self._min_radius = float(self.get_parameter("min_radius").value)
        self._imu_timeout_sec = float(self.get_parameter("imu_timeout_sec").value)
        self._gravity_align = bool(self.get_parameter("gravity_align").value)
        self._gravity_align_frame = str(self.get_parameter("gravity_align_frame").value)

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
            key=lambda m: abs(stamp_to_sec(m.header.stamp) - stamp_sec),
        )
        if abs(stamp_to_sec(best.header.stamp) - stamp_sec) > self._imu_timeout_sec:
            return None
        return best

    def _cloud_cb(self, msg: PointCloud2) -> None:
        n_points = msg.width * msg.height
        if n_points == 0:
            self._publisher.publish(msg)
            return

        field_names = [f.name for f in msg.fields]
        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            self._publisher.publish(msg)
            return

        cloud_stamp_sec = stamp_to_sec(msg.header.stamp)
        imu = self._nearest_imu(cloud_stamp_sec)
        dt_struct = build_pc2_dtype(msg)
        raw = np.frombuffer(msg.data, dtype=dt_struct)

        x = raw["x"].astype(np.float64)
        y = raw["y"].astype(np.float64)
        z = raw["z"].astype(np.float64)

        finite_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        radius_sq = x * x + y * y + z * z
        radius_mask = finite_mask & (radius_sq < self._min_radius ** 2)

        x_b = np.full_like(x, np.nan)
        y_b = np.full_like(y, np.nan)
        z_b = np.full_like(z, np.nan)
        if np.any(finite_mask):
            pts_b = self._R_mount @ np.stack(
                [x[finite_mask], y[finite_mask], z[finite_mask]], axis=0
            )
            x_b[finite_mask] = pts_b[0] + self._lidar_x
            y_b[finite_mask] = pts_b[1] + self._lidar_y
            z_b[finite_mask] = pts_b[2] + self._lidar_z

        self_box_mask = (
            finite_mask
            & (x_b >= self._robot_min_x) & (x_b <= self._robot_max_x)
            & (y_b >= self._robot_min_y) & (y_b <= self._robot_max_y)
            & (z_b >= self._robot_min_z) & (z_b <= self._robot_max_z)
        )

        remove = radius_mask | self_box_mask | (~finite_mask)

        out_data = bytearray(msg.data)
        out_arr = np.frombuffer(out_data, dtype=dt_struct)
        nan32 = np.float32(math.nan)

        cx = np.where(remove, nan32, raw["x"].astype(np.float32))
        cy = np.where(remove, nan32, raw["y"].astype(np.float32))
        cz = np.where(remove, nan32, raw["z"].astype(np.float32))

        out_frame_id = msg.header.frame_id
        gravity_aligned = False
        if self._gravity_align and imu is not None:
            roll, pitch = roll_pitch_from_imu(imu.orientation)
            R_body = rotation_from_rpy(roll, pitch, 0.0)
            keep = finite_mask & ~remove
            if np.any(keep):
                cx_keep, cy_keep, cz_keep = gravity_align_cloud(
                    cx, cy, cz, keep, R_body, self._R_mount
                )
                cx = np.where(keep, cx_keep, cx)
                cy = np.where(keep, cy_keep, cy)
                cz = np.where(keep, cz_keep, cz)
            gravity_aligned = True
            out_frame_id = self._gravity_align_frame

        out_arr["x"] = cx
        out_arr["y"] = cy
        out_arr["z"] = cz

        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.header.frame_id = out_frame_id
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.fields = msg.fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = msg.point_step
        out_msg.row_step = msg.row_step
        out_msg.data = bytes(out_data)
        out_msg.is_dense = False
        self._publisher.publish(out_msg)


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
