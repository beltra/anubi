#!/usr/bin/env python3
"""
cloud_self_filter.py
Filter robot-body returns from a LiDAR PointCloud2 and optionally
gravity-align the output to the lidar_stabilized frame.

Why simulation-only?
  The real Unitree L1 has a hardware minimum range of 0.8 m.  The Go2's
  body and legs are always within ~0.4 m of the sensor, so the hardware
  never returns self-body points.  Simulation (Gazebo Fortress gpu_lidar)
  uses range_min=0.1 m for stability, so body returns appear and must be
  removed in software before reaching the costmap / SLAM.

  On the real robot, distortion_corrector.py handles gravity alignment
  (no self-body filter needed).

Gravity alignment mirrors the behaviour of distortion_corrector.py so that
the simulation STVL costmap receives the same lidar_stabilized frame
representation as the real robot.
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
        # Static mount rotation from base_link to lidar_link.
        # Real Go2 + L1 uses upside-down mount with ~15 deg forward pitch.
        self.declare_parameter(
            "sensor_mount_rpy", [3.14159265358979, -0.26179938779915, 0.0]
        )
        self.declare_parameter("base_z", 0.28)
        self.declare_parameter("filter_floor", False)
        self.declare_parameter("min_world_z", 0.12)
        self.declare_parameter("rear_floor_adaptive", True)
        self.declare_parameter("rear_floor_start_x", -0.2)
        self.declare_parameter("rear_floor_z_per_meter", 0.02)
        self.declare_parameter("rear_floor_max_extra_z", 0.08)
        # Self-body bounding box in base_link frame.  Must NOT extend below
        # the ground plane (z ~ -0.28 in base_link at standing height 0.28 m)
        # or floor returns are deleted as "body" returns.
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
        self._base_z = float(self.get_parameter("base_z").value)
        self._filter_floor = bool(self.get_parameter("filter_floor").value)
        self._min_world_z = float(self.get_parameter("min_world_z").value)
        self._rear_floor_adaptive = bool(self.get_parameter("rear_floor_adaptive").value)
        self._rear_floor_start_x = float(self.get_parameter("rear_floor_start_x").value)
        self._rear_floor_z_per_meter = float(
            self.get_parameter("rear_floor_z_per_meter").value
        )
        self._rear_floor_max_extra_z = float(
            self.get_parameter("rear_floor_max_extra_z").value
        )
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

    def _world_z_row(self, imu: Optional[Imu]) -> Optional[np.ndarray]:
        """Return the world-frame z row of R_world_base from the IMU orientation quaternion."""
        if imu is None:
            return None
        q = imu.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        # Third row of R (body->world) from unit quaternion:
        # R[2] = [2(xz - yw),  2(yz + xw),  1 - 2(x^2 + y^2)]
        return np.array(
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
            dtype=np.float64,
        )

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
        R_z_row = self._world_z_row(imu)

        # Parse cloud into numpy arrays (zero-copy view)
        dt_struct = build_pc2_dtype(msg)
        raw = np.frombuffer(msg.data, dtype=dt_struct)

        x = raw["x"].astype(np.float64)
        y = raw["y"].astype(np.float64)
        z = raw["z"].astype(np.float64)

        finite_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        # Radius filter (near-body clutter)
        radius_sq = x * x + y * y + z * z
        radius_mask = finite_mask & (radius_sq < self._min_radius ** 2)

        # Self-body box filter
        # Transform finite sensor points to base_link frame using the
        # static mount rotation and translation.
        x_b = np.full_like(x, np.nan)
        y_b = np.full_like(y, np.nan)
        z_b = np.full_like(z, np.nan)
        if np.any(finite_mask):
            pts_b = self._R_mount @ np.stack(
                [x[finite_mask], y[finite_mask], z[finite_mask]], axis=0
            )  # 3 x N_valid
            x_b[finite_mask] = pts_b[0] + self._lidar_x
            y_b[finite_mask] = pts_b[1] + self._lidar_y
            z_b[finite_mask] = pts_b[2] + self._lidar_z

        self_box_mask = (
            finite_mask
            & (x_b >= self._robot_min_x) & (x_b <= self._robot_max_x)
            & (y_b >= self._robot_min_y) & (y_b <= self._robot_max_y)
            & (z_b >= self._robot_min_z) & (z_b <= self._robot_max_z)
        )

        # Below-ground gate (IMU-tilt-corrected)
        floor_mask = np.zeros_like(finite_mask, dtype=bool)
        if self._filter_floor:
            world_z = np.full_like(x, np.inf)
            if np.any(finite_mask):
                if R_z_row is not None:
                    world_z[finite_mask] = (
                        R_z_row[0] * x_b[finite_mask]
                        + R_z_row[1] * y_b[finite_mask]
                        + R_z_row[2] * z_b[finite_mask]
                        + self._base_z
                    )
                else:
                    world_z[finite_mask] = z_b[finite_mask] + self._base_z

            floor_threshold = np.full_like(world_z, self._min_world_z, dtype=np.float64)
            if self._rear_floor_adaptive:
                rear_distance = np.maximum(0.0, self._rear_floor_start_x - x_b)
                rear_extra = np.minimum(
                    rear_distance * self._rear_floor_z_per_meter,
                    self._rear_floor_max_extra_z,
                )
                floor_threshold = floor_threshold + rear_extra

            floor_mask = finite_mask & (world_z < floor_threshold)

        # Combine and remove bad points
        remove = radius_mask | self_box_mask | floor_mask | (~finite_mask)

        out_data = bytearray(msg.data)
        out_arr = np.frombuffer(out_data, dtype=dt_struct)
        nan32 = np.float32(math.nan)

        cx = np.where(remove, nan32, raw["x"].astype(np.float32))
        cy = np.where(remove, nan32, raw["y"].astype(np.float32))
        cz = np.where(remove, nan32, raw["z"].astype(np.float32))

        # Gravity alignment
        # Rotate filtered points to the gravity-levelled lidar_stabilized frame:
        #   P_stab = R_body @ R_mount @ P_lidar_link
        out_frame_id = msg.header.frame_id
        gravity_aligned = False
        if self._gravity_align and imu is not None:
            roll, pitch = roll_pitch_from_imu(imu.orientation)
            R_body = rotation_from_rpy(roll, pitch, 0.0)
            keep = finite_mask & ~remove
            if np.any(keep):
                # Only gravity-align points that survived filtering; NaN points
                # stay NaN and the frame_id changes for the whole cloud.
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
        out_msg.is_dense = False   # NaN points may be present
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
