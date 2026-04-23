#!/usr/bin/env python3

import math
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

import rclpy
from nav_msgs.msg import Odometry
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


class DistortionCorrector(Node):
    """Deskew a LiDAR point cloud using odometry twist and optional IMU rates."""

    def __init__(self) -> None:
        super().__init__("distortion_corrector")

        self.declare_parameter("input_topic", "/lidar/points")
        self.declare_parameter("output_topic", "/lidar/points_undistorted")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("use_imu", True)
        self.declare_parameter("use_twist", True)
        self.declare_parameter("use_3d_distortion_correction", True)
        self.declare_parameter("scan_period", 0.1)
        # Static mount rotation base_link -> lidar_link (Go2 + L1: upside-down, -15 deg pitch)
        self.declare_parameter(
            "sensor_mount_rpy", [3.14159265358979, -0.26179938779915, 0.0]
        )
        self.declare_parameter("timestamp_field_name", "")
        self.declare_parameter(
            "timestamp_field_candidates",
            ["time_stamp", "timestamp", "time", "t"],
        )
        self.declare_parameter("timestamp_unit", "auto")
        self.declare_parameter("fallback_mode", "azimuth")
        # cw because Rx(pi) upside-down mount flips arctan2(y,x), reversing apparent scan direction
        self.declare_parameter("azimuth_direction", "cw")
        self.declare_parameter("passthrough_on_missing_state", True)
        self.declare_parameter("state_timeout_sec", 0.25)
        # Drop stale clouds to avoid feeding old data into SLAM and costmaps
        self.declare_parameter("drop_stale_clouds", True)
        self.declare_parameter("max_input_delay_sec", 0.25)
        # Preserve input timestamp so downstream TF lookups use the acquisition time.
        # Using "now" caused 13-20 ms shifts that smeared the SLAM map.
        self.declare_parameter("output_stamp_mode", "now")
        self.declare_parameter("bypass", False)
        # Gravity alignment: rotate deskewed points to lidar_stabilized (R_body @ R_mount_inv).
        # Requires imu_stabilized_tf.py to broadcast the lidar_stabilized TF.
        self.declare_parameter("gravity_align", False)
        self.declare_parameter("gravity_align_frame", "lidar_stabilized")
        # Floor filter: drop gait-induced floor sweeps before they reach STVL costmap
        self.declare_parameter("filter_floor", False)
        self.declare_parameter("min_world_z", 0.12)
        self.declare_parameter("sensor_x_in_base", 0.28945)
        self.declare_parameter("sensor_y_in_base", 0.0)
        self.declare_parameter("sensor_z_in_base", -0.046825)
        self.declare_parameter("base_z_nominal", 0.28)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._use_imu = bool(self.get_parameter("use_imu").value)
        self._use_twist = bool(self.get_parameter("use_twist").value)
        self._use_3d = bool(self.get_parameter("use_3d_distortion_correction").value)
        self._scan_period = float(self.get_parameter("scan_period").value)
        self._timestamp_field_name = str(self.get_parameter("timestamp_field_name").value)
        self._timestamp_field_candidates = list(
            self.get_parameter("timestamp_field_candidates").value
        )
        self._timestamp_unit = str(self.get_parameter("timestamp_unit").value).lower()
        self._fallback_mode = str(self.get_parameter("fallback_mode").value).lower()
        self._azimuth_direction = str(self.get_parameter("azimuth_direction").value).lower()
        self._passthrough_on_missing_state = bool(
            self.get_parameter("passthrough_on_missing_state").value
        )
        self._state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self._drop_stale_clouds = bool(self.get_parameter("drop_stale_clouds").value)
        self._max_input_delay_sec = float(self.get_parameter("max_input_delay_sec").value)
        self._output_stamp_mode = str(self.get_parameter("output_stamp_mode").value).lower()
        self._bypass = bool(self.get_parameter("bypass").value)
        self._gravity_align = bool(self.get_parameter("gravity_align").value)
        self._gravity_align_frame = str(self.get_parameter("gravity_align_frame").value)
        self._filter_floor = bool(self.get_parameter("filter_floor").value)
        self._min_world_z = float(self.get_parameter("min_world_z").value)
        self._sensor_x_in_base = float(self.get_parameter("sensor_x_in_base").value)
        self._sensor_y_in_base = float(self.get_parameter("sensor_y_in_base").value)
        self._sensor_z_in_base = float(self.get_parameter("sensor_z_in_base").value)
        self._base_z_nominal = float(self.get_parameter("base_z_nominal").value)

        mount_rpy = list(self.get_parameter("sensor_mount_rpy").value)
        self._R_mount = rotation_from_rpy(mount_rpy[0], mount_rpy[1], mount_rpy[2])
        self._R_mount_inv = self._R_mount.T

        self._imu_history: Deque[Imu] = deque(maxlen=400)
        self._odom_history: Deque[Odometry] = deque(maxlen=200)
        self._warned_timestamp_mode: Optional[str] = None
        self._warned_passthrough_reason: Optional[str] = None
        self._warned_stale_drop: Optional[str] = None

        cloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._publisher = self.create_publisher(PointCloud2, self._output_topic, cloud_qos)
        self.create_subscription(PointCloud2, self._input_topic, self._cloud_cb, cloud_qos)
        self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, self._imu_topic, self._imu_cb, qos_profile_sensor_data
        )

        self.get_logger().info(
            "Distortion corrector active: "
            f"{self._input_topic} -> {self._output_topic} "
            f"(imu={str(self._use_imu).lower()}, "
            f"3d={str(self._use_3d).lower()}, fallback={self._fallback_mode}, "
            f"stamp={self._output_stamp_mode}, "
            f"gravity_align={str(self._gravity_align).lower()}"
            + (f"->{self._gravity_align_frame}" if self._gravity_align else "")
            + f", floor_filter={str(self._filter_floor).lower()}"
            + (f"(min_z={self._min_world_z:.2f}m)" if self._filter_floor else "")
            + ")"
        )

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_history.append(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_history.append(msg)

    def _nearest_msg(self, history: Deque, stamp_sec: float):
        if not history:
            return None

        best = min(history, key=lambda msg: abs(stamp_to_sec(msg.header.stamp) - stamp_sec))
        best_dt = abs(stamp_to_sec(best.header.stamp) - stamp_sec)
        if best_dt > self._state_timeout_sec:
            return None
        return best

    def _select_state(self, stamp_sec: float):
        odom_msg = self._nearest_msg(self._odom_history, stamp_sec) if self._use_twist else None
        imu_msg = self._nearest_msg(self._imu_history, stamp_sec) if self._use_imu else None

        linear_velocity = np.zeros(3, dtype=np.float64)
        angular_velocity = np.zeros(3, dtype=np.float64)

        if odom_msg is not None:
            twist = odom_msg.twist.twist
            linear_velocity = np.array(
                [twist.linear.x, twist.linear.y, twist.linear.z], dtype=np.float64
            )
            angular_velocity = np.array(
                [twist.angular.x, twist.angular.y, twist.angular.z], dtype=np.float64
            )

        if imu_msg is not None:
            # Prefer IMU angular velocity (higher rate, more accurate)
            angular_velocity = np.array(
                [
                    imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z,
                ],
                dtype=np.float64,
            )

        if not self._use_3d:
            linear_velocity[2] = 0.0
            angular_velocity[0] = 0.0
            angular_velocity[1] = 0.0

        # Transform velocities to sensor frame, including lever-arm correction.
        # Omitting the lever arm (omega x r) causes ~14 mm lateral error per scan
        # at omega_z = 0.5 rad/s -- about 1.8x the L1 noise floor.
        r_sensor = np.array(
            [self._sensor_x_in_base, self._sensor_y_in_base, self._sensor_z_in_base],
            dtype=np.float64,
        )
        linear_sensor = self._R_mount @ (linear_velocity + np.cross(angular_velocity, r_sensor))
        angular_sensor = self._R_mount @ angular_velocity

        return linear_sensor, angular_sensor, odom_msg, imu_msg

    def _resolve_time_field(self, msg: PointCloud2) -> Tuple[Optional[int], Optional[str]]:
        field_names = [field.name for field in msg.fields]
        if self._timestamp_field_name and self._timestamp_field_name in field_names:
            return field_names.index(self._timestamp_field_name), self._timestamp_field_name

        for candidate in self._timestamp_field_candidates:
            if candidate in field_names:
                return field_names.index(candidate), candidate
        return None, None

    def _relative_times_from_field_np(
        self, values: np.ndarray,
    ) -> np.ndarray:
        finite_mask = np.isfinite(values)
        if not np.any(finite_mask):
            return np.zeros(len(values), dtype=np.float64)

        finite_values = values[finite_mask]
        min_value = float(np.min(finite_values))
        max_abs = float(np.max(np.abs(finite_values)))

        if self._timestamp_unit == "seconds":
            scale = 1.0
        elif self._timestamp_unit == "milliseconds":
            scale = 1e-3
        elif self._timestamp_unit == "microseconds":
            scale = 1e-6
        elif self._timestamp_unit == "nanoseconds":
            scale = 1e-9
        else:
            if max_abs <= 1.0:
                scale = 1.0
            elif max_abs <= 1e3:
                scale = 1e-3
            elif max_abs <= 1e6:
                scale = 1e-6
            else:
                scale = 1e-9

        result = np.where(finite_mask, np.maximum(0.0, (values - min_value) * scale), 0.0)
        return result

    def _relative_times_from_azimuth_np(
        self, x: np.ndarray, y: np.ndarray,
    ) -> np.ndarray:
        finite_mask = np.isfinite(x) & np.isfinite(y)
        if not np.any(finite_mask):
            return np.zeros(len(x), dtype=np.float64)

        azimuths = np.arctan2(y, x)
        azimuths = np.where(azimuths < 0.0, azimuths + 2.0 * math.pi, azimuths)

        first_finite = int(np.argmax(finite_mask))
        start_azimuth = float(azimuths[first_finite])

        if self._azimuth_direction == "cw":
            progress = (start_azimuth - azimuths) % (2.0 * math.pi)
        else:
            progress = (azimuths - start_azimuth) % (2.0 * math.pi)

        times = (progress / (2.0 * math.pi)) * self._scan_period
        times = np.where(finite_mask, times, 0.0)
        return times

    def _relative_times_from_index_np(self, n: int) -> np.ndarray:
        if n <= 1:
            return np.zeros(max(n, 0), dtype=np.float64)
        return np.linspace(0.0, self._scan_period, n, dtype=np.float64)

    def _warn_timestamp_mode_once(self, mode: str) -> None:
        if self._warned_timestamp_mode == mode:
            return
        self._warned_timestamp_mode = mode
        self.get_logger().info(f"Point timing mode: {mode}")

    def _warn_passthrough_once(self, reason: str) -> None:
        if self._warned_passthrough_reason == reason:
            return
        self._warned_passthrough_reason = reason
        self.get_logger().warning(f"Publishing raw cloud without correction: {reason}")

    def _warn_stale_drop_once(self, reason: str) -> None:
        if self._warned_stale_drop == reason:
            return
        self._warned_stale_drop = reason
        self.get_logger().warning(f"Dropping stale cloud: {reason}")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if self._bypass:
            self._publisher.publish(msg)
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        cloud_stamp = stamp_to_sec(msg.header.stamp)
        input_delay_sec = now_sec - cloud_stamp
        if self._drop_stale_clouds and input_delay_sec > self._max_input_delay_sec:
            self._warn_stale_drop_once(
                f"input delay {input_delay_sec:.3f}s exceeds {self._max_input_delay_sec:.3f}s"
            )
            return

        n_points = msg.width * msg.height
        if n_points == 0:
            self._publisher.publish(msg)
            return

        field_names = [f.name for f in msg.fields]
        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            self._warn_passthrough_once("cloud is missing x/y/z fields")
            self._publisher.publish(msg)
            return

        linear_velocity, angular_velocity, odom_msg, imu_msg = self._select_state(cloud_stamp)

        if self._use_twist and odom_msg is None:
            if self._passthrough_on_missing_state:
                self._warn_passthrough_once(f"missing odometry near t={cloud_stamp:.3f}")
                self._publisher.publish(msg)
                return

        if self._use_imu and imu_msg is None:
            if self._passthrough_on_missing_state:
                self._warn_passthrough_once(f"missing imu near t={cloud_stamp:.3f}")
                self._publisher.publish(msg)
                return

        dt_struct = build_pc2_dtype(msg)
        raw = np.frombuffer(msg.data, dtype=dt_struct)

        x = raw["x"].astype(np.float64)
        y = raw["y"].astype(np.float64)
        z = raw["z"].astype(np.float64)

        time_field_index, time_field_name = self._resolve_time_field(msg)
        if time_field_index is not None and time_field_name is not None:
            t_vals = raw[time_field_name].astype(np.float64)
            rel_t = self._relative_times_from_field_np(t_vals)
            self._warn_timestamp_mode_once(f"field:{time_field_name}")
        elif self._fallback_mode == "index":
            rel_t = self._relative_times_from_index_np(n_points)
            self._warn_timestamp_mode_once("fallback:index")
        else:
            rel_t = self._relative_times_from_azimuth_np(x, y)
            self._warn_timestamp_mode_once("fallback:azimuth")

        # First-order Rodrigues deskew: R(dt) ~ I + dt*K, accurate to ~4% for
        # |omega| < 5 rad/s and scan_period=0.1 s.
        wx, wy, wz = angular_velocity
        vx, vy, vz = linear_velocity

        finite_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        dx = rel_t * (vx + wy * z - wz * y)
        dy = rel_t * (vy + wz * x - wx * z)
        dz = rel_t * (vz + wx * y - wy * x)

        cx = np.where(finite_mask, (x + dx).astype(np.float32), raw["x"])
        cy = np.where(finite_mask, (y + dy).astype(np.float32), raw["y"])
        cz = np.where(finite_mask, (z + dz).astype(np.float32), raw["z"])

        if self._filter_floor and imu_msg is not None:
            q = imu_msg.orientation
            R_z0 = 2.0 * (q.x * q.z - q.y * q.w)
            R_z1 = 2.0 * (q.y * q.z + q.x * q.w)
            R_z2 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

            pts_b = self._R_mount_inv @ np.stack(
                [
                    cx.astype(np.float64),
                    cy.astype(np.float64),
                    cz.astype(np.float64),
                ],
                axis=0,
            )
            x_b = pts_b[0] + self._sensor_x_in_base
            y_b = pts_b[1] + self._sensor_y_in_base
            z_b = pts_b[2] + self._sensor_z_in_base
            world_z = R_z0 * x_b + R_z1 * y_b + R_z2 * z_b + self._base_z_nominal

            floor_mask = finite_mask & (world_z < self._min_world_z)
            nan32 = np.float32(math.nan)
            cx = np.where(floor_mask, nan32, cx)
            cy = np.where(floor_mask, nan32, cy)
            cz = np.where(floor_mask, nan32, cz)

        gravity_aligned = False
        if self._gravity_align and imu_msg is not None:
            roll, pitch = roll_pitch_from_imu(imu_msg.orientation)
            R_body = rotation_from_rpy(roll, pitch, 0.0)
            cx, cy, cz = gravity_align_cloud(
                cx, cy, cz, finite_mask, R_body, self._R_mount_inv
            )
            gravity_aligned = True

        out_data = bytearray(msg.data)
        out_arr = np.frombuffer(out_data, dtype=dt_struct)
        out_arr["x"] = cx
        out_arr["y"] = cy
        out_arr["z"] = cz

        out_msg = PointCloud2()
        out_msg.header = msg.header
        if self._output_stamp_mode == "now":
            out_msg.header.stamp = self.get_clock().now().to_msg()
        if gravity_aligned:
            out_msg.header.frame_id = self._gravity_align_frame
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.fields = msg.fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = msg.point_step
        out_msg.row_step = msg.row_step
        out_msg.data = bytes(out_data)
        out_msg.is_dense = msg.is_dense and not self._filter_floor
        self._publisher.publish(out_msg)


def main() -> None:
    rclpy.init()
    node = DistortionCorrector()
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
