#!/usr/bin/env python3

import math
from collections import deque
from typing import Deque, List, Optional, Sequence, Tuple

import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _rotation_from_omega(omega: np.ndarray, dt: float) -> np.ndarray:
    theta = float(np.linalg.norm(omega) * dt)
    if theta < 1e-9:
        wx, wy, wz = omega * dt
        skew = np.array(
            [
                [0.0, -wz, wy],
                [wz, 0.0, -wx],
                [-wy, wx, 0.0],
            ],
            dtype=np.float64,
        )
        return np.eye(3, dtype=np.float64) + skew

    axis = omega / float(np.linalg.norm(omega))
    ax, ay, az = axis
    skew = np.array(
        [
            [0.0, -az, ay],
            [az, 0.0, -ax],
            [-ay, ax, 0.0],
        ],
        dtype=np.float64,
    )
    return (
        np.eye(3, dtype=np.float64)
        + math.sin(theta) * skew
        + (1.0 - math.cos(theta)) * (skew @ skew)
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
        self.declare_parameter("timestamp_field_name", "")
        self.declare_parameter(
            "timestamp_field_candidates",
            ["time_stamp", "timestamp", "time", "t"],
        )
        self.declare_parameter("timestamp_unit", "auto")
        self.declare_parameter("fallback_mode", "azimuth")
        self.declare_parameter("azimuth_direction", "ccw")
        self.declare_parameter("passthrough_on_missing_state", True)
        self.declare_parameter("state_timeout_sec", 0.25)
        self.declare_parameter("drop_stale_clouds", True)
        self.declare_parameter("max_input_delay_sec", 0.25)
        self.declare_parameter("output_stamp_mode", "now")

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
            f"stamp={self._output_stamp_mode})"
        )

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_history.append(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_history.append(msg)

    def _nearest_msg(self, history: Deque, stamp_sec: float):
        if not history:
            return None

        best = min(history, key=lambda msg: abs(_stamp_to_seconds(msg.header.stamp) - stamp_sec))
        best_dt = abs(_stamp_to_seconds(best.header.stamp) - stamp_sec)
        if best_dt > self._state_timeout_sec:
            return None
        return best

    def _select_state(self, stamp_sec: float):
        odom_msg = self._nearest_msg(self._odom_history, stamp_sec) if self._use_twist else None
        imu_msg = self._nearest_msg(self._imu_history, stamp_sec) if self._use_imu else None

        linear_velocity = np.zeros(3, dtype=np.float64)
        angular_velocity = np.zeros(3, dtype=np.float64)
        roll = 0.0
        pitch = 0.0

        if odom_msg is not None:
            twist = odom_msg.twist.twist
            linear_velocity = np.array(
                [twist.linear.x, twist.linear.y, twist.linear.z], dtype=np.float64
            )
            angular_velocity = np.array(
                [twist.angular.x, twist.angular.y, twist.angular.z], dtype=np.float64
            )

        if imu_msg is not None:
            imu_omega = np.array(
                [
                    imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z,
                ],
                dtype=np.float64,
            )
            angular_velocity = imu_omega
            orientation = imu_msg.orientation
            if math.isfinite(orientation.x) and math.isfinite(orientation.y) and math.isfinite(
                orientation.z
            ) and math.isfinite(orientation.w):
                roll, pitch, _ = _quat_to_rpy(
                    orientation.x, orientation.y, orientation.z, orientation.w
                )

        if not self._use_3d:
            linear_velocity[2] = 0.0
            angular_velocity[0] = 0.0
            angular_velocity[1] = 0.0
            roll = 0.0
            pitch = 0.0

        body_to_sensor = _rotation_from_rpy(roll, pitch, 0.0)
        linear_sensor = body_to_sensor @ linear_velocity

        return linear_sensor, angular_velocity, odom_msg, imu_msg

    def _resolve_time_field(self, msg: PointCloud2) -> Tuple[Optional[int], Optional[str]]:
        field_names = [field.name for field in msg.fields]
        if self._timestamp_field_name and self._timestamp_field_name in field_names:
            return field_names.index(self._timestamp_field_name), self._timestamp_field_name

        for candidate in self._timestamp_field_candidates:
            if candidate in field_names:
                return field_names.index(candidate), candidate
        return None, None

    def _resolve_xyz_indices(
        self, msg: PointCloud2
    ) -> Tuple[Optional[int], Optional[int], Optional[int]]:
        field_names = [field.name for field in msg.fields]
        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            return None, None, None
        return field_names.index("x"), field_names.index("y"), field_names.index("z")

    def _relative_times_from_field(
        self, points: Sequence[Tuple], field_index: int
    ) -> List[float]:
        values = []
        for point in points:
            raw = point[field_index]
            try:
                value = float(raw)
            except (TypeError, ValueError):
                value = float("nan")
            values.append(value)

        finite_values = [value for value in values if math.isfinite(value)]
        if not finite_values:
            return [0.0] * len(points)

        min_value = min(finite_values)
        max_abs = max(abs(value) for value in finite_values)

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

        return [
            max(0.0, (value - min_value) * scale) if math.isfinite(value) else 0.0
            for value in values
        ]

    def _relative_times_from_azimuth(
        self, points: Sequence[Tuple], x_index: int, y_index: int
    ) -> List[float]:
        start_azimuth = None
        azimuths = []
        for point in points:
            x = float(point[x_index])
            y = float(point[y_index])
            if math.isfinite(x) and math.isfinite(y):
                azimuth = math.atan2(y, x)
                if azimuth < 0.0:
                    azimuth += 2.0 * math.pi
                if start_azimuth is None:
                    start_azimuth = azimuth
            else:
                azimuth = float("nan")
            azimuths.append(azimuth)

        if start_azimuth is None:
            return [0.0] * len(points)

        times = []
        for azimuth in azimuths:
            if not math.isfinite(azimuth):
                times.append(0.0)
                continue

            if self._azimuth_direction == "cw":
                progress = (start_azimuth - azimuth) % (2.0 * math.pi)
            else:
                progress = (azimuth - start_azimuth) % (2.0 * math.pi)

            times.append((progress / (2.0 * math.pi)) * self._scan_period)
        return times

    def _relative_times_from_index(self, points: Sequence[Tuple]) -> List[float]:
        if not points:
            return []
        if len(points) == 1:
            return [0.0]
        return [
            (float(index) / float(len(points) - 1)) * self._scan_period
            for index in range(len(points))
        ]

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
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        cloud_stamp = _stamp_to_seconds(msg.header.stamp)
        input_delay_sec = now_sec - cloud_stamp
        if self._drop_stale_clouds and input_delay_sec > self._max_input_delay_sec:
            self._warn_stale_drop_once(
                f"input delay {input_delay_sec:.3f}s exceeds {self._max_input_delay_sec:.3f}s"
            )
            return

        points = list(point_cloud2.read_points(msg, field_names=None, skip_nans=False))
        if not points:
            self._publisher.publish(msg)
            return

        x_index, y_index, z_index = self._resolve_xyz_indices(msg)
        if x_index is None or y_index is None or z_index is None:
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

        time_field_index, time_field_name = self._resolve_time_field(msg)
        if time_field_index is not None and time_field_name is not None:
            relative_times = self._relative_times_from_field(points, time_field_index)
            self._warn_timestamp_mode_once(f"field:{time_field_name}")
        elif self._fallback_mode == "index":
            relative_times = self._relative_times_from_index(points)
            self._warn_timestamp_mode_once("fallback:index")
        else:
            relative_times = self._relative_times_from_azimuth(points, x_index, y_index)
            self._warn_timestamp_mode_once("fallback:azimuth")

        corrected_points = []
        for point, dt in zip(points, relative_times):
            point_values = list(point)
            x = float(point_values[x_index])
            y = float(point_values[y_index])
            z = float(point_values[z_index])

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                corrected_points.append(tuple(point_values))
                continue

            rotation = _rotation_from_omega(angular_velocity, dt)
            translation = linear_velocity * dt
            corrected_xyz = rotation @ np.array([x, y, z], dtype=np.float64) + translation

            point_values[x_index] = float(corrected_xyz[0])
            point_values[y_index] = float(corrected_xyz[1])
            point_values[z_index] = float(corrected_xyz[2])
            corrected_points.append(tuple(point_values))

        output_header = msg.header
        if self._output_stamp_mode == "now":
            output_header.stamp = self.get_clock().now().to_msg()

        corrected = point_cloud2.create_cloud(output_header, msg.fields, corrected_points)
        corrected.is_bigendian = msg.is_bigendian
        corrected.is_dense = msg.is_dense
        self._publisher.publish(corrected)


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
