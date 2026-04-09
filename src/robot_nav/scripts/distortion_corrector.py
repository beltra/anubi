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


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


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
        # Static mount rotation from body frame (base_link) to sensor frame
        # (lidar_link).  For the Unitree Go2 with the L1 LiDAR mounted
        # upside-down this is Rx(π) = [roll=π, pitch=0, yaw=0].
        self.declare_parameter("sensor_mount_rpy", [3.14159265358979, 0.0, 0.0])
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
        # Bypass mode: republish the input cloud unchanged.  Use in simulation
        # where Gazebo produces instantaneous snapshots with no scan distortion.
        self.declare_parameter("bypass", False)

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

        # Pre-compute the static rotation from body frame to sensor frame.
        mount_rpy = list(self.get_parameter("sensor_mount_rpy").value)
        self._R_mount = _rotation_from_rpy(mount_rpy[0], mount_rpy[1], mount_rpy[2])

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

        # Transform velocities from body frame to sensor frame using the
        # static mount rotation.  The point cloud lives in the sensor frame
        # (lidar_link), so the deskew corrections must be in that frame too.
        # Previous code incorrectly used IMU roll/pitch here instead of the
        # fixed mount rotation, and never transformed angular velocity.
        linear_sensor = self._R_mount @ linear_velocity
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

        # Find start azimuth from the first finite point
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

    @staticmethod
    def _build_dtype(msg: PointCloud2):
        """Build a numpy structured dtype from the PointCloud2 field descriptors.

        Accounts for padding between fields and at the end of each point so that
        the dtype itemsize equals msg.point_step.  Without this, np.frombuffer
        misaligns every point after the first.
        """
        _ROS_TO_NP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
                      5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
        _ROS_SIZES = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
        fields = {}
        for f in msg.fields:
            np_type = _ROS_TO_NP.get(f.datatype, np.uint8)
            size = _ROS_SIZES.get(f.datatype, 1)
            if f.count == 1:
                fields[f.name] = (np_type, f.offset)
            else:
                fields[f.name] = (np_type, (f.count,), f.offset)
        return np.dtype({"names": list(fields.keys()),
                         "formats": [v[0] if len(v) == 2 else (v[0], v[1])
                                     for v in fields.values()],
                         "offsets": [v[-1] for v in fields.values()],
                         "itemsize": msg.point_step})

    def _cloud_cb(self, msg: PointCloud2) -> None:
        # Bypass mode: republish unchanged (for simulation)
        if self._bypass:
            self._publisher.publish(msg)
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        cloud_stamp = _stamp_to_seconds(msg.header.stamp)
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

        # ── Parse field layout ────────────────────────────────────────────
        field_names = [f.name for f in msg.fields]
        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            self._warn_passthrough_once("cloud is missing x/y/z fields")
            self._publisher.publish(msg)
            return

        # ── State lookup ──────────────────────────────────────────────────
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

        # ── Read cloud as numpy structured array (zero-copy view) ─────────
        dt_struct = self._build_dtype(msg)
        raw = np.frombuffer(msg.data, dtype=dt_struct)

        x = raw["x"].astype(np.float64)
        y = raw["y"].astype(np.float64)
        z = raw["z"].astype(np.float64)

        # ── Compute per-point relative times (vectorised) ─────────────────
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

        # ── Vectorised deskew ─────────────────────────────────────────────
        # First-order Rodrigues: R(dt) ≈ I + dt·K  where K = skew(ω).
        # For typical scan_period=0.1 s and |ω| < 5 rad/s, max θ = 0.5 rad,
        # sin(θ)/θ ≈ 0.96 — first-order is accurate to ~4%.  This avoids
        # computing a per-point 3×3 matrix multiply.
        wx, wy, wz = angular_velocity
        vx, vy, vz = linear_velocity

        finite_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        # dx = dt*(vx + wy*z - wz*y)   (cross product + translation)
        # dy = dt*(vy + wz*x - wx*z)
        # dz = dt*(vz + wx*y - wy*x)
        dx = rel_t * (vx + wy * z - wz * y)
        dy = rel_t * (vy + wz * x - wx * z)
        dz = rel_t * (vz + wx * y - wy * x)

        cx = np.where(finite_mask, (x + dx).astype(np.float32), raw["x"])
        cy = np.where(finite_mask, (y + dy).astype(np.float32), raw["y"])
        cz = np.where(finite_mask, (z + dz).astype(np.float32), raw["z"])

        # ── Write corrected data back ─────────────────────────────────────
        out_data = bytearray(msg.data)
        out_arr = np.frombuffer(out_data, dtype=dt_struct)
        out_arr["x"] = cx
        out_arr["y"] = cy
        out_arr["z"] = cz

        # ── Publish ───────────────────────────────────────────────────────
        out_msg = PointCloud2()
        out_msg.header = msg.header
        if self._output_stamp_mode == "now":
            out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.fields = msg.fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = msg.point_step
        out_msg.row_step = msg.row_step
        out_msg.data = bytes(out_data)
        out_msg.is_dense = msg.is_dense
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
