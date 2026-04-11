#!/usr/bin/env python3

import math
from collections import deque
from typing import Deque, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll);  sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw);   sy = math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ], dtype=np.float64)


class CloudSelfFilter(Node):
    """Mask robot-body and below-floor returns from a point cloud.

    Uses a fully-vectorised numpy pipeline so the 10 Hz LiDAR rate can be
    processed in real time without queue build-up.

    When `gravity_align` is enabled, the filtered cloud is additionally
    rotated by R_body(roll, pitch, 0) — the current trunk orientation from
    the IMU — and published with `frame_id = gravity_align_frame` (default:
    `lidar_stabilized`).  This mirrors the gravity-align step in
    distortion_corrector.py used on the real robot, so that the simulation
    STVL costmap receives the same gravity-levelled point cloud representation.
    Requires imu_stabilized_tf.py to be running.
    """

    def __init__(self) -> None:
        super().__init__("cloud_self_filter")

        self.declare_parameter("input_topic", "/lidar/points")
        self.declare_parameter("output_topic", "/lidar/points_filtered")
        self.declare_parameter("lidar_x", 0.28945)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_z", -0.056825)
        self.declare_parameter("base_z", 0.28)
        # Raised from 0.03 → 0.12 m: aligns the floor gate with the STVL
        # min_obstacle_height (0.10 m) plus a safety margin so borderline floor
        # returns that slip through the approximation cannot be marked as obstacles.
        self.declare_parameter("min_world_z", 0.12)
        self.declare_parameter("robot_min_x", -0.40)
        self.declare_parameter("robot_max_x", 0.38)
        self.declare_parameter("robot_min_y", -0.26)
        self.declare_parameter("robot_max_y", 0.26)
        self.declare_parameter("robot_min_z", -0.42)
        self.declare_parameter("robot_max_z", 0.18)
        self.declare_parameter("min_radius", 0.12)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_timeout_sec", 0.25)
        # Gravity alignment: rotate the filtered cloud so it is expressed in the
        # gravity-levelled lidar_stabilized frame (same as distortion_corrector's
        # gravity_align option for the real robot).
        self.declare_parameter("gravity_align", False)
        self.declare_parameter("gravity_align_frame", "lidar_stabilized")

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

    def _R_body(self, imu: Optional[Imu]) -> Optional[np.ndarray]:
        """Return the full 3×3 body-to-world rotation for roll+pitch from IMU."""
        if imu is None:
            return None
        q = imu.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        return _rotation_from_rpy(roll, pitch, 0.0)

    @staticmethod
    def _build_dtype(msg: PointCloud2):
        """Build a numpy structured dtype from the PointCloud2 field descriptors."""
        _ROS_TO_NP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
                      5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
        fields = {}
        for f in msg.fields:
            np_type = _ROS_TO_NP.get(f.datatype, np.uint8)
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
        n_points = msg.width * msg.height
        if n_points == 0:
            self._publisher.publish(msg)
            return

        field_names = [f.name for f in msg.fields]
        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            self._publisher.publish(msg)
            return

        cloud_stamp_sec = _stamp_to_sec(msg.header.stamp)
        imu = self._nearest_imu(cloud_stamp_sec)
        R_z_row = self._world_z_row(imu)

        # ── Parse cloud into numpy arrays (zero-copy view) ────────────────
        dt_struct = self._build_dtype(msg)
        raw = np.frombuffer(msg.data, dtype=dt_struct)

        x = raw["x"].astype(np.float64)
        y = raw["y"].astype(np.float64)
        z = raw["z"].astype(np.float64)

        finite_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        # ── Radius filter (near-body clutter) ────────────────────────────
        radius_sq = x * x + y * y + z * z
        radius_mask = finite_mask & (radius_sq < self._min_radius ** 2)

        # ── Self-body box filter ──────────────────────────────────────────
        # Transform sensor frame → base_link frame.
        # L1 LiDAR is mounted with Rx(π): x_b = x + lidar_x,
        #                                  y_b = -y + lidar_y,
        #                                  z_b = -z + lidar_z
        x_b = x + self._lidar_x
        y_b = -y + self._lidar_y
        z_b = -z + self._lidar_z

        self_box_mask = (
            finite_mask
            & (x_b >= self._robot_min_x) & (x_b <= self._robot_max_x)
            & (y_b >= self._robot_min_y) & (y_b <= self._robot_max_y)
            & (z_b >= self._robot_min_z) & (z_b <= self._robot_max_z)
        )

        # ── Below-ground gate (IMU-tilt-corrected) ────────────────────────
        # Estimate world-frame z for each point using the IMU rotation row.
        # When IMU is unavailable, fall back to the static level-robot formula.
        if R_z_row is not None:
            world_z = (
                R_z_row[0] * x_b
                + R_z_row[1] * y_b
                + R_z_row[2] * z_b
                + self._base_z
            )
        else:
            world_z = z_b + self._base_z

        floor_mask = finite_mask & (world_z < self._min_world_z)

        # ── Combine and remove bad points ─────────────────────────────────
        remove = radius_mask | self_box_mask | floor_mask

        # Build output: copy raw buffer and NaN-out removed points so the
        # output cloud has the same layout (frame_id, fields, point_step).
        out_data = bytearray(msg.data)
        out_arr = np.frombuffer(out_data, dtype=dt_struct)
        nan32 = np.float32(math.nan)

        cx = np.where(remove, nan32, raw["x"].astype(np.float32))
        cy = np.where(remove, nan32, raw["y"].astype(np.float32))
        cz = np.where(remove, nan32, raw["z"].astype(np.float32))

        # ── Gravity alignment ─────────────────────────────────────────────
        # Rotate filtered points by R_body(roll, pitch, 0) from IMU so they are
        # expressed in the gravity-levelled lidar_stabilized frame:
        #   P_stab = R_body @ P_lidar_link
        # This mirrors distortion_corrector.py's gravity_align step so that the
        # simulation STVL receives the same frame representation as the real robot.
        out_frame_id = msg.header.frame_id
        gravity_aligned = False
        if self._gravity_align:
            R = self._R_body(imu)
            if R is not None:
                keep = finite_mask & ~remove
                # Apply LiDAR mount rotation Rx(π) then body rotation:
                #   P_stab = R_body @ R_mount @ P_lidar_link = R_body @ [x, −y, −z]
                # Without the Rx(π) flip the y-axis is not corrected, mirroring the map.
                pts = np.stack([cx.astype(np.float64),
                                -cy.astype(np.float64),
                                -cz.astype(np.float64)], axis=0)  # 3 × N
                pts_stab = R @ pts
                cx = np.where(keep, pts_stab[0].astype(np.float32), cx)
                cy = np.where(keep, pts_stab[1].astype(np.float32), cy)
                cz = np.where(keep, pts_stab[2].astype(np.float32), cz)
                gravity_aligned = True
            if gravity_aligned:
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
        out_msg.is_dense = False   # NaN points are present
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
