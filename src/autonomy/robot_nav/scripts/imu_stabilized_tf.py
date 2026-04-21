#!/usr/bin/env python3
"""Publish a dynamic TF for the gravity-stabilized LiDAR frame.

The `lidar_stabilized` frame tracks the physical LiDAR position as the robot
trunk pitches and rolls, while keeping orientation gravity-leveled
(zero roll, zero pitch). This decouples downstream consumers from trunk
dynamics.

Frame relationship
------------------
    base_footprint -> lidar_stabilized
      (yaw + pos)           (gravity-aligned, dynamic translation)

The translation is computed as:

    R_body = Rx(roll) Ry(pitch)          # trunk roll+pitch from IMU (no yaw)
    t      = R_body @ lidar_in_base_link + [0, 0, standing_height]

where `lidar_in_base_link` is the URDF joint offset from `base_link` to
`lidar_link` ([0.28945, 0, -0.046825] for the Go2 + L1 LiDAR).

The rotation is always the identity quaternion so that `lidar_stabilized`
stays gravity-levelled regardless of trunk pitch/roll.
"""

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from robot_nav.pointcloud_utils import roll_pitch_from_imu, rotation_from_rpy


class ImuStabilizedTf(Node):
    """Broadcast `parent_frame` -> `child_frame` at the IMU rate."""

    def __init__(self) -> None:
        super().__init__("imu_stabilized_tf")

        self.declare_parameter("imu_topic",      "/imu/data")
        self.declare_parameter("parent_frame",   "base_footprint")
        self.declare_parameter("child_frame",    "lidar_stabilized")
        # LiDAR joint origin in base_link (from go2_description/urdf/go2.urdf.xacro)
        self.declare_parameter("lidar_x_in_base",  0.28945)
        self.declare_parameter("lidar_y_in_base",  0.0)
        self.declare_parameter("lidar_z_in_base", -0.046825)
        # Nominal height of base_link above base_footprint / ground [m]
        self.declare_parameter("standing_height",  0.28)

        self._parent_frame   = str(self.get_parameter("parent_frame").value)
        self._child_frame    = str(self.get_parameter("child_frame").value)
        self._lidar_in_base  = np.array([
            float(self.get_parameter("lidar_x_in_base").value),
            float(self.get_parameter("lidar_y_in_base").value),
            float(self.get_parameter("lidar_z_in_base").value),
        ], dtype=np.float64)
        self._standing_height = float(self.get_parameter("standing_height").value)

        self._broadcaster = TransformBroadcaster(self)

        imu_topic = str(self.get_parameter("imu_topic").value)
        self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"imu_stabilized_tf: {self._parent_frame} -> {self._child_frame} "
            f"(lidar_in_base={self._lidar_in_base.tolist()}, "
            f"standing_height={self._standing_height:.3f} m)"
        )

    def _imu_cb(self, msg: Imu) -> None:
        # Yaw is intentionally ignored here.
        roll, pitch = roll_pitch_from_imu(msg.orientation)

        # Body-to-world rotation for roll+pitch only.
        R_body = rotation_from_rpy(roll, pitch, 0.0)

        # Physical LiDAR position in base_footprint.
        # When trunk is level R_body = I, so translation = lidar_in_base + [0,0,0.28]
        # = [0.28945, 0, 0.233175], matching the previous static TF exactly.
        t = R_body @ self._lidar_in_base + np.array([0.0, 0.0, self._standing_height])

        tf_msg = TransformStamped()
        tf_msg.header.stamp    = msg.header.stamp
        tf_msg.header.frame_id = self._parent_frame
        tf_msg.child_frame_id  = self._child_frame

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])

        # Rotation stays identity so lidar_stabilized remains gravity-leveled.
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self._broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = ImuStabilizedTf()
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
