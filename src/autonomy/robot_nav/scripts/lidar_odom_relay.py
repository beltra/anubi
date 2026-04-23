#!/usr/bin/env python3
"""
lidar_odom_relay.py

Republishes nav_msgs/Odometry with parameterizable diagonal covariances.
Required because DLIO publishes all-zero covariances, which robot_localization
EKF treats as infinite precision (hard overwrite on every scan).
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _diag_cov(values: list) -> list:
    mat = [0.0] * 36
    for i, v in enumerate(values[:6]):
        mat[i * 7] = float(v)
    return mat


class LidarOdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("lidar_odom_relay")

        self.declare_parameter("input_topic", "/lidar_odom")
        self.declare_parameter("output_topic", "/lidar_odom_reliable")
        # Diagonal pose covariance [x, y, z, roll, pitch, yaw] (m^2 or rad^2)
        self.declare_parameter(
            "pose_cov",
            [0.005, 0.005, 0.05, 0.005, 0.005, 0.002],
        )
        # Diagonal twist covariance [vx, vy, vz, vroll, vpitch, vyaw]
        self.declare_parameter(
            "twist_cov",
            [0.01, 0.01, 0.05, 0.005, 0.005, 0.005],
        )

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._pose_cov = _diag_cov(list(self.get_parameter("pose_cov").value))
        self._twist_cov = _diag_cov(list(self.get_parameter("twist_cov").value))

        self._pub = self.create_publisher(Odometry, out_topic, qos_profile_sensor_data)
        self.create_subscription(Odometry, in_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"lidar_odom_relay: {in_topic} -> {out_topic} "
            f"(pose_diag={[self._pose_cov[i*7] for i in range(6)]})"
        )

    def _cb(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist
        out.pose.covariance = self._pose_cov
        out.twist.covariance = self._twist_cov
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = LidarOdomRelay()
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
