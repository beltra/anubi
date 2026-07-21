#!/usr/bin/env python3
"""Frame-rewriting relay for Isaac Sim G1 host topics.

Maps an Isaac host sensor topic onto a canonical anubi topic
(docs/topic_map.md), rewriting only header.frame_id. Unlike the real-G1
cloud_restamp_relay (robot_nav), no stamp rewriting is needed: the Isaac
host stamps everything from the shared sim /clock, so header stamps and
the per-point relative 'time' field pass through byte-for-byte untouched
(the distortion corrector consumes the relative times for deskew).

Publisher QoS is RELIABLE for the same reason as cloud_restamp_relay:
DLIO subscribes reliable, and a best-effort publisher would silently
starve it while best-effort subscribers still match a reliable publisher.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2

MESSAGE_TYPES = {
    "pointcloud2": PointCloud2,
    "imu": Imu,
    "odometry": Odometry,
}


class FrameRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("frame_relay")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        # One of: pointcloud2, imu, odometry.
        self.declare_parameter("message_type", "pointcloud2")
        # New header.frame_id; empty keeps the incoming frame.
        self.declare_parameter("frame_id", "")
        # Odometry only: new child_frame_id; empty keeps the incoming one.
        self.declare_parameter("child_frame_id", "")

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        type_name = str(self.get_parameter("message_type").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._child_frame_id = str(self.get_parameter("child_frame_id").value)

        msg_type = MESSAGE_TYPES.get(type_name)
        if msg_type is None:
            raise ValueError(
                f"unknown message_type '{type_name}' "
                f"(expected one of {sorted(MESSAGE_TYPES)})"
            )
        if not in_topic or not out_topic:
            raise ValueError("input_topic and output_topic are required")

        pub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(msg_type, out_topic, pub_qos)
        self.create_subscription(msg_type, in_topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"frame_relay: {in_topic} -> {out_topic} "
            f"({type_name}, frame_id={self._frame_id or '<passthrough>'})"
        )

    def _cb(self, msg) -> None:
        if self._frame_id:
            msg.header.frame_id = self._frame_id
        if self._child_frame_id and isinstance(msg, Odometry):
            msg.child_frame_id = self._child_frame_id
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = FrameRelayNode()
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
