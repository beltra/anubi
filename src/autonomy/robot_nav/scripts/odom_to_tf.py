#!/usr/bin/env python3
"""Re-broadcast a nav_msgs/Odometry topic as a TF transform.

Used by the bag-replay tuning flow. The bag carries `/odom` (or `/odom_raw`)
as a topic but its `/tf` is dropped from playback (the recorded `/tf` already
contains a stale `map -> odom` from the past slam_toolbox run plus every
other dynamic TF -- replaying any of it competes with the fresh stack and
makes the map jump). This node turns the message stream back into the
parent->child transform the rest of the stack expects.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):

    def __init__(self):
        super().__init__("odom_to_tf")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_footprint")
        # DLIO republishes scan-match-corrected poses under an already-used
        # header stamp (plus rare zero stamps); rebroadcasting those makes
        # non-increasing TF. Hold each message until a strictly newer stamp
        # arrives so exactly one transform per unique stamp goes out and it
        # is the LAST (corrected) pose of a duplicate-stamp group, at the
        # cost of one message period of broadcast latency.
        self.declare_parameter("dedup_stamps", True)

        topic = self.get_parameter("odom_topic").value
        self._parent = self.get_parameter("parent_frame").value
        self._child = self.get_parameter("child_frame").value
        self._dedup = bool(self.get_parameter("dedup_stamps").value)
        self._pending: Odometry | None = None

        self._broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, topic, self._on_odom, 100)

        self.get_logger().info(
            f"odom_to_tf: {topic} -> {self._parent} -> {self._child} "
            f"(dedup_stamps={self._dedup})"
        )

    @staticmethod
    def _stamp_ns(msg: Odometry) -> int:
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _on_odom(self, msg: Odometry) -> None:
        if not self._dedup:
            self._broadcast(msg)
            return
        stamp = self._stamp_ns(msg)
        if stamp == 0:
            return
        if self._pending is None:
            self._pending = msg
            return
        pending_stamp = self._stamp_ns(self._pending)
        if stamp > pending_stamp:
            self._broadcast(self._pending)
            self._pending = msg
        elif stamp == pending_stamp:
            self._pending = msg
        # stamp < pending: stale out-of-order message, drop.

    def _broadcast(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._parent
        t.child_frame_id = self._child
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomToTf()
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
