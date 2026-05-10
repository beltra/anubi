#!/usr/bin/env python3
"""Accumulate short windows of non-repetitive LiDAR clouds for 2D scan slicing."""

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def stamp_to_sec(msg):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def fields_signature(msg):
    return tuple(
        (field.name, field.offset, field.datatype, field.count) for field in msg.fields
    )


class CloudAccumulator(Node):
    """Publish one cloud made from the latest frames in the same sensor frame."""

    def __init__(self):
        super().__init__("cloud_accumulator")
        self.declare_parameter("input_topic", "/lidar/points_undistorted")
        self.declare_parameter("output_topic", "/lidar/points_scan_accumulated")
        self.declare_parameter("accumulation_window_sec", 0.75)
        self.declare_parameter("max_frames", 12)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.window_sec = float(self.get_parameter("accumulation_window_sec").value)
        self.max_frames = int(self.get_parameter("max_frames").value)

        self.frames = deque()
        self.frame_id = ""
        self.signature = None
        self.point_step = 0

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            "cloud_accumulator: %s -> %s (window=%.2fs, max_frames=%d)"
            % (self.input_topic, self.output_topic, self.window_sec, self.max_frames)
        )

    def reset_if_layout_changed(self, msg):
        signature = fields_signature(msg)
        layout_changed = (
            self.frame_id
            and (
                msg.header.frame_id != self.frame_id
                or signature != self.signature
                or msg.point_step != self.point_step
            )
        )
        if layout_changed:
            self.frames.clear()
        self.frame_id = msg.header.frame_id
        self.signature = signature
        self.point_step = msg.point_step

    def prune_old_frames(self, newest_stamp):
        while len(self.frames) > self.max_frames:
            self.frames.popleft()
        while self.frames and newest_stamp - self.frames[0][0] > self.window_sec:
            self.frames.popleft()

    def cloud_callback(self, msg):
        if msg.point_step == 0 or not msg.data:
            return

        self.reset_if_layout_changed(msg)
        stamp_sec = stamp_to_sec(msg)
        self.frames.append((stamp_sec, msg))
        self.prune_old_frames(stamp_sec)

        point_count = sum(len(frame.data) // frame.point_step for _, frame in self.frames)
        if point_count == 0:
            return

        out = PointCloud2()
        newest = self.frames[-1][1]
        out.header = newest.header
        out.height = 1
        out.width = point_count
        out.fields = newest.fields
        out.is_bigendian = newest.is_bigendian
        out.point_step = newest.point_step
        out.row_step = out.point_step * out.width
        out.is_dense = all(frame.is_dense for _, frame in self.frames)
        out.data = b"".join(bytes(frame.data) for _, frame in self.frames)

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CloudAccumulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
