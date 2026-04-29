#!/usr/bin/env python3
"""Convert predicted tracks into Nav2 occupancy points and RViz markers."""

import numpy as np
import rclpy
from rclpy.node import Node
from robot_common_interfaces.msg import DynamicObstacleTrackArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from robot_nav.dynamic_obstacles import (
    cloud_from_points,
    oriented_footprint_points,
    stamp_seconds,
    yaw_from_quaternion,
)


class DynamicObstacleCostmapBridge(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_costmap_bridge")

        self.declare_parameter("input_topic", "/dynamic_obstacles/tracks")
        self.declare_parameter("occupancy_topic", "/dynamic_obstacles/predicted_occupancy")
        self.declare_parameter("marker_topic", "/dynamic_obstacles/markers")
        self.declare_parameter("watchdog_timeout_sec", 0.8)
        self.declare_parameter("occupancy_z", 0.45)
        self.declare_parameter("occupancy_resolution", 0.18)
        self.declare_parameter("footprint_padding", 0.12)
        self.declare_parameter("uncertainty_scale", 0.25)
        self.declare_parameter("max_prediction_padding", 0.35)
        self.declare_parameter("min_footprint_x", 0.35)
        self.declare_parameter("min_footprint_y", 0.35)
        self.declare_parameter("min_track_confidence", 0.6)
        self.declare_parameter("max_points", 5000)
        self.declare_parameter("publish_empty_on_timeout", True)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._occupancy_topic = str(self.get_parameter("occupancy_topic").value)
        self._marker_topic = str(self.get_parameter("marker_topic").value)
        self._watchdog_timeout = float(self.get_parameter("watchdog_timeout_sec").value)
        self._occupancy_z = float(self.get_parameter("occupancy_z").value)
        self._resolution = float(self.get_parameter("occupancy_resolution").value)
        self._footprint_padding = float(self.get_parameter("footprint_padding").value)
        self._uncertainty_scale = float(self.get_parameter("uncertainty_scale").value)
        self._max_prediction_padding = float(
            self.get_parameter("max_prediction_padding").value
        )
        self._min_footprint_x = float(self.get_parameter("min_footprint_x").value)
        self._min_footprint_y = float(self.get_parameter("min_footprint_y").value)
        self._min_track_confidence = float(
            self.get_parameter("min_track_confidence").value
        )
        self._max_points = int(self.get_parameter("max_points").value)
        self._publish_empty_on_timeout = bool(
            self.get_parameter("publish_empty_on_timeout").value
        )

        self._last_stamp: float | None = None
        self._last_header = None
        self._timed_out = False

        self._occupancy_pub = self.create_publisher(PointCloud2, self._occupancy_topic, 10)
        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self.create_subscription(DynamicObstacleTrackArray, self._input_topic, self._tracks_cb, 10)
        self.create_timer(0.2, self._watchdog_cb)

        self.get_logger().info(
            f"dynamic_obstacle_costmap_bridge: {self._input_topic} -> "
            f"{self._occupancy_topic}, {self._marker_topic}"
        )

    def _prediction_padding(self, track, prediction_idx: int) -> float:
        uncertainty = 0.0
        if prediction_idx < len(track.predicted_position_std):
            uncertainty = self._uncertainty_scale * float(track.predicted_position_std[prediction_idx])
        return self._footprint_padding + min(self._max_prediction_padding, uncertainty)

    def _footprint_points(self, pose, track, padding: float) -> np.ndarray:
        return oriented_footprint_points(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
            max(self._min_footprint_x, track.size.x),
            max(self._min_footprint_y, track.size.y),
            self._resolution,
            self._occupancy_z,
            padding,
        )

    def _occupancy_points(self, msg: DynamicObstacleTrackArray) -> np.ndarray:
        point_blocks: list[np.ndarray] = []
        point_count = 0
        for track in msg.tracks:
            if track.confidence < self._min_track_confidence:
                continue
            current = self._footprint_points(track.pose, track, self._footprint_padding)
            point_blocks.append(current)
            point_count += current.shape[0]
            for idx, pose in enumerate(track.predicted_poses):
                predicted = self._footprint_points(
                    pose,
                    track,
                    self._prediction_padding(track, idx),
                )
                point_blocks.append(predicted)
                point_count += predicted.shape[0]
                if self._max_points > 0 and point_count >= self._max_points:
                    return np.concatenate(point_blocks, axis=0)[: self._max_points]
        if not point_blocks:
            return np.empty((0, 3), dtype=np.float64)
        return np.concatenate(point_blocks, axis=0)

    def _markers(self, msg: DynamicObstacleTrackArray) -> MarkerArray:
        markers = MarkerArray()
        delete = Marker()
        delete.header = msg.header
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        marker_id = 1
        for track in msg.tracks:
            if track.confidence < self._min_track_confidence:
                continue
            cube = Marker()
            cube.header = msg.header
            cube.ns = "dynamic_obstacle_tracks"
            cube.id = marker_id
            marker_id += 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose = track.pose
            cube.pose.position.z = max(0.25, 0.5 * track.size.z)
            cube.scale.x = max(0.15, track.size.x)
            cube.scale.y = max(0.15, track.size.y)
            cube.scale.z = max(0.25, track.size.z)
            cube.color.r = 1.0
            cube.color.g = 0.28
            cube.color.b = 0.08
            cube.color.a = 0.45
            cube.lifetime.sec = 1
            markers.markers.append(cube)

            line = Marker()
            line.header = msg.header
            line.ns = "dynamic_obstacle_predictions"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.07
            line.color.r = 0.0
            line.color.g = 0.65
            line.color.b = 1.0
            line.color.a = 0.95
            line.lifetime.sec = 1
            line.points.append(track.pose.position)
            for pose in track.predicted_poses:
                line.points.append(pose.position)
            markers.markers.append(line)

            text = Marker()
            text.header = msg.header
            text.ns = "dynamic_obstacle_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = track.pose
            text.pose.position.z = max(0.8, track.size.z + 0.3)
            text.scale.z = 0.28
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.95
            text.text = f"id {track.id}  {track.speed:.2f} m/s"
            text.lifetime.sec = 1
            markers.markers.append(text)
        return markers

    def _tracks_cb(self, msg: DynamicObstacleTrackArray) -> None:
        self._last_stamp = stamp_seconds(msg.header.stamp)
        self._last_header = msg.header
        self._timed_out = False
        points = self._occupancy_points(msg)
        self._occupancy_pub.publish(cloud_from_points(msg.header, points))
        self._marker_pub.publish(self._markers(msg))

    def _publish_clear(self) -> None:
        if self._last_header is None:
            return
        header = self._last_header
        header.stamp = self.get_clock().now().to_msg()
        if self._publish_empty_on_timeout:
            self._occupancy_pub.publish(cloud_from_points(header, np.empty((0, 3))))
        delete = Marker()
        delete.header = header
        delete.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(delete)
        self._marker_pub.publish(markers)

    def _watchdog_cb(self) -> None:
        if self._last_stamp is None or self._timed_out:
            return
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - self._last_stamp > self._watchdog_timeout:
            self._timed_out = True
            self._publish_clear()


def main() -> None:
    rclpy.init()
    node = DynamicObstacleCostmapBridge()
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
