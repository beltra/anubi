#!/usr/bin/env python3
"""Track dynamic clusters and publish predicted CTRV trajectories."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from robot_common_interfaces.msg import (
    DynamicObstacleClusterArray,
    DynamicObstacleTrack,
    DynamicObstacleTrackArray,
)

from robot_nav.dynamic_obstacles import (
    TrackState,
    ctrv_jacobian,
    ctrv_predict_state,
    normalize_angle,
    pose_from_xy_yaw,
    stamp_plus_seconds,
    stamp_seconds,
    track_is_publishable,
    vector_msg,
)


class DynamicObstacleTracker(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_tracker")

        self.declare_parameter("input_topic", "/dynamic_obstacles/clusters")
        self.declare_parameter("output_topic", "/dynamic_obstacles/tracks")
        self.declare_parameter("publish_frequency", 10.0)
        self.declare_parameter("prediction_horizon_sec", 3.0)
        self.declare_parameter("prediction_step_sec", 0.3)
        self.declare_parameter("confirmation_hits", 2)
        self.declare_parameter("max_misses", 6)
        self.declare_parameter("max_publish_misses", 0)
        self.declare_parameter("min_publish_confidence", 0.6)
        self.declare_parameter("history_window", 8)
        self.declare_parameter("gating_threshold", 9.21)
        self.declare_parameter("initial_position_variance", 0.18)
        self.declare_parameter("initial_yaw_variance", 0.8)
        self.declare_parameter("initial_speed_variance", 0.6)
        self.declare_parameter("initial_yaw_rate_variance", 0.5)
        self.declare_parameter("measurement_variance", 0.08)
        self.declare_parameter("process_position_variance", 0.04)
        self.declare_parameter("process_yaw_variance", 0.08)
        self.declare_parameter("process_speed_variance", 0.20)
        self.declare_parameter("process_yaw_rate_variance", 0.20)
        self.declare_parameter("yaw_rate_epsilon", 0.05)
        self.declare_parameter("input_timeout_sec", 0.75)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._prediction_horizon = float(self.get_parameter("prediction_horizon_sec").value)
        self._prediction_step = float(self.get_parameter("prediction_step_sec").value)
        self._confirmation_hits = int(self.get_parameter("confirmation_hits").value)
        self._max_misses = int(self.get_parameter("max_misses").value)
        self._max_publish_misses = int(self.get_parameter("max_publish_misses").value)
        self._min_publish_confidence = float(
            self.get_parameter("min_publish_confidence").value
        )
        self._history_window = int(self.get_parameter("history_window").value)
        self._gating_threshold = float(self.get_parameter("gating_threshold").value)
        self._measurement_variance = float(self.get_parameter("measurement_variance").value)
        self._yaw_rate_eps = float(self.get_parameter("yaw_rate_epsilon").value)
        self._input_timeout = float(self.get_parameter("input_timeout_sec").value)
        self._process_q = np.diag(
            [
                float(self.get_parameter("process_position_variance").value),
                float(self.get_parameter("process_position_variance").value),
                float(self.get_parameter("process_yaw_variance").value),
                float(self.get_parameter("process_speed_variance").value),
                float(self.get_parameter("process_yaw_rate_variance").value),
            ]
        )

        self._tracks: dict[int, TrackState] = {}
        self._next_track_id = 1
        self._last_input_stamp: float | None = None
        self._last_header = None

        self._publisher = self.create_publisher(DynamicObstacleTrackArray, self._output_topic, 10)
        self.create_subscription(DynamicObstacleClusterArray, self._input_topic, self._clusters_cb, 10)
        publish_frequency = float(self.get_parameter("publish_frequency").value)
        self.create_timer(1.0 / max(1.0, publish_frequency), self._timer_cb)

        self.get_logger().info(
            f"dynamic_obstacle_tracker: {self._input_topic} -> {self._output_topic}"
        )

    def _predict_track(self, track: TrackState, stamp: float) -> None:
        dt = stamp - track.last_stamp
        if dt <= 1.0e-4:
            return
        f = ctrv_jacobian(track.x, dt, self._yaw_rate_eps)
        track.x = ctrv_predict_state(track.x, dt, self._yaw_rate_eps)
        track.p = f @ track.p @ f.T + self._process_q * max(dt, 1.0e-3)
        track.last_stamp = stamp
        track.age += 1

    def _new_track(self, cluster, stamp: float) -> None:
        speed = math.hypot(cluster.velocity.x, cluster.velocity.y)
        yaw = cluster.yaw
        if speed > 1.0e-3:
            yaw = math.atan2(cluster.velocity.y, cluster.velocity.x)
        track = TrackState(
            track_id=self._next_track_id,
            x=np.array(
                [cluster.centroid.x, cluster.centroid.y, yaw, speed, 0.0],
                dtype=np.float64,
            ),
            p=np.diag(
                [
                    float(self.get_parameter("initial_position_variance").value),
                    float(self.get_parameter("initial_position_variance").value),
                    float(self.get_parameter("initial_yaw_variance").value),
                    float(self.get_parameter("initial_speed_variance").value),
                    float(self.get_parameter("initial_yaw_rate_variance").value),
                ]
            ),
            size=np.array([cluster.size.x, cluster.size.y, cluster.size.z], dtype=np.float64),
            last_stamp=stamp,
        )
        self._tracks[track.track_id] = track
        self._next_track_id += 1

    def _distance_table(self, measurements: list[np.ndarray]) -> list[tuple[float, int, int]]:
        h = np.array([[1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0]])
        r = np.eye(2) * self._measurement_variance
        distances: list[tuple[float, int, int]] = []
        for track_id, track in self._tracks.items():
            s = h @ track.p @ h.T + r
            inv_s = np.linalg.pinv(s)
            predicted = h @ track.x
            for meas_idx, measurement in enumerate(measurements):
                innovation = measurement[:2] - predicted
                d = float(innovation.T @ inv_s @ innovation)
                if d <= self._gating_threshold:
                    distances.append((d, track_id, meas_idx))
        distances.sort(key=lambda item: item[0])
        return distances

    def _correct_track(self, track: TrackState, cluster, measurement: np.ndarray) -> None:
        h = np.array([[1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0]])
        r = np.eye(2) * self._measurement_variance
        innovation = measurement[:2] - h @ track.x
        s = h @ track.p @ h.T + r
        k = track.p @ h.T @ np.linalg.pinv(s)
        track.x = track.x + k @ innovation
        track.x[2] = normalize_angle(track.x[2])
        track.p = (np.eye(5) - k @ h) @ track.p

        measured_speed = math.hypot(cluster.velocity.x, cluster.velocity.y)
        if measured_speed > 1.0e-3:
            measured_yaw = math.atan2(cluster.velocity.y, cluster.velocity.x)
            track.x[2] = normalize_angle(0.65 * track.x[2] + 0.35 * measured_yaw)
            track.x[3] = 0.7 * track.x[3] + 0.3 * measured_speed
        track.size = 0.8 * track.size + 0.2 * np.array(
            [cluster.size.x, cluster.size.y, cluster.size.z], dtype=np.float64
        )
        track.hits += 1
        track.misses = 0
        track.history.append(True)
        track.history = track.history[-self._history_window :]

    def _clusters_cb(self, msg: DynamicObstacleClusterArray) -> None:
        stamp = stamp_seconds(msg.header.stamp)
        self._last_input_stamp = stamp
        self._last_header = msg.header
        for track in list(self._tracks.values()):
            self._predict_track(track, stamp)

        measurements = [
            np.array([c.centroid.x, c.centroid.y, c.centroid.z], dtype=np.float64)
            for c in msg.clusters
        ]
        assigned_tracks: set[int] = set()
        assigned_measurements: set[int] = set()
        for _, track_id, meas_idx in self._distance_table(measurements):
            if track_id in assigned_tracks or meas_idx in assigned_measurements:
                continue
            self._correct_track(self._tracks[track_id], msg.clusters[meas_idx], measurements[meas_idx])
            assigned_tracks.add(track_id)
            assigned_measurements.add(meas_idx)

        for track_id, track in list(self._tracks.items()):
            if track_id not in assigned_tracks:
                track.misses += 1
                track.history.append(False)
                track.history = track.history[-self._history_window :]
                if track.misses > self._max_misses:
                    del self._tracks[track_id]

        for idx, cluster in enumerate(msg.clusters):
            if idx not in assigned_measurements:
                self._new_track(cluster, stamp)

        self._publish(msg.header)

    def _timer_cb(self) -> None:
        if self._last_header is None:
            return
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self._last_input_stamp is not None and now - self._last_input_stamp > self._input_timeout:
            out = DynamicObstacleTrackArray()
            out.header = self._last_header
            out.header.stamp = self.get_clock().now().to_msg()
            self._publisher.publish(out)
            return
        self._publish(self._last_header)

    def _track_msg(self, track: TrackState, header) -> DynamicObstacleTrack:
        msg = DynamicObstacleTrack()
        msg.id = int(track.track_id)
        msg.pose = pose_from_xy_yaw(track.x[0], track.x[1], 0.0, track.x[2])
        msg.velocity = vector_msg(
            [track.x[3] * math.cos(track.x[2]), track.x[3] * math.sin(track.x[2]), 0.0]
        )
        msg.size = vector_msg(track.size)
        msg.speed = float(track.x[3])
        msg.yaw_rate = float(track.x[4])
        msg.confidence = self._track_confidence(track)

        prediction = track.x.copy()
        covariance = track.p.copy()
        t = self._prediction_step
        while t <= self._prediction_horizon + 1.0e-6:
            f = ctrv_jacobian(prediction, self._prediction_step, self._yaw_rate_eps)
            prediction = ctrv_predict_state(prediction, self._prediction_step, self._yaw_rate_eps)
            covariance = f @ covariance @ f.T + self._process_q * self._prediction_step
            msg.predicted_poses.append(
                pose_from_xy_yaw(prediction[0], prediction[1], 0.0, prediction[2])
            )
            msg.predicted_stamps.append(stamp_plus_seconds(header.stamp, t))
            std_xy = math.sqrt(max(0.0, covariance[0, 0] + covariance[1, 1]))
            msg.predicted_position_std.append(float(std_xy))
            t += self._prediction_step
        return msg

    def _track_confidence(self, track: TrackState) -> float:
        return float(sum(track.history) / max(1, len(track.history)))

    def _publish(self, header) -> None:
        out = DynamicObstacleTrackArray()
        out.header = header
        for track in self._tracks.values():
            if track_is_publishable(
                track.hits,
                track.misses,
                self._confirmation_hits,
                self._max_publish_misses,
                self._track_confidence(track),
                self._min_publish_confidence,
            ):
                out.tracks.append(self._track_msg(track, header))
        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = DynamicObstacleTracker()
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
