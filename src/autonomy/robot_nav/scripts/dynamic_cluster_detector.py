#!/usr/bin/env python3
"""Extract moving LiDAR clusters from a fixed-frame point cloud."""

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robot_common_interfaces.msg import DynamicObstacleCluster, DynamicObstacleClusterArray
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from robot_nav.dynamic_obstacles import (
    ShadowCone,
    euclidean_clusters,
    point_msg,
    points_in_shadow_cones,
    points_from_cloud,
    shadow_cone_from_obstacle,
    stamp_seconds,
    velocity_matches_ego_motion,
    vector_msg,
)


@dataclass
class PreviousCluster:
    centroid: np.ndarray
    stamp: float
    misses: int = 0
    dynamic_hits: int = 0
    consecutive_dynamic_hits: int = 0
    stationary_hits: int = 1


class DynamicClusterDetector(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_cluster_detector")

        self.declare_parameter("input_topic", "/dynamic_obstacles/points_world")
        self.declare_parameter("output_topic", "/dynamic_obstacles/clusters")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("use_static_map", True)
        self.declare_parameter("require_static_map", True)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("static_inflation_radius", 0.18)
        self.declare_parameter("cluster_tolerance", 0.35)
        self.declare_parameter("min_cluster_points", 4)
        self.declare_parameter("max_cluster_points", 1200)
        self.declare_parameter("max_clusters", 24)
        self.declare_parameter("association_distance", 0.75)
        self.declare_parameter("min_dynamic_displacement", 0.12)
        self.declare_parameter("min_dynamic_speed", 0.15)
        self.declare_parameter("min_dynamic_hits", 2)
        self.declare_parameter("max_track_misses", 5)
        self.declare_parameter("min_cluster_size_xy", 0.08)
        self.declare_parameter("shadow_management_enabled", True)
        self.declare_parameter("sensor_frame", "lidar_stabilized")
        self.declare_parameter("shadow_tf_timeout_sec", 0.03)
        self.declare_parameter("shadow_persistence_sec", 0.75)
        self.declare_parameter("shadow_angle_margin_rad", 0.06)
        self.declare_parameter("shadow_range_padding", 0.15)
        self.declare_parameter("shadow_max_range", 8.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("ego_velocity_rejection", True)
        self.declare_parameter("ego_speed_ratio_threshold", 0.55)
        self.declare_parameter("ego_min_robot_speed", 0.08)
        self.declare_parameter("ego_artifact_min_speed_ratio", 0.35)
        self.declare_parameter("ego_artifact_max_speed_ratio", 1.8)
        self.declare_parameter("ego_artifact_direction_cosine", 0.75)
        self.declare_parameter("filter_out_of_map", True)
        self.declare_parameter("min_consecutive_dynamic_hits", 2)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._map_topic = str(self.get_parameter("map_topic").value)
        self._use_static_map = bool(self.get_parameter("use_static_map").value)
        self._require_static_map = bool(self.get_parameter("require_static_map").value)
        self._occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self._static_inflation_radius = float(
            self.get_parameter("static_inflation_radius").value
        )
        self._cluster_tolerance = float(self.get_parameter("cluster_tolerance").value)
        self._min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self._max_cluster_points = int(self.get_parameter("max_cluster_points").value)
        self._max_clusters = int(self.get_parameter("max_clusters").value)
        self._association_distance = float(self.get_parameter("association_distance").value)
        self._min_dynamic_displacement = float(
            self.get_parameter("min_dynamic_displacement").value
        )
        self._min_dynamic_speed = float(self.get_parameter("min_dynamic_speed").value)
        self._min_dynamic_hits = max(1, int(self.get_parameter("min_dynamic_hits").value))
        self._max_track_misses = int(self.get_parameter("max_track_misses").value)
        self._min_cluster_size_xy = float(self.get_parameter("min_cluster_size_xy").value)
        self._shadow_management_enabled = bool(
            self.get_parameter("shadow_management_enabled").value
        )
        self._sensor_frame = str(self.get_parameter("sensor_frame").value)
        self._shadow_tf_timeout = float(self.get_parameter("shadow_tf_timeout_sec").value)
        self._shadow_persistence = float(self.get_parameter("shadow_persistence_sec").value)
        self._shadow_angle_margin = float(self.get_parameter("shadow_angle_margin_rad").value)
        self._shadow_range_padding = float(self.get_parameter("shadow_range_padding").value)
        self._shadow_max_range = float(self.get_parameter("shadow_max_range").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._ego_velocity_rejection = bool(self.get_parameter("ego_velocity_rejection").value)
        self._ego_speed_ratio_threshold = float(
            self.get_parameter("ego_speed_ratio_threshold").value
        )
        self._ego_min_robot_speed = float(self.get_parameter("ego_min_robot_speed").value)
        self._ego_artifact_min_speed_ratio = float(
            self.get_parameter("ego_artifact_min_speed_ratio").value
        )
        self._ego_artifact_max_speed_ratio = float(
            self.get_parameter("ego_artifact_max_speed_ratio").value
        )
        self._ego_artifact_direction_cosine = float(
            self.get_parameter("ego_artifact_direction_cosine").value
        )
        self._filter_out_of_map = bool(self.get_parameter("filter_out_of_map").value)
        self._min_consecutive_dynamic_hits = max(
            1, int(self.get_parameter("min_consecutive_dynamic_hits").value)
        )

        self._map: OccupancyGrid | None = None
        self._previous: dict[int, PreviousCluster] = {}
        self._shadow_cones: list[ShadowCone] = []
        self._next_id = 1
        self._ego_vx = 0.0
        self._ego_vy = 0.0
        self._ego_speed = 0.0
        self._ego_yaw_rate = 0.0
        self._ego_map_velocity = np.zeros(2, dtype=np.float64)
        self._ego_map_speed = 0.0
        self._last_sensor_xy: np.ndarray | None = None
        self._last_sensor_stamp: float | None = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            DynamicObstacleClusterArray, self._output_topic, 10
        )
        self.create_subscription(
            PointCloud2, self._input_topic, self._cloud_cb, qos_profile_sensor_data
        )
        self.create_subscription(OccupancyGrid, self._map_topic, self._map_cb, map_qos)
        if self._ego_velocity_rejection:
            self.create_subscription(
                Odometry, self._odom_topic, self._odom_cb, qos_profile_sensor_data
            )

        self.get_logger().info(
            f"dynamic_cluster_detector: {self._input_topic} -> {self._output_topic} "
            f"(static_map={self._use_static_map}, require_map={self._require_static_map}, "
            f"shadow_management={self._shadow_management_enabled}, "
            f"ego_rejection={self._ego_velocity_rejection}, "
            f"out_of_map_filter={self._filter_out_of_map})"
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self._ego_vx = msg.twist.twist.linear.x
        self._ego_vy = msg.twist.twist.linear.y
        self._ego_speed = math.hypot(self._ego_vx, self._ego_vy)
        self._ego_yaw_rate = abs(msg.twist.twist.angular.z)

    def _is_static_point(self, x: float, y: float) -> bool:
        if not self._use_static_map or self._map is None:
            return False
        info = self._map.info
        res = info.resolution
        if res <= 0.0:
            return False
        ox = info.origin.position.x
        oy = info.origin.position.y
        mx = int(math.floor((x - ox) / res))
        my = int(math.floor((y - oy) / res))
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            # Points outside the map bounds are treated as static (unknown/wall).
            return self._filter_out_of_map
        radius_cells = max(0, int(math.ceil(self._static_inflation_radius / res)))
        for yy in range(max(0, my - radius_cells), min(info.height, my + radius_cells + 1)):
            row = yy * info.width
            for xx in range(max(0, mx - radius_cells), min(info.width, mx + radius_cells + 1)):
                if self._map.data[row + xx] >= self._occupied_threshold:
                    return True
        return False

    def _foreground_points(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points.reshape((0, 3))
        if not self._use_static_map or self._map is None:
            return points
        keep = [not self._is_static_point(float(p[0]), float(p[1])) for p in points]
        return points[np.array(keep, dtype=bool)]

    def _sensor_xy(self, msg: PointCloud2) -> np.ndarray | None:
        if not self._shadow_management_enabled:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                msg.header.frame_id,
                self._sensor_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self._shadow_tf_timeout),
            )
        except TransformException:
            return None
        t = transform.transform.translation
        return np.array([t.x, t.y], dtype=np.float64)

    def _expire_shadow_cones(self, stamp: float) -> None:
        self._shadow_cones = [cone for cone in self._shadow_cones if cone.expires_at >= stamp]

    def _update_sensor_motion(self, sensor_xy: np.ndarray | None, stamp: float) -> None:
        if sensor_xy is None:
            return
        if self._last_sensor_xy is not None and self._last_sensor_stamp is not None:
            dt = stamp - self._last_sensor_stamp
            if 1.0e-3 <= dt <= 0.5:
                measured = (sensor_xy - self._last_sensor_xy) / dt
                self._ego_map_velocity = 0.6 * self._ego_map_velocity + 0.4 * measured
                self._ego_map_speed = float(np.linalg.norm(self._ego_map_velocity))
        self._last_sensor_xy = sensor_xy
        self._last_sensor_stamp = stamp

    def _remove_shadowed_points(self, points: np.ndarray) -> np.ndarray:
        if (
            not self._shadow_management_enabled
            or not self._shadow_cones
            or points.size == 0
        ):
            return points.reshape((-1, 3))
        shadowed = points_in_shadow_cones(points, self._shadow_cones)
        return points[~shadowed]

    def _is_shadowed_cluster(
        self,
        centroid: np.ndarray,
        points: np.ndarray,
        cones: list[ShadowCone],
    ) -> bool:
        if not self._shadow_management_enabled or not cones:
            return False
        centroid_shadowed = bool(points_in_shadow_cones(centroid.reshape((1, 3)), cones)[0])
        if centroid_shadowed:
            return True
        if points.size == 0:
            return False
        point_mask = points_in_shadow_cones(points, cones)
        return float(np.count_nonzero(point_mask)) / float(points.shape[0]) >= 0.5

    def _make_shadow_cone(
        self,
        sensor_xy: np.ndarray | None,
        centroid: np.ndarray,
        size: np.ndarray,
        stamp: float,
    ) -> ShadowCone | None:
        if not self._shadow_management_enabled or sensor_xy is None:
            return None
        return shadow_cone_from_obstacle(
            sensor_xy=sensor_xy,
            obstacle_xy=centroid[:2],
            obstacle_size_xy=size[:2],
            stamp=stamp,
            angle_margin=self._shadow_angle_margin,
            range_padding=self._shadow_range_padding,
            max_range=self._shadow_max_range,
            persistence=self._shadow_persistence,
        )

    def _associate(self, centroids: list[np.ndarray]) -> dict[int, int]:
        assignments: dict[int, int] = {}
        if not centroids:
            return assignments
        available = set(self._previous.keys())
        for idx, centroid in enumerate(centroids):
            best_id = None
            best_dist = self._association_distance
            for prev_id in available:
                dist = float(np.linalg.norm(centroid[:2] - self._previous[prev_id].centroid[:2]))
                if dist < best_dist:
                    best_id = prev_id
                    best_dist = dist
            if best_id is not None:
                assignments[idx] = best_id
                available.remove(best_id)
        return assignments

    def _cluster_shape(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        centroid = np.mean(pts, axis=0)
        mins = np.min(pts, axis=0)
        maxs = np.max(pts, axis=0)
        size = np.maximum(maxs - mins, [self._min_cluster_size_xy, self._min_cluster_size_xy, 0.05])
        yaw = 0.0
        if pts.shape[0] >= 3:
            centered = pts[:, :2] - centroid[:2]
            cov = centered.T @ centered / max(1, pts.shape[0] - 1)
            eigvals, eigvecs = np.linalg.eigh(cov)
            axis = eigvecs[:, int(np.argmax(eigvals))]
            yaw = math.atan2(float(axis[1]), float(axis[0]))
        return centroid, size, yaw

    def _cloud_cb(self, msg: PointCloud2) -> None:
        out = DynamicObstacleClusterArray()
        out.header = msg.header
        if self._use_static_map and self._require_static_map and self._map is None:
            self._publisher.publish(out)
            return

        stamp = stamp_seconds(msg.header.stamp)
        self._expire_shadow_cones(stamp)
        sensor_xy = self._sensor_xy(msg)
        self._update_sensor_motion(sensor_xy, stamp)
        points = self._foreground_points(points_from_cloud(msg))
        points = self._remove_shadowed_points(points)
        clusters = euclidean_clusters(
            points,
            self._cluster_tolerance,
            self._min_cluster_points,
            self._max_cluster_points,
            self._max_clusters,
        )

        shapes = [self._cluster_shape(points[idx]) for idx in clusters]
        centroids = [shape[0] for shape in shapes]
        assignments = self._associate(centroids)

        seen_ids = set()
        dynamic_candidates: list[
            tuple[float, np.ndarray, np.ndarray, np.ndarray, float, int, np.ndarray, int]
        ] = []
        for idx, (centroid, size, yaw) in enumerate(shapes):
            cluster_id = assignments.get(idx)
            velocity = np.zeros(3, dtype=np.float64)
            dynamic = False
            dynamic_hits = 0
            consecutive_dynamic_hits = 0
            stationary_hits = 1
            if cluster_id is None:
                cluster_id = self._next_id
                self._next_id += 1
            else:
                prev = self._previous[cluster_id]
                stationary_hits = prev.stationary_hits
                dt = max(1.0e-3, stamp - prev.stamp)
                displacement = centroid - prev.centroid
                velocity = displacement / dt
                speed_xy = float(np.linalg.norm(velocity[:2]))
                instant_dynamic = (
                    float(np.linalg.norm(displacement[:2])) >= self._min_dynamic_displacement
                    and speed_xy >= self._min_dynamic_speed
                )

                # Ego-velocity rejection: if the robot is moving, check if
                # the cluster's apparent velocity is consistent with static
                # viewpoint drift (centroid shift proportional to ego-motion).
                if instant_dynamic and self._ego_velocity_rejection:
                    effective_ego_speed = max(self._ego_speed, self._ego_map_speed)
                    ego_moving_linear = effective_ego_speed >= self._ego_min_robot_speed
                    ego_rotating = self._ego_yaw_rate >= 0.05  # ~3 deg/s

                    ego_correlated = velocity_matches_ego_motion(
                        velocity[:2],
                        self._ego_map_velocity,
                        self._ego_min_robot_speed,
                        self._ego_artifact_min_speed_ratio,
                        self._ego_artifact_max_speed_ratio,
                        self._ego_artifact_direction_cosine,
                    )
                    if ego_correlated and (
                        prev.stationary_hits >= 2
                        or speed_xy <= effective_ego_speed * 1.2
                    ):
                        instant_dynamic = False

                    if ego_moving_linear:
                        # A static cluster observed from a moving robot can
                        # exhibit apparent centroid displacement that is a
                        # fraction of the robot's velocity.  Reject if the
                        # cluster's apparent speed is below the threshold
                        # ratio of the robot's speed.
                        if speed_xy < effective_ego_speed * self._ego_speed_ratio_threshold:
                            instant_dynamic = False

                    if ego_rotating and instant_dynamic:
                        # When the robot rotates, static obstacles at range r
                        # exhibit apparent tangential velocity ≈ yaw_rate * r.
                        # If the cluster's speed is below that bound, it is
                        # likely a static viewpoint artifact.
                        if sensor_xy is not None:
                            obs_range = float(np.linalg.norm(
                                centroid[:2] - sensor_xy
                            ))
                        else:
                            obs_range = float(np.linalg.norm(centroid[:2]))
                        rotation_induced_speed = (
                            self._ego_yaw_rate * obs_range
                            * self._ego_speed_ratio_threshold
                        )
                        if speed_xy < rotation_induced_speed:
                            instant_dynamic = False

                if instant_dynamic:
                    dynamic_hits = prev.dynamic_hits + 1
                    consecutive_dynamic_hits = prev.consecutive_dynamic_hits + 1
                    stationary_hits = max(0, prev.stationary_hits - 1)
                else:
                    dynamic_hits = max(0, prev.dynamic_hits - 1)
                    consecutive_dynamic_hits = 0
                    stationary_hits = min(prev.stationary_hits + 1, 1000)
                dynamic = (
                    dynamic_hits >= self._min_dynamic_hits
                    and consecutive_dynamic_hits >= self._min_consecutive_dynamic_hits
                )
                if dynamic:
                    yaw = math.atan2(float(velocity[1]), float(velocity[0]))

            seen_ids.add(cluster_id)
            self._previous[cluster_id] = PreviousCluster(
                centroid=centroid,
                stamp=stamp,
                dynamic_hits=dynamic_hits,
                consecutive_dynamic_hits=consecutive_dynamic_hits,
                stationary_hits=stationary_hits,
            )

            if dynamic:
                cluster_points = points[clusters[idx]]
                sensor_range = (
                    float(np.linalg.norm(centroid[:2] - sensor_xy))
                    if sensor_xy is not None
                    else 0.0
                )
                dynamic_candidates.append(
                    (
                        sensor_range,
                        centroid,
                        size,
                        velocity,
                        yaw,
                        cluster_id,
                        cluster_points,
                        int(cluster_points.shape[0]),
                    )
                )

        for prev_id in list(self._previous.keys()):
            if prev_id not in seen_ids:
                self._previous[prev_id].misses += 1
                if self._previous[prev_id].misses > self._max_track_misses:
                    del self._previous[prev_id]

        frame_cones: list[ShadowCone] = []
        for _, centroid, size, velocity, yaw, cluster_id, cluster_points, point_count in sorted(
            dynamic_candidates,
            key=lambda candidate: candidate[0],
        ):
            if self._is_shadowed_cluster(centroid, cluster_points, frame_cones):
                continue
            cone = self._make_shadow_cone(sensor_xy, centroid, size, stamp)
            if cone is not None:
                frame_cones.append(cone)
                self._shadow_cones.append(cone)

            cluster = DynamicObstacleCluster()
            cluster.id = int(cluster_id)
            cluster.centroid = point_msg(centroid)
            cluster.velocity = vector_msg(velocity)
            cluster.size = vector_msg(size)
            cluster.yaw = float(yaw)
            cluster.point_count = point_count
            cluster.dynamic = True
            out.clusters.append(cluster)

        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = DynamicClusterDetector()
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
