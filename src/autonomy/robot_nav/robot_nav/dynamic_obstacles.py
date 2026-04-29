"""Utilities for LiDAR dynamic obstacle processing.

The helpers in this module are intentionally ROS-light: message conversion is
kept at the edges, while clustering and tracking math use NumPy arrays.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from robot_nav.pointcloud_utils import build_pc2_dtype, stamp_to_sec


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_to_matrix(transform) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = quaternion_to_matrix(transform.transform.rotation)
    t = transform.transform.translation
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def points_from_cloud(msg: PointCloud2) -> np.ndarray:
    if msg.width * msg.height == 0:
      return np.empty((0, 3), dtype=np.float64)
    names = [field.name for field in msg.fields]
    if not {"x", "y", "z"}.issubset(names):
      return np.empty((0, 3), dtype=np.float64)
    raw = np.frombuffer(msg.data, dtype=build_pc2_dtype(msg))
    pts = np.column_stack(
        [
            raw["x"].astype(np.float64),
            raw["y"].astype(np.float64),
            raw["z"].astype(np.float64),
        ]
    )
    return pts[np.isfinite(pts).all(axis=1)]


def cloud_from_points(header, points: np.ndarray) -> PointCloud2:
    pts = points.astype(np.float32, copy=False)
    return point_cloud2.create_cloud_xyz32(header, pts.tolist())


def transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.reshape((0, 3))
    hom = np.ones((points.shape[0], 4), dtype=np.float64)
    hom[:, :3] = points
    return (matrix @ hom.T).T[:, :3]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.size == 0 or voxel_size <= 0.0:
        return points.reshape((-1, 3))
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(unique_idx)]


def filter_height(points: np.ndarray, min_z: float, max_z: float) -> np.ndarray:
    if points.size == 0:
        return points.reshape((0, 3))
    mask = (
        np.isfinite(points).all(axis=1)
        & (points[:, 2] >= min_z)
        & (points[:, 2] <= max_z)
    )
    return points[mask]


def oriented_footprint_points(
    x: float,
    y: float,
    yaw: float,
    length: float,
    width: float,
    resolution: float,
    z: float,
    padding: float = 0.0,
) -> np.ndarray:
    step = max(0.05, resolution)
    half_length = max(0.05, 0.5 * length + padding)
    half_width = max(0.05, 0.5 * width + padding)
    nx = max(1, int(math.ceil(half_length / step)))
    ny = max(1, int(math.ceil(half_width / step)))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    points: list[list[float]] = []
    for ix in range(-nx, nx + 1):
        local_x = max(-half_length, min(half_length, ix * step))
        for iy in range(-ny, ny + 1):
            local_y = max(-half_width, min(half_width, iy * step))
            px = x + cos_yaw * local_x - sin_yaw * local_y
            py = y + sin_yaw * local_x + cos_yaw * local_y
            points.append([px, py, z])
    return np.array(points, dtype=np.float64).reshape((-1, 3))


def euclidean_clusters(
    points: np.ndarray,
    tolerance: float,
    min_points: int,
    max_points: int,
    max_clusters: int,
) -> list[np.ndarray]:
    if points.shape[0] < min_points:
        return []

    cell = max(tolerance, 1.0e-3)
    xy = points[:, :2]
    cells: dict[tuple[int, int], list[int]] = {}
    for idx, p in enumerate(xy):
        key = (math.floor(p[0] / cell), math.floor(p[1] / cell))
        cells.setdefault(key, []).append(idx)

    visited = np.zeros(points.shape[0], dtype=bool)
    clusters: list[np.ndarray] = []
    tol_sq = tolerance * tolerance

    for seed in range(points.shape[0]):
        if visited[seed]:
            continue
        queue = [seed]
        visited[seed] = True
        members: list[int] = []

        while queue:
            current = queue.pop()
            members.append(current)
            cx, cy = xy[current]
            key = (math.floor(cx / cell), math.floor(cy / cell))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for candidate in cells.get((key[0] + dx, key[1] + dy), []):
                        if visited[candidate]:
                            continue
                        delta = xy[candidate] - xy[current]
                        if float(delta @ delta) <= tol_sq:
                            visited[candidate] = True
                            queue.append(candidate)

        if min_points <= len(members) <= max_points:
            clusters.append(np.array(members, dtype=np.int64))
            if len(clusters) >= max_clusters:
                break

    return clusters


def point_msg(values: Iterable[float]) -> Point:
    vals = list(values)
    return Point(x=float(vals[0]), y=float(vals[1]), z=float(vals[2]))


def vector_msg(values: Iterable[float]) -> Vector3:
    vals = list(values)
    return Vector3(x=float(vals[0]), y=float(vals[1]), z=float(vals[2]))


def pose_from_xy_yaw(x: float, y: float, z: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = yaw_to_quaternion(yaw)
    return pose


@dataclass
class TrackState:
    track_id: int
    x: np.ndarray
    p: np.ndarray
    size: np.ndarray
    hits: int = 1
    misses: int = 0
    age: int = 1
    history: list[bool] = field(default_factory=lambda: [True])
    last_stamp: float = 0.0

    @property
    def confirmed(self) -> bool:
        return self.hits >= 3


def track_is_publishable(
    hits: int,
    misses: int,
    confirmation_hits: int,
    max_publish_misses: int,
    confidence: float,
    min_confidence: float,
) -> bool:
    return (
        hits >= confirmation_hits
        and misses <= max_publish_misses
        and confidence >= min_confidence
    )


@dataclass(frozen=True)
class ShadowCone:
    origin: np.ndarray
    bearing: float
    half_angle: float
    min_range: float
    max_range: float
    expires_at: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def shadow_cone_from_obstacle(
    sensor_xy: np.ndarray,
    obstacle_xy: np.ndarray,
    obstacle_size_xy: np.ndarray,
    stamp: float,
    angle_margin: float,
    range_padding: float,
    max_range: float,
    persistence: float,
) -> ShadowCone | None:
    origin = np.asarray(sensor_xy, dtype=np.float64).reshape((2,))
    center = np.asarray(obstacle_xy, dtype=np.float64).reshape((2,))
    size = np.asarray(obstacle_size_xy, dtype=np.float64).reshape((2,))
    delta = center - origin
    distance = float(np.linalg.norm(delta))
    if distance <= 1.0e-3:
        return None

    half_extent = 0.5 * float(np.linalg.norm(np.maximum(size, 0.0)))
    padded_extent = max(0.01, half_extent + max(0.0, range_padding))
    half_angle = math.asin(min(0.95, padded_extent / distance)) + max(0.0, angle_margin)
    min_range = distance + padded_extent
    return ShadowCone(
        origin=origin,
        bearing=math.atan2(float(delta[1]), float(delta[0])),
        half_angle=half_angle,
        min_range=min_range,
        max_range=max(min_range, max_range),
        expires_at=stamp + max(0.0, persistence),
    )


def points_in_shadow_cones(points: np.ndarray, cones: Iterable[ShadowCone]) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    mask = np.zeros(pts.shape[0], dtype=bool)
    if pts.shape[0] == 0:
        return mask

    xy = pts[:, :2]
    for cone in cones:
        delta = xy - cone.origin
        ranges = np.linalg.norm(delta, axis=1)
        bearings = np.arctan2(delta[:, 1], delta[:, 0])
        angle_error = np.abs(np.arctan2(np.sin(bearings - cone.bearing), np.cos(bearings - cone.bearing)))
        mask |= (
            (ranges >= cone.min_range)
            & (ranges <= cone.max_range)
            & (angle_error <= cone.half_angle)
        )
    return mask


def velocity_matches_ego_motion(
    velocity_xy: np.ndarray,
    ego_velocity_xy: np.ndarray,
    min_ego_speed: float,
    min_speed_ratio: float,
    max_speed_ratio: float,
    direction_cosine: float,
) -> bool:
    velocity = np.asarray(velocity_xy, dtype=np.float64).reshape((2,))
    ego_velocity = np.asarray(ego_velocity_xy, dtype=np.float64).reshape((2,))
    speed = float(np.linalg.norm(velocity))
    ego_speed = float(np.linalg.norm(ego_velocity))
    if speed <= 1.0e-6 or ego_speed < min_ego_speed:
        return False

    ratio = speed / ego_speed
    if ratio < min_speed_ratio or ratio > max_speed_ratio:
        return False

    cosine = float((velocity @ ego_velocity) / (speed * ego_speed))
    return abs(cosine) >= direction_cosine


def ctrv_predict_state(x: np.ndarray, dt: float, yaw_rate_eps: float) -> np.ndarray:
    out = x.copy()
    px, py, yaw, speed, yaw_rate = x
    if abs(yaw_rate) < yaw_rate_eps:
        out[0] = px + speed * dt * math.cos(yaw)
        out[1] = py + speed * dt * math.sin(yaw)
    else:
        yaw_next = yaw + yaw_rate * dt
        out[0] = px + (speed / yaw_rate) * (math.sin(yaw_next) - math.sin(yaw))
        out[1] = py + (speed / yaw_rate) * (-math.cos(yaw_next) + math.cos(yaw))
    out[2] = normalize_angle(yaw + yaw_rate * dt)
    out[3] = speed
    out[4] = yaw_rate
    return out


def ctrv_jacobian(x: np.ndarray, dt: float, yaw_rate_eps: float) -> np.ndarray:
    _, _, yaw, speed, yaw_rate = x
    f = np.eye(5, dtype=np.float64)
    if abs(yaw_rate) < yaw_rate_eps:
        f[0, 2] = -speed * dt * math.sin(yaw)
        f[0, 3] = dt * math.cos(yaw)
        f[1, 2] = speed * dt * math.cos(yaw)
        f[1, 3] = dt * math.sin(yaw)
    else:
        yaw_next = yaw + yaw_rate * dt
        f[0, 2] = (speed / yaw_rate) * (math.cos(yaw_next) - math.cos(yaw))
        f[0, 3] = (math.sin(yaw_next) - math.sin(yaw)) / yaw_rate
        f[0, 4] = (
            speed * dt * math.cos(yaw_next) / yaw_rate
            - speed * (math.sin(yaw_next) - math.sin(yaw)) / (yaw_rate * yaw_rate)
        )
        f[1, 2] = (speed / yaw_rate) * (math.sin(yaw_next) - math.sin(yaw))
        f[1, 3] = (-math.cos(yaw_next) + math.cos(yaw)) / yaw_rate
        f[1, 4] = (
            speed * dt * math.sin(yaw_next) / yaw_rate
            - speed * (-math.cos(yaw_next) + math.cos(yaw)) / (yaw_rate * yaw_rate)
        )
    f[2, 4] = dt
    return f


def track_to_pose(track: TrackState, z: float = 0.0) -> Pose:
    return pose_from_xy_yaw(track.x[0], track.x[1], z, track.x[2])


def stamp_plus_seconds(stamp, seconds: float):
    base = Time.from_msg(stamp).nanoseconds
    return Time(nanoseconds=base + int(seconds * 1.0e9)).to_msg()


def stamp_seconds(stamp) -> float:
    return stamp_to_sec(stamp)
