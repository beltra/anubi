import math

import numpy as np

from robot_nav.dynamic_obstacles import (
    oriented_footprint_points,
    points_in_shadow_cones,
    shadow_cone_from_obstacle,
    track_is_publishable,
    velocity_matches_ego_motion,
    yaw_from_quaternion,
    yaw_to_quaternion,
)


def _span(points: np.ndarray) -> tuple[float, float]:
    mins = np.min(points[:, :2], axis=0)
    maxs = np.max(points[:, :2], axis=0)
    return float(maxs[0] - mins[0]), float(maxs[1] - mins[1])


def test_yaw_quaternion_round_trip() -> None:
    yaw = 1.35
    assert math.isclose(yaw_from_quaternion(yaw_to_quaternion(yaw)), yaw, abs_tol=1.0e-9)


def test_oriented_footprint_uses_box_extents() -> None:
    points = oriented_footprint_points(
        x=0.0,
        y=0.0,
        yaw=0.0,
        length=2.8,
        width=1.2,
        resolution=0.2,
        z=0.45,
        padding=0.1,
    )

    span_x, span_y = _span(points)
    assert 2.9 <= span_x <= 3.0
    assert 1.3 <= span_y <= 1.4
    assert np.allclose(points[:, 2], 0.45)


def test_oriented_footprint_rotates_points() -> None:
    points = oriented_footprint_points(
        x=1.0,
        y=-2.0,
        yaw=math.pi / 2.0,
        length=1.0,
        width=0.4,
        resolution=0.1,
        z=0.2,
        padding=0.0,
    )

    span_x, span_y = _span(points)
    assert span_x < span_y
    assert math.isclose(float(np.mean(points[:, 0])), 1.0, abs_tol=1.0e-9)
    assert math.isclose(float(np.mean(points[:, 1])), -2.0, abs_tol=1.0e-9)


def test_shadow_cone_masks_only_occluded_points() -> None:
    cone = shadow_cone_from_obstacle(
        sensor_xy=np.array([0.0, 0.0]),
        obstacle_xy=np.array([2.0, 0.0]),
        obstacle_size_xy=np.array([0.6, 0.6]),
        stamp=10.0,
        angle_margin=0.02,
        range_padding=0.10,
        max_range=6.0,
        persistence=0.5,
    )

    assert cone is not None
    points = np.array(
        [
            [2.2, 0.0, 0.5],  # still inside the blocking obstacle footprint
            [3.4, 0.05, 0.5],  # behind the obstacle along the same ray
            [3.4, 1.5, 0.5],  # behind in range, outside the angular cone
        ],
        dtype=np.float64,
    )

    mask = points_in_shadow_cones(points, [cone])
    assert mask.tolist() == [False, True, False]
    assert math.isclose(cone.expires_at, 10.5, abs_tol=1.0e-9)


def test_velocity_matches_ego_motion_requires_direction_and_ratio() -> None:
    ego = np.array([0.4, 0.0])

    assert velocity_matches_ego_motion(
        np.array([0.35, 0.02]),
        ego,
        min_ego_speed=0.08,
        min_speed_ratio=0.35,
        max_speed_ratio=1.8,
        direction_cosine=0.75,
    )
    assert velocity_matches_ego_motion(
        np.array([-0.35, -0.02]),
        ego,
        min_ego_speed=0.08,
        min_speed_ratio=0.35,
        max_speed_ratio=1.8,
        direction_cosine=0.75,
    )
    assert not velocity_matches_ego_motion(
        np.array([0.0, 0.35]),
        ego,
        min_ego_speed=0.08,
        min_speed_ratio=0.35,
        max_speed_ratio=1.8,
        direction_cosine=0.75,
    )
    assert not velocity_matches_ego_motion(
        np.array([1.2, 0.0]),
        ego,
        min_ego_speed=0.08,
        min_speed_ratio=0.35,
        max_speed_ratio=1.8,
        direction_cosine=0.75,
    )


def test_track_publish_requires_recent_confident_observations() -> None:
    assert track_is_publishable(
        hits=4,
        misses=0,
        confirmation_hits=4,
        max_publish_misses=0,
        confidence=0.75,
        min_confidence=0.6,
    )
    assert not track_is_publishable(
        hits=4,
        misses=1,
        confirmation_hits=4,
        max_publish_misses=0,
        confidence=0.75,
        min_confidence=0.6,
    )
    assert not track_is_publishable(
        hits=4,
        misses=0,
        confirmation_hits=4,
        max_publish_misses=0,
        confidence=0.50,
        min_confidence=0.6,
    )
