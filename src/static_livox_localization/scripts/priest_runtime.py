"""ROS-free obstacle transformation and unpredictable-actor policy."""

from __future__ import annotations

import math
from typing import NamedTuple, Optional, Sequence

import numpy as np

from body_frame import lidar_to_body
from cluster_guard import Summary, object_box, object_motion
from cluster_tracking import STATIC
from priest_constraints import CANONICAL_FOOTPRINT


WAIT_RADIUS_M = 2.5
OBSTACLE_WAIT = "OBSTACLE_WAIT"


class ObstacleCircle(NamedTuple):
    x_m: float
    y_m: float
    radius_m: float
    motion: str


def _transform_valid(
        map_T_body: np.ndarray,
        lidar_in_body: np.ndarray,
        lidar_to_body_rotation: np.ndarray) -> bool:
    return (
        map_T_body.shape == (4, 4)
        and lidar_in_body.shape == (3,)
        and np.isfinite(map_T_body).all()
        and np.isfinite(lidar_in_body).all()
        and np.allclose(
            map_T_body[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-6)
        and _rotation_valid(map_T_body[:3, :3])
        and _rotation_valid(lidar_to_body_rotation)
    )


def _rotation_valid(rotation: np.ndarray) -> bool:
    return (
        rotation.shape == (3, 3)
        and np.isfinite(rotation).all()
        and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        and math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6)
    )


def _map_circle(
        item: dict,
        map_T_body: np.ndarray,
        lidar_in_body: np.ndarray,
        lidar_to_body_rotation: np.ndarray) -> Optional[ObstacleCircle]:
    box = object_box(item)
    if box is None:
        return None
    x, y, half_x, half_y = box
    body = lidar_to_body(
        np.array([[x, y, 0.0]], dtype=np.float64),
        lidar_in_body,
        lidar_to_body_rotation)[0]
    world = map_T_body[:3, :3] @ body + map_T_body[:3, 3]
    return ObstacleCircle(
        float(world[0]), float(world[1]), math.hypot(half_x, half_y),
        object_motion(item))


def planner_obstacles(
        objects: Sequence[dict],
        map_T_body: np.ndarray,
        lidar_in_body: Sequence[float],
        lidar_to_body_rotation: np.ndarray,
        limit: int = 24) -> tuple[list[list[float]], int]:
    """Return nearest confirmed-STATIC circles; uncertainty is never routed."""
    transform = np.asarray(map_T_body, dtype=np.float64)
    offset = np.asarray(lidar_in_body, dtype=np.float64)
    rotation = np.asarray(lidar_to_body_rotation, dtype=np.float64)
    integral_limit = (
        isinstance(limit, (int, np.integer))
        and not isinstance(limit, (bool, np.bool_)) and int(limit) > 0)
    if not _transform_valid(transform, offset, rotation) or not integral_limit:
        raise ValueError(
            "rigid obstacle transform and positive integral limit are required")
    circles = []
    for item in objects:
        if not isinstance(item, dict) or object_motion(item) != STATIC:
            continue
        circle = _map_circle(item, transform, offset, rotation)
        if circle is not None:
            circles.append(circle)
    circles.sort(key=lambda circle: math.hypot(
        circle.x_m - transform[0, 3], circle.y_m - transform[1, 3]))
    kept = circles[:int(limit)]
    return ([[circle.x_m, circle.y_m, circle.radius_m]
             for circle in kept], max(0, len(circles) - len(kept)))


def _lookahead_polyline(
        current_xy: np.ndarray,
        trajectory_xy: np.ndarray,
        reach_m: float,
        start_index: int) -> Optional[np.ndarray]:
    integral_index = (
        isinstance(start_index, (int, np.integer))
        and not isinstance(start_index, (bool, np.bool_)))
    if (current_xy.shape != (2,) or trajectory_xy.ndim != 2
            or trajectory_xy.shape[1:] != (2,)
            or not np.isfinite(current_xy).all()
            or not np.isfinite(trajectory_xy).all()
            or not math.isfinite(reach_m) or reach_m <= 0.0
            or not integral_index or start_index < 0
            or (len(trajectory_xy) and start_index >= len(trajectory_xy))):
        return None
    forward = trajectory_xy[int(start_index):]
    has_motion = len(forward) >= 2 and bool(np.any(
        np.linalg.norm(np.diff(forward, axis=0), axis=1) > 1e-9))
    if not has_motion:
        return current_xy[None, :]
    points = [current_xy]
    remaining = reach_m
    for target in forward:
        delta = target - points[-1]
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-9:
            continue
        if distance >= remaining:
            points.append(points[-1] + delta * (remaining / distance))
            break
        points.append(target)
        remaining -= distance
    return np.asarray(points, dtype=np.float64)


def _distance_to_polyline(point: np.ndarray, polyline: np.ndarray) -> float:
    if len(polyline) == 1:
        return float(np.linalg.norm(point - polyline[0]))
    start = polyline[:-1]
    delta = polyline[1:] - start
    length_sq = np.einsum("ij,ij->i", delta, delta)
    fraction = np.einsum("ij,ij->i", point - start, delta) \
        / np.maximum(length_sq, 1e-12)
    closest = start + np.clip(fraction, 0.0, 1.0)[:, None] * delta
    return float(np.linalg.norm(closest - point, axis=1).min())


def wait_reason(
        summary: Optional[Summary],
        map_T_body: np.ndarray,
        lidar_in_body: Sequence[float],
        lidar_to_body_rotation: np.ndarray,
        current_xy: np.ndarray,
        trajectory_xy: np.ndarray,
        wait_radius_m: float = WAIT_RADIUS_M,
        trajectory_start_index: int = 0) -> Optional[str]:
    """Hold on full-extent MOVING/UNKNOWN conflicts; never predict them."""
    if summary is None or not summary.usable:
        return OBSTACLE_WAIT
    transform = np.asarray(map_T_body, dtype=np.float64)
    offset = np.asarray(lidar_in_body, dtype=np.float64)
    rotation = np.asarray(lidar_to_body_rotation, dtype=np.float64)
    current = np.asarray(current_xy, dtype=np.float64)
    trajectory = np.asarray(trajectory_xy, dtype=np.float64)
    if not _transform_valid(transform, offset, rotation):
        return OBSTACLE_WAIT
    lookahead = _lookahead_polyline(
        current, trajectory, wait_radius_m, trajectory_start_index)
    if lookahead is None:
        return OBSTACLE_WAIT
    has_trajectory = len(lookahead) > 1
    chair_radius = (
        CANONICAL_FOOTPRINT.circumscribed_radius_m
        + CANONICAL_FOOTPRINT.sweep_margin_m)
    for item in summary.objects:
        if not isinstance(item, dict):
            return OBSTACLE_WAIT
        circle = _map_circle(item, transform, offset, rotation)
        if circle is None:
            return OBSTACLE_WAIT
        if circle.motion == STATIC:
            continue
        centre = np.array([circle.x_m, circle.y_m])
        clearance_radius = chair_radius + circle.radius_m
        if has_trajectory:
            conflict = _distance_to_polyline(centre, lookahead) \
                <= clearance_radius
        else:
            conflict = float(np.linalg.norm(centre - current)) \
                - clearance_radius <= wait_radius_m
        if conflict:
            return OBSTACLE_WAIT
    return None
