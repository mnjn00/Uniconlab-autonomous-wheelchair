"""ROS-free final certification for trajectories proposed by PRIEST."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from priest_constraints import (
    CANONICAL_FOOTPRINT,
    ConstraintTolerances,
    ConstraintViolations,
)


class BandContainment(Protocol):
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        ...


class PlannerLimits(Protocol):
    v_max: float
    a_max: float
    yaw_rate_max: float
    CONSTRAINT_TOLERANCES: ConstraintTolerances


@dataclass(frozen=True)
class TrajectoryCertificate:
    __slots__ = (
        "reason", "violations", "runtime_band_contained",
        "min_obstacle_clearance_m", "max_speed_mps",
        "max_acceleration_mps2", "max_yaw_rate_rps")

    reason: str
    violations: ConstraintViolations
    runtime_band_contained: bool
    min_obstacle_clearance_m: float
    max_speed_mps: float
    max_acceleration_mps2: float
    max_yaw_rate_rps: float

    @property
    def usable(self) -> bool:
        return self.reason == ""


def _reason(
        band_contained: bool,
        violations: ConstraintViolations,
        tolerances: ConstraintTolerances) -> str:
    if not band_contained:
        return "OUTSIDE_RUNTIME_BAND"
    if violations.obstacle_m > tolerances.obstacle_m:
        return "OBSTACLE_CLEARANCE"
    if violations.speed_mps > tolerances.speed_mps:
        return "SPEED_LIMIT"
    if violations.acceleration_mps2 > tolerances.acceleration_mps2:
        return "ACCELERATION_LIMIT"
    if violations.yaw_rate_rps > tolerances.yaw_rate_rps:
        return "YAW_RATE_LIMIT"
    return ""


def certify_trajectory(
        planner: PlannerLimits,
        *,
        points: np.ndarray,
        times_s: np.ndarray,
        band: BandContainment,
        obstacles: np.ndarray) -> TrajectoryCertificate:
    """Reject a proposed centre path unless every executable bound holds."""
    xy = np.asarray(points, dtype=np.float64)
    times = np.asarray(times_s, dtype=np.float64)
    circles = np.asarray(obstacles, dtype=np.float64)
    if (xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2
            or times.shape != (len(xy),)
            or not np.isfinite(xy).all() or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0.0)):
        raise ValueError("trajectory must contain finite Nx2 points at increasing times")
    if circles.size == 0:
        circles = np.empty((0, 3), dtype=np.float64)
    if (circles.ndim != 2 or circles.shape[1] != 3
            or not np.isfinite(circles).all()
            or np.any(circles[:, 2] < 0.0)):
        raise ValueError("obstacles must be finite non-negative-radius circles")

    contained = np.asarray(band.contains_many(xy, grace=0.0), dtype=bool)
    if contained.shape != (len(xy),):
        raise ValueError("band containment must return one result per point")
    band_contained = bool(contained.all())

    delta_t = np.diff(times)
    velocity = np.diff(xy, axis=0) / delta_t[:, None]
    speed = np.linalg.norm(velocity, axis=1)
    max_speed = float(speed.max())
    if len(velocity) > 1:
        interval_t = 0.5 * (delta_t[:-1] + delta_t[1:])
        acceleration = np.diff(velocity, axis=0) / interval_t[:, None]
        max_acceleration = float(np.linalg.norm(acceleration, axis=1).max())
        heading = np.unwrap(np.arctan2(velocity[:, 1], velocity[:, 0]))
        max_yaw_rate = float((np.abs(np.diff(heading)) / interval_t).max())
    else:
        max_acceleration = 0.0
        max_yaw_rate = 0.0

    if len(circles):
        separation = np.linalg.norm(
            xy[:, None, :] - circles[None, :, :2], axis=2)
        required = (
            circles[None, :, 2]
            + CANONICAL_FOOTPRINT.circumscribed_radius_m
            + CANONICAL_FOOTPRINT.planning_margin_m
        )
        min_clearance = float((separation - required).min())
    else:
        min_clearance = float("inf")

    violations = ConstraintViolations(
        obstacle_m=max(0.0, -min_clearance),
        corridor_m=0.0 if band_contained else float("inf"),
        speed_mps=max(0.0, max_speed - planner.v_max),
        acceleration_mps2=max(0.0, max_acceleration - planner.a_max),
        yaw_rate_rps=max(0.0, max_yaw_rate - planner.yaw_rate_max),
    )
    return TrajectoryCertificate(
        reason=_reason(band_contained, violations, planner.CONSTRAINT_TOLERANCES),
        violations=violations,
        runtime_band_contained=band_contained,
        min_obstacle_clearance_m=min_clearance,
        max_speed_mps=max_speed,
        max_acceleration_mps2=max_acceleration,
        max_yaw_rate_rps=max_yaw_rate,
    )
