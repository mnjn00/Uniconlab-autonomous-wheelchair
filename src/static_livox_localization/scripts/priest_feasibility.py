"""ROS-free final certification for trajectories proposed by PRIEST."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Protocol, Sequence

import numpy as np

from priest_constraints import (
    CANONICAL_FOOTPRINT,
    ConstraintTolerances,
    ConstraintViolations,
)
from priest_projection import bernstein_basis


MIN_CERTIFICATE_HZ = 10.0
MAX_RUNTIME_BAND_GRACE_M = 0.10


class BandContainment(Protocol):
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        ...


class PlannerLimits(Protocol):
    v_max: float
    a_max: float
    yaw_rate_max: float
    CONSTRAINT_TOLERANCES: ConstraintTolerances


def validated_certificate_settings(
        control_hz: float, band_grace_m: float) -> tuple[float, float]:
    rate = float(control_hz)
    grace = float(band_grace_m)
    if (not math.isfinite(rate) or rate < MIN_CERTIFICATE_HZ
            or not math.isfinite(grace) or not 0.0 <= grace
            <= MAX_RUNTIME_BAND_GRACE_M):
        raise ValueError("certificate rate/grace would weaken runtime safety")
    return rate, grace


def require_physical_limits(planner: PlannerLimits) -> None:
    limits = (planner.v_max, planner.a_max, planner.yaw_rate_max)
    if not all(math.isfinite(value) and value > 0.0 for value in limits):
        raise ValueError("physical limits must be finite and positive")


@dataclass(frozen=True)
class TrajectoryCertificate:
    __slots__ = (
        "reason", "violations", "tolerances", "runtime_band_contained",
        "min_obstacle_clearance_m", "max_speed_mps",
        "max_acceleration_mps2", "max_yaw_rate_rps")

    reason: str
    violations: ConstraintViolations
    tolerances: ConstraintTolerances
    runtime_band_contained: bool
    min_obstacle_clearance_m: float
    max_speed_mps: float
    max_acceleration_mps2: float
    max_yaw_rate_rps: float

    @property
    def usable(self) -> bool:
        return self.reason == ""

    @staticmethod
    def clear(tolerances: ConstraintTolerances) -> TrajectoryCertificate:
        return TrajectoryCertificate(
            "", ConstraintViolations(0.0, 0.0, 0.0, 0.0, 0.0),
            tolerances, True,
            float("inf"), 0.0, 0.0, 0.0)

    @staticmethod
    def refusal(
            reason: str,
            tolerances: ConstraintTolerances) -> TrajectoryCertificate:
        return TrajectoryCertificate(
            reason,
            ConstraintViolations(0.0, float("inf"), 0.0, 0.0, 0.0),
            tolerances, False,
            float("inf"), 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CertifiedTrajectory:
    __slots__ = (
        "points", "times_s", "velocity_xy_mps", "acceleration_xy_mps2",
        "yaw_rad", "yaw_rate_rps", "certificate")

    points: np.ndarray
    times_s: np.ndarray
    velocity_xy_mps: np.ndarray
    acceleration_xy_mps2: np.ndarray
    yaw_rad: np.ndarray
    yaw_rate_rps: np.ndarray
    certificate: TrajectoryCertificate


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


def _circles(obstacles: np.ndarray) -> np.ndarray:
    circles = np.asarray(obstacles, dtype=np.float64)
    if circles.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if (circles.ndim != 2 or circles.shape[1] != 3
            or not np.isfinite(circles).all()
            or np.any(circles[:, 2] < 0.0)):
        raise ValueError("obstacles must be finite non-negative-radius circles")
    return circles


def _yaw_evidence(
        velocity: np.ndarray,
        acceleration: np.ndarray,
        times_s: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    speed_sq = np.einsum("ij,ij->i", velocity, velocity)
    moving = speed_sq > 1e-12
    yaw = np.zeros(len(velocity), dtype=np.float64)
    if np.any(moving):
        moving_yaw = np.unwrap(np.arctan2(
            velocity[moving, 1], velocity[moving, 0]))
        yaw = np.interp(times_s, times_s[moving], moving_yaw)
    cross = velocity[:, 0] * acceleration[:, 1] \
        - velocity[:, 1] * acceleration[:, 0]
    curvature_rate = np.zeros(len(velocity), dtype=np.float64)
    curvature_rate[moving] = cross[moving] / speed_sq[moving]
    reference_rate = np.gradient(yaw, times_s)
    yaw_rate = np.where(
        np.abs(curvature_rate) >= np.abs(reference_rate),
        curvature_rate, reference_rate)
    return yaw, yaw_rate


def _certificate(
        planner: PlannerLimits,
        points: np.ndarray,
        velocity: np.ndarray,
        acceleration: np.ndarray,
        yaw_rate: np.ndarray,
        band: BandContainment,
        obstacles: np.ndarray,
        band_grace_m: float) -> TrajectoryCertificate:
    require_physical_limits(planner)
    if (not math.isfinite(band_grace_m) or band_grace_m < 0.0
            or band_grace_m > MAX_RUNTIME_BAND_GRACE_M):
        raise ValueError("band grace exceeds runtime containment semantics")
    try:
        contained = np.asarray(
            band.contains_many(points, grace=band_grace_m))
    except (TypeError, ValueError):
        return TrajectoryCertificate.refusal(
            "RUNTIME_BAND_INVALID", planner.CONSTRAINT_TOLERANCES)
    if contained.shape != (len(points),) or contained.dtype.kind != "b":
        return TrajectoryCertificate.refusal(
            "RUNTIME_BAND_INVALID", planner.CONSTRAINT_TOLERANCES)
    band_contained = bool(contained.all())
    max_speed = float(np.linalg.norm(velocity, axis=1).max())
    max_acceleration = float(np.linalg.norm(acceleration, axis=1).max())
    max_yaw_rate = float(np.abs(yaw_rate).max())
    circles = _circles(obstacles)
    if len(circles):
        separation = np.linalg.norm(
            points[:, None, :] - circles[None, :, :2], axis=2)
        required = (
            circles[None, :, 2]
            + CANONICAL_FOOTPRINT.circumscribed_radius_m
            + CANONICAL_FOOTPRINT.planning_margin_m)
        min_clearance = float((separation - required).min())
    else:
        min_clearance = float("inf")
    violations = ConstraintViolations(
        obstacle_m=max(0.0, -min_clearance),
        corridor_m=0.0 if band_contained else float("inf"),
        speed_mps=max(0.0, max_speed - planner.v_max),
        acceleration_mps2=max(0.0, max_acceleration - planner.a_max),
        yaw_rate_rps=max(0.0, max_yaw_rate - planner.yaw_rate_max))
    return TrajectoryCertificate(
        _reason(band_contained, violations, planner.CONSTRAINT_TOLERANCES),
        violations, planner.CONSTRAINT_TOLERANCES, band_contained,
        min_clearance, max_speed,
        max_acceleration, max_yaw_rate)


def certify_coefficients(
        planner: PlannerLimits,
        *,
        coefficients: np.ndarray,
        degree: int,
        horizon_s: float,
        control_hz: float,
        band: BandContainment,
        obstacles: np.ndarray,
        band_grace_m: float = 0.0) -> CertifiedTrajectory:
    """Evaluate a Bernstein candidate densely at its execution frequency."""
    control_hz, band_grace_m = validated_certificate_settings(
        control_hz, band_grace_m)
    xi = np.asarray(coefficients, dtype=np.float64)
    if (degree < 1 or xi.shape != (2 * (degree + 1),)
            or not np.isfinite(xi).all() or not math.isfinite(horizon_s)
            or horizon_s <= 0.0):
        raise ValueError("finite coefficients and positive dense timing required")
    sample_count = max(2, int(math.ceil(horizon_s * control_hz)) + 1)
    times = np.linspace(0.0, horizon_s, sample_count)
    position_basis, velocity_basis, acceleration_basis = bernstein_basis(
        degree, times, horizon_s)
    count = degree + 1
    control_x, control_y = xi[:count], xi[count:]
    points = np.stack([
        position_basis @ control_x, position_basis @ control_y], axis=1)
    velocity = np.stack([
        velocity_basis @ control_x, velocity_basis @ control_y], axis=1)
    acceleration = np.stack([
        acceleration_basis @ control_x,
        acceleration_basis @ control_y], axis=1)
    yaw, yaw_rate = _yaw_evidence(velocity, acceleration, times)
    certificate = _certificate(
        planner, points, velocity, acceleration, yaw_rate, band, obstacles,
        band_grace_m)
    return CertifiedTrajectory(
        points, times, velocity, acceleration, yaw, yaw_rate, certificate)


def lowest_certified_index(
        augmented_cost: np.ndarray,
        certificates: Sequence[TrajectoryCertificate]) -> Optional[int]:
    costs = np.asarray(augmented_cost, dtype=np.float64)
    if costs.ndim != 1 or len(costs) != len(certificates):
        raise ValueError("one augmented cost is required per certificate")
    usable = [index for index, certificate in enumerate(certificates)
              if certificate.usable and math.isfinite(float(costs[index]))]
    if not usable:
        return None
    return min(usable, key=lambda index: float(costs[index]))


def certify_trajectory(
        planner: PlannerLimits,
        *,
        points: np.ndarray,
        times_s: np.ndarray,
        band: BandContainment,
        obstacles: np.ndarray,
        band_grace_m: float = 0.0) -> TrajectoryCertificate:
    """Reject a proposed centre path unless every executable bound holds."""
    xy = np.asarray(points, dtype=np.float64)
    times = np.asarray(times_s, dtype=np.float64)
    if (xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2
            or times.shape != (len(xy),)
            or not np.isfinite(xy).all() or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0.0)):
        raise ValueError("trajectory must contain finite Nx2 points at increasing times")
    velocity = np.stack([
        np.gradient(xy[:, 0], times), np.gradient(xy[:, 1], times)], axis=1)
    acceleration = np.stack([
        np.gradient(velocity[:, 0], times),
        np.gradient(velocity[:, 1], times)], axis=1)
    _, yaw_rate = _yaw_evidence(velocity, acceleration, times)
    return _certificate(
        planner, xy, velocity, acceleration, yaw_rate, band, obstacles,
        band_grace_m)
