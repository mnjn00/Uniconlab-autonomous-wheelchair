"""Canonical physical dimensions and independently-unitized constraints."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
from typing import Final

import numpy as np


@dataclass(frozen=True)
class Footprint:
    """Axis-aligned chair footprint about the commanded body origin."""

    __slots__ = (
        "front_m", "rear_m", "half_width_m", "planning_margin_m",
        "sweep_margin_m")

    front_m: float
    rear_m: float
    half_width_m: float
    planning_margin_m: float
    sweep_margin_m: float

    @property
    def circumscribed_radius_m(self) -> float:
        return hypot(max(self.front_m, self.rear_m), self.half_width_m)


@dataclass(frozen=True)
class ConstraintTolerances:
    """Numerical slack per physical unit; values are never summed."""

    __slots__ = (
        "obstacle_m", "corridor_m", "speed_mps", "acceleration_mps2",
        "yaw_rate_rps")

    obstacle_m: float
    corridor_m: float
    speed_mps: float
    acceleration_mps2: float
    yaw_rate_rps: float


@dataclass(frozen=True)
class ConstraintViolations:
    """Non-negative maxima in their native physical units."""

    __slots__ = (
        "obstacle_m", "corridor_m", "speed_mps", "acceleration_mps2",
        "yaw_rate_rps")

    obstacle_m: float
    corridor_m: float
    speed_mps: float
    acceleration_mps2: float
    yaw_rate_rps: float

    def is_within(self, tolerances: ConstraintTolerances) -> bool:
        return (
            self.obstacle_m <= tolerances.obstacle_m
            and self.corridor_m <= tolerances.corridor_m
            and self.speed_mps <= tolerances.speed_mps
            and self.acceleration_mps2 <= tolerances.acceleration_mps2
            and self.yaw_rate_rps <= tolerances.yaw_rate_rps
        )


@dataclass(frozen=True)
class ProjectionViolations:
    """Per-trajectory maximum violations in their native physical units."""

    __slots__ = (
        "obstacle_m", "corridor_m", "speed_mps", "acceleration_mps2",
        "yaw_rate_rps")

    obstacle_m: np.ndarray
    corridor_m: np.ndarray
    speed_mps: np.ndarray
    acceleration_mps2: np.ndarray
    yaw_rate_rps: np.ndarray

    def is_within(self, tolerances: ConstraintTolerances) -> np.ndarray:
        return (
            (self.obstacle_m <= tolerances.obstacle_m)
            & (self.corridor_m <= tolerances.corridor_m)
            & (self.speed_mps <= tolerances.speed_mps)
            & (self.acceleration_mps2 <= tolerances.acceleration_mps2)
            & (self.yaw_rate_rps <= tolerances.yaw_rate_rps)
        )

    def score(self, tolerances: ConstraintTolerances) -> np.ndarray:
        """Dimensionless worst normalized violation for Algorithm 1 ranking."""
        fields = (
            _normalized(self.obstacle_m, tolerances.obstacle_m),
            _normalized(self.corridor_m, tolerances.corridor_m),
            _normalized(self.speed_mps, tolerances.speed_mps),
            _normalized(self.acceleration_mps2, tolerances.acceleration_mps2),
            _normalized(self.yaw_rate_rps, tolerances.yaw_rate_rps),
        )
        return np.maximum.reduce(fields)


def _normalized(values: np.ndarray, tolerance: float) -> np.ndarray:
    if tolerance > 0.0:
        return values / tolerance
    return np.where(values <= 0.0, 0.0, float("inf"))


def max_yaw_rate_rps(
        vx: np.ndarray,
        vy: np.ndarray,
        times_s: np.ndarray,
        moving_epsilon_mps: float = 1e-4,
        initial_yaw_rad: np.ndarray | None = None) -> np.ndarray:
    """Maximum wrapped velocity-heading change per second for each sample."""
    velocity_x = np.asarray(vx, dtype=np.float64)
    velocity_y = np.asarray(vy, dtype=np.float64)
    times = np.asarray(times_s, dtype=np.float64)
    if velocity_x.shape != velocity_y.shape or velocity_x.ndim != 2:
        raise ValueError("velocity components must share shape (batch, steps)")
    if times.shape != (velocity_x.shape[1],) or np.any(np.diff(times) <= 0.0):
        raise ValueError("times must be increasing with one value per step")
    initial_yaw = None
    if initial_yaw_rad is not None:
        initial_yaw = np.asarray(initial_yaw_rad, dtype=np.float64)
        if initial_yaw.shape != (velocity_x.shape[0],):
            raise ValueError("initial yaw must have one value per sample")
        if not np.all(np.isfinite(initial_yaw)):
            raise ValueError("initial yaw must be finite")

    maxima = np.zeros(velocity_x.shape[0], dtype=np.float64)
    speed = np.hypot(velocity_x, velocity_y)
    for sample_index in range(velocity_x.shape[0]):
        moving = np.flatnonzero(speed[sample_index] > moving_epsilon_mps)
        if not len(moving):
            continue
        heading = np.arctan2(
            velocity_y[sample_index, moving], velocity_x[sample_index, moving])
        heading_times = times[moving]
        if initial_yaw is not None:
            if moving[0] == 0:
                mismatch = np.arctan2(
                    np.sin(heading[0] - initial_yaw[sample_index]),
                    np.cos(heading[0] - initial_yaw[sample_index]))
                if abs(mismatch) > 1e-9:
                    maxima[sample_index] = float("inf")
            else:
                heading = np.concatenate((
                    [initial_yaw[sample_index]], heading))
                heading_times = np.concatenate(([times[0]], heading_times))
        if len(heading) >= 2:
            unwrapped = np.unwrap(heading)
            rate = np.abs(np.diff(unwrapped)) / np.diff(heading_times)
            maxima[sample_index] = max(
                maxima[sample_index], float(rate.max()))
    return maxima


def projected_violations(
        *,
        x: np.ndarray,
        y: np.ndarray,
        vx: np.ndarray,
        vy: np.ndarray,
        ax: np.ndarray,
        ay: np.ndarray,
        obstacles: np.ndarray,
        corridor_excess_m: np.ndarray,
        times_s: np.ndarray,
        v_max: float,
        a_max: float,
        yaw_rate_max: float) -> ProjectionViolations:
    """Compute maximum positive excess for every projection constraint."""
    if len(obstacles):
        reach = np.hypot(
            x[:, None, :] - obstacles[None, :, 0, None],
            y[:, None, :] - obstacles[None, :, 1, None])
        obstacle = np.clip(
            obstacles[None, :, 2, None] - reach, 0.0, None).max(axis=(1, 2))
    else:
        obstacle = np.zeros(x.shape[0], dtype=np.float64)
    if corridor_excess_m.shape[1]:
        corridor = np.clip(corridor_excess_m, 0.0, None).max(axis=1)
    else:
        corridor = np.zeros(x.shape[0], dtype=np.float64)
    speed = np.clip(np.hypot(vx, vy) - v_max, 0.0, None).max(axis=1)
    acceleration = np.clip(
        np.hypot(ax, ay) - a_max, 0.0, None).max(axis=1)
    yaw_rate = np.clip(
        max_yaw_rate_rps(vx, vy, times_s) - yaw_rate_max, 0.0, None)
    return ProjectionViolations(
        obstacle_m=obstacle,
        corridor_m=corridor,
        speed_mps=speed,
        acceleration_mps2=acceleration,
        yaw_rate_rps=yaw_rate,
    )


CANONICAL_FOOTPRINT: Final = Footprint(
    front_m=0.50,
    rear_m=0.50,
    half_width_m=0.30,
    planning_margin_m=0.10,
    sweep_margin_m=0.15,
)

DEFAULT_CONSTRAINT_TOLERANCES: Final = ConstraintTolerances(
    obstacle_m=1e-6,
    corridor_m=0.0,
    speed_mps=1e-3,
    acceleration_mps2=1e-3,
    yaw_rate_rps=1e-3,
)

PROJECTION_CONSTRAINT_TOLERANCES: Final = ConstraintTolerances(
    obstacle_m=0.01,
    corridor_m=0.01,
    speed_mps=0.01,
    acceleration_mps2=0.02,
    yaw_rate_rps=0.02,
)

# Loaded-chair measurement: below this faster-wheel-equivalent floor, a
# commanded turn does not begin reliably. Planner and controller share it.
TURN_FLOOR_SPEED_MPS: Final = 0.30

# A raw twist can survive both relay watchdogs after a single downstream
# process failure. The execution horizon covers both TTLs plus one nominal
# scheduling period at each stage; 1.30 rounds that 1.2867 s bound upward.
RAW_INPUT_STALE_S: Final = 0.6
GATED_INPUT_STALE_S: Final = 0.6
SAFETY_GATE_RATE_HZ: Final = 15.0
TIP_GUARD_RATE_HZ: Final = 50.0
COMMAND_RETENTION_HORIZON_S: Final = 1.30
GOAL_COMPARISON_EPSILON_M: Final = 1e-9


def within_goal_tolerance(distance_m: float, tolerance_m: float) -> bool:
    """Compare goal distance with only floating-point-scale slack."""
    return (isfinite(distance_m) and isfinite(tolerance_m)
            and distance_m >= 0.0 and tolerance_m >= 0.0
            and distance_m <= tolerance_m + GOAL_COMPARISON_EPSILON_M)
