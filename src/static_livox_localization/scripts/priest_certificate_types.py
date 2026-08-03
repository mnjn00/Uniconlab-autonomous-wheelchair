"""Immutable public evidence records returned by PRIEST certification."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np

from priest_constraints import (
    ConstraintTolerances,
    ConstraintViolations,
)


@dataclass(frozen=True)
class TrajectoryCertificate:
    __slots__ = (
        "reason", "violations", "tolerances", "runtime_band_contained",
        "min_obstacle_clearance_m", "max_speed_mps",
        "max_acceleration_mps2", "max_yaw_rate_rps",
        "max_turn_speed_deficit_mps")

    reason: str
    violations: ConstraintViolations
    tolerances: ConstraintTolerances
    runtime_band_contained: bool
    min_obstacle_clearance_m: float
    max_speed_mps: float
    max_acceleration_mps2: float
    max_yaw_rate_rps: float
    max_turn_speed_deficit_mps: float

    @property
    def usable(self) -> bool:
        return self.reason == ""

    @staticmethod
    def clear(tolerances: ConstraintTolerances) -> TrajectoryCertificate:
        return TrajectoryCertificate(
            "", ConstraintViolations(0.0, 0.0, 0.0, 0.0, 0.0),
            tolerances, True, float("inf"), 0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def refusal(
            reason: str,
            tolerances: ConstraintTolerances) -> TrajectoryCertificate:
        return TrajectoryCertificate(
            reason,
            ConstraintViolations(0.0, float("inf"), 0.0, 0.0, 0.0),
            tolerances, False, float("inf"), 0.0, 0.0, 0.0, 0.0)


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
