"""Public ROS-free value types shared by the PRIEST planner and follower."""

from __future__ import annotations

import math
from typing import Optional, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from priest_feasibility import TrajectoryCertificate


class Corridor(object):
    """A band centreline with lateral limits indexed by arc length."""

    def __init__(
            self,
            centres: np.ndarray,
            normals: np.ndarray,
            left_m: np.ndarray,
            right_m: np.ndarray) -> None:
        self.centres = np.asarray(centres, dtype=np.float64)
        self.normals = np.asarray(normals, dtype=np.float64)
        self.left_m = np.asarray(left_m, dtype=np.float64)
        self.right_m = np.asarray(right_m, dtype=np.float64)
        steps = np.linalg.norm(np.diff(self.centres, axis=0), axis=1)
        self.arc = np.concatenate([[0.0], np.cumsum(steps)])

    @property
    def length_m(self) -> float:
        return float(self.arc[-1])

    def arc_of(self, point: np.ndarray) -> float:
        """Continuous arc length of the nearest centreline segment."""
        query = np.asarray(point, dtype=np.float64)
        if query.shape != (2,) or not np.isfinite(query).all():
            raise ValueError("corridor query must be one finite planar point")
        if len(self.centres) < 2:
            return 0.0
        start = self.centres[:-1]
        delta = np.diff(self.centres, axis=0)
        length_sq = np.einsum("ij,ij->i", delta, delta)
        valid = length_sq > 1e-12
        if not np.any(valid):
            return 0.0
        fraction = np.einsum("ij,ij->i", query - start, delta) \
            / np.maximum(length_sq, 1e-12)
        fraction = np.clip(fraction, 0.0, 1.0)
        closest = start + fraction[:, None] * delta
        distance = np.linalg.norm(closest - query, axis=1)
        distance[~valid] = float("inf")
        index = int(np.argmin(distance))
        return float(self.arc[index] + fraction[index] * math.sqrt(
            length_sq[index]))

    def slice(
            self,
            start_arc: float,
            end_arc: float,
            steps: int,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Resample the corridor over ``[start_arc, end_arc]``."""
        end_arc = max(end_arc, start_arc + 1e-3)
        wanted = np.linspace(start_arc, min(end_arc, self.arc[-1]), steps)
        centres = np.stack([
            np.interp(wanted, self.arc, self.centres[:, 0]),
            np.interp(wanted, self.arc, self.centres[:, 1]),
        ], axis=1)
        normals = np.stack([
            np.interp(wanted, self.arc, self.normals[:, 0]),
            np.interp(wanted, self.arc, self.normals[:, 1]),
        ], axis=1)
        norm = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / np.maximum(norm, 1e-9)
        return (
            centres,
            normals,
            np.interp(wanted, self.arc, self.left_m),
            np.interp(wanted, self.arc, self.right_m),
        )


class Plan(object):
    """One planning cycle's answer, with evidence to refuse to drive it."""

    def __init__(
            self,
            xi: np.ndarray | None,
            x: np.ndarray | None,
            y: np.ndarray | None,
            times: np.ndarray | None,
            residual: float,
            cost: float,
            feasible_samples: int,
            horizon_s: float,
            reason: str = "",
            certificate: Optional[TrajectoryCertificate] = None,
            velocity_xy_mps: Optional[np.ndarray] = None,
            acceleration_xy_mps2: Optional[np.ndarray] = None,
            yaw_rad: Optional[np.ndarray] = None,
            yaw_rate_rps: Optional[np.ndarray] = None) -> None:
        self.xi = xi
        self.x = x
        self.y = y
        self.times = times
        self.residual = float(residual)
        self.cost = float(cost)
        self.feasible_samples = int(feasible_samples)
        self.horizon_s = float(horizon_s)
        self.reason = reason
        self.certificate = certificate
        self.velocity_xy_mps = velocity_xy_mps
        self.acceleration_xy_mps2 = acceleration_xy_mps2
        self.yaw_rad = yaw_rad
        self.yaw_rate_rps = yaw_rate_rps

    @property
    def usable(self) -> bool:
        return (self.reason == "" and self.certificate is not None
                and self.certificate.usable)

    def points(self) -> np.ndarray:
        return np.stack([self.x, self.y], axis=1)
