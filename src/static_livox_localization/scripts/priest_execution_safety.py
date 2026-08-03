"""Oriented-footprint and short-horizon differential-drive safety."""

from __future__ import annotations

import math
from typing import Protocol

import numpy as np

from priest_constraints import CANONICAL_FOOTPRINT
from safety_band import CHAIR_HALF_WIDTH


class BandContainment(Protocol):
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        ...


def _boolean_result(value: np.ndarray, count: int) -> np.ndarray:
    result = np.asarray(value)
    if result.shape != (count,) or result.dtype.kind != "b":
        raise ValueError("runtime band must return one boolean per point")
    return result


def oriented_footprint_contained(
        band: BandContainment,
        centres_xy: np.ndarray,
        yaw_rad: np.ndarray,
        grace_m: float = 0.0) -> np.ndarray:
    """Check body centres and all four physical corners against a centre band.

    SafetyBand limits are already inset by its 0.35 m chair proxy. Adding that
    proxy back only for corner queries cancels the old axis-aligned assumption;
    the actual rotated 1.00 m x 0.60 m footprint is then tested exactly.
    """
    centres = np.asarray(centres_xy, dtype=np.float64)
    yaw = np.asarray(yaw_rad, dtype=np.float64)
    grace = float(grace_m)
    if (centres.ndim != 2 or centres.shape[1:] != (2,)
            or yaw.shape != (len(centres),) or not len(centres)
            or not np.isfinite(centres).all() or not np.isfinite(yaw).all()
            or not math.isfinite(grace) or grace < 0.0):
        raise ValueError("finite oriented poses and non-negative grace required")
    footprint = CANONICAL_FOOTPRINT
    body_corners = np.array([
        [footprint.front_m, footprint.half_width_m],
        [footprint.front_m, -footprint.half_width_m],
        [-footprint.rear_m, footprint.half_width_m],
        [-footprint.rear_m, -footprint.half_width_m],
    ])
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotation = np.stack([
        np.stack([cosine, -sine], axis=1),
        np.stack([sine, cosine], axis=1),
    ], axis=1)
    corners = centres[:, None, :] + np.einsum(
        "nij,kj->nki", rotation, body_corners)
    corner_result = _boolean_result(
        band.contains_many(
            corners.reshape(-1, 2), grace=CHAIR_HALF_WIDTH + grace),
        4 * len(centres)).reshape(len(centres), 4).all(axis=1)
    centre_result = _boolean_result(
        band.contains_many(centres, grace=grace), len(centres))
    return centre_result & corner_result


def differential_drive_arc(
        x_m: float,
        y_m: float,
        yaw_rad: float,
        linear_mps: float,
        angular_rps: float,
        duration_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Sample the exact constant-twist arc over one control period."""
    values = (x_m, y_m, yaw_rad, linear_mps, angular_rps, duration_s)
    if (not all(math.isfinite(value) for value in values)
            or linear_mps < 0.0 or duration_s <= 0.0):
        raise ValueError("finite forward differential-drive command required")
    distance = linear_mps * duration_s
    turn = abs(angular_rps) * duration_s
    count = max(2, int(math.ceil(max(distance / 0.02, turn / 0.05))) + 1)
    elapsed = np.linspace(0.0, duration_s, count)
    yaw = yaw_rad + angular_rps * elapsed
    if abs(angular_rps) <= 1e-9:
        points = np.stack([
            x_m + linear_mps * elapsed * math.cos(yaw_rad),
            y_m + linear_mps * elapsed * math.sin(yaw_rad)], axis=1)
    else:
        radius = linear_mps / angular_rps
        points = np.stack([
            x_m + radius * (np.sin(yaw) - math.sin(yaw_rad)),
            y_m - radius * (np.cos(yaw) - math.cos(yaw_rad))], axis=1)
    return points, yaw
