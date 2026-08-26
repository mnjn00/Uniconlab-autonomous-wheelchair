"""ROS-independent route-mask and sidewalk-boundary command checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class TerrainDecision:
    reason: str
    minimum_clearance_m: Optional[float]
    speed_cap_mps: Optional[float]
    horizon_s: float

    @property
    def blocked(self) -> bool:
        return bool(self.reason)


def rollout_unicycle(
        pose_xy_yaw: Sequence[float],
        linear_speed_mps: float,
        angular_speed_rps: float,
        horizon_s: float,
        step_s: float = 0.05) -> np.ndarray:
    values = tuple(float(value) for value in pose_xy_yaw) + (
        float(linear_speed_mps), float(angular_speed_rps),
        float(horizon_s), float(step_s))
    if len(pose_xy_yaw) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("rollout inputs must be finite")
    if horizon_s < 0.0 or step_s <= 0.0:
        raise ValueError("rollout timing is invalid")
    count = max(1, int(math.ceil(horizon_s / step_s)))
    times = np.linspace(0.0, horizon_s, count + 1)
    x0, y0, yaw0 = (float(value) for value in pose_xy_yaw)
    v = float(linear_speed_mps)
    w = float(angular_speed_rps)
    if abs(w) < 1e-9:
        x = x0 + v * np.cos(yaw0) * times
        y = y0 + v * np.sin(yaw0) * times
        yaw = np.full_like(times, yaw0)
    else:
        yaw = yaw0 + w * times
        radius = v / w
        x = x0 + radius * (np.sin(yaw) - math.sin(yaw0))
        y = y0 - radius * (np.cos(yaw) - math.cos(yaw0))
    return np.stack([x, y, yaw], axis=1)


def stopping_horizon(
        speed_mps: float,
        reaction_s: float = 0.25,
        minimum_deceleration_mps2: float = 0.5,
        reserve_s: float = 0.5,
        minimum_horizon_s: float = 1.0,
        maximum_horizon_s: float = 3.0) -> float:
    values = (speed_mps, reaction_s, minimum_deceleration_mps2,
              reserve_s, minimum_horizon_s, maximum_horizon_s)
    if not all(math.isfinite(float(value)) for value in values) or \
            minimum_deceleration_mps2 <= 0.0 or \
            min(reaction_s, reserve_s, minimum_horizon_s) < 0.0 or \
            maximum_horizon_s < minimum_horizon_s:
        raise ValueError("invalid stopping-horizon inputs")
    horizon = float(reaction_s) + abs(float(speed_mps)) / \
        float(minimum_deceleration_mps2) + float(reserve_s)
    return min(float(maximum_horizon_s),
               max(float(minimum_horizon_s), horizon))


def evaluate_terrain_command(
        route_mask: Any,
        pose_xy_yaw: Sequence[float],
        linear_speed_mps: float,
        angular_speed_rps: float,
        hard_clearance_m: float,
        slow_clearance_m: float,
        edge_speed_mps: float,
        safety_band: Any = None,
        horizon_s: Optional[float] = None,
        step_s: float = 0.05) -> TerrainDecision:
    limits = (linear_speed_mps, angular_speed_rps, hard_clearance_m,
              slow_clearance_m, edge_speed_mps, step_s)
    if not all(math.isfinite(float(value)) for value in limits):
        return TerrainDecision("INPUT_INVALID", None, 0.0, 0.0)
    if linear_speed_mps < 0.0:
        return TerrainDecision("REVERSE", None, 0.0, 0.0)
    if min(hard_clearance_m, slow_clearance_m, edge_speed_mps) < 0.0 \
            or slow_clearance_m < hard_clearance_m or step_s <= 0.0:
        return TerrainDecision("CONFIG_INVALID", None, 0.0, 0.0)
    if horizon_s is None:
        try:
            horizon_s = stopping_horizon(linear_speed_mps)
        except ValueError:
            return TerrainDecision("CONFIG_INVALID", None, 0.0, 0.0)
    try:
        poses = rollout_unicycle(
            pose_xy_yaw, linear_speed_mps, angular_speed_rps,
            float(horizon_s), step_s)
    except (TypeError, ValueError):
        return TerrainDecision("POSE_INVALID", None, 0.0, float(horizon_s))

    points = poses[:, :2]
    try:
        contained = np.asarray(route_mask.contains_many(points), dtype=bool)
    except Exception:
        return TerrainDecision("MASK_UNAVAILABLE", None, 0.0, float(horizon_s))
    if len(contained) != len(points) or not contained.all():
        return TerrainDecision("MASK_BOUNDARY", 0.0, 0.0, float(horizon_s))
    try:
        for start, end in zip(points[:-1], points[1:]):
            if not route_mask.segment_is_contained(start, end):
                return TerrainDecision(
                    "MASK_BOUNDARY", 0.0, 0.0, float(horizon_s))
    except Exception:
        return TerrainDecision("MASK_UNAVAILABLE", None, 0.0, float(horizon_s))

    minimum_clearance = None
    if hasattr(route_mask, "clearance_many"):
        try:
            clearances = np.asarray(route_mask.clearance_many(points), dtype=float)
            if len(clearances) == len(points) and np.isfinite(clearances).all():
                minimum_clearance = float(clearances.min())
        except Exception:
            minimum_clearance = None
    if minimum_clearance is None:
        return TerrainDecision(
            "MASK_CLEARANCE_UNAVAILABLE", None, 0.0, float(horizon_s))
    if minimum_clearance < float(hard_clearance_m):
        return TerrainDecision(
            "MASK_CLEARANCE", minimum_clearance, 0.0, float(horizon_s))

    if safety_band is not None:
        try:
            if not all(safety_band.contains(point) for point in points):
                return TerrainDecision(
                    "BAND_BOUNDARY", minimum_clearance, 0.0,
                    float(horizon_s))
            for start, end in zip(points[:-1], points[1:]):
                if not safety_band.chord_is_contained(start, end):
                    return TerrainDecision(
                        "BAND_BOUNDARY", minimum_clearance, 0.0,
                        float(horizon_s))
        except Exception:
            return TerrainDecision(
                "BAND_UNAVAILABLE", minimum_clearance, 0.0,
                float(horizon_s))

    cap = None
    if minimum_clearance < float(slow_clearance_m):
        cap = min(max(0.0, float(linear_speed_mps)),
                  float(edge_speed_mps))
    return TerrainDecision("", minimum_clearance, cap, float(horizon_s))
