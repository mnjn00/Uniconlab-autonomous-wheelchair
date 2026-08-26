import sys
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from terrain_guard_policy import (  # noqa: E402
    evaluate_terrain_command,
    rollout_unicycle,
    stopping_horizon,
)


class Mask:
    def __init__(self, x_limit=10.0, clearance=1.0):
        self.x_limit = x_limit
        self.clearance = clearance

    def contains_many(self, points):
        points = np.asarray(points)
        return points[:, 0] <= self.x_limit

    def segment_is_contained(self, start, end):
        return max(start[0], end[0]) <= self.x_limit

    def clearance_many(self, points):
        return np.full(len(points), self.clearance)


class Band:
    def contains(self, point):
        return abs(point[1]) <= 2.0

    def chord_is_contained(self, start, end):
        return self.contains(start) and self.contains(end)


def test_straight_rollout_advances_in_heading_direction():
    poses = rollout_unicycle((1.0, 2.0, 0.0), 0.5, 0.0, 1.0)
    assert poses[-1, 0] == 1.5
    assert poses[-1, 1] == 2.0


def test_turning_rollout_is_finite():
    poses = rollout_unicycle((0.0, 0.0, 0.0), 0.5, 0.3, 2.0)
    assert np.isfinite(poses).all()
    assert poses[-1, 1] > 0.0


def test_boundary_crossing_is_blocked():
    decision = evaluate_terrain_command(
        Mask(x_limit=0.6), (0.0, 0.0, 0.0), 0.8, 0.0,
        0.1, 0.4, 0.35, Band(), horizon_s=1.0)
    assert decision.reason == "MASK_BOUNDARY"


def test_low_clearance_stops():
    decision = evaluate_terrain_command(
        Mask(clearance=0.05), (0.0, 0.0, 0.0), 0.2, 0.0,
        0.10, 0.40, 0.35, Band(), horizon_s=1.0)
    assert decision.reason == "MASK_CLEARANCE"


def test_edge_zone_caps_speed_without_stopping():
    decision = evaluate_terrain_command(
        Mask(clearance=0.25), (0.0, 0.0, 0.0), 0.8, 0.0,
        0.10, 0.40, 0.35, Band(), horizon_s=1.0)
    assert not decision.blocked
    assert decision.speed_cap_mps == 0.35


def test_roomy_mask_passes_unchanged():
    decision = evaluate_terrain_command(
        Mask(clearance=1.0), (0.0, 0.0, 0.0), 0.8, 0.0,
        0.10, 0.40, 0.35, Band(), horizon_s=1.0)
    assert not decision.blocked
    assert decision.speed_cap_mps is None


def test_reverse_is_rejected():
    decision = evaluate_terrain_command(
        Mask(), (0.0, 0.0, 0.0), -0.1, 0.0,
        0.10, 0.45, 0.35, Band(), horizon_s=1.0)
    assert decision.reason == "REVERSE"


def test_horizon_grows_with_speed():
    assert stopping_horizon(0.2) < stopping_horizon(0.8)
