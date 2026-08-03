"""Actual follower rejects unsafe oriented plans and command sweeps."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from test_priest_follower_execution import (
    DriveCommand,
    follower_with,
    plan_with,
)


class StripBand:
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        return np.abs(points[:, 1]) <= 0.25 + grace


class FixedStripBand:
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        del grace
        return np.abs(points[:, 1]) <= 0.35


class FakeClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def advance(self, duration_s: float) -> None:
        self.now_s += duration_s

    def age(self, stamp_s: float) -> float:
        return self.now_s - stamp_s


def test_plan_acceptance_checks_rotated_corners_not_only_centres() -> None:
    plan = plan_with()
    plan.y = np.array([0.22, 0.27, 0.32])
    plan.yaw_rad = np.full(3, math.pi / 4.0)
    follower = follower_with(plan)
    follower.band = StripBand()
    follower.centre_xy = np.array([0.0, 0.22])
    follower.pose_yaw = math.pi / 4.0

    assert follower.static_plan_reason(plan) == "OFF_BAND"


def test_published_turn_arc_is_footprint_checked_at_current_yaw() -> None:
    follower = follower_with(plan_with())
    follower.band = StripBand()
    follower.centre_xy = np.array([0.0, 0.22])
    follower.pose_yaw = math.pi / 2.0

    reason = follower.command_safety_reason(DriveCommand(0.30, -0.30))

    assert reason == "OFF_BAND"


def test_command_arc_covers_downstream_retention_window() -> None:
    follower = follower_with(plan_with())
    follower.band = FixedStripBand()

    reason = follower.command_safety_reason(DriveCommand(0.60, 0.20))

    assert reason == "OFF_BAND"


def test_follower_horizon_covers_normal_and_gate_crash_retention() -> None:
    from priest_constraints import (
        COMMAND_RETENTION_HORIZON_S,
        GATED_INPUT_STALE_S,
        RAW_INPUT_STALE_S,
        SAFETY_GATE_RATE_HZ,
        TIP_GUARD_RATE_HZ,
    )

    scripts = Path(__file__).parents[1] / "scripts"
    gate = (scripts / "safety_gate.py").read_text(encoding="utf-8")
    guard = (scripts / "tip_guard.py").read_text(encoding="utf-8")
    normal = FakeClock()
    normal.advance(RAW_INPUT_STALE_S)
    assert normal.age(0.0) <= RAW_INPUT_STALE_S
    normal.advance(1.0 / SAFETY_GATE_RATE_HZ + 1.0 / TIP_GUARD_RATE_HZ)
    crash = FakeClock()
    crash.advance(RAW_INPUT_STALE_S)
    last_gated_stamp = crash.now_s
    crash.advance(GATED_INPUT_STALE_S + 1.0 / TIP_GUARD_RATE_HZ)
    assert crash.age(last_gated_stamp) > GATED_INPUT_STALE_S
    conservative_bound = (
        RAW_INPUT_STALE_S + 1.0 / SAFETY_GATE_RATE_HZ
        + GATED_INPUT_STALE_S + 1.0 / TIP_GUARD_RATE_HZ)

    assert max(normal.now_s, crash.now_s, conservative_bound) \
        <= COMMAND_RETENTION_HORIZON_S
    assert "INPUT_STALE_S = RAW_INPUT_STALE_S" in gate
    assert "INPUT_STALE_S = GATED_INPUT_STALE_S" in guard
