"""Deterministic software-only rollout through the actual PRIEST controller."""

from __future__ import annotations

import sys
from pathlib import Path


TOOLS = Path(__file__).parents[3] / "tools"
sys.path.insert(0, str(TOOLS))
try:
    from validate_priest_navigation import run_validation
finally:
    sys.path.remove(str(TOOLS))


def test_actual_differential_drive_rollout_reaches_goal_and_holds_actor() -> None:
    report = run_validation(seed=0)

    assert report["qualification"] == "software_only"
    assert 0 < report["state_updates"] <= report["cycles"]
    assert report["moving_hold_count"] > 0
    assert report["max_band_violation_m"] <= 0.10
    assert report["minimum_footprint_clearance_m"] >= 0.0
    assert report["maximum_speed_mps"] <= 0.6 + 1e-9
    assert report["maximum_acceleration_mps2"] <= 0.18 + 1e-9
    assert report["maximum_emergency_command_rate_mps2"] > 0.0
    assert report["emergency_stop_command_policy"] \
        == "exact_zero_fail_closed"
    assert not report["emergency_stop_physical_deceleration_qualified"]
    assert 0.0 < report["terminal_stop_deceleration_mps2"] <= 0.6 + 1e-9
    assert report["maximum_yaw_rate_rps"] <= 0.5 + 1e-9
    assert report["command_safety_horizon_s"] == 1.3
    assert report["final_goal_error_m"] <= 0.05
    assert report["stopped_at_goal"]
    assert report["final_linear_mps"] == report["final_angular_rps"] == 0.0
