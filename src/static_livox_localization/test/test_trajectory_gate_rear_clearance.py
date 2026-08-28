import types

import numpy as np

from test_dwa_policy import load_follower


def gate_with_rear_points(module, yaw_rate_rps):
    gate = module.TrajectorySafetyGate.__new__(module.TrajectorySafetyGate)
    gate.last_override = None
    gate.minimum_bypass_turn_rps = 0.08
    gate.raw = types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.05),
        angular=types.SimpleNamespace(z=yaw_rate_rps))
    gate.evidence = {"horizon_s": 1.4}
    gate.collision_points = lambda: np.repeat(
        [[-0.53, 0.24]], 5, axis=0)
    gate.fresh_active_permit = lambda _now_s: None
    return gate


def test_forward_motion_ignores_points_wholly_behind_the_physical_rear(
        monkeypatch):
    # Given
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = gate_with_rear_points(module, yaw_rate_rps=-0.05)
    monkeypatch.setattr(
        module.base_gate.SafetyGate, "motion_blocked",
        lambda _self, _now: ("OBSTACLE_SWEEP", None))

    # When
    reason, cap = gate.motion_blocked(Stamp(100.0))

    # Then
    assert reason == ""
    assert cap is None


def test_turning_keeps_rear_points_in_the_raw_collision_sweep(monkeypatch):
    # Given
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = gate_with_rear_points(module, yaw_rate_rps=-0.25)
    monkeypatch.setattr(
        module.base_gate.SafetyGate, "motion_blocked",
        lambda _self, _now: ("OBSTACLE_SWEEP", None))

    # When
    reason, cap = gate.motion_blocked(Stamp(100.0))

    # Then
    assert reason == "OBSTACLE_SWEEP"
    assert cap is None
