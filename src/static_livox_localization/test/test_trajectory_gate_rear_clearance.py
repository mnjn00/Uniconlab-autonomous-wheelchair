import threading
import types

import numpy as np

from test_dwa_policy import load_follower


def gate_with_rear_points(
        module, yaw_rate_rps, linear_speed_mps=0.05, horizon_s=1.4,
        point=(-0.53, 0.24)):
    gate = module.TrajectorySafetyGate.__new__(module.TrajectorySafetyGate)
    gate.last_override = None
    gate.minimum_bypass_turn_rps = 0.08
    gate.raw = types.SimpleNamespace(
        linear=types.SimpleNamespace(x=linear_speed_mps),
        angular=types.SimpleNamespace(z=yaw_rate_rps))
    gate.evidence = {"horizon_s": horizon_s}
    points = np.repeat([point], 5, axis=0)
    gate.collision_snapshot = module.base_gate.CollisionSnapshot(
        points_xy=points, source_point_count=len(points))
    gate.fresh_active_permit = lambda _now_s: None
    return gate


def full_cycle_gate(module, Stamp, monkeypatch, obstacles, reason_speed,
                    yaw_rate):
    gate = module.TrajectorySafetyGate.__new__(module.TrajectorySafetyGate)
    gate.last_override = None
    gate.minimum_bypass_turn_rps = 0.08
    gate.cloud = np.zeros((100, 3), dtype=float)
    gate.cloud_stamp = Stamp(99.9)
    gate.raw = types.SimpleNamespace(
        linear=types.SimpleNamespace(x=reason_speed),
        angular=types.SimpleNamespace(z=yaw_rate))
    gate.motion = types.SimpleNamespace(
        linear_speed_mps=0.0, angular_speed_rps=yaw_rate)
    gate.fresh_active_permit = lambda _now_s: None
    filter_calls = []
    monkeypatch.setattr(
        module.base_gate, "motion_hold_reason", lambda *_args: "")
    monkeypatch.setattr(
        module.base_gate, "stopping_envelope",
        lambda **_kwargs: types.SimpleNamespace(
            distance_m=1.0, horizon_s=1.4))

    def filtered(*_args, **_kwargs):
        filter_calls.append(True)
        return obstacles

    monkeypatch.setattr(module.base_gate, "filter_obstacle_points", filtered)
    return gate, filter_calls


def test_forward_motion_never_waives_rear_points_without_exact_proposal(
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
    assert reason == "OBSTACLE_SWEEP"
    assert cap is None


def test_forward_turn_never_waives_rear_points_without_exact_proposal(
        monkeypatch):
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


def test_forward_turn_keeps_rear_points_inside_the_physical_tail_sweep(
        monkeypatch):
    # Given
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = gate_with_rear_points(
        module,
        yaw_rate_rps=0.350361555,
        linear_speed_mps=0.071167805,
        horizon_s=0.565694436,
        point=(-0.500903179, 0.246763662))
    monkeypatch.setattr(
        module.base_gate.SafetyGate, "motion_blocked",
        lambda _self, _now: ("OBSTACLE_SWEEP", None))

    # When
    reason, cap = gate.motion_blocked(Stamp(100.0))

    # Then
    assert reason == "OBSTACLE_SWEEP"
    assert cap is None


def test_missing_base_collision_snapshot_never_waives_a_stop(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = gate_with_rear_points(module, yaw_rate_rps=-0.05)
    gate.collision_snapshot = None
    monkeypatch.setattr(
        module.base_gate.SafetyGate, "motion_blocked",
        lambda _self, _now: ("OBSTACLE_SWEEP", None))

    reason, cap = gate.motion_blocked(Stamp(100.0))

    assert (reason, cap) == ("OBSTACLE_SWEEP", None)
    assert gate.evidence["trajectory_override_reason"] == "SNAPSHOT_MISSING"


def test_obstacle_full_cycle_builds_and_filters_one_snapshot(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    obstacles = np.repeat([[0.80, 0.40]], 5, axis=0)
    gate, filter_calls = full_cycle_gate(
        module, Stamp, monkeypatch, obstacles, reason_speed=0.35,
        yaw_rate=0.20)

    reason, cap = gate.motion_blocked(Stamp(100.0))

    assert (reason, cap) == ("OBSTACLE", None)
    assert len(filter_calls) == 1
    assert gate.evidence["filter_calls"] == 1
    assert gate.evidence["snapshot_builds"] == 1
    assert gate.collision_snapshot.points_xy is obstacles


def test_obstacle_sweep_rear_tail_full_cycle_reuses_one_snapshot(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    obstacles = np.repeat([[-0.53, 0.24]], 5, axis=0)
    gate, filter_calls = full_cycle_gate(
        module, Stamp, monkeypatch, obstacles, reason_speed=0.05,
        yaw_rate=-0.05)

    reason, cap = gate.motion_blocked(Stamp(100.0))

    assert (reason, cap) == ("OBSTACLE_SWEEP", None)
    assert len(filter_calls) == 1
    assert gate.evidence["filter_calls"] == 1
    assert gate.evidence["snapshot_builds"] == 1
    assert gate.evidence["pose_checks"] > 0
    assert gate.collision_snapshot.points_xy is obstacles


def test_rear_margin_stop_is_replaced_only_by_matched_exact_proposal(
        monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = gate_with_rear_points(
        module, yaw_rate_rps=0.15, linear_speed_mps=0.10,
        point=(-0.53, 0.42))
    gate.maximum_permit_age_s = 0.45
    gate.maximum_proposal_age_s = 0.30
    gate.proposal_linear_tolerance_mps = 0.02
    gate.proposal_angular_tolerance_rps = 0.03
    gate.proposal_buffer_size = 8
    gate.proposal_lock = threading.Lock()
    gate.immediate_front_margin_m = 0.0
    gate.immediate_side_margin_m = 0.0
    gate.immediate_point_count = 5
    gate.person_bypass_permit = module.bypass_policy.BypassPermit(
        capable=True, active=True, stamp_s=100.0, expires_s=100.45,
        track_id=7, target_x_m=-0.60, target_y_m=0.42,
        threat_label="person", static_for_s=2.0, max_speed_mps=0.35,
        min_clearance_m=0.50,
        reason=module.bypass_policy.STATIC_THREAT_BYPASS)
    state = module.proposal_contract.ActuatorState(0.08, 0.0, 0.0, 0.1)
    rollout = module.proposal_contract.rollout_actuation_timed(
        module.proposal_contract.RolloutSpec(
            pose=(0.0, 0.0, 0.0), target_speed_mps=0.35,
            target_yaw_rate_rps=0.50, actuator_state=state,
            distance_m=0.25, latency_s=0.0))
    poses, speeds, yaw_rates, time_steps = rollout
    gate.trajectory_proposals = [module.proposal_contract.TrajectoryProposal(
        proposal_seq=12, stamp_s=100.0, permit_track_id=7,
        committed_side="LEFT", frame_id="current_body",
        horizon_s=sum(time_steps), distance_m=0.25, latency_s=0.0,
        actuator_state=state, target_speed_mps=0.35,
        target_yaw_rate_rps=0.50,
        poses=poses, speeds_mps=speeds, yaw_rates_rps=yaw_rates,
        time_steps_s=time_steps)]
    gate.highest_proposal_seq = 12
    gate.proposal_receive_reason = "PROPOSAL_RECEIVED"
    gate.motion = types.SimpleNamespace(
        linear_speed_mps=0.0, angular_speed_rps=0.0)
    gate.fresh_active_permit = lambda _now_s: gate.person_bypass_permit
    monkeypatch.setattr(
        module.base_gate.SafetyGate, "motion_blocked",
        lambda _self, _now: ("OBSTACLE_SWEEP", None))

    reason, cap = gate.motion_blocked(Stamp(100.1))

    assert (reason, cap) == ("", 0.35)
    assert gate.evidence["trajectory_proposal_seq"] == 12
    assert gate.evidence["static_threat_tail_clear"] is True
