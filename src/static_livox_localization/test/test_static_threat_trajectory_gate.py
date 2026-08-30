import math
import threading
import types

import numpy as np

from test_dwa_policy import load_follower


def _permit(module, track_id=7, stamp_s=100.0, target_x_m=1.0):
    return module.bypass_policy.BypassPermit(
        capable=True, active=True, stamp_s=stamp_s, expires_s=100.45,
        track_id=track_id, target_x_m=target_x_m, target_y_m=0.0,
        threat_label="person", static_for_s=2.0, max_speed_mps=0.35,
        min_clearance_m=0.50,
        reason=module.bypass_policy.STATIC_THREAT_BYPASS)


def _proposal(module, *, sequence=11, track_id=7, side="LEFT",
              stamp_s=100.0, applied_v=0.10, applied_w=0.15,
              target_v=0.35, target_w=0.50, distance_m=0.25):
    start_speed = max(0.0, applied_v - 0.008)
    start_yaw = 0.0 if applied_v <= 0.02 else applied_w - 0.15
    state = module.proposal_contract.ActuatorState(
        start_speed, start_yaw, 0.0, 0.1)
    rollout = module.proposal_contract.rollout_actuation_timed(
        module.proposal_contract.RolloutSpec(
            pose=(0.0, 0.0, 0.0), target_speed_mps=target_v,
            target_yaw_rate_rps=target_w, actuator_state=state,
            distance_m=distance_m, latency_s=0.0))
    poses, speeds, yaw_rates, time_steps = rollout
    return module.proposal_contract.TrajectoryProposal(
        proposal_seq=sequence, stamp_s=stamp_s,
        permit_track_id=track_id, committed_side=side,
        frame_id="current_body", horizon_s=sum(time_steps),
        distance_m=distance_m, latency_s=0.0, actuator_state=state,
        target_speed_mps=target_v, target_yaw_rate_rps=target_w,
        poses=poses, speeds_mps=speeds, yaw_rates_rps=yaw_rates,
        time_steps_s=time_steps)


def _gate(module, *, obstacles=None, raw_v=0.10, raw_w=0.15, raw_seq=11,
          target_x_m=1.0):
    gate = module.TrajectorySafetyGate.__new__(module.TrajectorySafetyGate)
    gate.maximum_permit_age_s = 0.45
    gate.maximum_proposal_age_s = 0.30
    gate.proposal_linear_tolerance_mps = 0.02
    gate.proposal_angular_tolerance_rps = 0.03
    gate.proposal_buffer_size = 8
    gate.proposal_lock = threading.Lock()
    gate.minimum_bypass_turn_rps = 0.08
    gate.immediate_front_margin_m = 0.0
    gate.immediate_side_margin_m = 0.0
    gate.immediate_point_count = 5
    gate.person_bypass_permit = _permit(
        module, target_x_m=target_x_m)
    gate.trajectory_proposals = [_proposal(module)]
    gate.highest_proposal_seq = 11
    gate.proposal_receive_reason = "PROPOSAL_RECEIVED"
    gate.raw = types.SimpleNamespace(
        linear=types.SimpleNamespace(x=raw_v),
        angular=types.SimpleNamespace(x=float(raw_seq), z=raw_w))
    gate.motion = types.SimpleNamespace(
        linear_speed_mps=0.0, angular_speed_rps=0.0)
    gate.evidence = {"horizon_s": 1.0}
    points = np.empty((0, 2), dtype=float) if obstacles is None else obstacles
    gate.collision_snapshot = module.base_gate.CollisionSnapshot(
        points_xy=points, source_point_count=len(points))
    gate.last_override = None
    return gate


def _base_reason(monkeypatch, module, reason):
    monkeypatch.setattr(
        module.base_gate.SafetyGate, "motion_blocked",
        lambda self, now: (reason, None))


def _selected_path_hit(proposal):
    x_m, y_m, yaw_rad = proposal.poses[-1]
    return (
        x_m + 0.48 * math.cos(yaw_rad),
        y_m + 0.48 * math.sin(yaw_rad),
    )


def test_exact_actuator_curve_can_replace_fixed_corridor_obstacle(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    obstacles = np.repeat([[1.0, 0.0]], 5, axis=0)
    gate = _gate(module, obstacles=obstacles)
    _base_reason(monkeypatch, module, "OBSTACLE")

    reason, cap = gate.motion_blocked(Stamp(100.1))

    assert (reason, cap) == ("", 0.35)
    assert gate.evidence["trajectory_override_reason"] == \
        "STATIC_THREAT_TRAJECTORY_CLEAR"
    assert gate.evidence["trajectory_proposal_seq"] == 11
    assert gate.evidence["static_threat_bypass_track_id"] == 7


def test_gate_matches_incoming_command_to_first_applied_not_target(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = _gate(module, raw_v=0.10, raw_w=0.15)
    gate.trajectory_proposals = [_proposal(
        module, applied_v=0.10, applied_w=0.15,
        target_v=0.35, target_w=0.50)]
    _base_reason(monkeypatch, module, "OBSTACLE")

    reason, _ = gate.motion_blocked(Stamp(100.1))

    assert reason == ""
    assert gate.evidence["trajectory_applied_v"] == 0.10
    assert gate.evidence["trajectory_applied_w"] == 0.15
    assert gate.evidence["trajectory_target_w"] == 0.50


def test_stopped_first_applied_command_uses_target_turn_intent(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = _gate(module, raw_v=0.008, raw_w=0.0)
    gate.trajectory_proposals = [_proposal(
        module, applied_v=0.008, applied_w=0.0,
        target_v=0.35, target_w=0.50)]
    _base_reason(monkeypatch, module, "OBSTACLE")

    reason, cap = gate.motion_blocked(Stamp(100.1))

    assert (reason, cap) == ("", 0.35)
    assert gate.evidence["trajectory_applied_w"] == 0.0
    assert gate.evidence["trajectory_target_w"] == 0.50


def test_command_track_side_stale_and_replayed_proposals_never_waive_stop(
        monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    _base_reason(monkeypatch, module, "OBSTACLE")
    cases = (
        (_proposal(module, track_id=8), "PROPOSAL_TRACK_MISMATCH"),
        (_proposal(module, side="NONE"), "PROPOSAL_SIDE_MISMATCH"),
        (_proposal(module, side="RIGHT", applied_w=0.15),
         "PROPOSAL_SIDE_MISMATCH"),
        (_proposal(module, stamp_s=99.0), "PROPOSAL_STALE"),
        (_proposal(module, sequence=10), "PROPOSAL_SEQUENCE_STALE"),
    )
    for proposal, expected in cases:
        gate = _gate(module)
        gate.trajectory_proposals = [proposal]
        gate.highest_proposal_seq = 11
        reason, cap = gate.motion_blocked(Stamp(100.1))
        assert (reason, cap) == ("OBSTACLE", None)
        assert gate.evidence["trajectory_override_reason"] == expected

    gate = _gate(module, raw_v=0.14)
    reason, cap = gate.motion_blocked(Stamp(100.1))
    assert (reason, cap) == ("OBSTACLE", None)
    assert gate.evidence["trajectory_override_reason"] == \
        "PROPOSAL_COMMAND_MISMATCH"

    gate = _gate(module, raw_seq=10)
    reason, cap = gate.motion_blocked(Stamp(100.1))
    assert (reason, cap) == ("OBSTACLE", None)
    assert gate.evidence["trajectory_override_reason"] == \
        "PROPOSAL_SEQUENCE_MISMATCH"


def test_malformed_proposal_is_rejected_without_replacing_last_valid_one():
    module, _ = load_follower("trajectory_safety_gate")
    gate = _gate(module)
    valid = tuple(gate.trajectory_proposals)
    message = types.SimpleNamespace(data='{"schema":"wrong"}')

    gate.on_trajectory_proposal(message)

    assert tuple(gate.trajectory_proposals) == valid
    assert gate.proposal_receive_reason == "PROPOSAL_MALFORMED"


def test_bounded_proposal_buffer_matches_command_independent_of_arrival_order(
        monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = _gate(module)
    gate.trajectory_proposals = []
    gate.highest_proposal_seq = -1
    gate.proposal_receive_reason = "NO_PROPOSAL"
    gate.raw.angular.x = 10.0
    previous = _proposal(
        module, sequence=10, applied_v=0.10, applied_w=0.15)
    newest = _proposal(
        module, sequence=11, applied_v=0.20, applied_w=0.25)
    _base_reason(monkeypatch, module, "OBSTACLE")

    assert gate.motion_blocked(Stamp(100.1)) == ("OBSTACLE", None)
    gate.on_trajectory_proposal(types.SimpleNamespace(data=previous.to_json()))
    assert gate.motion_blocked(Stamp(100.1)) == ("", 0.35)
    gate.on_trajectory_proposal(types.SimpleNamespace(data=newest.to_json()))
    assert gate.motion_blocked(Stamp(100.1)) == ("", 0.35)
    assert gate.evidence["trajectory_proposal_seq"] == 10

    gate.raw.linear.x = 0.20
    gate.raw.angular.z = 0.25
    gate.raw.angular.x = 11.0
    assert gate.motion_blocked(Stamp(100.1)) == ("", 0.35)
    assert gate.evidence["trajectory_proposal_seq"] == 11

    replay = _proposal(
        module, sequence=9, applied_v=0.20, applied_w=0.25)
    gate.on_trajectory_proposal(types.SimpleNamespace(data=replay.to_json()))
    gate.on_trajectory_proposal(types.SimpleNamespace(data="{}"))
    assert len(gate.trajectory_proposals) == 2
    assert gate.motion_blocked(Stamp(100.1)) == ("", 0.35)
    assert gate.evidence["trajectory_proposal_seq"] == 11

    for sequence in range(12, 21):
        buffered = _proposal(
            module, sequence=sequence, applied_v=0.20, applied_w=0.25)
        gate.on_trajectory_proposal(
            types.SimpleNamespace(data=buffered.to_json()))
    assert len(gate.trajectory_proposals) == 8
    assert gate.trajectory_proposals[0].proposal_seq == 13
    assert gate.motion_blocked(Stamp(100.1)) == ("OBSTACLE", None)
    assert gate.evidence["trajectory_override_reason"] == \
        "PROPOSAL_SEQUENCE_MISMATCH"
    gate.raw.angular.x = 20.0
    assert gate.motion_blocked(Stamp(100.1)) == ("", 0.35)
    assert gate.evidence["trajectory_proposal_seq"] == 20


def test_exact_proposal_collision_current_footprint_and_carried_path_stop(
        monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    _base_reason(monkeypatch, module, "OBSTACLE")

    gate = _gate(module)
    proposal_hit = np.repeat(
        [_selected_path_hit(gate.trajectory_proposals[0])], 5, axis=0)
    gate.collision_snapshot = module.base_gate.CollisionSnapshot(
        points_xy=proposal_hit, source_point_count=len(proposal_hit))
    reason, _ = gate.motion_blocked(Stamp(100.1))
    assert reason == "OBSTACLE"
    assert gate.evidence["trajectory_override_reason"] == \
        "PROPOSAL_PATH_COLLISION"

    immediate_hit = np.repeat([[0.20, 0.0]], 5, axis=0)
    gate = _gate(module, obstacles=immediate_hit)
    reason, _ = gate.motion_blocked(Stamp(100.1))
    assert reason == "OBSTACLE"
    assert gate.evidence["trajectory_override_reason"] == \
        "IMMEDIATE_FOOTPRINT"

    gate = _gate(module, obstacles=np.repeat([[0.80, 0.0]], 5, axis=0))
    gate.motion = types.SimpleNamespace(
        linear_speed_mps=0.35, angular_speed_rps=0.0)
    reason, _ = gate.motion_blocked(Stamp(100.1))
    assert reason == "OBSTACLE"
    assert gate.evidence["trajectory_override_reason"] == \
        "CARRIED_PATH_COLLISION"


def test_collision_between_proposal_samples_cannot_pass(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    obstacles = np.repeat([[1.0, 0.75]], 5, axis=0)
    gate = _gate(module, obstacles=obstacles)
    gate.trajectory_proposals = [_proposal(module, distance_m=2.0)]
    _base_reason(monkeypatch, module, "OBSTACLE")

    reason, _ = gate.motion_blocked(Stamp(100.1))

    assert reason == "OBSTACLE"
    assert gate.evidence["trajectory_override_reason"] == \
        "PROPOSAL_PATH_COLLISION"


def test_absolute_base_reasons_are_never_replaced(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    for base_reason in (
            "NO_CLOUD", "ODOM_STALE", "INPUT_STALE", "CLOUD_STALE",
            "INPUT_INVALID", "REVERSE"):
        gate = _gate(module)
        _base_reason(monkeypatch, module, base_reason)
        assert gate.motion_blocked(Stamp(100.1)) == (base_reason, None)
        assert gate.evidence["static_threat_target_behind"] is False
        assert gate.evidence["static_threat_tail_clear"] is False


def test_clear_base_cycle_reports_three_exact_tail_clear_frames(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = _gate(module, target_x_m=-0.60)
    gate.cloud = np.zeros((100, 3), dtype=float)
    gate.cloud_stamp = Stamp(99.9)
    gate.motion = types.SimpleNamespace(
        valid=True, linear_speed_mps=0.0, angular_speed_rps=0.0,
        source_stamp_s=100.0, receipt_stamp_s=100.0, reason="OK")
    filter_calls = []
    monkeypatch.setattr(
        module.base_gate, "motion_hold_reason", lambda *_args: "")
    monkeypatch.setattr(
        module.base_gate, "stopping_envelope",
        lambda **_kwargs: types.SimpleNamespace(
            distance_m=1.0, horizon_s=1.0))

    def filtered(*_args, **_kwargs):
        filter_calls.append(True)
        return np.empty((0, 2), dtype=float)

    monkeypatch.setattr(module.base_gate, "filter_obstacle_points", filtered)

    observed = []
    for _ in range(3):
        observed.append(gate.motion_blocked(Stamp(100.1)))
        assert gate.evidence["static_threat_target_behind"] is True
        assert gate.evidence["static_threat_tail_clear"] is True
        assert gate.evidence["trajectory_proposal_seq"] == 11
        assert gate.evidence["filter_calls"] == 1
        assert gate.evidence["snapshot_builds"] == 1

    assert observed == [("", None)] * 3
    assert len(filter_calls) == 3


def test_tail_clear_status_requires_target_behind_and_real_selected_path_clear(
        monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    gate = _gate(module, target_x_m=-0.60)
    _base_reason(monkeypatch, module, "OBSTACLE")

    reason, _ = gate.motion_blocked(Stamp(100.1))

    assert reason == ""
    assert gate.evidence["static_threat_target_behind"] is True
    assert gate.evidence["static_threat_tail_clear"] is True

    rear_hit = np.repeat([[-0.524, 0.22]], 5, axis=0)
    gate = _gate(module, obstacles=rear_hit, target_x_m=-0.60)
    gate.trajectory_proposals = [_proposal(module, target_w=0.50)]
    reason, _ = gate.motion_blocked(Stamp(100.1))
    assert reason == "OBSTACLE"
    assert gate.evidence["static_threat_tail_clear"] is False

    gate = _gate(module, target_x_m=-0.60)
    forward_hit = np.repeat(
        [_selected_path_hit(gate.trajectory_proposals[0])], 5, axis=0)
    gate.collision_snapshot = module.base_gate.CollisionSnapshot(
        points_xy=forward_hit, source_point_count=len(forward_hit))
    reason, _ = gate.motion_blocked(Stamp(100.1))
    assert reason == "OBSTACLE"
    assert gate.evidence["static_threat_target_behind"] is True
    assert gate.evidence["static_threat_tail_clear"] is False


def test_exact_override_reuses_the_single_base_collision_snapshot(monkeypatch):
    module, Stamp = load_follower("trajectory_safety_gate")
    obstacles = np.repeat([[0.80, -0.40]], 5, axis=0)
    gate = _gate(module)
    gate.cloud = np.zeros((100, 3), dtype=float)
    gate.cloud_stamp = Stamp(99.9)
    gate.motion = types.SimpleNamespace(
        valid=True, linear_speed_mps=0.0, angular_speed_rps=0.0,
        source_stamp_s=100.0, receipt_stamp_s=100.0, reason="OK")
    filter_calls = []
    monkeypatch.setattr(
        module.base_gate, "motion_hold_reason", lambda *_args: "")
    monkeypatch.setattr(
        module.base_gate, "stopping_envelope",
        lambda **_kwargs: types.SimpleNamespace(
            distance_m=1.0, horizon_s=1.0))

    def filtered(*_args, **_kwargs):
        filter_calls.append(True)
        return obstacles

    monkeypatch.setattr(module.base_gate, "filter_obstacle_points", filtered)

    reason, cap = gate.motion_blocked(Stamp(100.1))

    assert (reason, cap) == ("", 0.35)
    assert len(filter_calls) == 1
    assert gate.evidence["filter_calls"] == 1
    assert gate.evidence["snapshot_builds"] == 1
    assert gate.collision_snapshot.points_xy is obstacles
