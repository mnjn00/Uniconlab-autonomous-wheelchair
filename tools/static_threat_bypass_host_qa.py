#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
# ─── How to run ───
# python3 tools/static_threat_bypass_host_qa.py
from __future__ import annotations

import json
import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "src" / \
    "static_livox_localization" / "scripts"
sys.path.insert(0, str(SCRIPTS))

import person_bypass_policy as policy  # noqa: E402
import trajectory_proposal as proposal_api  # noqa: E402
import motion_safety  # noqa: E402
import terrain_guard_policy  # noqa: E402


FAILED = False


class BoundaryMask:
    def contains_many(self, points):
        return [False] * len(points)

    def segment_is_contained(self, start, end) -> bool:
        return False


def emit(case: str, passed: bool, **evidence: bool | float | int | str) -> None:
    global FAILED
    FAILED = FAILED or not passed
    record = {"case": case, "passed": passed, **evidence}
    print(json.dumps(record, allow_nan=False, sort_keys=True))


def observation(stamp_s: float, label: str = "person", *, track_id: int = 7,
                x_m: float = 1.5, motion: str = "static",
                source: str = "geometric") -> policy.StaticThreatObservation:
    return policy.StaticThreatObservation(
        track_id=track_id, stamp_s=stamp_s, x_m=x_m, y_m=0.0,
        size_x_m=0.5, size_y_m=0.5, label=label, motion=motion,
        source=source,
    )


def qualify(label: str, *, report: bool) -> tuple[
        policy.StaticThreatBypassManager, policy.BypassPermit]:
    manager = policy.StaticThreatBypassManager()
    permit = manager.inactive(0.0, "NOT_STARTED")
    for tick in range(11):
        stamp_s = round(tick * 0.2, 1)
        permit = manager.update(
            (observation(stamp_s, label),), stamp_s, True,
            summary_stamp_s=stamp_s,
        )
        if report:
            phase = "commit" if stamp_s == 2.0 else "wait"
            expected_active = stamp_s == 2.0
            emit(
                f"{label}_{phase}_{stamp_s:.1f}".replace(".", "_"),
                permit.active is expected_active,
                active=permit.active, static_for_s=permit.static_for_s,
            )
    return manager, permit


def safe_left_proposal(permit: policy.BypassPermit) -> \
        proposal_api.TrajectoryProposal:
    actuator = proposal_api.ActuatorState(0.10, 0.10, 0.0, 0.1)
    spec = proposal_api.RolloutSpec(
        pose=(0.0, 0.0, 0.0), target_speed_mps=0.35,
        target_yaw_rate_rps=0.50, actuator_state=actuator, distance_m=0.30,
    )
    poses, speeds, yaw_rates = proposal_api.rollout_actuation(spec)
    proposal = proposal_api.TrajectoryProposal(
        proposal_seq=11, stamp_s=2.0, permit_track_id=permit.track_id,
        committed_side="LEFT", frame_id="body",
        horizon_s=len(poses) * actuator.control_step_s,
        actuator_state=actuator, target_speed_mps=spec.target_speed_mps,
        target_yaw_rate_rps=spec.target_yaw_rate_rps, poses=poses,
        speeds_mps=speeds, yaw_rates_rps=yaw_rates,
    )
    return proposal_api.TrajectoryProposal.from_json(proposal.to_json())


def gate(permit: policy.BypassPermit | None, now_s: float, *,
         immediate: bool = False, carried: bool = False,
         proposal_collision: bool = False) -> policy.GateOverrideDecision:
    return policy.evaluate_gate_override(
        permit=permit, now_s=now_s, requested_v_mps=0.35,
        requested_w_rps=0.25, immediate_collision=immediate,
        requested_path_collision=proposal_collision,
        carried_path_collision=carried,
    )


def adversarial_cases(active: policy.BypassPermit,
                      proposal: proposal_api.TrajectoryProposal) -> None:
    for case, threat in (
            ("moving", observation(0.0, motion="moving")),
            ("unknown", observation(0.0, motion="unknown")),
            ("learned_only", observation(0.0, source="learned_only"))):
        manager = policy.StaticThreatBypassManager()
        blocked = manager.update((threat,), 0.0, True, summary_stamp_s=0.0)
        emit(case, not blocked.active, command_v=0.0, reason=blocked.reason)

    changed_manager, _ = qualify("person", report=False)
    changed = changed_manager.update(
        (observation(2.2, track_id=8),), 2.2, True, summary_stamp_s=2.2)
    emit("changed_id", not changed.active, command_v=0.0, reason=changed.reason)

    dynamic_manager, _ = qualify("person", report=False)
    dynamic = dynamic_manager.update(
        (observation(2.2),), 2.2, True, dynamic_conflict=True,
        summary_stamp_s=2.2)
    emit("second_dynamic_threat", not dynamic.active, command_v=0.0,
         reason=dynamic.reason)

    no_permit = gate(None, 2.1)
    emit("raw_only_blockage", not no_permit.allowed, command_v=0.0,
         reason=no_permit.reason)
    legacy = policy.permit_from_payload('{"schema":"static-person-bypass/v1"}')
    emit("legacy_permit", legacy is None and not gate(legacy, 2.1).allowed,
         command_v=0.0)
    stale = gate(active, 3.0)
    emit("stale_permit", not stale.allowed, command_v=0.0, reason=stale.reason)

    parsed = proposal_api.TrajectoryProposal.from_json(proposal.to_json())
    stale_proposal = 2.4 - parsed.stamp_s > 0.30
    emit("stale_proposal", stale_proposal, command_v=0.0)
    mismatch = parsed.permit_track_id != active.track_id + 1
    emit("mismatched_proposal", mismatch, command_v=0.0)

    for case, decision in (
            ("immediate_collision", gate(active, 2.1, immediate=True)),
            ("carried_collision", gate(active, 2.1, carried=True)),
            ("proposal_collision", gate(active, 2.1, proposal_collision=True))):
        emit(case, not decision.allowed, command_v=0.0, reason=decision.reason)

    for case, kwargs in (
            ("localization_fault", {"localization_tracking": False}),
            ("perception_fault", {
                "localization_tracking": True, "summary_healthy": False}),
    ):
        manager = policy.StaticThreatBypassManager()
        decision = manager.update(
            (observation(0.0),), 0.0, summary_stamp_s=0.0, **kwargs)
        emit(case, not decision.active, command_v=0.0, reason=decision.reason)

    odom = motion_safety.MotionEstimate(
        False, 2.0, 2.0, 0.0, 0.0, "ODOM_STALE")
    odom_reason = motion_safety.motion_hold_reason(odom, 2.1, 0.35)
    emit("odom_fault", odom_reason == "ODOM_STALE", command_v=0.0,
         reason=odom_reason)
    terrain = terrain_guard_policy.evaluate_terrain_command(
        BoundaryMask(), (0.0, 0.0, 0.0), 0.35, 0.25,
        hard_clearance_m=0.12, slow_clearance_m=0.35,
        edge_speed_mps=0.35, horizon_s=1.0)
    emit("terrain_fault", terrain.blocked and terrain.speed_cap_mps == 0.0,
         command_v=0.0, reason=terrain.reason)


def main() -> int:
    person_manager, person_permit = qualify("person", report=True)
    qualify("object", report=True)
    side = person_manager.commit_pass_side("LEFT")
    proposal = safe_left_proposal(person_permit)
    safe = gate(person_permit, 2.1)
    emit("safe_left_proposal", safe.allowed and side == "left"
         and proposal.committed_side == "LEFT"
         and proposal.first_applied_yaw_rate_rps > 0.0,
         proposal_seq=proposal.proposal_seq, side=side)
    downstream_stop = gate(person_permit, 2.1, immediate=True)
    emit("accepted_zero_side_persistence",
         not downstream_stop.allowed and person_manager.committed
         and person_manager.pass_side == "left",
         accepted_v=0.0, reason=downstream_stop.reason,
         side=person_manager.pass_side)

    dropout = person_manager.update((), 2.2, True, summary_stamp_s=2.2)
    emit("single_dropout", dropout.active, reason=dropout.reason)
    for step, x_m in enumerate((1.2, 0.9, 0.6, 0.3, 0.0, -0.3, -0.6), 1):
        stamp_s = round(2.2 + step * 0.2, 1)
        behind = person_manager.update(
            (observation(stamp_s, x_m=x_m),), stamp_s, True,
            summary_stamp_s=stamp_s)
    emit("passing_behind", behind.active and behind.target_x_m < 0.0,
         target_x_m=behind.target_x_m)
    for frame in range(1, 4):
        released = person_manager.observe_tail_clear(True)
        emit(f"tail_clear_{frame}" + ("_release" if frame == 3 else ""),
             released is (frame == 3), released=released)
    resumed = person_manager.update((), 4.0, True, summary_stamp_s=4.0)
    emit("resume", not resumed.active and not person_manager.committed,
         command_v=0.0, reason=resumed.reason)

    adversarial_cases(person_permit, proposal)
    emit("summary", not FAILED, result="STATIC_THREAT_HOST_QA_PASS")
    return 1 if FAILED else 0


if __name__ == "__main__":
    raise SystemExit(main())
