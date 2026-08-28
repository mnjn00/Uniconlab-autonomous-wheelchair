from pathlib import Path
import sys

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from person_bypass_policy import (  # noqa: E402
    StaticPersonQualifier,
    evaluate_gate_override,
    permit_from_payload,
    permit_is_fresh,
    permit_matches_observation,
    person_observations,
)


def summary(stamp, people):
    return {"status": "OK", "stamp": stamp, "objects": people}


def person(track=7, x=3.0, y=0.0, motion="static", source="geometric"):
    return {
        "id": track, "class": "person", "motion": motion,
        "source": source, "x": x, "y": y, "size": [0.7, 0.7, 1.7],
    }


def qualify(qualifier, start=10.0):
    permit = None
    for index in range(18):
        stamp = start + 0.2 * index
        observations = person_observations(summary(stamp, [person()]))
        permit = qualifier.update(observations, stamp, True)
    return permit


def test_direct_same_track_static_person_becomes_bypassable():
    qualifier = StaticPersonQualifier(confirmation_s=3.0)
    permit = qualify(qualifier)
    assert permit.active
    assert permit.track_id == 7
    assert permit.max_speed_mps == 0.35
    assert permit.min_clearance_m == 0.80


def test_moving_unknown_learned_only_and_multiple_people_never_authorize():
    for item in (
        person(motion="moving"),
        person(motion="unknown"),
        person(track=-100, source="learned_only"),
    ):
        qualifier = StaticPersonQualifier(confirmation_s=0.5)
        observations = person_observations(summary(1.0, [item]))
        permit = qualifier.update(observations, 1.0, True)
        assert not permit.active
    qualifier = StaticPersonQualifier(confirmation_s=0.5)
    observations = person_observations(
        summary(1.0, [person(), person(track=8, y=.7)]))
    assert not qualifier.update(observations, 1.0, True).active


def test_dropout_identity_change_jump_and_tracking_loss_reset_the_timer():
    qualifier = StaticPersonQualifier(confirmation_s=1.0)
    qualifier.update(person_observations(summary(1.0, [person()])), 1.0, True)
    qualifier.update(person_observations(summary(1.2, [person()])), 1.2, True)
    assert not qualifier.update((), 1.4, True).active
    qualifier.update(person_observations(summary(2.0, [person()])), 2.0, True)
    qualifier.update(
        person_observations(summary(2.2, [person(track=8)])), 2.2, True)
    assert not qualifier.update(
        person_observations(summary(2.4, [person(track=8)])), 2.4, True).active
    qualifier.update(person_observations(summary(3.0, [person()])), 3.0, True)
    qualifier.update(
        person_observations(summary(3.2, [person(x=3.8)])), 3.2, True)
    assert not qualifier.update(
        person_observations(summary(3.4, [person(x=3.8)])), 3.4, True).active
    assert not qualifier.update(
        person_observations(summary(4.0, [person()])), 4.0, False).active


def test_too_close_person_is_stop_only():
    qualifier = StaticPersonQualifier(
        confirmation_s=0.1, minimum_near_distance_m=.60)
    observations = person_observations(summary(1.0, [person(x=.8)]))
    assert qualifier.update(
        observations, 1.0, True).reason == "PERSON_TOO_CLOSE"


def test_permit_round_trip_freshness_and_target_match():
    qualifier = StaticPersonQualifier(confirmation_s=.2)
    qualifier.update(person_observations(summary(1.0, [person()])), 1.0, True)
    permit = qualifier.update(
        person_observations(summary(1.2, [person()])), 1.2, True)
    parsed = permit_from_payload(permit.to_json())
    assert parsed is not None and parsed.active
    assert permit_is_fresh(parsed, 1.3)
    observation = person_observations(summary(1.2, [person()]))[0]
    assert permit_matches_observation(parsed, observation)
    assert not permit_is_fresh(parsed, 2.0)


def test_raw_gate_override_requires_curved_clear_path_and_stopped_carried_path():
    permit = qualify(StaticPersonQualifier(confirmation_s=3.0))
    common = dict(
        permit=permit, now_s=permit.stamp_s + .05,
        requested_v_mps=.35, requested_w_rps=.25,
        immediate_collision=False,
        requested_path_collision=False,
        carried_path_collision=False,
    )
    decision = evaluate_gate_override(**common)
    assert decision.allowed and decision.speed_cap_mps == .35
    assert not evaluate_gate_override(
        **dict(common, requested_w_rps=.01)).allowed
    assert not evaluate_gate_override(
        **dict(common, immediate_collision=True)).allowed
    assert not evaluate_gate_override(
        **dict(common, requested_path_collision=True)).allowed
    assert not evaluate_gate_override(
        **dict(common, carried_path_collision=True)).allowed
    assert not evaluate_gate_override(
        **dict(common, now_s=permit.expires_s + .01)).allowed


def test_one_missed_frame_does_not_reset_static_qualification():
    qualifier = StaticPersonQualifier(
        confirmation_s=3.0, maximum_gap_s=1.0)
    permit = None
    for stamp in (1.0, 1.2, 1.4, 2.0, 2.2, 2.4, 3.0, 3.2, 3.4, 4.0):
        permit = qualifier.update(
            person_observations(summary(stamp, [person()])), stamp, True)
    assert permit.active


def test_brief_motion_flicker_freezes_but_does_not_count_as_static():
    qualifier = StaticPersonQualifier(
        confirmation_s=1.0, maximum_gap_s=1.0, flicker_s=1.0)
    for stamp in (1.0, 1.2, 1.4, 1.6):
        permit = qualifier.update(
            person_observations(summary(stamp, [person()])), stamp, True)
    assert not permit.active
    permit = qualifier.update(
        person_observations(summary(1.8, [person(motion="unknown")])),
        1.8, True)
    assert not permit.active
    permit = qualifier.update(
        person_observations(summary(2.0, [person()])), 2.0, True)
    assert not permit.active
    permit = qualifier.update(
        person_observations(summary(2.2, [person()])), 2.2, True)
    assert not permit.active
    permit = qualifier.update(
        person_observations(summary(2.4, [person()])), 2.4, True)
    assert permit.active


def test_motion_immediately_revokes_a_committed_permit():
    qualifier = StaticPersonQualifier(confirmation_s=0.4)
    permit = None
    for stamp in (1.0, 1.2, 1.4):
        permit = qualifier.update(
            person_observations(summary(stamp, [person()])), stamp, True)
    assert permit.active
    permit = qualifier.update(
        person_observations(summary(1.6, [person(motion="moving")])),
        1.6, True)
    assert not permit.active
