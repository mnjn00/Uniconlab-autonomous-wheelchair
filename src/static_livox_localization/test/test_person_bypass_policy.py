from pathlib import Path
import sys
from dataclasses import replace

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from person_bypass_policy import (  # noqa: E402
    PersonObservation,
    StaticPersonQualifier,
    evaluate_gate_override,
    permit_from_payload,
    permit_is_fresh,
    permit_matches_observation,
    person_observations,
    static_obstacle_permit,
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
    assert permit.min_clearance_m == 0.50


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


def test_map_compensated_identity_ignores_body_frame_rotation_jump():
    qualifier = StaticPersonQualifier(
        confirmation_s=.3, maximum_position_jump_m=.35)
    first = PersonObservation(
        track_id=1689, stamp_s=1.0, x_m=3.35, y_m=-.20,
        size_x_m=.6, size_y_m=.6, motion="static", source="geometric",
        tracking_x_m=20.0, tracking_y_m=5.0)
    # The recorded body-frame y moved 0.93 m while the chair turned. The
    # map-frame identity remained the same stationary person.
    second = replace(
        first, stamp_s=1.2, x_m=3.25, y_m=.73,
        tracking_x_m=20.04, tracking_y_m=5.02)
    third = replace(
        second, stamp_s=1.4, x_m=3.05, y_m=1.20,
        tracking_x_m=20.06, tracking_y_m=5.03)

    assert not qualifier.update((first,), 1.0, True).active
    assert not qualifier.update((second,), 1.2, True).active
    permit = qualifier.update((third,), 1.4, True)

    assert permit.active
    assert permit.track_id == 1689
    assert permit.target_y_m == 1.20


def test_fusion_cadence_jitter_does_not_reset_a_static_person():
    qualifier = StaticPersonQualifier(confirmation_s=1.0)

    permit = None
    for index in range(7):
        stamp = 1.0 + 0.2 * index
        observations = person_observations(summary(stamp, [person()]))
        permit = qualifier.update(observations, stamp + 0.36, True)

    assert permit is not None and permit.active


def test_observation_beyond_the_jitter_budget_resets_authorization():
    qualifier = StaticPersonQualifier(confirmation_s=0.2)
    qualifier.update(
        person_observations(summary(1.0, [person()])), 1.0, True)
    permit = qualifier.update(
        person_observations(summary(1.2, [person()])), 1.66, True)

    assert not permit.active
    assert permit.reason == "PERSON_OBSERVATION_STALE"


def test_same_track_remains_authorized_across_the_lateral_boundary():
    qualifier = StaticPersonQualifier(confirmation_s=0.4)
    for stamp in (1.0, 1.2, 1.4):
        observations = person_observations(
            summary(stamp, [person(y=-0.8)]), maximum_lateral_m=1.25)
        permit = qualifier.update(observations, stamp, True)
    assert permit.active

    for stamp, lateral in ((1.6, -1.0), (1.8, -1.2), (2.0, -1.413)):
        observations = person_observations(
            summary(stamp, [person(y=lateral)]), maximum_lateral_m=1.25)
        permit = qualifier.update(observations, stamp, True)

    assert permit.active


def test_new_track_cannot_acquire_inside_only_the_lateral_hysteresis():
    qualifier = StaticPersonQualifier(confirmation_s=0.4)

    permit = None
    for stamp in (1.0, 1.2, 1.4, 1.6):
        observations = person_observations(
            summary(stamp, [person(y=-1.413)]), maximum_lateral_m=1.25)
        permit = qualifier.update(observations, stamp, True)

    assert permit is not None and not permit.active
    assert permit.reason == "PERSON_OUTSIDE_MANEUVER_REGION"


def test_too_close_person_is_stop_only():
    qualifier = StaticPersonQualifier(
        confirmation_s=0.1, minimum_near_distance_m=.60)
    observations = person_observations(summary(1.0, [person(x=.8)]))
    assert qualifier.update(
        observations, 1.0, True).reason == "PERSON_TOO_CLOSE"


def test_qualified_person_safely_beside_remains_authorized():
    qualifier = StaticPersonQualifier(
        confirmation_s=.4, minimum_near_distance_m=.60,
        min_clearance_m=.50)
    for stamp in (1.0, 1.2, 1.4):
        permit = qualifier.update(person_observations(
            summary(stamp, [person(x=2.0, y=.9)])), stamp, True)
    assert permit.active

    for stamp, x_m in ((1.6, 1.7), (1.8, 1.4), (2.0, 1.1), (2.2, .8)):
        beside = person_observations(summary(
            stamp, [person(x=x_m, y=.9, motion="static")]))
        permit = qualifier.update(beside, stamp, True)

    assert permit.active
    assert permit.reason == "STATIC_PERSON_PASSED_SIDE"
    assert permit_matches_observation(permit, beside[0])

    moving = person_observations(summary(
        2.4, [person(x=.7, y=.9, motion="moving")]))
    revoked = qualifier.update(moving, 2.4, True)
    assert not revoked.active
    assert revoked.reason == "PERSON_NOT_CONFIRMED_STATIC"


def test_person_near_the_path_cannot_use_passed_side_permission():
    qualifier = StaticPersonQualifier(
        confirmation_s=.4, minimum_near_distance_m=.60,
        min_clearance_m=.50)
    for stamp in (1.0, 1.2, 1.4):
        qualifier.update(person_observations(
            summary(stamp, [person(x=2.0, y=.9)])), stamp, True)

    permit = qualifier.update(person_observations(summary(
        1.6, [person(x=.7, y=.6)])), 1.6, True)

    assert not permit.active
    assert permit.reason == "PERSON_TOO_CLOSE"


def test_unqualified_or_moving_side_person_remains_stop_only():
    for motion in ("static", "moving", "unknown"):
        qualifier = StaticPersonQualifier(
            confirmation_s=.4, minimum_near_distance_m=.60,
            min_clearance_m=.50)
        permit = qualifier.update(person_observations(summary(
            1.0, [person(x=.7, y=.9, motion=motion)])), 1.0, True)
        assert not permit.active


def test_passed_person_dropout_gets_only_a_short_clearance_heartbeat():
    qualifier = StaticPersonQualifier(
        confirmation_s=.4, maximum_gap_s=.45,
        minimum_near_distance_m=.60, min_clearance_m=.50)
    for stamp in (1.0, 1.2, 1.4):
        qualifier.update(person_observations(
            summary(stamp, [person(x=2.0, y=.9)])), stamp, True)
    for stamp, x_m in ((1.6, 1.7), (1.8, 1.4), (2.0, 1.1), (2.2, .8)):
        permit = qualifier.update(person_observations(summary(
            stamp, [person(x=x_m, y=.9)])), stamp, True)
    assert permit.active and permit.reason == "STATIC_PERSON_PASSED_SIDE"

    permit = qualifier.update((), 2.4, True)
    assert permit.active and permit.reason == "STATIC_PERSON_PASSED_SIDE"
    permit = qualifier.update((), 2.7, True)
    assert not permit.active and permit.reason == "NO_PERSON"


def test_one_stale_side_observation_does_not_erase_a_safe_pass():
    qualifier = StaticPersonQualifier(
        confirmation_s=.4, maximum_gap_s=.45, passed_side_grace_s=1.0,
        minimum_near_distance_m=.60, min_clearance_m=.50)
    for stamp in (1.0, 1.2, 1.4):
        qualifier.update(person_observations(summary(
            stamp, [person(x=2.0, y=.9)])), stamp, True)
    for stamp, x_m in ((1.6, 1.7), (1.8, 1.4), (2.0, 1.1), (2.2, .8)):
        permit = qualifier.update(person_observations(summary(
            stamp, [person(x=x_m, y=.9)])), stamp, True)
    assert permit.active and permit.reason == "STATIC_PERSON_PASSED_SIDE"

    stale = person_observations(summary(2.2, [person(x=.7, y=.9)]))
    permit = qualifier.update(stale, 2.75, True)
    assert permit.active and permit.reason == "STATIC_PERSON_PASSED_SIDE"

    fresh = person_observations(summary(2.8, [person(x=.6, y=.9)]))
    permit = qualifier.update(fresh, 2.8, True)
    assert permit.active and permit.reason == "STATIC_PERSON_PASSED_SIDE"


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


def test_direct_static_object_gets_a_short_trajectory_permit():
    permit = static_obstacle_permit(
        now_s=10.0, observed_stamp_s=9.8, track_id=42,
        target_x_m=2.0, target_y_m=0.3, motion="static",
        directly_observed=True, geometry_valid=True)

    assert permit.active
    assert permit.track_id == 42
    assert permit.reason == "STATIC_OBJECT_BYPASS"
    assert permit.expires_s == 10.45


def test_untrusted_or_non_static_object_cannot_authorize_motion():
    common = dict(
        now_s=10.0, observed_stamp_s=9.8, track_id=42,
        target_x_m=2.0, target_y_m=0.3, motion="static",
        directly_observed=True, geometry_valid=True)
    variants = (
        dict(motion="moving"),
        dict(motion="unknown"),
        dict(track_id=None),
        dict(directly_observed=False),
        dict(geometry_valid=False),
        dict(observed_stamp_s=9.4),
    )

    for variant in variants:
        assert not static_obstacle_permit(
            **dict(common, **variant)).active


def test_object_permit_cannot_suppress_the_person_semantic_stop():
    permit = static_obstacle_permit(
        now_s=10.0, observed_stamp_s=9.8, track_id=7,
        target_x_m=3.0, target_y_m=0.0, motion="static",
        directly_observed=True, geometry_valid=True)
    observation = person_observations(summary(9.8, [person()]))[0]

    assert not permit_matches_observation(permit, observation)


def test_raw_gate_override_accepts_any_clear_swept_path_and_rejects_collisions():
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
    straight = evaluate_gate_override(
        **dict(common, requested_w_rps=0.0))
    assert straight.allowed
    assert not evaluate_gate_override(
        **dict(common, immediate_collision=True)).allowed
    assert not evaluate_gate_override(
        **dict(common, requested_path_collision=True)).allowed
    assert not evaluate_gate_override(
        **dict(common, carried_path_collision=True)).allowed
    assert not evaluate_gate_override(
        **dict(common, now_s=permit.expires_s + .01)).allowed
