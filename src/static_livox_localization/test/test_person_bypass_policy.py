from pathlib import Path
import sys

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from person_bypass_policy import (  # noqa: E402
    StaticThreatBypassManager,
    StaticThreatObservation,
    commit_pass_side,
    evaluate_gate_override,
    permit_from_payload,
    permit_is_fresh,
    permit_matches_observation,
    permit_matches_threat,
    person_observations,
    threat_observations,
    tail_clear_release,
)


def observed_threat(*, stamp=1.0, track=7, label="person", x=3.0,
                    y=0.0, motion="static", directly_observed=True,
                    geometry_valid=True):
    return StaticThreatObservation(
        track_id=track, stamp_s=stamp, x_m=x, y_m=y,
        size_x_m=0.7, size_y_m=0.7, label=label, motion=motion,
        source="geometric", directly_observed=directly_observed,
        geometry_valid=geometry_valid,
    )


def test_generic_manager_waits_exactly_two_seconds_for_person_and_object():
    for label in ("person", "box"):
        manager = StaticThreatBypassManager()
        for stamp in (10.0, 10.4, 10.8, 11.2, 11.6):
            permit = manager.update(
                (observed_threat(stamp=stamp, label=label),), stamp, True)
            assert not permit.active
        permit = manager.update(
            (observed_threat(stamp=12.0, label=label),), 12.0, True)
        assert permit.active
        assert permit.static_for_s == 2.0
        assert permit.reason == "STATIC_THREAT_BYPASS"
        assert manager.lifecycle == "BYPASS_COMMITTED"


def test_v2_parser_rejects_legacy_coercions_unknown_fields_and_weak_permits():
    manager = StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        permit = manager.update((observed_threat(stamp=stamp),), stamp, True)
    valid = permit.as_dict()
    assert permit_from_payload(valid) is not None

    invalid_payloads = []
    for key in tuple(valid):
        invalid_payloads.append({k: v for k, v in valid.items() if k != key})
    invalid_payloads.extend((
        dict(valid, schema="person-bypass/v1"),
        dict(valid, active="true"),
        dict(valid, capable=1),
        dict(valid, reason="STATIC_PERSON_BYPASS"),
        dict(valid, reason="bogus"),
        dict(valid, static_for_s=0.0),
        dict(valid, max_speed_mps=0.0),
        dict(valid, min_clearance_m=0.0),
        dict(valid, track_id=-1),
        dict(valid, threat_label="Person"),
        dict(valid, threat_label=""),
        dict(valid, threat_label="../person"),
        dict(valid, unexpected=True),
    ))
    for payload in invalid_payloads:
        assert permit_from_payload(payload) is None


def test_pass_side_commit_and_three_clear_frames_are_explicit_and_pure():
    assert commit_pass_side(None, "left") == "left"
    assert commit_pass_side("left", "right") == "left"
    assert tail_clear_release(0, True) == (1, False)
    assert tail_clear_release(1, False) == (0, False)
    assert tail_clear_release(2, True) == (3, True)

    manager = StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        manager.update((observed_threat(stamp=stamp),), stamp, True)
    assert manager.commit_pass_side("right") == "right"
    assert manager.lifecycle == "PASSING"
    assert not manager.observe_tail_clear(True)
    assert manager.lifecycle == "CLEARING"
    assert not manager.observe_tail_clear(True)
    assert manager.observe_tail_clear(True)
    assert manager.lifecycle == "IDLE"
    assert manager.pass_side is None


def test_transient_dynamic_frame_holds_identity_but_sustained_motion_resets():
    manager = StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        manager.update((observed_threat(stamp=stamp),), stamp, True)
    manager.commit_pass_side("left")

    transient = manager.update(
        (observed_threat(stamp=3.2, motion="moving"),), 3.2, True)
    assert not transient.active
    assert manager.track_id == 7
    assert manager.pass_side == "left"
    recovered = manager.update(
        (observed_threat(stamp=3.4),), 3.4, True)
    assert recovered.active
    assert manager.lifecycle == "PASSING"

    held = manager.update(
        (observed_threat(stamp=3.5, motion="unknown"),), 3.5, True)
    assert not held.active
    reset = manager.update(
        (observed_threat(stamp=4.0, motion="unknown"),), 4.0, True)
    assert not reset.active
    assert manager.lifecycle == "IDLE"
    assert manager.track_id is None


def test_dynamic_recovery_uses_producer_stamps_not_callback_lag():
    manager = StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        manager.update(
            (observed_threat(stamp=stamp),), stamp + 0.36, True)

    moving = (observed_threat(stamp=3.2, motion="moving"),)
    assert not manager.update(
        moving, 3.56, True, dynamic_conflict=True,
        summary_stamp_s=3.2).active
    recovered = manager.update(
        (observed_threat(stamp=3.4),), 3.76, True,
        summary_stamp_s=3.4)
    assert recovered.active


def test_committed_track_stays_permitted_past_rear_until_three_tail_clears():
    manager = StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        permit = manager.update(
            (observed_threat(stamp=stamp, x=3.0),), stamp, True)
    assert permit.active
    manager.commit_pass_side("left")

    stamp = 3.2
    for x_m in (2.7, 2.4, 2.1, 1.8, 1.5, 1.2, 0.9, 0.6,
                0.3, 0.0, -0.3, -0.6):
        permit = manager.update(
            (observed_threat(stamp=stamp, x=x_m),), stamp, True)
        assert permit.active
        assert permit.track_id == 7
        assert manager.lifecycle == "PASSING"
        stamp += 0.2

    assert not manager.observe_tail_clear(True)
    assert not manager.observe_tail_clear(True)
    assert manager.observe_tail_clear(True)
    assert manager.lifecycle == "IDLE"
    assert manager.track_id is None


def test_retained_track_observation_survives_entry_region_filters():
    behind = person(track=7, x=-0.6, y=1.4)
    payload = summary(4.0, [behind])
    assert threat_observations(payload) == ()
    retained = threat_observations(payload, retained_track_id=7)
    assert len(retained) == 1
    assert retained[0].track_id == 7


def test_precommit_missing_invalid_or_dynamic_observation_resets_timer():
    manager = StaticThreatBypassManager()
    manager.update((observed_threat(stamp=1.0),), 1.0, True)
    manager.update((observed_threat(stamp=1.8),), 1.8, True)

    for stamp, observations in (
        (2.0, ()),
        (3.0, (observed_threat(stamp=3.0, geometry_valid=False),)),
        (4.0, (observed_threat(stamp=4.0, motion="moving"),)),
        (5.0, (observed_threat(stamp=5.0, motion="unknown"),)),
    ):
        assert not manager.update(observations, stamp, True).active

    for stamp in (6.0, 6.4, 6.8, 7.2, 7.6):
        assert not manager.update(
            (observed_threat(stamp=stamp),), stamp, True).active
    assert manager.update((observed_threat(stamp=8.0),), 8.0, True).active


def test_malformed_summary_object_is_an_inactive_dynamic_conflict():
    observations = threat_observations(summary(1.0, [{"class": "box"}]))
    assert len(observations) == 1
    assert not observations[0].geometry_valid
    manager = StaticThreatBypassManager()
    assert not manager.update(
        observations, 1.0, True,
        dynamic_conflict=True).active


def test_postcommit_retains_one_bounded_healthy_dropout_only():
    manager = StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        permit = manager.update(
            (observed_threat(stamp=stamp),), stamp, True)
    assert permit.active

    retained = manager.update((), 3.2, True, summary_healthy=True)
    assert retained.active
    assert retained.reason == "STATIC_THREAT_DROPOUT_GRACE"
    assert permit_from_payload(retained.as_dict()) is not None
    assert not manager.update(
        (), 3.3, True, summary_healthy=True).active

    manager = StaticThreatBypassManager()
    for stamp in (4.0, 4.4, 4.8, 5.2, 5.6, 6.0):
        manager.update((observed_threat(stamp=stamp),), stamp, True)
    assert not manager.update(
        (), 6.2, True, summary_healthy=False).active


def test_postcommit_dynamic_conflict_or_current_motion_fails_closed():
    manager = StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        manager.update((observed_threat(stamp=stamp),), stamp, True)

    assert not manager.update(
        (observed_threat(stamp=3.2),), 3.2, True,
        dynamic_conflict=True).active

    manager = StaticThreatBypassManager()
    for stamp in (4.0, 4.4, 4.8, 5.2, 5.6, 6.0):
        manager.update((observed_threat(stamp=stamp),), stamp, True)
    assert not manager.update(
        (observed_threat(stamp=6.2, motion="moving"),), 6.2,
        True).active


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
    qualifier = StaticThreatBypassManager(confirmation_s=3.0)
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
        qualifier = StaticThreatBypassManager(confirmation_s=0.5)
        observations = person_observations(summary(1.0, [item]))
        permit = qualifier.update(observations, 1.0, True)
        assert not permit.active
    qualifier = StaticThreatBypassManager(confirmation_s=0.5)
    observations = person_observations(
        summary(1.0, [person(), person(track=8, y=.7)]))
    assert not qualifier.update(observations, 1.0, True).active


def test_dropout_identity_change_jump_and_tracking_loss_reset_the_timer():
    qualifier = StaticThreatBypassManager(confirmation_s=1.0)
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


def test_fusion_cadence_jitter_does_not_reset_a_static_person():
    qualifier = StaticThreatBypassManager(confirmation_s=1.0)

    permit = None
    for index in range(7):
        stamp = 1.0 + 0.2 * index
        observations = person_observations(summary(stamp, [person()]))
        permit = qualifier.update(observations, stamp + 0.36, True)

    assert permit is not None and permit.active


def test_observation_beyond_the_jitter_budget_resets_authorization():
    qualifier = StaticThreatBypassManager(confirmation_s=0.2)
    qualifier.update(
        person_observations(summary(1.0, [person()])), 1.0, True)
    permit = qualifier.update(
        person_observations(summary(1.2, [person()])), 1.66, True)

    assert not permit.active
    assert permit.reason == "THREAT_OBSERVATION_STALE"


def test_same_track_remains_authorized_across_the_lateral_boundary():
    qualifier = StaticThreatBypassManager(confirmation_s=0.4)
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
    qualifier = StaticThreatBypassManager(confirmation_s=0.4)

    permit = None
    for stamp in (1.0, 1.2, 1.4, 1.6):
        observations = person_observations(
            summary(stamp, [person(y=-1.413)]), maximum_lateral_m=1.25)
        permit = qualifier.update(observations, stamp, True)

    assert permit is not None and not permit.active
    assert permit.reason == "THREAT_OUTSIDE_MANEUVER_REGION"


def test_too_close_person_is_stop_only():
    qualifier = StaticThreatBypassManager(
        confirmation_s=0.1, minimum_near_distance_m=.60)
    observations = person_observations(summary(1.0, [person(x=.8)]))
    assert qualifier.update(
        observations, 1.0, True).reason == "THREAT_TOO_CLOSE"


def test_permit_round_trip_freshness_and_target_match():
    qualifier = StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        permit = qualifier.update(
            person_observations(summary(stamp, [person()])), stamp, True)
    parsed = permit_from_payload(permit.to_json())
    assert parsed is not None and parsed.active
    assert permit.as_dict()["schema"] == "static-threat-bypass/v2"
    assert permit_is_fresh(parsed, 3.1)
    observation = person_observations(summary(3.0, [person()]))[0]
    assert permit_matches_observation(parsed, observation)
    assert not permit_is_fresh(parsed, 2.0)


def test_direct_static_object_gets_a_short_trajectory_permit_after_wait():
    manager = StaticThreatBypassManager()
    for stamp in (7.8, 8.2, 8.6, 9.0, 9.4, 9.8):
        permit = manager.update(
            (observed_threat(
                stamp=stamp, track=42, label="box", x=2.0, y=0.3),),
            stamp, True)

    assert permit.active
    assert permit.track_id == 42
    assert permit.reason == "STATIC_THREAT_BYPASS"
    assert permit.expires_s == 10.25


def test_untrusted_or_non_static_object_cannot_authorize_motion():
    common = dict(
        stamp=9.8, track=42, label="box", x=2.0, y=0.3,
        motion="static", directly_observed=True, geometry_valid=True)
    variants = (
        dict(motion="moving"),
        dict(motion="unknown"),
        dict(track=-1),
        dict(directly_observed=False),
        dict(geometry_valid=False),
        dict(stamp=9.0),
    )

    for variant in variants:
        manager = StaticThreatBypassManager(confirmation_s=0.1)
        permit = manager.update(
            (observed_threat(**dict(common, **variant)),), 9.8, True)
        assert not permit.active


def test_object_permit_cannot_suppress_the_person_semantic_stop():
    manager = StaticThreatBypassManager(confirmation_s=0.1)
    manager.update(
        (observed_threat(stamp=9.6, label="box"),), 9.6, True)
    permit = manager.update(
        (observed_threat(stamp=9.8, label="box"),), 9.8, True)
    observation = person_observations(summary(9.8, [person()]))[0]

    assert not permit_matches_observation(permit, observation)


def test_raw_gate_override_requires_curved_clear_path_and_stopped_carried_path():
    permit = qualify(StaticThreatBypassManager(confirmation_s=3.0))
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
