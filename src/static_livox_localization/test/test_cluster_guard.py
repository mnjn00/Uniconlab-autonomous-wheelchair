"""Reading the object list, and deciding what to do about it.

This is the only guard left watching for people when the safety policies are
switched off, so its failure directions matter more than its accuracy. Every
one of them points the same way: unreadable is blocked, unjudged is moving,
and nothing that is not positively known to be standing still is ever driven
around.
"""

import importlib.util
import json
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    """Load a script module by path, with its siblings importable.

    Scoped and undone: leaving the scripts directory on sys.path for the
    rest of the session lets these module names shadow same-named ones
    elsewhere in the repo, which is a test failure somewhere unrelated and
    no clue at all as to why.
    """
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            name, SCRIPTS / ("%s.py" % name))
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


cg = load("cluster_guard")
ct = load("cluster_tracking")
policy = load("person_bypass_policy")


def summary(objects, status="OK", stamp=100.0):
    return cg.parse_summary(json.dumps(
        {"stamp": stamp, "status": status, "objects": objects}))


def obj(x, y, size=(0.5, 0.5, 1.0), motion=ct.STATIC, label="obstacle"):
    return {"class": label, "x": x, "y": y, "size": list(size),
            "points": 40, "motion": motion}


# ------------------------------------------------------------------ geometry

def test_a_box_whose_corner_reaches_the_corridor_is_a_threat():
    """The reason this uses extents and not centres. A van whose centre sits
    1.3 m to the side - well outside a 0.45 m corridor - still has its near
    flank 0.3 m from the centre line, and a guard comparing centres would
    drive along it."""
    van = obj(4.0, 1.3, size=(4.0, 2.0, 1.8), label="vehicle")
    threat = cg.nearest_threat(summary([van]), 0.45)
    assert threat is not None
    assert threat.distance_m == pytest.approx(2.0)   # near face, not centre


def test_an_object_clear_of_the_corridor_is_not_a_threat():
    assert cg.nearest_threat(summary([obj(4.0, 2.5)]), 0.45) is None


def test_the_nearest_of_several_wins():
    threat = cg.nearest_threat(
        summary([obj(8.0, 0.0), obj(3.0, 0.1), obj(5.0, -0.2)]), 0.45)
    assert threat.distance_m == pytest.approx(2.75)


def test_a_threat_preserves_direct_track_identity_and_producer_stamp():
    tracked = obj(3.0, 0.1, label="person")
    tracked["id"] = 1641

    threat = cg.nearest_threat(summary([tracked], stamp=123.4), 0.45)

    assert threat.track_id == 1641
    assert threat.observed_stamp_s == 123.4
    assert threat.directly_observed


def test_a_person_is_never_modelled_smaller_than_a_person():
    narrow = {
        "class": "person",
        "x": 2.0,
        "y": 0.0,
        "size": [0.18, 0.18, 1.7],
    }

    box = cg.object_box(narrow)

    assert box is not None
    assert box[2] >= 0.35
    assert box[3] >= 0.35


def test_person_size_floor_does_not_inflate_other_objects():
    thing = {
        "class": "obstacle",
        "x": 2.0,
        "y": 0.0,
        "size": [0.18, 0.18, 0.5],
    }

    assert cg.object_box(thing)[2:] == (0.09, 0.09)


def test_a_lateral_shift_moves_the_corridor_it_is_measured_against():
    """What the bypass probe rests on: stepping 0.6 m aside has to change
    which objects are in the way, or every offset looks equally blocked."""
    beside = obj(4.0, 0.9)
    assert cg.nearest_threat(summary([beside]), 0.45) is None
    assert cg.nearest_threat(summary([beside]), 0.45, lateral_shift_m=0.9) \
        is not None


def test_an_object_already_on_top_of_the_chair_is_here_not_behind_it():
    threat = cg.nearest_threat(summary([obj(0.2, 0.0, size=(2.0, 1.0, 1.0))]),
                               0.45)
    assert threat.distance_m == 0.0


def test_a_person_wholly_behind_the_chair_is_not_a_forward_threat():
    passed = obj(-0.8, 0.2, size=(0.5, 0.5, 1.7),
                 motion=ct.MOVING, label="person")

    assert cg.nearest_threat(summary([passed]), 0.50) is None


def test_a_passed_person_does_not_erase_front_obstacle_bypass_candidates():
    passed = obj(-0.8, 0.2, size=(0.5, 0.5, 1.7),
                 motion=ct.MOVING, label="person")
    front = obj(1.8, 0.0, size=(0.5, 0.5, 1.0), motion=ct.STATIC)

    blocks, points = cg.corridor_obstacle_points(
        summary([passed, front]), half_width_m=1.0)

    assert blocks
    assert points
    assert min(forward for forward, _lateral in points) > 1.0


# -------------------------------------------------------- failure directions

def test_an_unusable_summary_blocks_and_is_never_parked():
    threat = cg.nearest_threat(summary([], status="NO_CLOUD"), 0.45)
    assert threat.distance_m == cg.BLOCKED
    assert not threat.parked


@pytest.mark.parametrize("broken", [
    {"class": "obstacle", "y": 0.0, "size": [1, 1, 1]},        # no x
    {"class": "obstacle", "x": 3.0, "y": 0.0},                 # no size
    {"class": "obstacle", "x": "near", "y": 0.0, "size": [1, 1, 1]},
    {"class": "obstacle", "x": float("nan"), "y": 0.0, "size": [1, 1, 1]},
])
def test_a_malformed_object_blocks_rather_than_being_skipped(broken):
    """Skipping it means not seeing an obstacle. It also must not come out
    parked - a manoeuvre around something whose position did not parse is
    the one outcome a producer bug must not be able to cause."""
    threat = cg.nearest_threat(summary([broken]), 0.45)
    assert threat.distance_m == cg.BLOCKED
    assert not threat.parked


def test_an_unrecognised_motion_value_is_treated_as_moving():
    threat = cg.nearest_threat(summary([obj(4.0, 0.0, motion="parked?")]), 0.45)
    assert threat.motion == ct.MOVING


def test_an_object_with_no_motion_field_is_unknown_and_not_parked():
    bare = {"class": "obstacle", "x": 4.0, "y": 0.0, "size": [1, 1, 1]}
    threat = cg.nearest_threat(summary([bare]), 0.45)
    assert threat.motion == ct.UNKNOWN
    assert not threat.parked


@pytest.mark.parametrize("payload", [
    "", "not json", "[]", '{"objects": []}', '{"stamp": "now", "objects": []}',
    '{"stamp": 1.0}'])
def test_an_unparseable_summary_raises_rather_than_reading_as_empty(payload):
    with pytest.raises(ValueError):
        cg.parse_summary(payload)


def test_a_producer_that_never_spoke_is_stale_not_clear():
    assert cg.is_stale(None, 100.0)


def test_a_producer_that_went_quiet_is_stale():
    assert not cg.is_stale(100.0, 100.5)
    assert cg.is_stale(100.0, 100.0 + cg.STALE_S + 0.1)


def test_the_accumulation_window_matches_the_producer():
    """The consumer sizes its stopping envelope with this. If the producer's
    window grows and this does not, the chair brakes for where an object was
    rather than where it is."""
    producer = (SCRIPTS / "obstacle_clusters.py").read_text(encoding="utf-8")
    for line in producer.splitlines():
        if line.startswith("WINDOW_S"):
            assert float(line.split("=")[1].strip()) == cg.ACCUMULATION_S
            return
    raise AssertionError("obstacle_clusters.py no longer defines WINDOW_S")


# ------------------------------------------------------------ what to do next

def threat(distance, motion, label="obstacle"):
    return cg.Threat(
        distance, motion, label, lateral_m=0.0, track_id=1,
        observed_stamp_s=3.0, geometry_valid=True,
        center_x_m=distance, center_y_m=0.0)


def permit_for(threat_in):
    manager = policy.StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        permit = manager.update((policy.StaticThreatObservation(
            track_id=threat_in.track_id, stamp_s=stamp,
            x_m=threat_in.center_x_m, y_m=threat_in.center_y_m,
            size_x_m=0.5, size_y_m=0.5, label=threat_in.label,
            motion="static", source="geometric"),), stamp, True)
    return permit


def decide(threat_in, blocking=True, blocked_for_s=0.0, authorized=False):
    return cg.avoidance_decision(
        threat_in, blocking, 5.0,
        bypass_permit=permit_for(threat_in) if authorized else None,
        now_s=3.0)


def test_something_watched_standing_still_is_gone_around_from_a_distance():
    """The behaviour asked for: seen from far off and parked, drift past it
    rather than driving up to it and stopping."""
    assert decide(
        threat(4.0, ct.STATIC), blocking=False,
        authorized=True) == cg.GO_ROUND


def test_a_parked_thing_still_far_off_is_left_alone():
    assert decide(threat(9.0, ct.STATIC), blocking=False) == cg.CLEAR


@pytest.mark.parametrize("motion", [ct.MOVING, ct.UNKNOWN])
def test_anything_moving_or_unjudged_is_waited_out_not_driven_around(motion):
    assert decide(threat(2.0, motion)) == cg.WAIT


def test_a_moving_thing_is_never_gone_around_however_long_it_blocks():
    """The 3 s rule is evidence of parkedness for sources that cannot track.
    It must not overrule one that can: someone pacing in the corridor has
    blocked it for 3 s and is still going to step somewhere."""
    assert decide(threat(1.0, ct.MOVING), blocked_for_s=30.0) == cg.WAIT


def test_elapsed_blocked_time_never_authorizes_an_unknown_return():
    assert decide(threat(1.0, ct.UNKNOWN), blocked_for_s=4.0) == cg.WAIT


def person(distance, motion):
    return threat(distance, motion, cg.PERSON_LABEL)


def test_a_person_standing_still_is_waited_for_not_driven_around():
    """The rule the docstring always claimed and the code only half kept.

    CONFIRM_S is 1.5 s, so someone who stops to check a phone is STATIC,
    and STATIC is parked - which sent the chair around a stationary
    pedestrian from 8 m out without the blocked clock ever starting.
    """
    assert decide(person(4.0, ct.STATIC), blocking=False) == cg.WAIT
    assert decide(person(4.0, ct.STATIC)) == cg.WAIT


def test_a_person_is_not_gone_around_by_the_time_rule_either():
    """The slower road to the same place. Standing in the way for three
    seconds is evidence of parkedness for a thing; for a person it is
    evidence of nothing but that they are standing there."""
    assert decide(person(1.0, ct.STATIC), blocked_for_s=30.0) == cg.WAIT
    assert decide(person(1.0, ct.UNKNOWN), blocked_for_s=30.0) == cg.WAIT


def test_a_person_needs_explicit_static_bypass_authorization():
    assert decide(
        person(1.0, ct.STATIC),
        blocked_for_s=30.0,
        authorized=True) == cg.GO_ROUND
    assert decide(
        person(1.0, ct.MOVING),
        blocked_for_s=30.0,
        authorized=True) == cg.WAIT


def test_a_static_person_is_stopped_and_watched_before_blocking():
    assert decide(
        person(4.0, ct.STATIC),
        blocking=False,
        authorized=False) == cg.WAIT
    assert decide(
        person(4.0, ct.STATIC),
        blocking=False,
        authorized=True) == cg.GO_ROUND


def test_a_person_who_leaves_the_corridor_clears_it():
    """Outside the corridor there is no threat; distance is not absence."""
    assert decide(None, blocking=False) == cg.CLEAR


def test_the_same_geometry_without_the_label_is_still_gone_around():
    """The exclusion is the label, not a general loss of nerve: a parked
    motorcycle at the same range is still a thing to drift past."""
    assert decide(
        threat(4.0, ct.STATIC), blocking=False,
        authorized=True) == cg.GO_ROUND


def test_static_person_and_object_are_stop_only_until_shared_permit_is_ready():
    for candidate in (threat(2.0, ct.STATIC), person(2.0, ct.STATIC)):
        assert cg.avoidance_decision(
            candidate, True, 5.0, bypass_permit=None,
            now_s=3.0) == cg.WAIT
        assert cg.avoidance_decision(
            candidate, True, 5.0, bypass_permit=permit_for(candidate),
            now_s=3.0) == cg.GO_ROUND


def test_blocked_time_alone_never_authorizes_unknown_geometry():
    assert cg.avoidance_decision(
        threat(1.0, ct.UNKNOWN), True, 5.0,
        bypass_permit=None, now_s=3.0) == cg.WAIT


def test_decision_binds_permit_to_actual_corridor_track_label_and_geometry():
    manager = policy.StaticThreatBypassManager()
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        permit = manager.update((policy.StaticThreatObservation(
            track_id=7, stamp_s=stamp, x_m=2.0, y_m=0.2,
            size_x_m=0.6, size_y_m=0.6, label="box", motion="static",
            source="geometric"),), stamp, True)
    actual = cg.Threat(
        1.7, ct.STATIC, "box", lateral_m=0.2, track_id=7,
        observed_stamp_s=3.0, directly_observed=True,
        geometry_valid=True, center_x_m=2.0, center_y_m=0.2)
    assert cg.avoidance_decision(
        actual, True, 5.0, bypass_permit=permit,
        now_s=3.0) == cg.GO_ROUND

    for mismatch in (
        cg.Threat(1.7, ct.STATIC, "box", lateral_m=0.2, track_id=8,
                  observed_stamp_s=3.0, geometry_valid=True,
                  center_x_m=2.0, center_y_m=0.2),
        cg.Threat(1.7, ct.STATIC, "person", lateral_m=0.2, track_id=7,
                  observed_stamp_s=3.0, geometry_valid=True,
                  center_x_m=2.0, center_y_m=0.2),
        cg.Threat(1.7, ct.STATIC, "box", lateral_m=0.2, track_id=7,
                  observed_stamp_s=3.0, geometry_valid=False,
                  center_x_m=2.0, center_y_m=0.2),
        cg.Threat(1.7, ct.STATIC, "box", lateral_m=0.2, track_id=7,
                  observed_stamp_s=3.0, geometry_valid=True,
                  center_x_m=3.0, center_y_m=0.2),
        cg.Threat(1.7, ct.STATIC, "box", lateral_m=0.2, track_id=7,
                  observed_stamp_s=2.0, geometry_valid=True,
                  center_x_m=2.0, center_y_m=0.2),
    ):
        assert cg.avoidance_decision(
            mismatch, True, 5.0, bypass_permit=permit,
            now_s=3.0) == cg.WAIT
    assert cg.avoidance_decision(
        None, False, 5.0, bypass_permit=permit,
        now_s=3.0) == cg.CLEAR


@pytest.mark.parametrize("label", ["Person", " person ", "PERSON"])
def test_the_label_is_matched_the_way_producers_actually_write_it(label):
    assert cg.Threat(2.0, ct.STATIC, label).is_person


@pytest.mark.parametrize("label", ["", "obstacle", "vehicle", "personnel"])
def test_nothing_else_is_quietly_treated_as_a_person(label):
    """This exclusion only ever adds caution. A label it does not
    recognise must not become one it is unwilling to pass."""
    assert not cg.Threat(2.0, ct.STATIC, label).is_person


def test_a_clear_corridor_is_clear():
    assert decide(None, blocking=False) == cg.CLEAR


def test_the_chair_resumes_by_the_threat_going_away_not_by_a_timer():
    """A pedestrian crossing: blocked while they are in the corridor, clear
    the moment they are not. Nothing has to remember they were there."""
    assert decide(threat(1.5, ct.MOVING)) == cg.WAIT
    assert decide(None, blocking=False) == cg.CLEAR
