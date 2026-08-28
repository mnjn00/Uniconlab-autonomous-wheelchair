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

def threat(distance, motion):
    return cg.Threat(distance, motion)


def decide(threat_in, blocking=True, blocked_for_s=0.0,
           person_bypass_ready=False):
    return cg.avoidance_decision(
        threat_in, blocking, blocked_for_s, 5.0, 3.0,
        person_bypass_ready=person_bypass_ready)


def test_something_watched_standing_still_is_gone_around_from_a_distance():
    """The behaviour asked for: seen from far off and parked, drift past it
    rather than driving up to it and stopping."""
    assert decide(threat(4.0, ct.STATIC), blocking=False) == cg.GO_ROUND


def test_a_parked_thing_still_far_off_is_left_alone():
    assert decide(threat(9.0, ct.STATIC), blocking=False) == cg.CLEAR


@pytest.mark.parametrize("motion", [ct.MOVING, ct.UNKNOWN, ct.STATIC])
def test_objects_in_the_planning_distance_are_gone_around(motion):
    assert decide(threat(2.0, motion)) == cg.GO_ROUND


def test_a_moving_object_is_still_gone_around():
    assert decide(threat(1.0, ct.MOVING), blocked_for_s=0.0) == cg.GO_ROUND


def test_an_untrackable_return_is_gone_around_without_waiting_three_seconds():
    assert decide(threat(1.0, ct.UNKNOWN), blocked_for_s=0.0) == cg.GO_ROUND


def person(distance, motion):
    return cg.Threat(distance, motion, cg.PERSON_LABEL)


def test_a_person_in_the_planning_distance_is_gone_around():
    assert decide(person(4.0, ct.STATIC), blocking=False) == cg.PERSON_BYPASS
    assert decide(person(4.0, ct.STATIC)) == cg.PERSON_BYPASS
    assert decide(person(1.0, ct.MOVING)) == cg.PERSON_BYPASS
    assert decide(person(1.0, ct.UNKNOWN)) == cg.PERSON_BYPASS


def test_person_bypass_ready_is_not_required_to_go_around_a_person():
    assert decide(
        person(1.0, ct.STATIC),
        blocked_for_s=0.0,
        person_bypass_ready=False) == cg.PERSON_BYPASS
    assert decide(
        person(1.0, ct.MOVING),
        blocked_for_s=0.0,
        person_bypass_ready=False) == cg.PERSON_BYPASS


def test_a_static_person_ahead_is_gone_around_before_blocking():
    assert decide(
        person(4.0, ct.STATIC),
        blocking=False,
        person_bypass_ready=False) == cg.PERSON_BYPASS


def test_a_person_who_leaves_the_corridor_clears_it():
    """Outside the corridor there is no threat; distance is not absence."""
    assert decide(None, blocking=False) == cg.CLEAR


def test_the_same_geometry_without_the_label_is_still_gone_around():
    """The exclusion is the label, not a general loss of nerve: a parked
    motorcycle at the same range is still a thing to drift past."""
    assert decide(threat(4.0, ct.STATIC), blocking=False) == cg.GO_ROUND


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
    """A pedestrian crossing: gone around while they are in the corridor,
    clear the moment they are not."""
    assert decide(threat(1.5, ct.MOVING)) == cg.GO_ROUND
    assert decide(None, blocking=False) == cg.CLEAR


def test_unusable_geometry_is_waited_out_not_guessed_around():
    broken = cg.Threat(1.0, ct.STATIC, cg.PERSON_LABEL, geometry_valid=False)
    assert decide(broken) == cg.WAIT


def watched(stamp, motion=None, track_id=17, distance=1.6):
    if motion is None:
        motion = ct.STATIC
    return cg.Threat(
        distance, motion, cg.PERSON_LABEL, track_id=track_id,
        observed_stamp_s=stamp)


def tick(clock, stamp, motion=None, track_id=17, extra_moving=False,
         tracking_ok=True):
    return cg.advance_person_bypass_clock(
        clock, watched(stamp, motion=motion, track_id=track_id),
        extra_moving=extra_moving, tracking_ok=tracking_ok)


def test_three_seconds_of_static_same_track_authorizes_bypass():
    """2026-08-27 live stack: 3.0 s same-track STATIC, not 10.0 s."""
    clock = None
    ready = False
    for index in range(15):
        ready, clock = tick(clock, 100.0 + index * 0.2)
        assert not ready
    ready, clock = tick(clock, 103.0)
    assert ready
    assert clock[4] == 17


def test_one_missed_five_hertz_frame_does_not_zero_the_clock():
    """The 0.35 s gap on 08-27 reset QUALIFYING at a median 1.2 s."""
    clock = None
    for index in range(10):
        _ready, clock = tick(clock, 100.0 + index * 0.2)
    ready, clock = tick(clock, 102.4)
    assert not ready
    ready, _clock = tick(clock, 103.0)
    assert ready


def test_a_brief_unknown_or_moving_flicker_freezes_instead_of_resetting():
    clock = None
    for index in range(10):
        _ready, clock = tick(clock, 100.0 + index * 0.2)
    _ready, clock = tick(clock, 102.0, motion=ct.UNKNOWN)
    _ready, clock = tick(clock, 102.2, motion=ct.MOVING)
    ready, _clock = tick(clock, 103.2)
    assert ready


def test_a_walking_person_still_revokes_a_committed_bypass():
    clock = None
    ready = False
    for index in range(16):
        ready, clock = tick(clock, 100.0 + index * 0.2)
    assert ready
    ready, _clock = tick(clock, 103.2, motion=ct.MOVING)
    assert not ready


def test_a_second_moving_person_blocks_qualification():
    clock = None
    for index in range(16):
        ready, clock = tick(clock, 100.0 + index * 0.2, extra_moving=True)
        assert not ready


def test_an_empty_dropout_clears_the_clock():
    clock = None
    for index in range(16):
        _ready, clock = tick(clock, 100.0 + index * 0.2)
    ready, clock = cg.advance_person_bypass_clock(clock, None)
    assert not ready
    ready, _clock = tick(clock, 103.4)
    assert not ready
