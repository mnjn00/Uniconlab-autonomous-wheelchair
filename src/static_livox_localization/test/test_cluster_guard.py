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
           person_still_for_s=None):
    return cg.avoidance_decision(threat_in, blocking, blocked_for_s, 5.0, 3.0,
                                 person_still_for_s=person_still_for_s)


def test_something_watched_standing_still_is_gone_around_from_a_distance():
    """The behaviour asked for: seen from far off and parked, drift past it
    rather than driving up to it and stopping."""
    assert decide(threat(4.0, ct.STATIC), blocking=False) == cg.GO_ROUND


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


def test_an_untrackable_return_that_has_not_moved_for_3s_is_gone_around():
    """The raw scan has no identity, so standing there is all the evidence
    it can offer, and this is the pre-existing behaviour it keeps."""
    assert decide(threat(1.0, ct.UNKNOWN), blocked_for_s=4.0) == cg.GO_ROUND


def person(distance, motion):
    return cg.Threat(distance, motion, cg.PERSON_LABEL)


# Where the operator was standing when the bypass was finally granted and
# the planner could not act on it, 2026-08-25. The status line read
# HOLD:DWA_OBSTACLE: no arc clears anybody by PERSON_BYPASS_CLEARANCE_M
# from a metre away, dead ahead.
FIELD_TOO_LATE_M = 1.06
# Where they were standing when the chair first saw them holding still.
FIELD_IN_TIME_M = 3.83


def test_a_person_who_has_stood_still_long_enough_is_gone_round():
    """2026-08-25: the operator stood in the corridor and the chair waited
    them out. Five seconds is the line between someone pausing mid-stride
    and someone who is simply standing there."""
    assert decide(person(FIELD_IN_TIME_M, ct.STATIC),
                  person_still_for_s=6.0) == cg.GO_ROUND


def test_the_decision_is_made_while_there_is_still_room_to_act_on_it():
    """The first version timed this off the blocked clock, which only
    starts once they are inside the stopping radius. Over the five seconds
    it took to run, the chair closed from 3.1 m to 1.1 m, and at 1.1 m dead
    ahead there is no arc that clears anyone by 0.80 m - the planner said
    OBSTACLE and the chair stood there having been given permission it
    could no longer use.
    """
    assert FIELD_IN_TIME_M > FIELD_TOO_LATE_M
    assert decide(person(FIELD_IN_TIME_M, ct.STATIC),
                  blocking=False, blocked_for_s=None,
                  person_still_for_s=6.0) == cg.GO_ROUND, \
        "the clock must run before they are close enough to block"


def test_the_blocked_clock_alone_no_longer_grants_it():
    """Being stood in front of for a long time is not the evidence; having
    watched them hold still is."""
    assert decide(person(1.0, ct.STATIC), blocked_for_s=60.0) == cg.WAIT


def test_the_person_threshold_is_longer_than_the_one_for_a_thing():
    """decide() passes 3.0 for a thing. A person gets longer, because the
    evidence has to separate a pause from a stand, and 1.5 s of stillness
    is all the tracker needs to call someone STATIC."""
    assert cg.PERSON_BYPASS_AFTER_S >= 5.0
    assert decide(person(3.0, ct.STATIC), person_still_for_s=4.0) == cg.WAIT
    assert decide(person(3.0, ct.STATIC),
                  person_still_for_s=6.0) == cg.GO_ROUND


def test_someone_too_far_off_to_be_in_the_way_is_left_alone():
    """plan_ahead_m bounds it the way it bounds the parked rule: stepping
    around something that is not in the way yet is its own hazard."""
    assert decide(person(9.0, ct.STATIC), blocking=False,
                  person_still_for_s=60.0) == cg.CLEAR


def test_an_unconfirmed_person_is_never_gone_round_however_long():
    """UNKNOWN is what a track looks like before it has been watched long
    enough, and for a person the honest reading is someone about to step
    out. Standing in the way is not evidence about them."""
    assert decide(person(1.0, ct.UNKNOWN), person_still_for_s=60.0) == cg.WAIT


def test_a_person_who_is_moving_is_never_gone_round():
    assert decide(person(1.0, ct.MOVING), person_still_for_s=60.0) == cg.WAIT


def test_a_step_resets_the_clock():
    """avoidance_for clears person_still_since the moment the tracker stops
    calling them STATIC, so someone shifting their feet starts the five
    seconds again rather than accumulating toward a pass. Without a clock
    there is no pass on offer, only the wait."""
    assert decide(person(1.0, ct.STATIC), blocking=False,
                  person_still_for_s=None) == cg.WAIT


def test_the_chair_stops_and_watches_before_it_decides_anything():
    """It holds station for the five seconds instead of closing the
    distance, and it does so from outside the stopping radius.

    Waiting only once they are close enough to block is what left no room
    to go round them - and driving up to somebody and then swerving is not
    what anyone wants done around them either. So a confirmed-still person
    inside plan_ahead_m stops the chair whether or not they are blocking
    yet, and the pass starts from that stop.
    """
    assert decide(person(4.0, ct.STATIC), blocking=False) == cg.WAIT
    assert decide(person(4.0, ct.STATIC), blocked_for_s=0.0) == cg.WAIT


def test_the_thing_rule_does_not_reach_a_person():
    """BYPASS_AFTER_S would let an UNKNOWN through at three seconds. For a
    person that is the wrong evidence and the wrong threshold, and neither
    rule below is allowed to answer for one."""
    assert decide(person(1.0, ct.UNKNOWN), blocked_for_s=30.0) == cg.WAIT


def test_a_person_who_leaves_the_corridor_clears_it():
    """Nothing resumes the chair explicitly, here least of all. Out of the
    corridor there is no threat at all, which is the CLEAR above."""
    assert decide(None, blocking=False) == cg.CLEAR


def test_someone_walking_is_not_stopped_for_from_across_the_car_park():
    """The stop-and-watch is for a person standing in the way. Someone
    moving is handled by the blocking rule as before, so the chair does not
    halt for every pedestrian inside eight metres."""
    assert decide(person(6.0, ct.MOVING), blocking=False) == cg.CLEAR


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
    """A pedestrian crossing: blocked while they are in the corridor, clear
    the moment they are not. Nothing has to remember they were there."""
    assert decide(threat(1.5, ct.MOVING)) == cg.WAIT
    assert decide(None, blocking=False) == cg.CLEAR


# ------------------------------------------------- how big a person really is

def test_a_person_is_never_modelled_smaller_than_a_person():
    """3,803 person observations on 2026-08-25: 0.44 x 0.45 m on average,
    and a 5th-percentile width of 0.18 m. A lidar at armrest height catches
    a slice of a torso - the feet are under the band the clusterer keeps
    and the arms are outside whatever it did catch. Every clearance
    measured off that box is measured off the wrong body, which is how a
    0.80 m berth put the chair against somebody's side.
    """
    narrow = {"class": "person", "x": 2.0, "y": 0.0, "size": [0.18, 0.18, 1.7]}
    box = cg.object_box(narrow)
    assert box is not None
    assert box[2] >= cg.PERSON_MIN_HALF_EXTENT_M
    assert box[3] >= cg.PERSON_MIN_HALF_EXTENT_M


def test_a_producer_that_sees_more_than_that_is_believed():
    """It is a floor, not a size."""
    wide = {"class": "person", "x": 2.0, "y": 0.0, "size": [1.2, 1.4, 1.7]}
    box = cg.object_box(wide)
    assert box[2] == 0.6 and box[3] == 0.7


def test_nothing_else_is_inflated():
    """A thing is what it measures. Rounding every obstacle up to a person
    would close corridors that are known to be passable."""
    thing = {"class": "obstacle", "x": 2.0, "y": 0.0, "size": [0.18, 0.18, 0.5]}
    box = cg.object_box(thing)
    assert box[2] == 0.09 and box[3] == 0.09


def test_the_inflation_reaches_the_distance_as_well_as_the_shape():
    """object_box is the one place it happens, so the stopping distance and
    the planner's points cannot end up measuring different bodies."""
    narrow = {"class": "person", "x": 2.0, "y": 0.0, "size": [0.18, 0.18, 1.7],
              "points": 40, "motion": ct.STATIC}
    summary = cg.parse_summary(json.dumps(
        {"stamp": 100.0, "status": "OK", "objects": [narrow]}))
    near = cg.nearest_threat(summary, 1.0)
    assert near is not None
    assert near.distance_m <= 2.0 - cg.PERSON_MIN_HALF_EXTENT_M + 1e-9
