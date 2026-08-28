"""Going round the parked thing, waiting for the moving one - on the follower.

test_cluster_guard pins the policy in isolation. This runs it through the
node: the real corridor merge, the real offset search, and the real band
check that decides whether stepping aside is possible at all.

The band check is the part worth watching. Containment stopping the chair is
a judgement that ~safety_policies can switch off; the band knowing where
there is room to step aside is not, because the smallest offset on offer is
0.60 m and this route's measured median lateral clearance is 0.30 m. A build
that skipped it with the policies off would aim the chair at exactly what the
band exists to keep it off.
"""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from test_waypoint_follower_geometry import load_follower_module


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


class Band:
    """A band open on the chosen sides and closed on the others."""

    def __init__(self, open_offsets=(0.6, -0.6, 1.0, -1.0)):
        self.open_offsets = open_offsets

    def contains(self, point, grace=0.0):
        # The probe walks 0.5-3.5 m ahead at the offset; the chair here
        # faces +x, so the offset is the point's y.
        return any(abs(point[1] - o) < 1e-6 for o in self.open_offsets)

    def clamp(self, target):
        return target

    def lateral_limits(self, xy):
        # bypass_room_each_side asks the band how much room each side has.
        # The double answers with a corridor wide enough that the room test
        # never decides anything - these tests are about which side the
        # policy picks, not about how much band there is to pick it in. Room
        # has to clear BYPASS_OFFSET_MAX_M + BYPASS_EDGE_KEEP_M or the widest
        # rungs of the ladder disappear before the policy sees them.
        return 0.0, -3.0, 3.0


def follower_with(objects, open_offsets=(0.6, -0.6, 1.0, -1.0),
                  policies=False, status="OK"):
    module = load_follower_module()
    follower = module.WaypointFollower.__new__(module.WaypointFollower)
    follower.pose_xy = np.array([0.0, 0.0])
    follower.pose_yaw = 0.0
    follower.lateral_offset = 0.0
    follower.policies = policies
    follower.clusters_enabled = True
    follower.band = Band(open_offsets)
    follower.drivable_mask = type(
        "Mask", (), {"contains": staticmethod(lambda _point: True)}
    )()
    follower.cluster_summary = cg.parse_summary(json.dumps(
        {"stamp": 100.0, "status": status, "objects": objects}))
    return module, follower


def parked(x, y, size=(0.6, 0.6, 1.2)):
    return {"class": "obstacle", "x": x, "y": y, "size": list(size),
            "points": 40, "motion": ct.STATIC}


def walking(x, y, size=(0.6, 0.6, 1.7)):
    return {"class": "person", "x": x, "y": y, "size": list(size),
            "points": 40, "motion": ct.MOVING}


def test_a_parked_object_ahead_is_seen_at_its_near_face():
    _module, follower = follower_with([parked(4.0, 0.0)])
    threat = follower.corridor_threat()
    assert threat.distance_m == pytest.approx(3.7)
    assert threat.parked


# What the follower actually passes: max(guard_slow, PLAN_AHEAD_M). Tests
# that pass a short distance instead quietly accept a lane the object is
# still standing in, just beyond the braking distance, and then assert the
# wrong offset - which is how the first version of this file passed while
# the code stepped 0.6 m aside from something 0.6 m wide.
CLEAR_FOR_M = 5.0


def test_the_chair_steps_aside_for_something_parked():
    """0.6 m does not clear a 0.6 m object in a 0.45 m half-corridor: its
    flank still reaches 0.3 m past the offset centre line. The first offer
    that actually clears is 1.0 m, and that is what it has to take."""
    _module, follower = follower_with([parked(4.0, 0.0)])
    assert follower.take_a_way_round(CLEAR_FOR_M) is True
    assert follower.lateral_offset == 1.0


def test_a_narrow_object_is_cleared_by_the_small_offset():
    """A bollard is what the 0.6 m entries are for - the chair leaves the
    line by as little as the thing in the way requires."""
    _module, follower = follower_with([parked(4.0, 0.0, size=(0.2, 0.2, 1.0))])
    assert follower.take_a_way_round(CLEAR_FOR_M) is True
    assert follower.lateral_offset == 0.6


def test_it_takes_the_side_the_band_actually_leaves_open():
    """A kerb down one side is the normal case on this route."""
    _module, follower = follower_with([parked(4.0, 0.0)],
                                      open_offsets=(-1.0,))
    assert follower.take_a_way_round(CLEAR_FOR_M) is True
    assert follower.lateral_offset == -1.0


def test_with_no_room_in_the_band_it_waits_where_it_is():
    _module, follower = follower_with([parked(4.0, 0.0)], open_offsets=())
    assert follower.take_a_way_round(CLEAR_FOR_M) is False
    assert follower.lateral_offset == 0.0


def test_the_band_is_consulted_even_with_the_policies_off():
    """The property this whole file exists for. Same object, every side
    closed, policies off - and it must still refuse to step anywhere."""
    _module, follower = follower_with([parked(4.0, 0.0)],
                                      open_offsets=(), policies=False)
    assert follower.take_a_way_round(CLEAR_FOR_M) is False


def test_it_will_not_step_into_a_side_that_is_also_blocked():
    """A lane has to be clear over the planning distance, not merely as far
    as the chair could brake. A second object standing in the lane the chair
    was about to take sends it to the other side instead of into it."""
    _module, follower = follower_with([parked(4.0, 0.0), parked(4.0, 1.2)])
    assert follower.take_a_way_round(CLEAR_FOR_M) is True
    assert follower.lateral_offset == -1.0


def test_the_lane_check_reaches_the_planning_distance_not_the_brake():
    module = load_follower_module()
    text = (SCRIPTS / "waypoint_follower.py").read_text(encoding="utf-8")
    assert "self.take_a_way_round(max(guard_slow, PLAN_AHEAD_M))" in text
    assert module.PLAN_AHEAD_M > module.GUARD_STOP_MIN_M


def test_a_walking_person_is_gone_around():
    _module, follower = follower_with([walking(3.0, 0.0)])
    threat = follower.corridor_threat()
    assert not threat.parked
    assert cg.avoidance_decision(threat, True, 30.0, 5.0, 3.0) == cg.PERSON_BYPASS


def test_the_chair_moves_again_once_they_are_out_of_the_corridor():
    """No timer and nothing to reset: the person steps clear, the corridor
    has nothing in it, and the answer is CLEAR."""
    _module, follower = follower_with([walking(3.0, 1.4)])
    assert follower.corridor_threat() is None


def test_a_silent_producer_reads_as_blocked_not_as_clear():
    _module, follower = follower_with([])
    follower.cluster_summary = None
    threat = follower.corridor_threat()
    assert threat.distance_m == 0.0
    assert not threat.parked


def test_a_producer_that_cannot_see_reads_as_blocked():
    _module, follower = follower_with([], status="NO_CLOUD")
    threat = follower.corridor_threat()
    assert threat.distance_m == 0.0
    assert not threat.parked


def test_the_cluster_guard_survives_the_policies_being_switched_off():
    """The point of keeping it out from behind ~safety_policies: with them
    off it is the only source left, and it still reports."""
    _module, follower = follower_with([parked(4.0, 0.0)], policies=False)
    assert follower.corridor_threat() is not None
