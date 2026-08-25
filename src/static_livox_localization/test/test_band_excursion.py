"""The drawn corridor is a cost. A measured kerb, and the mask, are not.

Until 2026-08-23 a rollout leaving the safety band was rejected whatever
else was true. That is right for an edge with a drop under it and wrong for
an edge that is only where the operator stopped drawing: a parked
motorcycle 0.77 m off the centreline could stand the chair up inside a
corridor open 2.70 m the other way.

Measured on route v9 after the change, over 200 stations, the extra room
the drivable mask actually allows outside the band is 0.05 m at the median
and 0.08 m on average - the mask, not the band, is what bounds this in
practice, and the mask is still absolute.
"""
import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

import dwa_core
from safety_band import BAND_EXCURSION_MAX_M, SafetyBand

SPACING = 0.5


def band_file(path, left=0.5, right=0.5, left_kind=None, right_kind=None,
              count=80):
    stations = []
    for k in range(count):
        s = {"x": k * SPACING, "y": 0.0, "heading_deg": 0.0,
             "left_m": left, "right_m": right,
             "left_drop_m": 0.0, "right_drop_m": 0.0}
        if left_kind is not None:
            s["left_kind"] = left_kind
        if right_kind is not None:
            s["right_kind"] = right_kind
        stations.append(s)
    json.dump({"frame": "map", "route_id": "test:excursion",
               "station_spacing_m": SPACING, "stations": stations},
              open(path, "w"))
    return SafetyBand(path)


def route(count=80):
    return np.array([[k * SPACING, 0.0] for k in range(count)])


def usable_left(band, index=10):
    """What the band actually allows, which is the drawn edge minus the
    clearance usable_limit keeps from it - not the number in the file."""
    return float(band.left[index])


def test_an_edge_with_no_measured_drop_may_be_crossed(tmp_path):
    band = band_file(str(tmp_path / "b.json"), left_kind="candidate_keepout",
                     right_kind="candidate_mask_clip")
    just_outside = np.array([[5.0, usable_left(band) + 0.10]])
    assert not band.contains_many(just_outside)[0]
    assert band.passable(just_outside[0], BAND_EXCURSION_MAX_M)


def test_a_measured_drop_is_still_a_wall(tmp_path):
    band = band_file(str(tmp_path / "b.json"), left_kind="drop",
                     right_kind="drop")
    beyond = float(band.left[10]) + 0.10
    _, hazard = band.excursion_many(np.array([[5.0, beyond]]))
    assert hazard[0], "a drop edge has to read as a hazard"
    assert not band.passable(np.array([5.0, beyond]), BAND_EXCURSION_MAX_M)


def test_a_band_with_no_depth_information_stays_absolute(tmp_path):
    """is_severe treats a missing measurement as severe, so an old band
    keeps exactly the behaviour it was validated with."""
    band = band_file(str(tmp_path / "b.json"))
    for station in json.load(open(str(tmp_path / "b.json")))["stations"][:1]:
        station.pop("left_kind", None)
    assert not band.passable(np.array([5.0, 5.0]), BAND_EXCURSION_MAX_M)


def test_the_allowance_is_bounded(tmp_path):
    band = band_file(str(tmp_path / "b.json"), left_kind="candidate_keepout",
                     right_kind="candidate_mask_clip")
    edge = usable_left(band)
    inside_allowance = np.array([5.0, edge + BAND_EXCURSION_MAX_M - 0.01])
    beyond = np.array([5.0, edge + BAND_EXCURSION_MAX_M + 0.01])
    assert band.passable(inside_allowance, BAND_EXCURSION_MAX_M)
    assert not band.passable(beyond, BAND_EXCURSION_MAX_M)


def test_leaving_costs_more_than_anything_else_the_scorer_wants(tmp_path):
    """One centimetre out has to outweigh every preference in the sum, or
    an arc would leave because it was marginally tidier out there."""
    everything_else = (dwa_core.W_PATH + dwa_core.W_HEADING
                       + dwa_core.W_PROGRESS + dwa_core.W_OBSTACLE
                       + dwa_core.W_STEER + dwa_core.W_CENTRE
                       + dwa_core.W_MASK_BOUNDARY + dwa_core.W_VELOCITY
                       + dwa_core.W_SPEED)
    assert dwa_core.W_OUTSIDE_BAND * 0.01 > everything_else


def planner_on(band, **kwargs):
    return dwa_core.DwaPlanner(band, route(), **kwargs)


def one_sided(band, index=10):
    """An object filling the corridor from the right edge to just past the
    centreline, the way a parked thing does.

    Sized against the band's own usable limits and OBSTACLE_FLOOR_M so that
    NO arc staying inside can clear it - the widest inside line is left with
    less clearance than the floor - while a step over the drawn edge has
    room. Without that the corridor keeps a gap and the test passes whether
    or not the band is a cost, which is what the first version of it did.
    """
    left_edge = float(band.left[index])
    # Close enough that the SCORED arc has to clear it, not the straight
    # continuation past its end - at arm's length the planner solves it by
    # aiming left inside the corridor and running on, which is a fine answer
    # and not the one under test.
    # Its far side sits on the centreline, so clearing it by
    # OBSTACLE_FLOOR_M needs 0.50 m of offset and the corridor allows
    # 0.425. Every inside arc is short by 0.075 m; a step over the drawn
    # edge is not. That gap is the whole subject of this file, and it is
    # the reason the numbers here are derived from the band rather than
    # written down: a corridor with any slack left in it passes whether or
    # not the change is present.
    far = 0.0
    return np.array([(x, y) for x in np.linspace(1.0, 1.6, 9)
                     for y in np.linspace(-float(band.right[index]), far, 11)])


def test_a_clear_corridor_is_still_driven_down_the_middle(tmp_path):
    band = band_file(str(tmp_path / "b.json"), left=1.0, right=1.0,
                     left_kind="candidate_keepout",
                     right_kind="candidate_mask_clip")
    planner = planner_on(band)
    v, w, status = planner.plan((0.0, 0.0, 0.0), obstacles=(),
                                speed_cap=dwa_core.MAX_SPEED, last_speed=0.5)
    assert status == "OK"
    path = dwa_core.rollout((0.0, 0.0, 0.0), v, w)
    excursion, _ = band.excursion_many(path[:, :2])
    assert float(excursion.max()) == 0.0, \
        "nothing was in the way, so nothing should have left the corridor"


def test_it_steps_outside_rather_than_standing_in_front_of_something(tmp_path):
    """The behaviour the change is for. A narrow corridor with an object
    filling it: every arc that stays inside is killed, and the answer used
    to be a stop."""
    band = band_file(str(tmp_path / "b.json"), left=0.5, right=0.5,
                     left_kind="candidate_keepout",
                     right_kind="candidate_mask_clip")
    planner = planner_on(band)
    v, w, status = planner.plan((0.0, 0.0, 0.0), obstacles=one_sided(band),
                                speed_cap=dwa_core.MAX_SPEED, last_speed=0.5)
    assert status == "OK", status
    assert w > 0.0, "the room is to the left; it should have gone left"
    path = dwa_core.rollout((0.0, 0.0, 0.0), v, w,
                            distance_m=planner.preview_distance(0.5))
    excursion, _ = band.excursion_many(path[:, :2])
    assert float(excursion.max()) > 0.0, \
        "it stayed inside, so this is not testing the excursion at all"


def test_what_it_steps_into_is_still_inside_the_allowance(tmp_path):
    band = band_file(str(tmp_path / "b.json"), left=0.5, right=0.5,
                     left_kind="candidate_keepout",
                     right_kind="candidate_mask_clip")
    planner = planner_on(band)
    v, w, _ = planner.plan((0.0, 0.0, 0.0), obstacles=one_sided(band),
                           speed_cap=dwa_core.MAX_SPEED, last_speed=0.5)
    path = dwa_core.rollout((0.0, 0.0, 0.0), v, w,
                            distance_m=planner.preview_distance(0.5))
    excursion, hazard = band.excursion_many(path[:, :2])
    assert float(excursion.max()) <= BAND_EXCURSION_MAX_M + 1e-9
    assert not hazard.any()


def test_it_will_not_step_over_a_drop_to_get_round_something(tmp_path):
    """The same corridor, the same object, edges that are measured drops.
    Standing still is the right answer there."""
    band = band_file(str(tmp_path / "b.json"), left=0.5, right=0.5,
                     left_kind="drop", right_kind="drop")
    planner = planner_on(band)
    _v, _w, status = planner.plan((0.0, 0.0, 0.0), obstacles=one_sided(band),
                                  speed_cap=dwa_core.MAX_SPEED,
                                  last_speed=0.5)
    assert status in ("OBSTACLE", "OFF_BAND"), status


class NarrowMask:
    """A drivable area exactly as wide as the corridor."""

    def contains_many(self, points):
        return np.abs(np.asarray(points)[:, 1]) <= 0.5

    def paths_are_contained(self, paths):
        return np.all(np.abs(paths[:, :, 1]) <= 0.5, axis=1)

    def boundary_cost_many(self, points):
        return np.zeros(len(points))


def test_the_drivable_mask_is_still_absolute(tmp_path):
    """The band became a cost; the mask did not. On route v9 it is the mask
    that binds - 0.05 m of extra room at the median - and that is the part
    that is physical.

    Asserted as containment of whatever it chose rather than as a refusal:
    turning away from the object is a legitimate answer, and the guarantee
    is about where the chair may be, not about it having to give up.
    """
    band = band_file(str(tmp_path / "b.json"), left=0.5, right=0.5,
                     left_kind="candidate_keepout",
                     right_kind="candidate_mask_clip")
    mask = NarrowMask()
    planner = planner_on(band, route_mask=mask)
    v, w, status = planner.plan((0.0, 0.0, 0.0), obstacles=one_sided(band),
                                speed_cap=dwa_core.MAX_SPEED, last_speed=0.5)
    if status != "OK":
        return
    path = dwa_core.rollout((0.0, 0.0, 0.0), v, w,
                            distance_m=planner.preview_distance(0.5))
    assert bool(mask.paths_are_contained(path[None, :, :2])[0]), \
        "the chosen arc left the drivable area"


@pytest.mark.parametrize("fraction", [0.0, 0.4, -0.4, 0.99, -0.99])
def test_inside_the_corridor_reports_no_excursion(tmp_path, fraction):
    band = band_file(str(tmp_path / "b.json"), left_kind="candidate_keepout",
                     right_kind="candidate_mask_clip")
    edge = usable_left(band) if fraction >= 0 else float(band.right[10])
    offset = fraction * edge if fraction >= 0 else fraction * edge
    excursion, hazard = band.excursion_many(np.array([[5.0, offset]]))
    assert float(excursion[0]) == 0.0
    assert not hazard[0]
