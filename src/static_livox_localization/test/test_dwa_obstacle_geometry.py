"""What the DWA planner is told an obstacle IS, and where.

The planner's whole claim is that it avoids things by choosing an arc rather
than by stopping. That claim depends entirely on the obstacle arriving as
geometry: an object reduced to a distance and re-placed on the chair's own
heading axis is, to a rollout critic, a wall across the corridor no matter
which side of the corridor it is actually on.

On 2026-08-09 that is what happened at wp 905. A parked van whose nearest
corridor return sat half a metre to the RIGHT was handed over as a point
0.55 m dead ahead; no arc off this chair's 0.6 m minimum radius clears
0.40 m of a point that close, so every candidate was rejected while there
was 0.55 m of empty band on the left. The van was static, so the rejection
never expired - the run ended with a person taking over.

The recorded object is in here verbatim, and the first test is the one that
matters: with its own returns there is an admissible arc, and it turns left.
"""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
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
dwa = load("dwa_core")


# /perception/objects_summary, 2026-08-09 20:44, blackbox_20260809_191316.
# Reported class flickered between vehicle/crossing and outside_band as the
# band fraction hovered at 0.04-0.06; the profile did not move.
VAN = {
    "class": "vehicle", "raw_class": "vehicle", "band_relation": "crossing",
    "band_inside_fraction": 0.062, "x": 3.22, "y": 0.2,
    "size": [5.41, 3.3, 2.22], "motion": "static", "id": 13885,
    "profile": {"bin_m": 0.2, "y0": -1.6, "min_x": [
        1.83, 1.02, 0.86, 0.69, 0.51, 0.57, 0.75, 0.95, 0.99, 1.05,
        3.0, 3.65, 3.94, 4.03, 4.24, 4.26, 4.89, 5.02]},
}

CORRIDOR_HALF_WIDTH = 0.45
AT_THE_ORIGIN = (0.0, 0.0, 0.0)


def summary(objects, status="OK"):
    return cg.parse_summary(json.dumps(
        {"stamp": 100.0, "status": status, "objects": objects}))


def best_arc(obstacles, speed_cap=0.30):
    """The most clearance any candidate achieves, and which arc achieved it.

    Deliberately not DwaPlanner.plan: that needs a band and a route, and
    what is under test here is only whether an admissible arc EXISTS - the
    OBSTACLE rejection is the one that fired, and it fires before any of the
    scoring.
    """
    top = (0.0, None)
    for speed in dwa.speed_samples(speed_cap):
        if speed <= 0.0:
            continue
        for yaw_rate in dwa.yaw_samples():
            clearance = dwa.obstacle_clearance(
                dwa.rollout(AT_THE_ORIGIN, speed, yaw_rate), obstacles)
            if clearance > top[0]:
                top = (clearance, (speed, yaw_rate))
    return top


def test_the_van_that_deadlocked_wp905_has_an_admissible_arc():
    points = cg.corridor_obstacle_points(
        summary([VAN]), CORRIDOR_HALF_WIDTH, max_distance_m=5.0)
    clearance, arc = best_arc(points)
    assert clearance >= dwa.OBSTACLE_FLOOR_M
    # and it is a LEFT turn, which is the side the band had room on and the
    # side the chair was already asking for when it was rejected
    assert arc[1] > 0.0


def test_the_same_van_as_a_point_on_the_heading_axis_rejects_everything():
    """The model this replaced, kept so the regression cannot come back."""
    _blocks, distance, _motion = cg.corridor_reach(
        VAN, 0.0, CORRIDOR_HALF_WIDTH)
    clearance, _arc = best_arc([(distance, 0.0)])
    assert clearance < dwa.OBSTACLE_FLOOR_M


def test_the_distance_is_the_one_the_guard_already_reported():
    """Geometry replaces the bearing, not the measurement.

    The nearest point of the returned set, over the slices that reach the
    corridor, is still what corridor_reach says - if these two disagreed,
    one of the stop radius and the planner would be avoiding a different
    object.

    Reaching the corridor means OVERLAPPING it, which is how profile_reach
    selects: a slice centred at 0.5 m with a 0.2 m bin still has returns at
    0.4 m. Comparing slice centres against the half width instead would
    silently drop the very slice this van was measured at.
    """
    _blocks, distance, _motion = cg.corridor_reach(
        VAN, 0.0, CORRIDOR_HALF_WIDTH)
    points = cg.corridor_obstacle_points(summary([VAN]), CORRIDOR_HALF_WIDTH)
    reach = CORRIDOR_HALF_WIDTH + VAN["profile"]["bin_m"] / 2.0
    assert min(x for x, y in points if abs(y) < reach) ==         pytest.approx(distance)


def test_a_profile_keeps_which_side_each_return_was_on():
    points = cg.corridor_obstacle_points(summary([VAN]), CORRIDOR_HALF_WIDTH)
    assert len(points) == len(VAN["profile"]["min_x"])
    # slice index i covers [y0 + i * bin, y0 + (i + 1) * bin), placed at its
    # centre - the same arithmetic profile_reach selects slices with
    assert points[0] == pytest.approx((1.83, -1.5))
    assert points[4] == pytest.approx((0.51, -0.7))
    assert points[-1] == pytest.approx((5.02, 1.9))


def test_the_full_extent_is_passed_not_only_the_corridor_slices():
    """Going round something means clearing the part that sticks out."""
    points = cg.corridor_obstacle_points(summary([VAN]), CORRIDOR_HALF_WIDTH)
    assert any(y < -CORRIDOR_HALF_WIDTH for _x, y in points)


def test_a_wall_alongside_is_still_not_an_obstacle():
    """Which objects count is unchanged: only what corridor_reach blocks.

    Without this the planner acquires a new way to stop - every kerb and
    wall this route drives within half a metre of would be inside the
    0.40 m floor - and the corridor test that used to ignore them would
    have no say in it.
    """
    wall = {"x": 2.0, "y": 1.4, "size": [4.0, 0.4, 1.0], "motion": "static",
            "profile": {"bin_m": 0.2, "y0": 1.2, "min_x": [0.6, 0.6, 0.6]}}
    assert cg.corridor_obstacle_points(
        summary([wall]), CORRIDOR_HALF_WIDTH) == []


def test_something_further_off_than_the_planner_looks_is_left_out():
    far = {"x": 9.0, "y": 0.0, "size": [1.0, 1.0, 1.0], "motion": "static",
           "profile": {"bin_m": 0.2, "y0": -0.2, "min_x": [8.5, 8.5]}}
    assert cg.corridor_obstacle_points(
        summary([far]), CORRIDOR_HALF_WIDTH, max_distance_m=5.0) == []
    assert cg.corridor_obstacle_points(
        summary([far]), CORRIDOR_HALF_WIDTH) != []


def test_an_object_with_no_profile_falls_back_to_its_near_face():
    box = {"x": 2.0, "y": 0.0, "size": [1.0, 0.8, 1.0], "motion": "static"}
    points = cg.corridor_obstacle_points(summary([box]), CORRIDOR_HALF_WIDTH)
    assert sorted(points) == [(1.5, -0.4), (1.5, 0.0), (1.5, 0.4)]


@pytest.mark.parametrize("broken", [
    {"bin_m": 0.0, "y0": 0.0, "min_x": [1.0]},
    {"bin_m": 0.2, "y0": float("nan"), "min_x": [1.0]},
    {"bin_m": 0.2, "y0": 0.0, "min_x": [float("nan")]},
    {"bin_m": 0.2, "y0": 0.0, "min_x": ["near"]},
    {"bin_m": 0.2, "y0": 0.0, "min_x": []},
    {"bin_m": 0.2, "y0": 0.0, "min_x": [None]},
    "not a profile at all",
])
def test_an_unreadable_profile_blocks_at_the_chair(broken):
    """Fails closed the same way corridor_reach does, and for the reason.

    A point at the chair itself is inside every rollout, so every candidate
    is rejected. Skipping the object instead would let a producer bug read
    as clear road.
    """
    item = {"x": 2.0, "y": 0.0, "size": [1.0, 0.8, 1.0], "motion": "static",
            "profile": broken}
    assert cg.object_points(item) == [(0.0, 0.0)]


def test_an_unusable_summary_blocks_at_the_chair():
    for absent in (None, summary([VAN], status="STALE")):
        assert cg.corridor_obstacle_points(
            absent, CORRIDOR_HALF_WIDTH) == [(0.0, 0.0)]


def test_the_follower_puts_the_points_where_the_chair_is_pointing():
    """The rollouts live in the map frame, so the returns have to get there.

    Chair frame is x forward, y left; the summary is published in it and the
    band is not. A sign error here is a planner that avoids the mirror image
    of the obstacle, which no test of the geometry alone would catch.
    """
    from test_waypoint_follower_geometry import load_script_module

    module = load_script_module("dwa_follower", "dwa_follower_geometry_test")
    follower = module.DwaFollower.__new__(module.DwaFollower)
    follower.clusters_enabled = True
    follower.cluster_summary = summary([VAN])

    facing_east = follower.obstacle_points(np.array([10.0, 5.0, 0.0]))
    # the van's nearest corridor return is 0.57 m ahead and 0.5 m to the
    # right, which facing east is -y in the map
    assert np.min(np.linalg.norm(
        facing_east - np.array([10.57, 4.5]), axis=1)) == pytest.approx(0.0)

    facing_north = follower.obstacle_points(
        np.array([10.0, 5.0, math.pi / 2.0]))
    # a quarter turn left: forward is +y, and the chair's right is +x
    assert np.min(np.linalg.norm(
        facing_north - np.array([10.5, 5.57]), axis=1)) == pytest.approx(0.0)


def test_the_follower_asks_for_nothing_when_the_cluster_source_is_off():
    """Not the same question as an empty corridor.

    With clusters disabled there is no obstacle source at all, and the
    constructor refuses to start that way; the failsafe lives in the hold
    ladder, not here. Returning a blocking point instead would mean the
    profile could never be driven with clusters off even for a bench test.
    """
    from test_waypoint_follower_geometry import load_script_module

    module = load_script_module("dwa_follower", "dwa_follower_geometry_test")
    follower = module.DwaFollower.__new__(module.DwaFollower)
    follower.clusters_enabled = False
    follower.cluster_summary = None
    assert follower.obstacle_points(np.array([0.0, 0.0, 0.0])) == ()
