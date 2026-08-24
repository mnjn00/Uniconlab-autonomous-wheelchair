"""The planner must see as far as the gate can veto.

Reproduces the 2026-08-23 deadlock at station 577 of route v9. A parked
motorcycle stood 1.42 m ahead and 0.77 m to the right of the centreline,
inside a corridor open 2.70 m to the left. The chair stood in front of it
for 130 s: the planner previewed 1.05 m, scored every candidate clear,
commanded straight ahead at the floor speed, and safety_gate refused the
command. Refusing does not move the chair, so nothing about the next cycle
was different.
"""
import json
import os
import sys
import tempfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

import dwa_core
import safety_gate
from safety_band import SafetyBand


STATION_SPACING_M = 0.5
# Station 577 of 20260816_route_v9_clearance_safety_band.json, where it
# happened. The corridor is not the constraint and the test says so.
LEFT_M = 2.70
RIGHT_M = 0.70

# /perception/objects_summary at t+2513 of blackbox_20260823_200204:
# x 1.42, y -0.77, size 0.83 x 0.60, band_relation inside, inside 1.00.
MOTORCYCLE_X = 1.42
MOTORCYCLE_Y = -0.77
MOTORCYCLE_LENGTH = 0.83
MOTORCYCLE_WIDTH = 0.60


def straight_band(path, count=60):
    stations = [{"x": k * STATION_SPACING_M, "y": 0.0, "heading_deg": 0.0,
                 "left_m": LEFT_M, "right_m": RIGHT_M,
                 "left_drop_m": 0.0, "right_drop_m": 0.0}
                for k in range(count)]
    json.dump({"frame": "map", "route_id": "test:straight",
               "station_spacing_m": STATION_SPACING_M,
               "stations": stations}, open(path, "w"))
    return SafetyBand(path)


def straight_route(count=60):
    return np.array([[k * STATION_SPACING_M, 0.0] for k in range(count)])


@pytest.fixture
def planner(tmp_path):
    band = straight_band(str(tmp_path / "band.json"))
    return dwa_core.DwaPlanner(band, straight_route())


def motorcycle_points():
    """Lidar returns on the surfaces of it that face the chair."""
    xs = np.linspace(MOTORCYCLE_X - MOTORCYCLE_LENGTH / 2.0,
                     MOTORCYCLE_X + MOTORCYCLE_LENGTH / 2.0, 9)
    ys = np.linspace(MOTORCYCLE_Y - MOTORCYCLE_WIDTH / 2.0,
                     MOTORCYCLE_Y + MOTORCYCLE_WIDTH / 2.0, 7)
    return np.array([(x, y) for x in xs for y in ys])


def test_the_motorcycle_stood_beyond_the_preview_that_was_shipped():
    """The measurement the deadlock rests on, kept as an assertion.

    Not a tautology: if the preview ever grows past the obstacle on its
    own, the rest of this file is testing a situation that cannot arise
    and should be re-derived rather than trusted.
    """
    floor_speed = dwa_core.TURN_FLOOR_SPEED
    reach = max(dwa_core.SIM_DISTANCE_M,
                floor_speed * dwa_core.SIM_MIN_PREVIEW_S)
    assert reach < MOTORCYCLE_X, (
        "the scored arc reaches %.2f m and the motorcycle stood at %.2f m"
        % (reach, MOTORCYCLE_X))


def test_the_veto_horizon_reaches_past_what_the_gate_stops_for():
    """The gate's envelope carries a fixed 0.9 m margin, so it exceeds
    1.4 m even at the floor speed. Anything the gate can stop for, the
    planner has to have scored."""
    assert dwa_core.OBSTACLE_PREVIEW_M > MOTORCYCLE_X
    assert dwa_core.OBSTACLE_PREVIEW_M >= (
        safety_gate.GEOMETRY_MARGIN_M + safety_gate.FORWARD_CHECK_EXTRA_M)


def test_the_planner_may_not_propose_what_the_gate_hard_stops():
    """The second entrance to the same deadlock.

    safety_gate stops for any obstacle point inside a 0.50 m forward
    corridor. A planner clearance floor below that admits paths the gate
    refuses, and the chair stands still while both are behaving correctly.
    """
    assert dwa_core.OBSTACLE_FLOOR_M >= safety_gate.HALF_WIDTH_M


def test_it_goes_round_the_motorcycle_instead_of_standing_in_front_of_it(
        planner):
    v, w, status = planner.plan((0.0, 0.0, 0.0), obstacles=motorcycle_points(),
                                speed_cap=dwa_core.MAX_SPEED, last_speed=0.35)
    assert status == "OK", status
    assert v > 0.0
    assert w > 0.0, (
        "the corridor is open 2.70 m to the left and the motorcycle sits "
        "0.77 m to the right; a straight command is the deadlock")


def test_the_arc_it_picks_actually_clears_the_motorcycle(planner):
    v, w, _ = planner.plan((0.0, 0.0, 0.0), obstacles=motorcycle_points(),
                           speed_cap=dwa_core.MAX_SPEED, last_speed=0.35)
    path = dwa_core.rollout((0.0, 0.0, 0.0), v, w,
                            distance_m=dwa_core.OBSTACLE_PREVIEW_M)
    gap = dwa_core.obstacle_clearance(path, motorcycle_points())
    assert gap >= dwa_core.OBSTACLE_FLOOR_M, gap


def test_an_empty_corridor_is_still_driven_straight(planner):
    v, w, status = planner.plan((0.0, 0.0, 0.0), obstacles=(),
                                speed_cap=dwa_core.MAX_SPEED, last_speed=0.35)
    assert status == "OK"
    assert abs(w) < 0.1, "nothing to avoid, so it should hold the line"


def test_the_extension_is_straight_and_does_not_corkscrew(planner):
    """Holding a sampled yaw rate out to 3 m would be 8.6 s of it at the
    floor speed - 4 rad for a 0.5 rad/s candidate. The chair replans ten
    times a second and never drives that, so the extension straightens."""
    pairs = [(0.35, 0.5)]
    span = dwa_core.SIM_DISTANCE_M
    short = planner._rollouts(np.array([0.0, 0.0, 0.0]), pairs, span)
    grown = planner._obstacle_paths(short, span, dwa_core.OBSTACLE_PREVIEW_M)
    assert grown.shape[1] > short.shape[1]
    np.testing.assert_allclose(grown[0, :short.shape[1]], short[0])
    tail_yaw = grown[0, short.shape[1]:, 2]
    np.testing.assert_allclose(tail_yaw, short[0, -1, 2])


def test_the_extension_reaches_the_horizon_it_was_asked_for(planner):
    pairs = [(0.35, 0.0)]
    span = dwa_core.SIM_DISTANCE_M
    short = planner._rollouts(np.array([0.0, 0.0, 0.0]), pairs, span)
    grown = planner._obstacle_paths(short, span, dwa_core.OBSTACLE_PREVIEW_M)
    assert grown[0, -1, 0] >= dwa_core.OBSTACLE_PREVIEW_M - 1e-6


def test_a_horizon_inside_the_arc_leaves_the_arc_alone(planner):
    pairs = [(0.35, 0.2)]
    span = dwa_core.SIM_DISTANCE_M
    short = planner._rollouts(np.array([0.0, 0.0, 0.0]), pairs, span)
    same = planner._obstacle_paths(short, span, span * 0.5)
    assert same is short


def test_containment_still_uses_the_short_arc_only(planner):
    """The arc is short on purpose: at 1.7 m the admissible candidate
    count fell from 102 to 78 at wp 500. Lengthening the veto horizon must
    not quietly lengthen the arc the corridor test sees."""
    pairs = [(v, w) for v in dwa_core.speed_samples(dwa_core.MAX_SPEED,
                                                    current=0.35)
             if v > 0.0 for w in dwa_core.yaw_samples()]
    span = planner.preview_distance(0.35)
    paths = planner._rollouts(np.array([0.0, 0.0, 0.0]), pairs, span)
    assert paths.shape[1] == planner.steps
    assert span <= max(dwa_core.SIM_DISTANCE_M,
                       0.35 * dwa_core.SIM_MIN_PREVIEW_S) + 1e-9


def test_the_clearance_search_does_not_scale_with_the_crowd(planner):
    """A control loop with 100 ms to spend cannot pay per obstacle point.

    Asserted as a ratio rather than a millisecond bound so it means the
    same on any machine: what it catches is the shape of the search, not
    the speed of the host. Pairwise distances over candidates x steps x
    points measured 371 ms at 2000 points and 854 ms at 5000 on the NUC,
    against 36 ms flat for the nearest-neighbour search that replaced it.
    A regression here is a stall in the field, and the profile stops
    before the planner is even asked - so it would not look like this.
    """
    import time

    rng = np.random.default_rng(0)

    def elapsed(count):
        """Best of several runs, not the mean.

        A mean measures the machine's other work as much as this code:
        run inside the full suite it went over budget on scheduler noise
        while passing every time on its own. The fastest run is the one
        least contaminated, and an algorithmic regression cannot hide in
        it - pairwise distances at 100x the points were 20x the time.
        """
        points = rng.uniform(-5.0, 5.0, (count, 2)) + np.array([4.0, 0.0])
        best = float("inf")
        for _ in range(5):
            start = time.perf_counter()
            planner.plan((0.0, 0.0, 0.0), obstacles=points,
                         speed_cap=dwa_core.MAX_SPEED, last_speed=0.6)
            best = min(best, time.perf_counter() - start)
        return best

    small = elapsed(200)
    large = elapsed(20000)
    assert large < small * 3.0, (
        "100x the points cost %.1fx the time (%.0f ms against %.0f ms)"
        % (large / max(small, 1e-9), large * 1e3, small * 1e3))
