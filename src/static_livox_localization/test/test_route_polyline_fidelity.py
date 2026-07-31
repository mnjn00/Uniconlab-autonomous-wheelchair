"""Waypoints have to describe the path that was actually driven.

The band is not computed about the trace. It is computed about the polyline
through the waypoints: at every 1 m station it casts a lateral ray
perpendicular to that polyline and records where the ground breaks. So
wherever the polyline departs from the driven path, the band measures the
clearance of a line the chair was never on, at an angle the chair was never
at.

Spacing used to be chosen from a curvature estimate, which does not bound
that departure. Measured on the 0727 route, 14 emitted gaps of 5-6 m each
spanned 20 to 131 degrees of real turning, and the driven path came out up to
1.49 m from its own recorded line. Ten percent of stations then refused the
line the chair demonstrably drove, concentrated where the route bends.

Bounding the sag - the largest perpendicular distance from the chord to the
arc it replaces - fixes the property directly instead of approximating it.
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).parents[1]
TOOL = ROOT.parents[1] / "tools" / "make_route_waypoints_from_trace.py"


def load_tool():
    spec = importlib.util.spec_from_file_location("route_from_trace", TOOL)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = load_tool()


def sag_of(points, a, b):
    """Largest perpendicular distance from chord a->b to the points between."""

    start, end = points[a], points[b]
    span = end - start
    length = math.hypot(span[0], span[1])
    if length < 1e-9:
        return 0.0
    worst = 0.0
    for index in range(a + 1, b):
        offset = points[index] - start
        cross = abs(offset[0] * span[1] - offset[1] * span[0]) / length
        worst = max(worst, cross)
    return worst


def arc(radius, turn_rad, step=0.05):
    """A circular arc sampled finely, as a driven path would be."""

    count = max(int(radius * turn_rad / step), 8)
    angles = np.linspace(0.0, turn_rad, count)
    return np.stack([radius * np.sin(angles),
                     radius * (1.0 - np.cos(angles))], axis=1)


def test_the_tolerance_is_small_against_the_clearance_it_feeds():
    """The sag is an error in where the band thinks the chair was. It has to
    stay well inside BAND_MARGIN (0.10 m), not merely inside the 0.45 m
    clearance."""
    assert 0.0 < tool.MAX_SAG <= 0.10


@pytest.mark.parametrize("radius", [2.5, 4.0, 11.0, 40.0])
def test_a_bend_is_sampled_closely_enough_to_describe_itself(radius):
    """Every gap the picker emits must hold the arc it replaces to the
    tolerance - at any radius, without a curvature estimate in between."""
    points = arc(radius, math.radians(110.0))

    picked = tool.pick_waypoints(points)

    assert picked[0] == 0 and picked[-1] == len(points) - 1
    for a, b in zip(picked, picked[1:]):
        assert sag_of(points, a, b) <= tool.MAX_SAG + 1e-6, (
            "gap %d-%d departs the path by %.3f m" % (a, b, sag_of(points, a, b)))


def test_a_straight_is_not_over_sampled():
    """Sag is zero on a straight, so spacing there is bounded only by
    MAX_SPACING; a route dense everywhere would be as wrong as one coarse
    everywhere, just in the other direction."""
    points = np.stack([np.linspace(0.0, 60.0, 1200), np.zeros(1200)], axis=1)

    picked = tool.pick_waypoints(points)

    gaps = [np.linalg.norm(points[b] - points[a])
            for a, b in zip(picked, picked[1:])]
    assert max(gaps) <= tool.MAX_SPACING + 1e-6
    assert len(picked) <= 60.0 / (tool.MAX_SPACING - 1.0)


def test_a_sharper_bend_gets_more_waypoints_than_a_gentle_one():
    tight = tool.pick_waypoints(arc(2.5, math.radians(110.0)))
    gentle = tool.pick_waypoints(arc(40.0, math.radians(110.0)))
    tight_len = 2.5 * math.radians(110.0)
    gentle_len = 40.0 * math.radians(110.0)

    assert len(tight) / tight_len > len(gentle) / gentle_len


def test_the_ends_of_the_trace_are_always_kept():
    """The first waypoint is the auto-init seed and the last is where the
    route stops; neither may be dropped by a spacing rule."""
    points = arc(6.0, math.radians(200.0))

    picked = tool.pick_waypoints(points)

    assert picked[0] == 0
    assert picked[-1] == len(points) - 1


def test_a_two_point_path_is_handled():
    points = np.array([[0.0, 0.0], [1.0, 0.0]])

    assert tool.pick_waypoints(points) == [0, 1]
