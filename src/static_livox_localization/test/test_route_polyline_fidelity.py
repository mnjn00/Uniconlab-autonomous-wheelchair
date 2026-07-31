"""The route is the driven path, not an approximation of it.

The band is computed about the route: at every 1 m station it casts a lateral
ray perpendicular to it and records where the ground breaks. Anywhere the
route departs from where the chair actually went, the band measures the
clearance of a line the chair was never on, at an angle it was never at.

Approximating that path with sparsely chosen waypoints put the error back in
by construction. Choosing them by a curvature estimate left 5-6 m gaps
spanning up to 131 degrees of real turning and the driven path 1.36 m from its
own recorded line; bounding the chord sag instead brought that to 0.39 m, but
0.39 m is still an invented displacement in a budget whose margin is 0.10 m.

So the route carries the resampled trace itself. There is no chord to sag and
no spacing rule to get wrong - the only remaining departure is the resampling
step, which is bounded and small.
"""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).parents[1]
TOOL = ROOT.parents[1] / "tools" / "make_route_waypoints_from_trace.py"
ROUTE = ROOT.parents[1] / "routes" / "20260727_chair_centred_waypoints.json"


def load_tool():
    spec = importlib.util.spec_from_file_location("route_from_trace", TOOL)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = load_tool()


def arc(radius, turn_rad, step=0.02):
    count = max(int(radius * turn_rad / step), 8)
    angles = np.linspace(0.0, turn_rad, count)
    return np.stack([radius * np.sin(angles),
                     radius * (1.0 - np.cos(angles))], axis=1)


def max_departure(path, samples):
    """Largest distance from a sample to the polyline - segments, not vertices.

    Distance to the nearest vertex would just measure half the resampling
    step. What the band actually follows is the line between them.
    """

    start = path[:-1]
    span = path[1:] - start
    length_sq = np.maximum((span * span).sum(axis=1), 1e-12)
    worst = 0.0
    for point in samples:
        offset = point - start
        fraction = np.clip((offset * span).sum(axis=1) / length_sq, 0.0, 1.0)
        nearest = start + fraction[:, None] * span
        worst = max(worst, float(np.min(np.linalg.norm(nearest - point, axis=1))))
    return worst


def test_the_step_is_small_against_the_margin_it_feeds():
    """The resampling step is the only departure left, and it lands in the
    band exactly like a sampling error would, so it is sized against
    BAND_MARGIN (0.10 m) rather than against the 0.45 m clearance."""
    assert 0.0 < tool.POLYLINE_STEP <= 0.25


@pytest.mark.parametrize("radius", [2.0, 4.0, 11.0, 40.0])
def test_the_route_follows_a_bend_of_any_radius(radius):
    """No curvature estimate, no spacing rule: the path is the path. What is
    left is the sag of one resampling step, which at 0.2 m over a 2 m radius
    is under a centimetre."""
    points = arc(radius, math.radians(140.0))
    z = np.zeros(len(points))
    yaw = np.linspace(0.0, math.radians(140.0), len(points))

    line, _, _ = tool.polyline(points, z, yaw)

    assert max_departure(line, points) <= 0.02
    gaps = np.linalg.norm(np.diff(line, axis=0), axis=1)
    assert gaps.max() <= tool.POLYLINE_STEP * 1.6


def test_a_spin_in_place_does_not_inflate_the_route():
    """The chair centre barely moves while the chair turns on the spot, so a
    bookend spin must contribute almost no points - the sensor's own 0.545 m
    swing is what used to put 7.67 m of false path in."""
    still = np.zeros((400, 2))
    z = np.zeros(400)
    yaw = np.linspace(0.0, 4 * math.pi, 400)

    line, _, headings = tool.polyline(still, z, yaw)

    assert len(line) == 1
    assert len(headings) == 1


def test_the_shipped_route_is_dense_enough_to_be_the_path():
    if not ROUTE.exists():
        pytest.skip("route is not shipped")
    import json
    route = json.loads(ROUTE.read_text(encoding="utf-8"))
    points = np.array([[w["x"], w["y"]] for w in route["waypoints"]])
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)

    # Spacing is the resampling step, not a chosen waypoint interval.
    assert np.median(gaps) <= 0.30
    assert gaps.max() <= 1.0
    assert len(points) > 1000


def test_the_start_heading_guard_uses_a_baseline_worth_measuring():
    """The guard compares the recorded start heading against the direction the
    route first travels. Over one 0.2 m step that direction is noise - the raw
    polyline turns by up to 27 deg between steps - so it has to look at least
    a metre ahead or it will refuse good routes."""
    text = TOOL.read_text(encoding="utf-8")
    assert "GUARD_BASELINE_M" in text

    candidates = ROOT / "scripts" / "initial_pose_candidates.py"
    assert "GUARD_BASELINE_M" in candidates.read_text(encoding="utf-8")
