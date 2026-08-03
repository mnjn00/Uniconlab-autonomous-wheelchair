"""Measuring an obstacle from its returns instead of from its bounding box.

The defect these exist for is recorded in blackbox/manifest.json and was
replayed out of blackbox_20260731_212959: at waypoint 349 the guard reported

    {"class": "vehicle", "x": 3.68, "y": 0.18, "size": [5.97, 2.36, 2.23],
     "points": 1864, "motion": "static"}

a wall crossing the scan diagonally. The box's near face is
3.68 - 5.97/2 = 0.695 m, which is a corner of the bounding volume and not a
place the wall has any returns; its nearest return inside the corridor was
2.13 m. Worse, half_y = 1.18 m meant clearing the box needed a 1.63 m
sidestep when the largest offset the follower offers is 1.0 m, so the chair
decided GO_ROUND, found no lane, and held for 16 minutes on something that
by its own verdict was never going to move.

The reconstruction below is that wall: a straight diagonal through the
recorded box, which enters the +/-0.45 m corridor at x = 2.09 m and so
reproduces the 2.13 m the field notes recorded.
"""

import importlib.util
import json
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
ct = load("cluster_tracking")

CORRIDOR_HALF_WIDTH = 0.45          # waypoint_follower
BYPASS_OFFSETS = (0.6, -0.6, 1.0, -1.0)
CLEAR_FOR_M = 5.0                   # max(guard_slow, PLAN_AHEAD_M)

# obstacle_clusters imports rospy, which is not installed on the workstation.
# The profiler itself is pure numpy, so the real definition is lifted out of
# the real file by name - not copied here, and not read by line offsets that
# would silently pick up the wrong code if the file is reordered.
def lift(module_path, names):
    import ast
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    wanted = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            wanted.append(node)
        elif isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id in names
                for t in node.targets):
            wanted.append(node)
    namespace = {"np": np, "math": __import__("math")}
    exec(compile(ast.Module(body=wanted, type_ignores=[]),
                 str(module_path), "exec"), namespace)
    missing = set(names) - set(namespace)
    assert not missing, "obstacle_clusters no longer defines %s" % sorted(missing)
    return namespace


_producer = lift(SCRIPTS / "obstacle_clusters.py",
                 {"PROFILE_BIN_M", "MAX_PROFILE_BINS", "lateral_profile"})
lateral_profile = _producer["lateral_profile"]


def wall_returns(step=0.02):
    """The 2026-07-31 wall: a diagonal spanning the recorded box."""
    x = np.arange(0.695, 6.665 + step, step)
    y = -1.0 + (x - 0.695) * (2.36 / 5.97)
    z = np.full_like(x, 0.9)
    return np.column_stack([x, y, z])


def summary(objects):
    return cg.parse_summary(json.dumps(
        {"stamp": 100.0, "status": "OK", "objects": objects}))


def wall_object(with_profile=True):
    points = wall_returns()
    lo, hi = points.min(axis=0), points.max(axis=0)
    item = {"class": "vehicle", "x": round(float((lo[0] + hi[0]) / 2), 2),
            "y": round(float((lo[1] + hi[1]) / 2), 2),
            "size": [round(float(v), 2) for v in (hi - lo)],
            "points": len(points), "motion": ct.STATIC}
    if with_profile:
        item["profile"] = lateral_profile(points)
    return item


# ------------------------------------------------------- the reconstruction

def test_the_reconstruction_reproduces_the_recorded_box():
    """Otherwise the rest of this file is testing a different wall."""
    item = wall_object(with_profile=False)
    assert item["x"] == pytest.approx(3.68, abs=0.02)
    assert item["y"] == pytest.approx(0.18, abs=0.02)
    assert item["size"][0] == pytest.approx(5.97, abs=0.02)
    assert item["size"][1] == pytest.approx(2.36, abs=0.02)


def test_the_box_alone_reports_the_defect():
    """The behaviour as shipped on 2026-07-31, kept as the thing being fixed."""
    threat = cg.nearest_threat(summary([wall_object(with_profile=False)]),
                               CORRIDOR_HALF_WIDTH)
    assert threat is not None
    assert threat.distance_m == pytest.approx(0.695, abs=0.02)


def test_the_profile_reports_where_the_wall_actually_is():
    """The wall crosses the corridor edge at x = 2.09 m.

    The answer here is 1.72, not 2.09, and deliberately so: the slice
    spanning [-0.6, -0.4) straddles the corridor edge, so it counts, and it
    contributes its own nearest return at y = -0.6. Slice-overlap binning
    always errs toward the chair. On a shallow diagonal that costs the bin
    width times the wall's run - 0.37 m here - and it is still two and a
    half times the distance the box claimed.
    """
    threat = cg.nearest_threat(summary([wall_object()]), CORRIDOR_HALF_WIDTH)
    assert threat is not None
    assert threat.distance_m == pytest.approx(1.72, abs=0.1)
    assert threat.distance_m < 2.09, "binning must never read past the truth"
    assert threat.distance_m > 2 * 0.695, "the box answer must not survive"


# waypoint_follower.GUARD_STOP_MIN_M: the floor on the radius inside which
# anything reported is a full stop. It is a floor, so a chair already at rest
# is using exactly this.
GUARD_STOP_MIN_M = 0.9


def test_the_deadlock_is_gone():
    """Why the chair sat for 16 minutes, and why it no longer would.

    The hold is not really about bypass lanes - a wall that genuinely spans
    the corridor should not be sidestepped, and this one does span it. It is
    about the stop radius. The box put the wall at 0.695 m, inside the 0.9 m
    floor, so the chair stopped; stopped, its stopping envelope stays at that
    floor, so the phantom near face stayed inside it and nothing about the
    situation could ever change. Static object, permanent stop.

    Measured from the returns the wall is 1.72 m away, outside the stop
    radius and inside the slow radius, so the chair creeps instead - and the
    0727 route curves past this wall, which is what it was always going to
    do once it was allowed to keep moving.
    """
    boxed = cg.nearest_threat(summary([wall_object(with_profile=False)]),
                              CORRIDOR_HALF_WIDTH)
    profiled = cg.nearest_threat(summary([wall_object()]),
                                 CORRIDOR_HALF_WIDTH)
    assert boxed.distance_m < GUARD_STOP_MIN_M, "the recorded defect"
    assert profiled.distance_m > GUARD_STOP_MIN_M, \
        "the fix: no longer a full stop"
    # and it is still seen, still static, still worth slowing for
    assert profiled.parked
    assert profiled.distance_m < GUARD_STOP_MIN_M + 1.2, "still inside slow"


def test_a_lane_opens_where_one_really_exists():
    """The bypass path is not broken by measuring properly - a parked object
    beside the line still leaves the lane past it readable as clear."""
    bollard = {"class": "obstacle", "x": 3.0, "y": 0.0, "motion": ct.STATIC,
               "size": [0.4, 0.4, 1.0],
               "profile": {"bin_m": 0.2, "y0": -0.2, "min_x": [3.0, 3.0]}}
    s = summary([bollard])
    assert cg.nearest_threat(s, CORRIDOR_HALF_WIDTH) is not None
    lanes = [o for o in BYPASS_OFFSETS
             if (lambda t: t is None or t.distance_m > CLEAR_FOR_M)(
                 cg.nearest_threat(s, CORRIDOR_HALF_WIDTH, o))]
    assert lanes, "a 0.4 m bollard has to be passable"


# ------------------------------------------------------ failure directions

def test_an_absent_profile_still_uses_the_box():
    """An old producer must keep exactly the behaviour it was validated with."""
    van = {"class": "vehicle", "x": 4.0, "y": 1.3, "size": [4.0, 2.0, 1.8],
           "motion": ct.STATIC}
    threat = cg.nearest_threat(summary([van]), CORRIDOR_HALF_WIDTH)
    assert threat is not None and threat.distance_m == pytest.approx(2.0)


@pytest.mark.parametrize("broken", [
    {"bin_m": 0.2, "y0": 0.0},                       # no slices
    {"bin_m": 0.0, "y0": 0.0, "min_x": [1.0]},       # zero-width slices
    {"bin_m": 0.2, "y0": 0.0, "min_x": []},          # empty
    {"bin_m": 0.2, "y0": 0.0, "min_x": [float("nan")]},
    {"bin_m": 0.2, "y0": 0.0, "min_x": ["near"]},
    {"bin_m": "wide", "y0": 0.0, "min_x": [1.0]},
    "not a profile",
])
def test_a_broken_profile_blocks_rather_than_falling_back(broken):
    """Falling back to the box would restore the over-approximation this
    exists to remove, and would do it silently. A producer that cannot
    describe its own returns is not one whose box should be believed."""
    item = {"class": "obstacle", "x": 8.0, "y": 0.0, "size": [0.5, 0.5, 1.0],
            "motion": ct.STATIC, "profile": broken}
    threat = cg.nearest_threat(summary([item]), CORRIDOR_HALF_WIDTH)
    assert threat is not None
    assert threat.distance_m == cg.BLOCKED


def test_a_malformed_box_blocks_even_with_a_clean_profile():
    """The parse check comes first: an object whose position did not parse
    must never be driven around, whatever else it carries."""
    item = {"class": "obstacle", "y": 0.0, "size": [1, 1, 1],
            "motion": ct.STATIC,
            "profile": {"bin_m": 0.2, "y0": 9.0, "min_x": [9.0]}}
    threat = cg.nearest_threat(summary([item]), CORRIDOR_HALF_WIDTH)
    assert threat.distance_m == cg.BLOCKED
    assert not threat.parked


def test_a_slice_straddling_the_corridor_edge_still_counts():
    """Conservative binning: overlap, not centre-in."""
    item = {"class": "obstacle", "x": 3.0, "y": 0.5, "size": [0.4, 0.4, 1.0],
            "motion": ct.STATIC,
            # slice [0.40, 0.60) straddles the 0.45 m corridor edge
            "profile": {"bin_m": 0.2, "y0": 0.4, "min_x": [3.0]}}
    threat = cg.nearest_threat(summary([item]), CORRIDOR_HALF_WIDTH)
    assert threat is not None and threat.distance_m == pytest.approx(3.0)


def test_an_object_wholly_beside_the_corridor_is_not_a_threat():
    item = {"class": "obstacle", "x": 3.0, "y": 2.0, "size": [0.4, 0.4, 1.0],
            "motion": ct.STATIC,
            "profile": {"bin_m": 0.2, "y0": 1.8, "min_x": [3.0, 3.1]}}
    assert cg.nearest_threat(summary([item]), CORRIDOR_HALF_WIDTH) is None


def test_an_empty_slice_inside_the_span_does_not_block():
    """An L-shaped cluster has slices it does not occupy. Those are not a
    claim about free ground - every other cluster is profiled separately -
    but this object must not be reported in one it is not in."""
    item = {"class": "obstacle", "x": 3.0, "y": 0.0, "size": [2.0, 2.0, 1.0],
            "motion": ct.STATIC,
            "profile": {"bin_m": 0.2, "y0": -0.6,
                        "min_x": [2.0, None, None, None, None, 2.0]}}
    threat = cg.nearest_threat(summary([item]), 0.15)
    assert threat is None


# ------------------------------------------------------------- the profiler

def test_the_profiler_records_the_nearest_return_per_slice():
    points = np.array([[5.0, 0.05, 0.9], [3.0, 0.05, 0.9], [7.0, 0.45, 0.9]])
    profile = lateral_profile(points)
    assert profile["bin_m"] == pytest.approx(0.2)
    assert profile["y0"] == pytest.approx(0.0)
    assert profile["min_x"][0] == pytest.approx(3.0)   # slice [0.0, 0.2)
    assert profile["min_x"][-1] == pytest.approx(7.0)  # slice [0.4, 0.6)


def test_the_profiler_never_publishes_more_than_the_cap():
    wide = np.column_stack([np.full(4000, 4.0), np.linspace(-30, 30, 4000),
                            np.full(4000, 0.9)])
    profile = lateral_profile(wide)
    assert len(profile["min_x"]) <= 64
    assert profile["bin_m"] > 0.2, "slices widen rather than multiply"


def test_the_profiler_survives_a_cluster_with_no_finite_returns():
    assert lateral_profile(np.full((4, 3), np.nan)) is None
    assert lateral_profile(np.zeros((0, 3))) is None
