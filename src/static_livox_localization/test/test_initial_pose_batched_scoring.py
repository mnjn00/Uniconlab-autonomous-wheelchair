"""The batched scorer must return what the one-at-a-time scorer returned.

Batching the nearest-neighbour queries was done for speed - 3.5x measured on
the deployed map, and it is the shape an accelerator needs. Speed work on the
path that decides where the chair thinks it is only counts if the answer is
untouched, so the scalar `_placement` is kept as the reference and these
compare against it directly rather than against a recorded number.

The tie-break is pinned too. `_best_in_window` used to scan yaw -> dx -> dy
and skip on `cost >= best_cost`, which keeps the FIRST minimum; the batched
version relies on argmin doing the same over an array built in that order. A
grid with a flat cost region - open pavement, exactly where this runs - would
otherwise settle on a different pose depending on which version ran.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    import initial_pose_global_search as G
finally:
    sys.path.pop(0)


def a_map(rng):
    """A little streetscape: two walls, a floor, and some furniture."""
    wall_a = np.column_stack([
        rng.uniform(-10, 10, 4000), np.full(4000, 3.0),
        rng.uniform(0, 2.5, 4000)])
    wall_b = np.column_stack([
        rng.uniform(-10, 10, 4000), np.full(4000, -3.0),
        rng.uniform(0, 2.5, 4000)])
    floor = np.column_stack([
        rng.uniform(-10, 10, 3000), rng.uniform(-3, 3, 3000),
        np.zeros(3000)])
    post = np.column_stack([
        rng.uniform(1.8, 2.2, 500), rng.uniform(-0.2, 0.2, 500),
        rng.uniform(0, 1.5, 500)])
    return np.vstack([wall_a, wall_b, floor, post]).astype(np.float32)


@pytest.fixture(scope="module")
def scene():
    rng = np.random.default_rng(7)
    points = a_map(rng)
    sample = points[rng.choice(len(points), 400, replace=False)].copy()
    return points, sample


def test_coarse_scores_match_the_scalar_reference(scene):
    points, sample = scene
    candidates = tuple(
        (float(x), 0.0, 0.0, 0.0) for x in np.linspace(-4.0, 4.0, 9))
    scored = G.score_global_candidates(sample, points, candidates, 0.45)
    tree = cKDTree(points)
    for candidate in scored:
        world = sample @ G._rotation(candidate.yaw_rad).T + np.array(
            [candidate.x, candidate.y, candidate.z], np.float32)
        assert candidate.score == pytest.approx(
            G._inlier_fraction(tree, world, 0.45), abs=1e-12)


def test_every_hypothesis_is_still_scored(scene):
    points, sample = scene
    candidates = tuple((float(x), 0.0, 0.0, 0.3) for x in (-2.0, 0.0, 2.0))
    scored = G.score_global_candidates(sample, points, candidates, 0.45)
    assert len(scored) == len(candidates) * len(G.coarse_yaw_offsets())


def test_refinement_picks_the_same_pose_as_a_scalar_scan(scene):
    """The whole window, scored one placement at a time, in the original
    order, keeping the first minimum - and it has to land on the same pose."""
    points, sample = scene
    tree = cKDTree(points)
    centre = G.InitializationCandidate(
        x=0.4, y=0.3, z=0.0, yaw_rad=0.05, score=0.0, source="global_search")
    positions = G._grid(0.5, 0.25)
    yaws = G._grid(math.radians(6.0), math.radians(3.0))

    best, best_cost = None, math.inf
    for yaw_offset in yaws:
        yaw = centre.yaw_rad + yaw_offset
        rotated = sample @ G._rotation(yaw).T
        for dx in positions:
            for dy in positions:
                origin = np.array(
                    [centre.x + dx, centre.y + dy, centre.z], np.float32)
                cost, inliers = G._placement(tree, rotated + origin, 0.45)
                if cost < best_cost:
                    best_cost, best = cost, (centre.x + dx, centre.y + dy,
                                             yaw, inliers)

    got = G._best_in_window(tree, sample, centre, positions, yaws, 0.45)
    assert got is not None
    assert (got.x, got.y) == pytest.approx((best[0], best[1]))
    assert got.yaw_rad == pytest.approx(best[2])
    assert got.score == pytest.approx(best[3], abs=1e-12)


def test_chunking_does_not_change_the_answer(scene):
    """The chunk size bounds memory on the NUC and must be invisible."""
    points, sample = scene
    candidates = tuple((float(x), 0.0, 0.0, 0.0) for x in (-1.0, 0.0, 1.0))
    original = G.PLACEMENT_CHUNK
    try:
        G.PLACEMENT_CHUNK = 10_000
        whole = G.score_global_candidates(sample, points, candidates, 0.45)
        G.PLACEMENT_CHUNK = 3
        split = G.score_global_candidates(sample, points, candidates, 0.45)
    finally:
        G.PLACEMENT_CHUNK = original
    assert [c.score for c in whole] == [c.score for c in split]


def test_an_empty_candidate_list_is_not_an_error(scene):
    points, sample = scene
    assert G.score_global_candidates(sample, points, (), 0.45) == ()
