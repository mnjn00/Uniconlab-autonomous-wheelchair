"""The global fallback has to land a seed the ICP can actually close.

Placed anywhere but the recorded start, the chair must find itself from the
map alone. The coarse pass samples the mapping trajectory, so every
hypothesis sits exactly on the recorded line at a fixed spacing and yaw
granularity: up to half the spacing along track, the full lateral offset of
wherever the chair was actually parked, and - before this - 22.5 deg of
heading error from a six-offset yaw grid. A 22.5 deg error displaces a point
10 m out by 3.9 m, far beyond the 0.5 m correspondence the localizer matches
with, so the right place scored no better than the wrong ones and
initialization ran out of candidates. These cases pin the two changes that
close the gap: a yaw grid fine enough to bracket the truth, and a local
refinement that walks a bracketed hypothesis onto it.
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).parents[1]


def load_search():
    path = ROOT / "scripts" / "initial_pose_global_search.py"
    spec = importlib.util.spec_from_file_location("global_search_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


search = load_search()


def rotation(yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        np.float32,
    )


def scene():
    """Two perpendicular walls over ground: enough to fix x, y and yaw.

    Ground alone is what open pavement looks like to a horizontal fix -
    invariant under any x, y or yaw - so the walls are what a real score has
    to be earned on.
    """
    generator = np.random.RandomState(7)
    count = 2000
    wall_y = np.column_stack([
        generator.uniform(-12.0, 12.0, count),
        np.full(count, 6.0),
        generator.uniform(-0.4, 2.4, count),
    ])
    wall_x = np.column_stack([
        np.full(count, 8.0),
        generator.uniform(-12.0, 12.0, count),
        generator.uniform(-0.4, 2.4, count),
    ])
    ground = np.column_stack([
        generator.uniform(-12.0, 12.0, count),
        generator.uniform(-12.0, 12.0, count),
        np.full(count, -0.7),
    ])
    return np.vstack([wall_y, wall_x, ground]).astype(np.float32)


def body_sample(map_points, x, y, z, yaw):
    """The scan a chair at that pose would return, in its own body frame."""

    shifted = np.asarray(map_points, np.float64) - np.array([x, y, z], np.float64)
    # Accelerate/BLAS raises spurious divide-by-zero and overflow flags on this
    # matmul while returning finite values throughout; the flags are noise, not
    # a numeric problem, and only this helper's operand shape trips them.
    with np.errstate(all="ignore"):
        sample = (shifted @ rotation(yaw).astype(np.float64)).astype(np.float32)
    return search.voxel_downsample(sample, 0.4, 1200)


TRUTH = (1.3, 0.7, 0.0, 0.35)


def candidate_at(x, y, z, yaw, score=None):
    return search.InitializationCandidate(
        x=x, y=y, z=z, yaw_rad=yaw, score=score, source="global_search"
    )


def test_refinement_recovers_a_pose_the_coarse_grid_only_brackets():
    """0.8 m across, 0.6 m along and 12 deg off - inside the coarse bracket,
    far outside what a 0.5 m correspondence closes."""
    map_points = scene()
    x, y, z, yaw = TRUTH
    sample = body_sample(map_points, x, y, z, yaw)
    coarse = candidate_at(x + 0.8, y - 0.6, z, yaw + math.radians(12.0))

    refined = search.refine_candidates(sample, map_points, (coarse,), 0.45)

    assert len(refined) == 1
    best = refined[0]
    # The bar that matters is the localizer's 0.5 m correspondence: a seed
    # inside it has something to lock onto. The grid resolves finer than that.
    assert math.hypot(best.x - x, best.y - y) <= 0.30
    heading_error = abs(
        math.degrees((best.yaw_rad - yaw + math.pi) % (2 * math.pi) - math.pi)
    )
    assert heading_error <= 4.0
    coarse_score = search.refine_candidates(
        sample, map_points, (coarse,), 0.45,
        position_radius_m=0.0, yaw_radius_rad=0.0,
    )[0].score
    assert best.score > coarse_score


def test_refinement_leaves_an_already_correct_seed_where_it_is():
    """Scoring by inlier fraction alone, every offset under the 0.45 m radius
    read 1.0 and refinement drifted a correct seed to the corner of its own
    search box. Truncated distance keeps a gradient, so the answer wins."""
    map_points = scene()
    x, y, z, yaw = TRUTH
    sample = body_sample(map_points, x, y, z, yaw)

    exact = candidate_at(x, y, z, yaw)
    refined = search.refine_candidates(sample, map_points, (exact,), 0.45)[0]

    assert refined.score >= 0.95
    assert math.hypot(refined.x - x, refined.y - y) <= 1e-6
    assert abs(refined.yaw_rad - yaw) <= 1e-9


def test_refined_candidates_are_still_score_gated_fallbacks():
    """Refinement improves a guess; it does not make it trusted. The source
    has to stay a global-search fallback so the minimum-score gate applies."""
    map_points = scene()
    x, y, z, yaw = TRUTH
    sample = body_sample(map_points, x, y, z, yaw)

    refined = search.refine_candidates(
        sample, map_points, (candidate_at(x, y, z, yaw),), 0.45
    )

    assert refined[0].source == "global_search"
    assert refined[0].score is not None


def test_refinement_returns_candidates_best_first():
    map_points = scene()
    x, y, z, yaw = TRUTH
    sample = body_sample(map_points, x, y, z, yaw)
    near = candidate_at(x + 0.3, y - 0.2, z, yaw + math.radians(4.0))
    wrong = candidate_at(x + 9.0, y - 7.0, z, yaw + math.radians(140.0))

    refined = search.refine_candidates(sample, map_points, (wrong, near), 0.45)

    assert refined[0].score >= refined[1].score
    assert math.hypot(refined[0].x - x, refined[0].y - y) <= 0.35


def test_the_coarse_yaw_grid_leaves_no_gap_a_seed_cannot_close():
    """Six offsets left a 22.5 deg worst case, which no local refinement
    window can recover because the bracketing hypothesis scores no better
    than open ground. The grid must be fine enough that the truth is always
    inside the refinement window."""
    offsets = search.coarse_yaw_offsets()

    assert len(offsets) >= 12
    ordered = sorted(offset % (2 * math.pi) for offset in offsets)
    gaps = [
        ordered[index + 1] - ordered[index] for index in range(len(ordered) - 1)
    ]
    gaps.append(ordered[0] + 2 * math.pi - ordered[-1])
    assert max(gaps) <= math.radians(30.0) + 1e-9
    # Worst-case error is half a gap, and it has to fall inside the window
    # refinement actually searches.
    assert max(gaps) / 2.0 <= search.REFINE_YAW_RADIUS_RAD + 1e-9


def test_the_refinement_window_covers_half_the_trajectory_spacing():
    """Candidates sit on the recorded line, so the chair is off it by
    whatever it was parked at plus half the sample spacing. The window has to
    reach that far or the coarse bracket is not actually a bracket."""
    assert search.REFINE_POSITION_RADIUS_M >= 1.0
    assert search.REFINE_POSITION_STEP_M <= 0.25


def test_coarse_scoring_uses_the_same_grid_it_publishes():
    """A yaw grid the scorer does not use would make the coverage guarantee
    above a fiction."""
    map_points = scene()
    x, y, z, yaw = TRUTH
    sample = body_sample(map_points, x, y, z, yaw)

    scored = search.score_global_candidates(
        sample, map_points, ((x, y, z, yaw),), 0.45
    )

    assert len(scored) == len(search.coarse_yaw_offsets())
    assert scored[0].score >= 0.95
