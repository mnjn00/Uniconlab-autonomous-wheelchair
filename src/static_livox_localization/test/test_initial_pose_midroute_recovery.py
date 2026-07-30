"""End to end: find the chair from the map alone, with no seed given.

This is the case the field needs and the one that was failing. Placed partway
along the route rather than at the recorded start, the chair gets no usable
prior - the route's first waypoint is somewhere else entirely - so
initialization has to come from the map. The chain under test is the whole
fallback: drop ground, score every trajectory sample at every coarse heading,
shortlist the distinct ones, refine each onto the map, then decide whether the
answer is actually identified.

Two scenes, because the honest answer differs between them. Where the
surroundings are distinctive the chain finds the pose to a tenth of a metre.
On a long self-similar stretch it cannot, and the measured behaviour without a
guard is worse than failure: it ranks a place 40 m away first and scores it
0.88. Nothing downstream catches that - the localizer verifies that a
candidate is self-consistent, not that it is the right place - so the guard
that refuses is as much a part of the feature as the search.
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).parents[1]

# The node's own gate on global fallbacks.
MIN_SCORE = 0.25
# The localizer matches at 0.5 m correspondence with a 3 deg candidate
# tolerance; a seed inside those has something to lock onto.
CORRESPONDENCE_M = 0.5


def load_search():
    path = ROOT / "scripts" / "initial_pose_global_search.py"
    spec = importlib.util.spec_from_file_location("midroute_under_test", path)
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
        np.float64,
    )


def wall(generator, start, end, count):
    along = generator.uniform(0.0, 1.0, count)
    return np.column_stack([
        start[0] + along * (end[0] - start[0]),
        start[1] + along * (end[1] - start[1]),
        generator.uniform(-0.3, 3.0, count),
    ])


def poles(generator, centres):
    parts = []
    for x, y in centres:
        angle = generator.uniform(0.0, 2 * math.pi, 500)
        parts.append(np.column_stack([
            x + 0.12 * np.cos(angle),
            y + 0.12 * np.sin(angle),
            generator.uniform(0.0, 3.0, 500),
        ]))
    return parts


def corner_scene():
    """An L junction, the way the recorded route actually turns a block.

    A corner is identifiable: the two facades and the gap between them are only
    in that arrangement at one place, so the score can be earned rather than
    guessed.
    """
    generator = np.random.RandomState(5)
    parts = [
        wall(generator, (0.0, 8.0), (64.0, 8.0), 4000),
        wall(generator, (64.0, 8.0), (64.0, 70.0), 4000),
        wall(generator, (0.0, -6.0), (78.0, -6.0), 4500),
        wall(generator, (78.0, -6.0), (78.0, 70.0), 4500),
    ]
    parts.extend(poles(generator, (
        (18.5, 3.9), (41.2, -2.1), (70.5, 20.4), (72.0, 48.2),
    )))
    count = 120000
    xs = generator.uniform(-4.0, 80.0, count)
    ys = generator.uniform(-6.0, 70.0, count)
    paved = (ys < 8.0) | (xs > 64.0)
    parts.append(np.column_stack([
        xs[paved], ys[paved], np.full(paved.sum(), -0.75),
    ]))
    return np.vstack(parts).astype(np.float32)


def corner_trajectory(spacing_m=3.0):
    legs = [(float(x), 0.0, 0.0, 0.0) for x in np.arange(0.0, 71.0, spacing_m)]
    legs += [
        (71.0, float(y), 0.0, math.pi / 2)
        for y in np.arange(3.0, 66.0, spacing_m)
    ]
    return tuple(legs)


def straight_scene():
    """A long sidewalk between two facades: genuinely self-similar.

    Not a strawman - it is what the middle of a city block looks like, and it
    is where the search has to refuse instead of guessing.
    """
    generator = np.random.RandomState(11)
    parts = []
    segments = ((0.0, 27.0), (31.0, 19.0), (56.0, 34.0), (94.0, 26.0))
    for side, base in ((-1.0, -6.0), (1.0, 8.0)):
        for index, (start, length) in enumerate(segments):
            offset = 0.9 * index * side
            count = int(length * 60)
            parts.append(np.column_stack([
                generator.uniform(start, start + length, count),
                np.full(count, base + offset),
                generator.uniform(-0.3, 2.6, count),
            ]))
    parts.extend(poles(generator, (
        (14.3, 4.1), (37.9, -3.2), (52.6, 5.5),
        (66.1, -2.7), (78.4, 6.0), (103.2, -4.4),
    )))
    count = 90000
    parts.append(np.column_stack([
        generator.uniform(-4.0, 124.0, count),
        generator.uniform(-6.0, 8.0, count),
        np.full(count, -0.75),
    ]))
    return np.vstack(parts).astype(np.float32)


def straight_trajectory(spacing_m=3.0):
    return tuple(
        (float(x), 0.0, 0.0, 0.0)
        for x in np.arange(0.0, 123.0, spacing_m)
    )


def observed_scan(map_points, x, y, yaw, max_range_m=25.0):
    """The chair's accumulated submap from that pose, prepared as the node does."""

    visible = map_points[
        np.linalg.norm(map_points[:, :2] - np.array([x, y], np.float32), axis=1)
        < max_range_m
    ]
    shifted = np.asarray(visible, np.float64) - np.array([x, y, 0.0], np.float64)
    with np.errstate(all="ignore"):
        body = (shifted @ rotation(yaw)).astype(np.float32)
    structure, removed = search.structural_sample(body)
    assert removed, "the scene must have structure to score on"
    return search.voxel_downsample(structure, 0.4, 1800)


def run_fallback(map_points, trajectory, x, y, yaw):
    sample = observed_scan(map_points, x, y, yaw)
    scored = search.score_global_candidates(sample, map_points, trajectory, 0.45)
    shortlist = search.diverse_shortlist(scored, 4)
    refined = search.refine_candidates(sample, map_points, shortlist, 0.45)
    return scored, refined


def offset_from(candidate, x, y):
    return math.hypot(candidate.x - x, candidate.y - y)


def heading_error_deg(candidate, yaw):
    return abs(
        math.degrees(
            (candidate.yaw_rad - yaw + math.pi) % (2 * math.pi) - math.pi
        )
    )


# Parked partway along, off the recorded line, and not squared up to it: half
# the 3 m spacing plus a metre and a half of parking offset is more than one
# refinement window, which is what the re-centring rounds are for.
CORNER_X, CORNER_Y, CORNER_YAW = 58.6, 1.5, math.radians(21.0)


def test_a_distinctive_place_is_found_and_seeded_with_no_prior():
    map_points = corner_scene()

    _, refined = run_fallback(
        map_points, corner_trajectory(), CORNER_X, CORNER_Y, CORNER_YAW
    )
    decision = search.decide_fix(refined)

    assert decision.reason == "accepted", "refused a place it could identify"
    winner = decision.candidate
    assert offset_from(winner, CORNER_X, CORNER_Y) <= CORRESPONDENCE_M
    assert heading_error_deg(winner, CORNER_YAW) <= 4.0
    assert winner.score >= MIN_SCORE


def test_a_chair_parked_backwards_is_found_without_anyone_noticing():
    """The likeliest placement mistake there is. The coarse grid covers the
    full circle, so a reversed start must not need a human to spot it."""
    map_points = corner_scene()
    backwards = CORNER_YAW + math.pi

    _, refined = run_fallback(
        map_points, corner_trajectory(), CORNER_X, CORNER_Y, backwards
    )
    decision = search.decide_fix(refined)

    assert decision.reason == "accepted"
    assert offset_from(decision.candidate, CORNER_X, CORNER_Y) <= CORRESPONDENCE_M
    assert heading_error_deg(decision.candidate, backwards) <= 4.0


def test_without_refinement_the_right_answer_fails_the_nodes_own_gate():
    """The measurement that explains the whole failure. The best coarse
    hypothesis brackets the truth by 2 m and 9 deg and scores 0.147 - below the
    0.25 the node requires of a fallback - so the correct answer was thrown
    away before it was ever tried. Refined, the same hypothesis scores 1.0."""
    map_points = corner_scene()

    scored, refined = run_fallback(
        map_points, corner_trajectory(), CORNER_X, CORNER_Y, CORNER_YAW
    )

    coarse = scored[0]
    assert (
        offset_from(coarse, CORNER_X, CORNER_Y) > CORRESPONDENCE_M
        or heading_error_deg(coarse, CORNER_YAW) > 3.0
    )
    assert coarse.score < MIN_SCORE
    assert refined[0].score > coarse.score
    assert refined[0].score >= MIN_SCORE


def test_a_self_similar_stretch_is_refused_rather_than_seeded_wrongly():
    """Measured, not assumed: on this stretch the search ranks a place 21 m
    away first, and it wins outright - no near rival to give it away - so a
    relative margin alone would have seeded it. What gives it away is how
    little it explains: 0.65 where an identified place scores 1.0.

    A wrong pose would pass verification, because verification proves
    self-consistency rather than correctness, and the follower would then drive
    a route computed from it. Refusing is the safe answer, and one the operator
    can act on."""
    map_points = straight_scene()

    _, refined = run_fallback(
        map_points, straight_trajectory(), 61.4, 1.5, math.radians(21.0)
    )
    decision = search.decide_fix(refined)

    assert decision.candidate is None, (
        "seeded ({:.1f}, {:.1f}) when the chair was at (61.4, 1.5)".format(
            refined[0].x, refined[0].y
        )
    )
    assert decision.reason in ("ambiguous", "weak_support")
    # The pose it would have seeded really is wrong, so refusing is not the
    # guard being over-cautious about an answer that happened to be right.
    assert offset_from(refined[0], 61.4, 1.5) > 5.0
