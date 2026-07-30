"""The candidates actually attempted have to be distinct places.

Verification is the expensive step - each attempt costs a reset and up to
twenty seconds - so the handful of hypotheses that get one must not be four
views of the same spot. A sidewalk is self-similar along its length and the
coarse pass scores every yaw at every trajectory sample, so a naive top-N is
exactly that: the same location at neighbouring headings, or four points on
one straight stretch. If the true pose was not the single best coarse score,
initialization then spends its whole budget polishing near-duplicates and
gives up with candidates it never tried.

Suppressing near-duplicates keeps the same budget and spends it on genuinely
different answers. Two headings from one spot are different answers; five
degrees apart is not.
"""

import importlib.util
import math
import sys
from pathlib import Path


ROOT = Path(__file__).parents[1]


def load_search():
    path = ROOT / "scripts" / "initial_pose_global_search.py"
    spec = importlib.util.spec_from_file_location("shortlist_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


search = load_search()


def candidate(x, y, yaw_deg, score):
    return search.InitializationCandidate(
        x=x,
        y=y,
        z=0.0,
        yaw_rad=math.radians(yaw_deg),
        score=score,
        source="global_search",
    )


def test_one_place_at_nearly_one_heading_yields_one_attempt():
    scored = (
        candidate(10.0, 4.0, 30.0, 0.80),
        candidate(10.0, 4.0, 35.0, 0.78),
        candidate(10.2, 4.1, 25.0, 0.77),
    )

    shortlist = search.diverse_shortlist(scored, 3)

    assert len(shortlist) == 1
    assert shortlist[0].score == 0.80


def test_the_same_place_facing_the_other_way_is_a_different_answer():
    """A chair parked backwards is the single most likely mistake the coarse
    grid has to keep on the list, not collapse into its neighbour."""
    scored = (
        candidate(10.0, 4.0, 30.0, 0.80),
        candidate(10.0, 4.0, 210.0, 0.74),
    )

    shortlist = search.diverse_shortlist(scored, 4)

    assert len(shortlist) == 2
    headings = sorted(round(math.degrees(item.yaw_rad)) % 360 for item in shortlist)
    assert headings == [30, 210]


def test_distinct_places_along_the_route_all_survive():
    scored = (
        candidate(10.0, 4.0, 30.0, 0.80),
        candidate(30.0, 6.0, 30.0, 0.79),
        candidate(60.0, 9.0, 30.0, 0.78),
    )

    shortlist = search.diverse_shortlist(scored, 4)

    assert len(shortlist) == 3


def test_the_shortlist_respects_the_attempt_budget():
    scored = tuple(
        candidate(10.0 * index, 0.0, 0.0, 0.9 - 0.01 * index)
        for index in range(12)
    )

    assert len(search.diverse_shortlist(scored, 4)) == 4


def test_the_shortlist_stays_best_first():
    scored = (
        candidate(0.0, 0.0, 0.0, 0.60),
        candidate(40.0, 0.0, 0.0, 0.90),
        candidate(80.0, 0.0, 0.0, 0.75),
    )

    shortlist = search.diverse_shortlist(scored, 3)

    assert [item.score for item in shortlist] == [0.90, 0.75, 0.60]


def test_separation_is_not_wider_than_the_trajectory_spacing():
    """Suppressing further than adjacent trajectory samples would discard
    real alternatives; suppressing less would keep duplicates that the
    refinement window merges anyway."""
    assert 1.5 <= search.SHORTLIST_MIN_SEPARATION_M <= 3.0
    assert search.SHORTLIST_MIN_SEPARATION_M >= search.REFINE_POSITION_RADIUS_M


def test_an_empty_score_list_is_handled():
    assert search.diverse_shortlist((), 4) == ()
