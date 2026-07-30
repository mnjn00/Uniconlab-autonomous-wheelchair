"""Accepting a global fix is a safety decision, so it gets its own gate.

Downstream verification proves a candidate is self-consistent - the ICP
converges and stays converged - not that it is the right place. A pose that is
plausible and wrong therefore passes, and the follower goes on to compute the
route from it. There is nothing further along that catches this, so the two
ways a global fix can be untrustworthy are both refused here.
"""

import importlib.util
import math
import sys
from pathlib import Path


ROOT = Path(__file__).parents[1]


def load_search():
    path = ROOT / "scripts" / "initial_pose_global_search.py"
    spec = importlib.util.spec_from_file_location("decision_under_test", path)
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


def test_a_strongly_supported_unrivalled_pose_is_accepted():
    refined = (
        candidate(58.5, 1.5, 21.0, 0.97),
        candidate(54.2, 0.5, 201.0, 0.52),
    )

    decision = search.decide_fix(refined)

    assert decision.reason == "accepted"
    assert decision.candidate.score == 0.97
    assert decision.rival is None


def test_a_distant_near_tie_is_refused_as_ambiguous():
    """Two places explaining the scan equally well means the scan does not say
    which one it is."""
    refined = (
        candidate(58.5, 1.5, 21.0, 0.93),
        candidate(96.0, 1.0, 21.0, 0.90),
    )

    decision = search.decide_fix(refined)

    assert decision.reason == "ambiguous"
    assert decision.candidate is None
    assert decision.rival.x == 96.0


def test_the_same_place_facing_backwards_counts_as_a_rival():
    """The mistake with the worst consequences: driving the route in reverse
    from a pose that is otherwise self-consistent."""
    refined = (
        candidate(58.5, 1.5, 21.0, 0.93),
        candidate(58.5, 1.5, 201.0, 0.91),
    )

    decision = search.decide_fix(refined)

    assert decision.reason == "ambiguous"


def test_a_near_neighbour_at_the_same_heading_is_not_a_rival():
    """Refinement of two adjacent trajectory samples lands on nearly the same
    pose. That is agreement, not ambiguity."""
    refined = (
        candidate(58.5, 1.5, 21.0, 0.93),
        candidate(59.6, 1.4, 23.0, 0.92),
    )

    decision = search.decide_fix(refined)

    assert decision.reason == "accepted"


def test_a_lone_but_weakly_supported_winner_is_refused():
    """The measured self-similar-street failure: it wins outright, with no rival
    to expose it, and is still 21 m wrong. What gives it away is explaining only
    two thirds of what the chair can see."""
    refined = (
        candidate(82.8, 0.5, 201.0, 0.65),
        candidate(54.0, 0.5, 201.0, 0.51),
    )

    decision = search.decide_fix(refined)

    assert decision.reason == "weak_support"
    assert decision.candidate is None
    # The rejected pose is reported so the operator sees what was refused.
    assert decision.rival.x == 82.8


def test_the_support_threshold_is_a_parameter_for_field_calibration():
    """The default comes from synthetic scenes. Transient structure the map
    does not hold - parked cars, pedestrians - lowers a correct fix, so the
    number has to be adjustable without a code change."""
    refined = (candidate(82.8, 0.5, 201.0, 0.65),)

    assert search.decide_fix(refined, 0.80).reason == "weak_support"
    assert search.decide_fix(refined, 0.60).reason == "accepted"
    assert 0.5 <= search.MIN_REFINED_SCORE <= 0.95


def test_no_candidates_is_reported_distinctly_from_a_bad_one():
    decision = search.decide_fix(())

    assert decision.reason == "no_candidates"
    assert decision.candidate is None
    assert decision.rival is None


def test_ambiguity_is_checked_before_support():
    """Two mutually contradictory weak answers are ambiguous, and saying so is
    more use to an operator than calling the better one weak."""
    refined = (
        candidate(58.5, 1.5, 21.0, 0.60),
        candidate(96.0, 1.0, 21.0, 0.58),
    )

    assert search.decide_fix(refined).reason == "ambiguous"
