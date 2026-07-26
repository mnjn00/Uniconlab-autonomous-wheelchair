#!/usr/bin/env python3

from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


pytest.importorskip("rospy")

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from auto_initial_pose import (
    candidate_diagnostic_state,
    should_abandon_candidate,
)


def _status(message, **values):
    return SimpleNamespace(
        message=message,
        values=[
            SimpleNamespace(key=key, value=str(value))
            for key, value in values.items()
        ],
    )


def test_sparse_target_crop_rejects_candidate_immediately():
    state = candidate_diagnostic_state(
        _status(
            "VERIFYING",
            reason="INSUFFICIENT_TARGET_POINTS",
            target_points=31,
        )
    )

    assert state == {
        "message": "VERIFYING",
        "reason": "INSUFFICIENT_TARGET_POINTS",
        "target_points": 31,
    }
    assert should_abandon_candidate(state)


def test_out_of_route_bounds_rejects_candidate_immediately():
    state = candidate_diagnostic_state(
        _status(
            "VERIFYING",
            reason="OUT_OF_ROUTE_BOUNDS",
            target_points=20441,
        )
    )

    assert should_abandon_candidate(state)


def test_transient_non_convergence_keeps_verifying_until_timeout():
    state = candidate_diagnostic_state(
        _status(
            "VERIFYING",
            reason="NOT_CONVERGED",
            target_points=17356,
        )
    )

    assert not should_abandon_candidate(state)


def test_malformed_target_point_value_does_not_crash_diagnostic_callback():
    state = candidate_diagnostic_state(
        _status(
            "VERIFYING",
            reason="LOW_INLIER_RATIO",
            target_points="not-an-int",
        )
    )

    assert state["target_points"] is None
    assert not should_abandon_candidate(state)
