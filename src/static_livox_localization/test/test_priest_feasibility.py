"""Final trajectory certification at the planner/runtime safety boundary."""

import sys
from pathlib import Path

import numpy as np


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    import priest_planner as pl
finally:
    sys.path.remove(str(SCRIPTS))


class OffsetRejectingBand:
    """Runtime semantics stricter than an interpolated planner corridor."""

    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        return np.abs(points[:, 1]) <= 0.10 + grace


class OpenBand:
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        del grace
        return np.ones(len(points), dtype=bool)


def test_runtime_band_refuses_a_residual_zero_candidate_outside_its_bounds() -> None:
    """The projection's corridor approximation cannot overrule SafetyBand."""
    planner = pl.PriestPlanner(v_max=0.6, a_max=0.18)
    points = np.array([[0.0, 0.20], [1.0, 0.20], [2.0, 0.20]])
    times_s = np.array([0.0, 5.0, 10.0])

    certificate = planner.certify(
        points=points,
        times_s=times_s,
        band=OffsetRejectingBand(),
        obstacles=np.empty((0, 3)),
    )

    assert not certificate.usable
    assert certificate.reason == "OUTSIDE_RUNTIME_BAND"


def test_corner_obstacle_is_not_cleared_by_centre_distance_alone() -> None:
    """A chair corner collision must fail even when half-width clearance passes."""
    planner = pl.PriestPlanner(v_max=0.6, a_max=0.18)
    points = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    times_s = np.array([0.0, 5.0, 10.0])
    obstacle = np.array([[1.0, 0.60, 0.05]])

    certificate = planner.certify(
        points=points,
        times_s=times_s,
        band=OpenBand(),
        obstacles=obstacle,
    )

    assert not certificate.usable
    assert certificate.reason == "OBSTACLE_CLEARANCE"
