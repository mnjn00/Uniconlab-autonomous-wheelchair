"""Final trajectory certification at the planner/runtime safety boundary."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    import priest_feasibility as feasibility
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


class RecordingBand(OpenBand):
    def __init__(self) -> None:
        self.sample_count = 0

    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        self.sample_count = len(points)
        return super().contains_many(points, grace)


class MidpointRejectingBand:
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        del grace
        return np.abs(points[:, 0] - 0.5) > 0.02


class NumericBand:
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        del grace
        return np.ones(len(points), dtype=np.float64)


def coefficients(points: list[tuple[float, float]]) -> np.ndarray:
    xy = np.asarray(points, dtype=np.float64)
    return np.concatenate([xy[:, 0], xy[:, 1]])


def certify_coefficients(
        control_points: list[tuple[float, float]],
        *,
        horizon_s: float,
        planner: pl.PriestPlanner,
        band: OpenBand | MidpointRejectingBand | RecordingBand,
) -> feasibility.CertifiedTrajectory:
    return feasibility.certify_coefficients(
        planner,
        coefficients=coefficients(control_points),
        degree=len(control_points) - 1,
        horizon_s=horizon_s,
        control_hz=10.0,
        band=band,
        obstacles=np.empty((0, 3)),
    )


def test_valid_runtime_band_is_certified_at_control_frequency() -> None:
    band = RecordingBand()
    planner = pl.PriestPlanner(v_max=1.0, a_max=0.18, yaw_rate_max=0.5)

    dense = certify_coefficients(
        [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)],
        horizon_s=2.0, planner=planner, band=band)

    assert dense.certificate.usable
    assert band.sample_count == 21
    assert len(dense.times_s) == 21
    assert dense.certificate.max_speed_mps == pytest.approx(0.5)


def test_residual_zero_outside_dense_runtime_band_is_refused() -> None:
    planner = pl.PriestPlanner(v_max=1.0, a_max=0.18, yaw_rate_max=0.5)

    dense = certify_coefficients(
        [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)],
        horizon_s=2.0, planner=planner, band=MidpointRejectingBand())

    assert not dense.certificate.usable
    assert dense.certificate.reason == "OUTSIDE_RUNTIME_BAND"


def test_acceleration_physical_tolerance_is_point_one_eight() -> None:
    planner = pl.PriestPlanner(v_max=2.0, a_max=0.18, yaw_rate_max=2.0)

    dense = certify_coefficients(
        [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
        horizon_s=2.0, planner=planner, band=OpenBand())

    assert pl.PriestPlanner().a_max == pytest.approx(0.18)
    assert pl.PriestPlanner().yaw_rate_max == pytest.approx(0.5)
    assert dense.certificate.tolerances == planner.CONSTRAINT_TOLERANCES
    assert dense.certificate.max_acceleration_mps2 == pytest.approx(0.5)
    assert dense.certificate.reason == "ACCELERATION_LIMIT"


def test_curvature_yaw_rate_excess_is_refused() -> None:
    planner = pl.PriestPlanner(v_max=2.0, a_max=2.0, yaw_rate_max=0.5)

    dense = certify_coefficients(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        horizon_s=2.0, planner=planner, band=OpenBand())

    assert dense.certificate.max_yaw_rate_rps > 0.5
    assert dense.certificate.reason == "YAW_RATE_LIMIT"


def test_slow_nonzero_curvature_cannot_hide_yaw_rate_excess() -> None:
    planner = pl.PriestPlanner(v_max=1.0, a_max=1.0, yaw_rate_max=0.5)

    dense = certify_coefficients(
        [(0.0, 0.0), (0.005, 0.0), (0.005, 0.005)],
        horizon_s=2.0, planner=planner, band=OpenBand())

    assert np.linalg.norm(dense.velocity_xy_mps, axis=1).max() < 0.02
    assert dense.certificate.max_yaw_rate_rps > 0.5
    assert dense.certificate.reason == "YAW_RATE_LIMIT"


def test_zero_speed_heading_reversal_is_not_certified_as_zero_yaw() -> None:
    planner = pl.PriestPlanner()

    dense = certify_coefficients(
        [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
        horizon_s=5.0, planner=planner, band=OpenBand())

    assert dense.certificate.max_yaw_rate_rps > planner.yaw_rate_max
    assert dense.certificate.reason == "YAW_RATE_LIMIT"


@pytest.mark.parametrize("settings", [
    {"control_hz": 9.99},
    {"band_grace_m": 0.1001},
    {"v_max": np.nan},
    {"a_max": np.inf},
    {"yaw_rate_max": np.nan},
    {"turn_floor_speed_mps": 0.61},
])
def test_certificate_settings_cannot_weaken_runtime_contract(
        settings: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        pl.PriestPlanner(**settings)


def test_lowest_augmented_cost_candidate_must_also_certify() -> None:
    certificates = [
        feasibility.TrajectoryCertificate.refusal(
            "OUTSIDE_RUNTIME_BAND", pl.DEFAULT_CONSTRAINT_TOLERANCES),
        feasibility.TrajectoryCertificate.clear(
            pl.DEFAULT_CONSTRAINT_TOLERANCES),
    ]

    selected = feasibility.lowest_certified_index(
        np.array([0.1, 0.2]), certificates)

    assert selected == 1


def test_invalid_band_certificate_preserves_active_tolerances() -> None:
    planner = pl.PriestPlanner()
    planner.CONSTRAINT_TOLERANCES = pl.ConstraintTolerances(
        0.2, 0.3, 0.4, 0.5, 0.6)

    dense = feasibility.certify_coefficients(
        planner, coefficients=coefficients(
            [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]),
        degree=2, horizon_s=2.0, control_hz=10.0,
        band=NumericBand(), obstacles=np.empty((0, 3)))

    assert dense.certificate.reason == "RUNTIME_BAND_INVALID"
    assert dense.certificate.tolerances == planner.CONSTRAINT_TOLERANCES


def test_reason_empty_plan_with_refusal_certificate_is_not_usable() -> None:
    plan = pl.Plan(
        np.zeros(2), np.zeros(2), np.zeros(2), np.array([0.0, 1.0]),
        0.0, 0.0, 1, 1.0,
        certificate=feasibility.TrajectoryCertificate.refusal(
            "OUTSIDE_RUNTIME_BAND", pl.DEFAULT_CONSTRAINT_TOLERANCES))

    assert not plan.usable


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
