"""Native-unit projection evidence, kept separate from scalar ranking."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    import priest_projection as pp
finally:
    sys.path.remove(str(SCRIPTS))


class FakeKinematics:
    def __init__(
            self,
            x: np.ndarray,
            y: np.ndarray,
            vx: np.ndarray,
            vy: np.ndarray,
            ax: np.ndarray,
            ay: np.ndarray) -> None:
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ax = ax
        self.ay = ay
        self.steps = x.shape[1]
        self.times = np.linspace(0.0, 2.0, self.steps)

    def positions(self, coefficients: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        assert len(coefficients) == len(self.x)
        return self.x, self.y

    def derivatives(
            self,
            coefficients: np.ndarray,
    ) -> tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]:
        assert len(coefficients) == len(self.x)
        return (self.vx, self.vy), (self.ax, self.ay)


def projection_with(kinematics: FakeKinematics) -> pp.Projection:
    basis = pp.TrajectoryBasis(degree=3, steps=3, horizon_s=2.0)
    projection = pp.Projection(basis, max_obstacles=1, v_max=0.6, a_max=0.5)
    projection.basis = kinematics
    return projection


def test_physical_units_report_maximum_violation_in_each_named_field() -> None:
    kinematics = FakeKinematics(
        x=np.array([[0.20, 2.0, 2.0]]),
        y=np.zeros((1, 3)),
        vx=np.full((1, 3), 0.70),
        vy=np.zeros((1, 3)),
        ax=np.full((1, 3), 0.80),
        ay=np.zeros((1, 3)),
    )
    projection = projection_with(kinematics)

    violations = projection.violations(
        np.zeros((1, 1)), np.array([[0.0, 0.0, 0.30]]))

    assert violations.obstacle_m[0] == pytest.approx(0.10)
    assert violations.corridor_m[0] == pytest.approx(0.0)
    assert violations.speed_mps[0] == pytest.approx(0.10)
    assert violations.acceleration_mps2[0] == pytest.approx(0.30)


def test_clear_obstacle_distance_cannot_cancel_an_exceeded_speed_tolerance() -> None:
    kinematics = FakeKinematics(
        x=np.array([[0.0, 1.0, 2.0]]),
        y=np.zeros((1, 3)),
        vx=np.full((1, 3), 0.61),
        vy=np.zeros((1, 3)),
        ax=np.zeros((1, 3)),
        ay=np.zeros((1, 3)),
    )
    projection = projection_with(kinematics)

    violations = projection.violations(
        np.zeros((1, 1)), np.array([[20.0, 0.0, 0.30]]))

    assert violations.obstacle_m[0] == pytest.approx(0.0)
    assert not violations.is_within(pp.DEFAULT_CONSTRAINT_TOLERANCES)[0]


def test_yaw_rate_uses_wrapped_heading_change_per_second() -> None:
    vx = np.array([[1.0, 1.0], [1.0, 0.0]])
    vy = np.array([[0.0, 0.0], [0.0, 1.0]])

    yaw_rate = pp.max_yaw_rate_rps(vx, vy, np.array([0.0, 1.0]))

    assert yaw_rate[0] == pytest.approx(0.0)
    assert yaw_rate[1] == pytest.approx(np.pi / 2.0)


def test_yaw_rate_from_rest_includes_body_heading_mismatch() -> None:
    vx = np.array([[0.0, 1.0]])
    vy = np.array([[0.0, 0.0]])

    yaw_rate = pp.max_yaw_rate_rps(
        vx, vy, np.array([0.0, 1.0]),
        initial_yaw_rad=np.array([np.pi / 2.0]))

    assert yaw_rate[0] == pytest.approx(np.pi / 2.0)
