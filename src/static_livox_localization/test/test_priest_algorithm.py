"""Behavioral checks for the paper's Algorithm 1 and trajectory cost."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    import priest_planner as pl
finally:
    sys.path.remove(str(SCRIPTS))


class CostBasis:
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

    def positions(self, coefficients: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        assert len(coefficients) == len(self.x)
        return self.x, self.y

    def derivatives(
            self,
            coefficients: np.ndarray,
    ) -> tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]:
        assert len(coefficients) == len(self.x)
        return (self.vx, self.vy), (self.ax, self.ay)


class InvalidBatchBasis:
    n_c = 3
    times = np.linspace(0.0, 2.0, 3)


class InvalidBatchProjection:
    def set_corridor(
            self,
            centres: np.ndarray,
            normals: np.ndarray,
            left: np.ndarray,
            right: np.ndarray) -> None:
        assert len(centres) == len(normals) == len(left) == len(right)

    def solve(
            self,
            samples: np.ndarray,
            boundary: list[float],
            obstacles: np.ndarray,
            iterations: int) -> tuple[np.ndarray, np.ndarray]:
        assert len(boundary) == 8
        assert iterations > 0
        return samples, np.full(len(samples), np.inf)


class InvalidBatchPlanner(pl.PriestPlanner):
    def __init__(self) -> None:
        super().__init__(batch=2, iterations=1)
        self.invalid_basis = InvalidBatchBasis()
        self.invalid_projection = InvalidBatchProjection()

    def basis_for(
            self,
            horizon_s: float,
            n_obstacles: int,
    ) -> tuple[InvalidBatchBasis, InvalidBatchProjection]:
        assert horizon_s > 0.0
        assert n_obstacles >= 0
        return self.invalid_basis, self.invalid_projection

    def costs(
            self,
            basis: InvalidBatchBasis,
            xi: np.ndarray,
            local_goal: np.ndarray) -> np.ndarray:
        return np.full(len(xi), np.inf)


def test_nproj_filter_runs_before_augmented_cost_selection() -> None:
    selection = pl.select_priest_elite(
        primary_cost=np.array([10.0, 0.0, -1000.0]),
        residual_score=np.array([0.0, 0.10, 100.0]),
        nproj=2,
        nelite=1,
    )

    assert np.array_equal(selection.constraint_indices, np.array([0, 1]))
    assert selection.leader_index == 1


def test_augmented_all_invalid_batch_is_refused_before_weight_update() -> None:
    with pytest.raises(pl.NoFiniteCandidateError, match="finite augmented"):
        pl.select_priest_elite(
            primary_cost=np.array([np.nan, np.inf]),
            residual_score=np.array([np.inf, np.nan]),
            nproj=2,
            nelite=2,
        )


def test_planner_refuses_all_invalid_batch_without_nan_refit() -> None:
    corridor = pl.Corridor(
        centres=np.array([[0.0, 0.0], [2.0, 0.0]]),
        normals=np.array([[0.0, 1.0], [0.0, 1.0]]),
        left_m=np.ones(2),
        right_m=np.ones(2),
    )

    plan = InvalidBatchPlanner().attempt(
        start=np.zeros(2),
        velocity=np.zeros(2),
        acceleration=np.zeros(2),
        corridor=corridor,
        obstacles=np.empty((0, 3)),
        start_arc=0.0,
        reach=1.0,
    )

    assert not plan.usable
    assert plan.reason == "NO_FEASIBLE_TRAJECTORY"
    assert plan.xi is None


def test_alignment_cost_prefers_velocity_toward_the_goal() -> None:
    x = np.tile(np.array([0.0, 1.0, 2.0]), (2, 1))
    y = np.zeros((2, 3))
    basis = CostBasis(
        x=x,
        y=y,
        vx=np.array([[1.0, 1.0, 1.0], [0.0, 0.0, 0.0]]),
        vy=np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]),
        ax=np.zeros((2, 3)),
        ay=np.zeros((2, 3)),
    )

    cost = pl.PriestPlanner().costs(
        basis, np.zeros((2, 1)), np.array([2.0, 0.0]))

    assert cost[0] < cost[1], "equal speed was mistaken for equal alignment"


def test_curvature_cost_penalizes_lateral_not_tangential_acceleration() -> None:
    x = np.tile(np.array([0.0, 1.0, 2.0]), (2, 1))
    zeros = np.zeros((2, 3))
    basis = CostBasis(
        x=x,
        y=zeros,
        vx=np.ones((2, 3)),
        vy=zeros,
        ax=np.array([[1.0, 1.0, 1.0], [0.0, 0.0, 0.0]]),
        ay=np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]),
    )

    cost = pl.PriestPlanner().costs(
        basis, np.zeros((2, 1)), np.array([2.0, 0.0]))

    assert cost[0] < cost[1], "equal acceleration magnitude hid high curvature"
