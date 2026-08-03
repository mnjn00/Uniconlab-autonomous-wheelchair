"""Pure Algorithm 1 selection for projection-guided sampling."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


class NoFiniteCandidateError(ValueError):
    """Raised when Algorithm 1 has no candidate it can safely refit."""


class CostBasis(Protocol):
    def positions(
            self, coefficients: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ...

    def derivatives(
            self,
            coefficients: np.ndarray,
    ) -> tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]:
        ...


@dataclass(frozen=True)
class PriestSelection:
    __slots__ = (
        "constraint_indices", "elite_indices", "elite_augmented_cost",
        "leader_index")

    constraint_indices: np.ndarray
    elite_indices: np.ndarray
    elite_augmented_cost: np.ndarray
    leader_index: int


def select_priest_elite(
        *,
        primary_cost: np.ndarray,
        residual_score: np.ndarray,
        nproj: int,
        nelite: int) -> PriestSelection:
    """Apply paper lines 7-11 and return the lowest augmented-cost elite."""
    primary = np.asarray(primary_cost, dtype=np.float64)
    residual = np.asarray(residual_score, dtype=np.float64)
    if (primary.ndim != 1 or residual.shape != primary.shape
            or not len(primary)):
        raise ValueError("cost and residual must be non-empty matching vectors")
    if np.any(residual < 0.0):
        raise ValueError("constraint residual scores cannot be negative")
    primary = np.where(np.isfinite(primary), primary, float("inf"))
    residual = np.where(np.isfinite(residual), residual, float("inf"))

    projection_count = min(max(int(nproj), 1), len(primary))
    constraint_indices = np.argsort(
        residual, kind="mergesort")[:projection_count]
    augmented = primary[constraint_indices] + residual[constraint_indices]
    finite_order = np.argsort(augmented, kind="mergesort")
    finite_order = finite_order[np.isfinite(augmented[finite_order])]
    if not len(finite_order):
        raise NoFiniteCandidateError("no finite augmented-cost candidate")
    elite_count = min(max(int(nelite), 1), len(finite_order))
    order = finite_order[:elite_count]
    elite_indices = constraint_indices[order]
    elite_augmented = augmented[order]
    return PriestSelection(
        constraint_indices=constraint_indices,
        elite_indices=elite_indices,
        elite_augmented_cost=elite_augmented,
        leader_index=int(elite_indices[0]),
    )


def trajectory_costs(
        basis: CostBasis,
        coefficients: np.ndarray,
        local_goal: np.ndarray,
        local_tangent: np.ndarray | None = None) -> np.ndarray:
    """Smoothness, curvature, goal alignment and terminal reach."""
    x, y = basis.positions(coefficients)
    (vx, vy), (ax, ay) = basis.derivatives(coefficients)
    smooth = (ax ** 2 + ay ** 2).mean(axis=1)
    speed_sq = vx ** 2 + vy ** 2
    cross = np.abs(vx * ay - vy * ax)
    curvature = np.where(
        speed_sq > 0.05 ** 2,
        cross / np.maximum(speed_sq, 0.05 ** 2) ** 1.5,
        0.0).mean(axis=1)
    alignment_vector = (
        local_goal[None, :] - np.stack([x[:, 0], y[:, 0]], axis=1)
        if local_tangent is None
        else np.broadcast_to(np.asarray(local_tangent), (len(x), 2)))
    terminal_velocity = np.stack([vx[:, -1], vy[:, -1]], axis=1)
    alignment_denom = np.maximum(
        np.linalg.norm(alignment_vector, axis=1)
        * np.linalg.norm(terminal_velocity, axis=1), 1e-6)
    alignment = 1.0 - np.clip(
        np.einsum("ij,ij->i", alignment_vector, terminal_velocity)
        / alignment_denom, -1.0, 1.0)
    reach = np.hypot(x[:, -1] - local_goal[0], y[:, -1] - local_goal[1])
    return smooth + 0.25 * curvature + alignment + 4.0 * reach
