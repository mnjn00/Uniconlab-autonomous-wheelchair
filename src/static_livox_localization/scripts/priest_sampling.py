"""Pure Algorithm 1 selection for projection-guided sampling."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class NoFiniteCandidateError(ValueError):
    """Raised when Algorithm 1 has no candidate it can safely refit."""


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
