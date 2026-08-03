"""Canonical physical dimensions and independently-unitized constraints."""

from dataclasses import dataclass
from math import hypot
from typing import Final


@dataclass(frozen=True)
class Footprint:
    """Axis-aligned chair footprint about the commanded body origin."""

    __slots__ = (
        "front_m", "rear_m", "half_width_m", "planning_margin_m",
        "sweep_margin_m")

    front_m: float
    rear_m: float
    half_width_m: float
    planning_margin_m: float
    sweep_margin_m: float

    @property
    def circumscribed_radius_m(self) -> float:
        return hypot(max(self.front_m, self.rear_m), self.half_width_m)


@dataclass(frozen=True)
class ConstraintTolerances:
    """Numerical slack per physical unit; values are never summed."""

    __slots__ = (
        "obstacle_m", "corridor_m", "speed_mps", "acceleration_mps2",
        "yaw_rate_rps")

    obstacle_m: float
    corridor_m: float
    speed_mps: float
    acceleration_mps2: float
    yaw_rate_rps: float


@dataclass(frozen=True)
class ConstraintViolations:
    """Non-negative maxima in their native physical units."""

    __slots__ = (
        "obstacle_m", "corridor_m", "speed_mps", "acceleration_mps2",
        "yaw_rate_rps")

    obstacle_m: float
    corridor_m: float
    speed_mps: float
    acceleration_mps2: float
    yaw_rate_rps: float

    def is_within(self, tolerances: ConstraintTolerances) -> bool:
        return (
            self.obstacle_m <= tolerances.obstacle_m
            and self.corridor_m <= tolerances.corridor_m
            and self.speed_mps <= tolerances.speed_mps
            and self.acceleration_mps2 <= tolerances.acceleration_mps2
            and self.yaw_rate_rps <= tolerances.yaw_rate_rps
        )


CANONICAL_FOOTPRINT: Final = Footprint(
    front_m=0.50,
    rear_m=0.50,
    half_width_m=0.30,
    planning_margin_m=0.10,
    sweep_margin_m=0.15,
)

DEFAULT_CONSTRAINT_TOLERANCES: Final = ConstraintTolerances(
    obstacle_m=1e-6,
    corridor_m=0.0,
    speed_mps=1e-3,
    acceleration_mps2=1e-3,
    yaw_rate_rps=1e-3,
)
