"""Typed contracts for ROS-free PRIEST differential-drive control."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Protocol

import numpy as np

from priest_constraints import TURN_FLOOR_SPEED_MPS


class TimedPlan(Protocol):
    x: Optional[np.ndarray]
    y: Optional[np.ndarray]
    times: Optional[np.ndarray]
    velocity_xy_mps: Optional[np.ndarray]
    yaw_rad: Optional[np.ndarray]
    yaw_rate_rps: Optional[np.ndarray]
    reason: str

    @property
    def usable(self) -> bool:
        ...


@dataclass(frozen=True)
class Pose2D:
    __slots__ = ("x_m", "y_m", "yaw_rad")

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class DriveCommand:
    """The only executable planar inputs of a differential-drive chair."""

    linear_x_mps: float
    angular_z_rps: float
    reason: str = ""
    done: bool = False


@dataclass(frozen=True)
class ControllerLimits:
    max_speed_mps: float = 0.6
    max_acceleration_mps2: float = 0.18
    max_deceleration_mps2: float = 0.6
    max_yaw_rate_rps: float = 0.5
    max_yaw_acceleration_rps2: float = 1.5
    control_period_s: float = 0.2
    goal_tolerance_m: float = 0.05
    max_heading_error_rad: float = 0.6
    turn_floor_speed_mps: float = TURN_FLOOR_SPEED_MPS

    def __post_init__(self) -> None:
        values = (
            self.max_speed_mps, self.max_acceleration_mps2,
            self.max_deceleration_mps2, self.max_yaw_rate_rps,
            self.max_yaw_acceleration_rps2, self.control_period_s,
            self.goal_tolerance_m, self.max_heading_error_rad,
            self.turn_floor_speed_mps)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("controller limits must be finite")
        if min(values[:6]) <= 0.0 or self.goal_tolerance_m < 0.0 \
                or not 0.0 < self.max_heading_error_rad <= math.pi \
                or not 0.0 < self.turn_floor_speed_mps <= self.max_speed_mps:
            raise ValueError("controller limits must be physically positive")


DEFAULT_CONTROLLER_LIMITS = ControllerLimits()
