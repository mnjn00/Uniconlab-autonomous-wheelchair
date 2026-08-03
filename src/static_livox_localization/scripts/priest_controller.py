"""ROS-free time-indexed differential-drive tracking for PRIEST plans."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Protocol

import numpy as np


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
    control_period_s: float = 0.1
    goal_tolerance_m: float = 0.05
    turn_in_place_rad: float = 0.6

    def __post_init__(self) -> None:
        values = (
            self.max_speed_mps, self.max_acceleration_mps2,
            self.max_deceleration_mps2, self.max_yaw_rate_rps,
            self.max_yaw_acceleration_rps2, self.control_period_s,
            self.goal_tolerance_m, self.turn_in_place_rad)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("controller limits must be finite")
        if min(values[:6]) <= 0.0 or self.goal_tolerance_m < 0.0 \
                or not 0.0 < self.turn_in_place_rad <= math.pi:
            raise ValueError("controller limits must be physically positive")


DEFAULT_CONTROLLER_LIMITS = ControllerLimits()
STOPPED_SPEED_MPS = 0.02


def _stop(reason: str, done: bool = False) -> DriveCommand:
    return DriveCommand(0.0, 0.0, reason=reason, done=done)


def _plan_arrays(
        plan: TimedPlan,
) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    if plan.x is None or plan.y is None or plan.times is None:
        return None
    x = np.asarray(plan.x, dtype=np.float64)
    y = np.asarray(plan.y, dtype=np.float64)
    times = np.asarray(plan.times, dtype=np.float64)
    if (x.ndim != 1 or y.shape != x.shape or times.shape != x.shape
            or len(x) < 2 or not np.isfinite(x).all()
            or not np.isfinite(y).all() or not np.isfinite(times).all()
            or abs(times[0]) > 1e-9 or np.any(np.diff(times) <= 0.0)):
        return None
    return x, y, times


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _reference(
        plan: TimedPlan,
        x: np.ndarray,
        y: np.ndarray,
        times: np.ndarray,
        elapsed_s: float,
) -> Optional[tuple[float, float, float, float, float]]:
    raw = (
        getattr(plan, "velocity_xy_mps", None),
        getattr(plan, "yaw_rad", None),
        getattr(plan, "yaw_rate_rps", None))
    if any(value is not None for value in raw):
        if any(value is None for value in raw):
            return None
        velocity = np.asarray(raw[0], dtype=np.float64)
        yaw = np.asarray(raw[1], dtype=np.float64)
        yaw_rate = np.asarray(raw[2], dtype=np.float64)
        if (velocity.shape != (len(times), 2) or yaw.shape != times.shape
                or yaw_rate.shape != times.shape
                or not np.isfinite(velocity).all()
                or not np.isfinite(yaw).all()
                or not np.isfinite(yaw_rate).all()):
            return None
        velocity_x, velocity_y = velocity[:, 0], velocity[:, 1]
    else:
        velocity_x = np.gradient(x, times)
        velocity_y = np.gradient(y, times)
        moving = np.hypot(velocity_x, velocity_y) > 1e-6
        if not np.any(moving):
            return None
        moving_yaw = np.unwrap(np.arctan2(
            velocity_y[moving], velocity_x[moving]))
        yaw = np.interp(times, times[moving], moving_yaw)
        yaw_rate = np.gradient(yaw, times)
    speed = np.hypot(velocity_x, velocity_y)
    if not np.any(speed > 1e-6):
        return None
    return (
        float(np.interp(elapsed_s, times, x)),
        float(np.interp(elapsed_s, times, y)),
        float(np.interp(elapsed_s, times, yaw)),
        float(np.interp(elapsed_s, times, speed)),
        float(np.interp(elapsed_s, times, yaw_rate)),
    )


def _linear_slew(
        desired_mps: float,
        previous_mps: float,
        limits: ControllerLimits) -> float:
    lower = max(
        0.0,
        previous_mps - limits.max_deceleration_mps2 * limits.control_period_s)
    upper = min(
        limits.max_speed_mps,
        previous_mps + limits.max_acceleration_mps2 * limits.control_period_s)
    return min(max(desired_mps, lower), upper)


def _angular_slew(
        desired_rps: float,
        previous_rps: float,
        limits: ControllerLimits) -> float:
    step = limits.max_yaw_acceleration_rps2 * limits.control_period_s
    bounded = min(max(desired_rps, previous_rps - step), previous_rps + step)
    return min(max(bounded, -limits.max_yaw_rate_rps),
               limits.max_yaw_rate_rps)


def _valid_state(
        elapsed_s: float,
        pose: Pose2D,
        measured_speed_mps: float,
        previous: DriveCommand) -> bool:
    values = (
        elapsed_s, pose.x_m, pose.y_m, pose.yaw_rad, measured_speed_mps,
        previous.linear_x_mps, previous.angular_z_rps)
    return (all(math.isfinite(value) for value in values)
            and elapsed_s >= 0.0 and measured_speed_mps >= 0.0
            and previous.linear_x_mps >= 0.0)


def command_for(
        plan: TimedPlan,
        elapsed_s: float,
        pose: Pose2D,
        measured_speed_mps: float,
        previous_command: DriveCommand,
        limits: ControllerLimits = DEFAULT_CONTROLLER_LIMITS,
) -> DriveCommand:
    """Track the reference at ``elapsed_s`` without a lateral command."""
    if plan.reason:
        return _stop(plan.reason, done=plan.reason == "AT_GOAL")
    if not bool(getattr(plan, "usable", False)):
        return _stop("UNCERTIFIED_PLAN")
    arrays = _plan_arrays(plan)
    if arrays is None:
        return _stop("INVALID_PLAN")
    if not _valid_state(
            elapsed_s, pose, measured_speed_mps, previous_command):
        return _stop("INVALID_STATE")
    x, y, times = arrays
    if elapsed_s >= times[-1]:
        return _stop("AT_PLAN_END", done=True)
    if math.hypot(x[-1] - pose.x_m, y[-1] - pose.y_m) \
            <= limits.goal_tolerance_m:
        return _stop("AT_GOAL", done=True)

    reference = _reference(plan, x, y, times, elapsed_s)
    if reference is None:
        return _stop("INVALID_PLAN")
    ref_x, ref_y, ref_yaw, ref_speed, ref_yaw_rate = reference
    world_x = ref_x - pose.x_m
    world_y = ref_y - pose.y_m
    longitudinal = math.cos(pose.yaw_rad) * world_x \
        + math.sin(pose.yaw_rad) * world_y
    lateral = -math.sin(pose.yaw_rad) * world_x \
        + math.cos(pose.yaw_rad) * world_y
    heading_error = _wrap(ref_yaw - pose.yaw_rad)
    aligning = abs(heading_error) > limits.turn_in_place_rad

    desired_linear = 0.0 if aligning else (
        ref_speed * math.cos(heading_error) + 1.2 * longitudinal
        + 0.2 * (ref_speed - measured_speed_mps))
    linear = _linear_slew(
        min(max(desired_linear, 0.0), limits.max_speed_mps),
        previous_command.linear_x_mps, limits)
    desired_angular = 1.8 * heading_error if aligning else (
        ref_yaw_rate + 1.8 * heading_error + 2.0 * ref_speed * lateral)
    if aligning and max(linear, measured_speed_mps) > STOPPED_SPEED_MPS:
        desired_angular = 0.0
    angular = _angular_slew(
        desired_angular, previous_command.angular_z_rps, limits)
    return DriveCommand(
        linear, angular, reason="ALIGNING" if aligning else "")
