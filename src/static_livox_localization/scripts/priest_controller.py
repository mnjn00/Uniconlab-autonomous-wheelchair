"""ROS-free time-indexed differential-drive tracking for PRIEST plans."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from priest_actuator_control import (
    effective_acceleration_mps2,
    select_effective_twist,
)
from priest_control_types import (
    ControllerLimits,
    DEFAULT_CONTROLLER_LIMITS,
    DriveCommand,
    Pose2D,
    TimedPlan,
)
from priest_heading_pid import SteeringFeedback, angular_slew
from priest_terminal_control import (
    STOPPED_SPEED_MPS,
    preserve_terminal_viability,
    should_start_braking,
    terminal_brake,
)
from wheel_command_model import YAW_DEADBAND_RAD_S, required_turn_linear_mps


def _stop(reason: str, done: bool = False) -> DriveCommand:
    return DriveCommand(0.0, 0.0, reason=reason, done=done)


def plan_arrays(
        plan: TimedPlan,
) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    if plan.x is None or plan.y is None or plan.times is None:
        return None
    try:
        x = np.asarray(plan.x, dtype=np.float64)
        y = np.asarray(plan.y, dtype=np.float64)
        times = np.asarray(plan.times, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return None
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
        try:
            velocity = np.asarray(raw[0], dtype=np.float64)
            yaw = np.asarray(raw[1], dtype=np.float64)
            yaw_rate = np.asarray(raw[2], dtype=np.float64)
        except (TypeError, ValueError, OverflowError):
            return None
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


def command_acceleration_mps2(
        previous: DriveCommand,
        command: DriveCommand,
        period_s: float) -> float:
    """Cartesian acceleration implied by two constant-twist commands."""
    return effective_acceleration_mps2(
        previous.linear_x_mps, previous.angular_z_rps,
        command.linear_x_mps, command.angular_z_rps, period_s)


def _bounded_linear(
        desired_mps: float,
        previous_mps: float,
        angular_rps: float,
        limits: ControllerLimits) -> Optional[float]:
    inverse_period_sq = 1.0 / limits.control_period_s ** 2
    quadratic = inverse_period_sq + angular_rps ** 2
    linear = -2.0 * previous_mps * inverse_period_sq
    constant = previous_mps ** 2 * inverse_period_sq \
        - limits.max_acceleration_mps2 ** 2
    discriminant = linear ** 2 - 4.0 * quadratic * constant
    if discriminant < -1e-12:
        return None
    root = math.sqrt(max(discriminant, 0.0))
    lower = max(
        0.0, (-linear - root) / (2.0 * quadratic),
        previous_mps
        - limits.max_deceleration_mps2 * limits.control_period_s)
    upper = min(
        limits.max_speed_mps, (-linear + root) / (2.0 * quadratic),
        previous_mps
        + limits.max_acceleration_mps2 * limits.control_period_s)
    if lower > upper + 1e-12:
        return None
    return min(max(desired_mps, lower), upper)


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
        *,
        steering: Optional[SteeringFeedback] = None,
) -> DriveCommand:
    """Track the reference at ``elapsed_s`` without a lateral command."""
    if plan.reason:
        return _stop(plan.reason, done=plan.reason == "AT_GOAL")
    if not bool(getattr(plan, "usable", False)):
        return _stop("UNCERTIFIED_PLAN")
    arrays = plan_arrays(plan)
    if arrays is None:
        return _stop("INVALID_PLAN")
    if not _valid_state(
            elapsed_s, pose, measured_speed_mps, previous_command):
        return _stop("INVALID_STATE")
    x, y, times = arrays
    previous_effective = select_effective_twist(
        previous_command.linear_x_mps, previous_command.angular_z_rps,
        previous_command.linear_x_mps, previous_command.angular_z_rps,
        limits)
    if previous_effective is None:
        return _stop("INVALID_DYNAMICS")
    previous_linear = previous_effective.linear_x_mps
    previous_angular = previous_effective.angular_z_rps
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
    if abs(heading_error) > limits.max_heading_error_rad:
        return _stop("HEADING_REPLAN")
    if previous_command.reason == "TERMINAL_BRAKING" \
            or should_start_braking(
                x[-1] - pose.x_m, y[-1] - pose.y_m, pose.yaw_rad,
                previous_linear, previous_angular, measured_speed_mps,
                limits.goal_tolerance_m, limits):
        return terminal_brake(
            previous_linear, previous_angular, measured_speed_mps,
            math.hypot(x[-1] - pose.x_m, y[-1] - pose.y_m), limits)
    if elapsed_s >= times[-1]:
        return _stop("PLAN_EXPIRED")

    desired_linear = (
        ref_speed * math.cos(heading_error) + 1.2 * longitudinal
        + 0.2 * (ref_speed - measured_speed_mps))
    if steering is None:
        heading_command = ref_yaw_rate + 1.8 * heading_error
    else:
        if not math.isfinite(steering.measured_yaw_rate_rps):
            return _stop("INVALID_STATE")
        heading_command = steering.pid.update(
            heading_error, ref_yaw_rate, steering.measured_yaw_rate_rps)
    desired_angular = heading_command + 2.0 * ref_speed * lateral
    angular = angular_slew(desired_angular, previous_angular, limits)
    if abs(angular) <= YAW_DEADBAND_RAD_S:
        angular = 0.0
    if previous_linear > 1e-9:
        turn_cap = limits.max_acceleration_mps2 \
            / previous_linear
        angular = math.copysign(min(abs(angular), turn_cap), angular)
    required_turn_speed = max(
        limits.turn_floor_speed_mps, required_turn_linear_mps(angular)) \
        if abs(angular) > YAW_DEADBAND_RAD_S else 0.0
    desired_linear = max(desired_linear, required_turn_speed)
    linear = _bounded_linear(
        desired_linear, previous_linear, angular, limits)
    if linear is None:
        return _stop("INVALID_DYNAMICS")
    accelerating = abs(angular) > YAW_DEADBAND_RAD_S and min(
        linear, measured_speed_mps) + 1e-9 < required_turn_speed
    if accelerating:
        angular = 0.0
        linear = _bounded_linear(
            desired_linear, previous_linear, angular, limits)
        if linear is None:
            return _stop("INVALID_DYNAMICS")
    effective = select_effective_twist(
        linear, angular, previous_linear, previous_angular, limits)
    if effective is None:
        return _stop("INVALID_DYNAMICS")
    viable = preserve_terminal_viability(
        effective, x[-1] - pose.x_m, y[-1] - pose.y_m, pose.yaw_rad,
        previous_linear, previous_angular, measured_speed_mps,
        limits.goal_tolerance_m, limits)
    if viable is None:
        return terminal_brake(
            previous_linear, previous_angular, measured_speed_mps,
            math.hypot(x[-1] - pose.x_m, y[-1] - pose.y_m), limits)
    effective = viable
    turning_transition = accelerating or (
        abs(angular) > YAW_DEADBAND_RAD_S
        and abs(effective.angular_z_rps) <= YAW_DEADBAND_RAD_S)
    command = DriveCommand(
        effective.linear_x_mps, effective.angular_z_rps,
        reason="TURN_ACCELERATING" if turning_transition else "")
    if command_acceleration_mps2(
            previous_command, command, limits.control_period_s) \
            > limits.max_acceleration_mps2 + 1e-9:
        return _stop("INVALID_DYNAMICS")
    return command
