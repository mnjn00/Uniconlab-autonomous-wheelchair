"""Terminal braking and one-step stop-viability control."""

from __future__ import annotations

import math
from typing import Optional

from priest_actuator_control import (
    ActuatorLimits,
    select_effective_twist,
    stopping_displacement_m,
)
from priest_constraints import within_goal_tolerance
from priest_control_types import ControllerLimits, DriveCommand
from wheel_command_model import EffectiveTwist


STOPPED_SPEED_MPS = 0.02


def should_start_braking(
        goal_dx_m: float,
        goal_dy_m: float,
        pose_yaw_rad: float,
        previous_linear_mps: float,
        previous_angular_rps: float,
        measured_speed_mps: float,
        goal_tolerance_m: float,
        limits: ActuatorLimits) -> bool:
    forward, lateral = stopping_displacement_m(
        previous_linear_mps, previous_angular_rps,
        measured_speed_mps, limits)
    stopped_dx, stopped_dy = _remaining_after_body_displacement(
        goal_dx_m, goal_dy_m, pose_yaw_rad, forward, lateral)
    remaining = math.hypot(goal_dx_m, goal_dy_m)
    predicted_error = math.hypot(stopped_dx, stopped_dy)
    projected_gap = 0.0 if remaining <= 1e-9 else (
        stopped_dx * goal_dx_m + stopped_dy * goal_dy_m) / remaining
    return within_goal_tolerance(
        predicted_error, goal_tolerance_m) or projected_gap <= 0.0


def terminal_brake(
        previous_linear_mps: float,
        previous_angular_rps: float,
        measured_speed_mps: float,
        remaining_m: float,
        limits: ControllerLimits) -> DriveCommand:
    stopped = previous_linear_mps <= 1e-9 \
        and abs(previous_angular_rps) <= 1e-9 and measured_speed_mps \
        <= STOPPED_SPEED_MPS
    if stopped:
        done = within_goal_tolerance(
            remaining_m, limits.goal_tolerance_m)
        reason = "AT_PLAN_END" if done else "TERMINAL_REPLAN"
        return DriveCommand(0.0, 0.0, reason=reason, done=done)
    effective = select_effective_twist(
        0.0, 0.0, previous_linear_mps, previous_angular_rps, limits)
    if effective is None:
        return DriveCommand(0.0, 0.0, reason="INVALID_DYNAMICS")
    return DriveCommand(
        effective.linear_x_mps, effective.angular_z_rps,
        reason="TERMINAL_BRAKING")


def preserve_terminal_viability(
        proposed: EffectiveTwist,
        goal_dx_m: float,
        goal_dy_m: float,
        pose_yaw_rad: float,
        previous_linear_mps: float,
        previous_angular_rps: float,
        measured_speed_mps: float,
        goal_tolerance_m: float,
        limits: ControllerLimits) -> Optional[EffectiveTwist]:
    """Keep the next grid command inside the forward-only stopping set."""
    inputs = (
        goal_dx_m, goal_dy_m, pose_yaw_rad, previous_linear_mps,
        previous_angular_rps, measured_speed_mps, goal_tolerance_m)
    if not all(math.isfinite(value) for value in inputs):
        return None
    if _viable(proposed, *inputs[:3], measured_speed_mps,
               goal_tolerance_m, limits):
        return proposed
    candidates = []
    for desired_linear, desired_angular in (
            (proposed.linear_x_mps, 0.0),
            (previous_linear_mps, previous_angular_rps),
            (0.0, 0.0)):
        candidate = select_effective_twist(
            desired_linear, desired_angular,
            previous_linear_mps, previous_angular_rps, limits)
        if candidate is not None and _viable(
                candidate, goal_dx_m, goal_dy_m, pose_yaw_rad,
                measured_speed_mps, goal_tolerance_m, limits):
            candidates.append(candidate)
    if not candidates:
        return None
    linear_scale = max(limits.max_speed_mps, 1e-9)
    angular_scale = max(limits.max_yaw_rate_rps, 1e-9)
    return min(candidates, key=lambda candidate: (
        (candidate.linear_x_mps - proposed.linear_x_mps) / linear_scale) ** 2
        + ((candidate.angular_z_rps - proposed.angular_z_rps)
           / angular_scale) ** 2)


def _viable(
        candidate: EffectiveTwist,
        goal_dx_m: float,
        goal_dy_m: float,
        pose_yaw_rad: float,
        measured_speed_mps: float,
        goal_tolerance_m: float,
        limits: ControllerLimits) -> bool:
    period = limits.control_period_s
    yaw_change = candidate.angular_z_rps * period
    if abs(candidate.angular_z_rps) <= 1e-12:
        tick_x, tick_y = candidate.linear_x_mps * period, 0.0
    else:
        radius = candidate.linear_x_mps / candidate.angular_z_rps
        tick_x = radius * math.sin(yaw_change)
        tick_y = radius * (1.0 - math.cos(yaw_change))
    next_measured = max(
        candidate.linear_x_mps,
        measured_speed_mps - limits.max_deceleration_mps2 * period)
    forward, lateral = stopping_displacement_m(
        candidate.linear_x_mps, candidate.angular_z_rps,
        next_measured, limits)
    body_x = tick_x + math.cos(yaw_change) * forward \
        - math.sin(yaw_change) * lateral
    body_y = tick_y + math.sin(yaw_change) * forward \
        + math.cos(yaw_change) * lateral
    stopped_dx, stopped_dy = _remaining_after_body_displacement(
        goal_dx_m, goal_dy_m, pose_yaw_rad, body_x, body_y)
    predicted_error = math.hypot(stopped_dx, stopped_dy)
    remaining = math.hypot(goal_dx_m, goal_dy_m)
    if within_goal_tolerance(predicted_error, goal_tolerance_m):
        return True
    if remaining <= 1e-9:
        return False
    projected_gap = (
        stopped_dx * goal_dx_m + stopped_dy * goal_dy_m) / remaining
    return projected_gap > 0.0


def _remaining_after_body_displacement(
        goal_dx_m: float,
        goal_dy_m: float,
        pose_yaw_rad: float,
        body_x_m: float,
        body_y_m: float) -> tuple[float, float]:
    cosine, sine = math.cos(pose_yaw_rad), math.sin(pose_yaw_rad)
    return (
        goal_dx_m - cosine * body_x_m + sine * body_y_m,
        goal_dy_m - sine * body_x_m - cosine * body_y_m)
