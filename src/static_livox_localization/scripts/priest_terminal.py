"""Certified straight correction for a reachable corridor endpoint."""

from __future__ import annotations

import math
from typing import Optional, Protocol

import numpy as np

from priest_feasibility import BandContainment, certify_trajectory
from priest_types import Plan


STOPPED_SPEED_MPS = 0.02
MAX_LATERAL_SPEED_MPS = 0.02
MAX_HEADING_ERROR_RAD = 0.03


class TerminalPlanner(Protocol):
    v_max: float
    a_max: float
    control_hz: float
    band_grace_m: float


def terminal_correction(
        planner: TerminalPlanner,
        *,
        start_xy: np.ndarray,
        velocity_xy_mps: np.ndarray,
        goal_xy: np.ndarray,
        initial_yaw_rad: Optional[float],
        band: BandContainment,
        obstacles: np.ndarray,
) -> Optional[Plan]:
    """Return a straight quintic correction only when already aligned.

    The deployed base cannot turn in place. An offset terminal pose therefore
    gets this deterministic correction only if its body and current velocity
    already face the endpoint; otherwise the caller searches a larger arc.
    """
    start = np.asarray(start_xy, dtype=np.float64)
    velocity = np.asarray(velocity_xy_mps, dtype=np.float64)
    goal = np.asarray(goal_xy, dtype=np.float64)
    if (start.shape != (2,) or velocity.shape != (2,) or goal.shape != (2,)
            or not np.isfinite(start).all() or not np.isfinite(velocity).all()
            or not np.isfinite(goal).all()
            or initial_yaw_rad is not None
            and not math.isfinite(initial_yaw_rad)):
        return None
    delta = goal - start
    distance = float(np.linalg.norm(delta))
    if distance <= 1e-9:
        return None
    heading = math.atan2(float(delta[1]), float(delta[0]))
    direction = delta / distance
    forward_speed = float(np.dot(velocity, direction))
    lateral_speed = float(np.linalg.norm(
        velocity - forward_speed * direction))
    speed = float(np.linalg.norm(velocity))
    if forward_speed < 0.0 or lateral_speed > MAX_LATERAL_SPEED_MPS:
        return None
    evidence_yaw = heading if initial_yaw_rad is None else initial_yaw_rad
    if initial_yaw_rad is None and speed <= STOPPED_SPEED_MPS:
        return None
    heading_error = math.atan2(
        math.sin(heading - evidence_yaw), math.cos(heading - evidence_yaw))
    if abs(heading_error) > MAX_HEADING_ERROR_RAD:
        return None
    base_horizon = max(
        4.0,
        1.875 * distance / planner.v_max,
        math.sqrt(5.8 * distance / planner.a_max),
        1.1 * forward_speed / planner.a_max)
    for factor in (1.0, 1.15, 1.30):
        horizon = factor * base_horizon
        count = max(2, int(math.ceil(horizon * planner.control_hz)) + 1)
        times = np.linspace(0.0, horizon, count)
        phase = times / horizon
        initial_travel = forward_speed * horizon
        c3 = 10.0 * distance - 6.0 * initial_travel
        c4 = -15.0 * distance + 8.0 * initial_travel
        c5 = 6.0 * distance - 3.0 * initial_travel
        travel = initial_travel * phase + c3 * phase ** 3 \
            + c4 * phase ** 4 + c5 * phase ** 5
        travel_rate = (
            initial_travel + 3.0 * c3 * phase ** 2
            + 4.0 * c4 * phase ** 3 + 5.0 * c5 * phase ** 4
        ) / horizon
        travel_acceleration = (
            6.0 * c3 * phase + 12.0 * c4 * phase ** 2
            + 20.0 * c5 * phase ** 3
        ) / horizon ** 2
        if float(travel_rate.min()) < -1e-9:
            continue
        points = start + travel[:, None] * direction
        velocity_reference = travel_rate[:, None] * direction
        acceleration_reference = travel_acceleration[:, None] * direction
        yaw = np.full(count, heading)
        yaw_rate = np.zeros(count)
        certificate = certify_trajectory(
            planner, points=points, times_s=times, band=band,
            obstacles=obstacles, band_grace_m=planner.band_grace_m,
            initial_yaw_rad=evidence_yaw)
        if certificate.usable:
            return Plan(
                None, points[:, 0], points[:, 1], times, 0.0, distance, 1,
                horizon, certificate=certificate,
                velocity_xy_mps=velocity_reference,
                acceleration_xy_mps2=acceleration_reference,
                yaw_rad=yaw, yaw_rate_rps=yaw_rate)
    return None
