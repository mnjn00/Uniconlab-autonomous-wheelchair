"""Choose controller outputs on the deployed wheel-command grid."""

from __future__ import annotations

import math
from collections import deque
from functools import lru_cache
from typing import Optional, Protocol

from wheel_command_model import (
    EffectiveTwist,
    WHEEL_SEPARATION_M,
    YAW_DEADBAND_RAD_S,
    effective_twist,
    required_turn_linear_mps,
)


class ActuatorLimits(Protocol):
    max_speed_mps: float
    max_acceleration_mps2: float
    max_deceleration_mps2: float
    max_yaw_rate_rps: float
    max_yaw_acceleration_rps2: float
    control_period_s: float
    turn_floor_speed_mps: float


def effective_acceleration_mps2(
        previous_linear_mps: float,
        previous_angular_rps: float,
        linear_mps: float,
        angular_rps: float,
        period_s: float) -> float:
    previous = effective_twist(previous_linear_mps, previous_angular_rps)
    current = effective_twist(linear_mps, angular_rps)
    if previous is None or current is None or period_s <= 0.0:
        return math.inf
    tangential = (current.linear_x_mps - previous.linear_x_mps) / period_s
    centripetal = current.linear_x_mps * current.angular_z_rps
    return math.hypot(tangential, centripetal)


def _safe(
        candidate: EffectiveTwist,
        previous: EffectiveTwist,
        limits: ActuatorLimits) -> bool:
    acceleration = math.hypot(
        (candidate.linear_x_mps - previous.linear_x_mps)
        / limits.control_period_s,
        candidate.linear_x_mps * candidate.angular_z_rps)
    yaw_acceleration = abs(
        candidate.angular_z_rps - previous.angular_z_rps) \
        / limits.control_period_s
    required = max(
        limits.turn_floor_speed_mps,
        required_turn_linear_mps(candidate.angular_z_rps)) \
        if abs(candidate.angular_z_rps) > YAW_DEADBAND_RAD_S else 0.0
    replay = effective_twist(
        candidate.linear_x_mps, candidate.angular_z_rps)
    return (replay is not None
            and abs(replay.linear_x_mps - candidate.linear_x_mps) <= 1e-9
            and abs(replay.angular_z_rps - candidate.angular_z_rps) <= 1e-9
            and candidate.linear_x_mps <= limits.max_speed_mps + 1e-9
            and abs(candidate.angular_z_rps) <= limits.max_yaw_rate_rps + 1e-9
            and acceleration <= limits.max_acceleration_mps2 + 1e-9
            and yaw_acceleration <= limits.max_yaw_acceleration_rps2 + 1e-9
            and candidate.linear_x_mps + 1e-9 >= required)


@lru_cache(maxsize=4096)
def select_effective_twist(
        desired_linear_mps: float,
        desired_angular_rps: float,
        previous_linear_mps: float,
        previous_angular_rps: float,
        limits: ActuatorLimits) -> Optional[EffectiveTwist]:
    if not math.isfinite(desired_linear_mps) \
            or not math.isfinite(desired_angular_rps):
        return None
    previous = effective_twist(previous_linear_mps, previous_angular_rps)
    if previous is None or not _safe(previous, previous, limits):
        return None
    candidates = _actuator_grid(limits)
    steady = [candidate for candidate in candidates
              if _safe(candidate, candidate, limits)]
    if not steady:
        return None
    target = min(steady, key=lambda candidate: _distance(
        candidate, desired_linear_mps, desired_angular_rps, limits))
    return _next_grid_step(previous, target, steady, limits)


@lru_cache(maxsize=512)
def stopping_distance_m(
        previous_linear_mps: float,
        previous_angular_rps: float,
        measured_speed_mps: float,
        limits: ActuatorLimits) -> float:
    """Conservative distance through the actual grid path to exact zero."""
    return _stopping_motion(
        previous_linear_mps, previous_angular_rps,
        measured_speed_mps, limits)[0]


@lru_cache(maxsize=512)
def stopping_displacement_m(
        previous_linear_mps: float,
        previous_angular_rps: float,
        measured_speed_mps: float,
        limits: ActuatorLimits) -> tuple[float, float]:
    """Body-frame displacement through the grid path to exact zero."""
    motion = _stopping_motion(
        previous_linear_mps, previous_angular_rps,
        measured_speed_mps, limits)
    return motion[1], motion[2]


@lru_cache(maxsize=512)
def _stopping_motion(
        previous_linear_mps: float,
        previous_angular_rps: float,
        measured_speed_mps: float,
        limits: ActuatorLimits) -> tuple[float, float, float]:
    state = effective_twist(previous_linear_mps, previous_angular_rps)
    if state is None or not math.isfinite(measured_speed_mps) \
            or measured_speed_mps < 0.0:
        return math.inf, math.inf, math.inf
    excess = max(0.0, measured_speed_mps - state.linear_x_mps) ** 2 \
        / (2.0 * limits.max_acceleration_mps2)
    distance, x_m, y_m, yaw_rad = excess, excess, 0.0, 0.0
    for _ in range(200):
        following = select_effective_twist(
            0.0, 0.0, state.linear_x_mps, state.angular_z_rps, limits)
        if following is None:
            return math.inf, math.inf, math.inf
        period = limits.control_period_s
        distance += following.linear_x_mps * limits.control_period_s
        if abs(following.angular_z_rps) <= 1e-12:
            x_m += following.linear_x_mps * math.cos(yaw_rad) * period
            y_m += following.linear_x_mps * math.sin(yaw_rad) * period
        else:
            next_yaw = yaw_rad + following.angular_z_rps * period
            radius = following.linear_x_mps / following.angular_z_rps
            x_m += radius * (math.sin(next_yaw) - math.sin(yaw_rad))
            y_m -= radius * (math.cos(next_yaw) - math.cos(yaw_rad))
            yaw_rad = next_yaw
        if following.linear_x_mps <= 1e-9 \
                and abs(following.angular_z_rps) <= 1e-9:
            return distance, x_m, y_m
        if _key(following) == _key(state):
            return math.inf, math.inf, math.inf
        state = following
    return math.inf, math.inf, math.inf


def _distance(
        candidate: EffectiveTwist,
        linear_mps: float,
        angular_rps: float,
        limits: ActuatorLimits) -> float:
    linear_scale = max(limits.max_speed_mps, 1e-9)
    angular_scale = max(limits.max_yaw_rate_rps, 1e-9)
    return ((candidate.linear_x_mps - linear_mps) / linear_scale) ** 2 \
        + ((candidate.angular_z_rps - angular_rps) / angular_scale) ** 2


def _key(candidate: EffectiveTwist) -> tuple[int, int]:
    return round(candidate.left_kmh * 10.0), round(
        candidate.right_kmh * 10.0)


def _next_grid_step(
        previous: EffectiveTwist,
        target: EffectiveTwist,
        candidates: list[EffectiveTwist],
        limits: ActuatorLimits) -> Optional[EffectiveTwist]:
    states = {_key(candidate): candidate for candidate in candidates}
    start, goal = _key(previous), _key(target)
    if start not in states or goal not in states:
        return None
    queue = deque([start])
    parents = {start: None}
    while queue and goal not in parents:
        current_key = queue.popleft()
        current = states[current_key]
        neighbours = []
        for left_step in (-1, 0, 1):
            for right_step in (-1, 0, 1):
                if left_step == 0 and right_step == 0:
                    continue
                neighbour_key = (
                    current_key[0] + left_step,
                    current_key[1] + right_step)
                candidate = states.get(neighbour_key)
                if candidate is not None and neighbour_key not in parents \
                        and _safe(candidate, current, limits):
                    neighbours.append((neighbour_key, candidate))
        neighbours.sort(key=lambda item: _distance(
            item[1], target.linear_x_mps, target.angular_z_rps, limits))
        for neighbour_key, _ in neighbours:
            parents[neighbour_key] = current_key
            queue.append(neighbour_key)
    if goal not in parents:
        goal = min(parents, key=lambda item: _distance(
            states[item], target.linear_x_mps, target.angular_z_rps, limits))
    if goal == start:
        return previous
    path = []
    step = goal
    while step != start:
        path.append(step)
        parent = parents[step]
        if parent is None:
            return None
        step = parent
    for step in path:
        if _safe(states[step], previous, limits):
            return states[step]
    return None


@lru_cache(maxsize=16)
def _actuator_grid(limits: ActuatorLimits) -> tuple[EffectiveTwist, ...]:
    maximum_wheel_kmh = (
        limits.max_speed_mps
        + WHEEL_SEPARATION_M * limits.max_yaw_rate_rps / 2.0) * 3.6
    level_count = int(math.ceil(maximum_wheel_kmh * 10.0)) + 1
    candidates = []
    for left_level in range(level_count):
        left_kmh = left_level / 10.0
        for right_level in range(level_count):
            right_kmh = right_level / 10.0
            linear = (left_kmh + right_kmh) / 7.2
            angular = (right_kmh - left_kmh) \
                / (3.6 * WHEEL_SEPARATION_M)
            candidate = effective_twist(linear, angular)
            if candidate is not None:
                candidates.append(candidate)
    return tuple(candidates)
