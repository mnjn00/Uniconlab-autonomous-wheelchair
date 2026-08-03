#!/usr/bin/env python3
"""Pure deployed wheel-command encoding and effective-twist model."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


MAX_LINEAR_MPS = 1.5
MAX_ANGULAR_RAD_S = 0.6
WHEEL_SEPARATION_M = 0.54
COUNTS_PER_KMH = 10.0
MAGNITUDE_OFFSET = 33
TURN_AUTHORITY_KMH = 1.3
TURN_AUTHORITY_MAX_LINEAR_MPS = 0.30
YAW_DEADBAND_RAD_S = 0.05
STOP_COMMAND = (83, 33, 83, 33, 79)


@dataclass(frozen=True)
class EffectiveTwist:
    __slots__ = ("linear_x_mps", "angular_z_rps", "left_kmh", "right_kmh")

    linear_x_mps: float
    angular_z_rps: float
    left_kmh: float
    right_kmh: float


def required_turn_linear_mps(angular_z_rps: float) -> float:
    yaw_rate = abs(float(angular_z_rps))
    if not math.isfinite(yaw_rate):
        raise ValueError("yaw rate must be finite")
    if yaw_rate <= YAW_DEADBAND_RAD_S:
        return 0.0
    authority = TURN_AUTHORITY_KMH / 3.6 \
        - WHEEL_SEPARATION_M * yaw_rate / 2.0
    return max(TURN_AUTHORITY_MAX_LINEAR_MPS, authority)


def _encode_speed(speed_kmh: float) -> Optional[tuple[int, int]]:
    direction = 67 if speed_kmh > 0.0 else 87 if speed_kmh < 0.0 else 83
    magnitude = int(round(abs(speed_kmh) * COUNTS_PER_KMH)) \
        + MAGNITUDE_OFFSET
    if magnitude > 127:
        return None
    return direction, magnitude


def encode_wheel_command(
        linear_x_mps: float,
        angular_z_rps: float) -> Optional[tuple[int, int, int, int, int]]:
    try:
        linear = float(linear_x_mps)
        angular = float(angular_z_rps)
    except (TypeError, ValueError, OverflowError):
        return None
    if (not math.isfinite(linear) or not math.isfinite(angular)
            or linear < 0.0 or linear > MAX_LINEAR_MPS
            or abs(angular) > MAX_ANGULAR_RAD_S):
        return None
    half_difference = angular * WHEEL_SEPARATION_M / 2.0
    left = (linear - half_difference) * 3.6
    right = (linear + half_difference) * 3.6
    if abs(angular) > YAW_DEADBAND_RAD_S:
        fastest = max(abs(left), abs(right))
        headroom = (TURN_AUTHORITY_MAX_LINEAR_MPS - linear) * 3.6
        boost = min(max(TURN_AUTHORITY_KMH - fastest, 0.0),
                    max(headroom, 0.0))
        left += boost
        right += boost
    encoded_left = _encode_speed(left)
    encoded_right = _encode_speed(right)
    if encoded_left is None or encoded_right is None:
        return None
    effective_left = _decode_speed(*encoded_left)
    effective_right = _decode_speed(*encoded_right)
    if effective_left is None or effective_right is None:
        return None
    effective_yaw = (effective_right - effective_left) \
        / 3.6 / WHEEL_SEPARATION_M
    fastest_encoded = max(abs(effective_left), abs(effective_right))
    if abs(effective_yaw) > YAW_DEADBAND_RAD_S \
            and fastest_encoded + 1e-9 < TURN_AUTHORITY_KMH:
        return None
    return (encoded_left[0], encoded_left[1],
            encoded_right[0], encoded_right[1], 79)


def _decode_speed(direction: int, magnitude: int) -> Optional[float]:
    if direction not in (67, 87, 83) \
            or not MAGNITUDE_OFFSET <= magnitude <= 127:
        return None
    speed = (magnitude - MAGNITUDE_OFFSET) / COUNTS_PER_KMH
    if direction == 83 and speed != 0.0:
        return None
    return -speed if direction == 87 else speed


def decode_wheel_command(
        command: tuple[int, int, int, int, int]) -> Optional[EffectiveTwist]:
    if len(command) != 5 or command[4] != 79:
        return None
    left = _decode_speed(command[0], command[1])
    right = _decode_speed(command[2], command[3])
    if left is None or right is None:
        return None
    linear = (left + right) / 2.0 / 3.6
    angular = (right - left) / 3.6 / WHEEL_SEPARATION_M
    return EffectiveTwist(linear, angular, left, right)


def effective_twist(
        linear_x_mps: float,
        angular_z_rps: float) -> Optional[EffectiveTwist]:
    command = encode_wheel_command(linear_x_mps, angular_z_rps)
    return None if command is None else decode_wheel_command(command)
