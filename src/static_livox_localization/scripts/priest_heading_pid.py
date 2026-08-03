from __future__ import annotations

import math
from typing import Final, NamedTuple

from priest_control_types import ControllerLimits


class HeadingPidConfig(NamedTuple):
    kp: float = 1.0
    ki: float = 0.08
    kd: float = 0.35
    integral_limit_rad_s: float = 0.30
    derivative_filter_tau_s: float = 0.40
    period_s: float = 0.20
    max_output_rps: float = 0.50


DEFAULT_HEADING_PID_CONFIG: Final = HeadingPidConfig()


class HeadingPidConfigError(ValueError):
    pass


class HeadingPidInputError(ValueError):
    pass


class HeadingPid:
    """Mutable filtered PID state reset at every hold and replan boundary."""

    __slots__ = ("config", "_integral_error_rad_s", "_filtered_yaw_rate_rps")

    def __init__(
            self,
            config: HeadingPidConfig = DEFAULT_HEADING_PID_CONFIG) -> None:
        values = tuple(config)
        if not all(math.isfinite(value) for value in values):
            raise HeadingPidConfigError("PID configuration must be finite")
        if min(config.kp, config.ki, config.kd,
               config.integral_limit_rad_s,
               config.derivative_filter_tau_s) < 0.0 \
                or config.period_s <= 0.0 or config.max_output_rps <= 0.0:
            raise HeadingPidConfigError("PID limits and period must be valid")
        self.config = config
        self._integral_error_rad_s = 0.0
        self._filtered_yaw_rate_rps: float | None = None

    @property
    def integral_error_rad_s(self) -> float:
        return self._integral_error_rad_s

    def reset(self) -> None:
        self._integral_error_rad_s = 0.0
        self._filtered_yaw_rate_rps = None

    def update(
            self,
            heading_error_rad: float,
            reference_yaw_rate_rps: float,
            measured_yaw_rate_rps: float) -> float:
        values = (
            heading_error_rad, reference_yaw_rate_rps,
            measured_yaw_rate_rps)
        if not all(math.isfinite(value) for value in values):
            raise HeadingPidInputError("PID inputs must be finite")

        filtered = self._filtered_yaw_rate_rps
        if filtered is None:
            filtered = measured_yaw_rate_rps
        else:
            tau = self.config.derivative_filter_tau_s
            alpha = 1.0 if tau == 0.0 else \
                self.config.period_s / (tau + self.config.period_s)
            filtered += alpha * (measured_yaw_rate_rps - filtered)
        self._filtered_yaw_rate_rps = filtered

        integral_limit = self.config.integral_limit_rad_s
        candidate_integral = min(max(
            self._integral_error_rad_s
            + heading_error_rad * self.config.period_s,
            -integral_limit), integral_limit)
        candidate_output = self._output(
            heading_error_rad, reference_yaw_rate_rps,
            filtered, candidate_integral)
        bounded_output = self._bounded(candidate_output)
        is_saturated = not math.isclose(
            candidate_output, bounded_output, rel_tol=0.0, abs_tol=1e-12)
        if not is_saturated or candidate_output * heading_error_rad <= 0.0:
            self._integral_error_rad_s = candidate_integral
        return self._bounded(self._output(
            heading_error_rad, reference_yaw_rate_rps, filtered,
            self._integral_error_rad_s))

    def _output(
            self,
            heading_error_rad: float,
            reference_yaw_rate_rps: float,
            filtered_yaw_rate_rps: float,
            integral_error_rad_s: float) -> float:
        return (
            reference_yaw_rate_rps
            + self.config.kp * heading_error_rad
            + self.config.ki * integral_error_rad_s
            + self.config.kd
            * (reference_yaw_rate_rps - filtered_yaw_rate_rps))

    def _bounded(self, value: float) -> float:
        limit = self.config.max_output_rps
        return min(max(value, -limit), limit)


class SteeringFeedback(NamedTuple):
    pid: HeadingPid
    measured_yaw_rate_rps: float


def angular_slew(
        desired_rps: float,
        previous_rps: float,
        limits: ControllerLimits) -> float:
    step = limits.max_yaw_acceleration_rps2 * limits.control_period_s
    bounded = min(max(desired_rps, previous_rps - step), previous_rps + step)
    return min(max(bounded, -limits.max_yaw_rate_rps),
               limits.max_yaw_rate_rps)
