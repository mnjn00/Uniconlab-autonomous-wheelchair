from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from priest_constraints import DEFAULT_CONSTRAINT_TOLERANCES
    from priest_controller import DriveCommand, Pose2D, command_for
    from priest_feasibility import TrajectoryCertificate
    from priest_heading_pid import (
        HeadingPid,
        HeadingPidConfig,
        SteeringFeedback,
    )
    from priest_types import Plan
finally:
    sys.path.remove(str(SCRIPTS))


def test_pid_damps_a_turn_before_heading_error_crosses_zero() -> None:
    pid = HeadingPid(HeadingPidConfig(kp=1.0, ki=0.0, kd=0.4))

    undamped = pid.update(0.10, 0.0, 0.0)
    pid.reset()
    damped = pid.update(0.10, 0.0, 0.50)

    assert undamped > 0.0
    assert damped < 0.0


def test_pid_integral_is_bounded_and_resettable() -> None:
    pid = HeadingPid(HeadingPidConfig(
        kp=0.0, ki=1.0, kd=0.0, integral_limit_rad_s=0.20))

    for _ in range(40):
        output = pid.update(0.10, 0.0, 0.0)

    assert output == pytest.approx(0.20)
    assert pid.integral_error_rad_s == pytest.approx(0.20)
    pid.reset()
    assert pid.update(0.0, 0.0, 0.0) == 0.0


def test_pid_does_not_wind_up_while_output_is_saturated() -> None:
    pid = HeadingPid(HeadingPidConfig(
        kp=10.0, ki=1.0, kd=0.0, integral_limit_rad_s=1.0,
        max_output_rps=0.20))

    for _ in range(40):
        output = pid.update(1.0, 0.0, 0.0)

    assert output == pytest.approx(0.20)
    assert pid.integral_error_rad_s == 0.0


def test_pid_stabilizes_a_delayed_yaw_plant_without_repeated_reversals() -> None:
    pid = HeadingPid()
    yaw_rad, measured_yaw_rate_rps = -0.30, 0.0
    prior_sign, reversals = 0, 0

    for _ in range(80):
        command = pid.update(-yaw_rad, 0.0, measured_yaw_rate_rps)
        sign = 0 if abs(command) <= 0.02 else int(math.copysign(1, command))
        if sign and prior_sign and sign != prior_sign:
            reversals += 1
        if sign:
            prior_sign = sign
        measured_yaw_rate_rps += (command - measured_yaw_rate_rps) * 0.20
        yaw_rad += measured_yaw_rate_rps * 0.20

    assert reversals <= 1
    assert abs(yaw_rad) < 0.02


def test_actual_priest_controller_damps_delayed_yaw_without_reversal() -> None:
    plan = Plan(
        np.zeros(2), np.array([0.0, 6.0]), np.zeros(2),
        np.array([0.0, 20.0]), 0.0, 0.0, 1, 20.0,
        certificate=TrajectoryCertificate.clear(
            DEFAULT_CONSTRAINT_TOLERANCES))
    plan.velocity_xy_mps = np.tile(np.array([0.30, 0.0]), (2, 1))
    plan.yaw_rad = plan.yaw_rate_rps = np.zeros(2)
    pid = HeadingPid()
    x_m, y_m, yaw_rad, measured_yaw_rate_rps = 0.0, 0.0, -0.30, 0.0
    previous = DriveCommand(0.30, 0.0)
    prior_sign, reversals = 0, 0
    angular_commands: list[float] = []

    for step in range(80):
        command = command_for(
            plan, step * 0.20, Pose2D(x_m, y_m, yaw_rad), 0.30, previous,
            steering=SteeringFeedback(pid, measured_yaw_rate_rps))
        angular_commands.append(command.angular_z_rps)
        sign = 0 if abs(command.angular_z_rps) <= 0.05 else int(
            math.copysign(1, command.angular_z_rps))
        if sign and prior_sign and sign != prior_sign:
            reversals += 1
        if sign:
            prior_sign = sign
        measured_yaw_rate_rps += (
            command.angular_z_rps - measured_yaw_rate_rps) * 0.20
        yaw_rad += measured_yaw_rate_rps * 0.20
        x_m += 0.30 * math.cos(yaw_rad) * 0.20
        y_m += 0.30 * math.sin(yaw_rad) * 0.20
        previous = command

    assert reversals == 0
    assert max(abs(value) for value in angular_commands) < 0.35
    assert abs(yaw_rad) < 0.05
