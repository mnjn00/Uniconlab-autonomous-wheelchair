"""Timed, ROS-free differential-drive execution of a PRIEST plan."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from priest_controller import (
        ControllerLimits,
        DriveCommand,
        Pose2D,
        command_for,
    )
    from priest_types import Plan
finally:
    sys.path.remove(str(SCRIPTS))


def plan_with(
        points: list[tuple[float, float]],
        times: list[float],
        reason: str = "") -> Plan:
    xy = np.asarray(points, dtype=np.float64)
    return Plan(
        xi=np.zeros(2),
        x=xy[:, 0],
        y=xy[:, 1],
        times=np.asarray(times, dtype=np.float64),
        residual=0.0,
        cost=0.0,
        feasible_samples=1,
        horizon_s=float(times[-1]),
        reason=reason,
    )


def relaxed_limits() -> ControllerLimits:
    return ControllerLimits(
        max_speed_mps=2.0,
        max_acceleration_mps2=10.0,
        max_deceleration_mps2=10.0,
        max_yaw_rate_rps=2.0,
        max_yaw_acceleration_rps2=10.0,
        control_period_s=0.1,
        goal_tolerance_m=0.01,
        turn_in_place_rad=0.6,
    )


def stopped() -> DriveCommand:
    return DriveCommand(0.0, 0.0)


def test_time_indexed_reference_changes_when_only_plan_times_change() -> None:
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    pose = Pose2D(0.0, 0.0, 0.0)

    fast = command_for(
        plan_with(points, [0.0, 1.0, 2.0]), 0.5, pose, 0.0, stopped(),
        relaxed_limits())
    slow = command_for(
        plan_with(points, [0.0, 2.0, 4.0]), 0.5, pose, 0.0, stopped(),
        relaxed_limits())

    assert fast.linear_x_mps > slow.linear_x_mps


def test_output_has_only_differential_drive_motion_components() -> None:
    command = command_for(
        plan_with([(0.0, 0.0), (1.0, 0.0)], [0.0, 2.0]),
        0.2, Pose2D(0.0, 0.0, 0.0), 0.0, stopped())

    assert command.linear_x_mps >= 0.0
    assert abs(command.angular_z_rps) <= 0.5
    assert not hasattr(command, "linear_y_mps")


def test_from_rest_body_yaw_mismatch_turns_then_resumes_translation() -> None:
    plan = plan_with([(0.0, 0.0), (1.0, 0.0)], [0.0, 2.0])
    limits = ControllerLimits()

    turning = command_for(
        plan, 0.0, Pose2D(0.0, 0.0, np.pi / 2.0), 0.0, stopped(),
        limits)
    resumed = command_for(
        plan, 0.0, Pose2D(0.0, 0.0, 0.0), 0.0, turning, limits)

    assert turning.linear_x_mps == 0.0
    assert turning.angular_z_rps < 0.0
    assert turning.reason == "ALIGNING"
    assert resumed.linear_x_mps > 0.0
    assert resumed.reason == ""


def test_speed_slew_and_yaw_limit_are_bounded_per_control_period() -> None:
    limits = ControllerLimits()
    command = command_for(
        plan_with([(0.0, 0.0), (2.0, 0.0)], [0.0, 1.0]),
        0.0, Pose2D(0.0, 0.0, 0.4), 0.0, stopped(), limits)

    assert command.linear_x_mps <= (
        limits.max_acceleration_mps2 * limits.control_period_s + 1e-12)
    assert abs(command.angular_z_rps) <= (
        limits.max_yaw_acceleration_rps2 * limits.control_period_s + 1e-12)
    assert abs(command.angular_z_rps) <= limits.max_yaw_rate_rps


def test_time_indexed_turn_produces_bounded_angular_command() -> None:
    command = command_for(
        plan_with(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            [0.0, 1.0, 2.0]),
        0.75, Pose2D(0.75, 0.0, 0.0), 0.5,
        DriveCommand(0.5, 0.0), relaxed_limits())

    assert command.angular_z_rps > 0.0
    assert command.angular_z_rps <= relaxed_limits().max_yaw_rate_rps


def test_turning_from_motion_brakes_at_the_deceleration_limit() -> None:
    limits = ControllerLimits()
    previous = DriveCommand(0.5, 0.0)

    command = command_for(
        plan_with([(0.0, 0.0), (1.0, 0.0)], [0.0, 2.0]),
        0.0, Pose2D(0.0, 0.0, np.pi / 2.0), 0.5, previous, limits)

    assert command.reason == "ALIGNING"
    assert command.linear_x_mps == pytest.approx(
        previous.linear_x_mps
        - limits.max_deceleration_mps2 * limits.control_period_s)
    assert command.angular_z_rps == 0.0


def test_measured_motion_suppresses_turn_when_previous_command_is_stale() -> None:
    command = command_for(
        plan_with([(0.0, 0.0), (1.0, 0.0)], [0.0, 2.0]),
        0.0, Pose2D(0.0, 0.0, np.pi / 2.0), 0.5, stopped())

    assert command.reason == "ALIGNING"
    assert command.linear_x_mps == 0.0
    assert command.angular_z_rps == 0.0


def test_reference_is_continuous_across_a_timed_turn_knot() -> None:
    plan = plan_with(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        [0.0, 1.0, 2.0])
    limits = ControllerLimits(
        max_speed_mps=5.0,
        max_acceleration_mps2=100.0,
        max_deceleration_mps2=100.0,
        max_yaw_rate_rps=5.0,
        max_yaw_acceleration_rps2=100.0,
        control_period_s=0.1,
        goal_tolerance_m=0.001,
        turn_in_place_rad=float(np.pi),
    )
    pose = Pose2D(1.0, 0.0, np.pi / 4.0)
    previous = DriveCommand(0.7, 0.0)

    before = command_for(plan, 0.999, pose, 0.7, previous, limits)
    after = command_for(plan, 1.001, pose, 0.7, previous, limits)

    assert before.angular_z_rps > 0.0
    assert after.angular_z_rps > 0.0
    assert abs(before.angular_z_rps - after.angular_z_rps) < 0.02


def test_all_stationary_reference_is_rejected() -> None:
    command = command_for(
        plan_with([(0.0, 0.0), (0.0, 0.0)], [0.0, 1.0]),
        0.5, Pose2D(1.0, 0.0, 0.0), 0.0, stopped())

    assert command == DriveCommand(
        0.0, 0.0, reason="INVALID_PLAN", done=False)


def test_hard_speed_and_yaw_limits_saturate_independently_of_slew() -> None:
    limits = ControllerLimits(
        max_speed_mps=0.2,
        max_acceleration_mps2=100.0,
        max_deceleration_mps2=100.0,
        max_yaw_rate_rps=0.1,
        max_yaw_acceleration_rps2=100.0,
        control_period_s=0.1,
        goal_tolerance_m=0.001,
        turn_in_place_rad=float(np.pi),
    )

    command = command_for(
        plan_with(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            [0.0, 1.0, 2.0]),
        0.5, Pose2D(0.5, 0.0, 0.0), 0.2,
        DriveCommand(0.2, 0.0), limits)

    assert command.linear_x_mps == limits.max_speed_mps
    assert command.angular_z_rps == limits.max_yaw_rate_rps


@pytest.mark.parametrize("times", [
    [0.0, 0.0],
    [0.0, -1.0],
    [0.0, np.nan],
    [1.0, 2.0],
])
def test_invalid_times_fail_closed(times: list[float]) -> None:
    command = command_for(
        plan_with([(0.0, 0.0), (1.0, 0.0)], times),
        0.0, Pose2D(0.0, 0.0, 0.0), 0.0, stopped())

    assert command == DriveCommand(
        0.0, 0.0, reason="INVALID_PLAN", done=False)


def test_nonfinite_pose_or_elapsed_fails_closed() -> None:
    plan = plan_with([(0.0, 0.0), (1.0, 0.0)], [0.0, 2.0])

    bad_pose = command_for(
        plan, 0.0, Pose2D(np.nan, 0.0, 0.0), 0.0, stopped())
    bad_time = command_for(
        plan, -0.1, Pose2D(0.0, 0.0, 0.0), 0.0, stopped())

    assert bad_pose.reason == "INVALID_STATE"
    assert bad_pose.linear_x_mps == bad_pose.angular_z_rps == 0.0
    assert bad_time.reason == "INVALID_STATE"
    assert bad_time.linear_x_mps == bad_time.angular_z_rps == 0.0


def test_plan_refusal_and_goal_stop_are_exact_zero() -> None:
    refused = command_for(
        plan_with(
            [(0.0, 0.0), (1.0, 0.0)], [0.0, 2.0],
            reason="NO_FEASIBLE_TRAJECTORY"),
        0.0, Pose2D(0.0, 0.0, 0.0), 0.0, stopped())
    ended = command_for(
        plan_with([(0.0, 0.0), (1.0, 0.0)], [0.0, 2.0]),
        2.0, Pose2D(0.9, 0.0, 0.0), 0.2,
        DriveCommand(0.2, 0.1))
    at_goal = command_for(
        plan_with([(0.0, 0.0), (1.0, 0.0)], [0.0, 2.0]),
        1.0, Pose2D(0.99, 0.0, 0.0), 0.2,
        DriveCommand(0.2, 0.1))

    assert refused == DriveCommand(
        0.0, 0.0, reason="NO_FEASIBLE_TRAJECTORY", done=False)
    assert ended == DriveCommand(
        0.0, 0.0, reason="AT_PLAN_END", done=True)
    assert at_goal == DriveCommand(
        0.0, 0.0, reason="AT_GOAL", done=True)


def test_planner_at_goal_null_plan_is_done_and_exact_zero() -> None:
    plan = Plan(
        None, None, None, None, 0.0, 0.0, 0, 0.0, reason="AT_GOAL")

    command = command_for(
        plan, 0.0, Pose2D(0.0, 0.0, 0.0), 0.0, stopped())

    assert command == DriveCommand(
        0.0, 0.0, reason="AT_GOAL", done=True)
