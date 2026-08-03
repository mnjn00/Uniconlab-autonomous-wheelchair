"""Physical execution counterexamples at the final PRIEST boundary."""

from __future__ import annotations

import math
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
        command_acceleration_mps2,
        command_for,
    )
    from priest_constraints import DEFAULT_CONSTRAINT_TOLERANCES
    from priest_feasibility import TrajectoryCertificate, certify_trajectory
    from priest_planner import PriestPlanner
    from priest_terminal import terminal_correction
    from priest_types import Corridor, Plan
finally:
    sys.path.remove(str(SCRIPTS))


class StripBand:
    """A centre band already inset by the deployed 0.35 m proxy."""

    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        return np.abs(points[:, 1]) <= 0.25 + grace


class OpenBand:
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        del grace
        return np.ones(len(points), dtype=bool)


def certified_plan() -> Plan:
    return Plan(
        np.zeros(2), np.array([0.0, 1.0]), np.zeros(2),
        np.array([0.0, 2.0]), 0.0, 0.0, 1, 2.0,
        certificate=TrajectoryCertificate.clear(
            DEFAULT_CONSTRAINT_TOLERANCES))


def fast_straight_plan() -> Plan:
    plan = certified_plan()
    plan.velocity_xy_mps = np.tile(np.array([0.60, 0.0]), (2, 1))
    plan.yaw_rad = np.zeros(2)
    plan.yaw_rate_rps = np.zeros(2)
    return plan


def straight_corridor() -> Corridor:
    x = np.linspace(0.0, 5.0, 11)
    centres = np.stack([x, np.zeros_like(x)], axis=1)
    normals = np.tile(np.array([0.0, 1.0]), (len(x), 1))
    limits = np.ones(len(x))
    return Corridor(centres, normals, limits, limits)


def test_rotated_corners_cannot_pass_a_centre_only_band_check() -> None:
    planner = PriestPlanner(v_max=1.0, a_max=1.0, yaw_rate_max=1.0)
    points = np.array([[0.0, 0.22], [0.10, 0.32]])

    certificate = certify_trajectory(
        planner, points=points, times_s=np.array([0.0, 1.0]),
        band=StripBand(), obstacles=np.empty((0, 3)), band_grace_m=0.10)

    assert certificate.reason == "OUTSIDE_RUNTIME_BAND"


def test_slow_curving_plan_below_measured_turn_floor_is_refused() -> None:
    times = np.linspace(0.0, 2.0, 21)
    yaw_rate = 0.20
    radius = 0.50
    theta = yaw_rate * times
    points = np.stack([
        radius * np.sin(theta), radius * (1.0 - np.cos(theta))], axis=1)
    initial_yaw = math.atan2(
        points[1, 1] - points[0, 1], points[1, 0] - points[0, 0])

    certificate = certify_trajectory(
        PriestPlanner(v_max=1.0, a_max=1.0, yaw_rate_max=0.5),
        points=points, times_s=times, band=OpenBand(),
        obstacles=np.empty((0, 3)), initial_yaw_rad=initial_yaw)

    assert certificate.reason == "TURN_FLOOR_SPEED"


def test_turning_speed_floor_is_absolute_not_yaw_proportional() -> None:
    times = np.linspace(0.0, 2.0, 21)
    yaw_rate = 0.20
    radius = 0.75
    theta = yaw_rate * times
    points = np.stack([
        radius * np.sin(theta), radius * (1.0 - np.cos(theta))], axis=1)
    initial_yaw = math.atan2(
        points[1, 1] - points[0, 1], points[1, 0] - points[0, 0])

    certificate = certify_trajectory(
        PriestPlanner(v_max=1.0, a_max=1.0, yaw_rate_max=0.5),
        points=points, times_s=times, band=OpenBand(),
        obstacles=np.empty((0, 3)), initial_yaw_rad=initial_yaw)

    assert certificate.reason == "TURN_FLOOR_SPEED"


def test_large_heading_error_never_commands_turn_in_place() -> None:
    command = command_for(
        certified_plan(), 0.0, Pose2D(0.0, 0.0, math.pi / 2.0), 0.0,
        DriveCommand(0.0, 0.0))

    assert command == DriveCommand(0.0, 0.0, reason="HEADING_REPLAN")


def test_turn_waits_for_floor_then_uses_a_moving_arc() -> None:
    plan = certified_plan()
    pose = Pose2D(0.0, 0.0, 0.20)

    accelerating = command_for(
        plan, 0.0, pose, 0.0, DriveCommand(0.0, 0.0))
    turning = DriveCommand(0.30, 0.0)
    for _ in range(8):
        turning = command_for(
            plan, 0.0, pose, turning.linear_x_mps, turning)
        if turning.angular_z_rps < 0.0:
            break

    assert accelerating.linear_x_mps > 0.0
    assert accelerating.angular_z_rps == 0.0
    assert accelerating.reason == "TURN_ACCELERATING"
    assert turning.linear_x_mps > 0.0
    assert turning.angular_z_rps < 0.0
    required = 0.30 * abs(turning.angular_z_rps) / 0.50
    assert min(turning.linear_x_mps, 0.30) >= required


def test_turning_commands_bound_full_cartesian_acceleration() -> None:
    limits = ControllerLimits()
    previous = DriveCommand(7.0 / 12.0, 0.0)

    for _ in range(4):
        command = command_for(
            fast_straight_plan(), 0.0, Pose2D(0.0, -1.0, 0.0),
            0.60, previous, limits)
        acceleration = command_acceleration_mps2(
            previous, command, limits.control_period_s)
        assert acceleration <= limits.max_acceleration_mps2 + 1e-9
        previous = command


def test_dynamically_unsafe_previous_command_fails_closed() -> None:
    command = command_for(
        fast_straight_plan(), 0.0, Pose2D(0.0, -1.0, 0.0), 0.60,
        DriveCommand(0.60, 0.50))

    assert command == DriveCommand(0.0, 0.0, reason="INVALID_DYNAMICS")


def test_normal_plan_end_brakes_before_publishing_exact_zero() -> None:
    limits = ControllerLimits()
    previous = DriveCommand(0.30, 0.0)

    braking = command_for(
        certified_plan(), 2.0, Pose2D(1.0, 0.0, 0.0),
        0.30, previous, limits)

    assert braking.reason == "TERMINAL_BRAKING"
    assert 0.0 < braking.linear_x_mps < previous.linear_x_mps
    assert command_acceleration_mps2(
        previous, braking, limits.control_period_s) \
        <= limits.max_deceleration_mps2 + 1e-9


def test_terminal_braking_latches_early_enough_to_stop_at_endpoint() -> None:
    limits = ControllerLimits()
    plan = certified_plan()
    pose_x, elapsed = 0.40, 0.80
    previous = DriveCommand(0.35, 0.0)
    braking_started = False

    for _ in range(30):
        command = command_for(
            plan, elapsed, Pose2D(pose_x, 0.0, 0.0),
            previous.linear_x_mps, previous, limits)
        if braking_started and not command.done:
            assert command.reason == "TERMINAL_BRAKING"
        braking_started |= command.reason == "TERMINAL_BRAKING"
        assert command_acceleration_mps2(
            previous, command, limits.control_period_s) \
            <= limits.max_deceleration_mps2 + 1e-9
        pose_x += command.linear_x_mps * limits.control_period_s
        elapsed += limits.control_period_s
        previous = command
        if command.done:
            break

    assert braking_started and previous.done
    assert abs(1.0 - pose_x) <= limits.goal_tolerance_m


def test_cartesian_metric_counts_centripetal_acceleration() -> None:
    acceleration = command_acceleration_mps2(
        DriveCommand(0.60, 0.0), DriveCommand(0.60, 0.50), 0.10)

    assert acceleration > 0.30


def test_actual_yaw_boundary_still_finds_a_straight_from_rest_plan() -> None:
    corridor = straight_corridor()
    planner = PriestPlanner(runtime_band=OpenBand(), seed=0)

    plan = planner.plan(
        corridor.centres[0], np.zeros(2), np.array([0.18, 0.0]),
        corridor, [], initial_yaw_rad=0.0)

    assert plan.usable
    assert abs(plan.yaw_rad[0]) < 1e-6


def test_aligned_moving_terminal_correction_decelerates_without_turning() -> None:
    planner = PriestPlanner(runtime_band=OpenBand())

    plan = terminal_correction(
        planner, start_xy=np.array([4.15, 0.0]),
        velocity_xy_mps=np.array([0.35, 0.0]),
        goal_xy=np.array([5.0, 0.0]), initial_yaw_rad=0.0,
        band=OpenBand(), obstacles=np.empty((0, 3)))

    assert plan is not None and plan.usable
    assert np.all(plan.velocity_xy_mps[:, 0] >= -1e-9)
    assert np.max(np.abs(plan.yaw_rate_rps)) == 0.0
    assert plan.certificate.max_acceleration_mps2 \
        <= planner.a_max + planner.CONSTRAINT_TOLERANCES.acceleration_mps2
    assert np.linalg.norm(plan.points()[-1] - np.array([5.0, 0.0])) < 1e-9


@pytest.mark.parametrize("start", [
    np.array([5.0, 0.20]),
    np.array([5.10, 0.20]),
])
def test_terminal_plane_offset_requests_a_certified_correction(
        start: np.ndarray) -> None:
    corridor = straight_corridor()
    planner = PriestPlanner(runtime_band=OpenBand())
    calls: list[tuple[float, float]] = []

    def correction(
            current: np.ndarray, velocity: np.ndarray,
            acceleration: np.ndarray, active: Corridor,
            obstacles: list[list[float]], start_arc: float,
            reach: float, initial_yaw_rad: float | None = None) -> Plan:
        del velocity, acceleration, active, obstacles, initial_yaw_rad
        calls.append((start_arc, reach))
        return Plan(
            np.zeros(2), np.array([current[0], 5.0]),
            np.array([current[1], 0.0]), np.array([0.0, 2.0]),
            0.0, 0.0, 1, 2.0,
            certificate=TrajectoryCertificate.clear(
                DEFAULT_CONSTRAINT_TOLERANCES))

    planner.attempt = correction
    plan = planner.plan(
        start, np.zeros(2), np.zeros(2), corridor, [],
        initial_yaw_rad=0.0)

    assert plan.usable
    assert calls and calls[0][0] < corridor.length_m


@pytest.mark.parametrize(("start", "yaw"), [
    (np.array([5.0, 0.20]), -math.pi / 2.0),
    (np.array([5.10, 0.20]), math.atan2(-0.20, -0.10)),
])
def test_reachable_terminal_offset_gets_an_actual_certified_plan(
        start: np.ndarray, yaw: float) -> None:
    corridor = straight_corridor()
    direction = np.array([math.cos(yaw), math.sin(yaw)])

    plan = PriestPlanner(runtime_band=OpenBand(), seed=0).plan(
        start, np.zeros(2), 0.18 * direction, corridor, [],
        initial_yaw_rad=yaw)

    assert plan.usable
    assert np.linalg.norm(plan.points()[-1] - corridor.centres[-1]) < 1e-6
    assert plan.certificate is not None and plan.certificate.usable
