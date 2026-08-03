"""Terminal viability across a late quantized yaw transition."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from priest_constraints import DEFAULT_CONSTRAINT_TOLERANCES
    from priest_controller import DriveCommand, Pose2D, command_for
    from priest_feasibility import TrajectoryCertificate
    import priest_planner as planner_module
    from priest_planner import PriestPlanner
    from priest_types import Corridor, Plan
finally:
    sys.path.remove(str(SCRIPTS))


def _late_turn_plan(start_x_m: float) -> Plan:
    speed_mps = 5.0 / 12.0
    times = np.array([0.0, 0.2, 4.0])
    x = np.array([start_x_m, start_x_m + speed_mps * 0.2, 1.0])
    plan = Plan(
        np.zeros(2), x, np.zeros(3), times, 0.0, 0.0, 1, 4.0,
        certificate=TrajectoryCertificate.clear(
            DEFAULT_CONSTRAINT_TOLERANCES))
    plan.velocity_xy_mps = np.tile(np.array([speed_mps, 0.0]), (3, 1))
    plan.yaw_rad = np.zeros(3)
    plan.yaw_rate_rps = np.array([
        0.0, 0.102880658436214, 0.102880658436214])
    return plan


def _advance(pose: Pose2D, command: DriveCommand) -> Pose2D:
    period_s = 0.2
    if abs(command.angular_z_rps) <= 1e-12:
        return Pose2D(
            pose.x_m + command.linear_x_mps * math.cos(pose.yaw_rad)
            * period_s,
            pose.y_m + command.linear_x_mps * math.sin(pose.yaw_rad)
            * period_s,
            pose.yaw_rad)
    next_yaw = pose.yaw_rad + command.angular_z_rps * period_s
    radius = command.linear_x_mps / command.angular_z_rps
    return Pose2D(
        pose.x_m + radius * (math.sin(next_yaw) - math.sin(pose.yaw_rad)),
        pose.y_m - radius * (math.cos(next_yaw) - math.cos(pose.yaw_rad)),
        next_yaw)


class OpenBand:
    def contains_many(
            self, points: np.ndarray, grace: float = 0.0) -> np.ndarray:
        del grace
        return np.ones(len(points), dtype=bool)


def test_late_yaw_transition_preserves_a_reachable_terminal_stop() -> None:
    pose = Pose2D(1.0 - 0.74315, 0.0, 0.0)
    plan = _late_turn_plan(pose.x_m)
    previous = DriveCommand(5.0 / 12.0, 0.0)
    elapsed_s = 0.0

    for _ in range(30):
        command = command_for(
            plan, elapsed_s, pose, previous.linear_x_mps, previous)
        pose = _advance(pose, command)
        elapsed_s += 0.2
        previous = command
        if command.done:
            break

    assert previous.done
    assert math.hypot(1.0 - pose.x_m, pose.y_m) <= 0.05


def test_exact_goal_tolerance_boundary_finishes_without_replan() -> None:
    speed_mps = 10.0 / 36.0
    plan = _late_turn_plan(0.8)
    plan.x, plan.times = np.array([0.8, 1.0]), np.array([0.0, 4.0])
    plan.y = plan.yaw_rad = plan.yaw_rate_rps = np.zeros(2)
    plan.velocity_xy_mps = np.tile(np.array([speed_mps, 0.0]), (2, 1))
    pose, previous = Pose2D(0.8, 0.0, 0.0), DriveCommand(speed_mps, 0.0)

    for step in range(20):
        command = command_for(
            plan, step * 0.2, pose, previous.linear_x_mps, previous)
        pose, previous = _advance(pose, command), command
        if command.done:
            break

    assert previous.done and previous.reason == "AT_PLAN_END"
    assert math.hypot(1.0 - pose.x_m, pose.y_m) <= 0.05 + 1e-9


def test_planner_uses_the_same_numeric_goal_boundary(
        monkeypatch: pytest.MonkeyPatch) -> None:
    def unexpected_correction(
            planner: PriestPlanner, *, start_xy: np.ndarray,
            velocity_xy_mps: np.ndarray, goal_xy: np.ndarray,
            initial_yaw_rad: Optional[float], band: OpenBand,
            obstacles: np.ndarray) -> Plan:
        del planner, start_xy, velocity_xy_mps, goal_xy
        del initial_yaw_rad, band, obstacles
        raise AssertionError("goal boundary reached terminal search")

    monkeypatch.setattr(
        planner_module, "terminal_correction", unexpected_correction)
    centres = np.array([[0.0, 0.0], [1.0, 0.0]])
    corridor = Corridor(
        centres, np.tile(np.array([0.0, 1.0]), (2, 1)),
        np.ones(2), np.ones(2))
    plan = PriestPlanner(runtime_band=OpenBand()).plan(
        np.array([1.05, 0.0]), np.zeros(2), np.zeros(2), corridor, [],
        initial_yaw_rad=0.0)

    assert plan.reason == "AT_GOAL"
