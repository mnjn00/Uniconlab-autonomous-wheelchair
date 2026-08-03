#!/usr/bin/env python3
"""Deterministic software-only PRIEST differential-drive rollout."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Optional

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from cluster_guard import Summary
    from priest_constraints import (
        CANONICAL_FOOTPRINT,
        COMMAND_RETENTION_HORIZON_S,
        within_goal_tolerance,
    )
    from priest_controller import (
        DriveCommand,
        Pose2D,
        command_acceleration_mps2,
        command_for,
    )
    from priest_execution_safety import (
        differential_drive_arc,
        oriented_footprint_contained,
    )
    from priest_planner import Corridor, Plan, PriestPlanner
    from priest_runtime import OBSTACLE_WAIT, wait_reason
    from safety_band import SafetyBand
finally:
    sys.path.remove(str(SCRIPTS))


DT_S = 0.2
MAX_CYCLES = 900
GOAL_TOLERANCE_M = 0.05


def _band_payload() -> dict:
    stations = []
    x_values = np.linspace(0.0, 3.0, 31)
    y_values = 0.12 * np.sin(np.pi * x_values / 3.0)
    heading = np.arctan2(
        np.gradient(y_values, x_values), np.ones(len(x_values)))
    for x_m, y_m, yaw_rad in zip(x_values, y_values, heading):
        stations.append({
            "x": float(x_m), "y": float(y_m),
            "heading_deg": math.degrees(float(yaw_rad)),
            "left_m": 1.675, "right_m": 1.675,
            "left_drop_m": 0.0, "right_drop_m": 0.0,
            "left_kind": "open", "right_kind": "open",
        })
    return {"stations": stations}


def _safety_band() -> SafetyBand:
    descriptor, path = tempfile.mkstemp(suffix="-priest-band.json")
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(_band_payload(), handle)
        return SafetyBand(path)
    finally:
        os.unlink(path)


def _transform(x_m: float, y_m: float, yaw_rad: float) -> np.ndarray:
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    value = np.eye(4)
    value[:2, :2] = np.array([[cosine, -sine], [sine, cosine]])
    value[:2, 3] = [x_m, y_m]
    return value


def _moving_summary() -> Summary:
    return Summary(0.0, "OK", [{
        "x": 0.9, "y": 0.65, "size": [0.4, 0.8, 1.2],
        "map_x": 0.9, "map_y": 0.65,
        "motion": "moving",
    }])


def _moving_hold(
        state: np.ndarray,
        plan: Optional[Plan],
        plan_index: int,
) -> bool:
    trajectory = state[None, :2] if plan is None else plan.points()
    reason = wait_reason(
        _moving_summary(), _transform(*state), np.zeros(3), np.eye(3),
        state[:2], trajectory, trajectory_start_index=plan_index)
    return reason == OBSTACLE_WAIT


def _plan(
        planner: PriestPlanner,
        corridor: Corridor,
        state: np.ndarray,
        speed_mps: float,
        obstacles: np.ndarray,
) -> Plan:
    direction = np.array([math.cos(state[2]), math.sin(state[2])])
    velocity = speed_mps * direction
    acceleration = np.zeros(2) if speed_mps > 0.02 \
        else planner.a_max * direction
    return planner.plan(
        state[:2], velocity, acceleration, corridor, obstacles,
        initial_yaw_rad=float(state[2]))


def _plan_index(plan: Plan, elapsed_s: float) -> int:
    return int(np.clip(
        np.searchsorted(plan.times, elapsed_s, side="right") - 1,
        0, len(plan.times) - 1))


def _command_accelerations(
        previous: DriveCommand,
        command: DriveCommand) -> tuple[float, float]:
    value = command_acceleration_mps2(previous, command, DT_S)
    if command.reason in (
            "", "TURN_ACCELERATING", "TERMINAL_BRAKING"):
        return value, 0.0
    return 0.0, value


def run_validation(seed: int = 0) -> dict[str, float | int | bool | str]:
    band = _safety_band()
    corridor = Corridor(band.xy, band.normals, band.left, band.right)
    planner = PriestPlanner(
        runtime_band=band, seed=seed, batch=120, elite=12,
        constraint_elite=40, iterations=8, projection_iterations=12)
    static_obstacles = np.array([[1.50, 1.10, 0.10]])
    state = np.array([
        corridor.centres[0, 0], corridor.centres[0, 1],
        math.atan2(-corridor.normals[0, 0], corridor.normals[0, 1])])
    goal = corridor.centres[-1]
    plan: Optional[Plan] = None
    plan_started_s = 0.0
    previous = DriveCommand(0.0, 0.0)
    measured_speed = 0.0
    moving_holds = 0
    planner_refusals = 0
    state_updates = 0
    max_speed = max_acceleration = max_yaw_rate = 0.0
    max_emergency_command_rate = 0.0
    terminal_stop_deceleration = 0.0
    min_clearance = float("inf")
    band_violation = 0.0
    stopped_at_goal = False
    final_command = DriveCommand(0.0, 0.0)

    for cycle in range(MAX_CYCLES):
        now_s = cycle * DT_S
        goal_error = float(np.linalg.norm(state[:2] - goal))
        terminal_complete = False
        inject_actor = 12 <= cycle < 15
        elapsed = now_s - plan_started_s
        index = 0 if plan is None else _plan_index(plan, elapsed)
        if inject_actor and _moving_hold(state, plan, index):
            command = DriveCommand(0.0, 0.0, reason=OBSTACLE_WAIT)
            moving_holds += 1
            plan = None
        else:
            if plan is None:
                candidate = _plan(
                    planner, corridor, state, measured_speed, static_obstacles)
                if candidate.usable:
                    plan = candidate
                    plan_started_s = now_s
                    previous = DriveCommand(0.0, 0.0)
                    elapsed = 0.0
                else:
                    planner_refusals += 1
                    terminal_complete = candidate.reason == "AT_GOAL" \
                        and measured_speed <= 0.02
                    command = DriveCommand(
                        0.0, 0.0, reason=candidate.reason,
                        done=terminal_complete)
            if plan is not None:
                command = command_for(
                    plan, elapsed, Pose2D(*state), measured_speed, previous)
                terminal_complete = command.done and command.reason in (
                    "AT_GOAL", "AT_PLAN_END")
                if command.reason not in (
                        "", "TURN_ACCELERATING", "TERMINAL_BRAKING") \
                        and not terminal_complete:
                    command = DriveCommand(0.0, 0.0, reason=command.reason)
                    plan = None
                elif terminal_complete:
                    plan = None

        tracking_acceleration, emergency_command_rate = \
            _command_accelerations(previous, command)
        max_acceleration = max(max_acceleration, tracking_acceleration)
        max_emergency_command_rate = max(
            max_emergency_command_rate, emergency_command_rate)
        if command.reason == "TERMINAL_BRAKING":
            terminal_stop_deceleration = max(
                terminal_stop_deceleration, tracking_acceleration)
        swept_xy, swept_yaw = differential_drive_arc(
            *state, command.linear_x_mps, command.angular_z_rps,
            COMMAND_RETENTION_HORIZON_S)
        swept_ok = oriented_footprint_contained(
            band, swept_xy, swept_yaw, 0.10)
        if not bool(swept_ok.all()):
            band_violation = float("inf")
        distance = np.linalg.norm(
            swept_xy - static_obstacles[0, :2], axis=1).min()
        required = (
            static_obstacles[0, 2]
            + CANONICAL_FOOTPRINT.circumscribed_radius_m
            + CANONICAL_FOOTPRINT.planning_margin_m)
        min_clearance = min(min_clearance, float(distance - required))
        state[0] += command.linear_x_mps * math.cos(state[2]) * DT_S
        state[1] += command.linear_x_mps * math.sin(state[2]) * DT_S
        state[2] += command.angular_z_rps * DT_S
        state_updates += 1
        measured_speed = command.linear_x_mps
        previous = command
        final_command = command
        max_speed = max(max_speed, measured_speed)
        max_yaw_rate = max(max_yaw_rate, abs(command.angular_z_rps))
        footprint_ok = oriented_footprint_contained(
            band, state[None, :2], np.array([state[2]]), 0.10)
        if not bool(footprint_ok[0]):
            band_violation = float("inf")
        if terminal_complete and within_goal_tolerance(
                float(np.linalg.norm(state[:2] - goal)), GOAL_TOLERANCE_M):
            stopped_at_goal = True
            break
    else:
        cycle = MAX_CYCLES - 1

    return {
        "seed": int(seed),
        "cycles": int(cycle + 1),
        "state_updates": state_updates,
        "max_band_violation_m": band_violation,
        "minimum_footprint_clearance_m": min_clearance,
        "maximum_speed_mps": max_speed,
        "maximum_acceleration_mps2": max_acceleration,
        "maximum_emergency_command_rate_mps2": max_emergency_command_rate,
        "emergency_stop_command_policy": "exact_zero_fail_closed",
        "emergency_stop_physical_deceleration_qualified": False,
        "terminal_stop_deceleration_mps2": terminal_stop_deceleration,
        "maximum_yaw_rate_rps": max_yaw_rate,
        "command_safety_horizon_s": COMMAND_RETENTION_HORIZON_S,
        "moving_hold_count": moving_holds,
        "planner_refusal_count": planner_refusals,
        "final_goal_error_m": float(np.linalg.norm(state[:2] - goal)),
        "stopped_at_goal": stopped_at_goal,
        "final_linear_mps": final_command.linear_x_mps,
        "final_angular_rps": final_command.angular_z_rps,
        "qualification": "software_only",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    report = run_validation(args.seed)
    print(json.dumps(report, sort_keys=True, indent=None if args.json else 2))
    return 0 if report["stopped_at_goal"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
