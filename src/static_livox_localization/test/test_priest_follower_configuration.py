"""Construction and packaging contracts for the PRIEST follower."""

from __future__ import annotations

import types

import numpy as np

from test_priest_follower_execution import (
    Plan,
    ROOT,
    Stamp,
    follower_with,
    pf,
    plan_with,
)


def test_planner_is_bound_to_runtime_band_and_physical_limits() -> None:
    band = types.SimpleNamespace()
    planner = pf.make_planner(band)

    assert planner.runtime_band is band
    assert planner.a_max == 0.18 and planner.yaw_rate_max == 0.5
    assert planner.control_hz == 10.0 and planner.band_grace_m == 0.10
    assert planner.turn_floor_speed_mps == 0.30


def test_execution_rate_matches_controller_and_dense_certificate() -> None:
    assert pf.CONTROL_HZ == 5.0
    assert pf.DEFAULT_CONTROLLER_LIMITS.control_period_s \
        == 1.0 / pf.CONTROL_HZ
    assert pf.CERTIFICATE_HZ >= 10.0


def test_planner_at_goal_requests_replan_without_latching_done() -> None:
    follower = follower_with(plan_with())
    follower.plan, follower.done = None, False
    follower.pose_map = np.eye(4)
    follower.lidar_in_body, follower.lidar_rotation = np.zeros(3), np.eye(3)
    follower.cluster_summary = types.SimpleNamespace(usable=True, objects=[])
    at_goal = Plan(None, None, None, None, 0.0, 0.0, 0, 0.0, reason="AT_GOAL")
    follower.planner = types.SimpleNamespace(
        max_obstacles=24, a_max=0.18,
        plan=lambda *args, **kwargs: at_goal)
    follower.corridor = types.SimpleNamespace()

    assert follower.ensure_plan(Stamp(10.2)) == "PLAN_COMPLETE"
    assert not follower.done


def test_replan_from_rest_uses_actual_yaw_and_executable_acceleration() -> None:
    follower = follower_with(plan_with())
    follower.plan = None
    follower.corridor = types.SimpleNamespace()
    captured: list[tuple[np.ndarray, float]] = []
    at_goal = Plan(None, None, None, None, 0.0, 0.0, 0, 0.0, reason="AT_GOAL")

    def plan(*args, **kwargs):
        captured.append((args[2], kwargs["initial_yaw_rad"]))
        return at_goal

    follower.planner = types.SimpleNamespace(
        max_obstacles=24, a_max=0.18, plan=plan)

    assert follower.ensure_plan(Stamp(10.2)) == "PLAN_COMPLETE"
    assert np.allclose(captured[0][0], np.array([0.18, 0.0]))
    assert captured[0][1] == 0.0


def test_moving_replan_projects_small_pose_noise_onto_body_heading() -> None:
    follower = follower_with(plan_with())
    follower.plan = None
    follower.velocity = np.array([0.40, 0.000002])
    follower.corridor = types.SimpleNamespace()
    captured: list[np.ndarray] = []
    at_goal = Plan(None, None, None, None, 0.0, 0.0, 0, 0.0, reason="AT_GOAL")

    def plan(*args, **kwargs):
        captured.append(args[1])
        return at_goal

    follower.planner = types.SimpleNamespace(
        max_obstacles=24, a_max=0.18, plan=plan)

    assert follower.ensure_plan(Stamp(10.2)) == "PLAN_COMPLETE"
    assert np.allclose(captured[0], np.array([0.40, 0.0]))


def test_material_lateral_velocity_fails_before_planning() -> None:
    follower = follower_with(plan_with())
    follower.plan = None
    follower.velocity = np.array([0.0, 0.10])
    follower.corridor = types.SimpleNamespace()
    called: list[bool] = []
    follower.planner = types.SimpleNamespace(
        max_obstacles=24, a_max=0.18,
        plan=lambda *args, **kwargs: called.append(True))

    assert follower.ensure_plan(Stamp(10.2)) == "NONHOLONOMIC_STATE"
    assert not called


def test_controller_runtime_and_io_siblings_are_installed() -> None:
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    for name in (
            "priest_certificate_types.py", "priest_actuator_control.py",
            "priest_control_types.py",
            "priest_controller.py",
            "priest_runtime.py",
            "priest_execution_safety.py", "priest_follower_io.py",
            "priest_follower_planning.py", "priest_terminal.py",
            "priest_terminal_control.py",
            "obstacle_accumulator.py", "obstacle_cluster_geometry.py",
            "wheel_command_model.py"):
        assert "scripts/" + name in cmake
