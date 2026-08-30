import inspect
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import array_backend  # noqa: E402
import dwa_core  # noqa: E402
from gpu_dwa_backend import (DwaDistanceBackend, GpuRequiredError,
                             make_gpu_planner)  # noqa: E402


class WideBand:
    def margins_many(self, points):
        points = np.asarray(points, dtype=float)
        lateral = points[:, 1]
        return (lateral, np.full(len(points), -4.0),
                np.full(len(points), 4.0))

    @staticmethod
    def contained(lateral, lo, hi, grace=0.0):
        return (lateral >= lo - grace) & (lateral <= hi + grace)


class OpenMask:
    def contains_many(self, points):
        return np.ones(len(points), dtype=bool)

    def paths_are_contained(self, paths):
        return np.ones(len(paths), dtype=bool)

    def boundary_cost_many(self, points):
        return np.zeros(len(points), dtype=float)


def route():
    x = np.linspace(0.0, 12.0, 121)
    return np.stack((x, np.zeros_like(x)), axis=1)


def setup_function():
    array_backend.reset()


def teardown_function():
    array_backend.reset()


def test_cpu_distance_fallback_matches_nearest_points():
    backend = DwaDistanceBackend(
        route(), prefer_gpu=False, require_gpu=False)
    distance, index = backend.route_query([
        (0.04, 0.0),
        (2.06, 0.0),
        (11.96, 0.0),
    ])
    assert backend.backend_name == "numpy"
    assert np.allclose(distance, (0.04, 0.04, 0.04), atol=1e-6)
    assert np.allclose(route()[index, 0], (0.0, 2.1, 12.0))

    distance, _ = backend.obstacle_query(
        [(0.0, 0.0), (2.0, 0.0)],
        [(1.0, 0.0), (4.0, 0.0)],
    )
    assert np.allclose(distance, (1.0, 1.0))


def test_required_gpu_fails_closed_when_accelerator_is_disabled():
    with pytest.raises(GpuRequiredError):
        DwaDistanceBackend(route(), prefer_gpu=False, require_gpu=True)


def test_accelerated_planner_preserves_cpu_decision_on_reference_backend():
    base = dwa_core.DwaPlanner(
        WideBand(), route(), route_mask=OpenMask())
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)
    accelerated = GpuPlanner(
        WideBand(), route(), route_mask=OpenMask(),
        prefer_gpu=False, require_gpu=False)

    cases = [
        ((0.0, 0.0, 0.0), (), 0.8, 0.0, 0.35),
        ((1.0, 0.15, 0.03), ((2.4, 0.75),), 0.65, 0.05, 0.5),
        ((4.0, -0.2, -0.05), ((5.2, -0.85), (6.0, 0.8)),
         0.8, -0.1, 0.6),
    ]
    for state, obstacles, cap, yaw, speed in cases:
        expected = base.plan(
            state, obstacles=obstacles, speed_cap=cap,
            last_yaw_rate=yaw, last_speed=speed)
        observed = accelerated.plan(
            state, obstacles=obstacles, speed_cap=cap,
            last_yaw_rate=yaw, last_speed=speed)
        assert observed[2] == expected[2]
        assert np.allclose(observed[:2], expected[:2], atol=1e-9)


def test_planners_do_not_accept_unsequenced_gate_yaw_blacklists():
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)

    assert "rejected_yaw_rates" not in inspect.signature(
        dwa_core.DwaPlanner.plan).parameters
    assert "rejected_yaw_rates" not in inspect.signature(
        GpuPlanner.plan).parameters


def test_accelerated_planner_accepts_clearance_input():
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)
    planner = GpuPlanner(
        WideBand(), route(), route_mask=OpenMask(),
        prefer_gpu=False, require_gpu=False)

    first = planner.plan(
        (0.0, 0.0, 0.0), speed_cap=0.35,
        last_yaw_rate=0.0, last_speed=0.35,
        obstacle_floor_m=0.8)
    second = planner.plan(
        (0.0, 0.0, 0.0), speed_cap=0.35,
        last_yaw_rate=0.0, last_speed=0.35,
        obstacle_floor_m=0.8)

    assert first[2] == second[2] == "OK"
    assert second == first


def test_accelerated_planner_shares_actuator_rollout_and_side_commitment():
    base = dwa_core.DwaPlanner(
        WideBand(), route(), route_mask=OpenMask())
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)
    accelerated = GpuPlanner(
        WideBand(), route(), route_mask=OpenMask(),
        prefer_gpu=False, require_gpu=False)
    actuator = dwa_core.ActuatorState(0.0, 0.0, 0.0)
    kwargs = {
        "speed_cap": dwa_core.TURN_FLOOR_SPEED,
        "actuator_state": actuator,
        "committed_side": "RIGHT",
        "proposal_seq": 7,
        "stamp_s": 12.0,
        "permit_track_id": 18,
        "latency_s": 0.55,
        "return_proposal": True,
    }

    expected = base.plan((0.0, 0.0, 0.0), **kwargs)
    observed = accelerated.plan((0.0, 0.0, 0.0), **kwargs)

    assert expected[2] == observed[2] == "OK"
    assert expected[3] == observed[3]
    assert observed[3].target_yaw_rate_rps < 0.0


def test_cpu_and_gpu_fail_closed_on_invalid_proposal_metadata():
    base = dwa_core.DwaPlanner(
        WideBand(), route(), route_mask=OpenMask())
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)
    accelerated = GpuPlanner(
        WideBand(), route(), route_mask=OpenMask(),
        prefer_gpu=False, require_gpu=False)

    expected = base.plan(
        (0.0, 0.0, 0.0), actuator_state="invalid",
        proposal_seq=1, stamp_s=2.0, permit_track_id=3,
        return_proposal=True)
    observed = accelerated.plan(
        (0.0, 0.0, 0.0), actuator_state="invalid",
        proposal_seq=1, stamp_s=2.0, permit_track_id=3,
        return_proposal=True)

    assert expected == observed == (
        0.0, 0.0, "ACTUATOR_STATE_INVALID", None)


def test_proposal_minimum_turn_excludes_straight_cpu_and_gpu_candidates():
    base = dwa_core.DwaPlanner(
        WideBand(), route(), route_mask=OpenMask())
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)
    accelerated = GpuPlanner(
        WideBand(), route(), route_mask=OpenMask(),
        prefer_gpu=False, require_gpu=False)
    legacy_cpu = base.plan(
        (0.0, 0.0, 0.0), speed_cap=dwa_core.TURN_FLOOR_SPEED)
    legacy_gpu = accelerated.plan(
        (0.0, 0.0, 0.0), speed_cap=dwa_core.TURN_FLOOR_SPEED)
    assert legacy_cpu == legacy_gpu == (dwa_core.TURN_FLOOR_SPEED, 0.0, "OK")
    kwargs = {
        "speed_cap": dwa_core.TURN_FLOOR_SPEED,
        "actuator_state": dwa_core.ActuatorState(0.0, 0.0, 0.0),
        "proposal_seq": 31,
        "stamp_s": 50.0,
        "permit_track_id": 7,
        "minimum_turn_rps": 0.08,
        "return_proposal": True,
    }

    expected = base.plan((0.0, 0.0, 0.0), **kwargs)
    observed = accelerated.plan((0.0, 0.0, 0.0), **kwargs)

    assert expected[2] == observed[2] == "OK"
    assert expected[3] == observed[3]
    assert abs(observed[3].target_yaw_rate_rps) >= 0.08


@pytest.mark.parametrize("invalid_turn", [float("nan"), float("inf"), -0.01,
                                           True, "0.08"])
def test_invalid_proposal_minimum_turn_fails_closed_cpu_and_gpu(invalid_turn):
    base = dwa_core.DwaPlanner(
        WideBand(), route(), route_mask=OpenMask())
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)
    accelerated = GpuPlanner(
        WideBand(), route(), route_mask=OpenMask(),
        prefer_gpu=False, require_gpu=False)
    kwargs = {
        "actuator_state": dwa_core.ActuatorState(0.0, 0.0, 0.0),
        "proposal_seq": 32,
        "stamp_s": 51.0,
        "permit_track_id": 7,
        "minimum_turn_rps": invalid_turn,
        "return_proposal": True,
    }

    assert base.plan((0.0, 0.0, 0.0), **kwargs) == (
        0.0, 0.0, "ACTUATOR_STATE_INVALID", None)
    assert accelerated.plan((0.0, 0.0, 0.0), **kwargs) == (
        0.0, 0.0, "ACTUATOR_STATE_INVALID", None)


@pytest.mark.parametrize("invalid_latency", [float("nan"), float("inf"),
                                               -0.01, True, "0.55"])
def test_invalid_proposal_latency_fails_closed_cpu_and_gpu(invalid_latency):
    base = dwa_core.DwaPlanner(
        WideBand(), route(), route_mask=OpenMask())
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)
    accelerated = GpuPlanner(
        WideBand(), route(), route_mask=OpenMask(),
        prefer_gpu=False, require_gpu=False)
    kwargs = {
        "actuator_state": dwa_core.ActuatorState(0.0, 0.0, 0.0),
        "proposal_seq": 33,
        "stamp_s": 52.0,
        "permit_track_id": 7,
        "latency_s": invalid_latency,
        "return_proposal": True,
    }

    assert base.plan((0.0, 0.0, 0.0), **kwargs) == (
        0.0, 0.0, "ACTUATOR_STATE_INVALID", None)
    assert accelerated.plan((0.0, 0.0, 0.0), **kwargs) == (
        0.0, 0.0, "ACTUATOR_STATE_INVALID", None)
