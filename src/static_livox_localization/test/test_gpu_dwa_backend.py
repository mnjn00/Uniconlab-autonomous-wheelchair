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


def test_candidate_veto_skips_the_cheapest_arc_and_fails_closed():
    planner = dwa_core.DwaPlanner(
        WideBand(), route(), route_mask=OpenMask())
    state = (0.0, 0.0, 0.0)
    nominal = planner.plan(state, speed_cap=0.8, last_speed=0.35)
    seen = []

    def reject_first(v, w):
        seen.append((v, w))
        return len(seen) == 1

    alternate = planner.plan(
        state, speed_cap=0.8, last_speed=0.35,
        candidate_veto=reject_first)
    assert seen[0] == nominal[:2]
    assert alternate[2] == "OK"
    assert alternate[:2] != nominal[:2]

    refused = planner.plan(
        state, speed_cap=0.8, last_speed=0.35,
        candidate_veto=lambda _v, _w: True)
    assert refused == (0.0, 0.0, "GATE_TRAJECTORY")


def test_gpu_planner_honours_clearance_and_candidate_veto_contract():
    base = dwa_core.DwaPlanner(
        WideBand(), route(), route_mask=OpenMask())
    GpuPlanner = make_gpu_planner(dwa_core.DwaPlanner, dwa_core)
    accelerated = GpuPlanner(
        WideBand(), route(), route_mask=OpenMask(),
        prefer_gpu=False, require_gpu=False)
    kwargs = dict(
        obstacles=((2.0, 0.85),), speed_cap=0.8,
        last_yaw_rate=0.0, last_speed=0.35,
        obstacle_floor_m=0.80,
        candidate_veto=lambda _v, w: abs(w) < 0.20)
    expected = base.plan((0.0, 0.0, 0.0), **kwargs)
    observed = accelerated.plan((0.0, 0.0, 0.0), **kwargs)
    assert observed[2] == expected[2]
    assert np.allclose(observed[:2], expected[:2], atol=1e-9)
