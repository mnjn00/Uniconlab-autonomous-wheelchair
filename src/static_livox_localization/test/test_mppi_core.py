import sys
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from scipy_ckdtree_compat import install as install_ckdtree_compat
install_ckdtree_compat()

import array_backend  # noqa: E402
import dwa_core  # noqa: E402
import mppi_core  # noqa: E402


class WideBand:
    def margins_many(self, points):
        pts = np.asarray(points, dtype=float)
        lateral = pts[:, 1]
        return lateral, np.full(len(pts), -3.0), np.full(len(pts), 3.0)

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


class ClosedMask(OpenMask):
    def contains_many(self, points):
        return np.zeros(len(points), dtype=bool)

    def paths_are_contained(self, paths):
        return np.zeros(len(paths), dtype=bool)


def route():
    x = np.linspace(0.0, 12.0, 121)
    return np.stack((x, np.zeros_like(x)), axis=1)


def setup_function():
    array_backend.reset()


def teardown_function():
    array_backend.reset()


def planner(mask=None):
    return mppi_core.MppiPlanner(
        WideBand(), route(), route_mask=mask or OpenMask(),
        batch_size=64, horizon_steps=12, model_dt=0.1,
        seed=7, prefer_gpu=False, require_gpu=False)


def test_cpu_mppi_returns_executable_forward_command_on_open_route():
    p = planner()
    v, w, status = p.plan(
        (0.0, 0.0, 0.0, 0.35, 0.0), obstacles=(),
        speed_cap=0.65, last_yaw_rate=0.0, last_speed=0.35)
    assert status == "OK"
    assert dwa_core.TURN_FLOOR_SPEED <= v <= 0.65
    assert abs(w) <= dwa_core.MAX_YAW_RATE
    assert p.backend_name == "numpy"
    assert p.last_feasible > 0


def test_mppi_keeps_drivable_mask_as_hard_boundary():
    p = planner(ClosedMask())
    v, w, status = p.plan(
        (0.0, 0.0, 0.0, 0.35, 0.0), obstacles=(),
        speed_cap=0.65, last_yaw_rate=0.0, last_speed=0.35)
    assert (v, w) == (0.0, 0.0)
    assert status == "OFF_BAND"


def test_mppi_rejects_when_obstacle_floor_blocks_every_rollout():
    p = planner()
    # A dense wall just in front of the chair puts every first rollout sample
    # inside the same 0.50 m clearance floor used by DWA/safety_gate.
    wall = [(0.10, y) for y in np.linspace(-2.0, 2.0, 41)]
    v, w, status = p.plan(
        (0.0, 0.0, 0.0, 0.35, 0.0), obstacles=wall,
        speed_cap=0.65, last_yaw_rate=0.0, last_speed=0.35)
    assert (v, w) == (0.0, 0.0)
    assert status == "OBSTACLE"


def test_speed_cap_below_loaded_turn_floor_fails_closed():
    p = planner()
    v, w, status = p.plan(
        (0.0, 0.0, 0.0, 0.0, 0.0), obstacles=(),
        speed_cap=dwa_core.TURN_FLOOR_SPEED - 0.01,
        last_yaw_rate=0.0, last_speed=0.0)
    assert (v, w) == (0.0, 0.0)
    assert status == "SPEED_BELOW_FLOOR"
