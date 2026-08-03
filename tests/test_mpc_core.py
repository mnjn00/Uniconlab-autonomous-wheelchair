"""Regression coverage for the MPC follower core (docs/mpc_follower_design.md).

No ROS and no bag are needed. The solver tests additionally need osqp and
scipy; the geometry tests run without them.
"""
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = load("mpc_core", SCRIPTS / "mpc_core.py")
safety_band = load("safety_band", SCRIPTS / "safety_band.py")


def make_straight_band(n=60, spacing=1.0, half_width=1.5):
    """Wide straight corridor along +x; stations 1 m apart."""
    stations = [
        {"x": i * spacing, "y": 0.0, "heading_deg": 0.0,
         "left_m": half_width, "right_m": half_width}
        for i in range(n)
    ]
    tmp = tempfile.NamedTemporaryFile(
        "w", suffix=".json", delete=False, encoding="utf-8")
    json.dump({"stations": stations}, tmp)
    tmp.close()
    return safety_band.SafetyBand(tmp.name)


class RolloutTest(unittest.TestCase):
    def test_straight_constant_speed(self):
        x0 = np.array([0.0, 0.0, 0.0, 0.5, 0.0])
        U = np.zeros((10, 2))
        xs = core.rollout(x0, U, 0.1)
        self.assertAlmostEqual(xs[-1, 0], 0.5, places=6)
        self.assertAlmostEqual(xs[-1, 1], 0.0, places=6)
        self.assertAlmostEqual(xs[-1, 3], 0.5, places=6)

    def test_constant_yaw_rate_is_an_arc(self):
        x0 = np.array([0.0, 0.0, 0.0, 0.5, 0.5])
        U = np.zeros((20, 2))
        xs = core.rollout(x0, U, 0.1)
        # heading advanced by w * t = 0.5 * 2.0 = 1.0 rad
        self.assertAlmostEqual(xs[-1, 2], 1.0, places=2)
        # chord stays consistent with the arc radius v/w = 1 m
        r = np.linalg.norm(xs[-1, :2] - np.array([0.0, 1.0]))
        self.assertAlmostEqual(r, 1.0, delta=0.05)


class LinearizeTest(unittest.TestCase):
    def test_matches_finite_difference(self):
        xbar = np.array([1.0, 2.0, 0.3, 0.4, 0.1])
        A, B = core.linearize(xbar, 0.1)

        def f(x):
            return core.unicycle_step(x, np.zeros(2), 0.1) - x

        Jnum = np.zeros((5, 5))
        for i in range(5):
            dx = np.zeros(5)
            dx[i] = 1e-6
            Jnum[:, i] = (f(xbar + dx) - f(xbar - dx)) / 2e-6
        # the exact midpoint integrator couples w into X, Y at O(dt^2); the
        # linearisation is first order on purpose, so compare to that order
        np.testing.assert_allclose(A - np.eye(5), Jnum, atol=5e-3)


class ReferenceGeometryTest(unittest.TestCase):
    def setUp(self):
        self.band = make_straight_band()
        self.ref = core.Reference(self.band)

    def test_frame_on_centreline(self):
        normal, heading, lateral, lo, hi = self.ref.frame_at(
            np.array([10.0, 0.0]))
        np.testing.assert_allclose(normal, [0.0, 1.0], atol=1e-9)
        self.assertAlmostEqual(heading, 0.0, places=9)
        self.assertAlmostEqual(lateral, 0.0, places=9)
        self.assertGreater(hi, 0.5)
        self.assertLess(lo, -0.5)

    def test_more_restrictive_bracket_wins(self):
        # hand-build a band with one narrow station between wide ones
        stations = [
            {"x": 0.0, "y": 0.0, "heading_deg": 0.0,
             "left_m": 3.0, "right_m": 3.0},
            {"x": 1.0, "y": 0.0, "heading_deg": 0.0,
             "left_m": 0.6, "right_m": 0.6},
            {"x": 2.0, "y": 0.0, "heading_deg": 0.0,
             "left_m": 3.0, "right_m": 3.0},
        ]
        tmp = tempfile.NamedTemporaryFile(
            "w", suffix=".json", delete=False, encoding="utf-8")
        json.dump({"stations": stations}, tmp)
        tmp.close()
        band = safety_band.SafetyBand(tmp.name)
        _lat, lo, hi = band.lateral_limits(np.array([1.0, 0.0]))
        # the narrow station's usable limit must govern, not the wide ones
        self.assertLess(hi, 0.5)
        self.assertGreater(lo, -0.5)


class ObstacleHalfPlaneTest(unittest.TestCase):
    def test_pushes_toward_the_roomier_side(self):
        stations = [
            {"x": float(i), "y": 0.0, "heading_deg": 0.0,
             "left_m": 2.0, "right_m": 0.6}
            for i in range(20)
        ]
        tmp = tempfile.NamedTemporaryFile(
            "w", suffix=".json", delete=False, encoding="utf-8")
        json.dump({"stations": stations}, tmp)
        tmp.close()
        band = safety_band.SafetyBand(tmp.name)
        ref = core.Reference(band)
        ob = core.obstacle_half_plane(ref, np.array([5.0, 0.0]), 0.5)
        # left side is roomier; the normal must point left (+y)
        self.assertGreater(ob.normal[1], 0.9)
        self.assertAlmostEqual(ob.offset, 0.5, places=6)


def solver_or_skip(testcase):
    try:
        import osqp  # noqa: F401
        import scipy.sparse  # noqa: F401
    except ImportError:
        testcase.skipTest("osqp/scipy not installed")


def closed_loop(band, x0, steps, obstacles=(), v_target=0.4):
    ref = core.Reference(band)
    solver = core.MpcSolver(ref)
    x = x0.copy()
    traj = [x.copy()]
    statuses, lat, solve_ms = [], [], []
    warm = None
    for _ in range(steps):
        v_ref, th_ref = core.polyline_refs(band, x[:2], solver.p.horizon,
                                           solver.p.dt, v_target)
        obs = [core.obstacle_half_plane(ref, o, solver.p.obstacle_padding)
               for o in obstacles]
        u0, status, info = solver.solve_cycle(x, v_ref, th_ref, obs,
                                              warm=warm)
        statuses.append(status)
        lat.append(ref.frame_at(x[:2])[2])
        solve_ms.append(info.get("solve_ms", 0.0))
        x = core.unicycle_step(x, u0, solver.p.dt)
        traj.append(x.copy())
        warm = (info.get("xbar"), info.get("ubar")) \
            if status == core.STATUS_OK else warm
    return np.array(traj), statuses, np.array(lat), np.array(solve_ms)


class SolverBehaviourTest(unittest.TestCase):
    def setUp(self):
        solver_or_skip(self)
        self.band = make_straight_band(half_width=1.5)

    def test_drives_straight_inside_the_band(self):
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        traj, statuses, lat, solve_ms = closed_loop(
            self.band, x0, steps=60, v_target=0.4)
        self.assertTrue(all(s == core.STATUS_OK for s in statuses))
        self.assertGreater(traj[-1, 0], 1.0)           # actually moved
        self.assertLess(np.abs(lat).max(), 0.35)       # stayed centred
        self.assertLess(np.percentile(solve_ms, 99), 50.0)

    def test_avoids_obstacle_and_stays_in_band(self):
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        obstacle_xy = np.array([8.0, 0.0])
        traj, statuses, lat, _ = closed_loop(
            self.band, x0, steps=160,
            obstacles=[obstacle_xy], v_target=0.4)
        ok = sum(1 for s in statuses if s == core.STATUS_OK)
        self.assertGreater(ok, len(statuses) * 0.9)
        # never inside the padded obstacle, always inside the band
        dist = np.linalg.norm(traj[:, :2] - obstacle_xy, axis=1)
        self.assertGreater(dist.min(), 0.30)
        self.assertLess(np.abs(lat).max(), 1.2)

    def test_infeasible_start_stops_instead_of_driving(self):
        band = make_straight_band(half_width=0.4)
        ref = core.Reference(band)
        solver = core.MpcSolver(ref)
        x0 = np.array([5.0, 5.0, 0.0, 0.3, 0.0])  # 5 m outside the band
        v_ref, th_ref = core.polyline_refs(band, x0[:2], solver.p.horizon,
                                           solver.p.dt, 0.4)
        _u0, status, _info = solver.solve_cycle(x0, v_ref, th_ref)
        self.assertEqual(status, core.STATUS_INFEASIBLE_STOP)

    def test_unpassable_obstacle_blocks_instead_of_squeezing(self):
        # 0.8 m corridor, 0.5 m padded obstacle on the line: no room around
        band = make_straight_band(n=120, half_width=0.4)
        traj, statuses, _lat, _ms = closed_loop(
            band, np.array([0.0, 0.0, 0.0, 0.0, 0.0]), steps=400,
            obstacles=[np.array([10.0, 0.0])], v_target=0.4)
        self.assertIn(core.STATUS_BLOCKED_STOP, statuses)
        dist = np.linalg.norm(traj[:, :2] - np.array([10.0, 0.0]), axis=1)
        self.assertGreater(dist.min(), 0.35)


if __name__ == "__main__":
    unittest.main()
