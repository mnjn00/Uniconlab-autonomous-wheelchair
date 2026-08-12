"""The OFF_BAND stop, and what it takes to make it a state with a way out.

Every rollout point is tested for containment, so a chair one centimetre
outside the corridor has no admissible candidate at all - it cannot drive
back in, because driving back in starts outside. The 2026-08-08 analysis
measured that at 23 % of run 1 and called it a starting-state problem no
scoring change reaches. It is that, and this is the state machine it needs
rather than a weight.

The tests that matter here are the ones pinning what the recovery REFUSES to
do: it never widens the band, never outruns the follower's own OFF_BAND
grace, never asks for more speed than the deadband floor, and never drives
into anything. A recovery that relaxes one more thing than it has to is the
excursion it exists to end.
"""

import importlib.util
import json
import sys
import types
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).parents[3]
SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    """Load a script module by path, with its siblings importable."""
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            name, SCRIPTS / ("%s.py" % name))
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


dwa_core = load("dwa_core")
safety_band = load("safety_band")


# ----------------------------------------------------------- a bench corridor

def straight_band(tmp_path, half_width_m=2.0, length_m=40.0, step_m=0.5):
    """A corridor along +x, wide enough that its edges are the test's to set.

    Built rather than borrowed from routes/: the shipped band's widths vary
    station to station, and a recovery test has to know exactly how far
    outside the chair is standing.
    """
    stations = [{"x": float(x), "y": 0.0, "heading_deg": 0.0,
                 "left_m": half_width_m, "right_m": half_width_m,
                 "left_kind": "open", "right_kind": "open",
                 "left_drop_m": 0.0, "right_drop_m": 0.0}
                for x in np.arange(0.0, length_m + step_m, step_m)]
    path = tmp_path / "band.json"
    path.write_text(json.dumps({"stations": stations}), encoding="utf-8")
    return safety_band.SafetyBand(str(path))


def bench(tmp_path, half_width_m=2.0):
    band = straight_band(tmp_path, half_width_m=half_width_m)
    planner = dwa_core.DwaPlanner(band, band.xy.copy())
    _lateral, lo, hi = band.lateral_limits(np.array([10.0, 0.0]))
    return band, planner, lo, hi


def excursion(planner, state, v, w, sim_time_s):
    """How far outside the band the commanded arc actually goes.

    Integrated by dwa_core.rollout, which since 2026-08-11 is the same step
    DwaPlanner._rollouts takes in a batch. It was not: the two disagreed by
    one step of rotation, so a check written this way measured a trajectory
    the planner never scored.
    """
    path = dwa_core.rollout(np.asarray(state, dtype=float), v, w,
                            sim_time_s=sim_time_s)
    return planner._excursion(*planner.band.margins_many(path[:, :2]))


# --------------------------------------------------------- the recovery pass

def test_the_bench_corridor_is_the_width_the_tests_assume(tmp_path):
    """Otherwise every number below is measured against a different band."""
    _band, _planner, lo, hi = bench(tmp_path)
    assert hi > 0.5 and lo < -0.5
    assert hi == pytest.approx(-lo)


def test_a_chair_on_the_line_never_reaches_the_recovery(tmp_path):
    _band, planner, _lo, _hi = bench(tmp_path)
    v, w, status = planner.plan([10.0, 0.0, 0.0])
    assert status == "OK"
    assert w == pytest.approx(0.0, abs=0.06)
    assert v > 0.0


def test_a_chair_outside_the_corridor_is_given_a_way_back_in(tmp_path):
    """The stop that needed a person. Facing along a corridor it is standing
    5 cm to the left of, every full-length arc starts outside and dies; the
    recovery hands back the slowest arc that closes the gap."""
    _band, planner, _lo, hi = bench(tmp_path)
    state = [10.0, hi + 0.05, 0.0]

    v, w, status = planner.plan(state)

    assert status == "RECOVER"
    assert v == pytest.approx(dwa_core.RECOVER_SPEED_MPS)
    assert w < 0.0, "it has to turn back toward the corridor, not away"
    out = excursion(planner, state, v, w, dwa_core.RECOVER_SIM_TIME_S)
    assert out[-1] < 0.05 - dwa_core.RECOVER_CLOSE_M
    assert out.max() <= 0.05 + 1e-6, "it must never go further out than it is"


def test_the_recovery_stops_where_the_follower_starts_asking_for_a_person(
        tmp_path):
    """RECOVER_GRACE_M is the follower's own OFF_BAND_GRACE on purpose. Past
    it the hold ladder stops the chair, and a planner still creeping there
    would be quietly overruling it."""
    _band, planner, _lo, hi = bench(tmp_path)
    follower = (ROOT / "src/static_livox_localization/scripts"
                / "waypoint_follower.py").read_text(encoding="utf-8")
    assert "OFF_BAND_GRACE = %.2f" % dwa_core.RECOVER_GRACE_M in follower

    far_out = [10.0, hi + dwa_core.RECOVER_GRACE_M + 0.05, 0.0]
    assert planner.plan(far_out)[2] == "OFF_BAND"


def test_a_large_heading_error_gets_a_shorter_lookahead_not_a_wider_band(
        tmp_path):
    """The other way in, and the one the 2026-08-08 runs actually deadlocked
    at: 99 % of blocked samples were past 60 degrees of heading error. The
    chair is INSIDE the corridor, pointed 69 degrees across it, and no 1.7 s
    arc stays in - at 0.5 rad/s the heading does not come back inside the
    lookahead. Here the recovery relaxes NOTHING: every sampled point is
    still strictly contained, and the only thing shortened is how far ahead
    it looked."""
    _band, planner, _lo, hi = bench(tmp_path, half_width_m=0.3)
    state = [10.0, 0.0, 1.2]

    v, w, status = planner.plan(state)

    assert status == "RECOVER"
    assert w == pytest.approx(-dwa_core.MAX_YAW_RATE), \
        "69 degrees left of the corridor, so it steers right as hard as it can"
    assert excursion(planner, state, v, w,
                     dwa_core.RECOVER_SIM_TIME_S).max() == 0.0
    assert excursion(planner, state, v, w,
                     dwa_core.SIM_TIME_S).max() > 0.0, \
        "this case needs a full-length arc that really does leave the band"


def test_the_recovery_horizon_still_covers_the_chair_stopping(tmp_path):
    """Shortening the lookahead is the relaxation, so it has a floor: the
    chair must be able to stop inside what it looked at. 0.30 m/s against
    the 0.5 m/s^2 the safety gate assumes is 0.6 s."""
    assert dwa_core.RECOVER_SIM_TIME_S >= \
        dwa_core.RECOVER_SPEED_MPS / 0.5


def test_the_recovery_does_not_drive_into_something(tmp_path):
    """A recovery into an obstacle is not a recovery. Same floor as the
    ordinary pass, and the status says obstacle rather than corridor - the
    operator needs to know which of the two stopped the chair."""
    _band, planner, _lo, hi = bench(tmp_path)
    state = [10.0, hi + 0.05, 0.0]
    across = [[10.0 + 0.1 * k, hi + 0.05 + y]
              for k in range(1, 6) for y in np.arange(-0.6, 0.61, 0.1)]

    assert planner.plan(state)[2] == "RECOVER"
    assert planner.plan(state, obstacles=across)[2] == "OBSTACLE"


def test_a_speed_policy_below_the_deadband_never_reaches_the_recovery(
        tmp_path):
    """Creeping out of the corridor at a speed the wheels ignore is a
    command that never arrives. plan() has already refused by then, because
    speed_samples offers nothing executable under the deadband; the
    recovery's own guard is what makes it safe to call on its own."""
    _band, planner, _lo, hi = bench(tmp_path)
    state = [10.0, hi + 0.05, 0.0]

    assert planner.plan(state, speed_cap=0.1)[2] == "NO_CANDIDATE"
    assert planner.recover(state, speed_cap=0.1)[2] == "OFF_BAND"
    assert planner.plan(state, speed_cap=dwa_core.RECOVER_SPEED_MPS)[2] == \
        "RECOVER"


def test_recovery_never_asks_for_more_speed_than_the_floor(tmp_path):
    _band, planner, _lo, hi = bench(tmp_path)
    v, _w, status = planner.plan([10.0, hi + 0.05, 0.0], speed_cap=0.6)
    assert status == "RECOVER"
    assert v == pytest.approx(dwa_core.TURN_FLOOR_SPEED)


def test_the_recovery_walks_the_band_once_too(tmp_path):
    """The same property main pins for plan(). This path runs when the
    ordinary one has already failed, so it is the worse place to search the
    corridor twice, not the better one."""
    band, planner, _lo, hi = bench(tmp_path)
    calls = []
    real = band.margins_many

    def counting(points):
        calls.append(len(points))
        return real(points)

    band.margins_many = counting
    try:
        _v, _w, status = planner.recover([10.0, hi + 0.05, 0.0])
    finally:
        band.margins_many = real

    assert status == "RECOVER"
    # one for where the chair is, one for the rollouts
    assert len(calls) == 2, "searched the band %d times" % len(calls)
    assert calls[0] == 1
