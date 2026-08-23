"""The approach ramp may not ask for a speed the wheels will not turn for.

On 2026-08-23 the chair met a parked motorcycle, tracked it correctly as
static and crossing the band from 4.6 m out, and stopped 1.3 m short of it
for 2.4 minutes. Nothing was broken in the avoidance: the ramp had handed
the planner a cap of 0.15 m/s, dwa_core.speed_samples returns no executable
speed below 0.30, and with no candidate to score the manoeuvre the ramp was
slowing down for was never attempted.
"""

import importlib.util
from pathlib import Path

from pytest import approx


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(
        name, SCRIPTS / ("%s.py" % name))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


dwa_follower = load("dwa_follower")
dwa_core = load("dwa_core")

approach_cap = dwa_follower.approach_cap
FLOOR = dwa_core.TURN_FLOOR_SPEED
EXTRA = dwa_follower.GUARD_SLOW_EXTRA_M
CREEP = dwa_follower.CREEP_SPEED


def test_far_enough_away_the_ramp_does_not_apply():
    assert approach_cap(0.8, 10.0, 1.0, FLOOR) == 0.8


def test_the_ramp_slows_the_approach():
    """Between the stop radius and the extra, the cap falls with distance."""
    far = approach_cap(0.8, 1.0 + EXTRA * 0.9, 1.0, FLOOR)
    near = approach_cap(0.8, 1.0 + EXTRA * 0.3, 1.0, FLOOR)
    assert far > near
    assert far <= 0.8


def test_it_never_asks_below_the_speed_the_wheels_turn_for():
    """The whole point. At the stop radius the old ramp returned CREEP_SPEED,
    which is under the floor, and the planner had nothing to score."""
    assert CREEP < FLOOR, "otherwise this test proves nothing"
    for distance in (1.0, 1.05, 1.2, 1.5):
        assert approach_cap(0.8, distance, 1.0, FLOOR) >= FLOOR


def test_the_08_23_numbers_now_leave_the_planner_something_to_do():
    """The four caps the ramp actually handed down that day."""
    stop_m = 1.0
    for distance in (1.0, 1.3, 1.9, 2.1):
        cap = approach_cap(0.8, distance, stop_m, FLOOR)
        assert cap >= FLOOR
        assert len([v for v in dwa_core.speed_samples(cap) if v > 0.0]) > 0


def test_a_lower_cap_from_elsewhere_still_wins():
    """The ramp is not a licence to speed up. If the slope or the corridor
    has already asked for less, that answer stands - and if it is under the
    floor, SPEED_BELOW_FLOOR is the honest report."""
    assert approach_cap(0.25, 1.0, 1.0, FLOOR) == approx(0.25)
    assert approach_cap(0.5, 10.0, 1.0, FLOOR) == approx(0.5)
