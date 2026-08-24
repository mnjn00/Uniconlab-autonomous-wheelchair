"""The planner searches speeds it can actually reach, and pays to change one.

The 2026-08-23 run changed its target speed 1,035 times over the flat parts
of the route, about once a second, cycling 0.30 / 0.42 / 0.55 / 0.68 / 0.80
as five nearly-tied candidates reordered under tiny movements in path cost.
Two things were missing. The whole range was re-scored every cycle with no
regard for what the chair was doing, and nothing rewarded holding a speed -
the yaw axis had W_STEER for exactly that and the speed axis had nothing.

The ramp's asymmetry is what turned that into a lurch: it brakes at
0.60 m/s^2 and accelerates at 0.18, so every needless dip cost three times
longer to climb out of than to fall into.
"""

import importlib.util
from pathlib import Path


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(
        name, SCRIPTS / ("%s.py" % name))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


dwa_core = load("dwa_core")
samples = dwa_core.speed_samples
FLOOR = dwa_core.TURN_FLOOR_SPEED
TOP = dwa_core.MAX_SPEED


def moving(values):
    return [v for v in values if v > 0.0]


def test_without_a_current_speed_the_whole_range_is_offered():
    """The old behaviour, kept for callers that have no velocity to give."""
    v = moving(samples(TOP))
    assert min(v) == FLOOR
    assert max(v) == TOP


def test_the_window_is_drawn_around_where_the_chair_is():
    v = moving(samples(TOP, current=0.35))
    assert max(v) <= 0.35 + dwa_core.MAX_ACCEL * dwa_core.VELOCITY_WINDOW_S + 1e-9
    assert max(v) < TOP, "a standing start must not be able to pick top speed"


def test_standing_start_keeps_offering_only_the_turn_floor():
    """The final 2026-08-23 drive started with a 0.35 m/s target while the
    command ramp climbed from 0.01 m/s.  Until that ramp clears the measured
    wheel deadband, replanning must keep the floor available instead of
    returning no executable speed and resetting the ramp to zero.
    """
    for current in (0.0, 0.01, 0.18, FLOOR - 1e-6):
        assert moving(samples(TOP, current=current)) == [FLOOR]


def test_braking_stays_available_from_any_speed():
    """The window is asymmetric because the chair is. Slowing down must never
    be the thing the window forbids."""
    v = moving(samples(TOP, current=TOP))
    assert min(v) == FLOOR


def test_the_window_never_offers_less_than_the_floor():
    for current in (0.35, 0.5, 0.8):
        assert min(moving(samples(TOP, current=current))) >= FLOOR - 1e-9


def test_a_cap_under_the_floor_still_reports_no_executable_speed():
    """SPEED_BELOW_FLOOR must survive the window: the caller distinguishes it
    from an obstacle, and that distinction cost a 2.4 minute stall to learn."""
    assert moving(samples(0.2, current=0.5)) == []


def test_a_reachable_cap_is_honoured_over_the_window():
    v = moving(samples(0.45, current=0.8))
    assert max(v) <= 0.45 + 1e-9


def test_holding_a_speed_is_cheaper_than_changing_one():
    """W_SPEED is what breaks the ties that were flipping every cycle."""
    assert dwa_core.W_SPEED > 0.0
