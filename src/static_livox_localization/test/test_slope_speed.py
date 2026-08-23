"""A grade limits speed through braking, and only in one direction.

The rule this replaces was abs(pitch) > 3 deg -> 0.30 m/s, which held 47 %
of the 2026-08-23 route at the floor and spent the same caution climbing as
descending. The base holds whatever speed it is given - it adds drive or
brake as the grade demands - so the question was never whether the speed can
be produced. It is how much braking is left for a stop, and uphill there is
more of it, not less.
"""

import importlib.util
import math
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


mpc_speed = load("mpc_speed")
limit = mpc_speed.slope_speed_limit
FLOOR = mpc_speed.TURN_FLOOR_SPEED
DOWNHILL_MIN = mpc_speed.SLOPE_DOWNHILL_MIN_MPS
TOP = mpc_speed.MAX_SPEED


def deg(d):
    return math.radians(d)


def test_climbing_costs_nothing():
    """Gravity brakes for you going up. At 4 deg it adds 0.68 m/s^2 to
    whatever the brakes make, so there is no braking case for slowing."""
    for d in (-3.5, -4, -6, -8, -12):
        assert limit(deg(d)) == approx(TOP)


def test_the_flat_is_untouched():
    for d in (-3, -1, 0, 1, 3):
        assert limit(deg(d)) == approx(TOP)


def test_descending_gives_up_speed_as_the_grade_takes_the_brake():
    """Monotone, because a steeper hill never leaves more braking."""
    speeds = [limit(deg(d)) for d in (3.5, 4.5, 5.0, 5.5, 6.0, 7.0)]
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[0] == approx(TOP)
    assert speeds[-1] == approx(DOWNHILL_MIN)


def test_the_speed_is_one_the_chair_can_stop_from():
    """sqrt(2 a s) with a = brake - g sin(theta): the limit is whatever still
    stops inside the margin, so re-deriving it has to give the margin back."""
    brake = mpc_speed.SLOPE_BRAKE_MPS2
    margin = mpc_speed.SLOPE_STOP_MARGIN_M
    for d in (4.5, 5.0):
        v = limit(deg(d))
        remaining = brake - mpc_speed.GRAVITY_MPS2 * math.sin(deg(d))
        assert remaining > 0
        assert v * v / (2.0 * remaining) <= margin + 1e-9


def test_a_steep_descent_still_gets_a_usable_speed():
    """Past about 5 deg the braking term alone would fall below what the
    operator calls a safe descent. It does not: a long hill crawled at the
    actuation floor is minutes spent on it, which is its own hazard, and the
    descent floor is where that judgement lives."""
    for d in (7, 9, 15):
        assert limit(deg(d)) == approx(DOWNHILL_MIN)


def test_no_grade_is_given_less_than_the_descent_floor():
    for d in range(-15, 21):
        assert limit(deg(d)) >= DOWNHILL_MIN - 1e-9


def test_the_descent_floor_sits_in_the_range_the_operator_set():
    """0.6 to 0.8 m/s, said on 2026-08-23."""
    assert 0.6 <= DOWNHILL_MIN <= TOP <= 0.8


def test_a_weaker_brake_slows_the_descent_sooner():
    """The constant is a measurement, so the function has to move with it."""
    strong = limit(deg(5.0), brake_mps2=1.6)
    weak = limit(deg(5.0), brake_mps2=1.0)
    assert strong > weak
