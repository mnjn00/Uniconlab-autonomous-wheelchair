"""The acceleration ramp has a slope limit, and braking is exempt from it.

The DWA follower builds its command by dividing a velocity error by the
control step and clamping to MAX_ACCEL, which means the acceleration can go
from nothing to its limit between two cycles. The speed curve that produces
is trapezoidal, and the corners of a trapezoid are what a seated rider
feels. These tests pin the slope limit that rounds them, and the exemption
that keeps it from ever slowing a stop.
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


mpc_command = load("mpc_command")
jerk_limited = mpc_command.jerk_limited
MAX_JERK = mpc_command.MAX_JERK


def test_a_step_within_the_budget_passes_through_untouched():
    """The limit is a ceiling, not a filter. An acceleration the chair could
    already reach this cycle must arrive unchanged, or every ramp gains lag
    it did not ask for."""
    assert jerk_limited(0.05, 0.02, 0.1) == approx(0.05)
    assert jerk_limited(-0.05, -0.02, 0.1) == approx(-0.05)


def test_a_jump_is_cut_to_the_slope():
    """From standstill the follower asks for MAX_ACCEL outright. It gets the
    slope instead: 0.8 m/s^3 over a 0.1 s cycle is 0.08 m/s^2."""
    assert jerk_limited(0.18, 0.0, 0.1, max_jerk=0.8) == approx(0.08)
    assert jerk_limited(-0.6, 0.0, 0.1, max_jerk=0.8) == approx(-0.08)


def test_the_limit_is_symmetric():
    """Coming off the accelerator is a corner too."""
    assert jerk_limited(0.0, 0.5, 0.1, max_jerk=0.8) == approx(0.42)
    assert jerk_limited(0.0, -0.5, 0.1, max_jerk=0.8) == approx(-0.42)


def test_a_zero_length_cycle_buys_no_change():
    """A step of zero is not a licence to jump; it is no time passing."""
    assert jerk_limited(1.0, 0.1, 0.0) == approx(0.1)


def test_reaching_full_acceleration_takes_the_time_the_slope_implies():
    """MAX_ACCEL over MAX_JERK is 0.225 s. Anything much longer than that is
    a chair that will not keep up with its own speed policy."""
    accel = 0.0
    step = 0.1
    cycles = 0
    while accel < 0.18 - 1e-9 and cycles < 100:
        accel = jerk_limited(0.18, accel, step)
        cycles += 1
    assert cycles * step <= 0.35
    assert abs(accel - 0.18) < 1e-9


def test_the_speed_curve_has_no_corner():
    """The property the limit exists for: over a full ramp to target, the
    acceleration never changes by more than the slope allows in one cycle."""
    step = 0.1
    speed = 0.0
    accel = 0.0
    target = 0.8
    previous = 0.0
    worst = 0.0
    for _ in range(120):
        wanted = max(-0.6, min(0.18, (target - speed) / step))
        accel = jerk_limited(wanted, accel, step)
        worst = max(worst, abs(accel - previous) / step)
        previous = accel
        speed = max(0.0, min(target, speed + accel * step))
    assert worst <= MAX_JERK + 1e-9
    assert abs(speed - target) < 0.02
