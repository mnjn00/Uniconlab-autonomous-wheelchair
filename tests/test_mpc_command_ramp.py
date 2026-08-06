"""The command ramp, and the standstill it used to lock into.

These are the tests the simulation could not have been: the plant in
tools/sim_mpc_follower.py is a unicycle with no actuation deadband and a
loop that never misses its period, so it followed a 0.018 m/s command
perfectly and finished the route. The chair did not.
"""

import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"

sys.path.insert(0, str(SCRIPTS))
try:
    from mpc_command import (MAX_COMMAND_GAP_S, MIN_COMMAND_STEP_S,
                             advance_command)
finally:
    sys.path.pop(0)

A_MAX = 0.18
V_MAX = 0.6
YAW_MAX = 0.5
# The loaded base was measured not to move below roughly this - the same
# number TURN_FLOOR_SPEED comes from.
ACTUATION_FLOOR = 0.30


def ramp(cycles, period_s, accel=A_MAX, plant=None):
    """Run the integrator for a while and report the command it reaches.

    plant(commanded) -> what the wheels actually do, defaulting to a base
    that answers anything. The deadband version is what the chair is.
    """
    speed, yaw = 0.0, 0.0
    for _ in range(cycles):
        speed, yaw = advance_command(speed, yaw, np.array([accel, 0.0]),
                                     period_s, V_MAX, YAW_MAX)
        if plant is not None:
            plant(speed)
    return speed


def test_the_chair_gets_out_of_the_actuation_deadband():
    """The regression. Integrating onto the measured velocity could not do
    this: a base that ignores anything under 0.30 m/s reports 0, so the
    command never exceeded a_max * dt no matter how long it ran."""
    measured = []
    speed = ramp(40, 0.1, plant=lambda c: measured.append(
        c if c >= ACTUATION_FLOOR else 0.0))
    assert speed > ACTUATION_FLOOR
    assert max(measured) > 0.0, "the wheels never turned"


def test_integrating_on_the_measured_velocity_would_still_be_stuck():
    """The old arithmetic, kept as a counter-example so the fix cannot be
    quietly undone: with a deadband base it is a fixed point at ~0.018."""
    measured = 0.0
    for _ in range(200):
        command = min(measured + A_MAX * 0.1, V_MAX)
        measured = command if command >= ACTUATION_FLOOR else 0.0
    assert command < 0.02
    assert measured == 0.0


def test_a_slow_loop_still_ramps_at_the_intended_acceleration():
    """Same wall-clock, same speed, whatever the loop rate.

    The field run had a median period of 0.194 s against a nominal 0.1 s. A
    fixed dt halves the ramp; a measured one does not.
    """
    fast = ramp(20, 0.1)                       # 2.0 s at 10 Hz
    slow = ramp(10, 0.2)                       # 2.0 s at 5 Hz
    assert fast == pytest.approx(slow, abs=1e-9)
    assert fast == pytest.approx(min(A_MAX * 2.0, V_MAX), abs=1e-9)


def test_a_stalled_cycle_cannot_credit_itself_the_whole_gap():
    """p99 was 2.14 s. Integrating a_max over that is a 0.38 m/s step onto
    the wire from one cycle to the next, which is not a ramp."""
    one_long = advance_command(0.0, 0.0, np.array([A_MAX, 0.0]), 2.14,
                               V_MAX, YAW_MAX)[0]
    assert one_long <= A_MAX * MAX_COMMAND_GAP_S + 1e-9


def test_a_zero_length_cycle_does_not_divide_or_stall():
    speed, yaw = advance_command(0.2, 0.1, np.array([A_MAX, 0.1]), 0.0,
                                 V_MAX, YAW_MAX)
    assert speed == pytest.approx(0.2 + A_MAX * MIN_COMMAND_STEP_S)
    assert yaw > 0.1


def test_the_caps_hold_whatever_the_solver_asks():
    speed, yaw = advance_command(0.55, 0.45, np.array([10.0, 10.0]), 0.5,
                                 V_MAX, YAW_MAX)
    assert speed == pytest.approx(V_MAX)
    assert yaw == pytest.approx(YAW_MAX)
    speed, yaw = advance_command(0.05, -0.45, np.array([-10.0, -10.0]), 0.5,
                                 V_MAX, YAW_MAX)
    assert speed == 0.0
    assert yaw == pytest.approx(-YAW_MAX)


def test_deceleration_still_reaches_zero():
    speed = 0.6
    for _ in range(30):
        speed, _ = advance_command(speed, 0.0, np.array([-0.6, 0.0]), 0.1,
                                   V_MAX, YAW_MAX)
    assert speed == 0.0


# ------------------------------------------------- wiring, read as source

def follower():
    return (SCRIPTS / "mpc_follower.py").read_text(encoding="utf-8")


def test_the_node_ramps_the_command_not_the_measured_velocity():
    text = follower()
    assert "advance_command(" in text
    assert re.search(r"state\[3\]\s*\+\s*u0\[0\]", text) is None, (
        "the node is integrating onto the measured velocity again")


def test_the_node_measures_the_period_instead_of_assuming_it():
    text = follower()
    assert "self.last_command_stamp" in text
    assert "(now - self.last_command_stamp).to_sec()" in text


def test_every_hold_path_drops_the_command_stamp():
    """A hold zeroes the speed; leaving the stamp behind would let the first
    cycle after it credit itself the whole length of the hold."""
    text = follower()
    step = text[text.index("    def step(self):"):]
    stops = step.count("self.send_stop()")
    drops = step.count("self.last_command_stamp = None")
    assert drops >= stops, (
        "%d stop paths but only %d reset the command stamp" % (stops, drops))
