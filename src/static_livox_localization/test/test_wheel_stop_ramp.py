"""The base is stopped by ramping it down, not by declaring it stopped.

S is not a brake on this hardware. wheel_cmd.py in the 2023 install never
sent it from speed: ObstacleStop re-commanded a fraction of the MEASURED
wheel speed each cycle and only swapped in S once the count fell under 35.
wheel_cmd_tmp.py dropped that, and has been sending S straight from cruise.

Measured 2026-08-23 over every cruise stop of the evening, four for four:
the right wheel braked at 0.67-0.89 m/s^2, the left held its last setpoint
at -0.06 to 0.22, and the chair pivoted at about 71 deg/s toward the wheel
that had stopped.
"""
import importlib.util
import os
import sys
import types
from pathlib import Path

import pytest

SNAPSHOT = (Path(__file__).parents[3] / "docs" / "nuc_snapshot" /
            "base_model_wheel_cmd_tmp.py")
RUNNING = Path("/home/mprp3/catkin_ws/src/base_model/src/wheel_cmd_tmp.py")


def load_snapshot():
    """Import the snapshot with ROS stubbed out.

    The node itself is not in any repository - it lives only on the NUC -
    so what is tested here is the snapshot, and a test below fails if the
    two have drifted apart.
    """
    for name, attrs in (("rospy", ("Publisher", "Subscriber", "Time")),
                        ("std_msgs", ()), ("std_msgs.msg",
                                           ("Int16MultiArray",)),
                        ("geometry_msgs", ()), ("geometry_msgs.msg",
                                                ("Twist",))):
        module = types.ModuleType(name)
        for attr in attrs:
            setattr(module, attr, type(attr, (), {}))
        sys.modules.setdefault(name, module)
    sys.modules["rospy"].init_node = lambda *a, **k: None
    spec = importlib.util.spec_from_file_location("wheel_cmd_tmp", SNAPSHOT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


wheel = load_snapshot()


def decode(direction, magnitude):
    speed = (magnitude - wheel.MAGNITUDE_OFFSET) / wheel.COUNTS_PER_KMH / 3.6
    return {67: speed, 87: -speed}.get(direction, 0.0)


def commanded(left_mps, right_mps):
    out = wheel.stop_ramp_command(left_mps, right_mps)
    return (out[0], decode(out[0], out[1])), (out[2], decode(out[2], out[3]))


def test_a_stop_from_cruise_is_a_drive_command_not_a_stop_byte():
    (direction, speed), _ = commanded(0.80, 0.80)
    assert direction == 67, "S from cruise is the input the base mishandles"
    assert 0.0 < speed < 0.80


def test_the_command_always_asks_for_less_than_the_wheels_are_doing():
    """That is the whole mechanism: a standing request to decelerate."""
    for measured in (0.80, 0.65, 0.50, 0.35, 0.25, 0.20):
        (_, speed), _ = commanded(measured, measured)
        assert speed < measured, measured


def test_it_never_asks_for_more_than_the_chair_can_shed():
    """Taken off the MEASURED speed, so the demand cannot run away from
    what the wheels deliver however fast this is called. At 50 Hz a
    commanded-speed ramp would be asking for 6 m/s^2 by the second cycle."""
    measured = 0.80
    (_, speed), _ = commanded(measured, measured)
    assert measured - speed <= 0.25


def test_below_the_terminal_speed_it_becomes_the_ordinary_stop():
    out = wheel.stop_ramp_command(0.05, 0.05)
    assert tuple(out) == wheel.STOP_COMMAND


def test_a_chair_already_at_rest_is_sent_exactly_what_it_always_was():
    assert tuple(wheel.stop_ramp_command(0.0, 0.0)) == wheel.STOP_COMMAND


def test_a_reversing_wheel_is_ramped_in_its_own_direction():
    (direction, speed), _ = commanded(-0.60, -0.60)
    assert direction == 87
    assert -0.60 < speed < 0.0


def test_one_wheel_at_terminal_stops_both_together():
    """A one-wheel stop would pivot the chair after stop was requested."""
    left, right = commanded(0.70, 0.05)
    assert left[0] == 83
    assert right[0] == 83


def test_it_reaches_rest_from_cruise_in_a_reasonable_time():
    """Simulated against the deceleration the chair actually delivers,
    0.47 m/s^2, measured off the pedestrian stop the same evening."""
    speed = 0.80
    step = 1.0 / 50.0
    elapsed = 0.0
    while elapsed < 5.0:
        out = wheel.stop_ramp_command(speed, speed)
        if tuple(out) == wheel.STOP_COMMAND:
            break
        speed = max(0.0, speed - 0.47 * step)
        elapsed += step
    assert elapsed < 1.6, "took %.2f s to reach the stop byte" % elapsed


def test_a_magnitude_never_leaves_the_protocol():
    for measured in (0.5, 1.0, 2.0, 10.0):
        out = wheel.stop_ramp_command(measured, measured)
        assert 33 <= out[1] <= 127
        assert 33 <= out[3] <= 127


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), None, "x"])
def test_an_unusable_reading_stops_rather_than_guesses(bad):
    out = wheel.stop_ramp_command(bad, bad)
    assert tuple(out) == wheel.STOP_COMMAND


def test_the_status_decoder_matches_the_protocol():
    assert wheel.wheel_speed_mps(67, 33) == 0.0
    assert abs(wheel.wheel_speed_mps(67, 62) - 0.806) < 0.01
    assert abs(wheel.wheel_speed_mps(87, 62) + 0.806) < 0.01
    assert wheel.wheel_speed_mps(83, 62) == 0.0


@pytest.mark.skipif(not RUNNING.exists(), reason="not on the NUC")
def test_the_snapshot_still_matches_what_is_running():
    """This node is in no repository. The snapshot is the only copy under
    review, so a silent edit on the NUC has to fail something."""
    assert SNAPSHOT.read_text() == RUNNING.read_text(), (
        "docs/nuc_snapshot/base_model_wheel_cmd_tmp.py has drifted from "
        "%s - re-snapshot it" % RUNNING)
