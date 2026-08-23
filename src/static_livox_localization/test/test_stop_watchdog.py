"""A commanded stop that the wheels ignore has to be caught inside a second.

The fault it watches for ran 3.73 s on 2026-08-16 and 12.97 s on 08-19, the
second one spinning the chair about 2.9 times on the spot after it had
finished the route. Both were visible on /wheel_status the whole time and
nothing was looking.

The command stream runs at 50 Hz on the real chair, so these tests feed it
that way. A test that issues one frame and then jumps a second forward is
exercising the stale-command branch, which belongs to uart.py's watchdog.
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


sw = load("stop_watchdog")
S, C = ord("S"), ord("C")
# 0x21 is the zero of the protocol; 0x3B decodes to 0.72 m/s, the speed
# the left wheel held for 12.97 s on 2026-08-19.
STILL, ROLLING = 0x21, 0x3B


def stop_frame():
    return [S, STILL, S, STILL, ord("O")]


def drive_frame():
    return [C, ROLLING, C, ROLLING, ord("O")]


def status(left_byte, right_byte, mode=sw.AUTO_MODE):
    return [72, mode, C, left_byte, C, right_byte, ord("O"), 88, 0, 13, 10]


def stream(check, command, left, right, start_s, duration_s,
           mode=sw.AUTO_MODE, hz=50.0):
    """Run the bus for a while and collect whatever the check reports."""
    reasons = []
    step = 1.0 / hz
    now = start_s
    while now <= start_s + duration_s + 1e-9:
        check.observe_command(command(), now)
        reason = check.observe_status(status(left, right, mode), now, mode)
        if reason:
            reasons.append((now, reason))
        now += step
    return reasons


def test_a_stop_the_wheels_obey_raises_nothing():
    check = sw.StopHonouredCheck()
    assert stream(check, stop_frame, STILL, STILL, 0.0, 5.0) == []


def test_wheels_still_turning_past_the_grace_are_a_fault():
    check = sw.StopHonouredCheck(grace_s=0.4)
    inside = stream(check, stop_frame, ROLLING, STILL, 0.0, 0.3)
    assert inside == [], "a wheel coasting down inside the grace is not a fault"
    outside = stream(check, stop_frame, ROLLING, STILL, 0.32, 0.3)
    assert len(outside) == 1
    assert "not honoured" in outside[0][1]


def test_it_reports_once_and_not_every_frame():
    """An alarm at 100 Hz is not an alarm."""
    check = sw.StopHonouredCheck(grace_s=0.4)
    assert len(stream(check, stop_frame, ROLLING, STILL, 0.0, 3.0)) == 1


def test_it_arms_again_after_the_wheels_finally_stop():
    check = sw.StopHonouredCheck(grace_s=0.4)
    assert len(stream(check, stop_frame, ROLLING, STILL, 0.0, 1.0)) == 1
    assert stream(check, stop_frame, STILL, STILL, 1.0, 0.5) == []
    assert stream(check, drive_frame, ROLLING, ROLLING, 1.5, 0.5) == []
    assert len(stream(check, stop_frame, ROLLING, STILL, 2.0, 1.0)) == 1


def test_manual_mode_is_not_its_business():
    """In manual the base takes the joystick and ignores our frames, so a
    turning wheel under a stop command is the operator driving."""
    check = sw.StopHonouredCheck(grace_s=0.4)
    assert stream(check, stop_frame, ROLLING, ROLLING, 0.0, 3.0,
                  mode=sw.MANUAL_MODE) == []


def test_a_stale_command_stream_belongs_to_the_uart_watchdog():
    """Starved commands are a different fault with a different owner; this
    node must not also claim it, or one cause raises two alarms."""
    check = sw.StopHonouredCheck(grace_s=0.4, command_fresh_s=0.3)
    check.observe_command(stop_frame(), 0.0)
    assert check.observe_status(status(ROLLING, STILL), 1.0, sw.AUTO_MODE) is None


def test_a_drive_command_clears_the_arming():
    check = sw.StopHonouredCheck(grace_s=0.4)
    assert stream(check, drive_frame, ROLLING, ROLLING, 0.0, 3.0) == []


def test_the_08_19_numbers_are_caught_inside_a_second():
    """Left byte 0x37 is 0.72 m/s, right stopped - the frame pair that ran
    for 12.97 s. It has to be reported inside the grace plus one cycle."""
    check = sw.StopHonouredCheck()
    reasons = stream(check, stop_frame, ROLLING, STILL, 100.0, 13.0)
    assert len(reasons) == 1
    when, reason = reasons[0]
    assert when - 100.0 < sw.GRACE_S + 0.05
    assert "left 0.72" in reason


def test_speed_decoding_matches_the_base_protocol():
    assert abs(sw.wheel_speed(0x21) - 0.0) < 1e-9
    assert abs(sw.wheel_speed(0x3B) - 0.722) < 0.002
    assert abs(sw.wheel_speed(0x37) - 0.611) < 0.002
    assert sw.commanded_stop([S, STILL, S, STILL, ord("O")])
    assert not sw.commanded_stop([C, ROLLING, S, STILL, ord("O")])
    assert sw.reported_motion(status(ROLLING, STILL))[0] > 0.7
    assert sw.reported_motion([72, 65]) is None
