"""The wheel encoder, checked against the commands that failed in the field.

On 2026-07-29 the follower held its maximum +0.5 rad/s for four seconds
while the chair rotated at 0.03-0.05 rad/s, then swung at 0.57 rad/s once
the wheels sped up, carrying it onto a kerb. The encoded difference was
right the whole time; what was missing was enough absolute wheel speed for
the loaded chair to move at all. These cases pin that the same inputs now
produce a command the base can act on, and that the yaw asked for is never
altered to get there.
"""

import importlib.util
import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "tools" / "base_model_wheel_cmd_guard.py"


def load():
    """Import the guard for its encoding maths alone.

    The node half needs a ROS runtime that is not present outside the
    container, while compute_wheel_command is pure arithmetic - so the ROS
    imports are satisfied with stubs and the arithmetic is tested as-is,
    from the same file that is deployed.
    """
    for name in ("rospy", "geometry_msgs", "geometry_msgs.msg",
                 "std_msgs", "std_msgs.msg"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["geometry_msgs.msg"].Twist = type("Twist", (), {})
    sys.modules["std_msgs.msg"].Int16MultiArray = type(
        "Int16MultiArray", (), {})
    spec = importlib.util.spec_from_file_location("wheel_cmd_guard", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guard = load()


def decode(command):
    """Wheel speeds in km/h, signed, from an encoded command."""
    def one(direction, magnitude):
        counts = magnitude - guard.MAGNITUDE_OFFSET
        speed = counts / guard.COUNTS_PER_KMH
        return -speed if direction == 87 else speed
    return one(command[0], command[1]), one(command[2], command[3])


def yaw_of(command):
    left, right = decode(command)
    return (right - left) / 3.6 / guard.WHEEL_SEPARATION_M


def test_the_field_command_now_moves_the_wheels():
    """v=0.19, w=+0.5 is what the follower was sending while nothing
    happened: the faster wheel encoded 0.9 km/h, below what turns the
    loaded chair."""
    command = guard.compute_wheel_command(0.19, 0.5)
    assert command is not None
    left, right = decode(command)
    assert max(abs(left), abs(right)) >= guard.TURN_AUTHORITY_KMH - 0.05


def test_finding_turn_authority_does_not_change_the_yaw_asked_for():
    """Both wheels move by the same amount, so the difference - and with it
    the yaw rate - survives untouched. A boost that bent the turn would be
    worse than the stall it fixes."""
    for linear, angular in ((0.19, 0.5), (0.12, -0.4), (0.0, 0.3),
                            (0.25, 0.15)):
        command = guard.compute_wheel_command(linear, angular)
        assert command is not None
        assert abs(yaw_of(command) - angular) <= 0.06, (linear, angular)


def test_the_boost_cannot_run_the_chair_forward():
    """Turn authority costs forward speed, so it is capped: no combination
    may push the mean wheel speed past the documented limit."""
    worst = 0.0
    for i in range(0, 61):
        angular = -0.6 + i * 0.02
        for j in range(0, 16):
            linear = j * 0.1
            command = guard.compute_wheel_command(linear, angular)
            if command is None:
                continue
            left, right = decode(command)
            worst = max(worst, (left + right) / 2.0 / 3.6 - linear)
    assert worst <= guard.TURN_AUTHORITY_MAX_LINEAR_MPS + 0.03, worst


def test_straight_line_commands_are_untouched():
    """The floor only applies to turns; driving straight must encode the
    speed it was given."""
    for linear in (0.2, 0.6, 1.0):
        command = guard.compute_wheel_command(linear, 0.0)
        left, right = decode(command)
        assert abs(left - linear * 3.6) <= 0.06
        assert abs(right - linear * 3.6) <= 0.06


def test_speed_is_rounded_rather_than_dragged_toward_zero():
    """Truncation lost up to a full count on every wheel, which at turning
    speeds was most of the command."""
    left, _ = decode(guard.compute_wheel_command(0.0, 0.0) or (83, 33, 83, 33, 79))
    assert left == 0.0
    command = guard.compute_wheel_command(0.11, 0.0)   # 0.396 km/h
    left, right = decode(command)
    assert left == pytest.approx(0.4, abs=1e-9)
    assert right == pytest.approx(0.4, abs=1e-9)


def test_invalid_and_out_of_range_requests_are_still_refused():
    assert guard.compute_wheel_command(float("nan"), 0.0) is None
    assert guard.compute_wheel_command(0.5, float("inf")) is None
    assert guard.compute_wheel_command(-0.1, 0.0) is None
    assert guard.compute_wheel_command(guard.MAX_LINEAR_MPS + 0.1, 0.0) is None
    assert guard.compute_wheel_command(0.5, guard.MAX_ANGULAR_RAD_S + 0.1) is None
    assert guard.compute_wheel_command("x", 0.0) is None


def test_a_stopped_command_stays_stopped():
    assert guard.compute_wheel_command(0.0, 0.0) == guard.STOP_COMMAND


def test_stop_ramp_uses_each_measured_wheel_speed_and_direction():
    command = guard.stop_ramp_command(0.60, -0.40)

    assert command == (67, 49, 87, 43, 79)
    left, right = decode(command)
    assert left / 3.6 == pytest.approx(0.45, abs=0.02)
    assert right / 3.6 == pytest.approx(-0.27, abs=0.02)


def test_stop_ramp_switches_each_wheel_to_stop_at_terminal_speed():
    assert guard.stop_ramp_command(0.10, -0.06) == guard.STOP_COMMAND


def test_invalid_stop_ramp_measurement_fails_safe_per_wheel():
    command = guard.stop_ramp_command(float("nan"), 0.60)
    assert command[:2] == guard.STOP_COMMAND[:2]
    assert command[2:] == (67, 49, 79)


def test_zero_velocity_uses_fresh_measured_speed_ramp(monkeypatch):
    node = guard.WheelCommandGuard.__new__(guard.WheelCommandGuard)
    node.mode = guard.AUTO_MODE
    node.fault_latched = False
    node.measured_left_mps = 0.60
    node.measured_right_mps = -0.40
    node.measured_stamp = 9.80
    published = []
    node.publish = published.append
    monkeypatch.setattr(
        guard.rospy,
        "Time",
        types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_sec=lambda: 10.0)),
        raising=False,
    )
    message = types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.0),
        angular=types.SimpleNamespace(z=0.0),
        _connection_header={"callerid": guard.EXPECTED_CMD_CALLER},
    )

    node.on_velocity(message)

    assert published == [(67, 49, 87, 43, 79)]


def test_zero_velocity_with_stale_measurement_fails_safe(monkeypatch):
    node = guard.WheelCommandGuard.__new__(guard.WheelCommandGuard)
    node.mode = guard.AUTO_MODE
    node.fault_latched = False
    node.measured_left_mps = 0.60
    node.measured_right_mps = 0.60
    node.measured_stamp = 9.60
    published = []
    node.publish = published.append
    monkeypatch.setattr(
        guard.rospy,
        "Time",
        types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_sec=lambda: 10.0)),
        raising=False,
    )
    message = types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.0),
        angular=types.SimpleNamespace(z=0.0),
        _connection_header={"callerid": guard.EXPECTED_CMD_CALLER},
    )

    node.on_velocity(message)

    assert published == [guard.STOP_COMMAND]
