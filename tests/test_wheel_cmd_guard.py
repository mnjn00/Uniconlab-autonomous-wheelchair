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
import math
import subprocess
import sys
import types
from pathlib import Path

import numpy as np
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

SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from priest_controller import (
        DEFAULT_CONTROLLER_LIMITS,
        DriveCommand,
        Pose2D,
        command_for,
    )
    from wheel_command_model import effective_twist, encode_wheel_command
finally:
    sys.path.remove(str(SCRIPTS))


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


def timed_plan(speed, yaw_rate):
    return types.SimpleNamespace(
        x=np.array([0.0, speed * 20.0]), y=np.zeros(2),
        times=np.array([0.0, 20.0]),
        velocity_xy_mps=np.tile(np.array([speed, 0.0]), (2, 1)),
        yaw_rad=np.zeros(2), yaw_rate_rps=np.full(2, yaw_rate),
        reason="", usable=True)


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
                            (0.35, 0.15)):
        command = guard.compute_wheel_command(linear, angular)
        assert command is not None
        assert abs(yaw_of(command) - angular) <= 0.06, (linear, angular)


@pytest.mark.parametrize("linear,angular", [
    (0.30, 0.10), (0.30, 0.051), (0.30, 0.05), (0.25, 0.15),
])
def test_turns_below_measured_wheel_authority_are_refused(
        linear, angular):
    assert guard.compute_wheel_command(linear, angular) is None


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
            if abs(yaw_of(command)) > guard.YAW_DEADBAND_RAD_S:
                assert max(abs(left), abs(right)) >= guard.TURN_AUTHORITY_KMH
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


def test_priest_turn_floor_prevents_post_safety_speed_boost():
    under_floor = command_for(
        timed_plan(0.12, 0.10), 0.5, Pose2D(0.06, 0.0, 0.0), 0.12,
        DriveCommand(0.12, 0.0))
    assert under_floor.angular_z_rps == 0.0
    assert under_floor.reason == "TURN_ACCELERATING"

    turning = command_for(
        timed_plan(0.35, 0.10), 0.5, Pose2D(0.175, 0.0, 0.0), 0.35,
        DriveCommand(0.35, 0.0))
    encoded = guard.compute_wheel_command(
        turning.linear_x_mps, turning.angular_z_rps)
    assert encoded is not None and abs(turning.angular_z_rps) > 0.05
    left, right = decode(encoded)
    effective_linear = (left + right) / 2.0 / 3.6
    assert abs(effective_linear - turning.linear_x_mps) <= 0.015


def test_priest_commands_are_exhaustively_safe_on_the_actuator_grid():
    limits = DEFAULT_CONTROLLER_LIMITS
    assert guard.compute_wheel_command is encode_wheel_command
    for target_speed in (0.10, 0.30, 0.60):
        for target_yaw in (-0.50, -0.30, -0.10, 0.0, 0.10, 0.30, 0.50):
            previous = DriveCommand(0.0, 0.0)
            for _ in range(40):
                command = command_for(
                    timed_plan(target_speed, target_yaw), 0.0,
                    Pose2D(0.0, 0.0, 0.0),
                    previous.linear_x_mps, previous, limits)
                assert command.reason in ("", "TURN_ACCELERATING")
                prior_effective = effective_twist(
                    previous.linear_x_mps, previous.angular_z_rps)
                effective = effective_twist(
                    command.linear_x_mps, command.angular_z_rps)
                assert prior_effective is not None and effective is not None
                assert effective.linear_x_mps == pytest.approx(
                    command.linear_x_mps)
                assert effective.angular_z_rps == pytest.approx(
                    command.angular_z_rps)
                acceleration = math.hypot(
                    (effective.linear_x_mps - prior_effective.linear_x_mps)
                    / limits.control_period_s,
                    effective.linear_x_mps * effective.angular_z_rps)
                assert effective.linear_x_mps <= limits.max_speed_mps + 1e-9
                assert abs(effective.angular_z_rps) \
                    <= limits.max_yaw_rate_rps + 1e-9
                assert acceleration <= limits.max_acceleration_mps2 + 1e-9
                previous = command


def test_shared_model_receives_a_catkin_devel_relay():
    cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text(encoding="utf-8")
    programs = cmake.split("catkin_install_python(", 1)[1].split(
        "DESTINATION", 1)[0]
    assert "scripts/wheel_command_model.py" in programs


@pytest.mark.parametrize("layout", ["source", "devel", "install"])
def test_drop_in_guard_finds_sibling_catkin_package_model(tmp_path, layout):
    if layout == "source":
        base_source = tmp_path / "catkin_ws" / "src" / "base_model" / "src"
        model_source = tmp_path / "catkin_ws" / "src" \
            / "static_livox_localization" / "scripts"
    else:
        space = "devel" if layout == "devel" else "install"
        base_source = tmp_path / "catkin_ws" / space / "lib" / "base_model"
        model_source = tmp_path / "catkin_ws" / space / "lib" \
            / "static_livox_localization"
    base_source.mkdir(parents=True)
    model_source.mkdir(parents=True)
    copied_guard = base_source / "wheel_cmd_tmp.py"
    copied_guard.write_text(MODULE_PATH.read_text(encoding="utf-8"),
                            encoding="utf-8")
    copied_model = model_source / "wheel_command_model.py"
    copied_model.write_text(
        (SCRIPTS / "wheel_command_model.py").read_text(encoding="utf-8"),
        encoding="utf-8")
    script = """
import importlib.util
import sys
import types
from pathlib import Path
for name in ('rospy', 'geometry_msgs', 'geometry_msgs.msg',
             'std_msgs', 'std_msgs.msg'):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['geometry_msgs.msg'].Twist = type('Twist', (), {})
sys.modules['std_msgs.msg'].Int16MultiArray = type('Array', (), {})
fake = types.ModuleType('wheel_command_model')
for name in ('COUNTS_PER_KMH', 'MAGNITUDE_OFFSET', 'MAX_ANGULAR_RAD_S',
             'MAX_LINEAR_MPS', 'TURN_AUTHORITY_KMH',
             'TURN_AUTHORITY_MAX_LINEAR_MPS', 'WHEEL_SEPARATION_M',
             'YAW_DEADBAND_RAD_S'):
    setattr(fake, name, 0.0)
fake.STOP_COMMAND = ()
fake.encode_wheel_command = lambda *args: None
sys.modules['wheel_command_model'] = fake
path = Path(sys.argv[1])
spec = importlib.util.spec_from_file_location('deployed_guard', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
assert module.compute_wheel_command(0.35, 0.1) is not None
"""
    result = subprocess.run(
        [sys.executable, "-c", script, str(copied_guard)],
        check=False, capture_output=True, text=True, cwd=tmp_path)

    assert result.returncode == 0, result.stderr
