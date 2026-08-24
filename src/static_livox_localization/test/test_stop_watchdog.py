"""Stop intent must be honoured by both measured wheels.

These tests model the actual command chain:
  safety_gate /cmd_vel_gated -> tip_guard /cmd_vel -> wheel base/status.
The state machine is deliberately tested without a ROS installation, while
the final tests exercise the node callbacks and published alarm messages.
"""

import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


class Message:
    def __init__(self, data=None):
        self.data = data


class Twist:
    def __init__(self, linear=0.0, angular=0.0):
        self.linear = types.SimpleNamespace(x=float(linear))
        self.angular = types.SimpleNamespace(z=float(angular))


class FakeTime:
    current = 0.0

    @classmethod
    def now(cls):
        return types.SimpleNamespace(to_sec=lambda: cls.current)


def install_ros_stubs():
    rospy = sys.modules.setdefault("rospy", types.ModuleType("rospy"))
    rospy.Time = FakeTime
    rospy.init_node = lambda *args, **kwargs: None
    rospy.get_param = lambda name, default=None: default
    rospy.Publisher = lambda *args, **kwargs: None
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logerr = lambda *args, **kwargs: None
    rospy.spin = lambda: None

    geometry_msgs = sys.modules.setdefault(
        "geometry_msgs", types.ModuleType("geometry_msgs"))
    geometry_msg = sys.modules.setdefault(
        "geometry_msgs.msg", types.ModuleType("geometry_msgs.msg"))
    geometry_msg.Twist = Twist
    geometry_msgs.msg = geometry_msg

    std_msgs = sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs"))
    std_msg = sys.modules.setdefault(
        "std_msgs.msg", types.ModuleType("std_msgs.msg"))
    std_msg.Int16 = Message
    std_msg.Int16MultiArray = Message
    std_msg.String = Message
    std_msgs.msg = std_msg


def load(name):
    install_ros_stubs()
    spec = importlib.util.spec_from_file_location(
        name, SCRIPTS / ("%s.py" % name))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


sw = load("stop_watchdog")
C, W, S = ord("C"), ord("W"), ord("S")
STILL, ROLLING = 0x21, 0x3B


def status(left_byte, right_byte, mode=sw.AUTO_MODE,
           left_direction=C, right_direction=C):
    return [
        72, mode,
        left_direction, left_byte,
        right_direction, right_byte,
        ord("O"), 88, 0, 13, 10,
    ]


def arm(check, at=0.0):
    check.observe_gated_command(0.0, 0.0, at)


def keep_fresh(check, at, final_linear=0.3):
    check.observe_gated_command(0.0, 0.0, at)
    check.observe_final_command(final_linear, 0.0, at)


def test_gated_stop_arms_and_drive_command_disarms_the_check():
    check = sw.StopHonouredCheck()

    check.observe_gated_command(0.0, 0.0, 1.0)
    assert check.intent_since == pytest.approx(1.0)

    check.observe_gated_command(0.2, 0.0, 1.1)
    assert check.intent_since is None
    assert check.latched is False


def test_tip_guard_deceleration_does_not_start_final_stop_grace():
    check = sw.StopHonouredCheck(not_slowing_window_s=10.0)
    arm(check)
    check.observe_final_command(0.3, 0.0, 0.0)
    assert check.observe_status(
        status(ROLLING, ROLLING), 0.29, sw.AUTO_MODE) is None

    check.observe_gated_command(0.0, 0.0, 0.45)
    check.observe_final_command(0.0, 0.0, 0.50)
    keep_fresh(check, 0.89, final_linear=0.0)
    assert check.observe_status(
        status(ROLLING, ROLLING), 0.89, sw.AUTO_MODE) is None
    keep_fresh(check, 0.91, final_linear=0.0)
    fault = check.observe_status(
        status(ROLLING, ROLLING), 0.91, sw.AUTO_MODE)
    assert fault["code"] == "STOP_NOT_HONOURED"


def test_one_wheel_pivot_is_caught_before_generic_stop_timeout():
    check = sw.StopHonouredCheck(not_slowing_window_s=10.0)
    keep_fresh(check, 0.0)
    assert check.observe_status(
        status(STILL, ROLLING), 0.0, sw.AUTO_MODE) is None
    keep_fresh(check, 0.14)
    assert check.observe_status(
        status(STILL, ROLLING), 0.14, sw.AUTO_MODE) is None
    keep_fresh(check, 0.16)

    fault = check.observe_status(
        status(STILL, ROLLING), 0.16, sw.AUTO_MODE)

    assert fault["code"] == "ONE_WHEEL_PIVOT"
    assert fault["left_mps"] == pytest.approx(0.0)
    assert fault["right_mps"] == pytest.approx(0.722, abs=0.002)
    assert fault["pivot_yaw_rps"] > 0.5
    assert check.observe_status(
        status(STILL, ROLLING), 0.18, sw.AUTO_MODE) is None


def test_not_slowing_is_distinguished_from_a_generic_failed_stop():
    check = sw.StopHonouredCheck()
    keep_fresh(check, 0.0)
    assert check.observe_status(
        status(ROLLING, ROLLING), 0.0, sw.AUTO_MODE) is None
    keep_fresh(check, 0.51)

    fault = check.observe_status(
        status(0x3A, 0x3A), 0.51, sw.AUTO_MODE)

    assert fault["code"] == "NOT_SLOWING"
    assert fault["intent_age_s"] == pytest.approx(0.51)


def test_final_zero_that_wheels_ignore_is_a_generic_failed_stop():
    check = sw.StopHonouredCheck(not_slowing_window_s=10.0)
    keep_fresh(check, 0.0, final_linear=0.0)
    assert check.observe_status(
        status(ROLLING, ROLLING), 0.0, sw.AUTO_MODE) is None
    keep_fresh(check, 0.41, final_linear=0.0)

    fault = check.observe_status(
        status(ROLLING, ROLLING), 0.41, sw.AUTO_MODE)

    assert fault["code"] == "STOP_NOT_HONOURED"
    assert fault["final_age_s"] == pytest.approx(0.41)


def test_manual_stale_malformed_and_unarmed_statuses_do_not_alarm():
    check = sw.StopHonouredCheck()
    assert check.observe_status(
        status(STILL, ROLLING), 1.0, sw.AUTO_MODE) is None

    keep_fresh(check, 2.0)
    assert check.observe_status([72, sw.AUTO_MODE], 2.2, sw.AUTO_MODE) is None
    assert check.observe_status(
        status(STILL, ROLLING), 2.2, sw.MANUAL_MODE) is None
    assert check.observe_status(
        status(STILL, ROLLING), 2.31, sw.AUTO_MODE) is None


def test_fault_payload_has_stable_diagnostic_fields():
    check = sw.StopHonouredCheck(not_slowing_window_s=10.0)
    keep_fresh(check, 10.0)
    check.observe_status(status(STILL, ROLLING), 10.0, sw.AUTO_MODE)
    keep_fresh(check, 10.16)
    fault = check.observe_status(
        status(STILL, ROLLING), 10.16, sw.AUTO_MODE)

    assert set(fault) == {
        "code", "reason", "left_mps", "right_mps", "pivot_yaw_rps",
        "intent_age_s", "gated_age_s", "final_age_s",
    }


def test_speed_decoding_matches_direction_and_base_protocol():
    assert sw.wheel_speed(C, 0x21) == pytest.approx(0.0)
    assert sw.wheel_speed(C, 0x3B) == pytest.approx(0.722, abs=0.002)
    assert sw.wheel_speed(W, 0x37) == pytest.approx(-0.611, abs=0.002)
    assert sw.wheel_speed(S, 0x3B) == pytest.approx(0.0)
    assert sw.reported_wheel_speeds(status(ROLLING, STILL))[0] > 0.7
    assert sw.reported_wheel_speeds([72, sw.AUTO_MODE]) is None


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_ros_node_subscribes_to_intent_and_final_commands(monkeypatch):
    publishers = {}
    subscribers = {}

    def publisher(topic, _type, **_kwargs):
        publishers[topic] = RecordingPublisher()
        return publishers[topic]

    def subscriber(topic, _type, callback, **_kwargs):
        subscribers[topic] = callback

    monkeypatch.setattr(sw.rospy, "Publisher", publisher)
    monkeypatch.setattr(sw.rospy, "Subscriber", subscriber)

    sw.StopWatchdog()

    assert subscribers["/cmd_vel_gated"].__name__ == "on_gated_command"
    assert subscribers["/cmd_vel"].__name__ == "on_final_command"
    assert "wheel_cmd" not in subscribers


def test_ros_node_publishes_structured_alarm_and_mode_77_once(monkeypatch):
    publishers = {}

    def publisher(topic, _type, **_kwargs):
        publishers[topic] = RecordingPublisher()
        return publishers[topic]

    monkeypatch.setattr(sw.rospy, "Publisher", publisher)
    monkeypatch.setattr(sw.rospy, "Subscriber", lambda *args, **kwargs: None)
    node = sw.StopWatchdog()
    node.tx_diag = {"tx_fail": 0, "tx_slow": 0}

    FakeTime.current = 20.0
    node.on_gated_command(Twist(0.0, 0.0))
    node.on_final_command(Twist(0.3, 0.0))
    node.on_status(Message(status(STILL, ROLLING)))
    FakeTime.current = 20.16
    node.on_gated_command(Twist(0.0, 0.0))
    node.on_final_command(Twist(0.3, 0.0))
    node.on_status(Message(status(STILL, ROLLING)))
    node.on_status(Message(status(STILL, ROLLING)))

    alarm_messages = publishers["stop_watchdog/alarm"].messages
    mode_messages = publishers["mode_cmd"].messages
    assert len(alarm_messages) == 1
    alarm = json.loads(alarm_messages[0].data)
    assert alarm["code"] == "ONE_WHEEL_PIVOT"
    assert alarm["uart_tx"] == {"tx_fail": 0, "tx_slow": 0}
    assert [message.data for message in mode_messages] == [sw.MANUAL_MODE]
