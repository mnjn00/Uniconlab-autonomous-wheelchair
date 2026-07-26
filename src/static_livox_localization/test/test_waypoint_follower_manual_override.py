from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import sys

import pytest


pytest.importorskip("rospy")

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import waypoint_follower as module


class FakeAccumulator:
    def __init__(self):
        self.added = []
        self.merged_calls = 0
        self.clear_calls = 0

    def add_cloud(self, message):
        self.added.append(message)

    def merged(self):
        self.merged_calls += 1
        return "fresh-cloud", "fresh-stamp"

    def clear_clouds(self):
        self.clear_calls += 1


@pytest.fixture
def follower(monkeypatch):
    instance = module.WaypointFollower.__new__(module.WaypointFollower)
    instance.enabled = True
    instance.pause_reason = "PAUSED"
    instance.drive_mode = module.AUTO_MODE
    instance.accumulator = FakeAccumulator()
    instance.cloud = "old-cloud"
    instance.cloud_stamp = "old-stamp"
    instance.send_stop = Mock()
    monkeypatch.setattr(module.rospy.Time, "now", lambda: "now")
    monkeypatch.setattr(module.rospy, "logwarn", Mock())
    monkeypatch.setattr(module.rospy, "loginfo", Mock())
    return instance


def test_manual_mode_latches_running_follower_paused_and_clears_cloud(follower):
    follower.on_wheel_status(SimpleNamespace(data=[72, 77]))

    assert follower.drive_mode == 77
    assert follower.enabled is False
    assert follower.pause_reason == "JOYSTICK_OVERRIDE"
    assert follower.cloud is None
    assert follower.accumulator.clear_calls == 1
    follower.send_stop.assert_called_once_with()


def test_returning_to_auto_does_not_resume_after_override(follower):
    follower.on_wheel_status(SimpleNamespace(data=[72, 77]))
    follower.on_wheel_status(SimpleNamespace(data=[72, module.AUTO_MODE]))

    assert follower.drive_mode == module.AUTO_MODE
    assert follower.enabled is False
    assert follower.pause_reason == "JOYSTICK_OVERRIDE"
    follower.send_stop.assert_called_once_with()


def test_start_is_rejected_while_joystick_has_manual_control(follower):
    follower.drive_mode = 77

    response = follower.on_start(SimpleNamespace(data=True))

    assert response.success is False
    assert response.message == "AUTO_MODE_REQUIRED"
    assert follower.enabled is False
    assert follower.pause_reason == "AUTO_MODE_REQUIRED"
    follower.send_stop.assert_called_once_with()


def test_explicit_start_succeeds_only_after_auto_mode_confirmed(follower):
    follower.drive_mode = module.AUTO_MODE

    response = follower.on_start(SimpleNamespace(data=True))

    assert response.success is True
    assert response.message == "ENABLED"
    assert follower.enabled is True


def test_paused_follower_does_not_decode_or_merge_cloud(follower):
    follower.enabled = False
    message = object()

    follower.on_cloud(message)

    assert follower.accumulator.added == []
    assert follower.accumulator.merged_calls == 0


def test_manual_mode_does_not_decode_cloud_even_if_enabled_flag_is_stale(follower):
    follower.enabled = True
    follower.drive_mode = 77
    message = object()

    follower.on_cloud(message)

    assert follower.accumulator.added == []
    assert follower.accumulator.merged_calls == 0


def test_auto_mode_enabled_follower_processes_fresh_cloud(follower):
    message = object()

    follower.on_cloud(message)

    assert follower.accumulator.added == [message]
    assert follower.accumulator.merged_calls == 1
    assert follower.cloud == "fresh-cloud"
