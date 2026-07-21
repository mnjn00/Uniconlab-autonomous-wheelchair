from pathlib import Path


ROOT = Path(__file__).parents[1]


def follower_text():
    return (ROOT / "scripts" / "waypoint_follower.py").read_text(encoding="utf-8")


def test_follower_starts_paused_and_requires_explicit_start():
    text = follower_text()
    assert "self.enabled = False" in text
    assert '"/waypoint_follower/start"' in text


def test_follower_always_stops_on_shutdown():
    text = follower_text()
    assert "rospy.on_shutdown(self.send_stop)" in text


def test_follower_holds_on_lost_pose_cloud_or_manual_mode():
    text = follower_text()
    for guard in ("NO_POSE", "NO_CLOUD", "LOCALIZATION_LOST", "MANUAL_MODE"):
        assert guard in text
    assert "AUTO_MODE = 65" in text


def test_follower_guards_drops_and_unclimbable_steps():
    text = follower_text()
    assert "DROP_STEP_M = 0.08" in text
    assert "CLIMB_STEP_M" in text
    assert "corridor_assessment" in text
    stop = text.index("if dist < GUARD_STOP_M")
    assert stop > 0


def test_follower_speed_policy_is_bounded():
    text = follower_text()
    assert "MAX_SPEED = 0.5" in text
    assert "SLOPE_SPEED = 0.3" in text
    assert "MAX_ACCEL" in text and "MAX_DECEL" in text


def test_follower_bypasses_static_obstacles_only_when_side_is_clear():
    text = follower_text()
    assert "BYPASS_AFTER_S" in text
    wait = text.index("no clear side - waiting")
    bypass = text.index("bypassing static obstacle")
    assert bypass < wait


def test_missing_cloud_data_is_treated_as_blocked():
    text = follower_text()
    assert "return 0.0, 0.0  # no data = treat as blocked" in text
