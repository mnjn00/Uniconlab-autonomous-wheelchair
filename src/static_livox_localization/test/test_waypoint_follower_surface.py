from pathlib import Path


ROOT = Path(__file__).parents[1]


def follower_text():
    return (ROOT / "scripts" / "waypoint_follower.py").read_text(encoding="utf-8")


def policy_text():
    return (ROOT / "scripts" / "localization_policy.py").read_text(
        encoding="utf-8")


def test_follower_starts_paused_and_requires_explicit_start():
    text = follower_text()
    assert "self.enabled = False" in text
    assert '"/waypoint_follower/start"' in text


def test_follower_always_stops_on_shutdown():
    text = follower_text()
    assert "rospy.on_shutdown(self.send_stop)" in text


def test_follower_holds_on_lost_pose_cloud_or_manual_mode():
    text = follower_text() + policy_text()
    for guard in ("NO_POSE", "NO_CLOUD", "LOCALIZATION_LOST", "MANUAL_MODE"):
        assert guard in text
    assert "AUTO_MODE = 65" in text


def test_follower_keeps_wheelchair_inside_map_safety_band():
    text = follower_text()
    # SafetyBand now lives in its own ROS-free module so the geometry can
    # be unit-tested against the shipped band; see test_safety_band.py
    assert "from safety_band import SafetyBand" in text
    assert "self.band.clamp(target)" in text
    assert '"OFF_BAND"' in text
    band = (ROOT / "scripts" / "safety_band.py").read_text(encoding="utf-8")
    assert "CHAIR_HALF_WIDTH" in band and "BAND_MARGIN" in band


def test_follower_speed_policy_is_bounded():
    text = follower_text()
    assert "MAX_SPEED = 1.5" in text
    assert "GUARD_STOP_PER_MPS" in text
    assert "SLOPE_SPEED = 0.6" in text
    assert "MAX_ACCEL" in text and "MAX_DECEL" in text


def test_follower_bypasses_static_obstacles_only_inside_band():
    text = follower_text()
    assert "BYPASS_AFTER_S" in text
    assert "bypass_target_ok" in text
    wait = text.index("no clear side - waiting")
    bypass = text.index("bypassing static obstacle")
    assert bypass < wait


def test_missing_cloud_data_is_treated_as_blocked():
    text = follower_text()
    assert "return 0.0  # no data = treat as blocked" in text


def test_degraded_localization_times_out_to_a_hold():
    text = follower_text() + policy_text()
    assert "DEGRADED_STOP_S" in text
    assert '"LOCALIZATION_DEGRADED_TIMEOUT"' in text
    assert "self.degraded_since" in text


def test_follower_delegates_all_localization_states_to_fail_closed_policy():
    text = follower_text()
    assert "from localization_policy import localization_hold_reason" in text
    assert "reason = localization_hold_reason(" in text


def test_pure_pursuit_resyncs_globally_when_position_jumps_backward():
    text = follower_text()
    assert "NEAREST_RESYNC_M" in text
    assert "global_index = int(np.argmin(d))" in text
    assert "resyncing" in text
