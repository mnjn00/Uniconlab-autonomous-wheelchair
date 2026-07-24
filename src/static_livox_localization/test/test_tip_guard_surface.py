from pathlib import Path


ROOT = Path(__file__).parents[1]


def guard_text():
    return (ROOT / "scripts" / "tip_guard.py").read_text(encoding="utf-8")


def test_guard_sits_between_gated_command_and_final_cmd_vel():
    text = guard_text()
    assert '"/cmd_vel_gated"' in text
    assert '"/cmd_vel"' in text


def test_guard_predicts_deviation_ahead_instead_of_reacting_to_angle():
    text = guard_text()
    assert "LOOKAHEAD_S" in text
    assert "predicted = dev + self.pitch_rate * LOOKAHEAD_S" in text


def test_trip_logic_uses_terrain_baseline_so_hills_do_not_trip():
    text = guard_text()
    assert "BASELINE_TAU_S" in text
    assert "self.fused_pitch - self.baseline_pitch" in text
    assert "growing = (predicted * dev) >= 0.0" in text
    assert "return growing and abs(predicted) > TRIP_DEV_RAD" in text


def test_extreme_raw_rotation_rate_alone_trips_without_needing_axis_confirmation():
    text = guard_text()
    should_trip = text.index("def should_trip")
    rate_check = text.index("abs(self.pitch_rate) > TRIP_RATE_RAD_S", should_trip)
    axis_check = text.index("if not self.axis_config_ok", should_trip)
    assert should_trip < rate_check < axis_check


def test_release_is_self_recovering_on_slopes():
    text = guard_text()
    assert "abs(self.deviation()) < RELEASE_DEV_RAD" in text
    assert "abs(self.pitch_rate) < RELEASE_RATE_RAD_S" in text


def test_uncorrelated_imu_odometry_disables_predictive_trip_and_caps_accel():
    text = guard_text()
    assert "predictive trip DISABLED" in text
    assert "FALLBACK_ACCEL" in text
    fallback_use = text.index("ceiling = GOVERNOR_MAX_ACCEL if self.axis_config_ok "
                              "else FALLBACK_ACCEL")
    assert fallback_use > 0


def test_governor_throttles_down_on_caution_rate_and_recovers_slowly():
    text = guard_text()
    assert "CAUTION_RATE_RAD_S" in text
    assert "GOVERNOR_CUT_FACTOR" in text
    assert "GOVERNOR_RECOVER_PER_S" in text


def test_stale_or_tripped_forces_zero_and_node_always_stops_on_shutdown():
    text = guard_text()
    assert "desired = self.counter_motion_target()" in text
    assert "elif stale:\n                desired = 0.0" in text
    assert "rospy.on_shutdown(lambda: self.pub.publish(Twist()))" in text


def test_counter_motion_defaults_off_and_requires_verified_axis():
    text = guard_text()
    assert '"~enable_counter_motion", False' in text
    assert "not (self.enable_counter_motion and self.axis_config_ok)" in text
    assert "return 0.0" in text
    assert "COUNTER_SPEED_MAX" in text


def test_never_claims_direct_lidar_ground_tilt_sensing():
    text = guard_text()
    assert "is NOT usable here" in text
    assert "fused pitch from /Odometry" in text
