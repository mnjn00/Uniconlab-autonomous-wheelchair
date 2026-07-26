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


def test_climb_assist_is_speed_feedback_bounded_by_tip_governor():
    text = guard_text()
    assert "CLIMB_GAIN" in text
    assert "self.measured_speed" in text
    assert "min(\n                            CLIMB_GAIN * error, self.accel_budget)" in text or \
        "self.accel_budget) * dt" in text
    assert "CLIMB_BOOST_MAX" in text and "CLIMB_BRAKE_MAX" in text


def test_nothing_is_added_to_the_output_past_the_accel_limiter():
    """A second climb-assist integrator once added its boost straight to
    out.linear.x, bypassing the accel budget and the soft-launch cap and
    stacking with the governed one. Assist must reach the output only by
    way of `desired`, which the rate limiter then ramps."""
    text = guard_text()
    fold = text.index("desired = max(0.0, desired + self.climb_boost)")
    limiter = text.index("step = min(desired - self.current_speed, budget * dt)")
    publish = text.index("out.linear.x = max(-COUNTER_SPEED_MAX,")
    assert fold < limiter < publish


def test_assist_cannot_create_motion_from_a_stop_command():
    """Every upstream hold - OBSTACLE, TILT_LIMIT, OFF_BAND, LOCALIZATION_*,
    MANUAL_MODE - arrives here as linear.x == 0. A boost integrated on a
    slope must not survive it, or this stage drives on through the stop."""
    text = guard_text()
    assert "if abs(desired) <= 0.05:\n                    self.climb_boost = 0.0" \
        in text


def test_output_has_an_absolute_ceiling_of_its_own():
    """The gate clamps to HARD_V_LIMIT, then the assist can add up to
    CLIMB_BOOST_MAX on top, so this stage needs its own last word."""
    text = guard_text()
    assert "ABSOLUTE_V_LIMIT = " in text
    assert "out.linear.x = max(-COUNTER_SPEED_MAX," in text
    assert "min(ABSOLUTE_V_LIMIT" in text


def test_climb_boost_resets_on_trip_or_stale():
    text = guard_text()
    trip_block = text.index("if self.tripped:")
    reset = text.index("self.climb_boost = 0.0", trip_block)
    assert reset > trip_block

def test_no_duplicate_unbounded_boost_bypassing_the_accel_governor():
    text = guard_text()
    assert "def update_boost" not in text
    assert "self.boost" not in text
    assert "\nBOOST_MAX = " not in text
    # the output is the rate-limited speed, clamped - never a raw sum
    assert "out.linear.x = max(-COUNTER_SPEED_MAX," in text
    assert "min(ABSOLUTE_V_LIMIT" in text


def test_the_output_clamp_does_not_delete_counter_motion():
    """The ceiling clamp must not have a zero floor: counter_motion_target()
    exists to command a small REVERSE while a backward tilt is still
    growing, and max(0.0, ...) silently deleted it."""
    text = guard_text()
    assert "out.linear.x = max(-COUNTER_SPEED_MAX," in text
    assert "out.linear.x = max(0.0," not in text
