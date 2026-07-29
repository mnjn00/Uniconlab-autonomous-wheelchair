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
    assert "from safety_band import SafetyBand" in text
    assert "self.band.clamp(target)" in text
    assert '"OFF_BAND"' in text
    band = (ROOT / "scripts" / "safety_band.py").read_text(encoding="utf-8")
    assert "CHAIR_HALF_WIDTH" in band and "BAND_MARGIN" in band


def test_follower_speed_policy_is_bounded():
    text = follower_text()
    assert "MAX_SPEED = 0.6" in text
    assert "SLOPE_SPEED = 0.3" in text
    assert "MAX_ACCEL" in text and "MAX_DECEL" in text


def test_follower_obstacle_detection_uses_forward_fov_cone():
    text = follower_text()
    assert "FORWARD_FOV_HALF_DEG = 50.0" in text
    assert "CORRIDOR_MIN_RANGE_M = 0.50" in text
    assert "azimuth < FORWARD_FOV_HALF_DEG" in text
    assert "pts[:, 0] > CORRIDOR_MIN_RANGE_M" in text


def test_obstacle_stop_radius_covers_braking_distance_at_full_speed():
    """The stop radius must not be a constant. At MAX_SPEED the chair needs
    MAX_SPEED^2 / (2 * MAX_DECEL) just to brake, so a fixed radius chosen
    for a slower cap puts the stop point past the obstacle."""
    import re

    text = follower_text()
    values = {}
    for name in ("MAX_SPEED", "MAX_DECEL", "GUARD_STOP_MIN_M",
                 "GUARD_STOP_PER_MPS"):
        match = re.search(r"^%s = ([0-9.]+)$" % name, text, re.M)
        assert match, "%s must be a module-level constant" % name
        values[name] = float(match.group(1))

    braking = values["MAX_SPEED"] ** 2 / (2 * values["MAX_DECEL"])
    stop_radius = (values["GUARD_STOP_MIN_M"] +
                   values["GUARD_STOP_PER_MPS"] * values["MAX_SPEED"])
    assert stop_radius > braking, (
        "stop radius %.2f m does not cover %.2f m of braking at %.2f m/s"
        % (stop_radius, braking, values["MAX_SPEED"]))
    assert "self.guard_stop()" in text and "self.guard_slow()" in text


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
    assert "DEGRADED_STOP_S = 3.0" in text
    assert '"LOCALIZATION_DEGRADED_TIMEOUT"' in text
    assert "self.degraded_since" in text


def test_follower_delegates_localization_states_to_fail_closed_policy():
    text = follower_text()
    assert "from localization_policy import localization_hold_reason" in text
    assert "reason = localization_hold_reason(" in text


def test_pure_pursuit_resyncs_after_a_localization_position_jump():
    text = follower_text()
    assert "NEAREST_RESYNC_M = 2.0" in text
    assert "global_index = int(np.argmin(d))" in text
    assert "self.nearest_index = global_index" in text


def test_follower_checks_the_complete_chord_and_holds_if_none_is_safe():
    text = follower_text()
    assert "self.band.chord_is_contained(" in text
    assert '"HOLD:UNSAFE_CHORD"' in text
    assert "self.send_stop()" in text
    assert "BAND_RECOVER_MAX = OFF_BAND_GRACE" in text


def test_lookahead_is_interpolated_instead_of_snapped_to_the_next_waypoint():
    text = follower_text()
    assert "return start + (end - start) * (remaining / segment)" in text
    assert "LOOKAHEAD_BACKOFF_M" in text
    assert "self.chord_speed_cap" in text


def test_field_nodes_and_sibling_policy_modules_are_installed_together():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    for path in (
            "scripts/waypoint_follower.py",
            "scripts/tip_guard.py",
            "scripts/safety_gate.py",
            "scripts/obstacle_clusters.py",
            "scripts/body_frame.py",
            "scripts/localization_policy.py",
            "scripts/safety_band.py",
            "scripts/tip_guard_policy.py"):
        assert path in cmake


def test_scan_geometry_is_corrected_into_the_lidar_frame():
    """FAST-LIO publishes /cloud_registered_body in the IMU body frame. With
    the VN-100 that frame sits 14.5 cm forward, 6.8 cm up and yawed 2.80 deg
    from the lidar, while every geometry constant here - sensor height,
    corridor half-width, guard distances - is a lidar-frame quantity. The
    accumulator must undo the configured extrinsic rather than each constant
    being re-derived."""
    text = follower_text()
    assert "body_to_lidar" in text and "lidar_extrinsics" in text
    assert 'rospy.get_param("~body_frame_profile")' in text
    assert "body_to_lidar(np.vstack(parts)" in text


def test_the_gate_corrects_the_same_frame_independently():
    """The gate is a second, independent opinion on the forward corridor.
    If it read the body frame while the follower read the lidar frame, the
    two would disagree about where an obstacle is by the extrinsic."""
    gate = (ROOT / "scripts" / "safety_gate.py").read_text(encoding="utf-8")
    assert "body_to_lidar" in gate and "lidar_extrinsics" in gate
    assert 'rospy.get_param("~body_frame_profile")' in gate


def test_the_gate_uses_the_same_forward_fov_cone():
    gate = (ROOT / "scripts" / "safety_gate.py").read_text(encoding="utf-8")
    assert "FORWARD_FOV_HALF_DEG = 50.0" in gate
    assert "CORRIDOR_MIN_RANGE_M = 0.50" in gate
    assert "azimuth < FORWARD_FOV_HALF_DEG" in gate


def test_the_route_is_read_in_the_body_frame_it_was_captured_in():
    """A route records the path of FAST-LIO's IMU body origin, so it is only
    comparable to a pose read in the SAME body frame. The two profiles here
    are 15.5 cm and 2.80 deg apart; simulating the follower's own steering
    loop over the 2026-07-27 route, ignoring that costs 7 cm of mean
    cross-track against a 0.45 m kerb clearance budget."""
    text = follower_text()
    assert "pose_correction" in text
    assert 'route["body_frame_profile"]' in text
    # an unlabelled route must fail rather than be guessed at
    assert '"body_frame_profile" not in route' in text
    # and the correction has to reach the pose actually used
    assert "pose @ self.pose_correction" in text
    assert text.index("self.pose_correction = pose_correction") < text.index(
        "pose = pose @ self.pose_correction")


def test_steering_follows_the_recorded_line_not_the_band_midpoint():
    """Aiming at the band's safe_offset displaced the steering target by up
    to 1.10 m from the line a person actually drove, and in the field the
    chair wandered 2.68 m off at wp 7 and later headed for a kerb. The band
    is derived from a step-detection heuristic; it may CONTAIN the chair
    (clamp) but must not COMMAND it away from the proven path."""
    text = follower_text()
    steer = text.index("def target_at_lookahead")
    end = text.index("def ", steer + 10)
    body = text[steer:end]
    assert "recentre" not in body
    assert "self.band.clamp(target)" in body
