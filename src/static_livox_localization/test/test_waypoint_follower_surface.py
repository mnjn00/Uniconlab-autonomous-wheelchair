from pathlib import Path


ROOT = Path(__file__).parents[1]


def follower_text():
    return (ROOT / "scripts" / "waypoint_follower.py").read_text(encoding="utf-8")


def policy_text():
    return (ROOT / "scripts" / "localization_policy.py").read_text(
        encoding="utf-8")


def method_body(text, name):
    start = text.index(f"    def {name}(")
    end = text.find("\n    def ", start + 1)
    return text[start:] if end < 0 else text[start:end]


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
    assert "MAX_SPEED = 1.0" in text
    assert "SLOPE_SPEED = 0.3" in text
    assert "MAX_ACCEL" in text and "MAX_DECEL" in text


def test_follower_obstacle_detection_uses_forward_fov_cone():
    text = follower_text()
    # The follower's own raw five-point corridor check was removed on
    # 2026-08-05: a source that cannot say what it saw was authorising
    # bypass manoeuvres. The same geometry is still asserted against
    # safety_gate.py below, which keeps an independent raw check - it may
    # stop the chair, it just may not steer it.
    assert "def obstacle_distance" not in text
    assert "Threat(distance, UNKNOWN" not in text


def test_obstacle_stop_radius_covers_braking_distance_at_full_speed():
    text = follower_text()
    assert "PoseMotionEstimator" in text
    assert "motion_hold_reason" in text
    assert "stopping_envelope(" in text
    assert "self.motion.linear_speed_mps" in text
    assert "ACCUMULATION_WINDOW_S" in text
    # guard_slow() the method went with the raw check; the slow radius it
    # returned is now computed where it is used, in step().
    assert "self.guard_stop()" in text
    assert "guard_slow = guard_stop + GUARD_SLOW_EXTRA_M" in text


def test_follower_bypasses_static_obstacles_only_inside_band():
    text = follower_text()
    assert "BYPASS_AFTER_S" in text
    assert "bypass_target_ok" in text
    wait = text.index("no side of this has room in the band - waiting")
    bypass = text.index("going round a parked obstacle")
    assert bypass < wait


def test_the_band_still_vets_a_way_round_when_the_policies_are_off():
    """Containment stopping the chair is a judgement and can be switched
    off. The band knowing where there is room to step aside is not, and the
    smallest offset on offer is twice this route's median lateral
    clearance."""
    text = follower_text()
    start = text.index("def take_a_way_round")
    body = text[start:text.index("\n    def ", start + 1)]
    assert "self.bypass_target_ok(offset)" in body


def test_pursuit_enforces_authoritative_drivable_mask():
    text = follower_text()
    assert "from route_mask import RouteMask" in text
    assert "self.drivable_mask = RouteMask(" in text
    for method_name in ("bypass_target_ok", "target_at_lookahead", "safe_target"):
        body = method_body(text, method_name)
        assert "self.drivable_mask" in body


def test_missing_obstacle_data_is_treated_as_blocked():
    """The property outlived the check that used to carry it.

    The raw scan returned 0.0 - blocked - when it had no cloud. With that
    check gone the same guarantee has to come from the only remaining
    source: cluster_threat reports a missing summary as a blocking threat
    at zero distance, and a quiet producer is separately an OVERRIDE hold.
    Silence must never read as clear road.
    """
    text = follower_text()
    assert 'return Threat(0.0, MOVING, "no summary")' in text
    assert "CLUSTERS_STALE" in text


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
            "scripts/motion_safety.py",
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
    accumulator = (ROOT / "scripts" / "scan_accumulator.py").read_text(
        encoding="utf-8")
    assert "body_to_lidar(np.vstack(parts)" in accumulator
    text = follower_text()
    assert "CloudAccumulator" in text and "lidar_extrinsics" in text
    assert 'rospy.get_param("~body_frame_profile")' in text


def test_the_gate_corrects_the_same_frame_as_the_follower():
    """The gate is a second, independent opinion on the forward corridor,
    but not on where things are. If it read the body frame while the
    follower read the lidar frame, the two would disagree about an
    obstacle's position by the extrinsic - so they share one accumulator
    rather than each carrying a copy of the conversion to correct."""
    gate = (ROOT / "scripts" / "safety_gate.py").read_text(encoding="utf-8")
    assert "from scan_accumulator import CloudAccumulator" in gate
    assert "from scan_accumulator import CloudAccumulator" in follower_text()
    assert "class CloudAccumulator" not in gate
    assert "class CloudAccumulator" not in follower_text()
    assert 'rospy.get_param("~body_frame_profile")' in gate


def test_the_gate_uses_the_same_forward_fov_cone():
    gate = (ROOT / "scripts" / "safety_gate.py").read_text(encoding="utf-8")
    assert "FORWARD_FOV_HALF_DEG = 50.0" in gate
    assert "CORRIDOR_MIN_RANGE_M = 0.50" in gate
    assert "azimuth < FORWARD_FOV_HALF_DEG" in gate


def test_gate_checks_rotation_with_pose_derived_motion_and_full_footprint():
    gate = (ROOT / "scripts" / "safety_gate.py").read_text(encoding="utf-8")
    assert "PoseMotionEstimator" in gate
    assert "motion_hold_reason" in gate
    assert "stopping_envelope(" in gate
    assert "swept_footprint_collision(" in gate
    assert "abs(self.raw.angular.z) > MOTION_EPSILON" in gate
    assert "ODOM_STALE_S" in gate


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
