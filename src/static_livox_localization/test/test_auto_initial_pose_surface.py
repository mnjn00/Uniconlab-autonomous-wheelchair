from pathlib import Path


ROOT = Path(__file__).parents[1]


def script_text():
    return (ROOT / "scripts" / "auto_initial_pose.py").read_text(encoding="utf-8")


def test_auto_init_seeds_through_existing_verification_pipeline():
    text = script_text()
    assert "/fast_lio_icp/initialpose" in text
    assert "/fast_lio_icp/enable_auto_correction" in text
    assert "/fast_lio_icp/localization_diagnostics" in text
    assert "tracking_was_verified" in text
    assert 'state["reset_count"]' in text
    assert 'state["sequence"]' in text


def test_each_candidate_has_a_fresh_reset_and_disables_correction_on_failure():
    text = script_text()

    disable_before_seed = text.index("enable(False)")
    publish_seed = text.index("seed_pub.publish(seed)")
    enable_after_seed = text.index("enable(True)")
    assert disable_before_seed < publish_seed < enable_after_seed
    assert "candidate seed was not acknowledged" in text
    assert "disable_correction" in text


def test_initializer_publishes_an_explicit_success_receipt_for_startup():
    text = script_text()

    assert '"/fast_lio_icp/auto_initialization_verified", False' in text
    assert '"/fast_lio_icp/auto_initialization_verified", True' in text


def test_auto_init_falls_back_to_next_candidate_on_rejection():
    text = script_text()
    assert "failed verification, trying next" in text
    assert "--top" not in text or "args.top" in text


def test_auto_init_refuses_low_confidence_global_fallbacks():
    text = script_text()
    assert "min-score" in text or "min_score" in text
    assert "global_search" in text


def test_auto_init_supports_dry_run_without_publishing():
    text = script_text()
    dry_run_guard = text.index("if not args.dry_run")
    known_start_attempt = text.index("try_candidate(route_prior")
    assert dry_run_guard < known_start_attempt


def test_launch_exposes_optional_auto_init_node():
    launch = (ROOT / "launch" / "moving_localization.launch").read_text(
        encoding="utf-8"
    )
    assert '<arg name="auto_init" default="false"/>' in launch
    assert 'type="auto_initial_pose.py"' in launch
    assert 'if="$(arg auto_init)"' in launch


def test_known_start_route_is_attempted_before_global_map_search():
    text = script_text()
    candidate_policy = (
        ROOT / "scripts" / "initial_pose_candidates.py"
    ).read_text(encoding="utf-8")

    assert "load_known_start" in text
    assert "known_start_route" in candidate_policy
    assert text.index("try_candidate(route_prior") < text.index(
        "map_points = load_pcd_xyz"
    )


def test_launch_passes_known_start_route_and_body_profile():
    launch = (ROOT / "launch" / "moving_localization.launch").read_text(
        encoding="utf-8"
    )

    assert '<param name="route" value="$(arg auto_init_route)"/>' in launch
    assert (
        '<param name="body_frame_profile" '
        'value="$(arg auto_init_body_frame_profile)"/>' in launch
    )
    assert "--route $(arg auto_init_route)" not in launch


def test_launch_applies_map_override_to_every_map_consumer():
    launch = (ROOT / "launch" / "moving_localization.launch").read_text(
        encoding="utf-8"
    )
    map_preview = launch.split('name="map_preview_publisher"', 1)[1].split(
        "</node>", 1
    )[0]

    assert '<param name="map_path" value="$(arg map_path)"/>' in map_preview
    assert '<param name="map_sha256" value="$(arg map_sha256)"/>' in map_preview
