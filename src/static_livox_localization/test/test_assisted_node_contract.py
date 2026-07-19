from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def node_text():
    return (ROOT / "src" / "moving_icp_localizer.cpp").read_text(encoding="utf-8")


def test_node_has_manual_alignment_and_explicit_auto_correction_service():
    text = node_text()
    assert "AssistedAlignmentController" in text
    assert 'advertiseService("/fast_lio_icp/enable_auto_correction"' in text
    assert "std_srvs::SetBool" in text
    assert 'key_value("auto_correction_enabled"' in text
    assert 'key_value("consistent_candidate_count"' in text


def test_seed_is_immediately_visible_without_running_icp():
    text = node_text()
    assert "map_T_odom_ = seed_map_T_odom_guess_" in text
    assert "alignment_controller_.on_seed()" in text
    assert "publish_pose_tf_path_locked(latest_odom_)" in text


def test_assisted_tracking_uses_tight_roi_consensus_and_small_corrections():
    config = yaml.safe_load(
        (ROOT / "config" / "moving_localization.yaml").read_text(encoding="utf-8")
    )
    assert config["roi_radius"] == 8.0
    assert config["required_consistent_candidates"] == 3
    assert config["candidate_translation_tolerance_m"] == 0.30
    assert config["candidate_yaw_tolerance_deg"] == 3.0
    assert config["max_correction_translation_m"] == 0.20
    assert config["max_correction_yaw_deg"] == 2.0
    assert config["auto_correction_on_start"] is False


def test_verification_snaps_full_candidate_only_after_consensus():
    text = node_text()
    observe = text.index("observe_candidate(candidate_map_T_odom)")
    ready = text.index("consensus.ready", observe)
    snap = text.index("map_T_odom_ = candidate_map_T_odom;", ready)
    assert observe < ready < snap


def test_tracking_applies_limited_step_on_every_accepted_candidate():
    text = node_text()
    accepted = text.index("if (decision.accepted)")
    tracking_branch = text.index("} else {", accepted)
    apply = text.index("limit_map_T_odom_step", tracking_branch)
    observe_true = text.index("state_machine_.observe(true", apply)
    assert accepted < tracking_branch < apply < observe_true


def test_lost_tracking_triggers_reacquisition():
    text = node_text()
    assert "begin_reacquisition" in text
    lost = text.index("TrackingState::LOST)", text.index("observe(false"))
    reacquire = text.index("begin_reacquisition", lost)
    assert lost < reacquire


def test_std_srvs_is_declared_as_build_and_runtime_dependency():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (ROOT / "package.xml").read_text(encoding="utf-8")
    assert "std_srvs" in cmake
    assert "<depend>std_srvs</depend>" in package
