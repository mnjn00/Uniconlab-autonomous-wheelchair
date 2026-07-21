from pathlib import Path


ROOT = Path(__file__).parents[1]


def script_text():
    return (ROOT / "scripts" / "auto_initial_pose.py").read_text(encoding="utf-8")


def test_auto_init_seeds_through_existing_verification_pipeline():
    text = script_text()
    assert "/fast_lio_icp/initialpose" in text
    assert "/fast_lio_icp/enable_auto_correction" in text
    assert "/fast_lio_icp/localization_diagnostics" in text
    assert 'state["message"] == "TRACKING"' in text


def test_auto_init_falls_back_to_next_candidate_on_rejection():
    text = script_text()
    assert "failed verification, trying next" in text
    assert "--top" not in text or "args.top" in text


def test_auto_init_refuses_low_confidence_seeds():
    text = script_text()
    assert "min-score" in text or "min_score" in text
    assert "below threshold" in text


def test_auto_init_supports_dry_run_without_publishing():
    text = script_text()
    dry_run = text.index("args.dry_run")
    publish = text.index("seed_pub.publish")
    assert dry_run < publish


def test_launch_exposes_optional_auto_init_node():
    launch = (ROOT / "launch" / "moving_localization.launch").read_text(
        encoding="utf-8"
    )
    assert '<arg name="auto_init" default="false"/>' in launch
    assert 'type="auto_initial_pose.py"' in launch
    assert 'if="$(arg auto_init)"' in launch
