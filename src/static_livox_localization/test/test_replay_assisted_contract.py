from pathlib import Path


ROOT = Path(__file__).parents[1]
PROJECT = ROOT.parents[1]


def test_replay_enables_auto_correction_only_after_replayed_seed():
    launch = (ROOT / "test" / "moving_localization_replay.test").read_text(
        encoding="utf-8"
    )
    helper = (
        ROOT / "scripts" / "enable_auto_correction_after_seed.py"
    ).read_text(encoding="utf-8")

    assert "enable_auto_correction_after_seed.py" in launch
    assert "--delay=2" in launch
    assert '"/fast_lio_icp/initialpose"' in helper
    assert '"/fast_lio_icp/enable_auto_correction"' in helper
    assert "SetBool" in helper


def test_trial_recording_keeps_seed_and_alignment_state_evidence():
    recorder = (
        PROJECT / "runtime" / "record_moving_localization_trial.sh"
    ).read_text(encoding="utf-8")
    assert "/fast_lio_icp/initialpose" in recorder
    assert "/fast_lio_icp/localization_diagnostics" in recorder


def test_replay_metrics_requires_assisted_alignment_state_sequence():
    metrics = (ROOT / "scripts" / "replay_metrics.py").read_text(
        encoding="utf-8"
    )
    assert '"MANUAL_ALIGN"' in metrics
    assert '"VERIFYING"' in metrics
    assert '"TRACKING"' in metrics
    assert "MISSING_ALIGNMENT_STATE" in metrics
