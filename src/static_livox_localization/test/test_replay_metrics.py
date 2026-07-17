import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts" / "replay_metrics.py"
SPEC = importlib.util.spec_from_file_location("replay_metrics", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid_summary():
    return {
        "pose_samples": 500,
        "max_position_step_m": 0.12,
        "max_yaw_step_deg": 2.0,
        "return_translation_m": 0.30,
        "return_yaw_deg": 7.0,
        "state_duration_s": {"TRACKING": 50.0, "DEGRADED": 2.0},
    }


def test_accepts_repeatable_moving_trial_within_limits():
    accepted, reasons = MODULE.evaluate_summary(valid_summary())

    assert accepted
    assert reasons == []


def test_rejects_pose_jump_and_large_return_error():
    summary = valid_summary()
    summary["max_position_step_m"] = 0.8
    summary["return_translation_m"] = 0.7
    summary["return_yaw_deg"] = 15.0

    accepted, reasons = MODULE.evaluate_summary(summary)

    assert not accepted
    assert "POSE_JUMP" in reasons
    assert "RETURN_TRANSLATION" in reasons
    assert "RETURN_YAW" in reasons


def test_replay_launch_is_localization_only_and_requires_explicit_bag():
    path = ROOT / "test" / "moving_localization_replay.test"
    text = path.read_text(encoding="utf-8")
    tree = ET.parse(path)

    bag_arg = tree.find(".//arg[@name='bag']")
    assert bag_arg is not None
    assert bag_arg.attrib.get("default") == ""
    assert "moving_localization.launch" in text
    assert "rosbag" in text
    assert "--clock" in text
    assert "visualization" in text
    assert "/cmd_vel" not in text
    assert "move_base" not in text
