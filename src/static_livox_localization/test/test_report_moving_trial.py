import importlib.util
import math
from pathlib import Path


PATH = Path(__file__).parents[1] / "scripts" / "report_moving_trial.py"
SPEC = importlib.util.spec_from_file_location("report_moving_trial", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def pose(stamp, x, y, yaw):
    return {
        "stamp": stamp,
        "x": x,
        "y": y,
        "z": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": math.sin(yaw / 2.0),
        "qw": math.cos(yaw / 2.0),
    }


def state(stamp, name):
    return {"stamp": stamp, "state": name}


def test_summarizes_initialization_state_durations_jumps_and_return_error():
    poses = [
        pose(2.0, 0.0, 0.0, 0.0),
        pose(3.0, 1.0, 0.0, 0.10),
        pose(4.0, 0.1, 0.0, 0.05),
    ]
    states = [
        state(0.0, "WAITING_INITIALIZATION"),
        state(2.0, "TRACKING"),
        state(3.0, "DEGRADED"),
        state(4.0, "TRACKING"),
    ]

    summary = MODULE.summarize(poses, states)

    assert summary["initialization_time_s"] == 2.0
    assert summary["state_duration_s"]["WAITING_INITIALIZATION"] == 2.0
    assert summary["state_duration_s"]["TRACKING"] == 1.0
    assert summary["state_duration_s"]["DEGRADED"] == 1.0
    assert summary["max_position_step_m"] == 1.0
    assert abs(summary["max_yaw_step_deg"] - math.degrees(0.10)) < 1e-9
    assert abs(summary["return_translation_m"] - 0.1) < 1e-9
    assert abs(summary["return_yaw_deg"] - math.degrees(0.05)) < 1e-9


def test_empty_pose_log_is_reported_without_fabricating_success():
    summary = MODULE.summarize([], [state(0.0, "WAITING_INITIALIZATION")])

    assert summary["pose_samples"] == 0
    assert summary["initialization_time_s"] is None
    assert summary["return_translation_m"] is None
    assert summary["return_yaw_deg"] is None

