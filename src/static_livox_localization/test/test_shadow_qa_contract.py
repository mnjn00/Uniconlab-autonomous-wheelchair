"""Contract tests for the non-driving NUC shadow-QA evidence."""

import sys
import importlib.util
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from shadow_qa_contract import validate_snapshot
finally:
    sys.path.pop(0)


def diagnostics():
    return {
        "tracking_state": "TRACKING",
        "raw_scan_points": "1000",
        "dynamic_returns_dropped": "80",
        "post_box_points": "920",
        "map_filtered": "20",
        "post_map_points": "900",
        "rolling_submap_points": "4200",
    }


def test_wall_only_scene_is_clear_and_topics_agree():
    evidence = validate_snapshot(
        {"status": "OK", "objects": []},
        [],
        diagnostics(),
    )

    assert evidence["object_count"] == 0
    assert evidence["dynamic_box_count"] == 0
    assert evidence["tracking_state"] == "TRACKING"


def test_summary_and_box_count_must_agree():
    with pytest.raises(ValueError, match="object/box count mismatch"):
        validate_snapshot(
            {"status": "OK", "objects": [{"id": 1}]},
            [],
            diagnostics(),
        )


def test_invalid_box_and_missing_filter_diagnostics_fail_closed():
    with pytest.raises(ValueError, match="positive finite"):
        validate_snapshot(
            {"status": "OK", "objects": [{"id": 1}]},
            [{"size": [1.0, 0.0, 2.0]}],
            diagnostics(),
        )

    incomplete = diagnostics()
    incomplete.pop("post_map_points")
    with pytest.raises(ValueError, match="missing diagnostics"):
        validate_snapshot(
            {"status": "OK", "objects": []},
            [],
            incomplete,
        )


def load_validator():
    path = Path(__file__).parents[3] / "tools" / \
        "validate_nuc_shadow_snapshot.py"
    spec = importlib.util.spec_from_file_location("shadow_validator", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_ros_yaml_parsers_preserve_topic_contracts():
    validator = load_validator()
    summary = validator.parse_summary({
        "data": '{"status":"OK","objects":[{"id":7}]}'
    })
    boxes = validator.parse_boxes({
        "markers": [
            {"action": 3},
            {
                "action": 0,
                "header": {"frame_id": "body"},
                "scale": {"x": 0.5, "y": 0.6, "z": 1.7},
            },
        ]
    })
    parsed_diagnostics = validator.parse_diagnostics({
        "status": [{
            "name": "moving localization",
            "values": [
                {"key": key, "value": value}
                for key, value in diagnostics().items()
            ],
        }]
    })

    evidence = validate_snapshot(summary, boxes, parsed_diagnostics)
    assert evidence["object_count"] == 1


def test_shadow_runner_builds_fully_and_never_launches_motion_nodes():
    runner = (
        Path(__file__).parents[3] / "tools" / "run_nuc_shadow_qa.sh"
    ).read_text(encoding="utf-8")

    assert 'catkin_make > "$OUT/catkin-build.txt"' in runner
    assert "rosrun static_livox_localization obstacle_clusters.py" in runner
    assert "rosrun static_livox_localization waypoint_follower.py" not in runner
    assert "rosrun static_livox_localization dwa_follower.py" not in runner
    assert "rosrun static_livox_localization mpc_follower.py" not in runner
    assert "rosrun static_livox_localization safety_gate.py" not in runner
