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


def box(object_id=1, stamp=100.0):
    return {
        "id": object_id,
        "size": [0.5, 0.6, 1.7],
        "position": [1.0, 0.0, 0.5],
        "stamp": stamp,
    }


def test_wall_only_scene_is_clear_and_topics_agree():
    evidence = validate_snapshot(
        {"status": "OK", "stamp": 100.0, "objects": []},
        [],
        diagnostics(),
    )

    assert evidence["object_count"] == 0
    assert evidence["dynamic_box_count"] == 0
    assert evidence["tracking_state"] == "TRACKING"


def test_summary_and_box_count_must_agree():
    with pytest.raises(ValueError, match="object/box count mismatch"):
        validate_snapshot(
            {"status": "OK", "stamp": 100.0, "objects": [{"id": 1}]},
            [],
            diagnostics(),
        )


def test_invalid_box_and_missing_filter_diagnostics_fail_closed():
    with pytest.raises(ValueError, match="positive finite"):
        validate_snapshot(
            {"status": "OK", "stamp": 100.0, "objects": [{"id": 1}]},
            [{**box(), "size": [1.0, 0.0, 2.0]}],
            diagnostics(),
        )

    incomplete = diagnostics()
    incomplete.pop("post_map_points")
    with pytest.raises(ValueError, match="missing diagnostics"):
        validate_snapshot(
            {"status": "OK", "stamp": 100.0, "objects": []},
            [],
            incomplete,
        )


def test_filter_accounting_must_be_exact_and_nonempty():
    summary = {"status": "OK", "stamp": 100.0, "objects": []}
    broken = diagnostics()
    broken["dynamic_returns_dropped"] = "1"
    with pytest.raises(ValueError, match="box-filter accounting"):
        validate_snapshot(summary, [], broken)
    broken = diagnostics()
    broken["map_filtered"] = "1"
    with pytest.raises(ValueError, match="map-filter accounting"):
        validate_snapshot(summary, [], broken)
    broken = diagnostics()
    broken["raw_scan_points"] = "0"
    broken["dynamic_returns_dropped"] = "0"
    broken["post_box_points"] = "0"
    broken["map_filtered"] = "0"
    broken["post_map_points"] = "0"
    broken["rolling_submap_points"] = "0"
    with pytest.raises(ValueError, match="empty"):
        validate_snapshot(summary, [], broken)


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
        "data": '{"status":"OK","stamp":100.0,"objects":[{"id":7}]}'
    })
    boxes = validator.parse_boxes({
        "markers": [
            {"action": 3},
            {
                "action": 0,
                "id": 7,
                "header": {
                    "frame_id": "body",
                    "stamp": {"secs": 100, "nsecs": 0},
                },
                "pose": {
                    "position": {"x": 1.0, "y": 0.0, "z": 0.5}
                },
                "scale": {"x": 0.5, "y": 0.6, "z": 1.7},
            },
        ]
    })
    parsed_diagnostics = validator.parse_diagnostics({
        "status": [{
            "name": "moving localization",
            "values": [
                {"key": ("raw_state" if key == "tracking_state" else key),
                 "value": value}
                for key, value in diagnostics().items()
            ],
        }],
        "header": {"stamp": {"secs": 100, "nsecs": 0}},
    })

    stamp = parsed_diagnostics.pop("_stamp")
    evidence = validate_snapshot(
        summary, boxes, parsed_diagnostics, diagnostics_stamp=stamp)
    assert evidence["object_count"] == 1


def test_lost_or_incoherent_snapshot_never_passes():
    lost = diagnostics()
    lost["tracking_state"] = "LOST"
    with pytest.raises(ValueError, match="not TRACKING"):
        validate_snapshot(
            {"status": "OK", "stamp": 100.0, "objects": []},
            [],
            lost,
        )
    with pytest.raises(ValueError, match="stamps do not agree"):
        validate_snapshot(
            {"status": "OK", "stamp": 100.0, "objects": [{"id": 1}]},
            [box(stamp=110.0)],
            diagnostics(),
        )


def test_summary_and_boxes_must_share_one_source_cycle():
    with pytest.raises(ValueError, match="source cycle"):
        validate_snapshot(
            {"status": "OK", "stamp": 100.0, "objects": [{"id": 1}]},
            [box(stamp=100.0)],
            diagnostics(),
            boxes_source_stamp=100.2,
        )


def test_capture_tool_subscribes_before_selecting_a_shared_cycle():
    capture = (
        Path(__file__).parents[3] / "tools" / "capture_shadow_snapshot.py"
    ).read_text(encoding="utf-8")
    assert 'payload["stamp"]' in capture
    assert "shared = summaries.keys() & boxes.keys()" in capture
    assert "message_yaml(" in capture


def test_shadow_runner_builds_fully_and_never_launches_motion_nodes():
    runner = (
        Path(__file__).parents[3] / "tools" / "run_nuc_shadow_qa.sh"
    ).read_text(encoding="utf-8")

    assert 'if [ -d "$WS/.catkin_tools" ]; then' in runner
    assert "catkin build static_livox_localization" in runner
    assert "catkin run_tests static_livox_localization" in runner
    assert 'catkin_make > "$OUT/catkin-build.txt"' in runner
    assert "rosrun static_livox_localization obstacle_clusters.py" in runner
    assert "rosrun static_livox_localization waypoint_follower.py" not in runner
    assert "rosrun static_livox_localization dwa_follower.py" not in runner
    assert "rosrun static_livox_localization mpc_follower.py" not in runner
    assert "rosrun static_livox_localization safety_gate.py" not in runner
    assert 'SHADOW_QA=1 LOCALIZATION_WS="$WS"' in runner
    assert '"$REPO/tools/start_wheelchair_localization.sh"' in runner
    assert 'REPO="${REPO:-$HOME/wheelchair_localization_src}"' in runner
    assert 'MAP_SHA256="${MAP_SHA256:-ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431}"' \
        in runner
    assert 'ROUTE="${ROUTE:-$REPO/routes/20260812_route_v6_v8_waypoints.json}"' \
        in runner
    assert 'DRIVABLE_MASK="${DRIVABLE_MASK:-$REPO/routes/route_2d_map_v8.yaml}"' \
        in runner
    startup = runner[runner.index("STARTED_STACK=1"):runner.index(
        "if ! rostopic list | grep -qx '/cloud_registered_body';")]
    for variable in (
        "LOCALIZATION_WS", "MAP", "MAP_SHA256", "MAP_ID", "TRAJ",
        "ROUTE", "BAND", "DRIVABLE_MASK",
    ):
        assert f'{variable}="${variable if variable != "LOCALIZATION_WS" else "WS"}"' \
            in startup
    assert "cleanup || status=90" in runner
    assert "[r]oslaunch base_model vectornav" in runner
    assert "matching_pids" in runner
    assert 'wait "$pid"' in runner
    loop = runner[runner.index("for attempt in"):runner.index(
        'cp "$OUT/nuc-shadow-qa.txt"')]
    assert "capture_shadow_snapshot.py" in loop
    assert "rostopic echo -n 1" not in loop


def test_motion_graph_detection_checks_nodes_and_command_topics():
    from shadow_qa_contract import motion_surface_violations

    assert motion_surface_violations(
        {"/cmd_vel": ["/rogue"]}, {}
    ) == ["publisher /cmd_vel: /rogue"]
    violations = motion_surface_violations(
        {}, {"/cloud_registered_body": ["/wheel_driver"]}
    )
    assert violations == ["subscriber node: /wheel_driver"]


def test_graph_checker_is_atomic_and_fails_closed():
    checker = (
        Path(__file__).parents[3] / "tools" / "check_shadow_ros_graph.py"
    ).read_text(encoding="utf-8")
    assert "getSystemState" in checker
    assert "rostopic" not in checker
    assert '"status": "ERROR"' in checker
    assert 'parser.add_argument("--baseline")' in checker
    assert '"unexpected node: %s"' in checker
    runner = (
        Path(__file__).parents[3] / "tools" / "run_nuc_shadow_qa.sh"
    ).read_text(encoding="utf-8")
    assert "ros-graph-monitor.jsonl" in runner
    assert "ros-graph-final.json" in runner
    assert 'touch "$GRAPH_UNSAFE"' in runner


def test_cleanup_restores_process_and_ros_graph_baselines():
    runner = (
        Path(__file__).parents[3] / "tools" / "run_nuc_shadow_qa.sh"
    ).read_text(encoding="utf-8")
    for token in (
        "[r]oslaunch static_livox_localization moving_localization",
        "[b]ounded_cloud_preview",
        "[r]eference_marker",
        "[l]ocalization_state_marker",
    ):
        assert token in runner
    assert 'matching_pids > "$OUT/process-baseline.txt"' in runner
    assert 'kill -TERM -- "-$pgid"' in runner
    assert 'process-baseline-diff.txt' in runner
    assert '--baseline "$OUT/ros-graph-baseline.json"' in runner


def test_field_startup_has_sensor_only_shadow_exit_before_wheel_launch():
    startup = (
        Path(__file__).parents[3]
        / "tools"
        / "start_wheelchair_localization.sh"
    ).read_text(encoding="utf-8")

    shadow_exit = startup.index('if [ "$SHADOW_QA" = "1" ]')
    wheel_launch = startup.index("roslaunch base_model wheel.launch")
    assert shadow_exit < wheel_launch
    shadow_block = startup[shadow_exit:wheel_launch]
    assert "start_object_tracking" in shadow_block
    assert "SHADOW_QA_READY" in shadow_block
