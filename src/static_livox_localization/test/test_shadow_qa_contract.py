"""Contract tests for the non-driving NUC shadow-QA evidence."""

import importlib.util
import sys
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
    assert "shared = summaries.keys() & boxes.keys() & diagnostics.keys()" \
        in capture
    localizer = (
        Path(__file__).parents[1] / "src" / "moving_icp_localizer.cpp"
    ).read_text(encoding="utf-8")
    assert "source_boxes_stamp = dynamic_boxes_stamp_" in localizer
    assert "source_boxes_stamp.isZero() ? stamp : source_boxes_stamp" \
        in localizer
    assert "cloud_callback_mutex_" in localizer
    assert (
        "std::lock_guard<std::mutex> cloud_lock(cloud_callback_mutex_)"
        in localizer
    )
    commit = localizer.index("diagnostic_source_stamp_ =")
    publish = localizer.index(
        'publish_diagnostic_locked("CLOUD_ODOMETRY_TIME_MISMATCH"'
    )
    for counter in (
        "last_dynamic_dropped_ = dropped",
        "last_map_filtered_ = map_dropped",
        "last_raw_points_ = raw_points",
        "last_post_box_points_ = post_box_points",
        "last_post_map_points_ = post_map_points",
    ):
        assert commit < localizer.index(counter) < publish
    assert "message_yaml(" in capture


def test_shadow_runner_builds_fully_and_never_launches_motion_nodes():
    runner = (
        Path(__file__).parents[3] / "tools" / "run_nuc_shadow_qa.sh"
    ).read_text(encoding="utf-8")

    assert 'if [ -d "$WS/.catkin_tools" ]; then' in runner
    assert "catkin build static_livox_localization" in runner
    assert "catkin run_tests static_livox_localization" in runner
    assert 'catkin_make > "$OUT/catkin-build.txt"' in runner
    startup = (
        Path(__file__).parents[3]
        / "tools"
        / "start_wheelchair_localization.sh"
    ).read_text(encoding="utf-8")
    assert "rosrun static_livox_localization obstacle_clusters.py" in startup
    assert "rosrun static_livox_localization waypoint_follower.py" not in runner
    assert "rosrun static_livox_localization dwa_follower.py" not in runner
    assert "rosrun static_livox_localization mpc_follower.py" not in runner
    assert "rosrun static_livox_localization safety_gate.py" not in runner
    assert 'SHADOW_QA=1 LOCALIZATION_WS="$WS"' in runner
    assert '"$REPO/tools/start_wheelchair_localization.sh"' in runner
    assert 'REPO="${REPO:-$HOME/wheelchair_localization_src}"' in runner
    assert (
        'MAP_SHA256="${MAP_SHA256:-'
        'ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431}"'
        in runner
    )
    assert (
        'ROUTE="${ROUTE:-'
        '$REPO/routes/20260814_route_algorithm_waypoints.json}"'
        in runner
    )
    assert (
        'DRIVABLE_MASK="${DRIVABLE_MASK:-'
        '$REPO/routes/route_2d_map_algorithm.yaml}"'
        in runner
    )
    startup = runner[runner.index("setsid env SHADOW_QA=1"):runner.index(
        "wait \"$STACK_PID\"")]
    for variable in (
        "LOCALIZATION_WS", "MAP", "MAP_SHA256", "MAP_ID", "TRAJ",
        "ROUTE", "BAND", "DRIVABLE_MASK",
    ):
        assert f'{variable}="${variable if variable != "LOCALIZATION_WS" else "WS"}"' \
            in startup
    assert "if cleanup; then" in runner
    assert "status=90" in runner
    assert "[r]oslaunch base_model vectornav" in runner
    assert "matching_pids" in runner
    assert 'wait "$STACK_PID"' in runner
    loop_start = runner.index("for attempt in")
    loop = runner[loop_start:runner.index(
        'if [ -e "$GRAPH_UNSAFE" ] ||', loop_start)]
    assert "capture_shadow_snapshot.py" in loop
    assert "rostopic echo -n 1" not in loop


def test_motion_graph_detection_checks_nodes_and_command_topics():
    from shadow_qa_contract import (
        baseline_graph_violations,
        motion_surface_violations,
    )

    assert motion_surface_violations(
        {"/cmd_vel": ["/rogue"]}, {}
    ) == ["publisher /cmd_vel: /rogue"]
    violations = motion_surface_violations(
        {}, {"/cloud_registered_body": ["/wheel_driver"]}
    )
    assert violations == ["subscriber node: /wheel_driver"]
    graph = {
        "publishers": {"/clock": ["/shadow"]},
        "subscribers": {},
        "services": {"/shadow/get_loggers": ["/shadow"]},
    }
    assert baseline_graph_violations(
        graph,
        {"publishers": {}, "subscribers": {}, "services": {}},
    ) == [
        "unexpected publisher edge: /clock <- /shadow",
        "unexpected service edge: /shadow/get_loggers <- /shadow",
    ]
    assert baseline_graph_violations(
        {"publishers": {}, "subscribers": {}, "services": {}},
        graph,
    ) == [
        "missing publisher edge: /clock <- /shadow",
        "missing service edge: /shadow/get_loggers <- /shadow",
    ]


def test_graph_checker_is_atomic_and_fails_closed():
    checker = (
        Path(__file__).parents[3] / "tools" / "check_shadow_ros_graph.py"
    ).read_text(encoding="utf-8")
    assert "getSystemState" in checker
    assert "rostopic" not in checker
    assert '"status": "ERROR"' in checker
    assert 'parser.add_argument("--baseline")' in checker
    assert "baseline_graph_violations(graph, baseline)" in checker
    runner = (
        Path(__file__).parents[3] / "tools" / "run_nuc_shadow_qa.sh"
    ).read_text(encoding="utf-8")
    assert "ros-graph-monitor.jsonl" in runner
    assert "ros-graph-final.json" in runner
    assert 'touch "$3"' in runner


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
    assert 'OWN_ROSCORE_PID=""' in runner
    assert 'LAUNCHED_PIDS+=("$OWN_ROSCORE_PID")' not in runner
    assert 'trap finish EXIT HUP INT TERM' in runner
    assert 'ROS_MASTER_URI="http://127.0.0.1:$ROS_MASTER_PORT"' in runner
    assert 'setsid env SHADOW_QA=1' in runner
    assert 'OWNED_PGIDS+=("$STACK_PID")' in runner
    assert 'rm -f "$OUT/integration-green.txt"' in runner
    finish = runner[runner.index("finish() {"):runner.index(
        "trap finish EXIT HUP INT TERM")]
    assert 'cp "$OUT/nuc-shadow-qa.txt" "$OUT/integration-green.txt"' in finish


def test_shadow_refuses_old_stack_and_monitors_before_startup():
    runner = (
        Path(__file__).parents[3] / "tools" / "run_nuc_shadow_qa.sh"
    ).read_text(encoding="utf-8")
    assert "field or autonomous stack present" in runner
    monitor = runner.index("ros-graph-monitor.jsonl")
    startup = runner.index('SHADOW_QA=1 LOCALIZATION_WS="$WS"')
    assert monitor < startup
    startup_text = (
        Path(__file__).parents[3]
        / "tools"
        / "start_wheelchair_localization.sh"
    ).read_text(encoding="utf-8")
    assert 'BODY_FRAME_PROFILE="builtin"' in startup_text
    assert "_body_frame_profile:=vn100" not in runner


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
    cleanup = startup[startup.index('echo "[1/5] cleaning old processes"'):]
    assert 'if [ "$SHADOW_QA" != "1" ]; then' in cleanup


def human_aware_replay():
    return {
        "statuses": [
            {"stamp": 109.8, "decision": "OBSERVING", "track_id": 1641},
            {
                "stamp": 110.0,
                "decision": "BYPASS_COMMITTED",
                "track_id": 1641,
            },
            {
                "stamp": 110.2,
                "decision": "BYPASS_COMMITTED",
                "track_id": 1641,
            },
            {
                "stamp": 110.4,
                "decision": "BYPASS_COMMITTED",
                "track_id": 1641,
            },
            {"stamp": 111.2, "decision": "STOP_REQUIRED", "track_id": None},
        ],
        "summaries": [
            {"stamp": 110.0, "people": [{"id": 1641, "motion": "static"}]},
            {"stamp": 110.2, "people": [{"id": 1641, "motion": "moving"}]},
            {"stamp": 110.4, "people": [{"id": 1641, "motion": "static"}]},
            {"stamp": 111.2, "people": [{"id": 1641, "motion": "moving"}]},
        ],
        "tracked_agents": [
            {"stamp": 110.0, "frame_id": "map", "track_ids": [1641]},
            {"stamp": 110.2, "frame_id": "map", "track_ids": [1641]},
            {"stamp": 110.4, "frame_id": "map", "track_ids": [1641]},
        ],
        "velocity_proposals": [
            {"stamp": 110.2, "linear_x": 0.2, "angular_z": 0.1},
        ],
        "local_plans": [
            {"stamp": 110.2, "validation": "ACCEPTED", "point_count": 12},
        ],
        "agent_plans": [
            {
                "stamp": 110.2,
                "paths": [{"track_id": 1641, "point_count": 10}],
            },
        ],
    }


def test_human_aware_replay_proves_one_stable_safe_commitment():
    import shadow_qa_contract

    assert hasattr(shadow_qa_contract, "validate_human_aware_replay"), \
        "shadow QA cannot validate CoHAN/HATEB replay evidence"

    evidence = shadow_qa_contract.validate_human_aware_replay(
        human_aware_replay())

    assert evidence == {
        "accepted_plan_count": 1,
        "agent_plan_count": 1,
        "committed_sample_count": 3,
        "proposal_count": 1,
        "stable_track_id": 1641,
        "stop_go_reentries": 0,
        "unsafe_motion_stop_count": 1,
    }


def test_human_aware_replay_allows_optional_agent_plan_evidence():
    import shadow_qa_contract

    replay = human_aware_replay()
    replay["agent_plans"] = []

    evidence = shadow_qa_contract.validate_human_aware_replay(replay)

    assert evidence["agent_plan_count"] == 0


def test_human_aware_replay_rejects_stop_go_reentry():
    import shadow_qa_contract

    assert hasattr(shadow_qa_contract, "validate_human_aware_replay"), \
        "shadow QA cannot reject STOP-GO replay"
    replay = human_aware_replay()
    replay["statuses"].insert(
        3,
        {"stamp": 110.3, "decision": "STOP_REQUIRED", "track_id": None},
    )

    with pytest.raises(ValueError, match="re-entered"):
        shadow_qa_contract.validate_human_aware_replay(replay)


def test_human_aware_replay_allows_recommit_after_full_new_evidence():
    import shadow_qa_contract

    replay = human_aware_replay()
    replay["statuses"].extend([
        {
            "stamp": 120.0,
            "decision": "OBSERVING",
            "track_id": 1642,
            "evidence_s": 0.0,
        },
        {
            "stamp": 130.0,
            "decision": "BYPASS_COMMITTED",
            "track_id": 1642,
            "evidence_s": 10.0,
        },
        {
            "stamp": 130.2,
            "decision": "BYPASS_COMMITTED",
            "track_id": 1642,
            "evidence_s": 10.2,
        },
    ])
    replay["tracked_agents"].extend([
        {"stamp": 130.0, "frame_id": "map", "track_ids": [1642]},
        {"stamp": 130.2, "frame_id": "map", "track_ids": [1642]},
    ])
    replay["velocity_proposals"].extend([
        {"stamp": 111.2, "linear_x": 0.1, "angular_z": 0.0},
        {"stamp": 130.2, "linear_x": 0.2, "angular_z": 0.0},
    ])
    replay["local_plans"].append({
        "stamp": 130.2,
        "validation": "ACCEPTED",
        "point_count": 8,
    })
    replay["agent_plans"].append({
        "stamp": 130.2,
        "paths": [{"track_id": 1642, "point_count": 10}],
    })

    evidence = shadow_qa_contract.validate_human_aware_replay(replay)

    assert evidence["stop_go_reentries"] == 0
    assert evidence["proposal_count"] == 2
    assert evidence["accepted_plan_count"] == 2
    assert evidence["agent_plan_count"] == 2


def test_cohan_replay_runner_isolated_and_cleans_every_process():
    runner = (
        Path(__file__).parents[3] / "tools" / "run_cohan_shadow_replay.sh"
    )
    assert runner.exists(), "no isolated CoHAN bag-replay runner exists"
    text = runner.read_text(encoding="utf-8")

    assert "blackbox_20260826_220341.bag" in text
    assert "cohan_shadow.launch" in text
    assert "check_shadow_ros_graph.py" in text
    assert "validate_cohan_shadow_replay.py" in text
    assert "ROS_MASTER_PORT" in text
    assert "trap finish EXIT HUP INT TERM" in text
    assert "/human_aware_shadow/velocity_proposal" in text
    watcher = text.index("wait_for_cohan_shadow_commit.py")
    replay = text.index('setsid rosbag play "$BAG"')
    goal = text.index("geometry_msgs/PoseStamped")
    assert watcher < replay < goal
    assert 'wait "$COMMIT_PID"' in text[replay:goal]
    for forbidden in (
        "/cmd_vel_raw",
        "/cmd_vel_gated",
        "/cmd_vel ",
        "/wheel_cmd",
        "/mode_cmd",
    ):
        assert forbidden not in text


def test_cohan_replay_capture_uses_advisory_topics_and_done_signal():
    root = Path(__file__).parents[3]
    capture_path = root / "tools" / "capture_cohan_shadow_replay.py"
    validator_path = root / "tools" / "validate_cohan_shadow_replay.py"
    assert capture_path.exists(), "no coherent CoHAN replay capture exists"
    assert validator_path.exists(), "no offline CoHAN replay validator exists"

    capture = capture_path.read_text(encoding="utf-8")
    for topic in (
        '"/perception/objects_summary"',
        '"/human_aware_shadow/status"',
        '"/human_aware_shadow/tracked_agents"',
        '"/human_aware_shadow/velocity_proposal"',
        '"/human_aware_shadow/move_base/HATebLocalPlannerROS/local_plan"',
        '"/human_aware_shadow/move_base/HATebLocalPlannerROS/agents_local_plans"',
        '"/human_aware_shadow/replay_done"',
    ):
        assert topic in capture
    assert "ShadowTrajectoryValidator" in capture
    assert "rospy.on_shutdown" in capture
    validator = validator_path.read_text(encoding="utf-8")
    assert "validate_human_aware_replay" in validator
