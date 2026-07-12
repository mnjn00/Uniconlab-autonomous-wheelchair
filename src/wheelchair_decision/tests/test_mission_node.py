import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
for path in (SCRIPTS, Path(__file__).parents[2] / "wheelchair_navigation" / "scripts"):
    sys.path.insert(0, str(path))

spec = importlib.util.spec_from_file_location("mission_node", str(SCRIPTS / "mission_node.py"))
node = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = node
spec.loader.exec_module(node)

from mission_core import EventType, MissionConfig, MissionEvent, MissionFSM


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class Action:
    def __init__(self):
        self.cancels = 0

    def cancel_goal(self):
        self.cancels += 1


class Sink:
    def __init__(self):
        self.samples = []

    def __call__(self, output, stamp):
        self.samples.append((stamp, output))


class Manifest:
    map_id = "campus"
    map_sha256 = "a" * 64
    safety_manifest_sha256 = "b" * 64

    def __init__(self):
        self.routes = {
            "outbound": SimpleNamespace(route_id="route-out", route_manifest_sha256="c" * 64,
                                         waypoints=(object(), object())),
            "return": SimpleNamespace(route_id="route-back", route_manifest_sha256="d" * 64,
                                       waypoints=(object(), object())),
        }

    def route(self, direction):
        return self.routes[direction]


def goal(**changes):
    values = dict(mission_id="mission-1", route_id="route-out", direction=1,
                  map_id="campus", map_sha256="a" * 64,
                  route_manifest_sha256="c" * 64,
                  safety_manifest_sha256="b" * 64)
    values.update(changes)
    return SimpleNamespace(**values)


def harness(config=None):
    clock, action, sink = Clock(), Action(), Sink()
    fsm = MissionFSM(config or MissionConfig(), clock)
    runtime = node.MissionRuntime(fsm, EventType, MissionEvent, action, clock, sink)
    runtime.operator_arm()
    runtime.begin(node.bind_route(Manifest(), goal()))
    return clock, action, sink, runtime


def evidence(runtime, stamp=None):
    runtime.dispatch("LOCALIZATION", True, stamp=stamp)
    runtime.dispatch("GEOFENCE", True, stamp=stamp)
    runtime.dispatch("COLLISION", "clear", stamp=stamp)
    runtime.dispatch("SLOPE", "safe", stamp=stamp)
    runtime.dispatch("MODE", True, stamp=stamp)
    return runtime.dispatch("DRIVER", True, stamp=stamp)


def navigating(config=None):
    clock, action, sink, runtime = harness(config)
    ready = evidence(runtime)
    assert ready.send_waypoint_index == 0
    moving = runtime.dispatch(
        "MOVE_BASE_ACTIVE", {"generation": ready.goal_generation})
    assert moving.intent.name == "PROCEED"
    runtime.dispatch("PROGRESS", 0)
    return clock, action, sink, runtime


def test_route_binding_accepts_exact_identity_and_rejects_map_mismatch():
    binding = node.bind_route(Manifest(), goal())
    assert binding.direction == "outbound"
    assert binding.route_id == "route-out"
    try:
        node.bind_route(Manifest(), goal(map_sha256="e" * 64))
    except ValueError as exc:
        assert "map hash mismatch" in str(exc)
    else:
        raise AssertionError("mismatched map was accepted")


def test_collision_caution_remains_nonblocking_while_stop_and_unknown_block():
    assert node._collision_evidence(1, 1, 2) == "clear"
    assert node._collision_evidence(2, 1, 2) == "clear"
    assert node._collision_evidence(0, 1, 2) == "blocked"
    assert node._collision_evidence(3, 1, 2) == "blocked"


def test_geofence_margin_and_identity_mismatch_never_clear_mission_evidence():
    exact = dict(
        inside_state=1,
        reason_mask=0,
        route_id="route-out",
        expected_route_id="route-out",
        manifest_sha256="b" * 64,
        expected_manifest_sha256="b" * 64,
    )
    assert node._geofence_evidence(state=1, **exact)
    assert not node._geofence_evidence(state=2, **exact)
    assert not node._geofence_evidence(state=1, **dict(exact, reason_mask=1))
    assert not node._geofence_evidence(
        state=1, **dict(exact, route_id="route-forged"))
    assert not node._geofence_evidence(
        state=1, **dict(exact, manifest_sha256="c" * 64))


def test_normal_move_base_active_reason_is_not_an_internal_fault():
    assert not node._move_base_failure_reason("move_base_active")
    assert not node._move_base_failure_reason("ready")
    for reason in (
        "move_base_lost",
        "move_base_aborted",
        "stale_move_base",
        "move_base action unavailable",
        "action callback: invalid result",
    ):
        assert node._move_base_failure_reason(reason)


def test_route_progress_targets_the_waypoint_after_latest_reached():
    assert node.next_waypoint_index(0, 4) == 1
    assert node.next_waypoint_index(1, 4) == 2
    assert node.next_waypoint_index(3, 4) == 3
    for reached, count in ((-1, 4), (4, 4), (0, 0), (True, 4), (0, True)):
        try:
            node.next_waypoint_index(reached, count)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid route progress was accepted")


def test_speed_zone_classification_preserves_surfaces_and_exact_candidate():
    safety_hash = "93ca862dac1fbdd5914d93b2d2c325fe2742aef2a05289d44d0d4fe45989de57"
    map_hash = "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278"
    assert node.classify_speed_zone(["campus-road"]) == "road"
    assert node.classify_speed_zone(["north-sidewalk"]) == "sidewalk"
    assert node.classify_speed_zone(
        ["zone-simulation-candidate", "candidate-unsurveyed"],
        safety_hash, map_hash) == "simulation_unsurveyed"


def test_speed_zone_classification_rejects_unknown_and_mixed_candidate_tags():
    safety_hash = "93ca862dac1fbdd5914d93b2d2c325fe2742aef2a05289d44d0d4fe45989de57"
    map_hash = "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278"
    for zone_ids, bound_safety, bound_map in (
            ([], "", ""),
            (["unknown"], "", ""),
            (["candidate-unsurveyed", "candidate-unsurveyed"], safety_hash, map_hash),
            (["zone-simulation-candidate", "candidate-unsurveyed"], "", ""),
            (["zone-simulation-candidate", "candidate-unsurveyed"], safety_hash, "wrong"),
            (["candidate-unsurveyed", "unknown"], safety_hash, map_hash)):
        try:
            node.classify_speed_zone(zone_ids, bound_safety, bound_map)
        except ValueError:
            pass
        else:
            raise AssertionError("unclassified speed zone was accepted")


def test_empty_optional_localization_zone_preserves_simulation_binding():
    safety_hash = "93ca862dac1fbdd5914d93b2d2c325fe2742aef2a05289d44d0d4fe45989de57"
    map_hash = "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278"
    zone_ids = node._active_speed_zone_ids(
        SimpleNamespace(zone_id="zone-simulation-candidate"),
        SimpleNamespace(zone_id=""),
        SimpleNamespace(zone_ids=("candidate-unsurveyed",)),
    )

    assert zone_ids == ["zone-simulation-candidate", "candidate-unsurveyed"]
    assert node.classify_speed_zone(
        zone_ids, safety_hash, map_hash) == "simulation_unsurveyed"


def test_unexpected_nonempty_localization_zone_remains_speed_blocking():
    zone_ids = node._active_speed_zone_ids(
        SimpleNamespace(zone_id="zone-simulation-candidate"),
        SimpleNamespace(zone_id="unexpected-localization-zone"),
        SimpleNamespace(zone_ids=("candidate-unsurveyed",)),
    )

    with pytest.raises(ValueError, match="active zone is not speed classified"):
        node.classify_speed_zone(
            zone_ids,
            "93ca862dac1fbdd5914d93b2d2c325fe2742aef2a05289d44d0d4fe45989de57",
            "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278",
        )


def test_empty_geofence_and_localization_zones_do_not_classify_segment_alone():
    zone_ids = node._active_speed_zone_ids(
        SimpleNamespace(zone_id=""),
        SimpleNamespace(zone_id=""),
        SimpleNamespace(zone_ids=("candidate-unsurveyed",)),
    )

    assert zone_ids == ["candidate-unsurveyed"]
    with pytest.raises(ValueError, match="active zone is not speed classified"):
        node.classify_speed_zone(zone_ids)


def test_malformed_optional_zone_remains_speed_blocking():
    with pytest.raises(ValueError, match="active zone is not speed classified"):
        node._active_speed_zone_ids(
            SimpleNamespace(zone_id="zone-simulation-candidate"),
            SimpleNamespace(zone_id=object()),
            SimpleNamespace(zone_ids=("candidate-unsurveyed",)),
        )


def test_mission_cancelled_truth_table_and_sole_topic():
    for state in ("DISARMED", "LOCALIZING", "READY", "GOAL_REACHED",
                  "ABORTED", "FAULT"):
        assert node._mission_cancelled(state)
    for state in ("NAVIGATING", "PAUSED_OBSTACLE"):
        assert not node._mission_cancelled(state)

    source = (SCRIPTS / "mission_node.py").read_text(encoding="utf-8")
    assert source.count('"/safety/mission_cancelled"') == 1
    assert ('Publisher(\n        "/safety/mission_cancelled", Bool, '
            'queue_size=1, latch=False)') in source
    assert "mission_cancelled_pub.publish(" in source


def test_publication_limiter_bounds_high_rate_identical_samples():
    limiter = node.PublicationLimiter(0.1)
    emitted = [stamp for stamp in (0.0, 0.001, 0.02, 0.099, 0.1, 0.15, 0.2)
               if limiter.should_publish(("HOLD",), stamp)]
    assert emitted == [0.0, 0.1, 0.2]


def test_publication_limiter_publishes_changed_and_restrictive_samples_immediately():
    limiter = node.PublicationLimiter(0.1)
    assert limiter.should_publish(("PROCEED", 0.5), 1.0)
    assert not limiter.should_publish(("PROCEED", 0.5), 1.001)
    assert limiter.should_publish(("HOLD", 0.0), 1.002)
    assert limiter.should_publish(("FAULT", 0.0), 1.003)
    assert not limiter.should_publish(("FAULT", 0.0), 1.004)


def test_permissive_motion_transition_is_coalesced_without_sequence_gap():
    hold = (2, 0, 0, 0, 0.0, 0.0, "m", "r", "map", True)
    proceed = (3, 0, 1, 0, 0.25, 0.6, "m", "r", "map", False)
    slower = (3, 0, 1, 0, 0.12, 0.25, "m", "r", "map", False)
    fault = (8, 1024, 0, 1024, 0.0, 0.0, "m", "r", "map", True)

    limiter = node.PublicationLimiter(0.1)
    assert limiter.should_publish(hold, 1.0)
    assert not node.publication_change_is_urgent(
        limiter.published_signature, proceed, proceed_behavior=1)
    assert not limiter.should_publish(proceed, 1.001, urgent=False)
    assert limiter.published_signature == hold
    assert limiter.should_publish(proceed, 1.1, urgent=False)
    assert limiter.published_signature == proceed

    shaped = (3, 0, 1, 0, 0.23, 0.58, "m", "r", "map", False)
    assert not node.publication_change_is_urgent(
        limiter.published_signature, shaped, proceed_behavior=1)
    assert not limiter.should_publish(shaped, 1.101, urgent=False)
    assert limiter.published_signature == proceed

    assert node.publication_change_is_urgent(
        limiter.published_signature, slower, proceed_behavior=1)
    assert limiter.should_publish(slower, 1.101, urgent=True)
    assert node.publication_change_is_urgent(
        limiter.published_signature, fault, proceed_behavior=1)
    assert limiter.should_publish(fault, 1.102, urgent=True)


def test_publication_limiter_fails_closed_on_bad_or_regressed_time():
    limiter = node.PublicationLimiter(0.1)
    assert limiter.should_publish(("HOLD",), 10.0)
    for bad_time in (9.0, float("nan"), float("inf"), -float("inf")):
        assert not limiter.should_publish(("HOLD",), bad_time)
    assert limiter.should_publish(("FAULT",), float("nan"))
    assert not limiter.should_publish(("FAULT",), float("nan"))
    assert not limiter.should_publish(("FAULT",), 9.0)
    assert limiter.should_publish(("FAULT",), 10.1)


def test_external_sequence_advances_only_for_actual_publications():
    limiter = node.PublicationLimiter(0.1)
    sequence = 0
    observed = []
    samples = [(0.0, ("HOLD",)), (0.01, ("HOLD",)), (0.02, ("FAULT",)),
               (0.03, ("FAULT",)), (0.12, ("FAULT",))]
    for stamp, signature in samples:
        if limiter.should_publish(signature, stamp):
            sequence += 1
            observed.append(sequence)
    assert observed == [1, 2, 3]
    assert sequence == len(observed)


def test_waypoint_completion_defers_next_goal_outside_action_callback():
    queue = node.DeferredWaypointQueue()
    assert queue.pop() is None
    queue.defer(1)
    queue.defer(2)
    assert queue.pop() == 2
    assert queue.pop() is None
    queue.defer(3)
    queue.clear()
    assert queue.pop() is None
    for invalid in (-1, True, 1.5):
        try:
            queue.defer(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid waypoint index was queued")

    source = (SCRIPTS / "mission_node.py").read_text(encoding="utf-8")
    done_callback = source[source.index("        def done_cb("):
                           source.index("        def feedback_cb(")]
    execute_loop = source[source.index("    def execute(goal: Any) -> None:"):
                          source.index("    server = actionlib.SimpleActionServer")]
    assert "deferred=True" in done_callback
    assert "drain_deferred_waypoint()" in execute_loop



def test_blind_startup_stays_disarmed_then_requires_clear_to_authorize_motion():
    clock, action, sink = Clock(), Action(), Sink()
    runtime = node.MissionRuntime(
        MissionFSM(MissionConfig(), clock), EventType, MissionEvent,
        action, clock, sink)

    blind = runtime.dispatch("COLLISION", "blocked")
    assert blind.state.name == "DISARMED"
    assert not runtime.fault_latched

    runtime.operator_arm()
    waiting = runtime.begin(node.bind_route(Manifest(), goal()))
    runtime.dispatch("LOCALIZATION", True)
    runtime.dispatch("GEOFENCE", True)
    runtime.dispatch("SLOPE", "safe")
    runtime.dispatch("MODE", True)
    blocked = runtime.dispatch("DRIVER", True)
    assert waiting.state.name == "LOCALIZING"
    assert blocked.state.name == "LOCALIZING"
    assert blocked.intent.name == "HOLD"
    assert blocked.send_waypoint_index is None

    ready = runtime.dispatch("COLLISION", "clear")
    assert ready.state.name == "READY"
    assert ready.intent.name == "HOLD"
    assert ready.send_waypoint_index == 0
    moving = runtime.dispatch(
        "MOVE_BASE_ACTIVE", {"generation": ready.goal_generation})
    assert moving.state.name == "NAVIGATING"
    assert moving.intent.name == "PROCEED"


    assert not node._mission_cancelled(moving.state)


def test_success_sends_one_waypoint_at_a_time():
    _, action, _, runtime = navigating()
    first = runtime.dispatch(
        "MOVE_BASE_SUCCEEDED", {"generation": runtime.output.goal_generation})
    assert first.send_waypoint_index == 1
    runtime.dispatch("MOVE_BASE_ACTIVE", {"generation": first.goal_generation})
    complete = runtime.dispatch(
        "MOVE_BASE_SUCCEEDED", {"generation": runtime.output.goal_generation})
    assert complete.terminal_status == "SUCCEEDED"
    assert complete.intent.name == "HOLD"
    assert action.cancels == 0


def test_ready_hold_does_not_cancel_pending_waypoint_goal():
    _, action, _, runtime = harness()
    ready = evidence(runtime)
    assert ready.state.name == "READY"
    assert ready.intent.name == "HOLD"
    assert ready.send_waypoint_index == 0
    assert action.cancels == 0

    repeated = runtime.dispatch("TICK")
    assert repeated.state.name == "READY"
    assert repeated.intent.name == "HOLD"
    assert action.cancels == 0

    source = (SCRIPTS / "mission_node.py").read_text(encoding="utf-8")
    emit_source = source[source.index("    def emit("):source.index("    clock = lambda:")]
    assert "else:\n            move_base.cancel_goal()" not in emit_source
    assert "speed policy HOLD" in emit_source
    assert "move_base.cancel_goal()" in emit_source


def test_ros_evidence_callbacks_use_serialized_receipt_time():
    source = (SCRIPTS / "mission_node.py").read_text(encoding="utf-8")
    adapter_dispatch = source[
        source.index("    def dispatch(kind: str, value: Any, _source_stamp: Any)"):
        source.index("    def safety_cb(")
    ]
    assert "runtime.dispatch(kind, value=value)" in adapter_dispatch
    assert "stamp.to_sec()" not in adapter_dispatch
    assert "serialized receipt-time" in adapter_dispatch


def test_action_abort_cancels_and_latches_until_reset_and_rearm():
    _, action, _, runtime = navigating()
    aborted = runtime.dispatch(
        "MOVE_BASE_ABORTED", {"generation": runtime.output.goal_generation})
    assert aborted.terminal_status == "ABORTED"
    assert aborted.intent.name == "HOLD"
    assert action.cancels == 1
    assert runtime.fault_latched
    runtime.operator_reset()
    assert not runtime.armed_by_operator


def test_obstacle_pause_requires_hysteresis_and_explicit_resume():
    config = MissionConfig(evidence_timeout_s=3.0, action_timeout_s=3.0,
                           progress_timeout_s=3.0, obstacle_stop_entry_s=0.2,
                           obstacle_clear_s=1.0)
    clock, action, _, runtime = navigating(config)
    runtime.dispatch("COLLISION", "blocked")
    clock.advance(0.19)
    assert runtime.dispatch("TICK").state.name == "NAVIGATING"
    clock.advance(0.02)
    paused = runtime.dispatch("TICK")
    assert paused.state.name == "PAUSED_OBSTACLE"
    assert paused.intent.name == "HOLD"
    assert action.cancels == 1
    runtime.dispatch("MOVE_BASE_CANCELED", {"generation": paused.goal_generation})
    runtime.dispatch("COLLISION", "clear")
    clock.advance(1.01)
    assert runtime.dispatch("TICK").state.name == "PAUSED_OBSTACLE"
    resumed = runtime.dispatch("RESUME")
    assert resumed.state.name == "READY"
    assert resumed.send_waypoint_index == 0


def test_localization_loss_and_stale_progress_fail_closed():
    _, action, _, runtime = navigating()
    lost = runtime.dispatch("LOCALIZATION", False)
    assert lost.state.name == "FAULT"
    assert lost.intent.name == "HOLD"
    assert action.cancels == 1

    config = MissionConfig(evidence_timeout_s=5.0, action_timeout_s=5.0,
                           progress_timeout_s=0.2)
    clock, action, _, runtime = navigating(config)
    clock.advance(0.21)
    stale = runtime.dispatch("TICK")
    assert stale.reason == "stale_progress"
    assert stale.intent.name == "HOLD"
    assert action.cancels == 1


def test_emitted_intent_expires_when_node_dies():
    clock, _, sink, runtime = navigating()
    published_at = sink.samples[-1][0]
    timeout = 0.25
    clock.advance(timeout + 0.001)
    assert clock() - published_at > timeout
    assert sink.samples[-1][1].intent.name == "PROCEED"
    # The safety consumer must treat this old source stamp as HOLD; the node
    # deliberately uses a non-latched publisher and cannot refresh after death.


def test_execute_loop_cannot_fall_through_on_server_is_active():
    source = (SCRIPTS / "mission_node.py").read_text(encoding="utf-8")
    execute_source = source[source.index("    def execute(goal: Any) -> None:"):
                            source.index("    server = actionlib.SimpleActionServer")]
    assert "while not rospy.is_shutdown():" in execute_source
    assert "server.is_active()" not in execute_source
    assert "active_heartbeat = start_active_heartbeat(binding)" in execute_source
    assert "last_active_publish" not in execute_source
    assert "server.set_preempted(result, result.message)" in execute_source
    assert "server.set_succeeded(result, result.message)" in execute_source
    assert execute_source.count("server.set_aborted(result, result.message)") == 3
    assert "rospy.Timer(rospy.Duration(0.5)" not in source


def test_active_route_heartbeat_preserves_identity_sequence():
    source = (SCRIPTS / "mission_node.py").read_text(encoding="utf-8")
    publish_source = source[source.index("    def publish_active("):
                            source.index("    def send_waypoint(")]
    assert "if new_activation:" in publish_source
    assert publish_source.index("if new_activation:") < publish_source.index(
        "activation_sequence += 1")
    assert "message.activation_sequence = selected_sequence" in publish_source
def test_route_active_heartbeat_uses_monotonic_receipts_across_sim_time_reset():
    heartbeat = node.RouteActiveHeartbeat(0.5)
    heartbeat.record(10.0)
    assert heartbeat.delay_s(10.2) == pytest.approx(0.3)
    assert heartbeat.delay_s(9.0) == 0.0
    heartbeat.record(9.0)
    assert heartbeat.delay_s(9.5) == 0.0

    source = (SCRIPTS / "mission_node.py").read_text(encoding="utf-8")
    heartbeat_source = source[source.index("    def start_active_heartbeat("):
                              source.index("    def send_waypoint(")]
    assert "time.monotonic()" in heartbeat_source
    assert "stopped.wait(" in heartbeat_source
    assert 'queue_size=1, latch=False' in source


def test_execute_route_action_is_private_to_mission_node():
    source = (SCRIPTS / "mission_node.py").read_text(encoding="utf-8")
    assert 'SimpleActionServer("~execute_route",' in source
    assert 'SimpleActionServer("execute_route",' not in source
