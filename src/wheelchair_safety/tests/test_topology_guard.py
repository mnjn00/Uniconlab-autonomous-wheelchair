from dataclasses import replace
import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
MODULE = ROOT / "src" / "wheelchair_safety" / "scripts" / "topology_guard.py"
spec = importlib.util.spec_from_file_location("topology_guard", MODULE)
topology_guard = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = topology_guard
spec.loader.exec_module(topology_guard)


def valid_snapshot(profile="sim", input_topic="/cmd_vel_nav", output_topic=None, events=False,
                   motion_active=True):
    output_topic = output_topic or ("/shadow/cmd_vel_safe" if profile == "replay" else "/cmd_vel_safe")
    expected = topology_guard.expected_graph(
        profile, input_topic, output_topic,
        hardware_authority_proven=profile == "hardware_enabled")
    publishers, subscribers, observations = {}, {}, {}
    deadlines = topology_guard.profile_deadlines(expected)
    for topic, authority in expected.authorities.items():
        command_required = authority.required_when_motion_active and motion_active is True
        if not authority.publisher_optional or events or command_required:
            publishers[topic] = (authority.publishers[0],)
        subscribers[topic] = authority.subscribers
        if (
            authority.subscriber_alternatives
            and not authority.subscriber_alternatives_optional
        ):
            subscribers[topic] += (authority.subscriber_alternatives[0],)
        if topic in expected.timed_topics:
            subscribers[topic] += ("topology_guard",)
            if not authority.required_when_motion_active or motion_active is True:
                observations[topic] = topology_guard.TopicObservation(
                    1, True, 9.9, deadlines[topic])
    transforms = {
        edge: (topology_guard.TransformObservation(
            authority.owners[0], 9.9, 0.0 if authority.static else 100.0,
            authority.static, None if authority.static else 0.10),)
        for edge, authority in expected.transforms.items()
    }
    return expected, topology_guard.GraphSnapshot(
        publishers=publishers,
        subscribers=subscribers,
        transforms=transforms,
        observations=observations,
        captured_at_s=10.0,
        master_evidence_complete=True,
        tf_evidence_complete=True,
        timing_evidence_complete=True,
        motion_active=motion_active,
    )


@pytest.mark.parametrize(("profile", "output_topic", "safe_consumers"), (
    ("sim", "/cmd_vel_safe", ("collision_supervisor", "simulation_controller_adapter")),
    ("replay", "/shadow/cmd_vel_safe", ("collision_supervisor",)),
    ("hardware_shadow", "/cmd_vel_safe", ("collision_supervisor", "hardware_shadow_adapter")),
))
def test_exact_profile_snapshot_can_clear(profile, output_topic, safe_consumers):
    expected, snapshot = valid_snapshot(profile)
    assert expected.authorities[output_topic].subscribers == safe_consumers
    assert expected.authorities["/decision/motion_intent"].publishers == ("wheelchair_mission",)
    assert expected.authorities["/route/progress"].publishers == ("route_manager",)
    result = topology_guard.TopologyAuditor(expected).audit(snapshot)
    assert result.ok, result.violations
    assert result.reason_mask == 0


def test_missing_provider_and_evidence_fail_closed():
    expected, snapshot = valid_snapshot()
    publishers = dict(snapshot.publishers)
    publishers.pop("/hardware/driver_status")
    result = topology_guard.TopologyAuditor(expected).audit(replace(
        snapshot, publishers=publishers, tf_evidence_complete=False,
        timing_evidence_complete=False))
    assert not result.ok
    assert result.reason_mask & topology_guard.GRAPH_TOPOLOGY
    assert result.reason_mask & topology_guard.TF
    assert result.reason_mask & topology_guard.DEADLINE_MISS
    assert result.reason_mask & topology_guard.INPUT_UNKNOWN


def test_duplicate_and_rogue_publishers_and_subscribers_stop():
    expected, snapshot = valid_snapshot()
    publishers, subscribers = dict(snapshot.publishers), dict(snapshot.subscribers)
    publishers["/cmd_vel_nav"] += ("rogue_navigation",)
    subscribers["/cmd_vel_safe"] += ("twist_mux",)
    result = topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, publishers=publishers, subscribers=subscribers))
    assert not result.ok
    assert any("publisher authority" in item for item in result.violations)
    assert any("unauthorized subscribers" in item for item in result.violations)
    assert any("forbidden relay/mux/plugin" in item for item in result.violations)

    publishers = dict(snapshot.publishers)
    publishers["/cmd_vel_nav"] = ("/primary/move_base", "/duplicate/move_base")
    assert not topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, publishers=publishers)).ok


def test_profile_safe_command_consumers_include_collision_and_one_or_no_sink():
    sim, _ = valid_snapshot("sim")
    replay, _ = valid_snapshot("replay")
    shadow, _ = valid_snapshot("hardware_shadow")
    assert sim.authorities["/cmd_vel_safe"].subscribers == (
        "collision_supervisor", "simulation_controller_adapter")
    assert replay.authorities["/shadow/cmd_vel_safe"].subscribers == (
        "collision_supervisor",)
    assert shadow.authorities["/cmd_vel_safe"].subscribers == (
        "collision_supervisor", "hardware_shadow_adapter")


def test_topology_observers_are_passive_per_edge():
    expected, snapshot = valid_snapshot()
    result = topology_guard.TopologyAuditor(expected).audit(snapshot)
    assert result.ok
    assert "topology_guard" in result.passive_nodes
    # Its authoritative publication is still required; passive classification applies to subscriptions.
    publishers = dict(snapshot.publishers)
    publishers["/safety/topology"] = ("rogue",)
    assert not topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, publishers=publishers)).ok


def test_sim_qualification_observers_have_exact_per_topic_grants():
    expected, snapshot = valid_snapshot("sim")
    subscribers = dict(snapshot.subscribers)
    scenario_topics = topology_guard.SIM_OBSERVER_TOPIC_GRANTS["rc_scenario_driver"]
    collector_topics = topology_guard.SIM_OBSERVER_TOPIC_GRANTS["rc_metrics_collector"]

    for topic in scenario_topics:
        assert expected.authorities[topic].allowed_subscribers == (
            "rc_scenario_driver", "rc_metrics_collector")
        subscribers[topic] += ("rc_scenario_driver", "rc_metrics_collector")
    for topic in set(collector_topics) - set(scenario_topics):
        assert expected.authorities[topic].allowed_subscribers == ("rc_metrics_collector",)
        subscribers[topic] += ("rc_metrics_collector",)
    snapshot = replace(snapshot, subscribers=subscribers)
    assert topology_guard.TopologyAuditor(expected).audit(snapshot).ok

    publishers = dict(snapshot.publishers)
    publishers["/route/progress"] = ("rc_scenario_driver",)
    result = topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, publishers=publishers))
    assert not result.ok
    assert any("/route/progress publisher authority" in item for item in result.violations)

    for topic in set(collector_topics) - set(scenario_topics):
        assert "rc_scenario_driver" not in expected.authorities[topic].allowed_subscribers
    for topic in ("/safety/localization", "/localization/candidate", "/route/active"):
        assert "rc_scenario_driver" not in expected.authorities[topic].allowed_subscribers
        assert "rc_metrics_collector" not in expected.authorities[topic].allowed_subscribers


@pytest.mark.parametrize("profile", ("replay", "hardware_shadow", "hardware_enabled"))
@pytest.mark.parametrize(
    ("observer", "topic"),
    (
        ("rc_scenario_driver", "/route/progress"),
        ("rc_metrics_collector", "/route/progress"),
        ("rc_metrics_collector", "/cmd_vel_nav"),
    ),
)
def test_qualification_observers_are_rejected_outside_sim(profile, observer, topic):
    expected, snapshot = valid_snapshot(profile)
    subscribers = dict(snapshot.subscribers)
    subscribers[topic] += (observer,)
    result = topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, subscribers=subscribers))
    assert not result.ok
    assert any("{} unauthorized subscribers".format(topic) in item
               for item in result.violations)


def test_differently_named_observer_is_rejected_in_sim():
    expected, snapshot = valid_snapshot("sim")
    subscribers = dict(snapshot.subscribers)
    subscribers["/route/progress"] += ("rc_metrics_collector_extra",)
    result = topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, subscribers=subscribers))
    assert not result.ok
    assert any("/route/progress unauthorized subscribers" in item
               for item in result.violations)


def test_event_absence_is_allowed_but_presence_is_strict_and_untimed():
    expected, absent = valid_snapshot(events=False)
    assert not set(topology_guard.EVENT_TOPICS) & set(expected.timed_topics)
    assert topology_guard.TopologyAuditor(expected).audit(absent).ok

    _, present = valid_snapshot(events=True)
    assert topology_guard.TopologyAuditor(expected).audit(present).ok
    publishers = dict(present.publishers)
    publishers["/safety/estop"] = ("operator_io", "rogue_io")
    result = topology_guard.TopologyAuditor(expected).audit(replace(present, publishers=publishers))
    assert not result.ok
    assert any("/safety/estop publisher authority" in item for item in result.violations)

def test_exact_active_snapshot_without_explicit_motion_flag_is_compatible():
    expected, snapshot = valid_snapshot(motion_active=True)
    result = topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, motion_active=None))
    assert result.ok, result.violations


def test_hold_allows_absent_nav_command_but_active_motion_requires_fresh_command():
    expected, hold = valid_snapshot(motion_active=False)
    publishers, observations = dict(hold.publishers), dict(hold.observations)
    publishers.pop("/cmd_vel_nav", None)
    observations.pop("/cmd_vel_nav", None)
    hold = replace(hold, publishers=publishers, observations=observations)
    assert topology_guard.TopologyAuditor(expected).audit(hold).ok

    active = replace(hold, motion_active=True)
    result = topology_guard.TopologyAuditor(expected).audit(active)
    assert not result.ok
    assert any("/cmd_vel_nav publisher authority" in item for item in result.violations)
    assert any("/cmd_vel_nav missing queue/deadline observation" in item for item in result.violations)

    unknown = topology_guard.TopologyAuditor(expected).audit(
        replace(hold, motion_active=None))
    assert not unknown.ok
    assert unknown.reason_mask & topology_guard.INPUT_UNKNOWN


def test_topology_signal_is_not_a_circular_timing_permission():
    expected, snapshot = valid_snapshot()
    assert "/safety/topology" not in expected.timed_topics
    assert "/safety/topology" not in snapshot.observations
    assert topology_guard.TopologyAuditor(expected).audit(snapshot).ok


def test_actual_aliases_and_independent_localization_identities_are_exact():
    expected, snapshot = valid_snapshot()
    assert expected.authorities["/localization/candidate"].publishers == (
        "localization_adapter", "selected_localization_adapter", "base_model_localization_adapter")
    assert "independent_localization_guard" in expected.authorities["/localization/status"].publishers

    publishers = dict(snapshot.publishers)
    publishers["/localization/status"] = ("independent_localization_guard",)
    publishers["/safety/localization"] = ("independent_localization_guard",)
    subscribers = dict(snapshot.subscribers)
    subscribers["/localization/candidate"] = (
        "wheelchair_route_safety", "independent_localization_guard")
    assert topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, publishers=publishers, subscribers=subscribers)).ok

    for topic in ("/localization/status", "/safety/localization"):
        rogue_publishers = dict(snapshot.publishers)
        rogue_publishers[topic] = ("localization_adapter",)
        result = topology_guard.TopologyAuditor(expected).audit(
            replace(snapshot, publishers=rogue_publishers))
        assert not result.ok
        assert any("%s publisher authority" % topic in item for item in result.violations)


    transforms = dict(snapshot.transforms)
    transforms[("map", "odom")] += (topology_guard.TransformObservation(
        "selected_localization_adapter", 9.9, 100.0, False, 0.10),)
    result = topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, publishers=publishers, subscribers=subscribers, transforms=transforms))
    assert not result.ok
    assert result.reason_mask & topology_guard.TF


def test_deadline_boundary_is_inclusive_but_future_and_stale_stop():
    expected, snapshot = valid_snapshot()
    observations = dict(snapshot.observations)
    observations["/cmd_vel_nav"] = topology_guard.TopicObservation(1, True, 9.7, 0.3)
    assert topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, observations=observations)).ok
    for receipt in (9.699999, 10.001):
        observations = dict(snapshot.observations)
        observations["/cmd_vel_nav"] = topology_guard.TopicObservation(1, True, receipt, 0.3)
        result = topology_guard.TopologyAuditor(expected).audit(
            replace(snapshot, observations=observations))
        assert not result.ok
        assert result.reason_mask & topology_guard.DEADLINE_MISS


def test_queue_contract_is_exactly_latest_only_one():
    expected, snapshot = valid_snapshot()
    observations = dict(snapshot.observations)
    observations["/safety/mode"] = topology_guard.TopicObservation(2, False, 9.9, 0.15)
    result = topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, observations=observations))
    assert not result.ok
    assert result.reason_mask & topology_guard.BACKPRESSURE


def test_profile_command_topic_remap_is_audited_as_selected():
    expected, snapshot = valid_snapshot("sim", "/nav/selected_cmd", "/safe/selected_cmd")
    collector_topics = topology_guard.sim_observer_topic_grants(
        "/nav/selected_cmd", "/safe/selected_cmd")["rc_metrics_collector"]
    assert collector_topics == (
        "/route/progress", "/localization/status", "/route_safety/geofence_status",
        "/safety/collision_status", "/safety/slope_status", "/nav/selected_cmd",
        "/safe/selected_cmd", "/wheelchair_base_controller/cmd_vel",
    )
    assert expected.authorities["/nav/selected_cmd"].allowed_subscribers == (
        "rc_metrics_collector",)
    assert expected.authorities["/safe/selected_cmd"].allowed_subscribers == (
        "rc_metrics_collector",)
    assert expected.command_topics == (
        "/nav/selected_cmd",
        "/safe/selected_cmd",
        "/wheelchair_base_controller/cmd_vel",
    )
    assert topology_guard.TopologyAuditor(expected).audit(snapshot).ok
    publishers = dict(snapshot.publishers)
    publishers["/cmd_vel_nav"] = ("move_base",)
    assert not topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, publishers=publishers)).ok


def test_unknown_profile_and_wrong_replay_output_are_rejected():
    with pytest.raises(ValueError, match="unknown topology profile"):
        topology_guard.expected_graph("production", "/cmd_vel_nav", "/cmd_vel_safe")
    with pytest.raises(ValueError, match="replay safe output"):
        topology_guard.expected_graph("replay", "/cmd_vel_nav", "/cmd_vel_safe")
    with pytest.raises(ValueError, match="unknown topology profile"):
        topology_guard.expected_graph("native", "/cmd_vel_nav", "/cmd_vel_safe")


def test_hardware_enabled_requires_explicit_boundary_contract_proof():
    with pytest.raises(ValueError, match="proven hardware boundary authority"):
        topology_guard.expected_graph(
            "hardware_enabled", "/cmd_vel_nav", "/cmd_vel_safe")
    expected = topology_guard.expected_graph(
        "hardware_enabled", "/cmd_vel_nav", "/cmd_vel_safe",
        hardware_authority_proven=True)
    assert expected.profile == "hardware_enabled"
    assert expected.authorities["/cmd_vel_safe"].subscribers == (
        "collision_supervisor", "hardware_enabled_adapter")
    assert expected.authorities["/hardware/driver_status"].publishers == (
        "hardware_enabled_adapter",)


def test_duplicate_rogue_and_stale_dynamic_tf_stop():
    expected, snapshot = valid_snapshot()
    edge = ("map", "odom")
    for evidence in (
        snapshot.transforms[edge] + (topology_guard.TransformObservation(
            "rogue", 9.9, 100.0, False, 0.10),),
        (topology_guard.TransformObservation(expected.transforms[edge].owners[0],
                                             9.7, 100.0, False, 0.10),),
        (topology_guard.TransformObservation(expected.transforms[edge].owners[0], 9.9, 100.0,
                                             False, 0.251),),
    ):
        transforms = dict(snapshot.transforms)
        transforms[edge] = evidence
        result = topology_guard.TopologyAuditor(expected).audit(
            replace(snapshot, transforms=transforms))
        assert not result.ok
        assert result.reason_mask & topology_guard.TF


def test_odom_and_fixed_tf_are_unique_and_static_zero_stamp_is_valid():
    expected, snapshot = valid_snapshot()
    assert topology_guard.TopologyAuditor(expected).audit(snapshot).ok
    edge = ("base_link", "lidar_link")
    transforms = dict(snapshot.transforms)
    transforms[edge] += (topology_guard.TransformObservation(
        "rogue_state_publisher", 9.9, 0.0, True),)
    result = topology_guard.TopologyAuditor(expected).audit(
        replace(snapshot, transforms=transforms))
    assert not result.ok
    assert result.reason_mask & topology_guard.TF


def test_deadline_observer_freezes_deadlines_and_keeps_latest_receipt():
    source = {"/cmd_vel_nav": 0.30}
    observer = topology_guard.DeadlineObserver(source)
    source["/cmd_vel_nav"] = 99.0
    observer.observe("/cmd_vel_nav", 1.0)
    observer.observe("/cmd_vel_nav", 2.0)
    evidence = observer.evidence()["/cmd_vel_nav"]
    assert evidence.queue_size == 1
    assert evidence.latest_only is True
    assert evidence.deadline_s == 0.30
    assert evidence.last_receipt_s == 2.0
def test_route_active_deadline_is_a_monotonic_receipt_contract():
    observer = topology_guard.DeadlineObserver({"/route/active": 0.75})
    observer.observe("/route/active", 42.0)
    evidence = observer.evidence()["/route/active"]
    assert evidence.queue_size == 1
    assert evidence.latest_only is True
    assert evidence.deadline_s == 0.75
    assert evidence.last_receipt_s == 42.0

    source = MODULE.read_text(encoding="utf-8")
    observe_source = source[source.index("    def observe("):
                           source.index("    def observe_motion_intent(")]
    assert "time.monotonic()" in observe_source
    assert "header.stamp" not in observe_source
