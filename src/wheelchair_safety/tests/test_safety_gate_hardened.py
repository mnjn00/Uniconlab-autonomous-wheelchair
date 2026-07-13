from pathlib import Path
from dataclasses import replace
import inspect
import importlib.util
import math
import sys
import threading
import time
import types
import pytest

ROOT = Path(__file__).resolve().parents[3]
MODULE = ROOT / "src" / "wheelchair_safety" / "scripts" / "safety_gate.py"
spec = importlib.util.spec_from_file_location("safety_gate_hardened", MODULE)
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)

POLICIES = dict(gate._DEFAULT_POLICY_SHA256)


def evidence(name, stamp=100.0, state=gate.CLEAR, reason=0, policy=None,
             sequence=1, **caps):
    if policy is None:
        policy = "" if name == "motion_intent" else POLICIES[name]
    source = "topology_guard" if name == "topology" else name
    return gate.SignalEvidence(state, stamp, stamp, reason, source, policy, sequence, **caps)


def inputs(now=100.0, motion_intent_sequence=1, topology_sequence=1, **changes):
    values = dict(
        cmd=gate.VelocityCommand(0.2, 0.1), now_s=now,
        cmd_source_stamp_s=now, cmd_receipt_stamp_s=now,
        motion_intent=evidence("motion_intent", now, max_linear_mps=0.5,
                               max_angular_rps=0.8,
                               sequence=motion_intent_sequence),
        geofence=evidence("geofence", now), collision=evidence("collision", now),
        slope=evidence("slope", now), localization=evidence("localization", now),
        mode=evidence("mode", now), driver=evidence("driver", now),
        topology=evidence("topology", now, sequence=topology_sequence),
        e_stop=False, manual_or_disarmed=True, stationary=True,
        mission_cancelled=True,
        reset_driver_healthy=True,
    )
    values.update(changes)
    return gate.GateInputs(**values)


def assert_exact_safe_zero(decision):
    assert decision.command.values() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert all(math.isfinite(v) for v in decision.command.values())
    assert not decision.armed


def test_independent_estop_sources_require_both_reports_and_or_assertions():
    node = object.__new__(gate.SafetyGateRosNode)
    node.driver_estop, node.external_estop = None, False
    assert node._combined_estop() is None
    node.driver_estop, node.external_estop = True, False
    assert node._combined_estop() is True
    node.driver_estop, node.external_estop = False, True
    assert node._combined_estop() is True
    node.driver_estop = node.external_estop = False
    assert node._combined_estop() is False
def test_ros_request_callbacks_require_post_start_low_and_queue_rising_edges():
    class Bool:
        def __init__(self, data):
            self.data = data

    node = object.__new__(gate.SafetyGateRosNode)
    node._input_lock = threading.RLock()
    node.arm_level = node.reset_level = None
    node.arm_low_observed = node.reset_low_observed = False
    node.arm_requested = node.reset_requested = False
    node._arm_cb(Bool(True))
    assert not node.arm_requested
    node._arm_cb(Bool(False))
    node._arm_cb(Bool(True))
    assert node.arm_requested
    node.arm_requested = False
    node._arm_cb(Bool(True))
    assert not node.arm_requested
    node._reset_cb(Bool(False))
    node._reset_cb(Bool(True))
    assert node.reset_requested


def test_clock_fault_is_an_immediate_exact_zero():
    decision = gate.SafetyGateCore().evaluate(inputs(arm_request=True, clock_fault=True))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.CLOCK

def test_clean_unarmed_evidence_is_disarmed_startup():
    decision = gate.SafetyGateCore().evaluate(inputs())
    assert_exact_safe_zero(decision)
    assert decision.state == gate.DISARMED
    assert decision.reason_mask == gate.STARTUP
    assert decision.reason == "startup"


@pytest.mark.parametrize("change", [
    {"collision": None},
    {"collision": evidence("collision", state=gate.STOP, reason=gate.COLLISION)},
    {"internal_fault": True},
])
def test_evidence_faults_are_never_mislabeled_disarmed(change):
    decision = gate.SafetyGateCore().evaluate(inputs(**change))
    assert_exact_safe_zero(decision)
    assert decision.state in (gate.STOPPED, gate.FAULT, gate.LATCHED)
    assert decision.state != gate.DISARMED
    assert decision.reason_mask & ~gate.STARTUP


def test_reason_registry_is_exactly_bits_zero_through_36():
    assert tuple(gate.REASONS) == gate._REASON_NAMES
    assert len(gate.REASONS) == 37
    assert tuple(gate.REASONS.values()) == tuple(1 << bit for bit in range(37))
    assert gate.DEFINED_REASON_MASK == (1 << 37) - 1


@pytest.mark.parametrize("name,reason", [
    ("motion_intent", gate.STALE_INTENT), ("geofence", gate.GEOFENCE),
    ("collision", gate.COLLISION), ("slope", gate.SLOPE),
    ("localization", gate.LOCALIZATION), ("mode", gate.MODE),
    ("driver", gate.DRIVER), ("topology", gate.GRAPH_TOPOLOGY),
])
def test_every_missing_permission_is_unknown_zero_and_disarmed(name, reason):
    decision = gate.SafetyGateCore().evaluate(inputs(arm_request=True, **{name: None}))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.INPUT_UNKNOWN
    assert decision.reason_mask & reason


def test_missing_command_is_zero_and_disarmed():
    decision = gate.SafetyGateCore().evaluate(inputs(cmd=None, arm_request=True))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.STALE_CMD


def hold_intent(stamp):
    return evidence("motion_intent", stamp, state=gate.HOLD,
                    max_linear_mps=0.0, max_angular_rps=0.0)


def test_fresh_hold_arms_without_command_and_outputs_exact_finite_zero():
    decision = gate.SafetyGateCore().evaluate(inputs(
        cmd=None, motion_intent=hold_intent(100.0), arm_request=True))
    assert decision.armed
    assert decision.reason == "hold"
    assert decision.reason_mask == 0
    assert decision.command.values() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert all(math.isfinite(value) for value in decision.command.values())


@pytest.mark.parametrize("elapsed,armed", [
    (0.099999, True),
    (0.10, True),
    (0.100001, False),
])
def test_first_command_activation_grace_boundaries(elapsed, armed):
    core = gate.SafetyGateCore()
    assert core.evaluate(inputs(
        100.0, cmd=None, motion_intent=hold_intent(100.0),
        arm_request=True)).armed

    transition = core.evaluate(inputs(
        100.01, cmd=None, motion_intent_sequence=2, topology_sequence=2))
    assert transition.armed
    assert transition.reason == "activation_grace"
    decision = core.evaluate(inputs(
        100.01 + elapsed, cmd=None, motion_intent_sequence=3, topology_sequence=3))
    assert decision.armed is armed
    assert decision.command.values() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    if not armed:
        assert decision.reason_mask & gate.STALE_CMD


def test_pre_hold_command_is_not_reused_and_is_rejected_after_grace():
    core = gate.SafetyGateCore()
    old_command = dict(
        cmd=gate.VelocityCommand(0.3, 0.2),
        cmd_source_stamp_s=99.99, cmd_receipt_stamp_s=99.99,
    )
    hold = core.evaluate(inputs(
        100.0, motion_intent=hold_intent(100.0), arm_request=True,
        **old_command))
    assert hold.armed
    assert hold.command.is_zero()

    grace = core.evaluate(inputs(
        100.01, motion_intent_sequence=2, topology_sequence=2, **old_command))
    assert grace.armed
    assert grace.reason == "activation_grace"
    assert grace.command.is_zero()

    rejected = core.evaluate(inputs(
        100.110001, motion_intent_sequence=3, topology_sequence=3, **old_command))
    assert_exact_safe_zero(rejected)
    assert rejected.reason_mask & gate.STALE_CMD


def test_hold_transition_resets_command_activation():
    core = gate.SafetyGateCore()
    core.evaluate(inputs(100.0, cmd=None, motion_intent=hold_intent(100.0),
                         arm_request=True))
    core.evaluate(inputs(100.01, cmd=None, motion_intent_sequence=2, topology_sequence=2))
    moving = core.evaluate(inputs(
        100.02, motion_intent_sequence=3, topology_sequence=3,
        cmd_source_stamp_s=100.02, cmd_receipt_stamp_s=100.02))
    assert moving.armed
    assert not moving.command.is_zero()

    held = core.evaluate(inputs(
        100.03, motion_intent=evidence(
            "motion_intent", 100.03, state=gate.HOLD,
            max_linear_mps=0.0, max_angular_rps=0.0, sequence=4),
        topology_sequence=4, cmd_source_stamp_s=100.02, cmd_receipt_stamp_s=100.02))
    assert held.armed
    assert held.command.is_zero()
    released = core.evaluate(inputs(
        100.04, motion_intent_sequence=5, topology_sequence=5,
        cmd_source_stamp_s=100.02, cmd_receipt_stamp_s=100.02))
    assert released.armed
    assert released.reason == "activation_grace"
    assert released.command.is_zero()


def test_explicit_stop_from_hold_immediately_disarms():
    core = gate.SafetyGateCore()
    assert core.evaluate(inputs(
        100.0, cmd=None, motion_intent=hold_intent(100.0),
        arm_request=True)).armed
    stop = evidence("motion_intent", 100.01, state=gate.STOP,
                    reason=gate.MODE, max_linear_mps=0.0,
                    max_angular_rps=0.0)
    decision = core.evaluate(inputs(100.01, cmd=None, motion_intent=stop))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.MODE


def test_ros_adapter_maps_hold_as_distinct_zero_motion_intent():
    class Stamp:
        @staticmethod
        def to_sec():
            return 100.0

    class Header:
        stamp = Stamp()

    class Message:
        HOLD, PROCEED, SLOW, STOP = range(4)
        behavior = HOLD
        header = Header()
        reason_mask = 0
        sequence = 7
        max_linear_mps = 0.0
        max_angular_rps = 0.0

    node = gate.SafetyGateRosNode.__new__(gate.SafetyGateRosNode)
    node.evidence = {"motion_intent": None}
    node.internal_fault = False
    node._now = lambda: 100.0
    node._intent_cb(Message())

    mapped = node.evidence["motion_intent"]
    assert mapped.state == gate.HOLD
    assert mapped.state != gate.STOP
    assert mapped.max_linear_mps == 0.0
    assert mapped.max_angular_rps == 0.0


@pytest.mark.parametrize("name,ttl,reason", [
    ("motion_intent", 0.30, gate.STALE_INTENT),
    ("geofence", 0.25, gate.SENSOR_STALE),
    ("collision", 0.30, gate.SENSOR_STALE),
    ("slope", 0.10, gate.SENSOR_STALE),
    ("localization", 0.25, gate.SENSOR_STALE),
    ("mode", 0.15, gate.SENSOR_STALE),
    ("driver", 0.15, gate.SENSOR_STALE),
    ("topology", gate.TOPOLOGY_TTL_S, gate.GRAPH_TOPOLOGY),
])
@pytest.mark.parametrize("which", ["source", "receipt"])
def test_each_signal_checks_source_and_receipt_ttl_equality(name, ttl, reason, which):
    now = 100.0
    equal = evidence(name, now)
    equal = gate.SignalEvidence(equal.state,
        now - ttl if which == "source" else now,
        now - ttl if which == "receipt" else now,
        0, equal.source, equal.policy_sha256, 1,
        0.5 if name == "motion_intent" else None,
        0.8 if name == "motion_intent" else None)
    equal_decision = gate.SafetyGateCore().evaluate(
        inputs(now, arm_request=True, **{name: equal}))
    assert equal_decision.armed

    stale = gate.SignalEvidence(equal.state,
        now - ttl - 1e-6 if which == "source" else now,
        now - ttl - 1e-6 if which == "receipt" else now,
        0, equal.source, equal.policy_sha256, 1, equal.max_linear_mps, equal.max_angular_rps)
    stale_decision = gate.SafetyGateCore().evaluate(
        inputs(now, arm_request=True, **{name: stale}))
    assert_exact_safe_zero(stale_decision)
    assert stale_decision.reason_mask & reason

@pytest.mark.parametrize("delta,armed", [
    (gate.TOPOLOGY_TTL_S - 1e-6, True),
    (gate.TOPOLOGY_TTL_S, True),
    (gate.TOPOLOGY_TTL_S + 1e-6, False),
])
def test_topology_ttl_below_equal_and_above_is_exact(delta, armed):
    now = 100.0
    topology = evidence("topology", now - delta)
    decision = gate.SafetyGateCore().evaluate(
        inputs(now, topology=topology, arm_request=True))
    assert decision.armed is armed
    if not armed:
        assert_exact_safe_zero(decision)
        assert decision.reason_mask & gate.GRAPH_TOPOLOGY


def test_topology_death_stop_and_wrong_source_disarm_after_prior_clear():
    core = gate.SafetyGateCore()
    assert core.evaluate(inputs(100.0, arm_request=True)).armed

    stale = core.evaluate(inputs(
        100.0 + gate.TOPOLOGY_TTL_S + 1e-6,
        topology=evidence("topology", 100.0)))
    assert_exact_safe_zero(stale)
    assert stale.reason_mask & gate.GRAPH_TOPOLOGY

    for invalid in (
        evidence("topology", 101.0, state=gate.STOP),
        replace(evidence("topology", 101.0), source="localization_adapter"),
    ):
        decision = gate.SafetyGateCore().evaluate(inputs(101.0, topology=invalid))
        assert_exact_safe_zero(decision)
        assert decision.reason_mask & gate.GRAPH_TOPOLOGY


def test_topology_ttl_cannot_be_configured():
    assert gate.SafetyConfig().topology_ttl_s == gate.TOPOLOGY_TTL_S
    with pytest.raises(TypeError):
        gate.SafetyConfig(topology_ttl_s=9.0)



@pytest.mark.parametrize("command", [
    gate.VelocityCommand(float("nan"), 0.0),
    gate.VelocityCommand(float("inf"), 0.0),
    gate.VelocityCommand(0.1, 0.0, linear_y=1e-12),
    gate.VelocityCommand(0.1, 0.0, angular_x=-0.1),
])
def test_nonfinite_or_unused_twist_axis_never_passes(command):
    decision = gate.SafetyGateCore().evaluate(inputs(cmd=command, arm_request=True))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.INVALID_CMD


def test_reserved_reason_clear_and_malformed_cap_are_faults():
    reserved = evidence("collision", reason=1 << 50)
    decision = gate.SafetyGateCore().evaluate(inputs(collision=reserved, arm_request=True))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.INTERNAL_FAULT

    malformed = evidence("motion_intent", max_linear_mps=float("nan"), max_angular_rps=0.4)
    decision = gate.SafetyGateCore().evaluate(inputs(motion_intent=malformed, arm_request=True))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.CORRUPT_DATA


def test_future_regressing_and_policy_mismatch_disarm():
    future = evidence("geofence", 100.051)
    decision = gate.SafetyGateCore().evaluate(inputs(100.0, geofence=future, arm_request=True))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.CLOCK

    core = gate.SafetyGateCore()
    assert core.evaluate(inputs(100.0, arm_request=True)).armed
    regressed = core.evaluate(inputs(99.9))
    assert_exact_safe_zero(regressed)
    assert regressed.reason_mask & gate.CLOCK

    expected = "a" * 64
    hashes = dict(POLICIES)
    hashes["collision"] = expected
    cfg = gate.SafetyConfig(expected_policy_sha256=hashes)
    mismatch = evidence("collision", policy="b" * 64)
    decision = gate.SafetyGateCore(cfg).evaluate(inputs(collision=mismatch, arm_request=True))
    assert_exact_safe_zero(decision)
    assert decision.reason_mask & gate.POLICY_MISMATCH


@pytest.mark.parametrize("hashes", [
    {},
    {"collision": "a" * 64},
    dict(POLICIES, unknown="a" * 64),
])
def test_missing_or_unknown_expected_policy_identities_fail_construction(hashes):
    with pytest.raises(ValueError, match="exact keys"):
        gate.SafetyConfig(expected_policy_sha256=hashes)


def test_estop_reset_negative_matrix_and_separate_arm():
    for change in (
        {"e_stop": True}, {"stationary": False}, {"manual_or_disarmed": False},
        {"mission_cancelled": False}, {"topology": None}, {"mode": None},
    ):
        core = gate.SafetyGateCore()
        assert core.evaluate(inputs(arm_request=True)).armed
        core.evaluate(inputs(e_stop=True))
        reset_values = {"e_stop": False, "e_stop_reset": True}
        reset_values.update(change)
        decision = core.evaluate(inputs(**reset_values))
        assert_exact_safe_zero(decision)
        assert decision.e_stop_latched
        assert decision.reason_mask & gate.RESET_REJECTED

    core = gate.SafetyGateCore()
    assert core.evaluate(inputs(arm_request=True)).armed
    core.evaluate(inputs(e_stop=True))
    reset = core.evaluate(inputs(e_stop=False, e_stop_reset=True))
    assert_exact_safe_zero(reset)
    assert not reset.e_stop_latched
    assert reset.reason_mask & gate.STARTUP
    assert core.evaluate(inputs(arm_request=True)).armed


def test_deadline_backpressure_and_internal_fault_stop_on_next_publication_budget():
    cfg = gate.SafetyConfig(publication_period_s=0.02, deadline_limit_s=0.05)
    assert cfg.publication_period_s <= 0.02
    assert cfg.deadline_limit_s <= 0.05
    for fault, reason in (("deadline_missed", gate.DEADLINE_MISS),
                          ("backpressure", gate.BACKPRESSURE),
                          ("internal_fault", gate.INTERNAL_FAULT)):
        core = gate.SafetyGateCore(cfg)
        assert core.evaluate(inputs(arm_request=True)).armed
        decision = core.evaluate(inputs(now=100.02, **{fault: True}))
        assert_exact_safe_zero(decision)
        assert decision.reason_mask & reason


def test_fresh_latest_only_sequence_gap_is_counted_without_false_stop():
    core = gate.SafetyGateCore()
    first = inputs(100.0, arm_request=True)
    assert core.evaluate(first).armed

    jumped = replace(
        first,
        now_s=100.05,
        cmd_source_stamp_s=100.05,
        cmd_receipt_stamp_s=100.05,
        motion_intent=replace(
            first.motion_intent,
            sequence=3,
            source_stamp_s=100.05,
            receipt_stamp_s=100.05,
        ),
        geofence=replace(
            first.geofence, source_stamp_s=100.05, receipt_stamp_s=100.05,
        ),
        collision=replace(
            first.collision, source_stamp_s=100.05, receipt_stamp_s=100.05,
        ),
        slope=replace(
            first.slope, source_stamp_s=100.05, receipt_stamp_s=100.05,
        ),
        localization=replace(
            first.localization, source_stamp_s=100.05, receipt_stamp_s=100.05,
        ),
        mode=replace(
            first.mode, source_stamp_s=100.05, receipt_stamp_s=100.05,
        ),
        driver=replace(
            first.driver, source_stamp_s=100.05, receipt_stamp_s=100.05,
        ),
        topology=replace(
            first.topology, sequence=2, source_stamp_s=100.05, receipt_stamp_s=100.05,
        ),
        arm_request=False,
    )
    decision = core.evaluate(jumped)
    assert decision.armed
    assert decision.reason_mask == 0
    assert decision.dropped_input_count == 1


def test_hard_caps_cannot_be_configured_wider_and_stop_has_no_comfort_ramp():
    with pytest.raises(ValueError):
        gate.SafetyConfig(max_linear_speed=0.551)
    with pytest.raises(ValueError):
        gate.SafetyConfig(max_angular_speed=0.851)
    core = gate.SafetyGateCore()
    moving = core.evaluate(inputs(cmd=gate.VelocityCommand(0.5, 0.7), arm_request=True))
    assert moving.armed
    stopped = core.evaluate(inputs(now=100.02, collision=evidence(
        "collision", 100.02, gate.STOP, gate.COLLISION_DISTANCE)))
    assert_exact_safe_zero(stopped)


PAIR_HASH = "a" * 64


def pair_status(sequence=7, clear=True, source_stamp=100.0, evaluation_stamp=100.0,
                receipt=100.0, reason=0, source="canonical", policy=PAIR_HASH,
                max_linear_mps=None, max_angular_rps=None):
    return gate.StructuredEvidence(
        sequence, clear, source_stamp, evaluation_stamp, receipt, reason, source,
        policy, max_linear_mps, max_angular_rps)


def pair_signal(sequence=7, state=gate.CLEAR, stamp=100.0, receipt=100.0,
                reason=0, source="canonical", policy=PAIR_HASH):
    return gate.GenericEvidence(
        sequence, state, stamp, receipt, reason, source, policy)


def pair_buffer(names=("permission",), stamp_semantics="evaluation"):
    return gate.EvidencePairBuffer(names, 0.25, 0.05, stamp_semantics)

def pending_arm_node(pair, pending_wall=10.0):
    node = object.__new__(gate.SafetyGateRosNode)
    node.pairs = {"permission": pair}
    node.arm_pending_wall = pending_wall
    return node


def consume_pending_arm(node, now=100.0, wall_now=10.01):
    snapshots = {
        name: pair.evidence_and_pending_arm_state(now)
        for name, pair in node.pairs.items()
    }
    return node._consume_pending_arm_request(wall_now, snapshots)


@pytest.mark.parametrize("status_first", [True, False])
def test_pending_arm_waits_for_benign_pair_join_in_both_arrival_orders(status_first):
    pair = pair_buffer()
    status, signal = pair_status(), pair_signal()
    first, second = ((pair.update_status, status),
                     (lambda value: pair.update_signal("permission", value), signal))
    if not status_first:
        first, second = second, first

    node = pending_arm_node(pair)
    first[0](first[1])
    assert not consume_pending_arm(node)
    assert node.arm_pending_wall == 10.0

    second[0](second[1])
    assert consume_pending_arm(node)
    assert node.arm_pending_wall is None
    assert not consume_pending_arm(node)


@pytest.mark.parametrize("status_first", [True, False])
def test_pending_arm_ignores_prior_stop_during_new_clear_pair_join(status_first):
    pair = pair_buffer()
    pair.update_status(pair_status(clear=False))
    pair.update_signal("permission", pair_signal(state=gate.STOP))
    assert pair.evidence(100.0).state == gate.STOP

    status = pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05)
    signal = pair_signal(sequence=8, stamp=100.05, receipt=100.05)
    first, second = ((pair.update_status, status),
                     (lambda value: pair.update_signal("permission", value), signal))
    if not status_first:
        first, second = second, first

    node = pending_arm_node(pair)
    first[0](first[1])
    assert pair.evidence(100.05).state == gate.UNKNOWN
    assert not consume_pending_arm(node, now=100.05)
    assert node.arm_pending_wall == 10.0

    second[0](second[1])
    assert consume_pending_arm(node, now=100.05)
    assert node.arm_pending_wall is None
    assert not consume_pending_arm(node, now=100.05)


@pytest.mark.parametrize("status_first", [True, False])
def test_pending_arm_drops_for_current_generation_restrictive_half(status_first):
    pair = pair_buffer()
    status = pair_status(
        sequence=8, clear=not status_first, source_stamp=100.05,
        evaluation_stamp=100.05, receipt=100.05)
    signal = pair_signal(
        sequence=8, state=gate.CLEAR if status_first else gate.STOP,
        stamp=100.05, receipt=100.05)
    update = ((pair.update_status, status) if status_first else
              (lambda value: pair.update_signal("permission", value), signal))

    node = pending_arm_node(pair)
    update[0](update[1])
    assert not consume_pending_arm(node, now=100.05)
    assert node.arm_pending_wall is None

def test_pending_arm_drops_for_restrictive_partial_or_poisoned_pair():
    restrictive = pair_buffer()
    restrictive.update_status(pair_status(clear=False))
    node = pending_arm_node(restrictive)
    assert not consume_pending_arm(node)
    assert node.arm_pending_wall is None

    poisoned = pair_buffer()
    poisoned.update_status(pair_status())
    poisoned.update_status(pair_status(clear=False))
    node = pending_arm_node(poisoned)
    assert not consume_pending_arm(node)
    assert node.arm_pending_wall is None


def test_pending_arm_drops_after_monotonic_bound_expires():
    node = pending_arm_node(pair_buffer())
    assert not consume_pending_arm(
        node, wall_now=10.0 + gate.ARM_PENDING_TTL_S + 0.001)
    assert node.arm_pending_wall is None


@pytest.mark.parametrize("status_first", [True, False])
def test_pair_reconciles_only_complete_matching_sequence_in_both_arrival_orders(status_first):
    pair = pair_buffer()
    status, signal = pair_status(), pair_signal()
    updates = ((pair.update_status, status),
               (lambda value: pair.update_signal("permission", value), signal))
    if not status_first:
        updates = tuple(reversed(updates))
    updates[0][0](updates[0][1])
    assert pair.evidence(100.0).state == gate.UNKNOWN
    updates[1][0](updates[1][1])
    result = pair.evidence(100.0)
    assert result.state == gate.CLEAR
    assert result.sequence == 7
    assert result.source_stamp_s == 100.0

@pytest.mark.parametrize("status_first", [True, False])
def test_callback_commits_complete_clear_before_next_evidence_tick(status_first):
    pair = pair_buffer()
    status, signal = pair_status(), pair_signal()
    updates = ((pair.update_status, status),
               (lambda value: pair.update_signal("permission", value), signal))
    if not status_first:
        updates = tuple(reversed(updates))

    updates[0][0](updates[0][1])
    updates[1][0](updates[1][1])
    committed = pair.diagnostic_snapshot()
    assert committed["committed_status"] == 7
    assert not committed["hold_latched"]

    pair.update_status(pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05))
    held = pair.evidence(100.05)
    assert held.state == gate.CLEAR
    assert held.sequence == 7
    assert not pair.diagnostic_snapshot()["hold_latched"]


def test_callback_commits_two_signal_clear_and_holds_during_newer_partial():
    pair = pair_buffer(("mode", "driver"), "source")
    pair.update_status(pair_status())
    pair.update_signal("driver", pair_signal())
    pair.update_signal("mode", pair_signal())
    assert pair.diagnostic_snapshot()["committed_status"] == 7

    pair.update_signal("mode", pair_signal(
        sequence=8, stamp=100.05, receipt=100.05))
    held = pair.evidence(100.05)
    assert held.state == gate.CLEAR
    assert held.sequence == 7
    assert not pair.diagnostic_snapshot()["hold_latched"]


def test_callback_commit_latches_overwritten_incomplete_generation_until_exact_join():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.diagnostic_snapshot()["committed_status"] == 7

    pair.update_status(pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05))
    pair.update_signal("permission", pair_signal(
        sequence=9, stamp=100.06, receipt=100.06))
    snapshot = pair.diagnostic_snapshot()
    assert snapshot["committed_status"] == 7
    assert snapshot["hold_latched"]
    assert pair.evidence(100.06).state == gate.UNKNOWN

    pair.update_status(pair_status(
        sequence=9, source_stamp=100.06, evaluation_stamp=100.06, receipt=100.06))
    joined = pair.evidence(100.06)
    assert joined.state == gate.CLEAR
    assert joined.sequence == 9
    assert not pair.diagnostic_snapshot()["hold_latched"]


def test_callback_does_not_hide_complete_restrictive_generation_with_newer_partial():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.diagnostic_snapshot()["committed_status"] == 7

    pair.update_status(pair_status(
        sequence=8, clear=False, reason=gate.MODE, source_stamp=100.05,
        evaluation_stamp=100.05, receipt=100.05))
    pair.update_signal("permission", pair_signal(
        sequence=8, state=gate.STOP, reason=gate.MODE, stamp=100.05, receipt=100.05))
    assert pair.diagnostic_snapshot()["committed_status"] == 7

    pair.update_status(pair_status(
        sequence=9, source_stamp=100.06, evaluation_stamp=100.06, receipt=100.06))
    assert pair.diagnostic_snapshot()["hold_latched"]
    assert pair.evidence(100.06).state == gate.UNKNOWN

    pair.update_signal("permission", pair_signal(
        sequence=9, stamp=100.06, receipt=100.06))
    assert pair.evidence(100.06).sequence == 9


@pytest.mark.parametrize(
    "status,signal",
    [
        (pair_status(policy="not-a-hash"), pair_signal(policy="not-a-hash")),
        (pair_status(), pair_signal(source="other")),
        (pair_status(clear=1), pair_signal()),
        (pair_status(reason=0.0), pair_signal()),
        (pair_status(), pair_signal(state=True)),
        (pair_status(), pair_signal(reason=0.0)),
        (pair_status(source_stamp=99.74, evaluation_stamp=99.74, receipt=100.0),
         pair_signal(stamp=99.74, receipt=100.0)),
        (pair_status(source_stamp=100.051, evaluation_stamp=100.051, receipt=100.0),
         pair_signal(stamp=100.051, receipt=100.0)),
    ],
)
def test_callback_does_not_commit_nonpermissive_or_invalid_complete_candidates(status, signal):
    pair = pair_buffer()
    pair.update_status(status)
    pair.update_signal("permission", signal)
    assert pair.diagnostic_snapshot()["committed_status"] is None
    assert pair.evidence(100.0).state == gate.UNKNOWN


def test_callback_conflicting_duplicate_poison_clears_committed_authority():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.diagnostic_snapshot()["committed_status"] == 7

    pair.update_signal("permission", pair_signal(state=gate.STOP))
    snapshot = pair.diagnostic_snapshot()
    assert snapshot["poisoned"]
    assert snapshot["committed_status"] is None


def test_callback_commits_exact_tighter_cap_and_partial_tighter_cap_revokes():
    pair = pair_buffer()
    pair.update_status(pair_status(max_linear_mps=0.5))
    pair.update_signal("permission", pair_signal())
    pair.update_status(pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05,
        max_linear_mps=0.2))
    pair.update_signal("permission", pair_signal(
        sequence=8, stamp=100.05, receipt=100.05))
    assert pair.diagnostic_snapshot()["committed_status"] == 8

    pair.update_status(pair_status(
        sequence=9, source_stamp=100.06, evaluation_stamp=100.06, receipt=100.06))
    held = pair.evidence(100.06)
    assert held.sequence == 8
    assert held.max_linear_mps == 0.2

    revoking = pair_buffer()
    revoking.update_status(pair_status(max_linear_mps=0.2))
    revoking.update_signal("permission", pair_signal())
    revoking.update_status(pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05,
        max_linear_mps=0.1))
    assert revoking.evidence(100.05).state == gate.UNKNOWN


def test_callback_committed_clear_is_revalidated_after_its_ttl():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.diagnostic_snapshot()["committed_status"] == 7

    assert pair.evidence(100.251).state == gate.UNKNOWN


def test_pending_arm_waits_for_newer_partial_and_completes_only_in_timer_snapshot():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    node = pending_arm_node(pair)

    pair.update_status(pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05))
    assert not consume_pending_arm(node, now=100.05)
    assert node.arm_pending_wall == 10.0

    pair.update_signal("permission", pair_signal(
        sequence=8, stamp=100.05, receipt=100.05))
    assert pair.diagnostic_snapshot()["committed_status"] == 8
    assert node.arm_pending_wall == 10.0
    assert consume_pending_arm(node, now=100.05)
    assert node.arm_pending_wall is None


@pytest.mark.parametrize("status_first", [True, False])
def test_benign_newer_partial_generation_holds_committed_clear(status_first):
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).sequence == 7

    status = pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05)
    signal = pair_signal(sequence=8, stamp=100.05, receipt=100.05)
    updates = ((pair.update_status, status),
               (lambda value: pair.update_signal("permission", value), signal))
    if not status_first:
        updates = tuple(reversed(updates))
    updates[0][0](updates[0][1])
    held = pair.evidence(100.05)
    assert held.state == gate.CLEAR
    assert held.sequence == 7

    updates[1][0](updates[1][1])
    joined = pair.evidence(100.05)
    assert joined.state == gate.CLEAR
    assert joined.sequence == 8


@pytest.mark.parametrize(
    "update",
    [
        lambda pair: pair.update_status(pair_status(
            sequence=8, clear=False, source_stamp=100.05,
            evaluation_stamp=100.05, receipt=100.05)),
        lambda pair: pair.update_signal("permission", pair_signal(
            sequence=8, state=gate.STOP, stamp=100.05, receipt=100.05)),
    ],
)
def test_restrictive_newer_partial_generation_revokes_committed_clear(update):
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR

    update(pair)
    result = pair.evidence(100.05)
    assert result.state == gate.UNKNOWN
    assert result.reason_mask & gate.CORRUPT_DATA
    assert result.reason_mask & gate.INPUT_UNKNOWN


@pytest.mark.parametrize(
    "update",
    [
        lambda pair: pair.update_status(pair_status(
            sequence=8, source="other", source_stamp=100.05,
            evaluation_stamp=100.05, receipt=100.05)),
        lambda pair: pair.update_signal("permission", pair_signal(
            sequence=8, policy="b" * 64, stamp=100.05, receipt=100.05)),
    ],
)
def test_inconsistent_newer_partial_generation_revokes_committed_clear(update):
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR

    update(pair)
    assert pair.evidence(100.05).state == gate.UNKNOWN
@pytest.mark.parametrize(
    "committed,status",
    [
        (pair_status(max_linear_mps=0.2),
         pair_status(sequence=8, source_stamp=100.05, evaluation_stamp=100.05,
                     receipt=100.05, max_linear_mps=0.1)),
        (pair_status(),
         pair_status(sequence=8, source_stamp=100.05, evaluation_stamp=100.05,
                     receipt=100.05, max_angular_rps=0.1)),
        (pair_status(max_angular_rps=0.2),
         pair_status(sequence=8, source_stamp=100.05, evaluation_stamp=100.05,
                     receipt=100.05, max_angular_rps=0.1)),
        (pair_status(max_linear_mps=0.2),
         pair_status(sequence=8, source_stamp=100.05, evaluation_stamp=100.05,
                     receipt=100.05, max_linear_mps=0.0)),
    ],
)
def test_tighter_partial_cap_revokes_committed_clear(committed, status):
    pair = pair_buffer()
    pair.update_status(committed)
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR

    pair.update_status(status)
    assert pair.evidence(100.05).state == gate.UNKNOWN


def test_gap_or_overwrite_latches_hold_until_exact_pair_joins():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR

    pair.update_status(pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05))
    assert pair.evidence(100.05).state == gate.CLEAR
    pair.update_signal("permission", pair_signal(sequence=9, stamp=100.06, receipt=100.06))
    assert pair.evidence(100.06).state == gate.UNKNOWN

    pair.update_status(pair_status(
        sequence=9, source_stamp=100.06, evaluation_stamp=100.06, receipt=100.06))
    assert pair.evidence(100.06).state == gate.CLEAR


def test_two_signal_partial_overwrite_latches_hold_until_exact_pair_joins():
    pair = pair_buffer(("mode", "driver"), "source")
    pair.update_status(pair_status())
    pair.update_signal("mode", pair_signal())
    pair.update_signal("driver", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR

    pair.update_status(pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05))
    pair.update_signal("mode", pair_signal(sequence=9, stamp=100.06, receipt=100.06))
    assert pair.evidence(100.06).state == gate.UNKNOWN

    pair.update_status(pair_status(
        sequence=9, source_stamp=100.06, evaluation_stamp=100.06, receipt=100.06))
    pair.update_signal("driver", pair_signal(sequence=9, stamp=100.06, receipt=100.06))
    assert pair.evidence(100.06).state == gate.CLEAR


@pytest.mark.parametrize("state,reason", [(True, 0), (gate.CLEAR, True),
                                          (1.0, 0), (gate.CLEAR, 0.0)])
def test_bool_or_float_generic_fields_revoke_partial_and_complete(state, reason):
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR

    pair.update_signal("permission", pair_signal(
        sequence=8, state=state, reason=reason, stamp=100.05, receipt=100.05))
    assert pair.evidence(100.05).state == gate.UNKNOWN
    complete = pair_buffer()
    complete.update_status(pair_status())
    complete.update_signal("permission", pair_signal())
    complete.update_status(pair_status(
        sequence=8, source_stamp=100.05, evaluation_stamp=100.05, receipt=100.05))
    complete.update_signal("permission", pair_signal(
        sequence=8, state=state, reason=reason, stamp=100.05, receipt=100.05))
    assert complete.evidence(100.05).state == gate.UNKNOWN
@pytest.mark.parametrize("reason", [True, 0.0])
def test_bool_or_float_structured_reason_revokes_partial_and_complete(reason):
    partial = pair_buffer()
    partial.update_status(pair_status())
    partial.update_signal("permission", pair_signal())
    assert partial.evidence(100.0).state == gate.CLEAR

    partial.update_status(pair_status(
        sequence=8, reason=reason, source_stamp=100.05,
        evaluation_stamp=100.05, receipt=100.05))
    assert partial.evidence(100.05).state == gate.UNKNOWN

    complete = pair_buffer()
    complete.update_status(pair_status())
    complete.update_signal("permission", pair_signal())
    complete.update_status(pair_status(
        sequence=8, reason=reason, source_stamp=100.05,
        evaluation_stamp=100.05, receipt=100.05))
    complete.update_signal("permission", pair_signal(
        sequence=8, stamp=100.05, receipt=100.05))
    assert complete.evidence(100.05).state == gate.UNKNOWN
def test_pair_snapshot_keeps_pending_arm_and_evidence_atomic_against_stop():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    entered = threading.Event()
    release = threading.Event()
    original = pair._pending_arm_state_unlocked

    def delayed_pending(now, evidence):
        entered.set()
        assert release.wait(1.0)
        return original(now, evidence)

    pair._pending_arm_state_unlocked = delayed_pending
    snapshot = []
    reader = threading.Thread(
        target=lambda: snapshot.append(pair.evidence_and_pending_arm_state(100.0)))
    writer = threading.Thread(target=lambda: pair.update_status(pair_status(
        sequence=8, clear=False, source_stamp=100.05,
        evaluation_stamp=100.05, receipt=100.05)))
    reader.start()
    assert entered.wait(1.0)
    writer.start()
    time.sleep(0.01)
    assert writer.is_alive()
    release.set()
    reader.join(1.0)
    writer.join(1.0)

    assert snapshot == [(gate.SignalEvidence(
        gate.CLEAR, 100.0, 100.0, 0, "canonical", PAIR_HASH, 7, None, None),
        "complete")]
    assert pair.evidence(100.05).state == gate.UNKNOWN

def test_pair_updates_and_reads_are_serialized_across_ros_callback_threads():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).sequence == 7

    entered = threading.Event()
    release = threading.Event()
    original_accept = pair._accept_sequence

    def delayed_accept(stream, value):
        result = original_accept(stream, value)
        if stream == "status":
            entered.set()
            assert release.wait(1.0)
        return result

    pair._accept_sequence = delayed_accept
    writer = threading.Thread(target=pair.update_status, args=(
        pair_status(
            sequence=8,
            source_stamp=100.05,
            evaluation_stamp=100.05,
            receipt=100.05,
        ),
    ))
    observed = []
    reader = threading.Thread(
        target=lambda: observed.append(pair.evidence(100.05)),
    )
    writer.start()
    assert entered.wait(1.0)
    reader.start()
    time.sleep(0.01)
    assert reader.is_alive()
    release.set()
    writer.join(1.0)
    reader.join(1.0)

    assert not writer.is_alive()
    assert not reader.is_alive()
    assert observed[0].state == gate.CLEAR
    assert observed[0].sequence == 7


def test_incomplete_generation_cannot_outlive_committed_pair_ttl():
    pair = pair_buffer(("mode", "driver"), "source")
    pair.update_status(pair_status())
    pair.update_signal("mode", pair_signal())
    pair.update_signal("driver", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR

    pair.update_status(pair_status(
        sequence=8,
        source_stamp=100.10,
        evaluation_stamp=100.10,
        receipt=100.10,
    ))
    expired = pair.evidence(100.251)
    assert expired.state == gate.UNKNOWN
    assert expired.reason_mask & gate.CORRUPT_DATA
    assert expired.reason_mask & gate.INPUT_UNKNOWN

@pytest.mark.parametrize(
    "changed",
    [
        lambda signal: replace(signal, policy_sha256="b" * 64),
        lambda signal: replace(signal, source="other"),
        lambda signal: replace(signal, reason_mask=gate.COLLISION),
        lambda signal: replace(signal, state=gate.STOP),
        lambda signal: replace(signal, stamp_s=99.99),
        lambda signal: replace(signal, sequence=8),
    ],
)
def test_pair_contract_mismatch_is_corrupt_unknown(changed):
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", changed(pair_signal()))
    result = pair.evidence(100.0)
    assert result.state == gate.UNKNOWN
    assert result.reason_mask & gate.CORRUPT_DATA
    assert result.reason_mask & gate.INPUT_UNKNOWN


def test_localization_pair_uses_evaluation_stamp_not_source_stamp():
    pair = pair_buffer(stamp_semantics="evaluation")
    pair.update_status(pair_status(source_stamp=99.9, evaluation_stamp=100.0))
    pair.update_signal("permission", pair_signal(stamp=100.0))
    assert pair.evidence(100.0).state == gate.CLEAR


@pytest.mark.parametrize(
    "status,signal,now",
    [
        (pair_status(source_stamp=99.74), pair_signal(), 100.0),
        (pair_status(receipt=99.74), pair_signal(receipt=99.74), 100.0),
        (pair_status(source_stamp=100.051), pair_signal(), 100.0),
        (pair_status(receipt=100.051), pair_signal(receipt=100.051), 100.0),
        (pair_status(evaluation_stamp=99.74), pair_signal(stamp=99.74), 100.0),
        (pair_status(evaluation_stamp=100.051), pair_signal(stamp=100.051), 100.0),
        (pair_status(evaluation_stamp=float("nan")), pair_signal(stamp=float("nan")), 100.0),
        (pair_status(source_stamp=float("inf")), pair_signal(), 100.0),
        (pair_status(reason=1 << 50), pair_signal(reason=1 << 50), 100.0),
        (pair_status(policy="not-a-hash"), pair_signal(policy="not-a-hash"), 100.0),
    ],
)
def test_stale_future_and_malformed_pairs_are_unknown(status, signal, now):
    pair = pair_buffer()
    pair.update_status(status)
    pair.update_signal("permission", signal)
    result = pair.evidence(now)
    assert result.state == gate.UNKNOWN
    assert result.reason_mask & gate.CORRUPT_DATA
    assert result.reason_mask & gate.INPUT_UNKNOWN
    recovery = max(
        timestamp for timestamp in (
            status.source_stamp_s, status.evaluation_stamp_s, status.receipt_stamp_s,
            signal.stamp_s, signal.receipt_stamp_s, now,
        ) if gate._finite(timestamp)
    ) + 0.01
    pair.update_status(pair_status(
        sequence=8, source_stamp=recovery, evaluation_stamp=recovery,
        receipt=recovery))
    pair.update_signal("permission", pair_signal(
        sequence=8, stamp=recovery, receipt=recovery))
    assert pair.evidence(recovery).state == gate.CLEAR


@pytest.mark.parametrize(
    "status,signal",
    [
        (pair_status(sequence=8, source_stamp=99.99, evaluation_stamp=100.1,
                     receipt=100.1),
         pair_signal(sequence=8, stamp=100.1, receipt=100.1)),
        (pair_status(sequence=8, source_stamp=100.1, evaluation_stamp=100.1,
                     receipt=99.99),
         pair_signal(sequence=8, stamp=100.1, receipt=100.1)),
        (pair_status(sequence=8, source_stamp=100.1, evaluation_stamp=100.1,
                     receipt=100.1),
         pair_signal(sequence=8, stamp=100.1, receipt=99.99)),
    ],
)
def test_pair_independent_timestamp_regressions_poison_authority(status, signal):
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR
    pair.update_status(status)
    pair.update_signal("permission", signal)
    result = pair.evidence(100.1)
    assert result.state == gate.UNKNOWN
    assert result.reason_mask & gate.CORRUPT_DATA


def test_regression_or_conflicting_duplicate_poison_until_fresh_complete_higher_pair():
    pair = pair_buffer()
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).state == gate.CLEAR

    pair.update_signal("permission", pair_signal(sequence=6))
    assert pair.evidence(100.0).state == gate.UNKNOWN
    pair.update_status(pair_status())
    pair.update_signal("permission", pair_signal())
    assert pair.evidence(100.0).state == gate.UNKNOWN

    pair.update_status(pair_status(sequence=8))
    pair.update_signal("permission", pair_signal(sequence=8))
    assert pair.evidence(100.0).state == gate.CLEAR
    pair.update_signal("permission", pair_signal(sequence=8, state=gate.STOP))
    assert pair.evidence(100.0).state == gate.UNKNOWN
    pair.update_status(pair_status(sequence=9))
    pair.update_signal("permission", pair_signal(sequence=9))
    assert pair.evidence(100.0).state == gate.CLEAR


def test_driver_requires_mode_and_driver_signals_to_match_same_status():
    pair = pair_buffer(("mode", "driver"), "source")
    status = pair_status()
    pair.update_status(status)
    pair.update_signal("mode", pair_signal())
    assert pair.evidence(100.0).state == gate.UNKNOWN
    pair.update_signal("driver", pair_signal(reason=gate.DRIVER))
    assert pair.evidence(100.0).state == gate.UNKNOWN

    pair.update_status(pair_status(sequence=8))
    pair.update_signal("mode", pair_signal(sequence=8))
    pair.update_signal("driver", pair_signal(sequence=8))
    assert pair.evidence(100.0).state == gate.CLEAR


def test_generic_signal_alone_never_grants_permission():
    pair = pair_buffer()
    pair.update_signal("permission", pair_signal())
    result = pair.evidence(100.0)
    assert result.state == gate.UNKNOWN
    assert result.reason_mask & gate.INPUT_UNKNOWN

def test_timer_publishes_command_before_one_matching_state_snapshot():
    source = inspect.getsource(gate.SafetyGateRosNode._timer_cb)
    assert source.count("self.pub.publish(") == 1
    assert source.count("self.state_pub.publish(") == 1
    assert source.index("self.pub.publish(") < source.index("self.state_pub.publish(")
    assert "self.sequence += 1" in source
    assert "evidence_and_pending_arm_state(now)" in source
    assert "_consume_pending_arm_request(wall_now, snapshots)" in source
    assert "self._build_safety_state(self._observability_snapshot)" in source


def test_observability_publishes_diagnostics_without_republishing_state():
    source = inspect.getsource(gate.SafetyGateRosNode._publish_observability)
    assert "self.diag_pub.publish(diag)" in source
    assert "self.state_pub.publish(" not in source
    assert "self._build_safety_state(" not in source


def test_safety_state_snapshot_preserves_all_state_fields(monkeypatch):
    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=None)
            self.angular = types.SimpleNamespace(z=None)

    class SafetyState:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id=None)

    rospy = types.ModuleType("rospy")
    rospy.Time = types.SimpleNamespace(from_sec=lambda value: ("stamp", value))
    geometry = types.ModuleType("geometry_msgs.msg")
    geometry.Twist = Twist
    interfaces = types.ModuleType("wheelchair_interfaces.msg")
    interfaces.SafetyState = SafetyState
    monkeypatch.setitem(sys.modules, "rospy", rospy)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry)
    monkeypatch.setitem(sys.modules, "wheelchair_interfaces.msg", interfaces)

    node = object.__new__(gate.SafetyGateRosNode)
    node.cfg = types.SimpleNamespace(release_manifest_sha256="a" * 64)
    decision = gate.GateDecision(
        gate.VelocityCommand(0.2, -0.1), "nominal", True, reason_mask=7,
        state=gate.STATE_CLEAR, armed=True,
        ages={"command": 0.01, "motion_intent": 0.02, "geofence": 0.03,
              "collision": 0.04, "localization": 0.05, "slope": 0.06,
              "mode": 0.07, "driver": 0.08},
        deadline_miss_count=3, dropped_input_count=4)
    state = node._build_safety_state((
        42.5, 9, decision, gate.VelocityCommand(0.4, 0.3), {"sequence": 1}))

    assert (state.header.stamp, state.header.frame_id, state.sequence, state.state,
            state.reason_mask, state.armed, state.estop_latched) == (
                ("stamp", 42.5), "base_footprint", 9, gate.STATE_CLEAR, 7, True, True)
    assert (state.requested_command.linear.x, state.requested_command.angular.z,
            state.output_command.linear.x, state.output_command.angular.z) == (
                0.4, 0.3, 0.2, -0.1)
    assert tuple(getattr(state, name) for name in (
        "command_age_s", "intent_age_s", "geofence_age_s", "collision_age_s",
        "localization_age_s", "slope_age_s", "mode_age_s", "driver_age_s")) == (
            0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08)
    assert (state.deadline_miss_count, state.dropped_input_count,
            state.release_manifest_sha256) == (3, 4, "a" * 64)


def test_timer_serializes_sequence_and_snapshot_publication():
    source = inspect.getsource(gate.SafetyGateRosNode._timer_cb)
    lock_start = source.index("with self._observability_lock:")
    sequence = source.index("self.sequence += 1")
    snapshot = source.index("self._observability_snapshot =")
    state = source.index("self.state_pub.publish(state)")
    assert lock_start < sequence < snapshot < state
