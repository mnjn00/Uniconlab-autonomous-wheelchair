from pathlib import Path
from dataclasses import replace
import importlib.util
import math
import threading
import time
import pytest

ROOT = Path(__file__).resolve().parents[3]
MODULE = ROOT / "src" / "wheelchair_safety" / "scripts" / "safety_gate.py"
spec = importlib.util.spec_from_file_location("safety_gate_hardened", MODULE)
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)

POLICIES = dict(gate._DEFAULT_POLICY_SHA256)


def evidence(name, stamp=100.0, state=gate.CLEAR, reason=0, policy=None, **caps):
    if policy is None:
        policy = "" if name == "motion_intent" else POLICIES[name]
    source = "topology_guard" if name == "topology" else name
    return gate.SignalEvidence(state, stamp, stamp, reason, source, policy, 1, **caps)


def inputs(now=100.0, **changes):
    values = dict(
        cmd=gate.VelocityCommand(0.2, 0.1), now_s=now,
        cmd_source_stamp_s=now, cmd_receipt_stamp_s=now,
        motion_intent=evidence("motion_intent", now, max_linear_mps=0.5,
                               max_angular_rps=0.8),
        geofence=evidence("geofence", now), collision=evidence("collision", now),
        slope=evidence("slope", now), localization=evidence("localization", now),
        mode=evidence("mode", now), driver=evidence("driver", now),
        topology=evidence("topology", now),
        e_stop=False, manual_or_disarmed=True, stationary=True,
        mission_cancelled=True,
    )
    values.update(changes)
    return gate.GateInputs(**values)


def assert_exact_safe_zero(decision):
    assert decision.command.values() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert all(math.isfinite(v) for v in decision.command.values())
    assert not decision.armed

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

    transition = core.evaluate(inputs(100.01, cmd=None))
    assert transition.armed
    assert transition.reason == "activation_grace"
    decision = core.evaluate(inputs(100.01 + elapsed, cmd=None))
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

    grace = core.evaluate(inputs(100.01, **old_command))
    assert grace.armed
    assert grace.reason == "activation_grace"
    assert grace.command.is_zero()

    rejected = core.evaluate(inputs(100.110001, **old_command))
    assert_exact_safe_zero(rejected)
    assert rejected.reason_mask & gate.STALE_CMD


def test_hold_transition_resets_command_activation():
    core = gate.SafetyGateCore()
    core.evaluate(inputs(100.0, cmd=None, motion_intent=hold_intent(100.0),
                         arm_request=True))
    core.evaluate(inputs(100.01, cmd=None))
    moving = core.evaluate(inputs(
        100.02, cmd_source_stamp_s=100.02, cmd_receipt_stamp_s=100.02))
    assert moving.armed
    assert not moving.command.is_zero()

    held = core.evaluate(inputs(
        100.03, motion_intent=hold_intent(100.03),
        cmd_source_stamp_s=100.02, cmd_receipt_stamp_s=100.02))
    assert held.armed
    assert held.command.is_zero()
    released = core.evaluate(inputs(
        100.04, cmd_source_stamp_s=100.02, cmd_receipt_stamp_s=100.02))
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
            first.topology, source_stamp_s=100.05, receipt_stamp_s=100.05,
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
                max_linear_mps=None):
    return gate.StructuredEvidence(
        sequence, clear, source_stamp, evaluation_stamp, receipt, reason, source,
        policy, max_linear_mps)


def pair_signal(sequence=7, state=gate.CLEAR, stamp=100.0, receipt=100.0,
                reason=0, source="canonical", policy=PAIR_HASH):
    return gate.GenericEvidence(
        sequence, state, stamp, receipt, reason, source, policy)


def pair_buffer(names=("permission",), stamp_semantics="evaluation"):
    return gate.EvidencePairBuffer(names, 0.25, 0.05, stamp_semantics)


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



def test_complete_pair_remains_fresh_while_next_sequence_is_assembling():
    pair = pair_buffer(("mode", "driver"), "source")
    pair.update_status(pair_status())
    pair.update_signal("mode", pair_signal())
    pair.update_signal("driver", pair_signal())
    assert pair.evidence(100.0).sequence == 7

    pair.update_status(pair_status(
        sequence=8,
        source_stamp=100.05,
        evaluation_stamp=100.05,
        receipt=100.05,
    ))
    first_partial = pair.evidence(100.05)
    assert first_partial.state == gate.CLEAR
    assert first_partial.sequence == 7

    pair.update_signal("mode", pair_signal(
        sequence=8,
        stamp=100.05,
        receipt=100.05,
    ))
    second_partial = pair.evidence(100.05)
    assert second_partial.state == gate.CLEAR
    assert second_partial.sequence == 7

    pair.update_signal("driver", pair_signal(
        sequence=8,
        stamp=100.05,
        receipt=100.05,
    ))
    joined = pair.evidence(100.05)
    assert joined.state == gate.CLEAR
    assert joined.sequence == 8


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
    pair.update_status(pair_status(sequence=8))
    pair.update_signal("permission", pair_signal(sequence=8))
    assert pair.evidence(100.0).state == gate.CLEAR


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
