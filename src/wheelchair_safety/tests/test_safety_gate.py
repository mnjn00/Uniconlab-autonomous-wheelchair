from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[3]
SAFETY_GATE = ROOT / "src" / "wheelchair_safety" / "scripts" / "safety_gate.py"
spec = importlib.util.spec_from_file_location("safety_gate", SAFETY_GATE)
safety_gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(safety_gate)

VelocityCommand = safety_gate.VelocityCommand
SignalEvidence = safety_gate.SignalEvidence
SafetyConfig = safety_gate.SafetyConfig
GateInputs = safety_gate.GateInputs
SafetyGateCore = safety_gate.SafetyGateCore


def clear(name, stamp=10.0, sequence=1, **kwargs):
    policy = "" if name == "intent" else safety_gate._DEFAULT_POLICY_SHA256[name]
    source = "topology_guard" if name == "topology" else name
    return SignalEvidence(state=safety_gate.CLEAR, source_stamp_s=stamp,
                          receipt_stamp_s=stamp, source=source, sequence=sequence,
                          policy_sha256=policy, **kwargs)


def valid_inputs(**changes):
    values = dict(
        cmd=VelocityCommand(0.2, 0.1), now_s=10.0,
        cmd_source_stamp_s=10.0, cmd_receipt_stamp_s=10.0,
        motion_intent=clear("intent", max_linear_mps=0.5, max_angular_rps=0.8),
        geofence=clear("geofence"), collision=clear("collision"),
        slope=clear("slope"), localization=clear("localization"),
        mode=clear("mode"), driver=clear("driver"), topology=clear("topology"),
        e_stop=False,
        manual_or_disarmed=True, stationary=True, mission_cancelled=True,
        graph_valid=True,
        reset_driver_healthy=True,
    )
    values.update(changes)
    return GateInputs(**values)


def arm(core):
    decision = core.evaluate(valid_inputs(arm_request=True))
    assert decision.armed
    return decision


def test_startup_is_unknown_disarmed_and_zero():
    decision = SafetyGateCore().evaluate(GateInputs())
    assert decision.command == VelocityCommand()
    assert not decision.armed
    assert decision.reason_mask & safety_gate.INPUT_UNKNOWN
    assert decision.reason_mask & safety_gate.STARTUP


def test_e_stop_reset_is_guarded_and_never_rearms():
    core = SafetyGateCore()
    arm(core)
    stopped = core.evaluate(valid_inputs(e_stop=True))
    assert stopped.command.is_zero() and stopped.e_stop_latched

    rejected = core.evaluate(valid_inputs(e_stop=False, e_stop_reset=True, stationary=False))
    assert rejected.reason_mask & safety_gate.RESET_REJECTED
    assert rejected.e_stop_latched
    core.evaluate(valid_inputs(e_stop=False, e_stop_reset=False))

    reset = core.evaluate(valid_inputs(e_stop=False, e_stop_reset=True))
    assert reset.command.is_zero()
    assert not reset.e_stop_latched and not reset.armed
    assert reset.reason_mask == safety_gate.STARTUP
    core.evaluate(valid_inputs(e_stop=False, e_stop_reset=False))

    rearmed = core.evaluate(valid_inputs(arm_request=True))
    assert rearmed.armed and rearmed.command == VelocityCommand(0.2, 0.1)


def test_stale_watchdog_stops_at_above_but_not_equal_ttl():
    cfg = SafetyConfig(stale_timeout_s=0.30)
    equal = SafetyGateCore(cfg).evaluate(valid_inputs(
        cmd_source_stamp_s=9.70, cmd_receipt_stamp_s=9.70, arm_request=True))
    assert equal.armed
    old = SafetyGateCore(cfg).evaluate(valid_inputs(
        cmd_source_stamp_s=9.699999, cmd_receipt_stamp_s=9.70, arm_request=True))
    assert old.reason_mask & safety_gate.STALE_CMD
    assert old.command.is_zero() and not old.armed


def test_stop_evidence_cannot_be_masked_by_clear_inputs():
    core = SafetyGateCore()
    collision = SignalEvidence(state=safety_gate.STOP, source_stamp_s=10.0,
                               receipt_stamp_s=10.0, reason_mask=safety_gate.COLLISION_TTC,
                               source="collision", policy_sha256="0" * 64, sequence=1)
    decision = core.evaluate(valid_inputs(collision=collision, arm_request=True))
    assert decision.command.is_zero()
    assert decision.reason_mask & safety_gate.COLLISION_TTC
    assert not decision.armed


def test_speed_cap_is_minimum_of_hard_and_intent_caps():
    core = SafetyGateCore(SafetyConfig(max_linear_speed=0.55, max_angular_speed=0.85))
    intent = clear("intent", max_linear_mps=0.3, max_angular_rps=0.4)
    decision = core.evaluate(valid_inputs(cmd=VelocityCommand(0.8, -1.2),
                                          motion_intent=intent, arm_request=True))
    assert decision.reason == "speed_cap"
    assert decision.command == VelocityCommand(0.3, -0.4)


def test_legacy_boole_do_not_supply_missing_permissions():
    decision = SafetyGateCore().evaluate(GateInputs(
        cmd=VelocityCommand(0.1, 0.0), cmd_age_s=0.0,
        geofence_ok=True, mode_allowed=True, collision_stop=False,
        e_stop=False, graph_valid=True, arm_request=True))
    assert decision.command.is_zero()
    assert decision.reason_mask & safety_gate.INPUT_UNKNOWN


def test_arm_and_reset_levels_are_single_use_edges():
    core = SafetyGateCore()
    assert core.evaluate(valid_inputs(arm_request=True)).armed
    faulted = core.evaluate(valid_inputs(e_stop=True, arm_request=True))
    assert faulted.e_stop_latched and not faulted.armed

    # A held arm level cannot rearm after a later fault or reset.
    reset = core.evaluate(valid_inputs(e_stop=False, e_stop_reset=True, arm_request=True))
    assert not reset.e_stop_latched and not reset.armed
    assert not core.evaluate(valid_inputs(e_stop=False, arm_request=True)).armed

    core.evaluate(valid_inputs(e_stop=False, arm_request=False))
    assert core.evaluate(valid_inputs(e_stop=False, arm_request=True)).armed


def test_manual_driver_reset_evidence_does_not_grant_motion_authority():
    core = SafetyGateCore()
    assert core.evaluate(valid_inputs(arm_request=True)).armed
    core.evaluate(valid_inputs(e_stop=True, arm_request=False))
    manual = SignalEvidence(
        state=safety_gate.STOP, source_stamp_s=10.0, receipt_stamp_s=10.0,
        source="driver", policy_sha256=safety_gate._DEFAULT_POLICY_SHA256["driver"],
        sequence=1)
    mode = SignalEvidence(
        state=safety_gate.STOP, source_stamp_s=10.0, receipt_stamp_s=10.0,
        source="mode", policy_sha256=safety_gate._DEFAULT_POLICY_SHA256["mode"],
        sequence=1)
    reset = core.evaluate(valid_inputs(
        e_stop=False, e_stop_reset=True, arm_request=False, mode=mode, driver=manual))
    assert not reset.e_stop_latched and not reset.armed
    core.evaluate(valid_inputs(e_stop=False, e_stop_reset=False, mode=mode, driver=manual))
    denied = core.evaluate(valid_inputs(e_stop=False, arm_request=True, mode=mode, driver=manual))
    assert not denied.armed
    assert denied.reason_mask & safety_gate.MODE
def test_reset_requires_separate_strict_driver_health():
    core = SafetyGateCore()
    core.evaluate(valid_inputs(arm_request=True))
    core.evaluate(valid_inputs(e_stop=True, arm_request=False))
    denied = core.evaluate(valid_inputs(
        e_stop=False, e_stop_reset=True, arm_request=False,
        reset_driver_healthy=False))
    assert denied.e_stop_latched
    assert denied.reason_mask & safety_gate.RESET_REJECTED


def test_evidence_high_water_marks_never_regress():
    core = SafetyGateCore()
    core.evaluate(valid_inputs(now_s=10.0))
    regressed = core.evaluate(valid_inputs(now_s=10.0, motion_intent=clear(
        "intent", 9.0, sequence=2, max_linear_mps=0.5, max_angular_rps=0.8)))
    assert regressed.reason_mask & safety_gate.CLOCK
    still_regressed = core.evaluate(valid_inputs(now_s=10.0, motion_intent=clear(
        "intent", 9.5, sequence=3, max_linear_mps=0.5, max_angular_rps=0.8)))
    assert still_regressed.reason_mask & safety_gate.CLOCK


def test_changed_same_sequence_is_rejected():
    core = SafetyGateCore()
    core.evaluate(valid_inputs())
    changed = clear("topology", 10.0)
    changed = SignalEvidence(**{**changed.__dict__, "reason_mask": safety_gate.GRAPH_TOPOLOGY})
    decision = core.evaluate(valid_inputs(topology=changed))
    assert decision.reason_mask & safety_gate.CORRUPT_DATA
