from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[3]
SAFETY_GATE = ROOT / "src" / "wheelchair_safety" / "scripts" / "safety_gate.py"

spec = importlib.util.spec_from_file_location("safety_gate", SAFETY_GATE)
safety_gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(safety_gate)

VelocityCommand = safety_gate.VelocityCommand
SafetyConfig = safety_gate.SafetyConfig
GateInputs = safety_gate.GateInputs
SafetyGateCore = safety_gate.SafetyGateCore


def test_e_stop_latches_until_explicit_reset():
    core = SafetyGateCore(SafetyConfig())
    decision = core.evaluate(GateInputs(cmd=VelocityCommand(0.2, 0.1), cmd_age_s=0.0, e_stop=True))
    assert decision.command.is_zero()
    assert decision.reason == "e_stop_latched"
    assert decision.e_stop_latched is True

    decision = core.evaluate(GateInputs(cmd=VelocityCommand(0.2, 0.1), cmd_age_s=0.0, e_stop=False))
    assert decision.command.is_zero()
    assert decision.reason == "e_stop_latched"

    decision = core.evaluate(GateInputs(cmd=VelocityCommand(0.2, 0.1), cmd_age_s=0.0, e_stop_reset=True))
    assert decision.reason == "nominal"
    assert decision.e_stop_latched is False
    assert decision.command == VelocityCommand(0.2, 0.1)


def test_stale_watchdog_stops_missing_or_old_commands():
    core = SafetyGateCore(SafetyConfig(stale_timeout_s=0.30))
    assert core.evaluate(GateInputs(cmd=None, cmd_age_s=None)).reason == "stale_watchdog"
    old = core.evaluate(GateInputs(cmd=VelocityCommand(0.2, 0.0), cmd_age_s=0.31))
    assert old.reason == "stale_watchdog"
    assert old.command.is_zero()


def test_priority_orders_geofence_before_collision_and_speed_cap():
    core = SafetyGateCore(SafetyConfig(max_linear_speed=0.55, max_angular_speed=0.85))
    decision = core.evaluate(
        GateInputs(
            cmd=VelocityCommand(2.0, 3.0),
            cmd_age_s=0.0,
            geofence_ok=False,
            collision_stop=True,
        )
    )
    assert decision.reason == "geofence_or_mode_violation"
    assert decision.command.is_zero()

    decision = core.evaluate(GateInputs(cmd=VelocityCommand(2.0, 3.0), cmd_age_s=0.0, collision_stop=True))
    assert decision.reason == "collision_stop"
    assert decision.command.is_zero()


def test_speed_cap_limits_nominal_commands_without_zeroing():
    core = SafetyGateCore(SafetyConfig(max_linear_speed=0.55, max_angular_speed=0.85))
    decision = core.evaluate(GateInputs(cmd=VelocityCommand(0.80, -1.20), cmd_age_s=0.0))
    assert decision.reason == "speed_cap"
    assert decision.command == VelocityCommand(0.55, -0.85)

    decision = core.evaluate(GateInputs(cmd=VelocityCommand(0.30, 0.20), cmd_age_s=0.0))
    assert decision.reason == "nominal"
    assert decision.command == VelocityCommand(0.30, 0.20)
