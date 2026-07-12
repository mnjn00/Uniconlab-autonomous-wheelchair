#!/usr/bin/env python3
"""Run the fail-closed Gazebo fault matrix and emit typed adversarial evidence."""

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

from run_gazebo_rc_suite import (
    ACTUATOR_COMMAND_TOPIC, APPROVED_STOP_BUDGET_S, DEFINED_SAFETY_REASON_MASK,
    FAULT_EVENT_TOPIC, SAFETY_REASON_BITS, EvidenceError, PlatformUnavailable,
    RosGazeboBackend, load_config, make_report,
)


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def _reason_evidence(raw, trigger_stamp_s):
    evidence = raw.get("fault_evidence")
    if not isinstance(evidence, dict):
        return set(), False
    observations = evidence.get("reason_observations")
    if not isinstance(observations, list) or not observations:
        return set(), False
    derived = set()
    valid = True
    for observation in observations:
        if not isinstance(observation, dict):
            valid = False
            continue
        names = observation.get("names")
        reason_mask = observation.get("reason_mask")
        canonical_names = (
            sorted(name for bit, name in SAFETY_REASON_BITS.items()
                   if isinstance(reason_mask, int) and not isinstance(reason_mask, bool)
                   and reason_mask & bit)
        )
        if (not isinstance(observation.get("source"), str)
                or not observation["source"]
                or not _finite(observation.get("stamp_s"))
                or float(observation["stamp_s"]) < float(trigger_stamp_s)
                or not isinstance(reason_mask, int)
                or isinstance(reason_mask, bool)
                or reason_mask <= 0
                or reason_mask & ~DEFINED_SAFETY_REASON_MASK
                or not isinstance(names, list)
                or not names
                or names != sorted(set(names))
                or names != canonical_names):
            valid = False
            continue
        derived.update(canonical_names)
    return derived, valid


def validate_fault_result(raw, fault, stop_budget_s=APPROVED_STOP_BUDGET_S):
    failures = []
    if raw.get("live_evidence") is not True:
        failures.append("missing live ROS/Gazebo evidence")
    if (raw.get("simulation_only") is not True
            or raw.get("hardware_motion_authorized") is not False
            or raw.get("passenger_operation_authorized") is not False):
        failures.append("fault evidence does not preserve simulation-only authority")
    observed_topics = raw.get("source_topics")
    missing_topics = [
        topic for topic in (FAULT_EVENT_TOPIC, ACTUATOR_COMMAND_TOPIC)
        if not isinstance(observed_topics, list) or topic not in observed_topics
    ]
    if missing_topics:
        failures.append("missing live fault topics: " + ", ".join(missing_topics))
    if raw.get("fault_injected") != fault["id"]:
        failures.append("collector fault injection identity does not match configured fault")
    if raw.get("safe_abort") is not True:
        failures.append("fault did not produce a safe abort")
    if raw.get("footprint_collisions") != 0:
        failures.append("collision occurred")
    if raw.get("geofence_exits") != 0:
        failures.append("safety boundary exit occurred")
    command = raw.get("command", {})
    if not isinstance(command, dict):
        command = {}
    if command.get("finite") is not True or command.get("caps_respected") is not True:
        failures.append("command was non-finite or over cap")
    if command.get("nonzero_after_fault") != 0:
        failures.append("nonzero actuator command observed after fault")

    evidence = raw.get("fault_evidence")
    evidence = evidence if isinstance(evidence, dict) else {}
    trigger = evidence.get("trigger_stamp_s")
    zero = evidence.get("actuator_zero_stamp_s")
    latency = evidence.get("actuator_zero_latency_s")
    observed_budget = evidence.get("actuator_zero_budget_s")
    timing_valid = (
        all(_finite(value) for value in (trigger, zero, latency, observed_budget))
        and float(observed_budget) == float(stop_budget_s)
        and float(zero) >= float(trigger)
        and abs(float(latency) - (float(zero) - float(trigger))) <= 1e-9
        and 0.0 <= float(latency) <= float(stop_budget_s)
    )
    if raw.get("zero_within_budget") is not True or not timing_valid:
        failures.append("fault stop missed or did not prove its exact-zero timing budget")

    reason_events = raw.get("reason_events")
    reasons_well_formed = (
        isinstance(reason_events, list)
        and all(isinstance(reason, str) and reason for reason in reason_events)
        and reason_events == sorted(set(reason_events))
    )
    derived_reasons, provenance_valid = (
        _reason_evidence(raw, trigger) if _finite(trigger) else (set(), False))
    observed = set(reason_events) if reasons_well_formed else set()
    if not reasons_well_formed or not provenance_valid or observed != derived_reasons:
        failures.append("reason events lack live symbolic reason-mask provenance")
    expected = set(fault["expected_reasons"])
    if not expected.issubset(observed):
        failures.append("missing reason events: " + ", ".join(sorted(expected - observed)))
    if fault["requires_latch"] and raw.get("latched_until_guarded_reset") is not True:
        failures.append("fault did not prove its latch through a guarded reset attempt")
    result = dict(raw)
    result.update({
        "fault": fault["id"],
        "category": fault["category"],
        "expected_reasons": list(fault["expected_reasons"]),
        "requires_latch": fault["requires_latch"],
        "failures": failures,
        "passed": not failures,
    })
    return result


def fault_coverage_failures(configured_faults, results):
    configured = [fault["id"] for fault in configured_faults]
    executed = [result.get("fault") for result in results]
    failures = []
    for fault_id in configured:
        count = executed.count(fault_id)
        if count != 1:
            failures.append(
                "configured fault {} executed {} times (required exactly once)".format(
                    fault_id, count))
    unexpected = sorted(
        (value for value in set(executed) - set(configured)), key=lambda value: str(value))
    if unexpected:
        failures.append(
            "unconfigured faults executed: " + ", ".join(str(value) for value in unexpected))
    return failures


def execute_fault_matrix(config, backend):
    world_id = config["fault_world"]
    world = next(item for item in config["worlds"] if item["id"] == world_id)
    stop_budget_s = float(
        config["collector_command"][
            config["collector_command"].index("--stop-budget-s") + 1])
    results = []
    for index, fault in enumerate(config["fault_matrix"]):
        raw = backend.run_scenario(
            world, index, robustness=False, fault=fault["id"])
        results.append(validate_fault_result(raw, fault, stop_budget_s))
    coverage = fault_coverage_failures(config["fault_matrix"], results)
    failed = [item for item in results if not item["passed"]]
    failures = [failure for item in failed for failure in item["failures"]] + coverage
    return {
        "passed": not failures,
        "authority": {
            "claim_tag": config["claim_tag"],
            "simulation_only": config["simulation_only"],
            "hardware_motion_authorized": config["hardware_motion_authorized"],
            "passenger_operation_authorized": config["passenger_operation_authorized"],
        },
        "faults": results,
        "summary": {
            "total": len(results),
            "configured": len(config["fault_matrix"]),
            "passed": len(results) - len(failed),
            "failed": len(failed) + (1 if coverage else 0),
            "executed_fault_ids": [item["fault"] for item in results],
        },
        "failures": failures,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    default_config = Path(__file__).resolve().parents[1] / "src/wheelchair_gazebo/config/scenarios.yaml"
    parser.add_argument("--config", default=str(default_config))
    parser.add_argument("--output", required=True)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    invocation = [sys.executable, str(Path(__file__).resolve())] + list(argv if argv is not None else sys.argv[1:])
    try:
        config = load_config(args.config)
        result = execute_fault_matrix(config, RosGazeboBackend(config, args.startup_timeout))
        report = make_report(args.config, invocation, "PASS" if result["passed"] else "FAIL", result=result)
        exit_code = 0 if result["passed"] else 1
    except PlatformUnavailable as exc:
        report = make_report(args.config, invocation, "PLATFORM_UNAVAILABLE", error=str(exc))
        exit_code = 2
    except (EvidenceError, subprocess.SubprocessError, OSError, ValueError, json.JSONDecodeError) as exc:
        report = make_report(args.config, invocation, "FAIL", error=str(exc))
        exit_code = 1
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(str(output))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
