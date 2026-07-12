#!/usr/bin/env python3
"""Fail-closed verifier for A12 target NUC resource reports."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

from profile_target_nuc import (
    FINGERPRINT_FIELDS,
    REQUIRED_WINDOWS_SECONDS,
    SCHEMA,
    canonical_bytes,
    sha256_object,
    verify_run_signature,
)

MIN_CPU_HEADROOM_PCT = 30.0
MIN_RAM_HEADROOM_PCT = 25.0
REQUIRED_LOOP_METRICS = (
    "sample_count", "p50_ms", "p95_ms", "p99_ms", "p999_ms", "deadline_misses",
)


class VerificationError(Exception):
    """The report is not acceptable target qualification evidence."""


def fail(code: str, detail: str) -> None:
    raise VerificationError(f"{code}: {detail}")


def require_object(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        fail("E_MISSING_TELEMETRY", f"{label} must be an object")
    return value


def finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        fail("E_MISSING_TELEMETRY", f"{label} must be finite")
    return float(value)


def _without_report_hash(report: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in report.items() if key != "report_sha256"}


def verify_report(report: Mapping[str, Any], expected_fingerprint: Mapping[str, Any], key: bytes) -> dict[str, Any]:
    """Verify identity, provenance, complete windows, telemetry, and limits."""
    if report.get("schema") != SCHEMA:
        fail("E_SCHEMA", "unexpected or missing report schema")
    if report.get("status") == "BLOCKED_NOT_TARGET" or report.get("collection_mode") == "dry":
        fail("E_NOT_TARGET", "dry/workstation evidence cannot qualify the target NUC")
    if report.get("status") != "candidate" or report.get("collection_mode") != "target":
        fail("E_NOT_TARGET", "report is not a target candidate")
    if not key:
        fail("E_SIGNATURE", "verification key is empty")
    actual_hash = sha256_object(_without_report_hash(report))
    if report.get("report_sha256") != actual_hash:
        fail("E_REPORT_HASH", "report content does not match report_sha256")

    run_inputs = require_object(report.get("run_inputs"), "run_inputs")
    if report.get("run_inputs_sha256") != sha256_object(run_inputs):
        fail("E_RUN_INPUT_HASH", "run inputs hash mismatch")
    signature = report.get("run_signature_hmac_sha256")
    if not isinstance(signature, str) or not verify_run_signature(run_inputs, signature, key):
        fail("E_SIGNATURE", "run inputs signature is invalid")
    if report.get("signature_verified") is not True:
        fail("E_SIGNATURE", "collector did not record signature verification")
    if tuple(run_inputs.get("windows_seconds", ())) != REQUIRED_WINDOWS_SECONDS:
        fail("E_SHORT_WINDOW", "signed inputs do not request all required windows")

    fingerprint = require_object(report.get("fingerprint"), "fingerprint")
    missing = [field for field in FINGERPRINT_FIELDS if field not in fingerprint]
    if missing:
        fail("E_FINGERPRINT", "missing fingerprint fields: " + ", ".join(missing))
    if report.get("fingerprint_sha256") != sha256_object(fingerprint):
        fail("E_FINGERPRINT", "embedded fingerprint hash mismatch")
    expected_hash = sha256_object(expected_fingerprint)
    if fingerprint != expected_fingerprint or report.get("fingerprint_sha256") != expected_hash:
        fail("E_FINGERPRINT_DIVERGED", "observed target fingerprint differs from approved fingerprint")
    if fingerprint.get("machine_id") != run_inputs.get("expected_machine_id"):
        fail("E_WORKSTATION_SUBSTITUTION", "machine identity differs from signed expected target")
    for field in ("base_model_id", "release_id", "baseline_services"):
        if fingerprint.get(field) != run_inputs.get(field):
            fail("E_RELEASE_IDENTITY", f"fingerprint {field} differs from signed run inputs")

    summary = require_object(report.get("summary"), "summary")
    windows = summary.get("windows")
    if not isinstance(windows, list) or len(windows) != len(REQUIRED_WINDOWS_SECONDS):
        fail("E_SHORT_WINDOW", "required 15/60-minute and 8-hour windows are absent")
    observed_durations = []
    for expected, window in zip(REQUIRED_WINDOWS_SECONDS, windows):
        window = require_object(window, "window")
        duration = finite_number(window.get("duration_s"), "window.duration_s")
        observed = finite_number(window.get("observed_duration_s"), "window.observed_duration_s")
        count = finite_number(window.get("sample_count"), "window.sample_count")
        if duration != expected or observed < expected or count < 2 or window.get("complete") is not True:
            fail("E_SHORT_WINDOW", f"{expected}-second window is incomplete")
        observed_durations.append(observed)
    if observed_durations != sorted(observed_durations):
        fail("E_SHORT_WINDOW", "window coverage is not monotonic")

    loops = require_object(summary.get("loops"), "summary.loops")
    if not loops:
        fail("E_MISSING_TELEMETRY", "no loop timing telemetry")
    total_samples = 0
    for name, raw_metrics in loops.items():
        metrics = require_object(raw_metrics, f"loop {name}")
        missing = [metric for metric in REQUIRED_LOOP_METRICS if metric not in metrics]
        if missing:
            fail("E_MISSING_TELEMETRY", f"loop {name} missing: {', '.join(missing)}")
        numeric = {metric: finite_number(metrics[metric], f"loop {name}.{metric}") for metric in REQUIRED_LOOP_METRICS}
        if numeric["sample_count"] < 1:
            fail("E_MISSING_TELEMETRY", f"loop {name} has no samples")
        if not numeric["p50_ms"] <= numeric["p95_ms"] <= numeric["p99_ms"] <= numeric["p999_ms"]:
            fail("E_MISSING_TELEMETRY", f"loop {name} percentiles are not monotonic")
        if numeric["deadline_misses"] != 0:
            fail("E_DEADLINE_MISS", f"loop {name} has deadline misses")
        total_samples += int(numeric["sample_count"])

    resources = require_object(summary.get("resources"), "summary.resources")
    cpu = require_object(resources.get("cpu"), "resources.cpu")
    memory = require_object(resources.get("memory"), "resources.memory")
    disk = require_object(resources.get("disk"), "resources.disk")
    thermal = require_object(resources.get("thermal"), "resources.thermal")
    swap = require_object(resources.get("swap"), "resources.swap")
    throttle = require_object(resources.get("throttle"), "resources.throttle")
    if cpu.get("normalization") != "percent_of_total_logical_core_capacity":
        fail("E_CPU_NORMALIZATION", "CPU must be normalized to total capacity")
    cpu_headroom = finite_number(cpu.get("headroom_pct"), "cpu.headroom_pct")
    finite_number(cpu.get("p99_utilization_pct_total"), "cpu.p99_utilization_pct_total")
    if cpu_headroom < MIN_CPU_HEADROOM_PCT:
        fail("E_CPU_HEADROOM", f"{cpu_headroom:.3f}% is below {MIN_CPU_HEADROOM_PCT:.0f}%")
    ram_headroom = finite_number(memory.get("headroom_pct"), "memory.headroom_pct")
    finite_number(memory.get("peak_rss_bytes"), "memory.peak_rss_bytes")
    finite_number(memory.get("minimum_available_bytes"), "memory.minimum_available_bytes")
    if ram_headroom < MIN_RAM_HEADROOM_PCT:
        fail("E_RAM_HEADROOM", f"{ram_headroom:.3f}% is below {MIN_RAM_HEADROOM_PCT:.0f}%")
    finite_number(disk.get("p99_latency_ms"), "disk.p99_latency_ms")
    finite_number(thermal.get("maximum_c"), "thermal.maximum_c")
    finite_number(thermal.get("trend_c_per_hour"), "thermal.trend_c_per_hour")
    if finite_number(swap.get("maximum_used_bytes"), "swap.maximum_used_bytes") != 0:
        fail("E_SWAP", "swap was used during qualification")
    if finite_number(throttle.get("counter_delta"), "throttle.counter_delta") != 0:
        fail("E_THROTTLE", "hardware throttling occurred during qualification")

    return {
        "qualified": True,
        "fingerprint_sha256": expected_hash,
        "report_sha256": actual_hash,
        "windows_seconds": list(REQUIRED_WINDOWS_SECONDS),
        "loop_sample_count": total_samples,
        "cpu_headroom_pct": cpu_headroom,
        "ram_headroom_pct": ram_headroom,
    }


def load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail("E_INPUT", f"cannot read {label}: {exc}")
    if not isinstance(value, dict):
        fail("E_INPUT", f"{label} must contain a JSON object")
    return value


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--expected-fingerprint", type=Path, required=True)
    parser.add_argument("--signing-key-file", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        report = load_object(args.report, "report")
        expected = load_object(args.expected_fingerprint, "expected fingerprint")
        key = args.signing_key_file.read_bytes()
        result = verify_report(report, expected, key)
        rendered = json.dumps(result, sort_keys=True)
        if args.output:
            args.output.write_text(rendered + "\n", encoding="utf-8")
        print(rendered)
        return 0
    except (OSError, UnicodeError, VerificationError, ValueError, TypeError, KeyError) as exc:
        print(f"resource report rejected: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
