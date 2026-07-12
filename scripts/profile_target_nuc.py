#!/usr/bin/env python3
"""Produce a hash-bound A12 target NUC resource profile.

The default collector is deliberately inert.  Target collection must be selected
explicitly and its run inputs authenticated before any host telemetry is read.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import os
from pathlib import Path
import platform
import re
import shutil
import statistics
import time
from typing import Any, Mapping, Protocol, Sequence

SCHEMA = "a12-target-nuc-resource-report-v1"
REQUIRED_WINDOWS_SECONDS = (900, 3600, 28800)
FINGERPRINT_FIELDS = (
    "machine_id", "machine", "cpu_model", "logical_cores", "physical_cores",
    "ram_bytes", "storage", "bios", "kernel", "microcode", "governor",
    "thermal", "throttle", "swap", "base_model_id", "release_id",
    "baseline_services",
)
TELEMETRY_FIELDS = (
    "elapsed_s", "loop", "loop_duration_ms", "deadline_ms",
    "cpu_utilization_pct_total", "rss_bytes", "ram_available_bytes",
    "disk_latency_ms", "temperature_c", "throttle_count", "swap_used_bytes",
)


class ProfileError(Exception):
    """An input cannot produce an auditable profile."""


class Collector(Protocol):
    name: str

    def fingerprint(self, run_inputs: Mapping[str, Any]) -> Mapping[str, Any]: ...

    def samples(self, duration_s: int, interval_s: float) -> Sequence[Mapping[str, Any]]: ...


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def sha256_object(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def sign_run_inputs(run_inputs: Mapping[str, Any], key: bytes) -> str:
    if not key:
        raise ProfileError("signing key is empty")
    return hmac.new(key, canonical_bytes(run_inputs), hashlib.sha256).hexdigest()


def verify_run_signature(run_inputs: Mapping[str, Any], signature: str, key: bytes) -> bool:
    return hmac.compare_digest(sign_run_inputs(run_inputs, key), signature.lower())


def percentile(values: Sequence[float], quantile: float) -> float:
    """Return a deterministic linearly-interpolated percentile."""
    if not values:
        raise ProfileError("percentile requires at least one sample")
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def _read_text(path: str, default: str = "unavailable") -> str:
    try:
        return Path(path).read_text(encoding="utf-8").strip() or default
    except (OSError, UnicodeError):
        return default


def _field(text: str, label: str, default: str = "unavailable") -> str:
    match = re.search(rf"^{re.escape(label)}\s*:\s*(.+)$", text, re.MULTILINE)
    return match.group(1).strip() if match else default


def _meminfo() -> dict[str, int]:
    values: dict[str, int] = {}
    for line in _read_text("/proc/meminfo", "").splitlines():
        match = re.match(r"^(\w+):\s+(\d+)\s+kB$", line)
        if match:
            values[match.group(1)] = int(match.group(2)) * 1024
    return values


def _cpu_ticks() -> tuple[int, int]:
    fields = _read_text("/proc/stat", "").splitlines()[0].split()
    ticks = [int(value) for value in fields[1:]]
    idle = ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
    return sum(ticks), idle


def _process_rss() -> int:
    status = _read_text("/proc/self/status", "")
    match = re.search(r"^VmRSS:\s+(\d+)\s+kB$", status, re.MULTILINE)
    return int(match.group(1)) * 1024 if match else 0


def _temperature() -> float:
    values = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            value = float(path.read_text(encoding="ascii").strip())
            values.append(value / 1000.0 if value > 1000 else value)
        except (OSError, UnicodeError, ValueError):
            continue
    return max(values) if values else float("nan")


def _throttle_count() -> int:
    total = 0
    for path in Path("/sys/devices/system/cpu").glob("cpu*/thermal_throttle/*_throttle_count"):
        try:
            total += int(path.read_text(encoding="ascii").strip())
        except (OSError, UnicodeError, ValueError):
            continue
    return total


def _disk_latency_ms(path: Path) -> float:
    start = time.monotonic_ns()
    with path.open("rb") as stream:
        stream.read(4096)
    return (time.monotonic_ns() - start) / 1_000_000.0


class DryCollector:
    name = "dry"

    def fingerprint(self, run_inputs: Mapping[str, Any]) -> Mapping[str, Any]:
        raise ProfileError("BLOCKED_NOT_TARGET")

    def samples(self, duration_s: int, interval_s: float) -> Sequence[Mapping[str, Any]]:
        raise ProfileError("BLOCKED_NOT_TARGET")


class SystemCollector:
    """Linux collector used only after signed target identity authorization."""

    name = "system"

    def __init__(self, workload_file: Path):
        self.workload_file = workload_file

    def fingerprint(self, run_inputs: Mapping[str, Any]) -> Mapping[str, Any]:
        cpuinfo = _read_text("/proc/cpuinfo", "")
        mem = _meminfo()
        governors = sorted({path.read_text(encoding="ascii").strip() for path in
                            Path("/sys/devices/system/cpu").glob("cpu*/cpufreq/scaling_governor")})
        machine_id = _read_text("/etc/machine-id")
        expected = str(run_inputs.get("expected_machine_id", ""))
        if not expected or machine_id != expected:
            raise ProfileError("BLOCKED_MACHINE_ID_MISMATCH")
        stat = shutil.disk_usage(self.workload_file.parent)
        physical_ids = set(re.findall(r"^physical id\s*:\s*(\d+)$", cpuinfo, re.MULTILINE))
        core_ids = set(re.findall(r"^core id\s*:\s*(\d+)$", cpuinfo, re.MULTILINE))
        physical_cores = len(physical_ids) * len(core_ids) if physical_ids and core_ids else (os.cpu_count() or 0)
        return {
            "machine_id": machine_id,
            "machine": platform.machine(),
            "cpu_model": _field(cpuinfo, "model name"),
            "logical_cores": os.cpu_count() or 0,
            "physical_cores": physical_cores,
            "ram_bytes": mem.get("MemTotal", 0),
            "storage": {"path": str(self.workload_file), "total_bytes": stat.total,
                        "device_model": _read_text("/sys/block/nvme0n1/device/model")},
            "bios": {"vendor": _read_text("/sys/class/dmi/id/bios_vendor"),
                     "version": _read_text("/sys/class/dmi/id/bios_version")},
            "kernel": platform.release(),
            "microcode": _field(cpuinfo, "microcode"),
            "governor": governors or ["unavailable"],
            "thermal": {"zones_present": len(list(Path("/sys/class/thermal").glob("thermal_zone*")))},
            "throttle": {"counter_source": "linux_thermal_throttle", "initial_count": _throttle_count()},
            "swap": {"total_bytes": mem.get("SwapTotal", 0)},
            "base_model_id": run_inputs["base_model_id"],
            "release_id": run_inputs["release_id"],
            "baseline_services": list(run_inputs["baseline_services"]),
        }

    def samples(self, duration_s: int, interval_s: float) -> Sequence[Mapping[str, Any]]:
        result = []
        started = time.monotonic()
        previous_total, previous_idle = _cpu_ticks()
        loop = 0
        while True:
            loop_started = time.monotonic()
            elapsed = loop_started - started
            total, idle = _cpu_ticks()
            delta_total = total - previous_total
            busy = delta_total - (idle - previous_idle)
            mem = _meminfo()
            result.append({
                "elapsed_s": elapsed, "loop": "system", "loop_duration_ms": 0.0,
                "deadline_ms": float(interval_s * 1000.0),
                "cpu_utilization_pct_total": 100.0 * busy / delta_total if delta_total > 0 else 0.0,
                "rss_bytes": _process_rss(), "ram_available_bytes": mem.get("MemAvailable", 0),
                "disk_latency_ms": _disk_latency_ms(self.workload_file), "temperature_c": _temperature(),
                "throttle_count": _throttle_count(), "swap_used_bytes": mem.get("SwapTotal", 0) - mem.get("SwapFree", 0),
            })
            result[-1]["loop_duration_ms"] = (time.monotonic() - loop_started) * 1000.0
            previous_total, previous_idle = total, idle
            loop += 1
            if elapsed >= duration_s:
                break
            time.sleep(min(interval_s, duration_s - elapsed))
        return result


def validate_run_inputs(run_inputs: Mapping[str, Any]) -> None:
    required = ("expected_machine_id", "base_model_id", "release_id", "baseline_services", "windows_seconds")
    if any(not run_inputs.get(field) for field in required):
        raise ProfileError("run inputs omit required target identity/release fields")
    if tuple(run_inputs["windows_seconds"]) != REQUIRED_WINDOWS_SECONDS:
        raise ProfileError("run inputs must request exactly the 15/60-minute and 8-hour windows")
    if not isinstance(run_inputs["baseline_services"], list) or not all(
            isinstance(item, str) and item for item in run_inputs["baseline_services"]):
        raise ProfileError("baseline_services must be a nonempty string list")


def summarize_samples(samples: Sequence[Mapping[str, Any]], fingerprint: Mapping[str, Any]) -> dict[str, Any]:
    if not samples:
        raise ProfileError("collector returned no telemetry")
    for sample in samples:
        missing = [field for field in TELEMETRY_FIELDS if field not in sample]
        if missing:
            raise ProfileError("telemetry sample missing: " + ", ".join(missing))
    windows = []
    for duration in REQUIRED_WINDOWS_SECONDS:
        selected = [sample for sample in samples if float(sample["elapsed_s"]) <= duration]
        covered = max(float(sample["elapsed_s"]) for sample in selected) if selected else 0.0
        windows.append({"duration_s": duration, "observed_duration_s": covered,
                        "sample_count": len(selected), "complete": covered >= duration})
    loops: dict[str, list[Mapping[str, Any]]] = {}
    for sample in samples:
        loops.setdefault(str(sample["loop"]), []).append(sample)
    loop_statistics = {}
    for name, values in sorted(loops.items()):
        durations = [float(item["loop_duration_ms"]) for item in values]
        loop_statistics[name] = {
            "sample_count": len(values), "p50_ms": percentile(durations, .50),
            "p95_ms": percentile(durations, .95), "p99_ms": percentile(durations, .99),
            "p999_ms": percentile(durations, .999),
            "deadline_misses": sum(float(item["loop_duration_ms"]) > float(item["deadline_ms"]) for item in values),
        }
    temperatures = [float(item["temperature_c"]) for item in samples]
    elapsed = [float(item["elapsed_s"]) for item in samples]
    mean_x = statistics.fmean(elapsed)
    denominator = sum((value - mean_x) ** 2 for value in elapsed)
    thermal_slope = (sum((x - mean_x) * (y - statistics.fmean(temperatures)) for x, y in zip(elapsed, temperatures)) /
                     denominator * 3600.0) if denominator else 0.0
    ram_total = int(fingerprint["ram_bytes"])
    return {
        "windows": windows,
        "loops": loop_statistics,
        "resources": {
            "cpu": {"normalization": "percent_of_total_logical_core_capacity",
                    "p99_utilization_pct_total": percentile([float(x["cpu_utilization_pct_total"]) for x in samples], .99),
                    "headroom_pct": 100.0 - max(float(x["cpu_utilization_pct_total"]) for x in samples)},
            "memory": {"peak_rss_bytes": max(int(x["rss_bytes"]) for x in samples),
                       "minimum_available_bytes": min(int(x["ram_available_bytes"]) for x in samples),
                       "headroom_pct": 100.0 * min(int(x["ram_available_bytes"]) for x in samples) / ram_total if ram_total else 0.0},
            "disk": {"p99_latency_ms": percentile([float(x["disk_latency_ms"]) for x in samples], .99)},
            "thermal": {"maximum_c": max(temperatures), "trend_c_per_hour": thermal_slope},
            "swap": {"maximum_used_bytes": max(int(x["swap_used_bytes"]) for x in samples)},
            "throttle": {"counter_delta": max(int(x["throttle_count"]) for x in samples) - min(int(x["throttle_count"]) for x in samples)},
        },
    }


def build_report(collector: Collector, run_inputs: Mapping[str, Any], signature: str,
                 key: bytes, interval_s: float = 1.0) -> dict[str, Any]:
    validate_run_inputs(run_inputs)
    if not verify_run_signature(run_inputs, signature, key):
        raise ProfileError("BLOCKED_INVALID_RUN_SIGNATURE")
    fingerprint = dict(collector.fingerprint(run_inputs))
    missing = [field for field in FINGERPRINT_FIELDS if field not in fingerprint]
    if missing:
        raise ProfileError("fingerprint missing: " + ", ".join(missing))
    samples = list(collector.samples(REQUIRED_WINDOWS_SECONDS[-1], interval_s))
    summary = summarize_samples(samples, fingerprint)
    report = {
        "schema": SCHEMA, "status": "candidate", "collection_mode": "target",
        "collector": collector.name, "run_inputs": dict(run_inputs),
        "run_inputs_sha256": sha256_object(run_inputs), "run_signature_hmac_sha256": signature.lower(),
        "signature_verified": True, "fingerprint": fingerprint,
        "fingerprint_sha256": sha256_object(fingerprint), "summary": summary,
    }
    report["report_sha256"] = sha256_object(report)
    return report


def blocked_report() -> dict[str, Any]:
    report = {"schema": SCHEMA, "status": "BLOCKED_NOT_TARGET", "collection_mode": "dry",
              "qualification": False, "diagnostic": "explicit signed target collection was not requested"}
    report["report_sha256"] = sha256_object(report)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collector", choices=("dry", "system"), default="dry")
    parser.add_argument("--run-inputs", type=Path)
    parser.add_argument("--signature")
    parser.add_argument("--signing-key-file", type=Path)
    parser.add_argument("--workload-file", type=Path, default=Path("/etc/machine-id"))
    parser.add_argument("--interval-seconds", type=float, default=1.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.collector == "dry":
            report = blocked_report()
        else:
            if not args.run_inputs or not args.signature or not args.signing_key_file:
                raise ProfileError("system collection requires --run-inputs, --signature, and --signing-key-file")
            run_inputs = json.loads(args.run_inputs.read_text(encoding="utf-8"))
            key = args.signing_key_file.read_bytes()
            report = build_report(SystemCollector(args.workload_file), run_inputs, args.signature, key,
                                  args.interval_seconds)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if report["status"] != "candidate":
            print(report["status"])
            return 2
        print(f"target profile candidate: {args.output}")
        return 0
    except (OSError, UnicodeError, json.JSONDecodeError, KeyError, ValueError, ProfileError) as exc:
        print(f"profile blocked: {exc}", file=os.sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
