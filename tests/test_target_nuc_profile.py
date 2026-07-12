"""Synthetic, no-sleep tests for A12 target NUC profiling and verification."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

REPOSITORY = Path(__file__).resolve().parents[1]
SCRIPTS = REPOSITORY / "scripts"
sys.path.insert(0, str(SCRIPTS))

import profile_target_nuc as profiler  # noqa: E402
import verify_resource_report as verifier  # noqa: E402

KEY = b"unit-test-target-run-key"


def run_inputs():
    return {
        "expected_machine_id": "nuc-production-001",
        "base_model_id": "wheelchair-base-v3",
        "release_id": "release-2026.07",
        "baseline_services": ["roscore.service", "wheelchair.service"],
        "windows_seconds": [900, 3600, 28800],
    }


def fingerprint():
    return {
        "machine_id": "nuc-production-001",
        "machine": "x86_64",
        "cpu_model": "Intel NUC synthetic fixture",
        "logical_cores": 8,
        "physical_cores": 4,
        "ram_bytes": 16_000_000_000,
        "storage": {"path": "/qualification", "total_bytes": 512_000_000_000,
                    "device_model": "NVMe fixture"},
        "bios": {"vendor": "Intel", "version": "A12-fixture"},
        "kernel": "5.15.0-target",
        "microcode": "0x123",
        "governor": ["performance"],
        "thermal": {"zones_present": 2},
        "throttle": {"counter_source": "fixture", "initial_count": 7},
        "swap": {"total_bytes": 0},
        "base_model_id": "wheelchair-base-v3",
        "release_id": "release-2026.07",
        "baseline_services": ["roscore.service", "wheelchair.service"],
    }


class SyntheticCollector:
    name = "synthetic-target-fixture"

    def __init__(self, target_fingerprint=None, samples=None):
        self.target_fingerprint = target_fingerprint or fingerprint()
        self.synthetic_samples = samples or self._samples()
        self.requested_duration = None

    @staticmethod
    def _samples():
        return [
            {"elapsed_s": elapsed, "loop": "navigation", "loop_duration_ms": duration,
             "deadline_ms": 25.0, "cpu_utilization_pct_total": cpu,
             "rss_bytes": rss, "ram_available_bytes": available,
             "disk_latency_ms": disk, "temperature_c": temperature,
             "throttle_count": 7, "swap_used_bytes": 0}
            for elapsed, duration, cpu, rss, available, disk, temperature in (
                (0, 8, 35, 1_000_000_000, 8_000_000_000, 0.4, 42),
                (900, 10, 45, 1_100_000_000, 7_000_000_000, 0.5, 44),
                (3600, 12, 55, 1_200_000_000, 6_000_000_000, 0.7, 46),
                (28800, 15, 60, 1_300_000_000, 5_000_000_000, 0.9, 48),
            )
        ]

    def fingerprint(self, requested_inputs):
        return self.target_fingerprint

    def samples(self, duration_s, interval_s):
        self.requested_duration = duration_s
        return self.synthetic_samples


def candidate(collector=None):
    inputs = run_inputs()
    collector = collector or SyntheticCollector()
    report = profiler.build_report(collector, inputs, profiler.sign_run_inputs(inputs, KEY), KEY)
    return report, collector


def rehash(report):
    report["report_sha256"] = profiler.sha256_object(
        {key: value for key, value in report.items() if key != "report_sha256"})


class TargetNucProfileTests(unittest.TestCase):
    def assertRejected(self, report, diagnostic, expected=None):
        with self.assertRaisesRegex(verifier.VerificationError, diagnostic):
            verifier.verify_report(report, expected or fingerprint(), KEY)

    def test_synthetic_full_windows_pass_without_sleeping(self):
        report, collector = candidate()
        result = verifier.verify_report(report, fingerprint(), KEY)
        self.assertTrue(result["qualified"])
        self.assertEqual(result["windows_seconds"], [900, 3600, 28800])
        self.assertEqual(collector.requested_duration, 28800)
        self.assertEqual(report["summary"]["loops"]["navigation"]["deadline_misses"], 0)
        self.assertIn("p999_ms", report["summary"]["loops"]["navigation"])
        self.assertEqual(report["summary"]["resources"]["cpu"]["normalization"],
                         "percent_of_total_logical_core_capacity")

    def test_default_cli_is_an_explicit_blocker(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "blocked.json"
            result = subprocess.run(
                [sys.executable, str(SCRIPTS / "profile_target_nuc.py"), "--output", str(output)],
                check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            blocked = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(blocked["status"], "BLOCKED_NOT_TARGET")
            self.assertRejected(blocked, "E_NOT_TARGET")

    def test_invalid_signed_inputs_are_blocked_before_collection(self):
        inputs = run_inputs()
        collector = SyntheticCollector()
        with self.assertRaisesRegex(profiler.ProfileError, "BLOCKED_INVALID_RUN_SIGNATURE"):
            profiler.build_report(collector, inputs, "0" * 64, KEY)
        self.assertIsNone(collector.requested_duration)

    def test_workstation_machine_substitution_is_rejected(self):
        report, _ = candidate()
        substituted = copy.deepcopy(fingerprint())
        substituted["machine_id"] = "developer-workstation"
        report["fingerprint"] = substituted
        report["fingerprint_sha256"] = profiler.sha256_object(substituted)
        rehash(report)
        self.assertRejected(report, "E_FINGERPRINT_DIVERGED")

    def test_signed_expected_machine_must_match_observation(self):
        report, _ = candidate()
        report["run_inputs"]["expected_machine_id"] = "developer-workstation"
        report["run_inputs_sha256"] = profiler.sha256_object(report["run_inputs"])
        report["run_signature_hmac_sha256"] = profiler.sign_run_inputs(report["run_inputs"], KEY)
        rehash(report)
        self.assertRejected(report, "E_WORKSTATION_SUBSTITUTION")

    def test_short_eight_hour_window_is_rejected(self):
        report, _ = candidate()
        report["summary"]["windows"][-1]["observed_duration_s"] = 28799
        report["summary"]["windows"][-1]["complete"] = False
        rehash(report)
        self.assertRejected(report, "E_SHORT_WINDOW")

    def test_missing_disk_telemetry_is_rejected(self):
        report, _ = candidate()
        del report["summary"]["resources"]["disk"]["p99_latency_ms"]
        rehash(report)
        self.assertRejected(report, "E_MISSING_TELEMETRY")

    def test_swap_use_is_rejected(self):
        report, _ = candidate()
        report["summary"]["resources"]["swap"]["maximum_used_bytes"] = 4096
        rehash(report)
        self.assertRejected(report, "E_SWAP")

    def test_throttle_event_is_rejected(self):
        report, _ = candidate()
        report["summary"]["resources"]["throttle"]["counter_delta"] = 1
        rehash(report)
        self.assertRejected(report, "E_THROTTLE")

    def test_deadline_miss_is_rejected(self):
        samples = SyntheticCollector._samples()
        samples[-1]["loop_duration_ms"] = 26
        report, _ = candidate(SyntheticCollector(samples=samples))
        self.assertRejected(report, "E_DEADLINE_MISS")

    def test_cpu_headroom_below_thirty_percent_is_rejected(self):
        report, _ = candidate()
        report["summary"]["resources"]["cpu"]["headroom_pct"] = 29.999
        rehash(report)
        self.assertRejected(report, "E_CPU_HEADROOM")

    def test_ram_headroom_below_twenty_five_percent_is_rejected(self):
        report, _ = candidate()
        report["summary"]["resources"]["memory"]["headroom_pct"] = 24.999
        rehash(report)
        self.assertRejected(report, "E_RAM_HEADROOM")

    def test_divergent_approved_fingerprint_is_rejected(self):
        report, _ = candidate()
        approved = copy.deepcopy(fingerprint())
        approved["bios"]["version"] = "different-bios"
        self.assertRejected(report, "E_FINGERPRINT_DIVERGED", approved)

    def test_report_content_hash_is_enforced(self):
        report, _ = candidate()
        report["summary"]["resources"]["cpu"]["headroom_pct"] = 99
        self.assertRejected(report, "E_REPORT_HASH")

    def test_missing_sample_field_stops_report_generation(self):
        samples = SyntheticCollector._samples()
        del samples[-1]["temperature_c"]
        with self.assertRaisesRegex(profiler.ProfileError, "telemetry sample missing: temperature_c"):
            candidate(SyntheticCollector(samples=samples))


if __name__ == "__main__":
    unittest.main()
