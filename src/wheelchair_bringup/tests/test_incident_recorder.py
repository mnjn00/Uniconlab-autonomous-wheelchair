#!/usr/bin/env python3
"""Pure tests for the non-authoritative bounded incident recorder."""

import importlib.util
import json
from pathlib import Path

import pytest

HERE = Path(__file__).resolve()
RECORDER_PATH = HERE.parents[1] / "scripts" / "incident_recorder.py"
AUDITOR_PATH = HERE.parents[3] / "scripts" / "audit_incident_evidence.py"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


incident = load("incident_recorder_under_test", RECORDER_PATH)
auditor = load("audit_incident_evidence_under_test", AUDITOR_PATH)
HASH = "a" * 64
IDENTITIES = {name: {"available": True, "sha256": HASH, "source": "fixture"}
              for name in ("config", "map", "route", "release")}


class Clock:
    def __init__(self):
        self.wall = 1_700_000_000_000_000_000
        self.mono = 0

    def set(self, seconds):
        self.mono = int(seconds * 1e9)
        self.wall = 1_700_000_000_000_000_000 + self.mono


def config(**changes):
    values = dict(pre_event_s=1.0, post_event_s=1.0,
                  max_memory_bytes=1024 * 1024,
                  max_record_bytes=16 * 1024,
                  max_evidence_bytes=1024 * 1024,
                  max_write_queue=2, rate_limit_s=1.0, poll_s=0.01,
                  max_string_chars=128, max_collection_items=32,
                  max_gap_s=0.6)
    values.update(changes)
    return incident.RecorderConfig(**values)


def recorder(tmp_path, clock, **changes):
    return incident.IncidentRecorder(
        tmp_path, IDENTITIES, ["ESTOP", "COLLISION", "SAFETY_FAULT"],
        config(**changes), wall_time_ns=lambda: clock.wall,
        monotonic_ns=lambda: clock.mono, start_workers=False)


def make_complete_evidence(tmp_path):
    clock = Clock()
    core = recorder(tmp_path, clock)
    for moment in (0.0, 0.5):
        clock.set(moment)
        assert core.capture("/diagnostics", {"sequence": int(moment * 10)})
    clock.set(1.0)
    assert core.capture("/safety/state", {"state": "FAULT", "reason_mask": 1})
    assert core.trigger("ESTOP", "ESTOP", "FAULT",
                        {"reason_mask": 1, "sequence": 3})
    for moment in (1.5, 2.0):
        clock.set(moment)
        assert core.capture("/cmd_vel_safe", {"linear": 0.0})
    assert core.flush_ready() == 1
    path = core.write_one()
    assert path is not None
    return path, core


def test_ring_and_evidence_are_ordered(tmp_path):
    path, _ = make_complete_evidence(tmp_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    receipts = [row["received_monotonic_ns"] for row in document["records"]]
    sequences = [row["sequence"] for row in document["records"]]
    assert receipts == sorted(receipts)
    assert sequences == sorted(sequences)
    assert document["completeness"] == {
        "complete": True,
        "drop_counters": {
            "lock_contention": 0, "memory_overflow": 0,
            "oversize_record": 0, "pending_overflow": 0,
            "rate_limited_event": 0, "write_failure": 0,
            "write_queue_overflow": 0,
        },
        "reasons": [],
    }


def test_memory_overflow_evicts_oldest_and_counts_drop(tmp_path):
    clock = Clock()
    core = recorder(tmp_path, clock, max_memory_bytes=600,
                    max_record_bytes=300, max_evidence_bytes=600)
    for index in range(8):
        clock.set(index / 10.0)
        core.capture("/diagnostics", {"index": index, "padding": "x" * 80})
    counters = core.counters()
    assert counters["ring_bytes"] <= 600
    assert counters["memory_overflow"] > 0
    with core._lock:
        indices = [item.payload["index"] for item in core._records]
    assert indices == sorted(indices)
    assert 0 not in indices


def test_oversize_and_pending_backpressure_are_nonblocking(tmp_path):
    clock = Clock()
    core = recorder(tmp_path, clock, max_record_bytes=180, max_write_queue=1)
    assert not core.capture("/scan", {"large": "z" * 1000})
    assert core.counters()["oversize_record"] == 1
    assert core.trigger("ESTOP", "ESTOP", "FAULT", {"reason_mask": 1})
    assert not core.trigger("COLLISION", "COLLISION", "STOPPED", {"reason_mask": 16})
    assert core.counters()["pending_overflow"] == 1


def test_only_named_events_trigger_and_repeats_are_rate_limited(tmp_path):
    clock = Clock()
    core = recorder(tmp_path, clock)
    assert not core.trigger("NOT_A_SAFETY_EVENT", "x", "FAULT", {"x": 1})
    assert core.trigger("ESTOP", "ESTOP", "FAULT", {"reason_mask": 1})
    clock.set(0.5)
    assert not core.trigger("ESTOP", "ESTOP", "FAULT", {"reason_mask": 1})
    assert core.counters()["rate_limited_event"] == 1


def test_safety_detector_emits_new_reason_bits_and_state_transition_once():
    detector = incident.SafetyEventDetector()
    assert detector.events(4, (1 << 0) | (1 << 4)) == [
        ("ESTOP", "ESTOP"), ("COLLISION", "COLLISION"),
        ("SAFETY_FAULT", "STATE_TRANSITION")]
    assert detector.events(4, (1 << 0) | (1 << 4)) == []
    assert detector.events(1, 0) == []


def test_secrets_are_redacted_and_collections_are_bounded(tmp_path):
    clock = Clock()
    core = recorder(tmp_path, clock)
    secret = "do-not-leak-credential"
    assert core.capture("/diagnostics", {
        "password": secret,
        "detail": "Bearer abc.def.ghi",
        "values": list(range(100)),
    })
    with core._lock:
        payload = core._records[-1].payload
    rendered = json.dumps(payload)
    assert secret not in rendered
    assert "abc.def.ghi" not in rendered
    assert payload["password"] == incident.REDACTED
    assert payload["values"][-1] == {"_truncated_items": 68}


def test_atomic_write_leaves_only_complete_final_json(tmp_path):
    target = tmp_path / "nested" / "evidence.json"
    document = {"schema": "fixture", "value": [1, 2, 3]}
    size = incident.atomic_write_json(target, document, 1024)
    assert size == target.stat().st_size
    assert json.loads(target.read_text(encoding="utf-8")) == document
    assert list(target.parent.glob(".*.tmp")) == []
    with pytest.raises(ValueError):
        incident.atomic_write_json(target, {"large": "x" * 2048}, 32)
    assert json.loads(target.read_text(encoding="utf-8")) == document


def test_auditor_passes_complete_evidence_and_returns_typed_report(tmp_path):
    path, _ = make_complete_evidence(tmp_path)
    audit = auditor.audit_file(path)
    assert audit["status"] == "PASS"
    report = auditor.build_report([tmp_path])
    assert report["type"] == "api-algorithm-test-report"
    assert report["algorithm"] == "wheelchair-incident-evidence-audit-v1"
    assert report["status"] == "PASS"


def test_auditor_fails_corrupt_and_out_of_order_evidence(tmp_path):
    corrupt = tmp_path / "incident-corrupt.json"
    corrupt.write_text("{not-json", encoding="utf-8")
    assert auditor.audit_file(corrupt)["status"] == "FAIL"

    path, _ = make_complete_evidence(tmp_path / "valid")
    document = json.loads(path.read_text(encoding="utf-8"))
    document["records"][1], document["records"][2] = (
        document["records"][2], document["records"][1])
    bad = tmp_path / "incident-unordered.json"
    bad.write_text(json.dumps(document), encoding="utf-8")
    audit = auditor.audit_file(bad)
    assert audit["status"] == "FAIL"
    assert any(case["name"] == "receipt_order" and not case["passed"]
               for case in audit["cases"])


def test_missing_hash_or_fault_window_is_explicitly_incomplete(tmp_path):
    path, _ = make_complete_evidence(tmp_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    document["identities"]["map"] = {
        "available": False, "sha256": None, "source": "unavailable"}
    document["records"] = document["records"][2:3]
    incomplete = tmp_path / "incident-incomplete.json"
    incomplete.write_text(json.dumps(document), encoding="utf-8")
    audit = auditor.audit_file(incomplete)
    assert audit["status"] == "INCOMPLETE"
    failed = {case["name"] for case in audit["cases"] if not case["passed"]}
    assert {"identity.map", "fault_window"}.issubset(failed)


def test_recorder_and_artifacts_cannot_authorize_or_publish_motion(tmp_path):
    clock = Clock()
    core = recorder(tmp_path, clock)
    assert core.publishes_motion is False
    assert core.grants_permission is False
    path, _ = make_complete_evidence(tmp_path / "evidence")
    document = json.loads(path.read_text(encoding="utf-8"))
    assert all(value is False for value in document["authority"].values())
    source = RECORDER_PATH.read_text(encoding="utf-8")
    assert "rospy.Publisher(" not in source
    assert "rospy.Service(" not in source
