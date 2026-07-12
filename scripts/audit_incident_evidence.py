#!/usr/bin/env python3
"""Fail-closed audit for bounded wheelchair incident evidence."""

import argparse
import json
import re
import sys
from pathlib import Path

SCHEMA = "wheelchair-incident-evidence/v1"
REPORT_TYPE = "api-algorithm-test-report"
_HASH = re.compile(r"^[0-9a-f]{64}$")
REQUIRED_IDENTITIES = ("config", "map", "route", "release")

_SECRET_KEY = re.compile(r"(authorization|cookie|credential|passwd|password|private.?key|secret|token)", re.I)
_SECRET_VALUE = re.compile(r"(?i)bearer\s+[a-z0-9._~+/-]+=*")


def _contains_secret(value, key=""):
    if _SECRET_KEY.search(str(key)):
        return value != "[REDACTED]"
    if isinstance(value, dict):
        return any(_contains_secret(child, child_key) for child_key, child in value.items())
    if isinstance(value, list):
        return any(_contains_secret(child, key) for child in value)
    return isinstance(value, str) and bool(_SECRET_VALUE.search(value))


def _case(name, passed, detail, severity="error"):
    return {"name": name, "passed": bool(passed), "severity": severity,
            "detail": str(detail)}


def audit_document(document, source="<memory>", file_bytes=None):
    """Audit one parsed incident document and return cases plus disposition."""
    cases = []
    cases.append(_case("schema", isinstance(document, dict) and document.get("schema") == SCHEMA,
                       "schema must be %s" % SCHEMA))
    event = document.get("event") if isinstance(document, dict) else None
    event_ok = isinstance(event, dict)
    for field in ("name", "reason", "state"):
        value = event.get(field) if event_ok else None
        cases.append(_case("event.%s" % field, isinstance(value, str) and bool(value.strip()),
                           "required non-empty string"))
    context = event.get("context") if event_ok else None
    cases.append(_case("event.context", isinstance(context, dict) and bool(context),
                       "required non-empty object"))
    trigger = event.get("received_monotonic_ns") if event_ok else None
    cases.append(_case("event.timestamp", isinstance(event.get("timestamp_ns") if event_ok else None, int) and
                       isinstance(trigger, int), "wall and monotonic event timestamps must be integers"))

    identities = document.get("identities", {}) if isinstance(document, dict) else {}
    unavailable = []
    for name in REQUIRED_IDENTITIES:
        item = identities.get(name) if isinstance(identities, dict) else None
        valid = (isinstance(item, dict) and item.get("available") is True and
                 isinstance(item.get("sha256"), str) and _HASH.fullmatch(item["sha256"]))
        if not valid:
            unavailable.append(name)
        cases.append(_case("identity.%s" % name, valid,
                           "available lowercase SHA-256 required", severity="incomplete"))

    limits = document.get("limits", {}) if isinstance(document, dict) else {}
    limit_ok = (isinstance(limits, dict) and isinstance(limits.get("max_memory_bytes"), int) and
                0 < limits["max_memory_bytes"] <= 64 * 1024 * 1024 and
                isinstance(limits.get("max_evidence_bytes"), int) and
                0 < limits["max_evidence_bytes"] <= limits["max_memory_bytes"] and
                isinstance(limits.get("max_write_queue"), int) and limits["max_write_queue"] > 0)
    cases.append(_case("bounded_limits", limit_ok,
                       "memory <=64 MiB, evidence <= memory, and bounded positive queue required"))
    if file_bytes is not None:
        size_ok = limit_ok and file_bytes <= limits["max_evidence_bytes"]
        cases.append(_case("file_size", size_ok,
                           "%d bytes must not exceed max_evidence_bytes" % file_bytes))

    records = document.get("records") if isinstance(document, dict) else None
    records_ok = isinstance(records, list) and bool(records)
    cases.append(_case("records", records_ok, "non-empty record list required"))
    parsed = []
    if isinstance(records, list):
        for index, record in enumerate(records):
            valid = (isinstance(record, dict) and isinstance(record.get("timestamp_ns"), int) and
                     isinstance(record.get("received_monotonic_ns"), int) and
                     isinstance(record.get("sequence"), int) and
                     isinstance(record.get("topic"), str) and
                     isinstance(record.get("payload"), dict))
            if not valid:
                cases.append(_case("record[%d]" % index, False, "malformed record"))
            else:
                parsed.append(record)
    ordered = bool(parsed) and len(parsed) == len(records) and all(
        (left["received_monotonic_ns"], left["sequence"]) <
        (right["received_monotonic_ns"], right["sequence"])
        for left, right in zip(parsed, parsed[1:]))
    source_ordered = bool(parsed) and all(
        left["timestamp_ns"] <= right["timestamp_ns"] for left, right in zip(parsed, parsed[1:]))
    cases.append(_case("receipt_order", ordered, "receipt timestamp/sequence must be strictly ordered"))
    cases.append(_case("source_timestamp_order", source_ordered,
                       "source timestamps must be nondecreasing"))

    window_ok = False
    gap_ok = False
    if parsed and isinstance(trigger, int) and limit_ok:
        pre_ns = int(float(limits.get("pre_event_s", 0)) * 1e9)
        post_ns = int(float(limits.get("post_event_s", 0)) * 1e9)
        tolerance_ns = int(float(limits.get("max_gap_s", 0)) * 1e9)
        positive_window = pre_ns > 0 and post_ns > 0 and tolerance_ns > 0
        window_ok = (positive_window and
                     parsed[0]["received_monotonic_ns"] <= trigger - pre_ns + tolerance_ns and
                     parsed[-1]["received_monotonic_ns"] >= trigger + post_ns - tolerance_ns and
                     any(item["received_monotonic_ns"] <= trigger for item in parsed) and
                     any(item["received_monotonic_ns"] >= trigger for item in parsed))
        gap_ok = positive_window and all(
            right["received_monotonic_ns"] - left["received_monotonic_ns"] <= tolerance_ns
            for left, right in zip(parsed, parsed[1:]))
    cases.append(_case("fault_window", window_ok,
                       "records must cover configured pre/post bounds around fault", severity="incomplete"))
    cases.append(_case("fault_gaps", gap_ok,
                       "no receipt gap may exceed max_gap_s around fault", severity="incomplete"))

    completeness = document.get("completeness", {}) if isinstance(document, dict) else {}
    declared_complete = (isinstance(completeness, dict) and completeness.get("complete") is True and
                         completeness.get("reasons") == [])
    cases.append(_case("declared_complete", declared_complete,
                       "recorder explicitly marked evidence incomplete", severity="incomplete"))
    counters = completeness.get("drop_counters", {}) if isinstance(completeness, dict) else {}
    no_drops = isinstance(counters, dict) and all(
        isinstance(value, int) and value == 0 for key, value in counters.items()
        if key not in ("ring_records", "ring_bytes", "pending_incidents",
                       "write_queue_depth", "written_incidents"))
    cases.append(_case("drop_counters", no_drops, "all recorder drop counters must be zero",
                       severity="incomplete"))

    authority = document.get("authority", {}) if isinstance(document, dict) else {}
    authority_ok = isinstance(authority, dict) and all(
        authority.get(name) is False for name in
        ("hardware_motion_authorized", "passenger_operation_authorized",
         "publishes_motion", "grants_permission"))
    cases.append(_case("non_authority", authority_ok,
                       "evidence must explicitly deny motion, permission, hardware, and passenger authority"))
    cases.append(_case("redaction", not _contains_secret(document),
                       "credential-shaped keys and bearer values must be redacted"))

    hard_fail = any(not case["passed"] and case["severity"] == "error" for case in cases)
    incomplete = any(not case["passed"] and case["severity"] == "incomplete" for case in cases)
    status = "FAIL" if hard_fail else ("INCOMPLETE" if incomplete else "PASS")
    return {"source": source, "status": status, "cases": cases,
            "summary": {"passed": sum(case["passed"] for case in cases),
                        "failed": sum(not case["passed"] for case in cases),
                        "unavailable_identities": unavailable}}


def audit_file(path):
    candidate = Path(path)
    try:
        payload = candidate.read_bytes()
        document = json.loads(payload.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return {"source": str(candidate), "status": "FAIL",
                "cases": [_case("parse", False, "corrupt evidence: %s" % exc)],
                "summary": {"passed": 0, "failed": 1,
                            "unavailable_identities": list(REQUIRED_IDENTITIES)}}
    return audit_document(document, str(candidate), len(payload))


def discover(paths):
    files = []
    for raw in paths:
        path = Path(raw)
        if path.is_dir():
            files.extend(sorted(path.rglob("incident-*.json")))
        else:
            files.append(path)
    return files


def build_report(paths):
    files = discover(paths)
    audits = [audit_file(path) for path in files]
    if not audits:
        audits = [{"source": "<none>", "status": "FAIL",
                   "cases": [_case("evidence_inventory", False, "no incident evidence found")],
                   "summary": {"passed": 0, "failed": 1,
                               "unavailable_identities": list(REQUIRED_IDENTITIES)}}]
    status = "FAIL" if any(item["status"] == "FAIL" for item in audits) else (
        "INCOMPLETE" if any(item["status"] == "INCOMPLETE" for item in audits) else "PASS")
    return {
        "type": REPORT_TYPE,
        "artifact_type": REPORT_TYPE,
        "schema_version": 1,
        "algorithm": "wheelchair-incident-evidence-audit-v1",
        "status": status,
        "authority": {"hardware_motion_authorized": False,
                      "passenger_operation_authorized": False},
        "evidence": audits,
        "summary": {"files": len(files),
                    "pass": sum(item["status"] == "PASS" for item in audits),
                    "incomplete": sum(item["status"] == "INCOMPLETE" for item in audits),
                    "fail": sum(item["status"] == "FAIL" for item in audits)},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="Audit bounded incident evidence")
    parser.add_argument("paths", nargs="+", help="incident JSON files or directories")
    parser.add_argument("--report", help="optional JSON report path")
    args = parser.parse_args(argv)
    report = build_report(args.paths)
    rendered = json.dumps(report, sort_keys=True, indent=2) + "\n"
    if args.report:
        target = Path(args.report)
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_name(".%s.tmp" % target.name)
        temporary.write_text(rendered, encoding="utf-8")
        temporary.replace(target)
    sys.stdout.write(rendered)
    return 0 if report["status"] == "PASS" else (2 if report["status"] == "INCOMPLETE" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
