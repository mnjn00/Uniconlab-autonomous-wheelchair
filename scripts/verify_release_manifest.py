#!/usr/bin/env python3
"""Fail closed when a Noetic release manifest or its evidence is invalid."""

import argparse
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_release_manifest import (
    CATEGORIES, REQUIRED_RUNTIME_ENTRYPOINTS, SCHEMA, canonical_hash, file_hash,
)

HEX64 = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA = re.compile(r"^[0-9a-f]{40}$")
FALSE_AUTHORITY = (
    "hardware_motion_authorized", "passenger_operation_authorized",
    "physical_authority", "simulation_or_replay_is_physical_evidence",
)
VERIFIER_EXCLUDED_PARTS = {
    ".git", ".gjc", ".pytest_cache", ".mypy_cache", ".ruff_cache", ".tox",
    ".venv", "__pycache__", "artifacts", "build", "devel", "generated",
    "install", "logs", "log", "node_modules", "release_artifacts", "tmp", "temp",
}
VERIFIER_EXCLUDED_SUFFIXES = (".bag", ".db3", ".mcap", ".pyc", ".pyo", "~")
VERIFIER_GENERATED_NAMES = {"release-manifest.json"}
VERIFIER_REPORT_SCHEMAS = {
    ("algorithm-adversarial-test-report", 1): 2,
    ("simulation-test-report", 1): 2,
    ("test-report", 1): 0,
}
VERIFIER_CLAIMS = {"UNIT_ONLY": 0, "REPLAY_CONSISTENCY": 1, "SIMULATION_ONLY": 2}
VERIFIER_SURFACES = {
    "unit": 0, "unit_only": 0, "replay": 1, "replay_consistency": 1,
    "simulation": 2, "simulation_only": 2,
}


class ManifestError(ValueError):
    """The release bundle is incomplete, inconsistent, or unsafe."""


# Compatibility for callers/tests written before the verifier API was frozen.
VerificationError = ManifestError


def require(condition, message):
    if not condition:
        raise ManifestError(message)


def _load(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ManifestError("cannot read manifest: {}".format(exc)) from exc
    require(isinstance(value, dict), "manifest root must be an object")
    return value


def _verifier_excluded(relative):
    candidate = Path(relative)
    return (any(part in VERIFIER_EXCLUDED_PARTS for part in candidate.parts)
            or candidate.name in VERIFIER_GENERATED_NAMES
            or candidate.name.endswith(VERIFIER_EXCLUDED_SUFFIXES))


def _verifier_category(relative):
    candidate = Path(relative)
    parts, name, suffix = candidate.parts, candidate.name, candidate.suffix.lower()
    if not parts or _verifier_excluded(relative):
        return None
    if parts[0] == "src":
        if name in {"package.xml", "CMakeLists.txt", "setup.py", "setup.cfg"}:
            return "package_metadata"
        if any(part in {"msg", "action", "srv"} for part in parts):
            return "interfaces"
        if suffix == ".py" and any(part in {"scripts", "src"} for part in parts[2:]):
            return "python_runtime"
        if "config" in parts:
            return "configuration"
        if "launch" in parts:
            return "launch_configuration"
        if any(part in {"urdf", "worlds", "meshes", "models"} for part in parts):
            return "robot_assets"
        if "tests" in parts:
            return "qualification_tools"
        return "source_build_metadata"
    if parts[0] == "scripts":
        return "python_runtime" if suffix in {".py", ".sh"} else "source_build_metadata"
    if parts[0] == "contracts":
        return "contracts"
    if parts[0] == "data":
        return "routes" if "waypoint" in name.lower() or "route" in name.lower() else "maps"
    if parts[0] == "docs" or name == "LICENSE" or name.startswith("README"):
        return "operator_docs"
    if parts[0] in {".github", "tools"} or name.endswith(".lock"):
        return "ci_tools"
    if parts[0] == "tests":
        return "qualification_tools"
    if len(parts) == 1 and (
            name in {".catkin_workspace", ".dockerignore", ".gitignore",
                     ".gitattributes", "Dockerfile", "Makefile", "pyproject.toml",
                     "tox.ini"}
            or name.startswith(("requirements", "setup."))
            or suffix in {".cfg", ".ini", ".toml", ".lock"}):
        return "source_build_metadata"
    return None


def _expected_inventory(root, report_paths):
    expected = {category: [] for category in CATEGORIES}
    reports = set(report_paths)
    for candidate in root.rglob("*"):
        relative = candidate.relative_to(root).as_posix()
        if candidate.is_symlink():
            try:
                candidate.resolve(strict=True).relative_to(root)
            except (FileNotFoundError, ValueError) as exc:
                raise ManifestError("symlink escapes release root: " + relative) from exc
        if not candidate.is_file():
            continue
        if relative in reports:
            expected["qualification_evidence"].append(relative)
            continue
        if _verifier_excluded(relative):
            continue
        category = _verifier_category(relative)
        require(category is not None, "unclassified regular file: " + relative)
        expected[category].append(relative)
    for category, paths in expected.items():
        expected[category] = sorted(set(paths))
        require(expected[category], "required hash category is empty: " + category)
    return expected


def _report_has_failure(value):
    if isinstance(value, dict):
        for key, child in value.items():
            lowered = key.lower()
            if lowered in {"failed", "failure", "failures", "errors"}:
                if child not in (False, 0, None, "", []):
                    return True
            if lowered in {"passed", "pass"} and child is False:
                return True
            if lowered == "status" and isinstance(child, str):
                if child.upper() in {"FAIL", "FAILED", "ERROR", "PLATFORM_UNAVAILABLE"}:
                    return True
            if _report_has_failure(child):
                return True
    elif isinstance(value, list):
        return any(_report_has_failure(child) for child in value)
    return False


def _verify_json_report(path, relative, revision):
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ManifestError("invalid JSON test report: " + relative) from exc
    require(isinstance(document, dict), "JSON test report root must be an object: " + relative)
    report_type = document.get("artifactType", document.get("type"))
    version = document.get("schemaVersion", document.get("schema_version"))
    maximum = VERIFIER_REPORT_SCHEMAS.get((report_type, version))
    require(maximum is not None, "unallowlisted or missing JSON report type/schema: " + relative)
    require(document.get("status") in {"PASS", "passed"},
            "JSON test report does not record PASS: " + relative)
    for aliases in (
            ("hardwareMotionAuthorized", "hardware_motion_authorized"),
            ("passengerOperationAuthorized", "passenger_operation_authorized")):
        values = [document[key] for key in aliases if key in document]
        require(values and all(value is False for value in values),
                "JSON test report authority must remain false: " + relative)
    claim = document.get("claimTag", document.get("claim_tag"))
    require(claim in VERIFIER_CLAIMS, "JSON test report has invalid claim tag: " + relative)
    surface = document.get("surface")
    if surface is None:
        surface_level = maximum
    else:
        surface_level = VERIFIER_SURFACES.get(str(surface).lower())
        require(surface_level is not None, "JSON test report has invalid surface: " + relative)
    require(VERIFIER_CLAIMS[claim] <= min(maximum, surface_level),
            "JSON test report promotes its claim above its surface: " + relative)
    result = document.get("result")
    require(isinstance(result, dict) and result.get("passed") is True,
            "JSON test report contains a failed result: " + relative)
    require(not _report_has_failure(result),
            "JSON test report contains a failed result: " + relative)
    declared = document.get("source_revision", document.get("commit"))
    require(declared in (None, revision), "mixed source hash in report: " + relative)


def _verify_junit_report(path, relative):
    try:
        root = ET.parse(str(path)).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ManifestError("invalid JUnit test report: " + relative) from exc
    suites = [root] if root.tag == "testsuite" else list(root.findall("testsuite"))
    require(root.tag in {"testsuite", "testsuites"} and bool(suites),
            "test report is not JUnit XML: " + relative)
    require(not any(element.tag.rsplit("}", 1)[-1] in {"failure", "error", "skipped"}
                    for element in root.iter()),
            "JUnit report contains failed or skipped test cases: " + relative)
    total = 0
    for suite in suites:
        counts = {}
        for field in ("tests", "failures", "errors", "skipped"):
            raw = suite.get(field)
            require(isinstance(raw, str) and raw.isdigit(),
                    "JUnit report lacks valid {} count: {}".format(field, relative))
            counts[field] = int(raw)
        total += counts["tests"]
        require(not any(counts[field] for field in ("failures", "errors", "skipped")),
                "JUnit report is not a clean run: " + relative)
    require(total > 0, "JUnit report contains no tests: " + relative)


def _validate_entries(root, category, section, expected_paths, global_paths):
    require(isinstance(section, dict), "invalid hash category: " + category)
    entries = section.get("files")
    require(isinstance(entries, list) and entries, "empty hash category: " + category)
    require(all(isinstance(item, dict) for item in entries),
            "invalid hash entry: " + category)
    require(all(isinstance(item.get("path"), str) and item.get("path")
                for item in entries), "invalid path: " + category)
    require(entries == sorted(entries, key=lambda item: item.get("path", "")),
            "hash entries are not sorted: " + category)
    declared_paths = [entry.get("path") for entry in entries]
    require(declared_paths == expected_paths, "hash scope differs: " + category)
    for entry in entries:
        require(isinstance(entry, dict) and set(entry) == {"path", "sha256", "executable"},
                "invalid hash entry: " + category)
        path, expected = entry.get("path"), entry.get("sha256")
        executable = entry.get("executable")
        require(isinstance(path, str) and path, "invalid path: " + category)
        collision = path.casefold()
        require(collision not in global_paths,
                "duplicate or colliding inventory path: " + path)
        global_paths.add(collision)
        require(isinstance(expected, str) and HEX64.fullmatch(expected),
                "invalid sha256: " + path)
        require(isinstance(executable, bool), "invalid executable mode: " + path)
        candidate = root / path
        require(not candidate.is_symlink(), "bound file is a symlink: " + path)
        try:
            resolved = candidate.resolve(strict=True)
            resolved.relative_to(root)
        except (FileNotFoundError, ValueError) as exc:
            raise ManifestError("missing hashed file or path escapes release root: " + path) from exc
        require(resolved.is_file(), "missing hashed file: " + path)
        require(bool(candidate.stat().st_mode & 0o111) is executable,
                "executable mode mismatch: " + path)
        require(file_hash(candidate) == expected, "hash mismatch: " + path)
    require(section.get("digest") == canonical_hash(entries),
            "category digest mismatch: " + category)


def verify_manifest(path, root=None):
    path = Path(path).resolve()
    root = Path(root).resolve() if root else path.parent.resolve()
    manifest = _load(path)
    require(manifest.get("schema") == SCHEMA, "unsupported manifest schema")
    binding = manifest.get("release_binding_sha256")
    require(isinstance(binding, str) and HEX64.fullmatch(binding), "invalid release binding")
    unsigned = dict(manifest)
    unsigned.pop("release_binding_sha256", None)
    require(binding == canonical_hash(unsigned), "release binding mismatch")

    source = manifest.get("source")
    require(isinstance(source, dict), "missing source identity")
    require(source.get("kind") in {"git_commit", "worktree"}, "invalid source kind")
    revision = source.get("revision")
    require(isinstance(revision, str) and (GIT_SHA.fullmatch(revision) or revision.startswith("worktree:")), "invalid source revision")
    require(source.get("worktree_clean") is (source.get("kind") == "git_commit"), "unclean authority/source mismatch")

    hashes = manifest.get("hashes")
    require(isinstance(hashes, dict) and set(hashes) == set(CATEGORIES),
            "hash category inventory differs")
    reports = manifest.get("test_reports")
    require(isinstance(reports, list) and reports, "test-report evidence is missing")
    report_paths = []
    for report in reports:
        require(isinstance(report, dict), "invalid test-report reference")
        relative = report.get("path")
        require(isinstance(relative, str) and relative, "invalid test-report reference")
        report_paths.append(relative)
    require(report_paths == sorted(set(report_paths)), "duplicate or unsorted test reports")
    for relative in report_paths:
        candidate = root / relative
        try:
            resolved_report = candidate.resolve(strict=True)
            resolved_report.relative_to(root)
        except (FileNotFoundError, ValueError) as exc:
            raise ManifestError("missing test report or path escapes release root: " + relative) from exc
        require(resolved_report.is_file() and resolved_report.stat().st_size > 0,
                "missing test report: " + relative)
    for entrypoint in REQUIRED_RUNTIME_ENTRYPOINTS:
        require((root / entrypoint).is_file(),
                "required runtime entrypoint is missing: " + entrypoint)
    expected_inventory = _expected_inventory(root, report_paths)
    global_paths = set()
    for category in CATEGORIES:
        _validate_entries(
            root, category, hashes[category], expected_inventory[category], global_paths)
    for entrypoint in REQUIRED_RUNTIME_ENTRYPOINTS:
        require(entrypoint in expected_inventory["python_runtime"],
                "required runtime entrypoint is missing: " + entrypoint)

    authority = manifest.get("authority")
    require(isinstance(authority, dict) and authority.get("software_release_candidate") is True, "software RC authority is absent")
    require(authority.get("clean_release_authority") is True, "unclean release authority")
    for key in FALSE_AUTHORITY:
        require(authority.get(key) is False, "authority escalation: " + key)

    def reject_mixed_authority(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if (key in {"hardware_enabled", "passenger_enabled"}
                        or key.endswith("_authorized")
                        or (key.endswith("_authority")
                            and key not in {"software_release_authority",
                                           "clean_release_authority"})):
                    require(child is False, "mixed authority: " + key)
                reject_mixed_authority(child)
        elif isinstance(value, list):
            for child in value:
                reject_mixed_authority(child)

    reject_mixed_authority(manifest)
    qualification = manifest.get("qualification")
    require(isinstance(qualification, dict), "missing qualification status")
    for key in ("target_nuc", "hardware", "passenger"):
        require(qualification.get(key) == "blocked", key + " qualification must remain blocked")
    blockers = manifest.get("known_blockers")
    require(isinstance(blockers, list) and blockers and all(isinstance(item, str) and item for item in blockers), "known blockers are missing")

    evidence_entries = hashes["qualification_evidence"]["files"]
    evidence_by_path = {entry["path"]: entry for entry in evidence_entries}
    for report in reports:
        relative, expected = report.get("path"), report.get("sha256")
        executable = report.get("executable")
        require(isinstance(expected, str) and HEX64.fullmatch(expected)
                and isinstance(executable, bool), "invalid test-report reference")
        require(evidence_by_path.get(relative) == report,
                "test report is not bound to qualification evidence: " + relative)
        report_path = root / relative
        require(not report_path.is_symlink(), "test report is a symlink: " + relative)
        require(report_path.is_file() and report_path.stat().st_size > 0,
                "missing test report: " + relative)
        require(file_hash(report_path) == expected, "test report hash mismatch: " + relative)
        if report_path.suffix.lower() == ".json":
            _verify_json_report(report_path, relative, revision)
        elif report_path.suffix.lower() == ".xml":
            _verify_junit_report(report_path, relative)
        else:
            raise ManifestError("unsupported test report format: " + relative)

    rollback = manifest.get("rollback")
    require(isinstance(rollback, dict) and isinstance(rollback.get("parent"), str) and rollback["parent"], "rollback parent is missing")
    require(rollback.get("parent_state") == "unarmed", "rollback would restore an armed state")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--root", type=Path)
    args = parser.parse_args(argv)
    try:
        verify_manifest(args.manifest, args.root)
    except ManifestError as exc:
        parser.exit(2, "release manifest verification failed: {}\n".format(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
