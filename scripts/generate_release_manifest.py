#!/usr/bin/env python3
"""Generate a deterministic, hash-bound software release-candidate manifest."""

import argparse
import hashlib
import json
import os
import subprocess
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

SCHEMA = "wheelchair-noetic-release-manifest/v1"
CATEGORIES = (
    "source_build_metadata",
    "package_metadata",
    "interfaces",
    "python_runtime",
    "configuration",
    "launch_configuration",
    "robot_assets",
    "contracts",
    "maps",
    "routes",
    "operator_docs",
    "ci_tools",
    "qualification_tools",
    "qualification_evidence",
)
REQUIRED_RUNTIME_ENTRYPOINTS = (
    "src/wheelchair_safety/scripts/safety_gate.py",
    "src/wheelchair_safety/scripts/collision_supervisor.py",
    "src/wheelchair_safety/scripts/topology_guard.py",
    "src/wheelchair_gazebo/scripts/simulation_controller_adapter.py",
)
EXCLUDED_PARTS = {
    ".git", ".gjc", ".pytest_cache", ".mypy_cache", ".ruff_cache", ".tox",
    ".venv", "__pycache__", "artifacts", "build", "devel", "generated",
    "install", "logs", "log", "node_modules", "release_artifacts", "tmp", "temp",
}
EXCLUDED_SUFFIXES = (".bag", ".db3", ".mcap", ".pyc", ".pyo", "~")
GENERATED_ARTIFACT_NAMES = {"release-manifest.json"}
JSON_REPORT_SCHEMAS = {
    ("algorithm-adversarial-test-report", 1): "SIMULATION_ONLY",
    ("simulation-test-report", 1): "SIMULATION_ONLY",
    ("test-report", 1): "UNIT_ONLY",
}
CLAIM_LEVELS = {"UNIT_ONLY": 0, "REPLAY_CONSISTENCY": 1, "SIMULATION_ONLY": 2}
SURFACE_LEVELS = {
    "unit": 0, "unit_only": 0, "replay": 1, "replay_consistency": 1,
    "simulation": 2, "simulation_only": 2,
}
DEFAULT_BLOCKERS = [
    "target NUC qualification has not passed",
    "hardware motion qualification has not passed",
    "passenger operation qualification has not passed",
]


class ManifestError(ValueError):
    pass


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_hash(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _excluded(relative):
    path = Path(relative)
    return (any(part in EXCLUDED_PARTS for part in path.parts)
            or path.name in GENERATED_ARTIFACT_NAMES
            or path.name.endswith(EXCLUDED_SUFFIXES))


def _category(relative):
    path = Path(relative)
    parts = path.parts
    name = path.name
    suffix = path.suffix.lower()
    if not parts or _excluded(relative):
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
        if "waypoint" in name.lower() or "route" in name.lower():
            return "routes"
        return "maps"
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


def _report_authority_is_false(document):
    aliases = (
        ("hardwareMotionAuthorized", "hardware_motion_authorized"),
        ("passengerOperationAuthorized", "passenger_operation_authorized"),
    )
    return all(
        (values := [document[key] for key in keys if key in document])
        and all(value is False for value in values)
        for keys in aliases
    )


def _nested_failure(value):
    if isinstance(value, dict):
        for key, child in value.items():
            if key.lower() in {"failed", "failure", "failures", "errors"}:
                if child not in (False, 0, None, "", []):
                    return True
            if key.lower() in {"passed", "pass"} and child is False:
                return True
            if key.lower() == "status" and isinstance(child, str):
                if child.upper() in {"FAIL", "FAILED", "ERROR", "PLATFORM_UNAVAILABLE"}:
                    return True
            if _nested_failure(child):
                return True
    elif isinstance(value, list):
        return any(_nested_failure(child) for child in value)
    return False


def _validate_json_report(path):
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ManifestError("invalid JSON test report: " + str(path)) from exc
    if not isinstance(document, dict):
        raise ManifestError("JSON test report root must be an object: " + str(path))
    report_type = document.get("artifactType", document.get("type"))
    schema_version = document.get("schemaVersion", document.get("schema_version"))
    maximum = JSON_REPORT_SCHEMAS.get((report_type, schema_version))
    if maximum is None:
        raise ManifestError("unallowlisted or missing JSON report type/schema: " + str(path))
    status = document.get("status")
    if status not in {"PASS", "passed"}:
        raise ManifestError("JSON test report does not record PASS: " + str(path))
    if not _report_authority_is_false(document):
        raise ManifestError("JSON test report authority must remain false: " + str(path))
    claim = document.get("claimTag", document.get("claim_tag"))
    surface = document.get("surface")
    if claim not in CLAIM_LEVELS:
        raise ManifestError("JSON test report has invalid claim tag: " + str(path))
    surface_level = SURFACE_LEVELS.get(str(surface).lower(), CLAIM_LEVELS[maximum])
    if str(surface).lower() not in SURFACE_LEVELS and surface is not None:
        raise ManifestError("JSON test report has invalid surface: " + str(path))
    if CLAIM_LEVELS[claim] > min(CLAIM_LEVELS[maximum], surface_level):
        raise ManifestError("JSON test report promotes its claim above its surface: " + str(path))
    result = document.get("result")
    if not isinstance(result, dict) or result.get("passed") is not True:
        raise ManifestError("JSON test report lacks a passing result: " + str(path))
    if _nested_failure(result):
        raise ManifestError("JSON test report contains a failed result: " + str(path))


def _validate_junit_report(path):
    try:
        root = ET.parse(str(path)).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ManifestError("invalid JUnit test report: " + str(path)) from exc
    suites = [root] if root.tag == "testsuite" else list(root.findall("testsuite"))
    if root.tag not in {"testsuite", "testsuites"} or not suites:
        raise ManifestError("test report is not JUnit XML: " + str(path))
    if any(element.tag.rsplit("}", 1)[-1] in {"failure", "error", "skipped"}
           for element in root.iter()):
        raise ManifestError("JUnit report contains failed or skipped test cases: " + str(path))
    total = 0
    for suite in suites:
        counts = {}
        for field in ("tests", "failures", "errors", "skipped"):
            raw = suite.get(field)
            try:
                counts[field] = int(raw)
            except (TypeError, ValueError) as exc:
                raise ManifestError(
                    "JUnit report lacks valid {} count: {}".format(field, path)) from exc
            if counts[field] < 0:
                raise ManifestError("JUnit report contains a negative count: " + str(path))
        total += counts["tests"]
        if any(counts[field] for field in ("failures", "errors", "skipped")):
            raise ManifestError("JUnit report is not a clean run: " + str(path))
    if total == 0:
        raise ManifestError("JUnit report contains no tests: " + str(path))


def validate_report(path):
    if path.suffix.lower() == ".json":
        _validate_json_report(path)
    elif path.suffix.lower() == ".xml":
        _validate_junit_report(path)
    else:
        raise ManifestError("unsupported test report format: " + str(path))


def _safe_file(root, relative):
    path = root / relative
    if path.is_symlink():
        raise ManifestError("inventory entry is a symlink: " + relative)
    try:
        resolved = path.resolve(strict=True)
        resolved.relative_to(root)
    except (FileNotFoundError, ValueError) as exc:
        raise ManifestError("file is missing or escapes release root: " + relative) from exc
    if not resolved.is_file():
        raise ManifestError("inventory entry is not a regular file: " + relative)
    return path


def inventory_paths(root, report_paths):
    root = Path(root).resolve()
    result = {category: [] for category in CATEGORIES}
    reports = set(report_paths)
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if relative not in reports and _excluded(relative):
            continue
        if path.is_symlink():
            try:
                path.resolve(strict=True).relative_to(root)
            except (FileNotFoundError, ValueError) as exc:
                raise ManifestError("symlink escapes release root: " + relative) from exc
        if not path.is_file():
            continue
        if relative in reports:
            result["qualification_evidence"].append(relative)
            continue
        category = _category(relative)
        if category is None:
            raise ManifestError("unclassified regular file: " + relative)
        result[category].append(relative)
    for category in result:
        result[category] = sorted(set(result[category]))

    seen = {}
    for category, paths in result.items():
        if not paths:
            raise ManifestError("required hash category is empty: " + category)
        for relative in paths:
            collision = relative.casefold()
            if collision in seen:
                raise ManifestError(
                    "duplicate or colliding inventory path: {} and {}".format(
                        seen[collision], relative))
            seen[collision] = relative
            _safe_file(root, relative)
    for relative in REQUIRED_RUNTIME_ENTRYPOINTS:
        if relative not in result["python_runtime"]:
            raise ManifestError("required runtime entrypoint is missing: " + relative)
        if not ((root / relative).stat().st_mode & 0o111):
            raise ManifestError("required runtime entrypoint is not executable: " + relative)
    return result


def source_identity(root):
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "--verify", "HEAD"], cwd=str(root), check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        ).stdout.strip()
        dirty = bool(subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=all"], cwd=str(root), check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        ).stdout)
        if not dirty:
            return {"kind": "git_commit", "revision": commit, "worktree_clean": True}
    except (OSError, subprocess.CalledProcessError):
        commit = None

    inventory = []
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root).as_posix()
        if _excluded(relative):
            continue
        if path.is_symlink():
            try:
                path.resolve(strict=True).relative_to(root)
            except (FileNotFoundError, ValueError) as exc:
                raise ManifestError("symlink escapes release root: " + relative) from exc
        if not path.is_file():
            continue
        inventory.append({"path": relative, "sha256": file_hash(path)})
    return {
        "kind": "worktree",
        "revision": "worktree:" + canonical_hash(inventory),
        "worktree_clean": False,
    }


def _inventory(root, report_paths):
    result = {}
    for category, paths in inventory_paths(root, report_paths).items():
        entries = []
        for relative in paths:
            path = _safe_file(root, relative)
            entries.append({
                "path": relative,
                "sha256": file_hash(path),
                "executable": bool(path.stat().st_mode & 0o111),
            })
        result[category] = {"digest": canonical_hash(entries), "files": entries}
    return result


def generate_manifest(root, reports, rollback_parent, blockers=None, source=None):
    root = Path(root).resolve()
    if not rollback_parent or rollback_parent.lower() in {"armed", "hardware_armed", "none"}:
        raise ManifestError("rollback parent must identify a known unarmed parent")
    report_entries = []
    report_paths = set()
    for item in sorted((Path(item) for item in reports), key=lambda item: item.as_posix()):
        report = item if item.is_absolute() else root / item
        try:
            relative = report.relative_to(root).as_posix()
        except ValueError as exc:
            raise ManifestError("test reports must be inside the release root: " + str(report)) from exc
        report = _safe_file(root, relative)
        if report.stat().st_size == 0:
            raise ManifestError("missing or empty test report: " + str(report))
        validate_report(report)
        if relative in report_paths:
            raise ManifestError("duplicate test report: " + relative)
        report_paths.add(relative)
        report_entries.append({
            "path": relative,
            "sha256": file_hash(report),
            "executable": bool(report.stat().st_mode & 0o111),
        })
    if not report_entries:
        raise ManifestError("at least one test report is required")
    known_blockers = sorted(set(blockers or DEFAULT_BLOCKERS))
    if not known_blockers:
        raise ManifestError("known blockers must be recorded")

    manifest = {
        "schema": SCHEMA,
        "source": source or source_identity(root),
        "hashes": _inventory(root, report_paths),
        "authority": {
            "software_release_candidate": True,
            "clean_release_authority": True,
            "hardware_motion_authorized": False,
            "passenger_operation_authorized": False,
            "physical_authority": False,
            "simulation_or_replay_is_physical_evidence": False,
        },
        "qualification": {
            "target_nuc": "blocked",
            "hardware": "blocked",
            "passenger": "blocked",
        },
        "test_reports": report_entries,
        "known_blockers": known_blockers,
        "rollback": {"parent": rollback_parent, "parent_state": "unarmed"},
    }
    manifest["release_binding_sha256"] = canonical_hash(manifest)
    return manifest


def atomic_write(path, manifest):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(manifest, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
    descriptor, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=Path(__file__).resolve().parents[1], type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", action="append", required=True, type=Path)
    parser.add_argument("--rollback-parent", required=True)
    parser.add_argument("--blocker", action="append")
    args = parser.parse_args(argv)
    try:
        manifest = generate_manifest(args.root, args.report, args.rollback_parent, args.blocker)
        atomic_write(args.output, manifest)
    except (ManifestError, OSError) as exc:
        parser.exit(2, "release manifest generation failed: {}\n".format(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
