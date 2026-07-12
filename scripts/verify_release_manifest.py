#!/usr/bin/env python3
"""Independently verify a deterministic, fail-closed software release manifest."""

import argparse
import json
import re
from pathlib import Path

from generate_release_manifest import (
    CATEGORIES, GATE_CLAIM, GATE_REPORT_SCHEMA, GENERATED_ARTIFACT_NAMES,
    REQUIRED_GATES, REQUIRED_RUNTIME_ENTRYPOINTS, SCHEMA, canonical_hash, file_hash,
)

HEX64 = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA = re.compile(r"^[0-9a-f]{40}$")


class ManifestError(ValueError):
    """The release bundle is incomplete, inconsistent, or unsafe."""


VerificationError = ManifestError


def require(condition, message):
    if not condition:
        raise ManifestError(message)


def _root(root):
    root = Path(root).absolute()
    require(not root.is_symlink() and root.is_dir(), "release root must be a non-symlink directory")
    return root


def _safe_file(root, relative, label="file"):
    require(isinstance(relative, str) and relative and not Path(relative).is_absolute(),
            "invalid {} path".format(label))
    candidate = root / relative
    try:
        parts = candidate.relative_to(root).parts
    except ValueError as exc:
        raise ManifestError("{} path escapes release root: {}".format(label, relative)) from exc
    current = root
    for part in parts:
        current /= part
        require(not current.is_symlink(), "symlink is forbidden in release path: " + relative)
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root.resolve(strict=True))
    except (FileNotFoundError, ValueError) as exc:
        raise ManifestError("missing {} or path escapes release root: {}".format(label, relative)) from exc
    require(resolved.is_file(), "missing {}: {}".format(label, relative))
    return candidate


def _load(path):
    path = Path(path).absolute()
    require(not path.is_symlink(), "manifest is a symlink")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ManifestError("cannot read manifest: {}".format(exc)) from exc
    require(isinstance(value, dict), "manifest root must be an object")
    return value


def _excluded(relative):
    candidate = Path(relative)
    parts = candidate.parts
    excluded = {".git", ".gjc", ".pytest_cache", ".mypy_cache", ".ruff_cache", ".tox", ".venv",
                "__pycache__", "artifacts", "build", "devel", "generated", "install", "logs", "log",
                "node_modules", "release_artifacts", "tmp", "temp"}
    return (any(part in excluded for part in parts) or relative in GENERATED_ARTIFACT_NAMES
            or candidate.name.endswith((".bag", ".db3", ".mcap", ".pyc", ".pyo", "~")))


def _category(relative):
    candidate = Path(relative)
    parts, name, suffix = candidate.parts, candidate.name, candidate.suffix.lower()
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
        return "routes" if "waypoint" in name.lower() or "route" in name.lower() else "maps"
    if parts[0] == "docs" or name == "LICENSE" or name.startswith("README"):
        return "operator_docs"
    if parts[0] in {".github", "tools"} or name.endswith(".lock"):
        return "ci_tools"
    if parts[0] == "tests":
        return "qualification_tools"
    if len(parts) == 1 and (name in {".catkin_workspace", ".dockerignore", ".gitignore",
                                    ".gitattributes", "Dockerfile", "Makefile", "pyproject.toml", "tox.ini"}
                            or name.startswith(("requirements", "setup."))
                            or suffix in {".cfg", ".ini", ".toml", ".lock"}):
        return "source_build_metadata"
    return None


def _expected_inventory(root, report_paths):
    expected = {category: [] for category in CATEGORIES}
    report_paths = set(report_paths)
    for candidate in root.rglob("*"):
        relative = candidate.relative_to(root).as_posix()
        if _excluded(relative):
            continue
        require(not candidate.is_symlink(), "symlink is forbidden in release path: " + relative)
        if not candidate.is_file():
            continue
        if relative in report_paths:
            expected["qualification_evidence"].append(relative)
        else:
            category = _category(relative)
            require(category is not None, "unclassified regular file: " + relative)
            expected[category].append(relative)
    for category in CATEGORIES:
        expected[category] = sorted(set(expected[category]))
        require(expected[category], "required hash category is empty: " + category)
    return expected


def _validate_inventory(root, hashes, report_paths):
    require(isinstance(hashes, dict) and set(hashes) == set(CATEGORIES),
            "hash category inventory differs")
    for entrypoint in REQUIRED_RUNTIME_ENTRYPOINTS:
        candidate = _safe_file(root, entrypoint, "required runtime entrypoint")
        require(bool(candidate.stat().st_mode & 0o111),
                "required runtime entrypoint is not executable: " + entrypoint)
    expected = _expected_inventory(root, report_paths)
    seen = set()
    digests = {}
    for category in CATEGORIES:
        section = hashes[category]
        require(isinstance(section, dict) and set(section) == {"digest", "files"},
                "invalid hash category: " + category)
        entries = section["files"]
        require(isinstance(entries, list) and entries, "empty hash category: " + category)
        require(entries == sorted(entries, key=lambda entry: entry.get("path", "") if isinstance(entry, dict) else ""),
                "hash entries are not sorted: " + category)
        require([entry.get("path") if isinstance(entry, dict) else None for entry in entries] == expected[category],
                "hash scope differs: " + category)
        for entry in entries:
            require(isinstance(entry, dict) and set(entry) == {"path", "sha256", "executable"},
                    "invalid hash entry: " + category)
            relative, digest, executable = entry["path"], entry["sha256"], entry["executable"]
            require(isinstance(digest, str) and HEX64.fullmatch(digest) and isinstance(executable, bool),
                    "invalid hash entry: " + category)
            require(relative.casefold() not in seen, "duplicate or colliding inventory path: " + relative)
            seen.add(relative.casefold())
            candidate = _safe_file(root, relative, "hashed file")
            require(bool(candidate.stat().st_mode & 0o111) is executable,
                    "executable mode mismatch: " + relative)
            require(file_hash(candidate) == digest, "hash mismatch: " + relative)
        require(section["digest"] == canonical_hash(entries), "category digest mismatch: " + category)
        digests[category] = section["digest"]
    for entrypoint in REQUIRED_RUNTIME_ENTRYPOINTS:
        require(entrypoint in expected["python_runtime"],
                "required runtime entrypoint is missing: " + entrypoint)
    return digests


def _validate_source(source):
    require(isinstance(source, dict) and set(source) == {"kind", "revision", "worktree_clean"},
            "invalid source identity")
    require(source["kind"] in {"git_commit", "worktree"} and isinstance(source["revision"], str),
            "invalid source identity")
    require((source["kind"] == "git_commit" and GIT_SHA.fullmatch(source["revision"]))
            or (source["kind"] == "worktree" and source["revision"].startswith("worktree:")),
            "invalid source revision")
    require(source["worktree_clean"] is (source["kind"] == "git_commit"), "unclean authority/source mismatch")


def _expected_bindings(source, digests):
    release_input = canonical_hash({category: digests[category] for category in CATEGORIES
                                    if category != "qualification_evidence"})
    bundle = canonical_hash({"source_revision": source["revision"],
                             "configuration_digest": digests["configuration"],
                             "release_input_digest": release_input})
    return {"sourceRevision": source["revision"], "configurationDigest": digests["configuration"],
            "bundleDigest": bundle, "releaseInputDigest": release_input}


def _validate_gate(path, relative, bindings):
    try:
        report = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ManifestError("invalid gate report: " + relative) from exc
    keys = {"artifactType", "schemaVersion", "gateId", "status", "claimTag",
            "hardwareMotionAuthorized", "passengerOperationAuthorized", "sourceRevision",
            "configurationDigest", "bundleDigest", "releaseInputDigest", "result"}
    require(isinstance(report, dict) and set(report) == keys, "gate report has non-strict schema: " + relative)
    require((report["artifactType"], report["schemaVersion"]) == GATE_REPORT_SCHEMA,
            "gate report has invalid type/schema: " + relative)
    require(REQUIRED_GATES.get(report["gateId"]) == relative, "gate report ID/path does not match AC matrix: " + relative)
    require(report["status"] == "PASS" and report["claimTag"] == GATE_CLAIM,
            "gate report is not a passing software-only claim: " + relative)
    require(report["hardwareMotionAuthorized"] is False and report["passengerOperationAuthorized"] is False,
            "gate report authority must remain false: " + relative)
    require(all(report[key] == value for key, value in bindings.items()),
            "gate report has stale or mixed release binding: " + relative)
    result = report["result"]
    require(isinstance(result, dict) and set(result) == {"passed", "cases"} and result["passed"] is True
            and isinstance(result["cases"], int) and not isinstance(result["cases"], bool) and result["cases"] > 0,
            "gate report has invalid or trivial result: " + relative)
    return report["gateId"]


def _validate_rollback(rollback):
    keys = {"parentReleaseBindingSha256", "parentReleaseBindingReceiptSha256", "inventory", "restartReceipt"}
    require(isinstance(rollback, dict) and set(rollback) == keys, "rollback binding has non-strict schema")
    binding, receipt = rollback["parentReleaseBindingSha256"], rollback["restartReceipt"]
    receipt_hash, inventory = rollback["parentReleaseBindingReceiptSha256"], rollback["inventory"]
    require(isinstance(binding, str) and HEX64.fullmatch(binding) and isinstance(receipt_hash, str) and HEX64.fullmatch(receipt_hash),
            "rollback parent digest/receipt is invalid")
    kinds = {"binaries", "maps", "routes", "policies", "drivers"}
    require(isinstance(inventory, dict) and set(inventory) == kinds, "rollback cross-hash inventory is incomplete")
    paths = set()
    for kind in sorted(kinds):
        entries = inventory[kind]
        require(isinstance(entries, list) and entries and entries == sorted(entries, key=lambda entry: entry.get("path", "") if isinstance(entry, dict) else ""),
                "rollback cross-hash inventory is incomplete or unordered: " + kind)
        for entry in entries:
            require(isinstance(entry, dict) and set(entry) == {"path", "sha256"} and isinstance(entry["path"], str)
                    and entry["path"] and isinstance(entry["sha256"], str) and HEX64.fullmatch(entry["sha256"]),
                    "rollback cross-hash inventory entry is invalid: " + kind)
            require(entry["path"] not in paths, "rollback cross-hash inventory duplicates a path")
            paths.add(entry["path"])
    # This is a deterministic hash-bound receipt, not asymmetric cryptographic signature verification.
    require(receipt_hash == canonical_hash({"parentReleaseBindingSha256": binding, "inventory": inventory}),
            "rollback parent hash-bound receipt does not bind inventory")
    receipt_keys = {"state", "permissions", "localizationRequired", "missionResume", "parentReleaseBindingSha256", "inventoryDigest"}
    require(isinstance(receipt, dict) and set(receipt) == receipt_keys, "rollback restart receipt has non-strict schema")
    require(receipt["state"] == "DISARMED" and receipt["permissions"] == "UNKNOWN"
            and receipt["localizationRequired"] is True and receipt["missionResume"] is False
            and receipt["parentReleaseBindingSha256"] == binding
            and receipt["inventoryDigest"] == canonical_hash(inventory),
            "rollback restart receipt does not prove a disarmed matching restart")


def verify_manifest(path, root=None):
    manifest_path = Path(path).absolute()
    root = _root(root if root else manifest_path.parent)
    try:
        manifest_relative = manifest_path.relative_to(root).as_posix()
    except ValueError as exc:
        raise ManifestError("manifest path escapes release root") from exc
    _safe_file(root, manifest_relative, "manifest")
    manifest = _load(manifest_path)
    keys = {"schema", "source", "hashes", "gate_matrix", "authority", "qualification", "test_reports",
            "known_blockers", "rollback", "release_binding_sha256"}
    require(set(manifest) == keys and manifest["schema"] == SCHEMA, "unsupported or non-strict manifest schema")
    binding = manifest["release_binding_sha256"]
    require(isinstance(binding, str) and HEX64.fullmatch(binding), "invalid release binding")
    unsigned = dict(manifest)
    unsigned.pop("release_binding_sha256")
    require(binding == canonical_hash(unsigned), "release binding mismatch")
    _validate_source(manifest["source"])

    reports = manifest["test_reports"]
    require(isinstance(reports, list) and reports, "test-report evidence is missing")
    expected_paths = sorted(REQUIRED_GATES.values())
    require(len(reports) == len(expected_paths), "AC gate matrix is incomplete")
    require([entry.get("path") if isinstance(entry, dict) else None for entry in reports] == expected_paths,
            "test reports do not match the AC gate matrix")
    for entry in reports:
        require(isinstance(entry, dict) and set(entry) == {"path", "sha256", "executable"}
                and isinstance(entry["sha256"], str) and HEX64.fullmatch(entry["sha256"])
                and entry["executable"] is False, "invalid test-report reference")
        candidate = _safe_file(root, entry["path"], "test report")
        require(candidate.stat().st_size > 0 and file_hash(candidate) == entry["sha256"],
                "test report hash mismatch: " + entry["path"])

    digests = _validate_inventory(root, manifest["hashes"], expected_paths)
    evidence = {entry["path"]: entry for entry in manifest["hashes"]["qualification_evidence"]["files"]}
    require(evidence == {entry["path"]: entry for entry in reports}, "test reports are not bound to qualification evidence")
    bindings = _expected_bindings(manifest["source"], digests)
    matrix = manifest["gate_matrix"]
    require(isinstance(matrix, dict) and set(matrix) == {"requiredGateIds", "passedGateIds", "releaseBindings"}
            and matrix["requiredGateIds"] == sorted(REQUIRED_GATES)
            and matrix["passedGateIds"] == sorted(REQUIRED_GATES)
            and matrix["releaseBindings"] == bindings, "AC gate matrix is incomplete or stale")
    gate_ids = {_validate_gate(_safe_file(root, relative, "gate report"), relative, bindings) for relative in expected_paths}
    require(gate_ids == set(REQUIRED_GATES), "AC gate IDs are incomplete or duplicated")

    authority = manifest["authority"]
    expected_authority = {"software_release_candidate": True, "clean_release_authority": True,
                          "hardware_motion_authorized": False, "passenger_operation_authorized": False,
                          "physical_authority": False, "simulation_or_replay_is_physical_evidence": False}
    require(authority == expected_authority, "authority escalation or incomplete aggregate authority")
    require(manifest["qualification"] == {"target_nuc": "passed", "hardware": "blocked", "passenger": "blocked"},
            "qualification status contradicts the complete gate matrix")
    blockers = manifest["known_blockers"]
    require(isinstance(blockers, list) and blockers and blockers == sorted(set(blockers))
            and all(isinstance(item, str) and item for item in blockers)
            and "target NUC qualification has not passed" not in blockers
            and {"hardware motion qualification has not passed",
                 "passenger operation qualification has not passed"}.issubset(blockers),
            "known blockers contradict qualification or authority")
    _validate_rollback(manifest["rollback"])
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
