#!/usr/bin/env python3
"""Generate a deterministic, fail-closed software release-candidate manifest."""

import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path

SCHEMA = "wheelchair-noetic-release-manifest/v1"
CATEGORIES = (
    "source_build_metadata", "package_metadata", "interfaces", "python_runtime",
    "configuration", "launch_configuration", "robot_assets", "contracts", "maps",
    "routes", "operator_docs", "ci_tools", "qualification_tools",
    "qualification_evidence",
)
REQUIRED_RUNTIME_ENTRYPOINTS = (
    "src/wheelchair_safety/scripts/safety_gate.py",
    "src/wheelchair_safety/scripts/collision_supervisor.py",
    "src/wheelchair_safety/scripts/topology_guard.py",
    "src/wheelchair_gazebo/scripts/simulation_controller_adapter.py",
)
EXCLUDED_PARTS = {
    ".git", ".gjc", ".pytest_cache", ".mypy_cache", ".ruff_cache", ".tox", ".venv",
    "__pycache__", "artifacts", "build", "devel", "generated", "install", "logs", "log",
    "node_modules", "release_artifacts", "tmp", "temp",
}
EXCLUDED_SUFFIXES = (".bag", ".db3", ".mcap", ".pyc", ".pyo", "~")
GENERATED_ARTIFACT_NAMES = {"release-manifest.json", "release-bindings.json"}
HEX64 = re.compile(r"^[0-9a-f]{64}$")
GATE_REPORT_SCHEMA = ("wheelchair-ac-gate-report", 1)
GATE_CLAIM = "SOFTWARE_ONLY"
# WP7 is deliberately absent: it is separately authorized and cannot be claimed here.
REQUIRED_GATES = {
    "WP0-ABI-001": "evidence/contracts/abi-v1-report.json",
    "WP1-TOPOLOGY-001": "evidence/topology/command-graph-report.json",
    "WP1-GEOFENCE-001": "evidence/route-safety/anti-widening-report.json",
    "WP1-COLLISION-001": "evidence/safety/collision-ttc-report.json",
    "WP1-SLOPE-001": "evidence/safety/slope-policy-report.json",
    "WP3-LOCALIZATION-001": "evidence/localization/confidence-holdout-report.json",
    "WP2-CONVERSION-001": "evidence/conversion/determinism-and-corruption-report.json",
    "WP4-MISSION-001": "evidence/mission/fsm-contract-report.json",
    "WP1-CONTROL-001": "evidence/safety/gate-permission-matrix.json",
    "WP6-TIMING-001": "evidence/performance/target-nuc-60min-report.json",
    "WP6-SIMCLAIM-001": "evidence/simulation/fidelity-claim-report.json",
    "WP6-ROLLBACK-001": "evidence/release/rollback-drill-report.json",
    "WP0-HWGATE-NEG-001": "evidence/hardware/hardware-gate-negative-report.json",
    "WP0-PASSENGER-NEG-001": "evidence/release/passenger-authority-negative-report.json",
}
DEFAULT_BLOCKERS = [
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
            or relative in GENERATED_ARTIFACT_NAMES or path.name.endswith(EXCLUDED_SUFFIXES))


def _category(relative):
    path = Path(relative)
    parts, name, suffix = path.parts, path.name, path.suffix.lower()
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
                                    ".gitattributes", "Dockerfile", "Makefile", "pyproject.toml",
                                    "tox.ini"} or name.startswith(("requirements", "setup."))
                            or suffix in {".cfg", ".ini", ".toml", ".lock"}):
        return "source_build_metadata"
    return None


def _root(root):
    root = Path(root).absolute()
    if root.is_symlink() or not root.is_dir():
        raise ManifestError("release root must be a non-symlink directory")
    return root


def _safe_file(root, relative):
    candidate = root / relative
    try:
        relative_path = candidate.relative_to(root)
    except ValueError as exc:
        raise ManifestError("file escapes release root: " + str(relative)) from exc
    current = root
    for part in relative_path.parts:
        current /= part
        if current.is_symlink():
            raise ManifestError("symlink is forbidden in release path: " + relative_path.as_posix())
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root.resolve(strict=True))
    except (FileNotFoundError, ValueError) as exc:
        raise ManifestError("file is missing or escapes release root: " + relative_path.as_posix()) from exc
    if not resolved.is_file():
        raise ManifestError("inventory entry is not a regular file: " + relative_path.as_posix())
    return candidate


def inventory_paths(root, report_paths, require_evidence=True):
    root = _root(root)
    reports = set(report_paths)
    result = {category: [] for category in CATEGORIES}
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if _excluded(relative):
            continue
        if path.is_symlink():
            raise ManifestError("symlink is forbidden in release path: " + relative)
        if relative in reports:
            if path.is_file() and require_evidence:
                result["qualification_evidence"].append(relative)
            continue
        if not path.is_file():
            continue
        category = _category(relative)
        if category is None:
            raise ManifestError("unclassified regular file: " + relative)
        result[category].append(relative)
    for category, paths in result.items():
        result[category] = sorted(set(paths))
        if not result[category] and (require_evidence or category != "qualification_evidence"):
            raise ManifestError("required hash category is empty: " + category)
        for relative in result[category]:
            _safe_file(root, relative)
    for relative in REQUIRED_RUNTIME_ENTRYPOINTS:
        path = _safe_file(root, relative)
        if relative not in result["python_runtime"] or not (path.stat().st_mode & 0o111):
            raise ManifestError("required runtime entrypoint is missing or not executable: " + relative)
    return result


def _generated_path(relative):
    return relative in set(REQUIRED_GATES.values()) or relative in GENERATED_ARTIFACT_NAMES


def _git_dirty_paths(root):
    try:
        output = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=all", "-z"], cwd=str(root),
            check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    records = output.decode("utf-8", "surrogateescape").split("\0")
    paths = []
    index = 0
    while index < len(records) - 1:
        record = records[index]
        if len(record) < 4 or record[2] != " ":
            return ["<invalid git status record>"]
        status, relative = record[:2], record[3:]
        if status[0] in {"R", "C"} or status[1] in {"R", "C"}:
            return ["<rename-or-copy>"]
        paths.append(relative)
        index += 1
    return paths


def source_identity(root):
    root = _root(root)
    try:
        commit = subprocess.run(["git", "rev-parse", "--verify", "HEAD"], cwd=str(root), check=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True).stdout.strip()
        dirty_paths = _git_dirty_paths(root)
        if dirty_paths is not None and all(_generated_path(path) for path in dirty_paths):
            return {"kind": "git_commit", "revision": commit, "worktree_clean": True}
    except (OSError, subprocess.CalledProcessError):
        pass
    inventory = []
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root).as_posix()
        if _excluded(relative) or _generated_path(relative):
            continue
        if path.is_symlink():
            raise ManifestError("symlink is forbidden in release path: " + relative)
        if path.is_file():
            inventory.append({"path": relative, "sha256": file_hash(path)})
    return {"kind": "worktree", "revision": "worktree:" + canonical_hash(inventory), "worktree_clean": False}


def _inventory(root, report_paths, require_evidence=True):
    result = {}
    for category, paths in inventory_paths(root, report_paths, require_evidence).items():
        entries = [{"path": relative, "sha256": file_hash(_safe_file(root, relative)),
                    "executable": bool(_safe_file(root, relative).stat().st_mode & 0o111)}
                   for relative in paths]
        result[category] = {"digest": canonical_hash(entries), "files": entries}
    return result


def prepare_bindings(root, source=None):
    root = _root(root)
    source = source or source_identity(root)
    if not isinstance(source, dict) or not isinstance(source.get("revision"), str) or not source["revision"]:
        raise ManifestError("source identity lacks a revision")
    hashes = _inventory(root, set(REQUIRED_GATES.values()), require_evidence=False)
    return _bind_inputs(source, hashes)


def _bind_inputs(source, hashes):
    configuration_digest = hashes["configuration"]["digest"]
    release_input_digest = canonical_hash({category: hashes[category]["digest"] for category in CATEGORIES
                                           if category != "qualification_evidence"})
    bundle_digest = canonical_hash({"source_revision": source["revision"],
                                    "configuration_digest": configuration_digest,
                                    "release_input_digest": release_input_digest})
    return {"sourceRevision": source["revision"], "configurationDigest": configuration_digest,
            "bundleDigest": bundle_digest, "releaseInputDigest": release_input_digest}


def _validate_gate_report(path, relative, bindings):
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ManifestError("invalid gate report: " + relative) from exc
    required = {"artifactType", "schemaVersion", "gateId", "status", "claimTag",
                "hardwareMotionAuthorized", "passengerOperationAuthorized", "sourceRevision",
                "configurationDigest", "bundleDigest", "releaseInputDigest", "result"}
    if not isinstance(document, dict) or set(document) != required:
        raise ManifestError("gate report has non-strict schema: " + relative)
    if (document["artifactType"], document["schemaVersion"]) != GATE_REPORT_SCHEMA:
        raise ManifestError("gate report has invalid type/schema: " + relative)
    gate_id = document["gateId"]
    if REQUIRED_GATES.get(gate_id) != relative:
        raise ManifestError("gate report ID/path does not match AC matrix: " + relative)
    if document["status"] != "PASS" or document["claimTag"] != GATE_CLAIM:
        raise ManifestError("gate report is not a passing software-only claim: " + relative)
    if document["hardwareMotionAuthorized"] is not False or document["passengerOperationAuthorized"] is not False:
        raise ManifestError("gate report authority must remain false: " + relative)
    if any(document[key] != value for key, value in bindings.items()):
        raise ManifestError("gate report has stale or mixed release binding: " + relative)
    result = document["result"]
    if not isinstance(result, dict) or set(result) != {"passed", "cases"} or result["passed"] is not True:
        raise ManifestError("gate report has invalid result: " + relative)
    if not isinstance(result["cases"], int) or isinstance(result["cases"], bool) or result["cases"] <= 0:
        raise ManifestError("gate report is trivial: " + relative)
    return gate_id


def _validate_rollback(rollback):
    required = {"parentReleaseBindingSha256", "parentReleaseBindingReceiptSha256", "inventory", "restartReceipt"}
    if not isinstance(rollback, dict) or set(rollback) != required:
        raise ManifestError("rollback binding has non-strict schema")
    binding = rollback["parentReleaseBindingSha256"]
    receipt_hash = rollback["parentReleaseBindingReceiptSha256"]
    inventory = rollback["inventory"]
    if not isinstance(binding, str) or not HEX64.fullmatch(binding) or not isinstance(receipt_hash, str) or not HEX64.fullmatch(receipt_hash):
        raise ManifestError("rollback parent digest/receipt is invalid")
    kinds = {"binaries", "maps", "routes", "policies", "drivers"}
    if not isinstance(inventory, dict) or set(inventory) != kinds:
        raise ManifestError("rollback cross-hash inventory is incomplete")
    all_paths = set()
    for kind in sorted(kinds):
        entries = inventory[kind]
        if not isinstance(entries, list) or not entries:
            raise ManifestError("rollback cross-hash inventory is incomplete: " + kind)
        if entries != sorted(entries, key=lambda item: item.get("path", "") if isinstance(item, dict) else ""):
            raise ManifestError("rollback cross-hash inventory is not deterministic: " + kind)
        for entry in entries:
            if not isinstance(entry, dict) or set(entry) != {"path", "sha256"} or not isinstance(entry["path"], str) or not entry["path"] or not isinstance(entry["sha256"], str) or not HEX64.fullmatch(entry["sha256"]):
                raise ManifestError("rollback cross-hash inventory entry is invalid: " + kind)
            if entry["path"] in all_paths:
                raise ManifestError("rollback cross-hash inventory duplicates a path")
            all_paths.add(entry["path"])
    signed = {"parentReleaseBindingSha256": binding, "inventory": inventory}
    if receipt_hash != canonical_hash(signed):
        raise ManifestError("rollback parent hash-bound receipt does not bind inventory")
    receipt = rollback["restartReceipt"]
    expected_receipt = {"state", "permissions", "localizationRequired", "missionResume", "parentReleaseBindingSha256", "inventoryDigest"}
    if not isinstance(receipt, dict) or set(receipt) != expected_receipt:
        raise ManifestError("rollback restart receipt has non-strict schema")
    if (receipt["state"] != "DISARMED" or receipt["permissions"] != "UNKNOWN"
            or receipt["localizationRequired"] is not True or receipt["missionResume"] is not False
            or receipt["parentReleaseBindingSha256"] != binding
            or receipt["inventoryDigest"] != canonical_hash(inventory)):
        raise ManifestError("rollback restart receipt does not prove a disarmed matching restart")


def generate_manifest(root, reports, rollback_parent, blockers=None, source=None):
    root = _root(root)
    report_paths = set()
    for item in reports:
        report = Path(item)
        if report.is_absolute():
            try:
                relative = report.relative_to(root).as_posix()
            except ValueError as exc:
                raise ManifestError("gate reports must be inside the release root") from exc
        else:
            relative = report.as_posix()
        report = _safe_file(root, relative)
        if report.stat().st_size == 0 or relative in report_paths:
            raise ManifestError("missing, empty, or duplicate gate report: " + relative)
        report_paths.add(relative)
    expected_paths = set(REQUIRED_GATES.values())
    if report_paths != expected_paths:
        missing, unexpected = sorted(expected_paths - report_paths), sorted(report_paths - expected_paths)
        raise ManifestError("AC gate matrix is incomplete or contains unexpected reports: missing={} unexpected={}".format(missing, unexpected))
    source = source or source_identity(root)
    if not isinstance(source, dict) or not isinstance(source.get("revision"), str) or not source["revision"]:
        raise ManifestError("source identity lacks a revision")
    hashes = _inventory(root, report_paths)
    bindings = _bind_inputs(source, hashes)
    seen_gates = {_validate_gate_report(_safe_file(root, relative), relative, bindings) for relative in sorted(report_paths)}
    if seen_gates != set(REQUIRED_GATES):
        raise ManifestError("AC gate IDs are incomplete or duplicated")
    _validate_rollback(rollback_parent)
    known_blockers = sorted(set(blockers or DEFAULT_BLOCKERS))
    if (not known_blockers
            or "target NUC qualification has not passed" in known_blockers
            or not set(DEFAULT_BLOCKERS).issubset(known_blockers)):
        raise ManifestError("known blockers must record unresolved hardware/passenger qualification only")
    manifest = {
        "schema": SCHEMA, "source": source, "hashes": hashes,
        "gate_matrix": {"requiredGateIds": sorted(REQUIRED_GATES), "passedGateIds": sorted(seen_gates),
                        "releaseBindings": bindings},
        "authority": {"software_release_candidate": True,
                      "clean_release_authority": seen_gates == set(REQUIRED_GATES),
                      "hardware_motion_authorized": False, "passenger_operation_authorized": False,
                      "physical_authority": False, "simulation_or_replay_is_physical_evidence": False},
        "qualification": {"target_nuc": "passed" if "WP6-TIMING-001" in seen_gates else "blocked",
                          "hardware": "blocked", "passenger": "blocked"},
        "test_reports": [{"path": relative, "sha256": file_hash(_safe_file(root, relative)),
                          "executable": False} for relative in sorted(report_paths)],
        "known_blockers": known_blockers, "rollback": rollback_parent,
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
    parser.add_argument("--output", type=Path)
    parser.add_argument("--report", action="append", type=Path)
    parser.add_argument("--rollback-parent", help="strict JSON rollback binding")
    parser.add_argument("--blocker", action="append")
    parser.add_argument("--prepare-bindings", action="store_true")
    parser.add_argument("--bindings-output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.prepare_bindings:
            if any((args.output, args.report, args.rollback_parent, args.blocker)):
                raise ManifestError("--prepare-bindings cannot be combined with manifest arguments")
            if args.bindings_output:
                output = args.bindings_output
                if not output.is_absolute():
                    output = args.root / output
                if output.is_symlink():
                    raise ManifestError("bindings output is a symlink")
            bindings = prepare_bindings(args.root)
            if args.bindings_output:
                atomic_write(output, bindings)
            else:
                print(json.dumps(bindings, sort_keys=True, separators=(",", ":"), ensure_ascii=False))
            return 0
        if not args.output or not args.report or not args.rollback_parent or args.bindings_output:
            raise ManifestError("manifest output, reports, and rollback parent are required")
        rollback_parent = json.loads(args.rollback_parent)
        manifest = generate_manifest(args.root, args.report, rollback_parent, args.blocker)
        atomic_write(args.output, manifest)
    except (ManifestError, OSError, ValueError, json.JSONDecodeError) as exc:
        parser.exit(2, "release manifest generation failed: {}\n".format(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
