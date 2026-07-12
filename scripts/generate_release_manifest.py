#!/usr/bin/env python3
"""Generate deterministic, fail-closed Noetic software release manifests."""
import argparse
import hashlib
import hmac
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path

SCHEMA = "wheelchair-noetic-release-manifest/v2"
CATEGORIES = ("source_build_metadata", "package_metadata", "interfaces", "python_runtime", "configuration", "launch_configuration", "robot_assets", "contracts", "maps", "routes", "operator_docs", "ci_tools", "qualification_tools", "qualification_evidence")
REQUIRED_RUNTIME_ENTRYPOINTS = ("src/wheelchair_safety/scripts/safety_gate.py", "src/wheelchair_safety/scripts/collision_supervisor.py", "src/wheelchair_safety/scripts/topology_guard.py", "src/wheelchair_gazebo/scripts/simulation_controller_adapter.py")
EXCLUDED_PARTS = {".git", ".gjc", ".pytest_cache", ".mypy_cache", ".ruff_cache", ".tox", ".venv", "__pycache__", "artifacts", "build", "devel", "generated", "install", "logs", "log", "node_modules", "release_artifacts", "tmp", "temp"}
EXCLUDED_SUFFIXES = (".bag", ".db3", ".mcap", ".pyc", ".pyo", "~")
GENERATED_ARTIFACT_NAMES = {"release-manifest.json", "release-bindings.json"}
HEX64 = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA = re.compile(r"^[0-9a-f]{40}$")
GATE_REPORT_SCHEMA = ("wheelchair-ac-gate-report", 2)
GATE_CLAIM = "SOFTWARE_ONLY"
# Each AC obligation has a unique report and a unique, non-trivial measured metric/invariant.
_GATE_ROWS = (
 ("WP0-ABI-001", "evidence/contracts/abi-v1-report.json", "interfacesChecked", "abiCompatible"),
 ("WP1-TOPOLOGY-001", "evidence/topology/command-graph-report.json", "commandPathsChecked", "singleCommandPath"),
 ("WP1-GEOFENCE-001", "evidence/route-safety/anti-widening-report.json", "routeBoundsChecked", "routeNotWidened"),
 ("WP1-COLLISION-001", "evidence/safety/collision-ttc-report.json", "collisionScenarios", "ttcStopsEnforced"),
 ("WP1-SLOPE-001", "evidence/safety/slope-policy-report.json", "slopeScenarios", "slopePolicyEnforced"),
 ("WP1-CONTROL-001", "evidence/safety/gate-permission-matrix.json", "permissionCases", "unauthorizedCommandsDenied"),
 ("WP2-CONVERSION-001", "evidence/conversion/determinism-and-corruption-report.json", "conversionCases", "deterministicAndCorruptionSafe"),
 ("WP3-LOCALIZATION-001", "evidence/localization/confidence-holdout-report.json", "holdoutFrames", "lowConfidenceHeld"),
 ("WP3-GLIM-INPUT-001", "evidence/localization/glim-offline-input-report.json", "offlineInputFrames", "offlineInputPinned"),
 ("WP3-GLIM-REPRODUCTION-001", "evidence/localization/glim-offline-reproduction-report.json", "reproductionRuns", "offlineReproducible"),
 ("WP3-GLIM-COMPARISON-001", "evidence/localization/glim-offline-comparison-report.json", "comparisonFrames", "offlineComparisonWithinTolerance"),
 ("WP4-MISSION-001", "evidence/mission/fsm-contract-report.json", "fsmTransitions", "missionContractEnforced"),
 ("WP6-TIMING-001", "evidence/performance/target-nuc-60min-report.json", "measuredSeconds", "targetNucDurationMet"),
 ("WP6-SIMCLAIM-001", "evidence/simulation/fidelity-claim-report.json", "simulationCases", "simulationClaimBounded"),
 ("WP6-ROLLBACK-001", "evidence/release/rollback-drill-report.json", "rollbackDrills", "rollbackDisarmed"),
 ("WP0-HWGATE-NEG-001", "evidence/hardware/hardware-gate-negative-report.json", "deniedHardwareRequests", "hardwareMotionDenied"),
 ("WP0-PASSENGER-NEG-001", "evidence/release/passenger-authority-negative-report.json", "deniedPassengerRequests", "passengerOperationDenied"),
)
REQUIRED_GATES = {gate: path for gate, path, _metric, _invariant in _GATE_ROWS}
GATE_REQUIREMENTS = {gate: {"metric": metric, "invariant": invariant} for gate, _path, metric, invariant in _GATE_ROWS}
RESIDUAL_BLOCKERS = ("hardware_motion_unqualified", "passenger_operation_unqualified")
DEFAULT_BLOCKERS = list(RESIDUAL_BLOCKERS)

class ManifestError(ValueError): pass

def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""): digest.update(chunk)
    return digest.hexdigest()

def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")).hexdigest()

def canonical_bytes(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")

def sign_release_binding(binding, key):
    if not isinstance(key, bytes) or not key: raise ManifestError("release signing key is empty")
    return hmac.new(key, canonical_bytes({"releaseBindingSha256": binding}), hashlib.sha256).hexdigest()

def _root(root):
    root = Path(root).absolute()
    if root.is_symlink() or not root.is_dir(): raise ManifestError("release root must be a non-symlink directory")
    return root

def _excluded(relative):
    path = Path(relative)
    return any(part in EXCLUDED_PARTS for part in path.parts) or relative in GENERATED_ARTIFACT_NAMES or path.name.endswith(EXCLUDED_SUFFIXES)

def _category(relative):
    path = Path(relative); parts, name, suffix = path.parts, path.name, path.suffix.lower()
    if not parts or _excluded(relative): return None
    if parts[0] == "evidence": return "qualification_evidence"
    if parts[0] == "src":
        if name in {"package.xml", "CMakeLists.txt", "setup.py", "setup.cfg"}: return "package_metadata"
        if any(part in {"msg", "action", "srv"} for part in parts): return "interfaces"
        if suffix == ".py" and any(part in {"scripts", "src"} for part in parts[2:]): return "python_runtime"
        if "config" in parts: return "configuration"
        if "launch" in parts: return "launch_configuration"
        if any(part in {"urdf", "worlds", "meshes", "models"} for part in parts): return "robot_assets"
        if "tests" in parts: return "qualification_tools"
        return "source_build_metadata"
    if parts[0] == "scripts": return "python_runtime" if suffix in {".py", ".sh"} else "source_build_metadata"
    if parts[0] == "contracts": return "contracts"
    if parts[0] == "data": return "routes" if "waypoint" in name.lower() or "route" in name.lower() else "maps"
    if parts[0] == "docs" or name == "LICENSE" or name.startswith("README"): return "operator_docs"
    if parts[0] in {".github", "tools"} or name.endswith(".lock"): return "ci_tools"
    if parts[0] == "tests": return "qualification_tools"
    if len(parts) == 1 and (name in {".catkin_workspace", ".dockerignore", ".gitignore", ".gitattributes", "Dockerfile", "Makefile", "pyproject.toml", "tox.ini"} or name.startswith(("requirements", "setup.")) or suffix in {".cfg", ".ini", ".toml", ".lock"}): return "source_build_metadata"
    return None

def _safe_file(root, relative):
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute(): raise ManifestError("invalid release path")
    candidate = root / relative
    try: parts = candidate.relative_to(root).parts
    except ValueError as exc: raise ManifestError("file escapes release root: " + relative) from exc
    current = root
    for part in parts:
        current /= part
        if current.is_symlink(): raise ManifestError("symlink is forbidden in release path: " + relative)
    try: resolved = candidate.resolve(strict=True); resolved.relative_to(root.resolve(strict=True))
    except (FileNotFoundError, ValueError) as exc: raise ManifestError("file is missing or escapes release root: " + relative) from exc
    if not resolved.is_file(): raise ManifestError("inventory entry is not a regular file: " + relative)
    return candidate

def inventory_paths(root, report_paths=()):
    root = _root(root); result = {category: [] for category in CATEGORIES}
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if _excluded(relative): continue
        if path.is_symlink(): raise ManifestError("symlink is forbidden in release path: " + relative)
        if not path.is_file(): continue
        category = _category(relative)
        if category is None: raise ManifestError("unclassified regular file: " + relative)
        result[category].append(relative)
    for category in CATEGORIES:
        result[category] = sorted(set(result[category]))
        if not result[category]: raise ManifestError("required hash category is empty: " + category)
        for relative in result[category]: _safe_file(root, relative)
    for relative in REQUIRED_RUNTIME_ENTRYPOINTS:
        path = _safe_file(root, relative)
        if relative not in result["python_runtime"] or not path.stat().st_mode & 0o111: raise ManifestError("required runtime entrypoint is missing or not executable: " + relative)
    return result

def _inventory(root):
    return {category: {"digest": canonical_hash(entries := [{"path": p, "sha256": file_hash(_safe_file(root, p)), "executable": bool(_safe_file(root, p).stat().st_mode & 0o111)} for p in paths]), "files": entries} for category, paths in inventory_paths(root).items()}

def source_identity(root):
    root = _root(root)
    try:
        commit = subprocess.run(["git", "rev-parse", "--verify", "HEAD"], cwd=root, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=all"], cwd=root, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True).stdout
        if GIT_SHA.fullmatch(commit) and not dirty: return {"kind": "git_commit", "revision": commit, "worktree_clean": True}
    except (OSError, subprocess.CalledProcessError): pass
    files = []
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root).as_posix()
        if not _excluded(relative) and not relative.startswith("evidence/") and path.is_file(): files.append({"path": relative, "sha256": file_hash(path)})
    return {"kind": "worktree", "revision": "worktree:" + canonical_hash(files), "worktree_clean": False}

def _bind_inputs(source, hashes):
    release_input = canonical_hash({c: hashes[c]["digest"] for c in CATEGORIES if c != "qualification_evidence"})
    bundle = canonical_hash({"source_revision": source["revision"], "configuration_digest": hashes["configuration"]["digest"], "release_input_digest": release_input})
    return {"sourceRevision": source["revision"], "configurationDigest": hashes["configuration"]["digest"], "bundleDigest": bundle, "releaseInputDigest": release_input}

def prepare_bindings(root, source=None):
    source = source or source_identity(root)
    return _bind_inputs(source, _inventory(root))

def _positive(value): return isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0

def _validate_gate_report(path, relative, bindings, root):
    try: document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc: raise ManifestError("invalid gate report: " + relative) from exc
    keys = {"artifactType", "schemaVersion", "gateId", "status", "claimTag", "hardwareMotionAuthorized", "passengerOperationAuthorized", "sourceRevision", "configurationDigest", "bundleDigest", "releaseInputDigest", "result"}
    if not isinstance(document, dict) or set(document) != keys: raise ManifestError("gate report has non-strict schema: " + relative)
    gate = document["gateId"]
    if (document["artifactType"], document["schemaVersion"]) != GATE_REPORT_SCHEMA or REQUIRED_GATES.get(gate) != relative: raise ManifestError("gate report ID/path does not match AC matrix: " + relative)
    if document["status"] != "PASS" or document["claimTag"] != GATE_CLAIM or document["hardwareMotionAuthorized"] is not False or document["passengerOperationAuthorized"] is not False: raise ManifestError("gate report is not a passing software-only claim: " + relative)
    if any(document[k] != v for k, v in bindings.items()): raise ManifestError("gate report has stale or mixed release binding: " + relative)
    result = document["result"]; required = {"passed", "executedCommands", "environment", "tool", "durationSeconds", "metrics", "invariants", "artifacts"}
    if not isinstance(result, dict) or set(result) != required or result["passed"] is not True or not _positive(result["durationSeconds"]): raise ManifestError("gate report has shallow result: " + relative)
    if not isinstance(result["executedCommands"], list) or not result["executedCommands"] or not all(isinstance(c, str) and c.strip() for c in result["executedCommands"]): raise ManifestError("gate report lacks executed commands: " + relative)
    if not isinstance(result["environment"], dict) or set(result["environment"]) != {"os", "architecture"} or not all(isinstance(v, str) and v for v in result["environment"].values()): raise ManifestError("gate report lacks environment identity: " + relative)
    if not isinstance(result["tool"], dict) or set(result["tool"]) != {"name", "version"} or not all(isinstance(v, str) and v for v in result["tool"].values()): raise ManifestError("gate report lacks tool identity: " + relative)
    req = GATE_REQUIREMENTS[gate]
    if not isinstance(result["metrics"], dict) or set(result["metrics"]) != {req["metric"]} or not _positive(result["metrics"][req["metric"]]): raise ManifestError("gate report has invalid gate metrics: " + relative)
    if result["invariants"] != {req["invariant"]: True}: raise ManifestError("gate report has invalid gate invariants: " + relative)
    artifacts = result["artifacts"]
    if not isinstance(artifacts, list) or not artifacts or artifacts != sorted(artifacts, key=lambda x: x.get("path", "") if isinstance(x, dict) else ""): raise ManifestError("gate report artifacts are shallow or unordered: " + relative)
    seen = set()
    for artifact in artifacts:
        if not isinstance(artifact, dict) or set(artifact) != {"path", "sha256"} or not isinstance(artifact["path"], str) or not HEX64.fullmatch(artifact.get("sha256", "")) or artifact["path"] in seen: raise ManifestError("gate report artifact is invalid: " + relative)
        seen.add(artifact["path"]); candidate = _safe_file(root, artifact["path"])
        if candidate.as_posix() == path.as_posix() or file_hash(candidate) != artifact["sha256"]: raise ManifestError("gate report artifact hash mismatch: " + relative)
    return gate

def _validate_rollback(rollback):
    keys = {"parentReleaseBindingSha256", "parentManifestSha256", "parentManifestPath", "parentInventoryDigest", "restartReceipt"}
    if not isinstance(rollback, dict) or set(rollback) != keys: raise ManifestError("rollback binding has non-strict schema")
    if not all(isinstance(rollback[k], str) and rollback[k] for k in keys - {"restartReceipt"}) or not all(HEX64.fullmatch(rollback[k]) for k in ("parentReleaseBindingSha256", "parentManifestSha256", "parentInventoryDigest")): raise ManifestError("rollback parent references are invalid")
    if Path(rollback["parentManifestPath"]).is_absolute() or ".." in Path(rollback["parentManifestPath"]).parts: raise ManifestError("rollback parent manifest path is unsafe")
    receipt = rollback["restartReceipt"]; rkeys = {"path", "sha256", "parentReleaseBindingSha256", "parentInventoryDigest"}
    if not isinstance(receipt, dict) or set(receipt) != rkeys or any(not isinstance(receipt[k], str) or not receipt[k] for k in rkeys) or not all(HEX64.fullmatch(receipt[k]) for k in ("sha256", "parentReleaseBindingSha256", "parentInventoryDigest")) or receipt["parentReleaseBindingSha256"] != rollback["parentReleaseBindingSha256"] or receipt["parentInventoryDigest"] != rollback["parentInventoryDigest"]: raise ManifestError("rollback restart receipt reference is invalid")

def generate_manifest(root, reports, rollback_parent, blockers=None, source=None, signing_key=None):
    root = _root(root); report_paths = {Path(p).relative_to(root).as_posix() if Path(p).is_absolute() else Path(p).as_posix() for p in reports}
    if report_paths != set(REQUIRED_GATES.values()): raise ManifestError("AC gate matrix is incomplete or contains unexpected reports")
    for relative in report_paths:
        if _safe_file(root, relative).stat().st_size == 0: raise ManifestError("missing or empty gate report: " + relative)
    actual_source = source_identity(root)
    if source is not None and source != actual_source: raise ManifestError("caller-supplied source identity does not match worktree")
    authoritative = actual_source["kind"] == "git_commit" and actual_source["worktree_clean"] is True
    if authoritative and signing_key is None: raise ManifestError("authoritative release requires an explicitly supplied signing key")
    hashes = _inventory(root); bindings = _bind_inputs(actual_source, hashes)
    gates = {_validate_gate_report(_safe_file(root, p), p, bindings, root) for p in sorted(report_paths)}
    if gates != set(REQUIRED_GATES): raise ManifestError("AC gate IDs are incomplete or duplicated")
    _validate_rollback(rollback_parent)
    blockers = sorted(blockers if blockers is not None else DEFAULT_BLOCKERS)
    if blockers != list(RESIDUAL_BLOCKERS): raise ManifestError("residual blockers must be exactly the typed hardware/passenger set")
    authority = {"software_release_candidate": True, "clean_release_authority": authoritative, "hardware_motion_authorized": False, "passenger_operation_authorized": False, "physical_authority": False, "simulation_or_replay_is_physical_evidence": False}
    manifest = {"schema": SCHEMA, "source": actual_source, "hashes": hashes, "gate_matrix": {"requiredGateIds": sorted(REQUIRED_GATES), "passedGateIds": sorted(gates), "releaseBindings": bindings}, "authority": authority, "qualification": {"target_nuc": "passed", "hardware": "blocked", "passenger": "blocked"}, "test_reports": [{"path": p, "sha256": file_hash(_safe_file(root, p)), "executable": False} for p in sorted(report_paths)], "residual_blockers": blockers, "rollback": rollback_parent}
    manifest["release_binding_sha256"] = canonical_hash(manifest)
    if authoritative:
        manifest["release_signature_hmac_sha256"] = sign_release_binding(manifest["release_binding_sha256"], signing_key)
    else: manifest["release_signature_hmac_sha256"] = None
    return manifest

def atomic_write(path, manifest):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True); descriptor, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream: stream.write(json.dumps(manifest, sort_keys=True, indent=2, ensure_ascii=False) + "\n"); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try: os.unlink(temporary)
        except FileNotFoundError: pass
        raise

def _key(path):
    try: return Path(path).read_bytes()
    except OSError as exc: raise ManifestError("cannot read release signing key") from exc

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("--root", default=Path(__file__).resolve().parents[1], type=Path); parser.add_argument("--output", type=Path); parser.add_argument("--report", action="append", type=Path); parser.add_argument("--rollback-parent", help="strict JSON parent rollback reference"); parser.add_argument("--blocker", action="append"); parser.add_argument("--release-signing-key", type=Path); parser.add_argument("--prepare-bindings", action="store_true"); parser.add_argument("--bindings-output", type=Path); args = parser.parse_args(argv)
    try:
        if args.prepare_bindings:
            if any((args.output, args.report, args.rollback_parent, args.blocker, args.release_signing_key)): raise ManifestError("--prepare-bindings cannot be combined with manifest arguments")
            bindings = prepare_bindings(args.root); payload = json.dumps(bindings, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
            if args.bindings_output: atomic_write(args.bindings_output, bindings)
            else: print(payload)
            return 0
        if not args.output or not args.report or not args.rollback_parent: raise ManifestError("manifest output, reports, and rollback parent are required")
        manifest = generate_manifest(args.root, args.report, json.loads(args.rollback_parent), args.blocker, signing_key=_key(args.release_signing_key) if args.release_signing_key else None); atomic_write(args.output, manifest)
    except (ManifestError, OSError, ValueError, json.JSONDecodeError) as exc: parser.exit(2, "release manifest generation failed: {}\n".format(exc))
    return 0
if __name__ == "__main__": raise SystemExit(main())
