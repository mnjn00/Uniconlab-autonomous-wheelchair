#!/usr/bin/env python3
"""Independently verify a deterministic, fail-closed Noetic release manifest."""
import argparse
import hmac
import json
from pathlib import Path
from generate_release_manifest import (CATEGORIES, GATE_CLAIM, GATE_REPORT_SCHEMA, GATE_REQUIREMENTS, GENERATED_ARTIFACT_NAMES, GIT_SHA, HEX64, REQUIRED_GATES, REQUIRED_RUNTIME_ENTRYPOINTS, RESIDUAL_BLOCKERS, SCHEMA, ManifestError, _category, _excluded, _safe_file, _validate_gate_report, _validate_rollback, canonical_hash, file_hash, sign_release_binding)

VerificationError = ManifestError

def require(condition, message):
    if not condition: raise ManifestError(message)

def _root(root):
    root = Path(root).absolute(); require(root.is_dir() and not root.is_symlink(), "release root must be a non-symlink directory"); return root

def _load(path):
    path = Path(path).absolute(); require(not path.is_symlink(), "manifest is a symlink")
    try: value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc: raise ManifestError("cannot read manifest: {}".format(exc)) from exc
    require(isinstance(value, dict), "manifest root must be an object"); return value

def _expected_inventory(root):
    expected = {category: [] for category in CATEGORIES}
    for candidate in root.rglob("*"):
        relative = candidate.relative_to(root).as_posix()
        if _excluded(relative): continue
        require(not candidate.is_symlink(), "symlink is forbidden in release path: " + relative)
        if candidate.is_file():
            category = _category(relative); require(category is not None, "unclassified regular file: " + relative); expected[category].append(relative)
    for category in CATEGORIES:
        expected[category] = sorted(set(expected[category])); require(expected[category], "required hash category is empty: " + category)
    return expected

def _validate_inventory(root, hashes):
    require(isinstance(hashes, dict) and set(hashes) == set(CATEGORIES), "hash category inventory differs")
    expected = _expected_inventory(root); digests = {}; seen = set()
    for category in CATEGORIES:
        section = hashes[category]; require(isinstance(section, dict) and set(section) == {"digest", "files"}, "invalid hash category: " + category)
        entries = section["files"]; require(isinstance(entries, list) and entries and entries == sorted(entries, key=lambda x: x.get("path", "") if isinstance(x, dict) else ""), "invalid hash entries: " + category)
        require([x.get("path") if isinstance(x, dict) else None for x in entries] == expected[category], "hash scope differs: " + category)
        for entry in entries:
            require(isinstance(entry, dict) and set(entry) == {"path", "sha256", "executable"} and isinstance(entry["sha256"], str) and HEX64.fullmatch(entry["sha256"]) and isinstance(entry["executable"], bool), "invalid hash entry: " + category)
            require(entry["path"].casefold() not in seen, "duplicate or colliding inventory path: " + entry["path"]); seen.add(entry["path"].casefold())
            candidate = _safe_file(root, entry["path"]); require(file_hash(candidate) == entry["sha256"], "hash mismatch: " + entry["path"]); require(bool(candidate.stat().st_mode & 0o111) is entry["executable"], "executable mode mismatch: " + entry["path"])
        require(section["digest"] == canonical_hash(entries), "category digest mismatch: " + category); digests[category] = section["digest"]
    for entrypoint in REQUIRED_RUNTIME_ENTRYPOINTS:
        candidate = _safe_file(root, entrypoint); require(entrypoint in expected["python_runtime"] and bool(candidate.stat().st_mode & 0o111), "required runtime entrypoint is missing or not executable: " + entrypoint)
    return digests

def _validate_source(source):
    require(isinstance(source, dict) and set(source) == {"kind", "revision", "worktree_clean"}, "invalid source identity")
    require((source["kind"] == "git_commit" and isinstance(source["revision"], str) and GIT_SHA.fullmatch(source["revision"]) and source["worktree_clean"] is True) or (source["kind"] == "worktree" and isinstance(source["revision"], str) and source["revision"].startswith("worktree:") and source["worktree_clean"] is False), "invalid source revision")

def _bindings(source, digests):
    release_input = canonical_hash({c: digests[c] for c in CATEGORIES if c != "qualification_evidence"})
    return {"sourceRevision": source["revision"], "configurationDigest": digests["configuration"], "bundleDigest": canonical_hash({"source_revision": source["revision"], "configuration_digest": digests["configuration"], "release_input_digest": release_input}), "releaseInputDigest": release_input}

def _key(path):
    try: key = Path(path).read_bytes()
    except OSError as exc: raise ManifestError("cannot read release signing key") from exc
    require(key, "release signing key is empty"); return key

def verify_manifest(path, root=None, release_signing_key=None):
    manifest_path = Path(path).absolute(); root = _root(root if root else manifest_path.parent)
    try: relative = manifest_path.relative_to(root).as_posix()
    except ValueError as exc: raise ManifestError("manifest path escapes release root") from exc
    _safe_file(root, relative); manifest = _load(manifest_path)
    keys = {"schema", "source", "hashes", "gate_matrix", "authority", "qualification", "test_reports", "residual_blockers", "rollback", "release_binding_sha256", "release_signature_hmac_sha256"}
    require(set(manifest) == keys and manifest["schema"] == SCHEMA, "unsupported or non-strict manifest schema")
    binding = manifest["release_binding_sha256"]; require(isinstance(binding, str) and HEX64.fullmatch(binding), "invalid release binding")
    unsigned = dict(manifest); unsigned.pop("release_binding_sha256"); unsigned.pop("release_signature_hmac_sha256")
    require(binding == canonical_hash(unsigned), "release binding mismatch")
    _validate_source(manifest["source"]); authoritative = manifest["source"]["kind"] == "git_commit"
    authority = {"software_release_candidate": True, "clean_release_authority": authoritative, "hardware_motion_authorized": False, "passenger_operation_authorized": False, "physical_authority": False, "simulation_or_replay_is_physical_evidence": False}
    require(manifest["authority"] == authority, "authority escalation or source mismatch")
    signature = manifest["release_signature_hmac_sha256"]
    if authoritative:
        require(isinstance(signature, str) and HEX64.fullmatch(signature), "authoritative release lacks signature")
        if release_signing_key is not None: require(hmac.compare_digest(signature, sign_release_binding(binding, _key(release_signing_key))), "release signature is invalid")
    else: require(signature is None, "draft release must not carry authoritative signature")
    digests = _validate_inventory(root, manifest["hashes"]); bindings = _bindings(manifest["source"], digests)
    matrix = manifest["gate_matrix"]; require(isinstance(matrix, dict) and set(matrix) == {"requiredGateIds", "passedGateIds", "releaseBindings"} and matrix["requiredGateIds"] == sorted(REQUIRED_GATES) and matrix["passedGateIds"] == sorted(REQUIRED_GATES) and matrix["releaseBindings"] == bindings, "AC gate matrix is incomplete or stale")
    reports = manifest["test_reports"]; paths = sorted(REQUIRED_GATES.values()); require(isinstance(reports, list) and len(reports) == len(paths) and [x.get("path") if isinstance(x, dict) else None for x in reports] == paths, "test reports do not match AC gate matrix")
    for entry in reports:
        require(isinstance(entry, dict) and set(entry) == {"path", "sha256", "executable"} and entry["executable"] is False and isinstance(entry["sha256"], str) and HEX64.fullmatch(entry["sha256"]), "invalid test-report reference")
        candidate = _safe_file(root, entry["path"]); require(file_hash(candidate) == entry["sha256"], "test report hash mismatch: " + entry["path"])
    evidence = {x["path"]: x for x in manifest["hashes"]["qualification_evidence"]["files"]}; require(all(evidence.get(x["path"]) == x for x in reports), "test reports are not bound to qualification evidence")
    gate_ids = {_validate_gate_report(_safe_file(root, p), p, bindings, root) for p in paths}; require(gate_ids == set(REQUIRED_GATES), "AC gate IDs are incomplete or duplicated")
    require(manifest["qualification"] == {"target_nuc": "passed", "hardware": "blocked", "passenger": "blocked"}, "qualification status contradicts complete gate matrix")
    require(manifest["residual_blockers"] == list(RESIDUAL_BLOCKERS), "residual blockers contradict authority")
    _validate_rollback(manifest["rollback"])
    return manifest

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("manifest", type=Path); parser.add_argument("--root", type=Path); parser.add_argument("--release-signing-key", type=Path); args = parser.parse_args(argv)
    try: verify_manifest(args.manifest, args.root, args.release_signing_key)
    except ManifestError as exc: parser.exit(2, "release manifest verification failed: {}\n".format(exc))
    return 0
if __name__ == "__main__": raise SystemExit(main())
