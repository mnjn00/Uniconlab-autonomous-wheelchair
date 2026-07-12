#!/usr/bin/env python3
"""Filesystem-only v2 signed install and rollback lifecycle tests; no hardware is exercised."""

import hashlib
import hmac
import importlib.util
import json
import os
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


INSTALL = load("install_noetic_rc")
ROLLBACK = load("rollback_noetic_rc")
MANIFEST = load("generate_release_manifest")


def canonical(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")


def signed_manifest(root, key, rollback):
    """Build a complete authoritative v2 fixture accepted by the default verifier."""
    files = {
        "Makefile": b"all:\n\ttrue\n",
        "README.md": ("software-only fixture: " + root.name + "\n").encode(),
        "src/wheelchair_safety/package.xml": b"<package/>\n",
        "src/wheelchair_safety/msg/state.msg": b"string state\n",
        "src/wheelchair_safety/config/policy.yaml": b"safe: true\n",
        "src/wheelchair_safety/launch/safety.launch": b"<launch/>\n",
        "src/wheelchair_safety/urdf/base.urdf": b"<robot/>\n",
        "contracts/control.json": b"{}\n",
        "data/map.pgm": b"P2\n1 1\n255\n0\n",
        "data/route-waypoints.yaml": b"waypoints: []\n",
        "tools/check.sh": b"#!/bin/sh\ntrue\n",
        "tests/fixture_test.py": b"assert True\n",
        "docs/gate-artifact.txt": b"fixture artifact\n",
        "evidence/bootstrap.json": b"{\"fixture\":true}\n",
    }
    for path in MANIFEST.REQUIRED_RUNTIME_ENTRYPOINTS:
        files[path] = b"#!/usr/bin/env python3\n"
    for relative, content in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        if relative in MANIFEST.REQUIRED_RUNTIME_ENTRYPOINTS or relative == "tools/check.sh":
            path.chmod(0o755)

    # Qualification evidence is excluded from release-input bindings, avoiding a report/hash cycle.
    provisional = MANIFEST._bind_inputs(
        {"kind": "git_commit", "revision": "b" * 40, "worktree_clean": True}, MANIFEST._inventory(root))
    artifact = {"path": "docs/gate-artifact.txt", "sha256": digest(root / "docs/gate-artifact.txt")}
    for gate, relative in MANIFEST.REQUIRED_GATES.items():
        requirement = MANIFEST.GATE_REQUIREMENTS[gate]
        report = {
            "artifactType": MANIFEST.GATE_REPORT_SCHEMA[0], "schemaVersion": MANIFEST.GATE_REPORT_SCHEMA[1],
            "gateId": gate, "status": "PASS", "claimTag": MANIFEST.GATE_CLAIM,
            "hardwareMotionAuthorized": False, "passengerOperationAuthorized": False,
            **provisional,
            "result": {
                "passed": True, "executedCommands": ["fixture-check"],
                "environment": {"os": "test", "architecture": "x86_64"},
                "tool": {"name": "pytest", "version": "fixture"}, "durationSeconds": 1,
                "metrics": {requirement["metric"]: 1}, "invariants": {requirement["invariant"]: True},
                "artifacts": [artifact],
            },
        }
        dump(root / relative, report)

    source = {"kind": "git_commit", "revision": "b" * 40, "worktree_clean": True}
    hashes = MANIFEST._inventory(root)
    bindings = MANIFEST._bind_inputs(source, hashes)
    assert bindings == provisional
    manifest = {
        "schema": MANIFEST.SCHEMA, "source": source, "hashes": hashes,
        "gate_matrix": {"requiredGateIds": sorted(MANIFEST.REQUIRED_GATES), "passedGateIds": sorted(MANIFEST.REQUIRED_GATES), "releaseBindings": bindings},
        "authority": {"software_release_candidate": True, "clean_release_authority": True, "hardware_motion_authorized": False, "passenger_operation_authorized": False, "physical_authority": False, "simulation_or_replay_is_physical_evidence": False},
        "qualification": {"target_nuc": "passed", "hardware": "blocked", "passenger": "blocked"},
        "test_reports": [{"path": path, "sha256": digest(root / path), "executable": False} for path in sorted(MANIFEST.REQUIRED_GATES.values())],
        "residual_blockers": list(MANIFEST.RESIDUAL_BLOCKERS), "rollback": rollback,
    }
    manifest["release_binding_sha256"] = canonical(manifest)
    manifest["release_signature_hmac_sha256"] = hmac.new(key, json.dumps({"releaseBindingSha256": manifest["release_binding_sha256"]}, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode(), hashlib.sha256).hexdigest()
    dump(root / "release-manifest.json", manifest)
    return manifest


def placeholder_rollback():
    return {"parentReleaseBindingSha256": "a" * 64, "parentManifestSha256": "a" * 64,
            "parentManifestPath": "release-manifest.json", "parentInventoryDigest": "a" * 64,
            "restartReceipt": {"path": "evidence/restart.json", "sha256": "a" * 64,
                               "parentReleaseBindingSha256": "a" * 64, "parentInventoryDigest": "a" * 64}}


def make_release(base, name, key, rollback=None):
    root = base / name
    root.mkdir()
    return root, signed_manifest(root, key, rollback or placeholder_rollback())


def evidence(path, current_id, target_id, **changes):
    token = {"state": "DISARMED", "permissions": "UNKNOWN", "localizationRequired": True,
             "missionResume": False, "currentReleaseBindingSha256": current_id,
             "targetReleaseBindingSha256": target_id, "hardwareMotionAuthorized": False,
             "hardwareEnabled": False}
    token.update(changes)
    token["evidenceBindingSha256"] = canonical(token)
    dump(path, token)
    return path


@pytest.fixture
def release_key(tmp_path):
    path = tmp_path / "release.key"
    path.write_bytes(b"release-key")
    return path
@pytest.fixture(autouse=True)
def non_root(monkeypatch):
    monkeypatch.setattr(os, "geteuid", lambda: 1000)


def install(root, prefix, key, apply=True):
    return INSTALL.install_release(root, root / "release-manifest.json", prefix, apply, release_signing_key=key)


def raw_manifest(path, _):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def parent_and_child(tmp_path, release_key):
    prefix = tmp_path / "sandbox"
    parent_root, _ = make_release(tmp_path, "parent", release_key.read_bytes())
    parent_result = install(parent_root, prefix, release_key)
    installed_parent = prefix / "releases" / parent_result["release"]
    parent_manifest = json.loads((installed_parent / "release-manifest.json").read_text())
    inventory = canonical(parent_manifest["hashes"])
    restart = {"state": "DISARMED", "permissions": "UNKNOWN", "localizationRequired": True,
               "missionResume": False, "parentReleaseBindingSha256": parent_result["release"],
               "parentInventoryDigest": inventory, "hardwareMotionAuthorized": False,
               "passengerOperationAuthorized": False}
    dump(installed_parent / "evidence/restart.json", restart)
    rollback = {"parentReleaseBindingSha256": parent_result["release"],
                "parentManifestSha256": digest(installed_parent / "release-manifest.json"),
                "parentManifestPath": "release-manifest.json", "parentInventoryDigest": inventory,
                "restartReceipt": {"path": "evidence/restart.json", "sha256": digest(installed_parent / "evidence/restart.json"),
                                   "parentReleaseBindingSha256": parent_result["release"],
                                   "parentInventoryDigest": inventory}}
    child_root, child_manifest = make_release(tmp_path, "child", release_key.read_bytes(), rollback)
    child_result = install(child_root, prefix, release_key)
    return prefix, parent_root, parent_manifest, parent_result, child_root, child_manifest, child_result


def test_dry_run_is_default_and_does_not_create_prefix(tmp_path, release_key):
    root, _ = make_release(tmp_path, "dry", release_key.read_bytes())
    result = install(root, tmp_path / "sandbox", release_key, apply=False)
    assert result["applied"] is False
    assert result["state"] == "DISARMED"
    assert not (tmp_path / "sandbox").exists()


def test_install_is_atomic_idempotent_and_copies_exact_bound_inventory(tmp_path, release_key):
    root, manifest = make_release(tmp_path, "install", release_key.read_bytes())
    prefix = tmp_path / "sandbox"
    first = install(root, prefix, release_key)
    second = install(root, prefix, release_key)
    final = prefix / "releases" / first["release"]
    expected = {entry["path"] for section in manifest["hashes"].values() for entry in section["files"]}
    observed = {path.relative_to(final).as_posix() for path in final.rglob("*") if path.is_file()}
    assert os.readlink(prefix / "current") == "releases/" + first["release"]
    assert second["release"] == first["release"] and second["state"] == "DISARMED"
    assert observed == expected | {"release-manifest.json"}
    assert first["files"] == sorted(expected)
    assert (prefix / "receipts" / ("install-" + first["release"] + ".json")).is_file()


@pytest.mark.parametrize("mutation", ["source", "armed", "traversal", "symlink"])
def test_unsafe_install_inputs_are_rejected_without_changing_current(tmp_path, release_key, mutation):
    good_root, _ = make_release(tmp_path, "good", release_key.read_bytes())
    prefix = tmp_path / "sandbox"
    prior = install(good_root, prefix, release_key)
    bad_root, bad_manifest = make_release(tmp_path, "bad", release_key.read_bytes())
    if mutation == "source":
        (bad_root / "src/wheelchair_safety/config/policy.yaml").write_text("tampered\n")
    elif mutation == "armed":
        bad_manifest["authority"]["hardware_motion_authorized"] = True
        bad_manifest["release_binding_sha256"] = canonical({key: value for key, value in bad_manifest.items() if key not in {"release_binding_sha256", "release_signature_hmac_sha256"}})
        bad_manifest["release_signature_hmac_sha256"] = hmac.new(release_key.read_bytes(), json.dumps({"releaseBindingSha256": bad_manifest["release_binding_sha256"]}, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode(), hashlib.sha256).hexdigest()
        dump(bad_root / "release-manifest.json", bad_manifest)
    elif mutation == "traversal":
        bad_manifest["hashes"]["configuration"]["files"][0]["path"] = "../outside.yaml"
        dump(bad_root / "release-manifest.json", bad_manifest)
    else:
        path = bad_root / "src/wheelchair_safety/config/policy.yaml"
        path.unlink(); path.symlink_to(tmp_path / "outside.yaml")
    with pytest.raises(INSTALL.InstallError):
        install(bad_root, prefix, release_key)
    assert os.readlink(prefix / "current") == "releases/" + prior["release"]


def test_interrupted_staging_leaves_prior_current(tmp_path, release_key):
    prior_root, _ = make_release(tmp_path, "prior", release_key.read_bytes())
    prefix = tmp_path / "sandbox"
    prior = install(prior_root, prefix, release_key)
    root, _ = make_release(tmp_path, "interrupted", release_key.read_bytes())
    with pytest.raises(RuntimeError):
        INSTALL.install_release(root, root / "release-manifest.json", prefix, True,
                                interrupt_hook=lambda point, _: (_ for _ in ()).throw(RuntimeError("stop")) if point == "staged" else None,
                                release_signing_key=release_key)
    assert os.readlink(prefix / "current") == "releases/" + prior["release"]
    assert any(".staging-" in path.name for path in (prefix / "releases").iterdir())


@pytest.mark.parametrize("key_kind", ["missing", "wrong", "signature"])
def test_authoritative_install_requires_matching_explicit_signing_key(tmp_path, release_key, key_kind):
    root, manifest = make_release(tmp_path, "signed", release_key.read_bytes())
    key = release_key
    if key_kind == "missing":
        key = None
    elif key_kind == "wrong":
        key = tmp_path / "wrong.key"; key.write_bytes(b"wrong")
    else:
        manifest["release_signature_hmac_sha256"] = "0" * 64
        dump(root / "release-manifest.json", manifest)
    with pytest.raises(INSTALL.InstallError):
        INSTALL.install_release(root, root / "release-manifest.json", tmp_path / "sandbox", True, release_signing_key=key)


def test_rollback_accepts_actual_parent_manifest_inventory_signature_and_disarmed_receipt(tmp_path, release_key):
    prefix, _, parent, parent_result, _, _, child_result = parent_and_child(tmp_path, release_key)
    state = evidence(tmp_path / "current-state.json", child_result["release"], parent_result["release"])
    result = ROLLBACK.rollback_release(prefix, parent_result["release"], state, True, raw_manifest, release_signing_key=release_key)
    receipt = prefix / "receipts" / "rollback-{}-to-{}.json".format(child_result["release"], parent_result["release"])
    assert result["state"] == "DISARMED" and result["parent_inventory_digest"] == canonical(parent["hashes"])
    assert os.readlink(prefix / "current") == "releases/" + parent_result["release"]
    assert json.loads(receipt.read_text())["armed_state"] == "DISARMED"
    repeated = ROLLBACK.rollback_release(prefix, parent_result["release"], evidence(tmp_path / "repeat.json", parent_result["release"], parent_result["release"]), True, raw_manifest, release_signing_key=release_key)
    assert repeated["idempotent"] is True and receipt.is_file()
def test_parent_reference_rejects_traversal_and_missing_key(tmp_path, release_key):
    prefix, _, parent, parent_result, _, child, _ = parent_and_child(tmp_path, release_key)
    parent_root = prefix / "releases" / parent_result["release"]
    receipt, inventory = ROLLBACK._parent_rollback(child, parent_root, parent, parent_result["release"], release_key.read_bytes())
    assert receipt == child["rollback"]["restartReceipt"]["sha256"]
    assert inventory == child["rollback"]["parentInventoryDigest"]
    child["rollback"]["parentManifestPath"] = "../release-manifest.json"
    with pytest.raises(ROLLBACK.RollbackError):
        ROLLBACK._parent_rollback(child, parent_root, parent, parent_result["release"], release_key.read_bytes())
    with pytest.raises(ROLLBACK.RollbackError):
        ROLLBACK._key(tmp_path / "missing.key")



@pytest.mark.parametrize("mutation", ["manifest", "signature", "inventory", "receipt"])
def test_parent_tampering_is_rejected_without_switching_current(tmp_path, release_key, mutation):
    prefix, _, _, parent_result, _, _, child_result = parent_and_child(tmp_path, release_key)
    parent_root = prefix / "releases" / parent_result["release"]
    if mutation == "manifest":
        (parent_root / "release-manifest.json").write_text("{}")
    elif mutation == "signature":
        manifest = json.loads((parent_root / "release-manifest.json").read_text()); manifest["release_signature_hmac_sha256"] = "0" * 64; dump(parent_root / "release-manifest.json", manifest)
    elif mutation == "inventory":
        (parent_root / "src/wheelchair_safety/config/policy.yaml").write_text("tampered\n")
    else:
        (parent_root / "evidence/restart.json").write_text("{}")
    with pytest.raises(ROLLBACK.RollbackError):
        ROLLBACK.rollback_release(prefix, parent_result["release"], evidence(tmp_path / "state.json", child_result["release"], parent_result["release"]), True, raw_manifest, release_signing_key=release_key)
    assert os.readlink(prefix / "current") == "releases/" + child_result["release"]


def test_rollback_rejects_missing_legacy_mismatched_parent_and_bad_evidence(tmp_path, release_key):
    prefix, _, _, parent_result, _, child_manifest, child_result = parent_and_child(tmp_path, release_key)
    state = tmp_path / "state.json"
    child_root = prefix / "releases" / child_result["release"]
    for rollback in (None, {"parent": "legacy"}, {**child_manifest["rollback"], "parentReleaseBindingSha256": "f" * 64}):
        manifest = json.loads((child_root / "release-manifest.json").read_text())
        manifest["rollback"] = rollback
        dump(child_root / "release-manifest.json", manifest)
        with pytest.raises(ROLLBACK.RollbackError):
            ROLLBACK.rollback_release(prefix, parent_result["release"], evidence(state, child_result["release"], parent_result["release"]), verifier=raw_manifest, release_signing_key=release_key)
        dump(child_root / "release-manifest.json", child_manifest)
    for changes in ({"state": "ARMED"}, {"permissions": "CLEAR"}, {"missionResume": True}, {"targetReleaseBindingSha256": "f" * 64}):
        with pytest.raises(ROLLBACK.RollbackError):
            ROLLBACK.rollback_release(prefix, parent_result["release"], evidence(state, child_result["release"], parent_result["release"], **changes), verifier=raw_manifest, release_signing_key=release_key)
    with pytest.raises(ROLLBACK.RollbackError):
        ROLLBACK.rollback_release(prefix, parent_result["release"], "DISARMED", verifier=raw_manifest, release_signing_key=release_key)


def test_rollback_requires_signing_key_for_idempotent_and_non_idempotent_paths(tmp_path, release_key):
    prefix, _, _, parent_result, _, _, child_result = parent_and_child(tmp_path, release_key)
    with pytest.raises(ROLLBACK.RollbackError):
        ROLLBACK.rollback_release(prefix, parent_result["release"], evidence(tmp_path / "state.json", child_result["release"], parent_result["release"]))
    ROLLBACK.rollback_release(prefix, parent_result["release"], evidence(tmp_path / "rollback.json", child_result["release"], parent_result["release"]), True, raw_manifest, release_signing_key=release_key)
    with pytest.raises(ROLLBACK.RollbackError):
        ROLLBACK.rollback_release(prefix, parent_result["release"], evidence(tmp_path / "idempotent.json", parent_result["release"], parent_result["release"]))
