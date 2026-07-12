#!/usr/bin/env python3
"""Filesystem-only rollback binding tests; no hardware is exercised."""
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest
ROOT = Path(__file__).resolve().parents[1]
def load(name):
    spec=importlib.util.spec_from_file_location(name, ROOT/"scripts"/(name+".py")); module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module
ROLLBACK=load("rollback_noetic_rc")
def canonical(value): return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",",":"), ensure_ascii=False).encode()).hexdigest()
def dump(path, value): path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value, sort_keys=True))
def parent(root, key=b"release-key"):
    policy=root/"config/policy.yaml"; policy.parent.mkdir(parents=True); policy.write_text("safe: true\n")
    entries=[{"path":"config/policy.yaml","sha256":hashlib.sha256(policy.read_bytes()).hexdigest(),"executable":False}]
    hashes={"configuration":{"digest":canonical(entries),"files":entries}}
    manifest={"hashes":hashes,"release_binding_sha256":"a"*64}
    manifest["release_signature_hmac_sha256"]=__import__("hmac").new(key, json.dumps({"releaseBindingSha256":"a"*64},sort_keys=True,separators=(",",":"),ensure_ascii=False).encode(), hashlib.sha256).hexdigest()
    dump(root/"release-manifest.json", manifest); digest=canonical(hashes)
    receipt={"state":"DISARMED","permissions":"UNKNOWN","localizationRequired":True,"missionResume":False,"parentReleaseBindingSha256":"a"*64,"parentInventoryDigest":digest,"hardwareMotionAuthorized":False,"passengerOperationAuthorized":False}
    dump(root/"evidence/restart.json",receipt)
    reference={"path":"evidence/restart.json","sha256":hashlib.sha256((root/"evidence/restart.json").read_bytes()).hexdigest(),"parentReleaseBindingSha256":"a"*64,"parentInventoryDigest":digest}
    child={"rollback":{"parentReleaseBindingSha256":"a"*64,"parentManifestSha256":hashlib.sha256((root/"release-manifest.json").read_bytes()).hexdigest(),"parentManifestPath":"release-manifest.json","parentInventoryDigest":digest,"restartReceipt":reference}}
    return manifest, child

def test_parent_binding_uses_actual_manifest_inventory_signature_and_disarmed_receipt(tmp_path):
    key=tmp_path/"key"; key.write_bytes(b"release-key"); manifest, child=parent(tmp_path)
    receipt, inventory=ROLLBACK._parent_rollback(child,tmp_path,manifest,"a"*64,key.read_bytes())
    assert receipt == child["rollback"]["restartReceipt"]["sha256"]
    assert inventory == child["rollback"]["parentInventoryDigest"]

@pytest.mark.parametrize("mutation", [
    lambda root, child, manifest: (root/"release-manifest.json").write_text("tampered"),
    lambda root, child, manifest: manifest.update(release_signature_hmac_sha256="0"*64),
    lambda root, child, manifest: child["rollback"].update(parentInventoryDigest="0"*64),
    lambda root, child, manifest: (root/"evidence/restart.json").write_text("{}"),
])
def test_parent_tampering_bad_signature_inventory_and_stale_restart_are_rejected(tmp_path, mutation):
    key=tmp_path/"key"; key.write_bytes(b"release-key"); manifest, child=parent(tmp_path); mutation(tmp_path, child, manifest)
    with pytest.raises(ROLLBACK.RollbackError): ROLLBACK._parent_rollback(child,tmp_path,manifest,"a"*64,key.read_bytes())

def test_parent_reference_rejects_traversal_and_missing_key(tmp_path):
    manifest, child=parent(tmp_path); child["rollback"]["parentManifestPath"]="../release-manifest.json"
    with pytest.raises(ROLLBACK.RollbackError): ROLLBACK._parent_rollback(child,tmp_path,manifest,"a"*64,b"release-key")
    with pytest.raises(ROLLBACK.RollbackError): ROLLBACK._key(tmp_path/"missing-key")
def test_idempotent_rollback_stays_disarmed_without_parent_references(tmp_path):
    release = "a" * 64; prefix = tmp_path / "sandbox"; bundle = prefix / "releases" / release
    bundle.mkdir(parents=True); (bundle / "release-manifest.json").write_text("{}")
    (prefix / "current").symlink_to("releases/" + release)
    authority = {"software_release_candidate":True, "clean_release_authority":True,
                 "hardware_motion_authorized":False, "passenger_operation_authorized":False,
                 "physical_authority":False, "simulation_or_replay_is_physical_evidence":False}
    manifest = {"release_binding_sha256":release, "authority":authority}
    evidence = {"state":"DISARMED", "permissions":"UNKNOWN", "localizationRequired":True,
                "missionResume":False, "currentReleaseBindingSha256":release,
                "targetReleaseBindingSha256":release, "hardwareMotionAuthorized":False,
                "hardwareEnabled":False}
    evidence["evidenceBindingSha256"] = canonical(evidence); path = tmp_path / "current-state.json"; dump(path, evidence)
    result = ROLLBACK.rollback_release(prefix, release, path, verifier=lambda *_: manifest)
    assert result["idempotent"] is True and result["state"] == "DISARMED"
