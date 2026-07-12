#!/usr/bin/env python3
"""Atomically roll a sandboxed Noetic software release back while DISARMED."""
import argparse
import hashlib
import hmac
import importlib.util
import json
import os
import uuid
from pathlib import Path


def _install_module():
    path = Path(__file__).with_name("install_noetic_rc.py")
    spec = importlib.util.spec_from_file_location("wheelchair_release_installer", str(path)); module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module
_INSTALL = _install_module(); RollbackError = _INSTALL.InstallError

def _canonical(value): return _INSTALL._canonical_hash(value)
def _hash(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def _hex(value): return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)
def _safe_relative(path): return isinstance(path, str) and path and not Path(path).is_absolute() and ".." not in Path(path).parts

def _key(path):
    try: key = Path(path).read_bytes()
    except OSError as exc: raise RollbackError("release signing key is missing or unreadable") from exc
    if not key: raise RollbackError("release signing key is empty")
    return key

def _evidence(evidence, current_release, target_release):
    path = Path(str(evidence)).expanduser()
    if not path.is_file() or path.is_symlink(): raise RollbackError("separate current-state DISARMED evidence file is required")
    raw = path.read_bytes()
    try: token = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc: raise RollbackError("current-state evidence is not valid JSON") from exc
    keys = {"state", "permissions", "localizationRequired", "missionResume", "currentReleaseBindingSha256", "targetReleaseBindingSha256", "hardwareMotionAuthorized", "hardwareEnabled", "evidenceBindingSha256"}
    if not isinstance(token, dict) or set(token) != keys: raise RollbackError("current-state evidence has non-strict schema")
    if token["state"] != "DISARMED" or token["permissions"] != "UNKNOWN" or token["localizationRequired"] is not True or token["missionResume"] is not False or token["currentReleaseBindingSha256"] != current_release or token["targetReleaseBindingSha256"] != target_release or token["hardwareMotionAuthorized"] is not False or token["hardwareEnabled"] is not False or token["evidenceBindingSha256"] != _canonical({k:v for k,v in token.items() if k != "evidenceBindingSha256"}): raise RollbackError("current-state evidence does not prove a disarmed no-resume rollback")
    return hashlib.sha256(raw).hexdigest()

def _safe_file(root, relative, label):
    if not _safe_relative(relative): raise RollbackError("unsafe {} path".format(label))
    candidate = root / relative; current = root
    for part in candidate.relative_to(root).parts:
        current /= part
        if current.is_symlink(): raise RollbackError("symlink is forbidden in {} path".format(label))
    if not candidate.is_file() or candidate.is_symlink(): raise RollbackError("missing {}".format(label))
    try: candidate.resolve(strict=True).relative_to(root.resolve(strict=True))
    except ValueError as exc: raise RollbackError("{} escapes parent release".format(label)) from exc
    return candidate

def _inventory_digest(parent_root, manifest):
    hashes = manifest.get("hashes")
    if not isinstance(hashes, dict) or not hashes: raise RollbackError("parent release inventory is missing")
    for category, section in hashes.items():
        if not isinstance(category, str) or not isinstance(section, dict) or set(section) != {"digest", "files"} or not isinstance(section["files"], list) or not section["files"]:
            raise RollbackError("parent release inventory is invalid")
        entries = section["files"]
        if section["digest"] != _canonical(entries):
            raise RollbackError("parent release inventory digest is invalid")
        for entry in entries:
            if not isinstance(entry, dict) or set(entry) != {"path", "sha256", "executable"} or not isinstance(entry["executable"], bool) or not _hex(entry.get("sha256")):
                raise RollbackError("parent release inventory entry is invalid")
            candidate = _safe_file(parent_root, entry.get("path"), "parent inventory")
            if _hash(candidate) != entry["sha256"] or bool(candidate.stat().st_mode & 0o111) is not entry["executable"]:
                raise RollbackError("parent release inventory hash mismatch")
    return _canonical(hashes)

def _restart_receipt(parent_root, reference, binding, inventory_digest):
    keys = {"path", "sha256", "parentReleaseBindingSha256", "parentInventoryDigest"}
    if not isinstance(reference, dict) or set(reference) != keys or not all(isinstance(reference[k], str) for k in keys) or not _hex(reference["sha256"]) or reference["parentReleaseBindingSha256"] != binding or reference["parentInventoryDigest"] != inventory_digest: raise RollbackError("parent restart receipt reference is invalid")
    receipt_path = _safe_file(parent_root, reference["path"], "parent restart receipt")
    if _hash(receipt_path) != reference["sha256"]: raise RollbackError("parent restart receipt hash mismatch")
    try: receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc: raise RollbackError("parent restart receipt is invalid JSON") from exc
    required = {"state", "permissions", "localizationRequired", "missionResume", "parentReleaseBindingSha256", "parentInventoryDigest", "hardwareMotionAuthorized", "passengerOperationAuthorized"}
    if not isinstance(receipt, dict) or set(receipt) != required or receipt["state"] != "DISARMED" or receipt["permissions"] != "UNKNOWN" or receipt["localizationRequired"] is not True or receipt["missionResume"] is not False or receipt["hardwareMotionAuthorized"] is not False or receipt["passengerOperationAuthorized"] is not False or receipt["parentReleaseBindingSha256"] != binding or receipt["parentInventoryDigest"] != inventory_digest: raise RollbackError("parent restart receipt is stale or not DISARMED")
    return reference["sha256"]

def _authenticate_parent(parent_manifest, signing_key):
    binding = parent_manifest.get("release_binding_sha256"); signature = parent_manifest.get("release_signature_hmac_sha256")
    if not _hex(binding) or not _hex(signature): raise RollbackError("parent release signing contract is missing")
    expected = hmac.new(signing_key, json.dumps({"releaseBindingSha256": binding}, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8"), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(signature, expected): raise RollbackError("parent release signature is invalid")

def _parent_rollback(current_manifest, parent_root, parent_manifest, target_id, signing_key):
    rollback = current_manifest.get("rollback"); keys = {"parentReleaseBindingSha256", "parentManifestSha256", "parentManifestPath", "parentInventoryDigest", "restartReceipt"}
    if not isinstance(rollback, dict) or set(rollback) != keys: raise RollbackError("current release has no strict parent rollback binding")
    if rollback["parentReleaseBindingSha256"] != target_id or not _hex(rollback["parentManifestSha256"]) or not _hex(rollback["parentInventoryDigest"]) or not _safe_relative(rollback["parentManifestPath"]): raise RollbackError("target release or parent references are invalid")
    manifest_path = _safe_file(parent_root, rollback["parentManifestPath"], "parent manifest")
    if _hash(manifest_path) != rollback["parentManifestSha256"]: raise RollbackError("actual parent manifest hash differs from rollback binding")
    if parent_manifest.get("release_binding_sha256") != target_id: raise RollbackError("actual parent release binding differs from rollback binding")
    inventory_digest = _inventory_digest(parent_root, parent_manifest)
    if inventory_digest != rollback["parentInventoryDigest"]: raise RollbackError("actual parent inventory differs from rollback binding")
    _authenticate_parent(parent_manifest, signing_key)
    receipt_hash = _restart_receipt(parent_root, rollback["restartReceipt"], target_id, inventory_digest)
    return receipt_hash, inventory_digest

def _release(prefix, release_id, verifier, release_signing_key, custom_verifier):
    release_id = _INSTALL._safe_id(release_id)
    path = prefix / "releases" / release_id
    if not path.is_dir() or path.is_symlink():
        raise RollbackError("rollback release is missing or unsafe")
    manifest_path = path / "release-manifest.json"
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise RollbackError("release manifest is missing or unsafe")
    if custom_verifier:
        manifest = verifier(manifest_path, path)
    else:
        manifest = verifier(manifest_path, path, release_signing_key)
    if not isinstance(manifest, dict) or manifest.get("release_binding_sha256") != release_id:
        raise RollbackError("release directory and manifest binding differ")
    _INSTALL._authority_is_software_only(manifest)
    return path, manifest

def rollback_release(prefix, target_release, disarmed_evidence, apply=False, verifier=None,
                     interrupt_hook=None, release_signing_key=None):
    prefix = _INSTALL._check_prefix(prefix, apply)
    if release_signing_key is None:
        raise RollbackError("rollback requires an explicitly supplied release signing key")
    signing_key = _key(release_signing_key)
    if not prefix.is_dir():
        raise RollbackError("release prefix is missing")
    custom_verifier = verifier is not None
    verifier = verifier or _INSTALL._default_verifier
    current_id = _INSTALL._current_release(prefix)
    if current_id is None:
        raise RollbackError("there is no current release")
    target_id = _INSTALL._safe_id(target_release)
    current_path, current_manifest = _release(
        prefix, current_id, verifier, release_signing_key, custom_verifier)
    target_path, target_manifest = _release(
        prefix, target_id, verifier, release_signing_key, custom_verifier)
    evidence_sha256 = _evidence(disarmed_evidence, current_id, target_id)
    idempotent = current_id == target_id
    if not idempotent:
        _authenticate_parent(current_manifest, signing_key)
        receipt_hash, inventory_digest = _parent_rollback(
            current_manifest, target_path, target_manifest, target_id, signing_key)
    result = {"action":"rollback", "applied":bool(apply), "from_release":current_id, "to_release":target_id, "disarmed_evidence_sha256":evidence_sha256, "state":"DISARMED", "idempotent":idempotent}
    if not idempotent: result.update(parent_restart_receipt_sha256=receipt_hash, parent_inventory_digest=inventory_digest)
    if not apply: return result
    if interrupt_hook: interrupt_hook("verified", target_path)
    if not idempotent:
        temporary = prefix / (".current-rollback-" + uuid.uuid4().hex); os.symlink("releases/" + target_id, str(temporary))
        try: os.replace(str(temporary), str(prefix / "current"))
        finally:
            try: temporary.unlink()
            except FileNotFoundError: pass
    receipt = dict(result); receipt.update(current_manifest_sha256=current_id, target_manifest_sha256=target_manifest["release_binding_sha256"], services_modified=False, armed_state="DISARMED")
    _INSTALL._atomic_json(prefix / "receipts" / ("rollback-{}-to-{}.json".format(current_id, target_id)), receipt); return result

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("--prefix", required=True, type=Path); parser.add_argument("--target", required=True); parser.add_argument("--disarmed-evidence", required=True); parser.add_argument("--release-signing-key", required=True, type=Path); parser.add_argument("--apply", action="store_true"); args = parser.parse_args(argv)
    try: result = rollback_release(args.prefix, args.target, args.disarmed_evidence, args.apply, release_signing_key=args.release_signing_key)
    except (RollbackError, OSError) as exc: parser.exit(2, "rollback refused: {}\n".format(exc))
    print(json.dumps(result, sort_keys=True)); return 0
if __name__ == "__main__": raise SystemExit(main())
