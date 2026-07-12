#!/usr/bin/env python3
"""Atomically roll a sandboxed Noetic software release back while disarmed."""

import argparse
import hashlib
import importlib.util
import json
import os
import uuid
from pathlib import Path


def _install_module():
    path = Path(__file__).with_name("install_noetic_rc.py")
    spec = importlib.util.spec_from_file_location("wheelchair_release_installer", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_INSTALL = _install_module()
RollbackError = _INSTALL.InstallError


def _evidence(evidence, current_release, target_release):
    path = Path(str(evidence)).expanduser()
    if not path.is_file():
        raise RollbackError("separate current-state DISARMED evidence file is required")
    raw = path.read_bytes()
    try:
        token = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RollbackError("current-state evidence is not valid JSON") from exc
    required = {"state", "permissions", "localizationRequired", "missionResume",
                "currentReleaseBindingSha256", "targetReleaseBindingSha256",
                "hardwareMotionAuthorized", "hardwareEnabled", "evidenceBindingSha256"}
    if not isinstance(token, dict) or set(token) != required:
        raise RollbackError("current-state evidence has non-strict schema")
    if (token["state"] != "DISARMED" or token["permissions"] != "UNKNOWN"
            or token["localizationRequired"] is not True or token["missionResume"] is not False
            or token["currentReleaseBindingSha256"] != current_release
            or token["targetReleaseBindingSha256"] != target_release
            or token["hardwareMotionAuthorized"] is not False or token["hardwareEnabled"] is not False):
        raise RollbackError("current-state evidence does not prove a disarmed no-resume rollback")
    binding = token["evidenceBindingSha256"]
    expected = _INSTALL._canonical_hash({k: v for k, v in token.items()
                                         if k != "evidenceBindingSha256"})
    if binding != expected:
        raise RollbackError("current-state evidence binding is invalid")
    return hashlib.sha256(raw).hexdigest()


def _parent_rollback(manifest, target_id):
    rollback = manifest.get("rollback")
    required = {"parentReleaseBindingSha256", "parentReleaseBindingReceiptSha256",
                "inventory", "restartReceipt"}
    if not isinstance(rollback, dict) or set(rollback) != required:
        raise RollbackError("current release has no strict parent rollback binding")
    binding, inventory = rollback["parentReleaseBindingSha256"], rollback["inventory"]
    receipt_hash, receipt = (rollback["parentReleaseBindingReceiptSha256"],
                             rollback["restartReceipt"])
    if (binding != target_id or not isinstance(receipt_hash, str) or len(receipt_hash) != 64
            or any(char not in "0123456789abcdef" for char in receipt_hash)):
        raise RollbackError("target release ID or parent receipt binding is invalid")
    kinds = {"binaries", "maps", "routes", "policies", "drivers"}
    if (not isinstance(inventory, dict) or set(inventory) != kinds
            or receipt_hash != _INSTALL._canonical_hash(
                {"parentReleaseBindingSha256": binding, "inventory": inventory})):
        raise RollbackError("parent rollback inventory is not hash-bound")
    paths = set()
    for kind in kinds:
        entries = inventory[kind]
        if (not isinstance(entries, list) or not entries
                or entries != sorted(entries, key=lambda item: item.get("path", "")
                                    if isinstance(item, dict) else "")):
            raise RollbackError("parent rollback inventory is incomplete or unordered")
        for entry in entries:
            if (not isinstance(entry, dict) or set(entry) != {"path", "sha256"}
                    or not isinstance(entry["path"], str) or not entry["path"]
                    or not isinstance(entry["sha256"], str) or len(entry["sha256"]) != 64
                    or any(char not in "0123456789abcdef" for char in entry["sha256"])
                    or entry["path"] in paths):
                raise RollbackError("parent rollback inventory is invalid")
            paths.add(entry["path"])
    expected_receipt = {"state", "permissions", "localizationRequired", "missionResume",
                        "parentReleaseBindingSha256", "inventoryDigest"}
    if (not isinstance(receipt, dict) or set(receipt) != expected_receipt
            or receipt["state"] != "DISARMED" or receipt["permissions"] != "UNKNOWN"
            or receipt["localizationRequired"] is not True or receipt["missionResume"] is not False
            or receipt["parentReleaseBindingSha256"] != binding
            or receipt["inventoryDigest"] != _INSTALL._canonical_hash(inventory)):
        raise RollbackError("parent restart receipt is not disarmed, unknown, and no-resume")
    return receipt_hash, _INSTALL._canonical_hash(inventory)


def _validate_bundle(manifest):
    _INSTALL._authority_is_software_only(manifest)
    hashes = manifest.get("hashes")
    for name in ("configuration", "maps", "routes"):
        category = hashes.get(name) if isinstance(hashes, dict) else None
        if (not isinstance(category, dict) or not isinstance(category.get("digest"), str)
                or len(category["digest"]) != 64 or not category.get("files")):
            raise RollbackError("release has missing or mixed {} authority".format(name))


def _release(prefix, release_id, verifier):
    release_id = _INSTALL._safe_id(release_id)
    path = prefix / "releases" / release_id
    if not path.is_dir() or path.is_symlink():
        raise RollbackError("rollback release is missing or unsafe")
    manifest_path = path / "release-manifest.json"
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise RollbackError("release manifest is missing or unsafe")
    manifest = verifier(manifest_path, path)
    if not isinstance(manifest, dict) or manifest.get("release_binding_sha256") != release_id:
        raise RollbackError("release directory and manifest binding differ")
    _validate_bundle(manifest)
    return path, manifest


def rollback_release(prefix, target_release, disarmed_evidence, apply=False, verifier=None,
                     interrupt_hook=None):
    prefix = _INSTALL._check_prefix(prefix, apply)
    if not prefix.is_dir():
        raise RollbackError("release prefix is missing")
    verifier = verifier or _INSTALL._default_verifier
    current_id = _INSTALL._current_release(prefix)
    if current_id is None:
        raise RollbackError("there is no current release")
    target_id = _INSTALL._safe_id(target_release)
    current_path, current_manifest = _release(prefix, current_id, verifier)
    target_path, target_manifest = _release(prefix, target_id, verifier)
    evidence_sha256 = _evidence(disarmed_evidence, current_id, target_id)

    idempotent = current_id == target_id
    if not idempotent:
        receipt_hash, inventory_digest = _parent_rollback(current_manifest, target_id)
    result = {"action": "rollback", "applied": bool(apply), "from_release": current_id,
              "to_release": target_id, "disarmed_evidence_sha256": evidence_sha256,
              "state": "DISARMED", "idempotent": idempotent}
    if not idempotent:
        result["parent_release_binding_receipt_sha256"] = receipt_hash
        result["parent_inventory_digest"] = inventory_digest
    if not apply:
        return result

    if interrupt_hook:
        interrupt_hook("verified", target_path)
    if not idempotent:
        temporary_link = prefix / (".current-rollback-" + uuid.uuid4().hex)
        os.symlink("releases/" + target_id, str(temporary_link))
        try:
            os.replace(str(temporary_link), str(prefix / "current"))
        finally:
            try:
                temporary_link.unlink()
            except FileNotFoundError:
                pass
    receipt = dict(result)
    receipt["current_manifest_sha256"] = current_id
    receipt["target_manifest_sha256"] = target_manifest["release_binding_sha256"]
    receipt["services_modified"] = False
    receipt["armed_state"] = "DISARMED"
    _INSTALL._atomic_json(prefix / "receipts" / ("rollback-{}-to-{}.json".format(current_id, target_id)), receipt)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", required=True, type=Path)
    parser.add_argument("--target", required=True)
    parser.add_argument("--disarmed-evidence", required=True)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = rollback_release(args.prefix, args.target, args.disarmed_evidence,
                                  args.apply)
    except (RollbackError, OSError) as exc:
        parser.exit(2, "rollback refused: {}\n".format(exc))
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
