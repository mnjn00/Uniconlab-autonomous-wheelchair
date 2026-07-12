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
    text = str(evidence)
    if text == "DISARMED":
        return hashlib.sha256(b"DISARMED").hexdigest()
    path = Path(text).expanduser()
    if path.is_file():
        raw = path.read_bytes()
        try:
            token = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RollbackError("disarmed evidence is not valid JSON") from exc
        if not isinstance(token, dict) or token.get("state") != "DISARMED":
            raise RollbackError("disarmed evidence must explicitly state DISARMED")
        if token.get("current_release") != current_release or token.get("target_release") != target_release:
            raise RollbackError("disarmed evidence is not bound to this rollback")
        if token.get("hardware_enabled") is not False or token.get("hardware_motion_authorized") is not False:
            raise RollbackError("disarmed evidence must explicitly deny hardware authority")
        binding = token.get("evidence_binding_sha256")
        expected = _INSTALL._canonical_hash({k: v for k, v in token.items() if k != "evidence_binding_sha256"})
        if binding != expected:
            raise RollbackError("disarmed evidence binding is invalid")
        return hashlib.sha256(raw).hexdigest()
    raise RollbackError("explicit DISARMED evidence token is required")


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
        rollback = current_manifest.get("rollback")
        if (not isinstance(rollback, dict) or rollback.get("parent") != target_id
                or rollback.get("parent_state") != "unarmed"):
            raise RollbackError("target is not the verified unarmed parent of current release")
    result = {"action": "rollback", "applied": bool(apply), "from_release": current_id,
              "to_release": target_id, "disarmed_evidence_sha256": evidence_sha256,
              "state": "DISARMED", "idempotent": idempotent}
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
