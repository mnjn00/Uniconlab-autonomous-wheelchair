#!/usr/bin/env python3
"""Stage and atomically select a verified, software-only Noetic release."""

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import tempfile
import sys
import uuid
from pathlib import Path


class InstallError(ValueError):
    """A release is unsafe or cannot be installed."""


def _default_verifier(manifest_path, root, release_signing_key):
    verifier_path = Path(__file__).with_name("verify_release_manifest.py")
    if not verifier_path.is_file():
        raise InstallError("release verifier is unavailable")
    script_directory = str(verifier_path.parent)
    added_path = script_directory not in sys.path
    if added_path:
        sys.path.insert(0, script_directory)
    try:
        spec = importlib.util.spec_from_file_location("wheelchair_release_verifier", str(verifier_path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except (ImportError, OSError) as exc:
        raise InstallError("release verifier is unavailable: {}".format(exc)) from exc
    finally:
        if added_path:
            sys.path.remove(script_directory)
    try:
        return module.verify_manifest(Path(manifest_path), Path(root), release_signing_key)
    except module.ManifestError as exc:
        raise InstallError(str(exc)) from exc


def _canonical_hash(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _safe_id(value):
    if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise InstallError("release binding must be a lowercase SHA-256")
    return value


def _authority_is_software_only(manifest):
    authority = manifest.get("authority")
    required_false = (
        "hardware_motion_authorized",
        "passenger_operation_authorized",
        "physical_authority",
        "simulation_or_replay_is_physical_evidence",
    )
    if not isinstance(authority, dict):
        raise InstallError("manifest authority is missing")
    if authority.get("software_release_candidate") is not True or authority.get("clean_release_authority") is not True:
        raise InstallError("manifest lacks software release authority")
    if any(authority.get(key) is not False for key in required_false):
        raise InstallError("manifest must explicitly deny hardware and passenger authority")

    def walk(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == "hardware_enabled" and child is not False:
                    raise InstallError("hardware_enabled must be false")
                walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)

    walk(manifest)


def _manifest_files(manifest):
    paths = []
    seen = set()
    hashes = manifest.get("hashes")
    if not isinstance(hashes, dict):
        raise InstallError("manifest hashes are missing")
    for category in hashes.values():
        if not isinstance(category, dict) or not isinstance(category.get("files"), list):
            raise InstallError("malformed manifest hash category")
        for entry in category["files"]:
            if not isinstance(entry, dict):
                raise InstallError("malformed manifest file entry")
            relative = _safe_relative(entry.get("path"))
            collision = relative.as_posix().casefold()
            if collision in seen:
                raise InstallError("duplicate or colliding manifest path: " + relative.as_posix())
            seen.add(collision)
            paths.append(relative)
    reports = manifest.get("test_reports")
    if not isinstance(reports, list) or not reports:
        raise InstallError("manifest test reports are missing")
    for entry in reports:
        if not isinstance(entry, dict):
            raise InstallError("malformed test report entry")
        relative = _safe_relative(entry.get("path"))
        if relative.as_posix().casefold() not in seen:
            raise InstallError("test report is not in the bound inventory: " + relative.as_posix())
    return sorted(paths, key=lambda item: item.as_posix())


def _safe_relative(value):
    if not isinstance(value, str) or not value:
        raise InstallError("manifest contains an empty path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != value or value.startswith("/"):
        raise InstallError("manifest path escapes release root: {}".format(value))
    lowered = tuple(part.lower() for part in path.parts)
    if path.suffix.lower() in {".bag", ".db3", ".mcap"} or any(
            part in {"bag", "bags", "rosbag", "rosbags"} for part in lowered):
        raise InstallError("release installation never copies bag data: {}".format(value))
    return path


def _check_prefix(prefix, apply):
    prefix = Path(prefix).expanduser()
    if not prefix.is_absolute():
        raise InstallError("prefix must be an absolute caller-selected path")
    if apply and hasattr(os, "geteuid") and os.geteuid() == 0:
        raise InstallError("release installation must not run as root")
    if prefix.exists() and prefix.is_symlink():
        raise InstallError("prefix must not be a symlink")
    return prefix


def _copy_bound_files(source, staging, manifest, manifest_path):
    source = source.resolve()
    for relative in _manifest_files(manifest):
        src = source / relative
        try:
            resolved_src = src.resolve(strict=True)
            resolved_src.relative_to(source)
        except (FileNotFoundError, ValueError) as exc:
            raise InstallError("bound source is missing or escapes release root: {}".format(relative)) from exc
        if resolved_src != src or not src.is_file():
            raise InstallError("bound source is a symlink or unsafe: {}".format(relative))
        destination = staging / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(src), str(destination))
    shutil.copyfile(str(manifest_path), str(staging / "release-manifest.json"))


def _atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, str(path))
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _current_release(prefix):
    current = prefix / "current"
    if not current.exists() and not current.is_symlink():
        return None
    if not current.is_symlink():
        raise InstallError("current selector is not a symlink")
    target = os.readlink(str(current))
    parts = Path(target).parts
    if len(parts) != 2 or parts[0] != "releases":
        raise InstallError("current selector has an unsafe target")
    return _safe_id(parts[1])


def install_release(source, manifest_path, prefix, apply=False, verifier=None, interrupt_hook=None,
                    release_signing_key=None):
    source = Path(source).resolve()
    manifest_path = Path(manifest_path).resolve()
    prefix = _check_prefix(prefix, apply)
    if release_signing_key is None:
        raise InstallError("installation requires an explicitly supplied release signing key")
    if not source.is_dir() or not manifest_path.is_file():
        raise InstallError("source root or release manifest is missing")
    custom_verifier = verifier is not None
    verifier = verifier or _default_verifier

    def verify_bound(candidate, candidate_root):
        if custom_verifier:
            return verifier(candidate, candidate_root)
        return verifier(candidate, candidate_root, release_signing_key)

    manifest = verify_bound(manifest_path, source)
    if not isinstance(manifest, dict):
        raise InstallError("release verifier returned no verified manifest")
    _authority_is_software_only(manifest)
    release_id = _safe_id(manifest.get("release_binding_sha256"))
    unsigned = {
        key: value
        for key, value in manifest.items()
        if key not in {"release_binding_sha256", "release_signature_hmac_sha256"}
    }
    if _canonical_hash(unsigned) != release_id:
        raise InstallError("release binding does not match manifest")
    files = [item.as_posix() for item in _manifest_files(manifest)]
    previous = _current_release(prefix) if prefix.exists() else None
    result = {"action": "install", "applied": bool(apply), "release": release_id,
              "previous_release": previous, "files": files, "state": "DISARMED"}
    if not apply:
        return result

    releases = prefix / "releases"
    releases.mkdir(parents=True, exist_ok=True)
    final = releases / release_id
    if final.exists():
        if final.is_symlink() or not final.is_dir():
            raise InstallError("release destination is unsafe")
        installed = verify_bound(final / "release-manifest.json", final)
        if installed.get("release_binding_sha256") != release_id:
            raise InstallError("existing release does not match requested release")
    else:
        staging = releases / ("." + release_id + ".staging-" + uuid.uuid4().hex)
        staging.mkdir(mode=0o755)
        _copy_bound_files(source, staging, manifest, manifest_path)
        verify_bound(staging / "release-manifest.json", staging)
        if interrupt_hook:
            interrupt_hook("staged", staging)
        os.replace(str(staging), str(final))

    if interrupt_hook:
        interrupt_hook("released", final)
    temporary_link = prefix / (".current-" + uuid.uuid4().hex)
    os.symlink("releases/" + release_id, str(temporary_link))
    try:
        os.replace(str(temporary_link), str(prefix / "current"))
    finally:
        try:
            temporary_link.unlink()
        except FileNotFoundError:
            pass
    receipt = dict(result)
    receipt["manifest_sha256"] = release_id
    _atomic_json(prefix / "receipts" / ("install-" + release_id + ".json"), receipt)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--prefix", required=True, type=Path)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--release-signing-key", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = install_release(
            args.source,
            args.manifest,
            args.prefix,
            args.apply,
            release_signing_key=args.release_signing_key,
        )
    except (InstallError, OSError) as exc:
        parser.exit(2, "installation refused: {}\n".format(exc))
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
