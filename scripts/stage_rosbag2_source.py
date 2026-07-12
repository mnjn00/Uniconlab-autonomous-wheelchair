#!/usr/bin/env python3
"""Explicitly stage a verified rosbag2 source without changing its files."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

from verify_rosbag2_manifest import _sha256, verify

MANIFEST_NAME = "rosbag2_source_manifest.json"


def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def _stat_identity(path: Path) -> tuple:
    lstat = path.lstat()
    stat = path.stat()
    return (
        lstat.st_dev, lstat.st_ino, lstat.st_mode, lstat.st_size, lstat.st_mtime_ns, lstat.st_ctime_ns,
        stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns,
        os.readlink(path) if path.is_symlink() else None,
    )


def stage(source: Path, output: Path, enforce_livox_expectations: bool = True) -> dict:
    source = source.expanduser().resolve()
    output = output.expanduser().resolve()
    if _is_within(output, source):
        return {"schema_version": "wheelchair.rosbag2_manifest/v1", "status": "error", "operation": "stage", "errors": [{"code": "E_TRANSACTION", "message": "staging directory must not be inside the immutable source"}]}
    if output.exists():
        return {"schema_version": "wheelchair.rosbag2_manifest/v1", "status": "error", "operation": "stage", "errors": [{"code": "E_TRANSACTION", "message": f"staging output already exists: {output}"}]}
    if not output.parent.is_dir():
        return {"schema_version": "wheelchair.rosbag2_manifest/v1", "status": "error", "operation": "stage", "errors": [{"code": "E_TRANSACTION", "message": f"staging parent does not exist: {output.parent}"}]}

    evidence = verify(source, include_hash=True, enforce_livox_expectations=enforce_livox_expectations)
    evidence["operation"] = "stage"
    if evidence["status"] not in {"verified", "staging_required"}:
        return evidence

    snapshot_paths = {source / "metadata.yaml"}
    for item in evidence["source"]["sqlite_inventory"]:
        snapshot_paths.add(Path(item["path"]))
        snapshot_paths.add(Path(item["resolved_path"]))
    source_snapshot = {str(path): _stat_identity(path) for path in snapshot_paths}
    try:
        output.mkdir(mode=0o755)
        metadata_destination = output / "metadata.yaml"
        shutil.copy2(source / "metadata.yaml", metadata_destination)
        declared = evidence["metadata"]["declared_relative_file_paths"]
        if len(declared) != len(evidence["segments"]):
            raise RuntimeError("declared and resolved sqlite segment counts differ")
        for relative, segment in zip(declared, evidence["segments"]):
            destination = output / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            os.symlink(segment["source_path"], destination)
            segment["staged_path"] = str(destination)
        evidence["staged"] = {
            "directory": str(output), "metadata_path": str(metadata_destination),
            "metadata_sha256": _sha256(metadata_destination), "manifest_path": str(output / MANIFEST_NAME),
            "link_policy": "canonical_declared_names_with_absolute_symlinks_to_immutable_resolved_source",
            "source_mutated": False,
        }
        evidence["status"] = "staged"
        current_snapshot = {
            path: _stat_identity(Path(path))
            for path in source_snapshot
        }
        if current_snapshot != source_snapshot:
            raise RuntimeError("source changed while staging")
        manifest_path = output / MANIFEST_NAME
        manifest_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        evidence["staged"]["manifest_sha256"] = _sha256(manifest_path)
        # The sidecar contains the evidence whose hash is reported on stdout; it cannot contain its own hash.
        return evidence
    except (OSError, RuntimeError) as exc:
        shutil.rmtree(output, ignore_errors=True)
        evidence["status"] = "error"
        evidence["errors"].append({"code": "E_TRANSACTION", "message": str(exc)})
        evidence["staged"] = None
        return evidence


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--no-pinned-expectations", action="store_true", help="stage any internally consistent rosbag2 sqlite source")
    parser.add_argument("--output-json", type=Path, help="also write returned evidence outside the staging directory")
    args = parser.parse_args(argv)
    evidence = stage(args.source, args.output, not args.no_pinned_expectations)
    text = json.dumps(evidence, indent=2, sort_keys=True) + "\n"
    if args.output_json:
        args.output_json.write_text(text, encoding="utf-8")
    sys.stdout.write(text)
    return 0 if evidence["status"] == "staged" else 2


if __name__ == "__main__":
    raise SystemExit(main())
