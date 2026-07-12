#!/usr/bin/env python3
"""Read-only rosbag2 sqlite inspection with machine-readable evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sqlite3
import sys
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

SCHEMA_VERSION = "wheelchair.rosbag2_manifest/v1"
EXPECTED_DURATION_NS = 688_225_098_527
EXPECTED_TOTAL = 144_484
EXPECTED_TOPICS = {
    "/livox/lidar": ("livox_ros_driver2/msg/CustomMsg", 6_882),
    "/livox/imu": ("sensor_msgs/msg/Imu", 137_602),
}
IDL_HASHES = {
    "custom_msg_sha256": "f42d6709db951b1fa307e929e742c0593cbf0d1b0ff977d2ed63ad8d7cee0a96",
    "custom_point_sha256": "b64b31a8edc8c8b3765d82b5d3ccd2d2e1f217b9525ef7007ab918674c619c59",
    "composite_sha256": "8d51083a4570d6e81f3193c9b8c39e16d2d5fb2d776dd198a997c7c5c6f4aac7",
    "verification": "contract_only_unverified_against_source",
}


class VerificationError(Exception):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _failure(code: str, message: str, **context: Any) -> dict[str, Any]:
    result: dict[str, Any] = {"code": code, "message": message}
    if context:
        result["context"] = context
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_metadata(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    if yaml is None:
        raise VerificationError("E_SOURCE_MANIFEST", "PyYAML is required to read metadata.yaml")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        info = document["rosbag2_bagfile_information"]
    except (OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise VerificationError("E_SOURCE_MANIFEST", f"invalid metadata.yaml: {exc}") from exc
    if not isinstance(info, dict):
        raise VerificationError("E_SOURCE_MANIFEST", "rosbag2_bagfile_information must be a mapping")
    return document, info


def _metadata_topics(info: dict[str, Any]) -> dict[str, dict[str, Any]]:
    topics: dict[str, dict[str, Any]] = {}
    try:
        entries = info["topics_with_message_count"]
        for entry in entries:
            metadata = entry["topic_metadata"]
            name = str(metadata["name"])
            if name in topics:
                raise VerificationError("E_SOURCE_MANIFEST", f"duplicate metadata topic {name}")
            topics[name] = {
                "name": name,
                "type": str(metadata["type"]),
                "serialization_format": str(metadata["serialization_format"]),
                "metadata_count": int(entry["message_count"]),
            }
    except VerificationError:
        raise
    except (KeyError, TypeError, ValueError) as exc:
        raise VerificationError("E_SOURCE_MANIFEST", f"invalid topic metadata: {exc}") from exc
    return topics


def _declared_paths(info: dict[str, Any]) -> list[str]:
    paths = info.get("relative_file_paths")
    if not isinstance(paths, list) or not paths or not all(isinstance(item, str) for item in paths):
        raise VerificationError("E_SOURCE_DISCOVERY", "metadata has no valid relative_file_paths")
    if any(Path(item).is_absolute() or ".." in Path(item).parts for item in paths):
        raise VerificationError("E_SOURCE_DISCOVERY", "metadata contains an unsafe relative file path")
    return paths


def _physical_identity(path: Path) -> tuple[int, int]:
    stat = path.stat()
    return stat.st_dev, stat.st_ino


def _resolve_segment(source: Path, path: Path) -> Path:
    if path.is_symlink():
        try:
            resolved = path.resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise VerificationError("E_SOURCE_DISCOVERY", f"sqlite symlink is dangling or cyclic: {path.name}") from exc
        try:
            resolved.relative_to(source)
        except ValueError as exc:
            raise VerificationError("E_SOURCE_DISCOVERY", f"sqlite symlink escapes source directory: {path.name}") from exc
        if not resolved.is_file():
            raise VerificationError("E_SOURCE_DISCOVERY", f"sqlite symlink target is not a file: {path.name}")
        return resolved
    if not path.is_file():
        raise VerificationError("E_SOURCE_DISCOVERY", f"sqlite segment is not a regular file: {path.name}")
    return path


def _discover_segments(source: Path, declared: list[str]) -> tuple[list[Path], dict[str, str], list[dict[str, Any]]]:
    entries = sorted(source.glob("*.db3"))
    resolved_entries = [(path, _resolve_segment(source, path)) for path in entries]
    populated = [(path, target) for path, target in resolved_entries if target.stat().st_size > 0]
    mapping: dict[str, str] = {}
    mismatches: list[dict[str, Any]] = []
    used: set[tuple[int, int]] = set()
    selected: list[Path] = []
    for relative in declared:
        expected = source / relative
        if expected.is_symlink():
            actual = _resolve_segment(source, expected)
            if actual.stat().st_size == 0:
                raise VerificationError("E_SOURCE_DISCOVERY", f"declared sqlite symlink target is zero bytes: {relative}")
            mismatches.append(_failure(
                "E_SOURCE_DISCOVERY",
                "metadata-declared sqlite is a pre-existing source symlink; explicit staging preserves provenance",
                declared_path=str(expected), actual_path=str(actual),
                link_target=os.readlink(expected), mismatch_reason="pre_existing_source_symlink_alias",
                source_repair="pre_existing", verifier_mutated_source=False,
            ))
        elif expected.is_file() and expected.stat().st_size > 0:
            actual = expected
        elif expected.is_file() and expected.stat().st_size == 0:
            candidates_by_identity: dict[tuple[int, int], Path] = {}
            for path, target in populated:
                if path.name.startswith(expected.stem + "-"):
                    candidates_by_identity.setdefault(_physical_identity(target), target)
            candidates = list(candidates_by_identity.values())
            if len(candidates) != 1:
                raise VerificationError("E_SOURCE_DISCOVERY", f"zero-byte declared segment {relative} has {len(candidates)} populated replacements")
            actual = candidates[0]
            mismatches.append(_failure(
                "E_SOURCE_DISCOVERY",
                "metadata-declared sqlite is zero bytes; populated suffixed segment requires explicit staging",
                declared_path=str(expected), actual_path=str(actual), mismatch_reason="zero_byte_expected_populated_suffixed_segment",
                source_repair="staging_only", verifier_mutated_source=False,
            ))
        else:
            raise VerificationError("E_SOURCE_DISCOVERY", f"declared sqlite segment is missing: {relative}")
        identity = _physical_identity(actual)
        if identity in used:
            raise VerificationError("E_SOURCE_DISCOVERY", f"sqlite segment selected more than once: {actual.name}")
        used.add(identity)
        selected.append(actual)
        mapping[relative] = actual.name
    extras: dict[tuple[int, int], str] = {}
    for path, target in populated:
        identity = _physical_identity(target)
        if identity not in used:
            extras.setdefault(identity, path.name)
    if extras:
        raise VerificationError("E_SOURCE_DISCOVERY", f"unreferenced populated sqlite segments: {sorted(extras.values())}")
    return selected, mapping, mismatches


def _inspect_segment(path: Path, include_hash: bool, known_hash: str | None = None) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    uri = f"file:{path.resolve().as_posix()}?mode=ro&immutable=1"
    try:
        connection = sqlite3.connect(uri, uri=True)
        connection.execute("PRAGMA query_only=ON")
        table_rows = connection.execute("SELECT name, sql FROM sqlite_master WHERE type='table' ORDER BY name").fetchall()
        schema = {str(name): str(sql) for name, sql in table_rows}
        required = {"topics", "messages"}
        if not required.issubset(schema):
            raise VerificationError("E_SOURCE_MANIFEST", f"sqlite missing required tables: {sorted(required - set(schema))}")
        topic_columns = {row[1] for row in connection.execute("PRAGMA table_info(topics)")}
        message_columns = {row[1] for row in connection.execute("PRAGMA table_info(messages)")}
        if not {"id", "name", "type", "serialization_format"}.issubset(topic_columns):
            raise VerificationError("E_SOURCE_MANIFEST", "sqlite topics schema is unsupported")
        if not {"id", "topic_id", "timestamp", "data"}.issubset(message_columns):
            raise VerificationError("E_SOURCE_MANIFEST", "sqlite messages schema is unsupported")
        user_version = int(connection.execute("PRAGMA user_version").fetchone()[0])
        integrity = connection.execute("PRAGMA quick_check").fetchone()
        if not integrity or integrity[0] != "ok":
            raise VerificationError("E_SOURCE_MANIFEST", f"sqlite integrity check failed: {integrity}")
        raw_message_count = int(connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0])
        rows = connection.execute(
            "SELECT t.name,t.type,t.serialization_format,COUNT(m.id),MIN(m.timestamp),MAX(m.timestamp) "
            "FROM topics t LEFT JOIN messages m ON m.topic_id=t.id "
            "GROUP BY t.id,t.name,t.type,t.serialization_format ORDER BY t.name"
        ).fetchall()
    except sqlite3.DatabaseError as exc:
        raise VerificationError("E_SOURCE_MANIFEST", f"unreadable sqlite {path.name}: {exc}") from exc
    finally:
        if "connection" in locals():
            connection.close()
    topics = {
        str(row[0]): {
            "name": str(row[0]), "type": str(row[1]), "serialization_format": str(row[2]),
            "sqlite_count": int(row[3]), "first_storage_timestamp_ns": row[4], "last_storage_timestamp_ns": row[5],
        }
        for row in rows
    }
    counted_messages = sum(item["sqlite_count"] for item in topics.values())
    if counted_messages != raw_message_count:
        raise VerificationError("E_SOURCE_MANIFEST", "sqlite contains messages whose topic_id is not declared")
    first_values = [item["first_storage_timestamp_ns"] for item in topics.values() if item["first_storage_timestamp_ns"] is not None]
    last_values = [item["last_storage_timestamp_ns"] for item in topics.values() if item["last_storage_timestamp_ns"] is not None]
    segment = {
        "source_path": str(path.resolve()), "staged_path": None, "size_bytes": path.stat().st_size,
        "sha256": known_hash if include_hash and known_hash is not None else (_sha256(path) if include_hash else None),
        "sqlite_schema": {"user_version": user_version, "tables": schema},
        "message_count": raw_message_count,
        "starting_time_ns": min(first_values) if first_values else None,
        "ending_time_ns": max(last_values) if last_values else None,
    }
    return segment, topics


def verify(source: Path, include_hash: bool = False, enforce_livox_expectations: bool = True) -> dict[str, Any]:
    source = source.expanduser().resolve()
    evidence: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION, "status": "error", "operation": "verify",
        "source": {"directory": str(source), "metadata_path": str(source / "metadata.yaml"), "sqlite_inventory": []},
        "staged": None, "metadata": None, "segments": [], "topics": [], "totals": {},
        "expectations": {
            "duration_ns": EXPECTED_DURATION_NS, "duration_seconds": EXPECTED_DURATION_NS / 1e9,
            "message_count": EXPECTED_TOTAL,
            "topics": [{"name": name, "type": value[0], "message_count": value[1]} for name, value in sorted(EXPECTED_TOPICS.items())],
        } if enforce_livox_expectations else None,
        "provenance": {
            "manifest_schema": SCHEMA_VERSION, "storage_identifier": None, "ros_distro": None,
            "livox_driver_revision": None, "source_idl_hashes": IDL_HASHES,
        },
        "mismatches": [], "errors": [],
    }
    try:
        if not source.is_dir():
            raise VerificationError("E_SOURCE_DISCOVERY", f"source is not a directory: {source}")
        metadata_path = source / "metadata.yaml"
        if not metadata_path.is_file():
            raise VerificationError("E_SOURCE_DISCOVERY", "metadata.yaml is missing")
        _document, info = _load_metadata(metadata_path)
        if info.get("version") != 5:
            raise VerificationError("E_SCHEMA_VERSION", f"rosbag2 metadata version must be 5, got {info.get('version')!r}")
        storage = info.get("storage_identifier")
        if storage != "sqlite3":
            raise VerificationError("E_SOURCE_MANIFEST", f"storage_identifier must be sqlite3, got {storage!r}")
        evidence["provenance"]["storage_identifier"] = storage
        metadata_topics = _metadata_topics(info)
        declared = _declared_paths(info)
        metadata_files = info.get("files")
        if not isinstance(metadata_files, list) or {item.get("path") for item in metadata_files if isinstance(item, dict)} != set(declared):
            raise VerificationError("E_SOURCE_MANIFEST", "metadata files entries do not match relative_file_paths")
        metadata_files_by_path = {item["path"]: item for item in metadata_files}
        inventory_paths = sorted(source.glob("*.db3"))
        inventory_hashes: dict[tuple[int, int], str | None] = {}
        inventory: list[dict[str, Any]] = []
        for path in inventory_paths:
            target = _resolve_segment(source, path)
            identity = _physical_identity(target)
            if identity not in inventory_hashes:
                inventory_hashes[identity] = _sha256(target) if include_hash else None
            link_target = os.readlink(path) if path.is_symlink() else None
            inventory.append({
                "path": str(path), "resolved_path": str(target.resolve()), "is_symlink": path.is_symlink(),
                "link_target": link_target, "lstat_size_bytes": path.lstat().st_size,
                "stat_size_bytes": target.stat().st_size, "size_bytes": target.stat().st_size,
                "physical_identity": {"device": identity[0], "inode": identity[1]},
                "sha256": inventory_hashes[identity],
                "hash_status": "computed" if include_hash else "not_requested",
                "alias_status": "pre_existing_source_repair" if path.is_symlink() else "not_alias",
                "verifier_mutated_source": False,
            })
        evidence["source"]["sqlite_inventory"] = inventory
        actual_paths, mapping, discovery_mismatches = _discover_segments(source, declared)
        evidence["mismatches"].extend(discovery_mismatches)
        aggregate: dict[str, dict[str, Any]] = {}
        segment_file_checks: list[tuple[str, tuple[int, int, int], tuple[int, int | None, int]]] = []
        for relative, path in zip(declared, actual_paths):
            segment, segment_topics = _inspect_segment(path, include_hash, inventory_hashes.get(_physical_identity(path)))
            file_info = metadata_files_by_path[relative]
            file_start = int(file_info.get("starting_time", {}).get("nanoseconds_since_epoch", -1))
            file_duration = int(file_info.get("duration", {}).get("nanoseconds", -1))
            file_count = int(file_info.get("message_count", -1))
            segment_duration = (
                segment["ending_time_ns"] - segment["starting_time_ns"]
                if segment["starting_time_ns"] is not None and segment["ending_time_ns"] is not None
                else 0
            )
            segment_file_checks.append((
                relative,
                (file_count, file_start, file_duration),
                (segment["message_count"], segment["starting_time_ns"], segment_duration),
            ))
            evidence["segments"].append(segment)
            for name, topic in segment_topics.items():
                if name not in aggregate:
                    aggregate[name] = dict(topic)
                else:
                    current = aggregate[name]
                    if (current["type"], current["serialization_format"]) != (topic["type"], topic["serialization_format"]):
                        raise VerificationError("E_TOPIC_TYPE", f"topic definition changes between segments: {name}")
                    current["sqlite_count"] += topic["sqlite_count"]
                    times = [x for x in (current["first_storage_timestamp_ns"], topic["first_storage_timestamp_ns"]) if x is not None]
                    current["first_storage_timestamp_ns"] = min(times) if times else None
                    times = [x for x in (current["last_storage_timestamp_ns"], topic["last_storage_timestamp_ns"]) if x is not None]
                    current["last_storage_timestamp_ns"] = max(times) if times else None
        if set(aggregate) != set(metadata_topics):
            raise VerificationError("E_TOPIC_TYPE", f"metadata/sqlite topic sets differ: metadata={sorted(metadata_topics)}, sqlite={sorted(aggregate)}")
        for relative, metadata_values, sqlite_values in segment_file_checks:
            if metadata_values != sqlite_values:
                raise VerificationError("E_COUNT_DURATION", f"metadata files entry differs from sqlite segment: {relative}")
        for name in sorted(aggregate):
            actual = aggregate[name]
            expected = metadata_topics[name]
            actual["metadata_count"] = expected["metadata_count"]
            if actual["type"] != expected["type"]:
                raise VerificationError("E_TOPIC_TYPE", f"topic type mismatch for {name}: {expected['type']} != {actual['type']}")
            if actual["serialization_format"] != "cdr" or expected["serialization_format"] != "cdr":
                raise VerificationError("E_SOURCE_MANIFEST", f"serialization format for {name} must be cdr")
            if actual["sqlite_count"] != expected["metadata_count"]:
                raise VerificationError("E_COUNT_DURATION", f"topic count mismatch for {name}: {expected['metadata_count']} != {actual['sqlite_count']}")
        total = sum(item["sqlite_count"] for item in aggregate.values())
        first = min(item["first_storage_timestamp_ns"] for item in aggregate.values() if item["first_storage_timestamp_ns"] is not None)
        last = max(item["last_storage_timestamp_ns"] for item in aggregate.values() if item["last_storage_timestamp_ns"] is not None)
        duration = last - first
        metadata_total = int(info.get("message_count", -1))
        metadata_start = int(info.get("starting_time", {}).get("nanoseconds_since_epoch", -1))
        metadata_duration = int(info.get("duration", {}).get("nanoseconds", -1))
        if (metadata_total, metadata_start, metadata_duration) != (total, first, duration):
            raise VerificationError("E_COUNT_DURATION", f"metadata/sqlite totals or bounds differ: metadata=({metadata_total},{metadata_start},{metadata_duration}) sqlite=({total},{first},{duration})")
        if enforce_livox_expectations:
            expected_shape = {name: (kind, count) for name, (kind, count) in EXPECTED_TOPICS.items()}
            actual_shape = {name: (item["type"], item["sqlite_count"]) for name, item in aggregate.items()}
            if total != EXPECTED_TOTAL or duration != EXPECTED_DURATION_NS or actual_shape != expected_shape:
                raise VerificationError("E_SOURCE_MANIFEST", "bag does not match the pinned Livox count/type/duration expectations")
        evidence["metadata"] = {
            "version": info.get("version"), "storage_identifier": storage, "declared_relative_file_paths": declared,
            "resolved_relative_file_paths": [mapping[item] for item in declared], "message_count": metadata_total,
            "starting_time_ns": metadata_start, "duration_ns": metadata_duration,
            "size_bytes": metadata_path.stat().st_size,
            "sha256": _sha256(metadata_path) if include_hash else None,
            "hash_status": "computed" if include_hash else "not_requested",
        }
        evidence["topics"] = [aggregate[name] for name in sorted(aggregate)]
        evidence["totals"] = {"message_count": total, "starting_time_ns": first, "ending_time_ns": last, "duration_ns": duration, "duration_seconds": duration / 1e9}
        evidence["status"] = "staging_required" if discovery_mismatches else "verified"
    except VerificationError as exc:
        evidence["errors"].append(_failure(exc.code, str(exc)))
    except (OSError, ValueError, TypeError) as exc:
        evidence["errors"].append(_failure("E_SOURCE_MANIFEST", str(exc)))
    return evidence


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="rosbag2 directory containing metadata.yaml")
    parser.add_argument("--hash", action="store_true", help="stream SHA-256 for metadata and sqlite segments")
    parser.add_argument("--no-pinned-expectations", action="store_true", help="verify internal consistency without the pinned Livox dataset totals")
    parser.add_argument("--output", type=Path, help="write JSON evidence here instead of stdout")
    args = parser.parse_args(argv)
    evidence = verify(args.source, args.hash, not args.no_pinned_expectations)
    text = json.dumps(evidence, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0 if evidence["status"] in {"verified", "staging_required"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
