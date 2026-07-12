#!/usr/bin/env python3
"""Convert the immutable canonical ROS 1 bag to a hash-bound ROS 2 GLIM input."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import struct
import sys
import copy
import tempfile

TOPICS = {
    "/sensors/lidar/points": ("sensor_msgs/msg/PointCloud2", 6882),
    "/sensors/imu/data": ("sensor_msgs/msg/Imu", 137602),
}
ROSBAGS_VERSION = "0.10.11"
PINNED_NORMALIZED_BAG_SHA256 = "b317642b44140629b3447f5744adb068f74c57a58a3f6498df59ac582d8d8aa5"
CANONICAL_LIDAR_FIELDS = [
    ("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1),
    ("intensity", 12, 7, 1), ("offset_time", 16, 6, 1),
    ("line", 20, 2, 1), ("tag", 21, 2, 1),
    ("reflectivity", 22, 2, 1), ("lidar_id", 23, 2, 1),
]
GLIM_LIDAR_FIELDS = CANONICAL_LIDAR_FIELDS[:4] + [("t", 16, 6, 1)] + CANONICAL_LIDAR_FIELDS[5:]
GLIM_POINT_TIME_COMPATIBILITY_CONTRACT = {
    "schema_version": 1,
    "scope": "derived_glim_rosbag2_only",
    "translation": "pointcloud2_offset_time_to_t_metadata_only",
    "preserves": ["data_bytes", "point_order", "header_timestamp", "storage_timestamp", "uint32_values"],
    "source_field_abi": {"fields": [list(field) for field in CANONICAL_LIDAR_FIELDS], "point_step": 24},
    "derived_field_abi": {"fields": [list(field) for field in GLIM_LIDAR_FIELDS], "point_step": 24},
}
GLIM_POINT_TIME_COMPATIBILITY = {
    "contract": GLIM_POINT_TIME_COMPATIBILITY_CONTRACT,
    "contract_sha256": hashlib.sha256(
        (json.dumps(GLIM_POINT_TIME_COMPATIBILITY_CONTRACT, sort_keys=True,
                    separators=(",", ":"), allow_nan=False) + "\n").encode()).hexdigest(),
}


def point_field_abi(fields):
    try:
        result = tuple((field.name, field.offset, field.datatype, field.count) for field in fields)
    except (AttributeError, TypeError) as exc:
        raise ValueError("lidar PointCloud2 fields are malformed") from exc
    if any(not isinstance(name, str) or isinstance(offset, bool) or isinstance(datatype, bool)
           or isinstance(count, bool) or not all(isinstance(value, int)
           for value in (offset, datatype, count)) for name, offset, datatype, count in result):
        raise ValueError("lidar PointCloud2 fields are malformed")
    return result


def validate_canonical_lidar(message):
    try:
        fields = message.fields
    except AttributeError as exc:
        raise ValueError("lidar PointCloud2 fields are malformed") from exc
    if point_field_abi(fields) != tuple(CANONICAL_LIDAR_FIELDS):
        raise ValueError("lidar PointCloud2 canonical field ABI mismatch")
    try:
        height, width = message.height, message.width
        point_step, row_step = message.point_step, message.row_step
        is_bigendian, is_dense = message.is_bigendian, message.is_dense
        data = bytes(message.data)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("lidar PointCloud2 layout is malformed") from exc
    if (isinstance(height, bool) or isinstance(width, bool) or isinstance(point_step, bool)
            or isinstance(row_step, bool) or not all(isinstance(value, int)
            for value in (height, width, point_step, row_step))):
        raise ValueError("lidar PointCloud2 dimensions are malformed")
    if (height != 1 or width < 0 or is_bigendian is not False or is_dense is not True
            or point_step != 24 or row_step != point_step * width or len(data) != row_step * height):
        raise ValueError("lidar PointCloud2 canonical layout mismatch")
    return data


def glim_lidar_message(message):
    """Return the derived-only GLIM view without changing canonical payload bytes."""
    data = validate_canonical_lidar(message)
    translated = copy.copy(message)
    fields = list(message.fields)
    time_field = copy.copy(fields[4])
    try:
        time_field.name = "t"
    except (AttributeError, TypeError):
        time_field = type(time_field)("t", 16, 6, 1)
    fields[4] = time_field
    translated.fields = fields
    if point_field_abi(translated.fields) != tuple(GLIM_LIDAR_FIELDS) or bytes(translated.data) != data:
        raise ValueError("derived GLIM PointCloud2 translation mismatch")
    return translated


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical(value) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def load_source(manifest_path: Path):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    output = manifest.get("output", {})
    if manifest.get("schema_version") != 1 or manifest.get("artifact_id") != "wheelchair.normalized_livox/v1":
        raise ValueError("unsupported normalization manifest identity")
    expected = {name: {"type": kind, "count": count} for name, (kind, count) in TOPICS.items()}
    if output.get("format") != "rosbag1-v2" or output.get("topics") != expected:
        raise ValueError("canonical ROS 1 topic ABI mismatch")
    relative = output.get("bag_path")
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError("output.bag_path must be relative")
    bag = (manifest_path.parent / relative).resolve()
    bag.relative_to(manifest_path.parent.resolve())
    observed_bag_hash = sha256_file(bag) if bag.is_file() else None
    if observed_bag_hash != output.get("sha256"):
        raise ValueError("normalized bag SHA-256 mismatch")
    if observed_bag_hash != PINNED_NORMALIZED_BAG_SHA256:
        raise ValueError("normalized bag is not the pinned canonical A10 artifact")
    report_path = manifest_path.parent / "conversion_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    semantic = report.get("semantic_stream_sha256")
    if not isinstance(semantic, str) or len(semantic) != 64:
        raise ValueError("normalization report has no semantic stream hash")
    return manifest, bag, report_path, semantic


def metadata(start: int, end: int, database: str) -> str:
    duration = end - start
    lines = [
        "rosbag2_bagfile_information:", "  version: 5", "  storage_identifier: sqlite3",
        "  duration:", f"    nanoseconds: {duration}", "  starting_time:", f"    nanoseconds_since_epoch: {start}",
        f"  message_count: {sum(x[1] for x in TOPICS.values())}", "  topics_with_message_count:",
    ]
    for name, (kind, count) in TOPICS.items():
        lines += ["    - topic_metadata:", f"        name: {name}", f"        type: {kind}",
                  "        serialization_format: cdr", "        offered_qos_profiles: ''", f"      message_count: {count}"]
    lines += ["  compression_format: ''", "  compression_mode: ''", "  relative_file_paths:", f"    - {database}",
              "  files:", f"    - path: {database}", f"      starting_time:", f"        nanoseconds_since_epoch: {start}",
              f"      duration:", f"        nanoseconds: {duration}", f"      message_count: {sum(x[1] for x in TOPICS.values())}"]
    return "\n".join(lines) + "\n"


def convert(manifest_path: Path, output: Path) -> dict:
    manifest_path = manifest_path.resolve(strict=True)
    manifest, bag, report_path, expected_semantic = load_source(manifest_path)
    if output.exists():
        raise ValueError("output already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f".{output.name}.unaccepted-", dir=output.parent))
    try:
        try:
            import rosbags
            from rosbags.rosbag1 import Reader
            from rosbags.typesys import Stores, get_typestore
        except ImportError as exc:
            raise ValueError(f"rosbags=={ROSBAGS_VERSION} is required offline") from exc
        if getattr(rosbags, "__version__", ROSBAGS_VERSION) != ROSBAGS_VERSION:
            raise ValueError(f"rosbags=={ROSBAGS_VERSION} is required offline")
        ros1, ros2 = get_typestore(Stores.ROS1_NOETIC), get_typestore(Stores.ROS2_HUMBLE)
        database = "normalized.db3"
        connection = sqlite3.connect(str(stage / database))
        connection.executescript("""
PRAGMA journal_mode=DELETE;
PRAGMA synchronous=FULL;
CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
CREATE TABLE metadata(id INTEGER PRIMARY KEY, metadata_version INTEGER NOT NULL, metadata TEXT NOT NULL);
CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages(timestamp ASC);
""")
        connection.execute("INSERT INTO schema VALUES(4,'humble')")
        topic_ids = {}
        for topic_id, (name, (kind, _)) in enumerate(TOPICS.items(), 1):
            topic_ids[name] = topic_id
            connection.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (topic_id, name, kind, "cdr", ""))
        counts = {name: 0 for name in TOPICS}
        ros1_semantic, ros2_semantic = hashlib.sha256(), hashlib.sha256()
        first = last = previous = None
        with Reader(bag) as reader:
            selected = [item for item in reader.connections if item.topic in TOPICS]
            if {(item.topic, item.msgtype) for item in selected} != {(name, kind) for name, (kind, _) in TOPICS.items()}:
                raise ValueError("ROS 1 connections do not match canonical ABI")
            for item, timestamp, raw in reader.messages(connections=selected):
                if previous is not None and timestamp < previous:
                    raise ValueError("ROS 1 storage timestamps regress")
                previous = timestamp
                raw = bytes(raw)
                ros1_semantic.update(struct.pack("<IQ", len(item.topic), timestamp)); ros1_semantic.update(item.topic.encode())
                ros1_semantic.update(struct.pack("<I", len(raw))); ros1_semantic.update(raw)
                decoded = ros1.deserialize_ros1(raw, item.msgtype)
                if item.topic == "/sensors/lidar/points":
                    decoded = glim_lidar_message(decoded)
                cdr = bytes(ros2.serialize_cdr(decoded, item.msgtype))
                connection.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)", (topic_ids[item.topic], timestamp, cdr))
                ros2_semantic.update(struct.pack("<IQ", len(item.topic), timestamp)); ros2_semantic.update(item.topic.encode())
                ros2_semantic.update(struct.pack("<I", len(cdr))); ros2_semantic.update(cdr)
                counts[item.topic] += 1
                first = timestamp if first is None else first; last = timestamp
        expected_counts = {name: value[1] for name, value in TOPICS.items()}
        if counts != expected_counts or ros1_semantic.hexdigest() != expected_semantic:
            raise ValueError("ROS 1 count or semantic stream hash mismatch")
        source_output = manifest["output"]
        if first != source_output.get("first_storage_time_ns") or last != source_output.get("last_storage_time_ns"):
            raise ValueError("ROS 1 storage-time extent mismatch")
        connection.commit(); connection.close()
        (stage / "metadata.yaml").write_text(metadata(first, last, database), encoding="utf-8")
        result = {"schema_version": 1, "artifact_id": "wheelchair.glim_rosbag2_input/v1",
                  "execution_scope": "OFFLINE_WORKSTATION_ONLY", "nuc_runtime_artifact": False,
                  "source": {"normalization_manifest_sha256": sha256_file(manifest_path),
                             "normalized_bag_sha256": sha256_file(bag), "conversion_report_sha256": sha256_file(report_path),
                             "ros1_semantic_stream_sha256": expected_semantic},
                  "output": {"format": "rosbag2-sqlite3", "bag_path": ".", "database": database,
                             "database_sha256": sha256_file(stage / database), "metadata_sha256": sha256_file(stage / "metadata.yaml"),
                             "topics": {name: {"type": kind, "count": count} for name, (kind, count) in TOPICS.items()},
                             "first_storage_time_ns": first, "last_storage_time_ns": last,
                             "ros2_semantic_stream_sha256": ros2_semantic.hexdigest()},
                  "converter": {"rosbags_version": ROSBAGS_VERSION},
                  "compatibility": GLIM_POINT_TIME_COMPATIBILITY}
        (stage / "glim_rosbag2_manifest.json").write_bytes(canonical(result))
        os.replace(stage, output)
        return result
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        convert(args.input_manifest, args.output_dir)
    except (OSError, ValueError, sqlite3.DatabaseError) as exc:
        print(f"E_GLIM_ROSBAG2: {exc}", file=sys.stderr); return 2
    print(args.output_dir / "glim_rosbag2_manifest.json"); return 0


if __name__ == "__main__":
    raise SystemExit(main())
