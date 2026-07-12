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
import tempfile

TOPICS = {
    "/sensors/lidar/points": ("sensor_msgs/msg/PointCloud2", 6882),
    "/sensors/imu/data": ("sensor_msgs/msg/Imu", 137602),
}
ROSBAGS_VERSION = "0.10.11"
PINNED_NORMALIZED_BAG_SHA256 = "b317642b44140629b3447f5744adb068f74c57a58a3f6498df59ac582d8d8aa5"


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
                  "converter": {"rosbags_version": ROSBAGS_VERSION}}
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
