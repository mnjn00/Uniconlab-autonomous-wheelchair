import hashlib
import json
import os
import sqlite3
import sys
from pathlib import Path

import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from stage_rosbag2_source import stage  # noqa: E402
from verify_rosbag2_manifest import SCHEMA_VERSION, verify  # noqa: E402


def _metadata(filename, topics, start=100, duration=40):
    return {
        "rosbag2_bagfile_information": {
            "version": 5,
            "storage_identifier": "sqlite3",
            "duration": {"nanoseconds": duration},
            "starting_time": {"nanoseconds_since_epoch": start},
            "message_count": sum(topic[2] for topic in topics),
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": name,
                        "type": kind,
                        "serialization_format": serialization,
                        "offered_qos_profiles": "",
                    },
                    "message_count": count,
                }
                for name, kind, count, serialization in topics
            ],
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [filename],
            "files": [
                {
                    "path": filename,
                    "starting_time": {"nanoseconds_since_epoch": start},
                    "duration": {"nanoseconds": duration},
                    "message_count": sum(topic[2] for topic in topics),
                }
            ],
        }
    }


def _make_bag(tmp_path, *, declared="bag.db3", actual=None, metadata_topics=None, sqlite_topics=None, timestamps=None, duration=40):
    source = tmp_path / "source"
    source.mkdir()
    metadata_topics = metadata_topics or [("/lidar", "example/msg/Lidar", 2, "cdr"), ("/imu", "sensor_msgs/msg/Imu", 3, "cdr")]
    sqlite_topics = sqlite_topics or metadata_topics
    timestamps = timestamps or {"/lidar": [100, 140], "/imu": [110, 120, 130]}
    (source / "metadata.yaml").write_text(yaml.safe_dump(_metadata(declared, metadata_topics, duration=duration), sort_keys=False), encoding="utf-8")
    sqlite_name = actual or declared
    database = source / sqlite_name
    connection = sqlite3.connect(database)
    connection.executescript(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT NOT NULL,type TEXT NOT NULL,serialization_format TEXT NOT NULL,offered_qos_profiles TEXT NOT NULL);"
        "CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER NOT NULL,timestamp INTEGER NOT NULL,data BLOB NOT NULL);"
    )
    row_id = 1
    for topic_id, (name, kind, _count, serialization) in enumerate(sqlite_topics, 1):
        connection.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (topic_id, name, kind, serialization, ""))
        for timestamp in timestamps.get(name, []):
            connection.execute("INSERT INTO messages VALUES(?,?,?,?)", (row_id, topic_id, timestamp, b"payload"))
            row_id += 1
    connection.commit()
    connection.close()
    return source


def _tree_digest(root):
    result = {}
    for path in sorted(root.iterdir()):
        lstat = path.lstat()
        result[path.name] = (
            lstat.st_mode,
            lstat.st_size,
            lstat.st_mtime_ns,
            lstat.st_ctime_ns,
            os.readlink(path) if path.is_symlink() else None,
            path.stat().st_size,
            hashlib.sha256(path.read_bytes()).hexdigest(),
        )
    return result


def _error_code(evidence):
    assert evidence["status"] == "error"
    return evidence["errors"][0]["code"]


def test_exact_fixture_reports_schema_topics_bounds_and_optional_hashes(tmp_path):
    source = _make_bag(tmp_path)
    evidence = verify(source, include_hash=True, enforce_livox_expectations=False)
    assert evidence["schema_version"] == SCHEMA_VERSION
    assert evidence["status"] == "verified"
    assert evidence["totals"] == {
        "message_count": 5,
        "starting_time_ns": 100,
        "ending_time_ns": 140,
        "duration_ns": 40,
        "duration_seconds": 4e-08,
    }
    assert {topic["name"]: topic["sqlite_count"] for topic in evidence["topics"]} == {"/imu": 3, "/lidar": 2}
    assert evidence["segments"][0]["sha256"]
    assert evidence["segments"][0]["sqlite_schema"]["user_version"] == 0
    inventory = evidence["source"]["sqlite_inventory"]
    assert len(inventory) == 1
    assert inventory[0]["path"] == str(source / "bag.db3")
    assert inventory[0]["resolved_path"] == str(source / "bag.db3")
    assert inventory[0]["is_symlink"] is False
    assert inventory[0]["alias_status"] == "not_alias"


def test_pre_existing_symlink_alias_is_distinct_deduplicated_and_stageable(tmp_path):
    source = _make_bag(tmp_path, declared="bag.db3", actual="bag-001.db3")
    (source / "bag.db3").symlink_to("bag-001.db3")
    before = _tree_digest(source)

    evidence = verify(source, include_hash=True, enforce_livox_expectations=False)

    assert evidence["status"] == "staging_required"
    assert [item["path"] for item in evidence["source"]["sqlite_inventory"]] == [
        str(source / "bag-001.db3"),
        str(source / "bag.db3"),
    ]
    alias = next(item for item in evidence["source"]["sqlite_inventory"] if item["is_symlink"])
    target = next(item for item in evidence["source"]["sqlite_inventory"] if not item["is_symlink"])
    assert alias["path"] != alias["resolved_path"] == target["resolved_path"]
    assert alias["link_target"] == "bag-001.db3"
    assert alias["lstat_size_bytes"] == len("bag-001.db3")
    assert alias["stat_size_bytes"] == target["stat_size_bytes"] > 0
    assert alias["physical_identity"] == target["physical_identity"]
    assert alias["sha256"] == target["sha256"]
    mismatch = evidence["mismatches"][0]["context"]
    assert mismatch["mismatch_reason"] == "pre_existing_source_symlink_alias"
    assert mismatch["source_repair"] == "pre_existing"
    assert mismatch["verifier_mutated_source"] is False

    staged = tmp_path / "staged-alias"
    result = stage(source, staged, enforce_livox_expectations=False)
    assert result["status"] == "staged"
    assert (staged / "bag.db3").is_symlink()
    assert os.readlink(staged / "bag.db3") == str(source / "bag-001.db3")
    assert result["staged"]["source_mutated"] is False
    assert _tree_digest(source) == before


def test_zero_byte_declared_with_multiple_physical_targets_fails_closed(tmp_path):
    source = _make_bag(tmp_path, declared="bag.db3", actual="bag-001.db3")
    (source / "bag.db3").touch()
    (source / "bag-002.db3").write_bytes((source / "bag-001.db3").read_bytes())

    evidence = verify(source, enforce_livox_expectations=False)

    assert _error_code(evidence) == "E_SOURCE_DISCOVERY"
    assert "2 populated replacements" in evidence["errors"][0]["message"]


@pytest.mark.parametrize("kind", ["escape", "cycle"])
def test_declared_symlink_escape_and_cycle_fail_closed_without_mutation(tmp_path, kind):
    source = _make_bag(tmp_path, declared="bag.db3", actual="bag-001.db3")
    alias = source / "bag.db3"
    if kind == "escape":
        outside = tmp_path / "outside.db3"
        outside.write_bytes((source / "bag-001.db3").read_bytes())
        alias.symlink_to(outside)
    else:
        alias.symlink_to("bag.db3")
    before = {
        path.name: (path.lstat().st_mode, path.lstat().st_size, os.readlink(path) if path.is_symlink() else None)
        for path in source.iterdir()
    }

    evidence = verify(source, enforce_livox_expectations=False)

    assert _error_code(evidence) == "E_SOURCE_DISCOVERY"
    expected_word = "escapes" if kind == "escape" else "cyclic"
    assert expected_word in evidence["errors"][0]["message"]
    after = {
        path.name: (path.lstat().st_mode, path.lstat().st_size, os.readlink(path) if path.is_symlink() else None)
        for path in source.iterdir()
    }
    assert after == before


def test_zero_byte_declared_and_populated_suffix_is_explicit_and_stageable(tmp_path):
    source = _make_bag(tmp_path, declared="bag.db3", actual="bag-001.db3")
    (source / "bag.db3").touch()
    before = _tree_digest(source)
    evidence = verify(source, enforce_livox_expectations=False)
    assert evidence["status"] == "staging_required"
    assert evidence["mismatches"][0]["code"] == "E_SOURCE_DISCOVERY"
    assert evidence["mismatches"][0]["context"]["mismatch_reason"] == "zero_byte_expected_populated_suffixed_segment"

    staged = tmp_path / "staged"
    result = stage(source, staged, enforce_livox_expectations=False)
    assert result["status"] == "staged"
    assert (staged / "metadata.yaml").read_bytes() == (source / "metadata.yaml").read_bytes()
    assert (staged / "bag.db3").is_symlink()
    assert (staged / "bag.db3").resolve() == (source / "bag-001.db3").resolve()
    assert json.loads((staged / "rosbag2_source_manifest.json").read_text())["status"] == "staged"
    assert _tree_digest(source) == before


def test_extra_sqlite_topic_fails_closed(tmp_path):
    metadata = [("/lidar", "example/msg/Lidar", 2, "cdr"), ("/imu", "sensor_msgs/msg/Imu", 3, "cdr")]
    sqlite = metadata + [("/extra", "std_msgs/msg/String", 1, "cdr")]
    source = _make_bag(tmp_path, metadata_topics=metadata, sqlite_topics=sqlite, timestamps={"/lidar": [100, 140], "/imu": [110, 120, 130], "/extra": [125]})
    assert _error_code(verify(source, enforce_livox_expectations=False)) == "E_TOPIC_TYPE"


@pytest.mark.parametrize(
    "metadata_topics,sqlite_topics,timestamps,duration,code",
    [
        (
            [("/lidar", "example/msg/Lidar", 1, "cdr"), ("/imu", "sensor_msgs/msg/Imu", 3, "cdr")],
            None,
            None,
            40,
            "E_COUNT_DURATION",
        ),
        (None, None, None, 39, "E_COUNT_DURATION"),
        (
            None,
            [("/lidar", "wrong/msg/Lidar", 2, "cdr"), ("/imu", "sensor_msgs/msg/Imu", 3, "cdr")],
            None,
            40,
            "E_TOPIC_TYPE",
        ),
        (
            None,
            [("/lidar", "example/msg/Lidar", 2, "json"), ("/imu", "sensor_msgs/msg/Imu", 3, "cdr")],
            None,
            40,
            "E_SOURCE_MANIFEST",
        ),
    ],
)
def test_count_time_type_and_serialization_corruption_fail_stably(tmp_path, metadata_topics, sqlite_topics, timestamps, duration, code):
    source = _make_bag(tmp_path, metadata_topics=metadata_topics, sqlite_topics=sqlite_topics, timestamps=timestamps, duration=duration)
    assert _error_code(verify(source, enforce_livox_expectations=False)) == code


def test_failed_verification_does_not_create_stage_or_mutate_source(tmp_path):
    source = _make_bag(tmp_path, duration=39)
    before = _tree_digest(source)
    output = tmp_path / "must-not-exist"
    evidence = stage(source, output, enforce_livox_expectations=False)
    assert evidence["status"] == "error"
    assert not output.exists()
    assert _tree_digest(source) == before
