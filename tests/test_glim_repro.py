"""ROS-free contracts for the executable, offline-only GLIM pipeline."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
import struct
import subprocess
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
DOCKERFILE = ROOT / "tools/offline/Dockerfile.glim"
CONFIG = ROOT / "tools/offline/glim-config.json"
RUNNER_PATH = ROOT / "scripts/run_glim_repro.py"
CONVERTER_PATH = ROOT / "scripts/export_glim_rosbag2.py"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


runner = load("glim_runner", RUNNER_PATH)
converter = load("glim_converter", CONVERTER_PATH)


class GlimStaticContracts(unittest.TestCase):
    def test_dockerfile_builds_both_exact_pinned_sources_and_real_cli(self):
        text = DOCKERFILE.read_text()
        self.assertIn("d0eeebead1ab8240edf3645682ec12d79fbfa70a", text)
        self.assertIn("a62811dc3ab73076f4a43fc21005f96cd712903c", text)
        self.assertIn("github.com/koide3/glim.git", text)
        self.assertIn("github.com/koide3/glim_ros2.git", text)
        self.assertIn("colcon build", text)
        self.assertIn("test -x /opt/glim_ws/install/lib/glim_ros/glim_rosbag", text)
        self.assertIn('wheelchair.nuc-runtime="prohibited"', text)

    def test_config_selects_exact_topics_and_cpu_modules(self):
        value = json.loads(CONFIG.read_text())
        self.assertEqual(value["glim_ros"]["points_topic"], "/sensors/lidar/points")
        self.assertEqual(value["glim_ros"]["imu_topic"], "/sensors/imu/data")
        self.assertEqual(value["global"]["config_odometry"], "config_odometry_cpu.json")
        self.assertEqual(value["glim_ros"]["playback_speed"], 0.0)

    def test_real_glim_rosbag_api_is_positional_and_ros_parameters(self):
        command = runner.actual_glim_command(Path("/input/rosbag2"), Path("/opt/glim-config"), Path("/output/glim-dump"))
        self.assertEqual(command[0:2], [runner.GLIM_ROSBAG_EXECUTABLE, "/input/rosbag2"])
        self.assertIn("--ros-args", command)
        self.assertIn("config_path:=/opt/glim-config", command)
        self.assertIn("auto_quit:=true", command)
        self.assertIn("dump_path:=/output/glim-dump", command)
        for fictional in ("--bag", "--config", "--seed", "--threads"):
            self.assertNotIn(fictional, command)

    def test_container_is_networkless_read_only_and_has_no_noetic_runtime(self):
        args = type("Args", (), {"container_engine": "docker", "config": CONFIG, "image": "glim@sha256:" + "a" * 64})()
        command = runner.container_command(args, Path("/derived"), Path("/run"))
        self.assertIn("--network=none", command); self.assertIn("--read-only", command)
        self.assertIn("--cap-drop=ALL", command)
        self.assertIn("--user={}:{}".format(os.getuid(), os.getgid()), command)
        self.assertIn("--env=HOME=/tmp", command)
        combined = DOCKERFILE.read_text() + RUNNER_PATH.read_text() + CONVERTER_PATH.read_text()
        self.assertNotIn("ros:noetic", combined.lower())
        self.assertNotIn("/opt/ros/noetic", combined.lower())

    def test_production_runner_exposes_no_native_or_fake_executable_option(self):
        text = RUNNER_PATH.read_text()
        self.assertNotIn("--executable", text)
        self.assertNotIn("--test-command", text)
        self.assertNotIn("fake_glim", text)


class Rosbag2ValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.root = Path(self.temp.name)
        self.db = self.root / "normalized.db3"
        connection = sqlite3.connect(self.db)
        connection.executescript("CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT,offered_qos_profiles TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);")
        first, last = 100, 200
        for topic_id, (name, (kind, count)) in enumerate(runner.TOPICS.items(), 1):
            connection.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (topic_id, name, kind, "cdr", ""))
            timestamp = first if topic_id == 1 else last
            connection.executemany("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)", [(topic_id, timestamp, b"x")] * count)
        connection.commit()
        semantic = hashlib.sha256()
        for topic, timestamp, payload in connection.execute(
                "SELECT t.name,m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id ORDER BY m.timestamp,m.id"):
            semantic.update(struct.pack("<IQ", len(topic), timestamp)); semantic.update(topic.encode())
            semantic.update(struct.pack("<I", len(payload))); semantic.update(payload)
        connection.close()
        (self.root / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n")
        value = {"schema_version": 1, "artifact_id": "wheelchair.glim_rosbag2_input/v1", "nuc_runtime_artifact": False,
                 "source": {"normalized_bag_sha256": runner.PINNED_NORMALIZED_BAG_SHA256},
                 "output": {"format": "rosbag2-sqlite3", "database": self.db.name,
                            "database_sha256": runner.sha256_file(self.db), "metadata_sha256": runner.sha256_file(self.root / "metadata.yaml"),
                            "topics": {name: {"type": kind, "count": count} for name, (kind, count) in runner.TOPICS.items()},
                            "first_storage_time_ns": first, "last_storage_time_ns": last,
                            "ros2_semantic_stream_sha256": semantic.hexdigest()}}
        self.manifest = self.root / "glim_rosbag2_manifest.json"; self.manifest.write_text(json.dumps(value))

    def tearDown(self): self.temp.cleanup()

    def test_exact_types_counts_timestamps_and_hashes_are_validated(self):
        value, directory, database = runner.validate_ros2_manifest(self.manifest)
        self.assertEqual(directory, self.root); self.assertEqual(database, self.db)
        self.assertFalse(value["nuc_runtime_artifact"])

    def test_corrupt_database_is_rejected(self):
        with self.db.open("ab") as stream: stream.write(b"corrupt")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            runner.validate_ros2_manifest(self.manifest)


class ActualOutputTests(unittest.TestCase):
    def test_missing_real_dump_cannot_be_reported_as_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); (root / "dump").mkdir(); (root / "out").mkdir()
            with self.assertRaisesRegex(ValueError, "actual GLIM dump"):
                runner.derive_outputs(root / "dump", root / "out")

    def test_dump_trajectory_and_submap_are_required_before_map_derivation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); dump = root / "dump"; output = root / "output"; dump.mkdir(); output.mkdir()
            submap = dump / "000000"; submap.mkdir(); (submap / "points_compact.bin").write_bytes(b"points"); (submap / "data.txt").write_text("pose")
            (dump / "traj_lidar.txt").write_text("0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n2 2 0 0 0 0 0 1\n")
            def exported(command, check):
                target = Path(command[command.index("--output-dir") + 1]); target.mkdir()
                (target / "occupancy.pgm").write_bytes(b"P5\n1 1\n255\n\0")
                (target / "occupancy.yaml").write_text("image: occupancy.pgm\n")
                return subprocess.CompletedProcess(command, 0)
            with mock.patch.object(runner.subprocess, "run", side_effect=exported):
                evidence = runner.derive_outputs(dump, output)
            self.assertEqual(evidence, {"trajectory_rows": 3, "submap_count": 1})
            self.assertTrue((output / "trajectory.csv").is_file()); self.assertTrue((output / "occupancy.pgm").is_file())


class ConverterTransactionTests(unittest.TestCase):
    def test_corrupt_source_hash_leaves_no_partial_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); bag = root / "normalized.bag"; bag.write_bytes(b"corrupt")
            manifest = root / "normalization_manifest.yaml"
            manifest.write_text(json.dumps({"schema_version": 1, "artifact_id": "wheelchair.normalized_livox/v1",
                "output": {"bag_path": "normalized.bag", "sha256": "0" * 64, "format": "rosbag1-v2",
                           "topics": {name: {"type": kind, "count": count} for name, (kind, count) in converter.TOPICS.items()}}}))
            output = root / "derived"
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                converter.convert(manifest, output)
            self.assertFalse(output.exists())
            self.assertEqual(list(root.glob(".derived.unaccepted-*")), [])

    def test_converter_contract_hashes_ros1_and_ros2_semantic_streams(self):
        text = CONVERTER_PATH.read_text()
        self.assertIn("ros1_semantic_stream_sha256", text); self.assertIn("ros2_semantic_stream_sha256", text)
        self.assertIn("serialize_cdr", text); self.assertIn("deserialize_ros1", text)
        self.assertIn("os.replace(stage, output)", text)
        self.assertIn('"nuc_runtime_artifact": False', text)


if __name__ == "__main__": unittest.main()
