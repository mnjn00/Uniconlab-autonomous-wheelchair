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
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
DOCKERFILE = ROOT / "tools/offline/Dockerfile.glim"
CONFIG = ROOT / "tools/offline/glim-config.json"
DETERMINISTIC_CONFIG = ROOT / "tools/offline/glim-deterministic"
RUNNER_PATH = ROOT / "scripts/run_glim_repro.py"
CONVERTER_PATH = ROOT / "scripts/export_glim_rosbag2.py"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


runner = load("glim_runner", RUNNER_PATH)
converter = load("glim_converter", CONVERTER_PATH)
class CdrWriter:
    def __init__(self):
        self.data = bytearray(runner.CDR_LE)

    def primitive(self, code, value, alignment):
        self.data.extend(b"\0" * ((-(len(self.data) - 4)) % alignment))
        self.data.extend(struct.pack(code, value))

    def string(self, value):
        raw = value.encode() + b"\0"
        self.primitive("<I", len(raw), 4); self.data.extend(raw)


def lidar_cdr(times=(10, 20), *, bigendian=False, point_step=24, field_name="t", finite=True):
    writer = CdrWriter()
    writer.primitive("<i", 1, 4); writer.primitive("<I", 2, 4); writer.string("lidar")
    writer.primitive("<I", 1, 4); writer.primitive("<I", len(times), 4)
    writer.primitive("<I", len(runner.GLIM_LIDAR_FIELDS), 4)
    for name, offset, datatype, count in runner.GLIM_LIDAR_FIELDS:
        writer.string(field_name if name == "t" else name); writer.primitive("<I", offset, 4)
        writer.primitive("<B", datatype, 1); writer.primitive("<I", count, 4)
    writer.primitive("<?", bigendian, 1); writer.primitive("<I", point_step, 4)
    writer.primitive("<I", point_step * len(times), 4)
    point_x = 1.0 if finite else float("nan")
    data = b"".join(struct.pack("<ffffIBBBB", point_x, 2.0, 3.0, 4.0, time, 2, 3, 4, 5)
                    for time in times)
    writer.primitive("<I", len(data), 4); writer.data.extend(data)
    writer.primitive("<?", True, 1)
    return bytes(writer.data)


def imu_cdr(*, finite=True):
    writer = CdrWriter()
    writer.primitive("<i", 1, 4); writer.primitive("<I", 3, 4); writer.string("imu")
    for count in (4, 9, 3, 9, 3, 9):
        for _ in range(count):
            writer.primitive("<d", 0.0 if finite else float("nan"), 8)
    return bytes(writer.data)


def semantic_digest(connection):
    digest = hashlib.sha256()
    for topic, timestamp, payload in connection.execute(
            "SELECT t.name,m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
            "ORDER BY m.timestamp,m.id"):
        payload = bytes(payload)
        digest.update(struct.pack("<IQ", len(topic), timestamp)); digest.update(topic.encode())
        digest.update(struct.pack("<I", len(payload))); digest.update(payload)
    return digest.hexdigest()


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

    def test_deterministic_config_bundle_uses_fixed_seed_sampled_global_mapping_and_single_threads(self):
        entrypoint, mounts, digest = runner.validate_config_bundle(DETERMINISTIC_CONFIG)
        self.assertEqual(entrypoint, DETERMINISTIC_CONFIG / "config.json")
        self.assertEqual({name for _, name in mounts}, set(runner.DETERMINISTIC_CONFIG_FILES))
        self.assertEqual(len(digest), 64)
        manifest = json.loads((DETERMINISTIC_CONFIG / "manifest.json").read_text())
        self.assertEqual(manifest["determinism"], {
            "preprocess_random_grid": False,
            "preprocess_threads": 1,
            "odometry_target_downsampling_rate": 0.1,
            "odometry_threads": 1,
            "global_randomsampling_determinism": "fixed-seed/default-constructed-std::mt19937",
            "global_randomsampling_rate": 0.2,
        })

    def test_deterministic_config_bundle_rejects_tamper(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for source in DETERMINISTIC_CONFIG.iterdir():
                (root / source.name).write_bytes(source.read_bytes())
            (root / "config_preprocess.json").write_text("tampered")
            with self.assertRaisesRegex(ValueError, "file/hash mismatch"):
                runner.validate_config_bundle(root)

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
        _, mounts, _ = runner.validate_config_bundle(DETERMINISTIC_CONFIG)
        args = type("Args", (), {
            "container_engine": "docker", "config": DETERMINISTIC_CONFIG,
            "config_mounts": mounts, "image": "glim@sha256:" + "a" * 64,
        })()
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
            payload = lidar_cdr() if topic_id == 1 else imu_cdr()
            connection.executemany("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
                                   [(topic_id, timestamp, payload)] * count)
        connection.commit()
        semantic = semantic_digest(connection)
        connection.close()
        (self.root / "metadata.yaml").write_text(runner.expected_metadata(first, last, self.db.name))
        value = {"schema_version": 1, "artifact_id": "wheelchair.glim_rosbag2_input/v1", "nuc_runtime_artifact": False,
                 "source": {"normalized_bag_sha256": runner.PINNED_NORMALIZED_BAG_SHA256},
                 "compatibility": runner.GLIM_POINT_TIME_COMPATIBILITY,
                 "output": {"format": "rosbag2-sqlite3", "database": self.db.name,
                            "database_sha256": runner.sha256_file(self.db), "metadata_sha256": runner.sha256_file(self.root / "metadata.yaml"),
                            "topics": {name: {"type": kind, "count": count} for name, (kind, count) in runner.TOPICS.items()},
                            "first_storage_time_ns": first, "last_storage_time_ns": last,
                            "ros2_semantic_stream_sha256": semantic}}
        self.manifest = self.root / "glim_rosbag2_manifest.json"; self.manifest.write_text(json.dumps(value))

    def tearDown(self): self.temp.cleanup()
    def rebind(self):
        connection = sqlite3.connect(self.db)
        connection.commit()
        semantic = semantic_digest(connection)
        connection.close()
        value = json.loads(self.manifest.read_text())
        value["output"]["database_sha256"] = runner.sha256_file(self.db)
        value["output"]["metadata_sha256"] = runner.sha256_file(self.root / "metadata.yaml")
        value["output"]["ros2_semantic_stream_sha256"] = semantic
        self.manifest.write_text(json.dumps(value))

    def replace_topic_payload(self, topic, payload):
        connection = sqlite3.connect(self.db)
        connection.execute(
            "UPDATE messages SET data=? WHERE topic_id=(SELECT id FROM topics WHERE name=?)",
            (payload, topic))
        connection.commit(); connection.close()
        self.rebind()

    def test_cdr_deserializers_reject_opaque_malformed_and_wrong_field(self):
        with self.assertRaisesRegex(ValueError, "encapsulation"):
            runner.validate_lidar_cdr(b"x")
        with self.assertRaisesRegex(ValueError, "encapsulation"):
            runner.validate_imu_cdr(b"x")
        with self.assertRaisesRegex(ValueError, "field ABI"):
            runner.validate_lidar_cdr(lidar_cdr(field_name="q"))
        with self.assertRaisesRegex(ValueError, "layout"):
            runner.validate_lidar_cdr(lidar_cdr(bigendian=True))
        with self.assertRaisesRegex(ValueError, "layout"):
            runner.validate_lidar_cdr(lidar_cdr(point_step=20))
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            runner.validate_lidar_cdr(lidar_cdr(finite=False))
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            runner.validate_imu_cdr(imu_cdr(finite=False))

    def test_manifest_rejects_malformed_cdr_and_constant_point_time_after_rebinding(self):
        self.replace_topic_payload("/sensors/imu/data", b"\x00\x01\x00\x00")
        with self.assertRaisesRegex(ValueError, "malformed CDR"):
            runner.validate_ros2_manifest(self.manifest)
        self.replace_topic_payload("/sensors/imu/data", imu_cdr())
        self.replace_topic_payload("/sensors/lidar/points", lidar_cdr((7, 7)))
        with self.assertRaisesRegex(ValueError, "constant-invalid"):
            runner.validate_ros2_manifest(self.manifest)

    def test_wrong_cdr_connection_is_rejected_after_rebinding(self):
        connection = sqlite3.connect(self.db)
        connection.execute("UPDATE topics SET serialization_format='json' WHERE name='/sensors/imu/data'")
        connection.commit(); connection.close()
        self.rebind()
        with self.assertRaisesRegex(ValueError, "connection ABI"):
            runner.validate_ros2_manifest(self.manifest)
    def test_manifest_symlink_is_rejected(self):
        alias = self.root / "manifest-alias.json"
        alias.symlink_to(self.manifest.name)
        with self.assertRaisesRegex(ValueError, "manifest must not be a symlink"):
            runner.validate_ros2_manifest(alias)

    def test_exact_types_counts_timestamps_and_hashes_are_validated(self):
        value, directory, database = runner.validate_ros2_manifest(self.manifest)
        self.assertEqual(directory, self.root); self.assertEqual(database, self.db)
        self.assertFalse(value["nuc_runtime_artifact"])

    def test_corrupt_database_is_rejected(self):
        with self.db.open("ab") as stream: stream.write(b"corrupt")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            runner.validate_ros2_manifest(self.manifest)
    def test_missing_or_mismatched_compatibility_contract_is_rejected(self):
        value = json.loads(self.manifest.read_text())
        value.pop("compatibility")
        self.manifest.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, "point-time compatibility"):
            runner.validate_ros2_manifest(self.manifest)
        value["compatibility"] = json.loads(json.dumps(runner.GLIM_POINT_TIME_COMPATIBILITY))
        value["compatibility"]["contract"]["derived_field_abi"]["fields"][4][0] = "offset_time"
        self.manifest.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, "point-time compatibility"):
            runner.validate_ros2_manifest(self.manifest)
    def test_database_traversal_and_symlink_are_rejected(self):
        value = json.loads(self.manifest.read_text())
        value["output"]["database"] = "../normalized.db3"
        self.manifest.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, "database filename"):
            runner.validate_ros2_manifest(self.manifest)
        value["output"]["database"] = self.db.name
        self.manifest.write_text(json.dumps(value))
        target = self.root / "database-target.db3"
        self.db.replace(target)
        self.db.symlink_to(target.name)
        with self.assertRaisesRegex(ValueError, "non-symlink"):
            runner.validate_ros2_manifest(self.manifest)

    def test_metadata_binding_mismatch_is_rejected_after_rehash(self):
        value = json.loads(self.manifest.read_text())
        metadata = self.root / "metadata.yaml"
        metadata.write_text(runner.expected_metadata(100, 201, self.db.name))
        value["output"]["metadata_sha256"] = runner.sha256_file(metadata)
        self.manifest.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, "metadata binding"):
            runner.validate_ros2_manifest(self.manifest)

    def test_pseudo_point_time_diagnostics_are_reported(self):
        stdout, stderr = self.root / "stdout.log", self.root / "stderr.log"
        stdout.write_text("per-point timestamps are not given\n")
        stderr.write_text("Use pseudo per-point timestamps\n")
        self.assertEqual(runner.pseudo_point_time_diagnostics(stdout, stderr), [
            "per-point timestamps are not given", "use pseudo per-point timestamps"])
        self.assertEqual(runner.run_failure(0, [], ["use pseudo per-point timestamps"]),
                         "E_GLIM_PSEUDO_POINT_TIME")


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


class GlimPointTimeCompatibilityTests(unittest.TestCase):
    def cloud(self):
        fields = [SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)
                  for name, offset, datatype, count in converter.CANONICAL_LIDAR_FIELDS]
        header = SimpleNamespace(stamp=SimpleNamespace(sec=7, nanosec=11))
        payload = struct.pack("<ffffIBBBB", 1.0, 2.0, 3.0, 17.0, 91, 2, 3, 17, 4)
        return SimpleNamespace(header=header, height=1, width=1, is_bigendian=False,
                               is_dense=True, point_step=24, row_step=24, fields=fields, data=payload)

    def test_identity_rename_preserves_payload_order_and_timestamps(self):
        source = self.cloud()
        derived = converter.glim_lidar_message(source)
        self.assertEqual(bytes(derived.data), bytes(source.data))
        self.assertIs(derived.header, source.header)
        self.assertEqual(converter.point_field_abi(source.fields), tuple(converter.CANONICAL_LIDAR_FIELDS))
        self.assertEqual(converter.point_field_abi(derived.fields), tuple(converter.GLIM_LIDAR_FIELDS))
        self.assertEqual(struct.unpack_from("<I", derived.data, 16)[0], 91)

    def test_noncanonical_field_or_malformed_stride_is_rejected(self):
        source = self.cloud()
        source.fields[4].name = "t"
        with self.assertRaisesRegex(ValueError, "field ABI"):
            converter.validate_canonical_lidar(source)
        source = self.cloud()
        source.row_step = 23
        with self.assertRaisesRegex(ValueError, "layout"):
            converter.validate_canonical_lidar(source)
        source = self.cloud()
        source.data = source.data[:-1]
        with self.assertRaisesRegex(ValueError, "layout"):
            converter.validate_canonical_lidar(source)

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
