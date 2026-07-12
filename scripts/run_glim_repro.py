#!/usr/bin/env python3
"""Run three isolated networkless GLIM rosbag2 replays and record actual dump outputs."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sqlite3
import struct
import subprocess
import sys
import time

GLIM_REVISION = "d0eeebead1ab8240edf3645682ec12d79fbfa70a"
GLIM_ROS2_REVISION = "a62811dc3ab73076f4a43fc21005f96cd712903c"
PINNED_NORMALIZED_BAG_SHA256 = "b317642b44140629b3447f5744adb068f74c57a58a3f6498df59ac582d8d8aa5"
SEED = 20260707
THREADS = 1
GLIM_ROSBAG_EXECUTABLE = "/opt/glim_ws/install/lib/glim_ros/glim_rosbag"
TOPICS = {"/sensors/lidar/points": ("sensor_msgs/msg/PointCloud2", 6882),
          "/sensors/imu/data": ("sensor_msgs/msg/Imu", 137602)}
CANONICAL_LIDAR_FIELDS = [
    ("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1),
    ("intensity", 12, 7, 1), ("offset_time", 16, 6, 1),
    ("line", 20, 2, 1), ("tag", 21, 2, 1),
    ("reflectivity", 22, 2, 1), ("lidar_id", 23, 2, 1),
]
GLIM_LIDAR_FIELDS = CANONICAL_LIDAR_FIELDS[:4] + [("t", 16, 6, 1)] + CANONICAL_LIDAR_FIELDS[5:]
MAX_CDR_PAYLOAD_BYTES = 64 * 1024 * 1024
CDR_LE = b"\x00\x01\x00\x00"
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
PSEUDO_POINT_TIME_DIAGNOSTICS = (
    "per-point timestamps are not given",
    "use pseudo per-point timestamps",
)


def pseudo_point_time_diagnostics(*paths):
    needles = tuple(item.encode() for item in PSEUDO_POINT_TIME_DIAGNOSTICS)
    matches = set()
    for path in paths:
        carry = b""
        with Path(path).open("rb") as stream:
            while chunk := stream.read(64 * 1024):
                data = (carry + chunk).lower()
                matches.update(needle.decode() for needle in needles if needle in data)
                carry = data[-(max(map(len, needles)) - 1):]
    return sorted(matches)


def expected_metadata(start, end, database):
    duration = end - start
    lines = [
        "rosbag2_bagfile_information:", "  version: 5", "  storage_identifier: sqlite3",
        "  duration:", f"    nanoseconds: {duration}", "  starting_time:",
        f"    nanoseconds_since_epoch: {start}",
        f"  message_count: {sum(item[1] for item in TOPICS.values())}",
        "  topics_with_message_count:",
    ]
    for name, (kind, count) in TOPICS.items():
        lines += [
            "    - topic_metadata:", f"        name: {name}", f"        type: {kind}",
            "        serialization_format: cdr", "        offered_qos_profiles: ''",
            f"      message_count: {count}",
        ]
    lines += [
        "  compression_format: ''", "  compression_mode: ''", "  relative_file_paths:",
        f"    - {database}", "  files:", f"    - path: {database}", "      starting_time:",
        f"        nanoseconds_since_epoch: {start}", "      duration:",
        f"        nanoseconds: {duration}",
        f"      message_count: {sum(item[1] for item in TOPICS.values())}",
    ]
    return "\n".join(lines) + "\n"
def run_failure(returncode, missing, pseudo_time_diagnostics):
    if pseudo_time_diagnostics:
        return "E_GLIM_PSEUDO_POINT_TIME"
    if returncode:
        return f"E_GLIM_EXIT_{returncode}"
    return "E_GLIM_MISSING_ACTUAL_OUTPUT:" + ",".join(missing)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
def load_input_manifest(path):
    """Load and hash-check the canonical ROS 1 normalization manifest."""
    path = Path(path).resolve(strict=True)
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("normalization manifest is not an object")
    if (manifest.get("schema_version") != 1
            or manifest.get("artifact_id") != "wheelchair.normalized_livox/v1"
            or manifest.get("status") != "candidate"):
        raise ValueError("normalization manifest schema/artifact/status mismatch")
    output = manifest.get("output")
    expected_topics = {
        "/sensors/lidar/points": {"type": "sensor_msgs/msg/PointCloud2", "count": 6882},
        "/sensors/imu/data": {"type": "sensor_msgs/msg/Imu", "count": 137602},
    }
    if (not isinstance(output, dict)
            or output.get("bag_path") != "normalized.bag"
            or output.get("format") != "rosbag1-v2"
            or output.get("topics") != expected_topics):
        raise ValueError("normalization manifest GLIM input ABI mismatch")
    digest = output.get("sha256")
    if (not isinstance(digest, str) or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)):
        raise ValueError("normalization manifest output SHA-256 is invalid")
    bag = (path.parent / output["bag_path"]).resolve(strict=True)
    try:
        bag.relative_to(path.parent)
    except ValueError as exc:
        raise ValueError("normalized bag escapes its manifest directory") from exc
    if not bag.is_file() or sha256_file(bag) != digest:
        raise ValueError("normalized bag hash mismatch")
    return manifest, bag, digest



class CdrReader:
    """Small bounded CDR reader for the two frozen ROS 2 input message ABIs."""

    def __init__(self, payload):
        self.data = memoryview(payload)
        if len(self.data) < 4 or self.data[:4].tobytes() != CDR_LE:
            raise ValueError("CDR encapsulation is not ROS 2 little-endian CDR")
        self.offset = 4

    def _align(self, alignment):
        self.offset = 4 + ((self.offset - 4 + alignment - 1) & -alignment)

    def primitive(self, code, alignment):
        self._align(alignment)
        size = struct.calcsize(code)
        if self.offset + size > len(self.data):
            raise ValueError("malformed CDR payload")
        value = struct.unpack_from(code, self.data, self.offset)[0]
        self.offset += size
        return value

    def string(self):
        size = self.primitive("<I", 4)
        if size == 0 or size > len(self.data) - self.offset or self.data[self.offset + size - 1] != 0:
            raise ValueError("malformed CDR string")
        value = self.data[self.offset:self.offset + size - 1].tobytes()
        self.offset += size
        try:
            return value.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("malformed CDR UTF-8 string") from exc

    def bytes(self):
        size = self.primitive("<I", 4)
        if size > len(self.data) - self.offset:
            raise ValueError("malformed CDR byte sequence")
        value = self.data[self.offset:self.offset + size]
        self.offset += size
        return value

    def done(self):
        if self.offset != len(self.data):
            raise ValueError("malformed CDR trailing bytes")


def cdr_header(reader):
    sec = reader.primitive("<i", 4)
    nanosec = reader.primitive("<I", 4)
    frame_id = reader.string()
    if sec < 0 or nanosec >= 1_000_000_000 or not frame_id:
        raise ValueError("invalid CDR header/frame/time semantics")
    return sec * 1_000_000_000 + nanosec, frame_id


def validate_lidar_cdr(payload):
    reader = CdrReader(payload)
    header_time, frame_id = cdr_header(reader)
    height, width = reader.primitive("<I", 4), reader.primitive("<I", 4)
    field_count = reader.primitive("<I", 4)
    if field_count != len(GLIM_LIDAR_FIELDS):
        raise ValueError("lidar CDR PointCloud2 field ABI mismatch")
    fields = []
    for _ in range(field_count):
        fields.append((reader.string(), reader.primitive("<I", 4),
                       reader.primitive("<B", 1), reader.primitive("<I", 4)))
    is_bigendian = reader.primitive("<?", 1)
    point_step, row_step = reader.primitive("<I", 4), reader.primitive("<I", 4)
    data = reader.bytes()
    is_dense = reader.primitive("<?", 1)
    reader.done()
    if (tuple(fields) != tuple(GLIM_LIDAR_FIELDS) or height != 1 or width == 0
            or is_bigendian or not is_dense or point_step != 24
            or row_step != point_step * width or len(data) != row_step * height):
        raise ValueError("lidar CDR PointCloud2 layout/field ABI mismatch")
    time_min = time_max = None
    for offset in range(0, len(data), point_step):
        values = struct.unpack_from("<ffff", data, offset)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("lidar CDR contains nonfinite point fields")
        point_time = struct.unpack_from("<I", data, offset + 16)[0]
        time_min = point_time if time_min is None else min(time_min, point_time)
        time_max = point_time if time_max is None else max(time_max, point_time)
    if time_min is None:
        raise ValueError("lidar CDR has no point timestamps")
    return header_time, frame_id, time_min, time_max, width


def validate_imu_cdr(payload):
    reader = CdrReader(payload)
    header_time, frame_id = cdr_header(reader)
    values = []
    for count in (4, 9, 3, 9, 3, 9):
        values.extend(reader.primitive("<d", 8) for _ in range(count))
    reader.done()
    if not all(math.isfinite(value) for value in values):
        raise ValueError("IMU CDR contains nonfinite vector or covariance values")
    return header_time, frame_id


def validate_ros2_manifest(path):
    path = Path(path)
    if path.is_symlink():
        raise ValueError("derived rosbag2 manifest must not be a symlink")
    path = path.resolve(strict=True)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("derived rosbag2 manifest is not an object")
    if value.get("artifact_id") != "wheelchair.glim_rosbag2_input/v1" or value.get("nuc_runtime_artifact") is not False:
        raise ValueError("input is not an offline-only derived GLIM rosbag2 artifact")
    source = value.get("source")
    if not isinstance(source, dict) or source.get("normalized_bag_sha256") != PINNED_NORMALIZED_BAG_SHA256:
        raise ValueError("derived input is not bound to the canonical normalized bag")
    if value.get("compatibility") != GLIM_POINT_TIME_COMPATIBILITY:
        raise ValueError("derived input GLIM point-time compatibility contract mismatch")
    output = value.get("output")
    expected = {name: {"type": kind, "count": count} for name, (kind, count) in TOPICS.items()}
    if not isinstance(output, dict) or output.get("format") != "rosbag2-sqlite3" or output.get("topics") != expected:
        raise ValueError("derived rosbag2 ABI mismatch")
    first, last = output.get("first_storage_time_ns"), output.get("last_storage_time_ns")
    if (isinstance(first, bool) or isinstance(last, bool)
            or not isinstance(first, int) or not isinstance(last, int) or last < first):
        raise ValueError("derived rosbag2 storage-time extent mismatch")
    if output.get("database") != "normalized.db3":
        raise ValueError("derived rosbag2 database filename mismatch")
    database, metadata = path.parent / "normalized.db3", path.parent / "metadata.yaml"
    if (database.is_symlink() or metadata.is_symlink()
            or not database.is_file() or not metadata.is_file()):
        raise ValueError("derived rosbag2 files must be direct regular non-symlink files")
    if sha256_file(database) != output.get("database_sha256") or sha256_file(metadata) != output.get("metadata_sha256"):
        raise ValueError("derived rosbag2 hash mismatch")
    if metadata.read_bytes() != expected_metadata(first, last, database.name).encode():
        raise ValueError("derived rosbag2 metadata binding mismatch")

    semantic = hashlib.sha256()
    counts = {name: 0 for name in TOPICS}
    storage_first = storage_last = header_first = header_last = None
    lidar_t_min = lidar_t_max = None
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    try:
        connections = connection.execute(
            "SELECT id,name,type,serialization_format FROM topics ORDER BY name").fetchall()
        expected_connections = {(name, kind, "cdr") for name, (kind, _) in TOPICS.items()}
        if {(name, kind, serialization) for _, name, kind, serialization in connections} != expected_connections:
            raise ValueError("derived rosbag2 topic/type/CDR connection ABI mismatch")
        if len(connections) != len(expected_connections):
            raise ValueError("derived rosbag2 has duplicate topic connections")
        messages = connection.execute(
            "SELECT t.name,m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
            "ORDER BY m.timestamp,m.id")
        for topic, timestamp, payload in messages:
            if topic not in counts or isinstance(timestamp, bool) or not isinstance(timestamp, int):
                raise ValueError("derived rosbag2 message connection/timestamp mismatch")
            payload = bytes(payload)
            if len(payload) > MAX_CDR_PAYLOAD_BYTES:
                raise ValueError("CDR payload exceeds bounded validation limit")
            if topic == "/sensors/lidar/points":
                header_time, _frame, point_t_min, point_t_max, _point_count = validate_lidar_cdr(payload)
                lidar_t_min = point_t_min if lidar_t_min is None else min(lidar_t_min, point_t_min)
                lidar_t_max = point_t_max if lidar_t_max is None else max(lidar_t_max, point_t_max)
            else:
                header_time, _frame = validate_imu_cdr(payload)
            counts[topic] += 1
            storage_first = timestamp if storage_first is None else min(storage_first, timestamp)
            storage_last = timestamp if storage_last is None else max(storage_last, timestamp)
            header_first = header_time if header_first is None else min(header_first, header_time)
            header_last = header_time if header_last is None else max(header_last, header_time)
            semantic.update(struct.pack("<IQ", len(topic), timestamp))
            semantic.update(topic.encode())
            semantic.update(struct.pack("<I", len(payload)))
            semantic.update(payload)
    finally:
        connection.close()
    if (counts != {name: item["count"] for name, item in expected.items()}
            or storage_first != first or storage_last != last):
        raise ValueError("derived rosbag2 deserialized counts or storage timestamps mismatch")
    if header_first is None or header_last is None:
        raise ValueError("derived rosbag2 has no deserialized header timestamps")
    if lidar_t_min is None or lidar_t_max is None or lidar_t_min == lidar_t_max:
        raise ValueError("lidar CDR point timestamps are missing or constant-invalid")
    expected_semantic = output.get("ros2_semantic_stream_sha256")
    if not isinstance(expected_semantic, str) or semantic.hexdigest() != expected_semantic:
        raise ValueError("derived rosbag2 semantic stream hash mismatch")
    return value, path.parent, database


def actual_glim_command(bag, config_dir, dump):
    return [GLIM_ROSBAG_EXECUTABLE, str(bag), "--ros-args",
            "-p", f"config_path:={config_dir}", "-p", "auto_quit:=true", "-p", f"dump_path:={dump}"]


def derive_outputs(dump, output):
    trajectory = dump / "traj_lidar.txt"
    submaps = sorted(item for item in dump.iterdir() if item.is_dir() and (item / "points_compact.bin").is_file() and (item / "data.txt").is_file()) if dump.is_dir() else []
    if not trajectory.is_file() or trajectory.stat().st_size == 0 or not submaps:
        raise ValueError("actual GLIM dump lacks traj_lidar.txt or serialized submaps")
    rows = []
    for number, line in enumerate(trajectory.read_text(encoding="utf-8").splitlines(), 1):
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f"invalid GLIM TUM trajectory row {number}")
        values = [float(item) for item in fields]
        if not all(math.isfinite(item) for item in values):
            raise ValueError("nonfinite GLIM trajectory")
        stamp, x, y, _z, qx, qy, qz, qw = values
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        rows.append((stamp, x, y, yaw))
    if len(rows) < 3 or any(rows[index][0] <= rows[index - 1][0] for index in range(1, len(rows))):
        raise ValueError("actual GLIM trajectory is too short or timestamps regress")
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n"); writer.writerow(("timestamp", "x", "y", "yaw")); writer.writerows(rows)
    exported = output / ".map-export"
    command = [sys.executable, "/usr/local/lib/wheelchair/export_glim_2d_map.py",
               "--glim-dump", str(dump), "--trajectory", str(trajectory), "--output-dir", str(exported),
               "--map-name", "occupancy", "--route-name", "derived_route", "--footprint-source", "simulation",
               "--footprint-width", "0.70", "--footprint-length", "1.10", "--split-index", str(len(rows) // 2)]
    subprocess.run(command, check=True)
    for name in ("occupancy.pgm", "occupancy.yaml"):
        source = exported / name
        if not source.is_file() or source.stat().st_size == 0:
            raise ValueError(f"map exporter omitted {name}")
        os.replace(source, output / name)
    shutil.rmtree(exported)
    return {"trajectory_rows": len(rows), "submap_count": len(submaps)}


def inside_container(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True); parser.add_argument("--config-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    dump = args.output / "glim-dump"
    command = actual_glim_command(args.bag, args.config_dir, dump)
    write_json(args.output / "actual_glim_command.json", command)
    completed = subprocess.run(command, check=False)
    if completed.returncode:
        return completed.returncode
    try:
        evidence = derive_outputs(dump, args.output); write_json(args.output / "actual_output_evidence.json", evidence)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"E_GLIM_ACTUAL_OUTPUT: {exc}", file=sys.stderr); return 65
    return 0


DETERMINISTIC_CONFIG_FILES = (
    "config.json",
    "config_preprocess.json",
    "config_odometry_cpu.json",
    "config_global_mapping_cpu.json",
)


def validate_config_bundle(path):
    path = Path(path)
    if path.is_file() and not path.is_symlink():
        resolved = path.resolve(strict=True)
        json.loads(resolved.read_text(encoding="utf-8"))
        return resolved, ((resolved, "config.json"),), sha256_file(resolved)
    if path.is_symlink() or not path.is_dir():
        raise ValueError("GLIM config must be a regular file or deterministic bundle directory")
    root = path.resolve(strict=True)
    manifest_path = root / "manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError("deterministic GLIM config manifest is absent or unsafe")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected_keys = {
        "schema_version", "artifact_id", "glim_revision", "glim_ros2_revision",
        "determinism", "files",
    }
    expected_determinism = {
        "preprocess_random_grid": False,
        "preprocess_threads": 1,
        "odometry_target_downsampling_rate": 0.1,
        "odometry_threads": 1,
        "global_randomsampling_determinism": "fixed-seed/default-constructed-std::mt19937",
        "global_randomsampling_rate": 0.2,
    }
    if (not isinstance(manifest, dict) or set(manifest) != expected_keys
            or manifest.get("schema_version") != 1
            or manifest.get("artifact_id") != "wheelchair.glim-deterministic-config/v1"
            or manifest.get("glim_revision") != GLIM_REVISION
            or manifest.get("glim_ros2_revision") != GLIM_ROS2_REVISION
            or manifest.get("determinism") != expected_determinism
            or set(manifest.get("files", {})) != set(DETERMINISTIC_CONFIG_FILES)):
        raise ValueError("deterministic GLIM config contract mismatch")
    mounts = []
    for name in DETERMINISTIC_CONFIG_FILES:
        candidate = root / name
        digest = manifest["files"].get(name)
        if (candidate.is_symlink() or not candidate.is_file()
                or not isinstance(digest, str) or sha256_file(candidate) != digest):
            raise ValueError("deterministic GLIM config file/hash mismatch: " + name)
        mounts.append((candidate.resolve(strict=True), name))
    json.loads((root / "config.json").read_text(encoding="utf-8"))
    return root / "config.json", tuple(mounts), sha256_file(manifest_path)


def container_command(args, bag_dir, run_dir):
    command = [args.container_engine, "run", "--rm", "--network=none", "--read-only",
               "--security-opt=no-new-privileges", "--cap-drop=ALL",
               f"--user={os.getuid()}:{os.getgid()}",
               "--tmpfs=/tmp:rw,nosuid,nodev,size=1g",
               f"--mount=type=bind,src={bag_dir},dst=/input/rosbag2,readonly"]
    for source, name in args.config_mounts:
        command.append(
            f"--mount=type=bind,src={source},dst=/opt/glim-config/{name},readonly"
        )
    command += [f"--mount=type=bind,src={run_dir},dst=/output",
                "--env=HOME=/tmp", "--env=OMP_NUM_THREADS=1",
                "--env=OPENBLAS_NUM_THREADS=1", "--env=MKL_NUM_THREADS=1",
                f"--env=GLIM_REPRO_SEED={SEED}",
                "--label=wheelchair.offline-only=true", args.image,
                "--bag", "/input/rosbag2", "--config-dir", "/opt/glim-config",
                "--output", "/output"]
    return command


def artifacts(root):
    return {str(path.relative_to(root)): {"sha256": sha256_file(path), "size_bytes": path.stat().st_size}
            for path in sorted(root.rglob("*")) if path.is_file() and path.name != "run_manifest.json"}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros2-manifest", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True); parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--image", required=True); parser.add_argument("--container-engine", default="docker")
    parser.add_argument("--source-revision", default=GLIM_REVISION); parser.add_argument("--glim-ros2-revision", default=GLIM_ROS2_REVISION)
    return parser.parse_args(argv)


def main(argv=None):
    if argv is None: argv = sys.argv[1:]
    if argv and argv[0] == "--inside-container": return inside_container(argv[1:])
    args = parse_args(argv)
    try:
        if args.output_dir.exists(): raise ValueError("output directory already exists")
        if args.source_revision != GLIM_REVISION or args.glim_ros2_revision != GLIM_ROS2_REVISION: raise ValueError("source revision differs from image pins")
        if "@sha256:" not in args.image or len(args.image.rsplit("@sha256:", 1)[1]) != 64: raise ValueError("image must use an immutable digest")
        config, config_mounts, config_digest = validate_config_bundle(args.config)
        args.config_mounts = config_mounts
        source, bag_dir, database = validate_ros2_manifest(args.ros2_manifest)
    except (OSError, ValueError, json.JSONDecodeError, sqlite3.DatabaseError) as exc:
        print(f"E_GLIM_REPRO_INPUT: {exc}", file=sys.stderr); return 2
    args.output_dir.mkdir(parents=True)
    report = {"schema_version": 2, "artifact_id": "wheelchair.glim-reproduction/v2", "status": "failed",
              "execution_scope": "OFFLINE_WORKSTATION_ONLY", "nuc_runtime_dependency": False,
              "claim_label": "REPLAY_CONSISTENCY_NOT_TRUTH", "qualification": "candidate",
              "image": args.image, "source_revision": GLIM_REVISION, "glim_ros2_revision": GLIM_ROS2_REVISION,
              "seed": SEED, "threads": THREADS, "input_manifest_sha256": sha256_file(args.ros2_manifest),
              "ros2_database_sha256": sha256_file(database), "config_sha256": config_digest,
              "config_entrypoint_sha256": sha256_file(config), "runs": []}
    success = True
    for index in range(1, 4):
        run_dir = args.output_dir / f"run-{index:02d}"; run_dir.mkdir()
        command = container_command(args, bag_dir, run_dir); started = time.time()
        try: returncode = subprocess.run(command, check=False, stdout=(run_dir / "stdout.log").open("wb"), stderr=(run_dir / "stderr.log").open("wb")).returncode
        except OSError as exc: returncode = 127; (run_dir / "stderr.log").write_text(str(exc) + "\n")
        pseudo_time_diagnostics = pseudo_point_time_diagnostics(run_dir / "stdout.log", run_dir / "stderr.log")
        missing = [name for name in ("trajectory.csv", "occupancy.pgm", "occupancy.yaml", "actual_output_evidence.json") if not (run_dir / name).is_file()]
        okay = returncode == 0 and not missing and not pseudo_time_diagnostics; success &= okay
        failure = None if okay else run_failure(returncode, missing, pseudo_time_diagnostics)
        record = {"run_id": index, "directory": run_dir.name, "status": "success" if okay else "failed",
                  "failure": failure, "pseudo_point_time_diagnostics": pseudo_time_diagnostics,
                  "returncode": returncode, "command": command, "image": args.image, "source_revision": GLIM_REVISION,
                  "glim_ros2_revision": GLIM_ROS2_REVISION, "seed": SEED, "threads": THREADS,
                  "elapsed_s": time.time() - started, "artifacts": artifacts(run_dir)}
        write_json(run_dir / "run_manifest.json", record); report["runs"].append(record)
    report["status"] = "success" if success else "failed"; write_json(args.output_dir / "repro_manifest.json", report)
    print(args.output_dir / "repro_manifest.json"); return 0 if success else 1


if __name__ == "__main__": raise SystemExit(main())
