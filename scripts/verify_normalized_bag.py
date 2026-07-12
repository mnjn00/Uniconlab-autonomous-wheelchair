#!/usr/bin/env python3
"""Independently verify an Appendix-C normalized ROS 1 bag transaction."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import struct
import sys
from typing import Any, Iterable, Sequence

ROSBAGS_VERSION = "0.10.11"
DEPENDENCY_DIAGNOSTIC = (
    "E_DEPENDENCY: rosbags==0.10.11 is required; install with "
    "`python3 -m pip install --requirement tools/offline/requirements.lock` "
    "inside the offline conversion container (never on the NUC)"
)
CLOUD_TOPIC = "/sensors/lidar/points"
IMU_TOPIC = "/sensors/imu/data"
SOURCE_CLOUD_TOPIC = "/livox/lidar"
SOURCE_IMU_TOPIC = "/livox/imu"
POINT_STRUCT = struct.Struct("<ffffIBBBB")
UINT64_MAX = (1 << 64) - 1
EXPECTED_FIELDS = [
    ("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1),
    ("intensity", 12, 7, 1), ("offset_time", 16, 6, 1),
    ("line", 20, 2, 1), ("tag", 21, 2, 1),
    ("reflectivity", 22, 2, 1), ("lidar_id", 23, 2, 1),
]
EXPECTED_TOPICS = {
    CLOUD_TOPIC: {"type": "sensor_msgs/msg/PointCloud2", "count": 6882},
    IMU_TOPIC: {"type": "sensor_msgs/msg/Imu", "count": 137602},
}
OUTPUT_FIELDS = {
    "bag_path", "sha256", "format", "topics", "compression",
    "chunk_threshold_bytes", "counts", "first_storage_time_ns",
    "last_storage_time_ns", "first_source_time_ns", "last_source_time_ns",
    "clock_statistics", "offset_time_regression_statistics",
}
OFFSET_STATISTIC_FIELDS = {
    "cloud_count", "point_count", "clouds_with_adjacent_decreases",
    "adjacent_offset_decrease_count",
}


class VerificationError(Exception):
    pass


def fail(message: str) -> None:
    raise VerificationError(message)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp_ns(stamp: Any) -> int:
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        fail("invalid output header stamp")
    return sec * 1_000_000_000 + nanosec


def load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"cannot read canonical JSON artifact {path}: {exc}")
    if not isinstance(value, dict):
        fail(f"artifact {path} is not an object")
    return value


def validate_manifest_abi(manifest: dict[str, Any]) -> dict[str, Any]:
    if (manifest.get("schema_version") != 1 or
            manifest.get("artifact_id") != "wheelchair.normalized_livox/v1"):
        fail("normalization manifest schema/artifact mismatch")
    if manifest.get("status") != "candidate":
        fail("normalization manifest status mismatch")
    output = manifest.get("output")
    if not isinstance(output, dict) or set(output) != OUTPUT_FIELDS:
        fail("normalization manifest output ABI fields mismatch")
    if (output.get("bag_path") != "normalized.bag" or
            output.get("format") != "rosbag1-v2" or
            output.get("topics") != EXPECTED_TOPICS):
        fail("normalization manifest GLIM input ABI mismatch")
    statistics = output.get("offset_time_regression_statistics")
    if (not isinstance(statistics, dict) or set(statistics) != OFFSET_STATISTIC_FIELDS or
            any(isinstance(value, bool) or not isinstance(value, int) or value < 0
                for value in statistics.values()) or
            statistics["clouds_with_adjacent_decreases"] > statistics["cloud_count"] or
            statistics["adjacent_offset_decrease_count"] > statistics["point_count"]):
        fail("normalization manifest offset-time statistics ABI mismatch")
    digest = output.get("sha256")
    if (not isinstance(digest, str) or len(digest) != 64 or
            any(character not in "0123456789abcdef" for character in digest)):
        fail("normalization manifest output SHA-256 is invalid")
    return output


def records(path: Path) -> Iterable[dict[str, Any]]:
    try:
        with path.open("rb") as stream:
            for number, raw in enumerate(stream, 1):
                value = json.loads(raw)
                if not isinstance(value, dict):
                    fail(f"records line {number} is not an object")
                canonical = (json.dumps(value, sort_keys=True, separators=(",", ":"),
                                        ensure_ascii=False) + "\n").encode()
                if canonical != raw:
                    fail(f"records line {number} is not canonical JSON")
                yield value
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"cannot parse records index: {exc}")


def finite(values: Iterable[float], label: str) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        fail(f"{label} contains nonfinite values")


def verify_cloud(message: Any, record: dict[str, Any], lidar_offset: int) -> dict[str, int]:
    if message.header.frame_id != "lidar_link" or message.height != 1 or message.width != record["point_count"]:
        fail("PointCloud2 frame/dimensions mismatch")
    if message.is_bigendian or not message.is_dense or message.point_step != 24 or message.row_step != 24 * message.width:
        fail("PointCloud2 layout flags/steps mismatch")
    actual_fields = [(x.name, int(x.offset), int(x.datatype), int(x.count)) for x in message.fields]
    if actual_fields != EXPECTED_FIELDS:
        fail("PointCloud2 fields do not match the exact 24-byte ABI")
    data = bytes(message.data)
    if len(data) != 24 * message.width:
        fail("PointCloud2 data length mismatch")
    source_time = record.get("source_time_ns")
    if isinstance(source_time, bool) or not isinstance(source_time, int) or not 0 <= source_time <= UINT64_MAX:
        fail("cloud source time is outside uint64")
    offsets: list[int] = []
    adjacent_offset_decrease_count = 0
    previous_offset: int | None = None
    for index in range(message.width):
        x, y, z, intensity, offset, line, tag, reflectivity, lidar_id = POINT_STRUCT.unpack_from(data, 24 * index)
        finite((x, y, z, intensity), f"point {index}")
        if intensity != float(reflectivity):
            fail(f"point {index} intensity does not exactly equal reflectivity")
        if source_time > UINT64_MAX - offset:
            fail(f"point {index} source time overflows uint64")
        if previous_offset is not None and offset < previous_offset:
            adjacent_offset_decrease_count += 1
        previous_offset = offset
        offsets.append(offset)
        if not all(0 <= value <= 255 for value in (line, tag, reflectivity, lidar_id)):
            fail(f"point {index} byte field invalid")
    minimum_offset = min(offsets) if offsets else None
    maximum_offset = max(offsets) if offsets else None
    min_time = source_time + minimum_offset if minimum_offset is not None else source_time
    max_time = source_time + maximum_offset if maximum_offset is not None else source_time
    if (record.get("min_point_time_ns") != min_time or
            record.get("max_point_time_ns") != max_time or
            record.get("minimum_offset_time") != minimum_offset or
            record.get("maximum_offset_time") != maximum_offset or
            record.get("adjacent_offset_decrease_count") != adjacent_offset_decrease_count):
        fail("point source-time extent/regression statistics mismatch")
    if stamp_ns(message.header.stamp) != source_time + lidar_offset:
        fail("cloud normalized header/source/alignment mismatch")
    if abs(record["source_header_time_ns"] - record["source_time_ns"]) > 1_000_000:
        fail("cloud source header/timebase residual exceeds 1 ms")
    return {
        "point_count": message.width,
        "adjacent_offset_decrease_count": adjacent_offset_decrease_count,
    }


def verify_imu(message: Any, record: dict[str, Any], imu_offset: int) -> bool:
    if message.header.frame_id != "imu_link":
        fail("IMU canonical frame mismatch")
    if stamp_ns(message.header.stamp) != record["source_time_ns"] + imu_offset:
        fail("IMU normalized header/source/alignment mismatch")
    quaternion = tuple(float(getattr(message.orientation, key)) for key in ("x", "y", "z", "w"))
    vectors = tuple(float(getattr(vector, key)) for vector in
                    (message.angular_velocity, message.linear_acceleration) for key in ("x", "y", "z"))
    covariances = tuple(float(value) for name in
                        ("orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance")
                        for value in getattr(message, name))
    if len(covariances) != 27:
        fail("IMU covariance layout mismatch")
    finite(quaternion + vectors + covariances, "IMU")
    if covariances[0] != -1.0:
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm == 0.0 or abs(norm - 1.0) > 0.01:
            fail("IMU quaternion norm invalid")
    return covariances[0] != -1.0


def verify(directory: Path) -> dict[str, Any]:
    try:
        if importlib.metadata.version("rosbags") != ROSBAGS_VERSION:
            fail(DEPENDENCY_DIAGNOSTIC)
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore
    except (ImportError, ModuleNotFoundError, importlib.metadata.PackageNotFoundError):
        fail(DEPENDENCY_DIAGNOSTIC)
    manifest_path = directory / "normalization_manifest.yaml"
    bag_path = directory / "normalized.bag"
    index_path = directory / "records.jsonl"
    report_path = directory / "conversion_report.json"
    if not all(path.is_file() for path in (manifest_path, bag_path, index_path, report_path)):
        fail("accepted transaction is missing a required artifact")
    manifest = load_object(manifest_path)
    report = load_object(report_path)
    output = validate_manifest_abi(manifest)
    if output["sha256"] != sha256_file(bag_path):
        fail("normalized bag SHA-256 mismatch")
    if manifest.get("records_sha256") != sha256_file(index_path):
        fail("records index SHA-256 mismatch")
    if manifest.get("conversion_report_sha256") != sha256_file(report_path):
        fail("conversion report SHA-256 mismatch")
    if manifest.get("tool", {}).get("rosbags_version") != ROSBAGS_VERSION:
        fail("manifest rosbags pin mismatch")
    if output.get("compression") != "none" or output.get("chunk_threshold_bytes") != 768 * 1024:
        fail("bag compression/chunk settings mismatch")
    mappings = manifest.get("frame_mappings")
    if not isinstance(mappings, list) or len(mappings) != 2 or [x.get("canonical") for x in mappings] != ["lidar_link", "imu_link"]:
        fail("frame mapping manifest mismatch")
    parameters = manifest.get("parameters", {})
    lidar_offset, imu_offset = parameters.get("lidar_offset_ns"), parameters.get("imu_offset_ns")
    if not isinstance(lidar_offset, int) or not isinstance(imu_offset, int):
        fail("manifest alignment offsets missing")
    ros1 = get_typestore(Stores.ROS1_NOETIC)
    index_iter = iter(records(index_path))
    counts = {CLOUD_TOPIC: 0, IMU_TOPIC: 0}
    semantic = hashlib.sha256()
    first_timestamp = None
    last_timestamp = None
    clock_ages = {SOURCE_CLOUD_TOPIC: [], SOURCE_IMU_TOPIC: []}
    imu_orientation_available = True
    offset_statistics = {
        "cloud_count": 0, "point_count": 0, "clouds_with_adjacent_decreases": 0,
        "adjacent_offset_decrease_count": 0,
    }
    with Reader(bag_path) as reader:
        if [(x.topic, x.msgtype) for x in reader.connections] != [
                (CLOUD_TOPIC, "sensor_msgs/msg/PointCloud2"), (IMU_TOPIC, "sensor_msgs/msg/Imu")]:
            fail("connection order/topic/type mismatch or unexpected connection")
        for connection, timestamp, rawdata in reader.messages():
            try:
                record = next(index_iter)
            except StopIteration:
                fail("bag has more messages than records index")
            expected_source_topic = SOURCE_CLOUD_TOPIC if connection.topic == CLOUD_TOPIC else SOURCE_IMU_TOPIC
            if record.get("topic") != expected_source_topic or record.get("storage_time_ns") != timestamp:
                fail("record topic/storage time does not match bag message")
            raw = bytes(rawdata)
            if hashlib.sha256(raw).hexdigest() != record.get("decoded_canonical_payload_sha256"):
                fail("canonical payload SHA-256 mismatch")
            message = ros1.deserialize_ros1(raw, connection.msgtype)
            if connection.topic == CLOUD_TOPIC:
                cloud_statistics = verify_cloud(message, record, lidar_offset)
                offset_statistics["cloud_count"] += 1
                offset_statistics["point_count"] += cloud_statistics["point_count"]
                decreases = cloud_statistics["adjacent_offset_decrease_count"]
                offset_statistics["adjacent_offset_decrease_count"] += decreases
                if decreases:
                    offset_statistics["clouds_with_adjacent_decreases"] += 1
            elif connection.topic == IMU_TOPIC:
                available = verify_imu(message, record, imu_offset)
                imu_orientation_available = imu_orientation_available and available
            else:
                fail("unexpected output topic")
            if stamp_ns(message.header.stamp) - timestamp > 50_000_000:
                fail("normalized header is over 50 ms ahead of replay clock")
            clock_ages[expected_source_topic].append(timestamp - stamp_ns(message.header.stamp))
            semantic.update(struct.pack("<IQ", len(connection.topic), timestamp))
            semantic.update(connection.topic.encode())
            semantic.update(struct.pack("<I", len(raw)))
            semantic.update(raw)
            counts[connection.topic] += 1
            first_timestamp = timestamp if first_timestamp is None else first_timestamp
            last_timestamp = timestamp
    try:
        next(index_iter)
        fail("records index has more messages than bag")
    except StopIteration:
        pass
    expected_counts = {
        topic: properties["count"] for topic, properties in EXPECTED_TOPICS.items()
    }
    if counts != expected_counts or output.get("counts") != expected_counts or report.get("counts") != expected_counts:
        fail("bag/manifest/report counts mismatch")
    if first_timestamp != output.get("first_storage_time_ns") or last_timestamp != output.get("last_storage_time_ns"):
        fail("bag storage-time extent mismatch")
    observed_clock_statistics = {
        topic: {"minimum_age_ns": min(ages), "maximum_age_ns": max(ages)}
        for topic, ages in clock_ages.items()
    }
    if observed_clock_statistics != output.get("clock_statistics"):
        fail("manifest clock statistics mismatch")
    if (offset_statistics != output.get("offset_time_regression_statistics") or
            offset_statistics != report.get("offset_time_regression_statistics")):
        fail("manifest/report offset-time regression statistics mismatch")
    qualification = manifest.get("qualification", {})
    if (qualification.get("imu_orientation_available") is not imu_orientation_available or
            qualification.get("fusion_qualified") is not
            bool(qualification.get("alignment_verified") and imu_orientation_available)):
        fail("manifest fusion qualification mismatch")
    if semantic.hexdigest() != report.get("semantic_stream_sha256"):
        fail("semantic stream SHA-256 mismatch")
    if report.get("errors") != [] or report.get("gaps") != []:
        fail("conversion report contains errors or gaps")
    return {"bag_sha256": sha256_file(bag_path), "counts": counts,
            "semantic_stream_sha256": semantic.hexdigest(), "status": "ok"}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("normalized_directory")
    args = parser.parse_args(argv)
    try:
        result = verify(Path(args.normalized_directory).resolve())
        sys.stdout.write(json.dumps(result, sort_keys=True, separators=(",", ":")) + "\n")
        return 0
    except VerificationError as exc:
        sys.stderr.write(json.dumps({"code": "E_VERIFY", "message": str(exc), "status": "error"},
                                    sort_keys=True, separators=(",", ":")) + "\n")
        return 2
    except Exception as exc:
        sys.stderr.write(json.dumps({"code": "E_VERIFY", "message": f"verification failed: {exc}", "status": "error"},
                                    sort_keys=True, separators=(",", ":")) + "\n")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
