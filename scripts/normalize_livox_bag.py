#!/usr/bin/env python3
"""Deterministically normalize a staged ROS 2 Livox bag into a ROS 1 bag.

The conversion is deliberately offline-only.  Importing this module does not
require ROS, rosbags, or numpy; the pure validation/packing helpers are used by
unit tests and by the transaction after CDR deserialization.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import operator
from pathlib import Path
import shutil
import sqlite3
import struct
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence

ROSBAGS_VERSION = "0.10.11"
DEPENDENCY_DIAGNOSTIC = (
    "E_DEPENDENCY: rosbags==0.10.11 is required; install with "
    "`python3 -m pip install --requirement tools/offline/requirements.lock` "
    "inside the offline conversion container (never on the NUC)"
)
CUSTOM_MSG_SHA256 = "f42d6709db951b1fa307e929e742c0593cbf0d1b0ff977d2ed63ad8d7cee0a96"
CUSTOM_POINT_SHA256 = "b64b31a8edc8c8b3765d82b5d3ccd2d2e1f217b9525ef7007ab918674c619c59"
CUSTOM_COMPOSITE_SHA256 = "8d51083a4570d6e81f3193c9b8c39e16d2d5fb2d776dd198a997c7c5c6f4aac7"
LIVOX_TYPE = "livox_ros_driver2/msg/CustomMsg"
IMU_TYPE = "sensor_msgs/msg/Imu"
LIVOX_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
CLOUD_TOPIC = "/sensors/lidar/points"
OUTPUT_IMU_TOPIC = "/sensors/imu/data"
POINT_STRUCT = struct.Struct("<ffffIBBBB")
UINT64_MAX = (1 << 64) - 1
ALIGNMENT_FIELDS = {
    "schema_version", "artifact_id", "owner", "reviewer", "status",
    "provenance", "method", "lidar_offset_ns", "imu_offset_ns",
    "calibration_evidence_sha256", "verified",
    "p99_cross_sensor_residual_ms", "drift_ms_per_min",
}


class ConversionError(Exception):
    """Stable, row-addressable conversion failure."""

    def __init__(self, code: str, message: str, row_id: int | None = None,
                 topic: str | None = None) -> None:
        super().__init__(message)
        self.code, self.message, self.row_id, self.topic = code, message, row_id, topic

    def as_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {"code": self.code, "message": self.message}
        if self.row_id is not None:
            result["row_id"] = self.row_id
        if self.topic is not None:
            result["topic"] = self.topic
        return result


def _get(value: Any, name: str) -> Any:
    return value[name] if isinstance(value, Mapping) else getattr(value, name)


def stamp_to_ns(stamp: Any) -> int:
    sec = int(_get(stamp, "sec"))
    nanosec = int(_get(stamp, "nanosec"))
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ConversionError("E_CDR_DESERIALIZE", "invalid ROS time")
    result = sec * 1_000_000_000 + nanosec
    if result > UINT64_MAX:
        raise ConversionError("E_POINT_TIME_OVERFLOW", "ROS time exceeds uint64 nanoseconds")
    return result


def ns_to_parts(value: int) -> tuple[int, int]:
    if value < 0 or value > UINT64_MAX:
        raise ConversionError("E_ALIGNMENT_SCHEMA", "aligned stamp is outside uint64 range")
    return divmod(value, 1_000_000_000)


def require_nondecreasing(previous: Any | None, current: Any, code: str,
                          label: str) -> None:
    if previous is not None and current < previous:
        raise ConversionError(code, f"{label} regressed")


def canonicalize_cloud(message: Any, *, storage_time_ns: int,
                       source_frame: str, offset_ns: int = 0) -> dict[str, Any]:
    """Validate a decoded CustomMsg and return a byte-stable canonical record."""
    header = _get(message, "header")
    frame = str(_get(header, "frame_id"))
    if not frame or frame != source_frame:
        raise ConversionError("E_FRAME_MAPPING", f"Livox frame {frame!r} != declared {source_frame!r}")
    timebase = int(_get(message, "timebase"))
    if not 0 <= timebase <= UINT64_MAX:
        raise ConversionError("E_POINT_TIME_OVERFLOW", "Livox timebase is outside uint64")
    header_ns = stamp_to_ns(_get(header, "stamp"))
    if abs(header_ns - timebase) > 1_000_000:
        raise ConversionError("E_HEADER_TIME_RESIDUAL", "Livox header differs from timebase by more than 1 ms")
    points = list(_get(message, "points"))
    point_num = int(_get(message, "point_num"))
    if point_num != len(points):
        raise ConversionError("E_POINT_COUNT", f"point_num {point_num} != array length {len(points)}")
    lidar_id = int(_get(message, "lidar_id"))
    if not 0 <= lidar_id <= 255:
        raise ConversionError("E_POINT_LAYOUT", "lidar_id is outside uint8")
    reserved = [int(x) for x in _get(message, "rsvd")]
    if len(reserved) != 3 or any(x < 0 or x > 255 for x in reserved):
        raise ConversionError("E_CDR_DESERIALIZE", "rsvd is not exactly three uint8 values")
    packed = bytearray()
    offsets: list[int] = []
    previous: int | None = None
    adjacent_offset_decrease_count = 0
    for index, point in enumerate(points):
        raw_offset = _get(point, "offset_time")
        try:
            offset = operator.index(raw_offset)
        except TypeError as exc:
            raise ConversionError(
                "E_POINT_LAYOUT", f"point {index} offset_time is not uint32") from exc
        if isinstance(raw_offset, bool):
            raise ConversionError("E_POINT_LAYOUT", f"point {index} offset_time is not uint32")
        if not 0 <= offset <= 0xFFFFFFFF:
            raise ConversionError("E_POINT_LAYOUT", f"point {index} offset_time outside uint32")
        if previous is not None and offset < previous:
            adjacent_offset_decrease_count += 1
        if timebase > UINT64_MAX - offset:
            raise ConversionError("E_POINT_TIME_OVERFLOW", f"point {index} time overflows uint64")
        previous = offset
        xyz = tuple(float(_get(point, axis)) for axis in ("x", "y", "z"))
        if not all(math.isfinite(x) for x in xyz):
            raise ConversionError("E_NONFINITE", f"point {index} has nonfinite coordinate")
        reflectivity = int(_get(point, "reflectivity"))
        tag = int(_get(point, "tag"))
        line = int(_get(point, "line"))
        if any(x < 0 or x > 255 for x in (reflectivity, tag, line)):
            raise ConversionError("E_POINT_LAYOUT", f"point {index} byte field outside uint8")
        intensity = float(reflectivity)
        if not math.isfinite(intensity):
            raise ConversionError("E_NONFINITE", f"point {index} intensity is nonfinite")
        packed.extend(POINT_STRUCT.pack(xyz[0], xyz[1], xyz[2], intensity, offset,
                                        line, tag, reflectivity, lidar_id))
        offsets.append(offset)
    aligned_ns = timebase + int(offset_ns)
    ns_to_parts(aligned_ns)
    if aligned_ns - storage_time_ns > 50_000_000:
        raise ConversionError("E_CLOCK_FUTURE", "normalized cloud header is over 50 ms ahead of storage clock")
    return {
        "data": bytes(packed), "width": point_num, "source_time_ns": timebase,
        "header_time_ns": header_ns, "normalized_time_ns": aligned_ns,
        "min_point_time_ns": timebase + min(offsets) if offsets else timebase,
        "max_point_time_ns": timebase + max(offsets) if offsets else timebase,
        "minimum_offset_time": min(offsets) if offsets else None,
        "maximum_offset_time": max(offsets) if offsets else None,
        "adjacent_offset_decrease_count": adjacent_offset_decrease_count,
        "point_count": point_num, "reserved": reserved, "source_frame": frame,
        "storage_time_ns": int(storage_time_ns),
    }


def canonicalize_imu(message: Any, *, storage_time_ns: int,
                     source_frame: str, offset_ns: int = 0) -> dict[str, Any]:
    """Validate decoded Imu values without changing any floating-point value."""
    header = _get(message, "header")
    frame = str(_get(header, "frame_id"))
    if not frame or frame != source_frame:
        raise ConversionError("E_FRAME_MAPPING", f"IMU frame {frame!r} != declared {source_frame!r}")
    source_ns = stamp_to_ns(_get(header, "stamp"))
    normalized_ns = source_ns + int(offset_ns)
    ns_to_parts(normalized_ns)
    if normalized_ns - storage_time_ns > 50_000_000:
        raise ConversionError("E_CLOCK_FUTURE", "normalized IMU header is over 50 ms ahead of storage clock")
    orientation = tuple(float(_get(_get(message, "orientation"), x)) for x in ("x", "y", "z", "w"))
    angular = tuple(float(_get(_get(message, "angular_velocity"), x)) for x in ("x", "y", "z"))
    linear = tuple(float(_get(_get(message, "linear_acceleration"), x)) for x in ("x", "y", "z"))
    covariances = tuple(tuple(float(x) for x in _get(message, name)) for name in (
        "orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"))
    if any(len(x) != 9 for x in covariances):
        raise ConversionError("E_IMU_MALFORMED", "IMU covariance does not contain nine values")
    values = orientation + angular + linear + tuple(x for covariance in covariances for x in covariance)
    if not all(math.isfinite(x) for x in values):
        raise ConversionError("E_NONFINITE", "IMU contains a nonfinite value")
    unavailable = covariances[0][0] == -1.0
    norm = math.sqrt(sum(x * x for x in orientation))
    if not unavailable and (norm == 0.0 or abs(norm - 1.0) > 0.01):
        raise ConversionError("E_IMU_MALFORMED", f"orientation quaternion norm {norm} is invalid")
    return {
        "orientation": orientation, "angular_velocity": angular,
        "linear_acceleration": linear, "covariances": covariances,
        "source_time_ns": source_ns, "normalized_time_ns": normalized_ns,
        "source_frame": frame, "storage_time_ns": int(storage_time_ns),
        "orientation_available": not unavailable,
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()


def load_json(path: Path, code: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ConversionError(code, f"cannot read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConversionError(code, f"{path} must contain a JSON object")
    return value


def validate_alignment(value: Mapping[str, Any]) -> None:
    if set(value) != ALIGNMENT_FIELDS or value.get("schema_version") != 1:
        raise ConversionError("E_ALIGNMENT_SCHEMA", "alignment fields/schema do not match v1")
    if value.get("method") not in ("identity", "fixed_offset_ns"):
        raise ConversionError("E_ALIGNMENT_SCHEMA", "only identity or fixed_offset_ns is permitted")
    if not isinstance(value.get("lidar_offset_ns"), int) or not isinstance(value.get("imu_offset_ns"), int):
        raise ConversionError("E_ALIGNMENT_SCHEMA", "alignment offsets must be signed integers")
    if value["method"] == "identity" and (value["lidar_offset_ns"] or value["imu_offset_ns"]):
        raise ConversionError("E_ALIGNMENT_SCHEMA", "identity alignment requires zero offsets")
    provenance = value.get("provenance")
    if not isinstance(provenance, dict) or set(provenance) != {
            "source_bag_manifest_sha256", "converter_abi_sha256", "calibration_method"}:
        raise ConversionError("E_ALIGNMENT_SCHEMA", "alignment provenance is malformed")
    hash_values = [value.get("calibration_evidence_sha256"),
                   provenance.get("source_bag_manifest_sha256"),
                   provenance.get("converter_abi_sha256")]
    if any(not isinstance(item, str) or
           (item and (len(item) != 64 or any(char not in "0123456789abcdef" for char in item)))
           for item in hash_values):
        raise ConversionError("E_ALIGNMENT_SCHEMA", "alignment SHA-256 field is malformed")
    if not isinstance(provenance.get("calibration_method"), str) or not provenance["calibration_method"]:
        raise ConversionError("E_ALIGNMENT_SCHEMA", "alignment calibration method is missing")
    verified = value.get("verified")
    if not isinstance(verified, bool):
        raise ConversionError("E_ALIGNMENT_SCHEMA", "verified must be boolean")
    if verified and (value.get("status") != "qualified" or
                     any(not item for item in hash_values) or
                     not isinstance(value.get("p99_cross_sensor_residual_ms"), (int, float)) or
                     not isinstance(value.get("drift_ms_per_min"), (int, float)) or
                     not math.isfinite(value["p99_cross_sensor_residual_ms"]) or
                     not math.isfinite(value["drift_ms_per_min"]) or
                     not 0 <= value["p99_cross_sensor_residual_ms"] <= 2.0 or
                     not 0 <= value["drift_ms_per_min"] <= 0.5):
        raise ConversionError("E_ALIGNMENT_SCHEMA", "verified alignment lacks qualifying evidence")


def validate_idl(custom_msg: Path, custom_point: Path) -> tuple[str, str]:
    try:
        msg_bytes, point_bytes = custom_msg.read_bytes(), custom_point.read_bytes()
    except OSError as exc:
        raise ConversionError("E_SOURCE_IDL", f"cannot read source IDL: {exc}") from exc
    if hashlib.sha256(msg_bytes).hexdigest() != CUSTOM_MSG_SHA256:
        raise ConversionError("E_SOURCE_IDL", "CustomMsg source hash mismatch")
    if hashlib.sha256(point_bytes).hexdigest() != CUSTOM_POINT_SHA256:
        raise ConversionError("E_SOURCE_IDL", "CustomPoint source hash mismatch")
    composite = (b"CustomMsg.msg\0" + msg_bytes + b"\0" +
                 b"CustomPoint.msg\0" + point_bytes + b"\0")
    if hashlib.sha256(composite).hexdigest() != CUSTOM_COMPOSITE_SHA256:
        raise ConversionError("E_SOURCE_IDL", "source IDL composite hash mismatch")
    try:
        return msg_bytes.decode("utf-8"), point_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ConversionError("E_SOURCE_IDL", "source IDL is not UTF-8") from exc


def validate_staging(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    if (manifest.get("schema_version") != "wheelchair.rosbag2_manifest/v1" or
            manifest.get("status") != "staged" or manifest.get("operation") != "stage"):
        raise ConversionError("E_SOURCE_MANIFEST", "input is not a successful staged v1 manifest")
    if manifest.get("errors"):
        raise ConversionError("E_SOURCE_MANIFEST", "staging manifest reports errors")
    mismatches = manifest.get("mismatches")
    allowed_mismatch_reasons = {
        "zero_byte_expected_populated_suffixed_segment",
        "pre_existing_source_symlink_alias",
    }
    if not isinstance(mismatches, list) or any(
            not isinstance(item, Mapping) or
            item.get("code") != "E_SOURCE_DISCOVERY" or
            not isinstance(item.get("context"), Mapping) or
            item["context"].get("mismatch_reason") not in allowed_mismatch_reasons
            for item in mismatches):
        raise ConversionError("E_SOURCE_MANIFEST", "staging manifest has an unrecognized source mismatch")
    metadata = manifest.get("metadata", {})
    provenance = manifest.get("provenance", {})
    if (metadata.get("version") != 5 or metadata.get("storage_identifier") != "sqlite3" or
            provenance.get("storage_identifier") != "sqlite3"):
        raise ConversionError("E_SOURCE_MANIFEST", "metadata/storage plugin contract mismatch")
    expected = {
        LIVOX_TOPIC: (LIVOX_TYPE, 6882),
        IMU_TOPIC: (IMU_TYPE, 137602),
    }
    topics = manifest.get("topics")
    if not isinstance(topics, list) or len(topics) != 2:
        raise ConversionError("E_SOURCE_MANIFEST", "staging manifest must contain exactly two topics")
    for topic in topics:
        name = topic.get("name")
        if name not in expected or topic.get("type") != expected[name][0] or topic.get("serialization_format") != "cdr" or topic.get("metadata_count") != expected[name][1] or topic.get("sqlite_count") != expected[name][1]:
            raise ConversionError("E_SOURCE_MANIFEST", f"topic contract mismatch for {name!r}")
    totals = manifest.get("totals", {})
    if totals.get("message_count") != 144484 or totals.get("duration_ns") != 688225098527:
        raise ConversionError("E_COUNT_DURATION", "source total count/duration mismatch")
    segments = manifest.get("segments")
    if not isinstance(segments, list) or len(segments) != 1:
        raise ConversionError("E_SOURCE_DISCOVERY", "expected exactly one staged sqlite segment")
    if (Path(str(segments[0].get("source_path", ""))).name !=
            "livox_raw_20260707_191720_0-001.db3" or
            metadata.get("declared_relative_file_paths") !=
            ["livox_raw_20260707_191720_0.db3"] or
            segments[0].get("message_count") != 144484):
        raise ConversionError("E_SOURCE_MANIFEST", "observed filename/count staging contract mismatch")
    for segment in segments:
        path = segment.get("staged_path")
        digest = segment.get("sha256")
        if not isinstance(path, str) or not path or not isinstance(digest, str) or len(digest) != 64:
            raise ConversionError("E_SOURCE_MANIFEST", "each segment requires staged_path and SHA-256")
        file_path = Path(path)
        if not file_path.is_file() or sha256_file(file_path) != digest:
            raise ConversionError("E_SOURCE_MANIFEST", f"staged segment hash mismatch: {path}")
    return list(segments)


def require_rosbags() -> tuple[Any, Any, Any, Any, Any]:
    try:
        import importlib.metadata
        version = importlib.metadata.version("rosbags")
        if version != ROSBAGS_VERSION:
            raise ImportError(f"found {version}")
        import numpy
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
        return numpy, Writer, Stores, get_typestore, get_types_from_msg
    except (ImportError, ModuleNotFoundError) as exc:
        raise ConversionError("E_DEPENDENCY", DEPENDENCY_DIAGNOSTIC) from exc


def sqlite_rows(segments: Sequence[Mapping[str, Any]]) -> Iterable[tuple[int, str, str, int, bytes]]:
    previous: tuple[int, int] | None = None
    for segment_index, segment in enumerate(segments):
        connection = sqlite3.connect(f"file:{Path(segment['staged_path']).resolve()}?mode=ro", uri=True)
        try:
            connection.execute("PRAGMA query_only=ON")
            rows = connection.execute(
                "SELECT m.id,t.name,t.type,m.timestamp,m.data FROM messages m "
                "JOIN topics t ON t.id=m.topic_id ORDER BY m.timestamp,m.id")
            for row_id, topic, msgtype, timestamp, payload in rows:
                key = (int(timestamp), int(row_id))
                try:
                    require_nondecreasing(previous, key, "E_STORAGE_ORDER",
                                          "storage timestamp/row order")
                except ConversionError as exc:
                    exc.row_id, exc.topic = int(row_id), str(topic)
                    raise
                previous = key
                yield int(row_id), str(topic), str(msgtype), int(timestamp), bytes(payload)
        except sqlite3.DatabaseError as exc:
            raise ConversionError("E_CDR_DESERIALIZE", f"sqlite corruption in segment {segment_index}: {exc}") from exc
        finally:
            connection.close()


def _ros1_header(store: Any, stamp_ns: int, frame: str) -> Any:
    sec, nanosec = ns_to_parts(stamp_ns)
    Time = store.types["builtin_interfaces/msg/Time"]
    Header = store.types["std_msgs/msg/Header"]
    return Header(0, Time(sec, nanosec), frame)


def cloud_message(store: Any, numpy: Any, record: Mapping[str, Any]) -> Any:
    PointField = store.types["sensor_msgs/msg/PointField"]
    PointCloud2 = store.types["sensor_msgs/msg/PointCloud2"]
    fields = [
        PointField("x", 0, 7, 1), PointField("y", 4, 7, 1), PointField("z", 8, 7, 1),
        PointField("intensity", 12, 7, 1), PointField("offset_time", 16, 6, 1),
        PointField("line", 20, 2, 1), PointField("tag", 21, 2, 1),
        PointField("reflectivity", 22, 2, 1), PointField("lidar_id", 23, 2, 1),
    ]
    width = int(record["width"])
    return PointCloud2(_ros1_header(store, int(record["normalized_time_ns"]), "lidar_link"),
                       1, width, fields, False, 24, 24 * width,
                       numpy.frombuffer(record["data"], dtype=numpy.uint8).copy(), True)


def imu_message(store: Any, numpy: Any, record: Mapping[str, Any]) -> Any:
    Quaternion = store.types["geometry_msgs/msg/Quaternion"]
    Vector3 = store.types["geometry_msgs/msg/Vector3"]
    Imu = store.types["sensor_msgs/msg/Imu"]
    q, av, la = record["orientation"], record["angular_velocity"], record["linear_acceleration"]
    cov = record["covariances"]
    arrays = [numpy.asarray(values, dtype=numpy.float64) for values in cov]
    return Imu(_ros1_header(store, int(record["normalized_time_ns"]), "imu_link"),
               Quaternion(*q), arrays[0], Vector3(*av), arrays[1], Vector3(*la), arrays[2])


def convert(args: argparse.Namespace) -> None:
    manifest_path = Path(args.staging_manifest).resolve()
    output = Path(args.output_directory).resolve()
    if output.exists():
        raise ConversionError("E_TRANSACTION", f"output already exists: {output}")
    manifest = load_json(manifest_path, "E_SOURCE_MANIFEST")
    segments = validate_staging(manifest)
    alignment = load_json(Path(args.alignment), "E_ALIGNMENT_SCHEMA")
    validate_alignment(alignment)
    if alignment["verified"]:
        abi_path = Path(__file__).resolve().parents[1] / "contracts/wp0/A10-conversion-abi-v1.md"
        if (alignment["provenance"]["source_bag_manifest_sha256"] != sha256_file(manifest_path) or
                not abi_path.is_file() or
                alignment["provenance"]["converter_abi_sha256"] != sha256_file(abi_path)):
            raise ConversionError("E_ALIGNMENT_SCHEMA",
                                  "verified alignment provenance does not match source manifest/converter ABI")
    msg_text, point_text = validate_idl(Path(args.custom_msg_idl), Path(args.custom_point_idl))
    if len(args.frame_evidence_sha256) != 64 or any(x not in "0123456789abcdef" for x in args.frame_evidence_sha256):
        raise ConversionError("E_FRAME_MAPPING", "frame evidence must be a lowercase SHA-256")
    numpy, Writer, Stores, get_typestore, get_types_from_msg = require_rosbags()
    store_name = f"ROS2_{args.ros_distribution.upper()}"
    if store_name not in Stores.__members__:
        raise ConversionError("E_SOURCE_MANIFEST",
                              f"rosbags has no pinned typestore for {args.ros_distribution!r}")
    ros2 = get_typestore(Stores[store_name])
    types = get_types_from_msg(point_text, "livox_ros_driver2/msg/CustomPoint")
    types.update(get_types_from_msg(msg_text, LIVOX_TYPE))
    ros2.register(types)
    ros1 = get_typestore(Stores.ROS1_NOETIC)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output.name}.unaccepted-", dir=output.parent))
    counts = {CLOUD_TOPIC: 0, OUTPUT_IMU_TOPIC: 0}
    first_storage: int | None = None
    last_storage: int | None = None
    first_source: dict[str, int] = {}
    last_source: dict[str, int] = {}
    clock_ages: dict[str, list[int]] = {LIVOX_TOPIC: [], IMU_TOPIC: []}
    offset_statistics = {
        "cloud_count": 0, "point_count": 0, "clouds_with_adjacent_decreases": 0,
        "adjacent_offset_decrease_count": 0,
    }
    imu_orientation_available = True
    semantic = hashlib.sha256()
    record_path = temporary / "records.jsonl"
    bag_path = temporary / "normalized.bag"
    try:
        writer = Writer(bag_path)
        writer.chunk_threshold = 768 * 1024
        with writer, record_path.open("wb") as index:
            cloud_conn = writer.add_connection(CLOUD_TOPIC, "sensor_msgs/msg/PointCloud2", typestore=ros1,
                                               callerid="/offline_livox_normalizer")
            imu_conn = writer.add_connection(OUTPUT_IMU_TOPIC, IMU_TYPE, typestore=ros1,
                                             callerid="/offline_livox_normalizer")
            previous_source: dict[str, int] = {}
            for row_id, topic, msgtype, storage_ns, payload in sqlite_rows(segments):
                if topic not in (LIVOX_TOPIC, IMU_TOPIC) or msgtype != ({LIVOX_TOPIC: LIVOX_TYPE, IMU_TOPIC: IMU_TYPE}.get(topic)):
                    raise ConversionError("E_TOPIC_TYPE", f"unexpected {topic} ({msgtype})", row_id, topic)
                try:
                    decoded = ros2.deserialize_cdr(payload, msgtype)
                except Exception as exc:
                    raise ConversionError("E_CDR_DESERIALIZE", str(exc), row_id, topic) from exc
                try:
                    if topic == LIVOX_TOPIC:
                        canonical = canonicalize_cloud(decoded, storage_time_ns=storage_ns,
                                                       source_frame=args.lidar_source_frame,
                                                       offset_ns=alignment["lidar_offset_ns"])
                        outmsg, outtopic, conn = cloud_message(ros1, numpy, canonical), CLOUD_TOPIC, cloud_conn
                    else:
                        canonical = canonicalize_imu(decoded, storage_time_ns=storage_ns,
                                                     source_frame=args.imu_source_frame,
                                                     offset_ns=alignment["imu_offset_ns"])
                        imu_orientation_available = (
                            imu_orientation_available and canonical["orientation_available"])
                        outmsg, outtopic, conn = imu_message(ros1, numpy, canonical), OUTPUT_IMU_TOPIC, imu_conn
                except ConversionError as exc:
                    exc.row_id, exc.topic = row_id, topic
                    raise
                source_ns = int(canonical["source_time_ns"])
                try:
                    require_nondecreasing(previous_source.get(topic), source_ns,
                                          "E_SOURCE_TIME_REGRESSION", "source stamp")
                except ConversionError as exc:
                    exc.row_id, exc.topic = row_id, topic
                    raise
                previous_source[topic] = source_ns
                raw = bytes(ros1.serialize_ros1(outmsg, conn.msgtype))
                writer.write(conn, storage_ns, raw)
                semantic.update(struct.pack("<IQ", len(outtopic), storage_ns))
                semantic.update(outtopic.encode())
                semantic.update(struct.pack("<I", len(raw)))
                semantic.update(raw)
                counts[outtopic] += 1
                first_storage = storage_ns if first_storage is None else first_storage
                last_storage = storage_ns
                first_source.setdefault(topic, source_ns)
                last_source[topic] = source_ns
                clock_ages[topic].append(storage_ns - int(canonical["normalized_time_ns"]))
                if topic == LIVOX_TOPIC:
                    offset_statistics["cloud_count"] += 1
                    offset_statistics["point_count"] += canonical["point_count"]
                    decreases = canonical["adjacent_offset_decrease_count"]
                    offset_statistics["adjacent_offset_decrease_count"] += decreases
                    if decreases:
                        offset_statistics["clouds_with_adjacent_decreases"] += 1
                entry = {
                    "decoded_canonical_payload_sha256": hashlib.sha256(raw).hexdigest(),
                    "input_payload_sha256": hashlib.sha256(payload).hexdigest(),
                    "max_point_time_ns": canonical.get("max_point_time_ns"),
                    "min_point_time_ns": canonical.get("min_point_time_ns"),
                    "point_count": canonical.get("point_count"),
                    "minimum_offset_time": canonical.get("minimum_offset_time"),
                    "maximum_offset_time": canonical.get("maximum_offset_time"),
                    "adjacent_offset_decrease_count": canonical.get(
                        "adjacent_offset_decrease_count"),
                    "reserved": canonical.get("reserved"), "row_id": row_id,
                    "source_header_time_ns": canonical.get("header_time_ns"),
                    "source_time_ns": source_ns, "storage_time_ns": storage_ns,
                    "topic": topic,
                }
                index.write(canonical_json(entry))
        if counts != {CLOUD_TOPIC: 6882, OUTPUT_IMU_TOPIC: 137602}:
            raise ConversionError("E_COUNT_DURATION", f"output counts mismatch: {counts}")
        if first_storage != manifest["totals"]["starting_time_ns"] or last_storage != manifest["totals"]["ending_time_ns"]:
            raise ConversionError("E_COUNT_DURATION", "output first/last storage time mismatch")
        report = {
            "counts": counts, "errors": [], "gaps": [],
            "offset_time_regression_statistics": offset_statistics,
            "semantic_stream_sha256": semantic.hexdigest(),
        }
        (temporary / "conversion_report.json").write_bytes(canonical_json(report))
        output_topics = {
            CLOUD_TOPIC: {"type": "sensor_msgs/msg/PointCloud2", "count": counts[CLOUD_TOPIC]},
            OUTPUT_IMU_TOPIC: {"type": IMU_TYPE, "count": counts[OUTPUT_IMU_TOPIC]},
        }
        result_manifest = {
            "schema_version": 1, "artifact_id": "wheelchair.normalized_livox/v1",
            "owner": args.owner, "reviewer": args.reviewer, "status": "candidate",
            "provenance": {"source_staging_manifest_sha256": sha256_file(manifest_path),
                           "ros_distribution": args.ros_distribution,
                           "livox_driver_revision": args.livox_driver_revision},
            "source": {"segments": [{"sha256": x["sha256"], "size_bytes": x["size_bytes"]} for x in segments],
                       "custom_msg_sha256": CUSTOM_MSG_SHA256, "custom_point_sha256": CUSTOM_POINT_SHA256,
                       "custom_composite_sha256": CUSTOM_COMPOSITE_SHA256,
                       "storage": "sqlite3", "serialization_format": "cdr"},
            "output": {"bag_path": "normalized.bag", "sha256": sha256_file(bag_path),
                       "format": "rosbag1-v2", "topics": output_topics,
                       "compression": "none", "chunk_threshold_bytes": 768 * 1024,
                       "counts": counts, "first_storage_time_ns": first_storage,
                       "last_storage_time_ns": last_storage,
                       "first_source_time_ns": first_source, "last_source_time_ns": last_source,
                       "clock_statistics": {
                           topic: {"minimum_age_ns": min(ages), "maximum_age_ns": max(ages)}
                           for topic, ages in clock_ages.items()
                       },
                       "offset_time_regression_statistics": offset_statistics},
            "parameters": {"alignment_sha256": sha256_file(Path(args.alignment)),
                           "lidar_offset_ns": alignment["lidar_offset_ns"],
                           "imu_offset_ns": alignment["imu_offset_ns"]},
            "frame_mappings": [
                {"source": args.lidar_source_frame, "canonical": "lidar_link", "evidence_sha256": args.frame_evidence_sha256},
                {"source": args.imu_source_frame, "canonical": "imu_link", "evidence_sha256": args.frame_evidence_sha256}],
            "qualification": {"alignment_verified": alignment["verified"],
                              "imu_orientation_available": imu_orientation_available,
                              "fusion_qualified": bool(
                                  alignment["verified"] and imu_orientation_available)},
            "records_sha256": sha256_file(record_path),
            "conversion_report_sha256": sha256_file(temporary / "conversion_report.json"),
            "tool": {"rosbags_version": ROSBAGS_VERSION, "converter_sha256": sha256_file(Path(__file__))},
        }
        (temporary / "normalization_manifest.yaml").write_bytes(canonical_json(result_manifest))
        os.replace(temporary, output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("staging_manifest")
    value.add_argument("output_directory")
    value.add_argument("--alignment", required=True)
    value.add_argument("--custom-msg-idl", required=True)
    value.add_argument("--custom-point-idl", required=True)
    value.add_argument("--lidar-source-frame", required=True)
    value.add_argument("--imu-source-frame", required=True)
    value.add_argument("--frame-evidence-sha256", required=True)
    value.add_argument("--ros-distribution", required=True)
    value.add_argument("--livox-driver-revision", required=True)
    value.add_argument("--owner", required=True)
    value.add_argument("--reviewer", required=True)
    return value


def emit_error_report(output_directory: str, error: ConversionError) -> None:
    payload = canonical_json({"errors": [error.as_dict()], "status": "error"})
    sys.stderr.buffer.write(payload)
    destination = Path(str(Path(output_directory).resolve()) + ".conversion_error.json")
    temporary = destination.with_name(f".{destination.name}.tmp-{os.getpid()}")
    try:
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_bytes(payload)
        os.replace(temporary, destination)
    except OSError:
        try:
            temporary.unlink()
        except OSError:
            pass


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        convert(args)
        return 0
    except ConversionError as exc:
        emit_error_report(args.output_directory, exc)
        return 2
    except Exception as exc:
        error = ConversionError("E_TRANSACTION", f"unexpected transaction failure: {exc}")
        emit_error_report(args.output_directory, error)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
