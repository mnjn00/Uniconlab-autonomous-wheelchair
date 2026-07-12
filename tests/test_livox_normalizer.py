from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from scripts import normalize_livox_bag as normalizer
from scripts import run_glim_repro
from scripts import verify_normalized_bag as normalized_verifier


CUSTOM_MSG_BYTES = b"""# Livox publish pointcloud msg format.

std_msgs/Header header    # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data

"""
CUSTOM_POINT_BYTES = b"""# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar

"""


def stamp(ns: int) -> NS:
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def point(offset: int, x: float = 1.0, y: float = 2.0, z: float = 3.0,
          reflectivity: int = 17, tag: int = 4, line: int = 2) -> NS:
    return NS(offset_time=offset, x=x, y=y, z=z, reflectivity=reflectivity,
              tag=tag, line=line)


def cloud(points=None, *, point_num=None, timebase=10_000_000_000,
          header_ns=None, frame="livox_frame", lidar_id=7) -> NS:
    points = [point(3), point(9, -1.25, 0.5, 42.0, 255, 8, 6)] if points is None else points
    return NS(header=NS(stamp=stamp(timebase if header_ns is None else header_ns), frame_id=frame),
              timebase=timebase, point_num=len(points) if point_num is None else point_num,
              lidar_id=lidar_id, rsvd=[1, 2, 3], points=points)


def imu(*, frame="imu_source", source_ns=10_000_000_000,
        orientation=(0.1, 0.2, 0.3, math.sqrt(0.86))) -> NS:
    covariance = [0.01, 0.001, 0.002, 0.001, 0.02, 0.003, 0.002, 0.003, 0.03]
    return NS(
        header=NS(stamp=stamp(source_ns), frame_id=frame),
        orientation=NS(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]),
        orientation_covariance=covariance,
        angular_velocity=NS(x=0.11, y=-0.22, z=0.33),
        angular_velocity_covariance=[x * 2 for x in covariance],
        linear_acceleration=NS(x=1.25, y=-2.5, z=9.7),
        linear_acceleration_covariance=[x * 3 for x in covariance],
    )


def error_code(callable_) -> str:
    with pytest.raises(normalizer.ConversionError) as caught:
        callable_()
    return caught.value.code


def test_cloud_conversion_has_exact_little_endian_24_byte_layout_and_is_stable():
    first = normalizer.canonicalize_cloud(
        cloud(), storage_time_ns=10_000_000_000, source_frame="livox_frame")
    second = normalizer.canonicalize_cloud(
        cloud(), storage_time_ns=10_000_000_000, source_frame="livox_frame")
    assert first == second
    assert len(first["data"]) == 48
    assert first["reserved"] == [1, 2, 3]
    assert first["min_point_time_ns"] == 10_000_000_003
    assert first["max_point_time_ns"] == 10_000_000_009
    assert normalizer.POINT_STRUCT.unpack_from(first["data"], 0) == (
        1.0, 2.0, 3.0, 17.0, 3, 2, 4, 17, 7)
    assert normalizer.POINT_STRUCT.unpack_from(first["data"], 24) == (
        -1.25, 0.5, 42.0, 255.0, 9, 6, 8, 255, 7)
    assert first["adjacent_offset_decrease_count"] == 0
    assert first["minimum_offset_time"] == 3
    assert first["maximum_offset_time"] == 9


def test_cloud_preserves_interleaved_offset_order_and_reports_all_point_extents():
    source_points = [
        point(90, x=1.0), point(7, x=2.0), point(42, x=3.0), point(6, x=4.0),
    ]
    result = normalizer.canonicalize_cloud(
        cloud(points=source_points), storage_time_ns=10_000_000_000,
        source_frame="livox_frame")

    packed_offsets = [
        normalizer.POINT_STRUCT.unpack_from(result["data"], index * 24)[4]
        for index in range(len(source_points))
    ]
    assert packed_offsets == [90, 7, 42, 6]
    assert result["minimum_offset_time"] == 6
    assert result["maximum_offset_time"] == 90
    assert result["min_point_time_ns"] == 10_000_000_006
    assert result["max_point_time_ns"] == 10_000_000_090
    assert result["adjacent_offset_decrease_count"] == 2


def test_independent_cloud_verifier_recomputes_offset_statistics():
    canonical = normalizer.canonicalize_cloud(
        cloud(points=[point(90), point(7), point(42), point(6)]),
        storage_time_ns=10_000_000_000, source_frame="livox_frame")
    record = dict(canonical, source_header_time_ns=canonical["header_time_ns"])
    fields = [
        NS(name=name, offset=offset, datatype=datatype, count=count)
        for name, offset, datatype, count in normalized_verifier.EXPECTED_FIELDS
    ]
    message = NS(
        header=NS(frame_id="lidar_link", stamp=stamp(10_000_000_000)),
        height=1, width=4, is_bigendian=False, is_dense=True, point_step=24,
        row_step=96, fields=fields, data=canonical["data"],
    )

    assert normalized_verifier.verify_cloud(message, record, 0) == {
        "point_count": 4, "adjacent_offset_decrease_count": 2,
    }
    tampered = dict(record, adjacent_offset_decrease_count=1)
    with pytest.raises(normalized_verifier.VerificationError):
        normalized_verifier.verify_cloud(message, tampered, 0)


@pytest.mark.parametrize(
    ("value", "code"),
    [
        (lambda: cloud(point_num=3), "E_POINT_COUNT"),
        (lambda: cloud(points=[point(-1)]), "E_POINT_LAYOUT"),
        (lambda: cloud(points=[point(1 << 32)]), "E_POINT_LAYOUT"),
        (lambda: cloud(points=[point(1.5)]), "E_POINT_LAYOUT"),
        (lambda: cloud(points=[point(1)], timebase=normalizer.UINT64_MAX), "E_POINT_TIME_OVERFLOW"),
        (lambda: cloud(points=[point(0, x=float("nan"))]), "E_NONFINITE"),
        (lambda: cloud(frame=""), "E_FRAME_MAPPING"),
        (lambda: cloud(header_ns=10_002_000_000), "E_HEADER_TIME_RESIDUAL"),
    ],
)
def test_cloud_corruption_is_rejected_with_stable_code(value, code):
    assert error_code(lambda: normalizer.canonicalize_cloud(
        value(), storage_time_ns=10_000_000_000, source_frame="livox_frame")) == code


def test_cloud_future_time_is_not_repaired():
    assert error_code(lambda: normalizer.canonicalize_cloud(
        cloud(), storage_time_ns=9_949_999_999, source_frame="livox_frame")) == "E_CLOCK_FUTURE"


def test_imu_preserves_nontrivial_values_and_covariances():
    message = imu()
    result = normalizer.canonicalize_imu(
        message, storage_time_ns=10_000_000_000, source_frame="imu_source", offset_ns=-25)
    assert result["normalized_time_ns"] == 9_999_999_975
    assert result["orientation"] == (message.orientation.x, message.orientation.y,
                                      message.orientation.z, message.orientation.w)
    assert result["angular_velocity"] == (0.11, -0.22, 0.33)
    assert result["linear_acceleration"] == (1.25, -2.5, 9.7)
    assert result["covariances"][1] == tuple(message.angular_velocity_covariance)


def test_unavailable_imu_orientation_sentinel_is_preserved():
    message = imu(orientation=(0.0, 0.0, 0.0, 0.0))
    message.orientation_covariance[0] = -1.0
    result = normalizer.canonicalize_imu(
        message, storage_time_ns=10_000_000_000, source_frame="imu_source")
    assert result["orientation_available"] is False
    assert result["orientation"] == (0.0, 0.0, 0.0, 0.0)
    assert result["covariances"][0][0] == -1.0


@pytest.mark.parametrize(
    ("mutate", "code"),
    [
        (lambda value: setattr(value.orientation, "w", float("inf")), "E_NONFINITE"),
        (lambda value: setattr(value.orientation, "w", 0.1), "E_IMU_MALFORMED"),
        (lambda value: setattr(value, "linear_acceleration_covariance", [0.0] * 8), "E_IMU_MALFORMED"),
        (lambda value: setattr(value.header, "frame_id", "wrong"), "E_FRAME_MAPPING"),
    ],
)
def test_imu_corruption_is_rejected(mutate, code):
    value = imu()
    mutate(value)
    assert error_code(lambda: normalizer.canonicalize_imu(
        value, storage_time_ns=10_000_000_000, source_frame="imu_source")) == code


def alignment(**updates):
    value = {
        "schema_version": 1, "artifact_id": "alignment", "owner": "owner",
        "reviewer": "reviewer", "status": "candidate",
        "provenance": {"source_bag_manifest_sha256": "", "converter_abi_sha256": "",
                       "calibration_method": "none"},
        "method": "identity", "lidar_offset_ns": 0, "imu_offset_ns": 0,
        "calibration_evidence_sha256": "", "verified": False,
        "p99_cross_sensor_residual_ms": None, "drift_ms_per_min": None,
    }
    value.update(updates)
    return value


def test_alignment_rejects_unknown_fields_and_identity_offsets():
    unknown = alignment()
    unknown["warp"] = 1
    assert error_code(lambda: normalizer.validate_alignment(unknown)) == "E_ALIGNMENT_SCHEMA"
    assert error_code(lambda: normalizer.validate_alignment(
        alignment(lidar_offset_ns=1))) == "E_ALIGNMENT_SCHEMA"


def test_source_idl_byte_mismatch_is_rejected(tmp_path: Path):
    custom_msg = tmp_path / "CustomMsg.msg"
    custom_point = tmp_path / "CustomPoint.msg"
    custom_msg.write_bytes(b"not the pinned IDL")
    custom_point.write_bytes(b"not the pinned IDL")
    assert error_code(lambda: normalizer.validate_idl(
        custom_msg, custom_point)) == "E_SOURCE_IDL"


def test_exact_source_idl_and_composite_hash_are_accepted(tmp_path: Path):
    custom_msg = tmp_path / "CustomMsg.msg"
    custom_point = tmp_path / "CustomPoint.msg"
    custom_msg.write_bytes(CUSTOM_MSG_BYTES)
    custom_point.write_bytes(CUSTOM_POINT_BYTES)

    assert hashlib.sha256(
        b"CustomMsg.msg\0" + CUSTOM_MSG_BYTES + b"\0" +
        b"CustomPoint.msg\0" + CUSTOM_POINT_BYTES + b"\0"
    ).hexdigest() == normalizer.CUSTOM_COMPOSITE_SHA256
    assert normalizer.validate_idl(custom_msg, custom_point) == (
        CUSTOM_MSG_BYTES.decode(), CUSTOM_POINT_BYTES.decode())


def staging_manifest(segment: Path, mismatch_reason: str) -> dict:
    return {
        "schema_version": "wheelchair.rosbag2_manifest/v1",
        "status": "staged",
        "operation": "stage",
        "errors": [],
        "mismatches": [{
            "code": "E_SOURCE_DISCOVERY",
            "context": {"mismatch_reason": mismatch_reason},
        }],
        "metadata": {
            "version": 5,
            "storage_identifier": "sqlite3",
            "declared_relative_file_paths": ["livox_raw_20260707_191720_0.db3"],
        },
        "provenance": {"storage_identifier": "sqlite3"},
        "topics": [
            {
                "name": normalizer.LIVOX_TOPIC,
                "type": normalizer.LIVOX_TYPE,
                "serialization_format": "cdr",
                "metadata_count": 6882,
                "sqlite_count": 6882,
            },
            {
                "name": normalizer.IMU_TOPIC,
                "type": normalizer.IMU_TYPE,
                "serialization_format": "cdr",
                "metadata_count": 137602,
                "sqlite_count": 137602,
            },
        ],
        "totals": {"message_count": 144484, "duration_ns": 688225098527},
        "segments": [{
            "source_path": str(segment),
            "staged_path": str(segment),
            "sha256": normalizer.sha256_file(segment),
            "size_bytes": segment.stat().st_size,
            "message_count": 144484,
        }],
    }


@pytest.mark.parametrize("reason", [
    "zero_byte_expected_populated_suffixed_segment",
    "pre_existing_source_symlink_alias",
])
def test_only_audited_staging_aliases_are_accepted(tmp_path: Path, reason: str):
    source_segment = tmp_path / "livox_raw_20260707_191720_0-001.db3"
    source_segment.write_bytes(b"staged sqlite fixture")
    staged_segment = source_segment
    if reason == "pre_existing_source_symlink_alias":
        staged_segment = tmp_path / "staged-alias.db3"
        staged_segment.symlink_to(source_segment)
    manifest = staging_manifest(source_segment, reason)
    manifest["segments"][0]["staged_path"] = str(staged_segment)
    assert normalizer.validate_staging(manifest)[0]["staged_path"] == str(staged_segment)


def test_unaudited_staging_mismatch_remains_blocked(tmp_path: Path):
    segment = tmp_path / "livox_raw_20260707_191720_0-001.db3"
    segment.write_bytes(b"staged sqlite fixture")
    manifest = staging_manifest(segment, "unreviewed_alias")
    assert error_code(lambda: normalizer.validate_staging(manifest)) == "E_SOURCE_MANIFEST"


def test_normalization_manifest_is_exact_glim_consumer_abi(tmp_path: Path):
    bag = tmp_path / "normalized.bag"
    bag.write_bytes(b"canonical rosbag fixture")
    counts = {
        normalizer.CLOUD_TOPIC: 6882,
        normalizer.OUTPUT_IMU_TOPIC: 137602,
    }
    output = {
        "bag_path": "normalized.bag",
        "sha256": normalizer.sha256_file(bag),
        "format": "rosbag1-v2",
        "topics": normalized_verifier.EXPECTED_TOPICS,
        "compression": "none",
        "chunk_threshold_bytes": 768 * 1024,
        "counts": counts,
        "first_storage_time_ns": 1,
        "last_storage_time_ns": 2,
        "first_source_time_ns": {},
        "last_source_time_ns": {},
        "clock_statistics": {},
        "offset_time_regression_statistics": {
            "cloud_count": 6882,
            "point_count": 1_000_000,
            "clouds_with_adjacent_decreases": 6882,
            "adjacent_offset_decrease_count": 910296,
        },
    }
    manifest = {
        "schema_version": 1,
        "artifact_id": "wheelchair.normalized_livox/v1",
        "status": "candidate",
        "output": output,
    }
    manifest_path = tmp_path / "normalization_manifest.yaml"
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")

    assert normalized_verifier.validate_manifest_abi(manifest) == output
    loaded, loaded_bag, digest = run_glim_repro.load_input_manifest(manifest_path)
    assert loaded == manifest
    assert loaded_bag == bag
    assert digest == output["sha256"]

    legacy = json.loads(json.dumps(manifest))
    legacy["artifact_id"] = "normalized-livox-bag-v1"
    legacy["output"]["bag_sha256"] = legacy["output"].pop("sha256")
    with pytest.raises(normalized_verifier.VerificationError):
        normalized_verifier.validate_manifest_abi(legacy)

@pytest.mark.parametrize(
    ("previous", "current", "code"),
    [
        (101, 100, "E_SOURCE_TIME_REGRESSION"),
        ((100, 2), (100, 1), "E_STORAGE_ORDER"),
    ],
)
def test_time_regressions_are_rejected(previous, current, code):
    assert error_code(lambda: normalizer.require_nondecreasing(
        previous, current, code, "test time")) == code


def test_preflight_failure_is_transactional(tmp_path: Path):
    source = tmp_path / "staging.json"
    source.write_text(json.dumps({"schema_version": "wheelchair.rosbag2_manifest/v1",
                                  "status": "mismatch", "operation": "stage"}), encoding="utf-8")
    output = tmp_path / "accepted"
    args = argparse.Namespace(staging_manifest=str(source), output_directory=str(output))
    with pytest.raises(normalizer.ConversionError) as caught:
        normalizer.convert(args)
    assert caught.value.code == "E_SOURCE_MANIFEST"
    assert not output.exists()
    assert list(tmp_path.glob(".accepted.unaccepted-*")) == []


def test_synthetic_rosbags_cdr_to_ros1_integration():
    rosbags = pytest.importorskip(
        "rosbags", reason="optional integration requires pinned rosbags==0.10.11 offline dependency")
    del rosbags
    import importlib.metadata
    if importlib.metadata.version("rosbags") != normalizer.ROSBAGS_VERSION:
        pytest.skip("optional integration requires pinned rosbags==0.10.11 offline dependency")
    import numpy
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    custom_point = CUSTOM_POINT_BYTES.decode()
    custom_msg = CUSTOM_MSG_BYTES.decode()
    ros2 = get_typestore(Stores.ROS2_HUMBLE)
    types = get_types_from_msg(custom_point, "livox_ros_driver2/msg/CustomPoint")
    types.update(get_types_from_msg(custom_msg, normalizer.LIVOX_TYPE))
    ros2.register(types)
    Time = ros2.types["builtin_interfaces/msg/Time"]
    Header = ros2.types["std_msgs/msg/Header"]
    Point = ros2.types["livox_ros_driver2/msg/CustomPoint"]
    Custom = ros2.types[normalizer.LIVOX_TYPE]
    decoded_source = Custom(
        Header(Time(10, 0), "livox_frame"), 10_000_000_000, 2, 7,
        numpy.array([1, 2, 3], dtype=numpy.uint8),
        [Point(3, 1.0, 2.0, 3.0, 17, 4, 2),
         Point(9, -1.25, 0.5, 42.0, 255, 8, 6)],
    )
    cdr = bytes(ros2.serialize_cdr(decoded_source, normalizer.LIVOX_TYPE))
    decoded = ros2.deserialize_cdr(cdr, normalizer.LIVOX_TYPE)
    canonical = normalizer.canonicalize_cloud(
        decoded, storage_time_ns=10_000_000_000, source_frame="livox_frame")
    ros1 = get_typestore(Stores.ROS1_NOETIC)
    output = normalizer.cloud_message(ros1, numpy, canonical)
    raw1 = bytes(ros1.serialize_ros1(output, output.__msgtype__))
    raw2 = bytes(ros1.serialize_ros1(output, output.__msgtype__))
    assert raw1 == raw2
    roundtrip = ros1.deserialize_ros1(raw1, output.__msgtype__)
    assert roundtrip.header.frame_id == "lidar_link"
    assert roundtrip.point_step == 24
    assert bytes(roundtrip.data) == canonical["data"]
    with pytest.raises(Exception):
        ros2.deserialize_cdr(cdr[:-1], normalizer.LIVOX_TYPE)
