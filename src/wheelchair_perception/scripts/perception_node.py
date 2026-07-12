#!/usr/bin/env python3
"""ROS 1 adapter for canonical lidar data and navigation-only perception."""

import copy
from collections import deque
from dataclasses import dataclass
import math
import struct
import threading
import sys
import time
from pathlib import Path

_SCRIPT_DIRECTORY = str(Path(__file__).resolve().parent)
if _SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, _SCRIPT_DIRECTORY)

from perception_core import CloudInput, ImuSample, PerceptionConfig, PerceptionCore, Point


POINT_STEP = 24
_FLOAT32 = 7
_UINT32 = 6
_UINT8 = 2
CANONICAL_FIELDS = (
    ("x", 0, _FLOAT32, 1),
    ("y", 4, _FLOAT32, 1),
    ("z", 8, _FLOAT32, 1),
    ("intensity", 12, _FLOAT32, 1),
    ("offset_time", 16, _UINT32, 1),
    ("line", 20, _UINT8, 1),
    ("tag", 21, _UINT8, 1),
    ("reflectivity", 22, _UINT8, 1),
    ("lidar_id", 23, _UINT8, 1),
)
_POINT_STRUCT = struct.Struct("<ffffIBBBB")
_INT64_MAX = (1 << 63) - 1


class PointCloudCodecError(ValueError):
    """A stable, fail-closed canonical cloud validation error."""

    def __init__(self, code, message):
        super().__init__("{}: {}".format(code, message))
        self.code = code


def stamp_to_seconds(stamp):
    try:
        if hasattr(stamp, "to_sec"):
            value = float(stamp.to_sec())
        elif hasattr(stamp, "secs") and hasattr(stamp, "nsecs"):
            value = float(stamp.secs) + float(stamp.nsecs) * 1.0e-9
        else:
            value = float(stamp)
    except (AttributeError, TypeError, ValueError) as exc:
        raise PointCloudCodecError("E_SOURCE_TIME", "stamp is not numeric") from exc
    if not math.isfinite(value) or value <= 0.0:
        raise PointCloudCodecError("E_SOURCE_TIME", "stamp must be finite and positive")
    return value


def _stamp_nanoseconds(stamp_s):
    scaled_stamp = stamp_s * 1.0e9
    if not math.isfinite(scaled_stamp) or scaled_stamp > _INT64_MAX:
        raise PointCloudCodecError(
            "E_POINT_TIME_OVERFLOW", "stamp is not representable as int64 nanoseconds"
        )
    return int(scaled_stamp)


def _validate_acquisition_time(stamp_ns, offset_time):
    if type(offset_time) is not int or not 0 <= offset_time <= 0xFFFFFFFF:
        raise PointCloudCodecError("E_POINT_TIME_OVERFLOW", "offset_time is outside uint32")
    if stamp_ns + offset_time > _INT64_MAX:
        raise PointCloudCodecError(
            "E_POINT_TIME_OVERFLOW", "stamp plus offset is not representable"
        )


def _field_signature(fields):
    try:
        return tuple((field.name, field.offset, field.datatype, field.count) for field in fields)
    except (AttributeError, TypeError) as exc:
        raise PointCloudCodecError("E_POINT_LAYOUT", "invalid PointField sequence") from exc


def validate_pointcloud2(
    message,
    expected_frame="lidar_link",
    previous_stamp_s=None,
    now_s=None,
    max_age_s=None,
    max_future_s=0.05,
):
    """Validate the byte-exact WP0 A10 PointCloud2 contract and source time."""
    if getattr(message, "height", None) != 1:
        raise PointCloudCodecError("E_POINT_LAYOUT", "height must equal one")
    width = getattr(message, "width", None)
    if not isinstance(width, int) or isinstance(width, bool) or width < 0:
        raise PointCloudCodecError("E_POINT_LAYOUT", "width must be a nonnegative integer")
    if bool(getattr(message, "is_bigendian", True)):
        raise PointCloudCodecError("E_ENDIAN", "canonical clouds are little-endian")
    if not bool(getattr(message, "is_dense", False)):
        raise PointCloudCodecError("E_POINT_LAYOUT", "canonical clouds must be dense")
    if getattr(message, "point_step", None) != POINT_STEP:
        raise PointCloudCodecError("E_POINT_LAYOUT", "point_step must equal 24")
    expected_row_step = POINT_STEP * width
    if getattr(message, "row_step", None) != expected_row_step:
        raise PointCloudCodecError("E_POINT_LAYOUT", "row_step does not match width")
    if _field_signature(getattr(message, "fields", None)) != CANONICAL_FIELDS:
        raise PointCloudCodecError("E_POINT_LAYOUT", "fields, offsets, types, or counts differ")
    data = getattr(message, "data", None)
    try:
        data_length = len(data)
    except TypeError as exc:
        raise PointCloudCodecError("E_POINT_LAYOUT", "data is not a byte sequence") from exc
    if data_length != expected_row_step:
        raise PointCloudCodecError("E_POINT_LAYOUT", "data length does not match row_step")
    frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
    if not frame_id or frame_id != expected_frame:
        raise PointCloudCodecError("E_FRAME_MAPPING", "unexpected cloud frame")
    stamp_s = stamp_to_seconds(getattr(getattr(message, "header", None), "stamp", None))
    _stamp_nanoseconds(stamp_s)
    if previous_stamp_s is not None and stamp_s < previous_stamp_s:
        raise PointCloudCodecError("E_SOURCE_TIME_REGRESSION", "cloud stamp regressed")
    if now_s is not None:
        now_s = float(now_s)
        if not math.isfinite(now_s):
            raise PointCloudCodecError("E_CLOCK", "evaluation time is non-finite")
        if stamp_s - now_s > float(max_future_s):
            raise PointCloudCodecError("E_CLOCK_FUTURE", "cloud stamp is in the future")
        if max_age_s is not None and now_s - stamp_s > float(max_age_s):
            raise PointCloudCodecError("E_SENSOR_STALE", "cloud is stale")
    return stamp_s


def decode_pointcloud2(message, expected_frame="lidar_link", **time_checks):
    """Decode a validated canonical cloud in exact acquisition/source order."""
    stamp_s = validate_pointcloud2(message, expected_frame=expected_frame, **time_checks)
    stamp_ns = _stamp_nanoseconds(stamp_s)
    points = []
    for values in _POINT_STRUCT.iter_unpack(bytes(message.data)):
        x, y, z, intensity, offset_time, line, tag, reflectivity, lidar_id = values
        if not all(math.isfinite(value) for value in (x, y, z, intensity)):
            raise PointCloudCodecError("E_NONFINITE", "point contains a non-finite float")
        if intensity != float(reflectivity):
            raise PointCloudCodecError("E_POINT_LAYOUT", "intensity must exactly equal reflectivity")
        _validate_acquisition_time(stamp_ns, offset_time)
        points.append(Point(x, y, z, offset_time, reflectivity, tag, line, lidar_id))
    return CloudInput(stamp_s, expected_frame, tuple(points))


def _point_values(point):
    coordinates = (point.x, point.y, point.z)
    integers = (
        point.offset_time,
        point.reflectivity,
        point.tag,
        point.line,
        point.lidar_id,
    )
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in coordinates
    ):
        raise PointCloudCodecError("E_POINT_LAYOUT", "coordinates must be numeric")
    if any(type(value) is not int for value in integers):
        raise PointCloudCodecError("E_POINT_LAYOUT", "time and metadata require integer types")
    values = tuple(float(value) for value in coordinates) + integers
    if not all(math.isfinite(value) for value in values[:3]):
        raise PointCloudCodecError("E_NONFINITE", "point contains a non-finite coordinate")
    if not 0 <= values[3] <= 0xFFFFFFFF:
        raise PointCloudCodecError("E_POINT_TIME_OVERFLOW", "offset_time is outside uint32")
    if any(not 0 <= value <= 0xFF for value in values[4:]):
        raise PointCloudCodecError("E_POINT_LAYOUT", "byte field is outside uint8")
    return values


def encode_pointcloud2(template, points):
    """Encode ``points`` in iteration order without sorting or repairing offsets."""
    stamp_s = stamp_to_seconds(getattr(getattr(template, "header", None), "stamp", None))
    stamp_ns = _stamp_nanoseconds(stamp_s)
    encoded = bytearray()
    for point in points:
        x, y, z, offset_time, reflectivity, tag, line, lidar_id = _point_values(point)
        _validate_acquisition_time(stamp_ns, offset_time)
        encoded.extend(
            _POINT_STRUCT.pack(
                x, y, z, float(reflectivity), offset_time, line, tag, reflectivity, lidar_id
            )
        )
    output = copy.copy(template)
    output.height = 1
    output.width = len(encoded) // POINT_STEP
    output.fields = copy.copy(template.fields)
    output.is_bigendian = False
    output.point_step = POINT_STEP
    output.row_step = len(encoded)
    output.data = bytes(encoded)
    output.is_dense = True
    validate_pointcloud2(output, expected_frame=output.header.frame_id)
    return output


@dataclass(frozen=True)
class _CachedImu:
    stamp_s: float
    sample: ImuSample


class ImuCache:
    """Bounded IMU cache that accepts only monotonic, locally credible chronology."""

    def __init__(
        self,
        expected_frame="imu_link",
        max_skew_s=0.02,
        capacity=256,
        max_future_s=0.05,
        max_gap_s=0.50,
    ):
        if capacity < 1 or min(max_skew_s, max_future_s, max_gap_s) < 0.0:
            raise ValueError("invalid IMU cache configuration")
        self.expected_frame = expected_frame
        self.max_skew_s = float(max_skew_s)
        self.max_future_s = float(max_future_s)
        self.max_gap_s = float(max_gap_s)
        self._samples = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self._last_source_stamp_s = None
        self._last_receipt_s = None
        self._source_rate_hz = None
        self._receipt_rate_hz = None
        self._source_gap_s = None
        self._receipt_gap_s = None
        self._source_gap_count = 0
        self._receipt_gap_count = 0
        self._invalid_samples = 0
        self._chronology_failure = None

    def _reject_chronology(self, code, message):
        self._invalid_samples += 1
        self._chronology_failure = code
        raise PointCloudCodecError(code, message)

    def add(self, message, receipt_s=None, source_now_s=None):
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
        if frame_id != self.expected_frame:
            raise PointCloudCodecError("E_FRAME_MAPPING", "unexpected IMU frame")
        stamp_s = stamp_to_seconds(message.header.stamp)
        if receipt_s is None:
            receipt_s = stamp_s
        try:
            receipt_s = float(receipt_s)
        except (TypeError, ValueError) as exc:
            raise PointCloudCodecError("E_IMU_RECEIPT", "invalid IMU receipt time") from exc
        if not math.isfinite(receipt_s):
            raise PointCloudCodecError("E_IMU_RECEIPT", "non-finite IMU receipt time")
        if source_now_s is None:
            source_now_s = receipt_s
        try:
            source_now_s = float(source_now_s)
        except (TypeError, ValueError) as exc:
            raise PointCloudCodecError("E_IMU_SOURCE_NOW", "invalid IMU source-domain now") from exc
        if not math.isfinite(source_now_s):
            raise PointCloudCodecError("E_IMU_SOURCE_NOW", "non-finite IMU source-domain now")
        orientation = (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        )
        linear_acceleration = (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        angular_velocity = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        numeric = orientation + linear_acceleration + angular_velocity
        if not all(math.isfinite(float(value)) for value in numeric):
            raise PointCloudCodecError("E_IMU_MALFORMED", "IMU contains non-finite values")
        sample = ImuSample(
            stamp_s,
            frame_id,
            tuple(float(value) for value in orientation),
            tuple(float(value) for value in linear_acceleration),
            tuple(float(value) for value in angular_velocity),
        )
        with self._lock:
            if self._last_source_stamp_s is not None and stamp_s <= self._last_source_stamp_s:
                self._reject_chronology(
                    "E_IMU_SOURCE_CHRONOLOGY",
                    "IMU source stamp must strictly increase",
                )
            if self._last_receipt_s is not None and receipt_s <= self._last_receipt_s:
                self._reject_chronology(
                    "E_IMU_RECEIPT_CHRONOLOGY",
                    "IMU receipt time must strictly increase",
                )
            if stamp_s > source_now_s + self.max_future_s:
                self._reject_chronology(
                    "E_IMU_FUTURE",
                    "IMU source stamp exceeds source-domain future bound",
                )
            if self._last_source_stamp_s is not None:
                self._source_gap_s = stamp_s - self._last_source_stamp_s
                self._source_rate_hz = 1.0 / self._source_gap_s
                if self._source_gap_s > self.max_gap_s:
                    self._source_gap_count += 1
            if self._last_receipt_s is not None:
                self._receipt_gap_s = receipt_s - self._last_receipt_s
                self._receipt_rate_hz = 1.0 / self._receipt_gap_s
                if self._receipt_gap_s > self.max_gap_s:
                    self._receipt_gap_count += 1
            self._last_source_stamp_s = stamp_s
            self._last_receipt_s = receipt_s
            self._samples.append(_CachedImu(stamp_s, sample))
        return sample

    def diagnostics(self):
        with self._lock:
            return {
                "imu_source_rate_hz": self._source_rate_hz,
                "imu_receipt_rate_hz": self._receipt_rate_hz,
                "imu_source_gap_s": self._source_gap_s,
                "imu_receipt_gap_s": self._receipt_gap_s,
                "imu_source_gap_count": self._source_gap_count,
                "imu_receipt_gap_count": self._receipt_gap_count,
                "imu_invalid_samples": self._invalid_samples,
                "imu_chronology_failure": self._chronology_failure or "",
                "imu_future_bound_s": self.max_future_s,
            }

    def aligned(self, cloud_stamp_s):
        with self._lock:
            samples = tuple(self._samples)
        if not samples:
            return None
        candidate = min(samples, key=lambda item: abs(item.stamp_s - cloud_stamp_s))
        if abs(candidate.stamp_s - cloud_stamp_s) > self.max_skew_s:
            return None
        return candidate.sample


class PerceptionNode:
    def __init__(self, rospy, pointcloud_type, imu_type, diagnostic_types):
        self._rospy = rospy
        self._DiagnosticArray, self._DiagnosticStatus, self._KeyValue = diagnostic_types
        if int(rospy.get_param("~input_queue_size", 1)) != 1:
            raise RuntimeError("input_queue_size must remain latest-only (1)")
        self._source_profile = str(rospy.get_param("~source_profile", ""))
        if self._source_profile not in ("simulation", "replay", "hardware_shadow"):
            raise RuntimeError("source_profile must select one explicit canonical source")
        self._lidar_frame = str(rospy.get_param("~lidar_frame", "lidar_link"))
        imu_frame = str(rospy.get_param("~imu_frame", "imu_link"))
        self._max_cloud_age_s = float(rospy.get_param("~max_cloud_age_s", 0.30))
        self._last_cloud_stamp_s = None
        self._rejected = 0
        mapping = rospy.get_param("~core", {})
        config = PerceptionConfig.from_mapping(mapping)
        if self._lidar_frame != config.expected_cloud_frame or imu_frame != config.expected_imu_frame:
            raise RuntimeError("adapter frames contradict the closed perception configuration")
        if self._max_cloud_age_s > config.cloud_ttl_s:
            raise RuntimeError("adapter cloud age may not exceed the closed core TTL")
        self._imu_cache = ImuCache(
            expected_frame=imu_frame,
            max_skew_s=float(rospy.get_param("~imu_alignment_tolerance_s", 0.02)),
            capacity=int(rospy.get_param("~imu_cache_size", 256)),
            max_future_s=float(rospy.get_param("~imu_max_future_s", 0.05)),
            max_gap_s=float(rospy.get_param("~imu_max_gap_s", 0.50)),
        )
        self._core = PerceptionCore(config)
        self._obstacle_pub = rospy.Publisher(
            "/perception/obstacle_cloud", pointcloud_type, queue_size=1
        )
        self._diagnostics_pub = rospy.Publisher(
            "/perception/diagnostics", self._DiagnosticArray, queue_size=1
        )
        self._imu_sub = rospy.Subscriber(
            "input_imu", imu_type, self._on_imu, queue_size=1, tcp_nodelay=True
        )
        self._cloud_sub = rospy.Subscriber(
            "input_cloud", pointcloud_type, self._on_cloud, queue_size=1, tcp_nodelay=True
        )

    def _on_imu(self, message):
        try:
            self._imu_cache.add(
                message,
                receipt_s=time.monotonic(),
                source_now_s=self._rospy.Time.now().to_sec(),
            )
        except (PointCloudCodecError, TypeError, ValueError) as exc:
            self._rejected += 1
            self._rospy.logerr_throttle(1.0, "perception rejected IMU: %s", exc)
            self._publish_diagnostics(
                getattr(getattr(message, "header", None), "stamp", self._rospy.Time.now()),
                None,
                getattr(exc, "code", "E_IMU"),
                str(exc),
            )

    def _publish_diagnostics(self, stamp, health, code=None, message=None):
        imu_diagnostics = self._imu_cache.diagnostics()
        chronology_failure = imu_diagnostics["imu_chronology_failure"]
        status = self._DiagnosticStatus()
        ok = bool(
            health is not None
            and health.ok
            and code is None
            and not chronology_failure
        )
        status.level = self._DiagnosticStatus.OK if ok else self._DiagnosticStatus.ERROR
        status.name = "wheelchair_perception/canonical_adapter"
        status.hardware_id = "navigation_perception"
        status.message = message or (
            (",".join(health.reasons) or health.code)
            if health is not None
            else code or chronology_failure or "unknown"
        )
        values = {
            "code": code
            or chronology_failure
            or (health.code if health is not None else "E_UNKNOWN"),
            "rejected_messages": self._rejected,
            "source_profile": self._source_profile,
            "command_authority": "false",
            "input_points": getattr(health, "input_points", 0),
            "obstacle_points": getattr(health, "obstacle_points", 0),
            "rate_hz": getattr(health, "observed_rate_hz", None),
            "gap_s": getattr(health, "gap_s", None),
            "stale": str(
                health is None
                or "E_CLOUD_STALE" in getattr(health, "reasons", ())
            ).lower(),
        }
        values.update(imu_diagnostics)
        status.values = [self._KeyValue(str(key), str(value)) for key, value in values.items()]
        array = self._DiagnosticArray()
        array.header.stamp = stamp
        array.status = [status]
        self._diagnostics_pub.publish(array)

    def _on_cloud(self, message):
        try:
            now_s = self._rospy.Time.now().to_sec()
            cloud = decode_pointcloud2(
                message,
                expected_frame=self._lidar_frame,
                previous_stamp_s=self._last_cloud_stamp_s,
                now_s=now_s,
                max_age_s=self._max_cloud_age_s,
            )
            result = self._core.process(
                cloud, self._imu_cache.aligned(cloud.stamp_s), now_s=now_s
            )
            obstacle_points = tuple(
                sorted(
                    result.obstacle_points,
                    key=lambda point: (
                        point.offset_time,
                        point.x,
                        point.y,
                        point.z,
                        point.line,
                        point.tag,
                        point.reflectivity,
                        point.lidar_id,
                    ),
                )
            )
            output = encode_pointcloud2(message, obstacle_points)
            self._last_cloud_stamp_s = cloud.stamp_s
            self._obstacle_pub.publish(output)
            self._publish_diagnostics(message.header.stamp, result.health)
        except (PointCloudCodecError, TypeError, ValueError) as exc:
            self._rejected += 1
            code = getattr(exc, "code", "E_PERCEPTION")
            self._rospy.logerr_throttle(1.0, "perception rejected cloud: %s", exc)
            self._publish_diagnostics(message.header.stamp, None, code, str(exc))


def main():
    # ROS imports stay behind the executable boundary so codec/core tests are ROS-free.
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from sensor_msgs.msg import Imu, PointCloud2

    rospy.init_node("perception_node", anonymous=False)
    PerceptionNode(
        rospy,
        PointCloud2,
        Imu,
        (DiagnosticArray, DiagnosticStatus, KeyValue),
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
