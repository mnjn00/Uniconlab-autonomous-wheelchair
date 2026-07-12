#!/usr/bin/env python3
"""Simulation-only Gazebo sensor adapter for the canonical perception ABI."""

from __future__ import annotations

import copy
import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, Optional, Tuple

RAW_LIDAR_TOPIC = "/simulation/sensors/lidar/raw"
RAW_IMU_TOPIC = "/simulation/sensors/imu/raw"
CANONICAL_LIDAR_TOPIC = "/sensors/lidar/points"
CANONICAL_IMU_TOPIC = "/sensors/imu/data"
DIAGNOSTIC_TOPIC = "/diagnostics"
LIDAR_FRAME = "lidar_link"
IMU_FRAME = "imu_link"
FLOAT32 = 7
UINT32 = 6
UINT8 = 2
POINT_STEP = 24
MAX_POINT_COUNT = 1_000_000
MAX_CLOUD_DATA_BYTES = MAX_POINT_COUNT * POINT_STEP
LEGACY_POINT_BYTES = 12
LEGACY_CHANNEL_VALUE_BYTES = 4
CANONICAL_FIELDS = (
    ("x", 0, FLOAT32, 1),
    ("y", 4, FLOAT32, 1),
    ("z", 8, FLOAT32, 1),
    ("intensity", 12, FLOAT32, 1),
    ("offset_time", 16, UINT32, 1),
    ("line", 20, UINT8, 1),
    ("tag", 21, UINT8, 1),
    ("reflectivity", 22, UINT8, 1),
    ("lidar_id", 23, UINT8, 1),
)
_POINT = struct.Struct("<ffffIBBBB")
_FLOAT = struct.Struct("<f")


class SensorValidationError(ValueError):
    """A whole sensor message failed the simulation boundary contract."""

    def __init__(self, code: str, detail: str) -> None:
        super().__init__("{}: {}".format(code, detail))
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class CanonicalCloud:
    """ROS-independent encoded PointCloud2 payload and metadata."""

    stamp_ns: int
    frame_id: str
    height: int
    width: int
    fields: Tuple[Tuple[str, int, int, int], ...]
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool


def _stamp_ns(message: Any) -> int:
    try:
        stamp = message.header.stamp
        secs = stamp.secs
        nsecs = stamp.nsecs
    except AttributeError as error:
        raise SensorValidationError("E_STAMP", "missing ROS header stamp") from error
    if (
        isinstance(secs, bool)
        or isinstance(nsecs, bool)
        or not isinstance(secs, int)
        or not isinstance(nsecs, int)
        or secs < 0
        or not 0 <= nsecs < 1_000_000_000
    ):
        raise SensorValidationError("E_STAMP", "stamp must be normalized and non-negative")
    return secs * 1_000_000_000 + nsecs


def _frame(message: Any, expected: str) -> str:
    try:
        frame_id = message.header.frame_id
    except AttributeError as error:
        raise SensorValidationError("E_FRAME", "missing ROS header frame") from error
    if frame_id != expected:
        raise SensorValidationError("E_FRAME", "frame must be exactly {}".format(expected))
    return frame_id


def _finite(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def bounded_reflectivity(raw_intensity: Any) -> int:
    """Convert finite Gazebo intensity by truncation and uint8 saturation."""
    if not _finite(raw_intensity):
        raise SensorValidationError("E_NONFINITE", "intensity is not finite")
    return min(255, max(0, int(raw_intensity)))


def _raw_field_offsets(message: Any) -> Dict[str, int]:
    try:
        fields = tuple(message.fields)
        point_step = message.point_step
    except (AttributeError, TypeError) as error:
        raise SensorValidationError("E_LAYOUT", "missing PointCloud2 layout") from error
    if isinstance(point_step, bool) or not isinstance(point_step, int) or point_step <= 0:
        raise SensorValidationError("E_LAYOUT", "point_step must be a positive integer")
    offsets: Dict[str, int] = {}
    occupied = set()
    for field in fields:
        try:
            name, offset, datatype, count = field.name, field.offset, field.datatype, field.count
        except AttributeError as error:
            raise SensorValidationError("E_LAYOUT", "malformed PointField") from error
        if name not in ("x", "y", "z", "intensity") or name in offsets:
            raise SensorValidationError("E_LAYOUT", "raw fields must be unique x/y/z[/intensity]")
        if (
            datatype != FLOAT32
            or count != 1
            or isinstance(offset, bool)
            or not isinstance(offset, int)
            or offset < 0
            or offset + _FLOAT.size > point_step
        ):
            raise SensorValidationError("E_LAYOUT", "raw fields must be in-bounds float32 scalars")
        field_bytes = set(range(offset, offset + _FLOAT.size))
        if occupied.intersection(field_bytes):
            raise SensorValidationError("E_LAYOUT", "raw fields overlap")
        occupied.update(field_bytes)
        offsets[name] = offset
    if set(offsets) not in ({"x", "y", "z"}, {"x", "y", "z", "intensity"}):
        raise SensorValidationError("E_LAYOUT", "raw cloud requires x, y, and z")
    return offsets


def canonicalize_legacy_pointcloud(message: Any) -> CanonicalCloud:
    """Validate and encode one legacy sensor_msgs/PointCloud as PointCloud2."""
    stamp_ns = _stamp_ns(message)
    frame_id = _frame(message, LIDAR_FRAME)
    try:
        points = message.points
        channels = message.channels
        point_count = len(points)
        channel_count = len(channels)
    except (AttributeError, TypeError) as error:
        raise SensorValidationError("E_LAYOUT", "missing PointCloud points or channels") from error
    legacy_data_bytes = point_count * (
        LEGACY_POINT_BYTES + channel_count * LEGACY_CHANNEL_VALUE_BYTES
    )
    if (
        isinstance(point_count, bool)
        or point_count <= 0
        or point_count > MAX_POINT_COUNT
        or legacy_data_bytes > MAX_CLOUD_DATA_BYTES
    ):
        raise SensorValidationError("E_SIZE", "legacy cloud point or channel data is oversized")

    intensity_values = None
    for channel_index in range(channel_count):
        try:
            channel = channels[channel_index]
            name = channel.name
            values = channel.values
            value_count = len(values)
        except (AttributeError, IndexError, TypeError) as error:
            raise SensorValidationError("E_CHANNEL", "malformed PointCloud channel") from error
        if not isinstance(name, str) or value_count != point_count:
            raise SensorValidationError("E_CHANNEL", "every channel must have one value per point")
        for value_index in range(value_count):
            try:
                value = values[value_index]
            except (IndexError, TypeError) as error:
                raise SensorValidationError("E_CHANNEL", "malformed PointCloud channel values") from error
            if not _finite(value):
                raise SensorValidationError("E_NONFINITE", "channel contains a non-finite value")
        if name in ("intensity", "intensities"):
            if intensity_values is not None:
                raise SensorValidationError(
                    "E_CHANNEL", "intensity channel must be unique and unambiguous"
                )
            intensity_values = values

    encoded = bytearray()
    for index in range(point_count):
        try:
            point = points[index]
            x, y, z = point.x, point.y, point.z
        except (AttributeError, IndexError, TypeError) as error:
            raise SensorValidationError("E_LAYOUT", "malformed Point32 entry") from error
        if not all(_finite(value) for value in (x, y, z)):
            raise SensorValidationError("E_NONFINITE", "XYZ contains a non-finite value")
        reflectivity = bounded_reflectivity(intensity_values[index]) if intensity_values is not None else 0
        try:
            encoded.extend(
                _POINT.pack(
                    x,
                    y,
                    z,
                    float(reflectivity),
                    0,  # Gazebo provides no per-point acquisition timing evidence.
                    0,  # Generic ray grid provides no Livox line evidence.
                    0,  # Generic ray grid provides no Livox return tag evidence.
                    reflectivity,
                    0,  # Simulation has no physical Livox device identity.
                )
            )
        except (OverflowError, struct.error) as error:
            raise SensorValidationError("E_LAYOUT", "XYZ is outside the Point32 range") from error

    return CanonicalCloud(
        stamp_ns,
        frame_id,
        1,
        point_count,
        CANONICAL_FIELDS,
        False,
        POINT_STEP,
        POINT_STEP * point_count,
        bytes(encoded),
        True,
    )


def canonicalize_pointcloud(message: Any) -> CanonicalCloud:
    """Validate and encode one complete Gazebo cloud without reordering points."""
    stamp_ns = _stamp_ns(message)
    frame_id = _frame(message, LIDAR_FRAME)
    offsets = _raw_field_offsets(message)
    try:
        height, width = message.height, message.width
        point_step, row_step = message.point_step, message.row_step
        bigendian, dense = message.is_bigendian, message.is_dense
        raw_data = message.data
        raw_length = len(raw_data)
    except (AttributeError, TypeError) as error:
        raise SensorValidationError("E_LAYOUT", "incomplete PointCloud2 dimensions or data") from error
    if raw_length > MAX_CLOUD_DATA_BYTES:
        raise SensorValidationError("E_LAYOUT", "PointCloud2 data is oversized")
    try:
        raw = bytes(raw_data)
    except (TypeError, ValueError) as error:
        raise SensorValidationError("E_LAYOUT", "PointCloud2 data is not byte-compatible") from error
    if (
        isinstance(height, bool)
        or isinstance(width, bool)
        or not isinstance(height, int)
        or not isinstance(width, int)
        or height <= 0
        or width <= 0
    ):
        raise SensorValidationError("E_LAYOUT", "dimensions must be positive integers")
    count = height * width
    if (
        count > MAX_POINT_COUNT
        or len(raw) > MAX_CLOUD_DATA_BYTES
        or bigendian is not False
        or dense is not True
        or row_step != point_step * width
        or raw_length != row_step * height
    ):
        raise SensorValidationError(
            "E_LAYOUT", "invalid or oversized dimensions, byte order, density, or data length"
        )

    encoded = bytearray()
    for row in range(height):
        for column in range(width):
            base = row * row_step + column * point_step
            x = _FLOAT.unpack_from(raw, base + offsets["x"])[0]
            y = _FLOAT.unpack_from(raw, base + offsets["y"])[0]
            z = _FLOAT.unpack_from(raw, base + offsets["z"])[0]
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise SensorValidationError("E_NONFINITE", "XYZ contains a non-finite value")
            reflectivity = (
                bounded_reflectivity(_FLOAT.unpack_from(raw, base + offsets["intensity"])[0])
                if "intensity" in offsets
                else 0
            )
            encoded.extend(
                _POINT.pack(
                    x,
                    y,
                    z,
                    float(reflectivity),
                    0,  # Gazebo provides no per-point acquisition timing evidence.
                    0,  # Generic ray grid provides no Livox line evidence.
                    0,  # Generic ray grid provides no Livox return tag evidence.
                    reflectivity,
                    0,  # Simulation has no physical Livox device identity.
                )
            )
    return CanonicalCloud(
        stamp_ns,
        frame_id,
        1,
        count,
        CANONICAL_FIELDS,
        False,
        POINT_STEP,
        POINT_STEP * count,
        bytes(encoded),
        True,
    )


def validate_imu(message: Any) -> Any:
    """Validate an IMU as a whole and return the unchanged message."""
    _stamp_ns(message)
    _frame(message, IMU_FRAME)
    try:
        quaternion = tuple(getattr(message.orientation, axis) for axis in "xyzw")
        vectors = tuple(
            getattr(vector, axis)
            for vector in (message.angular_velocity, message.linear_acceleration)
            for axis in "xyz"
        )
        covariances = (
            tuple(message.orientation_covariance),
            tuple(message.angular_velocity_covariance),
            tuple(message.linear_acceleration_covariance),
        )
    except (AttributeError, TypeError) as error:
        raise SensorValidationError("E_IMU", "missing IMU components") from error
    if not all(_finite(value) for value in quaternion + vectors):
        raise SensorValidationError("E_NONFINITE", "IMU quaternion or vector is not finite")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-12 or abs(norm - 1.0) > 1e-3:
        raise SensorValidationError("E_ORIENTATION", "orientation must be a unit quaternion")
    if any(len(covariance) != 9 for covariance in covariances) or not all(
        _finite(value) for covariance in covariances for value in covariance
    ):
        raise SensorValidationError("E_COVARIANCE", "IMU covariances must contain nine finite values")
    return message


class SensorCanonicalizerCore:
    """Pure validation/codec boundary with simulation-only IMU anti-alias state."""

    def __init__(self, imu_window_samples: int = 20, imu_max_gap_ns: int = 10_000_000) -> None:
        if (
            isinstance(imu_window_samples, bool)
            or not isinstance(imu_window_samples, int)
            or imu_window_samples <= 0
            or isinstance(imu_max_gap_ns, bool)
            or not isinstance(imu_max_gap_ns, int)
            or imu_max_gap_ns <= 0
        ):
            raise ValueError("IMU window and maximum gap must be positive integers")
        self._lock = threading.Lock()
        self._last = {"lidar": (None, None), "imu": (None, None)}
        self._imu_window_samples = imu_window_samples
        self._imu_max_gap_ns = imu_max_gap_ns
        self._imu_window: Deque[Any] = deque(maxlen=imu_window_samples)
        self._imu_gap_count = 0
        self._imu_chronology_failure = ""
        self._imu_chronology_failure_count = 0
        self._imu_recovery_pending = False

    def _accept_times(self, stream: str, source_ns: int, receipt_ns: int) -> None:
        if isinstance(receipt_ns, bool) or not isinstance(receipt_ns, int) or receipt_ns < 0:
            raise SensorValidationError("E_RECEIPT_TIME", "receipt time must be non-negative integer ns")
        previous_source, previous_receipt = self._last[stream]
        if previous_source is not None and source_ns < previous_source:
            raise SensorValidationError("E_STAMP_REGRESSION", "source stamp regressed")
        if previous_receipt is not None and receipt_ns < previous_receipt:
            raise SensorValidationError("E_RECEIPT_REGRESSION", "receipt clock regressed")
        self._last[stream] = (source_ns, receipt_ns)

    def adapt_cloud(self, message: Any, receipt_ns: int) -> CanonicalCloud:
        """Adapt the retained PointCloud2 core path used by non-live callers."""
        cloud = canonicalize_pointcloud(message)
        with self._lock:
            self._accept_times("lidar", cloud.stamp_ns, receipt_ns)
        return cloud

    def adapt_legacy_cloud(self, message: Any, receipt_ns: int) -> CanonicalCloud:
        """Adapt the sensor_msgs/PointCloud emitted by Gazebo Classic."""
        cloud = canonicalize_legacy_pointcloud(message)
        with self._lock:
            self._accept_times("lidar", cloud.stamp_ns, receipt_ns)
        return cloud

    def _reset_imu(self) -> None:
        """Discard partial averaging evidence without lowering chronology high-water marks."""
        self._imu_window.clear()

    def _record_imu_failure(self, code: str) -> None:
        self._imu_chronology_failure = code
        self._imu_chronology_failure_count += 1
        self._imu_recovery_pending = True
        self._reset_imu()

    def imu_diagnostics(self) -> Dict[str, Any]:
        """Return bounded, sticky IMU chronology evidence for the ROS wrapper."""
        with self._lock:
            source_ns, receipt_ns = self._last["imu"]
            return {
                "imu_source_high_water_ns": source_ns,
                "imu_receipt_high_water_ns": receipt_ns,
                "imu_gap_count": self._imu_gap_count,
                "imu_chronology_failure": self._imu_chronology_failure,
                "imu_chronology_failure_count": self._imu_chronology_failure_count,
                "imu_recovery_pending": self._imu_recovery_pending,
                "imu_window_samples": len(self._imu_window),
                "imu_window_required": self._imu_window_samples,
            }

    def adapt_imu(self, message: Any, receipt_ns: int) -> Optional[Any]:
        """Average only a complete consecutive window after any chronology failure or gap."""
        try:
            validated = validate_imu(message)
            source_ns = _stamp_ns(validated)
        except SensorValidationError:
            with self._lock:
                self._reset_imu()
            raise
        if isinstance(receipt_ns, bool) or not isinstance(receipt_ns, int) or receipt_ns < 0:
            with self._lock:
                self._record_imu_failure("E_RECEIPT_TIME")
            raise SensorValidationError(
                "E_RECEIPT_TIME", "receipt time must be non-negative integer ns"
            )
        with self._lock:
            previous_source, previous_receipt = self._last["imu"]
            if previous_source is not None and source_ns <= previous_source:
                self._record_imu_failure("E_STAMP_REGRESSION")
                raise SensorValidationError(
                    "E_STAMP_REGRESSION",
                    "IMU source stamp must strictly increase; chronology high-water retained",
                )
            if previous_receipt is not None and receipt_ns <= previous_receipt:
                self._record_imu_failure("E_RECEIPT_REGRESSION")
                raise SensorValidationError(
                    "E_RECEIPT_REGRESSION",
                    "IMU receipt clock must strictly increase; chronology high-water retained",
                )
            gap = (
                previous_source is not None
                and source_ns - previous_source > self._imu_max_gap_ns
            ) or (
                previous_receipt is not None
                and receipt_ns - previous_receipt > self._imu_max_gap_ns
            )
            if gap:
                self._imu_gap_count += 1
                self._imu_recovery_pending = True
                self._reset_imu()
            self._last["imu"] = (source_ns, receipt_ns)
            self._imu_window.append(validated)
            if len(self._imu_window) < self._imu_window_samples:
                return None
            output = copy.deepcopy(validated)
            scale = 1.0 / self._imu_window_samples
            for field in ("angular_velocity", "linear_acceleration"):
                target = getattr(output, field)
                for axis in "xyz":
                    setattr(
                        target,
                        axis,
                        sum(getattr(getattr(sample, field), axis) for sample in self._imu_window)
                        * scale,
                    )
            self._imu_recovery_pending = False
            return output


class SimSensorCanonicalizer:
    """Thin ROS boundary; it creates no surface until all simulation gates pass."""

    def __init__(self) -> None:
        import rospy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from sensor_msgs.msg import Imu, PointCloud, PointCloud2, PointField

        self.rospy = rospy
        self.DiagnosticArray = DiagnosticArray
        self.DiagnosticStatus = DiagnosticStatus
        self.KeyValue = KeyValue
        self.PointCloud2 = PointCloud2
        self.PointField = PointField
        self.core = SensorCanonicalizerCore(
            rospy.get_param("~imu_filter_window_samples"),
            rospy.get_param("~imu_filter_max_gap_ns"),
        )

        required = (
            ("/simulation_only", True),
            ("/use_sim_time", True),
            ("/hardware_motion_authorized", False),
            ("/passenger_operation_authorized", False),
        )
        for parameter, expected in required:
            actual = rospy.get_param(parameter, None)
            if actual is not expected:
                raise RuntimeError("{} must be exactly {!r}".format(parameter, expected))

        self.lidar_publisher = rospy.Publisher(CANONICAL_LIDAR_TOPIC, PointCloud2, queue_size=1)
        self.imu_publisher = rospy.Publisher(CANONICAL_IMU_TOPIC, Imu, queue_size=1)
        self.diagnostic_publisher = rospy.Publisher(DIAGNOSTIC_TOPIC, DiagnosticArray, queue_size=1)
        self.lidar_subscriber = rospy.Subscriber(
            RAW_LIDAR_TOPIC, PointCloud, self._lidar_callback, queue_size=1, tcp_nodelay=True
        )
        self.imu_subscriber = rospy.Subscriber(
            RAW_IMU_TOPIC, Imu, self._imu_callback, queue_size=1, tcp_nodelay=True
        )

    def _diagnostic(self, sensor: str, level: int, code: str, detail: str) -> None:
        status = self.DiagnosticStatus()
        status.level = level
        status.name = "wheelchair_gazebo/sim_sensor_canonicalizer/{}".format(sensor)
        status.hardware_id = "simulation"
        status.message = code
        values = [self.KeyValue(key="detail", value=detail)]
        if sensor == "imu":
            values.extend(
                self.KeyValue(key=str(key), value=str(value))
                for key, value in self.core.imu_diagnostics().items()
            )
        status.values = values
        array = self.DiagnosticArray()
        array.header.stamp = self.rospy.get_rostime()
        array.status = [status]
        self.diagnostic_publisher.publish(array)

    def _lidar_callback(self, message: Any) -> None:
        try:
            cloud = self.core.adapt_legacy_cloud(message, time.monotonic_ns())
            output = self.PointCloud2()
            output.header = message.header
            output.height = cloud.height
            output.width = cloud.width
            output.fields = [
                self.PointField(name=name, offset=offset, datatype=datatype, count=count)
                for name, offset, datatype, count in cloud.fields
            ]
            output.is_bigendian = cloud.is_bigendian
            output.point_step = cloud.point_step
            output.row_step = cloud.row_step
            output.data = cloud.data
            output.is_dense = cloud.is_dense
            self.lidar_publisher.publish(output)
            self._diagnostic("lidar", self.DiagnosticStatus.OK, "OK", "canonical cloud published")
        except SensorValidationError as error:
            self._diagnostic("lidar", self.DiagnosticStatus.ERROR, error.code, error.detail)

    def _imu_callback(self, message: Any) -> None:
        try:
            output = self.core.adapt_imu(message, time.monotonic_ns())
            if output is not None:
                self.imu_publisher.publish(output)
                self._diagnostic(
                    "imu", self.DiagnosticStatus.OK, "OK", "simulation-filtered IMU published"
                )
        except SensorValidationError as error:
            self._diagnostic("imu", self.DiagnosticStatus.ERROR, error.code, error.detail)


def main() -> None:
    import rospy

    rospy.init_node("sim_sensor_canonicalizer")
    SimSensorCanonicalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
