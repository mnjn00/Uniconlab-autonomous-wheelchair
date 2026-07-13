"""ROS-free ABI and corruption tests for the canonical PointCloud2 codec."""

import importlib.util
from pathlib import Path
import struct
import sys
import types
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "perception_node.py"
SCRIPT_DIR = str(SCRIPT.parent)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
SPEC = importlib.util.spec_from_file_location("perception_node", SCRIPT)
codec = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = codec
SPEC.loader.exec_module(codec)


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class Field:
    def __init__(self, name, offset, datatype, count):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


class Cloud:
    pass


class Imu:
    pass


INTERLEAVED_RECORDS = (
    (1.0, -2.0, 0.5, 17.0, 1_250_000, 0, 4, 17, 2),
    (2.0, 3.0, -0.5, 201.0, 340_000, 1, 7, 201, 2),
    (3.0, -1.0, 0.25, 33.0, 1_260_000, 0, 5, 33, 2),
    (4.0, 2.0, -0.25, 99.0, 350_000, 1, 6, 99, 2),
)


def canonical_cloud(stamp=10.0, frame="lidar_link", bigendian=False,
                    records=INTERLEAVED_RECORDS):
    message = Cloud()
    message.header = types.SimpleNamespace(stamp=Stamp(stamp), frame_id=frame)
    message.height = 1
    message.width = len(records)
    message.fields = [Field(*signature) for signature in codec.CANONICAL_FIELDS]
    message.is_bigendian = bigendian
    message.point_step = 24
    message.row_step = 24 * len(records)
    message.is_dense = True
    message.data = b"".join(struct.pack("<ffffIBBBB", *record) for record in records)
    return message


class PointCloudCodecTests(unittest.TestCase):
    def test_exact_offsets_types_and_round_trip(self):
        message = canonical_cloud()
        decoded = codec.decode_pointcloud2(message)
        self.assertEqual(
            codec.CANONICAL_FIELDS,
            (
                ("x", 0, 7, 1),
                ("y", 4, 7, 1),
                ("z", 8, 7, 1),
                ("intensity", 12, 7, 1),
                ("offset_time", 16, 6, 1),
                ("line", 20, 2, 1),
                ("tag", 21, 2, 1),
                ("reflectivity", 22, 2, 1),
                ("lidar_id", 23, 2, 1),
            ),
        )
        self.assertEqual(decoded.stamp_s, 10.0)
        self.assertEqual(decoded.frame_id, "lidar_link")
        self.assertEqual(
            [1_250_000, 340_000, 1_260_000, 350_000],
            [point.offset_time for point in decoded.points],
        )
        self.assertEqual([1.0, 2.0, 3.0, 4.0], [point.x for point in decoded.points])
        self.assertEqual(decoded.points[0].reflectivity, 17)
        self.assertEqual(decoded.points[0].tag, 4)
        self.assertEqual(decoded.points[0].line, 0)
        self.assertEqual(decoded.points[1].lidar_id, 2)
        self.assertEqual([0, 1, 0, 1], [point.line for point in decoded.points])
        encoded = codec.encode_pointcloud2(message, decoded.points)
        self.assertEqual(encoded.data, message.data)
        self.assertEqual(encoded.point_step, 24)
        self.assertEqual(encoded.row_step, len(message.data))

    def test_layout_corruption_is_rejected_instead_of_guessed(self):
        mutations = (
            ("point_step", 20),
            ("row_step", 47),
            ("height", 2),
            ("is_dense", False),
        )
        for attribute, value in mutations:
            message = canonical_cloud()
            setattr(message, attribute, value)
            with self.subTest(attribute=attribute), self.assertRaises(codec.PointCloudCodecError):
                codec.decode_pointcloud2(message)
        message = canonical_cloud()
        message.fields[4].datatype = 7
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_POINT_LAYOUT"):
            codec.decode_pointcloud2(message)
        message = canonical_cloud()
        message.data = message.data[:-1]
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_POINT_LAYOUT"):
            codec.decode_pointcloud2(message)

    def test_big_endian_and_wrong_frame_are_rejected(self):
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_ENDIAN"):
            codec.decode_pointcloud2(canonical_cloud(bigendian=True))
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_FRAME_MAPPING"):
            codec.decode_pointcloud2(canonical_cloud(frame="base_link"))

    def test_intensity_corruption_is_rejected(self):
        message = canonical_cloud()
        data = bytearray(message.data)
        struct.pack_into("<f", data, 12, 18.0)
        message.data = bytes(data)
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_POINT_LAYOUT"):
            codec.decode_pointcloud2(message)

    def test_min_max_offsets_preserve_order_and_invalid_times_fail_closed(self):
        offsets = (0xFFFFFFFF, 0, 0xFFFFFFFF, 1)
        points = tuple(
            codec.Point(float(index), 0.0, 0.0, offset, 20, 0, index, 1)
            for index, offset in enumerate(offsets)
        )
        encoded = codec.encode_pointcloud2(canonical_cloud(), points)
        decoded = codec.decode_pointcloud2(encoded)
        self.assertEqual(offsets, tuple(point.offset_time for point in decoded.points))

        for offset in (-1, 1 << 32):
            invalid = (codec.Point(0.0, 0.0, 0.0, offset, 20, 0, 0, 1),)
            with self.subTest(offset=offset), self.assertRaisesRegex(
                codec.PointCloudCodecError, "E_POINT_TIME_OVERFLOW"
            ):
                codec.encode_pointcloud2(canonical_cloud(), invalid)
        noninteger = (codec.Point(0.0, 0.0, 0.0, 1.5, 20, 0, 0, 1),)
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_POINT_LAYOUT"):
            codec.encode_pointcloud2(canonical_cloud(), noninteger)

        near_limit = (codec._INT64_MAX - 1_000_000) / 1.0e9
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_POINT_TIME_OVERFLOW"):
            codec.decode_pointcloud2(canonical_cloud(stamp=near_limit))
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_POINT_TIME_OVERFLOW"):
            codec.decode_pointcloud2(canonical_cloud(stamp=1.0e10))

    def test_source_stamp_regression_staleness_and_future_time(self):
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_SOURCE_TIME_REGRESSION"):
            codec.decode_pointcloud2(canonical_cloud(9.0), previous_stamp_s=10.0)
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_SENSOR_STALE"):
            codec.decode_pointcloud2(canonical_cloud(10.0), now_s=10.31, max_age_s=0.30)
        with self.assertRaisesRegex(codec.PointCloudCodecError, "E_CLOCK_FUTURE"):
            codec.decode_pointcloud2(canonical_cloud(10.1), now_s=10.0, max_future_s=0.05)

    def test_imu_cache_aligns_by_monotonic_source_stamp_and_rejects_regression(self):
        cache = codec.ImuCache(max_skew_s=0.02, capacity=3)
        for stamp in (9.99, 10.03):
            message = Imu()
            message.header = types.SimpleNamespace(stamp=Stamp(stamp), frame_id="imu_link")
            message.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            message.linear_acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=9.81)
            message.angular_velocity = types.SimpleNamespace(x=0.1, y=-0.2, z=0.3)
            cache.add(message)
        self.assertEqual(cache.aligned(10.0).stamp_s, 9.99)
        self.assertIsNone(cache.aligned(10.2))

        regressing = Imu()
        regressing.header = types.SimpleNamespace(stamp=Stamp(9.98), frame_id="imu_link")
        regressing.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        regressing.linear_acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=9.81)
        regressing.angular_velocity = types.SimpleNamespace(x=0.1, y=-0.2, z=0.3)
        with self.assertRaisesRegex(
            codec.PointCloudCodecError, "E_IMU_SOURCE_CHRONOLOGY"
        ):
            cache.add(regressing)


if __name__ == "__main__":
    unittest.main()
