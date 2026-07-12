#!/usr/bin/env python3
"""Pure codec and static surface tests for the simulation sensor adapter."""

import importlib.util
import math
import pathlib
import struct
import sys
import unittest
from types import SimpleNamespace as NS


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_SRC = PACKAGE.parent
SCRIPT = PACKAGE / "scripts" / "sim_sensor_canonicalizer.py"
LAUNCH = PACKAGE / "launch" / "rc_sim.launch"
XACRO = REPOSITORY_SRC / "wheelchair_description" / "urdf" / "wheelchair.urdf.xacro"


def load_adapter():
    spec = importlib.util.spec_from_file_location("sim_sensor_canonicalizer_under_test", str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


adapter = load_adapter()


def stamp(ns=10_000_000_123):
    return NS(secs=ns // 1_000_000_000, nsecs=ns % 1_000_000_000)


def field(name, offset):
    return NS(name=name, offset=offset, datatype=adapter.FLOAT32, count=1)


def cloud(records=None, *, intensity=True, source_ns=10_000_000_123, frame="lidar_link"):
    if records is None:
        records = [(1.0, 2.0, 3.0, 17.9), (-4.0, 5.5, -6.25, 300.0)]
    fmt = "<ffff" if intensity else "<fff"
    names = (("x", 0), ("y", 4), ("z", 8), ("intensity", 12)) if intensity else (
        ("x", 0), ("y", 4), ("z", 8)
    )
    data = b"".join(struct.pack(fmt, *(record if intensity else record[:3])) for record in records)
    step = struct.calcsize(fmt)
    return NS(
        header=NS(stamp=stamp(source_ns), frame_id=frame),
        height=1,
        width=len(records),
        fields=[field(name, offset) for name, offset in names],
        is_bigendian=False,
        point_step=step,
        row_step=step * len(records),
        data=data,
        is_dense=True,
    )


def legacy_cloud(
    records=None, *, channel_name="intensity", channel_values=None, source_ns=10_000_000_123,
    frame="lidar_link"
):
    if records is None:
        records = [(1.0, 2.0, 3.0), (-4.0, 5.5, -6.25)]
    if channel_values is None:
        channel_values = [17.9, 300.0]
    channels = (
        [] if channel_name is None else [NS(name=channel_name, values=list(channel_values))]
    )
    return NS(
        header=NS(stamp=stamp(source_ns), frame_id=frame),
        points=[NS(x=x, y=y, z=z) for x, y, z in records],
        channels=channels,
    )


def imu(
    *,
    source_ns=10_000_000_123,
    frame="imu_link",
    linear=(0.0, 0.0, 9.81),
    angular=(0.1, -0.2, 0.3),
):
    return NS(
        header=NS(stamp=stamp(source_ns), frame_id=frame),
        orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=[1.0] * 9,
        angular_velocity=NS(x=angular[0], y=angular[1], z=angular[2]),
        angular_velocity_covariance=[2.0] * 9,
        linear_acceleration=NS(x=linear[0], y=linear[1], z=linear[2]),
        linear_acceleration_covariance=[3.0] * 9,
    )


class PointCloudCodecTest(unittest.TestCase):
    def test_exact_canonical_fields_offsets_dimensions_and_bytes(self):
        result = adapter.canonicalize_pointcloud(cloud())
        self.assertEqual(
            result.fields,
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
        self.assertEqual((result.height, result.width), (1, 2))
        self.assertEqual((result.point_step, result.row_step), (24, 48))
        self.assertFalse(result.is_bigendian)
        self.assertTrue(result.is_dense)
        expected = b"".join(
            (
                struct.pack("<ffffIBBBB", 1.0, 2.0, 3.0, 17.0, 0, 0, 0, 17, 0),
                struct.pack("<ffffIBBBB", -4.0, 5.5, -6.25, 255.0, 0, 0, 0, 255, 0),
            )
        )
        self.assertEqual(result.data, expected)

    def test_source_order_and_xyz_float32_values_are_preserved(self):
        records = [(9.0, 8.0, 7.0, 1.0), (-0.0, -2.5, 4.25, 2.0), (3.0, 2.0, 1.0, 3.0)]
        result = adapter.canonicalize_pointcloud(cloud(records))
        unpacked = list(struct.Struct("<ffffIBBBB").iter_unpack(result.data))
        self.assertEqual([(p[0], p[1], p[2]) for p in unpacked], [r[:3] for r in records])
        self.assertEqual([p[4] for p in unpacked], [0, 0, 0])
        self.assertEqual([p[5:7] + p[8:] for p in unpacked], [(0, 0, 0)] * 3)

    def test_intensity_uses_truncation_and_saturation_for_both_abi_values(self):
        records = [(0.0, 0.0, 0.0, value) for value in (-20.0, 0.9, 127.9, 255.9, 999.0)]
        result = adapter.canonicalize_pointcloud(cloud(records))
        unpacked = list(struct.Struct("<ffffIBBBB").iter_unpack(result.data))
        self.assertEqual([point[3] for point in unpacked], [0.0, 0.0, 127.0, 255.0, 255.0])
        self.assertEqual([point[7] for point in unpacked], [0, 0, 127, 255, 255])
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value), self.assertRaises(adapter.SensorValidationError):
                adapter.canonicalize_pointcloud(cloud([(0.0, 0.0, 0.0, value)]))

    def test_missing_optional_intensity_is_zero_without_invented_evidence(self):
        result = adapter.canonicalize_pointcloud(cloud([(1.0, 2.0, 3.0, 99.0)], intensity=False))
        point = struct.unpack("<ffffIBBBB", result.data)
        self.assertEqual(point, (1.0, 2.0, 3.0, 0.0, 0, 0, 0, 0, 0))

    def test_nonfinite_xyz_rejects_the_whole_cloud(self):
        for index in range(3):
            record = [1.0, 2.0, 3.0, 10.0]
            record[index] = math.nan
            with self.subTest(index=index), self.assertRaises(adapter.SensorValidationError):
                adapter.canonicalize_pointcloud(cloud([tuple(record)]))

    def test_malformed_layout_dimensions_frame_and_stamp_are_rejected(self):
        mutations = []
        bad = cloud()
        bad.row_step += 1
        mutations.append(bad)
        bad = cloud()
        bad.data = bad.data[:-1]
        mutations.append(bad)
        bad = cloud()
        bad.width = 0
        mutations.append(bad)
        bad = cloud()
        bad.is_bigendian = True
        mutations.append(bad)
        bad = cloud()
        bad.is_dense = False
        mutations.append(bad)
        bad = cloud(frame="base_link")
        mutations.append(bad)
        bad = cloud()
        bad.header.stamp.nsecs = 1_000_000_000
        mutations.append(bad)
        bad = cloud()
        bad.fields.append(field("ring", 0))
        mutations.append(bad)
        for index, malformed in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(adapter.SensorValidationError):
                adapter.canonicalize_pointcloud(malformed)

    def test_source_and_receipt_regressions_are_rejected_per_stream(self):
        core = adapter.SensorCanonicalizerCore()
        core.adapt_cloud(cloud(source_ns=100), 1000)
        core.adapt_cloud(cloud(source_ns=100), 1000)
        with self.assertRaises(adapter.SensorValidationError):
            core.adapt_cloud(cloud(source_ns=99), 1001)
        with self.assertRaises(adapter.SensorValidationError):
            core.adapt_cloud(cloud(source_ns=101), 999)


class LegacyPointCloudCodecTest(unittest.TestCase):
    def test_exact_canonical_fields_metadata_and_bytes(self):
        result = adapter.canonicalize_legacy_pointcloud(legacy_cloud())
        self.assertEqual(result.stamp_ns, 10_000_000_123)
        self.assertEqual(result.frame_id, "lidar_link")
        self.assertEqual(result.fields, adapter.CANONICAL_FIELDS)
        self.assertEqual((result.height, result.width), (1, 2))
        self.assertEqual((result.point_step, result.row_step), (24, 48))
        self.assertFalse(result.is_bigendian)
        self.assertTrue(result.is_dense)
        self.assertEqual(
            result.data,
            b"".join(
                (
                    struct.pack("<ffffIBBBB", 1.0, 2.0, 3.0, 17.0, 0, 0, 0, 17, 0),
                    struct.pack("<ffffIBBBB", -4.0, 5.5, -6.25, 255.0, 0, 0, 0, 255, 0),
                )
            ),
        )

    def test_intensities_alias_is_deterministic_and_no_channel_is_zero(self):
        aliased = adapter.canonicalize_legacy_pointcloud(
            legacy_cloud(
                [(1.0, 2.0, 3.0)], channel_name="intensities", channel_values=[127.9]
            )
        )
        self.assertEqual(
            struct.unpack("<ffffIBBBB", aliased.data),
            (1.0, 2.0, 3.0, 127.0, 0, 0, 0, 127, 0),
        )
        unchanneled = adapter.canonicalize_legacy_pointcloud(
            legacy_cloud([(1.0, 2.0, 3.0)], channel_name=None)
        )
        self.assertEqual(
            struct.unpack("<ffffIBBBB", unchanneled.data),
            (1.0, 2.0, 3.0, 0.0, 0, 0, 0, 0, 0),
        )

    def test_malformed_ambiguous_and_nonfinite_channels_are_rejected(self):
        malformed = [
            legacy_cloud(channel_values=[1.0]),
            legacy_cloud(channel_values=[1.0, math.nan]),
        ]
        duplicate = legacy_cloud()
        duplicate.channels.append(NS(name="intensities", values=[1.0, 2.0]))
        malformed.append(duplicate)
        unknown_nonfinite = legacy_cloud(channel_name="ring", channel_values=[0.0, math.inf])
        malformed.append(unknown_nonfinite)
        for index, message in enumerate(malformed):
            with self.subTest(index=index), self.assertRaises(adapter.SensorValidationError):
                adapter.canonicalize_legacy_pointcloud(message)

    def test_nonfinite_wrong_frame_invalid_stamp_and_oversized_input_are_rejected(self):
        malformed = [
            legacy_cloud([(math.nan, 0.0, 0.0)], channel_name=None),
            legacy_cloud(frame="base_link"),
        ]
        invalid_stamp = legacy_cloud()
        invalid_stamp.header.stamp.nsecs = 1_000_000_000
        malformed.append(invalid_stamp)
        oversized_points = legacy_cloud([(0.0, 0.0, 0.0)], channel_name=None)

        class OversizedPoints:
            def __len__(self):
                return adapter.MAX_POINT_COUNT + 1

        oversized_points.points = OversizedPoints()
        malformed.append(oversized_points)
        oversized_channels = legacy_cloud([(0.0, 0.0, 0.0)], channel_name=None)

        class OversizedChannels:
            def __len__(self):
                return adapter.MAX_CLOUD_DATA_BYTES // adapter.LEGACY_CHANNEL_VALUE_BYTES + 1

        oversized_channels.channels = OversizedChannels()
        malformed.append(oversized_channels)
        for index, message in enumerate(malformed):
            with self.subTest(index=index), self.assertRaises(adapter.SensorValidationError):
                adapter.canonicalize_legacy_pointcloud(message)

    def test_core_legacy_path_enforces_stream_time_contract(self):
        core = adapter.SensorCanonicalizerCore()
        core.adapt_legacy_cloud(legacy_cloud(source_ns=100), 1000)
        with self.assertRaises(adapter.SensorValidationError):
            core.adapt_legacy_cloud(legacy_cloud(source_ns=99), 1001)


class ImuValidationTest(unittest.TestCase):
    def test_valid_imu_validation_is_identity_but_filter_requires_exact_full_window(self):
        message = imu()
        self.assertIs(adapter.validate_imu(message), message)
        core = adapter.SensorCanonicalizerCore(imu_window_samples=4)
        for index in range(3):
            self.assertIsNone(core.adapt_imu(imu(source_ns=100 + index * 5), 500 + index))
        self.assertIsNotNone(core.adapt_imu(imu(source_ns=115), 503))

    def test_alternating_alias_cancels_and_preserves_latest_metadata_pose_and_covariance(self):
        core = adapter.SensorCanonicalizerCore(imu_window_samples=4)
        output = None
        for index, sign in enumerate((1.0, -1.0, 1.0, -1.0)):
            message = imu(
                source_ns=1_000_000_000 + index * 5_000_000,
                linear=(sign * 1.665, 0.0, 9.81 + sign * 0.891),
                angular=(0.0, sign * 0.0089, 0.0),
            )
            output = core.adapt_imu(message, 100 + index)
        self.assertIsNotNone(output)
        self.assertAlmostEqual(output.linear_acceleration.x, 0.0)
        self.assertAlmostEqual(output.linear_acceleration.z, 9.81)
        self.assertAlmostEqual(output.angular_velocity.y, 0.0)
        self.assertEqual(output.header.stamp.nsecs, 15_000_000)
        self.assertEqual(output.header.frame_id, "imu_link")
        self.assertEqual(output.orientation.w, 1.0)
        self.assertAlmostEqual(
            sum(getattr(output.orientation, axis) ** 2 for axis in "xyzw"), 1.0
        )
        self.assertEqual(output.orientation_covariance, [1.0] * 9)
        self.assertEqual(output.angular_velocity_covariance, [2.0] * 9)
        self.assertEqual(output.linear_acceleration_covariance, [3.0] * 9)

    def test_sustained_step_fully_propagates_within_window(self):
        core = adapter.SensorCanonicalizerCore(imu_window_samples=4)
        for index in range(4):
            core.adapt_imu(imu(source_ns=100 + index * 5), index)
        outputs = []
        for index in range(4):
            outputs.append(
                core.adapt_imu(
                    imu(source_ns=120 + index * 5, linear=(4.0, -2.0, 8.0)),
                    4 + index,
                )
            )
        self.assertTrue(all(output is not None for output in outputs))
        self.assertEqual(
            (
                outputs[-1].linear_acceleration.x,
                outputs[-1].linear_acceleration.y,
                outputs[-1].linear_acceleration.z,
            ),
            (4.0, -2.0, 8.0),
        )

    def test_forward_gap_is_distinct_and_requires_a_complete_fresh_window(self):
        core = adapter.SensorCanonicalizerCore(imu_window_samples=3, imu_max_gap_ns=10)
        self.assertIsNone(core.adapt_imu(imu(source_ns=100), 100))
        self.assertIsNone(core.adapt_imu(imu(source_ns=105), 101))
        self.assertIsNone(core.adapt_imu(imu(source_ns=120), 102))
        self.assertIsNone(core.adapt_imu(imu(source_ns=125), 103))
        self.assertIsNotNone(core.adapt_imu(imu(source_ns=130), 104))
        diagnostics = core.imu_diagnostics()
        self.assertEqual(1, diagnostics["imu_gap_count"])
        self.assertEqual("", diagnostics["imu_chronology_failure"])
        self.assertFalse(diagnostics["imu_recovery_pending"])

    def test_source_regression_is_sticky_never_rebased_or_appended(self):
        core = adapter.SensorCanonicalizerCore(imu_window_samples=3, imu_max_gap_ns=10)
        self.assertIsNone(core.adapt_imu(imu(source_ns=100), 100))
        self.assertIsNone(core.adapt_imu(imu(source_ns=105), 101))
        with self.assertRaisesRegex(adapter.SensorValidationError, "E_STAMP_REGRESSION"):
            core.adapt_imu(imu(source_ns=99), 102)
        with self.assertRaisesRegex(adapter.SensorValidationError, "E_STAMP_REGRESSION"):
            core.adapt_imu(imu(source_ns=105), 103)
        diagnostics = core.imu_diagnostics()
        self.assertEqual(105, diagnostics["imu_source_high_water_ns"])
        self.assertEqual(101, diagnostics["imu_receipt_high_water_ns"])
        self.assertEqual("E_STAMP_REGRESSION", diagnostics["imu_chronology_failure"])
        self.assertEqual(2, diagnostics["imu_chronology_failure_count"])
        self.assertEqual(0, diagnostics["imu_window_samples"])
        self.assertTrue(diagnostics["imu_recovery_pending"])
        self.assertIsNone(core.adapt_imu(imu(source_ns=110), 104))
        self.assertIsNone(core.adapt_imu(imu(source_ns=115), 105))
        self.assertIsNotNone(core.adapt_imu(imu(source_ns=120), 106))

    def test_receipt_regression_and_duplicate_are_sticky_never_appended(self):
        core = adapter.SensorCanonicalizerCore(imu_window_samples=3, imu_max_gap_ns=10)
        self.assertIsNone(core.adapt_imu(imu(source_ns=100), 100))
        self.assertIsNone(core.adapt_imu(imu(source_ns=105), 101))
        with self.assertRaisesRegex(adapter.SensorValidationError, "E_RECEIPT_REGRESSION"):
            core.adapt_imu(imu(source_ns=110), 100)
        with self.assertRaisesRegex(adapter.SensorValidationError, "E_RECEIPT_REGRESSION"):
            core.adapt_imu(imu(source_ns=115), 101)
        diagnostics = core.imu_diagnostics()
        self.assertEqual(105, diagnostics["imu_source_high_water_ns"])
        self.assertEqual(101, diagnostics["imu_receipt_high_water_ns"])
        self.assertEqual("E_RECEIPT_REGRESSION", diagnostics["imu_chronology_failure"])
        self.assertEqual(0, diagnostics["imu_window_samples"])
        self.assertIsNone(core.adapt_imu(imu(source_ns=120), 102))
        self.assertIsNone(core.adapt_imu(imu(source_ns=125), 103))
        self.assertIsNotNone(core.adapt_imu(imu(source_ns=130), 104))

    def test_nonfinite_resets_partial_evidence_and_requires_a_fresh_window(self):
        core = adapter.SensorCanonicalizerCore(imu_window_samples=3, imu_max_gap_ns=10)
        self.assertIsNone(core.adapt_imu(imu(source_ns=100), 100))
        self.assertIsNone(core.adapt_imu(imu(source_ns=105), 101))
        bad = imu(source_ns=110)
        bad.linear_acceleration.x = math.nan
        with self.assertRaises(adapter.SensorValidationError):
            core.adapt_imu(bad, 102)
        self.assertIsNone(core.adapt_imu(imu(source_ns=115), 103))
        self.assertIsNone(core.adapt_imu(imu(source_ns=120), 104))
        self.assertIsNotNone(core.adapt_imu(imu(source_ns=125), 105))

    def test_invalid_frame_stamp_quaternion_vectors_and_covariance_reject_whole_imu(self):
        malformed = [imu(frame="base_link")]
        bad = imu()
        bad.header.stamp.secs = -1
        malformed.append(bad)
        bad = imu()
        bad.orientation.w = 0.0
        malformed.append(bad)
        bad = imu()
        bad.orientation.x = math.nan
        malformed.append(bad)
        bad = imu()
        bad.angular_velocity.z = math.inf
        malformed.append(bad)
        bad = imu()
        bad.orientation_covariance = [0.0] * 8
        malformed.append(bad)
        bad = imu()
        bad.linear_acceleration_covariance[3] = math.nan
        malformed.append(bad)
        for index, message in enumerate(malformed):
            with self.subTest(index=index), self.assertRaises(adapter.SensorValidationError):
                adapter.validate_imu(message)


class StaticSensorSurfaceTest(unittest.TestCase):
    def test_raw_plugins_and_adapter_have_single_canonical_topic_owner(self):
        xacro = XACRO.read_text(encoding="utf-8")
        launch = LAUNCH.read_text(encoding="utf-8")
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("<topicName>/simulation/sensors/lidar/raw</topicName>", xacro)
        self.assertIn("<topicName>/simulation/sensors/imu/raw</topicName>", xacro)
        self.assertNotIn("<topicName>/sensors/lidar/points</topicName>", xacro)
        self.assertNotIn("<topicName>/sensors/imu/data</topicName>", xacro)
        self.assertEqual(source.count('CANONICAL_LIDAR_TOPIC = "/sensors/lidar/points"'), 1)
        self.assertEqual(source.count('CANONICAL_IMU_TOPIC = "/sensors/imu/data"'), 1)
        self.assertEqual(launch.count('type="sim_sensor_canonicalizer.py"'), 1)
        self.assertIn('required="true"', launch[launch.index('type="sim_sensor_canonicalizer.py"'):])
        self.assertIn('<param name="imu_filter_window_samples" value="20"/>', launch)
        self.assertIn('<param name="imu_filter_max_gap_ns" value="10000000"/>', launch)

    def test_live_wrapper_subscribes_to_legacy_cloud_and_publishes_pointcloud2(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            "from sensor_msgs.msg import Imu, PointCloud, PointCloud2, PointField", source
        )
        self.assertIn(
            "RAW_LIDAR_TOPIC, PointCloud, self._lidar_callback, queue_size=1", source
        )
        self.assertIn(
            "rospy.Publisher(CANONICAL_LIDAR_TOPIC, PointCloud2, queue_size=1)", source
        )
        self.assertIn("self.core.adapt_legacy_cloud(message, time.monotonic_ns())", source)

    def test_all_authority_gates_precede_every_publisher_and_queues_are_one(self):
        source = SCRIPT.read_text(encoding="utf-8")
        first_publisher = source.index("rospy.Publisher(")
        for parameter, expected in (
            ("/simulation_only", "True"),
            ("/use_sim_time", "True"),
            ("/hardware_motion_authorized", "False"),
            ("/passenger_operation_authorized", "False"),
        ):
            gate = '("{}", {})'.format(parameter, expected)
            self.assertIn(gate, source)
            self.assertLess(source.index(gate), first_publisher)
        self.assertEqual(source.count("rospy.Publisher("), 3)
        self.assertEqual(source.count("rospy.Subscriber("), 2)
        self.assertGreaterEqual(source.count("queue_size=1"), 5)
        self.assertNotIn("wheelchair_hardware", source)
        self.assertNotIn("cmd_vel", source)
        self.assertNotIn("safety permission", source.lower())


if __name__ == "__main__":
    unittest.main()
