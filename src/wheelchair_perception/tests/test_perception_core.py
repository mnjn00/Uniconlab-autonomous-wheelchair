#!/usr/bin/env python3
import math
from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from perception_core import (  # noqa: E402
    CANONICAL_FIELDS, CloudInput, ImuSample, PerceptionConfig, PerceptionCore, Point,
    count_adjacent_offset_decreases,
)


def configuration(**changes):
    values = dict(
        schema_version=1, policy_id="test-sim-v1", qualification="simulation_only",
        hardware_motion_authorized=False, passenger_operation_authorized=False,
        expected_cloud_frame="lidar_link", expected_imu_frame="imu_link",
        expected_source_id="canonical_livox", cloud_ttl_s=0.30, imu_ttl_s=0.10,
        future_tolerance_s=0.05, minimum_rate_hz=8.0, maximum_gap_s=0.30,
        roi_min_x_m=-2.0, roi_max_x_m=12.0, roi_min_y_m=-6.0, roi_max_y_m=6.0,
        roi_min_z_m=-2.0, roi_max_z_m=3.0,
        self_min_x_m=-0.8, self_max_x_m=0.8, self_min_y_m=-0.55,
        self_max_y_m=0.55, self_min_z_m=-0.3, self_max_z_m=1.4,
        voxel_size_m=0.05, ground_cell_size_m=0.25, ground_tolerance_m=0.08,
        max_ground_slope_deg=15.0, obstacle_min_height_m=0.10,
        obstacle_max_height_m=2.20, cluster_cell_size_m=0.20,
        cluster_tolerance_m=0.35, cluster_min_points=2, cluster_max_points=20000,
        min_ground_points=6, gravity_alignment_required=True,
    )
    values.update(changes)
    return PerceptionConfig.from_mapping(values)


def point(x, y, z, offset=0, reflectivity=20):
    return Point(x, y, z, offset, reflectivity, 0, 1, 1)


def ground(z_for_x=lambda _x: 0.0):
    return tuple(point(x, y, z_for_x(x)) for x in (1.0, 2.0, 3.0) for y in (-1.0, 1.0))


def cloud(points, stamp=1.0, **changes):
    return CloudInput(stamp, "lidar_link", tuple(points), **changes)


def imu(stamp=1.0, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return ImuSample(stamp, "imu_link", orientation_xyzw=quaternion)


class ConfigurationTests(unittest.TestCase):
    def test_configuration_is_closed_and_cannot_authorize_hardware(self):
        with self.assertRaisesRegex(ValueError, "closed configuration"):
            PerceptionConfig.from_mapping({**configuration().__dict__, "extra": 1})
        with self.assertRaisesRegex(ValueError, "cannot authorize"):
            configuration(hardware_motion_authorized=True)


class ValidationTests(unittest.TestCase):
    def test_exact_field_layout_corruption_fails_closed(self):
        bad_fields = CANONICAL_FIELDS[:-1]
        result = PerceptionCore(configuration()).process(
            cloud(ground(), fields=bad_fields), imu())
        self.assertFalse(result.health.ok)
        self.assertFalse(result.health.safety_clear)
        self.assertIn("E_CANONICAL_LAYOUT", result.health.reasons)
        self.assertEqual((), result.obstacle_points)

    def test_interleaved_point_offsets_are_valid_and_diagnosable(self):
        offsets = (1_250_000, 340_000, 1_260_000, 350_000, 1_270_000, 360_000)
        points = tuple(
            replace(source, offset_time=offset)
            for source, offset in zip(ground(), offsets)
        )
        self.assertEqual(3, count_adjacent_offset_decreases(points))
        result = PerceptionCore(configuration()).process(cloud(points), imu())
        self.assertTrue(result.health.ok, result.health.reasons)
        self.assertEqual(len(points), result.health.finite_points)

    def test_invalid_uint32_and_unrepresentable_acquisition_time_fail_closed(self):
        for offset in (-1, 1.5, 1 << 32):
            points = (replace(ground()[0], offset_time=offset),) + ground()[1:]
            result = PerceptionCore(configuration()).process(cloud(points), imu())
            with self.subTest(offset=offset):
                self.assertEqual("E_POINT_TIME", result.health.code)

        overflow = PerceptionCore(configuration()).process(
            cloud(ground(), stamp=1.0e10), imu(stamp=1.0e10)
        )
        self.assertEqual("E_POINT_TIME_OVERFLOW", overflow.health.code)
        self.assertFalse(overflow.health.safety_clear)

    def test_nonfinite_frame_and_source_are_rejected(self):
        broken = [replace(ground()[0], x=float("nan"))] + list(ground()[1:])
        result = PerceptionCore(configuration()).process(
            CloudInput(1.0, "wrong", broken, source_id="raw"), imu())
        self.assertEqual(
            {"E_CLOUD_FRAME", "E_CLOUD_SOURCE", "E_POINT_NONFINITE"},
            set(result.health.reasons),
        )


class GeometryTests(unittest.TestCase):
    def test_voxel_representative_and_output_order_are_input_order_independent(self):
        extras = (point(2.01, 2.01, 0.50), point(2.02, 2.02, 0.51),
                  point(2.30, 2.0, 0.5), point(2.31, 2.0, 0.5))
        points = ground() + extras
        first = PerceptionCore(configuration(cluster_min_points=1)).process(cloud(points), imu())
        second = PerceptionCore(configuration(cluster_min_points=1)).process(cloud(reversed(points)), imu())
        self.assertTrue(first.health.ok)
        self.assertEqual(first.obstacle_points, second.obstacle_points)
        self.assertEqual(first.clusters, second.clusters)

    def test_imu_tilt_is_removed_before_ground_segmentation(self):
        pitch = math.radians(10.0)
        body_ground = tuple(
            point(math.cos(pitch) * x, y, math.sin(pitch) * x)
            for x in (1.0, 2.0, 3.0) for y in (-1.0, 1.0)
        )
        q = (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))
        result = PerceptionCore(configuration()).process(cloud(body_ground), imu(quaternion=q))
        self.assertTrue(result.health.ok, result.health.reasons)
        self.assertEqual(6, result.health.ground_points)
        self.assertTrue(all(abs(p.z) < 1e-12 for p in result.ground_points))

    def test_sloped_ground_is_fitted_not_reported_as_obstacle(self):
        slope = math.tan(math.radians(10.0))
        result = PerceptionCore(configuration()).process(
            cloud(ground(lambda x: slope * x)), imu())
        self.assertTrue(result.health.ok, result.health.reasons)
        self.assertEqual(6, result.health.ground_points)
        self.assertEqual((), result.obstacle_points)

    def test_slope_beyond_policy_is_unhealthy(self):
        slope = math.tan(math.radians(20.0))
        result = PerceptionCore(configuration()).process(
            cloud(ground(lambda x: slope * x)), imu())
        self.assertFalse(result.health.ok)
        self.assertIn("E_GROUND_SLOPE", result.health.reasons)
        self.assertFalse(result.health.safety_clear)

    def test_only_configured_low_to_high_obstacles_are_retained(self):
        obstacles = (point(4.0, 1.0, 0.10), point(4.3, 1.0, 0.20),
                     point(4.6, 1.0, 2.20), point(4.9, 1.0, 2.21))
        result = PerceptionCore(configuration(
            cluster_min_points=1, ground_tolerance_m=0.04, obstacle_min_height_m=0.20,
        )).process(cloud(ground() + obstacles), imu())
        self.assertTrue(result.health.ok, result.health.reasons)
        self.assertEqual((obstacles[1], obstacles[2]), result.obstacle_points)

    def test_wheelchair_self_filter_is_counted(self):
        self_return = point(0.1, 0.1, 0.5)
        obstacle = point(4.0, 1.0, 0.5)
        result = PerceptionCore(configuration(cluster_min_points=1)).process(
            cloud(ground() + (self_return, obstacle)), imu())
        self.assertTrue(result.health.ok)
        self.assertEqual(1, result.health.self_filtered_points)
        self.assertEqual((obstacle,), result.obstacle_points)

    def test_cluster_separation_and_transitive_merge(self):
        obstacle_points = (
            point(4.00, 1.0, 0.5), point(4.25, 1.0, 0.5), point(4.50, 1.0, 0.5),
            point(6.00, 1.0, 0.5), point(6.20, 1.0, 0.5),
        )
        result = PerceptionCore(configuration()).process(
            cloud(ground() + obstacle_points), imu())
        self.assertTrue(result.health.ok, result.health.reasons)
        self.assertEqual((3, 2), tuple(len(c.points) for c in result.clusters))
        self.assertAlmostEqual(4.25, result.clusters[0].centroid[0])
        self.assertEqual((4.0, 1.0, 0.5), result.clusters[0].minimum)
        self.assertEqual((4.5, 1.0, 0.5), result.clusters[0].maximum)


class DiagnosticsTests(unittest.TestCase):
    def test_rate_gap_and_regression_are_health_failures(self):
        core = PerceptionCore(configuration())
        self.assertTrue(core.process(cloud(ground(), 1.0), imu(1.0)).health.ok)
        gap = core.process(cloud(ground(), 1.31), imu(1.31))
        self.assertFalse(gap.health.ok)
        self.assertIn("E_RATE_GAP", gap.health.reasons)
        regression = core.process(cloud(ground(), 1.30), imu(1.30))
        self.assertIn("E_TIME_REGRESSION", regression.health.reasons)

    def test_filtering_accounting_prevents_silent_drop(self):
        self_return = point(0.0, 0.0, 0.5)
        outside_roi = point(20.0, 0.0, 0.5)
        duplicate_a = point(4.001, 1.001, 0.5)
        duplicate_b = point(4.002, 1.002, 0.5)
        inputs = ground() + (self_return, outside_roi, duplicate_a, duplicate_b)
        result = PerceptionCore(configuration(cluster_min_points=1)).process(
            cloud(inputs), imu())
        self.assertTrue(result.health.ok, result.health.reasons)
        self.assertEqual(len(inputs), result.health.input_points)
        self.assertEqual(len(inputs), result.health.finite_points)
        self.assertEqual(1, result.health.self_filtered_points)
        self.assertEqual(1, result.health.roi_filtered_points)
        self.assertEqual(7, result.health.voxel_points)
        accounted_before_voxel = (result.health.voxel_points + result.health.self_filtered_points +
                                  result.health.roi_filtered_points + 1)  # one voxel duplicate
        self.assertEqual(len(inputs), accounted_before_voxel)
        self.assertFalse(result.health.safety_clear)

    def test_stale_cloud_never_produces_obstacle_evidence(self):
        result = PerceptionCore(configuration()).process(cloud(ground(), 1.0), imu(1.0), now_s=1.31)
        self.assertEqual("E_CLOUD_STALE", result.health.code)
        self.assertFalse(result.health.ok)
        self.assertEqual((), result.obstacle_points)
        self.assertFalse(result.health.safety_clear)


if __name__ == "__main__":
    unittest.main()
