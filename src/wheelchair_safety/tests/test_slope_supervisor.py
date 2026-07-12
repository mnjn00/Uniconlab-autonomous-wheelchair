#!/usr/bin/env python3
import hashlib
import importlib.util
import math
from pathlib import Path
import unittest
from types import SimpleNamespace


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "slope_supervisor.py"
SPEC = importlib.util.spec_from_file_location("slope_supervisor", str(MODULE_PATH))
slope = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(slope)


def quaternion(pitch_deg=0.0, roll_deg=0.0):
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    cy, sy = 1.0, 0.0
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def gravity(pitch_deg=0.0, roll_deg=0.0, magnitude=9.80665):
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    return (
        -magnitude * math.sin(pitch),
        magnitude * math.sin(roll) * math.cos(pitch),
        magnitude * math.cos(roll) * math.cos(pitch),
    )


def valid_core():
    core = slope.SlopeSupervisorCore()
    core.calibration_state = slope.CAL_VALID
    core.calibration_sha256 = "a" * 64
    return core


def decide(core, pitch=0.0, roll=0.0, now=1.0, **overrides):
    values = dict(
        quaternion=quaternion(pitch, roll),
        acceleration=gravity(pitch, roll),
        angular_velocity=(0.0, 0.0, 0.0),
        source_stamp_s=now - 0.01,
        receipt_stamp_s=now - 0.005,
        now_s=now,
        transform_age_s=0.01,
        transform_valid=True,
        time_valid=True,
        transform_verified=True,
        transform_label="simulation",
        zone="normal",
        input_provenance="gazebo:/sensors/imu/data:imu_link",
        zone_age_s=0.0,
    )
    values.update(overrides)
    return core.decide(**values)


class QuaternionTests(unittest.TestCase):
    def test_rep103_pitch_roll(self):
        pitch, roll = slope.quaternion_to_pitch_roll(quaternion(8.0, -3.0))
        self.assertAlmostEqual(math.degrees(pitch), 8.0, places=6)
        self.assertAlmostEqual(math.degrees(roll), -3.0, places=6)

    def test_invalid_quaternion_norm_fails_closed(self):
        result = decide(valid_core(), quaternion=(0.0, 0.0, 0.0, 0.0))
        self.assertEqual(result.state, slope.UNKNOWN)
        self.assertTrue(result.reason_mask & slope.CORRUPT_DATA)
        self.assertEqual(result.safety_signal_state, slope.UNKNOWN)


class ThresholdBoundaryTests(unittest.TestCase):
    def assert_state(self, expected, pitch=0.0, roll=0.0):
        self.assertEqual(decide(valid_core(), pitch, roll).state, expected)

    def test_uphill_equal_below_above_edges(self):
        self.assert_state(slope.CLEAR, pitch=7.0)
        self.assert_state(slope.SLOW, pitch=7.0001)
        self.assert_state(slope.SLOW, pitch=10.0)
        self.assert_state(slope.STOP, pitch=10.0001)

    def test_downhill_equal_below_above_edges(self):
        self.assert_state(slope.CLEAR, pitch=-5.0)
        self.assert_state(slope.SLOW, pitch=-5.0001)
        self.assert_state(slope.SLOW, pitch=-7.0)
        self.assert_state(slope.STOP, pitch=-7.0001)

    def test_cross_slope_is_symmetric_at_edges(self):
        for sign in (-1.0, 1.0):
            self.assert_state(slope.CLEAR, roll=sign * 4.0)
            self.assert_state(slope.SLOW, roll=sign * 4.0001)
            self.assert_state(slope.SLOW, roll=sign * 6.0)
            self.assert_state(slope.STOP, roll=sign * 6.0001)

    def test_slow_only_lowers_speed_and_signal_remains_clear(self):
        result = decide(valid_core(), pitch=8.0)
        self.assertEqual(result.state, slope.SLOW)
        self.assertEqual(result.recommended_max_linear_mps, 0.10)
        self.assertEqual(result.safety_signal_state, slope.CLEAR)

    def test_downhill_factor_only_applies_downhill(self):
        downhill = decide(valid_core(), pitch=-5.0)
        uphill = decide(valid_core(), pitch=5.0)
        self.assertAlmostEqual(downhill.downhill_factor, math.sin(math.radians(5.0)))
        self.assertEqual(uphill.downhill_factor, 0.0)


class FailClosedTests(unittest.TestCase):
    def test_stale_source_or_receipt_stops(self):
        core = valid_core()
        result = decide(core, now=1.0, source_stamp_s=0.89)
        self.assertEqual(result.state, slope.UNKNOWN)
        self.assertTrue(result.reason_mask & slope.IMU_STALE)
        result = decide(valid_core(), now=1.0, receipt_stamp_s=0.89)
        self.assertEqual(result.state, slope.UNKNOWN)
        self.assertTrue(result.reason_mask & slope.SENSOR_STALE)

    def test_uncalibrated_stops(self):
        result = decide(slope.SlopeSupervisorCore())
        self.assertEqual(result.state, slope.UNKNOWN)
        self.assertTrue(result.reason_mask & slope.IMU_UNCALIBRATED)

    def test_dynamic_imu_stops_after_residual_duration(self):
        core = valid_core()
        extra_acceleration = (0.0, 0.0, 10.80665)
        first = decide(core, now=1.0, acceleration=extra_acceleration)
        self.assertEqual(first.state, slope.CLEAR)
        result = decide(core, now=1.101, acceleration=extra_acceleration)
        self.assertEqual(result.state, slope.UNKNOWN)
        self.assertTrue(result.reason_mask & slope.INPUT_UNKNOWN)

    def test_orientation_gravity_disagreement_stops_immediately(self):
        result = decide(valid_core(), quaternion=quaternion(0.0), acceleration=gravity(4.0))
        self.assertEqual(result.state, slope.UNKNOWN)
        self.assertGreater(result.orientation_disagreement_rad, math.radians(3.0))

    def test_bad_tf_and_time_stop(self):
        self.assertEqual(decide(valid_core(), transform_valid=False).state, slope.UNKNOWN)
        self.assertEqual(decide(valid_core(), time_valid=False).state, slope.UNKNOWN)
        self.assertEqual(decide(valid_core(), transform_label="unknown").state, slope.UNKNOWN)

    def test_nonfinite_imu_stops(self):
        result = decide(valid_core(), acceleration=(math.nan, 0.0, 9.80665))
        self.assertEqual(result.state, slope.UNKNOWN)
        self.assertTrue(result.reason_mask & slope.CORRUPT_DATA)


class ZoneAndStateTests(unittest.TestCase):
    def test_zone_policy(self):
        self.assertEqual(decide(valid_core(), zone="manual_only").state, slope.STOP)
        self.assertEqual(decide(valid_core(), zone="degraded_stop").state, slope.STOP)
        self.assertEqual(decide(valid_core(), zone="not_in_manifest").state, slope.STOP)
    def test_duplicate_and_regressed_source_stamps_never_clear_or_lower_high_water(self):
        core = valid_core()
        self.assertEqual(decide(core, now=10.01, source_stamp_s=10.0, receipt_stamp_s=10.005).state, slope.CLEAR)
        duplicate = decide(core, now=10.02, source_stamp_s=10.0, receipt_stamp_s=10.015)
        self.assertEqual(duplicate.state, slope.UNKNOWN)
        self.assertTrue(duplicate.reason_mask & slope.CLOCK)
        regression = decide(core, now=10.03, source_stamp_s=9.0, receipt_stamp_s=10.025)
        self.assertEqual(regression.state, slope.UNKNOWN)
        self.assertTrue(regression.reason_mask & slope.CLOCK)
        partial_recovery = decide(core, now=10.04, source_stamp_s=9.1, receipt_stamp_s=10.035)
        self.assertEqual(partial_recovery.state, slope.UNKNOWN)
        self.assertTrue(partial_recovery.reason_mask & slope.CLOCK)
        self.assertEqual(core._last_source_stamp, 10.0)

    def test_restrictive_zone_precedence(self):
        self.assertEqual(decide(valid_core(), zone="normal", zone_age_s=0.0).state, slope.CLEAR)
        for zone in ("manual_only", "degraded_stop"):
            self.assertEqual(decide(valid_core(), zone=zone, zone_age_s=0.0).state, slope.STOP)


class StructuredZoneEvidenceTests(unittest.TestCase):
    @staticmethod
    def stamp(value):
        return SimpleNamespace(to_sec=lambda: value)

    def setUp(self):
        policy = slope.SlopePolicy(route_zone_policies=(
            ("zone-normal", "normal"),
            ("zone-manual", "manual_only"),
            ("zone-degraded", "degraded_stop"),
        ))
        self.node = slope.SlopeSupervisorRosNode.__new__(slope.SlopeSupervisorRosNode)
        self.node.core = slope.SlopeSupervisorCore(policy)
        self.node.core.calibration_state = slope.CAL_VALID
        self.node.rospy = SimpleNamespace(
            Time=SimpleNamespace(now=lambda: self.stamp(10.0))
        )
        self.node.zone = "unknown"
        self.node.zone_receipt_stamp_s = -math.inf
        self.node._initial_zone_bootstrap_active = False
        self.node._initial_zone_bootstrap_deadline_s = None
        self.node._bootstrap_zone_sequence_high_water = None
        self.node._bootstrap_zone_evaluation_high_water_stamp_s = None
        self.node._bootstrap_zone_source_high_water_stamp_s = None
        self.node.operation_mode = "simulation"
        self.node._zone_sequence_high_water = None
        self.node._zone_source_high_water_stamp_s = None
        self.node._zone_evaluation_high_water_stamp_s = None

    def message(self, **overrides):
        policy = self.node.core.policy
        values = dict(
            header=SimpleNamespace(stamp=self.stamp(9.95)),
            evaluation_stamp=self.stamp(9.98),
            sequence=1,
            state=1,
            reason_mask=0,
            source=policy.route_safety_source,
            manifest_id=policy.route_safety_manifest_id,
            manifest_sha256=policy.route_safety_manifest_sha256,
            zone_id="zone-normal",
            route_id="route-a",
            segment_id="segment-a",
        )
        values.update(overrides)
        return SimpleNamespace(**values)
    def bootstrap_message(self, **overrides):
        values = dict(
            header=SimpleNamespace(stamp=self.stamp(0.0)),
            evaluation_stamp=self.stamp(9.98),
            sequence=1,
            state=0,
            reason_mask=slope.INPUT_UNKNOWN | slope.GEOFENCE,
            source=self.node.core.policy.route_safety_source,
            manifest_id=self.node.core.policy.route_safety_manifest_id,
            manifest_sha256=self.node.core.policy.route_safety_manifest_sha256,
            route_id="",
            segment_id="",
            zone_id="",
        )
        values.update(overrides)
        return SimpleNamespace(**values)
    def zero_time_bootstrap_message(self, **overrides):
        values = dict(
            header=SimpleNamespace(stamp=self.stamp(0.0)),
            evaluation_stamp=self.stamp(0.0),
            sequence=0,
            state=0,
            reason_mask=slope.INPUT_UNKNOWN | slope.GEOFENCE,
            source=self.node.core.policy.route_safety_source,
            manifest_id=self.node.core.policy.route_safety_manifest_id,
            manifest_sha256=self.node.core.policy.route_safety_manifest_sha256,
            route_id="",
            segment_id="",
            zone_id="",
        )
        values.update(overrides)
        return SimpleNamespace(**values)



    def test_identity_sequence_staleness_and_replay_fail_closed(self):
        self.node._zone_cb(self.message())
        self.assertEqual(self.node.zone, "normal")
        self.node._zone_cb(self.message(sequence=1, zone_id="zone-manual"))
        self.assertEqual(self.node.zone, "unknown")
        self.assertEqual(self.node._zone_sequence_high_water, 1)
        self.node._zone_cb(self.message(sequence=2, evaluation_stamp=self.stamp(9.0)))
        self.assertEqual(self.node.zone, "unknown")
        self.node._zone_cb(self.message(sequence=3, manifest_sha256="0" * 64))
        self.assertEqual(self.node.zone, "unknown")
        self.node._zone_cb(self.message(sequence=4, source="forged"))
        self.assertEqual(self.node.zone, "unknown")

    def test_restrictive_structured_policy_cannot_be_masked_by_normal_label(self):
        self.node._zone_cb(self.message(zone_id="zone-manual"))
        self.assertEqual(self.node.zone, "manual_only")
        self.assertEqual(decide(self.node.core, zone=self.node.zone, zone_age_s=0.0).state, slope.STOP)
        self.node._zone_cb(self.message(
            sequence=2,
            header=SimpleNamespace(stamp=self.stamp(9.96)),
            evaluation_stamp=self.stamp(9.99),
            zone_id="zone-degraded",
        ))
        self.assertEqual(self.node.zone, "degraded_stop")
        self.assertEqual(decide(self.node.core, now=2.0, zone=self.node.zone, zone_age_s=0.0).state, slope.STOP)
    def _enable_bootstrap(self):
        self.node.zone = "normal"
        self.node.zone_receipt_stamp_s = 10.0
        self.node._initial_zone_bootstrap_active = True
        self.node._initial_zone_bootstrap_deadline_s = 20.1
        self.node._bootstrap_zone_sequence_high_water = None
        self.node._bootstrap_zone_evaluation_high_water_stamp_s = None
        self.node._bootstrap_zone_source_high_water_stamp_s = None

    def test_bootstrap_deadline_starts_at_first_valid_calibration_receipt(self):
        self._enable_bootstrap()
        self.node._initial_zone_bootstrap_deadline_s = None
        self.node._start_initial_zone_bootstrap_deadline(10.0)
        self.node._start_initial_zone_bootstrap_deadline(11.0)
        self.assertEqual(
            self.node._initial_zone_bootstrap_deadline_s,
            10.0 + self.node.core.policy.calibration_duration_s
            + self.node.core.policy.route_zone_ttl_s,
        )

    def test_simulation_pre_route_stop_preserves_bounded_bootstrap(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message())
        self.node._refresh_initial_zone_bootstrap(20.1)
        self.assertEqual(self.node.zone, "normal")
        self.assertEqual(self.node.zone_receipt_stamp_s, 10.0)
        self.assertTrue(self.node._initial_zone_bootstrap_active)

    def test_simulation_bootstrap_accepts_exact_tf_reason_only(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(reason_mask=slope.TF | slope.GEOFENCE))
        self.assertEqual(self.node.zone, "normal")
        self.assertTrue(self.node._initial_zone_bootstrap_active)
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(
            reason_mask=slope.TF | slope.GEOFENCE | slope.INPUT_UNKNOWN,
        ))
        self.assertEqual(self.node.zone, "unknown")

    def test_zero_time_bootstrap_preserves_without_refresh(self):
        self._enable_bootstrap()
        self.node.zone_receipt_stamp_s = 9.0
        self.node.rospy.Time.now = lambda: self.stamp(0.0)
        self.node._zone_cb(self.zero_time_bootstrap_message())
        self.assertEqual(self.node.zone, "normal")
        self.assertEqual(self.node.zone_receipt_stamp_s, 9.0)
        self.assertIsNone(self.node._bootstrap_zone_sequence_high_water)
        self.node.rospy.Time.now = lambda: self.stamp(10.0)
        self.node._zone_cb(self.zero_time_bootstrap_message(reason_mask=1))
        self.assertEqual(self.node.zone, "unknown")

    def test_simulation_bootstrap_accepts_active_route_missing_localization_stop(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(route_id="route-a"))
        self.assertEqual(self.node.zone, "normal")
        self.assertTrue(self.node._initial_zone_bootstrap_active)

    def test_simulation_bootstrap_requires_exact_reason_and_chronology(self):
        for message in (
            self.bootstrap_message(reason_mask=1),
            self.bootstrap_message(evaluation_stamp=self.stamp(9.89)),
        ):
            self._enable_bootstrap()
            self.node._zone_cb(message)
            self.assertEqual(self.node.zone, "unknown")
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message())
        self.node._zone_cb(self.bootstrap_message(sequence=2))
        self.assertEqual(self.node.zone, "unknown")
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message())
        self.node._zone_cb(self.bootstrap_message(
            sequence=1,
            evaluation_stamp=self.stamp(9.99),
        ))
        self.assertEqual(self.node.zone, "unknown")

    def test_simulation_bootstrap_accepts_monotonic_zero_source_evidence(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message())
        self.node._zone_cb(self.bootstrap_message(
            sequence=2,
            evaluation_stamp=self.stamp(9.99),
        ))
        self.assertEqual(self.node.zone, "normal")
        self.assertEqual(self.node._zone_source_high_water_stamp_s, None)

    def test_positive_source_bootstrap_accepts_heartbeats_and_rejects_regression(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.node._zone_cb(self.bootstrap_message(
            sequence=2,
            evaluation_stamp=self.stamp(9.99),
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.assertEqual(self.node.zone, "normal")
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.node._zone_cb(self.bootstrap_message(
            sequence=1,
            evaluation_stamp=self.stamp(9.99),
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.assertEqual(self.node.zone, "unknown")
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.node._zone_cb(self.bootstrap_message(
            sequence=2,
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.assertEqual(self.node.zone, "unknown")
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.node._zone_cb(self.bootstrap_message(
            sequence=2,
            evaluation_stamp=self.stamp(9.99),
            header=SimpleNamespace(stamp=self.stamp(9.94)),
        ))
        self.assertEqual(self.node.zone, "unknown")

    def test_static_transform_lookup_is_cached_but_dynamic_is_not(self):
        transform = SimpleNamespace(
            header=SimpleNamespace(frame_id="base_link", stamp=self.stamp(1.0)),
            child_frame_id="imu_link",
            transform=SimpleNamespace(
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
        )
        calls = []
        self.node.rospy.Duration = lambda value: value
        self.node.tf_buffer = SimpleNamespace(
            lookup_transform=lambda *args: calls.append(args) or transform
        )
        self.node.imu_frame = "imu_link"
        self.node.transform_is_static = True
        self.node.transform_verified = True
        self.node._static_imu_transform = None
        self.node._lookup_imu_transform(self.stamp(1.0), object())
        self.node._lookup_imu_transform(self.stamp(2.0), object())
        self.assertEqual(len(calls), 1)

        class Receipt:
            def __sub__(self, other):
                return SimpleNamespace(to_sec=lambda: 0.0)

        self.node.transform_is_static = False
        transform.header.stamp = self.stamp(1.0)
        receipt = Receipt()
        self.node._lookup_imu_transform(self.stamp(3.0), receipt)
        self.node._lookup_imu_transform(self.stamp(4.0), receipt)
        self.assertEqual(len(calls), 3)

    def test_static_transform_wrong_frame_is_not_cached(self):
        self.node.rospy.Duration = lambda value: value
        self.node.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(
            header=SimpleNamespace(frame_id="wrong", stamp=self.stamp(0.0)),
            child_frame_id="imu_link",
            transform=SimpleNamespace(
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
        ))
        self.node.imu_frame = "imu_link"
        self.node.transform_is_static = True
        self.node.transform_verified = True
        self.node._static_imu_transform = None
        with self.assertRaises(ValueError):
            self.node._lookup_imu_transform(self.stamp(1.0), object())
        self.assertIsNone(self.node._static_imu_transform)

    def test_static_transform_nonfinite_rotation_is_not_cached(self):
        self.node.rospy.Duration = lambda value: value
        self.node.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(
            header=SimpleNamespace(frame_id="base_link", stamp=self.stamp(0.0)),
            child_frame_id="imu_link",
            transform=SimpleNamespace(
                rotation=SimpleNamespace(x=float("nan"), y=0.0, z=0.0, w=1.0),
                translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
        ))
        self.node.imu_frame = "imu_link"
        self.node.transform_is_static = True
        self.node.transform_verified = True
        self.node._static_imu_transform = None
        with self.assertRaises(ValueError):
            self.node._lookup_imu_transform(self.stamp(1.0), object())
        self.assertIsNone(self.node._static_imu_transform)

    def test_static_transform_negative_stamp_is_not_cached(self):
        self.node.rospy.Duration = lambda value: value
        self.node.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(
            header=SimpleNamespace(frame_id="base_link", stamp=self.stamp(-0.1)),
            child_frame_id="imu_link",
            transform=SimpleNamespace(
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
        ))
        self.node.imu_frame = "imu_link"
        self.node.transform_is_static = True
        self.node.transform_verified = True
        self.node._static_imu_transform = None
        with self.assertRaises(ValueError):
            self.node._lookup_imu_transform(self.stamp(1.0), object())
        self.assertIsNone(self.node._static_imu_transform)

    def test_clear_handoff_must_follow_bootstrap_chronology(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message())
        self.node._zone_cb(self.message(
            sequence=0,
            evaluation_stamp=self.stamp(9.99),
        ))
        self.assertEqual(self.node.zone, "unknown")
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message())
        self.node._zone_cb(self.message(
            sequence=1,
            evaluation_stamp=self.stamp(9.99),
        ))
        self.assertEqual(self.node.zone, "unknown")
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message())
        self.node._zone_cb(self.message(
            sequence=2,
            header=SimpleNamespace(stamp=self.stamp(9.96)),
            evaluation_stamp=self.stamp(9.98),
        ))
        self.assertEqual(self.node.zone, "unknown")

    def test_clear_handoff_accepts_source_heartbeat_but_rejects_stale_chronology(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.node._zone_cb(self.message(
            sequence=2,
            evaluation_stamp=self.stamp(9.99),
        ))
        self.node._zone_cb(self.message(
            sequence=3,
            evaluation_stamp=self.stamp(10.0),
        ))
        self.assertEqual(self.node.zone, "normal")
        self.node.rospy.Time.now = lambda: self.stamp(10.02)
        self.node._zone_cb(self.message(
            sequence=3,
            evaluation_stamp=self.stamp(10.01),
        ))
        self.assertEqual(self.node.zone, "unknown")
        self._enable_bootstrap()
        self.node._zone_cb(self.bootstrap_message(
            header=SimpleNamespace(stamp=self.stamp(9.95)),
        ))
        self.node._zone_cb(self.message(
            sequence=2,
            evaluation_stamp=self.stamp(9.99),
        ))
        self.node._zone_cb(self.message(
            sequence=3,
            evaluation_stamp=self.stamp(9.99),
        ))
        self.assertEqual(self.node.zone, "unknown")

    def test_simulation_bootstrap_expires_fail_closed(self):
        self._enable_bootstrap()
        self.node._refresh_initial_zone_bootstrap(20.100001)
        self.assertEqual(self.node.zone, "unknown")
        self.assertFalse(self.node._initial_zone_bootstrap_active)

    def test_verified_route_evidence_replaces_simulation_bootstrap(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.message())
        self.assertEqual(self.node.zone, "normal")
        self.assertFalse(self.node._initial_zone_bootstrap_active)

    def test_route_handoff_never_falls_back_to_bootstrap(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.message())
        self.node._zone_cb(self.bootstrap_message(sequence=2))
        self.assertEqual(self.node.zone, "unknown")
        self.assertFalse(self.node._initial_zone_bootstrap_active)

    def test_active_route_stop_revokes_simulation_bootstrap(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.message(state=3, reason_mask=1))
        self.assertEqual(self.node.zone, "unknown")
        self.assertFalse(self.node._initial_zone_bootstrap_active)

    def test_malformed_evidence_revokes_simulation_bootstrap(self):
        self._enable_bootstrap()
        self.node._zone_cb(self.message(source="forged"))
        self.assertEqual(self.node.zone, "unknown")
        self.assertFalse(self.node._initial_zone_bootstrap_active)

    def test_non_simulation_bootstrap_is_denied(self):
        self._enable_bootstrap()
        self.node.operation_mode = "replay"
        self.node._refresh_initial_zone_bootstrap(10.01)
        self.assertEqual(self.node.zone, "unknown")
        self.assertFalse(self.node._initial_zone_bootstrap_active)

    def test_release_hysteresis_requires_tight_clear_for_full_hold(self):
        core = valid_core()
        self.assertEqual(decide(core, pitch=10.1, now=1.0).state, slope.STOP)
        self.assertEqual(decide(core, pitch=0.0, now=1.1).state, slope.STOP)
        self.assertEqual(decide(core, pitch=0.0, now=2.09).state, slope.STOP)
        self.assertEqual(decide(core, pitch=0.0, now=2.11).state, slope.CLEAR)

    def test_release_hysteresis_resets_outside_tightened_boundary(self):
        core = valid_core()
        self.assertEqual(decide(core, pitch=8.0, now=1.0).state, slope.SLOW)
        self.assertEqual(decide(core, pitch=6.5, now=1.1).state, slope.SLOW)
        self.assertEqual(decide(core, pitch=0.0, now=1.2).state, slope.SLOW)
        self.assertEqual(decide(core, pitch=0.0, now=2.21).state, slope.CLEAR)

    def test_sequence_hash_and_authority_are_inert(self):
        core = valid_core()
        first = decide(core)
        second = decide(core, now=1.02)
        self.assertEqual((first.sequence, second.sequence), (1, 2))
        self.assertEqual(len(first.policy_sha256), 64)
        self.assertFalse(first.hardware_motion_authorized)
        self.assertFalse(first.passenger_operation_authorized)
        self.assertFalse(core.policy.hardware_motion_authorized)
        self.assertFalse(core.policy.passenger_operation_authorized)

    def test_invalid_attempt_to_grant_hardware_authority_is_rejected(self):
        with self.assertRaises(ValueError):
            slope.SlopePolicy(hardware_motion_authorized=True)


class CalibrationTests(unittest.TestCase):
    def make_samples(
        self,
        *,
        count=1900,
        duration=10.0,
        gyro=0.0,
        acceleration_magnitude=9.80665,
        pitch_values=None,
        roll_values=None,
        transform=(0.0, 0.0, 0.0, 1.0),
    ):
        pitch_values = pitch_values or [0.0] * count
        roll_values = roll_values or [0.0] * count
        return [
            slope.CalibrationSample(
                quaternion=quaternion(
                    pitch_deg=pitch_values[index],
                    roll_deg=roll_values[index],
                ),
                acceleration=gravity(
                    pitch_deg=pitch_values[index],
                    roll_deg=roll_values[index],
                    magnitude=acceleration_magnitude,
                ),
                angular_velocity=(gyro, 0.0, 0.0),
                source_stamp_s=1.0 + duration * index / (count - 1),
                imu_to_base_quaternion=transform,
            )
            for index in range(count)
        ]

    def jittered_stamps(self):
        return [
            1.0 + 10.0 * index / 1899
            + (0.020 * (1899 - index) / 949 if index >= 950 else 0.0)
            for index in range(1900)
        ]

    def with_gravity_outliers(self, count):
        samples = self.make_samples()
        return [
            slope.CalibrationSample(
                quaternion=sample.quaternion,
                acceleration=gravity(magnitude=11.0) if index < count else sample.acceleration,
                angular_velocity=sample.angular_velocity,
                source_stamp_s=sample.source_stamp_s,
                imu_to_base_quaternion=sample.imu_to_base_quaternion,
            )
            for index, sample in enumerate(samples)
        ]


    def calibrate(self, core, samples, **overrides):
        evidence = dict(
            transform_verified=True,
            time_verified=True,
            transform_label="simulation",
            operation_mode="simulation",
            input_provenance="gazebo:/sensors/imu/data:imu_link",
            transform_is_static=True,
            transform_stamp_s=0.0,
        )
        evidence.update(overrides)
        return core.calibrate(samples, **evidence)

    def test_exact_contract_boundaries_calibrate_and_clear(self):
        pitches = [-0.25 if index % 2 else 0.25 for index in range(1900)]
        rolls = [0.25 if index % 2 else -0.25 for index in range(1900)]
        samples = self.make_samples(
            gyro=0.02,
            acceleration_magnitude=9.80665 + 0.15,
            pitch_values=pitches,
            roll_values=rolls,
        )
        core = slope.SlopeSupervisorCore()
        self.assertTrue(self.calibrate(core, samples))
        self.assertEqual(core.calibration_state, slope.CAL_VALID)
        result = decide(core)
        self.assertEqual(result.state, slope.CLEAR)
        self.assertEqual(result.safety_signal_state, slope.CLEAR)
        self.assertFalse(result.hardware_motion_authorized)
        self.assertFalse(result.passenger_operation_authorized)

    def test_jittered_dropped_samples_meeting_coverage_calibrate(self):
        samples = self.make_samples()
        stamps = self.jittered_stamps()
        samples = [
            slope.CalibrationSample(
                quaternion=sample.quaternion,
                acceleration=sample.acceleration,
                angular_velocity=sample.angular_velocity,
                source_stamp_s=stamp,
                imu_to_base_quaternion=sample.imu_to_base_quaternion,
            )
            for sample, stamp in zip(samples, stamps)
        ]

        core = slope.SlopeSupervisorCore()
        self.assertTrue(self.calibrate(core, samples))
        self.assertEqual(core.calibration_state, slope.CAL_VALID)

    def test_p95_stationarity_allows_fewer_than_five_percent_outliers(self):
        core = slope.SlopeSupervisorCore()
        self.assertTrue(self.calibrate(core, self.with_gravity_outliers(95)))
        self.assertEqual(core.calibration_state, slope.CAL_VALID)

    def test_p95_stationarity_rejects_more_than_five_percent_outliers(self):
        core = slope.SlopeSupervisorCore()
        self.assertFalse(self.calibrate(core, self.with_gravity_outliers(96)))
        self.assertEqual(core.calibration_state, slope.CAL_INVALID)
        result = decide(core)
        self.assertEqual(result.recommended_max_linear_mps, 0.0)
        self.assertTrue(result.reason_mask & slope.IMU_UNCALIBRATED)

    def test_window_duration_boundaries(self):
        for duration in (10.0, 10.005):
            core = slope.SlopeSupervisorCore()
            self.assertTrue(self.calibrate(core, self.make_samples(duration=duration)))

        core = slope.SlopeSupervisorCore()
        self.assertFalse(self.calibrate(core, self.make_samples(duration=10.005001)))
        self.assertEqual(core.calibration_state, slope.CAL_INVALID)

    def test_insufficient_coverage_or_short_window_fails_closed(self):
        for samples in (
            self.make_samples(count=1899),
            self.make_samples(duration=9.999),
        ):
            core = slope.SlopeSupervisorCore()
            self.assertFalse(self.calibrate(core, samples))
            self.assertEqual(decide(core).safety_signal_state, slope.UNKNOWN)

    def test_movement_above_boundary_is_rejected(self):
        core = slope.SlopeSupervisorCore()
        self.assertFalse(self.calibrate(core, self.make_samples(gyro=0.0200001)))
        self.assertEqual(core.calibration_state, slope.CAL_INVALID)

    def test_gravity_outside_boundary_is_rejected(self):
        core = slope.SlopeSupervisorCore()
        samples = self.make_samples(acceleration_magnitude=9.80665 + 0.150001)
        self.assertFalse(self.calibrate(core, samples))

    def test_transform_and_evidence_mismatch_are_rejected(self):
        rotated = quaternion(pitch_deg=1.0)
        core = slope.SlopeSupervisorCore()
        self.assertFalse(self.calibrate(core, self.make_samples(transform=rotated)))
        self.assertFalse(
            self.calibrate(
                core,
                self.make_samples(),
                transform_verified=False,
            )
        )
        self.assertFalse(
            self.calibrate(
                core,
                self.make_samples(),
                transform_is_static=False,
                transform_stamp_s=0.0,
            )
        )
        self.assertFalse(
            self.calibrate(
                core,
                self.make_samples(),
                time_verified=False,
            )
        )
        self.assertFalse(
            self.calibrate(
                core,
                self.make_samples(),
                input_provenance="",
            )
        )

    def test_calibration_is_simulation_only(self):
        for mode in ("hardware", "replay", "unverified"):
            core = slope.SlopeSupervisorCore()
            self.assertFalse(
                self.calibrate(core, self.make_samples(), operation_mode=mode)
            )
            self.assertEqual(decide(core).safety_signal_state, slope.UNKNOWN)

    def test_calibration_hash_is_deterministic_and_provenance_bound(self):
        samples = self.make_samples()
        first = slope.SlopeSupervisorCore()
        second = slope.SlopeSupervisorCore()
        self.assertTrue(self.calibrate(first, samples))
        self.assertTrue(self.calibrate(second, samples))
        self.assertEqual(first.calibration_sha256, second.calibration_sha256)

        changed = slope.SlopeSupervisorCore()
        self.assertTrue(
            self.calibrate(
                changed,
                samples,
                input_provenance="gazebo:/different_imu:imu_link",
            )
        )
        self.assertNotEqual(first.calibration_sha256, changed.calibration_sha256)

    def test_calibrating_output_is_unknown_stop(self):
        core = slope.SlopeSupervisorCore()
        core.calibration_state = slope.CAL_CALIBRATING
        result = decide(core)
        self.assertEqual(result.state, slope.UNKNOWN)
        self.assertEqual(result.recommended_max_linear_mps, 0.0)
        self.assertTrue(result.reason_mask & slope.IMU_UNCALIBRATED)

    def make_calibrating_node(self):
        node = slope.SlopeSupervisorRosNode.__new__(slope.SlopeSupervisorRosNode)
        node.core = slope.SlopeSupervisorCore()
        node.core.calibration_state = slope.CAL_CALIBRATING
        node.operation_mode = "simulation"
        node.calibration_enabled = True
        node.transform_verified = True
        node.transform_label = "simulation"
        node.transform_is_static = True
        node.input_provenance = "gazebo:/sensors/imu/data:imu_link"
        node._calibration_samples = []
        return node

    def observe(self, node, stamp, *, gyro=0.0, magnitude=9.80665, pitch=0.0):
        sample = slope.CalibrationSample(
            quaternion=quaternion(pitch),
            acceleration=gravity(pitch, magnitude=magnitude),
            angular_velocity=(gyro, 0.0, 0.0),
            source_stamp_s=stamp,
            imu_to_base_quaternion=(0.0, 0.0, 0.0, 1.0),
            imu_to_base_translation=(0.0, 0.0, 0.0),
        )
        node._observe_calibration_sample(
            sample,
            True,
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0),
            0.0,
        )

    def test_malformed_sample_resets_candidate_without_poisoning_retry(self):
        node = self.make_calibrating_node()
        self.observe(node, 1.0)
        malformed = slope.CalibrationSample(
            quaternion=(math.nan, 0.0, 0.0, 1.0),
            acceleration=gravity(),
            angular_velocity=(0.0, 0.0, 0.0),
            source_stamp_s=1.005,
            imu_to_base_quaternion=(0.0, 0.0, 0.0, 1.0),
            imu_to_base_translation=(0.0, 0.0, 0.0),
        )
        node._observe_calibration_sample(
            malformed, True, (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.0
        )
        self.assertEqual(node._calibration_samples, [])
        self.assertEqual(node.core.calibration_state, slope.CAL_CALIBRATING)

        for index in range(2001):
            self.observe(node, 2.0 + index / 200.0)
        self.assertEqual(node.core.calibration_state, slope.CAL_VALID)
        self.assertEqual(node._calibration_samples, [])

    def test_overlong_candidate_trims_to_current_sample(self):
        node = self.make_calibrating_node()
        self.observe(node, 1.0)
        self.observe(node, 11.005001)
        self.assertEqual(
            [sample.source_stamp_s for sample in node._calibration_samples],
            [11.005001],
        )
        self.assertEqual(node.core.calibration_state, slope.CAL_CALIBRATING)

    def test_overshot_crossing_trims_to_valid_suffix_and_calibrates(self):
        node = self.make_calibrating_node()
        for index in range(1900):
            self.observe(node, 1.0 + 9.99 * index / 1899)
        self.assertEqual(len(node._calibration_samples), 1900)
        self.assertLess(
            node._calibration_samples[-1].source_stamp_s
            - node._calibration_samples[0].source_stamp_s,
            10.0,
        )

        self.observe(node, 11.01)
        self.assertEqual(node.core.calibration_state, slope.CAL_VALID)
        self.assertEqual(node._calibration_samples, [])

    def test_live_collector_calibrates_jittered_dropped_window(self):
        node = self.make_calibrating_node()
        for stamp in self.jittered_stamps():
            self.observe(node, stamp)
        self.assertEqual(node.core.calibration_state, slope.CAL_VALID)
        self.assertEqual(node._calibration_samples, [])

    def test_live_collector_keeps_p95_eligible_outliers(self):
        node = self.make_calibrating_node()
        for index in range(1900):
            self.observe(
                node,
                1.0 + 10.0 * index / 1899,
                magnitude=11.0 if index < 95 else 9.80665,
            )
        self.assertEqual(node.core.calibration_state, slope.CAL_VALID)
        self.assertEqual(node._calibration_samples, [])


    def test_timestamp_gap_is_retained_and_regression_restarts_candidate(self):
        node = self.make_calibrating_node()
        self.observe(node, 1.0)
        self.observe(node, 1.005)
        self.observe(node, 1.020)
        self.assertEqual(
            [sample.source_stamp_s for sample in node._calibration_samples],
            [1.0, 1.005, 1.020],
        )

        self.observe(node, 1.010)
        self.assertEqual(
            [sample.source_stamp_s for sample in node._calibration_samples],
            [1.010],
        )
        self.assertEqual(node.core.calibration_state, slope.CAL_CALIBRATING)

    def test_failed_complete_window_validation_is_retryable(self):
        node = self.make_calibrating_node()
        for index in range(2001):
            pitch = -1.0 if index % 2 else 1.0
            self.observe(node, 1.0 + index / 200.0, pitch=pitch)
        self.assertEqual(node.core.calibration_state, slope.CAL_CALIBRATING)
        self.assertEqual(node.core.calibration_sha256, "")
        self.assertEqual(node._calibration_samples, [])
        result = decide(node.core)
        self.assertEqual(result.recommended_max_linear_mps, 0.0)
        self.assertTrue(result.reason_mask & slope.IMU_UNCALIBRATED)


class PublicationCadenceTests(unittest.TestCase):
    def test_two_hundred_hz_input_is_bounded_below_gate_rate(self):
        published = []
        last_stamp = None
        for index in range(201):
            stamp = index / 200.0
            if slope.publication_due(last_stamp, stamp, 1.0 / 20.0):
                published.append(stamp)
                last_stamp = stamp
        self.assertEqual(published, [index / 20.0 for index in range(21)])

    def test_regressed_or_nonfinite_time_is_not_hidden_by_rate_limit(self):
        self.assertTrue(slope.publication_due(1.0, 0.9, 0.02))
        self.assertTrue(slope.publication_due(1.0, float("nan"), 0.02))

class InitialZonePolicyTests(unittest.TestCase):
    def test_simulation_allow_normalizes_only_in_simulation(self):
        self.assertEqual(
            slope.normalize_initial_zone_policy("simulation_allow", "simulation"),
            "normal",
        )
        for mode in ("hardware", "replay", "unverified"):
            zone = slope.normalize_initial_zone_policy("simulation_allow", mode)
            self.assertEqual(zone, "unknown")
            self.assertEqual(decide(valid_core(), zone=zone).state, slope.STOP)

    def test_unknown_and_unrecognized_initial_zones_remain_stop(self):
        for value in ("unknown", "normal", "not_in_manifest"):
            zone = slope.normalize_initial_zone_policy(value, "simulation")
            self.assertEqual(zone, "unknown")
            self.assertEqual(decide(valid_core(), zone=zone).state, slope.STOP)


class PublicationPairingTests(unittest.TestCase):
    class Message:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    def setUp(self):
        self.node = slope.SlopeSupervisorRosNode.__new__(slope.SlopeSupervisorRosNode)
        self.node.SlopeStatus = self.Message
        self.node.SafetySignal = self.Message
        self.node.status_pub = self.Publisher()
        self.node.signal_pub = self.Publisher()

    def assert_paired(self, source_stamp, evaluation_stamp):
        status = self.node.status_pub.messages[-1]
        signal = self.node.signal_pub.messages[-1]
        self.assertEqual(status.header.stamp, source_stamp)
        self.assertEqual(status.evaluation_stamp, evaluation_stamp)
        self.assertEqual(signal.header.stamp, evaluation_stamp)
        self.assertIsNot(status.header, signal.header)
        self.assertEqual(
            (signal.sequence, signal.reason_mask, signal.source, signal.policy_sha256),
            (status.sequence, status.reason_mask, status.source, status.policy_sha256),
        )
        self.assertTrue(math.isfinite(status.input_age_s))
        self.assertTrue(math.isfinite(status.transform_age_s))

    def test_clear_and_fail_closed_publications_share_abi_identity(self):
        clear = decide(valid_core(), now=1.0)
        self.node._publish_decision(clear, 0.9, 1.0)
        self.assert_paired(0.9, 1.0)

        unavailable = decide(valid_core(), now=2.0, transform_age_s=math.inf)
        self.node._publish_decision(unavailable, 1.9, 2.0)
        self.assert_paired(1.9, 2.0)
        self.assertEqual(self.node.status_pub.messages[-1].transform_age_s, -1.0)
        self.assertEqual(self.node.status_pub.messages[-1].input_age_s, -1.0)

class TrustSealTests(unittest.TestCase):
    def test_policy_requires_current_launch_hash(self):
        policy_path = Path(__file__).parents[1] / "config" / "slope_policy.yaml"
        digest = hashlib.sha256(policy_path.read_bytes()).hexdigest()
        policy = slope.load_slope_policy(str(policy_path), digest)
        self.assertEqual(policy.policy_sha256, digest)
        with self.assertRaises(ValueError):
            slope.load_slope_policy(str(policy_path), "0" * 64)
        with self.assertRaises(ValueError):
            slope.load_slope_policy("/definitely/missing/slope_policy.yaml", digest)

    def test_future_tf_and_stale_zone_stop(self):
        self.assertEqual(decide(valid_core(), transform_age_s=-0.001).state, slope.UNKNOWN)
        self.assertEqual(decide(valid_core(), zone_age_s=0.100001).state, slope.STOP)

    def test_sealed_extrinsic_translation_and_provenance_must_match(self):
        core = slope.SlopeSupervisorCore()
        helper = CalibrationTests()
        self.assertTrue(helper.calibrate(core, helper.make_samples()))
        self.assertEqual(decide(core, imu_to_base_quaternion=(0.0, 0.0, 0.0, -1.0)).state, slope.CLEAR)
        self.assertEqual(decide(core, imu_to_base_translation=(0.001, 0.0, 0.0)).state, slope.UNKNOWN)
        self.assertEqual(decide(core, input_provenance="gazebo:/other:imu_link").state, slope.UNKNOWN)


class PublicationRateTests(unittest.TestCase):
    def test_fifty_hz_publication_period(self):
        self.assertTrue(slope.publication_due(1.0, 1.02, 0.02))
        self.assertFalse(slope.publication_due(1.0, 1.019, 0.02))

if __name__ == "__main__":
    unittest.main()
