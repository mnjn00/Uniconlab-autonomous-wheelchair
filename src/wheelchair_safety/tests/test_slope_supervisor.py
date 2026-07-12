#!/usr/bin/env python3
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
        )
        node._observe_calibration_sample(
            sample,
            True,
            (0.0, 0.0, 0.0, 1.0),
            0.0,
        )

    def test_startup_transient_resets_candidate_without_poisoning_retry(self):
        node = self.make_calibrating_node()
        self.observe(node, 1.0)
        self.observe(node, 1.005, magnitude=12.0)
        self.assertEqual(node._calibration_samples, [])
        self.assertEqual(node.core.calibration_state, slope.CAL_CALIBRATING)

        for index in range(2001):
            self.observe(node, 2.0 + index / 200.0)
        self.assertEqual(node.core.calibration_state, slope.CAL_VALID)
        self.assertEqual(node._calibration_samples, [])

    def test_candidate_memory_is_bounded_during_high_rate_startup(self):
        node = self.make_calibrating_node()
        maximum_samples = (
            math.ceil(
                node.core.policy.calibration_rate_hz
                * (
                    node.core.policy.calibration_duration_s
                    + 1.0 / node.core.policy.calibration_rate_hz
                )
            )
            + 1
        )
        for index in range(maximum_samples + 5):
            self.observe(node, 1.0 + index / 1000.0)
        self.assertLessEqual(len(node._calibration_samples), maximum_samples)
        self.assertEqual(node.core.calibration_state, slope.CAL_CALIBRATING)

    def test_timestamp_gap_and_regression_restart_consecutive_window(self):
        node = self.make_calibrating_node()
        self.observe(node, 1.0)
        self.observe(node, 1.005)
        self.observe(node, 1.020)
        self.assertEqual(
            [sample.source_stamp_s for sample in node._calibration_samples],
            [1.020],
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

if __name__ == "__main__":
    unittest.main()
