import math
import unittest
from dataclasses import replace
from pathlib import Path

import yaml

from wheelchair_navigation.speed_policy import (
    SpeedEvidence,
    SpeedPolicyConfig,
    SpeedPolicyCore,
    SpeedPolicyError,
)


SLOPE_HASH = "171d0febf5f3a691d1500d7b7839ef8f4a04637545b79dcb95d825bead7f6d0d"
COLLISION_POLICY = Path(__file__).parents[2] / "wheelchair_safety" / "config" / "collision_policy.yaml"
COLLISION_HASH = yaml.safe_load(COLLISION_POLICY.read_text(encoding="utf-8"))["policy_sha256"]
LOCALIZATION_HASH = "5d84ea824c98a53639a480ed162a62f015600ca0a0460df7186d5839303d52e8"

CONFIG = Path(__file__).parents[1] / "config" / "speed_policy.yaml"


def policy_mapping():
    return {
        "schema_version": 1,
        "policy_id": "speed-policy-sim-v1",
        "qualification": "simulation_only",
        "hardware_motion_authorized": False,
        "passenger_operation_authorized": False,
        "max_authority_cap_mps": 0.70,
        "sidewalk_cap_mps": 0.50,
        "road_cap_mps": 0.35,
        "simulation_unsurveyed_cap_mps": 0.20,
        "localization_uncertain_confidence": 0.65,
        "localization_stop_confidence": 0.30,
        "localization_uncertain_cap_mps": 0.10,
        "lateral_acceleration_max_mps2": 0.35,
        "curvature_epsilon_inv_m": 0.001,
        "slope_slow_cap_mps": 0.10,
        "collision_caution_cap_mps": 0.10,
        "collision_caution_ttc_s": 3.0,
        "collision_stop_ttc_margin_s": 0.50,
        "minimum_deceleration_mps2": 0.50,
        "driver_latency_s": 0.05,
        "pipeline_budget_s": 0.09,
        "acceleration_limit_mps2": 0.25,
        "deceleration_limit_mps2": 0.50,
        "jerk_limit_mps3": 1.0,
        "evidence_ttl_s": 0.10,
        "slope_policy_sha256": SLOPE_HASH,
        "collision_policy_sha256": COLLISION_HASH,
        "localization_policy_sha256": LOCALIZATION_HASH,
    }


def evidence(**changes):
    value = SpeedEvidence(
        segment_cap_mps=0.70,
        zone_cap_mps=0.70,
        hard_cap_mps=0.70,
        curvature_inv_m=0.0,
        zone="sidewalk",
        localization_state=2,
        localization_confidence=1.0,
        localization_policy_sha256=LOCALIZATION_HASH,
        slope_state=1,
        pitch_rad=0.0,
        slope_recommended_cap_mps=0.70,
        slope_policy_sha256=SLOPE_HASH,
        collision_state=1,
        collision_ttc_s=-1.0,
        collision_recommended_cap_mps=0.70,
        collision_policy_sha256=COLLISION_HASH,
        odometry_speed_mps=0.0,
        evidence_stamp_monotonic_s=10.0,
        now_monotonic_s=10.0,
    )
    return replace(value, **changes)


class SpeedPolicyConfigTests(unittest.TestCase):
    def test_closed_versioned_simulation_only_config(self):
        self.assertEqual(SpeedPolicyConfig.from_mapping(policy_mapping()).schema_version, 1)
        for key, value in (("extra", 1), ("schema_version", 2),
                           ("qualification", "hardware"),
                           ("hardware_motion_authorized", True),
                           ("passenger_operation_authorized", True)):
            raw = policy_mapping()
            raw[key] = value
            with self.assertRaises(SpeedPolicyError):
                SpeedPolicyConfig.from_mapping(raw)
        raw = policy_mapping()
        del raw["road_cap_mps"]
        with self.assertRaises(SpeedPolicyError):
            SpeedPolicyConfig.from_mapping(raw)
        raw = policy_mapping()
        raw["simulation_unsurveyed_cap_mps"] = 0.21
        with self.assertRaises(SpeedPolicyError):
            SpeedPolicyConfig.from_mapping(raw)

    def test_committed_policy_binds_embedded_collision_policy_identity(self):
        speed_policy = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        collision_policy = yaml.safe_load(COLLISION_POLICY.read_text(encoding="utf-8"))
        self.assertEqual(speed_policy["collision_policy_sha256"], collision_policy["policy_sha256"])

    def test_committed_policy_allows_one_10hz_period_plus_bounded_jitter(self):
        raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        config = SpeedPolicyConfig.from_mapping(raw)
        self.assertGreater(config.evidence_ttl_s, 0.10)
        self.assertLessEqual(config.evidence_ttl_s, 0.15)
        self.assertFalse(config.hardware_motion_authorized)
        self.assertFalse(config.passenger_operation_authorized)


class SpeedPolicyCoreTests(unittest.TestCase):
    def setUp(self):
        self.config = SpeedPolicyConfig.from_mapping(policy_mapping())

    def cap(self, item):
        return SpeedPolicyCore(self.config).evaluate(item)

    def test_authority_segment_zone_and_surface_caps_compose_by_minimum(self):
        self.assertEqual(self.cap(evidence(segment_cap_mps=0.42)), 0.42)
        self.assertEqual(self.cap(evidence(zone_cap_mps=0.31)), 0.31)
        self.assertEqual(self.cap(evidence(hard_cap_mps=0.29)), 0.29)
        self.assertEqual(self.cap(evidence(zone="road")), 0.35)
        combined = evidence(segment_cap_mps=0.44, zone_cap_mps=0.33,
                            hard_cap_mps=0.55, zone="road")
        self.assertEqual(self.cap(combined), 0.33)

    def test_simulation_unsurveyed_cap_is_conservative_and_composes_by_minimum(self):
        self.assertLessEqual(self.config.simulation_unsurveyed_cap_mps, 0.20)
        self.assertEqual(self.cap(evidence(zone="simulation_unsurveyed")), 0.20)
        self.assertEqual(
            self.cap(evidence(zone="simulation_unsurveyed", segment_cap_mps=0.12)),
            0.12,
        )
        self.assertFalse(self.config.hardware_motion_authorized)
        self.assertFalse(self.config.passenger_operation_authorized)

    def test_curvature_formula_and_epsilon_boundary(self):
        expected = math.sqrt(0.35 / 2.0)
        self.assertAlmostEqual(self.cap(evidence(curvature_inv_m=-2.0)), expected)
        self.assertEqual(self.cap(evidence(curvature_inv_m=0.001)), 0.50)

    def test_localization_uncertainty_and_loss(self):
        self.assertEqual(self.cap(evidence(localization_confidence=0.649)), 0.10)
        self.assertEqual(self.cap(evidence(localization_state=3)), 0.10)
        self.assertEqual(self.cap(evidence(localization_confidence=0.30)), 0.0)
        self.assertEqual(self.cap(evidence(localization_state=4)), 0.0)

    def test_slope_class_pitch_boundaries_and_downhill_braking(self):
        self.assertEqual(self.cap(evidence(pitch_rad=math.radians(7.0))), 0.50)
        self.assertEqual(self.cap(evidence(pitch_rad=math.radians(7.0001))), 0.10)
        self.assertEqual(self.cap(evidence(pitch_rad=math.radians(10.0))), 0.10)
        self.assertEqual(self.cap(evidence(pitch_rad=math.radians(10.0001))), 0.0)
        self.assertEqual(self.cap(evidence(slope_state=2)), 0.10)
        self.assertEqual(self.cap(evidence(slope_state=3)), 0.0)
        self.assertEqual(self.cap(evidence(pitch_rad=math.radians(-4.0))), 0.0)

    def test_ttc_exact_stop_and_caution_boundaries(self):
        # At zero speed and zero evidence age: 0.05 + 0.09 + 0.50 = 0.64 s.
        self.assertEqual(self.cap(evidence(collision_ttc_s=0.64)), 0.0)
        self.assertEqual(self.cap(evidence(collision_ttc_s=3.0)), 0.10)
        self.assertEqual(self.cap(evidence(collision_state=2, collision_ttc_s=4.0)), 0.10)
        self.assertEqual(
            self.cap(evidence(collision_state=2, collision_ttc_s=-1.0,
                              collision_recommended_cap_mps=0.10)),
            0.10,
        )
        self.assertEqual(self.cap(evidence(collision_state=3, collision_ttc_s=4.0)), 0.0)

    def test_recommended_caps_participate_in_minimum(self):
        item = evidence(slope_recommended_cap_mps=0.28,
                        collision_recommended_cap_mps=0.19,
                        segment_cap_mps=0.21)
        self.assertEqual(self.cap(item), 0.19)

    def test_clear_negative_one_recommendations_are_omitted_from_caps(self):
        item = evidence(slope_recommended_cap_mps=-1.0,
                        collision_recommended_cap_mps=-1.0,
                        segment_cap_mps=0.42)
        self.assertEqual(self.cap(item), 0.42)

    def test_invalid_negative_recommendations_fail_closed(self):
        cases = (
            evidence(slope_recommended_cap_mps=-0.5),
            evidence(collision_recommended_cap_mps=-0.5),
            evidence(slope_state=2, slope_recommended_cap_mps=-1.0),
            evidence(collision_state=2, collision_ttc_s=4.0,
                     collision_recommended_cap_mps=-1.0),
        )
        for item in cases:
            self.assertEqual(self.cap(item), 0.0)

    def test_hazard_stop_is_exact_and_bypasses_comfort_ramp(self):
        core = SpeedPolicyCore(self.config)
        self.assertEqual(core.evaluate(evidence()), 0.50)
        stop = evidence(collision_state=3, collision_ttc_s=0.2,
                        evidence_stamp_monotonic_s=10.01, now_monotonic_s=10.01)
        self.assertEqual(core.evaluate(stop), 0.0)

    def test_acceleration_deceleration_and_jerk_are_asymmetric(self):
        core = SpeedPolicyCore(self.config)
        low = evidence(collision_recommended_cap_mps=0.10)
        self.assertEqual(core.evaluate(low), 0.10)
        high = evidence(evidence_stamp_monotonic_s=11.0, now_monotonic_s=11.0)
        self.assertAlmostEqual(core.evaluate(high), 0.35)  # +0.25 m/s2
        lower = evidence(collision_recommended_cap_mps=0.10,
                         evidence_stamp_monotonic_s=11.1, now_monotonic_s=11.1)
        self.assertAlmostEqual(core.evaluate(lower), 0.365)  # acceleration changes by at most 0.1
        lower = replace(lower, evidence_stamp_monotonic_s=11.2, now_monotonic_s=11.2)
        self.assertAlmostEqual(core.evaluate(lower), 0.37)  # then +0.05 m/s2
        lower = replace(lower, evidence_stamp_monotonic_s=11.3, now_monotonic_s=11.3)
        self.assertAlmostEqual(core.evaluate(lower), 0.365)  # now decelerating

    def test_stale_nonfinite_hash_reverse_and_escalation_fail_closed(self):
        cases = (
            evidence(evidence_stamp_monotonic_s=9.899),
            evidence(curvature_inv_m=float("nan")),
            evidence(slope_policy_sha256="0" * 64),
            evidence(collision_policy_sha256="0" * 64),
            evidence(localization_policy_sha256="0" * 64),
            evidence(odometry_speed_mps=-0.01),
            evidence(hard_cap_mps=0.71),
            evidence(zone="unknown"),
        )
        for item in cases:
            self.assertEqual(self.cap(item), 0.0)

    def test_time_reset_fails_closed_and_restarts_from_zero(self):
        core = SpeedPolicyCore(self.config)
        self.assertEqual(core.evaluate(evidence()), 0.50)
        reset = evidence(evidence_stamp_monotonic_s=9.0, now_monotonic_s=9.0)
        self.assertEqual(core.evaluate(reset), 0.0)
        resumed = evidence(segment_cap_mps=0.20,
                           evidence_stamp_monotonic_s=9.1, now_monotonic_s=9.1)
        self.assertAlmostEqual(core.evaluate(resumed), 0.01)

    def test_deterministic_replay_is_finite(self):
        sequence = [
            evidence(segment_cap_mps=0.20),
            evidence(segment_cap_mps=0.50, evidence_stamp_monotonic_s=10.2, now_monotonic_s=10.2),
            evidence(segment_cap_mps=0.15, evidence_stamp_monotonic_s=10.4, now_monotonic_s=10.4),
        ]
        first, second = SpeedPolicyCore(self.config), SpeedPolicyCore(self.config)
        output_a = [first.evaluate(item) for item in sequence]
        output_b = [second.evaluate(item) for item in sequence]
        self.assertEqual(output_a, output_b)
        self.assertTrue(all(math.isfinite(value) and value >= 0.0 for value in output_a))


if __name__ == "__main__":
    unittest.main()
