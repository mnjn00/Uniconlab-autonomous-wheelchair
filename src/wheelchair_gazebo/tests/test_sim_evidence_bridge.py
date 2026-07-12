#!/usr/bin/env python3
"""Pure/static tests for the simulation evidence bridge contract."""

import importlib.util
import math
import pathlib
import sys
import unittest
from types import SimpleNamespace


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
BRIDGE_PATH = PACKAGE / "scripts" / "sim_evidence_bridge.py"
POLICY_PATH = PACKAGE.parent / "wheelchair_navigation" / "config" / "localization_confidence_sim.yaml"
LAUNCH_PATH = PACKAGE.parent / "wheelchair_bringup" / "launch" / "sim_bringup.launch"


def load_bridge():
    spec = importlib.util.spec_from_file_location("sim_evidence_bridge_under_test", str(BRIDGE_PATH))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


bridge = load_bridge()
def controller_odom(**changes):
    values = {
        "frame_id": "odom",
        "child_frame_id": "base_footprint",
        "stamp_s": 1.0,
        "position": (0.0, 0.0, 0.0),
        "orientation": (0.0, 0.0, 0.0, 1.0),
        "linear": (0.0, 0.0, 0.0),
        "angular": (0.0, 0.0, 0.0),
        "pose_covariance": (0.0,) * 36,
        "twist_covariance": (0.0,) * 36,
    }
    values.update(changes)
    point = lambda xyz: SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2])
    quaternion = SimpleNamespace(
        x=values["orientation"][0],
        y=values["orientation"][1],
        z=values["orientation"][2],
        w=values["orientation"][3],
    )
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=values["frame_id"],
            stamp=SimpleNamespace(to_sec=lambda: values["stamp_s"]),
        ),
        child_frame_id=values["child_frame_id"],
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=point(values["position"]), orientation=quaternion
            ),
            covariance=values["pose_covariance"],
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=point(values["linear"]), angular=point(values["angular"])
            ),
            covariance=values["twist_covariance"],
        ),
    )



class PlanarPoseCompositionTest(unittest.TestCase):
    def test_identity_origin_preserves_planar_pose(self):
        yaw = 0.7
        result = bridge.compose_planar_pose(
            0.0, 0.0, 0.0, 1.25, -2.5, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        )
        self.assertAlmostEqual(result[0], 1.25)
        self.assertAlmostEqual(result[1], -2.5)
        self.assertAlmostEqual(result[2], math.sin(yaw / 2.0))
        self.assertAlmostEqual(result[3], math.cos(yaw / 2.0))

    def test_translation_and_rotation_are_composed_map_to_odom_first(self):
        result = bridge.compose_planar_pose(
            10.0, 20.0, math.pi / 2.0, 2.0, 3.0, 0.0, 1.0
        )
        self.assertAlmostEqual(result[0], 7.0)
        self.assertAlmostEqual(result[1], 22.0)
        self.assertAlmostEqual(result[2], math.sin(math.pi / 4.0))
        self.assertAlmostEqual(result[3], math.cos(math.pi / 4.0))

    def test_yaw_wrap_and_output_quaternion_normalization(self):
        odom_yaw = math.radians(20.0)
        result = bridge.compose_planar_pose(
            0.0,
            0.0,
            math.radians(170.0),
            0.0,
            0.0,
            4.0 * math.sin(odom_yaw / 2.0),
            4.0 * math.cos(odom_yaw / 2.0),
        )
        output_yaw = 2.0 * math.atan2(result[2], result[3])
        self.assertAlmostEqual(output_yaw, math.radians(-170.0))
        self.assertAlmostEqual(math.hypot(result[2], result[3]), 1.0)

    def test_nonfinite_and_malformed_inputs_fail_closed(self):
        for bad_value in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                bridge.compose_planar_pose(bad_value, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        with self.assertRaises(ValueError):
            bridge.compose_planar_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def test_required_origin_parameter_rejects_nonfinite_values(self):
        params = SimpleNamespace(get_param=lambda _name: math.nan)
        with self.assertRaises(ValueError):
            bridge._required_finite_parameter(params, "~map_origin_x")



class EvidenceCoreTest(unittest.TestCase):
    def healthy_core(self):
        core = bridge.EvidenceCore()
        core.record_clock(10.0, 100.0)
        core.record_odom(10.0, 99.98)
        core.record_controller(10.0, True)
        return core

    def test_fresh_clock_odom_and_controller_are_required(self):
        self.assertTrue(self.healthy_core().evaluate(10.1).healthy)

        stale_clock = self.healthy_core().evaluate(10.16)
        self.assertFalse(stale_clock.healthy)
        self.assertTrue(stale_clock.reason_mask & bridge.CLOCK_STALE)

        core = self.healthy_core()
        core.record_clock(10.16, 100.16)
        stale_odom = core.evaluate(10.21)
        self.assertFalse(stale_odom.healthy)
        self.assertTrue(stale_odom.reason_mask & bridge.ODOM_STALE)

        core = self.healthy_core()
        core.record_clock(10.49, 100.49)
        core.record_odom(10.49, 100.48)
        stale_controller = core.evaluate(10.51)
        self.assertFalse(stale_controller.healthy)
        self.assertTrue(stale_controller.reason_mask & bridge.CONTROLLER_STALE)

    def test_clock_regression_and_odom_skew_fail_closed(self):
        core = self.healthy_core()
        core.record_clock(10.05, 99.0)
        self.assertTrue(core.evaluate(10.06).reason_mask & bridge.CLOCK_STALE)

        core = self.healthy_core()
        core.record_odom(10.05, 101.0)
        self.assertTrue(core.evaluate(10.06).reason_mask & bridge.TIME_INVALID)
    def test_controller_odom_validation_is_exact_and_finite(self):
        self.assertTrue(bridge._valid_controller_odom(controller_odom()))
        self.assertFalse(
            bridge._valid_controller_odom(controller_odom(frame_id="map"))
        )
        self.assertFalse(
            bridge._valid_controller_odom(
                controller_odom(child_frame_id="base_link")
            )
        )
        self.assertFalse(
            bridge._valid_controller_odom(
                controller_odom(linear=(math.nan, 0.0, 0.0))
            )
        )
        self.assertFalse(
            bridge._valid_controller_odom(
                controller_odom(orientation=(0.0, 0.0, 0.0, 0.0))
            )
        )
        self.assertFalse(
            bridge._valid_controller_odom(
                controller_odom(pose_covariance=(0.0,) * 35)
            )
        )

    def test_valid_odom_is_republished_without_frame_or_pose_changes(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        message = controller_odom(position=(1.0, 2.0, 3.0))
        instance = bridge.SimEvidenceBridge.__new__(bridge.SimEvidenceBridge)
        instance._odom_lock = __import__("threading").Lock()
        instance._latest_odom = None
        instance.core = SimpleNamespace(record_odom=lambda *_args: None)
        instance.odom_pub = Publisher()
        instance._odom_callback(message)
        self.assertIs(instance.odom_pub.messages[0], message)
        self.assertEqual(instance.odom_pub.messages[0].header.frame_id, "odom")
        self.assertEqual(instance.odom_pub.messages[0].pose.pose.position.x, 1.0)



class StaticContractTest(unittest.TestCase):
    def test_map_hash_and_calibrated_metadata_are_paired(self):
        metadata = dict(bridge.DIAGNOSTIC_METADATA)
        self.assertEqual(bridge.MAP_ID, "hanyang_aegimun_loop")
        self.assertEqual(metadata["map_sha256"], bridge.MAP_SHA256)
        self.assertEqual(metadata["policy_sha256"], bridge.POLICY_SHA256)
        for key in ("scan_residual_m", "inlier_ratio", "innovation_nis", "ambiguity_ratio"):
            self.assertTrue(math.isfinite(float(metadata[key])))
        self.assertEqual(metadata["transferable_to_replay"], "false")
        self.assertEqual(metadata["transferable_to_hardware"], "false")
        self.assertEqual(metadata["hardware_motion_authorized"], "false")
        self.assertEqual(metadata["passenger_operation_authorized"], "false")


    def test_launch_binds_map_origin_to_all_three_spawn_arguments(self):
        text = LAUNCH_PATH.read_text(encoding="utf-8")
        self.assertIn('<param name="map_origin_x" value="$(arg spawn_x)"/>', text)
        self.assertIn('<param name="map_origin_y" value="$(arg spawn_y)"/>', text)
        self.assertIn('<param name="map_origin_yaw" value="$(arg spawn_yaw)"/>', text)
    def test_covariance_is_finite_positive_planar_and_six_by_six(self):
        covariance = bridge.POSE_COVARIANCE
        self.assertEqual(len(covariance), 36)
        self.assertTrue(all(math.isfinite(value) for value in covariance))
        self.assertGreater(covariance[0], 0.0)
        self.assertGreater(covariance[7], 0.0)
        self.assertGreater(covariance[35], 0.0)
        self.assertLess(math.sqrt(covariance[0]), 0.20)
        self.assertLess(math.sqrt(covariance[35]), math.radians(5.0))

    def test_status_pair_uses_one_sequence_hash_and_health_decision(self):
        source = BRIDGE_PATH.read_text(encoding="utf-8")
        method = source[source.index("    def _publish_status_pair"):source.index("    def spin")]
        self.assertIn("signal.sequence = driver.sequence", method)
        self.assertIn("signal.header = driver.header", method)
        self.assertIn("signal.policy_sha256 = POLICY_SHA256", method)
        self.assertIn("driver.contract_sha256 = POLICY_SHA256", method)
        self.assertIn("if state.healthy else self.SafetySignal.STOP", method)

    def test_status_pair_identity_is_bounded_exact_and_unhealthy_is_inert(self):
        class DriverStatus(SimpleNamespace):
            AUTO_READY = 1
            AUTO_DISABLED = 2
            def __init__(self):
                super().__init__(header=SimpleNamespace())

        class SafetySignal(SimpleNamespace):
            CLEAR = 1
            STOP = 2

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        instance = bridge.SimEvidenceBridge.__new__(bridge.SimEvidenceBridge)
        instance.DriverStatus = DriverStatus
        instance.SafetySignal = SafetySignal
        instance.mode_pub = Publisher()
        instance.driver_signal_pub = Publisher()
        instance.driver_pub = Publisher()
        instance._sequence = 0
        instance._odom_lock = __import__("threading").Lock()
        instance._latest_odom = None
        instance.core = SimpleNamespace(
            limits=bridge.FreshnessLimits(),
        )
        stamp = object()
        state = bridge.EvidenceState(
            healthy=False,
            reason_mask=bridge.CLOCK_STALE,
            clock_stamp_s=None,
            odom_stamp_s=None,
            odom_generation=0,
        )

        instance._publish_status_pair(state, stamp)

        driver = instance.driver_pub.messages[0]
        signals = (
            instance.mode_pub.messages[0],
            instance.driver_signal_pub.messages[0],
        )
        self.assertTrue(bridge.SOURCE.isprintable())
        self.assertTrue(bridge.SOURCE)
        self.assertLessEqual(len(bridge.SOURCE.encode("utf-8")), 64)
        self.assertEqual(driver.state, DriverStatus.AUTO_DISABLED)
        self.assertFalse(driver.enabled)
        for signal in signals:
            self.assertEqual(signal.state, SafetySignal.STOP)
            self.assertIs(signal.header.stamp, stamp)
            self.assertEqual(signal.sequence, driver.sequence)
            self.assertEqual(signal.source, driver.source)
            self.assertEqual(signal.reason_mask, driver.reason_mask)
            self.assertEqual(signal.policy_sha256, driver.contract_sha256)
        self.assertEqual(driver.source, bridge.SOURCE)

    def test_graph_has_one_raw_controller_odom_input_and_canonical_odom_output(self):
        self.assertEqual(
            set(bridge.OBSERVED_TOPICS),
            {"/clock", "/wheelchair_base_controller/odom"},
        )
        self.assertIn("/odom", bridge.PUBLISHED_TOPICS)
        all_topics = set(bridge.OBSERVED_TOPICS) | set(bridge.PUBLISHED_TOPICS)
        forbidden = ("cmd_vel", "command", "motor", "estop", "e_stop", "reset")
        for topic in all_topics:
            self.assertFalse(any(token in topic.lower() for token in forbidden), topic)

    def test_policy_is_explicitly_nontransferable_and_hardware_false(self):
        text = POLICY_PATH.read_text(encoding="utf-8")
        self.assertIn("qualification_scope: gazebo_synthetic_ground_truth_calibration_only", text)
        self.assertIn("map_to_odom_assumption: identity_simulation_only", text)
        for field in (
            "hardware_motion_authorized",
            "passenger_operation_authorized",
            "transferable_to_replay",
            "transferable_to_hardware",
            "permits_grounded_actuation",
        ):
            self.assertRegex(text, r"(?m)^  %s: false$" % field)
        self.assertIn("may_report_ok: true", text)


if __name__ == "__main__":
    unittest.main()
