#!/usr/bin/env python3
"""Focused ROS-independent tests for the RC metrics evidence contract."""
import importlib.util
import hashlib
import json
import math
import pathlib
import sys
import tempfile
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "rc_metrics_collector.py"


def load_collector():
    spec = importlib.util.spec_from_file_location("rc_metrics_collector_under_test", str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


collector = load_collector()


class MetricsCoreTest(unittest.TestCase):
    def observe_command(self, core, stream, stamp, *components):
        if not components:
            components = (0.0,) * 6
        core.observe_command(stream, stamp, *components)

    def fault_event(self, core, fault_id, phase, stamp, detail="test"):
        payload = json.dumps({
            "schema": collector.FAULT_EVENT_SCHEMA,
            "fault_id": fault_id,
            "phase": phase,
            "stamp_s": stamp,
            "detail": detail,
        }, sort_keys=True, separators=(",", ":"))
        core.observe_fault_event(payload)

    def complete_fault_core(self, fault_id="lidar_loss", reason_mask=4096,
                            zero_stamp=1.32, post_zero_nonzero=False,
                            reset=True, persist_after_reset=True):
        core = self.complete_core(fault_id)
        self.fault_event(core, fault_id, "ready", 1.21)
        self.fault_event(core, fault_id, "triggered", 1.22)
        core.observe_status("safety", 1.23, 3, reason_mask, (2, 3, 4), True)
        self.observe_command(core, "actuator_sink", zero_stamp)
        if post_zero_nonzero:
            self.observe_command(
                core, "actuator_sink", zero_stamp + 0.01,
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0)
        if reset:
            self.fault_event(core, fault_id, "reset_attempted", 1.34)
            if persist_after_reset:
                core.observe_status("safety", 1.35, 3, reason_mask, (2, 3, 4), True)
        self.fault_event(core, fault_id, "completed", 1.36)
        core.observe_route(1.36, 4, 0.0, 0.2)
        return core

    def complete_core(self, fault_id="none"):
        core = collector.MetricsCore(fault_id=fault_id)
        core.observe_clock(1.0)
        core.observe_pose(1.0, 0.0, 0.0, 0.0)
        core.observe_contacts(1.0, 0)
        core.observe_route(1.0, 1, 0.10, 1.0)
        for name, state in (("localization", 2), ("collision", 1),
                            ("geofence", 1), ("slope", 1), ("safety", 1)):
            core.observe_status(name, 1.0, state)
        core.observe_collision_ttc(3.0)
        self.observe_command(core, "nav_command", 1.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.observe_command(core, "safe_command", 1.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        core.observe_clock(1.2)
        core.observe_pose(1.2, 0.1, 0.0, 0.0)
        core.observe_route(1.2, 3, -0.05, 0.1)
        for name, state in (("localization", 2), ("collision", 1),
                            ("geofence", 1), ("slope", 1), ("safety", 1)):
            core.observe_status(name, 1.2, state)
        self.observe_command(core, "nav_command", 1.2)
        self.observe_command(core, "safe_command", 1.2)
        return core

    def test_complete_finite_evidence_is_deterministic(self):
        result = self.complete_core().finalize()
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(result["route_outcome"], "completed")
        self.assertEqual(result["missing_topics"], [])
        self.assertAlmostEqual(result["cross_track_m"]["max"], 0.10)
        self.assertAlmostEqual(result["goal_error_yaw_deg"], 0.0)
        self.assertTrue(result["command"]["caps_respected"])

    def test_missing_timeout_nonfinite_and_clock_regression_fail_closed(self):
        core = collector.MetricsCore()
        core.observe_clock(2.0)
        core.observe_clock(1.0)
        self.observe_command(core, "safe_command", 1.0, math.nan, 0.0, 0.0, 0.0, 0.0, 0.0)
        result = core.finalize(timed_out=True)
        self.assertFalse(result["passed"])
        self.assertIn("ground_truth", result["missing_topics"])
        self.assertIn("absent terminal route evidence", result["failures"])
        self.assertIn("collector timeout", result["failures"])
        self.assertEqual(result["timestamps"]["clock_regressions"], 1)
        self.assertFalse(result["command"]["finite"])
        self.assertEqual(result["command"]["nonfinite_component_count"], 1)

    def test_every_twist_axis_rejects_nan_and_infinity_on_both_streams(self):
        for stream in ("nav_command", "safe_command"):
            for axis_index, axis in enumerate(collector.TWIST_AXES):
                for invalid in (math.nan, math.inf, -math.inf):
                    with self.subTest(stream=stream, axis=axis, invalid=invalid):
                        core = self.complete_core()
                        components = [0.0] * 6
                        components[axis_index] = invalid
                        self.observe_command(core, stream, 1.3, *components)
                        result = core.finalize()
                        reason = "nonfinite.{}.{}".format(stream, axis)
                        self.assertFalse(result["passed"])
                        self.assertFalse(result["command"]["finite"])
                        self.assertEqual(result["command"]["nonfinite_component_count"], 1)
                        self.assertEqual(result["command"]["violation_reasons"][reason], 1)

    def test_unsupported_axes_must_be_exact_zero_on_both_streams(self):
        for stream in ("nav_command", "safe_command"):
            for axis in collector.UNSUPPORTED_TWIST_AXES:
                with self.subTest(stream=stream, axis=axis):
                    core = self.complete_core()
                    components = [0.0] * 6
                    components[collector.TWIST_AXES.index(axis)] = 1e-300
                    self.observe_command(core, stream, 1.3, *components)
                    result = core.finalize()
                    reason = "unsupported_axis_nonzero.{}.{}".format(stream, axis)
                    self.assertFalse(result["passed"])
                    self.assertFalse(result["command"]["shape_respected"])
                    self.assertFalse(result["command"]["caps_respected"])
                    self.assertEqual(result["command"]["unsupported_axis_nonzero_count"], 1)
                    self.assertEqual(result["command"]["violation_reasons"][reason], 1)

    def test_signed_zero_is_accepted_on_every_axis_and_both_streams(self):
        core = self.complete_core()
        signed_zeros = (-0.0, 0.0, -0.0, 0.0, -0.0, 0.0)
        self.observe_command(core, "nav_command", 1.4, *signed_zeros)
        self.observe_command(core, "safe_command", 1.4, *signed_zeros)
        result = core.finalize()
        self.assertTrue(result["passed"], result["failures"])
        self.assertTrue(result["command"]["finite"])
        self.assertTrue(result["command"]["shape_respected"])
        self.assertEqual(result["command"]["violation_reasons"], {})

    def test_linear_and_angular_caps_apply_to_both_streams(self):
        cases = (("linear.x", 0.56), ("angular.z", 0.86))
        for stream in ("nav_command", "safe_command"):
            for axis, value in cases:
                with self.subTest(stream=stream, axis=axis):
                    core = self.complete_core()
                    components = [0.0] * 6
                    components[collector.TWIST_AXES.index(axis)] = value
                    self.observe_command(core, stream, 1.4, *components)
                    result = core.finalize()
                    reason = "cap_exceeded.{}.{}".format(stream, axis)
                    self.assertFalse(result["passed"])
                    self.assertFalse(result["command"]["caps_respected"])
                    self.assertEqual(result["command"]["cap_exceedance_count"], 1)
                    self.assertEqual(result["command"]["violation_reasons"][reason], 1)

    def test_fault_requires_exact_zero_command_within_envelope(self):
        core = self.complete_core()
        core.trigger_stop(1.3, "fault", 4)
        core.observe_pose(1.4, 0.2, 0.0, 0.0)
        self.observe_command(core, "safe_command", 1.4, 1e-12, 0.0, 0.0, 0.0, 0.0, 0.0)
        result = core.finalize()
        self.assertFalse(result["passed"])
        self.assertEqual(result["command"]["nonzero_after_fault"], 1)
        self.assertEqual(
            result["command"]["violation_reasons"]["post_stop_nonzero.safe_command"], 1)
        self.assertIn("nonzero safe command after stop trigger", result["failures"])

    def test_startup_stop_does_not_poison_later_motion_evidence(self):
        core = collector.MetricsCore()
        core.observe_status("safety", 1.0, 3, 1, (3,))
        self.assertIsNone(core.stop_trigger_s)
        self.observe_command(core, "safe_command", 1.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        core.observe_status("collision", 1.2, 3, 4, (3,))
        self.assertEqual(core.stop_trigger_s, 1.2)

    def test_ttc_sentinel_never_hides_a_later_finite_ttc(self):
        core = collector.MetricsCore()
        core.observe_collision_ttc(-1.0)
        self.assertEqual(core._minimum_ttc(), -1.0)
        core.observe_collision_ttc(2.5)
        core.observe_collision_ttc(1.5)
        self.assertEqual(core._minimum_ttc(), 1.5)
        core.observe_collision_ttc(-0.5)
        self.assertIn("invalid collision TTC sentinel", core.failures)

    def test_contact_evidence_is_separate_from_preventive_collision_stop(self):
        core = collector.MetricsCore()
        core.observe_status("collision", 1.0, 3, 4, (3,))
        self.assertEqual(core.footprint_collisions, 0)
        core.observe_contacts(1.0, 2)
        self.assertEqual(core.footprint_collisions, 2)

    def test_footprint_contact_and_geofence_margin_are_blocking(self):
        core = self.complete_core()
        core.observe_contacts(1.3, 1)
        core.observe_status("geofence", 1.3, 2, stop_states=(2, 3, 4))
        result = core.finalize()
        self.assertFalse(result["passed"])
        self.assertEqual(result["footprint_collisions"], 1)
        self.assertEqual(result["geofence_exits"], 1)
        self.assertIn("footprint contact observed", result["failures"])
        self.assertIn("geofence boundary violation", result["failures"])

    def test_safe_abort_does_not_require_terminal_goal_error(self):
        core = self.complete_core()
        core.observe_route(1.3, 4, 0.0, 0.2)
        core.goal_error_m = None
        core.goal_error_yaw_deg = None
        result = core.finalize()
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(result["route_outcome"], "safe_abort")
        self.assertIsNone(result["goal_error_m"])
        self.assertIsNone(result["goal_error_yaw_deg"])
    def test_contacts_are_required_live_topic_evidence(self):
        core = self.complete_core()
        core.seen.remove("contacts")
        result = core.finalize()
        self.assertFalse(result["passed"])
        self.assertIn("contacts", result["missing_topics"])

    def test_terminal_settle_time_cannot_be_reduced_below_point_six_seconds(self):
        self.assertEqual(collector.terminal_settle_time("0.60"), 0.60)
        with self.assertRaises(Exception):
            collector.terminal_settle_time("0.599")

    def test_atomic_artifact_is_strict_sorted_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "nested" / "evidence.json"
            collector.write_artifact(str(path), {"z": 1, "a": False})
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), {"a": False, "z": 1})
            self.assertTrue(path.read_text(encoding="utf-8").startswith('{"a":false'))

    def test_valid_fault_lifecycle_is_live_observation_backed(self):
        result = self.complete_fault_core().finalize()
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(result["fault_injected"], "lidar_loss")
        self.assertTrue(result["safe_abort"])
        self.assertTrue(result["zero_within_budget"])
        self.assertEqual(result["reason_events"], ["SENSOR_STALE"])
        self.assertTrue(result["latched_until_guarded_reset"])
        self.assertIn("fault_event", result["sample_counts"])
        self.assertIn("actuator_sink", result["sample_counts"])
        evidence = result["fault_evidence"]
        self.assertAlmostEqual(evidence["trigger_stamp_s"], 1.22)
        self.assertAlmostEqual(evidence["actuator_zero_stamp_s"], 1.32)
        self.assertAlmostEqual(evidence["actuator_zero_latency_s"], 0.10)
        self.assertAlmostEqual(evidence["actuator_zero_budget_s"], 0.15)
        self.assertEqual(
            evidence["reason_observations"],
            [{
                "source": "safety",
                "stream": "safety",
                "stamp_s": 1.23,
                "reason_mask": 4096,
                "names": ["SENSOR_STALE"],
            }, {
                "source": "safety",
                "stream": "safety",
                "stamp_s": 1.35,
                "reason_mask": 4096,
                "names": ["SENSOR_STALE"],
            }],
        )

    def test_malformed_fault_event_is_rejected_and_blocks_fault_identity(self):
        core = self.complete_fault_core()
        core.observe_fault_event(
            '{"detail":"test","fault_id":"lidar_loss","phase":"completed",'
            '"schema":"wheelchair.sim_fault/v1","stamp_s":1.4,"extra":true}')
        result = core.finalize()
        self.assertFalse(result["passed"])
        self.assertIsNone(result["fault_injected"])
        self.assertIn("malformed fault event", result["failures"])

    def test_missing_trigger_blocks_fault_evidence(self):
        core = collector.MetricsCore(fault_id="lidar_loss")
        self.fault_event(core, "lidar_loss", "ready", 1.0)
        result = core.finalize()
        self.assertIsNone(result["fault_injected"])
        self.assertFalse(result["zero_within_budget"])
        self.assertIn("missing triggered fault event", result["failures"])

    def test_wrong_fault_id_is_mismatched_evidence(self):
        core = collector.MetricsCore(fault_id="lidar_loss")
        self.fault_event(core, "imu_loss", "ready", 1.0)
        result = core.finalize()
        self.assertIsNone(result["fault_injected"])
        self.assertNotIn("fault_event", core.seen)
        self.assertIn("mismatched fault event", result["failures"])

    def test_reason_mask_expands_to_stable_safety_reason_names(self):
        result = self.complete_fault_core(
            reason_mask=1 | 4096 | 1073741824).finalize()
        self.assertEqual(
            result["reason_events"], ["ESTOP", "RESET_REJECTED", "SENSOR_STALE"])

    def test_actuator_zero_must_be_timely_and_remain_zero(self):
        timely = self.complete_fault_core(zero_stamp=1.369).finalize()
        self.assertTrue(timely["zero_within_budget"], timely["failures"])
        late = self.complete_fault_core(zero_stamp=1.371).finalize()
        self.assertFalse(late["zero_within_budget"])
        self.assertIn(
            "actuator sink zero response exceeded software budget", late["failures"])
        nonzero = self.complete_fault_core(post_zero_nonzero=True).finalize()
        self.assertFalse(nonzero["zero_within_budget"])
        self.assertEqual(nonzero["command"]["nonzero_after_fault"], 1)
        self.assertIn(
            "nonzero actuator sink command after fault trigger", nonzero["failures"])

    def test_reset_attempt_requires_observed_stop_persistence(self):
        no_persistence = self.complete_fault_core(persist_after_reset=False).finalize()
        self.assertFalse(no_persistence["latched_until_guarded_reset"])
        no_reset = self.complete_fault_core(reset=False).finalize()
        self.assertFalse(no_reset["latched_until_guarded_reset"])

    def test_normal_run_does_not_invent_or_require_fault_evidence(self):
        result = self.complete_core().finalize()
        self.assertTrue(result["passed"], result["failures"])
        for field in ("fault_injected", "safe_abort", "zero_within_budget",
                      "reason_events", "latched_until_guarded_reset",
                      "fault_evidence"):
            self.assertNotIn(field, result)
        self.assertNotIn("fault_event", result["missing_topics"])
        self.assertNotIn("actuator_sink", result["missing_topics"])

    def route_truth(self):
        return collector.DirectionalRouteTruth(
            "mission", "route", "map", "a" * 64, "b" * 64, "c" * 64,
            1, ((0.0, 0.0), (1.0, 0.0)), 0.2, 0.0)

    def test_forged_route_progress_cannot_replace_ground_truth(self):
        core = collector.MetricsCore()
        core.bind_route_truth(self.route_truth())
        core.observe_pose(1.0, 0.5, 0.2, 0.0)
        core.observe_route_evidence(1.0, 1, "mission", "route", "map", 1, 1.0,
                                    0.0, 0.5)
        self.assertIn("RouteProgress disagrees with Gazebo route truth", core.failures)

    def test_route_truth_rejects_wrong_identity_and_terminal_yaw_surrogate(self):
        core = collector.MetricsCore()
        core.bind_route_truth(self.route_truth())
        core.observe_pose(1.0, 1.0, 0.0, 0.2)
        core.observe_route_evidence(1.0, 3, "mission", "wrong", "map", 1, 1.0,
                                    0.0, 1.0)
        self.assertIn("invalid route identity or evidence", core.failures)
        core.observe_route_evidence(1.1, 3, "mission", "route", "map", 2, 1.0,
                                    0.0, 1.0)
        self.assertIn("RouteProgress COMPLETE before approved terminal truth", core.failures)

    def test_localization_truth_rejects_plausible_wrong_ok_candidate(self):
        core = collector.MetricsCore()
        core.bind_route_truth(self.route_truth())
        core.observe_pose(1.0, 0.0, 0.0, 0.0)
        core.observe_localization_pose(1.0, 0.3, 0.0, 0.0, "map", "a" * 64, "source")
        core.observe_localization_pose(1.1, 0.6, 0.0, 0.0, "map", "a" * 64, "source")
        result = core.finalize()
        self.assertFalse(result["passed"])
        self.assertIn("AC4 planar p95 exceeded", result["failures"])
    def test_ac4_p95_boundaries_and_invalid_interval_dwell(self):
        core = collector.MetricsCore()
        core.bind_route_truth(self.route_truth())
        core.observe_pose(1.0, 0.0, 0.0, 0.0)
        core.observe_localization_pose(1.0, 0.25, 0.0, math.radians(8.0), "map", "a" * 64, "source")
        core.observe_pose(1.2, 0.0, 0.0, 0.0)
        core.observe_localization_pose(1.2, 0.0, 0.0, 0.0, "map", "a" * 64, "source")
        self.assertEqual(core.localization_invalid_intervals, 0)
        core.observe_pose(2.0, 0.0, 0.0, 0.0)
        core.observe_localization_pose(2.0, 0.6, 0.0, 0.0, "map", "a" * 64, "source")
        core.observe_pose(2.6, 0.0, 0.0, 0.0)
        core.observe_localization_pose(2.6, 0.0, 0.0, 0.0, "map", "a" * 64, "source")
        self.assertEqual(core.localization_invalid_intervals, 1)

    def test_route_receipt_must_be_monotonic_and_fresh(self):
        core = collector.MetricsCore()
        core.bind_route_truth(self.route_truth())
        core.observe_pose(1.0, 0.5, 0.0, 0.0)
        core.observe_route_evidence(1.0, 1, "mission", "route", "map", 1, 1.0, 0.0, 0.5)
        core.observe_route_evidence(1.1, 1, "mission", "route", "map", 1, 1.1, 0.0, 0.5)
        self.assertIn("non-monotonic route sequence or source stamp", core.failures)
    def test_static_truth_requires_hash_bound_references(self):
        truth = PACKAGE / "config" / "route_truth_outbound.yaml"
        loaded = collector.load_route_truth(
            str(truth), "2ddbf2660ac98868732e14e5540abaa77c8b6a600e024f1d12a71dee06bebf1b",
            "rc-0123456789abcdef01234567")
        self.assertEqual(loaded.mission_id, "rc-0123456789abcdef01234567")
        self.assertEqual(loaded.corridor_clearance_m, 0.2)
        self.assertEqual(loaded.direction, 1)
    def test_derived_mission_identity_matches_driver_material(self):
        self.assertEqual(
            collector.derive_mission_id("qualification", 1701, "outbound",
                                        "hanyang_aegimun_engineering_outbound"),
            "rc-" + hashlib.sha256(
                b"qualification\n1701\noutbound\nhanyang_aegimun_engineering_outbound").hexdigest()[:24])
class CollectionLifecycleTest(unittest.TestCase):
    def test_terminal_settle_uses_monotonic_wall_deadline(self):
        decision = collector.collection_stop_reason(
            now=10.59, started=0.0, terminal_seen_wall=10.0,
            settle_time=0.60, timeout=30.0)
        self.assertIsNone(decision)
        self.assertEqual(
            collector.collection_stop_reason(
                now=10.60, started=0.0, terminal_seen_wall=10.0,
                settle_time=0.60, timeout=30.0),
            "terminal")

    def test_timeout_is_deterministic_when_no_clock_messages_arrive(self):
        self.assertEqual(
            collector.collection_stop_reason(
                now=30.0, started=0.0, terminal_seen_wall=None,
                settle_time=0.60, timeout=30.0),
            "timeout")


class StaticContractTest(unittest.TestCase):
    def test_ros_imports_are_lazy_and_authority_is_fixed_false(self):
        source = SCRIPT.read_text(encoding="utf-8")
        prefix = source[:source.index("class RosCollector")]
        self.assertNotIn("import rospy", prefix)
        self.assertIn('"hardware_motion_authorized": False', source)
        self.assertIn('"passenger_operation_authorized": False', source)
        self.assertNotIn("rospy.Publisher", source)
        self.assertNotIn("rospy.Rate", source)
        self.assertIn("time.sleep(WALL_POLL_INTERVAL_S)", source)


if __name__ == "__main__":
    unittest.main()
