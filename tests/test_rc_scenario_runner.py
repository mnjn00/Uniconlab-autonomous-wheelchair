"""ROS-free orchestration and fail-closed aggregation tests for Gazebo RC runners."""

import copy
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import run_fault_matrix
import run_gazebo_rc_suite


def load_metrics_collector():
    path = (
        REPOSITORY
        / "src"
        / "wheelchair_gazebo"
        / "scripts"
        / "rc_metrics_collector.py"
    )
    spec = importlib.util.spec_from_file_location(
        "rc_metrics_collector_registry_test", str(path)
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def good_scenario():
    return {
        "live_evidence": True,
        "passed": True,
        "simulation_only": True,
        "hardware_motion_authorized": False,
        "passenger_operation_authorized": False,
        "source_topics": list(run_gazebo_rc_suite.REQUIRED_TOPICS),
        "route_outcome": "completed",
        "cross_track_samples_m": [0.05, -0.10, 0.15],
        "goal_error_m": 0.10,
        "goal_error_yaw_deg": 2.0,
        "footprint_collisions": 0,
        "geofence_exits": 0,
        "command": {"finite": True, "caps_respected": True, "nonzero_after_fault": 0},
        "stop": {"envelope_respected": True, "minimum_ttc_s": 1.2},
        "hysteresis": {"stop_observed": True, "resume_after_clear": True,
                       "reason_events": ["COLLISION_STOP", "CLEAR_HYSTERESIS", "RESUMED"]},
    }


def good_fault(fault):
    reasons = sorted(fault["expected_reasons"])
    reason_bits = {
        name: bit for bit, name in run_gazebo_rc_suite.SAFETY_REASON_BITS.items()
    }
    reason_mask = 0
    for reason in reasons:
        reason_mask |= reason_bits[reason]
    return {
        "live_evidence": True,
        "simulation_only": True,
        "hardware_motion_authorized": False,
        "passenger_operation_authorized": False,
        "source_topics": list(run_gazebo_rc_suite.REQUIRED_TOPICS)
        + list(run_gazebo_rc_suite.FAULT_REQUIRED_TOPICS),
        "fault_injected": fault["id"],
        "safe_abort": True,
        "footprint_collisions": 0,
        "geofence_exits": 0,
        "command": {"finite": True, "caps_respected": True, "nonzero_after_fault": 0},
        "zero_within_budget": True,
        "reason_events": reasons,
        "latched_until_guarded_reset": True,
        "fault_evidence": {
            "trigger_stamp_s": 10.0,
            "actuator_zero_stamp_s": 10.1,
            "actuator_zero_latency_s": 0.1,
            "actuator_zero_budget_s": 0.15,
            "reason_observations": [{
                "source": "/safety/state",
                "stamp_s": 10.05,
                "reason_mask": reason_mask,
                "names": reasons,
            }],
        },
    }


class ScenarioOrchestrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = REPOSITORY / "src/wheelchair_gazebo/config/scenarios.yaml"
        cls.config = run_gazebo_rc_suite.load_config(cls.config_path)

    def backend_for_suite(self):
        values = {}
        for world in self.config["worlds"]:
            for seed in range(10):
                values[(world["id"], seed, False, None)] = good_scenario()
        robustness = next(world for world in self.config["worlds"] if world["id"] == self.config["robustness_world"])
        for seed in range(30):
            values[(robustness["id"], seed, True, None)] = good_scenario()
        return run_gazebo_rc_suite.FakeBackend(values)

    def test_runs_ten_repetitions_per_world_and_thirty_fixed_seeds(self):
        backend = self.backend_for_suite()
        result = run_gazebo_rc_suite.execute_suite(self.config, backend)
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(len(backend.calls), 70)
        robustness_calls = [call for call in backend.calls if call[2]]
        self.assertEqual([call[1] for call in robustness_calls], list(range(30)))
        self.assertEqual(result["summary"]["robustness_acceptable"], 30)

    def test_one_collision_fails_without_averaging_away(self):
        backend = self.backend_for_suite()
        bad_key = (self.config["worlds"][0]["id"], 7, False, None)
        backend.results[bad_key]["footprint_collisions"] = 1
        result = run_gazebo_rc_suite.execute_suite(self.config, backend)
        self.assertFalse(result["passed"])
        failed = [item for item in result["runs"] if not item["passed"]]
        self.assertEqual(len(failed), 1)
        self.assertIn("footprint_collisions is nonzero", failed[0]["failures"])

    def test_nonzero_after_fault_is_a_blocking_run_failure(self):
        raw = good_scenario()
        raw["command"]["nonzero_after_fault"] = 1
        result = run_gazebo_rc_suite.validate_result(raw, self.config["thresholds"], False)
        self.assertFalse(result["passed"])
        self.assertIn("nonzero command observed after fault", result["failures"])
    def test_completed_and_safe_abort_have_distinct_terminal_contracts(self):
        completed = run_gazebo_rc_suite.validate_result(
            good_scenario(), self.config["thresholds"], require_completion=True)
        self.assertTrue(completed["passed"], completed["failures"])

        safe_abort = good_scenario()
        safe_abort.update({
            "route_outcome": "safe_abort",
            "stop": {
                "envelope_respected": True,
                "minimum_ttc_s": 1.2,
                "trigger_stamp_s": 10.0,
                "zero_stamp_s": 10.1,
                "latency_s": 0.1,
                "overshoot_m": 0.01,
            },
        })
        del safe_abort["cross_track_samples_m"]
        del safe_abort["goal_error_m"]
        del safe_abort["goal_error_yaw_deg"]
        safe_abort["hysteresis"]["resume_after_clear"] = False
        safe_abort["hysteresis"]["reason_events"] = [{
            "event": "stop",
            "source": "route_invalid",
            "stamp_s": 10.0,
            "reason_mask": 0,
        }]
        result = run_gazebo_rc_suite.validate_result(
            safe_abort, self.config["thresholds"], require_completion=False)
        self.assertTrue(result["passed"], result["failures"])

    def test_collector_safety_failure_cannot_be_reported_as_pass(self):
        raw = good_scenario()
        raw["passed"] = False
        raw["missing_topics"] = ["contacts"]
        raw["verdicts"] = {"topics_complete": False}
        result = run_gazebo_rc_suite.validate_result(
            raw, self.config["thresholds"], require_completion=True)
        self.assertFalse(result["passed"])
        self.assertIn("collector reported blocking failures", result["failures"])
        self.assertIn("collector is missing required stream evidence", result["failures"])
    def test_config_selects_complete_bringup_and_live_contact_contract(self):
        self.assertEqual(
            self.config["launch"],
            {"package": "wheelchair_bringup", "file": "sim_bringup.launch"},
        )
        self.assertEqual(self.config["scenario_direction"], "outbound")
        self.assertEqual(
            self.config["canonical_topics"]["contacts"], "/simulation/contacts")
        self.assertIn("/simulation/contacts", run_gazebo_rc_suite.REQUIRED_TOPICS)

    def test_fault_reason_registry_matches_live_collector(self):
        collector = load_metrics_collector()
        self.assertEqual(
            run_gazebo_rc_suite.SAFETY_REASON_BITS,
            collector.SAFETY_REASON_BITS,
        )

    def assert_config_rejected(self, mutate, message=None):
        invalid = copy.deepcopy(self.config)
        mutate(invalid)
        with self.assertRaises(ValueError, msg=message):
            run_gazebo_rc_suite.validate_config(invalid)

    def test_committed_config_satisfies_frozen_ac5_matrix(self):
        self.assertIs(run_gazebo_rc_suite.validate_config(
            copy.deepcopy(self.config))["simulation_only"], True)

    def test_ac5_world_repetition_seed_and_acceptance_mutations_fail_closed(self):
        mutations = [
            ("world count", lambda value: value["worlds"].pop()),
            ("world ID", lambda value: value["worlds"][0].update(id="alternate")),
            ("repetition count", lambda value: value.update(deterministic_repetitions=9)),
            ("seed count", lambda value: value["robustness_seeds"].pop()),
            ("seed uniqueness", lambda value: value["robustness_seeds"].__setitem__(29, 28)),
            ("approved seed", lambda value: value["robustness_seeds"].__setitem__(29, 30)),
            ("minimum accepted", lambda value: value.update(minimum_robustness_acceptable=28)),
            ("accepted above seed count",
             lambda value: value.update(minimum_robustness_acceptable=31)),
            ("robustness world", lambda value: value.update(robustness_world="empty")),
            ("fault world", lambda value: value.update(fault_world="empty")),
        ]
        for name, mutate in mutations:
            with self.subTest(name=name):
                self.assert_config_rejected(mutate, name)

    def test_ac5_threshold_and_collector_cap_mutations_fail_closed(self):
        for field, ceiling in run_gazebo_rc_suite.APPROVED_CEILINGS.items():
            with self.subTest(field=field):
                self.assert_config_rejected(
                    lambda value, field=field, ceiling=ceiling:
                    value["thresholds"].__setitem__(field, ceiling + 0.01),
                    field,
                )

        def raise_collector_cap(value, option):
            index = value["collector_command"].index(option) + 1
            value["collector_command"][index] = "99.0"

        for option in ("--linear-cap-mps", "--angular-cap-rps"):
            with self.subTest(option=option):
                self.assert_config_rejected(
                    lambda value, option=option: raise_collector_cap(value, option),
                    option,
                )

    def test_fault_collector_topics_budget_and_single_options_are_frozen(self):
        mutations = [
            ("fault event topic", lambda value: value["collector_command"].__setitem__(
                value["collector_command"].index("--fault-event-topic") + 1,
                "/alternate/fault_event")),
            ("actuator topic", lambda value: value["collector_command"].__setitem__(
                value["collector_command"].index("--actuator-command-topic") + 1,
                "/alternate/cmd_vel")),
            ("stop budget", lambda value: value["collector_command"].__setitem__(
                value["collector_command"].index("--stop-budget-s") + 1, "0.16")),
            ("duplicate fault option", lambda value: value["collector_command"].extend([
                "--fault-event-topic", "/simulation/fault_event"])),
        ]
        for name, mutate in mutations:
            with self.subTest(name=name):
                self.assert_config_rejected(mutate, name)

    def test_ac5_fault_matrix_and_authority_mutations_fail_closed(self):
        mutations = [
            ("empty faults", lambda value: value.update(fault_matrix=[])),
            ("missing fault", lambda value: value["fault_matrix"].pop()),
            ("duplicate fault ID",
             lambda value: value["fault_matrix"][1].update(
                 id=value["fault_matrix"][0]["id"])),
            ("fault category",
             lambda value: value["fault_matrix"][0].update(category="alternate")),
            ("fault reasons",
             lambda value: value["fault_matrix"][0].update(
                 expected_reasons=["ALTERNATE"])),
            ("fault latch",
             lambda value: value["fault_matrix"][12].update(requires_latch=False)),
            ("missing fault latch field",
             lambda value: value["fault_matrix"][0].pop("requires_latch")),
            ("claim tag", lambda value: value.update(claim_tag="HARDWARE_READY")),
            ("simulation only", lambda value: value.update(simulation_only=False)),
            ("hardware authority",
             lambda value: value.update(hardware_motion_authorized=True)),
            ("passenger authority",
             lambda value: value.update(passenger_operation_authorized=True)),
        ]
        for name, mutate in mutations:
            with self.subTest(name=name):
                self.assert_config_rejected(mutate, name)

    def test_weakened_config_fails_before_live_launch_and_cannot_emit_pass(self):
        invalid = copy.deepcopy(self.config)
        invalid["thresholds"]["cross_track_mean_m"] = 0.21
        with tempfile.TemporaryDirectory() as temporary:
            config_path = Path(temporary) / "weakened.json"
            output = Path(temporary) / "report.json"
            config_path.write_text(json.dumps(invalid), encoding="utf-8")
            with mock.patch.object(
                    run_gazebo_rc_suite.RosGazeboBackend, "run_scenario") as live_run:
                code = run_gazebo_rc_suite.main([
                    "--config", str(config_path), "--output", str(output)])
            report = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(code, 1)
        live_run.assert_not_called()
        self.assertEqual(report["status"], "FAIL")
        self.assertNotEqual(report["status"], "PASS")

    def test_route_and_settle_timeout_configuration_rejects_impossible_runs(self):
        invalid = copy.deepcopy(self.config)
        timeout_index = invalid["collector_command"].index("--timeout") + 1
        invalid["collector_command"][timeout_index] = "1.0"
        with self.assertRaisesRegex(ValueError, "cannot finish selected"):
            run_gazebo_rc_suite.validate_config(invalid)

        invalid = copy.deepcopy(self.config)
        settle_index = invalid["collector_command"].index("--settle-time") + 1
        invalid["collector_command"][settle_index] = "0.59"
        with self.assertRaisesRegex(ValueError, "settle time"):
            run_gazebo_rc_suite.validate_config(invalid)

    def test_full_bringup_argv_and_graph_are_required_before_collector(self):
        backend = run_gazebo_rc_suite.RosGazeboBackend(self.config)
        process = mock.Mock()
        process.poll.return_value = None
        expected_nodes = "\n".join(self.config["required_nodes"])
        action_topics = [
            action + suffix
            for action in self.config["required_actions"]
            for suffix in ("/goal", "/cancel", "/status")
        ]
        expected_topics = "\n".join(
            list(self.config["canonical_topics"].values()) + action_topics)
        expected_services = "\n".join(self.config["required_services"])

        def completed(command, *args, **kwargs):
            if command[:2] == ["rospack", "find"]:
                return subprocess.CompletedProcess(
                    command, 0, "/workspace/wheelchair_gazebo\n", "")
            if command == ["rosnode", "list"]:
                return subprocess.CompletedProcess(command, 0, expected_nodes, "")
            if command == ["rostopic", "list"]:
                return subprocess.CompletedProcess(command, 0, expected_topics, "")
            if command == ["rosservice", "list"]:
                return subprocess.CompletedProcess(command, 0, expected_services, "")
            if command == ["rostopic", "echo", "-n", "1", "/clock"]:
                return subprocess.CompletedProcess(command, 0, "clock: 1.0\n", "")
            if command[:3] == [
                    "rosrun", "wheelchair_gazebo", "rc_metrics_collector.py"]:
                output = Path(command[command.index("--output") + 1])
                output.write_text(json.dumps(good_scenario()), encoding="utf-8")
                return subprocess.CompletedProcess(command, 0, "", "")
            raise AssertionError("unexpected command: {!r}".format(command))

        world = self.config["worlds"][0]
        with mock.patch.object(backend, "preflight"), \
                mock.patch.object(backend, "_stop_process"), \
                mock.patch.object(run_gazebo_rc_suite.subprocess, "run",
                                  side_effect=completed), \
                mock.patch.object(run_gazebo_rc_suite.subprocess, "Popen",
                                  return_value=process) as popen:
            backend.run_scenario(world, 7)

        expected_world = "/workspace/wheelchair_gazebo/worlds/empty.world"
        self.assertEqual(popen.call_args.args[0], [
            "roslaunch", "wheelchair_bringup", "sim_bringup.launch",
            "gui:=false", "headless:=true", "paused:=false", "auto_start:=true",
            "seed:=7", "scenario:=empty", "fault_id:=normal",
            "scenario_direction:=outbound",
            "world:=" + expected_world,
        ])

    def test_launch_argv_separates_scenario_identity_from_fault_selection(self):
        backend = run_gazebo_rc_suite.RosGazeboBackend(self.config)
        process = mock.Mock()
        process.poll.return_value = None
        world = {"id": "campus_loop", "file": "/tmp/campus.world"}

        cases = (
            ({"robustness": False, "fault": None}, "campus_loop", "normal"),
            ({"robustness": True, "fault": None}, "robustness", "normal"),
            ({"robustness": False, "fault": "lidar_loss"},
             "lidar_loss", "lidar_loss"),
        )
        for arguments, scenario, fault_id in cases:
            with self.subTest(scenario=scenario), \
                    mock.patch.object(backend, "preflight"), \
                    mock.patch.object(
                        backend, "_wait_live_graph",
                        side_effect=run_gazebo_rc_suite.EvidenceError("stop")), \
                    mock.patch.object(backend, "_stop_process"), \
                    mock.patch.object(
                        run_gazebo_rc_suite.subprocess, "Popen",
                        return_value=process) as popen:
                with self.assertRaises(run_gazebo_rc_suite.EvidenceError):
                    backend.run_scenario(world, 11, **arguments)

            command = popen.call_args.args[0]
            self.assertIn("scenario:={}".format(scenario), command)
            self.assertIn("fault_id:={}".format(fault_id), command)

    def test_idle_graph_is_behavioral_failure_with_launch_log_evidence(self):
        backend = run_gazebo_rc_suite.RosGazeboBackend(
            self.config, startup_timeout=0.0)
        process = mock.Mock()
        process.poll.return_value = None
        with self.assertRaisesRegex(
                run_gazebo_rc_suite.EvidenceError, "roslaunch log"):
            backend._wait_live_graph(process, "/tmp/qualification-launch.log")

    def test_early_roslaunch_exit_retains_log_and_is_not_platform_unavailable(self):
        backend = run_gazebo_rc_suite.RosGazeboBackend(self.config)
        process = mock.Mock()
        process.poll.return_value = 1
        world = {"id": "empty", "file": "/tmp/empty.world"}
        with mock.patch.object(backend, "preflight"), \
                mock.patch.object(run_gazebo_rc_suite.subprocess, "Popen",
                                  return_value=process):
            with self.assertRaises(run_gazebo_rc_suite.EvidenceError) as raised:
                backend.run_scenario(world, 3)
        message = str(raised.exception)
        log_path = Path(message.split("roslaunch log: ", 1)[1])
        self.assertTrue(log_path.is_file())
        log_path.unlink()

    def test_fake_data_cannot_satisfy_live_evidence_contract(self):
        raw = good_scenario()
        raw["live_evidence"] = False
        result = run_gazebo_rc_suite.validate_result(raw, self.config["thresholds"], False)
        self.assertFalse(result["passed"])
        self.assertIn("missing live ROS/Gazebo evidence", result["failures"])

    def test_entire_fault_matrix_is_orchestrated_and_one_fault_fails_suite(self):
        world = next(item for item in self.config["worlds"] if item["id"] == self.config["fault_world"])
        values = {}
        for index, fault in enumerate(self.config["fault_matrix"]):
            values[(world["id"], index, False, fault["id"])] = good_fault(fault)
        backend = run_gazebo_rc_suite.FakeBackend(values)
        result = run_fault_matrix.execute_fault_matrix(self.config, backend)
        self.assertTrue(result["passed"])
        self.assertEqual(result["summary"]["total"], len(self.config["fault_matrix"]))
        target = self.config["fault_matrix"][3]
        values[(world["id"], 3, False, target["id"])]["command"]["nonzero_after_fault"] = 1
        failed = run_fault_matrix.execute_fault_matrix(self.config, run_gazebo_rc_suite.FakeBackend(values))
        self.assertFalse(failed["passed"])
        self.assertEqual(failed["summary"]["failed"], 1)

    def test_fault_requires_live_event_and_actuator_topics(self):
        fault = self.config["fault_matrix"][0]
        for topic in run_gazebo_rc_suite.FAULT_REQUIRED_TOPICS:
            with self.subTest(topic=topic):
                raw = good_fault(fault)
                raw["source_topics"].remove(topic)
                result = run_fault_matrix.validate_fault_result(raw, fault)
                self.assertFalse(result["passed"])
                self.assertTrue(any("missing live fault topics" in failure
                                    for failure in result["failures"]))

    def test_collector_fault_identity_must_match_configured_fault(self):
        fault = self.config["fault_matrix"][0]
        raw = good_fault(fault)
        raw["fault_injected"] = self.config["fault_matrix"][1]["id"]
        result = run_fault_matrix.validate_fault_result(raw, fault)
        self.assertFalse(result["passed"])
        self.assertIn(
            "collector fault injection identity does not match configured fault",
            result["failures"])

    def test_configured_but_unexecuted_fault_is_an_aggregate_failure(self):
        faults = self.config["fault_matrix"][:2]
        results = [{"fault": faults[0]["id"]}]
        failures = run_fault_matrix.fault_coverage_failures(faults, results)
        self.assertEqual(
            failures,
            ["configured fault {} executed 0 times (required exactly once)".format(
                faults[1]["id"])])

    def test_expected_reason_names_cannot_be_fabricated(self):
        fault = self.config["fault_matrix"][0]
        raw = good_fault(fault)
        raw["fault_evidence"]["reason_observations"][0]["names"] = ["OTHER_REASON"]
        result = run_fault_matrix.validate_fault_result(raw, fault)
        self.assertFalse(result["passed"])
        self.assertIn(
            "reason events lack live symbolic reason-mask provenance",
            result["failures"])

    def test_late_exact_zero_fails_timing_budget(self):
        fault = self.config["fault_matrix"][0]
        raw = good_fault(fault)
        raw["fault_evidence"].update({
            "actuator_zero_stamp_s": 10.16,
            "actuator_zero_latency_s": 0.16,
        })
        result = run_fault_matrix.validate_fault_result(raw, fault)
        self.assertFalse(result["passed"])
        self.assertTrue(any("exact-zero timing budget" in failure
                            for failure in result["failures"]))

    def test_nonzero_actuator_output_fails_fault(self):
        fault = self.config["fault_matrix"][0]
        raw = good_fault(fault)
        raw["command"]["nonzero_after_fault"] = 1
        result = run_fault_matrix.validate_fault_result(raw, fault)
        self.assertFalse(result["passed"])
        self.assertIn(
            "nonzero actuator command observed after fault", result["failures"])

    def test_required_latch_needs_guarded_reset_proof(self):
        fault = next(item for item in self.config["fault_matrix"]
                     if item["requires_latch"])
        raw = good_fault(fault)
        del raw["latched_until_guarded_reset"]
        result = run_fault_matrix.validate_fault_result(raw, fault)
        self.assertFalse(result["passed"])
        self.assertIn(
            "fault did not prove its latch through a guarded reset attempt",
            result["failures"])

    def test_missing_noetic_commands_writes_platform_unavailable_artifact(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "report.json"
            with mock.patch.object(run_gazebo_rc_suite.shutil, "which", return_value=None):
                code = run_gazebo_rc_suite.main(["--config", str(self.config_path), "--output", str(output)])
            self.assertEqual(code, 2)
            report = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(report["artifactType"], "algorithm-adversarial-test-report")
            self.assertEqual(report["claimTag"], "SIMULATION_ONLY")
            self.assertEqual(report["status"], "PLATFORM_UNAVAILABLE")
            self.assertFalse(report["hardwareMotionAuthorized"])
            self.assertFalse(report["passengerOperationAuthorized"])
            self.assertIn("missing Noetic commands", report["platformUnavailable"])

    def test_real_cli_has_no_fake_backend_switch(self):
        completed = subprocess.run(
            [sys.executable, str(REPOSITORY / "scripts/run_gazebo_rc_suite.py"), "--help"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False,
            env=dict(os.environ), timeout=10,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertNotIn("fake", completed.stdout.lower())


if __name__ == "__main__":
    unittest.main()
