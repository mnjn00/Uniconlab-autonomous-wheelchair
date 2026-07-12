#!/usr/bin/env python3
import hashlib
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "rc_scenario_driver.py"
CMAKE = PACKAGE / "CMakeLists.txt"
SIM_LAUNCH = PACKAGE.parent / "wheelchair_bringup" / "launch" / "sim_bringup.launch"
SAFETY_LAUNCH = PACKAGE.parent / "wheelchair_safety" / "launch" / "safety.launch"

spec = importlib.util.spec_from_file_location("rc_scenario_driver", str(SCRIPT))
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


class FakeTime:
    @staticmethod
    def now():
        return FakeStamp(12.0)


class FakeStamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class FakeRospy:
    Time = FakeTime

    def __init__(self, params):
        self.params = params
        self.accessed = []

    def get_param(self, name, default=None):
        self.accessed.append(name)
        return self.params.get(name, default)



class SequenceTime:
    def __init__(self, values):
        self.values = iter(values)
        self.last = 0.0

    def now(self):
        try:
            self.last = next(self.values)
        except StopIteration:
            pass
        return FakeStamp(self.last)


class FakeWallClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.sleeps.append(duration)
        self.now += duration



class Attribute:
    pass


class FakeInitialPose:
    def __init__(self):
        self.header = Attribute()
        self.pose = Attribute()
        self.pose.pose = Attribute()
        self.pose.pose.position = Attribute()
        self.pose.pose.orientation = Attribute()
        self.pose.covariance = [0.0] * 36


class ConnectionPublisher:
    def __init__(self, connections):
        self.connections = iter(connections)
        self.last = 0
        self.messages = []

    def get_num_connections(self):
        try:
            self.last = next(self.connections)
        except StopIteration:
            pass
        return self.last

    def publish(self, message):
        self.messages.append(message)

class CallbackEvidence:
    def __init__(self, clear):
        self.clear = clear
        self.update_count = 0

    def update(self, name, message):
        self.update_count += 1

    def safety_clear(self, now):
        return self.clear

class ActiveAction:
    def __init__(self):
        self.cancel_count = 0

    def get_state(self):
        return "ACTIVE"

    def cancel_goal(self):
        self.cancel_count += 1


class FrozenClockRospy(FakeRospy):
    def __init__(self):
        super().__init__({})
        self.Time = SequenceTime([12.0])
        self.is_shutdown = lambda: False



class ScenarioDriverTests(unittest.TestCase):
    def setUp(self):
        self.document = {
            "immutable": True,
            "map": {"map_id": "map-a", "sha256": "a" * 64},
            "safety_manifest_sha256": "b" * 64,
            "outbound_route": {
                "route_id": "route-out", "direction": "outbound",
                "route_manifest_sha256": "c" * 64,
            },
            "return_route": {
                "route_id": "route-back", "direction": "return",
                "route_manifest_sha256": "c" * 64,
            },
        }

    def binding(self, direction):
        raw = yaml.safe_dump(self.document, sort_keys=True).encode("utf-8")
        temporary = tempfile.NamedTemporaryFile(delete=False)
        self.addCleanup(Path(temporary.name).unlink)
        temporary.write(raw)
        temporary.close()
        return driver.load_binding(temporary.name, hashlib.sha256(raw).hexdigest(),
                                   direction, "qualification", 1701)

    def test_binds_exact_outbound_and_return_routes(self):
        outbound = self.binding("outbound")
        returned = self.binding("return")
        self.assertEqual((outbound.route_id, outbound.direction), ("route-out", 1))
        self.assertEqual((returned.route_id, returned.direction), ("route-back", 2))
        for binding in (outbound, returned):
            self.assertEqual(binding.map_id, "map-a")
            self.assertEqual(binding.map_sha256, "a" * 64)
            self.assertEqual(binding.route_manifest_sha256, "c" * 64)
            self.assertEqual(binding.safety_manifest_sha256, "b" * 64)

    def test_mission_id_is_deterministic_and_scenario_seed_bound(self):
        first = self.binding("outbound")
        second = self.binding("outbound")
        self.assertEqual(first.mission_id, second.mission_id)
        raw = yaml.safe_dump(self.document, sort_keys=True).encode("utf-8")
        with tempfile.NamedTemporaryFile() as temporary:
            temporary.write(raw)
            temporary.flush()
            changed = driver.load_binding(temporary.name, hashlib.sha256(raw).hexdigest(),
                                          "outbound", "qualification", 1702)
        self.assertNotEqual(first.mission_id, changed.mission_id)

    def test_manifest_hash_mutability_and_direction_fail_closed(self):
        with self.assertRaises(driver.ScenarioError):
            self.binding("sideways")
        mutable = dict(self.document)
        mutable["immutable"] = False
        raw = yaml.safe_dump(mutable).encode("utf-8")
        with tempfile.NamedTemporaryFile() as temporary:
            temporary.write(raw)
            temporary.flush()
            with self.assertRaises(driver.ScenarioError):
                driver.load_binding(temporary.name, hashlib.sha256(raw).hexdigest(),
                                    "outbound", "qualification", 1701)
            with self.assertRaises(driver.ScenarioError):
                driver.load_binding(temporary.name, "0" * 64,
                                    "outbound", "qualification", 1701)

    def test_preflight_requires_false_authority_and_exact_manifest_identities(self):
        binding = self.binding("outbound")
        params = {
            "/simulation_only": True,
            "/hardware_motion_authorized": False,
            "/passenger_operation_authorized": False,
            "/use_sim_time": True,
            "/wheelchair_bringup/map_sha256": binding.map_sha256,
            "/wheelchair_bringup/route_sha256": binding.route_manifest_sha256,
            "~map_id": binding.map_id,
            "~safety_manifest_sha256": binding.safety_manifest_sha256,
        }
        driver.require_preflight(FakeRospy(params), binding)
        for name in ("/hardware_motion_authorized", "/passenger_operation_authorized"):
            invalid = dict(params)
            invalid[name] = True
            with self.assertRaises(driver.ScenarioError):
                driver.require_preflight(FakeRospy(invalid), binding)
    def test_startup_ready_rejects_margin_geofence(self):
        evidence = driver.Evidence(self.binding("outbound"), 1.0)

        def message(**values):
            status = Attribute()
            status.header = Attribute()
            status.header.stamp = FakeStamp(12.0)
            for name, value in values.items():
                setattr(status, name, value)
            return status

        evidence.update("safety", message(
            state="DISARMED", DISARMED="DISARMED", armed=False,
            estop_latched=False, reason_mask=driver.STARTUP_REASON))
        evidence.update("localization", message(
            state="OK", OK="OK", reason_mask=0, map_id="map-a", map_sha256="a" * 64))
        evidence.update("geofence", message(
            state="MARGIN", INSIDE="INSIDE", MARGIN="MARGIN", reason_mask=0,
            manifest_sha256="b" * 64, route_id="route-out"))
        evidence.update("collision", message(
            state="CLEAR", STATE_CLEAR="CLEAR", STATE_CAUTION="CAUTION", reason_mask=0))
        evidence.update("slope", message(
            state="CLEAR", STATE_CLEAR="CLEAR", STATE_SLOW="SLOW", reason_mask=0))
        evidence.update("route", message(
            state="ACTIVE", ACTIVE="ACTIVE", route_id="route-out", map_id="map-a"))

        self.assertFalse(evidence.startup_ready(12.0))
        evidence.values["geofence"].state = evidence.values["geofence"].INSIDE
        self.assertTrue(evidence.startup_ready(12.0))

    def test_pre_initialization_requires_fresh_stationary_route_evidence(self):
        evidence = driver.Evidence(self.binding("outbound"), 1.0)

        def message(**values):
            status = Attribute()
            status.header = Attribute()
            status.header.stamp = FakeStamp(12.0)
            for name, value in values.items():
                setattr(status, name, value)
            return status

        evidence.update("safety", message(
            state="DISARMED", DISARMED="DISARMED", STOPPED="STOPPED",
            armed=False, estop_latched=False, reason_mask=driver.STARTUP_REASON))
        evidence.update("collision", message(
            state="CAUTION", STATE_CLEAR="CLEAR", STATE_CAUTION="CAUTION", reason_mask=0))
        evidence.update("slope", message(
            state="SLOW", STATE_CLEAR="CLEAR", STATE_SLOW="SLOW", reason_mask=0))
        evidence.update("route", message(
            state="ACTIVE", ACTIVE="ACTIVE", route_id="route-out", map_id="map-a"))

        self.assertTrue(evidence.pre_initialization_ready(12.0))
        evidence.values["safety"].state = evidence.values["safety"].STOPPED
        self.assertTrue(evidence.pre_initialization_ready(12.0))
        evidence.values["safety"].state = "LATCHED"
        self.assertFalse(evidence.pre_initialization_ready(12.0))
        evidence.values["safety"].state = evidence.values["safety"].DISARMED
        evidence.values["safety"].armed = True
        self.assertFalse(evidence.pre_initialization_ready(12.0))
        evidence.values["safety"].armed = False
        evidence.values["safety"].estop_latched = True
        self.assertFalse(evidence.pre_initialization_ready(12.0))
        evidence.values["safety"].estop_latched = False
        evidence.values["slope"].reason_mask = 1
        self.assertFalse(evidence.pre_initialization_ready(12.0))
        evidence.values["slope"].reason_mask = 0
        evidence.values["route"].route_id = "other-route"
        self.assertFalse(evidence.pre_initialization_ready(12.0))
        evidence.values["route"].route_id = "route-out"
        evidence.values["collision"].header.stamp = FakeStamp(10.0)
        self.assertFalse(evidence.pre_initialization_ready(12.0))

    def test_sim_time_wait_succeeds_immediately_without_sleeping(self):
        rospy = FakeRospy({})
        wall = FakeWallClock()

        driver.wait_for_sim_time(rospy, 1.0, wall.monotonic, wall.sleep)

        self.assertEqual(wall.sleeps, [])

    def test_sim_time_wait_uses_wall_sleep_until_clock_is_positive(self):
        rospy = FakeRospy({})
        rospy.Time = SequenceTime([0.0, float("nan"), 2.5])
        wall = FakeWallClock()

        driver.wait_for_sim_time(rospy, 1.0, wall.monotonic, wall.sleep)

        self.assertEqual(wall.sleeps, [0.05, 0.05])
        self.assertAlmostEqual(wall.now, 0.1)

    def test_sim_time_wait_fails_closed_at_wall_clock_timeout(self):
        rospy = FakeRospy({})
        rospy.Time = SequenceTime([0.0])
        wall = FakeWallClock()

        with self.assertRaisesRegex(driver.ScenarioError,
                                    "timeout waiting for ROS simulated time"):
            driver.wait_for_sim_time(rospy, 0.12, wall.monotonic, wall.sleep)

        self.assertAlmostEqual(wall.now, 0.12)
        self.assertEqual(len(wall.sleeps), 3)
        self.assertAlmostEqual(sum(wall.sleeps), 0.12)

    def test_sim_time_wait_rejects_invalid_timeout(self):
        rospy = FakeRospy({})
        for invalid in (0, -1, float("nan"), float("inf"), "invalid", None):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(driver.ScenarioError,
                                            "sim_time_timeout_sec must be finite and positive"):
                    driver.wait_for_sim_time(rospy, invalid)

    def test_initial_pose_message_is_explicit_normalized_and_conservative(self):
        stamp = FakeStamp(4.0)
        message = driver.build_initial_pose(FakeInitialPose, stamp, 1.25, -2.5, 1.0)

        self.assertEqual(message.header.frame_id, "map")
        self.assertIs(message.header.stamp, stamp)
        self.assertEqual((message.pose.pose.position.x, message.pose.pose.position.y),
                         (1.25, -2.5))
        quaternion_norm = (message.pose.pose.orientation.z ** 2
                           + message.pose.pose.orientation.w ** 2)
        self.assertAlmostEqual(quaternion_norm, 1.0)
        self.assertGreater(message.pose.covariance[0], 0.0)
        self.assertGreater(message.pose.covariance[7], 0.0)
        self.assertGreater(message.pose.covariance[35], 0.0)
        self.assertTrue(all(value == 0.0 for index, value in enumerate(
            message.pose.covariance) if index not in (0, 7, 35)))

    def test_initial_pose_rejects_nonfinite_values_and_covariance(self):
        stamp = FakeStamp(4.0)
        for values, covariance in (
                ((float("nan"), 0.0, 0.0), (0.25, 0.25, 0.1)),
                ((0.0, float("inf"), 0.0), (0.25, 0.25, 0.1)),
                ((0.0, 0.0, float("-inf")), (0.25, 0.25, 0.1)),
                ((0.0, 0.0, 0.0), (0.25, float("nan"), 0.1)),
                ((0.0, 0.0, 0.0), (0.25, 0.25, 0.0))):
            with self.subTest(values=values, covariance=covariance):
                with self.assertRaises(driver.ScenarioError):
                    driver.build_initial_pose(
                        FakeInitialPose, stamp, *values, covariance=covariance)

    def test_required_initial_pose_subscriber_count_validation(self):
        self.assertEqual(driver.required_subscriber_count(2), 2)
        for invalid in (True, False, 0, -1, 65, 1.0, "2", None):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(
                        driver.ScenarioError, "must be an integer from 1 through 64"):
                    driver.required_subscriber_count(invalid)

    def initial_pose_scenario(self, connections, timeout=0.2):
        scenario = driver.ScenarioDriver.__new__(driver.ScenarioDriver)
        scenario.rospy = FakeRospy({})
        scenario.rospy.is_shutdown = lambda: False
        scenario.ready_timeout_s = timeout
        scenario._initial_pose = ConnectionPublisher(connections)
        scenario._initial_pose_required_subscribers = 2
        scenario._PoseWithCovarianceStamped = FakeInitialPose
        scenario._initial_pose_values = (0.0, 0.0, 0.0)
        return scenario

    def test_initial_pose_waits_for_zero_one_two_and_publishes_exactly_once(self):
        scenario = self.initial_pose_scenario([0, 1, 2])
        wall = FakeWallClock()
        original_monotonic, original_sleep = driver.time.monotonic, driver.time.sleep
        self.addCleanup(setattr, driver.time, "monotonic", original_monotonic)
        self.addCleanup(setattr, driver.time, "sleep", original_sleep)
        driver.time.monotonic, driver.time.sleep = wall.monotonic, wall.sleep

        scenario._publish_initial_pose()

        self.assertEqual(len(scenario._initial_pose.messages), 1)
        self.assertEqual(wall.sleeps, [0.05, 0.05])

    def test_initial_pose_accepts_required_connections_observed_at_deadline(self):
        scenario = self.initial_pose_scenario([0, 1, 2], timeout=0.1)
        wall = FakeWallClock()
        original_monotonic, original_sleep = driver.time.monotonic, driver.time.sleep
        self.addCleanup(setattr, driver.time, "monotonic", original_monotonic)
        self.addCleanup(setattr, driver.time, "sleep", original_sleep)
        driver.time.monotonic, driver.time.sleep = wall.monotonic, wall.sleep

        scenario._publish_initial_pose()

        self.assertAlmostEqual(wall.now, 0.1)
        self.assertEqual(len(scenario._initial_pose.messages), 1)
        self.assertTrue(all(duration > 0.0 for duration in wall.sleeps))

    def test_initial_pose_times_out_with_only_one_subscriber_and_no_publication(self):
        scenario = self.initial_pose_scenario([1], timeout=0.12)
        wall = FakeWallClock()
        original_monotonic, original_sleep = driver.time.monotonic, driver.time.sleep
        self.addCleanup(setattr, driver.time, "monotonic", original_monotonic)
        self.addCleanup(setattr, driver.time, "sleep", original_sleep)
        driver.time.monotonic, driver.time.sleep = wall.monotonic, wall.sleep

        with self.assertRaisesRegex(driver.ScenarioError,
                                    "timeout waiting for initial pose subscriber"):
            scenario._publish_initial_pose()

        self.assertEqual(scenario._initial_pose.messages, [])
        self.assertAlmostEqual(wall.now, 0.12)
        self.assertTrue(all(duration > 0.0 for duration in wall.sleeps))

    def startup_wait_scenario(self, connections=(2,), stamps=(1.0, 2.0, 3.0)):
        scenario = self.initial_pose_scenario(connections, timeout=3.0)
        scenario.rospy.Time = SequenceTime(stamps)
        wall = FakeWallClock()
        original_monotonic, original_sleep = driver.time.monotonic, driver.time.sleep
        self.addCleanup(setattr, driver.time, "monotonic", original_monotonic)
        self.addCleanup(setattr, driver.time, "sleep", original_sleep)
        driver.time.monotonic, driver.time.sleep = wall.monotonic, wall.sleep
        return scenario, wall

    def test_ordinary_waits_do_not_publish_initial_pose(self):
        scenario, unused_wall = self.startup_wait_scenario()
        calls = {"count": 0}

        def ready(unused_now):
            calls["count"] += 1
            return calls["count"] == 6

        scenario._wait(ready, 2.0, "ordinary wait")

        self.assertEqual(scenario._initial_pose.messages, [])

    def test_subscriber_disappearance_rejects_one_time_initialization(self):
        scenario, unused_wall = self.startup_wait_scenario(connections=(2, 1))
        self.assertEqual(scenario._initial_pose.get_num_connections(), 2)

        with self.assertRaisesRegex(driver.ScenarioError,
                                    "initial pose subscribers disappeared"):
            scenario._publish_initial_pose_message()

        self.assertEqual(scenario._initial_pose.messages, [])

    def test_startup_wait_timeout_is_wall_clock_bounded_without_publication(self):
        scenario, wall = self.startup_wait_scenario()

        with self.assertRaisesRegex(driver.ScenarioError,
                                    "timeout waiting for startup evidence"):
            scenario._wait(lambda unused_now: False, 1.5, "startup evidence")

        self.assertAlmostEqual(wall.now, 1.5)
        self.assertEqual(scenario._initial_pose.messages, [])

    def test_mission_action_timeout_cancels_once_with_frozen_sim_clock(self):
        scenario = driver.ScenarioDriver.__new__(driver.ScenarioDriver)
        scenario.rospy = FrozenClockRospy()
        scenario.action_timeout_s = 0.12
        scenario._action = ActiveAction()
        scenario._goal_sent = True
        scenario._canceled = False
        scenario._DiagnosticStatus = Attribute()
        scenario._DiagnosticStatus.ERROR = "ERROR"
        scenario._emit = lambda unused_level, unused_message: None
        scenario.evidence = CallbackEvidence(clear=True)
        wall = FakeWallClock()
        original_monotonic, original_sleep = driver.time.monotonic, driver.time.sleep
        self.addCleanup(setattr, driver.time, "monotonic", original_monotonic)
        self.addCleanup(setattr, driver.time, "sleep", original_sleep)
        driver.time.monotonic, driver.time.sleep = wall.monotonic, wall.sleep

        with self.assertRaisesRegex(driver.ScenarioError, "mission action timeout"):
            scenario._wait_for_mission_action({"SUCCEEDED"})

        self.assertAlmostEqual(wall.now, 0.12)
        self.assertEqual(scenario._action.cancel_count, 1)
        self.assertTrue(scenario._canceled)


    def test_preflight_precedes_every_ros_endpoint_constructor(self):
        source = SCRIPT.read_text(encoding="utf-8")
        main = source[source.index("def main():"):]
        self.assertLess(main.index("require_preflight(rospy, binding)"),
                        main.index("ScenarioDriver("))
        self.assertLess(main.index("wait_for_sim_time("),
                        main.index("ScenarioDriver("))
        self.assertIn('rospy.get_param("~initial_pose_required_subscribers", 2)', main)
        constructor = source[source.index("class ScenarioDriver:"):source.index("def main():")]
        self.assertIn('Publisher("/safety/arm"', constructor)
        self.assertIn('"/initialpose", PoseWithCovarianceStamped, queue_size=1', constructor)
        self.assertIn("initial_pose_required_subscribers", constructor)
        self.assertIn('ServiceProxy("/wheelchair_mission/arm"', constructor)
        self.assertIn('SimpleActionClient("/wheelchair_mission/execute_route"', constructor)

    def test_goal_activation_and_evidence_precede_gate_arming(self):
        source = SCRIPT.read_text(encoding="utf-8")
        run = source[source.index("    def run(self):"):source.index("\ndef main():")]
        mission_arm = run.index("self._mission_arm()")
        send_goal = run.index("self._action.send_goal(goal)")
        goal_active = run.index('"active mission goal"')
        pre_initialization = run.index('"pre-initialization evidence"')
        initial_pose = run.index("self._publish_initial_pose()")
        startup_evidence = run.index('"startup evidence"')
        gate_arm = run.index("self._arm.publish")
        gate_clear = run.index('"armed safety gate"')
        monitoring = run.index("self._safety_monitoring = True")

        self.assertLess(mission_arm, send_goal)
        self.assertLess(send_goal, goal_active)
        self.assertLess(goal_active, pre_initialization)
        self.assertLess(pre_initialization, initial_pose)
        self.assertLess(initial_pose, startup_evidence)
        self.assertLess(startup_evidence, gate_arm)
        self.assertLess(gate_arm, gate_clear)
        self.assertLess(gate_clear, monitoring)
        self.assertEqual(run.count("self._action.send_goal(goal)"), 1)
        self.assertEqual(run.count("self._publish_initial_pose()"), 1)
        self.assertEqual(source.count('"/initialpose"'), 1)
        self.assertEqual(source.count("self._initial_pose.publish"), 1)
        self.assertIn(
            'self._publish_initial_pose()\n'
            '            self._wait(self.evidence.startup_ready, self.ready_timeout_s, '
            '"startup evidence")',
            run)
        self.assertNotIn("tick=", run)
        self.assertIn('self._cancel("safety loss")', source)
        self.assertIn('self._cancel("action timeout")', source)
        self.assertNotIn("reset", run.lower())
        self.assertNotIn("resume", run.lower())
        pre_initialization_ready = source[
            source.index("    def pre_initialization_ready(self, now):"):
            source.index("    def safety_clear(self, now):")]
        self.assertIn("route.state == route.ACTIVE", pre_initialization_ready)
        self.assertIn("slope.STATE_SLOW", pre_initialization_ready)
        self.assertNotIn('"localization"', pre_initialization_ready)

    def test_pre_clear_safety_callback_cannot_cancel_submitted_goal(self):
        scenario = driver.ScenarioDriver.__new__(driver.ScenarioDriver)
        scenario._goal_sent = True
        scenario._safety_monitoring = False
        scenario.rospy = FakeRospy({})
        scenario.evidence = CallbackEvidence(clear=False)
        canceled = []
        scenario._cancel = canceled.append

        scenario._callback("safety")(object())

        self.assertEqual(canceled, [])
        self.assertEqual(scenario.evidence.update_count, 1)

    def test_post_clear_safety_loss_cancels_immediately(self):
        scenario = driver.ScenarioDriver.__new__(driver.ScenarioDriver)
        scenario._goal_sent = True
        scenario._safety_monitoring = True
        scenario.rospy = FakeRospy({})
        scenario.evidence = CallbackEvidence(clear=True)
        canceled = []
        scenario._cancel = canceled.append

        callback = scenario._callback("safety")
        callback(object())
        self.assertEqual(canceled, [])

        scenario.evidence.clear = False
        callback(object())
        self.assertEqual(canceled, ["safety loss"])

    def test_no_motion_command_surface_exists(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("Twist", source)
        self.assertNotIn("cmd_vel", source)
        self.assertNotIn("motor_command", source)
        self.assertNotIn("hardware/", source)
        self.assertEqual(source.count("send_goal(goal)"), 1)
        self.assertIn('queue_size=1', source)

    def test_sim_startup_readiness_bound_wires_driver_and_guard(self):
        sim_root = ET.parse(str(SIM_LAUNCH)).getroot()
        sim_args = {element.attrib["name"]: element.attrib for element in sim_root.findall("arg")}
        self.assertEqual(sim_args["startup_readiness_timeout_sec"]["default"], "30.0")

        safety_include = next(
            element for element in sim_root.findall("include")
            if element.attrib["file"] == "$(find wheelchair_safety)/launch/safety.launch")
        safety_args = {element.attrib["name"]: element.attrib["value"]
                       for element in safety_include.findall("arg")}
        self.assertEqual(
            safety_args["localization_initialization_attempt_timeout_s"],
            "$(arg startup_readiness_timeout_sec)")

        driver_node = next(
            element for element in sim_root.findall("node")
            if element.attrib.get("type") == "rc_scenario_driver.py")
        driver_params = {element.attrib["name"]: element.attrib["value"]
                         for element in driver_node.findall("param")}
        self.assertEqual(driver_params["readiness_timeout_sec"],
                         "$(arg startup_readiness_timeout_sec)")

        safety_root = ET.parse(str(SAFETY_LAUNCH)).getroot()
        guard_args = {element.attrib["name"]: element.attrib
                      for element in safety_root.findall("arg")}
        self.assertEqual(
            guard_args["localization_initialization_attempt_timeout_s"]["default"], "30.0")
        guard_node = next(
            element for element in safety_root.findall("node")
            if element.attrib.get("name") == "localization_guard")
        guard_params = {element.attrib["name"]: element.attrib["value"]
                        for element in guard_node.findall("param")}
        self.assertEqual(guard_params["initialization_attempt_timeout_s"],
                         "$(arg localization_initialization_attempt_timeout_s)")

    def test_launch_is_default_disabled_and_cmake_installs_driver(self):
        root = ET.parse(str(SIM_LAUNCH)).getroot()
        args = {element.attrib["name"]: element.attrib for element in root.findall("arg")}
        self.assertEqual(args["auto_start"]["default"], "false")
        nodes = [node for node in root.findall("node")
                 if node.attrib.get("type") == "rc_scenario_driver.py"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib.get("if"), "$(arg auto_start)")
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("scripts/rc_scenario_driver.py", cmake)
        self.assertIn("tests/test_rc_scenario_driver.py", cmake)


if __name__ == "__main__":
    unittest.main()
