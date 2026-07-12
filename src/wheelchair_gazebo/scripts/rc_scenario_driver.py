#!/usr/bin/env python3
"""Opt-in, simulation-only qualification mission driver.

This node owns no motion command topic.  It performs one fail-closed arm and
ExecuteRoute transaction, then exits.
"""

import hashlib
import math
import os
import sys
import threading
import time
from pathlib import Path
from dataclasses import dataclass

import yaml

STARTUP_REASON = 2048
DIRECTION_VALUES = {"outbound": 1, "return": 2}
ROUTE_KEYS = {"outbound": "outbound_route", "return": "return_route"}


class ScenarioError(RuntimeError):
    pass


@dataclass(frozen=True)
class RouteBinding:
    mission_id: str
    route_id: str
    direction: int
    direction_name: str
    scenario: str
    seed: int
    claim_tag: str
    map_id: str
    map_sha256: str
    raw_route_asset_sha256: str
    navigation_manifest_sha256: str
    directional_route_sha256: str
    route_safety_config_sha256: str
    safety_manifest_sha256: str
    route_truth_sha256: str
    scenario_sha256: str
    a13_sha256: str


def _sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def _valid_sha256(value):
    return (isinstance(value, str) and len(value) == 64
            and all(character in "0123456789abcdef" for character in value))


def _require_sha256(name, value):
    if not _valid_sha256(value):
        raise ScenarioError("%s must be a lowercase SHA-256" % name)
    return value


def _load_yaml(path, description):
    try:
        raw = path.read_bytes()
        document = yaml.safe_load(raw)
    except (OSError, yaml.YAMLError) as exc:
        raise ScenarioError("invalid %s: %s" % (description, exc))
    if not isinstance(document, dict):
        raise ScenarioError("%s must be a mapping" % description)
    return raw, document


def _contained_reference(root, base, reference, description):
    if not isinstance(reference, str) or not reference or Path(reference).is_absolute():
        raise ScenarioError("%s path must be relative" % description)
    root = root.resolve()
    unresolved = base / reference
    candidate = unresolved.resolve()
    try:
        candidate.relative_to(root)
    except ValueError:
        raise ScenarioError("%s path escapes repository" % description)
    current = base
    for component in Path(reference).parts:
        current /= component
        if current.is_symlink():
            raise ScenarioError("%s path must not be symlinked" % description)
    if not candidate.is_file():
        raise ScenarioError("%s path is missing or not a regular file" % description)
    return candidate


def _binding_value(document, name):
    value = document.get(name)
    if not isinstance(value, str) or not value:
        raise ScenarioError("%s is missing" % name)
    return value


def load_binding(route_truth_path, expected_route_truth_sha256, direction, scenario, seed,
                 scenario_sha256, a13_sha256, claim_tag):
    """Load the complete immutable route, map, and safety identity chain."""
    if direction not in DIRECTION_VALUES:
        raise ScenarioError("direction must be outbound or return")
    if not isinstance(scenario, str) or not scenario:
        raise ScenarioError("scenario is missing")
    if isinstance(seed, bool):
        raise ScenarioError("seed must be an integer")
    try:
        seed = int(seed)
    except (TypeError, ValueError):
        raise ScenarioError("seed must be an integer")
    _require_sha256("route truth SHA-256", expected_route_truth_sha256)
    _require_sha256("scenario SHA-256", scenario_sha256)
    _require_sha256("A13 SHA-256", a13_sha256)
    if claim_tag != "SIMULATION_ONLY":
        raise ScenarioError("claim tag must be SIMULATION_ONLY")

    route_truth = Path(route_truth_path)
    if route_truth.is_symlink() or not route_truth.is_file():
        raise ScenarioError("route truth must be a non-symlink regular file")
    root = route_truth.resolve().parents[3]
    try:
        route_truth.resolve().relative_to(root)
    except ValueError:
        raise ScenarioError("route truth path escapes repository")
    truth_raw, truth = _load_yaml(route_truth, "route truth")
    if _sha256(truth_raw) != expected_route_truth_sha256:
        raise ScenarioError("route truth SHA-256 mismatch")
    if truth.get("immutable") is not True or truth.get("direction") != direction:
        raise ScenarioError("route truth is not immutable for selected direction")

    navigation_ref = truth.get("navigation_manifest")
    safety_ref = truth.get("route_safety_config")
    if not isinstance(navigation_ref, dict) or not isinstance(safety_ref, dict):
        raise ScenarioError("route truth references are missing")
    navigation_sha256 = _require_sha256(
        "navigation manifest SHA-256", navigation_ref.get("sha256"))
    safety_config_sha256 = _require_sha256(
        "route-safety config SHA-256", safety_ref.get("sha256"))
    navigation_path = _contained_reference(
        root, route_truth.parent, navigation_ref.get("path"), "navigation manifest")
    safety_config_path = _contained_reference(
        root, route_truth.parent, safety_ref.get("path"), "route-safety config")
    navigation_raw, navigation = _load_yaml(navigation_path, "navigation manifest")
    safety_raw, safety = _load_yaml(safety_config_path, "route-safety config")
    if _sha256(navigation_raw) != navigation_sha256:
        raise ScenarioError("navigation manifest SHA-256 mismatch")
    if _sha256(safety_raw) != safety_config_sha256:
        raise ScenarioError("route-safety config SHA-256 mismatch")
    if navigation.get("immutable") is not True:
        raise ScenarioError("navigation manifest is not immutable")

    route = navigation.get(ROUTE_KEYS[direction])
    map_binding = navigation.get("map")
    waypoint_asset = navigation.get("waypoint_asset")
    if not all(isinstance(value, dict) for value in (route, map_binding, waypoint_asset)):
        raise ScenarioError("navigation route/map/asset binding is missing")
    route_id = _binding_value(route, "route_id")
    map_id = _binding_value(map_binding, "map_id")
    map_sha256 = _require_sha256("map SHA-256", map_binding.get("sha256"))
    raw_route_asset_sha256 = _require_sha256(
        "raw route asset SHA-256", waypoint_asset.get("sha256"))
    directional_route_sha256 = _require_sha256(
        "directional route SHA-256", route.get("route_manifest_sha256"))
    safety_manifest_sha256 = _require_sha256(
        "safety manifest SHA-256", navigation.get("safety_manifest_sha256"))
    if route.get("direction") != direction:
        raise ScenarioError("navigation route direction mismatch")
    raw_asset_path = _contained_reference(
        root, navigation_path.parent, waypoint_asset.get("path"), "raw route asset")
    if _sha256(raw_asset_path.read_bytes()) != raw_route_asset_sha256:
        raise ScenarioError("raw route asset SHA-256 mismatch")
    map_path = _contained_reference(root, navigation_path.parent, map_binding.get("pgm_path"), "map")
    if _sha256(map_path.read_bytes()) != map_sha256:
        raise ScenarioError("map SHA-256 mismatch")

    if safety.get("simulation_only") is not True:
        raise ScenarioError("route-safety config is not simulation-only")
    if safety.get("hardware_motion_authorized") is not False:
        raise ScenarioError("route-safety config authorizes hardware motion")
    if safety.get("passenger_operation_authorized") is not False:
        raise ScenarioError("route-safety config authorizes passenger operation")
    if _require_sha256("route-safety map SHA-256", safety.get("expected_map_sha256")) != map_sha256:
        raise ScenarioError("route-safety map binding mismatch")
    geometry = safety.get("simulation_geometry")
    if not isinstance(geometry, dict):
        raise ScenarioError("route-safety geometry binding is missing")
    if _require_sha256("route-safety raw route SHA-256", geometry.get("route_asset_sha256")) != raw_route_asset_sha256:
        raise ScenarioError("route-safety raw route binding mismatch")
    if _require_sha256("route-safety navigation SHA-256", geometry.get("navigation_route_manifest_sha256")) != navigation_sha256:
        raise ScenarioError("route-safety navigation binding mismatch")
    if _contained_reference(root, safety_config_path.parent,
                            geometry.get("route_asset_path"), "route-safety raw route asset") != raw_asset_path:
        raise ScenarioError("route-safety raw route path mismatch")
    if _contained_reference(root, safety_config_path.parent,
                            geometry.get("navigation_route_manifest_path"),
                            "route-safety navigation manifest") != navigation_path:
        raise ScenarioError("route-safety navigation path mismatch")
    if _require_sha256("route-safety safety manifest SHA-256",
                       safety.get("expected_manifest_sha256")) != safety_manifest_sha256:
        raise ScenarioError("route-safety safety manifest binding mismatch")
    expected_routes = safety.get("expected_route_hashes")
    route_bindings = geometry.get("route_bindings")
    if (not isinstance(expected_routes, dict) or
            _require_sha256("route-safety directional route SHA-256",
                            expected_routes.get(route_id)) != directional_route_sha256 or
            not isinstance(route_bindings, dict) or
            not isinstance(route_bindings.get(direction), dict) or
            route_bindings[direction].get("route_id") != route_id or
            route_bindings[direction].get("asset_key") != ROUTE_KEYS[direction]):
        raise ScenarioError("route-safety directional route binding mismatch")

    safety_manifest_path = _contained_reference(
        root, safety_config_path.parent, safety.get("manifest_path"), "safety manifest")
    safety_manifest_raw, safety_manifest = _load_yaml(safety_manifest_path, "safety manifest")
    if _sha256(safety_manifest_raw) != safety_manifest_sha256:
        raise ScenarioError("safety manifest SHA-256 mismatch")
    safety_map = safety_manifest.get("map")
    if not isinstance(safety_map, dict) or safety_map.get("map_id") != map_id:
        raise ScenarioError("safety manifest map identity mismatch")
    if _require_sha256("safety manifest map SHA-256", safety_map.get("sha256")) != map_sha256:
        raise ScenarioError("safety manifest map SHA-256 mismatch")
    approved = safety_manifest.get("approved_routes")
    matching = [item for item in approved if isinstance(item, dict)
                and item.get("route_id") == route_id and item.get("direction") == direction] if isinstance(approved, list) else []
    if len(matching) != 1 or _require_sha256(
            "approved directional route SHA-256",
            matching[0].get("route_manifest_sha256")) != directional_route_sha256:
        raise ScenarioError("safety manifest approved route mismatch")
    authority = safety_manifest.get("authority")
    if (matching[0].get("hardware_authorized") is not False or not isinstance(authority, dict)
            or authority.get("simulation_only") is not True
            or authority.get("hardware_authorized") is not False
            or authority.get("passenger_authorized") is not False):
        raise ScenarioError("safety manifest authorizes operation")

    material = "%s\n%s\n%s\n%s" % (scenario, seed, direction, route_id)
    return RouteBinding(
        mission_id="rc-" + hashlib.sha256(material.encode("utf-8")).hexdigest()[:24],
        route_id=route_id, direction=DIRECTION_VALUES[direction], direction_name=direction,
        scenario=scenario, seed=seed, claim_tag=claim_tag, map_id=map_id, map_sha256=map_sha256,
        raw_route_asset_sha256=raw_route_asset_sha256,
        navigation_manifest_sha256=navigation_sha256,
        directional_route_sha256=directional_route_sha256,
        route_safety_config_sha256=safety_config_sha256,
        safety_manifest_sha256=safety_manifest_sha256,
        route_truth_sha256=expected_route_truth_sha256,
        scenario_sha256=scenario_sha256, a13_sha256=a13_sha256)

def positive_timeout(name, value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ScenarioError("%s must be finite and positive" % name)
    if not math.isfinite(value) or value <= 0.0:
        raise ScenarioError("%s must be finite and positive" % name)
    return value


def wait_for_sim_time(rospy, timeout_s, monotonic=time.monotonic, sleep=time.sleep):
    """Wait for a finite positive ROS clock without depending on simulated sleeps."""
    timeout_s = positive_timeout("sim_time_timeout_sec", timeout_s)
    deadline = monotonic() + timeout_s
    while True:
        wall_now = monotonic()
        if wall_now > deadline:
            raise ScenarioError("timeout waiting for ROS simulated time")
        ros_now = float(rospy.Time.now().to_sec())
        if math.isfinite(ros_now) and ros_now > 0.0:
            return
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            raise ScenarioError("timeout waiting for ROS simulated time")
        sleep(min(0.05, remaining))


def finite_value(name, value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ScenarioError("%s must be finite" % name)
    if not math.isfinite(value):
        raise ScenarioError("%s must be finite" % name)
    return value


def required_subscriber_count(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 64:
        raise ScenarioError(
            "initial_pose_required_subscribers must be an integer from 1 through 64")
    return value


def build_initial_pose(message_type, stamp, x, y, yaw,
                       covariance=(0.25, 0.25, 0.06853891945200942)):
    """Build the explicit simulation localization initialization request."""
    x = finite_value("initial_pose_x", x)
    y = finite_value("initial_pose_y", y)
    yaw = finite_value("initial_pose_yaw", yaw)
    covariance = tuple(finite_value("initial pose covariance", value)
                       for value in covariance)
    if len(covariance) != 3 or any(value <= 0.0 for value in covariance):
        raise ScenarioError("initial pose covariance must be finite and positive")
    message = message_type()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.pose.covariance[0] = covariance[0]
    message.pose.covariance[7] = covariance[1]
    message.pose.covariance[35] = covariance[2]
    return message




def require_preflight(rospy, binding):
    """Validate all authority and identity parameters before ROS endpoints exist."""
    exact = {
        "/simulation_only": True,
        "/hardware_motion_authorized": False,
        "/passenger_operation_authorized": False,
        "/use_sim_time": True,
        "/wheelchair_bringup/map_id": binding.map_id,
        "/wheelchair_bringup/map_sha256": binding.map_sha256,
        "/wheelchair_bringup/route_sha256": binding.raw_route_asset_sha256,
        "/wheelchair_bringup/policies/route_sha256": binding.navigation_manifest_sha256,
        "/wheelchair_bringup/policies/route_safety_sha256": binding.route_safety_config_sha256,
        "/wheelchair_bringup/safety_manifest_sha256": binding.safety_manifest_sha256,
        "/wheelchair_bringup/route_truth_sha256": binding.route_truth_sha256,
        "/wheelchair_bringup/scenario_sha256": binding.scenario_sha256,
        "/wheelchair_bringup/a13_sha256": binding.a13_sha256,
        "/wheelchair_bringup/claim_tag": binding.claim_tag,
    }
    for name, expected in exact.items():
        value = rospy.get_param(name, None)
        if type(value) is not type(expected) or value != expected:
            raise ScenarioError("preflight parameter %s is not exactly %r" % (name, expected))


class Evidence:
    """Queue-one callback state and readiness policy."""

    NAMES = ("safety", "localization", "geofence", "collision", "slope", "route")

    def __init__(self, binding, freshness_s):
        self.binding = binding
        self.freshness_s = freshness_s
        self.values = {}
        self.lock = threading.Lock()

    def update(self, name, message):
        with self.lock:
            self.values[name] = message

    @staticmethod
    def _stamp(message):
        return float(message.header.stamp.to_sec())

    def _fresh(self, message, now):
        stamp = self._stamp(message)
        age = now - stamp
        return math.isfinite(stamp) and math.isfinite(age) and 0.0 <= age <= self.freshness_s

    def startup_ready(self, now):
        with self.lock:
            if set(self.values) != set(self.NAMES):
                return False
            safety = self.values["safety"]
            localization = self.values["localization"]
            geofence = self.values["geofence"]
            collision = self.values["collision"]
            slope = self.values["slope"]
            route = self.values["route"]
            if not all(self._fresh(self.values[name], now) for name in self.NAMES):
                return False
            if not (safety.state == safety.DISARMED and not safety.armed
                    and not safety.estop_latched and int(safety.reason_mask) == STARTUP_REASON):
                return False
            if not (localization.state == localization.OK and localization.reason_mask == 0
                    and localization.map_id == self.binding.map_id
                    and localization.map_sha256 == self.binding.map_sha256):
                return False
            if not (geofence.state == geofence.INSIDE
                    and geofence.reason_mask == 0
                    and geofence.manifest_sha256 == self.binding.safety_manifest_sha256
                    and geofence.route_id == self.binding.route_id):
                return False
            if not (collision.state in (collision.STATE_CLEAR, collision.STATE_CAUTION)
                    and collision.reason_mask == 0):
                return False
            if not (slope.state in (slope.STATE_CLEAR, slope.STATE_SLOW)
                    and slope.reason_mask == 0):
                return False
            return (route.state == route.ACTIVE
                    and route.mission_id == self.binding.mission_id
                    and route.route_id == self.binding.route_id
                    and route.map_id == self.binding.map_id)

    def pre_initialization_ready(self, now):
        """Require stationary startup evidence before the one localization request."""
        with self.lock:
            required = ("safety", "collision", "slope", "route")
            if not all(name in self.values and self._fresh(self.values[name], now)
                       for name in required):
                return False
            safety = self.values["safety"]
            collision = self.values["collision"]
            slope = self.values["slope"]
            route = self.values["route"]
            if not (safety.state in (safety.DISARMED, safety.STOPPED)
                    and not safety.armed and not safety.estop_latched):
                return False
            if not (collision.state in (collision.STATE_CLEAR, collision.STATE_CAUTION)
                    and collision.reason_mask == 0):
                return False
            if not (slope.state in (slope.STATE_CLEAR, slope.STATE_SLOW)
                    and slope.reason_mask == 0):
                return False
            return (route.state == route.ACTIVE
                    and route.mission_id == self.binding.mission_id
                    and route.route_id == self.binding.route_id
                    and route.map_id == self.binding.map_id)

    def safety_clear(self, now):
        with self.lock:
            safety = self.values.get("safety")
            return bool(safety is not None and self._fresh(safety, now)
                        and safety.state == safety.CLEAR and safety.armed
                        and not safety.estop_latched and int(safety.reason_mask) == 0)


class ScenarioDriver:
    def __init__(self, rospy, binding, freshness_s, ready_timeout_s, action_timeout_s,
                 initial_pose, initial_pose_required_subscribers):
        import actionlib
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger
        from wheelchair_interfaces.msg import (CollisionStatus, ExecuteRouteAction,
                                                GeofenceStatus, LocalizationStatus,
                                                RouteProgress, SafetyState, SlopeStatus)

        self.rospy = rospy
        self.binding = binding
        self.ready_timeout_s = positive_timeout("readiness_timeout_sec", ready_timeout_s)
        self.action_timeout_s = positive_timeout("action_timeout_sec", action_timeout_s)
        freshness_s = positive_timeout("freshness_sec", freshness_s)
        self.evidence = Evidence(binding, freshness_s)
        self._goal_sent = False
        self._canceled = False
        self._safety_monitoring = False
        self._DiagnosticArray = DiagnosticArray
        self._DiagnosticStatus = DiagnosticStatus
        self._KeyValue = KeyValue
        self._PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self._Bool = Bool
        self._initial_pose_values = initial_pose
        self._initial_pose_required_subscribers = required_subscriber_count(
            initial_pose_required_subscribers)
        self._diagnostics = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self._initial_pose = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=False)
        self._arm = rospy.Publisher("/safety/arm", self._Bool, queue_size=1, latch=False)
        self._mission_arm = rospy.ServiceProxy("/wheelchair_mission/arm", Trigger)
        self._action = actionlib.SimpleActionClient("/wheelchair_mission/execute_route", ExecuteRouteAction)
        topics = (
            ("safety", "/safety/state", SafetyState),
            ("localization", "/localization/status", LocalizationStatus),
            ("geofence", "/route_safety/geofence_status", GeofenceStatus),
            ("collision", "/safety/collision_status", CollisionStatus),
            ("slope", "/safety/slope_status", SlopeStatus),
            ("route", "/route/progress", RouteProgress),
        )
        self._subscribers = [rospy.Subscriber(topic, msg_type, self._callback(name), queue_size=1)
                             for name, topic, msg_type in topics]

    def _callback(self, name):
        def callback(message):
            self.evidence.update(name, message)
            if self._safety_monitoring and name == "safety":
                now = float(self.rospy.Time.now().to_sec())
                if not self.evidence.safety_clear(now):
                    self._cancel("safety loss")
        return callback

    def _emit(self, level, message):
        status = self._DiagnosticStatus()
        status.level = level
        status.name = "wheelchair_gazebo/rc_scenario_driver"
        status.hardware_id = "simulation-only"
        status.message = message
        status.values = [
            self._KeyValue(key=name, value=str(getattr(self.binding, name)))
            for name in ("mission_id", "route_id", "direction_name", "scenario", "seed",
                         "claim_tag", "map_id", "map_sha256", "raw_route_asset_sha256",
                         "navigation_manifest_sha256", "directional_route_sha256",
                         "route_safety_config_sha256", "safety_manifest_sha256",
                         "route_truth_sha256", "scenario_sha256", "a13_sha256")]
        array = self._DiagnosticArray()
        array.header.stamp = self.rospy.Time.now()
        array.status = [status]
        self._diagnostics.publish(array)

    def _wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not self.rospy.is_shutdown() and time.monotonic() <= deadline:
            if predicate(float(self.rospy.Time.now().to_sec())):
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.05, remaining))
        raise ScenarioError("timeout waiting for %s" % description)

    def _cancel(self, reason):
        if self._goal_sent and not self._canceled:
            self._canceled = True
            self._action.cancel_goal()
            self._emit(self._DiagnosticStatus.ERROR, "action canceled: " + reason)

    def _wait_for_mission_action(self, terminal):
        deadline = time.monotonic() + self.action_timeout_s
        while not self.rospy.is_shutdown():
            if self._action.get_state() in terminal:
                return
            if not self.evidence.safety_clear(float(self.rospy.Time.now().to_sec())):
                self._cancel("safety loss")
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self._cancel("action timeout")
                raise ScenarioError("mission action timeout")
            time.sleep(min(0.05, remaining))
        self._cancel("ROS shutdown")
        raise ScenarioError("ROS shutdown during mission action")



    def _publish_initial_pose_message(self):
        if (self._initial_pose.get_num_connections()
                < self._initial_pose_required_subscribers):
            raise ScenarioError("initial pose subscribers disappeared")
        stamp = self.rospy.Time.now()
        if finite_value("ROS simulated time", stamp.to_sec()) <= 0.0:
            raise ScenarioError("ROS simulated time is not active")
        self._initial_pose.publish(build_initial_pose(
            self._PoseWithCovarianceStamped, stamp, *self._initial_pose_values))

    def _publish_initial_pose(self):
        deadline = time.monotonic() + self.ready_timeout_s
        while not self.rospy.is_shutdown():
            wall_now = time.monotonic()
            if wall_now > deadline:
                break
            if (self._initial_pose.get_num_connections()
                    >= self._initial_pose_required_subscribers):
                self._publish_initial_pose_message()
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.05, remaining))
        raise ScenarioError("timeout waiting for initial pose subscriber")

    def _publish_arm_low_baseline(self):
        """Publish the bounded simulation-only low observation required before arming."""
        deadline = time.monotonic() + self.ready_timeout_s
        while not self.rospy.is_shutdown():
            try:
                connected = self._arm.get_num_connections() >= 1
            except Exception as exc:
                raise ScenarioError("unable to check safety arm subscribers") from exc
            if connected:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ScenarioError("timeout waiting for safety arm subscriber")
            time.sleep(min(0.05, remaining))
        else:
            raise ScenarioError("ROS shutdown waiting for safety arm subscriber")

        for unused_attempt in range(3):
            try:
                self._arm.publish(self._Bool(data=False))
            except Exception as exc:
                raise ScenarioError("unable to publish safety arm low baseline") from exc
            time.sleep(0.02)

    def _publish_arm_request(self):
        try:
            self._arm.publish(self._Bool(data=True))
        except Exception as exc:
            raise ScenarioError("unable to publish safety arm request") from exc

    def run(self):
        from actionlib_msgs.msg import GoalStatus
        from wheelchair_interfaces.msg import ExecuteRouteGoal

        self._publish_arm_low_baseline()
        self.rospy.wait_for_service("/wheelchair_mission/arm", timeout=self.ready_timeout_s)
        response = self._mission_arm()
        if not response.success:
            raise ScenarioError("mission arm rejected: " + response.message)
        if not self._action.wait_for_server(self.rospy.Duration(self.ready_timeout_s)):
            raise ScenarioError("mission action unavailable")
        goal = ExecuteRouteGoal()
        goal.mission_id = self.binding.mission_id
        goal.route_id = self.binding.route_id
        goal.direction = self.binding.direction
        goal.map_id = self.binding.map_id
        goal.map_sha256 = self.binding.map_sha256
        goal.route_manifest_sha256 = self.binding.directional_route_sha256
        goal.safety_manifest_sha256 = self.binding.safety_manifest_sha256
        self._action.send_goal(goal)
        self._goal_sent = True
        try:
            self._wait(lambda unused_now: self._action.get_state() == GoalStatus.ACTIVE,
                       self.ready_timeout_s, "active mission goal")
            self._emit(self._DiagnosticStatus.OK,
                       "waiting for fresh stationary startup evidence")
            self._wait(self.evidence.pre_initialization_ready, self.ready_timeout_s,
                       "pre-initialization evidence")
            self._emit(self._DiagnosticStatus.OK, "publishing initial localization pose")
            self._publish_initial_pose()
            self._wait(self.evidence.startup_ready, self.ready_timeout_s, "startup evidence")
            self._publish_arm_request()
            self._wait(self.evidence.safety_clear, self.ready_timeout_s, "armed safety gate")
        except ScenarioError:
            self._cancel("startup failure")
            raise
        self._safety_monitoring = True
        terminal = {GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                    GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST}
        self._wait_for_mission_action(terminal)
        if self._canceled or self._action.get_state() != GoalStatus.SUCCEEDED:
            raise ScenarioError("mission did not succeed (state %s)" % self._action.get_state())
        result = self._action.get_result()
        if result is None or not result.success:
            raise ScenarioError("mission returned an unsuccessful result")
        self._emit(self._DiagnosticStatus.OK, "mission succeeded")


def main():
    import rospy

    rospy.init_node("rc_scenario_driver", anonymous=False)
    try:
        direction = rospy.get_param("~direction", "outbound")
        route_truth = rospy.get_param("~route_truth")
        binding = load_binding(
            route_truth,
            rospy.get_param("~route_truth_sha256"),
            direction,
            rospy.get_param("~scenario"),
            rospy.get_param("~seed"),
            rospy.get_param("~scenario_sha256"),
            rospy.get_param("~a13_sha256"),
            rospy.get_param("~claim_tag"),
        )
        require_preflight(rospy, binding)
        wait_for_sim_time(
            rospy,
            rospy.get_param("~sim_time_timeout_sec", 30.0),
        )
        initial_pose = tuple(
            finite_value(name, rospy.get_param("~" + name, 0.0))
            for name in ("initial_pose_x", "initial_pose_y", "initial_pose_yaw")
        )
        driver = ScenarioDriver(
            rospy, binding,
            float(rospy.get_param("~freshness_sec", 0.5)),
            float(rospy.get_param("~readiness_timeout_sec", 30.0)),
            float(rospy.get_param("~action_timeout_sec", 900.0)),
            initial_pose,
            required_subscriber_count(
                rospy.get_param("~initial_pose_required_subscribers", 2)),
        )
        driver.run()
    except Exception as exc:
        rospy.logfatal("scenario driver fail-closed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
