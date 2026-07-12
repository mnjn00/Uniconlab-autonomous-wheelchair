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
    map_id: str
    map_sha256: str
    route_manifest_sha256: str
    safety_manifest_sha256: str


def _sha256(raw):
    return hashlib.sha256(raw).hexdigest()
def _valid_sha256(value):
    return (isinstance(value, str) and len(value) == 64
            and all(character in "0123456789abcdef" for character in value))




def load_binding(path, expected_file_sha256, direction, scenario, seed):
    """Load and exactly bind one route from an immutable manifest."""
    if direction not in DIRECTION_VALUES:
        raise ScenarioError("direction must be outbound or return")
    with open(path, "rb") as stream:
        raw = stream.read()
    if _sha256(raw) != expected_file_sha256:
        raise ScenarioError("route manifest SHA-256 mismatch")
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise ScenarioError("invalid route manifest: %s" % exc)
    if not isinstance(document, dict) or document.get("immutable") is not True:
        raise ScenarioError("route manifest is not immutable")
    route = document.get(ROUTE_KEYS[direction])
    map_binding = document.get("map")
    if not isinstance(route, dict) or not isinstance(map_binding, dict):
        raise ScenarioError("selected route/map binding is missing")
    required = {
        "route_id": route.get("route_id"),
        "map_id": map_binding.get("map_id"),
        "map_sha256": map_binding.get("sha256"),
        "route_manifest_sha256": route.get("route_manifest_sha256"),
        "safety_manifest_sha256": document.get("safety_manifest_sha256"),
    }
    if route.get("direction") != direction:
        raise ScenarioError("selected route direction mismatch")
    if any(not isinstance(value, str) or not value for value in required.values()):
        raise ScenarioError("route identity is incomplete")
    if not all(_valid_sha256(required[name]) for name in (
            "map_sha256", "route_manifest_sha256", "safety_manifest_sha256")):
        raise ScenarioError("route identity contains an invalid SHA-256")
    material = "%s\n%s\n%s\n%s" % (scenario, int(seed), direction, required["route_id"])
    mission_id = "rc-" + hashlib.sha256(material.encode("utf-8")).hexdigest()[:24]
    return RouteBinding(mission_id=mission_id, direction=DIRECTION_VALUES[direction], **required)
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
        "/wheelchair_bringup/map_sha256": binding.map_sha256,
        "/wheelchair_bringup/route_sha256": binding.route_manifest_sha256,
    }
    for name, expected in exact.items():
        value = rospy.get_param(name, None)
        if type(value) is not type(expected) or value != expected:
            raise ScenarioError("preflight parameter %s is not exactly %r" % (name, expected))
    if rospy.get_param("~map_id", None) != binding.map_id:
        raise ScenarioError("map identity mismatch")
    if rospy.get_param("~safety_manifest_sha256", None) != binding.safety_manifest_sha256:
        raise ScenarioError("safety identity mismatch")


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
            if not (geofence.state in (geofence.INSIDE, geofence.MARGIN)
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
        self._initial_pose_values = initial_pose
        self._initial_pose_required_subscribers = required_subscriber_count(
            initial_pose_required_subscribers)
        self._diagnostics = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self._initial_pose = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=False)
        self._arm = rospy.Publisher("/safety/arm", Bool, queue_size=1, latch=False)
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
        status.values = [self._KeyValue(key="mission_id", value=self.binding.mission_id)]
        array = self._DiagnosticArray()
        array.header.stamp = self.rospy.Time.now()
        array.status = [status]
        self._diagnostics.publish(array)

    def _wait(self, predicate, timeout, description, tick=None, tick_interval_s=1.0):
        deadline = time.monotonic() + timeout
        next_tick = time.monotonic() + tick_interval_s if tick is not None else None
        rate = self.rospy.Rate(20)
        while not self.rospy.is_shutdown() and time.monotonic() <= deadline:
            if predicate(float(self.rospy.Time.now().to_sec())):
                return
            wall_now = time.monotonic()
            if tick is not None and wall_now >= next_tick:
                tick()
                next_tick = wall_now + tick_interval_s
            if tick is None:
                rate.sleep()
            else:
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
    def run(self):
        from actionlib_msgs.msg import GoalStatus
        from std_msgs.msg import Bool
        from wheelchair_interfaces.msg import ExecuteRouteGoal

        self._publish_initial_pose()
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
        goal.route_manifest_sha256 = self.binding.route_manifest_sha256
        goal.safety_manifest_sha256 = self.binding.safety_manifest_sha256
        self._action.send_goal(goal)
        self._goal_sent = True
        try:
            self._wait(lambda unused_now: self._action.get_state() == GoalStatus.ACTIVE,
                       self.ready_timeout_s, "active mission goal")
            self._emit(self._DiagnosticStatus.OK, "waiting for fresh disarmed evidence")
            self._wait(self.evidence.startup_ready, self.ready_timeout_s, "startup evidence",
                       tick=self._publish_initial_pose_message)
            self._arm.publish(Bool(data=True))
            self._wait(self.evidence.safety_clear, self.ready_timeout_s, "armed safety gate")
        except ScenarioError:
            self._cancel("startup failure")
            raise
        self._safety_monitoring = True
        deadline = time.monotonic() + self.action_timeout_s
        rate = self.rospy.Rate(20)
        terminal = {GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                    GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST}
        while not self.rospy.is_shutdown() and self._action.get_state() not in terminal:
            if not self.evidence.safety_clear(float(self.rospy.Time.now().to_sec())):
                self._cancel("safety loss")
            if time.monotonic() > deadline:
                self._cancel("action timeout")
                raise ScenarioError("mission action timeout")
            rate.sleep()
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
        manifest = rospy.get_param("~route_manifest")
        binding = load_binding(
            manifest,
            rospy.get_param("~route_policy_sha256"),
            direction,
            rospy.get_param("~scenario"),
            rospy.get_param("~seed"),
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
