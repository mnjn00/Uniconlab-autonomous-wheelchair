#!/usr/bin/env python3
"""ROS adapter for the deterministic mission state machine.

ROS imports intentionally live in :func:`main`; importing this module is safe in
unit tests and on developer machines without ROS installed.  This process only
orchestrates ``move_base`` and publishes bounded intent evidence.  It has no
velocity-output authority.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional


@dataclass(frozen=True)
class RouteBinding:
    mission_id: str
    route_id: str
    direction: str
    map_id: str
    map_sha256: str
    route_manifest_sha256: str
    safety_manifest_sha256: str
    route: Any


def bind_route(manifest: Any, goal: Any) -> RouteBinding:
    """Resolve an action goal against every immutable manifest identity."""
    directions = {1: "outbound", 2: "return"}
    try:
        direction = directions[int(goal.direction)]
    except (KeyError, TypeError, ValueError):
        raise ValueError("unsupported route direction")
    route = manifest.route(direction)
    expected = (
        (goal.route_id, route.route_id, "route id"),
        (goal.map_id, manifest.map_id, "map id"),
        (goal.map_sha256, manifest.map_sha256, "map hash"),
        (goal.route_manifest_sha256, route.route_manifest_sha256, "route hash"),
        (goal.safety_manifest_sha256, manifest.safety_manifest_sha256, "safety hash"),
    )
    for requested, actual, label in expected:
        if not requested or requested != actual:
            raise ValueError("%s mismatch" % label)
    mission_id = str(goal.mission_id)
    if not mission_id or len(mission_id) > 64:
        raise ValueError("invalid mission id")
    return RouteBinding(mission_id, route.route_id, direction, manifest.map_id,
                        manifest.map_sha256, route.route_manifest_sha256,
                        manifest.safety_manifest_sha256, route)


def _enum_name(value: Any) -> str:
    return str(getattr(value, "name", value)).upper()


def _collision_evidence(state: int, clear_state: int, caution_state: int) -> str:
    """Treat non-blocking CAUTION as speed-limited clear, never as physical e-stop."""
    return "clear" if state in (clear_state, caution_state) else "blocked"


def _move_base_failure_reason(reason: Any) -> bool:
    """Distinguish action failures from the normal ``move_base_active`` state."""
    normalized = str(reason).strip().lower()
    return any(token in normalized for token in (
        "move_base_lost",
        "move_base_aborted",
        "stale_move_base",
        "move_base action unavailable",
        "action callback",
    ))


def classify_speed_zone(zone_ids: Any) -> str:
    """Classify surveyed tags or the exact simulation-only candidate zone."""
    normalized = {str(zone_id).strip().lower() for zone_id in zone_ids}
    normalized.discard("")
    if any("road" in zone_id for zone_id in normalized):
        return "road"
    if any("sidewalk" in zone_id for zone_id in normalized):
        return "sidewalk"
    if normalized == {"candidate-unsurveyed"}:
        return "simulation_unsurveyed"
    raise ValueError("active zone is not speed classified")


def next_waypoint_index(reached_index: int, waypoint_count: int) -> int:
    """Translate route-manager's latest-reached index into the next goal index."""
    if (isinstance(reached_index, bool) or not isinstance(reached_index, int)
            or isinstance(waypoint_count, bool) or not isinstance(waypoint_count, int)
            or waypoint_count <= 0 or reached_index < 0
            or reached_index >= waypoint_count):
        raise ValueError("invalid route progress index")
    return min(reached_index + 1, waypoint_count - 1)
@dataclass
class RouteActiveHeartbeat:
    """Schedule route ownership receipts against a reset-independent clock."""

    period_s: float
    last_publish_s: Optional[float] = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.period_s) or self.period_s <= 0.0:
            raise ValueError("route heartbeat period must be positive and finite")

    def record(self, now_s: float) -> None:
        if not math.isfinite(now_s):
            raise ValueError("route heartbeat time must be finite")
        self.last_publish_s = now_s

    def delay_s(self, now_s: float) -> float:
        if not math.isfinite(now_s):
            raise ValueError("route heartbeat time must be finite")
        if self.last_publish_s is None or now_s < self.last_publish_s:
            return 0.0
        return max(0.0, self.period_s - (now_s - self.last_publish_s))



def _mission_cancelled(state: Any, active_states: Any = None) -> bool:
    """Report whether no active-motion mission owns a move_base goal."""
    if active_states is not None:
        return state not in active_states
    return _enum_name(state) not in ("NAVIGATING", "PAUSED_OBSTACLE")


class PublicationLimiter:
    """Bound a stream while publishing restrictive changes immediately."""

    def __init__(self, period_sec: float) -> None:
        if (not isinstance(period_sec, (int, float))
                or isinstance(period_sec, bool)
                or not math.isfinite(period_sec)
                or period_sec <= 0.0):
            raise ValueError("publication period must be finite and positive")
        self._period_sec = float(period_sec)
        self._signature = None
        self._published_signature = None
        self._last_publish_s = None

    @property
    def published_signature(self) -> Any:
        return self._published_signature

    def should_publish(self, signature: Any, now_s: float,
                       urgent: bool = True) -> bool:
        finite_now = (isinstance(now_s, (int, float))
                      and not isinstance(now_s, bool)
                      and math.isfinite(now_s))
        changed = self._signature is None or signature != self._signature
        if changed:
            self._signature = signature
            if self._last_publish_s is None:
                if finite_now:
                    self._last_publish_s = float(now_s)
                self._published_signature = signature
                return True
            if urgent:
                if finite_now and now_s >= self._last_publish_s:
                    self._last_publish_s = float(now_s)
                self._published_signature = signature
                return True
        if (not finite_now or self._last_publish_s is None
                or now_s < self._last_publish_s):
            return False
        if now_s - self._last_publish_s + 1.0e-12 < self._period_sec:
            return False
        self._last_publish_s = float(now_s)
        self._published_signature = signature
        return True


URGENT_LINEAR_CAP_REDUCTION_MPS = 0.05
URGENT_ANGULAR_CAP_REDUCTION_RPS = 0.10


def publication_change_is_urgent(previous: Any, current: Any,
                                 proceed_behavior: int) -> bool:
    """Publish hazards immediately and coalesce bounded nominal cap shaping."""
    if previous is None:
        return True
    previous_state = previous[0]
    previous_state_reason = previous[1]
    previous_behavior = previous[2]
    previous_intent_reason = previous[3]
    previous_cancelled = previous[9]
    state = current[0]
    state_reason = current[1]
    behavior = current[2]
    intent_reason = current[3]
    cancelled = current[9]
    current_restrictive = (
        state_reason != 0
        or intent_reason != 0
        or behavior != proceed_behavior
        or cancelled
    )
    if current_restrictive:
        return True
    previous_restrictive = (
        previous_state_reason != 0
        or previous_intent_reason != 0
        or previous_behavior != proceed_behavior
        or previous_cancelled
    )
    if previous_restrictive:
        # Permission expansion is deliberately rate-limited.  A fresh PROCEED
        # sample is never more urgent than the already-published restriction.
        return False
    if (
        state != previous_state
        or previous[6:9] != current[6:9]
    ):
        return True
    previous_linear = float(previous[4])
    previous_angular = float(previous[5])
    current_linear = float(current[4])
    current_angular = float(current[5])
    return (
        previous_linear - current_linear
        >= URGENT_LINEAR_CAP_REDUCTION_MPS
        or previous_angular - current_angular
        >= URGENT_ANGULAR_CAP_REDUCTION_RPS
    )


class DeferredWaypointQueue:
    """Single-slot handoff that prevents action-client callback reentrancy."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pending: Optional[int] = None

    def defer(self, index: int) -> None:
        if isinstance(index, bool) or not isinstance(index, int) or index < 0:
            raise ValueError("waypoint index must be a nonnegative integer")
        with self._lock:
            self._pending = index

    def pop(self) -> Optional[int]:
        with self._lock:
            index = self._pending
            self._pending = None
        return index

    def clear(self) -> None:
        with self._lock:
            self._pending = None


class MissionRuntime:
    """Small ROS-agnostic bridge around ``MissionFSM`` and an action client."""

    def __init__(self, fsm: Any, event_type: Any, event_factory: Callable[..., Any],
                 action_client: Any, clock: Callable[[], float], emit: Callable[[Any, float], None]) -> None:
        self._fsm = fsm
        self._event_type = event_type
        self._event_factory = event_factory
        self._action = action_client
        self._clock = clock
        self._emit = emit
        self._lock = threading.RLock()
        self.output = None
        self.binding: Optional[RouteBinding] = None
        self.armed_by_operator = False
        self.fault_latched = False

    def operator_arm(self) -> None:
        with self._lock:
            state = _enum_name(self.output.state if self.output is not None else self._fsm.state)
            if state != "DISARMED":
                raise ValueError("arm is only valid while disarmed")
            self.armed_by_operator = True

    def operator_reset(self) -> Any:
        with self._lock:
            output = self.dispatch("RESET")
            if _enum_name(output.state) != "DISARMED":
                raise ValueError("reset is only valid after a terminal fault or result")
            self._action.cancel_goal()
            self.fault_latched = False
            self.armed_by_operator = False
            return output

    def dispatch(self, kind: str, value: Any = None,
                 stamp: Optional[float] = None) -> Any:
        with self._lock:
            member = getattr(self._event_type, kind)
            event = self._event_factory(member, value)
            output = self._fsm.update(event, self._clock() if stamp is None else stamp)
            self.output = output
            state = _enum_name(output.state)
            if state in ("FAULT", "ABORTED"):
                self.fault_latched = True
                self.armed_by_operator = False
            if bool(output.cancel_goal):
                self._action.cancel_goal()
            self._emit(output, self._clock() if stamp is None else stamp)
            return output

    def begin(self, binding: RouteBinding) -> Any:
        with self._lock:
            if not self.armed_by_operator or self.fault_latched:
                raise ValueError("operator arm/reset required")
            self.armed_by_operator = False
            self.binding = binding
            route = {
                "route_id": binding.route_id,
                "map_id": binding.map_id,
                "route_hash": binding.route_manifest_sha256,
                "waypoint_count": len(binding.route.waypoints),
                "valid": True,
                "map_valid": True,
                "hash_valid": True,
                "graph_valid": True,
            }
            output = self.dispatch("ARM", value=route)
            return output

    def fail_closed(self, reason: str) -> None:
        """Cancel first, then attempt to leave a HOLD/fault evidence sample."""
        with self._lock:
            self._action.cancel_goal()
            self.fault_latched = True
            self.armed_by_operator = False
            for event in ("MOVE_BASE_LOST", "DISARM"):
                if hasattr(self._event_type, event):
                    try:
                        self.dispatch(event, value=reason)
                        return
                    except Exception:
                        continue


def main() -> None:
    # Lazy ROS and generated-message imports are a deliberate safety property.
    import actionlib
    import rospy
    import rospkg
    from actionlib_msgs.msg import GoalStatus
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger, TriggerResponse
    from wheelchair_interfaces.msg import (ActiveRoute, CollisionStatus,
                                            GeofenceStatus, LocalizationStatus,
                                            MissionState, MotionIntent,
                                            RouteProgress, SafetyReason,
                                            SafetyState, SlopeStatus)
    from wheelchair_interfaces.msg import (ExecuteRouteAction, ExecuteRouteFeedback,
                                            ExecuteRouteResult)

    navigation_package = rospkg.RosPack().get_path("wheelchair_navigation")
    dependency_paths = (
        os.path.dirname(os.path.abspath(__file__)),
        os.path.join(navigation_package, "scripts"),
    )
    for dependency_path in reversed(dependency_paths):
        if dependency_path not in sys.path:
            sys.path.insert(0, dependency_path)
    from mission_core import EventType, MissionConfig, MissionEvent, MissionFSM
    from route_manager import load_manifest

    rospy.init_node("wheelchair_mission")
    manifest = load_manifest(rospy.get_param("~route_manifest"), verify_assets=True)
    intent_period = float(rospy.get_param("~intent_period_sec", 0.1))
    if not math.isfinite(intent_period) or intent_period <= 0.0 or intent_period > 0.25:
        raise ValueError("intent_period_sec must be in (0, 0.25]")
    policy_core = None
    policy_config = None
    policy_error = ""
    try:
        import yaml
        from wheelchair_navigation.speed_policy import (
            SpeedEvidence,
            SpeedPolicyConfig,
            SpeedPolicyCore,
        )
        policy_path = rospy.get_param(
            "~speed_policy_config",
            os.path.join(navigation_package, "config", "speed_policy.yaml"),
        )
        with open(policy_path, "r", encoding="utf-8") as stream:
            policy_mapping = yaml.safe_load(stream)
        policy_config = SpeedPolicyConfig.from_mapping(policy_mapping)
        policy_core = SpeedPolicyCore(policy_config)
    except Exception as exc:
        policy_error = "invalid speed policy: %s" % exc
        rospy.logerr(policy_error)

    active_pub = rospy.Publisher("/route/active", ActiveRoute, queue_size=1, latch=False)
    state_pub = rospy.Publisher("/mission/state", MissionState, queue_size=1, latch=False)
    intent_pub = rospy.Publisher("/decision/motion_intent", MotionIntent, queue_size=1, latch=False)
    mission_cancelled_pub = rospy.Publisher(
        "/safety/mission_cancelled", Bool, queue_size=1, latch=False)
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    latest_progress = RouteProgress()
    latest_state = MissionState()
    activation_sequence = 0
    active_lock = threading.Lock()
    sequence = 0
    publication_limiter = PublicationLimiter(intent_period)
    policy_inputs = {
        "progress": None,
        "geofence": None,
        "localization": None,
        "slope": None,
        "collision": None,
        "odometry_speed_mps": float("nan"),
        "arrival": {
            "progress": 0.0,
            "geofence": 0.0,
            "localization": 0.0,
            "slope": 0.0,
            "collision": 0.0,
            "odometry": 0.0,
        },
    }

    state_codes = {name: getattr(MissionState, name) for name in
                   ("DISARMED", "LOCALIZING", "READY", "NAVIGATING", "PAUSED_OBSTACLE",
                    "PAUSED_SAFETY", "GOAL_REACHED", "ABORTED", "FAULT")}
    intent_codes = {name: getattr(MotionIntent, name) for name in ("HOLD", "PROCEED", "SLOW")}

    def ros_stamp(seconds: float):
        return rospy.Time.from_sec(max(0.0, float(seconds)))

    def reason_mask(output: Any) -> int:
        reason = str(getattr(output, "reason", "")).lower()
        mask = 0
        for token, bit in (
            ("mode", SafetyReason.MODE),
            ("geofence", SafetyReason.GEOFENCE),
            ("collision", SafetyReason.COLLISION),
            ("obstacle", SafetyReason.COLLISION),
            ("localization", SafetyReason.LOCALIZATION),
            ("driver", SafetyReason.DRIVER),
            ("slope", SafetyReason.SLOPE),
            ("route", SafetyReason.ROUTE_MANIFEST),
            ("map", SafetyReason.MAP_MISMATCH),
            ("hash", SafetyReason.POLICY_MISMATCH),
            ("stale", SafetyReason.SENSOR_STALE),
            ("invalid", SafetyReason.INPUT_UNKNOWN),
        ):
            if token in reason:
                mask |= int(bit)
        if _move_base_failure_reason(reason):
            mask |= int(SafetyReason.INTERNAL_FAULT)
        return mask
    def active_segment():
        binding = runtime.binding
        progress = policy_inputs["progress"]
        if binding is None or progress is None:
            raise ValueError("route progress unavailable")
        if (progress.mission_id != binding.mission_id
                or progress.route_id != binding.route_id
                or progress.map_id != binding.map_id):
            raise ValueError("route progress identity mismatch")
        matches = [segment for segment in binding.route.segments
                   if segment.segment_id == progress.segment_id]
        if len(matches) != 1:
            raise ValueError("active segment unavailable")
        return matches[0]

    def curvature_for(segment: Any) -> float:
        route = runtime.binding.route
        index = segment.start_waypoint_index
        if index + 2 >= len(route.waypoints):
            return 0.0
        first = route.waypoints[index]
        second = route.waypoints[index + 1]
        third = route.waypoints[index + 2]
        heading_a = math.atan2(second.y_m - first.y_m, second.x_m - first.x_m)
        heading_b = math.atan2(third.y_m - second.y_m, third.x_m - second.x_m)
        turn = abs(math.atan2(math.sin(heading_b - heading_a),
                              math.cos(heading_b - heading_a)))
        distance = math.hypot(second.x_m - first.x_m,
                              second.y_m - first.y_m)
        if not math.isfinite(distance) or distance <= 0.0:
            raise ValueError("invalid route segment geometry")
        return turn / distance

    def policy_cap(output: Any) -> tuple:
        if policy_core is None or policy_config is None:
            raise ValueError(policy_error or "speed policy unavailable")
        segment = active_segment()
        geofence = policy_inputs["geofence"]
        localization = policy_inputs["localization"]
        zone_ids = []
        if geofence is not None:
            zone_ids.append(str(geofence.zone_id).lower())
        if localization is not None:
            zone_ids.append(str(localization.zone_id).lower())
        zone_ids.extend(str(zone).lower() for zone in segment.zone_ids)
        zone = classify_speed_zone(zone_ids)
        if zone == "road":
            zone_cap = policy_config.road_cap_mps
        elif zone == "sidewalk":
            zone_cap = policy_config.sidewalk_cap_mps
        else:
            zone_cap = policy_config.simulation_unsurveyed_cap_mps
        slope = policy_inputs["slope"]
        collision = policy_inputs["collision"]
        if localization is None or slope is None or collision is None:
            raise ValueError("speed evidence incomplete")
        arrivals = policy_inputs["arrival"]
        evidence_stamp = min(arrivals.values())
        now = time.monotonic()
        evidence = SpeedEvidence(
            segment_cap_mps=float(segment.max_linear_mps),
            zone_cap_mps=float(zone_cap),
            hard_cap_mps=float(output.max_linear_mps),
            curvature_inv_m=curvature_for(segment),
            zone=zone,
            localization_state=int(localization.state),
            localization_confidence=float(localization.inlier_ratio),
            localization_policy_sha256=str(localization.policy_sha256),
            slope_state=int(slope.state),
            pitch_rad=float(slope.pitch_rad),
            slope_recommended_cap_mps=float(slope.recommended_max_linear_mps),
            slope_policy_sha256=str(slope.policy_sha256),
            collision_state=int(collision.state),
            collision_ttc_s=float(collision.time_to_collision_s),
            collision_recommended_cap_mps=float(collision.recommended_max_linear_mps),
            collision_policy_sha256=str(collision.policy_sha256),
            odometry_speed_mps=float(policy_inputs["odometry_speed_mps"]),
            evidence_stamp_monotonic_s=evidence_stamp,
            now_monotonic_s=now,
        )
        evaluated_cap = float(policy_core.evaluate(evidence))
        if not math.isfinite(evaluated_cap) or evaluated_cap <= 0.0:
            raise ValueError(
                "speed policy rejected evidence "
                "(age={:.3f}s zone={} states={}/{}/{} "
                "caps={:.3f}/{:.3f}/{:.3f} curvature={:.3f})".format(
                    now - evidence_stamp,
                    zone,
                    evidence.localization_state,
                    evidence.slope_state,
                    evidence.collision_state,
                    evidence.segment_cap_mps,
                    evidence.zone_cap_mps,
                    evidence.hard_cap_mps,
                    evidence.curvature_inv_m,
                )
            )
        return evaluated_cap, float(segment.max_angular_rps)


    def emit(output: Any, stamp: float) -> None:
        nonlocal sequence, latest_state
        state_code = state_codes.get(_enum_name(output.state), MissionState.FAULT)
        state_reason = reason_mask(output)
        mission_id = runtime.binding.mission_id if runtime.binding else ""
        route_id = runtime.binding.route_id if runtime.binding else ""
        map_id = runtime.binding.map_id if runtime.binding else manifest.map_id
        behavior = intent_codes.get(_enum_name(output.intent), MotionIntent.HOLD)
        linear_cap = 0.0
        angular_cap = 0.0
        intent_reason = state_reason
        if runtime.fault_latched:
            behavior = MotionIntent.HOLD
            intent_reason |= int(SafetyReason.INTERNAL_FAULT)
            state_code = MissionState.FAULT
            state_reason |= int(SafetyReason.INTERNAL_FAULT)
        if behavior != MotionIntent.HOLD:
            try:
                evaluated_cap, segment_angular_cap = policy_cap(output)
                linear_cap = min(max(0.0, float(output.max_linear_mps)),
                                 max(0.0, float(evaluated_cap)))
                angular_cap = min(max(0.0, float(output.max_angular_rps)),
                                  max(0.0, segment_angular_cap))
                if (not math.isfinite(linear_cap)
                        or not math.isfinite(angular_cap)
                        or linear_cap <= 0.0):
                    raise ValueError("speed policy rejected current evidence")
            except Exception as exc:
                behavior = MotionIntent.HOLD
                intent_reason |= int(SafetyReason.INTERNAL_FAULT)
                runtime.fault_latched = True
                runtime.armed_by_operator = False
                move_base.cancel_goal()
                rospy.logerr_throttle(1.0, "speed policy HOLD: %s" % exc)
                state_code = MissionState.FAULT
                state_reason |= int(SafetyReason.INTERNAL_FAULT)
        cancelled = _mission_cancelled(
            state_code, (MissionState.NAVIGATING, MissionState.PAUSED_OBSTACLE))
        signature = (state_code, state_reason, behavior, intent_reason,
                     linear_cap, angular_cap, mission_id, route_id, map_id,
                     cancelled)
        urgent = publication_change_is_urgent(
            publication_limiter.published_signature,
            signature,
            MotionIntent.PROCEED,
        )
        if not publication_limiter.should_publish(signature, stamp, urgent=urgent):
            return
        sequence += 1
        header_stamp = ros_stamp(stamp)
        state = MissionState()
        state.header.stamp = header_stamp
        state.header.frame_id = "map"
        state.sequence = sequence
        state.state = state_code
        state.reason_mask = state_reason
        state.mission_id = mission_id
        state.route_id = route_id
        state.map_id = map_id
        latest_state = state
        state_pub.publish(state)
        mission_cancelled_pub.publish(Bool(data=cancelled))
        intent = MotionIntent()
        intent.header.stamp = header_stamp
        intent.header.frame_id = "base_link"
        intent.sequence = sequence
        intent.behavior = behavior
        intent.reason_mask = intent_reason
        intent.mission_id = mission_id
        intent.max_linear_mps = linear_cap
        intent.max_angular_rps = angular_cap
        intent_pub.publish(intent)

    clock = lambda: rospy.Time.now().to_sec()
    fsm = MissionFSM(MissionConfig(), clock)
    runtime = MissionRuntime(fsm, EventType, MissionEvent, move_base, clock, emit)

    def publish_active(binding: RouteBinding, new_activation: bool = True) -> None:
        nonlocal activation_sequence
        with active_lock:
            if new_activation:
                activation_sequence += 1
            selected_sequence = activation_sequence
        message = ActiveRoute()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.activation_sequence = selected_sequence
        message.direction = ActiveRoute.DIRECTION_OUTBOUND if binding.direction == "outbound" else ActiveRoute.DIRECTION_RETURN
        message.mission_id = binding.mission_id
        message.route_id = binding.route_id
        message.map_id = binding.map_id
        message.map_sha256 = binding.map_sha256
        message.route_manifest_sha256 = binding.route_manifest_sha256
        message.safety_manifest_sha256 = binding.safety_manifest_sha256
        active_pub.publish(message)
    def start_active_heartbeat(binding: RouteBinding) -> tuple:
        stopped = threading.Event()
        heartbeat = RouteActiveHeartbeat(0.5)
        heartbeat.record(time.monotonic())

        def run() -> None:
            while not stopped.wait(heartbeat.delay_s(time.monotonic())):
                try:
                    publish_active(binding, new_activation=False)
                    heartbeat.record(time.monotonic())
                except Exception as exc:
                    runtime.fail_closed("route active heartbeat: %s" % exc)
                    return

        thread = threading.Thread(target=run, name="route-active-heartbeat", daemon=True)
        thread.start()
        return stopped, thread

    def stop_active_heartbeat(handle: tuple) -> None:
        stopped, thread = handle
        stopped.set()
        thread.join()

    def send_waypoint(index: int) -> None:
        binding = runtime.binding
        if binding is None or index < 0 or index >= len(binding.route.waypoints):
            runtime.fail_closed("invalid waypoint request")
            return
        waypoint = binding.route.waypoints[index]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.x_m
        goal.target_pose.pose.position.y = waypoint.y_m
        goal.target_pose.pose.orientation.z = math.sin(waypoint.yaw_rad * 0.5)
        goal.target_pose.pose.orientation.w = math.cos(waypoint.yaw_rad * 0.5)

        def active_cb() -> None:
            runtime.dispatch("MOVE_BASE_ACTIVE")

        def done_cb(status: int, _result: Any) -> None:
            event = {GoalStatus.SUCCEEDED: "MOVE_BASE_SUCCEEDED",
                     GoalStatus.ABORTED: "MOVE_BASE_ABORTED",
                     GoalStatus.REJECTED: "MOVE_BASE_ABORTED",
                     GoalStatus.LOST: "MOVE_BASE_LOST",
                     GoalStatus.PREEMPTED: "MOVE_BASE_LOST",
                     GoalStatus.RECALLED: "MOVE_BASE_LOST"}.get(status, "MOVE_BASE_LOST")
            try:
                apply(runtime.dispatch(event), deferred=True)
            except Exception as exc:
                runtime.fail_closed("action callback: %s" % exc)

        def feedback_cb(_feedback: Any) -> None:
            try:
                apply(runtime.dispatch("MOVE_BASE_ACTIVE"))
            except Exception as exc:
                runtime.fail_closed("action feedback: %s" % exc)

        if not move_base.wait_for_server(rospy.Duration(0.2)):
            runtime.fail_closed("move_base action unavailable")
            return
        move_base.send_goal(goal, done_cb=done_cb, active_cb=active_cb,
                            feedback_cb=feedback_cb)

    last_sent = None
    waypoint_queue = DeferredWaypointQueue()

    def send_pending_index(index: int) -> None:
        nonlocal last_sent
        if index == last_sent:
            return
        last_sent = index
        send_waypoint(index)

    def apply(output: Any, deferred: bool = False) -> None:
        index = getattr(output, "send_waypoint_index", None)
        if index is None or index == last_sent:
            return
        if deferred:
            waypoint_queue.defer(int(index))
            return
        send_pending_index(int(index))

    def drain_deferred_waypoint() -> None:
        index = waypoint_queue.pop()
        if index is not None:
            send_pending_index(index)

    def dispatch(kind: str, value: Any, _source_stamp: Any) -> None:
        try:
            # Independent guard publications are not globally timestamp ordered.
            # The FSM's freshness clock must therefore use serialized receipt-time
            # at the runtime lock, while each guard validates its own source age.
            output = runtime.dispatch(kind, value=value)
            apply(output)
        except Exception as exc:
            runtime.fail_closed("evidence callback: %s" % exc)

    def safety_cb(msg: Any) -> None:
        good = msg.state == SafetyState.CLEAR and msg.armed and not msg.estop_latched
        dispatch("MODE", good, msg.header.stamp)
        dispatch("DRIVER", good, msg.header.stamp)

    def geofence_cb(msg: Any) -> None:
        policy_inputs["geofence"] = msg
        policy_inputs["arrival"]["geofence"] = time.monotonic()
        binding = runtime.binding
        good = bool(
            binding is not None
            and msg.state in (GeofenceStatus.INSIDE, GeofenceStatus.MARGIN)
            and msg.route_id == binding.route_id
            and msg.manifest_sha256 == binding.safety_manifest_sha256
        )
        stamp = msg.evaluation_stamp if msg.evaluation_stamp.to_sec() else msg.header.stamp
        dispatch("GEOFENCE", good, stamp)

    def localization_cb(msg: Any) -> None:
        policy_inputs["localization"] = msg
        policy_inputs["arrival"]["localization"] = time.monotonic()
        valid_map = msg.map_id == manifest.map_id and msg.map_sha256 == manifest.map_sha256
        dispatch("LOCALIZATION", msg.state == LocalizationStatus.OK and valid_map,
                 msg.evaluation_stamp if msg.evaluation_stamp.to_sec() else msg.header.stamp)

    def collision_cb(msg: Any) -> None:
        policy_inputs["collision"] = msg
        policy_inputs["arrival"]["collision"] = time.monotonic()
        value = _collision_evidence(
            msg.state,
            CollisionStatus.STATE_CLEAR,
            CollisionStatus.STATE_CAUTION,
        )
        dispatch("COLLISION", value, msg.evaluation_stamp if msg.evaluation_stamp.to_sec() else msg.header.stamp)

    def slope_cb(msg: Any) -> None:
        policy_inputs["slope"] = msg
        policy_inputs["arrival"]["slope"] = time.monotonic()
        value = {SlopeStatus.STATE_CLEAR: "safe",
                 SlopeStatus.STATE_SLOW: "slow",
                 SlopeStatus.STATE_STOP: "unsafe"}.get(msg.state, "unsafe")
        dispatch("SLOPE", value, msg.evaluation_stamp if msg.evaluation_stamp.to_sec() else msg.header.stamp)

    def progress_cb(msg: Any) -> None:
        nonlocal latest_progress
        latest_progress = msg
        policy_inputs["progress"] = msg
        policy_inputs["arrival"]["progress"] = time.monotonic()
        identity_valid = (
            runtime.binding is not None
            and msg.mission_id == runtime.binding.mission_id
            and msg.route_id == runtime.binding.route_id
            and msg.map_id == runtime.binding.map_id
        )
        state_name = (_enum_name(runtime.output.state)
                      if runtime.output is not None else "DISARMED")
        if state_name == "LOCALIZING":
            if msg.state == RouteProgress.INACTIVE:
                return
            seed_valid = identity_valid and msg.state in (
                RouteProgress.ACTIVE, RouteProgress.AT_STOP)
            seed = -1
            if seed_valid:
                seed = next_waypoint_index(
                    int(msg.waypoint_index),
                    len(runtime.binding.route.waypoints),
                )
            dispatch("PROGRESS", seed, msg.header.stamp)
            return
        if state_name != "NAVIGATING":
            return
        current = int(runtime.output.progress)
        active_valid = identity_valid and msg.state != RouteProgress.INVALID
        value = max(current, int(msg.waypoint_index)) if active_valid else -1
        dispatch("PROGRESS", value, msg.header.stamp)
    def odometry_cb(msg: Any) -> None:
        policy_inputs["odometry_speed_mps"] = abs(float(msg.twist.twist.linear.x))
        policy_inputs["arrival"]["odometry"] = time.monotonic()


    rospy.Subscriber("/safety/state", SafetyState, safety_cb, queue_size=1)
    rospy.Subscriber("/route_safety/geofence_status", GeofenceStatus, geofence_cb, queue_size=1)
    rospy.Subscriber("/localization/status", LocalizationStatus, localization_cb, queue_size=1)
    rospy.Subscriber("/safety/collision_status", CollisionStatus, collision_cb, queue_size=1)
    rospy.Subscriber("/safety/slope_status", SlopeStatus, slope_cb, queue_size=1)
    rospy.Subscriber("/route/progress", RouteProgress, progress_cb, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odometry_cb, queue_size=1)

    def arm_service(_request: Any) -> Any:
        try:
            runtime.operator_arm()
            return TriggerResponse(success=True, message="armed for one validated mission")
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def reset_service(_request: Any) -> Any:
        try:
            runtime.operator_reset()
            return TriggerResponse(success=True, message="fault reset; explicit arm required")
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def resume_service(_request: Any) -> Any:
        try:
            output = runtime.dispatch("RESUME")
            apply(output)
            return TriggerResponse(success=_enum_name(output.state) != "FAULT",
                                   message=str(output.reason))
        except Exception as exc:
            runtime.fail_closed("resume service: %s" % exc)
            return TriggerResponse(success=False, message=str(exc))

    rospy.Service("~arm", Trigger, arm_service)
    rospy.Service("~reset", Trigger, reset_service)
    rospy.Service("~resume", Trigger, resume_service)

    def execute(goal: Any) -> None:
        nonlocal last_sent
        active_heartbeat = None

        def finish_active_heartbeat() -> None:
            nonlocal active_heartbeat
            if active_heartbeat is not None:
                stop_active_heartbeat(active_heartbeat)
                active_heartbeat = None
        result = ExecuteRouteResult()
        try:
            binding = bind_route(manifest, goal)
            output = runtime.begin(binding)
            last_sent = None
            waypoint_queue.clear()
            publish_active(binding)
            active_heartbeat = start_active_heartbeat(binding)
            apply(output)
        except Exception as exc:
            finish_active_heartbeat()
            runtime.fail_closed("goal rejected: %s" % exc)
            result.success = False
            result.result_code = ExecuteRouteResult.REJECTED
            result.message = str(exc)
            result.reason_mask = int(SafetyReason.ROUTE_MANIFEST)
            server.set_aborted(result, result.message)
            return
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            if server.is_preempt_requested():
                runtime.dispatch("DISARM")
                move_base.cancel_goal()
                result.success = False
                result.result_code = ExecuteRouteResult.CANCELED
                result.message = "mission canceled"
                result.reason_mask = int(SafetyReason.MANUAL_OVERRIDE)
                finish_active_heartbeat()
                server.set_preempted(result, result.message)
                return
            try:
                drain_deferred_waypoint()
                output = runtime.dispatch("TICK")
                apply(output)
                feedback = ExecuteRouteFeedback()
                feedback.progress = latest_progress
                feedback.mission_state = latest_state
                server.publish_feedback(feedback)
            except Exception as exc:
                runtime.fail_closed("watchdog: %s" % exc)
            terminal = _enum_name(getattr(runtime.output, "terminal_status", ""))
            if terminal in ("SUCCEEDED", "COMPLETE"):
                result.success = True
                result.result_code = ExecuteRouteResult.SUCCEEDED
                result.message = "route complete"
                result.reason_mask = reason_mask(runtime.output)
                finish_active_heartbeat()
                server.set_succeeded(result, result.message)
                return
            if terminal in ("ABORTED", "FAULT", "LOST") or runtime.fault_latched:
                move_base.cancel_goal()
                result.success = False
                result.result_code = ExecuteRouteResult.FAULT if terminal == "FAULT" else ExecuteRouteResult.ABORTED
                result.message = terminal or "mission fault"
                result.reason_mask = reason_mask(runtime.output)
                finish_active_heartbeat()
                server.set_aborted(result, result.message)
                return
            rate.sleep()
        move_base.cancel_goal()
        result.success = False
        result.result_code = ExecuteRouteResult.ABORTED
        result.message = "node shutdown"
        result.reason_mask = int(SafetyReason.INTERNAL_FAULT)
        finish_active_heartbeat()
        server.set_aborted(result, result.message)

    server = actionlib.SimpleActionServer("~execute_route", ExecuteRouteAction,
                                          execute_cb=execute, auto_start=False)
    server.start()

    def watchdog(_event: Any) -> None:
        try:
            apply(runtime.dispatch("TICK"))
        except Exception as exc:
            runtime.fail_closed("watchdog: %s" % exc)


    rospy.Timer(rospy.Duration(intent_period), watchdog)
    rospy.on_shutdown(lambda: (move_base.cancel_goal(), runtime.fail_closed("node shutdown")))
    rospy.spin()


if __name__ == "__main__":
    main()
