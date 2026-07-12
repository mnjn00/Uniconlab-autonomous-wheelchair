#!/usr/bin/env python3
"""Read-only control diagnostics with a ROS-independent observation core.

This module deliberately has no command publisher and no safety-state publisher.  ROS
is imported only by :class:`ControlMonitorNode`, keeping the core replayable in tests.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
import json
import math
from typing import Deque, Dict, Optional, Tuple

HOLD, PROCEED, SLOW, STOP = range(4)
DISARMED, CLEAR, STOPPED, LATCHED, FAULT = range(5)


@dataclass(frozen=True)
class MonitorConfig:
    command_ttl_s: float = 0.30
    intent_ttl_s: float = 0.30
    odom_ttl_s: float = 0.25
    route_ttl_s: float = 0.30
    safety_ttl_s: float = 0.15
    deadline_limit_s: float = 0.05
    stop_persistence_s: float = 0.05
    zero_linear_mps: float = 0.01
    zero_angular_rps: float = 0.02
    comparison_tolerance: float = 1e-6
    statistics_window: int = 200
    event_history: int = 100

    def __post_init__(self) -> None:
        positive = (
            self.command_ttl_s, self.intent_ttl_s, self.odom_ttl_s,
            self.route_ttl_s, self.safety_ttl_s, self.deadline_limit_s,
            self.stop_persistence_s, self.zero_linear_mps,
            self.zero_angular_rps, self.comparison_tolerance,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("monitor limits must be finite and positive")
        if self.statistics_window < 2 or self.event_history < 1:
            raise ValueError("monitor histories are too small")


@dataclass(frozen=True)
class TimedVelocity:
    linear_mps: float
    angular_rps: float
    source_stamp_s: float
    receipt_stamp_s: float


@dataclass(frozen=True)
class RouteObservation:
    along_track_m: float
    cross_track_m: float
    distance_remaining_m: float
    source_stamp_s: float
    receipt_stamp_s: float


@dataclass(frozen=True)
class IntentObservation:
    behavior: int
    max_linear_mps: float
    max_angular_rps: float
    source_stamp_s: float
    receipt_stamp_s: float


@dataclass(frozen=True)
class SafetyObservation:
    state: int
    deadline_miss_count: int
    source_stamp_s: float
    receipt_stamp_s: float
    requested_linear_mps: float = 0.0
    requested_angular_rps: float = 0.0
    output_linear_mps: float = 0.0
    output_angular_rps: float = 0.0


@dataclass(frozen=True)
class MonitorInputs:
    now_s: float
    route: Optional[RouteObservation] = None
    odom: Optional[TimedVelocity] = None
    nav_command: Optional[TimedVelocity] = None
    safe_command: Optional[TimedVelocity] = None
    intent: Optional[IntentObservation] = None
    safety: Optional[SafetyObservation] = None


@dataclass(frozen=True)
class MonitorResult:
    sequence: int
    faults: Tuple[str, ...]
    events: Tuple[str, ...]
    ages_s: Dict[str, float]
    linear_tracking_error_mps: float
    angular_tracking_error_rps: float
    linear_acceleration_mps2: float
    angular_acceleration_rps2: float
    linear_jerk_mps3: float
    angular_jerk_rps3: float
    cross_track_mean_m: float
    cross_track_rms_m: float
    cross_track_max_abs_m: float
    stop_count: int
    intervention_count: int
    saturation_count: int
    cap_event_count: int
    deadline_miss_count: int
    fault_counts: Dict[str, int] = field(default_factory=dict)


class ControlMonitorCore:
    """Bounded, deterministic observer.  It has no actuator or permission API."""

    STREAM_TTLS = {
        "route": "route_ttl_s", "odom": "odom_ttl_s",
        "nav_command": "command_ttl_s", "safe_command": "command_ttl_s",
        "intent": "intent_ttl_s", "safety": "safety_ttl_s",
    }

    def __init__(self, config: Optional[MonitorConfig] = None):
        self.config = config or MonitorConfig()
        size = self.config.statistics_window
        self._cross_track: Deque[float] = deque(maxlen=size)
        self._events: Deque[str] = deque(maxlen=self.config.event_history)
        self._fault_counts: Counter = Counter()
        self._sequence = 0
        self._last_now_s: Optional[float] = None
        self._last_odom: Optional[TimedVelocity] = None
        self._last_acceleration: Optional[Tuple[float, float, float]] = None
        self._last_source_stamps: Dict[str, float] = {}
        self._last_receipt_stamps: Dict[str, float] = {}
        self._last_intent_stop = False
        self._last_intervention = False
        self._stop_since_s: Optional[float] = None
        self._last_safety_deadlines = 0
        self.stop_count = 0
        self.intervention_count = 0
        self.saturation_count = 0
        self.cap_event_count = 0
        self.deadline_miss_count = 0

    @property
    def retained_sample_count(self) -> int:
        return len(self._cross_track) + len(self._events)

    def observe(self, inputs: MonitorInputs) -> MonitorResult:
        self._sequence += 1
        faults = set()
        events = []
        ages: Dict[str, float] = {}
        now = inputs.now_s
        if not _finite(now):
            faults.add("nonfinite")
        elif self._last_now_s is not None and now < self._last_now_s:
            faults.add("time_regression")
        else:
            if self._last_now_s is not None and now - self._last_now_s > self.config.deadline_limit_s + 1e-12:
                faults.add("deadline_miss")
                self.deadline_miss_count += 1
                events.append("deadline_miss")
            self._last_now_s = now

        for name, ttl_name in self.STREAM_TTLS.items():
            value = getattr(inputs, name)
            if value is None:
                ages[name + "_source"] = -1.0
                ages[name + "_receipt"] = -1.0
                faults.add("stale_" + name)
                continue
            numeric = [getattr(value, field_name) for field_name in value.__dataclass_fields__
                       if field_name != "behavior" and field_name != "state" and field_name != "deadline_miss_count"]
            if not all(_finite(item) for item in numeric):
                faults.add("nonfinite")
                continue
            source_age = now - value.source_stamp_s
            receipt_age = now - value.receipt_stamp_s
            ages[name + "_source"] = source_age
            ages[name + "_receipt"] = receipt_age
            previous_source = self._last_source_stamps.get(name)
            previous_receipt = self._last_receipt_stamps.get(name)
            if ((previous_source is not None and value.source_stamp_s < previous_source) or
                    (previous_receipt is not None and value.receipt_stamp_s < previous_receipt)):
                faults.add("time_regression")
            self._last_source_stamps[name] = value.source_stamp_s
            self._last_receipt_stamps[name] = value.receipt_stamp_s
            if source_age < 0.0 or receipt_age < 0.0:
                faults.add("time_regression")
            if source_age > getattr(self.config, ttl_name) + 1e-12 or receipt_age > getattr(self.config, ttl_name) + 1e-12:
                faults.add("stale_" + name)

        linear_error = angular_error = 0.0
        if inputs.safe_command is not None and inputs.odom is not None and _velocities_finite(inputs.safe_command, inputs.odom):
            linear_error = inputs.odom.linear_mps - inputs.safe_command.linear_mps
            angular_error = inputs.odom.angular_rps - inputs.safe_command.angular_rps

        linear_accel = angular_accel = linear_jerk = angular_jerk = 0.0
        if inputs.odom is not None and _velocities_finite(inputs.odom):
            if self._last_odom is not None:
                dt = inputs.odom.source_stamp_s - self._last_odom.source_stamp_s
                if dt < 0.0:
                    faults.add("time_regression")
                elif dt > 0.0:
                    linear_accel = (inputs.odom.linear_mps - self._last_odom.linear_mps) / dt
                    angular_accel = (inputs.odom.angular_rps - self._last_odom.angular_rps) / dt
                    if self._last_acceleration is not None:
                        previous_linear, previous_angular, previous_stamp = self._last_acceleration
                        accel_dt = inputs.odom.source_stamp_s - previous_stamp
                        if accel_dt > 0.0:
                            linear_jerk = (linear_accel - previous_linear) / accel_dt
                            angular_jerk = (angular_accel - previous_angular) / accel_dt
                    self._last_acceleration = (linear_accel, angular_accel, inputs.odom.source_stamp_s)
            self._last_odom = inputs.odom

        if inputs.route is not None and _finite(inputs.route.cross_track_m):
            self._cross_track.append(inputs.route.cross_track_m)

        intent_stop = inputs.intent is not None and inputs.intent.behavior in (HOLD, STOP)
        if intent_stop and not self._last_intent_stop:
            self.stop_count += 1
            events.append("stop")
        self._last_intent_stop = intent_stop

        safe_nonzero = inputs.safe_command is not None and not self._is_zero(inputs.safe_command)
        safety_stop = inputs.safety is not None and inputs.safety.state != CLEAR
        stop_required = intent_stop or safety_stop
        if stop_required and safe_nonzero:
            faults.add("command_after_stop")
            if self._stop_since_s is None:
                self._stop_since_s = now
            if _finite(now) and now - self._stop_since_s >= self.config.stop_persistence_s - 1e-12:
                faults.add("unsafe_command_persistence")
        else:
            self._stop_since_s = None

        intervention = False
        if inputs.nav_command is not None and inputs.safe_command is not None and _velocities_finite(inputs.nav_command, inputs.safe_command):
            tolerance = self.config.comparison_tolerance
            nav, safe = inputs.nav_command, inputs.safe_command
            intervention = (abs(nav.linear_mps - safe.linear_mps) > tolerance or
                            abs(nav.angular_rps - safe.angular_rps) > tolerance)
            if intervention and not self._last_intervention:
                self.intervention_count += 1
                events.append("intervention")
            saturated = (abs(safe.linear_mps) + tolerance < abs(nav.linear_mps) or
                         abs(safe.angular_rps) + tolerance < abs(nav.angular_rps))
            if saturated:
                self.saturation_count += 1
                events.append("saturation")
            if _wrong_sign(nav.linear_mps, safe.linear_mps, tolerance) or _wrong_sign(nav.angular_rps, safe.angular_rps, tolerance):
                faults.add("wrong_sign")
        self._last_intervention = intervention

        if inputs.intent is not None and inputs.safe_command is not None and _velocities_finite(inputs.safe_command):
            tolerance = self.config.comparison_tolerance
            if inputs.intent.behavior not in (HOLD, PROCEED, SLOW, STOP):
                faults.add("invalid_intent")
            if inputs.intent.max_linear_mps < 0.0 or inputs.intent.max_angular_rps < 0.0:
                faults.add("invalid_intent")
            if inputs.safe_command.linear_mps < -tolerance:
                faults.add("reverse_autonomy")
            if (abs(inputs.safe_command.linear_mps) > inputs.intent.max_linear_mps + tolerance or
                    abs(inputs.safe_command.angular_rps) > inputs.intent.max_angular_rps + tolerance):
                faults.add("safe_exceeds_intent")
                self.cap_event_count += 1
                events.append("cap_violation")

        if inputs.safety is not None:
            safety = inputs.safety
            if (safety.state not in (DISARMED, CLEAR, STOPPED, LATCHED, FAULT) or
                    isinstance(safety.deadline_miss_count, bool) or
                    not isinstance(safety.deadline_miss_count, int) or safety.deadline_miss_count < 0):
                faults.add("invalid_safety_state")
            tolerance = self.config.comparison_tolerance
            if inputs.nav_command is not None and _velocities_finite(inputs.nav_command):
                if (abs(safety.requested_linear_mps - inputs.nav_command.linear_mps) > tolerance or
                        abs(safety.requested_angular_rps - inputs.nav_command.angular_rps) > tolerance):
                    faults.add("safety_requested_mismatch")
            if inputs.safe_command is not None and _velocities_finite(inputs.safe_command):
                if (abs(safety.output_linear_mps - inputs.safe_command.linear_mps) > tolerance or
                        abs(safety.output_angular_rps - inputs.safe_command.angular_rps) > tolerance):
                    faults.add("safety_output_mismatch")

        if inputs.safety is not None:
            count = inputs.safety.deadline_miss_count
            if count < self._last_safety_deadlines:
                faults.add("time_regression")
            elif count > self._last_safety_deadlines:
                self.deadline_miss_count += count - self._last_safety_deadlines
                events.append("safety_deadline_miss")
            self._last_safety_deadlines = max(self._last_safety_deadlines, count)

        for fault in sorted(faults):
            self._fault_counts[fault] += 1
            events.append("fault:" + fault)
        self._events.extend(events)
        mean, rms, maximum = self._cross_track_statistics()
        return MonitorResult(
            self._sequence, tuple(sorted(faults)), tuple(events), ages,
            linear_error, angular_error, linear_accel, angular_accel,
            linear_jerk, angular_jerk, mean, rms, maximum,
            self.stop_count, self.intervention_count, self.saturation_count,
            self.cap_event_count, self.deadline_miss_count, dict(self._fault_counts),
        )

    def _is_zero(self, velocity: TimedVelocity) -> bool:
        return (abs(velocity.linear_mps) <= self.config.zero_linear_mps and
                abs(velocity.angular_rps) <= self.config.zero_angular_rps)

    def _cross_track_statistics(self) -> Tuple[float, float, float]:
        if not self._cross_track:
            return 0.0, 0.0, 0.0
        count = len(self._cross_track)
        mean = sum(self._cross_track) / count
        rms = math.sqrt(sum(value * value for value in self._cross_track) / count)
        maximum = max(abs(value) for value in self._cross_track)
        return mean, rms, maximum


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _velocities_finite(*values: TimedVelocity) -> bool:
    return all(all(_finite(item) for item in (
        value.linear_mps, value.angular_rps, value.source_stamp_s, value.receipt_stamp_s))
               for value in values)


def _wrong_sign(requested: float, output: float, tolerance: float) -> bool:
    return abs(requested) > tolerance and abs(output) > tolerance and requested * output < 0.0


class ControlMonitorNode:
    """Lazy ROS adapter; callbacks cache data and the timer performs observation."""

    def __init__(self) -> None:
        import rospy
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from wheelchair_interfaces.msg import MotionIntent, RouteProgress, SafetyState

        self._rospy = rospy
        self._DiagnosticArray = DiagnosticArray
        self._String = String
        get = rospy.get_param
        self._core = ControlMonitorCore(MonitorConfig(
            command_ttl_s=get("~command_ttl_s", 0.30), intent_ttl_s=get("~intent_ttl_s", 0.30),
            odom_ttl_s=get("~odom_ttl_s", 0.25), route_ttl_s=get("~route_ttl_s", 0.30),
            safety_ttl_s=get("~safety_ttl_s", 0.15), deadline_limit_s=get("~deadline_limit_s", 0.05),
            stop_persistence_s=get("~stop_persistence_s", 0.05),
            statistics_window=get("~statistics_window", 200), event_history=get("~event_history", 100),
        ))
        self._latest = {name: None for name in self._core.STREAM_TTLS}
        self._diagnostics = rospy.Publisher(get("~diagnostics_topic", "/diagnostics"), DiagnosticArray, queue_size=1)
        self._event_publisher = rospy.Publisher(get("~events_topic", "/navigation/control_events"), String, queue_size=1)
        rospy.Subscriber(get("~route_topic", "/route/progress"), RouteProgress, self._route, queue_size=1)
        rospy.Subscriber(get("~odom_topic", "/odom"), Odometry, self._odom, queue_size=1)
        rospy.Subscriber(get("~nav_command_topic", "/cmd_vel_nav"), Twist, lambda msg: self._twist("nav_command", msg), queue_size=1)
        rospy.Subscriber(get("~safe_command_topic", "/cmd_vel_safe"), Twist, lambda msg: self._twist("safe_command", msg), queue_size=1)
        rospy.Subscriber(get("~intent_topic", "/decision/motion_intent"), MotionIntent, self._intent, queue_size=1)
        rospy.Subscriber(get("~safety_topic", "/safety/state"), SafetyState, self._safety, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(1.0 / get("~publish_rate_hz", 20.0)), self._publish)

    def _now(self) -> float:
        return self._rospy.get_time()

    @staticmethod
    def _stamp(message, fallback: float) -> float:
        value = message.header.stamp.to_sec()
        return value if value > 0.0 else fallback

    def _route(self, message) -> None:
        receipt = self._now()
        self._latest["route"] = RouteObservation(message.along_track_m, message.cross_track_error_m,
                                                   message.distance_remaining_m, self._stamp(message, receipt), receipt)

    def _odom(self, message) -> None:
        receipt = self._now()
        twist = message.twist.twist
        self._latest["odom"] = TimedVelocity(twist.linear.x, twist.angular.z, self._stamp(message, receipt), receipt)

    def _twist(self, name: str, message) -> None:
        receipt = self._now()
        self._latest[name] = TimedVelocity(message.linear.x, message.angular.z, receipt, receipt)

    def _intent(self, message) -> None:
        receipt = self._now()
        self._latest["intent"] = IntentObservation(message.behavior, message.max_linear_mps,
                                                     message.max_angular_rps, self._stamp(message, receipt), receipt)

    def _safety(self, message) -> None:
        receipt = self._now()
        requested, output = message.requested_command, message.output_command
        self._latest["safety"] = SafetyObservation(
            message.state, message.deadline_miss_count, self._stamp(message, receipt), receipt,
            requested.linear.x, requested.angular.z, output.linear.x, output.angular.z)

    def _publish(self, _event) -> None:
        from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

        result = self._core.observe(MonitorInputs(now_s=self._now(), **self._latest))
        status = DiagnosticStatus()
        status.name = "wheelchair_navigation/control_monitor"
        status.hardware_id = "observer_only"
        status.level = DiagnosticStatus.ERROR if result.faults else DiagnosticStatus.OK
        status.message = ",".join(result.faults) if result.faults else "nominal"
        values = {
            "sequence": result.sequence, "linear_tracking_error_mps": result.linear_tracking_error_mps,
            "angular_tracking_error_rps": result.angular_tracking_error_rps,
            "linear_acceleration_mps2": result.linear_acceleration_mps2,
            "linear_jerk_mps3": result.linear_jerk_mps3, "cross_track_mean_m": result.cross_track_mean_m,
            "cross_track_rms_m": result.cross_track_rms_m, "cross_track_max_abs_m": result.cross_track_max_abs_m,
            "stop_count": result.stop_count, "intervention_count": result.intervention_count,
            "saturation_count": result.saturation_count, "cap_event_count": result.cap_event_count,
            "deadline_miss_count": result.deadline_miss_count,
        }
        values.update(result.ages_s)
        status.values = [KeyValue(str(key), str(value)) for key, value in sorted(values.items())]
        array = self._DiagnosticArray()
        array.header.stamp = self._rospy.Time.now()
        array.status = [status]
        self._diagnostics.publish(array)
        for event in result.events:
            self._event_publisher.publish(self._String(data=json.dumps(
                {"sequence": result.sequence, "stamp_s": self._now(), "event": event},
                sort_keys=True, separators=(",", ":"))))


def main() -> None:
    import rospy

    rospy.init_node("control_monitor")
    ControlMonitorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
