#!/usr/bin/env python3
"""Deterministic, read-only ROS/Gazebo release-candidate metrics collector.

The MetricsCore has no ROS dependency.  RosCollector imports ROS only when the
command-line entry point is run and only subscribes; it cannot authorize or
publish motion.
"""
import argparse
import json
import math
import os
import tempfile
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

SCHEMA = "wheelchair_gazebo.rc_metrics"
SCHEMA_VERSION = 1
REQUIRED_STREAMS = (
    "clock", "ground_truth", "contacts", "route",
    "localization", "collision", "geofence", "slope", "safety",
    "nav_command", "safe_command",
)
MINIMUM_TERMINAL_SETTLE_S = 0.60
TWIST_AXES = ("linear.x", "linear.y", "linear.z",
              "angular.x", "angular.y", "angular.z")
UNSUPPORTED_TWIST_AXES = ("linear.y", "linear.z", "angular.x", "angular.y")
FAULT_EVENT_SCHEMA = "wheelchair.sim_fault/v1"
FAULT_EVENT_FIELDS = {"schema", "fault_id", "phase", "stamp_s", "detail"}
FAULT_PHASE_TRANSITIONS = {
    None: {"ready"},
    "ready": {"triggered", "failed"},
    "triggered": {"reset_attempted", "completed", "failed"},
    "reset_attempted": {"completed", "failed"},
    "completed": set(),
    "failed": set(),
}
FAULT_STOP_BUDGET_S = 0.15
SAFETY_REASON_BITS = {
    1: "ESTOP",
    2: "STALE_CMD",
    4: "MODE",
    8: "GEOFENCE",
    16: "COLLISION",
    32: "LOCALIZATION",
    64: "DRIVER",
    128: "INVALID_CMD",
    256: "CLOCK",
    512: "STALE_INTENT",
    1024: "INTERNAL_FAULT",
    2048: "STARTUP",
    4096: "SENSOR_STALE",
    8192: "COLLISION_BLIND",
    16384: "COLLISION_TTC",
    32768: "COLLISION_DISTANCE",
    65536: "SLOPE",
    131072: "IMU_UNCALIBRATED",
    262144: "ROUTE_MANIFEST",
    524288: "GRAPH_TOPOLOGY",
    1048576: "TF",
    2097152: "BACKPRESSURE",
    4194304: "DEADLINE_MISS",
    8388608: "MANUAL_OVERRIDE",
    16777216: "HARDWARE_UNVERIFIED",
    33554432: "MAP_MISMATCH",
    67108864: "COLLISION_OCCLUDED",
    134217728: "LOCALIZATION_INCONSISTENT",
    268435456: "RESOURCE",
    536870912: "CORRUPT_DATA",
    1073741824: "RESET_REJECTED",
    2147483648: "INPUT_UNKNOWN",
    4294967296: "ROUTE_STATE",
    8589934592: "ODOM_STALE",
    17179869184: "IMU_STALE",
    34359738368: "LIDAR_STALE",
    68719476736: "POLICY_MISMATCH",
}


def finite(*values: float) -> bool:
    return all(isinstance(value, (int, float)) and not isinstance(value, bool)
               and math.isfinite(float(value)) for value in values)


def percentile(values: List[float], percent: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("percentile requires samples")
    rank = (len(ordered) - 1) * percent / 100.0
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (rank - low)


@dataclass(frozen=True)
class Limits:
    linear_mps: float = 0.55
    angular_rps: float = 0.85
    linear_accel_mps2: float = 0.80
    angular_accel_rps2: float = 1.50
    linear_jerk_mps3: float = 4.0
    angular_jerk_rps3: float = 8.0
    stop_latency_s: float = 0.50
    stop_overshoot_m: float = 0.50
    stale_s: float = 1.0
    zero_epsilon: float = 1e-6
    fault_stop_budget_s: float = FAULT_STOP_BUDGET_S


class MetricsCore:
    """Pure event accumulator.  Every malformed observation is sticky-failing."""

    def __init__(self, limits: Limits = Limits(), fault_id: str = "none") -> None:
        self.limits = limits
        if (
            not finite(self.limits.fault_stop_budget_s)
            or self.limits.fault_stop_budget_s <= 0.0
            or self.limits.fault_stop_budget_s > FAULT_STOP_BUDGET_S
        ):
            raise ValueError("fault stop budget must be in (0, 0.15]")
        self.fault_id = fault_id
        self.fault_run = fault_id != "none"
        self.seen: Set[str] = set()
        self.counts: Dict[str, int] = {name: 0 for name in REQUIRED_STREAMS}
        self.failures: List[str] = []
        self.clock: List[float] = []
        self.last_stamp: Dict[str, float] = {}
        self.poses: List[Tuple[float, float, float, float]] = []
        self.cross_track: List[float] = []
        self.route_terminal: Optional[str] = None
        self.goal_error_m: Optional[float] = None
        self.goal_error_yaw_deg: Optional[float] = None
        self.status_counts: Dict[str, Dict[str, int]] = {
            name: {} for name in ("localization", "collision", "geofence", "slope", "safety")
        }
        self.nav_commands: List[Tuple[float, float, float]] = []
        self.safe_commands: List[Tuple[float, float, float]] = []
        self.maxima = {"linear_speed_mps": 0.0, "angular_speed_rps": 0.0,
                       "linear_acceleration_mps2": 0.0, "angular_acceleration_rps2": 0.0,
                       "linear_jerk_mps3": 0.0, "angular_jerk_rps3": 0.0}
        self.nonfinite_samples = 0
        self.command_nonfinite_components = 0
        self.command_shape_violations = 0
        self.command_cap_exceedances = 0
        self.command_violation_reasons: Dict[str, int] = {}
        self.clock_regressions = 0
        self.stop_trigger_s: Optional[float] = None
        self.stop_trigger_pose: Optional[Tuple[float, float]] = None
        self.stop_observed_s: Optional[float] = None
        self.stop_pose: Optional[Tuple[float, float]] = None
        self.nonzero_after_fault = 0
        self.reason_events: List[Dict[str, object]] = []
        self.clear_after_stop = False
        self.motion_started = False
        self.footprint_collisions = 0
        self.collision_ttc: List[float] = []
        self.no_finite_ttc_observed = False
        self._last_accel: Optional[Tuple[float, float, float]] = None
        self._command_speed_maxima = {"linear.x": 0.0, "angular.z": 0.0}
        self.fault_phase: Optional[str] = None
        self.fault_trigger_s: Optional[float] = None
        self.fault_zero_s: Optional[float] = None
        self.fault_sink_nonzero = 0
        self.fault_status_history: List[
            Tuple[str, float, int, int, str]
        ] = []
        self.safety_stop_history: List[Tuple[float, bool]] = []
        self.reset_safety_index: Optional[int] = None
        self.reset_stamp_s: Optional[float] = None
        self.reset_had_stop = False

    def _reject(self, stream: str, stamp: float, *values: float) -> bool:
        if not finite(stamp, *values) or stamp < 0.0:
            self.nonfinite_samples += 1
            self.failures.append("nonfinite or invalid {} sample".format(stream))
            return True
        self.seen.add(stream)
        self.counts[stream] = self.counts.get(stream, 0) + 1
        self.last_stamp[stream] = float(stamp)
        return False

    def observe_clock(self, stamp: float) -> None:
        if not finite(stamp) or stamp < 0.0:
            self.nonfinite_samples += 1
            self.failures.append("nonfinite or invalid clock sample")
            return
        if self.clock and stamp < self.clock[-1]:
            self.clock_regressions += 1
            self.failures.append("clock regression")
        self.clock.append(float(stamp))
        self.seen.add("clock")
        self.counts["clock"] += 1
        self.last_stamp["clock"] = float(stamp)

    def observe_pose(self, stamp: float, x_m: float, y_m: float, yaw_rad: float) -> None:
        if not self._reject("ground_truth", stamp, x_m, y_m, yaw_rad):
            self.poses.append((float(stamp), float(x_m), float(y_m), float(yaw_rad)))

    def observe_route(self, stamp: float, state: int, cross_track_m: float,
                      distance_remaining_m: float, complete_state: int = 3,
                      invalid_state: int = 4) -> None:
        if self._reject("route", stamp, cross_track_m, distance_remaining_m):
            return
        self.cross_track.append(float(cross_track_m))
        self.goal_error_m = abs(float(distance_remaining_m))
        if state == complete_state:
            self.route_terminal = "completed"
            self.trigger_stop(stamp, "route_complete")
            # RouteProgress does not carry a goal heading.  Estimate the terminal
            # route tangent from distinct ground-truth poses and compare it with
            # the canonical model yaw; absence of motion remains a hard failure.
            if len(self.poses) >= 2:
                terminal = self.poses[-1]
                for previous in reversed(self.poses[:-1]):
                    dx = terminal[1] - previous[1]
                    dy = terminal[2] - previous[2]
                    if math.hypot(dx, dy) > 1e-3:
                        error = terminal[3] - math.atan2(dy, dx)
                        error = math.atan2(math.sin(error), math.cos(error))
                        self.goal_error_yaw_deg = abs(math.degrees(error))
                        break
        elif state == invalid_state:
            self.route_terminal = "safe_abort"
            self.trigger_stop(stamp, "route_invalid")
            if len(self.poses) >= 2:
                terminal = self.poses[-1]
                for previous in reversed(self.poses[:-1]):
                    dx = terminal[1] - previous[1]
                    dy = terminal[2] - previous[2]
                    if math.hypot(dx, dy) > 1e-3:
                        error = terminal[3] - math.atan2(dy, dx)
                        error = math.atan2(math.sin(error), math.cos(error))
                        self.goal_error_yaw_deg = abs(math.degrees(error))
                        break

    def observe_fault_event(self, payload: str) -> None:
        try:
            event = json.loads(payload)
            canonical = json.dumps(event, sort_keys=True, separators=(",", ":"), allow_nan=False)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.failures.append("malformed fault event")
            return
        if not isinstance(event, dict) or set(event) != FAULT_EVENT_FIELDS or canonical != payload:
            self.failures.append("malformed fault event")
            return
        if (event["schema"] != FAULT_EVENT_SCHEMA or
                not isinstance(event["fault_id"], str) or
                not isinstance(event["phase"], str) or
                not isinstance(event["detail"], str) or
                not finite(event["stamp_s"]) or event["stamp_s"] < 0.0):
            self.failures.append("malformed fault event")
            return
        if not self.fault_run or event["fault_id"] != self.fault_id:
            self.failures.append("mismatched fault event")
            return
        phase = event["phase"]
        if phase not in FAULT_PHASE_TRANSITIONS.get(self.fault_phase, set()):
            self.failures.append("regressing fault event lifecycle")
            return
        stamp = float(event["stamp_s"])
        if "fault_event" in self.last_stamp and stamp < self.last_stamp["fault_event"]:
            self.failures.append("regressing fault event timestamp")
            return
        self.seen.add("fault_event")
        self.counts["fault_event"] = self.counts.get("fault_event", 0) + 1
        self.last_stamp["fault_event"] = stamp
        self.fault_phase = phase
        if phase == "triggered":
            self.fault_trigger_s = stamp
        elif phase == "reset_attempted":
            self.reset_safety_index = len(self.safety_stop_history)
            self.reset_stamp_s = stamp
            self.reset_had_stop = bool(
                self.safety_stop_history and self.safety_stop_history[-1][1] and
                self.safety_stop_history[-1][0] <= stamp)
        elif phase == "failed":
            self.failures.append("fault injector reported failure")

    def observe_status(self, stream: str, stamp: float, state: int, reason_mask: int = 0,
                       stop_states: Tuple[int, ...] = (), latched: bool = False,
                       source: str = "") -> None:
        if stream not in self.status_counts:
            raise ValueError("unknown status stream: " + stream)
        if (isinstance(state, bool) or not isinstance(state, int) or
                isinstance(reason_mask, bool) or not isinstance(reason_mask, int) or
                reason_mask < 0):
            self.failures.append("invalid {} status sample".format(stream))
            return
        source = str(source or stream)
        if not source or any(ord(character) < 32 for character in source):
            self.failures.append("invalid {} status source".format(stream))
            return
        if self._reject(stream, stamp, state, reason_mask):
            return
        key = str(state)
        self.status_counts[stream][key] = self.status_counts[stream].get(key, 0) + 1
        stopped = state in stop_states
        if self.fault_run:
            self.fault_status_history.append(
                (stream, float(stamp), state, reason_mask, source)
            )
            if stream == "safety":
                self.safety_stop_history.append((float(stamp), stopped or bool(latched)))
        if stopped:
            if self.motion_started:
                self.trigger_stop(stamp, stream, reason_mask)
        elif self.stop_trigger_s is not None:
            self.clear_after_stop = True

    def trigger_stop(self, stamp: float, source: str, reason_mask: int = 0) -> None:
        if self.stop_trigger_s is None:
            self.stop_trigger_s = float(stamp)
            if self.poses:
                self.stop_trigger_pose = self.poses[-1][1:3]
            self.reason_events.append({"stamp_s": float(stamp), "event": "stop",
                                       "source": source, "reason_mask": int(reason_mask)})
            if self.safe_commands:
                command_stamp, linear, angular = self.safe_commands[-1]
                recent = 0.0 <= stamp - command_stamp <= self.limits.stale_s
                zero = linear == 0.0 and angular == 0.0
                if recent and zero:
                    self.stop_observed_s = float(stamp)
                    if self.poses:
                        self.stop_pose = self.poses[-1][1:3]

    def observe_contacts(self, stamp: float, contact_count: int) -> None:
        if isinstance(contact_count, bool) or not isinstance(contact_count, int) or contact_count < 0:
            self.failures.append("invalid contact count")
            return
        if not self._reject("contacts", stamp, float(contact_count)):
            self.footprint_collisions += contact_count

    def observe_collision_ttc(self, value: float) -> None:
        if not finite(value):
            self.nonfinite_samples += 1
            self.failures.append("nonfinite collision TTC")
        elif value == -1.0:
            self.no_finite_ttc_observed = True
        elif value < 0.0:
            self.failures.append("invalid collision TTC sentinel")
        else:
            self.collision_ttc.append(float(value))
    def _record_command_violation(self, reason: str) -> None:
        self.command_violation_reasons[reason] = self.command_violation_reasons.get(reason, 0) + 1



    def observe_command(self, stream: str, stamp: float,
                        linear_x: float, linear_y: float, linear_z: float,
                        angular_x: float, angular_y: float, angular_z: float) -> None:
        if stream not in ("nav_command", "safe_command", "actuator_sink"):
            raise ValueError("unknown command stream: " + stream)
        components = {
            "linear.x": linear_x,
            "linear.y": linear_y,
            "linear.z": linear_z,
            "angular.x": angular_x,
            "angular.y": angular_y,
            "angular.z": angular_z,
        }
        previous_stamp = self.last_stamp.get(stream)
        for axis in TWIST_AXES:
            if not finite(components[axis]):
                self.command_nonfinite_components += 1
                self._record_command_violation("nonfinite.{}.{}".format(stream, axis))
        if self._reject(stream, stamp, *(components[axis] for axis in TWIST_AXES)):
            return
        if previous_stamp is not None and stamp < previous_stamp:
            self.failures.append("regressing {} timestamp".format(stream))

        for axis in UNSUPPORTED_TWIST_AXES:
            if components[axis] != 0.0:
                self.command_shape_violations += 1
                self._record_command_violation(
                    "unsupported_axis_nonzero.{}.{}".format(stream, axis))
                self.failures.append(
                    "unsupported Twist axis nonzero: {} {}".format(stream, axis))

        for axis, value, limit in (
                ("linear.x", linear_x, self.limits.linear_mps),
                ("angular.z", angular_z, self.limits.angular_rps)):
            self._command_speed_maxima[axis] = max(
                self._command_speed_maxima[axis], abs(value))
            if abs(value) > limit:
                self.command_cap_exceedances += 1
                self._record_command_violation(
                    "cap_exceeded.{}.{}".format(stream, axis))
        self.maxima["linear_speed_mps"] = self._command_speed_maxima["linear.x"]
        self.maxima["angular_speed_rps"] = self._command_speed_maxima["angular.z"]
        zero = all(value == 0.0 for value in components.values())
        if stream == "actuator_sink":
            if self.fault_trigger_s is not None and stamp >= self.fault_trigger_s:
                if zero and self.fault_zero_s is None:
                    self.fault_zero_s = float(stamp)
                elif not zero:
                    self.fault_sink_nonzero += 1
                    self._record_command_violation("post_fault_nonzero.actuator_sink")
            return

        sample = (float(stamp), float(linear_x), float(angular_z))
        if stream == "nav_command":
            self.nav_commands.append(sample)
            return
        previous = self.safe_commands[-1] if self.safe_commands else None
        self.safe_commands.append(sample)
        if previous is not None:
            dt = stamp - previous[0]
            if dt <= 0.0:
                self.failures.append("non-increasing safe command timestamp")
            else:
                la = (linear_x - previous[1]) / dt
                aa = (angular_z - previous[2]) / dt
                self.maxima["linear_acceleration_mps2"] = max(self.maxima["linear_acceleration_mps2"], abs(la))
                self.maxima["angular_acceleration_rps2"] = max(self.maxima["angular_acceleration_rps2"], abs(aa))
                if self._last_accel is not None:
                    adt = stamp - self._last_accel[0]
                    if adt > 0.0:
                        self.maxima["linear_jerk_mps3"] = max(self.maxima["linear_jerk_mps3"], abs(la - self._last_accel[1]) / adt)
                        self.maxima["angular_jerk_rps3"] = max(self.maxima["angular_jerk_rps3"], abs(aa - self._last_accel[2]) / adt)
                self._last_accel = (float(stamp), la, aa)
        if not zero:
            self.motion_started = True
        if self.stop_trigger_s is not None and stamp >= self.stop_trigger_s:
            if zero and self.stop_observed_s is None:
                self.stop_observed_s = float(stamp)
                if self.poses:
                    self.stop_pose = self.poses[-1][1:3]
            elif not zero:
                self.nonzero_after_fault += 1
                self._record_command_violation("post_stop_nonzero.safe_command")

    def set_goal_yaw_error(self, error_deg: float) -> None:
        if not finite(error_deg):
            self.nonfinite_samples += 1
            self.failures.append("nonfinite goal yaw error")
        else:
            self.goal_error_yaw_deg = abs(float(error_deg))

    def finalize(self, timed_out: bool = False) -> Dict[str, object]:
        failures = list(self.failures)
        required_streams = set(REQUIRED_STREAMS)
        if self.fault_run:
            required_streams.update(("fault_event", "actuator_sink"))
        missing = sorted(required_streams - self.seen)
        failures.extend("missing required topic evidence: " + name for name in missing)
        if timed_out:
            failures.append("collector timeout")
        if self.route_terminal is None:
            failures.append("absent terminal route evidence")
        if self.goal_error_yaw_deg is None:
            failures.append("absent terminal goal yaw evidence")
        end = self.clock[-1] if self.clock else None
        stale: List[str] = []
        if end is not None:
            for stream in REQUIRED_STREAMS:
                if stream in self.last_stamp and end - self.last_stamp[stream] > self.limits.stale_s:
                    stale.append(stream)
                    failures.append("stale terminal input: " + stream)
        stop_latency = None if self.stop_trigger_s is None or self.stop_observed_s is None else self.stop_observed_s - self.stop_trigger_s
        overshoot = None
        if self.stop_trigger_pose is not None and self.stop_pose is not None:
            overshoot = math.hypot(self.stop_pose[0] - self.stop_trigger_pose[0], self.stop_pose[1] - self.stop_trigger_pose[1])
        if self.stop_trigger_s is not None and self.stop_observed_s is None:
            failures.append("stop trigger has no zero-command response")
        if self.nonzero_after_fault:
            failures.append("nonzero safe command after stop trigger")
        cap_checks = {
            "linear_speed": self._command_speed_maxima["linear.x"] <= self.limits.linear_mps,
            "angular_speed": self._command_speed_maxima["angular.z"] <= self.limits.angular_rps,
            "linear_acceleration": self.maxima["linear_acceleration_mps2"] <= self.limits.linear_accel_mps2,
            "angular_acceleration": self.maxima["angular_acceleration_rps2"] <= self.limits.angular_accel_rps2,
            "linear_jerk": self.maxima["linear_jerk_mps3"] <= self.limits.linear_jerk_mps3,
            "angular_jerk": self.maxima["angular_jerk_rps3"] <= self.limits.angular_jerk_rps3,
        }
        command_shape_respected = self.command_shape_violations == 0
        command_caps_respected = all(cap_checks.values()) and command_shape_respected
        if not all(cap_checks.values()):
            failures.append("command dynamics limit exceeded")
        envelope = (self.stop_trigger_s is None or
                    (stop_latency is not None and stop_latency <= self.limits.stop_latency_s and
                     overshoot is not None and overshoot <= self.limits.stop_overshoot_m))
        if not envelope:
            failures.append("stopping envelope violated")
        fault_reason_names: Set[str] = set()
        fault_reason_observations: List[Dict[str, object]] = []
        fault_safe_stop = False
        unknown_reason_bits = 0
        if self.fault_run:
            if self.fault_trigger_s is None:
                failures.append("missing triggered fault event")
            if self.fault_phase != "completed":
                failures.append("incomplete fault event lifecycle")
            for stream, stamp, state, reason_mask, source in self.fault_status_history:
                if self.fault_trigger_s is None or stamp < self.fault_trigger_s:
                    continue
                names = sorted(
                    name for bit, name in SAFETY_REASON_BITS.items()
                    if reason_mask & bit
                )
                for name in names:
                    fault_reason_names.add(name)
                unknown_reason_bits |= reason_mask & ~sum(SAFETY_REASON_BITS)
                if reason_mask:
                    fault_reason_observations.append({
                        "source": source,
                        "stream": stream,
                        "stamp_s": stamp,
                        "reason_mask": reason_mask,
                        "names": names,
                    })
                if stream == "safety" and state in (2, 3, 4):
                    fault_safe_stop = True
            if unknown_reason_bits:
                failures.append("unknown SafetyReason mask bits")
            if not fault_reason_observations:
                failures.append("fault produced no symbolic safety reason evidence")
            if self.fault_zero_s is None:
                failures.append("fault trigger has no actuator sink zero response")
            elif self.fault_trigger_s is not None and (
                    self.fault_zero_s - self.fault_trigger_s >
                    self.limits.fault_stop_budget_s):
                failures.append("actuator sink zero response exceeded software budget")
            if self.fault_sink_nonzero:
                failures.append("nonzero actuator sink command after fault trigger")
        fault_event_invalid = any(
            "fault event" in failure or "fault injector" in failure
            for failure in failures)
        zero_within_budget = bool(
            self.fault_trigger_s is not None and self.fault_zero_s is not None and
            0.0 <= self.fault_zero_s - self.fault_trigger_s <=
            self.limits.fault_stop_budget_s and
            self.fault_sink_nonzero == 0 and not fault_event_invalid)
        fault_zero_latency = (
            None if self.fault_trigger_s is None or self.fault_zero_s is None
            else self.fault_zero_s - self.fault_trigger_s
        )
        latched_until_reset = bool(
            self.reset_had_stop and self.reset_safety_index is not None and
            self.reset_stamp_s is not None and
            any(stamp >= self.reset_stamp_s and stopped
                for stamp, stopped in self.safety_stop_history[self.reset_safety_index:]))
        absolute = [abs(value) for value in self.cross_track]
        unique_failures = list(dict.fromkeys(failures))
        document = {
            "live_evidence": True,
            "route_outcome": self.route_terminal,
            "cross_track_samples_m": list(self.cross_track),
            "cross_track_m": None if not absolute else {"mean": sum(absolute) / len(absolute),
                                                         "p95": percentile(absolute, 95.0), "max": max(absolute)},
            "goal_error_m": self.goal_error_m,
            "goal_error_yaw_deg": self.goal_error_yaw_deg,
            "footprint_collisions": self.footprint_collisions,
            "geofence_exits": sum(count for state, count in self.status_counts["geofence"].items() if state in ("3", "4")),
            "command": {
                "finite": self.command_nonfinite_components == 0,
                "caps_respected": command_caps_respected,
                "shape_respected": command_shape_respected,
                "limit_checks": cap_checks,
                "maxima": dict(self.maxima),
                "nonfinite_component_count": self.command_nonfinite_components,
                "unsupported_axis_nonzero_count": self.command_shape_violations,
                "cap_exceedance_count": self.command_cap_exceedances,
                "nonzero_after_fault": (self.nonzero_after_fault + self.fault_sink_nonzero
                                        if self.fault_run else self.nonzero_after_fault),
                "violation_reasons": dict(sorted(self.command_violation_reasons.items())),
            },
            "stop": {"trigger_stamp_s": self.stop_trigger_s, "zero_stamp_s": self.stop_observed_s,
                     "latency_s": stop_latency, "overshoot_m": overshoot,
                     "envelope_respected": envelope, "minimum_ttc_s": self._minimum_ttc()},
            "hysteresis": {"stop_observed": self.stop_observed_s is not None,
                           "resume_after_clear": self.clear_after_stop,
                           "reason_events": list(self.reason_events)},
            "timestamps": {"clock_start_s": self.clock[0] if self.clock else None,
                           "clock_end_s": end, "last_by_stream_s": dict(sorted(self.last_stamp.items())),
                           "clock_regressions": self.clock_regressions},
            "sample_counts": dict(sorted(self.counts.items())),
            "status_counts": self.status_counts,
            "missing_topics": missing,
            "verdicts": {
                "topics_complete": not missing,
                "samples_finite": self.nonfinite_samples == 0,
                "clock_monotonic": self.clock_regressions == 0,
                "terminal_evidence": self.route_terminal is not None,
                "terminal_inputs_fresh": not stale,
                "command_limits": command_caps_respected,
                "zero_after_stop": self.stop_observed_s is not None and self.nonzero_after_fault == 0,
                "stopping_envelope": envelope,
            },
            "failures": unique_failures,
            "passed": not unique_failures,
        }
        if self.fault_run:
            fault_contract_valid = not unique_failures
            document.update({
                "fault_injected": self.fault_id if (
                    fault_contract_valid and self.fault_trigger_s is not None) else None,
                "safe_abort": bool(fault_contract_valid and fault_safe_stop and
                                   self.route_terminal == "safe_abort"),
                "zero_within_budget": zero_within_budget,
                "reason_events": sorted(fault_reason_names) if fault_contract_valid else [],
                "latched_until_guarded_reset": bool(
                    fault_contract_valid and latched_until_reset),
                "fault_evidence": {
                    "trigger_stamp_s": self.fault_trigger_s,
                    "actuator_zero_stamp_s": self.fault_zero_s,
                    "actuator_zero_latency_s": fault_zero_latency,
                    "actuator_zero_budget_s": self.limits.fault_stop_budget_s,
                    "reason_observations": fault_reason_observations,
                },
            })
        return document

    def _minimum_ttc(self) -> Optional[float]:
        if self.collision_ttc:
            return min(self.collision_ttc)
        return -1.0 if self.no_finite_ttc_observed else None


def write_artifact(path: str, document: Dict[str, object]) -> None:
    directory = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".rc_metrics-", suffix=".json", dir=directory)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(document, handle, sort_keys=True, separators=(",", ":"), allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


class RosCollector:
    """Lazy, subscriber-only ROS adapter."""

    def __init__(self, args: argparse.Namespace, core: MetricsCore) -> None:
        import rospy
        from gazebo_msgs.msg import ContactsState, ModelStates
        from geometry_msgs.msg import Twist
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from wheelchair_interfaces.msg import (CollisionStatus, GeofenceStatus,
                                                LocalizationStatus, RouteProgress,
                                                SafetyState, SlopeStatus)
        self.rospy = rospy
        self.args = args
        self.core = core
        self.started = time.monotonic()
        self.last_clock = 0.0
        self.terminal_seen_wall: Optional[float] = None
        self.model_missing = False
        rospy.init_node("rc_metrics_collector", anonymous=True, disable_signals=True)
        rospy.Subscriber(args.clock_topic, Clock, self._clock, queue_size=1)
        rospy.Subscriber(args.ground_truth_topic, ModelStates, self._models, queue_size=1)
        rospy.Subscriber(args.contact_topic, ContactsState, self._contacts, queue_size=1)
        rospy.Subscriber(args.route_topic, RouteProgress, self._route, queue_size=1)
        rospy.Subscriber(args.localization_topic, LocalizationStatus,
                         lambda m: self._status("localization", m, (4, 5)), queue_size=1)
        rospy.Subscriber(args.collision_topic, CollisionStatus,
                         self._collision, queue_size=1)
        rospy.Subscriber(args.geofence_topic, GeofenceStatus,
                         lambda m: self._status("geofence", m, (3, 4)), queue_size=1)
        rospy.Subscriber(args.slope_topic, SlopeStatus,
                         lambda m: self._status("slope", m, (3,)), queue_size=1)
        rospy.Subscriber(args.safety_topic, SafetyState,
                         lambda m: self._status("safety", m, (2, 3, 4)), queue_size=1)
        rospy.Subscriber(args.nav_command_topic, Twist,
                         lambda m: self._command("nav_command", m), queue_size=1)
        rospy.Subscriber(args.safe_command_topic, Twist,
                         lambda m: self._command("safe_command", m), queue_size=1)
        rospy.Subscriber(args.fault_event_topic, String, self._fault_event, queue_size=10)
        rospy.Subscriber(args.actuator_sink_topic, Twist,
                         lambda m: self._command("actuator_sink", m), queue_size=1)

    def _stamp(self, message=None) -> float:
        if message is not None and hasattr(message, "header") and message.header.stamp.to_sec() > 0.0:
            return message.header.stamp.to_sec()
        return self.last_clock

    def _clock(self, message) -> None:
        self.last_clock = message.clock.to_sec()
        self.core.observe_clock(self.last_clock)

    def _models(self, message) -> None:
        try:
            index = message.name.index(self.args.model_name)
        except ValueError:
            self.model_missing = True
            return
        pose = message.pose[index]
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.core.observe_pose(self.last_clock, pose.position.x, pose.position.y, yaw)

    def _route(self, message) -> None:
        self.core.observe_route(self._stamp(message), message.state, message.cross_track_error_m,
                                message.distance_remaining_m, message.COMPLETE, message.INVALID)
        if message.state in (message.COMPLETE, message.INVALID) and self.terminal_seen_wall is None:
            self.terminal_seen_wall = time.monotonic()
        if message.state == message.COMPLETE and hasattr(message, "goal_yaw_error_deg"):
            self.core.set_goal_yaw_error(message.goal_yaw_error_deg)
    def _contacts(self, message) -> None:
        self.core.observe_contacts(self._stamp(message), len(message.states))

    def _fault_event(self, message) -> None:
        self.core.observe_fault_event(message.data)



    def _status(self, stream, message, stops) -> None:
        self.core.observe_status(
            stream, self._stamp(message), message.state,
            getattr(message, "reason_mask", 0), stops,
            latched=bool(getattr(message, "estop_latched", False)),
            source=str(getattr(message, "source", stream)),
        )

    def _collision(self, message) -> None:
        self._status("collision", message, (3,))
        self.core.observe_collision_ttc(float(message.time_to_collision_s))

    def _command(self, stream, message) -> None:
        self.core.observe_command(
            stream, self.last_clock,
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z)

    def collect(self) -> Tuple[Dict[str, object], bool]:
        rate = self.rospy.Rate(50)
        timed_out = False
        while not self.rospy.is_shutdown():
            if (
                self.terminal_seen_wall is not None
                and time.monotonic() - self.terminal_seen_wall >= self.args.settle_time
            ):
                break
            if time.monotonic() - self.started >= self.args.timeout:
                timed_out = True
                break
            rate.sleep()
        result = self.core.finalize(timed_out=timed_out)
        if self.model_missing and "ground_truth" not in self.core.seen:
            result["failures"].append("model '{}' absent from ground truth".format(self.args.model_name))
            result["passed"] = False
        return result, timed_out


def terminal_settle_time(value: str) -> float:
    settle = float(value)
    if not math.isfinite(settle) or settle < MINIMUM_TERMINAL_SETTLE_S:
        raise argparse.ArgumentTypeError(
            "settle time must be at least {:.2f} seconds".format(
                MINIMUM_TERMINAL_SETTLE_S))
    return settle


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("--output", required=True)
    value.add_argument("--world", required=True)
    value.add_argument("--seed", required=True, type=int)
    value.add_argument("--robustness", default="false", choices=("true", "false"))
    value.add_argument("--fault", default="none")
    value.add_argument("--timeout", type=float, default=170.0)
    value.add_argument("--model-name", default="wheelchair")
    value.add_argument("--settle-time", type=terminal_settle_time, default=0.60)
    value.add_argument("--clock-topic", default="/clock")
    value.add_argument("--ground-truth-topic", default="/gazebo/model_states")
    value.add_argument("--contact-topic", default="/simulation/contacts")
    value.add_argument("--route-topic", default="/route/progress")
    value.add_argument("--localization-topic", default="/localization/status")
    value.add_argument("--collision-topic", default="/safety/collision_status")
    value.add_argument("--geofence-topic", default="/route_safety/geofence_status")
    value.add_argument("--slope-topic", default="/safety/slope_status")
    value.add_argument("--safety-topic", default="/safety/state")
    value.add_argument("--nav-command-topic", default="/cmd_vel_nav")
    value.add_argument("--safe-command-topic", default="/cmd_vel_safe")
    value.add_argument("--fault-event-topic", default="/simulation/fault_event")
    value.add_argument(
        "--actuator-command-topic", "--actuator-sink-topic",
        dest="actuator_sink_topic",
        default="/wheelchair_base_controller/cmd_vel",
    )
    value.add_argument("--linear-cap-mps", type=float, default=0.55)
    value.add_argument("--angular-cap-rps", type=float, default=0.85)
    value.add_argument("--stop-budget-s", type=float, default=FAULT_STOP_BUDGET_S)
    return value


def main() -> int:
    args = parser().parse_args()
    limits = Limits(
        linear_mps=args.linear_cap_mps,
        angular_rps=args.angular_cap_rps,
        fault_stop_budget_s=args.stop_budget_s,
    )
    core = MetricsCore(limits, args.fault)
    error = None
    timed_out = False
    try:
        result, timed_out = RosCollector(args, core).collect()
    except BaseException as exc:
        error = "collector exception: {}: {}".format(type(exc).__name__, exc)
        result = core.finalize(timed_out=False)
        result["failures"] = list(dict.fromkeys(result["failures"] + [error]))
        result["passed"] = False
    artifact = {
        "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
        "scenario": {"world": args.world, "seed": args.seed,
                     "robustness": args.robustness == "true", "fault": args.fault},
        "authority": {"simulation_only": True, "hardware_motion_authorized": False,
                      "passenger_operation_authorized": False},
        "simulation_only": True,
        "hardware_motion_authorized": False,
        "passenger_operation_authorized": False,
        "collector": {"timeout_s": args.timeout, "timed_out": timed_out, "error": error},
        "source_topics": [
            topic for stream, topic in (
                ("clock", args.clock_topic),
                ("ground_truth", args.ground_truth_topic),
                ("contacts", args.contact_topic),
                ("route", args.route_topic),
                ("localization", args.localization_topic),
                ("collision", args.collision_topic),
                ("geofence", args.geofence_topic),
                ("slope", args.slope_topic),
                ("safety", args.safety_topic),
                ("nav_command", args.nav_command_topic),
                ("safe_command", args.safe_command_topic),
                ("fault_event", args.fault_event_topic),
                ("actuator_sink", args.actuator_sink_topic),
            ) if stream in core.seen
        ],
    }
    artifact.update(result)
    write_artifact(args.output, artifact)
    return 0 if artifact["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
