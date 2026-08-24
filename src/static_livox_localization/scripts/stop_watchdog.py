#!/usr/bin/env python3
"""Verify that a gated stop intent reaches the measured wheels.

The final wheel command is intentionally not used as the stop intent.  The
base now ramps a stop with C/C or C/S frames, and tip_guard may spend time
decelerating before /cmd_vel itself reaches zero.  The watchdog therefore
observes three separate facts:

* /cmd_vel_gated: the safety chain requested a stop;
* /cmd_vel: the final command reached zero after its rate limit;
* /wheel_status: the two wheels actually slowed symmetrically.

The 2026-08-23 field failure had one stopped wheel and one wheel at 0.36 m/s.
That is detected as a pivot before the generic failed-stop timeout, logged
with the UART counters, and followed by one best-effort request for mode 77.
"""

import json
import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray, String


AUTO_MODE = 65
MANUAL_MODE = 77
FORWARD_DIRECTION = ord("C")
REVERSE_DIRECTION = ord("W")
STOP_DIRECTION = ord("S")
WHEEL_SEPARATION_M = 0.54

FINAL_STOP_GRACE_S = 0.40
MOVING_MPS = 0.15
COMMAND_FRESH_S = 0.30
NOT_SLOWING_WINDOW_S = 0.50
MIN_DECELERATION_MPS = 0.05
PIVOT_STATIONARY_MPS = 0.15
PIVOT_MOVING_MPS = 0.30
PIVOT_YAW_RPS = 0.50
PIVOT_CONFIRM_S = 0.15


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _is_zero_twist(linear, angular):
    linear = _finite(linear)
    angular = _finite(angular)
    return linear is not None and angular is not None and \
        abs(linear) <= 1e-6 and abs(angular) <= 1e-6


def wheel_speed(direction, byte_value):
    """Decode one signed wheel speed from the base status protocol."""
    try:
        direction = int(direction)
        speed = (float(byte_value) - 0x21) / 10.0 / 3.6
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(speed):
        return 0.0
    if direction == FORWARD_DIRECTION:
        return speed
    if direction == REVERSE_DIRECTION:
        return -speed
    return 0.0


def reported_wheel_speeds(data):
    """Return signed (left, right) speeds, or None for a short frame."""
    if len(data) < 6:
        return None
    return wheel_speed(data[2], data[3]), wheel_speed(data[4], data[5])


def reported_motion(data):
    """Compatibility helper returning absolute measured speeds."""
    speeds = reported_wheel_speeds(data)
    if speeds is None:
        return None
    return abs(speeds[0]), abs(speeds[1])


class StopHonouredCheck:
    """Pure stop-intent state machine, independent of ROS."""

    def __init__(
            self,
            final_stop_grace_s=FINAL_STOP_GRACE_S,
            moving_mps=MOVING_MPS,
            command_fresh_s=COMMAND_FRESH_S,
            not_slowing_window_s=NOT_SLOWING_WINDOW_S,
            min_deceleration_mps=MIN_DECELERATION_MPS,
            pivot_stationary_mps=PIVOT_STATIONARY_MPS,
            pivot_moving_mps=PIVOT_MOVING_MPS,
            pivot_yaw_rps=PIVOT_YAW_RPS,
            pivot_confirm_s=PIVOT_CONFIRM_S):
        self.final_stop_grace_s = float(final_stop_grace_s)
        self.moving_mps = float(moving_mps)
        self.command_fresh_s = float(command_fresh_s)
        self.not_slowing_window_s = float(not_slowing_window_s)
        self.min_deceleration_mps = float(min_deceleration_mps)
        self.pivot_stationary_mps = float(pivot_stationary_mps)
        self.pivot_moving_mps = float(pivot_moving_mps)
        self.pivot_yaw_rps = float(pivot_yaw_rps)
        self.pivot_confirm_s = float(pivot_confirm_s)

        self.intent_since = None
        self.final_stop_since = None
        self.last_gated_s = None
        self.last_final_s = None
        self.baseline_speed_mps = None
        self.pivot_since = None
        self.latched = False

    def _disarm(self):
        self.intent_since = None
        self.final_stop_since = None
        self.baseline_speed_mps = None
        self.pivot_since = None
        self.latched = False

    def observe_gated_command(self, linear, angular, now_s):
        now_s = float(now_s)
        self.last_gated_s = now_s
        if _is_zero_twist(linear, angular):
            if self.intent_since is None:
                self.intent_since = now_s
                self.final_stop_since = None
                self.baseline_speed_mps = None
                self.pivot_since = None
                self.latched = False
            return
        self._disarm()

    def observe_final_command(self, linear, angular, now_s):
        now_s = float(now_s)
        self.last_final_s = now_s
        if _is_zero_twist(linear, angular):
            if self.final_stop_since is None:
                self.final_stop_since = now_s
        else:
            self.final_stop_since = None

    def _fault(self, code, now_s, left, right, yaw):
        intent_age = float(now_s) - self.intent_since
        gated_age = float(now_s) - self.last_gated_s
        final_age = -1.0 if self.final_stop_since is None else \
            float(now_s) - self.final_stop_since
        reasons = {
            "ONE_WHEEL_PIVOT": "one wheel stopped while the other kept moving",
            "NOT_SLOWING": "measured wheels did not slow after stop intent",
            "STOP_NOT_HONOURED": "final zero command was not honoured",
        }
        self.latched = True
        return {
            "code": code,
            "reason": reasons[code],
            "left_mps": left,
            "right_mps": right,
            "pivot_yaw_rps": yaw,
            "intent_age_s": intent_age,
            "gated_age_s": gated_age,
            "final_age_s": final_age,
        }

    def observe_status(self, data, now_s, mode):
        """Return one structured fault while a fresh stop intent is armed."""
        speeds = reported_wheel_speeds(data)
        if speeds is None or int(mode) != AUTO_MODE:
            return None
        if self.intent_since is None or self.last_gated_s is None:
            return None
        now_s = float(now_s)
        if now_s - self.last_gated_s > self.command_fresh_s or self.latched:
            return None

        left, right = speeds
        magnitudes = abs(left), abs(right)
        current = max(magnitudes)
        yaw = abs(right - left) / WHEEL_SEPARATION_M
        if self.baseline_speed_mps is None:
            self.baseline_speed_mps = current

        pivoting = min(magnitudes) <= self.pivot_stationary_mps and \
            current >= self.pivot_moving_mps and yaw >= self.pivot_yaw_rps
        if pivoting:
            if self.pivot_since is None:
                self.pivot_since = now_s
            if now_s - self.pivot_since >= self.pivot_confirm_s:
                return self._fault(
                    "ONE_WHEEL_PIVOT", now_s, left, right, yaw)
        else:
            self.pivot_since = None

        intent_age = now_s - self.intent_since
        if intent_age >= self.not_slowing_window_s and \
                self.baseline_speed_mps > self.moving_mps and \
                current > self.baseline_speed_mps - self.min_deceleration_mps:
            return self._fault("NOT_SLOWING", now_s, left, right, yaw)

        final_is_fresh = self.last_final_s is not None and \
            now_s - self.last_final_s <= self.command_fresh_s
        if self.final_stop_since is not None and final_is_fresh and \
                now_s - self.final_stop_since >= self.final_stop_grace_s and \
                current > self.moving_mps:
            return self._fault("STOP_NOT_HONOURED", now_s, left, right, yaw)
        return None


class StopWatchdog:
    def __init__(self):
        rospy.init_node("stop_watchdog")
        legacy_grace = float(rospy.get_param("~grace_s", FINAL_STOP_GRACE_S))
        self.check = StopHonouredCheck(
            final_stop_grace_s=float(rospy.get_param(
                "~final_stop_grace_s", legacy_grace)),
            moving_mps=float(rospy.get_param("~moving_mps", MOVING_MPS)),
        )
        self.attempt_stop = bool(rospy.get_param("~attempt_stop", True))
        self.tx_diag = {}
        self.alarm_pub = rospy.Publisher(
            "stop_watchdog/alarm", String, queue_size=4, latch=True)
        self.mode_pub = rospy.Publisher("mode_cmd", Int16, queue_size=1)
        rospy.Subscriber(
            "/cmd_vel_gated", Twist, self.on_gated_command, queue_size=1)
        rospy.Subscriber(
            "/cmd_vel", Twist, self.on_final_command, queue_size=1)
        rospy.Subscriber(
            "wheel_status", Int16MultiArray, self.on_status, queue_size=5)
        rospy.Subscriber("uart_tx_diag", String, self.on_tx_diag, queue_size=5)
        rospy.loginfo(
            "stop watchdog: intent %.2f s, final %.2f s, pivot %.2f s",
            self.check.not_slowing_window_s,
            self.check.final_stop_grace_s,
            self.check.pivot_confirm_s,
        )

    def on_tx_diag(self, message):
        try:
            self.tx_diag = json.loads(message.data)
        except (TypeError, ValueError):
            pass

    def on_gated_command(self, message):
        self.check.observe_gated_command(
            message.linear.x,
            message.angular.z,
            rospy.Time.now().to_sec(),
        )

    def on_final_command(self, message):
        self.check.observe_final_command(
            message.linear.x,
            message.angular.z,
            rospy.Time.now().to_sec(),
        )

    def on_status(self, message):
        data = list(message.data)
        if len(data) < 2:
            return
        now_s = rospy.Time.now().to_sec()
        fault = self.check.observe_status(data, now_s, data[1])
        if fault is None:
            return

        alarm = {"stamp": now_s}
        alarm.update(fault)
        alarm["uart_tx"] = self.tx_diag
        rospy.logerr(
            "STOP FAULT %s - %s | uart tx %s",
            fault["code"],
            fault["reason"],
            json.dumps(self.tx_diag),
        )
        self.alarm_pub.publish(String(data=json.dumps(alarm)))
        if self.attempt_stop:
            self.mode_pub.publish(Int16(data=MANUAL_MODE))


if __name__ == "__main__":
    StopWatchdog()
    rospy.spin()
