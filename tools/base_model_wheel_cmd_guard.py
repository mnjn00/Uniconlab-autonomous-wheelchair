#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray


AUTO_MODE = 65
EXPECTED_CMD_CALLER = "/tip_guard"
EXPECTED_STATUS_CALLER = "/uart"
MAX_LINEAR_MPS = 1.5
MAX_ANGULAR_RAD_S = 0.6
WHEEL_SEPARATION_M = 0.54
STOP_COMMAND = (83, 33, 83, 33, 79)

# Wheel speeds are sent as tenths of a km/h on top of a 33 offset, so one
# count is 0.1 km/h (0.028 m/s).
COUNTS_PER_KMH = 10.0
MAGNITUDE_OFFSET = 33

# S is a stopped state on this base, not a brake command.  A stop request
# therefore ramps from the measured wheel speeds and changes each wheel to S
# only after that wheel is already close to rest.
RAMP_DECAY = 0.9
RAMP_BLEED_MPS = 0.09
RAMP_TERMINAL_MPS = 0.06
STATUS_FRESH_S = 0.30

# Below this the wheels were observed not to turn the loaded chair at all.
# On 2026-07-29 the follower held its maximum +0.5 rad/s for four seconds
# while the faster wheel sat at 0.9 km/h and the chair rotated at 0.03-0.05
# rad/s - effectively not at all - then rotated at 0.57 rad/s once the
# wheels reached 1.3-1.6 km/h. The differential itself was correct
# throughout: 0.9 km/h across 0.54 m is the 0.46 rad/s that was asked for.
# What was missing was enough absolute speed to break stiction with a rider
# aboard, so the encoded difference described a turn the base never made.
TURN_AUTHORITY_KMH = 1.3
# Reaching that floor means driving both wheels faster, which adds forward
# speed the planner did not ask for. That is capped hard: enough to turn,
# never enough to run away.
TURN_AUTHORITY_MAX_LINEAR_MPS = 0.30
YAW_DEADBAND_RAD_S = 0.05


def wheel_speed_mps(direction, magnitude):
    """Decode one wheel from a status frame, negative when reversing."""
    try:
        speed = (float(magnitude) - MAGNITUDE_OFFSET) / COUNTS_PER_KMH / 3.6
        direction = int(direction)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(speed):
        return 0.0
    if direction == 67:
        return speed
    if direction == 87:
        return -speed
    return 0.0


def stop_ramp_command(left_mps, right_mps):
    """Build one measured-speed braking step for the two wheels."""
    def one(speed):
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            return 83, MAGNITUDE_OFFSET
        if not math.isfinite(speed):
            return 83, MAGNITUDE_OFFSET
        target = abs(speed) * RAMP_DECAY - RAMP_BLEED_MPS
        if target < RAMP_TERMINAL_MPS:
            return 83, MAGNITUDE_OFFSET
        magnitude = int(round(target * 3.6 * COUNTS_PER_KMH)) + \
            MAGNITUDE_OFFSET
        return (67 if speed > 0.0 else 87), min(magnitude, 127)

    left = one(left_mps)
    right = one(right_mps)
    return left[0], left[1], right[0], right[1], 79


def message_caller_id(message):
    header = getattr(message, "_connection_header", None)
    if not isinstance(header, dict):
        return ""
    return str(header.get("callerid", "")).strip()


def compute_wheel_command(linear_x, angular_z):
    try:
        linear_x = float(linear_x)
        angular_z = float(angular_z)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(linear_x) or not math.isfinite(angular_z) or \
            linear_x < 0.0 or linear_x > MAX_LINEAR_MPS or \
            abs(angular_z) > MAX_ANGULAR_RAD_S:
        return None
    left = (linear_x - angular_z * WHEEL_SEPARATION_M / 2.0) * 3.6
    right = (linear_x + angular_z * WHEEL_SEPARATION_M / 2.0) * 3.6

    # Give a requested turn enough wheel speed to actually happen. Adding
    # the SAME amount to both wheels leaves right-left untouched, so the
    # yaw rate is exactly the one asked for; only the forward speed rises,
    # and only as far as TURN_AUTHORITY_MAX_LINEAR_MPS allows.
    if abs(angular_z) > YAW_DEADBAND_RAD_S:
        fastest = max(abs(left), abs(right))
        headroom = (TURN_AUTHORITY_MAX_LINEAR_MPS - linear_x) * 3.6
        boost = min(max(TURN_AUTHORITY_KMH - fastest, 0.0), max(headroom, 0.0))
        left += boost
        right += boost

    def encode(speed):
        direction = 67 if speed > 0.0 else (87 if speed < 0.0 else 83)
        # Round rather than truncate. Truncation drags every wheel toward
        # zero by up to a full count, which at these speeds is most of the
        # command: the 0.198 km/h inner wheel above became 1 count, 0.1
        # km/h, half of what was asked.
        magnitude = int(round(abs(speed) * COUNTS_PER_KMH)) + MAGNITUDE_OFFSET
        if magnitude > 127:
            return None
        return direction, magnitude

    left_encoded = encode(left)
    right_encoded = encode(right)
    if left_encoded is None or right_encoded is None:
        return None
    return (
        left_encoded[0],
        left_encoded[1],
        right_encoded[0],
        right_encoded[1],
        79,
    )


class WheelCommandGuard:
    def __init__(self):
        rospy.init_node("wheel_cmd")
        self.publisher = rospy.Publisher(
            "wheel_cmd", Int16MultiArray, queue_size=1)
        self.mode = None
        self.fault_latched = False
        self.measured_left_mps = 0.0
        self.measured_right_mps = 0.0
        self.measured_stamp = None
        rospy.Subscriber(
            "cmd_vel", Twist, self.on_velocity, queue_size=1)
        rospy.Subscriber(
            "wheel_status",
            Int16MultiArray,
            self.on_wheel_status,
            queue_size=5,
        )

    def publish(self, values):
        message = Int16MultiArray()
        message.data = list(values)
        self.publisher.publish(message)

    def publish_stop(self):
        now = rospy.Time.now().to_sec()
        if self.measured_stamp is None or \
                now - self.measured_stamp > STATUS_FRESH_S:
            self.publish(STOP_COMMAND)
            return
        self.publish(stop_ramp_command(
            self.measured_left_mps, self.measured_right_mps))

    def on_velocity(self, message):
        if self.mode != AUTO_MODE:
            self.publish_stop()
            return
        if message_caller_id(message) != EXPECTED_CMD_CALLER:
            self.fault_latched = True
        command = compute_wheel_command(
            message.linear.x, message.angular.z)
        if command is None:
            self.fault_latched = True
        if self.fault_latched:
            self.publish_stop()
            return
        if command == STOP_COMMAND:
            self.publish_stop()
            return
        self.publish(command)

    def on_wheel_status(self, message):
        if message_caller_id(message) != EXPECTED_STATUS_CALLER or \
                len(message.data) <= 1:
            self.mode = None
            self.fault_latched = True
            self.publish_stop()
            return
        if len(message.data) >= 6:
            self.measured_left_mps = wheel_speed_mps(
                message.data[2], message.data[3])
            self.measured_right_mps = wheel_speed_mps(
                message.data[4], message.data[5])
            self.measured_stamp = rospy.Time.now().to_sec()
        next_mode = int(message.data[1])
        if next_mode != AUTO_MODE:
            self.fault_latched = False
        elif self.mode != AUTO_MODE:
            self.fault_latched = False
        self.mode = next_mode


if __name__ == "__main__":
    WheelCommandGuard()
    rospy.spin()
