#!/usr/bin/env python3
"""Watches whether a commanded stop actually reaches the wheels.

On 2026-08-19 the chair was told to stop, kept the left wheel at 0.72 m/s for
12.97 seconds, and turned about 2.9 times on the spot at the finish line. The
IMU measured the rotation and /wheel_status reported the speed, so every fact
needed to catch it inside a second was already on the bus - nothing was
watching. On 08-16 the same fault ran 3.7 s. Neither was noticed live.

What this cannot do is fix it. If the reason a stop is not honoured is that
the serial write is blocked, then the mode frame this node sends is blocked
behind it, and the joystick is the only thing left. So the alarm is the
product here and the stop attempt is a bonus: the node exists to put a loud,
timestamped, self-explaining record on the bus at the moment it happens,
carrying the uart TX counters that say whether the frames reached the wire.

Decoding, from base_model/src/wheel_cmd_tmp.py and uart.py:
  wheel_cmd     [ll, lv, rr, rv, brk]      direction C forward, W back, S stop
  wheel_status  [72, mode, ll, lv, rr, rv, brk, battery, checksum, 13, 10]
  speed         (byte - 0x21) / 10 / 3.6 m/s
"""

import json

import rospy
from std_msgs.msg import Int16, Int16MultiArray, String


AUTO_MODE = 65
MANUAL_MODE = 77
STOP_DIRECTION = ord("S")

# A stop is judged on whether the wheels are SLOWING, not on how long they
# take. The first version of this node used a flat 0.40 s grace, read off
# two clean stops in the 08-20 run that settled in 0.22 s and 0.25 s. Those
# were stops from a crawl. From the 0.80 m/s cruise this chair coasts down
# at about 0.47 m/s^2 - measured 2026-08-23, 0.75 m/s at the stop command
# to rest 1.7 s later - so reaching MOVING_MPS inside 0.40 s would need
# 1.6 m/s^2, which it cannot do. The grace therefore fired on every stop
# from cruise, and the failsafe below it dropped the chair into manual. It
# did exactly that in front of a pedestrian: the profile saw the person,
# chose to wait, commanded the stop, the wheels were slowing properly, and
# this node took the chair off auto anyway.
#
# The fault it is looking for does not look like slow braking. In all four
# recorded events the speed held flat or ROSE after the stop - 0.78 to
# 0.94, 0.72 to 0.83, 0.61 to 0.67 - because the base was still running a
# ramp toward the previous setpoint. Inertia cannot do that. So the test is
# an envelope: from the speed the wheels carried when the stop was issued,
# they must keep shedding at least this much, or something is driving them.
STOP_DECEL_FLOOR_MPS2 = 0.15
# Never judge inside this. One frame of noise at the moment of the command
# is not evidence of anything.
GRACE_S = 0.5
# Still rolling this long after a stop is a fault whatever the envelope
# says. From 0.80 m/s the envelope alone expects rest by 4.3 s.
CEILING_S = 5.0
# A rise this far above the speed carried into the stop is the fault's own
# signature and is not waited out any further than the grace.
RISE_MARGIN_MPS = 0.06
# One wheel stopped while the other still carries this much is a split, and
# a split is a pivot: 0.67 m/s across a 0.54 m track is 71 deg/s. Inertia
# cannot produce it - both wheels carry the same momentum into a stop - so
# it needs no envelope and no patience, only long enough to be sure it is
# not one noisy frame.
#
# Measured 2026-08-23 alongside a pedestrian: the right wheel honoured the
# stop in 0.5 s at 1.66 m/s^2 while the left held 0.64-0.69 for 0.7 s, and
# the chair turned about 50 degrees before this node caught it on the
# deceleration envelope instead. Both wheels are told to stop by the same
# frame - /wheel_cmd read S0.00 / S0.00 throughout - so a split is the base
# ignoring one channel, not anything upstream of it.
SPLIT_MPS = 0.35
SPLIT_HOLD_S = 0.15
# Below this the report is noise on a stationary wheel, not motion.
MOVING_MPS = 0.15
# A command older than this says nothing about now.
COMMAND_FRESH_S = 0.30


def wheel_speed(byte_value):
    return (float(byte_value) - 0x21) / 10.0 / 3.6


def commanded_stop(data):
    """True when the frame on the wire tells both wheels to stop."""
    return len(data) >= 3 and \
        int(data[0]) == STOP_DIRECTION and int(data[2]) == STOP_DIRECTION


def reported_motion(data):
    """(left, right) speed in m/s from a status frame, or None if malformed."""
    if len(data) < 6:
        return None
    return (abs(wheel_speed(data[3])), abs(wheel_speed(data[5])))


class StopHonouredCheck:
    """The decision, with no ROS in it so it can be tested without a chair.

    Feed it commands and status frames with timestamps; it returns a reason
    string the first cycle a stop has gone unhonoured past the grace, then
    None until the wheels stop and the condition can arm again.
    """

    def __init__(self, grace_s=GRACE_S, moving_mps=MOVING_MPS,
                 command_fresh_s=COMMAND_FRESH_S,
                 decel_floor_mps2=STOP_DECEL_FLOOR_MPS2,
                 ceiling_s=CEILING_S, rise_margin_mps=RISE_MARGIN_MPS,
                 split_mps=SPLIT_MPS, split_hold_s=SPLIT_HOLD_S):
        self.grace_s = float(grace_s)
        self.moving_mps = float(moving_mps)
        self.command_fresh_s = float(command_fresh_s)
        self.decel_floor_mps2 = float(decel_floor_mps2)
        self.ceiling_s = float(ceiling_s)
        self.rise_margin_mps = float(rise_margin_mps)
        self.split_mps = float(split_mps)
        self.split_hold_s = float(split_hold_s)
        self.split_since = None
        self.stop_since = None
        self.entry_speed = None
        self.latched = False
        self.last_command_s = 0.0

    def observe_command(self, data, now_s):
        if commanded_stop(data):
            if self.stop_since is None:
                self.stop_since = float(now_s)
        else:
            self.stop_since = None
            self.entry_speed = None
            self.split_since = None
            self.latched = False
        self.last_command_s = float(now_s)

    def observe_status(self, data, now_s, mode):
        """Returns a reason when the stop has not been honoured, else None."""
        motion = reported_motion(data)
        if motion is None:
            return None
        left, right = motion
        if int(mode) != AUTO_MODE or self.stop_since is None:
            return None
        if float(now_s) - self.last_command_s > self.command_fresh_s:
            # The command stream has stopped; that is a different fault and
            # uart.py's own watchdog owns it.
            return None
        speed = max(left, right)
        if self.entry_speed is None:
            # The speed the stop was issued against. Taken from the first
            # frame after it rather than the last one before, which is the
            # conservative side: any braking already done shrinks the
            # envelope this has to stay inside.
            self.entry_speed = speed
        if speed <= self.moving_mps:
            # Honoured. Arm again for the next stop.
            self.split_since = None
            self.latched = False
            return None
        held = float(now_s) - self.stop_since
        # Judged before the envelope and before the grace, because the
        # chair is turning while this is true and every frame spent
        # confirming it is more of the turn.
        if min(left, right) <= self.moving_mps and speed >= self.split_mps:
            if self.split_since is None:
                self.split_since = float(now_s)
            if float(now_s) - self.split_since >= self.split_hold_s \
                    and not self.latched:
                self.latched = True
                return ("one wheel ignored a stop: left %.2f m/s, right "
                        "%.2f m/s, %.0f deg/s of pivot"
                        % (left, right, abs(left - right) / 0.54 * 57.3))
        else:
            self.split_since = None
        if held < self.grace_s or self.latched:
            return None
        allowed = self.entry_speed - self.decel_floor_mps2 * held
        if speed > self.entry_speed + self.rise_margin_mps:
            fault = ("wheels sped up after a stop: %.2f m/s against %.2f "
                     "when it was issued" % (speed, self.entry_speed))
        elif held >= self.ceiling_s:
            fault = "still rolling %.2f s after a stop" % held
        elif speed > max(self.moving_mps, allowed):
            fault = ("not slowing after a stop: %.2f m/s at %.2f s, "
                     "%.2f expected" % (speed, held, allowed))
        else:
            return None
        self.latched = True
        return ("%s (left %.2f m/s, right %.2f m/s)"
                % (fault, left, right))


class StopWatchdog:
    def __init__(self):
        rospy.init_node("stop_watchdog")
        self.check = StopHonouredCheck(
            grace_s=float(rospy.get_param("~grace_s", GRACE_S)),
            moving_mps=float(rospy.get_param("~moving_mps", MOVING_MPS)))
        self.attempt_stop = bool(
            rospy.get_param("~attempt_stop", True))
        self.tx_diag = {}
        self.alarm_pub = rospy.Publisher(
            "stop_watchdog/alarm", String, queue_size=4, latch=True)
        self.mode_pub = rospy.Publisher("mode_cmd", Int16, queue_size=1)
        rospy.Subscriber("wheel_cmd", Int16MultiArray, self.on_command)
        rospy.Subscriber("wheel_status", Int16MultiArray, self.on_status)
        rospy.Subscriber("uart_tx_diag", String, self.on_tx_diag)
        rospy.loginfo(
            "stop watchdog: %.2f s grace, %.2f m/s floor, stop attempt %s",
            self.check.grace_s, self.check.moving_mps,
            "on" if self.attempt_stop else "off")

    def on_tx_diag(self, message):
        try:
            self.tx_diag = json.loads(message.data)
        except ValueError:
            pass

    def on_command(self, message):
        self.check.observe_command(
            list(message.data), rospy.Time.now().to_sec())

    def on_status(self, message):
        data = list(message.data)
        if len(data) < 2:
            return
        reason = self.check.observe_status(
            data, rospy.Time.now().to_sec(), data[1])
        if reason is None:
            return
        motion = reported_motion(data) or (0.0, 0.0)
        alarm = {
            "stamp": rospy.Time.now().to_sec(),
            "reason": reason,
            "left_mps": round(motion[0], 3),
            "right_mps": round(motion[1], 3),
            # Whether our frames reached the wire is the whole question, and
            # this is the only place the answer is recorded at the moment it
            # matters.
            "uart_tx": self.tx_diag,
        }
        rospy.logerr("STOP NOT HONOURED - %s | uart tx %s", reason,
                     json.dumps(self.tx_diag))
        self.alarm_pub.publish(String(data=json.dumps(alarm)))
        if self.attempt_stop:
            # Best effort only. If the serial write is what is stuck, this
            # frame is stuck behind it and the joystick is the failsafe.
            self.mode_pub.publish(Int16(data=MANUAL_MODE))


if __name__ == "__main__":
    StopWatchdog()
    rospy.spin()
