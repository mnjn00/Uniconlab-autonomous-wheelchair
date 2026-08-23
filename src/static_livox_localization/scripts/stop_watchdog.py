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

# How long a stop may go unhonoured before it is a fault rather than the
# wheels coasting down. The two clean stops in the 08-20 run settled in
# 0.22 s and 0.25 s; the faults ran 2.15 s, 2.44 s, 3.73 s and 12.97 s.
# 0.40 s sits clear of both.
GRACE_S = 0.40
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
                 command_fresh_s=COMMAND_FRESH_S):
        self.grace_s = float(grace_s)
        self.moving_mps = float(moving_mps)
        self.command_fresh_s = float(command_fresh_s)
        self.stop_since = None
        self.latched = False
        self.last_command_s = 0.0

    def observe_command(self, data, now_s):
        if commanded_stop(data):
            if self.stop_since is None:
                self.stop_since = float(now_s)
        else:
            self.stop_since = None
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
        if max(left, right) <= self.moving_mps:
            # Honoured. Arm again for the next stop.
            self.latched = False
            return None
        held = float(now_s) - self.stop_since
        if held < self.grace_s or self.latched:
            return None
        self.latched = True
        return ("stop not honoured for %.2f s: left %.2f m/s, right %.2f m/s"
                % (held, left, right))


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
