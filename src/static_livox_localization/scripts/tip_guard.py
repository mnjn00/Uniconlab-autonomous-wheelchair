#!/usr/bin/env python3
"""Final-stage command relay: forwards /cmd_vel_gated to /cmd_vel through a
rate limiter, and stops on a stale command stream.

Sits as the LAST node before the wheel base (safety_gate -> tip_guard ->
wheel_cmd_tmp.py). Tip-over detection/prevention (predictive pitch-rate
trip against /livox/imu, fused-pitch release logic, the correlation
self-check and its accel governor) was removed: on this deployment's
terrain the trip trigger fired on ordinary pavement bumps (a bump and a
real tip produce the same brief pitch-rate spike, and ordinary bumps are
far more frequent), and a false stop was judged more dangerous than no
dynamic tip check at all. This node keeps its name and position in the
chain because downstream code authorizes /cmd_vel only from caller
"/tip_guard".
"""

import os
import sys

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
# catkin_install_python leaves a relay in devel/lib that exec()s this file,
# so sys.path[0] is the relay's directory, not this one, and the policy
# modules sitting beside this file are not importable - the relay does set
# __file__ to this source path, so recover the directory from it. Without
# this the node dies at import on the vehicle while every offline test,
# which imports the modules directly, still passes.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from priest_constraints import GATED_INPUT_STALE_S, TIP_GUARD_RATE_HZ
from tip_guard_policy import next_linear_speed

GUARD_HZ = TIP_GUARD_RATE_HZ
INPUT_STALE_S = GATED_INPUT_STALE_S
ACCEL_LIMIT = 0.30


class TipGuard:
    def __init__(self):
        rospy.init_node("tip_guard")

        self.raw = Twist()
        self.raw_stamp = rospy.Time(0)
        self.current_speed = 0.0
        self.status = ""

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/tip_guard/status", String, queue_size=2)
        rospy.Subscriber("/cmd_vel_gated", Twist, self.on_raw, queue_size=1)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    # ------------------------------------------------------------ callbacks
    def on_raw(self, message):
        self.raw = message
        self.raw_stamp = rospy.Time.now()

    # ------------------------------------------------------------ logic
    def spin(self):
        rate = rospy.Rate(GUARD_HZ)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(1e-3, (now - last).to_sec())
            last = now

            stale = (now - self.raw_stamp).to_sec() > INPUT_STALE_S

            self.current_speed = next_linear_speed(
                self.current_speed,
                self.raw.linear.x,
                ACCEL_LIMIT,
                dt,
                stale)

            out = Twist()
            out.linear.x = self.current_speed
            out.angular.z = 0.0 if stale else self.raw.angular.z
            self.pub.publish(out)

            state = "STALE" if stale else "OK"
            self.status_pub.publish(String(
                data="%s v=%.2f" % (state, self.current_speed)))
            if state != self.status:
                rospy.loginfo("tip_guard: %s", state)
                self.status = state
            rate.sleep()


if __name__ == "__main__":
    TipGuard().spin()
