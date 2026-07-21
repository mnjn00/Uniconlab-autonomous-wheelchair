#!/usr/bin/env python3
"""Independent last-line safety gate between the planner and the wheel base.

Deliberately knows nothing about routes or planning: it forwards
/cmd_vel_raw to /cmd_vel only when its OWN forward-corridor check passes,
clamps speeds, replaces stale or missing input with a stop, and publishes
continuously so the base always has a live command stream. If the planner
misbehaves or dies, this gate stops the chair; if this gate dies, the
uart-level watchdog stops the chair.
"""

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2

GATE_HZ = 15.0
INPUT_STALE_S = 0.6
CLOUD_STALE_S = 1.0
HARD_V_LIMIT = 0.6
HARD_W_LIMIT = 0.6
STOP_DISTANCE_M = 0.8
CHECK_RANGE_M = 1.4
HALF_WIDTH_M = 0.5
DROP_M = 0.09
STEP_M = 0.13
OBSTACLE_MIN_Z = 0.15
OBSTACLE_MAX_Z = 1.9


class SafetyGate:
    def __init__(self):
        rospy.init_node("safety_gate")
        self.raw = Twist()
        self.raw_stamp = rospy.Time(0)
        self.cloud = None
        self.cloud_stamp = rospy.Time(0)
        self.blocked_reason = ""
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/cmd_vel_raw", Twist, self.on_raw, queue_size=1)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.on_cloud, queue_size=2)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    def on_raw(self, message):
        self.raw = message
        self.raw_stamp = rospy.Time.now()

    def on_cloud(self, message):
        self.cloud = np.array(list(pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32)
        self.cloud_stamp = message.header.stamp

    def forward_blocked(self):
        if self.cloud is None or len(self.cloud) < 100:
            return "NO_CLOUD"
        pts = self.cloud
        near = pts[(pts[:, 0] > -0.6) & (pts[:, 0] < 1.0) &
                   (np.abs(pts[:, 1]) < 0.7)]
        if len(near) < 20:
            return "NO_GROUND_REF"
        ego_ground = np.percentile(near[:, 2], 10)

        zone = pts[(pts[:, 0] > 0.25) & (pts[:, 0] < CHECK_RANGE_M) &
                   (np.abs(pts[:, 1]) < HALF_WIDTH_M)]
        rel = zone[:, 2] - ego_ground if len(zone) else np.empty(0)
        obstacles = zone[(rel > OBSTACLE_MIN_Z) & (rel < OBSTACLE_MAX_Z)] \
            if len(zone) else zone
        if len(obstacles) >= 5 and np.percentile(obstacles[:, 0], 5) < STOP_DISTANCE_M:
            return "OBSTACLE"
        for lo in np.arange(0.3, CHECK_RANGE_M, 0.25):
            band = zone[(zone[:, 0] >= lo) & (zone[:, 0] < lo + 0.25)] \
                if len(zone) else zone
            ground_band = band[band[:, 2] - ego_ground < OBSTACLE_MIN_Z] \
                if len(band) else band
            if len(ground_band) < 3:
                if lo < 1.1:
                    return "GROUND_GAP"
                continue
            step = np.percentile(ground_band[:, 2], 15) - ego_ground
            if step < -DROP_M:
                return "DROP"
            if step > STEP_M:
                return "STEP"
        return ""

    def spin(self):
        rate = rospy.Rate(GATE_HZ)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            out = Twist()
            reason = ""
            if (now - self.raw_stamp).to_sec() > INPUT_STALE_S:
                reason = "INPUT_STALE"
            elif (now - self.cloud_stamp).to_sec() > CLOUD_STALE_S:
                reason = "CLOUD_STALE"
            else:
                wants_motion = abs(self.raw.linear.x) > 0.02
                if wants_motion:
                    reason = self.forward_blocked()
                if not reason:
                    out.linear.x = max(0.0, min(HARD_V_LIMIT, self.raw.linear.x))
                    out.angular.z = max(-HARD_W_LIMIT,
                                        min(HARD_W_LIMIT, self.raw.angular.z))
            if reason and reason != self.blocked_reason:
                rospy.logwarn("safety gate stop: %s", reason)
            self.blocked_reason = reason
            self.pub.publish(out)
            rate.sleep()


if __name__ == "__main__":
    SafetyGate().spin()
