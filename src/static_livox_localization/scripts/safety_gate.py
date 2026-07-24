#!/usr/bin/env python3
"""Independent obstacle safety gate between the planner and tip_guard.

Deliberately knows nothing about routes or planning: it forwards
/cmd_vel_raw to /cmd_vel_gated only when its OWN forward-corridor check
passes, clamps speeds, replaces stale or missing input with a stop, and
publishes continuously so the chain always has a live command stream.
tip_guard.py is the final stage after this (guards against tip-over
independently of obstacles); wheel_cmd_tmp.py/uart.py consume its output
on /cmd_vel. If the planner misbehaves or dies, this gate stops the chair;
if this gate dies, tip_guard's own staleness check stops the chair; if
that dies too, the uart-level watchdog stops the chair.
"""

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft


class CloudAccumulator:
    """Merge ~1 s of sparse MID360 scans into the current body frame.

    A single 0.1 s sweep leaves the forward corridor nearly empty (the
    non-repetitive pattern needs accumulation), so per-scan ground checks
    false-trigger. Scans are motion-compensated via /Odometry.
    """

    def __init__(self, window_s=0.6):
        self.window_s = window_s
        self.scans = []
        self.odoms = []

    def add_odom(self, message):
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = (p.x, p.y, p.z)
        self.odoms.append((message.header.stamp.to_sec(), T))
        self.odoms = self.odoms[-60:]

    def nearest_odom(self, stamp):
        if not self.odoms:
            return None
        times = np.array([t for t, _ in self.odoms])
        k = int(np.argmin(np.abs(times - stamp)))
        if abs(times[k] - stamp) > 0.15:
            return None
        return self.odoms[k][1]

    def add_cloud(self, message):
        pts = np.array(list(pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32)
        stamp = message.header.stamp.to_sec()
        self.scans.append((stamp, pts))
        self.scans = [s for s in self.scans if stamp - s[0] <= self.window_s + 0.3]

    def merged(self):
        if not self.scans:
            return None, rospy.Time(0)
        newest_stamp = self.scans[-1][0]
        T_ref = self.nearest_odom(newest_stamp)
        if T_ref is None:
            return None, rospy.Time(0)
        inv_ref = np.linalg.inv(T_ref)
        parts = []
        for stamp, pts in self.scans:
            if newest_stamp - stamp > self.window_s or not len(pts):
                continue
            T = self.nearest_odom(stamp)
            if T is None:
                continue
            M = (inv_ref @ T).astype(np.float32)
            parts.append(pts @ M[:3, :3].T + M[:3, 3])
        if not parts:
            return None, rospy.Time(0)
        return np.vstack(parts), rospy.Time.from_sec(newest_stamp)


GATE_HZ = 15.0
INPUT_STALE_S = 0.6
CLOUD_STALE_S = 1.0
HARD_V_LIMIT = 1.1
HARD_W_LIMIT = 0.6
STOP_DISTANCE_MIN_M = 0.8
STOP_DISTANCE_PER_MPS = 1.5
CHECK_RANGE_M = 3.0
HALF_WIDTH_M = 0.5
SENSOR_HEIGHT_M = 0.30
OBSTACLE_MIN_Z = 0.15
OBSTACLE_MAX_Z = 1.9


class SafetyGate:
    def __init__(self):
        rospy.init_node("safety_gate")
        self.raw = Twist()
        self.raw_stamp = rospy.Time(0)
        self.accumulator = CloudAccumulator()
        self.cloud = None
        self.cloud_stamp = rospy.Time(0)
        self.blocked_reason = ""
        self.pub = rospy.Publisher("/cmd_vel_gated", Twist, queue_size=1)
        rospy.Subscriber("/cmd_vel_raw", Twist, self.on_raw, queue_size=1)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.on_cloud, queue_size=2)
        rospy.Subscriber("/Odometry", Odometry,
                         self.on_odom, queue_size=50)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    def on_raw(self, message):
        self.raw = message
        self.raw_stamp = rospy.Time.now()

    def on_cloud(self, message):
        self.accumulator.add_cloud(message)
        self.cloud, self.cloud_stamp = self.accumulator.merged()

    def on_odom(self, message):
        self.accumulator.add_odom(message)

    def forward_blocked(self):
        """Obstacle-only check: the MID360 cannot see near ground (vertical
        FOV -7 deg, low mount), so drop protection is the follower's
        map-band containment; this gate independently blocks visible
        obstacles and stale sensing."""
        if self.cloud is None or len(self.cloud) < 100:
            return "NO_CLOUD"
        pts = self.cloud
        ground_plane = -SENSOR_HEIGHT_M
        zone = pts[(pts[:, 0] > 0.25) & (pts[:, 0] < CHECK_RANGE_M) &
                   (np.abs(pts[:, 1]) < HALF_WIDTH_M)]
        if not len(zone):
            return ""
        rel = zone[:, 2] - ground_plane
        obstacles = zone[(rel > OBSTACLE_MIN_Z) & (rel < OBSTACLE_MAX_Z)]
        stop_distance = max(STOP_DISTANCE_MIN_M,
                            STOP_DISTANCE_PER_MPS * abs(self.raw.linear.x))
        if len(obstacles) >= 5 and \
                np.percentile(obstacles[:, 0], 5) < stop_distance:
            return "OBSTACLE"
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
