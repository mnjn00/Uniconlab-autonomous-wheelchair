"""Merge ~1 s of sparse MID360 scans and express them in the lidar frame.

A single 0.1 s sweep leaves the forward corridor nearly empty - the
non-repetitive scan pattern needs accumulating before a corridor check means
anything - so per-scan ground tests false-trigger. Scans are motion
compensated into the newest one's frame via /Odometry.

The merged cloud comes out in the LIDAR frame, not the body frame it arrives
in. FAST-LIO publishes /cloud_registered_body about its IMU, which the VN-100
swap moved 0.145 m forward of the lidar and yawed 2.8 deg from it, while
every geometry constant in both consumers - sensor height, corridor half
width, guard distances - was measured in the lidar/chair frame. Inverting the
extrinsic once here is what keeps those constants meaning what they say.

PointCloud2 is decoded through ``cloud_points.points_xyz``.  The previous
the old per-point generator path allocated one Python tuple per
return in both the follower and the safety gate, consuming whole CPU cores and
making perception/control timestamps stale under load.

Lifted from waypoint_follower and safety_gate, which carried identical copies
of this: same fields, same method bodies, differing only in a variable name
and how the docstring was worded. Two copies of a frame conversion is two
places to correct an extrinsic in.
"""

import numpy as np
import rospy

import tf.transformations as tft
from body_frame import body_to_lidar
from cloud_points import points_xyz


class CloudAccumulator:
    def __init__(self, lidar_in_body, lidar_to_body_rotation, window_s=1.0):
        self.window_s = window_s
        self.lidar_in_body = lidar_in_body
        self.lidar_to_body_rotation = lidar_to_body_rotation
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

    def add_cloud(self, message, read_points=None):
        # Fast zero-copy/structured-array decoder for the FAST-LIO layout,
        # with the old reader retained only as the explicit fallback.
        pts = points_xyz(message, read_points)
        stamp = message.header.stamp.to_sec()
        self.scans.append((stamp, pts))
        self.scans = [s for s in self.scans
                      if stamp - s[0] <= self.window_s + 0.3]

    def merged(self):
        if not self.scans:
            return None, rospy.Time(0)
        newest = self.scans[-1][0]
        T_ref = self.nearest_odom(newest)
        if T_ref is None:
            return None, rospy.Time(0)
        inv_ref = np.linalg.inv(T_ref)
        parts = []
        for stamp, pts in self.scans:
            if newest - stamp > self.window_s or not len(pts):
                continue
            T = self.nearest_odom(stamp)
            if T is None:
                continue
            M = (inv_ref @ T).astype(np.float32)
            parts.append(pts @ M[:3, :3].T + M[:3, 3])
        if not parts:
            return None, rospy.Time(0)
        merged = body_to_lidar(np.vstack(parts), self.lidar_in_body,
                               self.lidar_to_body_rotation)
        return merged, rospy.Time.from_sec(newest)
