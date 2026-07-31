#!/usr/bin/env python3
"""Live obstacle clustering: people / vehicles / other, published for
monitoring and logging.

Observer only - nothing in the motion chain consumes these topics. The
safety_gate keeps its own independent corridor check on the raw cloud,
so a bug here cannot affect stopping behavior; this node exists so the
operator (and the black box) can SEE what the chair was driving past.

Input is /cloud_registered_body (FAST-LIO's motion-undistorted scan in
the body frame), accumulated over a short window because a single 0.1 s
MID360 sweep is too sparse to cluster. Clustering is connected
components over a 2D occupancy grid - O(n) and fully deterministic, no
learned components. Classification is a footprint/height heuristic:
  person   small footprint, 1.1-2.0 m tall
  vehicle  footprint over 1.5 m with a 0.9-2.5 m body
  obstacle everything else that stands above ground

The rider sitting on the wheelchair is excluded by a self-exclusion
box and a forward-only FOV cone so the chair's own occupant is never
reported as an obstacle.

Topics:
  /perception/objects          MarkerArray (RViz boxes, color per class)
  /perception/objects_summary  String, one JSON object per cycle -
                               consumed by the black-box recording
"""

import json
import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import sensor_msgs.point_cloud2 as pc2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from body_frame import (CHAIR_CENTRE_IN_BODY_XYZ, body_to_lidar,
                        lidar_extrinsics, lidar_to_body)
from cluster_tracking import UNKNOWN, Tracker
import tf.transformations as tft

PROCESS_HZ = 5.0
WINDOW_S = 0.6
SENSOR_HEIGHT_M = 0.30
# Forward-only FOV: the rider sits behind and around the lidar, so
# rear/side returns are the rider's body, the wheelchair frame, and
# irrelevant scenery. Clustering is limited to the forward sector the
# chair is actually driving through.
ROI_X = (0.50, 12.0)
ROI_Y = (-6.0, 6.0)
REL_Z = (0.15, 2.4)
FORWARD_FOV_HALF_DEG = 50.0
# Rider self-exclusion box in the lidar frame. The MID360 sees the
# rider's torso, legs, and feet at close range; without this mask the
# rider is the largest "obstacle" in every scan.
# Centred on the rider, not the sensor: the mount is on the left armrest, so
# the rider's body sits CHAIR_CENTRE_IN_BODY_XYZ[1] = -0.173 m from it.
RIDER_EXCLUDE_X = (-1.0, 0.55)
RIDER_EXCLUDE_Y_HALF = 0.40
RIDER_EXCLUDE_Z = (-0.5, 1.8)
# 0.20 m cells: small enough that a person standing 0.3 m from a car
# keeps an empty cell column between them (8-connectivity would bridge
# that gap at 0.25 m), large enough that accumulated scans still fill
# cells at driving-relevant range
CELL_M = 0.20
MIN_CELL_POINTS = 2
MIN_CLUSTER_POINTS = 8
MAX_CLUSTERS = 40

PERSON_MAX_FOOTPRINT_M = 0.9
PERSON_HEIGHT_M = (1.1, 2.0)
VEHICLE_MIN_FOOTPRINT_M = 1.5
VEHICLE_HEIGHT_M = (0.9, 2.5)

CLASS_COLORS = {
    "person": (0.9, 0.2, 0.2),
    "vehicle": (0.2, 0.4, 0.9),
    "obstacle": (0.9, 0.7, 0.1),
}


class Accumulator:
    """Short scan history motion-compensated into the newest body frame."""

    def __init__(self, lidar_in_body, lidar_to_body_rotation):
        self.lidar_in_body = lidar_in_body
        self.lidar_to_body_rotation = lidar_to_body_rotation
        self.scans = []
        self.odoms = []
        # The pose the merged cloud is expressed about, kept because motion
        # can only be judged in a frame that does not move with the chair.
        self.reference = None

    def add_odom(self, message):
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        stamp = message.header.stamp.to_sec()
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not all(math.isfinite(v) for v in
                   (stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or \
                stamp <= 0.0 or abs(norm - 1.0) > 0.05 or \
                (self.odoms and stamp <= self.odoms[-1][0]):
            return
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = (p.x, p.y, p.z)
        self.odoms.append((stamp, T))
        self.odoms = self.odoms[-80:]

    def nearest(self, stamp):
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
        if not math.isfinite(stamp) or stamp <= 0.0 or not len(pts) or \
                (self.scans and stamp <= self.scans[-1][0]):
            return
        self.scans.append((stamp, pts))
        self.scans = [s for s in self.scans
                      if stamp - s[0] <= WINDOW_S + 0.3]

    def merged(self):
        self.reference = None
        if not self.scans:
            return None
        newest = self.scans[-1][0]
        T_ref = self.nearest(newest)
        if T_ref is None:
            return None
        self.reference = (newest, T_ref)
        inv_ref = np.linalg.inv(T_ref)
        parts = []
        for stamp, pts in self.scans:
            if newest - stamp > WINDOW_S:
                continue
            T = self.nearest(stamp)
            if T is None:
                continue
            M = (inv_ref @ T).astype(np.float32)
            parts.append(pts @ M[:3, :3].T + M[:3, 3])
        if not parts:
            return None
        return body_to_lidar(np.vstack(parts), self.lidar_in_body,
                             self.lidar_to_body_rotation)


def cluster_grid(points):
    """Connected components (8-neighbour) over a 2D cell grid."""
    cells = np.floor(points[:, :2] / CELL_M).astype(np.int64)
    order = np.lexsort((cells[:, 1], cells[:, 0]))
    cells, points = cells[order], points[order]
    keys, starts, counts = np.unique(
        cells, axis=0, return_index=True, return_counts=True)
    occupied = {tuple(k): i for i, k in enumerate(keys)
                if counts[i] >= MIN_CELL_POINTS}
    labels = {}
    clusters = []
    for cell in occupied:
        if cell in labels:
            continue
        member_cells, stack = [], [cell]
        labels[cell] = len(clusters)
        while stack:
            c = stack.pop()
            member_cells.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in occupied and n not in labels:
                        labels[n] = len(clusters)
                        stack.append(n)
        idx = np.concatenate([
            np.arange(starts[occupied[c]],
                      starts[occupied[c]] + counts[occupied[c]])
            for c in member_cells])
        if len(idx) >= MIN_CLUSTER_POINTS:
            clusters.append(points[idx])
    return clusters


def classify(cluster):
    rel = cluster[:, 2] + SENSOR_HEIGHT_M
    height = float(rel.max())
    span = cluster[:, :2].max(axis=0) - cluster[:, :2].min(axis=0)
    footprint = float(np.hypot(span[0], span[1]))
    if footprint <= PERSON_MAX_FOOTPRINT_M and \
            PERSON_HEIGHT_M[0] <= height <= PERSON_HEIGHT_M[1]:
        return "person"
    if footprint >= VEHICLE_MIN_FOOTPRINT_M and \
            VEHICLE_HEIGHT_M[0] <= height <= VEHICLE_HEIGHT_M[1]:
        return "vehicle"
    return "obstacle"


class ObstacleClusters:
    def __init__(self):
        rospy.init_node("obstacle_clusters")
        profile = str(rospy.get_param("~body_frame_profile", "vn100"))
        lidar_in_body, lidar_to_body_rotation = lidar_extrinsics(profile)
        self.accumulator = Accumulator(lidar_in_body, lidar_to_body_rotation)
        self.lidar_in_body = lidar_in_body
        self.lidar_to_body_rotation = lidar_to_body_rotation
        self.tracker = Tracker()
        self.marker_pub = rospy.Publisher(
            "/perception/objects", MarkerArray, queue_size=1)
        self.summary_pub = rospy.Publisher(
            "/perception/objects_summary", String, queue_size=1)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.accumulator.add_cloud, queue_size=2)
        rospy.Subscriber("/Odometry", Odometry,
                         self.accumulator.add_odom, queue_size=50)

    def track(self, boxes):
        """Follow each box in the odom frame and return its Track, or [].

        Motion is only a question in a frame that does not move with the
        chair, so this needs the pose the merged cloud was expressed about.
        Without one there is nothing to say, and the caller reports saying
        nothing as UNKNOWN rather than as standing still.
        """
        reference = self.accumulator.reference
        if reference is None or not boxes:
            return []
        stamp_s, T_ref = reference
        centres = np.array([box[1] for box in boxes], dtype=np.float64)
        in_body = lidar_to_body(centres, self.lidar_in_body,
                                self.lidar_to_body_rotation)
        in_odom = in_body @ T_ref[:3, :3].T + T_ref[:3, 3]
        return self.tracker.update(
            [(float(point[0]), float(point[1]), boxes[i][0])
             for i, point in enumerate(in_odom)], stamp_s)

    def step(self):
        merged = self.accumulator.merged()
        stamp = rospy.Time.now()
        if merged is None:
            self.summary_pub.publish(String(data=json.dumps(
                {"stamp": stamp.to_sec(), "status": "NO_CLOUD",
                 "objects": []})))
            return
        rel = merged[:, 2] + SENSOR_HEIGHT_M
        keep = (merged[:, 0] > ROI_X[0]) & (merged[:, 0] < ROI_X[1]) & \
               (merged[:, 1] > ROI_Y[0]) & (merged[:, 1] < ROI_Y[1]) & \
               (rel > REL_Z[0]) & (rel < REL_Z[1])
        # forward FOV cone
        azimuth = np.abs(np.degrees(np.arctan2(merged[:, 1], merged[:, 0])))
        keep &= azimuth < FORWARD_FOV_HALF_DEG
        # rider self-exclusion
        rider = (merged[:, 0] > RIDER_EXCLUDE_X[0]) & \
                (merged[:, 0] < RIDER_EXCLUDE_X[1]) & \
                (np.abs(merged[:, 1] - CHAIR_CENTRE_IN_BODY_XYZ[1])
                 < RIDER_EXCLUDE_Y_HALF) & \
                (merged[:, 2] > RIDER_EXCLUDE_Z[0]) & \
                (merged[:, 2] < RIDER_EXCLUDE_Z[1])
        keep &= ~rider
        points = merged[keep]
        clusters = cluster_grid(points) if len(points) else []
        clusters = sorted(clusters, key=len, reverse=True)[:MAX_CLUSTERS]

        boxes = []
        for cluster in clusters:
            lo = cluster.min(axis=0)
            hi = cluster.max(axis=0)
            boxes.append((classify(cluster), (lo + hi) / 2.0,
                          np.maximum(hi - lo, 0.1), len(cluster)))
        tracks = self.track(boxes)

        markers, objects = MarkerArray(), []
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)
        for i, (label, center, size, points) in enumerate(boxes):
            track = tracks[i] if tracks else None
            objects.append({
                "class": label,
                "x": round(float(center[0]), 2),
                "y": round(float(center[1]), 2),
                "size": [round(float(v), 2) for v in size],
                "points": int(points),
                # A consumer that steers around what this says is parked
                # needs to know when nothing said it. Without a reference
                # pose there is no frame to judge motion in, and the honest
                # answer is UNKNOWN, which every consumer handles as moving.
                "id": 0 if track is None else int(track.id),
                "motion": UNKNOWN if track is None else
                          track.motion(stamp.to_sec()),
                "speed_mps": 0.0 if track is None else
                             round(float(track.speed_mps()), 2),
                "age_s": 0.0 if track is None else
                         round(float(track.age_s(stamp.to_sec())), 1),
            })
            m = Marker()
            m.header.frame_id = "body"
            m.header.stamp = stamp
            m.ns = label
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(*[float(v) for v in center])
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in size)
            r, g, b = CLASS_COLORS[label]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.55
            m.lifetime = rospy.Duration(1.0 / PROCESS_HZ * 3.0)
            markers.markers.append(m)
        self.marker_pub.publish(markers)
        self.summary_pub.publish(String(data=json.dumps({
            "stamp": stamp.to_sec(),
            "status": "OK",
            # Stated because a consumer now steers by these numbers. They
            # are chair-aligned lidar-frame, the same frame every clearance
            # constant in the follower is written in - NOT the "body" the
            # markers above are drawn in, which is the IMU frame and sits
            # 0.14 m forward of it.
            "frame": "lidar",
            "counts": {
                label: sum(1 for o in objects if o["class"] == label)
                for label in CLASS_COLORS},
            "objects": objects,
        })))

    def spin(self):
        rate = rospy.Rate(PROCESS_HZ)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    ObstacleClusters().spin()