#!/usr/bin/env python3
"""Live obstacle clustering: people / vehicles / other, published for
monitoring, logging and tracked-cluster avoidance.

The hand-drawn route corridor is consulted after clustering. A cluster whose
sampled returns all sit outside the effective safety band is kept and tracked,
but is labelled ``outside_band`` rather than guessed to be a vehicle from its
length. Keeping it is deliberate: a person or car beside the corridor may move
into it, and the motion guard must continue to see that box. If a synchronized
map pose is unavailable, semantic classification is left untouched rather than
using a stale transform to hide an object.

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
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import sensor_msgs.point_cloud2 as pc2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from body_frame import (CHAIR_CENTRE_IN_BODY_XYZ, body_to_lidar,
                        lidar_extrinsics, lidar_to_body)
from cloud_points import (COLLISION_MAX_HEIGHT_M,
                          COLLISION_MIN_HEIGHT_M, points_xyzi)
from cluster_tracking import MOVING, STATIC, UNKNOWN, Tracker
from fixed_map_filter import FixedMapFilter
from safety_band import SafetyBand
import tf.transformations as tft

PROCESS_HZ = 5.0
WINDOW_S = 0.6
CLOUD_STALE_S = 1.0
MAP_MATCH_TOLERANCE_M = 0.15
SENSOR_HEIGHT_M = 0.725
# Forward-only FOV: the rider sits behind and around the lidar, so
# rear/side returns are the rider's body, the wheelchair frame, and
# irrelevant scenery. Clustering is limited to the forward sector the
# chair is actually driving through.
ROI_X = (0.50, 12.0)
ROI_Y = (-6.0, 6.0)
FORWARD_FOV_HALF_DEG = 50.0
# Retroreflector blooming: traffic signs and reflective surfaces saturate
# the Livox detector, producing a halo of points around the real (thin) sign
# that reads as a solid obstacle.  Livox reflectivity is 0-255; anything at
# or near 255 is a retroreflector.  Dropping these points removes the halo
# while leaving the thin pole the sign sits on — its metal has ordinary
# reflectivity and survives the cut.  See:
#   https://www.robosense.ai/en/tech-show-55
RETROREFLECTOR_INTENSITY = 200.0
# Rider self-exclusion box in the lidar frame. The MID360 sees the
# rider's torso, legs, and feet at close range; without this mask the
# rider is the largest "obstacle" in every scan.
# Centred on the rider, not the sensor: the mount is on the left armrest, so
# the rider's body sits CHAIR_CENTRE_IN_BODY_XYZ[1] = -0.173 m from it.
RIDER_EXCLUDE_X = (-1.0, 0.55)
RIDER_EXCLUDE_Y_HALF = 0.40
# Raw lidar z, not height above ground, so this moved when the mount height
# was corrected. The lower bound has to sit BELOW the ground plane or the
# rider's feet and the footrest fall outside the box and get clustered as an
# obstacle riding along in front of the chair. At the old 0.30 m mount, -0.5
# was 0.2 m under the ground; at the measured 0.725 m it was 0.225 m ABOVE
# it, which leaves everything below the rider's shins exposed.
RIDER_EXCLUDE_Z = (-SENSOR_HEIGHT_M - 0.1, 1.8)
# 0.20 m cells: small enough that a person standing 0.3 m from a car
# keeps an empty cell column between them (8-connectivity would bridge
# that gap at 0.25 m), large enough that accumulated scans still fill
# cells at driving-relevant range
CELL_M = 0.20
MIN_CELL_POINTS = 2
MIN_CLUSTER_POINTS = 8
MAX_CLUSTERS = 40

# The classifier and the band live in different frames. A localization pose
# this far from the newest accumulated scan is not evidence about where that
# cluster sits on the map; in that case the original class wins.
MAP_POSE_MAX_DELTA_S = 0.30
# Classification tolerates the same order of localization/corridor error as
# the follower's containment check. This is a semantic relabel only, never a
# reason to delete a collision box.
OBJECT_BAND_GRACE_M = 0.10
MAX_BAND_SAMPLE_POINTS = 96
OUTSIDE_MAX_INSIDE_FRACTION = 0.05
INSIDE_MIN_INSIDE_FRACTION = 0.95
OUTSIDE_BAND = "outside_band"

# Lateral slices of a cluster's OWN returns, so a consumer can ask how far
# this object actually is inside a corridor of its choosing. A box cannot
# answer that. An axis-aligned box around a wall running diagonally across
# the scan reports a near face at a corner where the wall has no returns at
# all: on 2026-07-31 run 1 the box put a wall 0.69 m dead ahead when its
# nearest return inside the corridor was 2.13 m, and the follower held for
# 16 minutes because every candidate bypass lane was inside the same box.
# 0.2 m matches the clustering cell, so a slice is never finer than the
# evidence that built it.
PROFILE_BIN_M = 0.2
# A cluster spanning the whole ROI would otherwise publish hundreds of
# numbers five times a second. Past this the slices are widened instead,
# which costs resolution and stays conservative.
MAX_PROFILE_BINS = 64

PERSON_MAX_FOOTPRINT_M = 0.9
PERSON_HEIGHT_M = (1.1, 2.0)
VEHICLE_MIN_FOOTPRINT_M = 1.5
VEHICLE_HEIGHT_M = (0.9, 2.5)

CLASS_COLORS = {
    "person": (0.9, 0.2, 0.2),
    "vehicle": (0.2, 0.4, 0.9),
    "obstacle": (0.9, 0.7, 0.1),
    OUTSIDE_BAND: (0.45, 0.45, 0.45),
}


class MapPoseBuffer:
    """Small timestamped history of map_T_body localization poses."""

    def __init__(self):
        self.poses = []

    def add(self, stamp_s, matrix):
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            return
        matrix = np.asarray(matrix, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            return
        if self.poses and stamp_s <= self.poses[-1][0]:
            return
        self.poses.append((float(stamp_s), matrix))
        self.poses = self.poses[-80:]

    def nearest(self, stamp_s, max_delta_s=MAP_POSE_MAX_DELTA_S):
        if not self.poses or not math.isfinite(stamp_s):
            return None
        times = np.array([t for t, _ in self.poses])
        k = int(np.argmin(np.abs(times - stamp_s)))
        if abs(times[k] - stamp_s) > max_delta_s:
            return None
        return self.poses[k][1]


def cluster_band_relation(cluster, map_T_body, band, lidar_in_body,
                          lidar_to_body_rotation, grace_m):
    """Return (relation, inside_fraction) for one lidar-frame cluster.

    ``outside`` means the sampled returns are outside the band; it does not
    claim they are a wall. The distinction matters on this route because the
    hand drawing also excludes road traffic and open forecourt that the chair
    should not enter.
    """
    if map_T_body is None or band is None:
        return "unavailable", None
    count = min(len(cluster), MAX_BAND_SAMPLE_POINTS)
    if not count:
        return "unavailable", None
    indexes = np.linspace(0, len(cluster) - 1, count, dtype=int)
    sampled = np.asarray(cluster[indexes], dtype=np.float64)
    in_body = lidar_to_body(
        sampled, lidar_in_body, lidar_to_body_rotation)
    in_map = in_body @ map_T_body[:3, :3].T + map_T_body[:3, 3]
    inside = band.contains_many(in_map[:, :2], grace=grace_m)
    fraction = float(np.mean(inside))
    if fraction <= OUTSIDE_MAX_INSIDE_FRACTION:
        return "outside", fraction
    if fraction >= INSIDE_MIN_INSIDE_FRACTION:
        return "inside", fraction
    return "crossing", fraction


def lateral_profile(cluster, bin_m=PROFILE_BIN_M, max_bins=MAX_PROFILE_BINS):
    """Nearest forward return in each lateral slice of one cluster.

    ``{"bin_m", "y0", "min_x"}`` where ``min_x[k]`` is the closest return
    whose y falls in ``[y0 + k*bin_m, y0 + (k+1)*bin_m)``, or None where
    this cluster has no return in that slice.

    An empty slice is not a claim that the ground there is free - only that
    THIS object is not in it. Every other cluster is profiled separately and
    the consumer takes the nearest across all of them, so the guard still
    sees anything that is really there.

    Chair-aligned lidar frame, the same frame as x/y/size beside it.
    """
    points = np.asarray(cluster, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] < 2:
        return None
    finite = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1])
    if not finite.any():
        return None
    x, y = points[finite, 0], points[finite, 1]
    span = float(y.max()) - float(y.min())
    while bin_m > 0.0 and (span / bin_m) + 1.0 > max_bins:
        bin_m *= 2.0
    first = int(math.floor(float(y.min()) / bin_m))
    count = int(math.floor(float(y.max()) / bin_m)) - first + 1
    index = np.clip(np.floor(y / bin_m).astype(int) - first, 0, count - 1)
    nearest = np.full(count, np.inf)
    np.minimum.at(nearest, index, x)
    return {
        "bin_m": round(float(bin_m), 3),
        "y0": round(float(first * bin_m), 3),
        "min_x": [None if not math.isfinite(v) else round(float(v), 2)
                  for v in nearest],
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
        self.lock = threading.Lock()

    def add_odom(self, message):
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        stamp = message.header.stamp.to_sec()
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not all(math.isfinite(v) for v in
                   (stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or \
                stamp <= 0.0 or abs(norm - 1.0) > 0.05:
            return
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = (p.x, p.y, p.z)
        with self.lock:
            if self.odoms and stamp <= self.odoms[-1][0]:
                return
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
        pts = points_xyzi(message)
        stamp = message.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0.0 or not len(pts):
            return
        with self.lock:
            if self.scans and stamp <= self.scans[-1][0]:
                return
            self.scans.append((stamp, pts))
            self.scans = [s for s in self.scans
                          if stamp - s[0] <= WINDOW_S + 0.3]

    def newest_stamp(self):
        with self.lock:
            return self.scans[-1][0] if self.scans else None

    def merged(self):
        self.reference = None
        with self.lock:
            scans = list(self.scans)
            odoms = list(self.odoms)
        if not scans:
            return None
        newest = scans[-1][0]

        def nearest(stamp):
            if not odoms:
                return None
            times = np.array([t for t, _ in odoms])
            k = int(np.argmin(np.abs(times - stamp)))
            return odoms[k][1] if abs(times[k] - stamp) <= 0.15 else None

        T_ref = nearest(newest)
        if T_ref is None:
            return None
        self.reference = (newest, T_ref)
        inv_ref = np.linalg.inv(T_ref)
        parts = []
        for stamp, pts in scans:
            if newest - stamp > WINDOW_S:
                continue
            T = nearest(stamp)
            if T is None:
                continue
            M = (inv_ref @ T).astype(np.float32)
            # Motion-compensate xyz (columns 0-2); carry intensity
            # (column 3) through unchanged.
            xyz = pts[:, :3] @ M[:3, :3].T + M[:3, 3]
            if pts.shape[1] >= 4:
                parts.append(np.concatenate(
                    [xyz, pts[:, 3:4]], axis=1))
            else:
                parts.append(np.concatenate(
                    [xyz, np.zeros((len(xyz), 1), dtype=np.float32)],
                    axis=1))
        if not parts:
            return None
        merged = np.vstack(parts)
        # body_to_lidar transforms xyz; intensity is frame-independent.
        xyz_lidar = body_to_lidar(merged[:, :3], self.lidar_in_body,
                                  self.lidar_to_body_rotation)
        return np.concatenate(
            [xyz_lidar, merged[:, 3:4].astype(np.float32, copy=False)],
            axis=1)


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
        self.fixed_map_filter = FixedMapFilter(
            rospy.get_param("~map_path"),
            rospy.get_param("~map_sha256"),
            rospy.get_param("~map_match_tolerance",
                            MAP_MATCH_TOLERANCE_M))
        self.band = SafetyBand(rospy.get_param("~safety_band"))
        self.band_grace_m = float(rospy.get_param(
            "~object_band_grace", OBJECT_BAND_GRACE_M))
        if not math.isfinite(self.band_grace_m) or self.band_grace_m < 0.0:
            raise rospy.ROSInitException(
                "~object_band_grace must be a finite non-negative distance")
        self.map_poses = MapPoseBuffer()
        self._last_processed_stamp = None
        self._last_bloom_removed = 0
        self.marker_pub = rospy.Publisher(
            "/perception/objects", MarkerArray, queue_size=1)
        # Boxes the localizer must not register against. Separate from the
        # RViz markers above: those are for a person to look at and carry
        # every class, these are consumed by moving_icp_localizer and carry
        # only what is confirmed moving. Body frame, because that is the
        # frame the rolling submap accumulates in.
        self.dynamic_pub = rospy.Publisher(
            "/perception/dynamic_boxes", MarkerArray, queue_size=1)
        self.summary_pub = rospy.Publisher(
            "/perception/objects_summary", String, queue_size=1)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.accumulator.add_cloud, queue_size=2)
        rospy.Subscriber("/Odometry", Odometry,
                         self.accumulator.add_odom, queue_size=50)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.add_map_pose, queue_size=20)

    def add_map_pose(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        stamp_s = message.header.stamp.to_sec()
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not all(math.isfinite(v) for v in values) or abs(norm - 1.0) > 0.05:
            rospy.logwarn_throttle(
                5.0, "object band ignored an invalid localization pose")
            return
        matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[:3, 3] = (p.x, p.y, p.z)
        self.map_poses.add(stamp_s, matrix)

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

    def publish_empty(self, source_stamp, status):
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        clear = MarkerArray()
        clear.markers = [delete_all]
        self.marker_pub.publish(clear)
        self.dynamic_pub.publish(clear)
        self.summary_pub.publish(String(data=json.dumps(
            {"stamp": source_stamp, "status": status, "objects": []}
        )))

    def step(self):
        stamp = rospy.Time.now()
        newest_stamp = self.accumulator.newest_stamp()
        if newest_stamp is None or stamp.to_sec() - newest_stamp > CLOUD_STALE_S:
            source_stamp = newest_stamp if newest_stamp is not None else 0.0
            self.publish_empty(source_stamp, "NO_CLOUD")
            return
        if self._last_processed_stamp is not None and \
                newest_stamp <= self._last_processed_stamp:
            return
        merged = self.accumulator.merged()
        if merged is None:
            self.publish_empty(newest_stamp, "NO_CLOUD")
            return
        newest_stamp = self.accumulator.reference[0]
        self._last_processed_stamp = newest_stamp
        rel = merged[:, 2] + SENSOR_HEIGHT_M
        keep = (merged[:, 0] > ROI_X[0]) & (merged[:, 0] < ROI_X[1]) & \
               (merged[:, 1] > ROI_Y[0]) & (merged[:, 1] < ROI_Y[1]) & \
               (rel >= COLLISION_MIN_HEIGHT_M) & \
               (rel <= COLLISION_MAX_HEIGHT_M)
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
        # retroreflector blooming: drop points whose intensity is at or
        # near detector saturation.  These are traffic signs, reflective
        # stickers, and mirror surfaces — all of which produce a point-cloud
        # halo larger than the physical object.  The sign's thin metal pole
        # has ordinary reflectivity and survives.
        bloom_removed = 0
        if merged.shape[1] >= 4:
            bloom = merged[:, 3] >= RETROREFLECTOR_INTENSITY
            bloom_removed = int(np.count_nonzero(bloom & keep))
            keep &= ~bloom
        points = merged[keep][:, :3]
        # Observability: report how many points the intensity filter took,
        # so a field run that stops for nothing can be distinguished from
        # one that stopped for a ghost the filter missed.
        self._last_bloom_removed = bloom_removed
        cloud_stamp = None if self.accumulator.reference is None else \
            self.accumulator.reference[0]
        map_pose = None if cloud_stamp is None else \
            self.map_poses.nearest(cloud_stamp)
        if map_pose is None:
            self.publish_empty(newest_stamp, "NO_MAP_POSE")
            return
        body_T_lidar = np.eye(4)
        body_T_lidar[:3, :3] = np.asarray(
            self.lidar_to_body_rotation, dtype=np.float64)
        body_T_lidar[:3, 3] = np.asarray(
            self.lidar_in_body, dtype=np.float64)
        points = self.fixed_map_filter.retain_novel(
            points, map_pose @ body_T_lidar)
        clusters = cluster_grid(points) if len(points) else []
        clusters = sorted(clusters, key=len, reverse=True)[:MAX_CLUSTERS]
        band_status = "OK"

        boxes, band_context = [], []
        for cluster in clusters:
            lo = cluster.min(axis=0)
            hi = cluster.max(axis=0)
            raw_label = classify(cluster)
            relation, inside_fraction = cluster_band_relation(
                cluster, map_pose, self.band, self.lidar_in_body,
                self.lidar_to_body_rotation, self.band_grace_m)
            label = OUTSIDE_BAND if relation == "outside" else raw_label
            boxes.append((label, (lo + hi) / 2.0,
                          np.maximum(hi - lo, 0.1), len(cluster)))
            band_context.append((raw_label, relation, inside_fraction,
                                 lateral_profile(cluster)))
        tracks = self.track(boxes)

        markers, objects = MarkerArray(), []
        dynamic = MarkerArray()
        wipe = Marker()
        wipe.header.frame_id = "body"
        wipe.header.stamp = rospy.Time.from_sec(newest_stamp)
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)
        dynamic.markers.append(wipe)
        for i, ((label, center, size, points), context) in enumerate(
                zip(boxes, band_context)):
            body_center = lidar_to_body(
                np.asarray([center], dtype=np.float64),
                self.lidar_in_body, self.lidar_to_body_rotation)[0]
            body_size = (
                np.abs(np.asarray(self.lidar_to_body_rotation)) @ size)
            raw_label, band_relation, inside_fraction, profile = context
            track = tracks[i] if tracks else None
            object_id = i if track is None else int(track.id)
            objects.append({
                "class": label,
                "raw_class": raw_label,
                "band_relation": band_relation,
                "band_inside_fraction": None if inside_fraction is None else
                                        round(inside_fraction, 3),
                "x": round(float(center[0]), 2),
                "y": round(float(center[1]), 2),
                "size": [round(float(v), 2) for v in size],
                # Where this object's returns ACTUALLY are, slice by slice.
                # The box above is kept for the markers and for a consumer
                # that predates this, but the guard measures distance from
                # here - see cluster_guard.corridor_reach.
                "profile": profile,
                "points": int(points),
                # A consumer that steers around what this says is parked
                # needs to know when nothing said it. Without a reference
                # pose there is no frame to judge motion in, and the honest
                # answer is UNKNOWN, which every consumer handles as moving.
                "id": object_id,
                "motion": UNKNOWN if track is None else
                          track.motion(stamp.to_sec()),
                "speed_mps": 0.0 if track is None else
                             round(float(track.speed_mps()), 2),
                "age_s": 0.0 if track is None else
                         round(float(track.age_s(stamp.to_sec())), 1),
            })
            # Every map-novel object is excluded from registration. Motion
            # only controls avoidance policy; a stationary person or parked
            # vehicle is still absent from the immutable localization map.
            box = Marker()
            box.header.stamp = rospy.Time.from_sec(newest_stamp)
            box.header.frame_id = "body"
            box.ns = "dynamic"
            box.id = object_id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(body_center[0])
            box.pose.position.y = float(body_center[1])
            box.pose.position.z = float(body_center[2])
            box.pose.orientation.w = 1.0
            box.scale.x = float(body_size[0])
            box.scale.y = float(body_size[1])
            box.scale.z = float(body_size[2])
            box.color.r, box.color.a = 1.0, 0.3
            dynamic.markers.append(box)
            m = Marker()
            m.header.frame_id = "body"
            m.header.stamp = rospy.Time.from_sec(newest_stamp)
            m.ns = label
            m.id = object_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(*[float(v) for v in body_center])
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (
                float(v) for v in body_size)
            r, g, b = CLASS_COLORS[label]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.55
            m.lifetime = rospy.Duration(1.0 / PROCESS_HZ * 3.0)
            markers.markers.append(m)
        self.marker_pub.publish(markers)
        self.dynamic_pub.publish(dynamic)
        self.summary_pub.publish(String(data=json.dumps({
            "stamp": newest_stamp,
            "status": "OK",
            "band_status": band_status,
            # Stated because a consumer now steers by these numbers. They
            # are chair-aligned lidar-frame, the same frame every clearance
            # constant in the follower is written in - NOT the "body" the
            # markers above are drawn in, which is the IMU frame and sits
            # 0.14 m forward of it.
            "frame": "lidar",
            "bloom_filtered": self._last_bloom_removed,
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
