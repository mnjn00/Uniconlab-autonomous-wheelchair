#!/usr/bin/env python3
"""Publish tracked obstacle boxes with source-time map coordinates."""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from body_frame import (CHAIR_CENTRE_IN_BODY_XYZ, lidar_extrinsics,
                        lidar_to_body)
from cluster_tracking import UNKNOWN, Tracker
from obstacle_accumulator import Accumulator
from obstacle_cluster_geometry import (
    MapPoseBuffer,
    OBJECT_BAND_GRACE_M,
    OUTSIDE_BAND,
    SENSOR_HEIGHT_M,
    classify,
    cluster_band_relation,
    cluster_grid,
    lateral_profile,
)
from safety_band import SafetyBand
import tf.transformations as tft


PROCESS_HZ = 5.0
WINDOW_S = 0.6
ROI_X = (0.50, 12.0)
ROI_Y = (-6.0, 6.0)
REL_Z = (0.15, 2.4)
FORWARD_FOV_HALF_DEG = 50.0
RIDER_EXCLUDE_X = (-1.0, 0.55)
RIDER_EXCLUDE_Y_HALF = 0.40
RIDER_EXCLUDE_Z = (-SENSOR_HEIGHT_M - 0.1, 1.8)
MAX_CLUSTERS = 40

CLASS_COLORS = {
    "person": (0.9, 0.2, 0.2),
    "vehicle": (0.2, 0.4, 0.9),
    "obstacle": (0.9, 0.7, 0.1),
    OUTSIDE_BAND: (0.45, 0.45, 0.45),
}


class ObstacleClusters:
    def __init__(self):
        rospy.init_node("obstacle_clusters")
        profile = str(rospy.get_param("~body_frame_profile", "vn100"))
        lidar_in_body, lidar_to_body_rotation = lidar_extrinsics(profile)
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        odom_frame = str(rospy.get_param("~odom_frame", "camera_init"))
        body_frame = str(rospy.get_param("~base_frame", "body"))
        cloud_frame = str(rospy.get_param("~cloud_frame", body_frame))
        if not all((self.map_frame, odom_frame, body_frame, cloud_frame)):
            raise rospy.ROSInitException("obstacle frames must be non-empty")
        self.accumulator = Accumulator(
            lidar_in_body, lidar_to_body_rotation,
            odom_frame, body_frame, cloud_frame)
        self.lidar_in_body = lidar_in_body
        self.lidar_to_body_rotation = lidar_to_body_rotation
        self.tracker = Tracker()
        self.band = SafetyBand(rospy.get_param("~safety_band"))
        self.band_grace_m = float(rospy.get_param(
            "~object_band_grace", OBJECT_BAND_GRACE_M))
        if not math.isfinite(self.band_grace_m) or self.band_grace_m < 0.0:
            raise rospy.ROSInitException(
                "~object_band_grace must be a finite non-negative distance")
        self.map_poses = MapPoseBuffer()
        self.map_pose_input_valid = True
        self.marker_pub = rospy.Publisher(
            "/perception/objects", MarkerArray, queue_size=1)
        self.summary_pub = rospy.Publisher(
            "/perception/objects_summary", String, queue_size=1)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.accumulator.add_cloud, queue_size=2)
        rospy.Subscriber("/Odometry", Odometry,
                         self.accumulator.add_odom, queue_size=50)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.add_map_pose, queue_size=20)

    def add_map_pose(self, message):
        with self.map_poses.lock:
            self._add_map_pose(message)

    def _add_map_pose(self, message):
        try:
            position = message.pose.pose.position
            quaternion = message.pose.pose.orientation
            stamp_s = message.header.stamp.to_sec()
            frame_id = str(message.header.frame_id)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._invalidate_map_pose()
            return
        values = (position.x, position.y, position.z, quaternion.x,
                  quaternion.y, quaternion.z, quaternion.w, stamp_s)
        norm = math.sqrt(sum(value * value for value in values[3:7]))
        if not all(math.isfinite(value) for value in values) \
                or stamp_s <= 0.0 or abs(norm - 1.0) > 0.05 \
                or frame_id != getattr(self, "map_frame", "map"):
            self._invalidate_map_pose()
            return
        matrix = tft.quaternion_matrix(values[3:7])
        matrix[:3, 3] = values[:3]
        if not self.map_poses.add(stamp_s, matrix):
            self._invalidate_map_pose()
            return
        self.map_pose_input_valid = True

    def _invalidate_map_pose(self):
        self.map_poses.clear()
        self.map_pose_input_valid = False
        rospy.logwarn_throttle(5.0, "invalid object-band localization pose")

    def track(self, boxes, reference=None):
        if reference is None:
            reference = self.accumulator.reference
        if reference is None or not boxes:
            return []
        stamp_s, transform = reference
        centres = np.array([box[1] for box in boxes], dtype=np.float64)
        body = lidar_to_body(
            centres, self.lidar_in_body, self.lidar_to_body_rotation)
        odom = body @ transform[:3, :3].T + transform[:3, 3]
        return self.tracker.update([
            (float(point[0]), float(point[1]), boxes[index][0])
            for index, point in enumerate(odom)], stamp_s)

    def _clusters(self, merged):
        relative_z = merged[:, 2] + SENSOR_HEIGHT_M
        keep = (merged[:, 0] > ROI_X[0]) & (merged[:, 0] < ROI_X[1]) \
            & (merged[:, 1] > ROI_Y[0]) & (merged[:, 1] < ROI_Y[1]) \
            & (relative_z > REL_Z[0]) & (relative_z < REL_Z[1])
        keep &= np.abs(np.degrees(
            np.arctan2(merged[:, 1], merged[:, 0]))) \
            < FORWARD_FOV_HALF_DEG
        rider = (merged[:, 0] > RIDER_EXCLUDE_X[0]) \
            & (merged[:, 0] < RIDER_EXCLUDE_X[1]) \
            & (np.abs(merged[:, 1] - CHAIR_CENTRE_IN_BODY_XYZ[1])
               < RIDER_EXCLUDE_Y_HALF) \
            & (merged[:, 2] > RIDER_EXCLUDE_Z[0]) \
            & (merged[:, 2] < RIDER_EXCLUDE_Z[1])
        points = merged[keep & ~rider]
        clusters = cluster_grid(points) if len(points) else []
        return sorted(clusters, key=len, reverse=True)[:MAX_CLUSTERS]

    def _boxes(self, clusters, map_pose):
        boxes, contexts = [], []
        for cluster in clusters:
            low, high = cluster.min(axis=0), cluster.max(axis=0)
            raw_label = classify(cluster)
            relation, inside_fraction = cluster_band_relation(
                cluster, map_pose, self.band, self.lidar_in_body,
                self.lidar_to_body_rotation, self.band_grace_m)
            label = OUTSIDE_BAND if relation == "outside" else raw_label
            boxes.append((label, (low + high) / 2.0,
                          np.maximum(high - low, 0.1), len(cluster)))
            contexts.append((raw_label, relation, inside_fraction,
                             lateral_profile(cluster)))
        return boxes, contexts

    def _map_centre(self, centre, map_pose):
        if map_pose is None:
            return None
        body = lidar_to_body(
            np.asarray(centre, dtype=np.float64)[None, :],
            self.lidar_in_body, self.lidar_to_body_rotation)[0]
        return map_pose[:3, :3] @ body + map_pose[:3, 3]

    def _object(self, box, context, track, map_pose, source_stamp_s):
        label, centre, size, point_count = box
        raw_label, relation, inside_fraction, profile = context
        mapped = self._map_centre(centre, map_pose)
        return {
            "class": label,
            "raw_class": raw_label,
            "band_relation": relation,
            "band_inside_fraction": None if inside_fraction is None else
                                    round(inside_fraction, 3),
            "x": round(float(centre[0]), 2),
            "y": round(float(centre[1]), 2),
            "map_x": None if mapped is None else round(float(mapped[0]), 3),
            "map_y": None if mapped is None else round(float(mapped[1]), 3),
            "size": [round(float(value), 2) for value in size],
            "profile": profile,
            "points": int(point_count),
            "id": 0 if track is None else int(track.id),
            "motion": UNKNOWN if track is None else
                      track.motion(source_stamp_s),
            "speed_mps": 0.0 if track is None else
                         round(float(track.speed_mps()), 2),
            "age_s": 0.0 if track is None else
                     round(float(track.age_s(source_stamp_s)), 1),
        }

    def _marker(self, box, index, stamp):
        label, centre, size, _ = box
        marker = Marker()
        marker.header.frame_id = "body"
        marker.header.stamp = stamp
        marker.ns = label
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(*[float(value) for value in centre])
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            *CLASS_COLORS[label], 0.55)
        marker.lifetime = rospy.Duration(3.0 / PROCESS_HZ)
        return marker

    def step(self):
        with self.accumulator.lock, self.map_poses.lock:
            self._step()

    def _step(self):
        merged, reference = (
            self.accumulator.merged(), self.accumulator.reference)
        published_at = rospy.Time.now()
        if merged is None or reference is None:
            self.summary_pub.publish(String(data=json.dumps({
                "stamp": published_at.to_sec(), "status": "NO_CLOUD",
                "objects": []})))
            return
        source_stamp_s = reference[0]
        map_pose = self.map_poses.nearest(source_stamp_s)
        boxes, contexts = self._boxes(self._clusters(merged), map_pose)
        tracks = self.track(boxes, reference)
        markers, objects = MarkerArray(), []
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)
        for index, (box, context) in enumerate(zip(boxes, contexts)):
            track = tracks[index] if tracks else None
            objects.append(self._object(
                box, context, track, map_pose, source_stamp_s))
            markers.markers.append(self._marker(box, index, published_at))
        self.marker_pub.publish(markers)
        status = "OK" if getattr(
            self, "map_pose_input_valid", True) else "NO_MAP_POSE"
        self.summary_pub.publish(String(data=json.dumps({
            "stamp": source_stamp_s,
            "published_at": published_at.to_sec(),
            "status": status,
            "band_status": "OK" if map_pose is not None else "NO_MAP_POSE",
            "frame": "lidar",
            "map_frame": getattr(self, "map_frame", "map"),
            "counts": {
                label: sum(1 for item in objects if item["class"] == label)
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
