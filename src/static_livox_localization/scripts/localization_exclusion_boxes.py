#!/usr/bin/env python3
"""Publish only dynamic/person evidence as localization exclusion boxes.

``hybrid_geometric_objects.py`` keeps mapped surfaces for collision avoidance,
so its complete MarkerArray is remapped away from the localizer. This node
joins those accurate body-frame boxes with the fused semantic summary and
republishes only the subset selected by ``localization_exclusion_policy``.
Learned-only boxes, which have no geometric marker, are transformed from the
route chair frame back to the running FAST-LIO body frame.
"""

import copy
import json
import math
import os
import sys
import threading

import numpy as np
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import lidar_extrinsics
from localization_exclusion_policy import should_exclude


PUBLISH_HZ = 10.0


class LocalizationExclusionBoxes:
    def __init__(self):
        rospy.init_node("localization_exclusion_boxes")
        self.lock = threading.Lock()
        self.candidates = {}
        self.summary = None
        self.summary_receipt_s = 0.0
        follower_profile = str(rospy.get_param(
            "/waypoint_follower/body_frame_profile", "builtin"))
        self.running_profile = str(rospy.get_param(
            "~body_frame_profile", follower_profile))
        running_offset, running_rotation = lidar_extrinsics(
            self.running_profile)
        self.running_lidar_in_body = np.asarray(running_offset, dtype=float)
        self.running_lidar_to_body_rotation = np.asarray(
            running_rotation, dtype=float)
        self.transform_cache = {}

        self.maximum_age_s = float(rospy.get_param("~maximum_age_s", 1.5))
        if not math.isfinite(self.maximum_age_s) or self.maximum_age_s <= 0.0:
            raise rospy.ROSInitException(
                "~maximum_age_s must be finite and positive")

        candidates_topic = str(rospy.get_param(
            "~candidates_topic", "/perception/geometric_exclusion_candidates"))
        summary_topic = str(rospy.get_param(
            "~summary_topic", "/perception/objects_summary"))
        output_topic = str(rospy.get_param(
            "~output_topic", "/perception/dynamic_boxes"))
        self.pub = rospy.Publisher(output_topic, MarkerArray, queue_size=1)
        rospy.Subscriber(candidates_topic, MarkerArray,
                         self.on_candidates, queue_size=2)
        rospy.Subscriber(summary_topic, String,
                         self.on_summary, queue_size=2)
        rospy.loginfo(
            "localization exclusions: candidates=%s summary=%s output=%s "
            "running_profile=%s",
            candidates_topic, summary_topic, output_topic,
            self.running_profile)

    def on_candidates(self, message):
        with self.lock:
            if any(marker.action == Marker.DELETEALL for marker in message.markers):
                self.candidates = {}
            for marker in message.markers:
                if marker.action == Marker.ADD and marker.type == Marker.CUBE:
                    self.candidates[int(marker.id)] = copy.deepcopy(marker)
                elif marker.action == Marker.DELETE:
                    self.candidates.pop(int(marker.id), None)

    def on_summary(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("summary is not an object")
        except (TypeError, ValueError):
            data = None
        with self.lock:
            self.summary = data
            self.summary_receipt_s = rospy.Time.now().to_sec()

    def _route_chair_to_running_body(self, summary, point, size):
        profile = str(summary.get("body_frame_profile", ""))
        centre = summary.get("chair_centre_in_body_xyz")
        if not profile or not isinstance(centre, list) or len(centre) != 3:
            raise ValueError("fused summary has no route frame contract")
        centre = np.asarray([float(value) for value in centre], dtype=float)
        if not np.isfinite(centre).all():
            raise ValueError("route chair centre is not finite")

        key = (profile, tuple(float(value) for value in centre))
        cached = self.transform_cache.get(key)
        if cached is None:
            route_offset, route_rotation = lidar_extrinsics(profile)
            route_offset = np.asarray(route_offset, dtype=float)
            route_rotation = np.asarray(route_rotation, dtype=float)
            route_chair_translation = route_offset - centre
            route_chair_to_running_body_rotation = \
                self.running_lidar_to_body_rotation @ route_rotation.T
            cached = (
                route_chair_translation,
                route_rotation,
                route_chair_to_running_body_rotation,
            )
            self.transform_cache[key] = cached
        route_chair_translation, route_rotation, body_R_route_chair = cached

        point = np.asarray(point, dtype=float)
        size = np.asarray(size, dtype=float)
        point_in_lidar = route_rotation.T @ (
            point - route_chair_translation)
        point_in_running_body = self.running_lidar_to_body_rotation @ \
            point_in_lidar + self.running_lidar_in_body
        size_in_running_body = np.abs(body_R_route_chair) @ size
        return point_in_running_body, size_in_running_body

    def _learned_marker(self, summary, item, stamp, marker_id):
        try:
            raw_size = item["size"]
            point = np.asarray((
                float(item["x"]), float(item["y"]),
                float(item.get("z", 0.0))), dtype=float)
            size = np.asarray((
                abs(float(raw_size[0])), abs(float(raw_size[1])),
                abs(float(raw_size[2]) if len(raw_size) > 2 else 0.1)),
                dtype=float)
            point, size = self._route_chair_to_running_body(
                summary, point, size)
        except (KeyError, IndexError, TypeError, ValueError):
            return None
        if not np.isfinite(point).all() or not np.isfinite(size).all() or \
                (size <= 0.0).any():
            return None

        marker = Marker()
        marker.header.frame_id = "body"
        marker.header.stamp = stamp
        marker.ns = "hybrid_dynamic"
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(size[0])
        marker.scale.y = float(size[1])
        marker.scale.z = float(size[2])
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.8
        marker.color.a = 0.30
        marker.lifetime = rospy.Duration(0.5)
        return marker

    @staticmethod
    def _clear(stamp):
        wipe = Marker()
        wipe.header.frame_id = "body"
        wipe.header.stamp = stamp
        wipe.action = Marker.DELETEALL
        result = MarkerArray()
        result.markers = [wipe]
        return result

    def step(self):
        now = rospy.Time.now()
        now_s = now.to_sec()
        with self.lock:
            summary = copy.deepcopy(self.summary)
            receipt_s = self.summary_receipt_s
            candidates = copy.deepcopy(self.candidates)

        result = self._clear(now)
        if not isinstance(summary, dict) or summary.get("status") != "OK" or \
                summary.get("frame") != "chair_centre":
            self.pub.publish(result)
            return
        try:
            source_stamp_s = float(summary["stamp"])
        except (KeyError, TypeError, ValueError):
            self.pub.publish(result)
            return
        if not math.isfinite(source_stamp_s) or source_stamp_s > now_s + 0.05 or \
                now_s - source_stamp_s > self.maximum_age_s or \
                receipt_s <= 0.0 or now_s - receipt_s > self.maximum_age_s:
            self.pub.publish(result)
            return
        objects = summary.get("objects")
        if not isinstance(objects, list):
            self.pub.publish(result)
            return

        stamp = rospy.Time.from_sec(source_stamp_s)
        used_ids = set()
        next_learned_id = -1000000
        for item in objects:
            if not should_exclude(item):
                continue
            object_id = item.get("id") if isinstance(item, dict) else None
            if isinstance(object_id, int) and not isinstance(object_id, bool) \
                    and object_id in candidates:
                marker = candidates[object_id]
                marker.header.stamp = stamp
                marker.header.frame_id = "body"
                marker.ns = "hybrid_dynamic"
                marker.action = Marker.ADD
                marker.lifetime = rospy.Duration(0.5)
                if marker.id not in used_ids:
                    result.markers.append(marker)
                    used_ids.add(marker.id)
                continue

            marker_id = object_id if isinstance(object_id, int) \
                and not isinstance(object_id, bool) else next_learned_id
            while marker_id in used_ids:
                marker_id -= 1
            next_learned_id = min(next_learned_id, marker_id - 1)
            marker = self._learned_marker(summary, item, stamp, marker_id)
            if marker is not None:
                result.markers.append(marker)
                used_ids.add(marker_id)

        self.pub.publish(result)

    def spin(self):
        rate = rospy.Rate(PUBLISH_HZ)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    try:
        LocalizationExclusionBoxes().spin()
    except rospy.ROSInterruptException:
        pass