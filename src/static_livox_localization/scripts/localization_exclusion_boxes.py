#!/usr/bin/env python3
"""Publish only dynamic/person evidence as localization exclusion boxes.

``hybrid_geometric_objects.py`` keeps mapped surfaces for collision avoidance,
so its complete MarkerArray is remapped away from the localizer. This node
joins those accurate body-frame boxes with the fused semantic summary and
republishes only the subset selected by ``localization_exclusion_policy``.
Learned-only boxes, which have no geometric marker, are reconstructed from the
chair-centred fused box.
"""

import copy
import json
import math
import os
import sys
import threading

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import CHAIR_CENTRE_IN_BODY_XYZ
from localization_exclusion_policy import should_exclude


PUBLISH_HZ = 10.0


class LocalizationExclusionBoxes:
    def __init__(self):
        rospy.init_node("localization_exclusion_boxes")
        self.lock = threading.Lock()
        self.candidates = {}
        self.summary = None
        self.summary_receipt_s = 0.0
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
            "localization exclusions: candidates=%s summary=%s output=%s",
            candidates_topic, summary_topic, output_topic)

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

    @staticmethod
    def _learned_marker(item, stamp, marker_id):
        try:
            size = item["size"]
            x = float(item["x"])
            y = float(item["y"])
            z = float(item.get("z", 0.0))
            sx = abs(float(size[0]))
            sy = abs(float(size[1]))
            sz = abs(float(size[2]) if len(size) > 2 else 0.1)
        except (KeyError, IndexError, TypeError, ValueError):
            return None
        values = (x, y, z, sx, sy, sz)
        if not all(math.isfinite(value) for value in values) or \
                min(sx, sy, sz) <= 0.0:
            return None

        marker = Marker()
        marker.header.frame_id = "body"
        marker.header.stamp = stamp
        marker.ns = "hybrid_dynamic"
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x + CHAIR_CENTRE_IN_BODY_XYZ[0]
        marker.pose.position.y = y + CHAIR_CENTRE_IN_BODY_XYZ[1]
        marker.pose.position.z = z + CHAIR_CENTRE_IN_BODY_XYZ[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz
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
            marker = self._learned_marker(item, stamp, marker_id)
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
