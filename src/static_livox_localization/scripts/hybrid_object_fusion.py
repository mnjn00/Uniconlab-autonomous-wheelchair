#!/usr/bin/env python3
"""Fuse geometric cluster boxes with optional learned 3D detections.

The geometric topic is mandatory and is never weakened by learning. Learned
boxes improve semantics and may add high-confidence geometry. Output remains
the existing ``/perception/objects_summary`` JSON contract so the pursuit,
MPC, DWA, black box, and Bluetooth UI do not need a new message package.

The output axes and origin are read from the route. This matters whenever the
running IMU profile differs from the one that recorded the route: the follower
corrects its pose into the route body frame, so relative obstacle geometry must
be expressed in that same frame before it is rotated into the map.
"""

import json
import math
import os
import sys
import threading

import numpy as np
import rospy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import (CHAIR_CENTRE_IN_BODY_XYZ, lidar_extrinsics,
                        route_chair_centre)
from hybrid_perception import fuse_summaries


PUBLISH_HZ = 10.0


class HybridObjectFusion:
    def __init__(self):
        rospy.init_node("hybrid_object_fusion")
        self.lock = threading.Lock()
        self.geometric_payload = None
        self.learned_payload = None
        self.geometric_receipt_s = 0.0
        self.learned_receipt_s = 0.0

        running_profile = str(rospy.get_param(
            "~body_frame_profile", "builtin"))
        route_path = str(rospy.get_param("~route", ""))
        route_profile = str(rospy.get_param(
            "~output_body_frame_profile", running_profile))
        chair_centre = tuple(CHAIR_CENTRE_IN_BODY_XYZ)
        if route_path:
            try:
                with open(route_path, encoding="utf-8") as stream:
                    route = json.load(stream)
                route_profile = str(route["body_frame_profile"])
                chair_centre = route_chair_centre(route)
            except (IOError, OSError, KeyError, TypeError, ValueError) as error:
                raise rospy.ROSInitException(
                    "cannot derive hybrid output frame from route %s: %s"
                    % (route_path, error))

        lidar_in_route_body, lidar_to_route_body_rotation = \
            lidar_extrinsics(route_profile)
        self.rotation = np.asarray(
            lidar_to_route_body_rotation, dtype=float)
        # route_chair_T_lidar: p_route_chair = route_body_R_lidar p_lidar
        #                                 + route_body_p_lidar
        #                                 - route_body_p_chair
        self.translation = np.asarray(lidar_in_route_body, dtype=float) - \
            np.asarray(chair_centre, dtype=float)
        self.running_profile = running_profile
        self.output_profile = route_profile
        self.output_chair_centre = tuple(float(value) for value in chair_centre)

        self.require_learned = bool(rospy.get_param("~require_learned", False))
        self.geometric_max_age_s = float(rospy.get_param(
            "~geometric_max_age_s", 1.5))
        self.learned_max_age_s = float(rospy.get_param(
            "~learned_max_age_s", 1.0))
        self.maximum_skew_s = float(rospy.get_param(
            "~maximum_skew_s", 0.40))
        self.association_gate_m = float(rospy.get_param(
            "~association_gate_m", 0.85))
        self.person_score_threshold = float(rospy.get_param(
            "~person_score_threshold", 0.35))
        self.class_score_threshold = float(rospy.get_param(
            "~class_score_threshold", 0.50))
        self.learned_only_score_threshold = float(rospy.get_param(
            "~learned_only_score_threshold", 0.65))
        self.person_min_extent_m = float(rospy.get_param(
            "~person_min_extent_m", 0.70))
        for name in (
                "geometric_max_age_s", "learned_max_age_s",
                "maximum_skew_s", "association_gate_m",
                "person_min_extent_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise rospy.ROSInitException("~%s must be finite and positive" % name)
        for name in (
                "person_score_threshold", "class_score_threshold",
                "learned_only_score_threshold"):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise rospy.ROSInitException("~%s must be within [0, 1]" % name)

        geometric_topic = str(rospy.get_param(
            "~geometric_topic", "/perception/geometric_objects_summary"))
        learned_topic = str(rospy.get_param(
            "~learned_topic", "/perception/learned_objects_summary"))
        output_topic = str(rospy.get_param(
            "~output_topic", "/perception/objects_summary"))
        status_topic = str(rospy.get_param(
            "~status_topic", "/perception/hybrid_status"))

        self.output_pub = rospy.Publisher(output_topic, String, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=1)
        rospy.Subscriber(geometric_topic, String,
                         self.on_geometric, queue_size=2)
        rospy.Subscriber(learned_topic, String,
                         self.on_learned, queue_size=2)

        rospy.loginfo(
            "hybrid fusion: geometric=%s learned=%s required=%s output=%s "
            "frame=chair_centre running_profile=%s route_profile=%s",
            geometric_topic, learned_topic, self.require_learned,
            output_topic, running_profile, route_profile)

    @staticmethod
    def _parse(message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return None
        return value if isinstance(value, dict) else None

    def on_geometric(self, message):
        payload = self._parse(message)
        with self.lock:
            self.geometric_payload = payload
            self.geometric_receipt_s = rospy.Time.now().to_sec()

    def on_learned(self, message):
        payload = self._parse(message)
        with self.lock:
            self.learned_payload = payload
            self.learned_receipt_s = rospy.Time.now().to_sec()

    def step(self):
        now_s = rospy.Time.now().to_sec()
        with self.lock:
            geometric = self.geometric_payload
            learned = self.learned_payload
            geometric_receipt = self.geometric_receipt_s
            learned_receipt = self.learned_receipt_s

        result = fuse_summaries(
            geometric,
            learned,
            now_s=now_s,
            rotation=self.rotation,
            translation=self.translation,
            geometric_frame="lidar",
            learned_frame="lidar",
            output_frame="chair_centre",
            geometric_max_age_s=self.geometric_max_age_s,
            learned_max_age_s=self.learned_max_age_s,
            maximum_skew_s=self.maximum_skew_s,
            association_gate_m=self.association_gate_m,
            person_score_threshold=self.person_score_threshold,
            class_score_threshold=self.class_score_threshold,
            learned_only_score_threshold=self.learned_only_score_threshold,
            person_min_extent_m=self.person_min_extent_m,
            require_learned=self.require_learned,
        )
        result["body_frame_profile"] = self.output_profile
        result["chair_centre_in_body_xyz"] = list(self.output_chair_centre)
        self.output_pub.publish(String(data=json.dumps(
            result, separators=(",", ":"), sort_keys=True)))

        health = {
            "stamp": now_s,
            "status": result.get("status", "UNKNOWN"),
            "mode": result.get("mode", "blocked"),
            "frame": result.get("frame", ""),
            "object_count": len(result.get("objects", [])),
            "sources": result.get("sources", {}),
            "running_body_frame_profile": self.running_profile,
            "output_body_frame_profile": self.output_profile,
            "geometric_receipt_age_s": None if geometric_receipt <= 0.0 else
                round(max(0.0, now_s - geometric_receipt), 3),
            "learned_receipt_age_s": None if learned_receipt <= 0.0 else
                round(max(0.0, now_s - learned_receipt), 3),
            "require_learned": self.require_learned,
        }
        self.status_pub.publish(String(data=json.dumps(
            health, separators=(",", ":"), sort_keys=True)))

    def spin(self):
        rate = rospy.Rate(PUBLISH_HZ)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    try:
        HybridObjectFusion().spin()
    except rospy.ROSInterruptException:
        pass