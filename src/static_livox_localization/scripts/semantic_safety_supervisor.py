#!/usr/bin/env python3
"""Independent semantic stop supervisor ahead of the raw safety gate.

DWA owns avoidance of confirmed static geometry.  This node owns the simpler,
safer rule that must not depend on a planner: wait for people and for any
moving/unknown object inside the braking envelope.  It also latches a person
stop through the dynamic envelope shrinking after the chair stops.

Command chain in the hybrid profile::

    dwa_follower -> /cmd_vel_planned
                 -> semantic_safety_supervisor -> /cmd_vel_raw
                 -> safety_gate -> terrain_guard -> tip_guard -> base
"""

import json
import math
import os
import sys

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from cluster_guard import PERSON_LABEL, nearest_threat, parse_summary
from semantic_safety_policy import (
    PersonStopLatch,
    ThreatView,
    decide_semantic_stop,
    stopping_distance,
)


SUPERVISOR_HZ = 20.0
WHEEL_SEPARATION_M = 0.54


class SemanticSafetySupervisor:
    def __init__(self):
        rospy.init_node("semantic_safety_supervisor")
        self.command = Twist()
        self.command_stamp = rospy.Time(0)
        self.summary = None
        self.summary_frame = ""
        self.measured_speed = 0.0
        self.person_memory = None
        self.person_latch = PersonStopLatch(float(rospy.get_param(
            "~person_release_margin_m", 0.30)))

        self.maximum_summary_age_s = float(rospy.get_param(
            "~maximum_summary_age_s", 1.5))
        self.maximum_command_age_s = float(rospy.get_param(
            "~maximum_command_age_s", 0.6))
        self.corridor_half_width_m = float(rospy.get_param(
            "~corridor_half_width_m", 0.50))
        self.person_half_width_m = float(rospy.get_param(
            "~person_half_width_m", 0.65))
        self.person_memory_s = float(rospy.get_param(
            "~person_memory_s", 1.0))
        self.accumulation_s = float(rospy.get_param(
            "~accumulation_s", 0.6))
        self.pipeline_s = float(rospy.get_param("~pipeline_s", 0.2))
        self.minimum_deceleration_mps2 = float(rospy.get_param(
            "~minimum_deceleration_mps2", 0.5))
        self.geometry_margin_m = float(rospy.get_param(
            "~geometry_margin_m", 0.9))
        expected_frame = str(rospy.get_param(
            "~expected_summary_frame", "chair_centre"))
        self.expected_summary_frame = expected_frame

        for name in (
                "maximum_summary_age_s", "maximum_command_age_s",
                "corridor_half_width_m", "person_half_width_m",
                "person_memory_s",
                "accumulation_s", "pipeline_s",
                "minimum_deceleration_mps2", "geometry_margin_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise rospy.ROSInitException("~%s must be finite and positive" % name)

        planned_topic = str(rospy.get_param(
            "~planned_topic", "/cmd_vel_planned"))
        summary_topic = str(rospy.get_param(
            "~summary_topic", "/perception/objects_summary"))
        output_topic = str(rospy.get_param(
            "~output_topic", "/cmd_vel_raw"))
        status_topic = str(rospy.get_param(
            "~status_topic", "/semantic_safety/status"))

        self.pub = rospy.Publisher(output_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=1)
        rospy.Subscriber(planned_topic, Twist, self.on_command, queue_size=1)
        rospy.Subscriber(summary_topic, String, self.on_summary, queue_size=2)
        rospy.Subscriber("/wheel_status", Int16MultiArray,
                         self.on_wheel_status, queue_size=5)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))
        rospy.loginfo(
            "semantic safety: %s -> %s, objects=%s frame=%s",
            planned_topic, output_topic, summary_topic, expected_frame)

    def on_command(self, message):
        self.command = message
        self.command_stamp = rospy.Time.now()

    def on_summary(self, message):
        try:
            raw = json.loads(message.data)
            if not isinstance(raw, dict):
                raise ValueError("summary is not an object")
            self.summary_frame = str(raw.get("frame", ""))
            self.summary = parse_summary(message.data)
        except (TypeError, ValueError) as error:
            self.summary = None
            self.summary_frame = ""
            rospy.logwarn_throttle(
                2.0, "semantic safety received an unreadable summary: %s",
                error)

    @staticmethod
    def _reported_speed(data):
        def one(direction, magnitude):
            try:
                speed = (float(magnitude) - 0x21) / 10.0 / 3.6
                letter = chr(int(direction))
            except (TypeError, ValueError, OverflowError):
                return 0.0
            if letter == "C":
                return speed
            if letter == "W":
                return -speed
            return 0.0
        if len(data) < 6:
            return 0.0
        left = one(data[2], data[3])
        right = one(data[4], data[5])
        return (left + right) * 0.5

    def on_wheel_status(self, message):
        self.measured_speed = self._reported_speed(message.data)

    @staticmethod
    def _view(threat):
        if threat is None:
            return None
        track_id = getattr(threat, "track_id", None)
        return ThreatView(
            float(threat.distance_m),
            str(threat.motion),
            str(threat.label),
            track_id if isinstance(track_id, int) else None,
        )

    def step(self):
        now = rospy.Time.now()
        now_s = now.to_sec()
        command_age = (now - self.command_stamp).to_sec()
        summary_age = math.inf
        summary_usable = False
        person = None
        nearest = None
        if self.summary is not None:
            summary_age = now_s - self.summary.stamp_s
            summary_usable = self.summary.usable and \
                self.summary_frame == self.expected_summary_frame
            if summary_usable:
                person = nearest_threat(
                    self.summary, self.person_half_width_m,
                    labels=(PERSON_LABEL,))
                if person is not None:
                    self.person_memory = (self.summary.stamp_s, person)
                elif self.person_memory is not None:
                    memory_stamp, remembered = self.person_memory
                    memory_age = self.summary.stamp_s - memory_stamp
                    if 0.0 <= memory_age <= self.person_memory_s:
                        person = remembered
                    else:
                        self.person_memory = None
                nearest = nearest_threat(
                    self.summary, self.corridor_half_width_m)

        stop_m = stopping_distance(
            measured_speed_mps=self.measured_speed,
            requested_speed_mps=self.command.linear.x,
            cloud_age_s=max(0.0, summary_age) if math.isfinite(summary_age)
                        else math.inf,
            accumulation_s=self.accumulation_s,
            pipeline_s=self.pipeline_s,
            minimum_deceleration_mps2=self.minimum_deceleration_mps2,
            geometry_margin_m=self.geometry_margin_m,
        )
        decision = decide_semantic_stop(
            summary_usable=summary_usable,
            summary_age_s=summary_age,
            command_age_s=command_age,
            maximum_summary_age_s=self.maximum_summary_age_s,
            maximum_command_age_s=self.maximum_command_age_s,
            stop_distance_m=stop_m,
            person=self._view(person),
            nearest=self._view(nearest),
            person_latch=self.person_latch,
        )

        out = Twist()
        if not decision.blocked:
            values = (
                self.command.linear.x, self.command.linear.y,
                self.command.linear.z, self.command.angular.x,
                self.command.angular.y, self.command.angular.z,
            )
            if all(math.isfinite(value) for value in values) \
                    and self.command.linear.x >= 0.0:
                out = self.command
            else:
                decision = type(decision)(
                    "INPUT_INVALID", decision.stop_distance_m,
                    decision.release_distance_m)
        self.pub.publish(out)

        chosen = person if decision.reason == "PERSON" else nearest
        report = {
            "stamp": now_s,
            "reason": decision.reason,
            "blocked": decision.blocked,
            "summary_frame": self.summary_frame,
            "summary_age_s": None if not math.isfinite(summary_age)
                             else round(summary_age, 3),
            "command_age_s": round(command_age, 3),
            "stop_distance_m": None if not math.isfinite(stop_m)
                               else round(stop_m, 3),
            "release_distance_m": None if decision.release_distance_m is None
                                  or not math.isfinite(decision.release_distance_m)
                                  else round(decision.release_distance_m, 3),
            "threat_class": None if chosen is None else chosen.label,
            "threat_motion": None if chosen is None else chosen.motion,
            "threat_distance_m": None if chosen is None else
                round(float(chosen.distance_m), 3),
            "planned_v": round(float(self.command.linear.x), 3),
            "out_v": round(float(out.linear.x), 3),
        }
        self.status_pub.publish(String(data=json.dumps(
            report, separators=(",", ":"), sort_keys=True)))

    def spin(self):
        rate = rospy.Rate(SUPERVISOR_HZ)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    try:
        SemanticSafetySupervisor().spin()
    except rospy.ROSInterruptException:
        pass
