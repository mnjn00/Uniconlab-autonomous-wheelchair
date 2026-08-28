#!/usr/bin/env python3
"""Semantic supervisor that permits one independently qualified static person.

Moving/unknown people and objects remain stop-only. A static person is omitted
from the person latch only while a fresh DWA permit matches the same current
geometric track. The permit cannot authorize motion by itself: stale
perception, a changed track, another moving object, or an invalid command still
produces a stop.
"""

import copy
import json
import math
import os
import sys

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from cluster_guard import (  # noqa: E402
    PERSON_LABEL,
    Threat,
    corridor_reach,
    nearest_threat,
)
from cluster_tracking import STATIC  # noqa: E402
from person_bypass_policy import (  # noqa: E402
    permit_from_payload,
    permit_is_fresh,
    permit_matches_observation,
    person_observations,
)
from semantic_safety_policy import (  # noqa: E402
    SemanticDecision,
    decide_semantic_stop,
    stopping_distance,
)
from semantic_safety_supervisor import (  # noqa: E402
    SemanticSafetySupervisor,
)


def nearest_dynamic_threat(summary, half_width_m):
    """Nearest moving/unknown object, even behind a nearer static target."""
    if summary is None or not summary.usable:
        return None
    nearest = None
    for item in summary.objects:
        if not isinstance(item, dict):
            return Threat(0.0, "moving", "malformed")
        blocks, distance, motion = corridor_reach(
            item, 0.0, float(half_width_m))
        if not blocks or motion == STATIC:
            continue
        threat = Threat(distance, motion, str(item.get("class", "")))
        if nearest is None or threat.distance_m < nearest.distance_m:
            nearest = threat
    return nearest


class PersonBypassSemanticSupervisor(SemanticSafetySupervisor):
    def __init__(self):
        super(PersonBypassSemanticSupervisor, self).__init__()
        self.bypass_permit = None
        self.maximum_permit_age_s = float(rospy.get_param(
            "~maximum_person_bypass_permit_age_s", 0.45))
        self.maximum_target_error_m = float(rospy.get_param(
            "~maximum_person_bypass_target_error_m", 0.45))
        self.bypass_maximum_forward_m = float(rospy.get_param(
            "~person_bypass_maximum_forward_m", 8.0))
        self.bypass_maximum_lateral_m = float(rospy.get_param(
            "~person_bypass_maximum_lateral_m", 1.0))
        for name in (
                "maximum_permit_age_s", "maximum_target_error_m",
                "bypass_maximum_forward_m", "bypass_maximum_lateral_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise rospy.ROSInitException(
                    "~%s must be finite and positive" % name)
        permit_topic = str(rospy.get_param(
            "~person_bypass_permit_topic", "/person_bypass/permit"))
        rospy.Subscriber(permit_topic, String, self.on_bypass_permit,
                         queue_size=2)
        rospy.set_param("~person_bypass_capable", True)
        rospy.loginfo(
            "semantic static-person bypass enabled; permit=%s max-age=%.2f s",
            permit_topic, self.maximum_permit_age_s)

    def on_bypass_permit(self, message):
        self.bypass_permit = permit_from_payload(message.data)

    def validated_bypass(self, now_s):
        permit = self.bypass_permit
        if permit is None or not permit.active or not permit_is_fresh(
                permit, now_s, self.maximum_permit_age_s):
            return None, None
        observations = person_observations(
            self.summary,
            maximum_forward_m=self.bypass_maximum_forward_m,
            maximum_lateral_m=self.bypass_maximum_lateral_m,
        )
        if len(observations) != 1:
            return None, None
        observation = observations[0]
        if not permit_matches_observation(
                permit, observation, self.maximum_target_error_m):
            return None, None
        return permit, observation

    def step(self):
        now = rospy.Time.now()
        now_s = now.to_sec()
        command_age = (now - self.command_stamp).to_sec()
        summary_age = math.inf
        summary_usable = False
        person = None
        nearest_dynamic = None
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
                nearest_dynamic = nearest_dynamic_threat(
                    self.summary, self.corridor_half_width_m)

        permit, bypass_person = self.validated_bypass(now_s) \
            if summary_usable else (None, None)
        bypass_active = permit is not None
        if bypass_active:
            self.person_latch.reset()
            person_for_stop = None
        else:
            person_for_stop = person

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
            person=self._view(person_for_stop),
            nearest=self._view(nearest_dynamic),
            person_latch=self.person_latch,
        )

        out = Twist()
        if not decision.blocked:
            values = (
                self.command.linear.x, self.command.linear.y,
                self.command.linear.z, self.command.angular.x,
                self.command.angular.y, self.command.angular.z,
            )
            if all(math.isfinite(value) for value in values) and \
                    self.command.linear.x >= 0.0:
                out = copy.deepcopy(self.command)
                if bypass_active:
                    out.linear.x = min(
                        float(out.linear.x), float(permit.max_speed_mps))
            else:
                decision = SemanticDecision(
                    "INPUT_INVALID", decision.stop_distance_m,
                    decision.release_distance_m)
        self.pub.publish(out)

        chosen = person_for_stop if decision.reason == "PERSON" \
            else nearest_dynamic
        permit_age = None
        if self.bypass_permit is not None:
            permit_age = now_s - self.bypass_permit.stamp_s
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
            "person_bypass_capable": True,
            "person_bypass_active": bypass_active,
            "person_bypass_track_id": None if permit is None
                                      else permit.track_id,
            "person_bypass_static_for_s": None if permit is None
                                         else round(permit.static_for_s, 3),
            "person_bypass_permit_age_s": None if permit_age is None
                                          or not math.isfinite(permit_age)
                                          else round(permit_age, 3),
        }
        if bypass_person is not None:
            report["person_bypass_target_x_m"] = round(bypass_person.x_m, 3)
            report["person_bypass_target_y_m"] = round(bypass_person.y_m, 3)
        self.status_pub.publish(String(data=json.dumps(
            report, separators=(",", ":"), sort_keys=True)))


if __name__ == "__main__":
    try:
        PersonBypassSemanticSupervisor().spin()
    except rospy.ROSInterruptException:
        pass
