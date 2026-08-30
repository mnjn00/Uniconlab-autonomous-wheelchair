#!/usr/bin/env python3
"""Strict v2 semantic supervisor for one qualified static-threat track."""

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
    matching_threats,
    object_box,
)
from cluster_tracking import STATIC  # noqa: E402
from person_bypass_policy import (  # noqa: E402
    STATIC_THREAT_DROPOUT_GRACE,
    permit_from_payload,
    permit_is_fresh,
    permit_matches_observation,
    threat_observations,
)
from semantic_safety_policy import (  # noqa: E402
    SemanticDecision,
    decide_semantic_stop,
    stopping_distance,
)
from semantic_safety_supervisor import SemanticSafetySupervisor  # noqa: E402


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
        track_id = item.get("id")
        if isinstance(track_id, bool) or not isinstance(track_id, int):
            track_id = None
        box = object_box(item)
        threat = Threat(
            distance, motion, str(item.get("class", "")),
            track_id=track_id, observed_stamp_s=summary.stamp_s,
            directly_observed=item.get("directly_observed", True) is True,
            geometry_valid=item.get("geometry_valid", True) is True,
            center_x_m=None if box is None else box[0],
            center_y_m=None if box is None else box[1],
        )
        if nearest is None or threat.distance_m < nearest.distance_m:
            nearest = threat
    return nearest


class PersonBypassSemanticSupervisor(SemanticSafetySupervisor):
    def __init__(self):
        super(PersonBypassSemanticSupervisor, self).__init__()
        self.bypass_permit = None
        self.maximum_permit_age_s = float(rospy.get_param(
            "~static_threat_bypass_maximum_permit_age_s", 0.45))
        self.maximum_target_error_m = float(rospy.get_param(
            "~static_threat_bypass_maximum_target_error_m", 0.45))
        self.bypass_maximum_forward_m = float(rospy.get_param(
            "~static_threat_bypass_maximum_forward_m", 8.0))
        self.bypass_maximum_lateral_m = float(rospy.get_param(
            "~static_threat_bypass_maximum_lateral_m", 1.0))
        for name in (
                "maximum_permit_age_s", "maximum_target_error_m",
                "bypass_maximum_forward_m", "bypass_maximum_lateral_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise rospy.ROSInitException(
                    "~%s must be finite and positive" % name)
        permit_topic = str(rospy.get_param(
            "~static_threat_bypass_permit_topic",
            "/static_threat_bypass/permit"))
        rospy.Subscriber(permit_topic, String, self.on_bypass_permit,
                         queue_size=2)
        rospy.set_param("~static_threat_bypass_capable", True)
        rospy.loginfo(
            "semantic static-threat bypass enabled; permit=%s max-age=%.2f s",
            permit_topic, self.maximum_permit_age_s)

    def on_bypass_permit(self, message):
        self.bypass_permit = permit_from_payload(message.data)

    def validated_bypass(self, now_s):
        permit = self.bypass_permit
        dynamic = nearest_dynamic_threat(self.summary, self.corridor_half_width_m)
        if permit is None or not permit.active:
            return None, None, "", dynamic
        if not permit_is_fresh(permit, now_s, self.maximum_permit_age_s):
            return None, None, "STATIC_THREAT_PERMIT_STALE", dynamic
        observations = threat_observations(
            self.summary,
            maximum_forward_m=self.bypass_maximum_forward_m,
            maximum_lateral_m=self.bypass_maximum_lateral_m,
        )
        matching = tuple(observation for observation in observations
                         if observation.track_id == permit.track_id)
        if len(matching) == 1 and permit_matches_observation(
                permit, matching[0], self.maximum_target_error_m):
            return permit, matching[0], "", dynamic
        if permit.reason == STATIC_THREAT_DROPOUT_GRACE and \
                not observations and dynamic is None:
            return permit, None, "", None
        return None, None, "STATIC_THREAT_PERMIT_MISMATCH", dynamic

    def step(self):
        now = rospy.Time.now()
        now_s = now.to_sec()
        command_age = (now - self.command_stamp).to_sec()
        summary_age = math.inf
        summary_usable = False
        nearest_dynamic = None
        if self.summary is not None:
            summary_age = now_s - self.summary.stamp_s
            summary_usable = self.summary.usable and \
                self.summary_frame == self.expected_summary_frame
        permit, bypass_target, permit_fault, nearest_dynamic = \
            self.validated_bypass(now_s) if summary_usable \
            else (None, None, "", None)
        bypass_active = permit is not None
        permitted_person_id = permit.track_id if bypass_active and \
            permit.threat_label == PERSON_LABEL else None
        objects_valid = summary_usable and all(
            isinstance(item, dict) for item in self.summary.objects)
        people = matching_threats(
            self.summary, self.person_half_width_m, labels=(PERSON_LABEL,)) \
            if objects_valid else ()
        people = tuple(
            threat for threat in people
            if threat.track_id != permitted_person_id)
        person_for_stop = people[0] if people else None
        if person_for_stop is not None:
            self.person_memory = (self.summary.stamp_s, person_for_stop)
        elif self.person_memory is not None:
            memory_stamp, remembered = self.person_memory
            memory_age = self.summary.stamp_s - memory_stamp
            if remembered.track_id == permitted_person_id:
                self.person_memory = None
            elif 0.0 <= memory_age <= self.person_memory_s:
                person_for_stop = remembered
            else:
                self.person_memory = None
        if permitted_person_id is not None:
            self.person_latch.release_track(permitted_person_id)

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
        if not decision.blocked and bypass_active and \
                nearest_dynamic is not None:
            decision = SemanticDecision(
                "DYNAMIC_THREAT_CONFLICT", decision.stop_distance_m,
                decision.release_distance_m)
        if not decision.blocked and permit_fault:
            decision = SemanticDecision(
                permit_fault, decision.stop_distance_m,
                decision.release_distance_m)

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
            "static_threat_bypass_capable": True,
            "static_threat_bypass_active": bypass_active,
            "static_threat_bypass_validation": permit_fault or (
                "VALID" if bypass_active else "INACTIVE"),
            "static_threat_bypass_track_id": None if permit is None
                                             else permit.track_id,
            "static_threat_bypass_label": None if permit is None
                                          else permit.threat_label,
            "static_threat_bypass_static_for_s": None if permit is None
                                                else round(
                                                    permit.static_for_s, 3),
            "static_threat_bypass_permit_age_s": None if permit_age is None
                                                 or not math.isfinite(permit_age)
                                                 else round(permit_age, 3),
        }
        if bypass_target is not None:
            report["static_threat_bypass_target_x_m"] = round(bypass_target.x_m, 3)
            report["static_threat_bypass_target_y_m"] = round(bypass_target.y_m, 3)
        self.status_pub.publish(String(data=json.dumps(
            report, separators=(",", ":"), sort_keys=True)))


if __name__ == "__main__":
    try:
        PersonBypassSemanticSupervisor().spin()
    except rospy.ROSInterruptException:
        pass
