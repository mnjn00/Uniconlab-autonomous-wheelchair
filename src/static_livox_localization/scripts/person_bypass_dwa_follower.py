#!/usr/bin/env python3
"""RTX DWA follower that conditionally passes a continuously static threat.

The base follower intentionally waits for every threat without a shared
permit. This wrapper changes only that policy transition: after direct
same-track STATIC evidence, DWA may plan around exactly one threat at the
turn-speed floor. Moving, unknown, learned-only, too-close, multiple, stale,
or geometrically invalid threats remain stop-only.
"""

import json
from dataclasses import replace
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scipy_ckdtree_compat import install as install_ckdtree_compat
install_ckdtree_compat()

import rospy
from std_msgs.msg import String

import dwa_core
from gpu_dwa_backend import GpuRequiredError, install_gpu_planner

# Install before DwaFollower constructs dwa_core.DwaPlanner. Environment and
# ROS params still choose CuPy or the diagnostic CPU path.
install_gpu_planner(dwa_core)
from cluster_guard import GO_ROUND  # noqa: E402
from dwa_follower import (BYPASS_MINIMUM_TURN_RPS, DwaFollower)  # noqa: E402
from person_bypass_policy import (  # noqa: E402
    STATIC,
    StaticThreatBypassManager,
    permit_matches_threat,
    threat_observations,
)


class PersonBypassDwaFollower(DwaFollower):
    CONTROL_LAW = "dwa"

    def __init__(self):
        super(PersonBypassDwaFollower, self).__init__()
        self.person_bypass_confirmation_s = float(rospy.get_param(
            "~static_threat_bypass_confirmation_s", 2.0))
        self.person_bypass_maximum_gap_s = float(rospy.get_param(
            "~static_threat_bypass_maximum_gap_s", 0.45))
        self.person_bypass_position_jump_m = float(rospy.get_param(
            "~static_threat_bypass_position_jump_m", 0.35))
        self.person_bypass_permit_lifetime_s = float(rospy.get_param(
            "~static_threat_bypass_permit_lifetime_s", 0.45))
        self.person_bypass_maximum_forward_m = float(rospy.get_param(
            "~static_threat_bypass_maximum_forward_m", 8.0))
        self.person_bypass_maximum_lateral_m = float(rospy.get_param(
            "~static_threat_bypass_maximum_lateral_m", 1.0))
        self.person_bypass_lateral_hysteresis_m = float(rospy.get_param(
            "~static_threat_bypass_lateral_hysteresis_m", 0.25))
        self.person_bypass_minimum_near_m = float(rospy.get_param(
            "~static_threat_bypass_minimum_near_m", 0.60))
        self.person_bypass_speed_mps = float(rospy.get_param(
            "~static_threat_bypass_speed_mps", 0.35))
        self.person_bypass_clearance_m = float(rospy.get_param(
            "~static_threat_bypass_clearance_m", 0.35))
        self.qualifier = StaticThreatBypassManager(
            confirmation_s=self.person_bypass_confirmation_s,
            maximum_gap_s=self.person_bypass_maximum_gap_s,
            maximum_position_jump_m=self.person_bypass_position_jump_m,
            permit_lifetime_s=self.person_bypass_permit_lifetime_s,
            maximum_forward_m=self.person_bypass_maximum_forward_m,
            maximum_lateral_m=self.person_bypass_maximum_lateral_m,
            lateral_hysteresis_m=self.person_bypass_lateral_hysteresis_m,
            minimum_near_distance_m=self.person_bypass_minimum_near_m,
            max_speed_mps=self.person_bypass_speed_mps,
            min_clearance_m=self.person_bypass_clearance_m,
        )
        permit_topic = str(rospy.get_param(
            "~static_threat_bypass_permit_topic",
            "/static_threat_bypass/permit"))
        self.permit_pub = rospy.Publisher(
            permit_topic, String, queue_size=1, latch=False)
        self.bypass_status_pub = rospy.Publisher(
            "/static_threat_bypass/status", String,
            queue_size=1, latch=False)
        self._permit_published_this_cycle = False
        self._qualification_key = None
        self._latest_permit = None
        self.active_proposal_seq = None
        self._clear_released = False
        rospy.set_param("~static_threat_bypass_capable", True)
        rospy.set_param("~static_threat_bypass_proposal_capable", True)
        rospy.loginfo(
            "stationary-threat bypass: %.1f s same-track STATIC, "
            "v<=%.2f m/s, clearance>=%.2f m",
            self.person_bypass_confirmation_s,
            self.person_bypass_speed_mps,
            self.person_bypass_clearance_m)

    def publish_permit(self, permit):
        self.permit_pub.publish(String(data=permit.to_json()))
        self._permit_published_this_cycle = True
        self.bypass_status_pub.publish(String(data=json.dumps({
            "active": bool(permit.active),
            "clear_frames": int(self.qualifier.clear_frames),
            "committed_side": self.qualifier.pass_side,
            "lifecycle": self.qualifier.lifecycle,
            "proposal_seq": self.active_proposal_seq,
            "reason": permit.reason,
            "static_for_s": round(float(permit.static_for_s), 3),
            "track_id": permit.track_id,
        }, separators=(",", ":"), sort_keys=True)))

    def activate_trajectory_bypass(self, permit, detail):
        self.planner.max_speed = min(
            float(self.planner.max_speed), float(permit.max_speed_mps))
        dwa_core.OBSTACLE_FLOOR_M = max(
            float(dwa_core.OBSTACLE_FLOOR_M),
            float(permit.min_clearance_m))
        self.gate_reason = ""
        self.gate_blocked_since = None
        self.gate_detail = detail

    def inactive_permit(self, now, reason):
        return self.qualifier.inactive(now.to_sec(), reason)

    def observed_threat_permit(self, now, threat=None):
        """Update qualification even while the motion service is paused.

        The base follower returns from its hold ladder before asking
        ``avoidance_for`` when it is paused. Qualification here lets the
        shared permit become ready without sending a motion command.
        """
        if threat is None:
            threat = self.corridor_threat(0.0)
        target_track_id = getattr(threat, "track_id", None)
        observations = tuple(
            observation for observation in threat_observations(
            self.cluster_summary,
            maximum_forward_m=self.person_bypass_maximum_forward_m,
            maximum_lateral_m=(
                self.person_bypass_maximum_lateral_m
                + self.person_bypass_lateral_hysteresis_m),
            retained_track_id=(
                getattr(self.qualifier, "track_id", None)
                if getattr(self.qualifier, "committed", False) else None),
            ) if observation.track_id == target_track_id)
        summary_healthy = (
            self.cluster_summary is not None
            and self.cluster_summary.usable)
        summary_stamp_s = (
            self.cluster_summary.stamp_s if self.cluster_summary is not None
            else None)
        qualification_key = (
            summary_stamp_s, summary_healthy,
            self.tracking_state == "TRACKING", target_track_id)
        if qualification_key == getattr(self, "_qualification_key", None) and \
                getattr(self, "_latest_permit", None) is not None:
            return self._latest_permit
        dynamic_conflict = any(
            observation.motion.strip().lower() != STATIC
            for observation in observations)
        permit = self.qualifier.update(
            observations, now.to_sec(), self.tracking_state == "TRACKING",
            summary_healthy=summary_healthy,
            dynamic_conflict=dynamic_conflict,
            summary_stamp_s=summary_stamp_s)
        self._qualification_key = qualification_key
        self._latest_permit = permit
        return permit

    def avoidance_for(self, now, threat, blocking):
        permit = self.observed_threat_permit(now, threat)
        ordinary = super(PersonBypassDwaFollower, self).avoidance_for(
            now, threat, blocking, bypass_permit=permit)
        self.publish_permit(permit)
        if not permit.active or ordinary != GO_ROUND:
            return ordinary

        # DWA, semantic supervision, and raw trajectory validation consume
        # this same short-lived authorization. No stage infers it from a label.
        self.activate_trajectory_bypass(
            permit, "static-threat trajectory permit")
        return ordinary

    def may_bypass_gate_stall(self, now, threat):
        return permit_matches_threat(
            getattr(self, "_latest_permit", None), threat, now.to_sec())

    def bypass_proposal_identity(self, now, threat, decision):
        permit = getattr(self, "_latest_permit", None)
        if decision != GO_ROUND or not permit_matches_threat(
                permit, threat, now.to_sec()):
            return None
        return permit.track_id, self.qualifier.pass_side

    def accept_bypass_proposal(self, proposal):
        if abs(proposal.target_yaw_rate_rps) < BYPASS_MINIMUM_TURN_RPS:
            return None
        proposed_side = (
            "left" if proposal.target_yaw_rate_rps > 0.0 else "right")
        committed_side = self.qualifier.commit_pass_side(proposed_side)
        accepted = replace(proposal, committed_side=committed_side.upper())
        self.active_proposal_seq = accepted.proposal_seq
        return accepted

    def publish_bypass_diagnostics(self, proposal, planner_ms):
        self.bypass_status_pub.publish(String(data=json.dumps({
            "applied_v": round(proposal.first_applied_speed_mps, 4),
            "applied_w": round(proposal.first_applied_yaw_rate_rps, 4),
            "candidate_count": int(getattr(
                self.planner, "last_candidate_count", 0)),
            "committed_side": proposal.committed_side,
            "event": "PROPOSAL_SELECTED",
            "planner_ms": round(float(planner_ms), 3),
            "proposal_seq": proposal.proposal_seq,
            "target_v": round(proposal.target_speed_mps, 4),
            "target_w": round(proposal.target_yaw_rate_rps, 4),
            "track_id": proposal.permit_track_id,
        }, separators=(",", ":"), sort_keys=True)))

    def consume_bypass_gate_report(self, report):
        qualifier = getattr(self, "qualifier", None)
        if qualifier is None or not getattr(qualifier, "committed", False) or \
                qualifier.pass_side is None:
            return
        sequence = report.get("trajectory_proposal_seq")
        track_id = report.get("static_threat_bypass_track_id")
        matched = (
            isinstance(sequence, int) and not isinstance(sequence, bool)
            and sequence == self.active_proposal_seq
            and isinstance(track_id, int) and not isinstance(track_id, bool)
            and track_id == qualifier.track_id)
        clear = (
            matched
            and report.get("static_threat_target_behind") is True
            and report.get("static_threat_tail_clear") is True)
        released = qualifier.observe_tail_clear(clear)
        if released:
            self.active_proposal_seq = None
            self._qualification_key = None
            self._latest_permit = None
            self._clear_released = True

    def step(self):
        self._permit_published_this_cycle = False
        saved_max_speed = float(self.planner.max_speed)
        saved_clearance = float(dwa_core.OBSTACLE_FLOOR_M)
        now = rospy.Time.now()
        # PAUSED preflight qualifies without motion. During driving,
        # avoidance_for remains the single update point for each control cycle.
        if not self.enabled:
            self.publish_permit(self.observed_threat_permit(now))
        try:
            super(PersonBypassDwaFollower, self).step()
        finally:
            self.planner.max_speed = saved_max_speed
            dwa_core.OBSTACLE_FLOOR_M = saved_clearance
            if not self._permit_published_this_cycle:
                if self.tracking_state != "TRACKING":
                    self.qualifier.reset()
                self.publish_permit(self.inactive_permit(
                    now, "FOLLOWER_NOT_EVALUATING_THREAT"))


if __name__ == "__main__":
    try:
        PersonBypassDwaFollower().run()
    except GpuRequiredError as error:
        rospy.logfatal("required RTX DWA backend failed: %s", error)
        raise SystemExit(2)
    except rospy.ROSInterruptException:
        pass
