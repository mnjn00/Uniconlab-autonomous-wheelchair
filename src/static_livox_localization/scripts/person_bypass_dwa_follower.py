#!/usr/bin/env python3
"""RTX DWA follower that conditionally passes a continuously static person.

The stock hybrid follower intentionally waits for every person. This wrapper
changes only that policy transition: after direct same-track STATIC evidence,
DWA may plan around exactly one person at the turn-speed floor. Moving,
unknown, learned-only, multiple, stale, or geometrically invalid people remain
stop-only. A qualified person that is already safely beside the chair does
not force another stop merely because its longitudinal distance is small.
"""

import math
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
from cluster_guard import CLEAR, GO_ROUND, WAIT  # noqa: E402
from trajectory_safety_gate import make_raw_gate_candidate_veto  # noqa: E402
from dwa_follower import DwaFollower  # noqa: E402
from person_bypass_policy import (  # noqa: E402
    StaticPersonQualifier,
    person_observations,
    static_obstacle_permit,
)


class PersonBypassDwaFollower(DwaFollower):
    CONTROL_LAW = "dwa"

    def __init__(self):
        self._committed_bypass_track_id = None
        self._committed_bypass_side = 0
        self.active_trajectory_permit = None
        super(PersonBypassDwaFollower, self).__init__()
        self.person_bypass_confirmation_s = float(rospy.get_param(
            "~person_bypass_confirmation_s", 3.0))
        self.person_bypass_maximum_gap_s = float(rospy.get_param(
            "~person_bypass_maximum_gap_s", 0.45))
        self.person_bypass_position_jump_m = float(rospy.get_param(
            "~person_bypass_position_jump_m", 0.35))
        self.person_bypass_permit_lifetime_s = float(rospy.get_param(
            "~person_bypass_permit_lifetime_s", 0.45))
        self.person_bypass_maximum_forward_m = float(rospy.get_param(
            "~person_bypass_maximum_forward_m", 8.0))
        self.person_bypass_observation_forward_m = float(rospy.get_param(
            "~person_bypass_observation_forward_m", 10.0))
        self.person_bypass_maximum_lateral_m = float(rospy.get_param(
            "~person_bypass_maximum_lateral_m", 1.0))
        self.person_bypass_lateral_hysteresis_m = float(rospy.get_param(
            "~person_bypass_lateral_hysteresis_m", 0.25))
        self.person_bypass_minimum_near_m = float(rospy.get_param(
            "~person_bypass_minimum_near_m", 0.60))
        self.person_bypass_passed_side_grace_s = float(rospy.get_param(
            "~person_bypass_passed_side_grace_s", 1.0))
        self.person_bypass_speed_mps = float(rospy.get_param(
            "~person_bypass_speed_mps", 0.35))
        self.person_bypass_clearance_m = float(rospy.get_param(
            "~person_bypass_clearance_m", 0.50))
        self.minimum_person_bypass_turn_rps = float(rospy.get_param(
            "/safety_gate/minimum_person_bypass_turn_rps", 0.08))
        self.qualifier = StaticPersonQualifier(
            confirmation_s=self.person_bypass_confirmation_s,
            maximum_gap_s=self.person_bypass_maximum_gap_s,
            maximum_position_jump_m=self.person_bypass_position_jump_m,
            permit_lifetime_s=self.person_bypass_permit_lifetime_s,
            maximum_forward_m=self.person_bypass_maximum_forward_m,
            observation_forward_m=self.person_bypass_observation_forward_m,
            maximum_lateral_m=self.person_bypass_maximum_lateral_m,
            lateral_hysteresis_m=self.person_bypass_lateral_hysteresis_m,
            minimum_near_distance_m=self.person_bypass_minimum_near_m,
            passed_side_grace_s=self.person_bypass_passed_side_grace_s,
            max_speed_mps=self.person_bypass_speed_mps,
            min_clearance_m=self.person_bypass_clearance_m,
        )
        self.permit_pub = rospy.Publisher(
            "/person_bypass/permit", String, queue_size=1, latch=False)
        self._permit_published_this_cycle = False
        rospy.set_param("~person_bypass_capable", True)
        rospy.set_param("~raw_gate_candidate_precheck", True)
        rospy.loginfo(
            "stationary-person bypass: %.1f s same-track STATIC, "
            "v<=%.2f m/s, clearance>=%.2f m",
            self.person_bypass_confirmation_s,
            self.person_bypass_speed_mps,
            self.person_bypass_clearance_m)

    def publish_permit(self, permit):
        self.permit_pub.publish(String(data=permit.to_json()))
        self._permit_published_this_cycle = True

    def reset_bypass_commitment(self):
        self._committed_bypass_track_id = None
        self._committed_bypass_side = 0

    def activate_trajectory_bypass(self, permit, detail):
        self.active_trajectory_permit = permit
        if self._committed_bypass_track_id != permit.track_id:
            # Commit away from the observed obstacle/person for the whole
            # pass. A stable side prevents small point-cloud changes from
            # alternating the winning DWA candidate left/right every frame.
            lateral = float(permit.target_y_m or 0.0)
            if abs(lateral) > 1e-3:
                self._committed_bypass_side = -1 if lateral > 0.0 else 1
            elif abs(getattr(self, "last_yaw_rate", 0.0)) > 1e-3:
                self._committed_bypass_side = (
                    1 if self.last_yaw_rate > 0.0 else -1)
            else:
                self._committed_bypass_side = 1
            self._committed_bypass_track_id = permit.track_id
        self.planner.max_speed = min(
            float(self.planner.max_speed), float(permit.max_speed_mps))
        dwa_core.OBSTACLE_FLOOR_M = max(
            float(dwa_core.OBSTACLE_FLOOR_M),
            float(permit.min_clearance_m))
        # Gate failures are checked again from the current raw cloud every
        # cycle. Never blacklist a yaw for the lifetime of a track: an arc
        # that was blocked one frame can become safe as the chair advances.
        self.planner.rejected_yaw_rates = ()
        self.gate_reason = ""
        self.gate_blocked_since = None
        self.gate_detail = detail

    def inactive_permit(self, now, reason):
        return self.qualifier.inactive(now.to_sec(), reason)

    def observed_person_permit(self, now):
        """Update qualification even while the motion service is paused.

        The base follower returns from its hold ladder before asking
        ``avoidance_for`` when it is paused. If qualification lived only in
        that method, a person already standing in front of the chair would
        make ``go`` impossible forever: the permit needs motion to start and
        semantic preflight needs the permit before motion may start. Reading
        perception here breaks that cycle without sending any command.
        """
        # Qualification is about the observed PERSON, not whichever object
        # wins the narrow forward-corridor query this cycle. On 2026-08-28
        # the 0.55 m follower query missed a static person seen by the 0.65 m
        # semantic stop query for ~21 s. It also reset a qualified person
        # when an unrelated, nearer object appeared. Use the full maneuver
        # region here; the qualifier still rejects moving/unknown, missing,
        # multiple, stale, changed-ID and too-close observations.
        observations = person_observations(
            self.cluster_summary,
            maximum_forward_m=self.person_bypass_observation_forward_m,
            maximum_lateral_m=(
                self.person_bypass_maximum_lateral_m
                + self.person_bypass_lateral_hysteresis_m),
        )
        permit = self.qualifier.update(
            observations, now.to_sec(), self.tracking_state == "TRACKING")
        if not permit.active:
            self.reset_bypass_commitment()
        return permit

    def avoidance_for(self, now, threat, blocking):
        ordinary = super(PersonBypassDwaFollower, self).avoidance_for(
            now, threat, blocking)
        permit = self.observed_person_permit(now)
        if permit.reason != "NO_PERSON":
            # Never replace a person's qualification with a generic object
            # permit. A closer object may require WAIT, but is not evidence
            # that this directly observed person moved or disappeared.
            self.publish_permit(permit)
            if not permit.active:
                self.reset_bypass_commitment()
                # Qualification may start in the wider observation region.
                # Outside the dynamic braking envelope, keep approaching;
                # the raw/semantic collision layers remain fully active.
                # Once threat_blocks says the chair must stop, WAIT remains
                # mandatory until the same-track static permit is active.
                if not blocking and (
                        threat is None or threat.is_person):
                    return ordinary if ordinary != WAIT else CLEAR
                return WAIT
            if threat is not None and not threat.is_person and (
                    ordinary == WAIT or not threat.parked):
                return WAIT
            if permit.reason == "STATIC_PERSON_PASSED_SIDE" and threat is None:
                # Keep the same maneuver active while the person is beside
                # the chair. Straight is now a valid candidate when its real
                # swept footprint is clear, but an abrupt opposite turn back
                # to the centreline is withheld until the person has left the
                # side region. This removes the observed S-shaped snap-back.
                self.activate_trajectory_bypass(
                    permit, "static person safely passing down the side")
                return GO_ROUND
            self.activate_trajectory_bypass(
                permit, "static-person trajectory permit")
            return GO_ROUND

        # A remembered person is sufficient to stop, never to authorize an
        # arc. NO_PERSON has already reset the qualifier above.
        if threat is not None and threat.is_person:
            self.publish_permit(permit)
            self.reset_bypass_commitment()
            # The base decision already applies the dynamic braking radius:
            # CLEAR outside it and WAIT inside it.  Replacing that result with
            # unconditional WAIT caused the 2026-08-30 stop at 8.3 m.
            # Do not, however, accept a remembered PERSON_BYPASS result after
            # direct qualification disappeared at collision distance.
            return WAIT if blocking else ordinary
        if threat is None or ordinary != GO_ROUND:
            self.reset_bypass_commitment()
            self.publish_permit(permit)
            return ordinary
        permit = static_obstacle_permit(
            now_s=now.to_sec(),
            observed_stamp_s=threat.observed_stamp_s,
            track_id=threat.track_id,
            target_x_m=threat.distance_m,
            target_y_m=threat.lateral_m,
            motion=threat.motion,
            directly_observed=threat.directly_observed,
            geometry_valid=threat.geometry_valid,
            maximum_observation_age_s=self.person_bypass_maximum_gap_s,
            permit_lifetime_s=self.person_bypass_permit_lifetime_s,
            max_speed_mps=self.person_bypass_speed_mps,
            min_clearance_m=self.person_bypass_clearance_m,
        )
        self.publish_permit(permit)
        if permit.active:
            self.activate_trajectory_bypass(
                permit, "static-object trajectory permit")
        else:
            self.reset_bypass_commitment()
        return ordinary

    def planner_candidate_veto(self, now, _decision, command_for_target):
        if self.active_trajectory_permit is None:
            return None
        raw_veto = make_raw_gate_candidate_veto(
            self.cloud, self.motion,
            (now - self.cloud_stamp).to_sec(), command_for_target,
            minimum_turn_rps=self.minimum_person_bypass_turn_rps,
            now_s=now.to_sec())
        committed_side = self._committed_bypass_side

        def veto(target_v, target_w):
            # Zero yaw is intentionally allowed: if its exact raw sweep is
            # clear, continuing forward is safer and smoother than forcing a
            # turn. Only a turn toward the opposite side is suppressed.
            if committed_side and float(target_w) * committed_side < -1e-6:
                return True
            return raw_veto(target_v, target_w)

        return veto

    def step(self):
        self.active_trajectory_permit = None
        self._permit_published_this_cycle = False
        saved_max_speed = float(self.planner.max_speed)
        saved_clearance = float(dwa_core.OBSTACLE_FLOOR_M)
        saved_rejected_yaws = getattr(
            self.planner, "rejected_yaw_rates", ())
        now = rospy.Time.now()
        # Publish a continuously refreshed qualification heartbeat before the
        # base hold ladder can return for PAUSED/MANUAL/STARTUP. This does not
        # bypass any guard; it only lets the later preflight distinguish a
        # stable person from a moving or unknown one before enabling motion.
        self.publish_permit(self.observed_person_permit(now))
        try:
            super(PersonBypassDwaFollower, self).step()
        finally:
            self.planner.max_speed = saved_max_speed
            dwa_core.OBSTACLE_FLOOR_M = saved_clearance
            self.planner.rejected_yaw_rates = saved_rejected_yaws
            if not self._permit_published_this_cycle:
                if self.tracking_state != "TRACKING":
                    self.qualifier.reset()
                self.publish_permit(self.inactive_permit(
                    now, "FOLLOWER_NOT_EVALUATING_PERSON"))


if __name__ == "__main__":
    try:
        PersonBypassDwaFollower().run()
    except GpuRequiredError as error:
        rospy.logfatal("required RTX DWA backend failed: %s", error)
        raise SystemExit(2)
    except rospy.ROSInterruptException:
        pass
