#!/usr/bin/env python3
"""RTX DWA follower that conditionally passes a continuously static person.

The stock hybrid follower intentionally waits for every person. This wrapper
changes only that policy transition: after direct same-track STATIC evidence,
DWA may plan around exactly one person at the turn-speed floor. Moving,
unknown, learned-only, too-close, multiple, stale, or geometrically invalid
people remain stop-only.
"""

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
from dwa_follower import DwaFollower  # noqa: E402
from person_bypass_policy import (  # noqa: E402
    StaticPersonQualifier,
    person_observations,
)


class PersonBypassDwaFollower(DwaFollower):
    CONTROL_LAW = "dwa"

    def __init__(self):
        super(PersonBypassDwaFollower, self).__init__()
        self.person_bypass_confirmation_s = float(rospy.get_param(
            "~person_bypass_confirmation_s", 3.0))
        self.person_bypass_maximum_gap_s = float(rospy.get_param(
            "~person_bypass_maximum_gap_s", 0.35))
        self.person_bypass_position_jump_m = float(rospy.get_param(
            "~person_bypass_position_jump_m", 0.35))
        self.person_bypass_permit_lifetime_s = float(rospy.get_param(
            "~person_bypass_permit_lifetime_s", 0.45))
        self.person_bypass_maximum_forward_m = float(rospy.get_param(
            "~person_bypass_maximum_forward_m", 8.0))
        self.person_bypass_maximum_lateral_m = float(rospy.get_param(
            "~person_bypass_maximum_lateral_m", 1.0))
        self.person_bypass_minimum_near_m = float(rospy.get_param(
            "~person_bypass_minimum_near_m", 0.60))
        self.person_bypass_speed_mps = float(rospy.get_param(
            "~person_bypass_speed_mps", 0.35))
        self.person_bypass_clearance_m = float(rospy.get_param(
            "~person_bypass_clearance_m", 0.80))
        self.qualifier = StaticPersonQualifier(
            confirmation_s=self.person_bypass_confirmation_s,
            maximum_gap_s=self.person_bypass_maximum_gap_s,
            maximum_position_jump_m=self.person_bypass_position_jump_m,
            permit_lifetime_s=self.person_bypass_permit_lifetime_s,
            maximum_forward_m=self.person_bypass_maximum_forward_m,
            maximum_lateral_m=self.person_bypass_maximum_lateral_m,
            minimum_near_distance_m=self.person_bypass_minimum_near_m,
            max_speed_mps=self.person_bypass_speed_mps,
            min_clearance_m=self.person_bypass_clearance_m,
        )
        self.permit_pub = rospy.Publisher(
            "/person_bypass/permit", String, queue_size=1, latch=False)
        self._permit_published_this_cycle = False
        rospy.set_param("~person_bypass_capable", True)
        rospy.loginfo(
            "stationary-person bypass: %.1f s same-track STATIC, "
            "v<=%.2f m/s, clearance>=%.2f m",
            self.person_bypass_confirmation_s,
            self.person_bypass_speed_mps,
            self.person_bypass_clearance_m)

    def publish_permit(self, permit):
        self.permit_pub.publish(String(data=permit.to_json()))
        self._permit_published_this_cycle = True

    def inactive_permit(self, now, reason):
        return self.qualifier.inactive(now.to_sec(), reason)

    def avoidance_for(self, now, threat, blocking):
        ordinary = super(PersonBypassDwaFollower, self).avoidance_for(
            now, threat, blocking)
        if threat is None or not threat.is_person:
            self.qualifier.reset()
            self.publish_permit(self.inactive_permit(
                now, "NEAREST_THREAT_NOT_PERSON"))
            return ordinary

        observations = person_observations(
            self.cluster_summary,
            maximum_forward_m=self.person_bypass_maximum_forward_m,
            maximum_lateral_m=self.person_bypass_maximum_lateral_m,
        )
        permit = self.qualifier.update(
            observations, now.to_sec(), self.tracking_state == "TRACKING")
        self.publish_permit(permit)
        if not permit.active:
            return ordinary

        # DWA-only authorization. The semantic and raw trajectory gates both
        # consume the same short-lived permit; neither can infer authorization
        # from a class label or from this return value alone.
        self.planner.max_speed = min(
            float(self.planner.max_speed), float(permit.max_speed_mps))
        dwa_core.OBSTACLE_FLOOR_M = max(
            float(dwa_core.OBSTACLE_FLOOR_M),
            float(permit.min_clearance_m))

        # The base GATE_STALL diagnostic is for an obstacle absent from planner
        # geometry. This person is present in geometry and the trajectory
        # gate is now the authority, so the old fixed-corridor reason must not
        # pre-empt the DWA cycle before a curved proposal exists.
        self.gate_reason = ""
        self.gate_blocked_since = None
        self.gate_detail = "static-person trajectory permit"
        return GO_ROUND

    def step(self):
        self._permit_published_this_cycle = False
        saved_max_speed = float(self.planner.max_speed)
        saved_clearance = float(dwa_core.OBSTACLE_FLOOR_M)
        now = rospy.Time.now()
        try:
            super(PersonBypassDwaFollower, self).step()
        finally:
            self.planner.max_speed = saved_max_speed
            dwa_core.OBSTACLE_FLOOR_M = saved_clearance
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
