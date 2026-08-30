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
from dataclasses import replace

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
    BYPASS_SIDE_LOOKAHEAD_WAYPOINTS = 28  # about 5 m on the v9 route

    def __init__(self):
        self._committed_bypass_track_id = None
        self._committed_bypass_side = 0
        self._committed_bypass_kind = None
        self._static_object_track_id = None
        self._static_object_world_xy = None
        self._static_object_half_forward_m = 0.0
        self._static_object_half_lateral_m = 0.0
        self._static_object_passed_side = False
        self._static_object_commit_until_s = 0.0
        self._post_pass_track_id = None
        self._post_pass_origin_xy = None
        self._maneuver_track_previous_cycle = None
        self._maneuver_track_this_cycle = None
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
        self.static_object_commit_grace_s = float(rospy.get_param(
            "~static_object_commit_grace_s", 1.5))
        self.static_object_reacquire_radius_m = float(rospy.get_param(
            "~static_object_reacquire_radius_m", 1.5))
        self.static_object_slowdown_distance_m = float(rospy.get_param(
            "~static_object_slowdown_distance_m", 3.0))
        self.person_bypass_clearance_m = float(rospy.get_param(
            "~person_bypass_clearance_m", 0.50))
        self.post_pass_straight_distance_m = float(rospy.get_param(
            "~post_pass_straight_distance_m", 0.80))
        self.post_pass_straight_yaw_rate = float(rospy.get_param(
            "~post_pass_straight_yaw_rate", 0.05))
        self.post_pass_recovery_yaw_rate = float(rospy.get_param(
            "~post_pass_recovery_yaw_rate", 0.08))
        self.post_pass_alignment_lateral_m = float(rospy.get_param(
            "~post_pass_alignment_lateral_m", 0.25))
        self.post_pass_alignment_heading_rad = float(rospy.get_param(
            "~post_pass_alignment_heading_rad", 0.12))
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

    def static_object_commitment_active(self, now_s=None):
        if getattr(self, "_committed_bypass_kind", None) != "object":
            return False
        if now_s is None:
            try:
                now_s = rospy.Time.now().to_sec()
            except (AttributeError, TypeError):
                return False
        try:
            return float(now_s) <= float(
                getattr(self, "_static_object_commit_until_s", 0.0))
        except (TypeError, ValueError):
            return False

    def reset_bypass_commitment(self, force=False, now_s=None):
        # A geometric bicycle changed STATIC/UNKNOWN/missing four times in
        # five seconds during the 02:06 trial.  Forgetting the chosen side on
        # every one-frame dropout let ordinary route scoring reverse the
        # steering.  Remember only the direction (never an authorization to
        # move) for a short bounded grace period.  Raw sweeps and WAIT remain
        # fully binding while the fresh permit is absent.
        if not force and self.static_object_commitment_active(now_s):
            return
        self._committed_bypass_track_id = None
        self._committed_bypass_side = 0
        self._committed_bypass_kind = None
        self._static_object_track_id = None
        self._static_object_world_xy = None
        self._static_object_half_forward_m = 0.0
        self._static_object_half_lateral_m = 0.0
        self._static_object_passed_side = False
        self._static_object_commit_until_s = 0.0

    def reset_post_pass_recovery(self):
        self._post_pass_track_id = None
        self._post_pass_origin_xy = None

    def remember_maneuver_cycle(self, track_id):
        if track_id is not None:
            self._maneuver_track_this_cycle = track_id

    def route_alignment_error(self):
        """Distance and heading error to the ideal route at the chair."""
        pose = getattr(self, "pose_xy", None)
        yaw = getattr(self, "pose_yaw", None)
        planner = getattr(self, "planner", None)
        if pose is None or yaw is None or planner is None:
            return None
        try:
            distance, index = planner.tree.query(pose)
            heading = float(planner.heading[int(index)])
            yaw = float(yaw)
            distance = float(distance)
        except (AttributeError, IndexError, TypeError, ValueError):
            return None
        heading_error = abs(math.atan2(
            math.sin(yaw - heading), math.cos(yaw - heading)))
        if not math.isfinite(distance) or not math.isfinite(heading_error):
            return None
        return distance, heading_error

    def latch_post_pass_recovery(self, permit):
        """Remember where a safe side-pass completed.

        Losing the cluster is expected just after a pass.  It must not switch
        directly from the low-speed straight pass to unrestricted route
        recovery: both recorded passes on 2026-08-30 jumped from w=0 to the
        +0.50 rad/s limit as soon as the PERSON track disappeared.  Distance
        from this map-frame origin keeps the recovery independent of cluster
        flicker and chair rotation.
        """
        pose = getattr(self, "pose_xy", None)
        track_id = getattr(permit, "track_id", None)
        self.latch_post_pass_track(track_id, pose)

    def latch_post_pass_track(self, track_id, pose=None):
        """Start gentle recovery for one completed/interrupted maneuver."""
        if pose is None:
            pose = getattr(self, "pose_xy", None)
        if pose is None or track_id is None:
            return
        try:
            origin = (float(pose[0]), float(pose[1]))
        except (IndexError, TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in origin):
            return
        if self._post_pass_track_id != track_id or \
                self._post_pass_origin_xy is None:
            self._post_pass_track_id = track_id
            self._post_pass_origin_xy = origin

    def finish_maneuver_cycle(self):
        previous_track = getattr(self, "_maneuver_track_previous_cycle", None)
        current_track = getattr(self, "_maneuver_track_this_cycle", None)
        if previous_track is not None and current_track is None and \
                abs(float(getattr(self, "last_yaw_rate", 0.0))) >= 0.05 and \
                not self.static_object_commitment_active():
            # A qualified maneuver disappeared before PASSED_SIDE.  The
            # 00:01 trial lost track 1689 during chair rotation, then the
            # ordinary route cost selected +0.50 rad/s.  Enter the same
            # gentle recovery used after a clean pass; semantic and raw
            # gates still stop rather than authorizing an unseen person.
            self.latch_post_pass_track(previous_track)
        self._maneuver_track_previous_cycle = current_track

    def post_pass_distance_m(self):
        pose = getattr(self, "pose_xy", None)
        origin = getattr(self, "_post_pass_origin_xy", None)
        if pose is None or origin is None:
            return None
        try:
            distance = math.hypot(
                float(pose[0]) - float(origin[0]),
                float(pose[1]) - float(origin[1]))
        except (IndexError, TypeError, ValueError):
            return None
        return distance if math.isfinite(distance) else None

    def post_pass_yaw_cap(self):
        """Return the gentle-rejoin yaw cap until pose and heading align."""
        distance = self.post_pass_distance_m()
        if distance is None:
            return None
        straight_distance = max(
            float(getattr(self, "post_pass_straight_distance_m", .8)), 0.0)
        if distance < straight_distance:
            return max(float(getattr(
                self, "post_pass_straight_yaw_rate", .05)), 0.0)
        alignment = self.route_alignment_error()
        if alignment is not None and \
                alignment[0] <= float(getattr(
                    self, "post_pass_alignment_lateral_m", .25)) and \
                alignment[1] <= float(getattr(
                    self, "post_pass_alignment_heading_rad", .12)):
            self.reset_post_pass_recovery()
            return None
        return max(float(getattr(
            self, "post_pass_recovery_yaw_rate", .08)), 0.0)

    def post_pass_recovery_active(self):
        return self.post_pass_yaw_cap() is not None

    def update_static_object_pass(self, threat):
        """Track a confirmed-static object in map coordinates.

        The live cluster can appear to stay ahead while the chair turns.
        Anchoring its first confirmed-static geometry in the map applies ego
        motion exactly once and provides the side-pass state that a semantic
        PERSON track already has. Raw LiDAR remains the final authority.
        """
        pose = getattr(self, "pose_xy", None)
        yaw = getattr(self, "pose_yaw", None)
        track_id = getattr(threat, "track_id", None)
        lateral = getattr(threat, "lateral_m", None)
        if pose is None or yaw is None or track_id is None or lateral is None:
            return False
        try:
            yaw = float(yaw)
            centre_x = getattr(threat, "centre_forward_m", None)
            half_x = getattr(threat, "half_forward_m", None)
            half_y = getattr(threat, "half_lateral_m", None)
            half_x = 0.0 if half_x is None else max(0.0, float(half_x))
            half_y = 0.0 if half_y is None else max(0.0, float(half_y))
            centre_x = (float(threat.distance_m) + half_x
                        if centre_x is None else float(centre_x))
            lateral = float(lateral)
            px, py = float(pose[0]), float(pose[1])
        except (IndexError, TypeError, ValueError):
            return False
        values = (yaw, centre_x, lateral, half_x, half_y, px, py)
        if not all(math.isfinite(value) for value in values):
            return False
        cosine, sine = math.cos(yaw), math.sin(yaw)
        observed_world_xy = (
            px + cosine * centre_x - sine * lateral,
            py + sine * centre_x + cosine * lateral)
        same_world_object = False
        if self._static_object_world_xy is not None and \
                self.static_object_commitment_active():
            same_world_object = math.hypot(
                observed_world_xy[0] - self._static_object_world_xy[0],
                observed_world_xy[1] - self._static_object_world_xy[1],
            ) <= max(float(getattr(
                self, "static_object_reacquire_radius_m", 1.5)), 0.0)
        if self._static_object_track_id != track_id or \
                self._static_object_world_xy is None:
            self._static_object_track_id = track_id
            if same_world_object:
                # The geometric tracker may mint a new ID while the chair is
                # rotating. Keep the first map anchor and selected side when
                # the replacement cluster lands on the same physical object.
                self._committed_bypass_track_id = track_id
                self._static_object_half_forward_m = max(
                    self._static_object_half_forward_m, half_x)
                self._static_object_half_lateral_m = max(
                    self._static_object_half_lateral_m, half_y)
            else:
                # A genuinely different object must earn a fresh side choice;
                # never transfer a remembered turn merely because it appeared
                # inside the time grace.
                self._committed_bypass_track_id = None
                self._committed_bypass_side = 0
                self._committed_bypass_kind = None
                self._static_object_commit_until_s = 0.0
                self._static_object_world_xy = observed_world_xy
                self._static_object_half_forward_m = half_x
                self._static_object_half_lateral_m = half_y
                self._static_object_passed_side = False

        dx = self._static_object_world_xy[0] - px
        dy = self._static_object_world_xy[1] - py
        cosine, sine = math.cos(yaw), math.sin(yaw)
        relative_x = cosine * dx + sine * dy
        relative_y = -sine * dx + cosine * dy
        near_forward = relative_x - self._static_object_half_forward_m
        lateral_clearance = (
            abs(relative_y) - self._static_object_half_lateral_m)
        if near_forward < self.person_bypass_minimum_near_m and \
                lateral_clearance >= self.person_bypass_clearance_m:
            self._static_object_passed_side = True
        return self._static_object_passed_side

    def static_object_permit_speed_mps(self, threat):
        """Keep cruise speed until a static object enters the near zone.

        The raw gate and the ordinary approach cap still own braking.  This
        only avoids applying the person's 0.35 m/s close-pass speed to a
        bicycle five to eight metres away, which repeatedly drained speed
        during geometric track flicker.
        """
        try:
            distance_m = float(threat.distance_m)
            cruise_mps = float(self.planner.max_speed)
        except (AttributeError, TypeError, ValueError):
            return float(self.person_bypass_speed_mps)
        if not math.isfinite(distance_m) or not math.isfinite(cruise_mps):
            return float(self.person_bypass_speed_mps)
        if distance_m <= max(float(
                getattr(self, "static_object_slowdown_distance_m", 3.0)), 0.0):
            return float(self.person_bypass_speed_mps)
        return max(float(self.person_bypass_speed_mps), cruise_mps)

    def planner_excluded_track_ids(self):
        # Exclude only after ego-compensated safe-side confirmation. The raw
        # gate still receives the complete point cloud and vetoes every real
        # collision, while DWA stops re-planning around an object already
        # safely down the side.
        excluded = []
        if self._static_object_passed_side and \
                self._static_object_track_id is not None:
            excluded.append(self._static_object_track_id)
        if self.post_pass_recovery_active() and \
                self._post_pass_track_id is not None and \
                self._post_pass_track_id not in excluded:
            excluded.append(self._post_pass_track_id)
        return tuple(excluded)

    @staticmethod
    def room_preferred_side(lo_m, hi_m, current_lateral_m,
                            target_y_m, clearance_m):
        """Choose the side with more v9 room after clearing the obstacle.

        Limits are chair-centre offsets from the recorded route.  The target
        is in the current chair frame, so translate the two pass positions by
        the chair's current cross-track offset before comparing their slack.
        Positive is left and negative is right, matching ROS yaw.
        """
        values = (lo_m, hi_m, current_lateral_m, target_y_m, clearance_m)
        if not all(math.isfinite(float(value)) for value in values):
            return 0
        left_pass_m = (float(current_lateral_m) + float(target_y_m)
                       + float(clearance_m))
        right_pass_m = (float(current_lateral_m) + float(target_y_m)
                        - float(clearance_m))
        left_slack_m = float(hi_m) - left_pass_m
        right_slack_m = right_pass_m - float(lo_m)
        if left_slack_m < 0.0 and right_slack_m < 0.0:
            return 0
        return 1 if left_slack_m >= right_slack_m else -1

    def v9_room_preferred_side(self, permit):
        """Use the restrictive current+forward v9 widths for side choice."""
        pose_xy = getattr(self, "pose_xy", None)
        band = getattr(self, "band", None)
        waypoints = getattr(self, "waypoints", ())
        if pose_xy is None or band is None or not len(waypoints) or \
                permit.target_y_m is None:
            return 0
        start = max(0, int(getattr(self, "nearest_index", 0)))
        end = min(len(waypoints),
                  start + self.BYPASS_SIDE_LOOKAHEAD_WAYPOINTS)
        points = [pose_xy]
        # A few forward samples are enough; lateral_limits itself applies the
        # more restrictive pair of v9 stations at every sample.
        points.extend(waypoints[start:end:4])
        limits = []
        for point in points:
            try:
                lateral, lo, hi = band.lateral_limits(point)
            except (AttributeError, TypeError, ValueError):
                return 0
            if not all(math.isfinite(float(value))
                       for value in (lateral, lo, hi)):
                return 0
            limits.append((float(lateral), float(lo), float(hi)))
        if not limits:
            return 0
        current_lateral = limits[0][0]
        # A side is only as wide as its narrowest point over the maneuver.
        lo = max(value[1] for value in limits)
        hi = min(value[2] for value in limits)
        return self.room_preferred_side(
            lo, hi, current_lateral, float(permit.target_y_m),
            float(permit.min_clearance_m))

    def activate_trajectory_bypass(self, permit, detail):
        self.active_trajectory_permit = permit
        self.remember_maneuver_cycle(getattr(permit, "track_id", None))
        reason = str(getattr(permit, "reason", ""))
        object_permit = reason.startswith("STATIC_OBJECT_")
        if reason.endswith("_PASSED_SIDE"):
            self.latch_post_pass_recovery(permit)
        carry_object_side = (
            object_permit
            and self.static_object_commitment_active(
                getattr(permit, "stamp_s", None))
            and self._committed_bypass_side != 0
            and self._static_object_track_id == permit.track_id
        )
        if self._committed_bypass_track_id != permit.track_id and \
                not carry_object_side:
            # Prefer the side with more usable v9 width over the next ~3 m.
            # The previous target-opposite rule sent the 23:01 bicycle trial
            # into the 0.80 m right side while v9 exposed 2.55 m on the left.
            # Raw point sweeps and the hard route mask still veto collisions.
            lateral = float(permit.target_y_m or 0.0)
            preferred = self.v9_room_preferred_side(permit)
            if preferred:
                self._committed_bypass_side = preferred
            elif abs(lateral) > 1e-3:
                self._committed_bypass_side = -1 if lateral > 0.0 else 1
            elif abs(getattr(self, "last_yaw_rate", 0.0)) > 1e-3:
                self._committed_bypass_side = (
                    1 if self.last_yaw_rate > 0.0 else -1)
            else:
                self._committed_bypass_side = 1
            self._committed_bypass_track_id = permit.track_id
        elif carry_object_side:
            self._committed_bypass_track_id = permit.track_id
        self._committed_bypass_kind = "object" if object_permit else "person"
        if object_permit:
            self._static_object_commit_until_s = (
                float(permit.stamp_s)
                + max(float(getattr(
                    self, "static_object_commit_grace_s", 1.5)), 0.0))
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

    def ego_compensated_person_observations(self, observations):
        """Attach map-frame identity coordinates to body-frame detections.

        The permit still carries current body-frame x/y for planning.  Only
        the same-track jump check uses these map coordinates, so rotating the
        chair no longer makes one stationary person appear to teleport from
        y=-0.2 to y=+1.2 m and erase an active bypass.
        """
        pose = getattr(self, "pose_xy", None)
        yaw = getattr(self, "pose_yaw", None)
        if pose is None or yaw is None:
            return observations
        try:
            px, py = float(pose[0]), float(pose[1])
            yaw = float(yaw)
        except (IndexError, TypeError, ValueError):
            return observations
        if not all(math.isfinite(value) for value in (px, py, yaw)):
            return observations
        cosine, sine = math.cos(yaw), math.sin(yaw)
        compensated = []
        for observation in observations:
            world_x = px + cosine * observation.x_m - sine * observation.y_m
            world_y = py + sine * observation.x_m + cosine * observation.y_m
            compensated.append(replace(
                observation, tracking_x_m=world_x, tracking_y_m=world_y))
        return tuple(compensated)

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
        observations = self.ego_compensated_person_observations(observations)
        permit = self.qualifier.update(
            observations, now.to_sec(), self.tracking_state == "TRACKING")
        if permit.active and \
                str(getattr(permit, "reason", "")).endswith("_PASSED_SIDE"):
            self.latch_post_pass_recovery(permit)
        if not permit.active and permit.reason != "NO_PERSON":
            self.reset_bypass_commitment()
        return permit

    def avoidance_for(self, now, threat, blocking):
        if self.post_pass_recovery_active():
            self.planner.max_speed = min(
                float(self.planner.max_speed),
                float(self.person_bypass_speed_mps))
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
            max_speed_mps=self.static_object_permit_speed_mps(threat),
            min_clearance_m=self.person_bypass_clearance_m,
            permit_reason=(
                "STATIC_OBJECT_PASSED_SIDE"
                if self.update_static_object_pass(threat)
                else "STATIC_OBJECT_BYPASS"),
        )
        self.publish_permit(permit)
        if permit.active:
            self.activate_trajectory_bypass(
                permit, "static-object trajectory permit")
        else:
            self.reset_bypass_commitment()
        return ordinary

    def planner_candidate_veto(self, now, _decision, command_for_target):
        recovery_yaw_cap = self.post_pass_yaw_cap()
        object_direction_grace = self.static_object_commitment_active(
            now.to_sec())
        if self.active_trajectory_permit is None and \
                recovery_yaw_cap is None and not object_direction_grace:
            return None
        raw_veto = None
        committed_side = 0
        if self.active_trajectory_permit is not None:
            raw_veto = make_raw_gate_candidate_veto(
                self.cloud, self.motion,
                (now - self.cloud_stamp).to_sec(), command_for_target,
                minimum_turn_rps=self.minimum_person_bypass_turn_rps,
                now_s=now.to_sec())
            committed_side = self._committed_bypass_side
        elif object_direction_grace:
            # Suppress only an immediate reversal. Straight and the already
            # chosen side remain available, while all normal route-mask and
            # downstream raw-gate checks stay in force.
            committed_side = self._committed_bypass_side

        def veto(target_v, target_w):
            if recovery_yaw_cap is not None and \
                    abs(float(target_w)) > recovery_yaw_cap + 1e-6:
                return True
            # Zero yaw is intentionally allowed: if its exact raw sweep is
            # clear, continuing forward is safer and smoother than forcing a
            # turn. Only a turn toward the opposite side is suppressed.
            if committed_side and float(target_w) * committed_side < -1e-6:
                return True
            return raw_veto is not None and raw_veto(target_v, target_w)

        return veto

    def step(self):
        self.active_trajectory_permit = None
        self._maneuver_track_this_cycle = None
        self._permit_published_this_cycle = False
        saved_max_speed = float(self.planner.max_speed)
        saved_clearance = float(dwa_core.OBSTACLE_FLOOR_M)
        saved_rejected_yaws = getattr(
            self.planner, "rejected_yaw_rates", ())
        now = rospy.Time.now()
        if not getattr(self, "enabled", False) or (
                getattr(self, "drive_mode", None) not in (None, 65)):
            self.reset_post_pass_recovery()
            self.reset_bypass_commitment(force=True)
        # Publish person evidence before the base hold ladder can return for
        # PAUSED/MANUAL/STARTUP.  Do not publish the ordinary NO_PERSON value
        # here: on an active cycle avoidance_for may issue a generic
        # STATIC_OBJECT_BYPASS a few milliseconds later. Publishing both made
        # the raw gate see an alternating inactive/active permit and broke
        # bypass of static bicycles and other unclassified geometry. The
        # finally block still publishes a fail-closed inactive heartbeat when
        # the follower returns before evaluating an object.
        person_permit = self.observed_person_permit(now)
        if person_permit.reason != "NO_PERSON":
            self.publish_permit(person_permit)
        try:
            super(PersonBypassDwaFollower, self).step()
        finally:
            self.finish_maneuver_cycle()
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
