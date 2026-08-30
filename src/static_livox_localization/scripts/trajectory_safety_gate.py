#!/usr/bin/env python3
"""Raw LiDAR gate for an exact planner-selected static-threat trajectory.

Only ``OBSTACLE`` and ``OBSTACLE_SWEEP`` may be replaced, and only when a
fresh static-threat permit, monotonic body-frame trajectory proposal, committed
pass side, and incoming command all match. The validator checks the selected
actuator-ramped poses against the one collision snapshot built by the base
gate. Current-footprint, carried-path, selected-path, stale-health, malformed,
and identity mismatches remain stops.
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

import safety_gate as base_gate  # noqa: E402
import person_bypass_policy as bypass_policy  # noqa: E402
import trajectory_proposal as proposal_contract  # noqa: E402
from motion_safety import footprint_collision_at_pose  # noqa: E402


BYPASS_SIDE_MARGIN_M = 0.05


def proposal_path_collision(obstacles, poses, *, margin_m,
                            pose_checked=None):
    radius_m = math.hypot(
        max(base_gate.FOOTPRINT_FRONT_M, base_gate.FOOTPRINT_REAR_M)
        + margin_m,
        base_gate.FOOTPRINT_HALF_WIDTH_M + margin_m)
    validation_poses = []
    previous = (0.0, 0.0, 0.0)
    for pose in poses:
        boundary_distance_m = math.hypot(
            pose[0] - previous[0], pose[1] - previous[1]) + \
            abs(pose[2] - previous[2]) * radius_m
        steps = max(1, int(math.ceil(boundary_distance_m / 0.02)))
        for index in range(1, steps + 1):
            fraction = float(index) / float(steps)
            validation_poses.append(tuple(
                previous[axis] + fraction * (pose[axis] - previous[axis])
                for axis in range(3)))
        previous = pose
    for pose_x_m, pose_y_m, pose_yaw_rad in validation_poses:
        if pose_checked is not None:
            pose_checked()
        if footprint_collision_at_pose(
                obstacles,
                pose_x_m=pose_x_m,
                pose_y_m=pose_y_m,
                pose_yaw_rad=pose_yaw_rad,
                front_m=base_gate.FOOTPRINT_FRONT_M,
                rear_m=base_gate.FOOTPRINT_REAR_M,
                half_width_m=base_gate.FOOTPRINT_HALF_WIDTH_M,
                margin_m=margin_m):
            return True
    return False


def current_footprint_points(obstacles, extra_front_m, extra_side_m):
    front = (base_gate.FOOTPRINT_FRONT_M + base_gate.SWEEP_MARGIN_M
             + float(extra_front_m))
    side = (base_gate.FOOTPRINT_HALF_WIDTH_M + BYPASS_SIDE_MARGIN_M
            + float(extra_side_m))
    return obstacles[
        (obstacles[:, 0] >= -base_gate.FOOTPRINT_REAR_M
         - base_gate.SWEEP_MARGIN_M) &
        (obstacles[:, 0] <= front) &
        (np.abs(obstacles[:, 1]) <= side)
    ] if len(obstacles) else obstacles


def bypass_swept_footprint_collision(
        obstacles, linear_speed_mps, angular_speed_rps, horizon_s,
        pose_checked=None):
    longitudinal_padding = (
        base_gate.SWEEP_MARGIN_M - BYPASS_SIDE_MARGIN_M)
    rear = obstacles[
        obstacles[:, 0] < -base_gate.FOOTPRINT_REAR_M]
    forward = obstacles[
        obstacles[:, 0] >= -base_gate.FOOTPRINT_REAR_M]
    forward_collision = base_gate.swept_footprint_collision(
        forward,
        linear_speed_mps=linear_speed_mps,
        angular_speed_rps=angular_speed_rps,
        horizon_s=horizon_s,
        front_m=base_gate.FOOTPRINT_FRONT_M + longitudinal_padding,
        rear_m=base_gate.FOOTPRINT_REAR_M + longitudinal_padding,
        half_width_m=base_gate.FOOTPRINT_HALF_WIDTH_M,
        margin_m=BYPASS_SIDE_MARGIN_M,
        pose_checked=pose_checked)
    if forward_collision:
        return True
    return base_gate.swept_footprint_collision(
        rear,
        linear_speed_mps=linear_speed_mps,
        angular_speed_rps=angular_speed_rps,
        horizon_s=horizon_s,
        front_m=base_gate.FOOTPRINT_FRONT_M,
        rear_m=base_gate.FOOTPRINT_REAR_M,
        half_width_m=base_gate.FOOTPRINT_HALF_WIDTH_M,
        margin_m=0.0,
        pose_checked=pose_checked)


class TrajectorySafetyGate(base_gate.SafetyGate):
    def __init__(self):
        super(TrajectorySafetyGate, self).__init__()
        self.person_bypass_permit = None
        self.maximum_permit_age_s = float(rospy.get_param(
            "~static_threat_bypass_maximum_permit_age_s", 0.45))
        self.minimum_bypass_turn_rps = float(rospy.get_param(
            "~static_threat_bypass_minimum_turn_rps", 0.08))
        self.maximum_proposal_age_s = float(rospy.get_param(
            "~maximum_static_threat_proposal_age_s", 0.30))
        self.proposal_linear_tolerance_mps = float(rospy.get_param(
            "~static_threat_proposal_linear_tolerance_mps", 0.02))
        self.proposal_angular_tolerance_rps = float(rospy.get_param(
            "~static_threat_proposal_angular_tolerance_rps", 0.03))
        self.proposal_buffer_size = int(rospy.get_param(
            "~static_threat_proposal_buffer_size", 8))
        # Zero by default: SWEEP_MARGIN_M already expands the measured chair
        # footprint by 0.15 m. Adding the previous extra 0.10 m recreated the
        # 0.75 m straight gate that made a valid curved bypass impossible.
        self.immediate_front_margin_m = float(rospy.get_param(
            "~static_threat_bypass_immediate_front_margin_m", 0.0))
        self.immediate_side_margin_m = float(rospy.get_param(
            "~static_threat_bypass_immediate_side_margin_m", 0.0))
        self.immediate_point_count = int(rospy.get_param(
            "~static_threat_bypass_immediate_point_count", 5))
        for name in (
                "maximum_permit_age_s", "minimum_bypass_turn_rps",
                "maximum_proposal_age_s", "proposal_linear_tolerance_mps",
                "proposal_angular_tolerance_rps"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise rospy.ROSInitException(
                    "~%s must be finite and positive" % name)
        for name in ("immediate_front_margin_m", "immediate_side_margin_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise rospy.ROSInitException(
                    "~%s must be finite and non-negative" % name)
        if self.immediate_point_count <= 0:
            raise rospy.ROSInitException(
                "~static_threat_bypass_immediate_point_count must be positive")
        if self.proposal_buffer_size <= 0:
            raise rospy.ROSInitException(
                "~static_threat_proposal_buffer_size must be positive")
        self.trajectory_proposals = []
        self.highest_proposal_seq = -1
        self.proposal_receive_reason = "NO_PROPOSAL"
        self.proposal_lock = threading.Lock()
        permit_topic = str(rospy.get_param(
            "~static_threat_bypass_permit_topic",
            "/static_threat_bypass/permit"))
        rospy.Subscriber(permit_topic, String, self.on_static_threat_permit,
                         queue_size=2)
        proposal_topic = str(rospy.get_param(
            "~static_threat_bypass_proposal_topic",
            "/static_threat_bypass/proposal"))
        rospy.Subscriber(proposal_topic, String, self.on_trajectory_proposal,
                         queue_size=2)
        self.last_override = None
        rospy.set_param("~static_threat_bypass_capable", True)
        rospy.set_param("~static_threat_bypass_proposal_capable", True)
        rospy.loginfo(
            "trajectory-aware raw gate enabled; permit=%s turn>=%.2f rad/s",
            permit_topic, self.minimum_bypass_turn_rps)

    def on_static_threat_permit(self, message):
        self.person_bypass_permit = bypass_policy.permit_from_payload(
            message.data)

    def on_trajectory_proposal(self, message):
        try:
            proposal = proposal_contract.TrajectoryProposal.from_json(
                message.data)
        except proposal_contract.ProposalValidationError:
            with self.proposal_lock:
                self.proposal_receive_reason = "PROPOSAL_MALFORMED"
            return
        with self.proposal_lock:
            if proposal.proposal_seq <= self.highest_proposal_seq:
                self.proposal_receive_reason = "PROPOSAL_SEQUENCE_STALE"
                return
            self.trajectory_proposals.append(proposal)
            self.trajectory_proposals = self.trajectory_proposals[
                -self.proposal_buffer_size:]
            self.highest_proposal_seq = proposal.proposal_seq
            self.proposal_receive_reason = "PROPOSAL_RECEIVED"

    def matching_proposal(self, now_s, permit):
        with self.proposal_lock:
            proposals = list(self.trajectory_proposals)
            highest_proposal_seq = self.highest_proposal_seq
            receive_reason = self.proposal_receive_reason
        if not proposals:
            return None, receive_reason
        if max(proposal.proposal_seq for proposal in proposals) < \
                highest_proposal_seq:
            return None, "PROPOSAL_SEQUENCE_STALE"
        fresh = []
        stale_seen = False
        for proposal in proposals:
            age_s = now_s - proposal.stamp_s
            if -0.05 <= age_s <= self.maximum_proposal_age_s:
                fresh.append(proposal)
            else:
                stale_seen = True
        if not fresh:
            return None, "PROPOSAL_STALE" if stale_seen else "NO_PROPOSAL"
        raw_speed = float(self.raw.linear.x)
        raw_yaw = float(self.raw.angular.z)
        raw_sequence_value = float(self.raw.angular.x)
        if not math.isfinite(raw_speed) or not math.isfinite(raw_yaw) or \
                not math.isfinite(raw_sequence_value):
            return None, "PROPOSAL_COMMAND_MISMATCH"
        if raw_sequence_value < 0.0 or not raw_sequence_value.is_integer():
            return None, "PROPOSAL_SEQUENCE_MISMATCH"
        raw_sequence = int(raw_sequence_value)
        mismatch_reason = None
        for proposal in sorted(
                fresh, key=lambda value: value.proposal_seq, reverse=True):
            if proposal.proposal_seq != raw_sequence:
                if mismatch_reason is None:
                    mismatch_reason = "PROPOSAL_SEQUENCE_MISMATCH"
                continue
            if proposal.permit_track_id != permit.track_id:
                if mismatch_reason is None:
                    mismatch_reason = "PROPOSAL_TRACK_MISMATCH"
                continue
            target_yaw = proposal.target_yaw_rate_rps
            if proposal.committed_side == "NONE" or \
                    not proposal_contract.yaw_matches_side(
                        proposal.committed_side, target_yaw):
                if mismatch_reason is None:
                    mismatch_reason = "PROPOSAL_SIDE_MISMATCH"
                continue
            if abs(raw_speed - proposal.first_applied_speed_mps) > \
                    self.proposal_linear_tolerance_mps or \
                    abs(raw_yaw - proposal.first_applied_yaw_rate_rps) > \
                    self.proposal_angular_tolerance_rps:
                if mismatch_reason is None:
                    mismatch_reason = "PROPOSAL_COMMAND_MISMATCH"
                continue
            return proposal, "PROPOSAL_MATCHED"
        return None, mismatch_reason or "NO_PROPOSAL"

    def fresh_active_permit(self, now_s):
        permit = self.person_bypass_permit
        if permit is None or not permit.active:
            return None
        return permit if bypass_policy.permit_is_fresh(
            permit, now_s, self.maximum_permit_age_s) else None

    def motion_blocked(self, now):
        reason, cap = super(TrajectorySafetyGate, self).motion_blocked(now)
        self.last_override = None
        self.evidence.update({
            "static_threat_target_behind": False,
            "static_threat_tail_clear": False,
        })
        if reason not in ("", "OBSTACLE", "OBSTACLE_SWEEP"):
            return reason, cap

        snapshot = getattr(self, "collision_snapshot", None)
        if snapshot is None:
            self.evidence["trajectory_override_reason"] = "SNAPSHOT_MISSING"
            return reason, cap
        obstacles = snapshot.points_xy

        def pose_checked():
            self.evidence["pose_checks"] = \
                self.evidence.get("pose_checks", 0) + 1

        now_s = now.to_sec()
        permit = self.fresh_active_permit(now_s)
        if permit is None:
            self.evidence["trajectory_override_reason"] = "NO_FRESH_PERMIT"
            return reason, cap

        proposal, proposal_reason = self.matching_proposal(now_s, permit)
        if proposal is None:
            self.evidence["trajectory_override_reason"] = proposal_reason
            return reason, cap
        proposal_age_s = now_s - proposal.stamp_s
        applied_speed = proposal.first_applied_speed_mps
        applied_yaw = proposal.first_applied_yaw_rate_rps
        requested_speed = max(0.0, min(
            base_gate.HARD_V_LIMIT, applied_speed,
            float(permit.max_speed_mps)))
        requested_yaw = max(-base_gate.HARD_W_LIMIT, min(
            base_gate.HARD_W_LIMIT, proposal.target_yaw_rate_rps))
        try:
            horizon_s = float(self.evidence["horizon_s"])
        except (KeyError, TypeError, ValueError):
            self.evidence["trajectory_override_reason"] = "HORIZON_MISSING"
            return reason, cap

        # Keep the full longitudinal reserve, but size lateral protection to
        # the measured sub-0.60 m chair width plus 0.05 m per side.
        immediate = current_footprint_points(
            obstacles,
            extra_front_m=self.immediate_front_margin_m,
            extra_side_m=self.immediate_side_margin_m)
        immediate_collision = len(immediate) >= self.immediate_point_count

        requested_collision = proposal_path_collision(
            obstacles, proposal.poses, margin_m=BYPASS_SIDE_MARGIN_M,
            pose_checked=pose_checked)

        rear_points = obstacles[
            obstacles[:, 0] < -base_gate.FOOTPRINT_REAR_M]
        tail_collision = proposal_path_collision(
            rear_points, proposal.poses, margin_m=0.0,
            pose_checked=pose_checked)
        target_behind = permit.target_x_m is not None and \
            float(permit.target_x_m) < -base_gate.FOOTPRINT_REAR_M
        self.evidence["static_threat_target_behind"] = bool(target_behind)

        carried_speed = max(0.0, float(self.motion.linear_speed_mps))
        carried_collision = False
        if carried_speed > base_gate.MOTION_EPSILON:
            carried_collision = bypass_swept_footprint_collision(
                obstacles,
                linear_speed_mps=carried_speed,
                angular_speed_rps=float(self.motion.angular_speed_rps),
                horizon_s=horizon_s,
                pose_checked=pose_checked)

        decision = bypass_policy.evaluate_gate_override(
            permit=permit,
            now_s=now_s,
            requested_v_mps=requested_speed,
            requested_w_rps=requested_yaw,
            immediate_collision=immediate_collision,
            requested_path_collision=requested_collision,
            carried_path_collision=carried_collision,
            minimum_turn_rps=self.minimum_bypass_turn_rps,
            maximum_permit_age_s=self.maximum_permit_age_s,
        )
        self.last_override = decision
        self.evidence["static_threat_tail_clear"] = bool(
            decision.allowed and not tail_collision)
        decision_reason = (
            "PROPOSAL_PATH_COLLISION"
            if decision.reason == "REQUESTED_PATH_COLLISION" else
            "STATIC_THREAT_TRAJECTORY_CLEAR" if decision.allowed else
            decision.reason)
        self.evidence.update({
            "trajectory_override_reason": decision_reason,
            "trajectory_override_allowed": decision.allowed,
            "static_threat_bypass_track_id": permit.track_id,
            "static_threat_bypass_static_for_s": round(permit.static_for_s, 3),
            "static_threat_bypass_permit_age_s": round(
                now_s - permit.stamp_s, 3),
            "trajectory_requested_collision": bool(requested_collision),
            "trajectory_carried_collision": bool(carried_collision),
            "trajectory_current_footprint_points": int(len(immediate)),
            "trajectory_proposal_seq": proposal.proposal_seq,
            "trajectory_proposal_age_s": round(proposal_age_s, 3),
            "trajectory_applied_v": round(applied_speed, 3),
            "trajectory_applied_w": round(applied_yaw, 3),
            "trajectory_target_v": round(proposal.target_speed_mps, 3),
            "trajectory_target_w": round(proposal.target_yaw_rate_rps, 3),
        })
        if not decision.allowed:
            return reason, cap
        if reason == "":
            return reason, cap
        return "", decision.speed_cap_mps

    def publish_status(self, reason, out):
        report = base_gate.status_report(
            self.evidence, reason, self.sweep_cap,
            out.linear.x, out.angular.z, self.policies)
        report["static_threat_bypass_capable"] = True
        report["static_threat_bypass_proposal_capable"] = True
        permit = self.person_bypass_permit
        now_s = rospy.Time.now().to_sec()
        report["static_threat_bypass_permit_active"] = bool(
            permit is not None and permit.active and
            bypass_policy.permit_is_fresh(
                permit, now_s, self.maximum_permit_age_s))
        if reason and self.cloud is not None and len(self.cloud):
            try:
                reference = base_gate.ground_reference(
                    self.cloud[:, :3], base_gate.SENSOR_HEIGHT_M)
                report["ground_ref_max_m"] = round(float(reference.max()), 3)
            except Exception:
                pass
        self.status_pub.publish(String(data=json.dumps(report, sort_keys=True)))


if __name__ == "__main__":
    TrajectorySafetyGate().spin()
