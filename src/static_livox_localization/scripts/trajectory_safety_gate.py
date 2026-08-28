#!/usr/bin/env python3
"""Raw LiDAR gate with a narrow trajectory-aware static-person exception.

The ordinary safety gate remains unchanged for every condition except its
fixed straight forward-corridor ``OBSTACLE`` verdict. A fresh, independently
qualified static-person permit may replace that one verdict only when:

* the command is a real curved DWA proposal,
* the person is not already inside the protected current footprint,
* the motion the chair is still carrying is collision-free (or stopped), and
* the requested curved swept footprint is clear against all raw points.

Stale sensors, invalid input, reverse, unknown/moving people, and straight
motion are never overridden. ``OBSTACLE_SWEEP`` enters the same independent
trajectory check so a colliding yaw is reported back to DWA for retry; it is
not waived. The old roughly 0.75 m expanded straight box is deliberately not
recreated here: the current footprint and the requested curve are measured
separately, so a clear curve can recover from the fixed-corridor deadlock this
node exists to remove.
"""

import json
import math
import os
import sys

import numpy as np
import rospy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import safety_gate as base_gate  # noqa: E402
from person_bypass_policy import (  # noqa: E402
    evaluate_gate_override,
    permit_from_payload,
    permit_is_fresh,
)


def collision_points_from_cloud(cloud):
    """The exact raw-point population used by the independent gate."""
    if cloud is None:
        return np.empty((0, 2), dtype=float)
    return base_gate.filter_obstacle_points(
        cloud,
        sensor_height_m=base_gate.SENSOR_HEIGHT_M,
        min_height_m=base_gate.COLLISION_MIN_HEIGHT_M,
        max_height_m=base_gate.COLLISION_MAX_HEIGHT_M,
        self_x_min_m=base_gate.RIDER_EXCLUDE_X_MIN_M,
        self_x_max_m=base_gate.RIDER_EXCLUDE_X_MAX_M,
        self_half_width_m=base_gate.RIDER_EXCLUDE_HALF_WIDTH_M,
        self_y_centre_m=base_gate.CHAIR_CENTRE_IN_BODY_XYZ[1])


def make_raw_gate_candidate_veto(
        cloud, motion, cloud_age_s, command_for_target,
        minimum_turn_rps=0.08, now_s=None):
    """Return a DWA veto using the raw gate's own swept-footprint contract.

    The callback is evaluated in DWA cost order.  It predicts the *ramped*
    command that will actually be published, builds the same dynamic horizon
    as :class:`safety_gate.SafetyGate`, and rejects any command the
    trajectory-aware gate would later reject.  Missing cloud/motion is
    fail-closed; the ordinary hold ladder should catch it first, but a stale
    callback must never turn that race into motion.
    """
    if (cloud is None or len(cloud) < 100 or motion is None
            or not motion.valid or not math.isfinite(float(cloud_age_s))
            or cloud_age_s < 0.0 or cloud_age_s > base_gate.CLOUD_STALE_S):
        return lambda _v, _w: True
    if now_s is not None and base_gate.motion_hold_reason(
            motion, now_s, base_gate.ODOM_STALE_S):
        return lambda _v, _w: True
    obstacles = collision_points_from_cloud(cloud)
    cloud_age_s = max(0.0, float(cloud_age_s))
    minimum_turn_rps = abs(float(minimum_turn_rps))
    cache = {}

    def veto(target_v, target_w):
        requested_v, requested_w, _accel = command_for_target(
            target_v, target_w)
        if not (math.isfinite(requested_v) and math.isfinite(requested_w)):
            return True
        if requested_v < 0.0:
            return True
        # The command ramp collapses many solver targets to the same next
        # command.  Exact raw-cloud sweeps are intentionally conservative
        # but not cheap; evaluate each command once per control cycle.
        key = (round(requested_v, 6), round(requested_w, 6))
        if key in cache:
            return cache[key]
        # SafetyGate does not run world geometry below its motion epsilon;
        # DwaFollower also publishes zero yaw in this range.
        if requested_v <= base_gate.MOTION_EPSILON:
            cache[key] = False
            return False
        if abs(requested_w) < minimum_turn_rps:
            cache[key] = True
            return True
        envelope = base_gate.stopping_envelope(
            measured_speed_mps=motion.linear_speed_mps,
            requested_speed_mps=requested_v,
            measured_yaw_rate_rps=motion.angular_speed_rps,
            requested_yaw_rate_rps=requested_w,
            cloud_age_s=cloud_age_s,
            accumulation_s=base_gate.ACCUMULATION_WINDOW_S,
            pipeline_s=base_gate.PIPELINE_BUDGET_S,
            min_linear_decel_mps2=base_gate.MIN_BRAKE_DECEL_MPS2,
            min_angular_decel_rps2=base_gate.MIN_YAW_DECEL_RPS2,
            geometry_margin_m=base_gate.GEOMETRY_MARGIN_M)
        requested_collision = base_gate.swept_footprint_collision(
            obstacles,
            linear_speed_mps=requested_v,
            angular_speed_rps=requested_w,
            horizon_s=envelope.horizon_s,
            front_m=base_gate.FOOTPRINT_FRONT_M,
            rear_m=base_gate.FOOTPRINT_REAR_M,
            half_width_m=base_gate.FOOTPRINT_HALF_WIDTH_M,
            margin_m=base_gate.SWEEP_MARGIN_M)
        if requested_collision:
            cache[key] = True
            return True
        carried_v = max(0.0, float(motion.linear_speed_mps))
        if carried_v <= base_gate.MOTION_EPSILON:
            cache[key] = False
            return False
        cache[key] = base_gate.swept_footprint_collision(
            obstacles,
            linear_speed_mps=carried_v,
            angular_speed_rps=float(motion.angular_speed_rps),
            horizon_s=envelope.horizon_s,
            front_m=base_gate.FOOTPRINT_FRONT_M,
            rear_m=base_gate.FOOTPRINT_REAR_M,
            half_width_m=base_gate.FOOTPRINT_HALF_WIDTH_M,
            margin_m=base_gate.SWEEP_MARGIN_M)
        return cache[key]

    return veto


class TrajectorySafetyGate(base_gate.SafetyGate):
    def __init__(self):
        super(TrajectorySafetyGate, self).__init__()
        self.person_bypass_permit = None
        self.maximum_permit_age_s = float(rospy.get_param(
            "~maximum_person_bypass_permit_age_s", 0.45))
        self.minimum_bypass_turn_rps = float(rospy.get_param(
            "~minimum_person_bypass_turn_rps", 0.08))
        # Zero by default: SWEEP_MARGIN_M already expands the measured chair
        # footprint by 0.15 m. Adding the previous extra 0.10 m recreated the
        # 0.75 m straight gate that made a valid curved bypass impossible.
        self.immediate_front_margin_m = float(rospy.get_param(
            "~person_bypass_immediate_front_margin_m", 0.0))
        self.immediate_side_margin_m = float(rospy.get_param(
            "~person_bypass_immediate_side_margin_m", 0.0))
        self.immediate_point_count = int(rospy.get_param(
            "~person_bypass_immediate_point_count", 5))
        for name in ("maximum_permit_age_s", "minimum_bypass_turn_rps"):
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
                "~person_bypass_immediate_point_count must be positive")
        permit_topic = str(rospy.get_param(
            "~person_bypass_permit_topic", "/person_bypass/permit"))
        rospy.Subscriber(permit_topic, String, self.on_person_bypass_permit,
                         queue_size=2)
        self.last_override = None
        rospy.set_param("~trajectory_person_bypass_capable", True)
        rospy.loginfo(
            "trajectory-aware raw gate enabled; permit=%s turn>=%.2f rad/s",
            permit_topic, self.minimum_bypass_turn_rps)

    def on_person_bypass_permit(self, message):
        self.person_bypass_permit = permit_from_payload(message.data)

    def fresh_active_permit(self, now_s):
        permit = self.person_bypass_permit
        if permit is None or not permit.active:
            return None
        return permit if permit_is_fresh(
            permit, now_s, self.maximum_permit_age_s) else None

    def collision_points(self):
        return collision_points_from_cloud(self.cloud)

    def motion_blocked(self, now):
        reason, cap = super(TrajectorySafetyGate, self).motion_blocked(now)
        self.last_override = None
        if reason not in ("OBSTACLE", "OBSTACLE_SWEEP"):
            return reason, cap

        now_s = now.to_sec()
        permit = self.fresh_active_permit(now_s)
        if permit is None:
            self.evidence["trajectory_override_reason"] = "NO_FRESH_PERMIT"
            return reason, cap

        obstacles = self.collision_points()
        requested_speed = max(0.0, min(
            base_gate.HARD_V_LIMIT,
            float(self.raw.linear.x),
            float(permit.max_speed_mps)))
        requested_yaw = max(
            -base_gate.HARD_W_LIMIT,
            min(base_gate.HARD_W_LIMIT, float(self.raw.angular.z)))
        try:
            horizon_s = float(self.evidence["horizon_s"])
        except (KeyError, TypeError, ValueError):
            self.evidence["trajectory_override_reason"] = "HORIZON_MISSING"
            return reason, cap

        # Current pose only, using the same protected dimensions as the raw
        # swept-footprint check. The requested sweep below also samples t=0;
        # this explicit count is retained only for a named diagnostic and a
        # separate fail-closed reason, not as a larger straight corridor.
        front = (base_gate.FOOTPRINT_FRONT_M + base_gate.SWEEP_MARGIN_M
                 + self.immediate_front_margin_m)
        side = (base_gate.FOOTPRINT_HALF_WIDTH_M + base_gate.SWEEP_MARGIN_M
                + self.immediate_side_margin_m)
        immediate = obstacles[
            (obstacles[:, 0] >= -base_gate.FOOTPRINT_REAR_M
             - base_gate.SWEEP_MARGIN_M) &
            (obstacles[:, 0] <= front) &
            (np.abs(obstacles[:, 1]) <= side)
        ] if len(obstacles) else obstacles
        immediate_collision = len(immediate) >= self.immediate_point_count

        requested_collision = base_gate.swept_footprint_collision(
            obstacles,
            linear_speed_mps=requested_speed,
            angular_speed_rps=requested_yaw,
            horizon_s=horizon_s,
            front_m=base_gate.FOOTPRINT_FRONT_M,
            rear_m=base_gate.FOOTPRINT_REAR_M,
            half_width_m=base_gate.FOOTPRINT_HALF_WIDTH_M,
            margin_m=base_gate.SWEEP_MARGIN_M)

        carried_speed = max(0.0, float(self.motion.linear_speed_mps))
        carried_collision = False
        if carried_speed > base_gate.MOTION_EPSILON:
            carried_collision = base_gate.swept_footprint_collision(
                obstacles,
                linear_speed_mps=carried_speed,
                angular_speed_rps=float(self.motion.angular_speed_rps),
                horizon_s=horizon_s,
                front_m=base_gate.FOOTPRINT_FRONT_M,
                rear_m=base_gate.FOOTPRINT_REAR_M,
                half_width_m=base_gate.FOOTPRINT_HALF_WIDTH_M,
                margin_m=base_gate.SWEEP_MARGIN_M)

        decision = evaluate_gate_override(
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
        self.evidence.update({
            "trajectory_override_reason": decision.reason,
            "trajectory_override_allowed": decision.allowed,
            "person_bypass_track_id": permit.track_id,
            "person_bypass_static_for_s": round(permit.static_for_s, 3),
            "person_bypass_permit_age_s": round(now_s - permit.stamp_s, 3),
            "trajectory_requested_collision": bool(requested_collision),
            "trajectory_carried_collision": bool(carried_collision),
            "trajectory_current_footprint_points": int(len(immediate)),
            "trajectory_requested_v": round(requested_speed, 3),
            "trajectory_requested_w": round(requested_yaw, 3),
        })
        if not decision.allowed:
            return reason, cap
        return "", decision.speed_cap_mps

    def publish_status(self, reason, out):
        report = base_gate.status_report(
            self.evidence, reason, self.sweep_cap,
            out.linear.x, out.angular.z, self.policies)
        report["trajectory_person_bypass_capable"] = True
        permit = self.person_bypass_permit
        now_s = rospy.Time.now().to_sec()
        report["person_bypass_permit_active"] = bool(
            permit is not None and permit.active and
            permit_is_fresh(permit, now_s, self.maximum_permit_age_s))
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
