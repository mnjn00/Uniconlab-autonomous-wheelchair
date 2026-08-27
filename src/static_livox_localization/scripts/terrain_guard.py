#!/usr/bin/env python3
"""Independent map-boundary and optional cliff guard before ``tip_guard``.

The MID-360 cannot see the pavement directly in front of the chair. This node
therefore treats the reviewed route mask and safety band as hard motion
boundaries. It checks both the incoming command arc and the trajectory already
being carried by the wheels; a zero command must not make a still-moving chair
look stationary. An external downward cliff sensor may be made mandatory with
``~cliff_required:=true``.
"""

import json
import math
import os
import sys

import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Int16MultiArray, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import (pose_correction, reference_correction,
                        route_chair_centre)
from route_mask import RouteMask
from safety_band import SafetyBand
from terrain_guard_policy import evaluate_terrain_command, stopping_horizon


GUARD_HZ = 30.0
WHEEL_SEPARATION_M = 0.54
MEASURED_MOTION_EPSILON = 0.02


class TerrainGuard:
    def __init__(self):
        rospy.init_node("terrain_guard")
        route_path = str(rospy.get_param("~route"))
        band_path = str(rospy.get_param("~safety_band"))
        mask_path = str(rospy.get_param("~drivable_mask"))
        profile = str(rospy.get_param("~body_frame_profile", "builtin"))

        with open(route_path, encoding="utf-8") as stream:
            route = json.load(stream)
        if "body_frame_profile" not in route or "reference_point" not in route:
            raise rospy.ROSInitException(
                "terrain guard requires route body_frame_profile and reference_point")
        route_centre = route_chair_centre(route)
        self.pose_correction = pose_correction(
            profile, str(route["body_frame_profile"])) @ reference_correction(
                str(route["reference_point"]), route_centre)
        self.mask = RouteMask(mask_path)
        self.band = SafetyBand(band_path)

        self.input_stale_s = float(rospy.get_param("~input_stale_s", 0.6))
        self.pose_stale_s = float(rospy.get_param("~pose_stale_s", 1.0))
        self.wheel_stale_s = float(rospy.get_param("~wheel_stale_s", 1.5))
        self.hard_clearance_m = float(rospy.get_param(
            "~hard_clearance_m", 0.12))
        self.slow_clearance_m = float(rospy.get_param(
            "~slow_clearance_m", 0.35))
        self.edge_speed_mps = float(rospy.get_param(
            "~edge_speed_mps", 0.35))
        self.minimum_deceleration_mps2 = float(rospy.get_param(
            "~minimum_deceleration_mps2", 0.5))
        self.reaction_s = float(rospy.get_param("~reaction_s", 0.25))
        self.reserve_s = float(rospy.get_param("~reserve_s", 0.5))
        self.minimum_horizon_s = float(rospy.get_param(
            "~minimum_horizon_s", 1.0))
        self.maximum_horizon_s = float(rospy.get_param(
            "~maximum_horizon_s", 3.0))
        self.rollout_step_s = float(rospy.get_param(
            "~rollout_step_s", 0.05))
        self.cliff_required = bool(rospy.get_param("~cliff_required", False))
        self.cliff_stale_s = float(rospy.get_param("~cliff_stale_s", 0.5))

        for name in (
                "input_stale_s", "pose_stale_s", "wheel_stale_s",
                "hard_clearance_m", "slow_clearance_m", "edge_speed_mps",
                "minimum_deceleration_mps2", "reaction_s", "reserve_s",
                "minimum_horizon_s", "maximum_horizon_s", "rollout_step_s",
                "cliff_stale_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise rospy.ROSInitException("~%s must be finite and non-negative" % name)
        if self.minimum_deceleration_mps2 <= 0.0 or self.rollout_step_s <= 0.0:
            raise rospy.ROSInitException(
                "deceleration and rollout step must be positive")
        if self.slow_clearance_m < self.hard_clearance_m:
            raise rospy.ROSInitException(
                "slow clearance must not be smaller than hard clearance")

        self.command = Twist()
        self.command_stamp = rospy.Time(0)
        self.pose = None
        self.pose_stamp = rospy.Time(0)
        self.measured_speed = 0.0
        self.measured_yaw_rate = 0.0
        self.wheel_stamp = rospy.Time(0)
        self.cliff_safe = not self.cliff_required
        self.cliff_reason = "NOT_REQUIRED" if not self.cliff_required else "NEVER_SEEN"
        self.cliff_stamp = rospy.Time(0)

        input_topic = str(rospy.get_param(
            "~input_topic", "/cmd_vel_gated"))
        output_topic = str(rospy.get_param(
            "~output_topic", "/cmd_vel_terrain_safe"))
        status_topic = str(rospy.get_param(
            "~status_topic", "/terrain_guard/status"))
        cliff_topic = str(rospy.get_param(
            "~cliff_topic", "/terrain/cliff_status"))

        self.pub = rospy.Publisher(output_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=1)
        rospy.Subscriber(input_topic, Twist, self.on_command, queue_size=1)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.on_pose, queue_size=5)
        rospy.Subscriber("/wheel_status", Int16MultiArray,
                         self.on_wheel_status, queue_size=5)
        rospy.Subscriber(cliff_topic, String, self.on_cliff, queue_size=2)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))
        rospy.loginfo(
            "terrain guard: %s -> %s, mask=%s hard=%.2f slow=%.2f "
            "cliff_required=%s",
            input_topic, output_topic, mask_path,
            self.hard_clearance_m, self.slow_clearance_m,
            self.cliff_required)

    def on_command(self, message):
        self.command = message
        self.command_stamp = rospy.Time.now()

    def on_pose(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(float(value)) for value in values):
            self.pose = None
            return
        matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[:3, 3] = (p.x, p.y, p.z)
        corrected = matrix @ self.pose_correction
        yaw = tft.euler_from_matrix(corrected)[2]
        self.pose = np.array([
            corrected[0, 3], corrected[1, 3], yaw], dtype=float)
        self.pose_stamp = message.header.stamp \
            if not message.header.stamp.isZero() else rospy.Time.now()

    @staticmethod
    def _reported_wheel_speeds(data):
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
            return 0.0, 0.0
        return one(data[2], data[3]), one(data[4], data[5])

    def on_wheel_status(self, message):
        left, right = self._reported_wheel_speeds(message.data)
        self.measured_speed = (left + right) * 0.5
        self.measured_yaw_rate = (right - left) / WHEEL_SEPARATION_M
        self.wheel_stamp = rospy.Time.now()

    def on_cliff(self, message):
        now = rospy.Time.now()
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("cliff status is not an object")
            status = str(data.get("status", ""))
            safe = data.get("safe")
            self.cliff_safe = bool(safe) and status in ("OK", "CLEAR")
            self.cliff_reason = status or "UNKNOWN"
            stamp = data.get("stamp")
            self.cliff_stamp = rospy.Time.from_sec(float(stamp)) \
                if isinstance(stamp, (int, float)) and math.isfinite(stamp) \
                else now
        except (TypeError, ValueError):
            self.cliff_safe = False
            self.cliff_reason = "MALFORMED"
            self.cliff_stamp = now

    def _cliff_block(self, now):
        if not self.cliff_required:
            return ""
        if self.cliff_stamp.isZero() or \
                (now - self.cliff_stamp).to_sec() > self.cliff_stale_s:
            return "CLIFF_STALE"
        if not self.cliff_safe:
            return "CLIFF_" + self.cliff_reason
        return ""

    def _decision(self, linear_speed, angular_speed, horizon):
        return evaluate_terrain_command(
            self.mask, self.pose, linear_speed, angular_speed,
            self.hard_clearance_m, self.slow_clearance_m,
            self.edge_speed_mps, safety_band=self.band,
            horizon_s=horizon, step_s=self.rollout_step_s)

    @staticmethod
    def _combine_decisions(command_decision, carried_decision):
        if carried_decision is not None and carried_decision.blocked:
            return "CARRIED_" + carried_decision.reason, carried_decision
        if command_decision.blocked:
            return command_decision.reason, command_decision
        if carried_decision is None:
            return "", command_decision
        clearances = [value for value in (
            command_decision.minimum_clearance_m,
            carried_decision.minimum_clearance_m) if value is not None]
        minimum = min(clearances) if clearances else None
        caps = [value for value in (
            command_decision.speed_cap_mps,
            carried_decision.speed_cap_mps) if value is not None]
        cap = min(caps) if caps else None
        horizon = max(command_decision.horizon_s, carried_decision.horizon_s)
        return "", type(command_decision)("", minimum, cap, horizon)

    def step(self):
        now = rospy.Time.now()
        reason = ""
        decision = None
        command_age = (now - self.command_stamp).to_sec()
        pose_age = math.inf if self.pose is None else \
            (now - self.pose_stamp).to_sec()
        wheel_age = (now - self.wheel_stamp).to_sec()

        if command_age > self.input_stale_s:
            reason = "INPUT_STALE"
        elif self.pose is None or pose_age > self.pose_stale_s:
            reason = "POSE_STALE"
        elif self.wheel_stamp.isZero() or wheel_age > self.wheel_stale_s:
            reason = "WHEEL_STALE"
        else:
            values = (
                self.command.linear.x, self.command.linear.y,
                self.command.linear.z, self.command.angular.x,
                self.command.angular.y, self.command.angular.z,
                self.measured_speed, self.measured_yaw_rate,
            )
            if not all(math.isfinite(value) for value in values):
                reason = "INPUT_INVALID"
            elif self.measured_speed < -MEASURED_MOTION_EPSILON:
                reason = "MEASURED_REVERSE"
            else:
                cliff = self._cliff_block(now)
                if cliff:
                    reason = cliff
                else:
                    carried_speed = max(0.0, self.measured_speed)
                    horizon_speed = max(
                        abs(self.command.linear.x), carried_speed)
                    try:
                        horizon = stopping_horizon(
                            horizon_speed,
                            reaction_s=self.reaction_s,
                            minimum_deceleration_mps2=
                                self.minimum_deceleration_mps2,
                            reserve_s=self.reserve_s,
                            minimum_horizon_s=self.minimum_horizon_s,
                            maximum_horizon_s=self.maximum_horizon_s,
                        )
                    except ValueError:
                        horizon = 0.0
                        reason = "CONFIG_INVALID"
                    if not reason:
                        commanded = self._decision(
                            self.command.linear.x,
                            self.command.angular.z, horizon)
                        carried = None
                        if carried_speed > MEASURED_MOTION_EPSILON or \
                                abs(self.measured_yaw_rate) > 0.05:
                            carried = self._decision(
                                carried_speed, self.measured_yaw_rate, horizon)
                        reason, decision = self._combine_decisions(
                            commanded, carried)

        out = Twist()
        if not reason:
            out = self.command
            if decision is not None and decision.speed_cap_mps is not None:
                out.linear.x = min(
                    max(0.0, out.linear.x), decision.speed_cap_mps)
                if out.linear.x <= 0.0:
                    out.angular.z = 0.0
        self.pub.publish(out)

        report = {
            "stamp": now.to_sec(),
            "status": "OK" if not reason else "HOLD",
            "reason": reason,
            "blocked": bool(reason),
            "command_age_s": round(command_age, 3),
            "pose_age_s": None if not math.isfinite(pose_age)
                          else round(pose_age, 3),
            "wheel_age_s": round(wheel_age, 3),
            "minimum_clearance_m": None if decision is None
                or decision.minimum_clearance_m is None
                else round(decision.minimum_clearance_m, 3),
            "horizon_s": None if decision is None
                         else round(decision.horizon_s, 3),
            "speed_cap_mps": None if decision is None
                or decision.speed_cap_mps is None
                else round(decision.speed_cap_mps, 3),
            "in_v": round(float(self.command.linear.x), 3),
            "in_w": round(float(self.command.angular.z), 3),
            "measured_v": round(float(self.measured_speed), 3),
            "measured_w": round(float(self.measured_yaw_rate), 3),
            "out_v": round(float(out.linear.x), 3),
            "out_w": round(float(out.angular.z), 3),
            "cliff_required": self.cliff_required,
            "cliff_safe": self.cliff_safe,
            "cliff_reason": self.cliff_reason,
        }
        self.status_pub.publish(String(data=json.dumps(
            report, separators=(",", ":"), sort_keys=True)))

    def spin(self):
        rate = rospy.Rate(GUARD_HZ)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    try:
        TerrainGuard().spin()
    except rospy.ROSInterruptException:
        pass