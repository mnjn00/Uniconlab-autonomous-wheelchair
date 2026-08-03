#!/usr/bin/env python3
"""Independent obstacle safety gate between the planner and tip_guard.

Deliberately knows nothing about routes or planning: it forwards
/cmd_vel_raw to /cmd_vel_gated only when its own dynamic stopping-distance
and swept-footprint checks pass, clamps speeds, replaces stale or missing
input with a stop, and publishes continuously so the chain always has a
live command stream.
tip_guard.py is the final stage after this (rate-limited relay with its
own staleness fail-safe); wheel_cmd_tmp.py/uart.py consume its output on
/cmd_vel. If the planner misbehaves or dies, this gate stops the chair;
if this gate dies, tip_guard's own staleness check stops the chair; if
that dies too, the uart-level watchdog stops the chair.

_safety_policies:=false keeps the chain-integrity checks - stale input,
non-finite input, reverse, the speed ceilings - and switches off the
judgements about the world: the stopping envelope, the swept footprint,
and scan staleness, which only matters because those two read the scan.
Suppressed blocks are still computed and logged. See drive_policy.py.
"""
import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import sensor_msgs.point_cloud2 as pc2
# catkin_install_python leaves a relay in devel/lib that exec()s this file,
# so sys.path[0] is the relay's directory, not this one, and the policy
# modules sitting beside this file are not importable - the relay does set
# __file__ to this source path, so recover the directory from it. Without
# this the node dies at import on the vehicle while every offline test,
# which imports the modules directly, still passes.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import CHAIR_CENTRE_IN_BODY_XYZ, lidar_extrinsics
from drive_policy import announce
from motion_safety import (MotionEstimate, PoseMotionEstimator,
                           filter_obstacle_points, motion_hold_reason,
                           stopping_envelope, swept_footprint_collision)
from priest_constraints import (
    CANONICAL_FOOTPRINT,
    RAW_INPUT_STALE_S,
    SAFETY_GATE_RATE_HZ,
)
from scan_accumulator import CloudAccumulator


GATE_HZ = SAFETY_GATE_RATE_HZ
INPUT_STALE_S = RAW_INPUT_STALE_S
CLOUD_STALE_S = 1.0
ODOM_STALE_S = 0.35
MOTION_EPSILON = 0.02
# Ceilings, not setpoints: this gate exists to bound whatever the planner
# asks for, so HARD_V_LIMIT must sit ABOVE the follower's MAX_SPEED or it
# silently becomes the real speed limit. It was 0.6 while the follower
# asked for 0.5; raising the follower to the measured 1.2 m/s without
# raising this would have halved the commanded speed at this stage with no
# error anywhere. test_safety_chain asserts the ordering holds.
HARD_V_LIMIT = 1.4
HARD_W_LIMIT = 0.6
HALF_WIDTH_M = 0.5
SENSOR_HEIGHT_M = 0.725
OBSTACLE_MIN_Z = 0.15
OBSTACLE_MAX_Z = 1.9
ACCUMULATION_WINDOW_S = 1.0
PIPELINE_BUDGET_S = 0.2
MIN_BRAKE_DECEL_MPS2 = 0.5
MIN_YAW_DECEL_RPS2 = 0.5
GEOMETRY_MARGIN_M = 0.9
FORWARD_CHECK_EXTRA_M = 0.6
FOOTPRINT_FRONT_M = CANONICAL_FOOTPRINT.front_m
FOOTPRINT_REAR_M = CANONICAL_FOOTPRINT.rear_m
FOOTPRINT_HALF_WIDTH_M = CANONICAL_FOOTPRINT.half_width_m
SWEEP_MARGIN_M = CANONICAL_FOOTPRINT.sweep_margin_m
RIDER_EXCLUDE_X_MIN_M = -1.0
RIDER_EXCLUDE_X_MAX_M = 0.55
RIDER_EXCLUDE_HALF_WIDTH_M = 0.40
# Forward FOV cone: the gate only checks obstacles the chair is
# driving toward. Side/rear returns are the rider and the wheelchair
# frame; the minimum range skips the rider's knees and footrest.
FORWARD_FOV_HALF_DEG = 50.0
CORRIDOR_MIN_RANGE_M = 0.50


class SafetyGate:
    def __init__(self):
        rospy.init_node("safety_gate")
        self.raw = Twist()
        self.raw_stamp = rospy.Time(0)
        profile = str(rospy.get_param("~body_frame_profile"))
        lidar_in_body, lidar_to_body_rotation = lidar_extrinsics(profile)
        self.accumulator = CloudAccumulator(
            lidar_in_body, lidar_to_body_rotation)
        odom_frame = str(rospy.get_param("~odom_frame", "camera_init"))
        base_frame = str(rospy.get_param("~base_frame", "body"))
        self.motion_estimator = PoseMotionEstimator(odom_frame, base_frame)
        self.motion = MotionEstimate(
            False, 0.0, 0.0, 0.0, 0.0, "ODOM_INITIALIZING")
        self.cloud = None
        self.cloud_stamp = rospy.Time(0)
        self.blocked_reason = ""
        self.policies = bool(rospy.get_param("~safety_policies", True))
        if self.policies:
            rospy.loginfo(announce(True, "safety_gate", []))
        else:
            rospy.logwarn(announce(False, "safety_gate", [
                "the stopping envelope", "the swept footprint",
                "scan staleness", "the motion-estimate gate"]))
        self.pub = rospy.Publisher("/cmd_vel_gated", Twist, queue_size=1)
        rospy.Subscriber("/cmd_vel_raw", Twist, self.on_raw, queue_size=1)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.on_cloud, queue_size=2)
        rospy.Subscriber("/Odometry", Odometry,
                         self.on_odom, queue_size=50)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    def on_raw(self, message):
        self.raw = message
        self.raw_stamp = rospy.Time.now()

    def on_cloud(self, message):
        self.accumulator.add_cloud(message, pc2.read_points)
        self.cloud, self.cloud_stamp = self.accumulator.merged()

    def on_odom(self, message):
        self.accumulator.add_odom(message)
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        self.motion = self.motion_estimator.update(
            source_stamp_s=message.header.stamp.to_sec(),
            receipt_stamp_s=rospy.Time.now().to_sec(),
            frame_id=message.header.frame_id,
            child_frame_id=message.child_frame_id,
            x=p.x,
            y=p.y,
            quaternion_xyzw=(q.x, q.y, q.z, q.w))

    def motion_blocked(self, now):
        """Check visible obstacles; drop safety remains map-band containment."""
        if self.cloud is None or len(self.cloud) < 100:
            return "NO_CLOUD"
        reason = motion_hold_reason(
            self.motion, now.to_sec(), ODOM_STALE_S)
        if reason:
            return reason

        requested_speed = max(0.0, min(HARD_V_LIMIT,
                                       self.raw.linear.x))
        requested_yaw_rate = max(
            -HARD_W_LIMIT, min(HARD_W_LIMIT, self.raw.angular.z))
        cloud_age = max(0.0, (now - self.cloud_stamp).to_sec())
        envelope = stopping_envelope(
            measured_speed_mps=self.motion.linear_speed_mps,
            requested_speed_mps=requested_speed,
            measured_yaw_rate_rps=self.motion.angular_speed_rps,
            requested_yaw_rate_rps=requested_yaw_rate,
            cloud_age_s=cloud_age,
            accumulation_s=ACCUMULATION_WINDOW_S,
            pipeline_s=PIPELINE_BUDGET_S,
            min_linear_decel_mps2=MIN_BRAKE_DECEL_MPS2,
            min_angular_decel_rps2=MIN_YAW_DECEL_RPS2,
            geometry_margin_m=GEOMETRY_MARGIN_M)
        obstacles = filter_obstacle_points(
            self.cloud,
            sensor_height_m=SENSOR_HEIGHT_M,
            min_height_m=OBSTACLE_MIN_Z,
            max_height_m=OBSTACLE_MAX_Z,
            self_x_min_m=RIDER_EXCLUDE_X_MIN_M,
            self_x_max_m=RIDER_EXCLUDE_X_MAX_M,
            self_half_width_m=RIDER_EXCLUDE_HALF_WIDTH_M,
            self_y_centre_m=CHAIR_CENTRE_IN_BODY_XYZ[1])

        if len(obstacles):
            azimuth = np.abs(np.degrees(np.arctan2(
                obstacles[:, 1], obstacles[:, 0])))
            zone = obstacles[
                (obstacles[:, 0] > CORRIDOR_MIN_RANGE_M) &
                (obstacles[:, 0] <
                 envelope.distance_m + FORWARD_CHECK_EXTRA_M) &
                (azimuth < FORWARD_FOV_HALF_DEG) &
                (np.abs(obstacles[:, 1]) < HALF_WIDTH_M)]
        else:
            zone = obstacles
        if len(zone) >= 5 and \
                np.percentile(zone[:, 0], 5) < envelope.distance_m:
            return "OBSTACLE"

        speed = max(self.motion.linear_speed_mps, requested_speed)
        yaw_rates = [requested_yaw_rate]
        if abs(self.motion.angular_speed_rps - requested_yaw_rate) > 0.05:
            yaw_rates.append(self.motion.angular_speed_rps)
        for yaw_rate in yaw_rates:
            if swept_footprint_collision(
                    obstacles,
                    linear_speed_mps=speed,
                    angular_speed_rps=yaw_rate,
                    horizon_s=envelope.horizon_s,
                    front_m=FOOTPRINT_FRONT_M,
                    rear_m=FOOTPRINT_REAR_M,
                    half_width_m=FOOTPRINT_HALF_WIDTH_M,
                    margin_m=SWEEP_MARGIN_M):
                return "OBSTACLE_SWEEP"
        return ""

    def spin(self):
        rate = rospy.Rate(GATE_HZ)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            out = Twist()
            reason = ""
            if (now - self.raw_stamp).to_sec() > INPUT_STALE_S:
                reason = "INPUT_STALE"
            elif self.policies and \
                    (now - self.cloud_stamp).to_sec() > CLOUD_STALE_S:
                reason = "CLOUD_STALE"
            elif not math.isfinite(self.raw.linear.x) or \
                    not math.isfinite(self.raw.angular.z):
                reason = "INPUT_INVALID"
            elif self.raw.linear.x < -MOTION_EPSILON:
                reason = "REVERSE"
            else:
                wants_motion = \
                    abs(self.raw.linear.x) > MOTION_EPSILON or \
                    abs(self.raw.angular.z) > MOTION_EPSILON
                if wants_motion:
                    blocked = self.motion_blocked(now)
                    if self.policies:
                        reason = blocked
                    elif blocked:
                        # Still measured, still logged, just not acted on:
                        # this log is where the run finds out how often the
                        # envelope fires on the real thing.
                        rospy.logwarn_throttle(
                            5.0, "policies off: would have stopped on %s",
                            blocked)
                if not reason:
                    out.linear.x = max(0.0, min(HARD_V_LIMIT,
                                                self.raw.linear.x))
                    out.angular.z = max(-HARD_W_LIMIT,
                                        min(HARD_W_LIMIT, self.raw.angular.z))
            if reason and reason != self.blocked_reason:
                rospy.logwarn("safety gate stop: %s", reason)
            self.blocked_reason = reason
            self.pub.publish(out)
            rate.sleep()


if __name__ == "__main__":
    SafetyGate().spin()
