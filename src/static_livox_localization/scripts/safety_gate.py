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
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import sensor_msgs.point_cloud2 as pc2
# catkin_install_python leaves a relay in devel/lib that exec()s this file,
# so sys.path[0] is the relay's directory, not this one, and the policy
# modules sitting beside this file are not importable - the relay does set
# __file__ to this source path, so recover the directory from it. Without
# this the node dies at import on the vehicle while every offline test,
# which imports the modules directly, still passes.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import CHAIR_CENTRE_IN_BODY_XYZ, lidar_extrinsics
from cloud_points import (COLLISION_MAX_HEIGHT_M,
                          COLLISION_MIN_HEIGHT_M)
from drive_policy import announce
from motion_safety import (CollisionSnapshot, MotionEstimate,
                           PoseMotionEstimator,
                           filter_obstacle_points,
                           ground_reference, motion_hold_reason,
                           stopping_envelope,
                           swept_footprint_collision)
from scan_accumulator import CloudAccumulator


GATE_HZ = 15.0
INPUT_STALE_S = 0.6
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
# Ceiling of the obstacle window, measured from the ground. 1.35 m is the
# top of the seated occupant (measured 2026-08-09), and this is that plus a
# 0.15 m margin.
#
# It was 1.9 m, which is above anything on this chair and therefore stopped
# it for things it cannot reach. On 2026-08-09 the gate held the chair at
# wp 905 on TWO returns at 1.88 m - the underside of an overhanging branch
# mass whose other 42 returns, from 1.90 to 2.42 m, the same window already
# ignored. Nothing at all was returned between 0.15 and 1.80 m: no wall, no
# person, nothing the chair could hit. A branch 0.53 m above the rider's
# head is not an obstacle, and a ceiling above the vehicle cannot tell the
# difference.
#
# Lowering this loses only objects that exist ABOVE 1.5 m with nothing below
# them. A standing adult is caught by their torso at 0.8-1.6 m; a pole, a
# bollard and a parked car all return well below. What is given up is
# canopy, signage and awnings - which is the point.
ACCUMULATION_WINDOW_S = 1.0
PIPELINE_BUDGET_S = 0.2
MIN_BRAKE_DECEL_MPS2 = 0.5
MIN_YAW_DECEL_RPS2 = 0.5
GEOMETRY_MARGIN_M = 0.9
FORWARD_CHECK_EXTRA_M = 0.6
FOOTPRINT_FRONT_M = 0.50
FOOTPRINT_REAR_M = 0.50
FOOTPRINT_HALF_WIDTH_M = 0.30
SWEEP_MARGIN_M = 0.15
RIDER_EXCLUDE_X_MIN_M = -1.0
RIDER_EXCLUDE_X_MAX_M = 0.55
RIDER_EXCLUDE_HALF_WIDTH_M = 0.40
# Forward FOV cone: the gate only checks obstacles the chair is
# driving toward. Side/rear returns are the rider and the wheelchair
# frame; the minimum range skips the rider's knees and footrest.
FORWARD_FOV_HALF_DEG = 50.0
CORRIDOR_MIN_RANGE_M = 0.50

# OBSTACLE_SWEEP is a speed limit, not a stop.
#
# The swept footprint grows with the speed it is evaluated at, so a binary
# stop makes standing still the condition for being allowed to move: stopped
# -> short sweep -> gate opens -> accelerate -> sweep lengthens -> blocked ->
# stopped. Measured on 2026-08-16 that cycle ran every 2.5-5 s with
# cmd_vel_raw held at 0.79 the whole time, and raising the cap from 0.6 to
# 0.8 made it fire more often. Capping instead of stopping settles: the chair
# holds the fastest speed whose sweep is clear and the loop has no gain.
#
# Braking safety is unaffected. The separate OBSTACLE test above still stops
# outright when a return sits inside the stopping envelope; this one only
# governs how fast the chair may drive past something it can already stop for.
SWEEP_BISECTION_STEPS = 5
# Below this a cap is not worth having, and the number is the chair's, not a
# preference: under roughly 0.3 m/s the loaded wheels do not turn at all, so
# capping into that band commands a speed that does nothing while reading as
# motion. dwa_core.TURN_FLOOR_SPEED and mpc_speed.TURN_FLOOR_SPEED carry the
# same measurement; the gate keeps its own copy because it must hold for any
# control law, including one that never imports either.
#
# Two modules disagreeing about this floor is not hypothetical. The cluster
# guard ramps from CREEP_SPEED = 0.15 while the DWA sampler refuses anything
# under 0.30, so a tracked object at exactly the guard distance produced a
# cap of 0.15, no executable candidate, and a stop reported as NO_CANDIDATE -
# the 2026-08-20 stall.
SWEEP_MIN_SPEED_MPS = 0.35
# The cap drops instantly and recovers at this rate, so a return that flickers
# in and out of the sweep cannot chatter the command.
SWEEP_CAP_RELEASE_MPS2 = 0.5


def status_report(evidence, reason, cap, out_v, out_w, policies):
    """What the gate decided, and the numbers it decided on.

    Pure so it can be tested: a diagnostic that quietly stops carrying the
    quantity that settles the argument is worse than none, because the
    next run is read as if the field had been checked.

    reason is "" when nothing is blocking. Everything else is whatever
    motion_blocked managed to measure before it returned - an early exit
    leaves the later fields absent rather than zero, because a missing
    measurement and a measurement of zero mean different things here.
    """
    report = dict(evidence or {})
    report["reason"] = str(reason or "")
    report["blocked"] = bool(reason)
    report["cap"] = round(float(cap), 3)
    report["out_v"] = round(float(out_v), 3)
    report["out_w"] = round(float(out_w), 3)
    report["policies"] = bool(policies)
    return report


class SafetyGate:
    def __init__(self):
        rospy.init_node("safety_gate")
        self.raw = Twist()
        self.raw_stamp = rospy.Time(0)
        self.latest_raw = self.raw
        self.latest_raw_stamp = self.raw_stamp
        self.raw_lock = threading.Lock()
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
        self.collision_snapshot = None
        self.blocked_reason = ""
        self.evidence = {}
        self.sweep_cap = HARD_V_LIMIT
        self.policies = bool(rospy.get_param("~safety_policies", True))
        if self.policies:
            rospy.loginfo(announce(True, "safety_gate", []))
        else:
            rospy.logwarn(announce(False, "safety_gate", [
                "the stopping envelope", "the swept footprint",
                "scan staleness", "the motion-estimate gate"]))
        self.pub = rospy.Publisher("/cmd_vel_gated", Twist, queue_size=1)
        # Why the chair stopped, with the numbers the decision was made on.
        #
        # This node used to publish nothing but a Twist, so a stop left no
        # record of its own reason anywhere - not in the bag, not in a
        # topic. Every stop then had to be re-derived from what the chair
        # did afterwards, and on 2026-08-23 two of them cost most of a day:
        # a 130 s deadlock in front of a parked motorcycle and two 1.3 s
        # stops on a crest, both diagnosed only by borrowing
        # /perception/objects_summary as a window onto what the gate might
        # have been looking at. The reason alone would not have been
        # enough either - what settled both was the range the envelope
        # reached and the range the nearest return sat at. So those go out
        # with it.
        self.status_pub = rospy.Publisher("/safety_gate/status", String,
                                          queue_size=1)
        rospy.Subscriber("/cmd_vel_raw", Twist, self.on_raw, queue_size=1)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.on_cloud, queue_size=2)
        rospy.Subscriber("/Odometry", Odometry,
                         self.on_odom, queue_size=50)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    def on_raw(self, message):
        with self.raw_lock:
            self.latest_raw = message
            self.latest_raw_stamp = rospy.Time.now()

    def snapshot_raw_command(self):
        with self.raw_lock:
            self.raw = self.latest_raw
            self.raw_stamp = self.latest_raw_stamp

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
        self.collision_snapshot = None
        self.evidence = {
            "cloud_points": 0 if self.cloud is None else int(len(self.cloud)),
            "cloud_age_s": round(max(0.0, (now - self.cloud_stamp).to_sec()), 3),
            "snapshot_builds": 0,
            "filter_calls": 0,
            "pose_checks": 0,
        }
        if self.cloud is None or len(self.cloud) < 100:
            return "NO_CLOUD", None
        reason = motion_hold_reason(
            self.motion, now.to_sec(), ODOM_STALE_S)
        if reason:
            return reason, None

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
            min_height_m=COLLISION_MIN_HEIGHT_M,
            max_height_m=COLLISION_MAX_HEIGHT_M,
            self_x_min_m=RIDER_EXCLUDE_X_MIN_M,
            self_x_max_m=RIDER_EXCLUDE_X_MAX_M,
            self_half_width_m=RIDER_EXCLUDE_HALF_WIDTH_M,
            self_y_centre_m=CHAIR_CENTRE_IN_BODY_XYZ[1])
        self.evidence["filter_calls"] += 1
        self.collision_snapshot = CollisionSnapshot(
            points_xy=obstacles, source_point_count=len(self.cloud))
        self.evidence["snapshot_builds"] += 1

        self.evidence["requested_v"] = round(requested_speed, 3)
        self.evidence["requested_w"] = round(requested_yaw_rate, 3)
        self.evidence["measured_v"] = round(self.motion.linear_speed_mps, 3)
        self.evidence["envelope_m"] = round(float(envelope.distance_m), 3)
        self.evidence["horizon_s"] = round(float(envelope.horizon_s), 3)
        self.evidence["obstacle_points"] = int(len(obstacles))
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
        self.evidence["zone_points"] = int(len(zone))
        # The exact quantity the stop test compares, not a summary of it.
        nearest = (float(np.percentile(zone[:, 0], 5)) if len(zone) >= 5
                   else None)
        self.evidence["zone_nearest_m"] = (None if nearest is None
                                           else round(nearest, 3))
        if len(zone) >= 5 and nearest < envelope.distance_m:
            self.evidence["zone_lateral_m"] = round(
                float(np.abs(zone[:, 1]).min()), 3)
            return "OBSTACLE", None

        yaw_rates = [requested_yaw_rate]
        if abs(self.motion.angular_speed_rps - requested_yaw_rate) > 0.05:
            yaw_rates.append(self.motion.angular_speed_rps)
        self.evidence["sweep_calls"] = 0

        def sweep_hits(candidate_speed):
            for yaw_rate in yaw_rates:
                self.evidence["sweep_calls"] += 1
                if swept_footprint_collision(
                        self.collision_snapshot.points_xy,
                        linear_speed_mps=candidate_speed,
                        angular_speed_rps=yaw_rate,
                        horizon_s=envelope.horizon_s,
                        front_m=FOOTPRINT_FRONT_M,
                        rear_m=FOOTPRINT_REAR_M,
                        half_width_m=FOOTPRINT_HALF_WIDTH_M,
                        margin_m=SWEEP_MARGIN_M,
                        pose_checked=lambda: self.evidence.__setitem__(
                            "pose_checks",
                            self.evidence["pose_checks"] + 1)):
                    return True
            return False

        # The search is over what may be COMMANDED. The measured speed is not
        # folded in here on purpose: at 0.6 m/s measured, max(measured, v) is
        # 0.6 for every candidate, the search returns zero, and the binary
        # stop is back. What the chair is already carrying is the stopping
        # envelope's business, and OBSTACLE above has already ruled on it.
        if not sweep_hits(requested_speed):
            return "", None
        if requested_speed <= SWEEP_MIN_SPEED_MPS + 1e-6:
            self.evidence["sweep_clear_v"] = 0.0
            return "OBSTACLE_SWEEP", None
        low, high = 0.0, requested_speed
        for _ in range(SWEEP_BISECTION_STEPS):
            middle = 0.5 * (low + high)
            if sweep_hits(middle):
                high = middle
            else:
                low = middle
        self.evidence["sweep_clear_v"] = round(float(low), 3)
        if low < SWEEP_MIN_SPEED_MPS:
            return "OBSTACLE_SWEEP", None
        return "", low

    def spin(self):
        rate = rospy.Rate(GATE_HZ)
        while not rospy.is_shutdown():
            self.snapshot_raw_command()
            now = rospy.Time.now()
            out = Twist()
            reason = ""
            # Cleared every cycle, not just where motion_blocked refills it.
            # The reasons raised above it - INPUT_STALE, CLOUD_STALE,
            # INPUT_INVALID, REVERSE - never call it, and a report that
            # pairs one of those with the previous cycle's envelope and
            # nearest range is worse than one that carries no numbers: it
            # reads as a measurement that was taken.
            self.evidence = {}
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
                    gate_started = time.perf_counter()
                    blocked, cap = self.motion_blocked(now)
                    self.evidence["gate_ms"] = round(
                        (time.perf_counter() - gate_started) * 1000.0, 3)
                    if self.policies:
                        reason = blocked
                        step = SWEEP_CAP_RELEASE_MPS2 / GATE_HZ
                        target = HARD_V_LIMIT if cap is None else cap
                        # Down instantly, up on a leash. Taking min() with the
                        # standing cap instead would let one tight frame hold
                        # the chair slow long after the return had gone.
                        if target < self.sweep_cap:
                            self.sweep_cap = target
                        else:
                            self.sweep_cap = min(
                                target, self.sweep_cap + step)
                    elif blocked:
                        # Still measured, still logged, just not acted on:
                        # this log is where the run finds out how often the
                        # envelope fires on the real thing.
                        rospy.logwarn_throttle(
                            5.0, "policies off: would have stopped on %s",
                            blocked)
                else:
                    self.sweep_cap = min(
                        HARD_V_LIMIT,
                        self.sweep_cap + SWEEP_CAP_RELEASE_MPS2 / GATE_HZ)
                if not reason:
                    out.linear.x = max(0.0, min(HARD_V_LIMIT,
                                                self.sweep_cap,
                                                self.raw.linear.x))
                    out.angular.z = max(-HARD_W_LIMIT,
                                        min(HARD_W_LIMIT, self.raw.angular.z))
            if reason and reason != self.blocked_reason:
                rospy.logwarn("safety gate stop: %s", reason)
            self.blocked_reason = reason
            self.pub.publish(out)
            self.publish_status(reason, out)
            rate.sleep()

    def publish_status(self, reason, out):
        report = status_report(self.evidence, reason, self.sweep_cap,
                               out.linear.x, out.angular.z, self.policies)
        if reason and self.cloud is not None and len(self.cloud):
            # Only on the blocking path. It is a second pass over the cloud
            # and costs about 10 ms at 100k points, which is affordable when
            # the chair is standing still and not when it is not. It says how
            # far the terrain reference has moved off the chair plane, which
            # is what separates a real object from the road seen at a pitch.
            try:
                reference = ground_reference(self.cloud[:, :3],
                                             SENSOR_HEIGHT_M)
                report["ground_ref_max_m"] = round(float(reference.max()), 3)
            except Exception:
                pass
        self.status_pub.publish(String(data=json.dumps(report,
                                                       sort_keys=True)))


if __name__ == "__main__":
    SafetyGate().spin()
