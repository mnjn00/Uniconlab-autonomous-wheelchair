#!/usr/bin/env python3
"""Certified PRIEST follower behind the existing waypoint-follower contract.

PRIEST remains opt-in. This node publishes only time-indexed differential-
drive commands from a runtime-band-certified plan; every uncertainty holds.
"""

from __future__ import annotations

import json
import os
import sys
import threading
from typing import Optional

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import SetBool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import (lidar_extrinsics, pose_correction,
                        reference_correction, route_chair_centre)
from motion_safety import MotionEstimate, PoseMotionEstimator
from priest_constraints import within_goal_tolerance
from priest_controller import (
    DEFAULT_CONTROLLER_LIMITS,
    DriveCommand,
    Pose2D,
    STOPPED_SPEED_MPS,
    command_for,
    plan_arrays,
)
from priest_corridor import corridor_arrays
from priest_heading_pid import HeadingPid, SteeringFeedback
from priest_follower_io import (
    CERTIFICATE_HZ,
    CONTROL_HZ,
    FollowerIOMixin,
    GOAL_TOLERANCE_M,
    MAX_ACCEL,
    MAX_DECEL,
    MAX_SPEED,
    MAX_YAW_RATE,
    OFF_BAND_GRACE,
    TURN_FLOOR_SPEED,
    require_guards,
)
from priest_follower_planning import FollowerPlanningMixin
from priest_planner import Corridor, PriestPlanner
from priest_runtime import (
    OBSTACLE_WAIT,
    WAIT_RADIUS_M,
    wait_reason as runtime_wait_reason,
)
from safety_band import SafetyBand


def make_planner(band: SafetyBand) -> PriestPlanner:
    return PriestPlanner(
        v_max=MAX_SPEED, a_max=MAX_ACCEL, yaw_rate_max=MAX_YAW_RATE,
        runtime_band=band, control_hz=CERTIFICATE_HZ,
        band_grace_m=OFF_BAND_GRACE,
        turn_floor_speed_mps=TURN_FLOOR_SPEED, seed=0)


class PriestFollower(FollowerIOMixin, FollowerPlanningMixin):
    def __init__(self) -> None:
        rospy.init_node("waypoint_follower")
        require_guards(bool(rospy.get_param("~safety_policies", True)))
        with open(rospy.get_param("~route")) as handle:
            route = json.load(handle)
        band_path = rospy.get_param("~safety_band")
        profile = str(rospy.get_param("~body_frame_profile"))
        self.pose_frame = str(rospy.get_param("~map_frame", "map"))
        self.lidar_in_body, self.lidar_rotation = lidar_extrinsics(profile)
        self.pose_correction = pose_correction(
            profile, str(route["body_frame_profile"])) @ reference_correction(
                str(route["reference_point"]), route_chair_centre(route))
        self.band = SafetyBand(band_path)
        centres, normals, left, right = corridor_arrays(band_path)
        self.corridor = Corridor(centres, normals, left, right)
        self.planner = make_planner(self.band)
        self.controller_limits = DEFAULT_CONTROLLER_LIMITS
        self.heading_pid = HeadingPid()

        self.enabled = False
        self.done = False
        self.pose_map = None
        self.centre_xy = None
        self.pose_yaw = 0.0
        self.pose_stamp = rospy.Time(0)
        self.prev_centre = None
        self.prev_stamp = None
        self.velocity = np.zeros(2)
        self.tracking_state = ""
        self.diag_stamp = rospy.Time(0)
        self.degraded_since = None
        self.drive_mode = None
        self.wheel_status_stamp = rospy.Time(0)
        self.cluster_summary = None
        self.motion = MotionEstimate(
            False, 0.0, 0.0, 0.0, 0.0, "ODOM_INITIALIZING")
        self.motion_estimator = PoseMotionEstimator(
            str(rospy.get_param("~odom_frame", "camera_init")),
            str(rospy.get_param("~base_frame", "body")))
        self.plan = None
        self.plan_stamp = rospy.Time(0)
        self.command_lock = threading.RLock()
        self.control_epoch = 0
        self.previous_command = DriveCommand(0.0, 0.0)
        self.current_speed = 0.0
        self.last_yaw_rate = 0.0
        self.status = "PAUSED"

        self.cmd_pub = rospy.Publisher(
            rospy.get_param("~cmd_topic", "/cmd_vel_raw"), Twist,
            queue_size=1)
        self.status_pub = rospy.Publisher(
            "/waypoint_follower/status", String, queue_size=2)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.on_pose, queue_size=5)
        rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber("/fast_lio_icp/localization_diagnostics",
                         DiagnosticArray, self.on_diag, queue_size=5)
        rospy.Subscriber("/wheel_status", Int16MultiArray,
                         self.on_wheel_status, queue_size=5)
        rospy.Subscriber("/perception/objects_summary", String,
                         self.on_clusters, queue_size=2)
        rospy.Service("/waypoint_follower/start", SetBool, self.on_start)
        rospy.on_shutdown(self.send_stop)
        rospy.loginfo("priest_follower: certified corridor %.1f m, guards ON",
                      self.corridor.length_m)

    def unpredictable_reason(self, now: rospy.Time) -> Optional[str]:
        trajectory = self.centre_xy[None, :]
        start_index = 0
        if self.plan is not None and self.plan.usable:
            arrays = plan_arrays(self.plan)
            if arrays is None:
                return OBSTACLE_WAIT
            trajectory = np.stack([arrays[0], arrays[1]], axis=1)
            times = arrays[2]
            elapsed = max(0.0, (now - self.plan_stamp).to_sec())
            start_index = int(np.clip(
                np.searchsorted(times, elapsed, side="right") - 1,
                0, len(trajectory) - 1))
        return runtime_wait_reason(
            self.cluster_summary, self.pose_map, self.lidar_in_body,
            self.lidar_rotation, self.centre_xy, trajectory, WAIT_RADIUS_M,
            trajectory_start_index=start_index)

    def track(self, now: rospy.Time) -> Optional[str]:
        plan = self.plan
        if plan is None:
            self.send_stop()
            return "NO_PLAN"
        elapsed = (now - self.plan_stamp).to_sec()
        measured = max(0.0, float(self.motion.linear_speed_mps))
        command = command_for(
            plan, elapsed,
            Pose2D(float(self.centre_xy[0]), float(self.centre_xy[1]),
                   float(self.pose_yaw)),
            measured, self.previous_command, self.controller_limits,
            steering=SteeringFeedback(
                self.heading_pid, float(self.motion.angular_speed_rps)))
        with self.command_lock:
            if not self.enabled or self.done or self.plan is not plan:
                reason = "DONE" if self.done else "PAUSED"
                self.send_stop()
                return reason
            if command.reason in ("AT_GOAL", "AT_PLAN_END"):
                self.send_stop()
                return "PLAN_COMPLETE"
            if command.reason not in (
                    "", "TURN_ACCELERATING", "TERMINAL_BRAKING"):
                self.send_stop()
                return command.reason
            safety_reason = self.command_safety_reason(command)
            if safety_reason is not None:
                self.send_stop()
                return safety_reason
            message = Twist()
            message.linear.x = command.linear_x_mps
            message.angular.z = command.angular_z_rps
            self.cmd_pub.publish(message)
            self.previous_command = command
            self.current_speed = command.linear_x_mps
            self.last_yaw_rate = command.angular_z_rps
        return None

    def step(self) -> None:
        now = rospy.Time.now()
        with self.command_lock:
            if self.tracking_state == "DEGRADED":
                if self.degraded_since is None:
                    self.degraded_since = now
            else:
                self.degraded_since = None
            reason = self.hold_reason(now)
            if reason is None and within_goal_tolerance(
                    float(np.linalg.norm(
                        self.corridor.centres[-1] - self.centre_xy)),
                    GOAL_TOLERANCE_M) \
                    and self.motion.linear_speed_mps <= STOPPED_SPEED_MPS \
                    and self.previous_command.linear_x_mps <= 1e-9:
                self.done = True
                reason = "DONE"
                rospy.loginfo("GOAL REACHED")
            if reason is None:
                reason = self.unpredictable_reason(now)
        if reason is None:
            planning_reason = self.ensure_plan(now)
        else:
            planning_reason = None
        execution_now = rospy.Time.now()
        with self.command_lock:
            reason = self.hold_reason(execution_now)
            if reason is None:
                reason = self.unpredictable_reason(execution_now)
            if reason is None and planning_reason is not None:
                reason = planning_reason
            if reason is None and self.plan is not None:
                elapsed = max(
                    0.0, (execution_now - self.plan_stamp).to_sec())
                reason = self.static_plan_reason(self.plan, elapsed)
            if reason is None:
                reason = self.track(execution_now)
            if reason:
                if reason != self.status:
                    rospy.loginfo("hold: %s", reason)
                    self.status = reason
                self.status_pub.publish(String(data="HOLD:" + reason))
                self.send_stop()
                return
            arc = self.corridor.arc_of(self.centre_xy)
            if not self.enabled or self.done or self.plan is None:
                reason = "DONE" if self.done else "PAUSED"
                self.status_pub.publish(String(data="HOLD:" + reason))
                self.status = reason
                self.send_stop()
                return
            self.status_pub.publish(String(
                data="DRIVING arc=%.1f/%.1f v=%.2f PRIEST" % (
                    arc, self.corridor.length_m, self.current_speed)))
            self.status = "DRIVING"

    def run(self) -> None:
        rate = rospy.Rate(CONTROL_HZ)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    PriestFollower().run()
