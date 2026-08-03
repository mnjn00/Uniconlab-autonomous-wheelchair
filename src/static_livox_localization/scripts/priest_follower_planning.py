"""Fail-closed planning and current-to-plan execution certification."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rospy

from priest_constraints import COMMAND_RETENTION_HORIZON_S
from priest_controller import (
    DriveCommand,
    STOPPED_SPEED_MPS,
    TimedPlan,
    plan_arrays,
)
from priest_execution_safety import (
    differential_drive_arc,
    oriented_footprint_contained,
)
from priest_follower_io import (
    CROSS_TRACK_REPLAN_M,
    FUTURE_TOLERANCE_S,
    OFF_BAND_GRACE,
)
from priest_runtime import (
    OBSTACLE_WAIT,
    execution_path,
    planner_obstacles,
    static_obstacles_clear,
)


MAX_LATERAL_STATE_MPS = 0.05
REVERSE_STATE_TOLERANCE_MPS = 0.02


class FollowerPlanningMixin:
    """Planning lifecycle separated from ROS callbacks and command output."""

    def static_plan_reason(
            self, plan: TimedPlan, elapsed_s: float = 0.0) -> Optional[str]:
        arrays = plan_arrays(plan)
        if arrays is None:
            return "INVALID_PLAN"
        trajectory = np.stack([arrays[0], arrays[1]], axis=1)
        executable = execution_path(
            self.centre_xy, trajectory, arrays[2], elapsed_s)
        if executable is None:
            return "INVALID_PLAN"
        try:
            yaw = np.asarray(plan.yaw_rad, dtype=np.float64)
            if yaw.shape != arrays[2].shape or not np.isfinite(yaw).all():
                return "INVALID_PLAN"
            contained = np.asarray(self.band.contains_many(
                executable, grace=OFF_BAND_GRACE))
            suffix = int(np.searchsorted(
                arrays[2], elapsed_s, side="right"))
            reference = np.array([
                np.interp(elapsed_s, arrays[2], arrays[0]),
                np.interp(elapsed_s, arrays[2], arrays[1])])
            reference_yaw = float(np.interp(
                elapsed_s, arrays[2], np.unwrap(yaw)))
            remaining_xy = np.vstack([reference, trajectory[suffix:]])
            remaining_yaw = np.concatenate([[reference_yaw], yaw[suffix:]])
            footprint_ok = oriented_footprint_contained(
                self.band, remaining_xy, remaining_yaw, OFF_BAND_GRACE)
            current_ok = oriented_footprint_contained(
                self.band, self.centre_xy[None, :],
                np.array([self.pose_yaw]), OFF_BAND_GRACE)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return "OFF_BAND"
        if contained.shape != (len(executable),) \
                or contained.dtype.kind != "b" or not bool(contained.all()) \
                or not bool(footprint_ok.all()) or not bool(current_ok.all()):
            return "OFF_BAND"
        summary = self.cluster_summary
        if summary is None or not summary.usable:
            return OBSTACLE_WAIT
        try:
            circles, dropped = planner_obstacles(
                summary.objects, self.pose_map, self.lidar_in_body,
                self.lidar_rotation, limit=self.planner.max_obstacles)
        except ValueError:
            return OBSTACLE_WAIT
        if dropped or not static_obstacles_clear(executable, circles):
            return OBSTACLE_WAIT
        return None

    def command_safety_reason(
            self, command: DriveCommand) -> Optional[str]:
        try:
            points, yaw = differential_drive_arc(
                float(self.centre_xy[0]), float(self.centre_xy[1]),
                float(self.pose_yaw), command.linear_x_mps,
                command.angular_z_rps,
                COMMAND_RETENTION_HORIZON_S)
            contained = oriented_footprint_contained(
                self.band, points, yaw, OFF_BAND_GRACE)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return "OFF_BAND"
        if not bool(contained.all()):
            return "OFF_BAND"
        summary = self.cluster_summary
        if summary is None or not summary.usable:
            return OBSTACLE_WAIT
        try:
            circles, dropped = planner_obstacles(
                summary.objects, self.pose_map, self.lidar_in_body,
                self.lidar_rotation, limit=self.planner.max_obstacles)
        except ValueError:
            return OBSTACLE_WAIT
        if dropped or not static_obstacles_clear(points, circles):
            return OBSTACLE_WAIT
        return None

    def ensure_plan(self, now: rospy.Time) -> Optional[str]:
        with self.command_lock:
            plan = self.plan
            age = (now - self.plan_stamp).to_sec()
            drifted = False
            if plan is not None and plan.usable:
                arrays = plan_arrays(plan)
                if arrays is None:
                    self.send_stop()
                    return "INVALID_PLAN"
                points = np.stack([arrays[0], arrays[1]], axis=1)
                drifted = float(np.min(np.linalg.norm(
                    points - self.centre_xy, axis=1))) \
                    > CROSS_TRACK_REPLAN_M
            replan = plan is None or age < -FUTURE_TOLERANCE_S \
                or drifted
            if not replan:
                return None if plan.usable else plan.reason or "INVALID_PLAN"
            self.send_stop()
            epoch = self.control_epoch
            summary = self.cluster_summary
            if summary is None or not summary.usable:
                return OBSTACLE_WAIT
            pose = np.asarray(self.pose_map, dtype=np.float64).copy()
            start = np.asarray(self.centre_xy, dtype=np.float64).copy()
            observed_velocity = np.asarray(
                self.velocity, dtype=np.float64).copy()
            heading = np.array([
                np.cos(self.pose_yaw), np.sin(self.pose_yaw)])
            lateral_axis = np.array([-heading[1], heading[0]])
            forward_speed = float(np.dot(observed_velocity, heading))
            lateral_speed = float(np.dot(observed_velocity, lateral_axis))
            if (not np.isfinite(observed_velocity).all()
                    or forward_speed < -REVERSE_STATE_TOLERANCE_MPS
                    or abs(lateral_speed) > MAX_LATERAL_STATE_MPS):
                return "NONHOLONOMIC_STATE"
            velocity = max(0.0, forward_speed) * heading
            acceleration = np.zeros(2)
            if forward_speed <= STOPPED_SPEED_MPS:
                acceleration = self.planner.a_max * np.array([
                    np.cos(self.pose_yaw), np.sin(self.pose_yaw)])
            try:
                circles, dropped = planner_obstacles(
                    summary.objects, pose, self.lidar_in_body,
                    self.lidar_rotation, limit=self.planner.max_obstacles)
            except ValueError:
                return OBSTACLE_WAIT
            if dropped:
                return OBSTACLE_WAIT
        started = rospy.Time.now()
        try:
            candidate = self.planner.plan(
                start, velocity, acceleration, self.corridor, circles,
                initial_yaw_rad=float(self.pose_yaw))
        except (ValueError, FloatingPointError, np.linalg.LinAlgError):
            with self.command_lock:
                self.send_stop()
            return "PLAN_ERROR"
        completed = rospy.Time.now()
        duration_s = (completed - started).to_sec()
        if duration_s > 0.5:
            rospy.logwarn("replan took %.2f s", duration_s)
        with self.command_lock:
            if self.control_epoch != epoch or not self.enabled or self.done:
                return "PLAN_SUPERSEDED"
            if candidate.reason == "AT_GOAL":
                return "PLAN_COMPLETE"
            if not candidate.usable:
                return candidate.reason or "INVALID_PLAN"
            reason = self.static_plan_reason(candidate)
            arrays = plan_arrays(candidate)
            if reason is not None or arrays is None:
                return reason or "INVALID_PLAN"
            points = np.stack([arrays[0], arrays[1]], axis=1)
            if float(np.min(np.linalg.norm(
                    points - self.centre_xy, axis=1))) \
                    > CROSS_TRACK_REPLAN_M:
                return "PLAN_SUPERSEDED"
            self.plan = candidate
            self.plan_stamp = completed
        return None
