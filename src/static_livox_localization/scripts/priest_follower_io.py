"""ROS callbacks and fail-closed state transitions for PRIEST."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import SetBoolRequest, SetBoolResponse
import tf.transformations as tft

from cluster_guard import is_stale, parse_summary
from localization_policy import localization_hold_reason
from motion_safety import motion_hold_reason
from priest_controller import DriveCommand


MAX_SPEED = 0.6
MAX_YAW_RATE = 0.5
MAX_ACCEL = 0.18
MAX_DECEL = 0.6
CONTROL_HZ = 5.0
CERTIFICATE_HZ = 10.0
TURN_FLOOR_SPEED = 0.30
POSE_STALE_S = 1.0
BASE_STALE_S = 1.5
ODOM_STALE_S = 0.35
DIAG_STALE_S = 1.5
AUTO_MODE = 65
DEGRADED_STOP_S = 3.0
GOAL_TOLERANCE_M = 0.05
OFF_BAND_GRACE = 0.10
CROSS_TRACK_REPLAN_M = 0.5
FUTURE_TOLERANCE_S = 0.05


def require_guards(safety_policies_enabled: bool) -> None:
    if not safety_policies_enabled:
        raise ValueError(
            "priest_follower has no diagnostic mode: safety guards required")


class FollowerIOMixin:
    """Callbacks shared by the thin node; all holds reset executable state."""

    def _invalidate_pose(self) -> None:
        with self.command_lock:
            self.pose_map = None
            self.centre_xy = None
            self.pose_stamp = rospy.Time(0)
            self.prev_centre = self.prev_stamp = None
            self.velocity = np.zeros(2)
            self.send_stop()

    def on_pose(self, message: PoseWithCovarianceStamped) -> None:
        try:
            position = message.pose.pose.position
            quaternion = message.pose.pose.orientation
            values = np.array([
                position.x, position.y, position.z, quaternion.x,
                quaternion.y, quaternion.z, quaternion.w], dtype=np.float64)
            stamp = message.header.stamp
            stamp_s = float(stamp.to_sec())
            receipt_s = float(rospy.Time.now().to_sec())
            frame_id = str(message.header.frame_id)
        except (AttributeError, TypeError, ValueError):
            self._invalidate_pose()
            return
        quaternion_xyzw = values[3:]
        quaternion_norm = float(np.linalg.norm(quaternion_xyzw))
        if (not np.isfinite(values).all() or not math.isfinite(stamp_s)
                or not math.isfinite(receipt_s)
                or stamp_s - receipt_s > FUTURE_TOLERANCE_S
                or quaternion_norm <= 1e-6 or frame_id != self.pose_frame):
            self._invalidate_pose()
            return
        pose = tft.quaternion_matrix(quaternion_xyzw / quaternion_norm)
        pose[:3, 3] = values[:3]
        corrected = pose @ self.pose_correction
        centre = np.array([corrected[0, 3], corrected[1, 3]])
        with self.command_lock:
            if self.prev_stamp is not None \
                    and stamp_s <= float(self.prev_stamp.to_sec()):
                self._invalidate_pose()
                return
            if self.prev_centre is not None and self.prev_stamp is not None:
                dt = (stamp - self.prev_stamp).to_sec()
                if 1e-3 < dt < 1.0:
                    velocity = (centre - self.prev_centre) / dt
                    speed = float(np.linalg.norm(velocity))
                    self.velocity = velocity if speed <= MAX_SPEED else \
                        velocity * (MAX_SPEED / speed)
            self.prev_centre, self.prev_stamp = centre, stamp
            self.centre_xy = centre
            self.pose_yaw = math.atan2(corrected[1, 0], corrected[0, 0])
            self.pose_stamp = stamp
            self.pose_map = pose

    def on_odom(self, message: Odometry) -> None:
        quaternion = message.pose.pose.orientation
        position = message.pose.pose.position
        estimate = self.motion_estimator.update(
            source_stamp_s=message.header.stamp.to_sec(),
            receipt_stamp_s=rospy.Time.now().to_sec(),
            frame_id=message.header.frame_id,
            child_frame_id=message.child_frame_id,
            x=position.x, y=position.y,
            quaternion_xyzw=(quaternion.x, quaternion.y,
                             quaternion.z, quaternion.w))
        with self.command_lock:
            self.motion = estimate

    def on_diag(self, message: DiagnosticArray) -> None:
        state = ""
        try:
            for status in message.status:
                if status.name == "fast_lio_icp":
                    state = status.message
                    break
            stamp = message.header.stamp if state else rospy.Time(0)
            stamp_s = float(stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            state, stamp, stamp_s = "", rospy.Time(0), 0.0
        now_s = float(rospy.Time.now().to_sec())
        if not math.isfinite(stamp_s) or stamp_s - now_s \
                > FUTURE_TOLERANCE_S:
            state, stamp = "", rospy.Time(0)
        with self.command_lock:
            self.tracking_state = state
            self.diag_stamp = stamp

    def on_wheel_status(self, message: Int16MultiArray) -> None:
        with self.command_lock:
            if len(message.data) <= 1:
                self.drive_mode = None
                return
            self.wheel_status_stamp = rospy.Time.now()
            self.drive_mode = message.data[1]

    def on_clusters(self, message: String) -> None:
        try:
            summary = parse_summary(message.data)
        except ValueError as error:
            summary = None
            rospy.logwarn_throttle(
                5.0, "objects_summary unreadable: %s", error)
        with self.command_lock:
            self.cluster_summary = summary

    def on_start(self, request: SetBoolRequest) -> SetBoolResponse:
        with self.command_lock:
            self.control_epoch += 1
            self.enabled = request.data
            if not request.data:
                self.send_stop()
        state = "ENABLED" if self.enabled else "PAUSED"
        rospy.loginfo("priest follower %s", state)
        return SetBoolResponse(success=True, message=state)

    def send_stop(self) -> None:
        with self.command_lock:
            self.control_epoch += 1
            self.current_speed = 0.0
            self.last_yaw_rate = 0.0
            self.previous_command = DriveCommand(0.0, 0.0)
            self.heading_pid.reset()
            self.plan = None
            self.cmd_pub.publish(Twist())

    def hold_reason(self, now: rospy.Time) -> Optional[str]:
        if not self.enabled or self.done:
            return "DONE" if self.done else "PAUSED"
        pose_age = (now - self.pose_stamp).to_sec()
        if self.centre_xy is None or not math.isfinite(pose_age) \
                or pose_age < -FUTURE_TOLERANCE_S or pose_age > POSE_STALE_S:
            return "NO_POSE"
        reason = motion_hold_reason(self.motion, now.to_sec(), ODOM_STALE_S)
        if reason:
            return reason
        diag_age = (now - self.diag_stamp).to_sec()
        if not math.isfinite(diag_age) or diag_age < -FUTURE_TOLERANCE_S \
                or diag_age > DIAG_STALE_S:
            return "LOCALIZATION_NOT_TRACKING"
        degraded_age = None if self.degraded_since is None else \
            (now - self.degraded_since).to_sec()
        reason = localization_hold_reason(
            self.tracking_state, degraded_age, DEGRADED_STOP_S)
        if reason:
            return reason
        wheel_age = (now - self.wheel_status_stamp).to_sec()
        if not math.isfinite(wheel_age) or wheel_age < -FUTURE_TOLERANCE_S \
                or wheel_age > BASE_STALE_S:
            return "BASE_STALE"
        if self.drive_mode != AUTO_MODE:
            return "MANUAL_MODE"
        summary = self.cluster_summary
        stamp = None if summary is None else summary.stamp_s
        if is_stale(stamp, now.to_sec()) or stamp is not None \
                and stamp - now.to_sec() > FUTURE_TOLERANCE_S:
            return "CLUSTERS_STALE"
        if not self.band.contains(self.centre_xy, grace=OFF_BAND_GRACE):
            return "OFF_BAND"
        return None
