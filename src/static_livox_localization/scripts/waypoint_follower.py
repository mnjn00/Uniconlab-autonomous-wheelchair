#!/usr/bin/env python3
"""Safety-guarded waypoint follower for the wheelchair.

Follows the curvature-adaptive route waypoints with pure pursuit while
enforcing, every control cycle:
  - drop guard: never drive toward a sudden ground drop (curb/road falloff)
    or an un-climbable step in the forward corridor
  - obstacle guard: slow down near obstacles/pedestrians, stop when close
  - stuck-obstacle bypass: if a static obstacle blocks the path for a while,
    side-step within a clear, drop-free lateral offset and rejoin
  - slope guard: reduced speed while pitched (campus hills)
  - safe speed policy: hard speed cap, curvature slowdown, accel limiting
  - dead-man guards: starts PAUSED until /waypoint_follower/start, stops on
    stale localization, LOST tracking, or manual joystick mode, and always
    sends a stop command on shutdown.
"""

import json
import math

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import SetBool, SetBoolResponse

import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft

MAX_SPEED = 0.5
SLOPE_SPEED = 0.3
CREEP_SPEED = 0.15
MAX_YAW_RATE = 0.5
MAX_ACCEL = 0.25
MAX_DECEL = 0.6
CONTROL_HZ = 10.0

CORRIDOR_HALF_WIDTH = 0.45
GUARD_STOP_M = 1.1
GUARD_SLOW_M = 2.2
DROP_STEP_M = 0.08
CLIMB_STEP_M = 0.12
OBSTACLE_MIN_Z = 0.18
OBSTACLE_MAX_Z = 1.9
SLOPE_PITCH_RAD = math.radians(3.0)
BYPASS_AFTER_S = 10.0
BYPASS_OFFSETS = (0.6, -0.6, 1.0, -1.0)
GOAL_TOLERANCE_M = 1.0
POSE_STALE_S = 1.0
AUTO_MODE = 65


class WaypointFollower:
    def __init__(self):
        rospy.init_node("waypoint_follower")
        route_path = rospy.get_param("~route")
        with open(route_path) as f:
            route = json.load(f)
        self.waypoints = np.array(
            [[w["x"], w["y"]] for w in route["waypoints"]], dtype=np.float64)
        rospy.loginfo("route: %d waypoints, %.0f m",
                      len(self.waypoints),
                      np.linalg.norm(
                          np.diff(self.waypoints, axis=0), axis=1).sum())

        self.enabled = False
        self.done = False
        self.pose_xy = None
        self.pose_yaw = 0.0
        self.pose_pitch = 0.0
        self.pose_stamp = rospy.Time(0)
        self.tracking_state = ""
        self.drive_mode = None
        self.cloud = None
        self.cloud_stamp = rospy.Time(0)
        self.nearest_index = 0
        self.current_speed = 0.0
        self.blocked_since = None
        self.lateral_offset = 0.0
        self.status = "PAUSED"

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.on_pose, queue_size=5)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.on_cloud, queue_size=2)
        rospy.Subscriber("/fast_lio_icp/localization_diagnostics",
                         DiagnosticArray, self.on_diag, queue_size=5)
        rospy.Subscriber("/wheel_status", Int16MultiArray,
                         self.on_wheel_status, queue_size=5)
        rospy.Service("/waypoint_follower/start", SetBool, self.on_start)
        rospy.on_shutdown(self.send_stop)

    # ------------------------------------------------------------ callbacks
    def on_pose(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_xy = np.array([p.x, p.y])
        self.pose_yaw = yaw
        self.pose_pitch = pitch
        self.pose_stamp = message.header.stamp

    def on_cloud(self, message):
        pts = np.array(list(pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32)
        self.cloud = pts
        self.cloud_stamp = message.header.stamp

    def on_diag(self, message):
        for status in message.status:
            if status.name == "fast_lio_icp":
                self.tracking_state = status.message

    def on_wheel_status(self, message):
        if len(message.data) > 1:
            self.drive_mode = message.data[1]

    def on_start(self, request):
        self.enabled = request.data
        if not request.data:
            self.send_stop()
        rospy.loginfo("follower %s", "ENABLED" if self.enabled else "PAUSED")
        return SetBoolResponse(success=True,
                               message="ENABLED" if self.enabled else "PAUSED")

    # ------------------------------------------------------------ safety
    def corridor_assessment(self, lateral_shift=0.0):
        """Return (drop_dist, obstacle_dist): nearest blocking distances in
        the forward corridor, or None when clear. Body frame, +x forward."""
        if self.cloud is None or len(self.cloud) < 100:
            return 0.0, 0.0  # no data = treat as blocked
        pts = self.cloud
        near = pts[(pts[:, 0] > -0.6) & (pts[:, 0] < 1.0) &
                   (np.abs(pts[:, 1]) < 0.7)]
        if len(near) < 20:
            return 0.0, 0.0
        ego_ground = np.percentile(near[:, 2], 10)

        m = ((pts[:, 0] > 0.25) & (pts[:, 0] < GUARD_SLOW_M + 0.6) &
             (np.abs(pts[:, 1] - lateral_shift) < CORRIDOR_HALF_WIDTH))
        zone = pts[m]
        drop_dist = None
        obstacle_dist = None
        if len(zone):
            rel_z = zone[:, 2] - ego_ground
            obstacles = zone[(rel_z > OBSTACLE_MIN_Z) & (rel_z < OBSTACLE_MAX_Z)]
            if len(obstacles) >= 5:
                obstacle_dist = float(np.percentile(obstacles[:, 0], 5))

        # ground continuity in 0.25 m bins ahead
        for lo in np.arange(0.3, GUARD_SLOW_M, 0.25):
            band = pts[(pts[:, 0] >= lo) & (pts[:, 0] < lo + 0.25) &
                       (np.abs(pts[:, 1] - lateral_shift) < CORRIDOR_HALF_WIDTH)]
            ground_band = band[band[:, 2] - ego_ground < OBSTACLE_MIN_Z]
            if len(ground_band) < 3:
                if lo < 1.4:
                    drop_dist = drop_dist if drop_dist is not None else float(lo)
                continue
            step = np.percentile(ground_band[:, 2], 15) - ego_ground
            if step < -DROP_STEP_M or step > CLIMB_STEP_M:
                drop_dist = float(lo)
                break
        return drop_dist, obstacle_dist

    def send_stop(self):
        self.current_speed = 0.0
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------ control
    def pure_pursuit_target(self):
        d = np.linalg.norm(self.waypoints - self.pose_xy, axis=1)
        window_end = min(self.nearest_index + 15, len(self.waypoints))
        self.nearest_index = int(
            self.nearest_index + np.argmin(d[self.nearest_index:window_end]))
        lookahead = 1.0 + 1.6 * self.current_speed
        target = self.waypoints[-1].copy()
        acc = 0.0
        for i in range(self.nearest_index, len(self.waypoints) - 1):
            acc += np.linalg.norm(self.waypoints[i + 1] - self.waypoints[i])
            if acc >= lookahead:
                target = self.waypoints[i + 1].copy()
                break
        if abs(self.lateral_offset) > 0.01:
            direction = target - self.pose_xy
            n = np.linalg.norm(direction)
            if n > 1e-3:
                normal = np.array([-direction[1], direction[0]]) / n
                target = target + normal * self.lateral_offset
        return target

    def step(self):
        now = rospy.Time.now()
        reason = None
        if not self.enabled or self.done:
            reason = "DONE" if self.done else "PAUSED"
        elif self.pose_xy is None or (now - self.pose_stamp).to_sec() > POSE_STALE_S:
            reason = "NO_POSE"
        elif (now - self.cloud_stamp).to_sec() > 1.0:
            reason = "NO_CLOUD"
        elif self.tracking_state == "LOST":
            reason = "LOCALIZATION_LOST"
        elif self.drive_mode is not None and self.drive_mode != AUTO_MODE:
            reason = "MANUAL_MODE"
        if reason:
            if reason != self.status:
                rospy.loginfo("hold: %s", reason)
                self.status = reason
            self.send_stop()
            return

        if np.linalg.norm(self.waypoints[-1] - self.pose_xy) < GOAL_TOLERANCE_M:
            self.done = True
            self.send_stop()
            rospy.loginfo("GOAL REACHED")
            return

        drop_dist, obstacle_dist = self.corridor_assessment(self.lateral_offset)

        allowed = MAX_SPEED
        if abs(self.pose_pitch) > SLOPE_PITCH_RAD:
            allowed = min(allowed, SLOPE_SPEED)
        if self.tracking_state == "DEGRADED":
            allowed = min(allowed, SLOPE_SPEED)

        blocking = None
        for dist, kind in ((drop_dist, "DROP"), (obstacle_dist, "OBSTACLE")):
            if dist is None:
                continue
            if dist < GUARD_STOP_M:
                blocking = kind
                allowed = 0.0
            elif dist < GUARD_SLOW_M:
                ratio = (dist - GUARD_STOP_M) / (GUARD_SLOW_M - GUARD_STOP_M)
                allowed = min(allowed, CREEP_SPEED + ratio * (MAX_SPEED - CREEP_SPEED))

        if blocking == "OBSTACLE":
            if self.blocked_since is None:
                self.blocked_since = now
            elif (now - self.blocked_since).to_sec() > BYPASS_AFTER_S and \
                    abs(self.lateral_offset) < 0.01:
                for offset in BYPASS_OFFSETS:
                    d2, o2 = self.corridor_assessment(offset)
                    if (d2 is None or d2 > GUARD_SLOW_M) and \
                            (o2 is None or o2 > GUARD_SLOW_M):
                        self.lateral_offset = offset
                        rospy.logwarn("bypassing static obstacle: offset %+.1f m",
                                      offset)
                        break
                else:
                    rospy.logwarn_throttle(10, "path blocked, no clear side - waiting")
        elif blocking is None:
            self.blocked_since = None
            if abs(self.lateral_offset) > 0.01:
                d0, o0 = self.corridor_assessment(0.0)
                if (d0 is None or d0 > GUARD_SLOW_M) and \
                        (o0 is None or o0 > GUARD_SLOW_M):
                    self.lateral_offset = 0.0
                    rospy.loginfo("bypass complete, rejoining route")

        target = self.pure_pursuit_target()
        to_target = target - self.pose_xy
        heading = math.atan2(to_target[1], to_target[0])
        heading_error = math.atan2(math.sin(heading - self.pose_yaw),
                                   math.cos(heading - self.pose_yaw))
        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, 1.2 * heading_error))
        allowed = min(allowed, max(0.12, MAX_SPEED * (1.0 - abs(heading_error) / 1.2)))
        if blocking:
            allowed = 0.0

        dt = 1.0 / CONTROL_HZ
        if allowed >= self.current_speed:
            self.current_speed = min(allowed, self.current_speed + MAX_ACCEL * dt)
        else:
            self.current_speed = max(allowed, self.current_speed - MAX_DECEL * dt)

        command = Twist()
        command.linear.x = self.current_speed
        command.angular.z = yaw_rate if self.current_speed > 0.02 else 0.0
        self.cmd_pub.publish(command)

        state = blocking or ("BYPASS" if abs(self.lateral_offset) > 0.01 else "DRIVING")
        if state != self.status:
            rospy.loginfo("state: %s (wp %d/%d, v=%.2f)",
                          state, self.nearest_index, len(self.waypoints),
                          self.current_speed)
            self.status = state

    def run(self):
        rate = rospy.Rate(CONTROL_HZ)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    WaypointFollower().run()
