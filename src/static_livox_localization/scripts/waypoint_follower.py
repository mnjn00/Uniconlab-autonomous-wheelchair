#!/usr/bin/env python3
"""Safety-guarded waypoint follower for the wheelchair.

Drop safety comes from the map, not the live scan: the MID360 (vertical FOV
-7..+52 deg, ~0.3 m mount) cannot see ground within ~2.4 m, so curbs are
avoided by keeping the wheelchair inside the pre-computed drop-free lateral
band along the route (tools/make_route_safety_band.py). The live accumulated
scan is used for what the sensor CAN see: obstacles and pedestrians.

Per control cycle:
  - band containment: the current position must lie inside the safety band;
    steering targets and bypass offsets are clamped into the band
  - obstacle guard: slow near obstacles/pedestrians, stop when close
  - stuck-obstacle bypass: after 10 s, side-step within the band only
  - slope guard and bounded DEGRADED-localization grace, tilt aborts
  - speed policy: 0.5 m/s cap, curvature slowdown, accel/yaw-rate limiting
  - dead-man guards: starts PAUSED until /waypoint_follower/start, holds on
    stale pose/cloud/base, LOST or sustained DEGRADED localization, manual
    joystick mode, or geofence violation, and always stops on shutdown.
"""

import json
import math

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse

import sensor_msgs.point_cloud2 as pc2
from localization_policy import localization_hold_reason
from body_frame import body_to_lidar, lidar_extrinsics
from safety_band import SafetyBand
import tf.transformations as tft

MAX_SPEED = 1.1
SLOPE_SPEED = 0.3
CREEP_SPEED = 0.15
MAX_YAW_RATE = 0.5
MAX_ACCEL = 0.18
MAX_DECEL = 0.6
CONTROL_HZ = 10.0

CORRIDOR_HALF_WIDTH = 0.45
# Obstacle guard distances scale with speed: at MAX_SPEED the chair
# needs v^2/(2*MAX_DECEL) = 1.0 m to brake, plus up to ~1.2 s of
# sensing+control latency (1 s cloud accumulation, 10 Hz control) at
# 1.1 m/s = 1.3 m more. A fixed 1.1 m stop radius was sized for the
# old 0.5 m/s cap and is not enough once the chair cruises faster.
GUARD_STOP_MIN_M = 0.9
GUARD_STOP_PER_MPS = 1.4
GUARD_SLOW_EXTRA_M = 1.2
OBSTACLE_MIN_Z = 0.18
OBSTACLE_MAX_Z = 1.9
NARROW_SPEED = 0.2
OFF_BAND_GRACE = 0.10
SLOPE_PITCH_RAD = math.radians(3.0)
# People clear the path on their own within a couple of seconds, so a
# blocker still there after 3 s is treated as scenery and side-stepped.
# Anything that moves out of the corridor sooner simply stops blocking
# and the chair resumes without ever planning a bypass.
BYPASS_AFTER_S = 3.0
BYPASS_OFFSETS = (0.6, -0.6, 1.0, -1.0)
GOAL_TOLERANCE_M = 1.0
POSE_STALE_S = 1.0
BASE_STALE_S = 1.5
MAX_TILT_ROLL = math.radians(6.0)
MAX_TILT_PITCH = math.radians(8.0)
BAND_RECOVER_MAX = OFF_BAND_GRACE
GEOFENCE_M = 3.5
AUTO_MODE = 65
DEGRADED_STOP_S = 3.0
NEAREST_RESYNC_M = 2.0
MIN_LOOKAHEAD_M = 0.9
LOOKAHEAD_BACKOFF_M = 0.4


class CloudAccumulator:
    """Merge ~1 s of sparse MID360 scans into the current body frame."""

    def __init__(self, window_s=1.0):
        self.window_s = window_s
        self.scans = []
        self.odoms = []

    def add_odom(self, message):
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = (p.x, p.y, p.z)
        self.odoms.append((message.header.stamp.to_sec(), T))
        self.odoms = self.odoms[-60:]

    def nearest_odom(self, stamp):
        if not self.odoms:
            return None
        times = np.array([t for t, _ in self.odoms])
        k = int(np.argmin(np.abs(times - stamp)))
        if abs(times[k] - stamp) > 0.15:
            return None
        return self.odoms[k][1]

    def add_cloud(self, message):
        pts = np.array(list(pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32)
        stamp = message.header.stamp.to_sec()
        self.scans.append((stamp, pts))
        self.scans = [s for s in self.scans
                      if stamp - s[0] <= self.window_s + 0.3]

    def merged(self):
        if not self.scans:
            return None, rospy.Time(0)
        newest = self.scans[-1][0]
        T_ref = self.nearest_odom(newest)
        if T_ref is None:
            return None, rospy.Time(0)
        inv_ref = np.linalg.inv(T_ref)
        parts = []
        for stamp, pts in self.scans:
            if newest - stamp > self.window_s or not len(pts):
                continue
            T = self.nearest_odom(stamp)
            if T is None:
                continue
            M = (inv_ref @ T).astype(np.float32)
            parts.append(pts @ M[:3, :3].T + M[:3, 3])
        if not parts:
            return None, rospy.Time(0)
        return np.vstack(parts), rospy.Time.from_sec(newest)




class WaypointFollower:
    def __init__(self):
        rospy.init_node("waypoint_follower")
        with open(rospy.get_param("~route")) as f:
            route = json.load(f)
        self.waypoints = np.array(
            [[w["x"], w["y"]] for w in route["waypoints"]], dtype=np.float64)
        self.band = SafetyBand(rospy.get_param("~safety_band"))
        self.sensor_height = rospy.get_param("~sensor_height", 0.30)
        # /cloud_registered_body arrives in FAST-LIO's IMU BODY frame,
        # which is only the lidar frame when FAST-LIO runs on the lidar's
        # built-in IMU. With the VN-100 driving it the body origin sits
        # 14.5 cm behind and 6.8 cm below the lidar and is yawed 2.80 deg,
        # and every geometry constant below (sensor height, corridor
        # half-width, stop distances) was tuned in the lidar/chair frame.
        # Left uncorrected that reads obstacles 14.5 cm farther away than
        # they are and skews the corridor by 0.17 m at 3.4 m - more than
        # the chair's own half-width. The profile is required rather than
        # defaulted so a silent mismatch is impossible.
        self.body_offset, self.body_rotation = lidar_extrinsics(
            rospy.get_param("~body_frame_profile"))
        rospy.loginfo("route: %d waypoints, band stations: %d",
                      len(self.waypoints), len(self.band.xy))

        self.enabled = False
        self.done = False
        self.pose_xy = None
        self.pose_yaw = 0.0
        self.pose_pitch = 0.0
        self.pose_roll = 0.0
        self.pose_stamp = rospy.Time(0)
        self.tracking_state = ""
        self.degraded_since = None
        self.drive_mode = None
        self.wheel_status_stamp = rospy.Time(0)
        self.route_locked = False
        self.accumulator = CloudAccumulator()
        self.cloud = None
        self.cloud_stamp = rospy.Time(0)
        self.nearest_index = 0
        self.current_speed = 0.0
        self.blocked_since = None
        self.lateral_offset = 0.0
        self.chord_speed_cap = MAX_SPEED
        self.chord_safe = True
        self.last_yaw_rate = 0.0
        self.status = "PAUSED"

        cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel_raw")
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/waypoint_follower/status", String, queue_size=2)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.on_pose, queue_size=5)
        rospy.Subscriber("/cloud_registered_body", PointCloud2,
                         self.on_cloud, queue_size=2)
        rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=50)
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
        self.pose_roll = roll
        self.pose_stamp = message.header.stamp

    def on_cloud(self, message):
        self.accumulator.add_cloud(message)
        cloud, self.cloud_stamp = self.accumulator.merged()
        self.cloud = None if cloud is None else body_to_lidar(
            cloud, self.body_offset, self.body_rotation)

    def on_odom(self, message):
        self.accumulator.add_odom(message)

    def on_diag(self, message):
        for status in message.status:
            if status.name == "fast_lio_icp":
                self.tracking_state = status.message

    def on_wheel_status(self, message):
        self.wheel_status_stamp = rospy.Time.now()
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
    def obstacle_distance(self, lateral_shift=0.0):
        """Nearest obstacle in the forward corridor from the live scan,
        or None. The scan sees people and objects, not near ground."""
        if self.cloud is None or len(self.cloud) < 100:
            return 0.0  # no data = treat as blocked
        pts = self.cloud
        ground_plane = -self.sensor_height
        # The scan window has to reach past the slow radius, or the chair
        # meets an obstacle already inside its own braking distance with
        # no room left to have slowed down for it first.
        window = self.guard_stop() + GUARD_SLOW_EXTRA_M + 0.6
        m = ((pts[:, 0] > 0.25) & (pts[:, 0] < window) &
             (np.abs(pts[:, 1] - lateral_shift) < CORRIDOR_HALF_WIDTH))
        zone = pts[m]
        if not len(zone):
            return None
        rel = zone[:, 2] - ground_plane
        obstacles = zone[(rel > OBSTACLE_MIN_Z) & (rel < OBSTACLE_MAX_Z)]
        if len(obstacles) < 5:
            return None
        return float(np.percentile(obstacles[:, 0], 5))

    def bypass_target_ok(self, offset):
        """A lateral bypass is allowed only if the offset corridor stays
        inside the safety band for the next few meters."""
        if self.pose_xy is None:
            return False
        heading = np.array([math.cos(self.pose_yaw), math.sin(self.pose_yaw)])
        normal = np.array([-heading[1], heading[0]])
        for ahead in (0.5, 1.5, 2.5, 3.5):
            p = self.pose_xy + heading * ahead + normal * offset
            if not self.band.contains(p):
                return False
        return True

    def send_stop(self):
        self.current_speed = 0.0
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------ control
    def guard_stop(self):
        """Stop radius for the CURRENT speed: braking distance plus the
        distance covered while a new obstacle is still working its way
        through cloud accumulation and the control cycle."""
        return GUARD_STOP_MIN_M + GUARD_STOP_PER_MPS * max(
            0.0, self.current_speed)

    def pure_pursuit_target(self):
        d = np.linalg.norm(self.waypoints - self.pose_xy, axis=1)
        if not self.route_locked:
            self.nearest_index = int(np.argmin(d))
            self.route_locked = True
        window_end = min(self.nearest_index + 15, len(self.waypoints))
        windowed_index = int(
            self.nearest_index + np.argmin(d[self.nearest_index:window_end]))
        global_index = int(np.argmin(d))
        if d[global_index] + NEAREST_RESYNC_M < d[windowed_index]:
            rospy.logwarn(
                "waypoint_follower: position diverged from windowed search "
                "(wp %d, %.1fm) vs global nearest (wp %d, %.1fm) - resyncing",
                windowed_index, d[windowed_index],
                global_index, d[global_index])
            self.nearest_index = global_index
        else:
            self.nearest_index = windowed_index

        wanted = 1.0 + 1.6 * self.current_speed
        target, self.chord_speed_cap, self.chord_safe = \
            self.safe_target(wanted)
        return target

    def route_projection(self):
        """Nearest forward route-segment projection around current progress."""
        first = max(0, self.nearest_index - 1)
        last = min(len(self.waypoints) - 1, self.nearest_index + 1)
        best = None
        for i in range(first, last):
            start = self.waypoints[i]
            delta = self.waypoints[i + 1] - start
            length_sq = float(np.dot(delta, delta))
            if length_sq < 1e-12:
                continue
            fraction = float(np.clip(
                np.dot(self.pose_xy - start, delta) / length_sq, 0.0, 1.0))
            point = start + fraction * delta
            distance = float(np.linalg.norm(self.pose_xy - point))
            if best is None or distance < best[0]:
                best = (distance, i, point)
        if best is None:
            return min(self.nearest_index, len(self.waypoints) - 2), \
                self.waypoints[self.nearest_index].copy()
        return best[1], best[2]

    def lookahead_point(self, lookahead):
        """Interpolate lookahead from the chair's route projection."""
        segment_index, projection = self.route_projection()
        remaining = lookahead
        start = projection
        for i in range(segment_index, len(self.waypoints) - 1):
            end = self.waypoints[i + 1]
            segment = float(np.linalg.norm(end - start))
            if segment >= 1e-6:
                if remaining <= segment:
                    return start + (end - start) * (remaining / segment)
                remaining -= segment
            start = end
        return self.waypoints[-1].copy()

    def target_at_lookahead(self, lookahead):
        target = self.lookahead_point(lookahead)
        if abs(self.lateral_offset) > 0.01:
            direction = target - self.pose_xy
            norm = np.linalg.norm(direction)
            if norm > 1e-3:
                normal = np.array([-direction[1], direction[0]]) / norm
                target = target + normal * self.lateral_offset
        return self.band.clamp(target)

    def safe_target(self, wanted):
        """Return the longest target whose complete drive chord is in band."""
        candidate = max(MIN_LOOKAHEAD_M, wanted)
        while True:
            target = self.target_at_lookahead(candidate)
            if self.band.chord_is_contained(
                    self.pose_xy, target, grace=OFF_BAND_GRACE):
                implied_speed = max(
                    CREEP_SPEED, (candidate - 1.0) / 1.6)
                speed_cap = MAX_SPEED if candidate >= wanted - 1e-9 else \
                    min(MAX_SPEED, implied_speed)
                return target, speed_cap, True
            if candidate <= MIN_LOOKAHEAD_M + 1e-9:
                break
            candidate = max(
                MIN_LOOKAHEAD_M, candidate - LOOKAHEAD_BACKOFF_M)

        target = self.target_at_lookahead(MIN_LOOKAHEAD_M)
        rospy.logerr_throttle(
            5.0, "waypoint_follower: no band-safe chord - holding")
        return target, 0.0, False

    def step(self):
        now = rospy.Time.now()
        if self.tracking_state == "DEGRADED":
            if self.degraded_since is None:
                self.degraded_since = now
        else:
            self.degraded_since = None
        reason = None
        if not self.enabled or self.done:
            reason = "DONE" if self.done else "PAUSED"
        elif self.pose_xy is None or \
                (now - self.pose_stamp).to_sec() > POSE_STALE_S:
            reason = "NO_POSE"
        elif (now - self.cloud_stamp).to_sec() > 1.0:
            reason = "NO_CLOUD"
        else:
            degraded_age_s = None if self.degraded_since is None else \
                (now - self.degraded_since).to_sec()
            reason = localization_hold_reason(
                self.tracking_state, degraded_age_s, DEGRADED_STOP_S)
        if reason is None and \
                (now - self.wheel_status_stamp).to_sec() > BASE_STALE_S:
            reason = "BASE_STALE"
        elif reason is None and self.drive_mode is not None and \
                self.drive_mode != AUTO_MODE:
            reason = "MANUAL_MODE"
        elif reason is None and (
                abs(self.pose_roll) > MAX_TILT_ROLL or
                abs(self.pose_pitch) > MAX_TILT_PITCH):
            reason = "TILT_LIMIT"
        elif reason is None and self.route_locked and np.min(np.linalg.norm(
                self.waypoints - self.pose_xy, axis=1)) > GEOFENCE_M:
            reason = "OFF_ROUTE"
        elif reason is None and self.route_locked and not self.band.contains(
                self.pose_xy, grace=BAND_RECOVER_MAX):
            reason = "OFF_BAND"
        if reason:
            if reason != self.status:
                rospy.loginfo("hold: %s", reason)
                self.status = reason
            self.status_pub.publish(String(data="HOLD:" + reason))
            self.send_stop()
            return

        if np.linalg.norm(self.waypoints[-1] - self.pose_xy) < GOAL_TOLERANCE_M:
            self.done = True
            self.send_stop()
            rospy.loginfo("GOAL REACHED")
            return

        recovering = self.route_locked and not self.band.contains(
            self.pose_xy, grace=OFF_BAND_GRACE)

        obstacle_dist = self.obstacle_distance(self.lateral_offset)

        allowed = MAX_SPEED
        if recovering:
            allowed = min(allowed, CREEP_SPEED)
        if self.band.is_narrow(self.pose_xy):
            allowed = min(allowed, NARROW_SPEED)
        if abs(self.pose_pitch) > SLOPE_PITCH_RAD:
            allowed = min(allowed, SLOPE_SPEED)
        if self.tracking_state == "DEGRADED":
            allowed = min(allowed, SLOPE_SPEED)

        blocking = None
        guard_stop = self.guard_stop()
        guard_slow = guard_stop + GUARD_SLOW_EXTRA_M
        if obstacle_dist is not None:
            if obstacle_dist < guard_stop:
                blocking = "OBSTACLE"
                allowed = 0.0
            elif obstacle_dist < guard_slow:
                ratio = (obstacle_dist - guard_stop) / GUARD_SLOW_EXTRA_M
                allowed = min(allowed,
                              CREEP_SPEED + ratio * (MAX_SPEED - CREEP_SPEED))

        if blocking == "OBSTACLE":
            if self.blocked_since is None:
                self.blocked_since = now
            elif (now - self.blocked_since).to_sec() > BYPASS_AFTER_S and \
                    abs(self.lateral_offset) < 0.01:
                for offset in BYPASS_OFFSETS:
                    clear = self.obstacle_distance(offset)
                    if (clear is None or clear > guard_slow) and \
                            self.bypass_target_ok(offset):
                        self.lateral_offset = offset
                        rospy.logwarn(
                            "bypassing static obstacle: offset %+.1f m",
                            offset)
                        break
                else:
                    rospy.logwarn_throttle(
                        10, "path blocked, no clear side - waiting")
        elif blocking is None:
            self.blocked_since = None
            if abs(self.lateral_offset) > 0.01:
                back = self.obstacle_distance(0.0)
                if back is None or back > guard_slow:
                    self.lateral_offset = 0.0
                    rospy.loginfo("bypass complete, rejoining route")

        target = self.pure_pursuit_target()
        if not self.chord_safe:
            self.status = "UNSAFE_CHORD"
            self.status_pub.publish(String(data="HOLD:UNSAFE_CHORD"))
            self.send_stop()
            return
        allowed = min(allowed, self.chord_speed_cap)
        to_target = target - self.pose_xy
        heading = math.atan2(to_target[1], to_target[0])
        heading_error = math.atan2(math.sin(heading - self.pose_yaw),
                                   math.cos(heading - self.pose_yaw))
        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, 1.2 * heading_error))
        slew = 1.5 / CONTROL_HZ
        yaw_rate = max(self.last_yaw_rate - slew,
                       min(self.last_yaw_rate + slew, yaw_rate))
        self.last_yaw_rate = yaw_rate
        allowed = min(allowed,
                      max(0.12, MAX_SPEED * (1.0 - abs(heading_error) / 1.2)))
        if blocking:
            allowed = 0.0

        dt = 1.0 / CONTROL_HZ
        if allowed >= self.current_speed:
            self.current_speed = min(allowed,
                                     self.current_speed + MAX_ACCEL * dt)
        else:
            self.current_speed = max(allowed,
                                     self.current_speed - MAX_DECEL * dt)

        command = Twist()
        command.linear.x = self.current_speed
        command.angular.z = yaw_rate if self.current_speed > 0.02 else 0.0
        self.cmd_pub.publish(command)

        state = blocking or (
            "RECOVER" if recovering else
            ("BYPASS" if abs(self.lateral_offset) > 0.01 else "DRIVING"))
        self.status_pub.publish(String(data="%s wp=%d/%d v=%.2f" % (
            state, self.nearest_index, len(self.waypoints),
            self.current_speed)))
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
