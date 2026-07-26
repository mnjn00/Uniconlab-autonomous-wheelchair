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
  - slope guard and DEGRADED-localization slowdown, tilt aborts
  - speed policy: 1.5 m/s cap (typical powered-wheelchair pace)
    with speed-scaled obstacle guard distances, curvature
    slowdown, accel/yaw-rate limiting; tip_guard adds closed-loop
    climb assist so slopes get torque without tipping
  - dead-man guards: starts PAUSED until /waypoint_follower/start, holds on
    stale pose/cloud/base, any non-TRACKING localization state (with a bounded
    DEGRADED grace period), manual joystick mode, or
    geofence violation, and always sends stop on shutdown.
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

from body_frame import (LIDAR_IN_BODY_XYZ,
                        LIDAR_IN_BODY_YAW_RAD, body_to_lidar)
from localization_policy import localization_hold_reason
from safety_band import SafetyBand
import tf.transformations as tft

MAX_SPEED = 1.5
SLOPE_SPEED = 0.6
CREEP_SPEED = 0.15
MAX_YAW_RATE = 0.5
MAX_ACCEL = 0.18
MAX_DECEL = 0.6
CONTROL_HZ = 10.0

CORRIDOR_HALF_WIDTH = 0.45
# obstacle guard distances scale with speed: braking distance + sensing
# latency must always fit inside the stop radius (near-stationary floor
# 0.9 m, 1.8 m+ at full speed)
GUARD_STOP_MIN_M = 0.9
GUARD_STOP_PER_MPS = 1.2
GUARD_SLOW_EXTRA_M = 1.2
OBSTACLE_MIN_Z = 0.18
OBSTACLE_MAX_Z = 1.9
NARROW_SPEED = 0.2
OFF_BAND_GRACE = 0.10
SLOPE_PITCH_RAD = math.radians(3.0)
# on steep terrain, hug the proven driven line: creep, shorter
# lookahead (no corner cutting), and no lateral bypass - the line
# driven on 7/7 is the one place the camber is known passable
STEEP_PITCH_RAD = math.radians(4.0)
STEEP_SPEED = 0.3
STEEP_LOOKAHEAD_FACTOR = 0.6
BYPASS_AFTER_S = 10.0
# micro offsets first: street furniture (barrier bars, sign posts)
# usually needs only a small shift; large offsets after
BYPASS_OFFSETS = (0.35, -0.35, 0.6, -0.6, 1.0, -1.0)
MICRO_BYPASS_M = 0.4
GOAL_TOLERANCE_M = 1.0
POSE_STALE_S = 1.0
BASE_STALE_S = 1.5
# static attitude backstop only - dynamic tip detection lives in
# tip_guard (50 Hz deviation/rate/accel). Hanyang's steepest route ramp
# measures ~10 deg fused pitch and its cambered ramps ~6.3 deg roll, so
# both aborts sit above the measured terrain with margin while staying
# far below static rollover attitudes.
MAX_TILT_ROLL = math.radians(11.0)
MAX_TILT_PITCH = math.radians(12.0)
BAND_RECOVER_MAX = 0.5
GEOFENCE_M = 3.5
AUTO_MODE = 65
DEGRADED_STOP_S = 3.0
NEAREST_RESYNC_M = 2.0
# the chord from the chair to the target is sampled at this spacing and every
# sample must be in band; clamping only the endpoint checked nothing between
CHORD_SAMPLE_M = 0.25
MIN_LOOKAHEAD_M = 0.9
LOOKAHEAD_BACKOFF_M = 0.4


class CloudAccumulator:
    """Merge ~1 s of sparse MID360 scans into the current body frame."""

    def __init__(self, window_s=0.6, lidar_in_body=LIDAR_IN_BODY_XYZ,
                 lidar_in_body_yaw=LIDAR_IN_BODY_YAW_RAD):
        self.window_s = window_s
        self.lidar_in_body = lidar_in_body
        self.lidar_in_body_yaw = lidar_in_body_yaw
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
        merged = body_to_lidar(np.vstack(parts), self.lidar_in_body,
                               self.lidar_in_body_yaw)
        return merged, rospy.Time.from_sec(newest)


class WaypointFollower:
    def __init__(self):
        rospy.init_node("waypoint_follower")
        with open(rospy.get_param("~route")) as f:
            route = json.load(f)
        self.waypoints = np.array(
            [[w["x"], w["y"]] for w in route["waypoints"]], dtype=np.float64)
        self.band = SafetyBand(rospy.get_param("~safety_band"))
        self.sensor_height = rospy.get_param("~sensor_height", 0.30)
        # operator-authorized band grace for SMALL bypass offsets only:
        # lets the chair dodge mapped-as-step street furniture when a
        # human on site confirms the adjacent surface is flat. Default
        # off; never applies to large offsets.
        self.micro_bypass_grace = rospy.get_param(
            "~micro_bypass_grace", 0.0)
        # how far outside the usable band the chair may sit and still
        # creep back on its own; raise only with an operator on site
        # (e.g. after a manual reposition past street furniture)
        self.band_recover_max = rospy.get_param(
            "~band_recover_max", BAND_RECOVER_MAX)
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
        self.cloud, self.cloud_stamp = self.accumulator.merged()

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
        guard_slow = self.guard_stop() + GUARD_SLOW_EXTRA_M
        m = ((pts[:, 0] > 0.25) & (pts[:, 0] < guard_slow + 0.6) &
             (np.abs(pts[:, 1] - lateral_shift) < CORRIDOR_HALF_WIDTH))
        zone = pts[m]
        if not len(zone):
            return None
        rel = zone[:, 2] - ground_plane
        obstacles = zone[(rel > OBSTACLE_MIN_Z) & (rel < OBSTACLE_MAX_Z)]
        if len(obstacles) < 5:
            return None
        return float(np.percentile(obstacles[:, 0], 5))

    def guard_stop(self):
        return max(GUARD_STOP_MIN_M,
                   0.6 + GUARD_STOP_PER_MPS * self.current_speed)

    def bypass_target_ok(self, offset):
        """A lateral bypass is allowed only if the offset corridor stays
        inside the safety band for the next few meters."""
        if self.pose_xy is None:
            return False
        heading = np.array([math.cos(self.pose_yaw), math.sin(self.pose_yaw)])
        normal = np.array([-heading[1], heading[0]])
        grace = self.micro_bypass_grace if abs(offset) <= MICRO_BYPASS_M else 0.0
        for ahead in (0.5, 1.5, 2.5, 3.5):
            p = self.pose_xy + heading * ahead + normal * offset
            if not self.band.contains(p, grace=grace):
                return False
        if grace > 0.0:
            rospy.logwarn("micro-bypass using operator-authorized band "
                          "grace %.2f m (offset %+.2f)", grace, offset)
        return True

    def send_stop(self):
        self.current_speed = 0.0
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------ control
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
                windowed_index, d[windowed_index], global_index, d[global_index])
            self.nearest_index = global_index
        else:
            self.nearest_index = windowed_index
        lookahead = 1.0 + 1.6 * self.current_speed
        if abs(self.pose_pitch) > STEEP_PITCH_RAD:
            lookahead = max(0.8, lookahead * STEEP_LOOKAHEAD_FACTOR)
        # A straight chord can only stay inside a +-0.15 m band on curves
        # gentler than about a 10 m radius, so on tighter corners the answer
        # is a SHORTER lookahead - i.e. less speed - not a collapsed target.
        # Measured on the shipped route: 3.4 m chords are in band at 91.5% of
        # driven-line positions, 1.8 m at 98.9%, and ~1 m essentially always.
        lookahead, self.chord_speed_cap = self.safe_lookahead(lookahead)
        target = self.lookahead_point(lookahead)
        if abs(self.lateral_offset) > 0.01:
            direction = target - self.pose_xy
            n = np.linalg.norm(direction)
            if n > 1e-3:
                normal = np.array([-direction[1], direction[0]]) / n
                target = target + normal * self.lateral_offset
        # never steer to a point outside the drop-free band
        return self.band.clamp(target)

    def safe_lookahead(self, wanted):
        """Longest lookahead whose chord stays in band, and the speed it implies.

        Returns (lookahead, speed_cap). The chord is what the chair actually
        traverses; band.clamp only ever constrained the ENDPOINT, and since
        that endpoint was a route waypoint its lateral offset was ~0, so the
        clamp was a no-op and nothing checked the path in between.

        The shipped route makes this necessary rather than theoretical: 4 of
        its 85 waypoints lie OUTSIDE the band and 8 of 84 chords leave it,
        because the 353-station driven line was sparsified to 85 points
        without regard for the band - cutting corners by up to 0.63 m where
        the usable band is often 0.15 m. Filed separately; until the route is
        regenerated this is what keeps a wheel on the pavement.
        """
        candidate = wanted
        while candidate >= MIN_LOOKAHEAD_M:
            target = self.band.clamp(self.lookahead_point(candidate))
            if self.chord_in_band(target):
                implied = max(0.0, (candidate - 1.0) / 1.6)
                return candidate, (MAX_SPEED if candidate >= wanted - 1e-9
                                   else max(CREEP_SPEED, implied))
            candidate -= LOOKAHEAD_BACKOFF_M
        # even the shortest chord is not provably safe: creep, and say so
        rospy.logwarn_throttle(
            5.0, "waypoint_follower: no band-safe chord at this pose, creeping")
        return MIN_LOOKAHEAD_M, CREEP_SPEED

    def lookahead_point(self, lookahead):
        """Point `lookahead` metres along the route polyline from the chair.

        Interpolated WITHIN a segment rather than snapped to a waypoint. The
        deployed route has 85 waypoints over 353 m - median spacing 4.60 m,
        and 63 of 84 segments longer than the 3.4 m maximum lookahead - so
        snapping made the target "the next waypoint" almost every cycle. The
        effective lookahead was then 4.6-6.9 m, 2-7x what was asked for, and
        STEEP_LOOKAHEAD_FACTOR (which exists to stop corner-cutting on steep
        ground) could never take effect at all.
        """
        start = self.waypoints[self.nearest_index]
        remaining = lookahead
        for i in range(self.nearest_index, len(self.waypoints) - 1):
            a = self.waypoints[i] if i > self.nearest_index else start
            b = self.waypoints[i + 1]
            seg = np.linalg.norm(b - a)
            if seg < 1e-6:
                continue
            if remaining <= seg:
                return a + (b - a) * (remaining / seg)
            remaining -= seg
        return self.waypoints[-1].copy()

    def chord_in_band(self, target):
        """Is every sample along pose->target inside the band?

        Uses the same OFF_BAND_GRACE the containment hold already tolerates,
        so this cannot refuse a line the hold logic would accept.
        """
        span = float(np.linalg.norm(target - self.pose_xy))
        if span < 1e-6:
            return True
        steps = max(2, int(math.ceil(span / CHORD_SAMPLE_M)))
        for k in range(1, steps + 1):
            point = self.pose_xy + (target - self.pose_xy) * (float(k) / steps)
            if not self.band.contains(point, grace=OFF_BAND_GRACE):
                return False
        return True

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
                self.pose_xy, grace=self.band_recover_max):
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
        # a chord longer than the band allows is answered with less speed,
        # which shortens the lookahead, which shortens the chord
        allowed = min(allowed, self.chord_speed_cap)
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
