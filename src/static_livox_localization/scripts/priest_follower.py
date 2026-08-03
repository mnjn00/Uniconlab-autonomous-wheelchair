#!/usr/bin/env python3
"""PRIEST-planned follower: a start, a goal, and the band - no fixed line.

Drop-in alternative to waypoint_follower.py behind the same external
contract: it names itself waypoint_follower, publishes /cmd_vel_raw and
/waypoint_follower/status, and offers /waypoint_follower/start - so go.sh,
stop.sh, the black box and the gate chain neither know nor care which
implementation is running. Exactly one of the two is launched; the bringup's
PLANNER variable chooses, and the field-validated route follower remains the
default.

What changes is where the line comes from. The route follower tracks a
recorded polyline and can only stop when that line is blocked. This node
replans a trajectory every couple of seconds with priest_planner - inside
the band via priest_corridor, around objects the cluster tracker has watched
stand still - and pure-pursuits its own plan. Which side of a parked bicycle
the chair passes is decided per cycle, not at recording time.

The obstacle split matches the deployed follower's policy: STATIC and
UNKNOWN objects become planner obstacles and are routed around; anything
not confirmed parked that stands close in the corridor is WAITED for, since
steering around a person is a manoeuvre into where they are about to be.
Waiting resumes on its own when the corridor clears - no timer.

There is deliberately NO ~safety_policies switch here. That switch exists on
the route follower so a diagnostic run can measure one thing on a path a
person demonstrably drove; an unvalidated planner with its guards off is not
a diagnostic configuration, it is an unsupervised experiment, so
_safety_policies:=false is refused at startup rather than honoured.
"""

import json
import math
import os
import sys

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse

# catkin_install_python leaves a relay in devel/lib that exec()s this file,
# so sys.path[0] is the relay's directory, not this one - recover it from
# __file__ or the sibling policy modules are not importable on the vehicle.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import (lidar_extrinsics, lidar_to_body, pose_correction,
                        reference_correction, route_chair_centre)
from cluster_guard import (is_stale, nearest_threat, object_box,
                           object_motion, parse_summary)
from cluster_tracking import MOVING
from localization_policy import localization_hold_reason
from motion_safety import (MotionEstimate, PoseMotionEstimator,
                           motion_hold_reason)
from priest_corridor import corridor_arrays
from priest_planner import Corridor, PriestPlanner
from safety_band import SafetyBand
import tf.transformations as tft

MAX_SPEED = 0.6
CREEP_SPEED = 0.15
MAX_YAW_RATE = 0.5
TURN_FLOOR_SPEED = 0.30
MAX_ACCEL = 0.18
MAX_DECEL = 0.6
CONTROL_HZ = 10.0
CORRIDOR_HALF_WIDTH = 0.45
POSE_STALE_S = 1.0
BASE_STALE_S = 1.5
ODOM_STALE_S = 0.35
AUTO_MODE = 65
DEGRADED_STOP_S = 3.0
GOAL_TOLERANCE_M = 1.0
OFF_BAND_GRACE = 0.10
MIN_LOOKAHEAD_M = 0.9
# How often the world is re-asked. Between replans the chair tracks a plan
# that was feasible when made; the cross-track trigger below catches the
# case where reality and the plan have drifted apart sooner.
REPLAN_S = 2.0
CROSS_TRACK_REPLAN_M = 0.5
# Anything not confirmed parked inside this range in the corridor is waited
# for rather than planned around. Sized like the route follower's stop
# geometry: at 0.6 m/s the envelope floor is 0.9 m and the accumulated scan
# is up to 0.8 s old, so 2.5 m leaves the wait decision made well before the
# stop would have to be.
WAIT_RADIUS_M = 2.5


def require_guards(safety_policies_enabled):
    """Refuse the diagnostic switch. See the module docstring for why."""
    if not safety_policies_enabled:
        raise ValueError(
            "priest_follower has no diagnostic mode: _safety_policies:=false "
            "would run an unvalidated planner with its guards off. Use the "
            "route follower (PLANNER=route) for diagnostic runs.")


def planner_obstacles(objects, map_T_body, lidar_in_body,
                      lidar_to_body_rotation, limit=24):
    """Map-frame circles [x, y, r] for everything worth routing around.

    STATIC and UNKNOWN objects are included - planning around something not
    yet confirmed parked is conservative, and the wait policy still holds
    the chair if it stands close. MOVING objects are excluded: a trajectory
    around where a walker is NOW is a trajectory into where they are next.

    The radius is the circumscribed circle of the object's footprint, so a
    box is never entered on the diagonal the way its inscribed circle would
    allow. Malformed objects are skipped here - nearest_threat() already
    reports them as blocking at zero range, which holds the chair without
    this list inventing a position for them.

    Sorted nearest-first before the cap, because the planner's obstacle
    slots are finite and the far ones are the ones a later replan can still
    catch.
    """
    circles = []
    for item in objects:
        box = object_box(item)
        if box is None:
            continue
        if object_motion(item) == MOVING:
            continue
        x, y, half_x, half_y = box
        body = lidar_to_body(np.array([[x, y, 0.0]]), lidar_in_body,
                             lidar_to_body_rotation)[0]
        world = map_T_body[:3, :3] @ body + map_T_body[:3, 3]
        circles.append([float(world[0]), float(world[1]),
                        float(math.hypot(half_x, half_y))])
    circles.sort(key=lambda c: math.hypot(c[0] - map_T_body[0, 3],
                                          c[1] - map_T_body[1, 3]))
    return circles[:int(limit)], max(0, len(circles) - int(limit))


def wait_reason(summary, half_width_m=CORRIDOR_HALF_WIDTH,
                wait_radius_m=WAIT_RADIUS_M):
    """OBSTACLE_WAIT when something not confirmed parked stands close ahead.

    An unusable summary comes back from nearest_threat() as a blocking
    MOVING threat at zero range, so a producer that cannot see holds the
    chair exactly like a person standing on the bumper would - which is the
    correct reading of "cannot see".
    """
    threat = nearest_threat(summary, half_width_m)
    if threat is not None and not threat.parked and \
            threat.distance_m < wait_radius_m:
        return "OBSTACLE_WAIT"
    return None


class PriestFollower(object):
    def __init__(self):
        rospy.init_node("waypoint_follower")
        require_guards(bool(rospy.get_param("~safety_policies", True)))

        with open(rospy.get_param("~route")) as handle:
            route = json.load(handle)
        band_path = rospy.get_param("~safety_band")
        profile = str(rospy.get_param("~body_frame_profile"))
        self.lidar_in_body, self.lidar_rotation = lidar_extrinsics(profile)
        # Same reference discipline as the route follower: the pose is
        # corrected about the point the ROUTE was recorded about, so the
        # band's stations and the chair's position mean the same point.
        self.pose_correction = pose_correction(
            profile, str(route["body_frame_profile"])) @ reference_correction(
                str(route["reference_point"]), route_chair_centre(route))

        centres, normals, left, right = corridor_arrays(band_path)
        self.corridor = Corridor(centres, normals, left, right)
        self.band = SafetyBand(band_path)
        self.planner = PriestPlanner(v_max=MAX_SPEED, seed=0)
        rospy.loginfo(
            "priest_follower: corridor %.1f m, %d stations, guards ON "
            "(no diagnostic mode)", self.corridor.length_m, len(centres))

        self.enabled = False
        self.done = False
        self.pose_map = None          # raw map_T_body, for obstacle transform
        self.centre_xy = None         # corrected chair centre, for planning
        self.pose_yaw = 0.0
        self.pose_stamp = rospy.Time(0)
        self.prev_centre = None
        self.prev_stamp = None
        self.velocity = np.zeros(2)
        self.tracking_state = ""
        self.degraded_since = None
        self.drive_mode = None
        self.wheel_status_stamp = rospy.Time(0)
        self.cluster_summary = None
        self.motion = MotionEstimate(False, 0.0, 0.0, 0.0, 0.0,
                                     "ODOM_INITIALIZING")
        self.motion_estimator = PoseMotionEstimator(
            str(rospy.get_param("~odom_frame", "camera_init")),
            str(rospy.get_param("~base_frame", "body")))
        self.plan = None
        self.plan_stamp = rospy.Time(0)
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

    # ------------------------------------------------------------ callbacks
    def on_pose(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        pose = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        pose[:3, 3] = (p.x, p.y, p.z)
        self.pose_map = pose
        corrected = pose @ self.pose_correction
        centre = np.array([corrected[0, 3], corrected[1, 3]])
        stamp = message.header.stamp
        if self.prev_centre is not None and self.prev_stamp is not None:
            dt = (stamp - self.prev_stamp).to_sec()
            if 1e-3 < dt < 1.0:
                velocity = (centre - self.prev_centre) / dt
                speed = float(np.linalg.norm(velocity))
                if speed > MAX_SPEED:
                    velocity = velocity * (MAX_SPEED / speed)
                self.velocity = velocity
        self.prev_centre, self.prev_stamp = centre, stamp
        self.centre_xy = centre
        self.pose_yaw = math.atan2(corrected[1, 0], corrected[0, 0])
        self.pose_stamp = stamp

    def on_odom(self, message):
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        self.motion = self.motion_estimator.update(
            source_stamp_s=message.header.stamp.to_sec(),
            receipt_stamp_s=rospy.Time.now().to_sec(),
            frame_id=message.header.frame_id,
            child_frame_id=message.child_frame_id,
            x=p.x, y=p.y,
            quaternion_xyzw=(q.x, q.y, q.z, q.w))

    def on_diag(self, message):
        for status in message.status:
            if status.name == "fast_lio_icp":
                self.tracking_state = status.message

    def on_wheel_status(self, message):
        self.wheel_status_stamp = rospy.Time.now()
        if len(message.data) > 1:
            self.drive_mode = message.data[1]

    def on_clusters(self, message):
        try:
            self.cluster_summary = parse_summary(message.data)
        except ValueError as error:
            self.cluster_summary = None
            rospy.logwarn_throttle(
                5.0, "objects_summary unreadable: %s", error)

    def on_start(self, request):
        self.enabled = request.data
        if not request.data:
            self.send_stop()
        rospy.loginfo("priest follower %s",
                      "ENABLED" if self.enabled else "PAUSED")
        return SetBoolResponse(success=True,
                               message="ENABLED" if self.enabled else "PAUSED")

    def send_stop(self):
        self.current_speed = 0.0
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------- planning
    def ensure_plan(self, now):
        """Replan when the plan is old, drifted from, or missing.

        Returns the hold reason when no usable plan exists. The planner's
        own refusal is surfaced verbatim: NO_FEASIBLE_TRAJECTORY after its
        reach backoff is a statement about the corridor, and the operator
        should see it under that name.
        """
        age = (now - self.plan_stamp).to_sec()
        drifted = False
        if self.plan is not None and self.plan.usable:
            points = self.plan.points()
            drifted = float(np.min(np.linalg.norm(
                points - self.centre_xy, axis=1))) > CROSS_TRACK_REPLAN_M
        if self.plan is None or age > REPLAN_S or drifted:
            circles, dropped = planner_obstacles(
                [] if self.cluster_summary is None
                else self.cluster_summary.objects,
                self.pose_map, self.lidar_in_body, self.lidar_rotation,
                limit=self.planner.max_obstacles)
            if dropped:
                rospy.logwarn_throttle(
                    10.0, "obstacle coverage capped: %d beyond the %d "
                    "nearest were dropped this cycle", dropped,
                    self.planner.max_obstacles)
            started = rospy.Time.now()
            self.plan = self.planner.plan(
                self.centre_xy, self.velocity, np.zeros(2), self.corridor,
                circles)
            self.plan_stamp = now
            took = (rospy.Time.now() - started).to_sec()
            if took > 0.5:
                rospy.logwarn(
                    "replan took %.2f s - inside the gate's 0.6 s input "
                    "staleness but with little left over", took)
        if self.plan.reason == "AT_GOAL":
            return "DONE"
        if not self.plan.usable:
            return self.plan.reason
        return None

    # ------------------------------------------------------------- tracking
    def track(self):
        """Pure pursuit on the plan this node made for itself."""
        points = self.plan.points()
        distances = np.linalg.norm(points - self.centre_xy, axis=1)
        nearest = int(np.argmin(distances))
        lookahead = MIN_LOOKAHEAD_M + 1.6 * self.current_speed
        target = points[-1]
        travelled = 0.0
        for index in range(nearest, len(points) - 1):
            travelled += float(np.linalg.norm(points[index + 1]
                                              - points[index]))
            if travelled >= lookahead:
                target = points[index + 1]
                break

        to_target = target - self.centre_xy
        heading = math.atan2(to_target[1], to_target[0])
        error = math.atan2(math.sin(heading - self.pose_yaw),
                           math.cos(heading - self.pose_yaw))
        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, 1.2 * error))
        slew = 1.5 / CONTROL_HZ
        yaw_rate = max(self.last_yaw_rate - slew,
                       min(self.last_yaw_rate + slew, yaw_rate))
        self.last_yaw_rate = yaw_rate

        turning = min(abs(yaw_rate) / MAX_YAW_RATE, 1.0)
        allowed = min(MAX_SPEED,
                      max(TURN_FLOOR_SPEED * turning, CREEP_SPEED,
                          MAX_SPEED * (1.0 - abs(error) / 1.2)))
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

    # ----------------------------------------------------------------- step
    def hold_reason(self, now):
        """Every guard, always on, in dependency order - NO_POSE first
        because everything after it reads the position it guarantees."""
        if not self.enabled or self.done:
            return "DONE" if self.done else "PAUSED"
        if self.centre_xy is None or \
                (now - self.pose_stamp).to_sec() > POSE_STALE_S:
            return "NO_POSE"
        reason = motion_hold_reason(self.motion, now.to_sec(), ODOM_STALE_S)
        if reason:
            return reason
        degraded_age = None if self.degraded_since is None else \
            (now - self.degraded_since).to_sec()
        reason = localization_hold_reason(
            self.tracking_state, degraded_age, DEGRADED_STOP_S)
        if reason:
            return reason
        if (now - self.wheel_status_stamp).to_sec() > BASE_STALE_S:
            return "BASE_STALE"
        if self.drive_mode is not None and self.drive_mode != AUTO_MODE:
            return "MANUAL_MODE"
        if is_stale(None if self.cluster_summary is None
                    else self.cluster_summary.stamp_s, now.to_sec()):
            return "CLUSTERS_STALE"
        if not self.band.contains(self.centre_xy, grace=OFF_BAND_GRACE):
            return "OFF_BAND"
        return None

    def step(self):
        now = rospy.Time.now()
        if self.tracking_state == "DEGRADED":
            if self.degraded_since is None:
                self.degraded_since = now
        else:
            self.degraded_since = None

        reason = self.hold_reason(now)
        if reason is None and np.linalg.norm(
                self.corridor.centres[-1] - self.centre_xy) \
                < GOAL_TOLERANCE_M:
            self.done = True
            reason = "DONE"
            rospy.loginfo("GOAL REACHED")
        if reason is None:
            reason = wait_reason(self.cluster_summary)
        if reason is None:
            reason = self.ensure_plan(now)
        if reason:
            if reason != self.status:
                rospy.loginfo("hold: %s", reason)
                self.status = reason
            self.status_pub.publish(String(data="HOLD:" + reason))
            self.send_stop()
            return

        self.track()
        arc = self.corridor.arc_of(self.centre_xy)
        state = "DRIVING arc=%.1f/%.1f v=%.2f PRIEST" % (
            arc, self.corridor.length_m, self.current_speed)
        self.status_pub.publish(String(data=state))
        if self.status != "DRIVING":
            rospy.loginfo("state: DRIVING (PRIEST, arc %.1f m)", arc)
            self.status = "DRIVING"

    def run(self):
        rate = rospy.Rate(CONTROL_HZ)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    PriestFollower().run()
