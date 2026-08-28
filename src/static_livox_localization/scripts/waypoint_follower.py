#!/usr/bin/env python3
"""Safety-guarded waypoint follower for the wheelchair.

Drop safety comes from the map, not the live scan: the MID360 (vertical FOV
-7..+52 deg, 0.725 m mount) cannot see ground within ~5.9 m, so curbs are
avoided by keeping the wheelchair inside the pre-computed drop-free lateral
band along the route (tools/make_route_safety_band.py). The live accumulated
scan is used for what the sensor CAN see: obstacles and pedestrians.

That blind radius was documented as 2.4 m until the mount height was
measured on 2026-07-31. It followed from a 0.30 m mount that was never a
measurement, and 0.725/tan(7 deg) is 5.9 m - so the band is carrying more
of the drop safety than the number here used to admit, not less.

Per control cycle:
  - band containment: the current position must lie inside the safety band;
    steering targets and bypass offsets are clamped into the band
  - obstacle guard: slow near obstacles/pedestrians, stop when close, with
    stop and slow radii scaled to the speed being carried
  - parked vs moving: an object the cluster tracker has watched stand still
    is stepped around within the band, from 5 m out so the chair drifts past
    rather than stopping first; anything moving, or not yet watched long
    enough to say, is waited out where it stands and driving resumes on its
    own once the corridor is clear. Classified objects are the ONLY obstacle
    source here: the raw five-point scan check was removed on 2026-08-05
    because a source with no identity was authorising bypass manoeuvres.
    safety_gate.py keeps its own raw check and can still stop the chair
  - slope guard and bounded DEGRADED-localization grace
  - speed policy: 1.0 m/s cap (operator-directed), curvature slowdown,
    accel/yaw-rate limiting
  - dead-man guards: starts PAUSED until /waypoint_follower/start, holds on
    stale pose/cloud/base, LOST or sustained DEGRADED localization, manual
    joystick mode, or geofence violation, and always stops on shutdown.

_safety_policies:=false switches off everything in that list that is a
judgement about the world, leaving the joystick override and the checks it
rests on. It exists so a run can measure one thing - whether localization
stays attached over the whole route - without a band refusal or an obstacle
stop ending the measurement first and looking the same from outside. The
suppressed policies are still evaluated and published as WOULD_HOLD, since
that run is also the only place their thresholds can be calibrated. See
drive_policy.py.
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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse

import sensor_msgs.point_cloud2 as pc2
# catkin_install_python leaves a relay in devel/lib that exec()s this file,
# so sys.path[0] is the relay's directory, not this one, and the policy
# modules sitting beside this file are not importable - the relay does set
# __file__ to this source path, so recover the directory from it. Without
# this the node dies at import on the vehicle while every offline test,
# which imports the modules directly, still passes.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from body_frame import (CHAIR_CENTRE_IN_BODY_XYZ, REFERENCE_BODY,
                        lidar_extrinsics, pose_correction,
                        reference_correction, route_chair_centre)
from cluster_guard import (ACCUMULATION_S as CLUSTER_ACCUMULATION_S,
                           BYPASS_EDGE_KEEP_M, BYPASS_OFFSET_MAX_M,
                           BYPASS_OFFSET_MIN_M, BYPASS_OFFSETS,
                           BYPASS_PROBE_AHEAD_M, GO_ROUND, Threat,
                           PERSON_BYPASS, PERSON_LABEL,
                           avoidance_decision, bypass_offsets_for_room,
                           is_stale, matching_threats, nearest_threat,
                           parse_summary)
from cluster_tracking import MOVING
from drive_policy import OVERRIDE, POLICY, announce, evaluate_holds
from localization_policy import SUPPRESSED_WHILE_PARKED
from localization_policy import localization_hold_reason
from scan_accumulator import CloudAccumulator
from motion_safety import (MotionEstimate, PoseMotionEstimator,
                           clamp_pose_step,
                           motion_hold_reason, stopping_envelope)
import mpc_speed
from safety_band import SafetyBand
from route_mask import RouteMask
from route_assets import validate_asset_binding
import tf.transformations as tft

# Matched to how the route is actually driven. Measured over the
# 2026-07-27 manual run of this route (full_debug_20260727_214306.bag,
# /fast_lio_icp/pose, spin-in-place bookends trimmed): median 0.96 m/s,
# p75 1.00, p90 1.21, p95 1.58. The operator raised the current cap to
# 1.0 m/s on 2026-08-12; the measured p90 remains the upper reference.
MAX_SPEED = 0.8
SLOPE_SPEED = 0.3
CREEP_SPEED = 0.15
MAX_YAW_RATE = 0.5
# Speed floor while steering hard, scaled by how hard. At full yaw the
# faster wheel then runs near the 1.3 km/h the loaded chair was measured
# to need before it rotates at all; below that a turn is commanded and
# simply does not occur. The wheel guard defends the same floor
# independently, so a slow command still turns even if this is bypassed.
# Under roughly 0.30 m/s the loaded wheels do not turn at all. The
# operator asked for 0.35 on 2026-08-23: 0.30 is the edge of the
# deadband and a command sitting on an edge is not a stable command.
TURN_FLOOR_SPEED = 0.35
# Centre to centre, measured on the chair. The base converts (v, w) to wheel
# speeds with the same figure; this copy is for reading them back.
WHEEL_SEPARATION_M = 0.54
MAX_ACCEL = 0.18
MAX_DECEL = 0.6
CONTROL_HZ = 10.0

CORRIDOR_HALF_WIDTH = 0.45
# A person gets a wider stop corridor than the geometry handed to the
# avoidance planner.  This keeps the DWA obstacle corridor unchanged while
# catching a walking person's body at the edge of the route.
PERSON_STOP_HALF_WIDTH_M = 0.55
# DWA may bend beyond the stop corridor while passing. A second person in
# that maneuver region vetoes authorization even when they do not overlap
# the route centreline.
PERSON_BYPASS_HALF_WIDTH_M = 1.0
PERSON_STOP_DISTANCE_SCALE = 1.20
PERSON_MEMORY_S = 1.0
# Once a person has required a stop, do not let the stopped chair's smaller
# dynamic envelope immediately authorize motion again. The field bag showed
# one unchanged person at 1.2 m producing 58 WAIT/DWA pairs. Thirty
# centimetres is wider than that envelope swing while still releasing a
# person who actually clears the corridor or moves away.
PERSON_STOP_RELEASE_MARGIN_M = 0.30
# A tracked person is not a parked object after the producer's 1.5-second
# STATIC confirmation. Bypass is a separate, longer authorization based on
# direct same-track producer evidence; any missed 5 Hz frame resets it.
PERSON_BYPASS_CONFIRM_S = 10.0
PERSON_BYPASS_MAX_GAP_S = 0.35
# The forward-cone and minimum-range constants that used to live here
# belonged to the raw five-point scan check, removed 2026-08-05. The same
# geometry still exists in safety_gate.py, which keeps its own independent
# raw check - see corridor_threat for why the follower no longer has one.
GUARD_STOP_MIN_M = 0.9
GUARD_SLOW_EXTRA_M = 1.2
ACCUMULATION_WINDOW_S = 1.0
PIPELINE_BUDGET_S = 0.2
MIN_BRAKE_DECEL_MPS2 = 0.5
MIN_YAW_DECEL_RPS2 = 0.5
ODOM_STALE_S = 0.35
# Speed follows how much lateral slack the chair actually has, not whether
# the band happens to be under a width threshold. The old binary test
# (total width < 1.2 m -> 0.2 m/s) spent the same caution on a corridor
# pinched between two kerbs and on one with a kerb on a single side and
# open pavement opposite. Slack is the smaller of the room to the band
# edges and the distance from a wheel to a mapped fall hazard.
SLACK_FULL_SPEED_M = 0.8
SLACK_CREEP_M = 0.15
OFF_BAND_GRACE = 0.10
SLOPE_PITCH_RAD = math.radians(3.0)
# What separates "an obstacle" from "a person". Anything still in the way
# after this long is treated as parked and gets stepped around; anything
# that clears sooner - a pedestrian crossing the path - is simply waited
# out, and driving resumes the moment the corridor is clear again.
BYPASS_AFTER_S = 3.0

# How far ahead a confirmed-parked object is stepped around. The number that
# matters is the WARNING TIME it buys, which is what turns "stop, wait 3 s,
# then edge sideways" into one continuous drift past the thing: 5.0 m was
# eight seconds at the 0.6 m/s cap it was chosen under. At the 1.0 m/s cap
# the same 5.0 m is five seconds, and the full stopping envelope at that
# speed is 3.20 m of the 5.0 - so 8.0 m restores the warning time rather
# than quietly spending it. Still bounded for the reason it always was:
# past this the object is often not even on the stretch the chair will
# drive, and stepping around something that is not in the way is its own
# hazard.
PLAN_AHEAD_M = 8.0
GOAL_TOLERANCE_M = 1.0
POSE_STALE_S = 1.0
BASE_STALE_S = 1.5
BAND_RECOVER_MAX = OFF_BAND_GRACE
GEOFENCE_M = 3.5
AUTO_MODE = 65
DEGRADED_STOP_S = 3.0
NEAREST_RESYNC_M = 2.0
# How far ahead the nearest-point search may advance in one cycle. Generous
# against the 0.06 m the chair covers per 10 Hz cycle at full speed, and the
# global-nearest resync below still catches a real divergence.
PROGRESS_WINDOW_M = 20.0
ROUTE_STEP_M = 0.2
MIN_LOOKAHEAD_M = 0.9
LOOKAHEAD_BACKOFF_M = 0.4


class WaypointFollower:
    person_memory = None
    person_stop_release_m = None
    direct_person_threats = ()
    person_static_track_id = None
    person_static_since_s = None
    person_static_last_stamp_s = None
    person_bypass_committed_track_id = None

    # Which control law this class turns a pose into a Twist with. Both
    # profiles run as the same node under the same name, so the node alone
    # does not say which one started; subclasses override this and __init__
    # publishes it. See the ~control_law param below for why it exists.
    CONTROL_LAW = "pursuit"

    def __init__(self):
        rospy.init_node("waypoint_follower")
        route_path = rospy.get_param("~route")
        band_path = rospy.get_param("~safety_band")
        with open(route_path) as f:
            route = json.load(f)
        mask_path = rospy.get_param("~drivable_mask")
        try:
            self.asset_binding = validate_asset_binding(
                route_path, band_path, mask_path,
            )
        except ValueError as error:
            raise rospy.ROSInitException(str(error))
        self.waypoints = np.array(
            [[w["x"], w["y"]] for w in route["waypoints"]], dtype=np.float64)
        self.band = SafetyBand(band_path)
        self.drivable_mask = RouteMask(mask_path)
        self.sensor_height = rospy.get_param("~sensor_height", 0.725)
        rospy.loginfo("route: %d waypoints, band stations: %d",
                      len(self.waypoints), len(self.band.xy))

        self.enabled = False
        self.done = False
        self.pose_xy = None
        self.pose_yaw = 0.0
        self.pose_pitch = 0.0
        self.pose_stamp = rospy.Time(0)
        self.tracking_state = ""
        self.tracking_reason = ""
        # Where the chair stood when a parked-and-blind hold began, so the
        # creep that earns the next registration stays bounded.
        self.reacquire_origin = None
        self.degraded_since = None
        self.drive_mode = None
        self.wheel_status_stamp = rospy.Time(0)
        # What the wheels are actually doing. /Odometry carries no twist -
        # FAST-LIO leaves it at zero - so this frame is the only velocity
        # measurement on the bus, and until 2026-08-23 nothing decoded it.
        self.measured_speed = 0.0
        self.measured_yaw_rate = 0.0
        self.route_locked = False
        profile = str(rospy.get_param("~body_frame_profile"))
        lidar_in_body, lidar_to_body_rotation = lidar_extrinsics(profile)
        # A route records the path of FAST-LIO's IMU body origin, so it is
        # only comparable to a pose read in the SAME body frame. Refuse to
        # guess: an unlabelled route is of unknown provenance, and reading
        # it in the wrong frame silently spends part of the kerb clearance
        # budget rather than failing.
        if "body_frame_profile" not in route:
            raise rospy.ROSInitException(
                "route %s does not say which body frame it was captured in; "
                "add body_frame_profile (see body_frame.py)"
                % rospy.get_param("~route"))
        # ... and about the same POINT. The sensor sits at the front of the
        # left armrest, 0.517 m forward and 0.173 m left of the centre the
        # chair turns about, so a route of the sensor's path and a route of
        # the chair's are displaced 0.173 m sideways from each other. Every
        # geometry constant here lays the chair out symmetrically about the
        # pose, which is only true of the chair centre, so an undeclared
        # reference is refused for the same reason an undeclared body frame
        # is.
        if "reference_point" not in route:
            raise rospy.ROSInitException(
                "route %s does not say which point it is about; add "
                "reference_point: \"%s\" for a sensor-path route, or "
                "re-express it with tools/recentre_route_to_chair.py"
                % (rospy.get_param("~route"), REFERENCE_BODY))
        # The route's own offset, not the current one. Reproducing a recorded
        # drive means measuring the chair about the point it was recorded
        # about; taking the newer number here would slide the whole path
        # sideways by the difference and call it tracking.
        route_centre = route_chair_centre(route)
        self.pose_correction = pose_correction(
            profile, str(route["body_frame_profile"])) @ reference_correction(
                str(route["reference_point"]), route_centre)
        drift = float(np.linalg.norm(
            np.asarray(route_centre) - np.asarray(CHAIR_CENTRE_IN_BODY_XYZ)))
        if drift > 1e-6:
            rospy.logwarn(
                "route was built about (%.3f, %.3f), the chair is now "
                "measured at (%.3f, %.3f) - driving the route about its own "
                "point so the path is reproduced, but the band's clearances "
                "are stated %.3f m from where the chair centre actually is",
                route_centre[0], route_centre[1],
                CHAIR_CENTRE_IN_BODY_XYZ[0], CHAIR_CENTRE_IN_BODY_XYZ[1],
                drift)
        if str(route["reference_point"]) == REFERENCE_BODY:
            rospy.logwarn(
                "route is about the sensor, not the chair centre: the chair "
                "extends %.2f m further right than every clearance here "
                "assumes", abs(CHAIR_CENTRE_IN_BODY_XYZ[1]))
        if str(route["body_frame_profile"]) != profile:
            rospy.logwarn(
                "route captured on the %s body frame but running %s: "
                "correcting pose by %.3f m / %.2f deg",
                route["body_frame_profile"], profile,
                float(np.linalg.norm(self.pose_correction[:3, 3])),
                math.degrees(math.atan2(self.pose_correction[1, 0],
                                        self.pose_correction[0, 0])))
        self.accumulator = CloudAccumulator(
            lidar_in_body, lidar_to_body_rotation)
        odom_frame = str(rospy.get_param("~odom_frame", "camera_init"))
        base_frame = str(rospy.get_param("~base_frame", "body"))
        self.motion_estimator = PoseMotionEstimator(odom_frame, base_frame)
        self.motion = MotionEstimate(
            False, 0.0, 0.0, 0.0, 0.0, "ODOM_INITIALIZING")
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
        self.cluster_summary = None
        self.person_memory = None
        self.person_stop_release_m = None
        self.direct_person_threats = ()
        self.person_static_track_id = None
        self.person_static_since_s = None
        self.person_static_last_stamp_s = None
        self.person_bypass_committed_track_id = None
        # Deliberately NOT behind ~safety_policies. The raw corridor check is
        # switched off with the rest of the judgements because it stops on
        # five returns and is the loudest false-positive source in the chain;
        # this one reads classified clusters and is what remains looking for
        # people when everything else is off.
        self.clusters_enabled = bool(
            rospy.get_param("~cluster_avoidance", True))
        # Since 2026-08-05 this is the only thing looking for obstacles. It
        # used to be the half that survived ~safety_policies:=false while the
        # raw check covered the other case; with the raw check gone, starting
        # with it off is starting blind, and no combination of the remaining
        # guards would notice. Refuse rather than drive.
        if not self.clusters_enabled:
            raise rospy.ROSInitException(
                "~cluster_avoidance:=false leaves no obstacle source at all "
                "since the raw scan check was removed; run with it on")
        # Absent means guarded, and anything that arrives as a string rather
        # than a bool is truthy and therefore also guarded. Both failure
        # directions leave the guards on; the startup line below is how the
        # operator finds out which way it went.
        self.policies = bool(rospy.get_param("~safety_policies", True))
        # The identity of the control law that is about to drive, published
        # by the class that implements it rather than by whoever launched it.
        # PROFILE picks a node in the bringup; nothing downstream could see
        # that choice, because both profiles are this node name, these
        # topics, and this service. A preflight that wants to confirm the
        # chair is running the law the operator asked for had nothing to read.
        #
        # It is set here, from the class attribute, for a specific reason:
        # `PLANNER=priest` was a shell variable a preflight compared against
        # itself, so it went on reporting "priest" after the PRIEST planner
        # was reverted in 81fed5d and the bringup stopped forwarding it. An
        # identity the launcher asserts proves nothing about the process.
        rospy.set_param("~control_law", self.CONTROL_LAW)
        rospy.loginfo("control law: %s", self.CONTROL_LAW)
        rospy.loginfo("cluster avoidance: %s",
                      "ON" if self.clusters_enabled else "OFF")
        if self.policies:
            rospy.loginfo(announce(True, "waypoint_follower", []))
        else:
            rospy.logwarn(announce(
                False, "waypoint_follower",
                ["band containment", "hazard-clearance speed",
                 "the raw corridor scan", "localization health",
                 "the motion-estimate gate", "the geofence",
                 "the slope limit"],
                still_watching=(
                    ["tracked-cluster avoidance",
                     "the band, for whether there is room to step aside"]
                    if self.clusters_enabled else [])))

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
        pose = pose @ self.pose_correction
        _, pitch, yaw = tft.euler_from_matrix(pose)
        elapsed = None if self.pose_stamp is None else \
            (message.header.stamp - self.pose_stamp).to_sec()
        self.pose_xy, withheld = clamp_pose_step(
            self.pose_xy, (pose[0, 3], pose[1, 3]), elapsed)
        if withheld > 0.0:
            rospy.logwarn_throttle(
                2.0, "pose step clamped: %.2f m withheld over %.2f s - the "
                "chair cannot move that fast, so this is the fix correcting",
                withheld, elapsed or 0.0)
        self.pose_yaw = yaw
        self.pose_pitch = pitch
        self.pose_stamp = message.header.stamp

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

    def on_diag(self, message):
        for status in message.status:
            if status.name == "fast_lio_icp":
                self.tracking_state = status.message
                self.tracking_reason = ""
                for value in status.values:
                    if value.key == "reason":
                        self.tracking_reason = value.value
                if self.tracking_state == "TRACKING":
                    self.reacquire_origin = None

    def reacquire_distance(self):
        """How far the chair has crept since a parked-and-blind hold began.

        The origin is taken the first time it is asked for, which is the
        cycle the hold would otherwise have become permanent - not when
        DEGRADED started, since the chair is still driving through the grace
        period and would spend the whole budget before it ever stopped.
        """
        if self.pose_xy is None:
            return None
        here = (float(self.pose_xy[0]), float(self.pose_xy[1]))
        if self.reacquire_origin is None:
            self.reacquire_origin = here
        return math.hypot(here[0] - self.reacquire_origin[0],
                          here[1] - self.reacquire_origin[1])

    def on_clusters(self, message):
        try:
            self.cluster_summary = parse_summary(message.data)
        except ValueError as error:
            # Unreadable is not empty. Dropping the last good summary lets
            # the staleness hold below stop the chair, where keeping it
            # would drive on increasingly old object positions.
            self.cluster_summary = None
            rospy.logwarn_throttle(
                5.0, "objects_summary unreadable: %s", error)

    @staticmethod
    def reported_wheel_speeds(data):
        """(left, right) in m/s from a base status frame, signed.

        Frame and scaling are the base's, from base_model/src/uart.py and
        wheel_cmd_tmp.py: direction in bytes 2 and 4 as C forward, W back,
        S stop, and speed in 3 and 5 as (byte - 0x21) / 10 km/h.
        """
        def one(direction, magnitude):
            speed = (float(magnitude) - 0x21) / 10.0 / 3.6
            letter = chr(int(direction))
            if letter == "C":
                return speed
            if letter == "W":
                return -speed
            return 0.0
        return one(data[2], data[3]), one(data[4], data[5])

    def on_wheel_status(self, message):
        self.wheel_status_stamp = rospy.Time.now()
        data = message.data
        if len(data) > 1:
            self.drive_mode = data[1]
        if len(data) >= 6:
            left, right = self.reported_wheel_speeds(data)
            self.measured_speed = (left + right) / 2.0
            self.measured_yaw_rate = (right - left) / WHEEL_SEPARATION_M

    def on_start(self, request):
        if request.data:
            unsafe = self.band.route_centre_clearance_violations()
            unsafe_chords = self.band.route_centre_chord_violations()
            if unsafe or unsafe_chords:
                self.enabled = False
                self.send_stop()
                message = (
                    "REFUSED: route centre lacks bilateral wheel clearance "
                    "at stations %s or leaves the band on chords %s"
                    % (unsafe[:20], unsafe_chords[:20]))
                rospy.logerr(message)
                return SetBoolResponse(success=False, message=message)
        self.enabled = request.data
        if not request.data:
            self.send_stop()
        rospy.loginfo("follower %s", "ENABLED" if self.enabled else "PAUSED")
        return SetBoolResponse(success=True,
                               message="ENABLED" if self.enabled else "PAUSED")

    # ------------------------------------------------------------ safety
    def hazard_slack(self):
        """Distance from the nearest wheel to the nearest mapped fall
        hazard, in metres, or infinity where there is none.

        Deliberately NOT the band's total width. Width without a drop
        behind it is not a safety quantity - a hedge or a wall is something
        the obstacle guard sees and stops for, whereas a kerb into a road
        is invisible to the MID360 and only this band knows about it.
        Measured on this route the distinction costs almost nothing anyway:
        of the 78 stations where width would have been the binding term,
        75 have a hazard nearby and are governed by it regardless.
        """
        return self.band.hazard_clearance(self.pose_xy)

    def slack_speed(self):
        """Full speed with room to spare, creep when a hazard is alongside,
        and a continuous ramp between - never a step change."""
        slack = self.hazard_slack()
        if slack >= SLACK_FULL_SPEED_M:
            return MAX_SPEED
        span = SLACK_FULL_SPEED_M - SLACK_CREEP_M
        ratio = max(0.0, min(1.0, (slack - SLACK_CREEP_M) / span))
        return CREEP_SPEED + ratio * (MAX_SPEED - CREEP_SPEED)

    def guard_stop(self):
        """Stop radius from measured motion, scan age, and braking physics."""
        if not self.motion.valid:
            return float("inf")
        cloud_age = max(
            0.0, rospy.Time.now().to_sec() - self.cloud_stamp.to_sec())
        envelope = stopping_envelope(
            measured_speed_mps=self.motion.linear_speed_mps,
            requested_speed_mps=self.current_speed,
            measured_yaw_rate_rps=self.motion.angular_speed_rps,
            requested_yaw_rate_rps=self.last_yaw_rate,
            cloud_age_s=cloud_age,
            accumulation_s=ACCUMULATION_WINDOW_S,
            pipeline_s=PIPELINE_BUDGET_S,
            min_linear_decel_mps2=MIN_BRAKE_DECEL_MPS2,
            min_angular_decel_rps2=MIN_YAW_DECEL_RPS2,
            geometry_margin_m=GUARD_STOP_MIN_M)
        return envelope.distance_m

    def cluster_stop_radius(self):
        """Stop radius for the cluster guard.

        guard_stop() returns infinity when the motion estimate is invalid,
        which is right for a guard the motion gate stands behind and wrong
        for this one: with the policies off nothing holds the chair for a
        bad estimate, so an infinite radius would report every object as
        blocking and the chair would never leave the start. Fall back to
        the speed being commanded, which is known whatever the estimator
        is doing.
        """
        valid = self.motion.valid
        age = 0.0 if self.cluster_summary is None else max(
            0.0, rospy.Time.now().to_sec() - self.cluster_summary.stamp_s)
        envelope = stopping_envelope(
            measured_speed_mps=(self.motion.linear_speed_mps if valid
                                else self.current_speed),
            requested_speed_mps=self.current_speed,
            measured_yaw_rate_rps=(self.motion.angular_speed_rps if valid
                                   else self.last_yaw_rate),
            requested_yaw_rate_rps=self.last_yaw_rate,
            cloud_age_s=age,
            accumulation_s=CLUSTER_ACCUMULATION_S,
            pipeline_s=PIPELINE_BUDGET_S,
            min_linear_decel_mps2=MIN_BRAKE_DECEL_MPS2,
            min_angular_decel_rps2=MIN_YAW_DECEL_RPS2,
            geometry_margin_m=GUARD_STOP_MIN_M)
        return envelope.distance_m

    def stop_radius(self):
        """The distance inside which anything reported is a stop.

        guard_stop() is the braking envelope - reaction time plus stopping
        distance - and stands whatever reports the obstacle, so it remains
        the floor. cluster_stop_radius() covers the extra latency a summary
        carries over the scan this node accumulated itself. The radius has
        to cover the OLDEST data feeding it, hence the larger of the two.
        """
        radii = []
        if self.policies:
            radii.append(self.guard_stop())
        if self.clusters_enabled:
            radii.append(self.cluster_stop_radius())
        return max(radii) if radii else 0.0

    def stop_radius_for(self, threat):
        """Dynamic stop radius, with the operator's person-only lead."""
        radius = self.stop_radius()
        if threat is not None and threat.is_person:
            return radius * PERSON_STOP_DISTANCE_SCALE
        return radius

    def threat_blocks(self, threat, stop_m):
        """Apply person-stop hysteresis around the dynamic braking radius."""
        blocking = threat is not None and threat.distance_m < stop_m
        if threat is None or not threat.is_person:
            self.person_stop_release_m = None
            return blocking
        if self.person_stop_release_m is not None:
            self.person_stop_release_m = max(
                self.person_stop_release_m,
                stop_m + PERSON_STOP_RELEASE_MARGIN_M)
            if threat.distance_m <= self.person_stop_release_m:
                return True
            self.person_stop_release_m = None
        if blocking:
            self.person_stop_release_m = \
                stop_m + PERSON_STOP_RELEASE_MARGIN_M
        return blocking

    def cluster_threat(self, lateral_shift=0.0):
        """Nearest classified object overlapping its stop corridor.

        People get a slightly wider corridor and survive a single bounded
        producer dropout. The memory uses the producer stamp, so delayed
        callback execution cannot extend it. Unusable summaries still fail
        closed through the ordinary threat query and are never replaced by
        remembered geometry.
        """
        if self.cluster_summary is None:
            self.direct_person_threats = ()
            return Threat(0.0, MOVING, "no summary")
        ordinary = nearest_threat(
            self.cluster_summary, CORRIDOR_HALF_WIDTH, lateral_shift)
        person = nearest_threat(
            self.cluster_summary, PERSON_STOP_HALF_WIDTH_M, lateral_shift,
            labels=(PERSON_LABEL,))
        bypass_people = matching_threats(
            self.cluster_summary, PERSON_BYPASS_HALF_WIDTH_M, lateral_shift,
            labels=(PERSON_LABEL,))
        self.direct_person_threats = tuple(
            bypass_people if self.cluster_summary.usable else ())
        committed_person = next((
            candidate for candidate in bypass_people
            if candidate.track_id == self.person_bypass_committed_track_id
        ), None)
        if person is None and len(bypass_people) == 1:
            person = committed_person
        if self.cluster_summary.usable:
            if person is not None:
                self.person_memory = (self.cluster_summary.stamp_s, person)
            elif self.person_memory is not None:
                stamp_s, remembered = self.person_memory
                age_s = self.cluster_summary.stamp_s - stamp_s
                if 0.0 <= age_s <= PERSON_MEMORY_S:
                    person = Threat(
                        remembered.distance_m, remembered.motion,
                        remembered.label, remembered.lateral_m,
                        track_id=remembered.track_id,
                        observed_stamp_s=remembered.observed_stamp_s,
                        directly_observed=False,
                        geometry_valid=remembered.geometry_valid)
                else:
                    self.person_memory = None
        if ordinary is None:
            return person
        if person is None or ordinary.distance_m <= person.distance_m:
            return ordinary
        return person

    def corridor_threat(self, lateral_shift=0.0):
        """Nearest classified object in the corridor, or None if clear.

        Classified objects are the only obstacle source. The raw five-points-
        in-a-box check that used to sit alongside them was removed on
        2026-08-05 after it stopped three runs in one evening, and the way it
        failed is worth keeping written down.

        It reported UNKNOWN, because five loose returns have no identity from
        one scan to the next. avoidance_decision cannot watch an UNKNOWN
        stand still, so it fell through to the time rule - blocked for three
        seconds and not MOVING - and UNKNOWN is never MOVING. Every raw
        false positive therefore became an authorisation to drive around
        something nobody could name, and the offset it chose steered the
        chair at a wall. Detection and identity were never separable here:
        a source that cannot say what it saw should not be able to say what
        to do about it.

        Removing it makes perception a single point of failure, which is why
        CLUSTERS_STALE is an OVERRIDE hold and why the constructor refuses to
        start with no obstacle source at all.
        """
        if not self.clusters_enabled:
            return None
        return self.cluster_threat(lateral_shift)

    def avoidance_for(self, now, threat, blocking):
        """Parked or moving, and the blocked-for clock that backs the answer.

        Here rather than in step() for the reason handled_before_driving is:
        a second control law has to be put behind the SAME policy, not
        behind a copy of it. dwa_follower and mpc_follower replace step()
        entirely, and until 2026-08-11 neither of them asked this question
        at all - they handed the nearest threat to their solver whatever it
        was doing, so the DWA profile would pick an arc around a walking
        person at OBSTACLE_FLOOR_M while the pursuit profile stood and
        waited for the same person. That is the drift a copied guard makes,
        arrived at by omission instead, and it went the way copied guards
        always go: toward the follower that does not stop.

        blocked_since lives here too, because the time rule reads it and a
        second copy of the clock is a second answer to how long something
        has been in the way.
        """
        if blocking:
            if self.blocked_since is None:
                self.blocked_since = now
        else:
            self.blocked_since = None
        return avoidance_decision(
            threat, blocking,
            None if self.blocked_since is None
            else (now - self.blocked_since).to_sec(),
            PLAN_AHEAD_M, BYPASS_AFTER_S,
            person_bypass_ready=self.person_bypass_ready(threat, blocking))

    def reset_person_bypass_evidence(self):
        self.person_static_track_id = None
        self.person_static_since_s = None
        self.person_static_last_stamp_s = None
        self.person_bypass_committed_track_id = None

    def person_bypass_ready(self, threat, _blocking):
        """Direct same-track STATIC evidence required before a person arc."""
        people = self.direct_person_threats
        eligible = (
            threat is not None
            and threat.is_person
            and threat.parked
            and threat.distance_m < PLAN_AHEAD_M
            and threat.directly_observed
            and threat.track_id is not None
            and threat.observed_stamp_s is not None
            and threat.geometry_valid
            and self.tracking_state == "TRACKING"
            and len(people) == 1
            and people[0].track_id == threat.track_id
            and people[0].parked
            and people[0].directly_observed
            and people[0].geometry_valid
        )
        if not eligible:
            self.reset_person_bypass_evidence()
            return False
        if self.person_bypass_committed_track_id == threat.track_id:
            return True
        stamp_s = threat.observed_stamp_s
        if self.person_static_track_id != threat.track_id or \
                self.person_static_last_stamp_s is None:
            self.person_static_track_id = threat.track_id
            self.person_static_since_s = stamp_s
            self.person_static_last_stamp_s = stamp_s
            return False
        gap_s = stamp_s - self.person_static_last_stamp_s
        if gap_s < 0.0 or gap_s > PERSON_BYPASS_MAX_GAP_S:
            self.person_static_since_s = stamp_s
            self.person_static_last_stamp_s = stamp_s
            return False
        if gap_s > 0.0:
            self.person_static_last_stamp_s = stamp_s
        ready = stamp_s - self.person_static_since_s >= \
            PERSON_BYPASS_CONFIRM_S
        if ready:
            self.person_bypass_committed_track_id = threat.track_id
        return ready

    def take_a_way_round(self, clear_for_m):
        """Offset far enough to clear the corridor without leaving the band.

        The band is what makes this safe and it is checked here whatever the
        policies are set to. Containment stopping the chair is a judgement
        that can be switched off; the band knowing where there is room to
        step aside is not an opinion, and without it the smallest offset on
        offer - 0.60 m - is twice the 0.30 m median lateral clearance this
        route actually has.

        An offset lane has to be clear for clear_for_m, not merely as far as
        the chair could brake. Checking only the stopping distance picks a
        lane with something standing 4 m down it, which is a sidestep into
        the second of two objects and then a stop between them.

        The offsets tried are bounded by the room the band actually has over
        the step - see bypass_offsets() - so a corridor that cannot take a
        0.60 m step is offered a smaller one instead of that one being
        admitted on a centre-point test.
        """
        for offset in self.bypass_offsets():
            clear = self.corridor_threat(offset)
            if (clear is None or clear.distance_m > clear_for_m) and \
                    self.bypass_target_ok(offset):
                self.lateral_offset = offset
                rospy.logwarn("going round a parked obstacle: offset %+.1f m",
                              offset)
                return True
        rospy.logwarn_throttle(
            10, "no side of this has room in the band - waiting")
        return False

    def bypass_room_each_side(self):
        """Lateral room the band has over the step, per side, in metres.

        Measured at the same look-ahead points bypass_target_ok checks, and
        reduced to the tightest one: a step is only as safe as its worst
        station. The value returned is what the chair's CENTRE may take,
        already net of the room kept from the edge, so it can be used as an
        offset directly.
        """
        if self.pose_xy is None:
            return 0.0, 0.0
        heading = np.array([math.cos(self.pose_yaw), math.sin(self.pose_yaw)])
        left_room = right_room = float("inf")
        for ahead in BYPASS_PROBE_AHEAD_M:
            _, lo, hi = self.band.lateral_limits(self.pose_xy + heading * ahead)
            left_room = min(left_room, hi)
            right_room = min(right_room, -lo)
        keep = BYPASS_EDGE_KEEP_M
        return (max(0.0, left_room - keep), max(0.0, right_room - keep))

    def bypass_offsets(self):
        """Offsets worth trying here, in preference order."""
        return bypass_offsets_for_room(*self.bypass_room_each_side())

    def bypass_target_ok(self, offset):
        """A lateral bypass is allowed only if the offset corridor stays
        inside the safety band for the next few meters."""
        if self.pose_xy is None:
            return False
        heading = np.array([math.cos(self.pose_yaw), math.sin(self.pose_yaw)])
        normal = np.array([-heading[1], heading[0]])
        for ahead in BYPASS_PROBE_AHEAD_M:
            p = self.pose_xy + heading * ahead + normal * offset
            if (not self.band.contains(p)
                    or not self.drivable_mask.contains(p)):
                return False
        return True

    def send_stop(self):
        self.current_speed = 0.0
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------ control
    def advance_progress(self):
        """Move the progress marker along the route, and lock on first use.

        Extracted from pure_pursuit_target() without a change of behaviour
        so a second control law advances progress the SAME way - see
        mpc_follower.py. This is not bookkeeping for the status line: it is
        what sets route_locked, and the OFF_ROUTE geofence and the OFF_BAND
        containment test are both written to apply only once locked. A
        follower that never called this would run with those two guards
        silently inert for the whole route.
        """
        d = np.linalg.norm(self.waypoints - self.pose_xy, axis=1)
        if not self.route_locked:
            self.nearest_index = int(np.argmin(d))
            self.route_locked = True
        # Distance-based, not index-based: the route carries the resampled
        # trace, so an index window would mean whatever the resampling step
        # happens to be. 15 waypoints was 75 m at the old 5 m spacing and
        # would be 3 m at 0.2 m.
        window_end = min(
            self.nearest_index + int(PROGRESS_WINDOW_M / ROUTE_STEP_M) + 1,
            len(self.waypoints))
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

    def pure_pursuit_target(self):
        self.advance_progress()
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
        # Steer at the recorded line. Aiming at the middle of the usable
        # band instead was tried and driven: it moves the target off that
        # line by up to 1.10 m (6 of 75 waypoints beyond 0.5 m), and in the
        # field the chair wandered 2.68 m off the line at wp 7 - which the
        # lean displaces by +0.50 m - and later headed for a kerb until it
        # was stopped by hand.
        #
        # The asymmetry is the point: the recorded line is where a person
        # actually drove, while the band edges come from a step-detection
        # heuristic over the map. Trading a proven path for an inferred
        # midpoint means any error in the inferred kerb steers straight at
        # the real one. Containment below still clamps into the band, so
        # the band constrains the chair without commanding it.
        if abs(self.lateral_offset) > 0.01:
            direction = target - self.pose_xy
            norm = np.linalg.norm(direction)
            if norm > 1e-3:
                normal = np.array([-direction[1], direction[0]]) / norm
                target = target + normal * self.lateral_offset
        target = self.band.clamp(target) if self.policies else target
        return target if self.drivable_mask.contains(target) else self.pose_xy

    def safe_target(self, wanted):
        """Return the longest target whose complete drive chord is in band."""
        # With containment off the target is the recorded line itself, which
        # is where a person drove this route on 0727 - the band is what
        # constrains a departure from it, and there is nothing left here to
        # depart from it.
        if not self.policies:
            return self.target_at_lookahead(
                max(MIN_LOOKAHEAD_M, wanted)), MAX_SPEED, True
        candidate = max(MIN_LOOKAHEAD_M, wanted)
        while True:
            target = self.target_at_lookahead(candidate)
            mask_chord_ok = self.drivable_mask.segment_is_contained(
                self.pose_xy, target
            )
            if self.band.chord_is_contained(
                    self.pose_xy, target, grace=OFF_BAND_GRACE
                    ) and mask_chord_ok:
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

    def hold_candidates(self, now):
        """Every reason to hold, highest priority first, each tagged with
        whether it is a safety policy or an override.

        A generator, because the order is both the priority order and the
        order in which the tests are safe to run at all: NO_POSE is what
        guarantees the position tests below it have a position to read, so
        nothing after it may be evaluated once it fires. See drive_policy
        for which tags mean what and why the line is drawn there.
        """
        if not self.enabled or self.done:
            yield ("DONE" if self.done else "PAUSED"), OVERRIDE
            return
        if self.pose_xy is None or \
                (now - self.pose_stamp).to_sec() > POSE_STALE_S:
            yield "NO_POSE", OVERRIDE
            return
        if (now - self.cloud_stamp).to_sec() > 1.0:
            yield "NO_CLOUD", POLICY
        motion_reason = motion_hold_reason(
            self.motion, now.to_sec(), ODOM_STALE_S)
        if motion_reason:
            yield motion_reason, POLICY
        degraded_age_s = None if self.degraded_since is None else \
            (now - self.degraded_since).to_sec()
        localization_reason = localization_hold_reason(
            self.tracking_state, degraded_age_s, DEGRADED_STOP_S,
            reason=self.tracking_reason,
            reacquire_m=(self.reacquire_distance()
                         if self.tracking_reason == SUPPRESSED_WHILE_PARKED
                         else None))
        if localization_reason:
            yield localization_reason, POLICY
        if (now - self.wheel_status_stamp).to_sec() > BASE_STALE_S:
            yield "BASE_STALE", OVERRIDE
            return
        if self.drive_mode is not None and self.drive_mode != AUTO_MODE:
            yield "MANUAL_MODE", OVERRIDE
            return
        # Liveness, not judgement: with the policies off this guard is the
        # only thing still watching for people, so a producer that died
        # silently would leave the chair driving on an empty object list
        # that looks exactly like clear road. Tagged OVERRIDE for the same
        # reason BASE_STALE is - it is how the failsafe is observed to
        # exist, not an opinion about what is out there.
        if self.clusters_enabled and is_stale(
                None if self.cluster_summary is None
                else self.cluster_summary.stamp_s, now.to_sec()):
            yield "CLUSTERS_STALE", OVERRIDE
            return
        if self.route_locked and np.min(np.linalg.norm(
                self.waypoints - self.pose_xy, axis=1)) > GEOFENCE_M:
            yield "OFF_ROUTE", POLICY
        if self.route_locked and not self.band.contains(
                self.pose_xy, grace=BAND_RECOVER_MAX):
            yield "OFF_BAND", POLICY

    def handled_before_driving(self, now):
        """Run every hold and the goal test, and say whether one of them has
        already taken the cycle.

        Extracted from step() without a change of behaviour so that a second
        control law can be put behind the SAME guards rather than behind a
        copy of them - see mpc_follower.py. A copy is the failure mode worth
        engineering against here: the judgements themselves live in
        hold_candidates and evaluate_holds and would have been shared
        anyway, but the dispatch around them carries the DEGRADED
        bookkeeping, the WOULD_HOLD publish that the black box records, and
        the stop - and a second copy of that drifts from this one silently,
        in the direction of a follower that does not stop.
        """
        if self.tracking_state == "DEGRADED":
            if self.degraded_since is None:
                self.degraded_since = now
        else:
            self.degraded_since = None
        reason, suppressed = evaluate_holds(
            self.hold_candidates(now), self.policies)
        if suppressed:
            # Not a hold, but the whole reason to drive with the policies
            # off is to learn where they would have bitten. It goes on the
            # topic the black box records, not just into a log.
            self.status_pub.publish(String(data="WOULD_HOLD:" + suppressed))
            rospy.logwarn_throttle(
                5.0, "policies off: would have held on %s", suppressed)
        if reason:
            if reason != self.status:
                rospy.loginfo("hold: %s", reason)
                self.status = reason
            self.status_pub.publish(String(data="HOLD:" + reason))
            self.send_stop()
            return True

        if np.linalg.norm(self.waypoints[-1] - self.pose_xy) < GOAL_TOLERANCE_M:
            self.done = True
            self.send_stop()
            rospy.loginfo("GOAL REACHED")
            return True
        return False

    def step(self):
        now = rospy.Time.now()
        if self.handled_before_driving(now):
            return

        recovering = self.policies and self.route_locked and \
            not self.band.contains(self.pose_xy, grace=OFF_BAND_GRACE)

        # None reads downstream as "nothing in the corridor". A missing
        # cluster summary does NOT come back as None - cluster_threat
        # reports it as blocking - so a quiet producer stops the chair
        # rather than reading as clear road.
        threat = self.corridor_threat(self.lateral_offset)
        obstacle_dist = None if threat is None else threat.distance_m

        allowed = MAX_SPEED
        if recovering:
            allowed = min(allowed, CREEP_SPEED)
        if self.policies:
            allowed = min(allowed, self.slack_speed())
            allowed = min(allowed, mpc_speed.slope_speed_limit(
                self.pose_pitch))
            if self.tracking_state == "DEGRADED":
                allowed = min(allowed, SLOPE_SPEED)

        blocking = None
        guard_stop = self.stop_radius_for(threat)
        guard_slow = guard_stop + GUARD_SLOW_EXTRA_M
        if obstacle_dist is not None:
            if obstacle_dist < guard_stop:
                blocking = "OBSTACLE"
                allowed = 0.0
            elif obstacle_dist < guard_slow:
                ratio = (obstacle_dist - guard_stop) / GUARD_SLOW_EXTRA_M
                allowed = min(allowed,
                              CREEP_SPEED + ratio * (MAX_SPEED - CREEP_SPEED))

        decision = self.avoidance_for(
            now, threat, self.threat_blocks(threat, guard_stop))
        if decision == PERSON_BYPASS:
            self.status_pub.publish(String(data="HOLD:PERSON_BYPASS_DWA_ONLY"))
            self.send_stop()
            return
        if decision == GO_ROUND and abs(self.lateral_offset) < 0.01:
            self.take_a_way_round(max(guard_slow, PLAN_AHEAD_M))
        if blocking is None:
            if abs(self.lateral_offset) > 0.01:
                back = self.corridor_threat(0.0)
                if back is None or back.distance_m > guard_slow:
                    self.lateral_offset = 0.0
                    rospy.loginfo("way round complete, rejoining the line")

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
        # Slowing for a turn must not slow past the speed at which the base
        # can still perform one. The wheel encoder sends tenths of a km/h,
        # and below roughly 1.3 km/h at the faster wheel the loaded chair
        # was measured not to rotate at all - so commanding a hard turn at a
        # crawl asks for something that never arrives, then arrives at once.
        # The floor rises with how hard the turn is.
        turning = min(abs(yaw_rate) / MAX_YAW_RATE, 1.0)
        curvature_floor = TURN_FLOOR_SPEED * turning
        allowed = min(allowed,
                      max(curvature_floor, CREEP_SPEED,
                          MAX_SPEED * (1.0 - abs(heading_error) / 1.2)))
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
        self.status_pub.publish(String(data="%s wp=%d/%d v=%.2f%s" % (
            state, self.nearest_index, len(self.waypoints),
            self.current_speed,
            "" if self.policies else " POLICIES_OFF")))
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
