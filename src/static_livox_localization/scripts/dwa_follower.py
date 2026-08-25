#!/usr/bin/env python3
"""The DWA control law, put behind the pursuit follower's guards.

Third profile, same contract as the other two. It subclasses
WaypointFollower and replaces only the part that turns a pose into a Twist;
the hold ladder, geofence, band containment, localisation health, cluster
liveness failsafe, manual-mode override, the parked-or-moving decision and
stop-on-shutdown are inherited and run unmodified. Nothing safety-bearing is
restated here - a copied guard drifts from the original silently, and always
toward the follower that does not stop.

The parked-or-moving decision was on that list by intention and not in fact
until 2026-08-11. Replacing step() dropped it, because it lived in the body
of the method rather than beside the guards that were carefully extracted,
and no copy drifted - the profile simply never asked. The planner was handed
the nearest threat whatever the tracker said about it, so where the pursuit
profile stands and waits for someone walking, this one picked an arc round
them at OBSTACLE_FLOOR_M. It is WaypointFollower.avoidance_for now, asked by
every profile.

WHAT THIS ONE IS FOR
--------------------
Pursuit and the MPC both follow a line well and avoid things badly. The
avoidance in this stack is take_a_way_round: a fixed +-0.6 m lateral offset
applied to the pursuit target. On 2026-08-04 that offset, taken from a
standstill where the lookahead has collapsed to MIN_LOOKAHEAD_M 0.9, is a
demand for atan(0.6 / 0.9) = 34 degrees, and it steered the chair at a wall
at WP585, WP910 and WP924 in one evening. Rolling candidate velocities out
and scoring them cannot produce that: a candidate is a (v, w) the chair can
hold, and one whose rollout leaves the corridor is discarded rather than
commanded.

The corridor is enforced as a rollout critic, not a costmap layer, and
dwa_core's docstring carries the measurement behind that choice.

WHAT IT SHARES WITH THE MPC PROFILE
-----------------------------------
The command ramp (mpc_command.advance_command). Both profiles hit the same
wall on 2026-08-05: integrating the solver's acceleration onto the MEASURED
velocity cannot start a chair whose wheels ignore anything under 0.30 m/s.
The ramp integrates onto the last command over the time that actually
elapsed, and dwa_core's speed sampling keeps the deadband out of the
candidate set in the first place. Two defences, because the failure was
silent both times.

STATE
-----
Driven once, on 2026-08-08, and it failed: two runs covered 44 m and 48 m of
the 380 m route in 1009 s and 726 s. The planner's score had no heading term,
which made it a bang-bang regulator, and standing still was a candidate that
beat every moving arc whenever the chair sat on the line - 180 s stuck in one
run, 77 s in the other. Both are fixed in dwa_core, whose docstrings carry the
numbers. Not re-driven since. PROFILE still defaults to pursuit, which drove
this route twice on 2026-07-31 with a person in the chair.

Two further changes on 2026-08-11, neither of them driven either, and both
found by reading rather than by a run: the parked-or-moving decision above,
and the object's shape instead of its nearest point (obstacle_points). Each
is written against a defect visible in the code and in the recorded data,
and neither is field evidence.

`OFF_BAND` is still a stop only a person can clear - every rollout point is
tested, so a chair a centimetre outside the corridor has no admissible
candidate at all, and the 08-08 analysis measured that at 23 %. A bounded
recovery for it is written and tested but deliberately NOT merged: it is the
only change in this stack that turns a stop into motion, and it is waiting
on one run of the route to be worth its risk. That run is what this profile
needs before anything else.
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import dwa_core
import mpc_speed
from cluster_guard import (GO_ROUND, PERSON_BYPASS_CLEARANCE_M,
                           PERSON_BYPASS_SPEED_MPS, PERSON_LABEL, WAIT,
                           corridor_obstacle_points)
from mpc_anchor import DEFAULT_GAIN, StateAnchor
from mpc_command import MAX_COMMAND_GAP_S, advance_command, jerk_limited
from route_mask import RouteMask
from waypoint_follower import (WaypointFollower, CONTROL_HZ, CREEP_SPEED,
                               GUARD_SLOW_EXTRA_M, MAX_ACCEL, MAX_DECEL,
                               MAX_SPEED, MAX_YAW_RATE, PLAN_AHEAD_M)

# The solver returns a velocity target, not an acceleration, so the ramp is
# fed the acceleration that would close the gap in one period - clamped to
# what the base can do. Deceleration is allowed to be brisker than
# acceleration for the same reason the pursuit follower allows it: stopping
# sooner is never the unsafe direction.
YAW_SLEW_RPS2 = 1.5

# Actuation lag, measured 2026-08-11 (see led_state). Overridable with
# ~latency_s for a vehicle this has not been measured on; at 0.0 the lead is
# the identity and the profile behaves exactly as it did before.
#
# Back to 0.0 on 2026-08-15. The 0.55 was measured while mpc_speed.MAX_SPEED
# was still 0.6, and the lead it produces is a distance, not a time:
# speed x 0.55, so 0.33 m at the speed it was measured at and 0.55 m at the
# 1.0 the cap was raised to afterwards. The 08-15 drive steered in S-curves
# that the 08-12 drive at 0.6 did not, and led_state's own warning is that a
# lead longer than the real lag over-steers. Unmeasured at this speed is
# what it now is, so it takes the value the docstring prescribes for that.
# Re-measure at 1.0 m/s (command angular.z against yaw rate differentiated
# from /fast_lio_icp/pose) before putting a non-zero number back.
# Back on, and for the first time it will do something. led_state leads the
# pose by speed x lag, and until 2026-08-23 the speed in the state was always
# zero - /Odometry carries no twist and nothing decoded the wheel report - so
# every value of this constant was identical to nought. The 08-15 note that
# set it to 0.0 blamed a weave on a lead that was never applied.
#
# The measurement stands and has now been corroborated twice: 2026-08-11
# cross-correlated commanded angular.z against the yaw rate differentiated
# from /fast_lio_icp/pose and found the peak at 0.55 s with gain 1.03 and
# R^2 0.90, and the steady-state gain measured off /wheel_status on 08-23,
# over 15 samples above 0.7 m/s where the command was held 0.8 s or longer,
# comes out at 1.03 as well. The base turns as asked; it just takes 0.55 s
# to get there, and at 0.8 m/s that is 0.44 m of travel the planner was
# correcting for a place the chair had already left.
LATENCY_S = 0.55

# How wide a slice of each object the planner is shown, either side of the
# centreline. Wider than the follower's CORRIDOR_HALF_WIDTH 0.45, which is
# the width the parked/moving DECISION is taken over: a rollout is free to
# step aside anywhere the band allows, so the geometry it is scored against
# has to cover where it may go and not only where the route runs. Bounded
# because every extra slice is another point in an O(candidates x steps x
# points) batch inside a 0.1 s period.
OBSTACLE_HALF_WIDTH_M = 1.0

# The gate refusing something this follower cannot see.
#
# There are two obstacle sources on this chair and they do not see the same
# world. safety_gate works on raw returns inside a height band and has no
# idea what anything is; the follower and the planner work on classified
# clusters and cannot see what the producer did not cluster. When something
# lands only in the first - a bush leaning into the corridor, a thin post,
# clutter the producer files as outside_band - the follower reports a clear
# corridor and keeps commanding 0.80 m/s while the gate zeroes every frame
# of it. Nothing resolves: the follower never registers a threat, so
# blocked_since never starts, and the chair stands there reading DWA v=0.80
# until somebody walks over to it. That shape was on the wire at wp 1218 on
# 2026-08-23.
#
# What this does NOT do is route around it. The gate can say a distance and
# a side and nothing else, and a source that cannot say what it saw must
# not be allowed to say what to do about it - the raw-scan guard removed on
# 2026-08-05 taught that at the cost of three stopped runs and a chair
# steered at a wall. So the deadlock becomes a named, recorded hold with
# the gate's own numbers on it, and a person decides.
GATE_STALL_S = 1.5
# Only the obstacle-shaped vetoes. CLOUD_STALE, INPUT_STALE and the rest
# are their own faults with their own handling, and folding them in here
# would relabel a dead sensor as an object in the road.
GATE_OBSTACLE_REASONS = ("OBSTACLE", "OBSTACLE_SWEEP")


def gate_stall(reason, blocked_for_s, stall_s=GATE_STALL_S):
    """True when the gate has been refusing an obstacle we cannot see.

    Pure, and deliberately ignorant of what the follower thinks: the caller
    reaches it only after its own WAIT has declined to fire, so by then the
    absence of a threat of our own is established rather than re-tested.
    """
    if reason not in GATE_OBSTACLE_REASONS:
        return False
    if blocked_for_s is None:
        return False
    return float(blocked_for_s) >= float(stall_s)


def approach_cap(base_cap, distance_m, stop_m, floor_mps):
    """Speed for closing on something the chair may have to go round.

    The ramp exists so the chair arrives at a parked object already slow
    enough to steer past it, rather than at cruise. It used to ramp down to
    CREEP_SPEED, which is below the speed the loaded wheels turn at, and that
    turned the approach into a stop: dwa_core.speed_samples returns nothing
    executable under its floor, the planner has no candidate to score, and
    the manoeuvre the ramp was preparing for never gets attempted.

    Measured on 2026-08-23 against a parked motorcycle at wp 1437-1441: the
    ramp handed down 0.21, 0.25, 0.28 and finally 0.15 m/s, and the chair
    stopped 1.3 m short and sat there for 2.4 minutes with the object
    correctly tracked as static and crossing the band the whole time.

    So the ramp stops at the floor. Below it there is no such thing as a
    slower approach, only a stop, and a stop is what WAIT and the planner's
    own obstacle critic are for - both of which still hold: WAIT halts before
    this is reached, rollouts inside OBSTACLE_FLOOR_M are rejected, and the
    gate stops outright for anything inside the braking envelope.
    """
    if distance_m >= stop_m + GUARD_SLOW_EXTRA_M:
        return base_cap
    ratio = max(0.0, (distance_m - stop_m) / GUARD_SLOW_EXTRA_M)
    ramped = CREEP_SPEED + ratio * (MAX_SPEED - CREEP_SPEED)
    return min(base_cap, max(floor_mps, ramped))


class DwaFollower(WaypointFollower):
    CONTROL_LAW = "dwa"

    def __init__(self):
        WaypointFollower.__init__(self)
        self.latency_s = float(rospy.get_param("~latency_s", LATENCY_S))
        self.planner = dwa_core.DwaPlanner(
            self.band, self.waypoints,
            distance_m=float(rospy.get_param("~sim_distance_m",
                                              dwa_core.SIM_DISTANCE_M)),
            grace=float(rospy.get_param("~band_grace", 0.0)),
            route_mask=RouteMask(rospy.get_param("~drivable_mask")))
        self.anchor = StateAnchor(
            gain=float(rospy.get_param("~anchor_gain", DEFAULT_GAIN)))
        self.last_command_stamp = None
        self.odom_v = 0.0
        self.odom_w = 0.0
        self.dwa_status = ""
        # Carried across cycles so the ramp has a slope to be limited against.
        self.command_accel = 0.0
        self.gate_reason = ""
        self.gate_blocked_since = None
        self.gate_detail = ""
        rospy.Subscriber("/safety_gate/status", String,
                         self.on_gate_status, queue_size=1)
        rospy.loginfo(
            "DWA profile: sim %.2f m, %d speeds x %d yaw rates, band and "
            "drivable mask as hard rejects", self.planner.distance_m,
            len(dwa_core.speed_samples()), len(dwa_core.yaw_samples()))

    def on_gate_status(self, message):
        """Track how long the gate has been refusing, and why."""
        try:
            report = json.loads(message.data)
        except (ValueError, TypeError):
            return
        reason = str(report.get("reason") or "")
        if reason != self.gate_reason:
            self.gate_reason = reason
            self.gate_blocked_since = rospy.Time.now() if reason else None
        nearest = report.get("zone_nearest_m")
        envelope = report.get("envelope_m")
        self.gate_detail = "%s at %s m, envelope %s m" % (
            reason or "clear",
            "?" if nearest is None else nearest,
            "?" if envelope is None else envelope)

    def gate_blocked_for(self, now):
        if self.gate_blocked_since is None:
            return None
        return (now - self.gate_blocked_since).to_sec()

    def send_stop(self):
        # The jerk limit shapes driving, never braking. Dropping the carried
        # acceleration here is what keeps a stop as abrupt as it was before.
        self.command_accel = 0.0
        WaypointFollower.send_stop(self)

    def on_odom(self, message):
        """Wheel odometry, kept for the anchor as well as for the base."""
        WaypointFollower.on_odom(self, message)
        self.odom_v = float(message.twist.twist.linear.x)
        self.odom_w = float(message.twist.twist.angular.z)

    def obstacle_points(self, state, only_label=None):
        """The objects ahead, as the returns the rollouts must clear.

        Where they actually are, not straight ahead: placing every threat on
        the heading is what turned a wall beside the chair into one in front
        of it, and with the corridor 0.3 m wide at that station nothing could
        clear OBSTACLE_FLOOR_M and the profile held for 211 cycles on
        2026-08-09.

        And the whole of them, not one point. The nearest return of an object
        is where a stopping radius is measured from, and it was the only
        thing passed here until 2026-08-11 - so a wall spanning the corridor
        became a single point, and an arc could be admitted for clearing that
        one point by 0.41 m while driving through the metres of wall either
        side of it. cluster_guard.object_points publishes each lateral slice
        the object occupies, which is the same measurement, ungeneralised.
        """
        if not self.clusters_enabled or self.cluster_summary is None:
            return ()
        blocks, points = corridor_obstacle_points(
            self.cluster_summary, OBSTACLE_HALF_WIDTH_M,
            max_distance_m=PLAN_AHEAD_M, only_label=only_label)
        if not blocks or not points:
            return ()
        heading = np.array([math.cos(state[2]), math.sin(state[2])])
        left = np.array([-heading[1], heading[0]])
        return [state[:2] + heading * forward + left * lateral
                for forward, lateral in points]

    def led_state(self, state):
        """The state advanced by the actuation lag, along the arc it is on.

        Only the pose is led; the speed and yaw rate stay as measured,
        because those are what the ramp integrates from. A lead longer than
        the real lag over-steers exactly as badly as no lead under-steers,
        so this is a measured number and not a tuning knob - and at zero it
        is the identity, which is what an unmeasured vehicle should use.
        """
        lead = self.latency_s
        if lead <= 0.0:
            return state
        led = np.array(state, dtype=float)
        led[0] += state[3] * math.cos(state[2]) * lead
        led[1] += state[3] * math.sin(state[2]) * lead
        led[2] += state[4] * lead
        return led

    def step(self):
        now = rospy.Time.now()
        # Before the guards: route_locked is what arms OFF_ROUTE and
        # OFF_BAND, and it is set here.
        if self.pose_xy is not None:
            self.advance_progress()
        if self.handled_before_driving(now):
            self.anchor.reset("held")
            self.last_command_stamp = None
            return

        # /Odometry carries no twist - FAST-LIO leaves it at zero, and all
        # 13,395 samples of the 08-23 run are exactly 0.000 - so the anchor
        # had been folding in a velocity of nought and the rollouts started
        # from a chair that was never moving. The base reports what the
        # wheels are doing at 100 Hz; that is the measurement.
        state = self.anchor.update(self.pose_xy, self.pose_yaw,
                                   self.measured_speed, self.measured_yaw_rate,
                                   now.to_sec())
        # The speed policy still applies: the corridor ahead, the slope and
        # a DEGRADED fix all cap what any candidate may be worth choosing.
        v_ref, stop_reason = mpc_speed.shaped_reference(
            self.band, state[:2], 1, pitch_rad=self.pose_pitch,
            degraded=(self.tracking_state == "DEGRADED"))
        if stop_reason:
            self.publish_state("HOLD:SLOWER_THAN_FLOOR", "HOLD")
            self.send_stop()
            self.last_command_stamp = None
            return

        # Parked or moving, answered before the planner is asked anything.
        # Rolling velocities out and scoring them is a way of GOING ROUND
        # something, and going round is only ever the answer for an object
        # the tracker has watched stand still. Anything moving, or not yet
        # watched long enough to say, is waited out where it stands - a
        # sidestep is a manoeuvre into where they are about to be. Until
        # 2026-08-11 this profile never asked: it handed the nearest threat
        # to the planner whichever it was, and the planner would clear a
        # walking person by OBSTACLE_FLOOR_M and drive past.
        threat = self.corridor_threat(0.0)
        stop_m = self.stop_radius_for(threat)
        decision = self.avoidance_for(
            now, threat,
            threat is not None and threat.distance_m < stop_m)
        if decision == WAIT:
            self.publish_state("HOLD:DWA_WAIT", "HOLD:DWA_WAIT")
            self.dwa_status = "WAIT"
            self.send_stop()
            self.last_command_stamp = None
            return

        # Past our own WAIT, so whatever the gate is holding, it is not
        # something the cluster producer gave us. Say so and stop asking.
        if gate_stall(self.gate_reason, self.gate_blocked_for(now)):
            if self.dwa_status != "GATE_STALL":
                rospy.logwarn(
                    "gate is refusing an obstacle the follower cannot see: "
                    "%s - the cluster producer reports a clear corridor",
                    self.gate_detail)
            self.publish_state("HOLD:GATE_STALL " + self.gate_detail,
                               "HOLD:GATE_STALL")
            self.dwa_status = "GATE_STALL"
            self.send_stop()
            self.last_command_stamp = None
            return

        # Approach at the pursuit profile's pace, whatever the decision. A
        # planner that only knows stop-or-cruise arrives at what it is about
        # to wait for at full speed and stops there.
        cap = float(v_ref[0])
        if threat is not None:
            cap = approach_cap(cap, threat.distance_m, stop_m,
                               dwa_core.TURN_FLOOR_SPEED)

        # Geometry only when going round it. Handing the planner an object it
        # is not allowed to go round would let it sidestep anyway.
        obstacles = self.obstacle_points(state) if decision == GO_ROUND else ()
        # Going round a PERSON is the same manoeuvre executed differently:
        # a wider berth, and slowly enough that if they move after all
        # there is time to stop rather than to swerve. avoidance_decision
        # has already established they have stood still long enough to be
        # standing there rather than pausing.
        wide = ()
        clearance = None
        if decision == GO_ROUND and threat is not None and threat.is_person:
            # Only THEIR returns get the wider berth. Everything else in the
            # corridor - the wall the chair is turning towards to make room
            # - keeps the ordinary one.
            wide = self.obstacle_points(state, only_label=PERSON_LABEL)
            clearance = PERSON_BYPASS_CLEARANCE_M
            cap = min(cap, PERSON_BYPASS_SPEED_MPS)
            if self.dwa_status != "PERSON_BYPASS":
                rospy.logwarn(
                    "going round someone who has stood still: %.2f m ahead, "
                    "%.2f m berth, %.2f m/s", threat.distance_m,
                    PERSON_BYPASS_CLEARANCE_M, cap)
            self.dwa_status = "PERSON_BYPASS"
        # Plan from where the chair will be when the command lands, not from
        # where it is. The gap was measured on 2026-08-11 by cross-correlating
        # commanded angular.z against the yaw rate differentiated from
        # /fast_lio_icp/pose: the correlation peaks at 0.55 s with gain 1.03
        # and R^2 0.90, and the identical figure comes off /Odometry, so it is
        # upstream of the ICP correction. Unfixed, that lag costs 0.33 m of
        # travel at 0.6 m/s and 0.55 m at 1.0 - the planner correcting for
        # where the chair no longer is, which is what lateral hunting is.
        state = self.led_state(state)
        target_v, target_w, status = self.planner.plan(
            state, obstacles, speed_cap=cap,
            last_yaw_rate=self.last_yaw_rate,
            last_speed=self.current_speed,
            wide_obstacles=wide, wide_clearance_m=clearance)
        if status != "OK":
            if status != self.dwa_status:
                if status == "SPEED_BELOW_FLOOR":
                    rospy.logwarn(
                        "DWA %s at wp %d/%d: cap %.2f m/s is under the "
                        "%.2f m/s the wheels turn for (v_ref %.2f%s)",
                        status, self.nearest_index, len(self.waypoints),
                        cap, dwa_core.TURN_FLOOR_SPEED, float(v_ref[0]),
                        ", threat guard" if threat is not None else "")
                else:
                    rospy.logwarn("DWA %s at wp %d/%d", status,
                                  self.nearest_index, len(self.waypoints))
            self.dwa_status = status
            self.publish_state("HOLD:DWA_" + status, "HOLD:DWA_" + status)
            self.send_stop()
            self.last_command_stamp = None
            return
        self.dwa_status = status

        elapsed = 1.0 / CONTROL_HZ
        if self.last_command_stamp is not None:
            elapsed = (now - self.last_command_stamp).to_sec()
        if elapsed > MAX_COMMAND_GAP_S:
            rospy.logwarn_throttle(
                5.0, "control loop gap %.2f s - resyncing command to measured",
                elapsed)
            self.current_speed = float(state[3])
            self.last_yaw_rate = float(state[4])
        self.last_command_stamp = now
        step = max(elapsed, 1e-3)
        wanted_accel = np.clip((target_v - self.current_speed) / step,
                               -MAX_DECEL, MAX_ACCEL)
        self.command_accel = jerk_limited(
            wanted_accel, self.command_accel, step)
        accel = np.array([
            self.command_accel,
            np.clip((target_w - self.last_yaw_rate) / step,
                    -YAW_SLEW_RPS2, YAW_SLEW_RPS2)])
        speed, yaw_rate = advance_command(
            self.current_speed, self.last_yaw_rate, accel, elapsed,
            dwa_core.MAX_SPEED, MAX_YAW_RATE)

        command = Twist()
        command.linear.x = speed
        command.angular.z = yaw_rate if speed > 0.02 else 0.0
        self.cmd_pub.publish(command)
        self.current_speed = speed
        self.last_yaw_rate = yaw_rate
        self.publish_state(
            "DWA wp=%d/%d v=%.2f w=%+.2f target %.2f/%+.2f%s" % (
                self.nearest_index, len(self.waypoints), speed, yaw_rate,
                target_v, target_w,
                "" if self.policies else " POLICIES_OFF"), "DWA:OK")

    def publish_state(self, text, state=None):
        """Publish every cycle, log only on a change of state.

        The line carries v and w, which move every cycle, so comparing the
        whole line to decide whether to log would log at 10 Hz.
        """
        self.status_pub.publish(String(data=text))
        state = state or text
        if state != self.status:
            rospy.loginfo("state: %s", text)
            self.status = state


if __name__ == "__main__":
    try:
        DwaFollower().run()
    except rospy.ROSInterruptException:
        pass
