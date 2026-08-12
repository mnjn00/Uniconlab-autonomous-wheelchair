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

Three further changes on 2026-08-11, none of them driven either, and all
three found by reading rather than by a run: the parked-or-moving decision
above, the object's shape instead of its nearest point (obstacle_points),
and dwa_core's bounded recovery for the OFF_BAND stop that the 08-08
analysis recorded at 23 % and could not reach by scoring. Each is written
against a defect that is visible in the code and the recorded data, and none
of them is field evidence.

The first two shipped; the recovery is the one still held back, and this
docstring is on the branch that carries it. It is the only change in this
stack that turns a stop into motion - everything else here fails closed -
and it is waiting on one complete run of the route, which is what this
profile needs before anything else.
"""

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
from cluster_guard import GO_ROUND, WAIT, corridor_obstacle_points
from mpc_anchor import DEFAULT_GAIN, StateAnchor
from mpc_command import MAX_COMMAND_GAP_S, advance_command
from waypoint_follower import (WaypointFollower, CONTROL_HZ, CREEP_SPEED,
                               GUARD_SLOW_EXTRA_M, MAX_ACCEL, MAX_DECEL,
                               MAX_SPEED, MAX_YAW_RATE, PLAN_AHEAD_M)

# The solver returns a velocity target, not an acceleration, so the ramp is
# fed the acceleration that would close the gap in one period - clamped to
# what the base can do. Deceleration is allowed to be brisker than
# acceleration for the same reason the pursuit follower allows it: stopping
# sooner is never the unsafe direction.
YAW_SLEW_RPS2 = 1.5

# How wide a slice of each object the planner is shown, either side of the
# centreline. Wider than the follower's CORRIDOR_HALF_WIDTH 0.45, which is
# the width the parked/moving DECISION is taken over: a rollout is free to
# step aside anywhere the band allows, so the geometry it is scored against
# has to cover where it may go and not only where the route runs. Bounded
# because every extra slice is another point in an O(candidates x steps x
# points) batch inside a 0.1 s period.
OBSTACLE_HALF_WIDTH_M = 1.0


class DwaFollower(WaypointFollower):
    CONTROL_LAW = "dwa"

    def __init__(self):
        WaypointFollower.__init__(self)
        self.planner = dwa_core.DwaPlanner(
            self.band, self.waypoints,
            sim_time_s=float(rospy.get_param("~sim_time_s",
                                             dwa_core.SIM_TIME_S)),
            grace=float(rospy.get_param("~band_grace", 0.0)))
        self.anchor = StateAnchor(
            gain=float(rospy.get_param("~anchor_gain", DEFAULT_GAIN)))
        self.last_command_stamp = None
        self.odom_v = 0.0
        self.odom_w = 0.0
        self.dwa_status = ""
        rospy.loginfo(
            "DWA profile: sim %.2f s, %d speeds x %d yaw rates, band as a "
            "hard reject", self.planner.sim_time_s,
            len(dwa_core.speed_samples()), len(dwa_core.yaw_samples()))

    def on_odom(self, message):
        """Wheel odometry, kept for the anchor as well as for the base."""
        WaypointFollower.on_odom(self, message)
        self.odom_v = float(message.twist.twist.linear.x)
        self.odom_w = float(message.twist.twist.angular.z)

    def obstacle_points(self, state):
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
            max_distance_m=PLAN_AHEAD_M)
        if not blocks or not points:
            return ()
        heading = np.array([math.cos(state[2]), math.sin(state[2])])
        left = np.array([-heading[1], heading[0]])
        return [state[:2] + heading * forward + left * lateral
                for forward, lateral in points]

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

        state = self.anchor.update(self.pose_xy, self.pose_yaw,
                                   self.odom_v, self.odom_w, now.to_sec())
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
        stop_m = self.stop_radius()
        decision = self.avoidance_for(
            now, threat,
            threat is not None and threat.distance_m < stop_m)
        if decision == WAIT:
            self.publish_state("HOLD:DWA_WAIT", "HOLD:DWA_WAIT")
            self.dwa_status = "WAIT"
            self.send_stop()
            self.last_command_stamp = None
            return

        # Approach at the pursuit profile's pace, whatever the decision. A
        # planner that only knows stop-or-cruise arrives at what it is about
        # to wait for at full speed and stops there.
        cap = float(v_ref[0])
        if threat is not None and threat.distance_m < stop_m + \
                GUARD_SLOW_EXTRA_M:
            ratio = max(0.0, (threat.distance_m - stop_m) / GUARD_SLOW_EXTRA_M)
            cap = min(cap, CREEP_SPEED + ratio * (MAX_SPEED - CREEP_SPEED))

        # Geometry only when going round it. Handing the planner an object it
        # is not allowed to go round would let it sidestep anyway.
        obstacles = self.obstacle_points(state) if decision == GO_ROUND else ()
        target_v, target_w, status = self.planner.plan(
            state, obstacles, speed_cap=cap,
            last_yaw_rate=self.last_yaw_rate)
        if status not in ("OK", "RECOVER"):
            if status != self.dwa_status:
                rospy.logwarn("DWA %s at wp %d/%d", status,
                              self.nearest_index, len(self.waypoints))
            self.dwa_status = status
            self.publish_state("HOLD:DWA_" + status, "HOLD:DWA_" + status)
            self.send_stop()
            self.last_command_stamp = None
            return
        if status == "RECOVER" and status != self.dwa_status:
            rospy.logwarn(
                "DWA recovering toward the corridor at wp %d/%d",
                self.nearest_index, len(self.waypoints))
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
        accel = np.array([
            np.clip((target_v - self.current_speed) / step,
                    -MAX_DECEL, MAX_ACCEL),
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
        # RECOVER goes on the topic the black box records, not only into a
        # log: a run where the chair spent a third of its time creeping back
        # to the corridor drove, and a report that reads it as ordinary
        # driving hides the thing worth measuring.
        self.publish_state(
            "DWA%s wp=%d/%d v=%.2f w=%+.2f target %.2f/%+.2f%s" % (
                "" if status == "OK" else ":" + status,
                self.nearest_index, len(self.waypoints), speed, yaw_rate,
                target_v, target_w,
                "" if self.policies else " POLICIES_OFF"), "DWA:" + status)

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
