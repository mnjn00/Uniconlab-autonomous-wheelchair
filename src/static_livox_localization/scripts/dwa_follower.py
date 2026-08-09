#!/usr/bin/env python3
"""The DWA control law, put behind the pursuit follower's guards.

Third profile, same contract as the other two. It subclasses
WaypointFollower and replaces only the part that turns a pose into a Twist;
the hold ladder, geofence, band containment, localisation health, cluster
liveness failsafe, manual-mode override and stop-on-shutdown are inherited
and run unmodified. Nothing safety-bearing is restated here - a copied guard
drifts from the original silently, and always toward the follower that does
not stop.

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
from cluster_guard import corridor_obstacle_points
from mpc_anchor import DEFAULT_GAIN, StateAnchor
from mpc_command import MAX_COMMAND_GAP_S, advance_command
from waypoint_follower import (WaypointFollower, CONTROL_HZ,
                               CORRIDOR_HALF_WIDTH, MAX_ACCEL, MAX_DECEL,
                               MAX_YAW_RATE, PLAN_AHEAD_M)

# The solver returns a velocity target, not an acceleration, so the ramp is
# fed the acceleration that would close the gap in one period - clamped to
# what the base can do. Deceleration is allowed to be brisker than
# acceleration for the same reason the pursuit follower allows it: stopping
# sooner is never the unsafe direction.
YAW_SLEW_RPS2 = 1.5


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
        """The corridor obstacles ahead, where the producer measured them.

        Rolled out against their own returns rather than against one point
        on the chair's heading axis. Collapsing an object to its distance
        and then re-inflating it dead ahead throws away the only thing a
        rollout critic can act on - which side the thing is on - and on
        2026-08-09 that turned a passable obstruction into a permanent hold
        at wp 905. A parked van whose nearest corridor return sat 0.5 m to
        the RIGHT was handed to the planner as a point 0.55 m straight
        ahead. The minimum turning radius here is TURN_FLOOR_SPEED /
        MAX_YAW_RATE = 0.6 m, and no arc off a 0.6 m radius clears 0.40 m of
        a point 0.55 m ahead - the best any candidate achieves is 0.20 m -
        so every candidate was rejected. Nothing recovers from that: a stop
        is not a candidate, there is no reverse, and the van was static, so
        the threat that rejected everything never changed. Meanwhile the
        chair had 0.55 m of empty band to its left and was asking for full
        left yaw. The distance was right; only the bearing was invented.

        Which objects count is corridor_obstacle_points' business and is
        unchanged - what the corridor test already calls blocking, nothing
        more, so this cannot start stopping for the walls it drives past.
        Their full extent is passed, not only the slices inside the
        corridor: going round something means clearing the part of it that
        sticks out where the chair is headed.
        """
        if not self.clusters_enabled:
            return ()
        points = corridor_obstacle_points(
            self.cluster_summary, CORRIDOR_HALF_WIDTH,
            max_distance_m=PLAN_AHEAD_M)
        if not points:
            return ()
        yaw = float(state[2])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        return np.asarray(points, dtype=float) @ rotation.T + state[:2]

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

        target_v, target_w, status = self.planner.plan(
            state, self.obstacle_points(state), speed_cap=float(v_ref[0]),
            last_yaw_rate=self.last_yaw_rate)
        if status != "OK":
            if status != self.dwa_status:
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
