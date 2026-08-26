#!/usr/bin/env python3
"""The MPC control law, put behind the pursuit follower's guards.

This is the vehicle layer for mpc_core: it subclasses WaypointFollower and
replaces ONLY the part that turns a pose into a Twist. Everything that
decides whether the chair may move at all - the hold ladder, the geofence,
the band containment test, the localisation health rules, the cluster
liveness failsafe, the manual-mode override, the stop on shutdown - is
inherited and runs unmodified, through the same handled_before_driving()
the pursuit follower calls. Nothing safety-bearing is reimplemented here,
because a second copy of a guard drifts from the first silently and always
toward the follower that does not stop.

What that leaves this file responsible for is small and stated plainly:

  * anchoring the horizon (mpc_anchor) rather than re-seeding it on the raw
    localiser pose - design section 7, mandate 1;
  * shaping the velocity reference (mpc_speed) rather than asking for a
    constant 0.6 - section 7, mandate 2, implemented in the form the
    2026-08-04 measurements support and NOT in the form the section asks
    for; mpc_speed's docstring carries the numbers and the reason;
  * compensating for actuation latency, which is a hook here with a
    measured-on-the-NUC default of zero (see LATENCY_S);
  * translating the solver's ladder into the same status vocabulary and the
    same stop the rest of the stack already speaks.

READ THIS BEFORE PUTTING IT ON THE CHAIR
----------------------------------------
It completes the 0727 route in simulation, at an injected jitter measured
from the black box to sit around the p98 of what the chair actually sees,
across seeds, never leaving the band. It has never driven the chair.

Those are different claims and the gap between them is the whole reason
PROFILE still defaults to pursuit. Pursuit drove this route twice on
2026-07-31, on the real ground, with a person in the chair. This has passed
a simulation - one whose fidelity is bounded by a unicycle plant with no
actuation lag, on a band that is itself a recording. Simulation is what
caught the three defects behind the old 350 m stop; it is not what
establishes that a controller is safe to sit in.

So: run it when an operator asks for it by name, watch it, and promote it
only on evidence from the chair. The runbook carries what to watch and
what is still unmeasured (latency, NUC solve time, the feel of the
steering).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import mpc_core
import mpc_speed
from cluster_guard import GO_ROUND, WAIT
from mpc_anchor import DEFAULT_GAIN, StateAnchor
from mpc_command import MAX_COMMAND_GAP_S, advance_command
from waypoint_follower import (WaypointFollower, CONTROL_HZ, MAX_YAW_RATE,
                               PLAN_AHEAD_M)

# Time between reading a pose and the base acting on the Twist that came of
# it. Rolling the state forward by it before solving is the difference
# between steering from where the chair is and from where it was. The
# default is zero because it has NOT been measured on the NUC yet, and a
# guessed latency is a guessed lead - it would bias every command on the
# route in one direction with nothing to point at. The runbook carries the
# procedure; set ~latency_s once there is a number.
LATENCY_S = 0.0

class MpcFollower(WaypointFollower):
    # Read by the inherited __init__ and published as ~control_law, which is
    # what tools/go_mpc.sh checks before it will start a drive. Without it
    # the two profiles are indistinguishable from outside the process: same
    # node name, same topics, same service, and a status line that reads
    # HOLD:PAUSED for both until the chair is already moving.
    CONTROL_LAW = "mpc"

    def __init__(self):
        WaypointFollower.__init__(self)
        self.solver = mpc_core.MpcSolver(mpc_core.Reference(self.band))
        self.anchor = StateAnchor(
            gain=float(rospy.get_param("~anchor_gain", DEFAULT_GAIN)))
        self.latency_s = float(rospy.get_param("~latency_s", LATENCY_S))
        self.warm = None
        self.last_command_stamp = None
        self.odom_v = 0.0
        self.odom_w = 0.0
        self.mpc_status = ""
        rospy.loginfo(
            "MPC profile: horizon %d x %.2f s, anchor gain %.2f, "
            "latency %.3f s", self.solver.p.horizon, self.solver.p.dt,
            self.anchor.gain, self.latency_s)
        if self.latency_s <= 0.0:
            rospy.logwarn("latency compensation OFF (~latency_s unset); "
                          "see docs/runbooks/mpc_profile.md to measure it")

    # ---- state in ---------------------------------------------------

    def on_odom(self, message):
        """Wheel odometry, kept for the anchor as well as for the base.

        The velocities the solver starts from come from here rather than
        from differencing the localiser: this signal is smooth, local, and
        is what the actuator actually just did.
        """
        WaypointFollower.on_odom(self, message)
        self.odom_v = float(message.twist.twist.linear.x)
        self.odom_w = float(message.twist.twist.angular.z)

    def anchored_state(self, now):
        """(X, Y, theta, v, w) to plan from, latency included."""
        state = self.anchor.update(self.pose_xy, self.pose_yaw,
                                   self.odom_v, self.odom_w, now.to_sec())
        if self.latency_s > 0.0:
            state = mpc_core.unicycle_step(state, np.zeros(2), self.latency_s)
        return state

    # ---- obstacles --------------------------------------------------

    def mpc_obstacles(self, state, threat):
        """The threat ahead, as a half-plane the solver may bend for.

        The side is chosen by mpc_core from the band's own limits, so an
        obstacle can never authorise ground the band says breaks. Only the
        nearest one is passed: the corridor is a metre wide, so a second
        obstacle behind the first constrains nothing the first does not.

        The caller decides whether there is anything to bend for at all -
        bending round something is a bypass, and only avoidance_for may
        authorise one. Passing a threat here that the tracker has not
        watched stand still is what this profile did until 2026-08-11.

        Placement is still frontal, which the DWA profile stopped doing on
        2026-08-09 after a wall beside the chair became one in front of it.
        A half-plane is not a point and takes a different fix than
        obstacle_points did; it is an open defect and not an oversight.
        """
        if threat is None or threat.distance_m > PLAN_AHEAD_M:
            return ()
        heading = np.array([math.cos(state[2]), math.sin(state[2])])
        centre = state[:2] + heading * threat.distance_m
        return (mpc_core.obstacle_half_plane(
            self.solver.ref, centre, self.solver.p.obstacle_padding),)

    # ---- the control law --------------------------------------------

    def step(self):
        now = rospy.Time.now()
        # Before the guards, not after: route_locked is what arms OFF_ROUTE
        # and OFF_BAND, and it is set here. Guarded against a missing pose
        # because the guards are what normally establish there is one, and
        # on the first cycles there is not.
        if self.pose_xy is not None:
            self.advance_progress()
        if self.handled_before_driving(now):
            self.anchor.reset("held")
            self.warm = None
            # send_stop zeroes current_speed; drop the stamp too so the
            # first cycle after a hold ramps from rest rather than crediting
            # itself the whole length of the hold.
            self.last_command_stamp = None
            return

        state = self.anchored_state(now)
        v_ref, stop_reason = mpc_speed.shaped_reference(
            self.band, state[:2], self.solver.p.horizon,
            pitch_rad=self.pose_pitch,
            degraded=(self.tracking_state == "DEGRADED"))
        if stop_reason:
            # The policy wants a speed this controller cannot deliver: below
            # about 0.22 m/s the solver settles at a standstill and reports
            # OK while doing it, and below 0.30 the loaded base does not
            # rotate. Holding is the honest version of both.
            self.publish_state("HOLD:SLOWER_THAN_FLOOR")
            self.send_stop()
            self.warm = None
            self.last_command_stamp = None
            return

        # Parked or moving, before the solver is asked to bend for anything.
        # Same policy, same clock, same module as the pursuit profile - see
        # WaypointFollower.avoidance_for for why it is not restated here.
        threat = self.corridor_threat(0.0) if self.clusters_enabled else None
        stop_m = self.stop_radius_for(threat)
        decision = self.avoidance_for(
            now, threat, self.threat_blocks(threat, stop_m))
        if decision == WAIT:
            self.publish_state("HOLD:MPC_WAIT")
            self.mpc_status = "WAIT"
            self.send_stop()
            self.warm = None
            self.last_command_stamp = None
            return

        _v, th_ref = mpc_core.polyline_refs(
            self.band, state[:2], self.solver.p.horizon, self.solver.p.dt,
            float(v_ref[0]))
        u0, status, info = self.solver.solve_cycle(
            state, v_ref, th_ref,
            self.mpc_obstacles(state, threat if decision == GO_ROUND else None),
            self.warm)
        if status in (mpc_core.STATUS_OK, mpc_core.STATUS_REUSED):
            self.warm = (info["xbar"], info["ubar"]) if "xbar" in info \
                else self.warm
        else:
            self.warm = None

        if status.endswith("_STOP"):
            if status != self.mpc_status:
                rospy.logwarn("MPC %s at wp %d/%d", status,
                              self.nearest_index, len(self.waypoints))
            self.mpc_status = status
            self.publish_state("HOLD:" + status)
            self.send_stop()
            self.last_command_stamp = None
            return
        self.mpc_status = status

        # The solver returns accelerations; the base takes velocities. The
        # conversion integrates onto the previous COMMAND over the time that
        # actually elapsed - see advance_command for why both halves of that
        # sentence are the fix rather than a detail - and is clamped rather
        # than trusted: an inaccurate solve must not put a velocity on the
        # wire that the caps forbid.
        elapsed = 1.0 / CONTROL_HZ
        if self.last_command_stamp is not None:
            elapsed = (now - self.last_command_stamp).to_sec()
        if elapsed > MAX_COMMAND_GAP_S:
            # Long enough that the base has been running on a stale command.
            # Re-sync to what it is measured to be doing before ramping on.
            rospy.logwarn_throttle(
                5.0, "control loop gap %.2f s - resyncing command to measured",
                elapsed)
            self.current_speed = float(state[3])
            self.last_yaw_rate = float(state[4])
        self.last_command_stamp = now
        speed, yaw_rate = advance_command(
            self.current_speed, self.last_yaw_rate, u0, elapsed,
            self.solver.p.v_max, MAX_YAW_RATE)
        command = Twist()
        command.linear.x = speed
        command.angular.z = yaw_rate if speed > 0.02 else 0.0
        self.cmd_pub.publish(command)
        self.current_speed = speed
        self.last_yaw_rate = yaw_rate

        self.publish_state("MPC wp=%d/%d v=%.2f w=%+.2f %s%s" % (
            self.nearest_index, len(self.waypoints), speed, yaw_rate,
            status, "" if self.policies else " POLICIES_OFF"),
            state="MPC:" + status)

    def publish_state(self, text, state=None):
        """Publish every cycle, log only on a change of state.

        The published line carries v and w, which move every cycle - so
        comparing the whole line to decide whether to log would log at
        10 Hz. `state` is the coarse word the operator actually watches.
        """
        self.status_pub.publish(String(data=text))
        state = state or text
        if state != self.status:
            rospy.loginfo("state: %s", text)
            self.status = state


if __name__ == "__main__":
    try:
        MpcFollower().run()
    except rospy.ROSInterruptException:
        pass
