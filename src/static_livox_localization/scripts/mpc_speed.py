"""The velocity reference the MPC tracks, and the floor below which this
controller does not drive at all.

Section 7 of docs/mpc_follower_design.md asks the node to shape v_ref with
"the follower's existing speed policy (narrow-band creep, slope slowdown)
instead of a constant 0.6", on the grounds that arriving at the route's
~334 m choke at cruise is what ends the jittered runs in an INFEASIBLE_STOP.
Both halves of that were measured on 2026-08-04 before implementing them,
and both came back different. The mandate is implemented here in the shape
the measurements support, not the shape it was written in.

WHAT THE MEASUREMENTS SAID
--------------------------
1. Slowing down does not relieve the choke. Over the v4 band with 2 cm of
   lateral jitter and the EMA anchor, a constant 0.6 reference reaches
   350 m; a constant 0.3 reference stops EARLIER, at 334 m. The pinch is
   geometric, not dynamic - the band leaves the chair centre 0.13 m of
   lateral freedom there, and a hard half-plane corridor that narrow is
   infeasible against jitter no matter how slowly it is entered. Speed is
   not the lever, so nothing here claims to fix it. See the runbook: the
   MPC profile does not complete the route, and ships gated off.

2. Below a threshold the controller does not move at all. Sweeping a
   constant reference over 20 s from a standing start:

       v_ref  0.10 0.15 0.18 0.20 0.22 | 0.25 0.30 0.40 0.60
       actual 0.00 0.00 0.00 0.00 0.00 | 0.24 0.29 0.40 0.60

   At and below 0.22 the chair settles at exactly zero - and the solve
   returns OK while it does. There is no progress term in the objective;
   forward motion is bought only by w_vel * (v - v_ref)^2, and below about
   0.22 that purchase no longer covers the lateral, heading and rate cost
   of moving, so standing still becomes optimal. A creep reference of 0.15
   would therefore have parked the chair at the first narrow station and
   reported nothing - worse than the INFEASIBLE_STOP it was meant to avoid,
   because an infeasible stop at least announces itself.

WHAT THIS MODULE DOES ABOUT IT
------------------------------
The reference is floored at TURN_FLOOR_SPEED, and anything the policy wants
slower than that floor is returned as a STOP rather than as a slower
reference. This controller has no creep regime: it drives at 0.30 or it
holds. The floor is not chosen to dodge the dead zone - it is the follower's
own constant, measured on the chair, because below roughly 1.3 km/h at the
faster wheel the loaded base does not rotate at all. That the physical floor
sits above the solver's dead zone is luck, but it means one number serves
both and neither is a fudge.

The policy sources are the follower's, not new tuning: the hazard ramp is
slack_speed() transcribed, and the slope and DEGRADED rules are the ones
around it in update(). is_narrow is deliberately NOT among them - the
pursuit follower does not consult it, and adding it here would be new
tuning wearing the clothes of an existing policy.

CAVEAT ON THE HAZARD RAMP: hazard_clearance is finite at 0 of 758 stations
on the v4 band, as it was at 0 of 152 sampled on v5. The ramp below is
faithful to the follower and inert on both shipped routes. It earns its
place when a band carries measured drop semantics; today it is dead code
kept honest, not a working safeguard, and the open item to re-measure the
band's edge kinds still stands.
"""

import math

import numpy as np

# The pursuit follower's constants, transcribed rather than re-tuned. Kept
# literal because waypoint_follower pulls in rospy and this must stay
# importable at a desk.
MAX_SPEED = 0.6
SLOPE_SPEED = 0.3
CREEP_SPEED = 0.15
SLACK_FULL_SPEED_M = 0.8
SLACK_CREEP_M = 0.15
SLOPE_PITCH_RAD = math.radians(3.0)
# Below this the loaded base was measured not to rotate; it is also above
# the solver's measured standstill threshold of 0.22.
TURN_FLOOR_SPEED = 0.30

STOP = "STOP"


def hazard_speed(clearance_m):
    """slack_speed() from waypoint_follower, to the letter: full speed with
    room to spare, creep with a kerb alongside, a continuous ramp between."""
    if not np.isfinite(clearance_m) or clearance_m >= SLACK_FULL_SPEED_M:
        return MAX_SPEED
    span = SLACK_FULL_SPEED_M - SLACK_CREEP_M
    ratio = max(0.0, min(1.0, (clearance_m - SLACK_CREEP_M) / span))
    return CREEP_SPEED + ratio * (MAX_SPEED - CREEP_SPEED)


def policy_speed(band, point, pitch_rad=0.0, degraded=False,
                 obstacle_speed=None):
    """What the follower's policy would allow here, before the floor.

    May legitimately return something below TURN_FLOOR_SPEED - deciding what
    that means is shaped_reference's job, not this one's.
    """
    limit = hazard_speed(band.hazard_clearance(point))
    if abs(float(pitch_rad)) > SLOPE_PITCH_RAD:
        limit = min(limit, SLOPE_SPEED)
    if degraded:
        limit = min(limit, SLOPE_SPEED)
    if obstacle_speed is not None:
        limit = min(limit, float(obstacle_speed))
    return min(limit, MAX_SPEED)


def horizon_speed(band, point, horizon_m=None, pitch_rad=0.0, degraded=False,
                  obstacle_speed=None, samples=6):
    """The policy's verdict over the stations the horizon actually reaches.

    Looking only under the wheels is what leaves a restriction to be found on
    arrival. The walk is along the band's own stations, so it follows the
    corridor round a bend rather than probing a straight line through
    whatever the bend encloses.
    """
    point = np.asarray(point, dtype=float)
    if horizon_m is None:
        horizon_m = MAX_SPEED * 2.5
    at = lambda q: policy_speed(band, q, pitch_rad, degraded, obstacle_speed)
    k0 = int(np.argmin(np.linalg.norm(band.xy - point, axis=1)))
    tail = band.xy[k0:]
    limit = at(point)
    if len(tail) > 1:
        seg = np.linalg.norm(np.diff(tail, axis=0), axis=1)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        for target in np.linspace(0.0, horizon_m, samples)[1:]:
            j = int(np.searchsorted(arc, target))
            if j >= len(tail):
                break
            limit = min(limit, at(tail[j]))
    return float(limit)


def shaped_reference(band, point, horizon, pitch_rad=0.0, degraded=False,
                     obstacle_speed=None):
    """(v_ref, stop_reason) for mpc_core.solve_cycle.

    stop_reason is None when the chair may drive, or a string when the
    policy wants a speed this controller cannot deliver - in which case
    v_ref is all zeros and the caller must stop rather than creep.

    The reference is flat across the horizon on purpose. The lookahead
    already happened; a per-step profile would encode it twice and make the
    reference disagree with itself between cycles as the chair advances.
    """
    limit = horizon_speed(band, point, pitch_rad=pitch_rad, degraded=degraded,
                          obstacle_speed=obstacle_speed)
    if limit < TURN_FLOOR_SPEED:
        return np.zeros(int(horizon)), STOP
    return np.full(int(horizon), limit), None
