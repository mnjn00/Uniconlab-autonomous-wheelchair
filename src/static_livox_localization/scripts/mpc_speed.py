"""The velocity reference the MPC tracks, and the floor below which this
controller does not drive at all.

Section 7 of docs/mpc_follower_design.md asks the node to shape v_ref with
"the follower's existing speed policy (narrow-band creep, slope slowdown)
instead of a constant 0.6", on the grounds that arriving at the route's
~334 m choke at cruise is what ends the jittered runs in an INFEASIBLE_STOP.
Both halves of that were measured on 2026-08-04 before implementing them.
Its instinct - slow down for the pinch - turned out right; the quantity to
slow for and the speed to slow to were both wrong, and finding that out
meant fixing a defect in the heading reference first. The mandate is
implemented here in the shape the measurements support.

WHAT THE MEASUREMENTS SAID
--------------------------
1. Slowing down relieves the choke - but only after a defect underneath it
   was fixed, and the first pass here concluded the opposite. Measured over
   the v4 band at 2 cm jitter, a constant 0.6 reached 350 m and a constant
   0.3 stopped EARLIER, at 334 m, which reads as clear evidence that speed
   is not the lever. It was not. The dominant fault was in the HEADING
   reference: mpc_core.polyline_refs snapped to the nearest polyline
   segment, turning a 26.7-degree inter-station step into a demand to
   rotate at 2.2 rad/s against a 0.5 rad/s cap, and both speeds were
   drowning in that. With the heading interpolated along arc instead, the
   same pinch is infeasible at 0.5 m/s and solves at 0.4.

   The lesson is kept here because the reasoning was sound and the answer
   was still wrong: a lever measured on top of a defect measures the
   defect. Section 7 of the design doc is amended with the same story.

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

On top of that floor the reference is shaped by the corridor ahead, which
IS the lever once the heading reference is honest: the route's narrowest
metre is infeasible at 0.5 m/s and solves at 0.4, so the chair arrives at
it already slow. That rule is this controller's own and is not transcribed
from pursuit, for a reason worth stating - pursuit never writes the band
into a solver, so a pinch is only a line it must not cross, while here it
is a constraint the whole horizon has to satisfy in advance.

The remaining policy sources ARE the follower's, not new tuning: the hazard
ramp is slack_speed() transcribed, and the slope and DEGRADED rules are the
ones around it in update().

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

# Corridor-width shaping. This one is NOT transcribed from the pursuit
# follower, and the reason it is justified here and absent there is the
# whole difference between the two controllers: this one writes the band
# into the QP as HARD half-planes over a 2.5 s horizon, so a corridor that
# pinches is a constraint it must satisfy in advance, not a limit it merely
# must not cross. Pursuit has no such obligation and needs no such rule.
#
# The route's narrowest metre leaves 0.13 m. Measured at that state: 0.5 m/s
# is infeasible, 0.4 m/s solves. The ramp below puts the chair at the floor
# there, with the numbers chosen so the whole v4 route completes at 2 cm
# jitter across seeds.
#
# Recalibrated for the v5 band on 2026-08-09. Those numbers came from v4 and
# on v5 they never bind: the corridor there runs 0.52 to 1.44 m total, all
# of it above the old 0.60 m full-speed point, so the ramp sat at full ratio
# everywhere and returned 0.52-0.60 m/s through a pinch leaving the chair's
# CENTRE 0.26 m either side. It even rose as the corridor closed, because
# the 15 m lookahead reads the widening beyond a pinch the chair is still
# inside.
#
# Width here is the total, so these are twice the room the chair's centre
# gets: the floor from 0.275 m per side, full speed from 0.325 m. That looks
# a hair's breadth apart and is: the v5 corridor is either roomy or it is
# not, and the lookahead does the smoothing. Chosen against the band's own
# distribution rather than by feel - 60 % of stations still reach full speed
# and the pinch drops to 0.33 m/s. Widening the ramp to 0.70 m costs 7
# points of that 60 % for 0.01 m/s in the pinch, which is the wrong trade:
# a policy that slows everywhere is just a slower chair, and
# test_corridor_shaping_leaves_open_road_alone exists to say so.
#
# v4's 0.13 m still lands on the floor, which is what that measurement asked.
CORRIDOR_TIGHT_M = 0.55
CORRIDOR_FULL_M = 0.65
# Deliberately far longer than braking needs - 0.6 to 0.3 takes under a
# metre. The horizon is what has to be feasible, not just the wheels, so
# the chair should already be slow when the pinch enters the horizon, not
# when it reaches the axle.
CORRIDOR_LOOKAHEAD_M = 15.0
# The curvature cap needs only to cover the horizon it is shaping - a bend
# 15 m away does not constrain the yaw rate of the next 1.5 m, and using the
# corridor's lookahead here would drag the whole route down to the sharpest
# turn anywhere in sight.
CURVE_LOOKAHEAD_M = 2.5

STOP = "STOP"


def corridor_speed(band, point, lookahead_m=CORRIDOR_LOOKAHEAD_M):
    """Speed the narrowest corridor within reach will accept.

    Takes the MINIMUM width ahead rather than the width here: arriving at a
    pinch fast is what makes it infeasible, and by the time the narrow
    station is under the chair it is far too late to be braking for it.
    """
    point = np.asarray(point, dtype=float)
    k0 = int(np.argmin(np.linalg.norm(band.xy - point, axis=1)))
    tail = band.xy[k0:]
    if len(tail) > 1:
        seg = np.linalg.norm(np.diff(tail, axis=0), axis=1)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        tail = tail[arc <= lookahead_m]
    width = min(_corridor_width(band, q) for q in tail) \
        if len(tail) else _corridor_width(band, point)
    return speed_for_width(width)


def speed_for_width(width_m):
    """The ramp itself, as a function of corridor width alone.

    Separated from the band lookup because the measurement behind it is
    about WIDTH, not about a particular route: a 0.13 m corridor was
    infeasible at 0.5 m/s and solved at 0.4. Which band ships is a
    deployment decision that has already changed once.
    """
    span = CORRIDOR_FULL_M - CORRIDOR_TIGHT_M
    ratio = max(0.0, min(1.0, (float(width_m) - CORRIDOR_TIGHT_M) / span))
    return TURN_FLOOR_SPEED + ratio * (MAX_SPEED - TURN_FLOOR_SPEED)


def _corridor_width(band, point):
    _lateral, lo, hi = band.lateral_limits(point)
    return hi - lo


def curvature_speed(band, point, w_max=None, lookahead_m=CURVE_LOOKAHEAD_M):
    """Speed at which the chair can actually hold the reference heading.

    The heading reference sweeps at curvature x speed. Where the route bends
    hardest that product reaches 0.563 rad/s at cruise, against a 0.5 rad/s
    cap - so at exactly two of 756 stations the reference asks for a turn
    the chair cannot make. Capping v at w_max / curvature makes the demand
    equal to what the chair has, which is the honest version of the same
    request.

    This is the residual left over from fixing polyline_refs, and it is left
    to the speed policy on purpose: the alternative was smoothing the
    curvature away, which would have hidden a real 71-degree turn rather
    than slowing for it.
    """
    if w_max is None:
        # taken from the solver's own limit rather than kept as a second
        # copy of it: two constants for one physical cap disagree quietly,
        # and the direction they disagree in is a reference the chair
        # cannot follow.
        import mpc_core
        w_max = mpc_core.MpcParams().w_max
    point = np.asarray(point, dtype=float)
    k0 = int(np.argmin(np.linalg.norm(band.xy - point, axis=1)))
    span = band.xy[k0:]
    if len(span) < 3:
        return MAX_SPEED
    seg = np.diff(span, axis=0)
    ds = np.linalg.norm(seg, axis=1)
    within = np.cumsum(ds) <= lookahead_m
    if within.sum() < 2:
        within[:2] = True
    heading = np.unwrap(np.arctan2(seg[within, 1], seg[within, 0]))
    if len(heading) < 2:
        return MAX_SPEED
    kappa = np.abs(np.diff(heading)) / np.maximum(ds[within][1:], 1e-6)
    peak = float(kappa.max())
    if peak < 1e-6:
        return MAX_SPEED
    return min(MAX_SPEED, float(w_max) / peak)


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

    corridor_speed is folded in once rather than at every sample: it already
    carries its own, much longer lookahead, so calling it per sample would
    re-scan the same 15 m six times for the same answer.
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
    return float(min(limit, corridor_speed(band, point),
                     curvature_speed(band, point)))


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
