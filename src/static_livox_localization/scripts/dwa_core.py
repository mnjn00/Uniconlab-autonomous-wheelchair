"""Trajectory-rollout local planning with the safety band as a hard reject.

The third control law, and the reason it exists is narrow: pure pursuit and
the MPC both follow a line, and neither of them avoids anything well. What
avoidance the stack has is take_a_way_round - a fixed +-0.6 m lateral offset
applied to the pursuit target - and on 2026-08-04 that offset, applied from
a standstill where the lookahead has collapsed to MIN_LOOKAHEAD_M, demanded
atan(0.6 / 0.9) = 34 degrees and steered the chair at a wall three times in
one evening. Rolling out candidate velocities and scoring them cannot
produce that: every candidate is a velocity pair the chair can actually
hold, and one that leaves the corridor is discarded rather than commanded.

WHY THE BAND IS A CRITIC AND NOT A COSTMAP LAYER
------------------------------------------------
The obvious way to give move_base this corridor is a costmap_2d layer that
paints outside-the-band lethal. Measured on 2026-08-06 with the 0707
single-drive map, that does not work: the band occupies 1.15 % of a 0.20 m
grid and is one to two cells wide where it matters, so rasterising it breaks
the corridor into disconnected fragments and no plan exists at any inflation
radius. The band is a continuous, station-indexed lateral limit; forcing it
through a grid is what destroys it.

A rollout critic never rasterises anything. Each candidate trajectory is a
handful of points, each tested against the band's own geometry - the same
containment the follower already uses. So the corridor survives intact, and
it survives at its real width rather than at the grid's.

TRAJECTORY ROLLOUT, NOT THE DYNAMIC WINDOW
------------------------------------------
Textbook DWA samples only velocities reachable within one control period.
At a_max 0.18 and dt 0.1 that window is +-0.018 m/s, and the loaded base was
measured not to move below about 0.30 m/s - so from rest every reachable
candidate is one the wheels ignore, and the search never leaves zero. This
is the same standstill trap the MPC node fell into (see mpc_command).

So the velocity space is sampled whole, as base_local_planner's trajectory
rollout does, and the acceleration limit is enforced downstream by the
command ramp. Candidate speeds are drawn from {0} union [floor, max]: the
gap is not a tuning choice, it is the part of the range the chair cannot
execute.
"""

import math

import numpy as np

# The follower's constants. Kept literal rather than imported because
# waypoint_follower pulls in rospy and this has to stay testable at a desk.
MAX_SPEED = 0.8
# Under roughly 0.30 m/s the loaded wheels do not turn at all. The
# operator asked for 0.35 on 2026-08-23: 0.30 is the edge of the
# deadband and a command sitting on an edge is not a stable command.
TURN_FLOOR_SPEED = 0.35
MAX_YAW_RATE = 0.5

# How far ahead a candidate is simulated, as a DISTANCE rather than a time.
# Long enough to see a corridor bend arriving and short enough that a
# constant-curvature arc is still a fair description of what the chair will
# do. Both ends matter: lengthen it and every candidate fails on a curve
# because no single arc stays in a bending corridor; shorten it and the
# chair drives into pinches it never looked at.
#
# It was 1.7 s, which is the same thing only while the speed never changes.
# Raising the cap to 1.0 m/s turned that into a 1.7 m arc, and measured
# against the shipped band the admissible candidate count fell from 102 to
# 78 at wp 500, 93 to 74 at wp 1500 and 82 to 67 at wp 1773 - the planner
# lost a fifth to a quarter of its choices in exactly the pinches where it
# has the fewest. A distance keeps the geometry the planner reasons about
# identical at every speed, which is also what stops the steering gain -
# and with it the lateral hunting - from growing with speed.
SIM_DISTANCE_M = 1.05
# Retained for callers that still ask in seconds; the planner does not use
# it. 1.05 m is 1.75 s at the old 0.6 m/s cruise, which is where it came
# from.
SIM_TIME_S = 1.7
SIM_STEPS = 17

SPEED_SAMPLES = 5
# How far ahead the reachable-speed window is drawn. A textbook DWA takes it
# over one control period, but this chair accelerates at 0.18 m/s^2, so one
# 0.1 s period is +/- 0.018 m/s and the window would pin the speed forever.
# Over a second the window is +0.18 / -0.60, which is the range the ramp can
# actually deliver before the next plan replaces it.
VELOCITY_WINDOW_S = 1.0
# The ramp's own limits, kept here so the window can be drawn without
# importing the follower. They match waypoint_follower; the asymmetry is the
# chair's, and it is why a needless speed change costs more than it looks:
# the brake takes it away three times faster than the drive gives it back.
MAX_ACCEL = 0.18
MAX_DECEL = 0.6
YAW_SAMPLES = 21

# Scoring weights. Path first: this stack's whole safety argument is that the
# recorded line is ground a person actually drove, so deviation is a cost and
# not merely a preference. Progress second, obstacles third - an obstacle
# that is not in the corridor has already been excluded by the band.
W_PATH = 3.0
W_PROGRESS = 1.0
W_OBSTACLE = 2.0

# Where the chair is POINTED, not only where it stands. Measured from the two
# runs on 2026-08-08: without this term the score is a position-only cost, and
# a position-only cost driving a saturating actuator is a bang-bang regulator.
# It picked +-MAX_YAW_RATE for half of every commanding sample, reversed sign
# every 1.8 s (a 1.6 m wavelength), and the two runs covered 44 m and 48 m of
# a 380 m route. Replayed against the recorded poses, this term drops
# saturation from 9 % to 2 % and from 28 % to 3 %. Anything from 0.5 up works;
# 2.0 sits in the middle of that plateau.
W_HEADING = 2.0

# Reversing the steer is not free. Small on its own - the heading term does
# the real work - but it is what stops the residual chatter between adjacent
# yaw samples. Above about 2.0 the chair starts cutting corners: at 4.0 the
# closed-loop replay lost a third of its progress and tripled its cross-track.
W_STEER = 1.0
# The same idea as W_STEER, for the other axis, and missing until 2026-08-23.
# Nothing rewarded holding a speed, so on flat ground the winner changed
# about once a second - 1,035 target changes over the flat sections of the
# 08-23 run - as the tiniest movement in path cost reordered five candidates
# that were nearly tied. Each change is felt: the ramp brakes at 0.60 m/s^2
# and accelerates at 0.18, so a flip down and back is a lurch and a long
# crawl out of it.
W_SPEED = 1.0

# How dearly the corridor's edge is bought. Containment is a hard reject, so
# without this the middle of the band and a hair inside its edge score the
# same and the chair has no reason to prefer either. On 2026-08-09 it settled
# at a steady -0.12 m and a bend put it 6 mm outside a corridor with 0.58 m
# of room each way. Squared rather than linear: the centre has to be nearly
# free and the last few centimetres nearly unaffordable, or a term that is
# cheap at the edge just biases the whole drive without ever stopping the
# excursion that matters.
#
# Held at 2.0 rather than the 4.0 first fitted. A term that pulls hard
# towards the middle also pulls hard towards a middle that moved for the
# wrong reason: on 2026-08-09 a 0.5 m swing in the map correction put the
# chair at the edge as far as the planner could tell, and the recovery it
# demanded was a real steering excursion on a chair that had not moved.
# clamp_pose_step now bounds how fast that input can move; this bounds what
# the planner does with what gets through.
#
# Not lower. Measured from 80 % of the way to the edge at three stations,
# 1.0 commands +0.00 and the rollout ends FURTHER out - it prices the
# margin without ever paying to fix it. 2.0 asks -0.20 and closes 0.80 to
# 0.47; 4.0 asks -0.40. The closed-loop figures that first suggested 1.0
# were measured on a chair that never leaves the middle, where the term has
# nothing to do, and they said nothing about the case it exists for.
W_CENTRE = 2.0
# Separate from the station band: the v8 raster is the operator's
# authoritative chair-centre region. Outside it is never selectable, and
# the last 0.5 m inside it gets progressively more expensive.
W_MASK_BOUNDARY = 3.0

# A candidate whose rollout passes closer than this to a tracked object is
# discarded outright rather than scored - the same floor mpc_core keeps.
OBSTACLE_FLOOR_M = 0.40


def speed_samples(max_speed=MAX_SPEED, floor=TURN_FLOOR_SPEED,
                  count=SPEED_SAMPLES, current=None,
                  accel=MAX_ACCEL, decel=MAX_DECEL,
                  window_s=VELOCITY_WINDOW_S):
    """Executable speeds only: stop, or something the wheels will turn for.

    The gap between them is the actuation deadband, measured on the loaded
    chair. Sampling inside it produces candidates that score well, get
    commanded, and do nothing.

    With current set, the samples are also bounded by what the ramp can
    reach from there inside window_s - the dynamic window this had been
    missing. Without it every cycle re-scored the whole range regardless of
    what the chair was doing, so a candidate two steps away could win on a
    hair and the ramp would spend a second chasing it.
    """
    if max_speed < floor:
        return (0.0,)
    low, high = floor, max_speed
    if current is not None:
        room_up = abs(float(accel)) * float(window_s)
        room_down = abs(float(decel)) * float(window_s)
        high = min(high, float(current) + room_up)
        low = max(low, float(current) - room_down)
        if high < floor:
            # Even flat out the ramp cannot reach an executable speed this
            # cycle; a stop is the only honest answer.
            return (0.0,)
        low = min(low, high)
    return (0.0,) + tuple(np.linspace(low, high, max(count, 1)))


def yaw_samples(limit=MAX_YAW_RATE, count=YAW_SAMPLES):
    return tuple(np.linspace(-limit, limit, max(count, 3)))


def rollout(state, v, w, distance_m=SIM_DISTANCE_M, steps=SIM_STEPS):
    """Where a constant (v, w) takes the chair, sampled along the way.

    Returns (n, 3) of (x, y, yaw). The intermediate points are the point -
    a candidate that ends inside the corridor having crossed out of it on
    the way is not a candidate.

    Sampled over a fixed DISTANCE, so the arc a candidate is judged on is
    the same shape whatever speed it carries. A slow candidate simply takes
    longer to walk it.

    The step rotates and THEN translates, which is what DwaPlanner._rollouts
    does in one batch. It used to translate first, so the two disagreed by a
    single step of rotation - a whole heading step at the sample spacing.
    That matters because this function is how tests check the planner's own
    output: stays_in_band(rollout(state, *planner.plan(...))) was measuring a
    trajectory the planner never scored, in exactly the direction that hides
    a candidate leaving the band on the first step.
    """
    x, y = float(state[0]), float(state[1])
    yaw0 = float(state[2])
    out = []
    steps = max(int(steps), 1)
    if v <= 0.0:
        return np.array([(x, y, yaw0)] * steps, dtype=float)
    step_s = float(distance_m) / (float(v) * steps)
    for step in range(1, steps + 1):
        yaw = yaw0 + w * step * step_s
        x += v * math.cos(yaw) * step_s
        y += v * math.sin(yaw) * step_s
        out.append((x, y, yaw))
    return np.array(out, dtype=float)


def stays_in_band(band, path, grace=0.0):
    """Every sampled point of the rollout is inside the corridor."""
    if len(path) == 0:
        return True
    return bool(np.all(band.contains_many(path[:, :2], grace=grace)))


def obstacle_clearance(path, obstacles):
    """Closest approach of a rollout to any obstacle, or inf when clear."""
    if not len(obstacles) or len(path) == 0:
        return float("inf")
    pts = np.asarray(obstacles, dtype=float).reshape(-1, 2)
    d = np.linalg.norm(path[:, None, :2] - pts[None, :, :], axis=2)
    return float(d.min())


class DwaPlanner:
    """Scores rollouts against the recorded line, inside the band.

    Every candidate is evaluated in one batch rather than one at a time.
    That is not tidiness: scored singly, 126 candidates x a 17-step rollout
    x a 2004-point route is four million distances per control cycle, which
    on this NUC is seconds, not the 0.1 s it has. The band test and the
    path distance are each a single call over the whole stack of rollouts.
    """

    def __init__(self, band, route, distance_m=SIM_DISTANCE_M,
                 grace=0.0, max_speed=MAX_SPEED, steps=SIM_STEPS,
                 route_mask=None):
        from scipy.spatial import cKDTree
        self.band = band
        self.route = np.asarray(route, dtype=float)
        self.tree = cKDTree(self.route)
        self.distance_m = float(distance_m)
        self.grace = float(grace)
        self.max_speed = float(max_speed)
        seg = np.linalg.norm(np.diff(self.route, axis=0), axis=1)
        self.arc = np.concatenate([[0.0], np.cumsum(seg)])
        self.steps = max(int(steps), 1)
        self.route_mask = route_mask
        self.last_diagnostics = {
            "total": 0,
            "band_ok": 0,
            "mask_ok": 0,
            "geometry_ok": 0,
            "obstacle_ok": 0,
            "all_ok": 0,
            "max_clearance_m": None,
        }
        tangent = np.gradient(self.route, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True),
                              1e-9)
        self.heading = np.arctan2(tangent[:, 1], tangent[:, 0])

    def arc_at(self, point):
        return float(self.arc[int(self.tree.query(np.asarray(point))[1])])

    def _rollouts(self, state, pairs):
        """(candidates, steps, 3) for every (v, w) at once."""
        v = np.asarray([p[0] for p in pairs], dtype=float)[:, None]
        w = np.asarray([p[1] for p in pairs], dtype=float)[:, None]
        k = np.arange(1, self.steps + 1)[None, :]
        # Every candidate walks the same distance, so each gets its own time
        # step. This is what makes the arc a candidate is judged on independent
        # of the speed it carries.
        dt = self.distance_m / (np.maximum(v, 1e-6) * self.steps)
        yaw = state[2] + w * k * dt
        # position by integrating the same constant-curvature arc the chair
        # would drive, step by step, so an arc that leaves the band midway
        # is caught rather than judged only on where it ends up
        dx = np.cumsum(v * np.cos(yaw) * dt, axis=1)
        dy = np.cumsum(v * np.sin(yaw) * dt, axis=1)
        return np.stack([state[0] + dx, state[1] + dy, yaw], axis=2)

    def plan(self, state, obstacles=(), speed_cap=None, last_yaw_rate=0.0,
             last_speed=None):
        """Best executable (v, w) from here, or a stop with a reason.

        Returns (v, w, status). status is OK, or the reason every candidate
        was rejected - which the operator needs: a corridor with no
        admissible arc is a different fault from one with an object in it.

        Standing still is not among the candidates. It used to be, and it
        beat every moving arc for 180 s in one 2026-08-08 run and 77 s in the
        other: a stationary rollout is a single point, so on the line its
        path cost is exactly zero, and W_PROGRESS could not outweigh W_PATH.
        The chair was scoring a reward for not moving. A stop is a refusal
        here, never a choice - it is what the caller does when this returns
        a reason instead of a command.
        """
        cap = self.max_speed if speed_cap is None else min(self.max_speed,
                                                           float(speed_cap))
        pairs = [(v, w) for v in speed_samples(cap, current=last_speed)
                 if v > 0.0
                 # Turning on the spot is not something this chair does below
                 # its rotation floor, and it is the manoeuvre that put it at
                 # a wall on 2026-08-04. Excluded.
                 for w in yaw_samples()]
        if not pairs:
            # Not a planning failure and not an obstacle: the cap handed in
            # is below the speed the wheels will actually turn for, so there
            # is nothing executable to score. It reads as a mystery stop
            # unless it says so - on 2026-08-20 it cost a stall that took an
            # hour to attribute, because the name suggested the geometry had
            # run out. The caller that set the cap is the one to look at.
            self.last_diagnostics = {
                "total": 0,
                "band_ok": 0,
                "mask_ok": 0,
                "geometry_ok": 0,
                "obstacle_ok": 0,
                "all_ok": 0,
                "max_clearance_m": None,
            }
            return 0.0, 0.0, "SPEED_BELOW_FLOOR"
        paths = self._rollouts(np.asarray(state, dtype=float), pairs)
        flat = paths[:, :, :2].reshape(-1, 2)
        # ONE pass over the band geometry, used twice: to reject the arcs that
        # leave the corridor, and below to score how near its edge the rest of
        # them run. contains_many recomputes margins_many internally, so
        # asking it and then asking margins_many searched 802 stations for
        # 1,785 points twice over - 24.4 ms each on the target NUC, 96 % of a
        # cycle with 100 ms to spend, for one answer computed twice.
        lateral, lo, hi = self.band.margins_many(flat)
        band_inside = self.band.contained(lateral, lo, hi, self.grace)
        band_ok = band_inside.reshape(len(pairs), self.steps).all(axis=1)
        if self.route_mask is not None:
            mask_inside = self.route_mask.contains_many(flat)
            mask_ok = mask_inside.reshape(
                len(pairs), self.steps).all(axis=1)
            mask_ok &= self.route_mask.paths_are_contained(paths[:, :, :2])
        else:
            mask_ok = np.ones(len(pairs), dtype=bool)
        geometry_ok = band_ok & mask_ok
        if len(obstacles):
            pts = np.asarray(obstacles, dtype=float).reshape(-1, 2)
            clear = np.linalg.norm(
                paths[:, :, None, :2] - pts[None, None, :, :],
                axis=3).min(axis=(1, 2))
        else:
            clear = np.full(len(pairs), np.inf)
        obstacle_ok = clear >= OBSTACLE_FLOOR_M
        all_ok = geometry_ok & obstacle_ok
        geometry_clearance = clear[geometry_ok]
        self.last_diagnostics = {
            "total": len(pairs),
            "band_ok": int(np.count_nonzero(band_ok)),
            "mask_ok": int(np.count_nonzero(mask_ok)),
            "geometry_ok": int(np.count_nonzero(geometry_ok)),
            "obstacle_ok": int(np.count_nonzero(obstacle_ok)),
            "all_ok": int(np.count_nonzero(all_ok)),
            "max_clearance_m": (
                float(np.max(geometry_clearance))
                if len(geometry_clearance) else None),
        }
        if not geometry_ok.any():
            return 0.0, 0.0, "OFF_BAND"
        if not all_ok.any():
            return 0.0, 0.0, "OBSTACLE"
        ok = all_ok
        d, idx = self.tree.query(flat, workers=-1)
        path_cost = d.reshape(len(pairs), self.steps).mean(axis=1)
        here = self.arc_at(state[:2])
        ends = self.tree.query(paths[:, -1, :2])[1]
        progress = self.arc[ends] - here
        penalty = np.where(np.isfinite(clear), np.maximum(0.0, 1.0 - clear), 0.0)
        # How far off the corridor's own direction each arc leaves the chair,
        # averaged over the rollout. The route indices come free from the
        # path-distance query above.
        ref = self.heading[idx].reshape(len(pairs), self.steps)
        aim = np.abs(np.arctan2(np.sin(paths[:, :, 2] - ref),
                                np.cos(paths[:, :, 2] - ref))).mean(axis=1)
        steer = np.abs(np.asarray([p[1] for p in pairs]) - float(last_yaw_rate))
        held = 0.0 if last_speed is None else float(last_speed)
        speed_change = np.abs(np.asarray([p[0] for p in pairs]) - held)
        # Where the rollout sits between the corridor's two edges: 0 in the
        # middle, 1 against either edge. Taken from the band's own geometry
        # rather than from distance to the recorded line, because the line is
        # not always the middle - where the band is asymmetric the clearance
        # that matters is the smaller side, not the deviation. The margins are
        # the ones containment was already decided from, above.
        half = np.maximum((hi - lo) / 2.0, 1e-6)
        edge = np.abs(lateral - (hi + lo) / 2.0) / half
        centre = np.square(np.minimum(edge, 1.0)).reshape(
            len(pairs), self.steps).mean(axis=1)
        if self.route_mask is None:
            mask_boundary = np.zeros(len(pairs), dtype=float)
        else:
            mask_boundary = self.route_mask.boundary_cost_many(flat).reshape(
                len(pairs), self.steps).mean(axis=1)
        cost = (W_SPEED * speed_change
                + W_PATH * path_cost + W_HEADING * aim - W_PROGRESS * progress
                + W_OBSTACLE * penalty + W_STEER * steer + W_CENTRE * centre
                + W_MASK_BOUNDARY * mask_boundary)
        cost = np.where(ok, cost, np.inf)
        best = int(np.argmin(cost))
        return float(pairs[best][0]), float(pairs[best][1]), "OK"
