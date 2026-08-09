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
MAX_SPEED = 0.6
TURN_FLOOR_SPEED = 0.30
MAX_YAW_RATE = 0.5

# How far ahead a candidate is simulated. Long enough to see a corridor bend
# arriving - 1.7 s is a metre at cruise - and short enough that a constant
# curvature arc is still a fair description of what the chair will do. Both
# ends matter: lengthen it and every candidate fails on a curve because no
# single arc stays in a bending corridor; shorten it and the chair drives
# into pinches it never looked at.
SIM_TIME_S = 1.7
SIM_STEP_S = 0.1

SPEED_SAMPLES = 5
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

# A candidate whose rollout passes closer than this to a tracked object is
# discarded outright rather than scored - the same floor mpc_core keeps.
OBSTACLE_FLOOR_M = 0.40


def speed_samples(max_speed=MAX_SPEED, floor=TURN_FLOOR_SPEED,
                  count=SPEED_SAMPLES):
    """Executable speeds only: stop, or something the wheels will turn for.

    The gap between them is the actuation deadband, measured on the loaded
    chair. Sampling inside it produces candidates that score well, get
    commanded, and do nothing.
    """
    if max_speed < floor:
        return (0.0,)
    return (0.0,) + tuple(np.linspace(floor, max_speed, max(count, 1)))


def yaw_samples(limit=MAX_YAW_RATE, count=YAW_SAMPLES):
    return tuple(np.linspace(-limit, limit, max(count, 3)))


def rollout(state, v, w, sim_time_s=SIM_TIME_S, step_s=SIM_STEP_S):
    """Where a constant (v, w) takes the chair, sampled along the way.

    Returns (n, 3) of (x, y, yaw). The intermediate points are the point -
    a candidate that ends inside the corridor having crossed out of it on
    the way is not a candidate.
    """
    x, y, yaw = float(state[0]), float(state[1]), float(state[2])
    out = []
    steps = max(int(round(sim_time_s / step_s)), 1)
    for _ in range(steps):
        x += v * math.cos(yaw) * step_s
        y += v * math.sin(yaw) * step_s
        yaw += w * step_s
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

    def __init__(self, band, route, sim_time_s=SIM_TIME_S,
                 grace=0.0, max_speed=MAX_SPEED):
        from scipy.spatial import cKDTree
        self.band = band
        self.route = np.asarray(route, dtype=float)
        self.tree = cKDTree(self.route)
        self.sim_time_s = float(sim_time_s)
        self.grace = float(grace)
        self.max_speed = float(max_speed)
        seg = np.linalg.norm(np.diff(self.route, axis=0), axis=1)
        self.arc = np.concatenate([[0.0], np.cumsum(seg)])
        self.steps = max(int(round(self.sim_time_s / SIM_STEP_S)), 1)
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
        yaw = state[2] + w * k * SIM_STEP_S
        # position by integrating the same constant-curvature arc the chair
        # would drive, step by step, so an arc that leaves the band midway
        # is caught rather than judged only on where it ends up
        dx = np.cumsum(v * np.cos(yaw) * SIM_STEP_S, axis=1)
        dy = np.cumsum(v * np.sin(yaw) * SIM_STEP_S, axis=1)
        return np.stack([state[0] + dx, state[1] + dy, yaw], axis=2)

    def plan(self, state, obstacles=(), speed_cap=None, last_yaw_rate=0.0):
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
        pairs = [(v, w) for v in speed_samples(cap) if v > 0.0
                 # Turning on the spot is not something this chair does below
                 # its rotation floor, and it is the manoeuvre that put it at
                 # a wall on 2026-08-04. Excluded.
                 for w in yaw_samples()]
        if not pairs:
            return 0.0, 0.0, "NO_CANDIDATE"
        paths = self._rollouts(np.asarray(state, dtype=float), pairs)
        flat = paths[:, :, :2].reshape(-1, 2)
        inside = self.band.contains_many(flat, grace=self.grace)
        ok = inside.reshape(len(pairs), self.steps).all(axis=1)
        reasons = {}
        if not ok.any():
            return 0.0, 0.0, "OFF_BAND"
        reasons["OFF_BAND"] = int((~ok).sum())
        if len(obstacles):
            pts = np.asarray(obstacles, dtype=float).reshape(-1, 2)
            clear = np.linalg.norm(
                paths[:, :, None, :2] - pts[None, None, :, :],
                axis=3).min(axis=(1, 2))
        else:
            clear = np.full(len(pairs), np.inf)
        ok &= clear >= OBSTACLE_FLOOR_M
        if not ok.any():
            return 0.0, 0.0, "OBSTACLE"
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
        # Where the rollout sits between the corridor's two edges: 0 in the
        # middle, 1 against either edge. Taken from the band's own geometry
        # rather than from distance to the recorded line, because the line is
        # not always the middle - where the band is asymmetric the clearance
        # that matters is the smaller side, not the deviation.
        lateral, lo, hi = self.band.margins_many(flat)
        half = np.maximum((hi - lo) / 2.0, 1e-6)
        edge = np.abs(lateral - (hi + lo) / 2.0) / half
        centre = np.square(np.minimum(edge, 1.0)).reshape(
            len(pairs), self.steps).mean(axis=1)
        cost = (W_PATH * path_cost + W_HEADING * aim - W_PROGRESS * progress
                + W_OBSTACLE * penalty + W_STEER * steer + W_CENTRE * centre)
        cost = np.where(ok, cost, np.inf)
        best = int(np.argmin(cost))
        return float(pairs[best][0]), float(pairs[best][1]), "OK"
