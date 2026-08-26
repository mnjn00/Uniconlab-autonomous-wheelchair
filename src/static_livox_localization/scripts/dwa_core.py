"""Trajectory-rollout planning with a preferred band and hard drivable mask.

The third control law, and the reason it exists is narrow: pure pursuit and
the MPC both follow a line, and neither of them avoids anything well. What
avoidance the stack has is take_a_way_round - a fixed +-0.6 m lateral offset
applied to the pursuit target - and on 2026-08-04 that offset, applied from
a standstill where the lookahead has collapsed to MIN_LOOKAHEAD_M, demanded
atan(0.6 / 0.9) = 34 degrees and steered the chair at a wall three times in
one evening. Rolling out candidate velocities and scoring them cannot
produce that: every candidate is a velocity pair the chair can actually
hold. The recorded band is strongly preferred, while the raster drivable
mask remains the physical boundary no candidate may cross.

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
# ...but never less than this many seconds of it. A fixed distance means the
# preview shrinks as the chair speeds up - 1.05 m is 3.0 s at 0.35 m/s and
# 1.3 s at 0.80 - so the planner turns myopic exactly where it can least
# afford to, and the actuation lag grows from a fifth of the preview to
# nearly half of it. On 2026-08-23 that produced a weave that grew 0.15 m,
# 0.24, 0.53 and saturated the yaw rate three times in thirteen seconds.
#
# The floor is a time, applied to the arc every candidate in a cycle walks -
# not per candidate, which would make the arc depend on the speed carrying
# it and undo what the fixed distance is for.
SIM_MIN_PREVIEW_S = 2.0

# How far out the obstacle test looks, however short the scored arc is.
#
# These are two different questions and they were being answered with one
# number. The arc is a STEERING horizon and it is deliberately short: at
# 1.7 m the admissible candidate count fell from 102 to 78 at wp 500,
# because no single constant-curvature arc stays in a bending corridor.
# The obstacle test is a VETO horizon, and it has to reach at least as far
# as the thing that can veto the chair - safety_gate, whose stopping
# envelope carries a fixed 0.9 m geometry margin and so exceeds 1.4 m even
# at the 0.35 m/s floor.
#
# On 2026-08-23 that gap deadlocked the chair for 130 s. A parked
# motorcycle stood 1.42 m ahead and 0.77 m to the right, well inside a
# corridor that was open 2.70 m to the left. The preview reached 1.05 m, so
# every candidate scored clear, the planner held straight at the floor
# speed with w within 0.05 of zero, and the gate vetoed it. The chair
# stopped, which shortened nothing, so the next cycle repeated it exactly.
# Neither side was wrong on its own terms; they were looking at different
# distances.
OBSTACLE_PREVIEW_M = 3.0

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
W_PATH = 3.3
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

# Reversing the steer is not free. With the quadratic preferred-route cost,
# 1.61 gives 9 reversals over 381.64 m; 1.62 is the first centesimal value
# below the 0.02/m limit at 7. It remains below the corner-cutting region.
W_STEER = 1.62
# Speed is rewarded here and nowhere else, which is not obvious and cost a
# drive to learn. The rollout is sampled over a fixed DISTANCE, so every
# candidate walks the same 1.05 m arc: for one yaw rate, path cost, heading,
# centring, clearance and progress all come out identical whatever speed
# carries them. Before 2026-08-23 that left the five speeds exactly tied and
# the winner decided by which one argmin happened to see first - which is
# the 1,035 target changes measured over the flat sections of that run, once
# a second, all over the route. Not near-ties. No difference at all.
#
# So the reward is explicit. Faster is better - but only just. The size of it
# is the whole question, and 2.0 was too much: across the 0.35 to 0.80 range
# it is worth 0.9, and W_PATH turns 0.3 m of lateral error into the same 0.9,
# so the planner would buy a third of a metre off the line to go half a metre
# per second faster. It did. The first run at this weight wove with a growing
# amplitude - 0.15 m, then 0.24, then 0.53 - and saturated the yaw rate at
# +/-0.50 three times in thirteen seconds before the gate stopped it.
#
# At 0.8 the reward spans 0.36 across the same range, which W_PATH matches at
# 0.11 m of error. That is the trade this is allowed to make: a hand's width,
# not half a metre.
W_VELOCITY = 0.8
# ...and a smaller penalty for changing, which is the W_STEER idea on the
# other axis. It breaks the remaining ties toward what the chair is already
# doing without ever outweighing the reward for speeding up - the ramp brakes
# at 0.60 m/s^2 and accelerates at 0.18, so a needless dip costs three times
# longer to climb out of than it took to fall into.
W_SPEED = 0.25

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
# Leaving the recorded band is a last-resort manoeuvre, not ordinary path
# tracking. The fixed surcharge makes every in-band candidate cheaper before
# distance is considered; the quadratic term then prices how far and how long
# the rollout leaves it. Both remain finite only when route_mask independently
# proves every point and crossed raster cell physically drivable.
BAND_ESCAPE_BASE_COST = 1000.0
W_BAND_OVERFLOW = 1000.0
# W_PATH is the ordinary line-following term. This second, quadratic term
# makes a large route excursion disproportionately expensive without turning
# it into another hard boundary: an obstacle can still force the choice.
W_ROUTE_DEVIATION = 25.0
# Separate from the station band: the v8 raster is the operator's
# authoritative chair-centre region. Outside it is never selectable, and
# the last 0.5 m inside it gets progressively more expensive.
W_MASK_BOUNDARY = 3.0

# A candidate whose rollout passes closer than this to a tracked object is
# discarded outright rather than scored - the same floor mpc_core keeps.
# Matched to safety_gate's own veto geometry, not chosen independently.
# The gate hard-stops for any obstacle point inside a HALF_WIDTH_M = 0.50 m
# forward corridor within the stopping envelope. At 0.40 the planner would
# happily propose a path threading a 0.45 m gap that the gate then refuses,
# and because refusing does not move the chair the next cycle proposes it
# again. That is the second entrance to the 2026-08-23 motorcycle deadlock:
# the nearest surface of it sat 0.47 m off the centreline - clear to the
# planner, a stop to the gate. A planner must not propose what the gate
# forbids; where they disagree the chair simply stands still.
OBSTACLE_FLOOR_M = 0.50


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
        # The window bounds what the ramp reaches; the floor is what the
        # wheels execute. Those are different things, and letting the window
        # push the ceiling under the floor is what stopped the chair from
        # ever pulling away on 2026-08-23: from rest the reachable ceiling is
        # 0.18 m/s, the floor is 0.35, no candidate existed, and it reported
        # SPEED_BELOW_FLOOR from a standstill forever. Aiming at the floor
        # from rest is right - the ramp takes about two seconds to arrive,
        # which is the acceleration the chair has and not a planning error.
        high = max(floor, min(high, float(current) + room_up))
        low = max(low, min(high, float(current) - room_down))
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
    """Scores rollouts against the route and its preferred safety band.

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
        tangent = np.gradient(self.route, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True),
                              1e-9)
        self.heading = np.arctan2(tangent[:, 1], tangent[:, 0])

    def arc_at(self, point):
        return float(self.arc[int(self.tree.query(np.asarray(point))[1])])

    def preview_distance(self, current_speed=None):
        """How far ahead this cycle looks, in metres.

        The distance is the floor and the time is the other one: at speed the
        arc is stretched so the preview does not collapse to a length the
        actuation lag eats. One distance for the whole cycle, from the speed
        the chair is carrying - per-candidate would make the arc depend on the
        speed judged on it, which is the thing the fixed distance exists to
        avoid.
        """
        if current_speed is None:
            return self.distance_m
        return max(self.distance_m,
                   abs(float(current_speed)) * SIM_MIN_PREVIEW_S)

    def _rollouts(self, state, pairs, distance_m=None):
        """(candidates, steps, 3) for every (v, w) at once."""
        v = np.asarray([p[0] for p in pairs], dtype=float)[:, None]
        w = np.asarray([p[1] for p in pairs], dtype=float)[:, None]
        k = np.arange(1, self.steps + 1)[None, :]
        # Every candidate walks the same distance, so each gets its own time
        # step. This is what makes the arc a candidate is judged on independent
        # of the speed it carries.
        span = self.distance_m if distance_m is None else float(distance_m)
        dt = span / (np.maximum(v, 1e-6) * self.steps)
        yaw = state[2] + w * k * dt
        # position by integrating the same constant-curvature arc the chair
        # would drive, step by step, so an arc that leaves the band midway
        # is caught rather than judged only on where it ends up
        dx = np.cumsum(v * np.cos(yaw) * dt, axis=1)
        dy = np.cumsum(v * np.sin(yaw) * dt, axis=1)
        return np.stack([state[0] + dx, state[1] + dy, yaw], axis=2)

    def _obstacle_paths(self, paths, span_m, reach_m):
        """The scored arc continued straight, out to the veto horizon.

        Straight, not curved: holding the sampled yaw rate out to 3 m would
        be 8.6 s of it at the floor speed, and a 0.5 rad/s candidate would
        corkscrew through 4 rad - a trajectory the chair never drives,
        because the planner replans ten times a second. What it does do is
        take the arc for about a preview and then straighten onto whatever
        heading that left it with, which is what this builds.

        The extension is used ONLY to see obstacles. Corridor containment
        stays on the short arc, so the candidate count is untouched.
        """
        extra = float(reach_m) - float(span_m)
        if extra <= 0.0:
            return paths
        spacing = float(span_m) / self.steps
        count = int(math.ceil(extra / spacing))
        if count <= 0:
            return paths
        tail = paths[:, -1, :]
        k = np.arange(1, count + 1)[None, :] * spacing
        yaw = tail[:, 2][:, None]
        xs = tail[:, 0][:, None] + k * np.cos(yaw)
        ys = tail[:, 1][:, None] + k * np.sin(yaw)
        grown = np.stack([xs, ys, np.repeat(yaw, count, axis=1)], axis=2)
        return np.concatenate([paths, grown], axis=1)

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
            return 0.0, 0.0, "SPEED_BELOW_FLOOR"
        span = self.preview_distance(last_speed)
        paths = self._rollouts(np.asarray(state, dtype=float), pairs, span)
        flat = paths[:, :, :2].reshape(-1, 2)
        # ONE pass over the band geometry, used twice: to reject the arcs that
        # leave the corridor, and below to score how near its edge the rest of
        # them run. contains_many recomputes margins_many internally, so
        # asking it and then asking margins_many searched 802 stations for
        # 1,785 points twice over - 24.4 ms each on the target NUC, 96 % of a
        # cycle with 100 ms to spend, for one answer computed twice.
        lateral, lo, hi = self.band.margins_many(flat)
        band_inside = self.band.contained(lateral, lo, hi, self.grace)
        if self.route_mask is None:
            # Without an independent physical map there is no authority for
            # deciding that an off-band point is safe, so retain the original
            # hard-band behavior.
            ok = band_inside.reshape(len(pairs), self.steps).all(axis=1)
        else:
            # The mask, not the preferred band, is the immutable boundary.
            ok = self.route_mask.contains_many(flat).reshape(
                len(pairs), self.steps).all(axis=1)
            ok &= self.route_mask.paths_are_contained(paths[:, :, :2])
        if not ok.any():
            return 0.0, 0.0, "OFF_BAND"
        if len(obstacles):
            pts = np.asarray(obstacles, dtype=float).reshape(-1, 2)
            watched = self._obstacle_paths(paths, span, OBSTACLE_PREVIEW_M)
            # Nearest-neighbour rather than every pair. The brute force this
            # replaces materialised candidates x steps x points distances -
            # 126 x 65 x 2000 is 16 million per cycle, 371 ms on this NUC,
            # for a control loop with 100 ms to spend. It was already 158 ms
            # before the veto horizon lengthened the rollouts; extending
            # them without changing this would have been a stall waiting
            # for a crowded frame.
            from scipy.spatial import cKDTree
            flat_watched = watched[:, :, :2].reshape(-1, 2)
            distance, _ = cKDTree(pts).query(flat_watched, workers=-1)
            clear = distance.reshape(len(pairs), -1).min(axis=1)
        else:
            clear = np.full(len(pairs), np.inf)
        ok &= clear >= OBSTACLE_FLOOR_M
        if not ok.any():
            return 0.0, 0.0, "OBSTACLE"
        d, idx = self.tree.query(flat, workers=-1)
        route_distance = d.reshape(len(pairs), self.steps)
        path_cost = route_distance.mean(axis=1)
        route_deviation = np.square(route_distance).mean(axis=1)
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
        overflow = np.maximum(lo - lateral, 0.0) + \
            np.maximum(lateral - hi, 0.0)
        escaped = (~band_inside).reshape(
            len(pairs), self.steps).any(axis=1)
        band_escape = (
            BAND_ESCAPE_BASE_COST * escaped.astype(float)
            + W_BAND_OVERFLOW * np.square(overflow).reshape(
                len(pairs), self.steps).mean(axis=1)
        )
        if self.route_mask is None:
            mask_boundary = np.zeros(len(pairs), dtype=float)
        else:
            mask_boundary = self.route_mask.boundary_cost_many(flat).reshape(
                len(pairs), self.steps).mean(axis=1)
        cost = (W_SPEED * speed_change
                - W_VELOCITY * np.asarray([p[0] for p in pairs])
                + W_PATH * path_cost + W_ROUTE_DEVIATION * route_deviation
                + W_HEADING * aim - W_PROGRESS * progress
                + W_OBSTACLE * penalty + W_STEER * steer + W_CENTRE * centre
                + band_escape + W_MASK_BOUNDARY * mask_boundary)
        cost = np.where(ok, cost, np.inf)
        best = int(np.argmin(cost))
        return float(pairs[best][0]), float(pairs[best][1]), "OK"
