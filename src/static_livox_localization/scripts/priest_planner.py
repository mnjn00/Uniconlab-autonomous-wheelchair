"""Algorithm 1 of PRIEST: sample, project, rank, refit - inside the band.

The chair is given a start and a goal and nothing else to follow. What it is
given instead of a line is a region: the safety band, which says where the
ground does not break. PRIEST searches inside that region.

That distinction is the whole point and worth being precise about, because
the band is derived from a recorded route and it would be easy to claim more
novelty than there is. The band supplies two things: the lateral limits at
each station (a hard constraint - see priest_projection.Projection) and the
arc length that tells the planner which way the goal lies. It does not supply
the path. Where the chair actually goes between those limits, which side of a
parked car it passes, and how it trades smoothness against progress are all
decided here, per cycle, from live obstacles. Pure pursuit had none of those
choices: it tracked one fixed line and could only stop when that line was
blocked.

Horizon sizing is not a tuning knob. A horizon shorter than
path_length / v_max is infeasible by construction, and the optimiser cannot
report that as anything but a stubborn residual - which reads exactly like a
blocked corridor. Every early failure while building this was that, so the
horizon is computed from the reach the chair actually needs and the class
refuses to pretend otherwise.
"""

import numpy as np

from priest_constraints import (
    CANONICAL_FOOTPRINT,
    DEFAULT_CONSTRAINT_TOLERANCES,
    ConstraintTolerances,
    ConstraintViolations,
)
from priest_feasibility import certify_trajectory
from priest_projection import Projection, TrajectoryBasis
from priest_types import Corridor, Plan


class PriestPlanner(object):
    """Projection-guided sampling over polynomial coefficients.

    Perturbing coefficients rather than control inputs is the paper's stated
    exploration advantage: a Gaussian on Bernstein coefficients moves whole
    trajectory shapes, so one sample can sit in a different homotopy - the
    other side of an obstacle - instead of being a slightly noisier version
    of the same line.
    """

    FEASIBLE_M = 0.05
    CONSTRAINT_TOLERANCES = DEFAULT_CONSTRAINT_TOLERANCES
    RETRIES = 4
    REACH_BACKOFF = 0.65
    MIN_REACH_M = 1.5
    # The projection constrains a POINT to stay outside each circle, so the
    # circles have to be grown by everything the point is standing in for.
    # Without this the planner returns trajectories whose clearance to an
    # obstacle surface is a few centimetres - correct for a point, and a
    # collision for a 0.70 m wide chair.
    CHAIR_HALF_WIDTH_M = 0.35
    CHAIR_RADIUS_M = CANONICAL_FOOTPRINT.circumscribed_radius_m
    OBSTACLE_MARGIN_M = CANONICAL_FOOTPRINT.planning_margin_m
    # Horizons are quantised so the basis and its KKT inverse are actually
    # reused. Recomputed every cycle as `remaining` shrinks, that inverse
    # was most of the planning time.
    HORIZON_QUANTUM_S = 2.0

    def __init__(self, degree=10, steps=40, v_max=0.6, a_max=0.5,
                 yaw_rate_max=0.6,
                 max_obstacles=24, batch=200, elite=20, constraint_elite=60,
                 iterations=12, projection_iterations=15, rho=1.0,
                 learning_rate=0.6, temperature=0.5, margin=1.6, seed=0):
        self.degree = degree
        self.steps = steps
        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.yaw_rate_max = float(yaw_rate_max)
        self.max_obstacles = int(max_obstacles)
        self.batch = int(batch)
        self.elite = int(elite)
        self.constraint_elite = int(constraint_elite)
        self.iterations = int(iterations)
        self.projection_iterations = int(projection_iterations)
        self.rho = float(rho)
        # One number, used to size the horizon AND to cut the reach back out
        # of it. Split across two places it silently stops being a margin:
        # once the horizon saturates its ceiling, only the reach divisor is
        # still doing anything, which is how 1.35 looked like a 35 percent
        # allowance while actually leaving none.
        #
        # 1.6 rather than the smallest value that works. Measured over four
        # scenes and twelve seeds, 1.35 lands between 0.01 and 0.05 m of
        # residual - under the feasibility threshold only because the reach
        # backoff rescues it - while 1.6 returns exactly zero everywhere. A
        # margin that needs the retry to pass is not a margin.
        self.margin = float(margin)
        self.learning_rate = float(learning_rate)
        self.temperature = float(temperature)
        self.rng = np.random.default_rng(seed)
        self._basis = {}

    def basis_for(self, horizon_s, n_obstacles):
        """Basis and projection for this horizon and obstacle count.

        F's row count scales with max_obstacles, and so does every iteration
        of the projection: 24 slots cost 31 ms a solve where 6 cost 9. The
        cap exists so one F can cover a varying scene, but sizing it to the
        obstacles actually present costs nothing, because set_corridor
        already refactorises the KKT matrix every cycle.
        """
        slots = int(np.clip(max(n_obstacles, 1), 1, self.max_obstacles))
        key = (round(horizon_s, 2), slots)
        if key not in self._basis:
            basis = TrajectoryBasis(self.degree, self.steps, key[0])
            self._basis[key] = (basis, Projection(
                basis, slots, self.v_max, self.a_max, self.rho))
        return self._basis[key]

    def inflate(self, obstacles):
        """Grow each obstacle by the chair and a margin.

        Done here rather than in Projection because it is a property of the
        vehicle, not of the optimiser, and because the residual reported to
        the caller should be the one the chair actually has to satisfy.
        """
        if not len(obstacles):
            return obstacles
        grown = np.array(obstacles, dtype=np.float64).copy()
        grown[:, 2] += self.CHAIR_RADIUS_M + self.OBSTACLE_MARGIN_M
        return grown

    certify = certify_trajectory

    def horizon_for(self, reach_m, floor_s=4.0, ceiling_s=40.0):
        """Seconds needed to cover `reach_m`, with room to go round things.

        The margin is not padding for its own sake. A trajectory that has to
        leave the centreline to pass an obstacle is longer than the corridor
        it follows, and a horizon sized for the corridor exactly makes every
        avoidance manoeuvre infeasible - which the optimiser can only report
        as a residual that looks like a blocked road.
        """
        raw = self.margin * reach_m / max(self.v_max, 1e-3)
        quantum = self.HORIZON_QUANTUM_S
        return float(np.clip(np.ceil(raw / quantum) * quantum,
                             floor_s, ceiling_s))

    def sample_seed(self, basis, start, local_goal):
        """Coefficients of the straight line from start to the local goal."""
        line = np.linspace(np.asarray(start, dtype=np.float64),
                           np.asarray(local_goal, dtype=np.float64),
                           basis.n_c)
        return np.hstack([line[:, 0], line[:, 1]])

    def costs(self, basis, xi, local_goal):
        """c1: smoothness and progress. Non-smooth terms are allowed here.

        No centreline-following term. The band already says where the chair
        may be; adding a pull toward its middle would re-impose the recorded
        line as a preference and quietly turn this back into route tracking
        with extra steps.
        """
        x, y = basis.positions(xi)
        (vx, vy), (ax, ay) = basis.derivatives(xi)
        smooth = (ax ** 2 + ay ** 2).mean(axis=1)
        # Arrive pointing along the way you are going, not sideways into the
        # corridor wall.
        heading = np.hypot(vx[:, -1], vy[:, -1])
        reach = np.hypot(x[:, -1] - local_goal[0], y[:, -1] - local_goal[1])
        return smooth + 4.0 * reach - 0.5 * heading

    def plan(self, start, velocity, acceleration, corridor, obstacles,
             goal_arc=None):
        """One planning cycle. Returns a Plan, usable or with a refusal.

        A cycle that cannot find a feasible trajectory takes a smaller bite
        and tries again. That is the physically meaningful response and not
        a retry loop papering over a tuning constant: the residual says the
        chair cannot get that far through that corridor in that horizon, and
        the honest answer to it is to plan less far, not to relax a limit.
        Only after the reach has been cut down to nothing does the planner
        report that the corridor is blocked, which is then a claim about the
        world rather than about the budget.
        """
        start = np.asarray(start, dtype=np.float64)
        start_arc = corridor.arc_of(start)
        goal_arc = corridor.length_m if goal_arc is None else float(goal_arc)
        remaining = max(goal_arc - start_arc, 0.0)
        if remaining < 1e-3:
            return Plan(None, None, None, None, 0.0, 0.0, 0, 0.0,
                        reason="AT_GOAL")

        attempt = min(remaining, self.v_max * self.horizon_for(remaining)
                      / self.margin)
        best_plan = None
        for _ in range(self.RETRIES):
            plan = self.attempt(start, velocity, acceleration, corridor,
                                obstacles, start_arc, attempt)
            if best_plan is None or plan.residual < best_plan.residual:
                best_plan = plan
            if plan.usable:
                return plan
            attempt *= self.REACH_BACKOFF
            if attempt < self.MIN_REACH_M:
                break
        return best_plan

    def attempt(self, start, velocity, acceleration, corridor, obstacles,
                start_arc, reach):
        """One sample-project-rank-refit run over a fixed slice of corridor."""
        obstacles = self.inflate(obstacles)
        horizon_s = self.horizon_for(reach)
        basis, projection = self.basis_for(horizon_s, len(obstacles))
        centres, normals, left, right = corridor.slice(
            start_arc, start_arc + reach, self.steps)
        projection.set_corridor(centres, normals, left, right)
        local_goal = centres[-1]

        boundary = [start[0], velocity[0], acceleration[0], local_goal[0],
                    start[1], velocity[1], acceleration[1], local_goal[1]]
        mean = self.sample_seed(basis, start, local_goal)
        cov = np.eye(len(mean)) * 0.6 ** 2

        best = None
        for _ in range(self.iterations):
            samples = self.rng.multivariate_normal(mean, cov, size=self.batch)
            samples[0] = mean                      # keep the incumbent
            xi, residual = projection.solve(
                samples, boundary, obstacles, self.projection_iterations)
            keep = np.argsort(residual)[:self.constraint_elite]
            augmented = (self.costs(basis, xi[keep], local_goal)
                         + 40.0 * residual[keep])
            order = np.argsort(augmented)[:self.elite]
            elite_xi = xi[keep][order]
            elite_cost = augmented[order]

            # Ranked by residual, not by cost. A cheaper trajectory that is
            # further from feasible is not the one to carry forward - the
            # cost is only meaningful among trajectories that can be driven.
            elite_residual = residual[keep][order]
            leader = int(np.argmin(elite_residual))
            top = elite_xi[leader], elite_cost[leader], elite_residual[leader]
            if best is None or top[2] < best[2]:
                best = top

            weights = np.exp(-(elite_cost - elite_cost.min())
                             / max(self.temperature, 1e-6))
            weights = weights / max(weights.sum(), 1e-9)
            blend = weights @ elite_xi
            mean = (1 - self.learning_rate) * mean + self.learning_rate * blend
            spread = elite_xi - blend
            cov = ((1 - self.learning_rate) * cov + self.learning_rate
                   * (spread.T * weights) @ spread + 1e-6 * np.eye(len(mean)))

        x, y = basis.positions(best[0][None, :])
        reason = "" if best[2] <= self.FEASIBLE_M else "NO_FEASIBLE_TRAJECTORY"
        return Plan(best[0], x[0], y[0], basis.times, best[2], best[1],
                    int(best[2] <= self.FEASIBLE_M), horizon_s, reason)
