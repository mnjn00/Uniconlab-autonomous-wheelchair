"""PRIEST Algorithm 1 proposal search inside an authoritative safety band.

Projection proposes polynomial paths; dense runtime certification alone may
make one usable. The recorded centreline supplies stationing, not a path.
"""

import numpy as np

from priest_constraints import (
    CANONICAL_FOOTPRINT,
    DEFAULT_CONSTRAINT_TOLERANCES,
    TURN_FLOOR_SPEED_MPS,
    ConstraintTolerances,
    ConstraintViolations,
    within_goal_tolerance,
)
from priest_feasibility import (
    certify_coefficients,
    certify_trajectory,
    lowest_certified_index,
    require_physical_limits,
    validated_certificate_settings,
)
from priest_projection import Projection, TrajectoryBasis
from priest_sampling import (
    NoFiniteCandidateError,
    select_priest_elite,
    trajectory_costs,
)
from priest_terminal import STOPPED_SPEED_MPS, terminal_correction
from priest_types import Corridor, Plan


class PriestPlanner(object):
    """Projection-guided sampling and certified elite selection."""

    CONSTRAINT_TOLERANCES = DEFAULT_CONSTRAINT_TOLERANCES
    RETRIES = 4
    REACH_BACKOFF = 0.65
    MIN_REACH_M = 1.5
    GOAL_TOLERANCE_M = 0.05
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

    def __init__(self, degree=10, steps=40, v_max=0.6, a_max=0.18,
                 yaw_rate_max=0.5,
                 max_obstacles=24, batch=200, elite=20, constraint_elite=60,
                 iterations=12, projection_iterations=15, rho=1.0,
                 learning_rate=0.6, temperature=0.5, margin=1.6, seed=0,
                 runtime_band=None, control_hz=10.0, band_grace_m=0.10,
                 turn_floor_speed_mps=TURN_FLOOR_SPEED_MPS):
        self.degree = degree
        self.steps = steps
        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.yaw_rate_max = float(yaw_rate_max)
        self.turn_floor_speed_mps = float(turn_floor_speed_mps)
        require_physical_limits(self)
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
        self.runtime_band = runtime_band
        self.control_hz, self.band_grace_m = validated_certificate_settings(
            control_hz, band_grace_m)
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
                basis, slots, self.v_max, self.a_max, self.rho,
                self.yaw_rate_max))
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

    def costs(self, basis, xi, local_goal, local_tangent=None):
        return trajectory_costs(basis, xi, local_goal, local_tangent)

    def plan(self, start, velocity, acceleration, corridor, obstacles,
             goal_arc=None, initial_yaw_rad=None):
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
        goal_arc = float(np.clip(goal_arc, 0.0, corridor.length_m))
        goal_xy = np.array([
            np.interp(goal_arc, corridor.arc, corridor.centres[:, 0]),
            np.interp(goal_arc, corridor.arc, corridor.centres[:, 1])])
        goal_distance = float(np.linalg.norm(goal_xy - start))
        speed = float(np.linalg.norm(np.asarray(velocity, dtype=np.float64)))
        if within_goal_tolerance(goal_distance, self.GOAL_TOLERANCE_M) \
                and speed <= STOPPED_SPEED_MPS:
            return Plan(None, None, None, None, 0.0, 0.0, 0, 0.0,
                        reason="AT_GOAL")
        if self.runtime_band is None:
            return Plan(None, None, None, None, float("inf"), float("inf"),
                        0, 0.0, reason="RUNTIME_BAND_UNBOUND")
        remaining = max(goal_arc - start_arc, 0.0)
        correction = terminal_correction(
            self, start_xy=start, velocity_xy_mps=velocity,
            goal_xy=goal_xy, initial_yaw_rad=initial_yaw_rad,
            band=self.runtime_band,
            obstacles=np.asarray(obstacles, dtype=np.float64))
        if correction is not None:
            return correction
        if remaining < 1e-3:
            correction_span = max(self.MIN_REACH_M, goal_distance)
            start_arc = max(0.0, goal_arc - correction_span)
            remaining = max(goal_arc - start_arc, goal_distance, 1e-3)

        attempt = min(remaining, self.v_max * self.horizon_for(remaining)
                      / self.margin)
        best_plan = None
        for _ in range(self.RETRIES):
            plan = self.attempt(start, velocity, acceleration, corridor,
                                obstacles, start_arc, attempt,
                                initial_yaw_rad)
            if best_plan is None or plan.residual < best_plan.residual:
                best_plan = plan
            if plan.usable:
                return plan
            attempt *= self.REACH_BACKOFF
            if attempt < self.MIN_REACH_M:
                break
        return best_plan

    def attempt(self, start, velocity, acceleration, corridor, obstacles,
                start_arc, reach, initial_yaw_rad=None):
        """One sample-project-rank-refit run over a fixed slice of corridor."""
        if self.runtime_band is None:
            return Plan(None, None, None, None, float("inf"), float("inf"),
                        0, 0.0, reason="RUNTIME_BAND_UNBOUND")
        raw_obstacles = np.asarray(obstacles, dtype=np.float64)
        if raw_obstacles.size == 0:
            raw_obstacles = np.empty((0, 3), dtype=np.float64)
        obstacles = self.inflate(raw_obstacles)
        horizon_s = self.horizon_for(reach)
        basis, projection = self.basis_for(horizon_s, len(obstacles))
        centres, normals, left, right = corridor.slice(
            start_arc, start_arc + reach, self.steps)
        projection.set_corridor(centres, normals, left, right)
        local_goal = centres[-1]
        local_tangent = np.array([normals[-1, 1], -normals[-1, 0]])

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
            try:
                selection = select_priest_elite(
                    primary_cost=self.costs(
                        basis, xi, local_goal, local_tangent),
                    residual_score=residual,
                    nproj=self.constraint_elite,
                    nelite=self.elite)
            except NoFiniteCandidateError:
                break
            elite_xi = xi[selection.elite_indices]
            elite_cost = selection.elite_augmented_cost
            elite_residual = residual[selection.elite_indices]
            dense_candidates = []
            for candidate in elite_xi:
                dense = certify_coefficients(
                    self, coefficients=candidate, degree=self.degree,
                    horizon_s=horizon_s, control_hz=self.control_hz,
                    band=self.runtime_band, obstacles=raw_obstacles,
                    band_grace_m=self.band_grace_m,
                    initial_yaw_rad=initial_yaw_rad)
                dense_candidates.append(dense)
                if dense.certificate.usable:
                    break
            chosen = lowest_certified_index(
                elite_cost[:len(dense_candidates)],
                [dense.certificate for dense in dense_candidates])
            if chosen is not None:
                top = (elite_xi[chosen], elite_cost[chosen],
                       elite_residual[chosen], dense_candidates[chosen])
                if best is None or top[1] < best[1]:
                    best = top

            weights = np.exp(-(elite_cost - elite_cost.min())
                             / max(self.temperature, 1e-6))
            weights = weights / max(weights.sum(), 1e-9)
            blend = weights @ elite_xi
            mean = (1 - self.learning_rate) * mean + self.learning_rate * blend
            spread = elite_xi - blend
            cov = ((1 - self.learning_rate) * cov + self.learning_rate
                   * (spread.T * weights) @ spread + 1e-6 * np.eye(len(mean)))

        if best is None:
            return Plan(
                None, None, None, None, float("inf"), float("inf"), 0,
                horizon_s, reason="NO_FEASIBLE_TRAJECTORY")

        dense = best[3]
        return Plan(
            best[0], dense.points[:, 0], dense.points[:, 1], dense.times_s,
            best[2], best[1], 1, horizon_s,
            certificate=dense.certificate,
            velocity_xy_mps=dense.velocity_xy_mps,
            acceleration_xy_mps2=dense.acceleration_xy_mps2,
            yaw_rad=dense.yaw_rad, yaw_rate_rps=dense.yaw_rate_rps)
