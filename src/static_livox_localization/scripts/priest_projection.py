"""The convex core of PRIEST: push a batch of trajectories onto the feasible set.

From Rastgar et al., "PRIEST: Projection Guided Sampling-Based Optimization
For Autonomous Navigation" (arXiv:2309.08235). This is the projection block -
the paper's one structural difference from CEM/MPPI, sitting between sampling
and cost evaluation. Sampling alone fails when every sample lands in the
infeasible region, because there is then nothing left to rank; projecting
first means the ranking always happens over trajectories that are at least
close to drivable.

Two ideas make it cheap enough to run on a batch.

First, the awkward constraints are rewritten so that they become affine once
a closed-form quantity is fixed. "Stay outside this circle" is not something
a QP can hold, but

    x - x_o = a d cos(alpha),  y - y_o = a d sin(alpha),  d >= 1

is affine in the trajectory once alpha and d are known - and for a given
trajectory the best alpha and d have a closed form (alpha is the bearing from
the obstacle, d the clipped range ratio). Velocity and acceleration limits
take the same shape with d in [0, 1]. The corridor takes the slack form of
the paper's appendix. So the problem alternates between a closed-form update
and a QP in the trajectory.

Second, that QP has only equality constraints, so its KKT system is one fixed
matrix, built once and reused across every sample and every iteration.

Every constraint block here is written to be SELF-CANCELLING when it is
already satisfied: e equals the current value, the row contributes nothing,
and the projection leaves that trajectory alone. That property is what lets
one fixed-size F cover a varying number of obstacles, and getting it wrong is
not a small error - an early version divided by an effective radius in d and
multiplied by the true radius in e, which made a zero-radius padding circle
read as "be exactly at the obstacle centre" and dragged the whole batch a
kilometre off the map.

The paper writes the right-hand side with a minus on rho F^T e. Deriving the
stationarity condition of

    L = 1/2||xi - xi_bar||^2 + rho/2||F xi - e||^2 - <lambda, xi> + <nu, A xi - b>

gives (I + rho F^T F) xi + A^T nu = xi_bar + rho F^T e + lambda, so the sign
here is positive. Only feasibility of the result settles a sign, so that is
what the tests check rather than the formula.

Two dimensions, not three: the chair is planar, and dropping z removes the
beta angles the paper needs for a quadrotor.
"""

from __future__ import annotations

import numpy as np

from priest_constraints import (
    DEFAULT_CONSTRAINT_TOLERANCES,
    PROJECTION_CONSTRAINT_TOLERANCES,
    ProjectionViolations,
    max_yaw_rate_rps,
    projected_violations,
)

# Some BLAS builds raise spurious divide/overflow flags out of matmul on
# perfectly finite operands. The values are checked for finiteness where it
# matters instead; see test_priest_projection.
np.seterr(over="ignore", divide="ignore", invalid="ignore")


def bernstein_basis(degree, times, horizon_s):
    """Bernstein basis and its first two derivatives, sampled at `times`.

    Bernstein rather than monomials because the coefficients then live on the
    same scale as the positions they describe, which matters when the sampler
    perturbs them: a Gaussian in monomial-coefficient space puts almost all
    its mass on wild trajectories.
    """
    from math import comb

    tau = np.clip(np.asarray(times, dtype=np.float64) / horizon_s, 0.0, 1.0)
    n = degree
    basis = np.zeros((len(tau), n + 1))
    first = np.zeros_like(basis)
    second = np.zeros_like(basis)
    for i in range(n + 1):
        basis[:, i] = comb(n, i) * tau ** i * (1 - tau) ** (n - i)
    lower = np.zeros((len(tau), n))
    for i in range(n):
        lower[:, i] = comb(n - 1, i) * tau ** i * (1 - tau) ** (n - 1 - i)
    for i in range(n + 1):
        left = lower[:, i - 1] if i > 0 else 0.0
        right = lower[:, i] if i < n else 0.0
        first[:, i] = n * (left - right)
    lower2 = np.zeros((len(tau), max(n - 1, 1)))
    for i in range(n - 1):
        lower2[:, i] = comb(n - 2, i) * tau ** i * (1 - tau) ** (n - 2 - i)
    for i in range(n + 1):
        left = lower2[:, i - 2] if i >= 2 else 0.0
        mid = lower2[:, i - 1] if 1 <= i <= n - 1 else 0.0
        right = lower2[:, i] if i <= n - 2 else 0.0
        second[:, i] = n * (n - 1) * (left - 2 * mid + right)
    return basis, first / horizon_s, second / (horizon_s ** 2)


class TrajectoryBasis(object):
    """Position, velocity and acceleration bases over one planning horizon."""

    def __init__(self, degree=10, steps=40, horizon_s=8.0):
        self.degree = degree
        self.steps = steps
        self.horizon_s = float(horizon_s)
        self.times = np.linspace(0.0, self.horizon_s, steps)
        self.P, self.Pdot, self.Pddot = bernstein_basis(
            degree, self.times, self.horizon_s)
        self.n_c = degree + 1

    def positions(self, xi):
        cx, cy = xi[:, :self.n_c], xi[:, self.n_c:]
        return cx @ self.P.T, cy @ self.P.T

    def derivatives(self, xi):
        cx, cy = xi[:, :self.n_c], xi[:, self.n_c:]
        return ((cx @ self.Pdot.T, cy @ self.Pdot.T),
                (cx @ self.Pddot.T, cy @ self.Pddot.T))


class Projection(object):
    """Alternating minimisation onto boundary, limit, obstacle and corridor sets.

    The corridor is the safety band, and it is not decoration. The MID360 sits
    0.725 m up with a -7 degree lower field of view, so it cannot see ground
    within about 5.9 m: kerbs and drop-offs are precisely what this sensor
    cannot detect in time. Planning start-to-goal on visible obstacles alone
    would happily cross a kerb into a road. The band is the only thing that
    knows where the ground breaks, so it bounds the search rather than
    advising it.

    `max_obstacles` fixes the row count of F, and therefore of the KKT matrix.
    Fewer are padded with inert circles; more must be dropped nearest-first by
    the caller, which is a coverage limit worth logging rather than hiding.
    """

    def __init__(self, basis, max_obstacles=24, v_max=0.6, a_max=0.5,
                 rho=1.0, yaw_rate_max=0.6):
        self.basis = basis
        self.max_obstacles = int(max_obstacles)
        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.rho = float(rho)
        self.yaw_rate_max = float(yaw_rate_max)

        obstacle_block = np.tile(basis.P, (self.max_obstacles, 1))
        axis = np.vstack([obstacle_block, basis.Pdot, basis.Pddot])
        self.F_polar = np.block([[axis, np.zeros_like(axis)],
                                 [np.zeros_like(axis), axis]])
        # Initial position, velocity and acceleration, plus final position.
        # Final velocity is deliberately free: forcing it would make every
        # horizon end in a stop, and this runs receding.
        axis_eq = np.vstack([basis.P[0], basis.Pdot[0], basis.Pddot[0],
                             basis.P[-1]])
        self.A = np.block([[axis_eq, np.zeros_like(axis_eq)],
                           [np.zeros_like(axis_eq), axis_eq]])
        self.n_v = 2 * basis.n_c
        self.n_eq = self.A.shape[0]
        self.G = np.zeros((0, self.n_v))
        self.tau = np.zeros(0)
        self._refactor()

    # ------------------------------------------------------------- corridor
    def set_corridor(self, centres, normals, left_m, right_m):
        """Bound every timestep to one lateral slice of the band.

        Each planning step t gets the band station it is expected to be near,
        and a pair of half-planes: the signed offset along that station's
        normal must lie in [-right, +left]. Affine in the coefficients, so it
        joins F as the paper's G block.

        The assignment is fixed for the whole projection - it comes from the
        caller once per planning cycle - which is what keeps the KKT matrix
        constant across the alternating iterations. Re-deciding which station
        each step belongs to inside the loop would mean refactorising on every
        iteration for a choice that barely moves.
        """
        centres = np.asarray(centres, dtype=np.float64)
        normals = np.asarray(normals, dtype=np.float64)
        left_m = np.asarray(left_m, dtype=np.float64)
        right_m = np.asarray(right_m, dtype=np.float64)
        steps = self.basis.steps
        if centres.shape != (steps, 2) or normals.shape != (steps, 2):
            raise ValueError("corridor must give one centre and normal per step")

        lateral = np.hstack([normals[:, 0:1] * self.basis.P,
                             normals[:, 1:2] * self.basis.P])
        offset = np.einsum("ij,ij->i", normals, centres)
        self.G = np.vstack([lateral, -lateral])
        self.tau = np.hstack([left_m + offset, right_m - offset])
        self._refactor()

    def clear_corridor(self):
        self.G = np.zeros((0, self.n_v))
        self.tau = np.zeros(0)
        self._refactor()

    def _refactor(self):
        self.F = np.vstack([self.F_polar, self.G]) if len(self.G) \
            else self.F_polar
        kkt = np.zeros((self.n_v + self.n_eq, self.n_v + self.n_eq))
        kkt[:self.n_v, :self.n_v] = (np.eye(self.n_v)
                                     + self.rho * self.F.T @ self.F)
        kkt[:self.n_v, self.n_v:] = self.A.T
        kkt[self.n_v:, :self.n_v] = self.A
        self.M = np.linalg.inv(kkt)

    # ------------------------------------------------------------ obstacles
    def pad_obstacles(self, obstacles):
        """(m, 3) [x, y, radius] -> exactly max_obstacles rows, padding inert."""
        padded = np.zeros((self.max_obstacles, 3))
        padded[:, 0] = 1.0e3
        padded[:, 1] = 1.0e3
        if len(obstacles):
            take = min(len(obstacles), self.max_obstacles)
            padded[:take] = np.asarray(obstacles, dtype=np.float64)[:take]
        return padded

    def polar_targets(self, xi, obstacles):
        """The closed-form half of the alternation: best angles, d, slack, e."""
        basis = self.basis
        x, y = basis.positions(xi)
        (vx, vy), (ax, ay) = basis.derivatives(xi)
        n_b = xi.shape[0]

        ox, oy, radius = obstacles[:, 0], obstacles[:, 1], obstacles[:, 2]
        dx = x[:, None, :] - ox[None, :, None]
        dy = y[:, None, :] - oy[None, :, None]
        reach = np.hypot(dx, dy)
        # The SAME effective radius divides in d and multiplies in e, so a
        # cleared obstacle reproduces the current point exactly and a padding
        # circle of radius 0 is genuinely inert.
        safe = np.where(radius[None, :, None] > 1e-9, radius[None, :, None],
                        1.0)
        d_o = np.maximum(reach / safe, 1.0)
        span = np.maximum(reach, 1e-9)
        cos_o = np.where(reach > 1e-9, dx / span, 1.0)
        sin_o = np.where(reach > 1e-9, dy / span, 0.0)
        e_ox = ox[None, :, None] + safe * d_o * cos_o
        e_oy = oy[None, :, None] + safe * d_o * sin_o

        speed = np.hypot(vx, vy)
        d_v = np.clip(speed / self.v_max, 0.0, 1.0)
        cos_v = np.where(speed > 1e-9, vx / np.maximum(speed, 1e-9), 1.0)
        sin_v = np.where(speed > 1e-9, vy / np.maximum(speed, 1e-9), 0.0)

        accel = np.hypot(ax, ay)
        d_a = np.clip(accel / self.a_max, 0.0, 1.0)
        cos_a = np.where(accel > 1e-9, ax / np.maximum(accel, 1e-9), 1.0)
        sin_a = np.where(accel > 1e-9, ay / np.maximum(accel, 1e-9), 0.0)

        e_x = np.hstack([e_ox.reshape(n_b, -1),
                         d_v * self.v_max * cos_v,
                         d_a * self.a_max * cos_a])
        e_y = np.hstack([e_oy.reshape(n_b, -1),
                         d_v * self.v_max * sin_v,
                         d_a * self.a_max * sin_a])
        e = np.hstack([e_x, e_y])
        if len(self.G):
            # Slack form from the appendix: s = max(0, tau - G xi) >= 0 gives
            # e_G = tau - s = min(G xi, tau). Inside the band the row
            # reproduces the current offset and pulls nothing; outside it
            # targets the edge exactly.
            e = np.hstack([e, np.minimum(xi @ self.G.T, self.tau)])
        return e

    # ---------------------------------------------------------------- solve
    def solve(self, samples, boundary, obstacles, iterations=12):
        """Project every sampled trajectory onto the feasible set.

        Returns (xi, residual). The residual is what Algorithm 1 ranks on
        before the cost is looked at, and what gets appended to the cost
        afterwards, so it comes back rather than being thresholded here.
        """
        samples = np.atleast_2d(np.asarray(samples, dtype=np.float64))
        padded = self.pad_obstacles(obstacles)
        b_eq = np.tile(np.asarray(boundary, dtype=np.float64),
                       (samples.shape[0], 1))
        xi = samples.copy()
        lam = np.zeros_like(xi)
        for _ in range(max(1, int(iterations))):
            e = self.polar_targets(xi, padded)
            rhs = samples + self.rho * (e @ self.F) + lam
            xi = (np.hstack([rhs, b_eq]) @ self.M.T)[:, :self.n_v]
            lam = lam - self.rho * ((xi @ self.F.T) - e) @ self.F
        return xi, self.residual(xi, padded)

    def violations(self, xi, obstacles) -> ProjectionViolations:
        """Maximum positive excess for every constraint, in native units."""
        obstacles = self.pad_obstacles(obstacles)
        x, y = self.basis.positions(xi)
        (vx, vy), (ax, ay) = self.basis.derivatives(xi)
        corridor_excess = ((xi @ self.G.T) - self.tau if len(self.G)
                           else np.zeros((len(xi), 0), dtype=np.float64))
        return projected_violations(
            x=x, y=y, vx=vx, vy=vy, ax=ax, ay=ay,
            obstacles=obstacles, corridor_excess_m=corridor_excess,
            times_s=self.basis.times, v_max=self.v_max, a_max=self.a_max,
            yaw_rate_max=self.yaw_rate_max)

    def residual(self, xi, obstacles):
        """Dimensionless worst-unit score used only for Algorithm 1 ranking."""
        return self.violations(
            xi, obstacles).score(PROJECTION_CONSTRAINT_TOLERANCES)
