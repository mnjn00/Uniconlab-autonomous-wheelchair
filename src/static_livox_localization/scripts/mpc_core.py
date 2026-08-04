"""Receding-horizon QP-MPC core for the wheelchair follower, without ROS.

Implements docs/mpc_follower_design.md: an MPCC-style contouring objective
(alexliniger/MPCC) adapted to a slow unicycle, the safety band as HARD
lateral half-plane constraints, obstacles as soft half-planes, and OSQP as
the solver. The per-cycle problem size is constant in route length - the
specific shape the PRIEST failure (revert 81fed5d) proved we cannot afford
to get wrong.

Everything here is offline and testable: the ROS node (future
mpc_follower.py) wraps this module the same way waypoint_follower.py wraps
safety_band.py.
"""

import math
import time
from dataclasses import dataclass

import numpy as np

# state: X, Y, theta, v, w ; input: a, alpha
NX = 5
NU = 2

STATUS_OK = "OK"
STATUS_REUSED = "REUSED"
STATUS_BUDGET_STOP = "BUDGET_STOP"
STATUS_INFEASIBLE_STOP = "INFEASIBLE_STOP"
STATUS_BLOCKED_STOP = "BLOCKED_STOP"


@dataclass
class MpcParams:
    """Defaults are the follower's existing constants, not new tuning."""
    dt: float = 0.1            # CONTROL_HZ = 10
    horizon: int = 25          # 2.5 s, ~1.5 m at v_max
    v_max: float = 0.6         # MAX_SPEED
    a_min: float = -0.6        # -MAX_DECEL
    a_max: float = 0.18        # MAX_ACCEL
    w_max: float = 0.5         # MAX_YAW_RATE
    al_max: float = 1.5        # follower yaw slew
    w_lat: float = 8.0
    w_head: float = 1.0
    w_vel: float = 2.0
    w_rate: float = 0.4
    w_slack: float = 400.0
    obstacle_padding: float = 0.45   # CHAIR_HALF_WIDTH 0.35 + 0.10 margin
    # The plane guarantees clearance along its normal, but on a curving
    # route the closest approach to a point obstacle can sit a few cm
    # inside the padding circle, so the hard distance floor sits just
    # under it - still 5 cm outside the chair's own half-width.
    obstacle_floor_m: float = 0.40
    band_inset: float = 0.08         # linearisation-error reserve, never lent out
    obstacle_plan_m: float = 10.0    # start bending round this far ahead
    obstacle_ramp_done_m: float = 2.0  # full clearance demanded this far before
    obstacle_pass_m: float = 1.5     # ...and hold the plane until this far past
    slack_stop: float = 0.20         # sustained slack beyond this = impassable
    slack_stop_cycles: int = 10      # ...held for this many consecutive cycles
    slack_progress_m: float = 0.20   # ...unless that much arc was covered
    solve_budget_s: float = 0.040
    max_reuse: int = 3
    osqp_eps: float = 1e-3
    osqp_max_iter: int = 8000


def unicycle_step(x, u, dt):
    """Exact per-step integration: midpoint heading, no small-angle shortcut."""
    X, Y, th, v, w = x
    a, al = u
    th_mid = th + 0.5 * w * dt
    v_mid = v + 0.5 * a * dt
    return np.array([
        X + v_mid * math.cos(th_mid) * dt,
        Y + v_mid * math.sin(th_mid) * dt,
        th + w * dt,
        v + a * dt,
        w + al * dt,
    ])


def rollout(x0, U, dt):
    """State trajectory (N+1, NX) for input sequence U (N, NU)."""
    xs = np.empty((len(U) + 1, NX))
    xs[0] = x0
    for k in range(len(U)):
        xs[k + 1] = unicycle_step(xs[k], U[k], dt)
    return xs


def linearize(xbar, dt):
    """Discretised Jacobian of the unicycle about state xbar.

    A = I + dt * df/dx. With |w| <= 0.5 rad/s and dt = 0.1 the heading
    advance per step is <= 0.05 rad, which keeps this honest. The inputs
    enter linearly already, so B is constant.
    """
    th, v = float(xbar[2]), float(xbar[3])
    cth, sth = math.cos(th), math.sin(th)
    A = np.eye(NX)
    A[0, 2] = -v * sth * dt
    A[0, 3] = cth * dt
    A[1, 2] = v * cth * dt
    A[1, 3] = sth * dt
    A[2, 4] = dt
    B = np.zeros((NX, NU))
    B[3, 0] = dt
    B[4, 1] = dt
    return A, B


class Reference:
    """Band-centred geometry shared by the cost and the constraints, so the
    two can never disagree about what 'lateral' means."""

    def __init__(self, band):
        self.band = band
        seg = np.linalg.norm(np.diff(band.xy, axis=0), axis=1)
        self.arc = np.concatenate([[0.0], np.cumsum(seg)])

    def arc_at(self, point):
        """Arc length along the band of the station nearest to a point."""
        k = int(np.argmin(np.linalg.norm(self.band.xy - point, axis=1)))
        return float(self.arc[k])

    def frame_at(self, point):
        """(normal, heading, lateral, lo, hi) at a map-frame point, using
        exactly safety_band's more-restrictive-of-two-stations rule."""
        lateral, lo, hi = self.band.lateral_limits(point)
        k = int(np.argmin(np.linalg.norm(self.band.xy - point, axis=1)))
        normal = self.band.normals[k]
        heading = math.atan2(normal[1], normal[0]) - math.pi / 2.0
        return normal, heading, lateral, lo, hi


class Obstacle:
    """Soft half-plane n . p >= offset (slackened in the QP)."""

    def __init__(self, xy, normal, offset):
        self.xy = np.asarray(xy, dtype=float)
        self.normal = np.asarray(normal, dtype=float)
        self.offset = float(offset)


def polyline_refs(band, xy, horizon, dt, v_target):
    """Velocity and heading references for the horizon, walked along the
    band station polyline at v_target from the nearest station."""
    xy_s = band.xy
    k0 = int(np.argmin(np.linalg.norm(xy_s - xy, axis=1)))
    tail = xy_s[k0:]
    if len(tail) < 2:
        return (np.full(horizon, v_target),
                np.full(horizon, math.atan2(0.0, 1.0)))
    seg = np.linalg.norm(np.diff(tail, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    heading = np.arctan2(np.diff(tail[:, 1]), np.diff(tail[:, 0]))
    v_ref = np.full(horizon, v_target)
    th_ref = np.empty(horizon)
    for k in range(horizon):
        s = v_target * (k + 1) * dt
        j = int(np.searchsorted(arc, s)) - 1
        j = min(max(j, 0), len(heading) - 1)
        th_ref[k] = heading[j]
    return v_ref, th_ref


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def obstacle_half_plane(ref, obstacle_xy, padding):
    """Cut plane that keeps the chair on the roomier side of the obstacle.

    The side is chosen from the band's own limits at the obstacle, so an
    obstacle can never authorise ground the band says breaks: the plane
    only pushes toward whichever side already has more measured room.
    """
    _normal, _heading, lateral, lo, hi = ref.frame_at(obstacle_xy)
    k = int(np.argmin(np.linalg.norm(ref.band.xy - obstacle_xy, axis=1)))
    normal = ref.band.normals[k]
    room_left = hi - lateral
    room_right = lateral - lo
    n = normal if room_left >= room_right else -normal
    return Obstacle(obstacle_xy, n, float(n @ obstacle_xy) + padding)


class MpcSolver:
    """One condensed QP per cycle, warm-started, with a timed ladder.

    Decision vector: [z_1..z_N (NX), mu_0..mu_{N-1} (NU), slack_1..slack_N]
    with z the deviation from the linearisation trajectory xbar and mu the
    deviation from the linearisation input ubar. z_0 = 0 because xbar is
    re-anchored on the measured state every cycle.
    """

    def __init__(self, ref, params=None):
        import osqp  # deferred: geometry tests must run without the solver
        import scipy.sparse as sp
        self.osqp = osqp
        self.sp = sp
        self.ref = ref
        self.p = params or MpcParams()
        N = self.p.horizon
        self.n_state = N * NX
        self.n_input = N * NU
        self.n_slack = N
        self.n_dec = self.n_state + self.n_input + self.n_slack
        self._model = None
        self._prev_x = None
        self._prev_y = None
        self.reuse_streak = 0
        self.slack_streak = 0
        self.block_anchor_arc = None

    # ------------------------------------------------ index helpers
    def _z(self, k):      # state block of z_{k+1}, k = 0..N-1
        return k * NX

    def _u(self, k):
        return self.n_state + k * NU

    def _s(self, k):
        return self.n_state + self.n_input + k

    # ------------------------------------------------ assembly
    def _assemble(self, xbar, ubar, v_ref, th_ref, obstacles):
        p, N = self.p, self.p.horizon
        n_dec = self.n_dec
        P_rows, P_cols, P_vals = [], [], []
        q = np.zeros(n_dec)
        A_rows, A_cols, A_vals = [], [], []
        l_list, u_list = [], []
        n_rows = 0

        def add_row(coeffs, lo, hi):
            nonlocal n_rows
            for col, val in coeffs:
                A_rows.append(n_rows)
                A_cols.append(col)
                A_vals.append(val)
            l_list.append(lo)
            u_list.append(hi)
            n_rows += 1

        # ---- dynamics: z_{k+1} = A_k z_k + B mu_k + c_k
        # Every A/B entry is emitted even when zero: the sparsity pattern
        # must not depend on the linearisation point, otherwise the solver
        # problem changes shape between cycles.
        for k in range(N):
            A, B = linearize(xbar[k], p.dt)
            c = unicycle_step(xbar[k], ubar[k], p.dt) - xbar[k + 1]
            for i in range(NX):
                coeffs = [(self._z(k) + i, 1.0)]
                if k > 0:
                    for j in range(NX):
                        coeffs.append((self._z(k - 1) + j, -A[i, j]))
                for j in range(NU):
                    coeffs.append((self._u(k) + j, -B[i, j]))
                add_row(coeffs, c[i], c[i])

        # ---- band half-planes (hard, never slacked)
        # lateral is station-relative: lo <= n . (p - s_k) <= hi, so in
        # deviations: sign * n . dz <= sign * (bound - lat_of_iterate).
        # The limits are inset by band_inset: the constraint is linearised
        # around the iterate and the normals are frozen per step, so the
        # applied trajectory needs a small reserve it can borrow without
        # borrowing from the drop margin itself.
        for k in range(N):
            pbar = xbar[k + 1, 0:2]
            normal, _h, lat_iter, lo_lim, hi_lim = self.ref.frame_at(pbar)
            # the linearisation reserve may not eat more than a quarter of
            # the remaining corridor, or a narrow choke becomes infeasible
            inset = min(p.band_inset, (hi_lim - lo_lim) * 0.25)
            hi_lim -= inset
            lo_lim += inset
            for sign, bound in ((1.0, hi_lim), (-1.0, lo_lim)):
                add_row([(self._z(k) + 0, sign * normal[0]),
                         (self._z(k) + 1, sign * normal[1])],
                        -np.inf, sign * bound - sign * lat_iter)

        # ---- obstacle half-planes (soft): n . p + slack >= required, so a
        # positive slack RELAXES the row; the band rows have no slack at
        # all. The plane is infinite, so both its reach and its demanded
        # clearance are scheduled by arc distance: the row activates
        # obstacle_plan_m before the obstacle, the required clearance ramps
        # linearly from 0 there to the full padding obstacle_ramp_done_m
        # BEFORE the obstacle, then holds until obstacle_pass_m past it.
        # Two lessons shaped this: full clearance over the whole approach
        # deadlocked (the planner would rather stop than hold a half-metre
        # lean for 10 m), and a ramp that only completes AT the obstacle
        # left the chair chasing its own requirement - it converges
        # asymptotically and arrives centimetres short. Completing the ramp
        # early gives the bend somewhere to finish. Far steps keep a
        # trivially-satisfied row so the matrix shape never changes.
        for ob in obstacles:
            ob.arc = self.ref.arc_at(ob.xy)
        ramp_span = max(p.obstacle_plan_m - p.obstacle_ramp_done_m, 1e-3)
        for k in range(N):
            pbar = xbar[k + 1, 0:2]
            s_k = self.ref.arc_at(pbar)
            for ob in obstacles:
                if s_k < ob.arc - p.obstacle_plan_m or s_k > ob.arc + p.obstacle_pass_m:
                    add_row([(self._s(k), 1.0)], -np.inf, np.inf)
                    continue
                ramp = min(max((s_k - (ob.arc - p.obstacle_plan_m))
                               / ramp_span, 0.0), 1.0)
                required = float(ob.normal @ ob.xy) + p.obstacle_padding * ramp
                base = float(ob.normal @ pbar)
                add_row([(self._z(k) + 0, ob.normal[0]),
                         (self._z(k) + 1, ob.normal[1]),
                         (self._s(k), 1.0)],
                        required - base, np.inf)

        # ---- input bounds, v in [0, v_max], w in [-w_max, w_max],
        # slack >= 0. Both caps are HARD, not cost terms: an inaccurate
        # solve must never accumulate velocity or yaw rate past the
        # follower's limits. The yaw-rate bound lives on the STATE, not the
        # input - bounding the yaw acceleration al with w_max (a bug this
        # replaced) clamped the wrong variable and left w itself unbounded,
        # which let a 90-degree heading error command 0.86 rad/s, twice the
        # cap; the safety gate's clamp would then have executed a trajectory
        # the plan never validated.
        for k in range(N):
            add_row([(self._u(k) + 0, 1.0)],
                    p.a_min - ubar[k, 0], p.a_max - ubar[k, 0])
            add_row([(self._u(k) + 1, 1.0)],
                    -p.al_max - ubar[k, 1], p.al_max - ubar[k, 1])
            add_row([(self._z(k) + 3, 1.0)], -xbar[k + 1, 3], np.inf)
            add_row([(self._z(k) + 3, 1.0)],
                    -np.inf, p.v_max - xbar[k + 1, 3])
            add_row([(self._z(k) + 4, 1.0)],
                    -p.w_max - xbar[k + 1, 4], p.w_max - xbar[k + 1, 4])
            add_row([(self._s(k), 1.0)], 0.0, np.inf)

        # ---- cost: 0.5 xi' P xi + q' xi  (P built upper-triangular, doubled)
        for k in range(N):
            zb = self._z(k)
            pbar = xbar[k + 1, 0:2]
            normal, heading_ref, c0, _lo, _hi = self.ref.frame_at(pbar)
            c0 = float(c0)                                 # lateral of iterate
            P_rows += [zb + 0, zb + 1, zb + 0]
            P_cols += [zb + 0, zb + 1, zb + 1]
            P_vals += [2.0 * p.w_lat * normal[0] ** 2,
                       2.0 * p.w_lat * normal[1] ** 2,
                       2.0 * p.w_lat * normal[0] * normal[1]]
            q[zb + 0] += 2.0 * p.w_lat * c0 * normal[0]
            q[zb + 1] += 2.0 * p.w_lat * c0 * normal[1]

            dth = wrap_angle(xbar[k + 1, 2] - th_ref[k])
            P_rows.append(zb + 2)
            P_cols.append(zb + 2)
            P_vals.append(2.0 * p.w_head)
            q[zb + 2] += 2.0 * p.w_head * dth

            dv = xbar[k + 1, 3] - v_ref[k]
            P_rows.append(zb + 3)
            P_cols.append(zb + 3)
            P_vals.append(2.0 * p.w_vel)
            q[zb + 3] += 2.0 * p.w_vel * dv

            ub = self._u(k)
            for j in range(NU):
                P_rows.append(ub + j)
                P_cols.append(ub + j)
                P_vals.append(2.0 * p.w_rate)
                q[ub + j] += 2.0 * p.w_rate * ubar[k, j]

            P_rows.append(self._s(k))
            P_cols.append(self._s(k))
            P_vals.append(2.0 * p.w_slack)

        P = self.sp.csc_matrix(
            (np.array(P_vals), (P_rows, P_cols)), shape=(n_dec, n_dec))
        P = (P + P.T) * 0.5
        A = self.sp.csc_matrix(
            (np.array(A_vals), (A_rows, A_cols)), shape=(n_rows, n_dec))
        return P, q, A, np.array(l_list), np.array(u_list)

    # ------------------------------------------------ cycle
    def initial_guess(self, x0, v_target):
        N = self.p.horizon
        U = np.zeros((N, NU))
        xbar = np.empty((N + 1, NX))
        xbar[0] = x0
        for k in range(N):
            U[k, 0] = float(np.clip(v_target - xbar[k, 3],
                                    self.p.a_min, self.p.a_max))
            xbar[k + 1] = unicycle_step(xbar[k], U[k], self.p.dt)
        return xbar, U

    def solve_cycle(self, x0, v_ref, th_ref, obstacles=(), warm=None):
        """One control cycle. Returns (u0, status, info).

        Ladder, nothing silent: OK -> apply; BUDGET -> reuse the previous
        first input up to max_reuse cycles, then BUDGET_STOP; infeasible ->
        INFEASIBLE_STOP with a controlled decel command.
        """
        p = self.p
        if warm is not None:
            _xbar_prev, U_prev = warm
            U_guess = np.vstack([U_prev[1:], U_prev[-1:]])
            xbar = rollout(x0, U_guess, p.dt)
            ubar = U_guess
        else:
            xbar, ubar = self.initial_guess(x0, float(np.mean(v_ref)))
        # The iterate is only a linearisation anchor, but the band rows are
        # written relative to it: if a previous cycle's solution drifted a
        # predicted point outside the band (obstacle slack pressure in a
        # narrow stretch does this), the next cycle starts with a negative
        # feasibility margin no input can recover in one step. Re-anchoring
        # the iterate inside the band keeps the margin non-negative; the
        # dynamics' affine term absorbs the clamp. Step 0 is NOT clamped:
        # it must stay exactly at the measured state (z_0 = 0).
        for k in range(1, len(xbar)):
            xbar[k, 0:2] = self.ref.band.clamp(xbar[k, 0:2])

        t0 = time.monotonic()
        # Ladder of solves, each on a freshly built model (osqp's in-place
        # update path proved unreliable once the linearisation point moves):
        #   1. warm iterate + warm start from the previous primal/dual
        #   2. same matrices, cold start - separates an osqp infeasibility
        #      certificate misfire from a real conflict
        #   3. cold iterate (straight-line guess) - separates a degenerate
        #      linearisation point from a real conflict
        # Only after all three fail does the chair stop.
        P, q, A, l, u = self._assemble(xbar, ubar, v_ref, th_ref,
                                       list(obstacles))
        res = self._try_solve(P, q, A, l, u, warm=True)
        if res.info.status not in ("solved", "solved_inaccurate"):
            res = self._try_solve(P, q, A, l, u, warm=False)
        if res.info.status not in ("solved", "solved_inaccurate"):
            xbar, ubar = self.initial_guess(x0, float(np.mean(v_ref)))
            P, q, A, l, u = self._assemble(xbar, ubar, v_ref, th_ref,
                                           list(obstacles))
            res = self._try_solve(P, q, A, l, u, warm=False)

        solved = res.info.status in ("solved", "solved_inaccurate")
        # The budget verdict belongs to the first rung that produced a
        # solved answer. The refinement below is best-effort and must never
        # demote that answer: a slow refinement used to turn solved cycles
        # into REUSED ones, which is worse than the drift it polishes.
        first_ok_ms = (time.monotonic() - t0) * 1e3 if solved else None
        if solved and res.x is not None:
            self._prev_x = res.x
            self._prev_y = res.y
        # Second SQP iteration, re-linearising about the solution just
        # found: one pass leaves residual linearisation drift exactly where
        # the band narrows fast. Only attempted when the first answer came
        # back with enough of the budget left to spend.
        if solved and first_ok_ms < p.solve_budget_s * 1e3 * 0.4:
            z_sol = res.x[:self.n_state].reshape(p.horizon, NX)
            mu_sol = res.x[self.n_state:self.n_state + self.n_input].reshape(
                p.horizon, NU)
            xbar2 = xbar.copy()
            xbar2[1:] = xbar[1:] + z_sol
            xbar2[0] = x0
            for k in range(1, len(xbar2)):
                xbar2[k, 0:2] = self.ref.band.clamp(xbar2[k, 0:2])
            ubar2 = np.clip(ubar + mu_sol,
                            [p.a_min, -p.al_max], [p.a_max, p.al_max])
            P2, q2, A2, l2, u2 = self._assemble(xbar2, ubar2, v_ref, th_ref,
                                                list(obstacles))
            res2 = self._try_solve(P2, q2, A2, l2, u2, warm=True)
            if res2.info.status in ("solved", "solved_inaccurate") \
                    and res2.x is not None:
                res, xbar, ubar = res2, xbar2, ubar2
                self._prev_x = res.x
                self._prev_y = res.y
        solve_ms = (time.monotonic() - t0) * 1e3
        if not solved:
            self.reuse_streak += 1
            return (np.array([p.a_min, 0.0]), STATUS_INFEASIBLE_STOP,
                    {"solve_ms": solve_ms, "raw": res.info.status})
        if first_ok_ms > p.solve_budget_s * 1e3:
            self.reuse_streak += 1
            if self.reuse_streak <= p.max_reuse and warm is not None:
                u0 = np.clip(warm[1][0], [p.a_min, -p.al_max],
                             [p.a_max, p.al_max])
                return u0, STATUS_REUSED, {"solve_ms": solve_ms}
            return (np.array([p.a_min, 0.0]), STATUS_BUDGET_STOP,
                    {"solve_ms": solve_ms})
        self.reuse_streak = 0
        u0 = ubar[0] + res.x[self._u(0):self._u(0) + NU]
        u0 = np.clip(u0, [p.a_min, -p.al_max], [p.a_max, p.al_max])
        slack_max = float(np.max(res.x[self.n_state + self.n_input:]))
        # Blocked detection: slack may borrow room for a moment while passing
        # an obstacle, but leaning on it is not passing. Two rungs: never
        # sit inside the obstacle's padding, and stop once a large slack has
        # been held for slack_stop_cycles consecutive cycles WITHOUT arc
        # progress - passing through shows as slack plus forward motion,
        # being wedged shows as slack and none.
        for ob in obstacles:
            if np.linalg.norm(x0[:2] - ob.xy) < p.obstacle_floor_m:
                return (np.array([p.a_min, 0.0]), STATUS_BLOCKED_STOP,
                        {"solve_ms": solve_ms, "slack_max": slack_max})
        if slack_max > p.slack_stop:
            arc_now = self.ref.arc_at(x0[:2])
            if self.slack_streak == 0:
                self.block_anchor_arc = arc_now
            self.slack_streak += 1
            if arc_now - self.block_anchor_arc > p.slack_progress_m:
                self.slack_streak = 1        # moving through, keep watching
                self.block_anchor_arc = arc_now
        else:
            self.slack_streak = 0
            self.block_anchor_arc = None
        if self.slack_streak >= p.slack_stop_cycles:
            return (np.array([p.a_min, 0.0]), STATUS_BLOCKED_STOP,
                    {"solve_ms": solve_ms, "slack_max": slack_max})
        info = {"solve_ms": solve_ms, "xbar": xbar, "ubar": ubar,
                "raw": res.info.status, "slack_max": slack_max}
        return u0, STATUS_OK, info

    def _fresh_model(self, P, q, A, l, u):
        model = self.osqp.OSQP()
        model.setup(P=P, q=q, A=A, l=l, u=u, verbose=False,
                    eps_abs=self.p.osqp_eps, eps_rel=self.p.osqp_eps,
                    max_iter=self.p.osqp_max_iter, polish=False,
                    warm_start=True)
        return model

    def _try_solve(self, P, q, A, l, u, warm):
        model = self._fresh_model(P, q, A, l, u)
        if warm and self._prev_x is not None and len(self._prev_x) == self.n_dec:
            try:
                model.warm_start(x=self._prev_x, y=self._prev_y)
            except Exception:
                pass
        self._model = model
        return model.solve()
