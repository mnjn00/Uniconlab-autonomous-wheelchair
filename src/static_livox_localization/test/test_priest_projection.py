"""The projection block, and the ways it silently produced nonsense.

PRIEST's one structural difference from CEM/MPPI is that samples are pushed
onto the feasible set before they are ranked. That only helps if "feasible"
is what the projection actually enforces, and while building this it twice
was not - in both cases without an error, a warning, or a NaN. So these
tests check feasibility of the OUTPUT rather than the shape of the formulae.

Everything here is offline numpy. The optimiser has no ROS dependency on
purpose: it can be argued with at a desk.
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            name, SCRIPTS / ("%s.py" % name))
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


pp = load("priest_projection")


def straight_corridor(basis, length=8.0, half_width=0.8):
    s = np.linspace(0.0, length, basis.steps)
    centres = np.stack([s, np.zeros_like(s)], axis=1)
    normals = np.tile(np.array([0.0, 1.0]), (basis.steps, 1))
    limits = np.full(basis.steps, half_width)
    return centres, normals, limits, limits


def seeded_batch(basis, start, goal, count=200, spread=0.4, seed=0):
    line = np.linspace(np.asarray(start), np.asarray(goal), basis.n_c)
    flat = np.hstack([line[:, 0], line[:, 1]])
    rng = np.random.default_rng(seed)
    return np.tile(flat, (count, 1)) + rng.normal(0, spread, (count, 2 * basis.n_c))


# ----------------------------------------------------------------- the basis

def test_the_basis_derivatives_are_the_derivatives():
    """A wrong Pddot does not crash, it just quietly stops bounding
    acceleration. Compared against finite differences at a step large enough
    that the second difference is not swamped by its own rounding."""
    basis = pp.TrajectoryBasis(degree=10, steps=40, horizon_s=8.0)
    h = 1e-3
    p0, _, _ = pp.bernstein_basis(10, basis.times, 8.0)
    plus, _, _ = pp.bernstein_basis(10, basis.times + h, 8.0)
    minus, _, _ = pp.bernstein_basis(10, basis.times - h, 8.0)

    assert np.abs(basis.Pdot[2:-2] - (plus - minus)[2:-2] / (2 * h)).max() < 1e-6
    assert np.abs(basis.Pddot[2:-2]
                  - (plus - 2 * p0 + minus)[2:-2] / h ** 2).max() < 1e-5


def test_constant_coefficients_are_a_point_at_rest():
    """Partition of unity. If it fails, every trajectory carries a spurious
    velocity proportional to where it is on the map."""
    basis = pp.TrajectoryBasis(degree=10, steps=40, horizon_s=8.0)
    coefficients = np.full((1, basis.n_c), 3.0)

    assert np.allclose(coefficients @ basis.P.T, 3.0)
    assert np.abs(coefficients @ basis.Pdot.T).max() < 1e-12
    assert np.abs(coefficients @ basis.Pddot.T).max() < 1e-12


# ------------------------------------------------------------ the projection

def test_the_projection_produces_feasible_trajectories():
    """The whole contract, and the only thing that settles the sign of the
    KKT right-hand side - the paper writes it with a minus, the derivation
    here gives a plus, and no amount of reading decides that."""
    basis = pp.TrajectoryBasis(degree=10, steps=40, horizon_s=20.0)
    projection = pp.Projection(basis, max_obstacles=4, v_max=0.6, a_max=0.5)
    centres, normals, left, right = straight_corridor(basis)
    projection.set_corridor(centres, normals, left, right)
    start, goal = centres[0], centres[-1]
    boundary = [start[0], 0, 0, goal[0], start[1], 0, 0, goal[1]]
    obstacles = np.array([[4.0, 0.3, 0.45]])

    xi, residual = projection.solve(
        seeded_batch(basis, start, goal), boundary, obstacles, iterations=40)

    assert residual.min() == pytest.approx(0.0, abs=1e-6)
    best = xi[int(np.argmin(residual))][None, :]
    x, y = basis.positions(best)
    (vx, vy), (ax, ay) = basis.derivatives(best)
    assert np.hypot(vx, vy).max() <= 0.6 + 1e-6
    assert np.hypot(ax, ay).max() <= 0.5 + 1e-6
    assert np.abs(y).max() <= 0.8 + 1e-6
    clearance = np.hypot(x[0] - 4.0, y[0] - 0.3).min() - 0.45
    assert clearance >= -1e-6


def test_the_boundary_conditions_are_met_exactly():
    """They are equality constraints inside the KKT system, so "close" would
    mean the solve is not doing what it claims."""
    basis = pp.TrajectoryBasis(degree=10, steps=40, horizon_s=20.0)
    projection = pp.Projection(basis, max_obstacles=2)
    start, goal = np.array([1.0, -0.5]), np.array([7.0, 0.25])
    boundary = [start[0], 0.2, 0.0, goal[0], start[1], -0.1, 0.0, goal[1]]

    xi, _ = projection.solve(
        seeded_batch(basis, start, goal), boundary, [], iterations=10)
    x, y = basis.positions(xi)
    (vx, vy), _ = basis.derivatives(xi)

    assert np.abs(x[:, 0] - start[0]).max() < 1e-8
    assert np.abs(y[:, 0] - start[1]).max() < 1e-8
    assert np.abs(x[:, -1] - goal[0]).max() < 1e-8
    assert np.abs(vx[:, 0] - 0.2).max() < 1e-8


def test_padding_obstacles_never_pull_the_batch_off_the_map():
    """The bug that cost the most. F has a fixed row count, so unused slots
    are padded - and the polar constraint x - x_o = a*d*cos(alpha) with a = 0
    reads "be exactly AT the obstacle centre", the opposite of ignore me. The
    padding sits a kilometre away, so the symptom was a whole batch dragged
    off the map with no error anywhere; the fix is that the same effective
    radius must divide in d and multiply in e.

    What is pinned is feasibility, not equality. Padding rows are inert as
    TARGETS but they still enter rho*F^T F, so they change the metric the
    proximity operator uses and the solve lands on a different feasible
    point - here 0.38 m apart. That is why the planner sizes its projection
    to the obstacles actually present rather than treating max_obstacles as
    free capacity.
    """
    basis = pp.TrajectoryBasis(degree=10, steps=30, horizon_s=15.0)
    start, goal = np.array([0.0, 0.0]), np.array([5.0, 0.0])
    boundary = [start[0], 0, 0, goal[0], start[1], 0, 0, goal[1]]
    samples = seeded_batch(basis, start, goal, count=40)
    one = np.array([[2.5, 0.9, 0.4]])

    for slots in (1, 4, 20):
        projection = pp.Projection(basis, max_obstacles=slots)
        xi, residual = projection.solve(samples, boundary, one, iterations=25)
        x, y = basis.positions(xi[int(np.argmin(residual))][None, :])

        assert residual.min() == pytest.approx(0.0, abs=1e-6), slots
        # The padding is 1 km away; a trajectory near it is the old bug.
        assert np.abs(x).max() < 50.0 and np.abs(y).max() < 50.0, slots


def test_a_cleared_obstacle_pulls_on_nothing():
    """Same self-cancelling property from the other side: a trajectory that
    is already outside a circle must be left exactly where it is, which is
    what lets one fixed F cover a scene that changes every cycle."""
    basis = pp.TrajectoryBasis(degree=10, steps=30, horizon_s=15.0)
    start, goal = np.array([0.0, 0.0]), np.array([5.0, 0.0])
    boundary = [start[0], 0, 0, goal[0], start[1], 0, 0, goal[1]]
    samples = seeded_batch(basis, start, goal, count=20)
    projection = pp.Projection(basis, max_obstacles=2)

    free, _ = projection.solve(samples, boundary, [], iterations=20)
    far, _ = projection.solve(
        samples, boundary, np.array([[2.5, 40.0, 0.5]]), iterations=20)

    assert np.abs(free - far).max() < 1e-6


def test_the_corridor_is_a_hard_limit_not_a_preference():
    """The band is the only thing that knows where the ground breaks - the
    sensor cannot see it within 5.9 m. A corridor that merely discouraged
    leaving would be no drop safety at all.

    Bounded, not exact. This is an augmented-Lagrangian penalty, so it
    approaches the constraint from outside and never quite lands: 200
    iterations still leave 15 micrometres. Demanding equality here would be
    demanding something the method does not offer, and the guarantee the
    stack actually leans on is the companion test below - that whatever
    overshoot remains is reported in the residual.
    """
    basis = pp.TrajectoryBasis(degree=10, steps=40, horizon_s=20.0)
    projection = pp.Projection(basis, max_obstacles=2)
    centres, normals, _, _ = straight_corridor(basis)
    narrow = np.full(basis.steps, 0.25)
    projection.set_corridor(centres, normals, narrow, narrow)
    start, goal = centres[0], centres[-1]
    boundary = [start[0], 0, 0, goal[0], start[1], 0, 0, goal[1]]

    # Samples deliberately pushed far outside the corridor.
    samples = seeded_batch(basis, start, goal, count=100, spread=1.5, seed=7)
    xi, residual = projection.solve(samples, boundary, [], iterations=200)

    _, y = basis.positions(xi[int(np.argmin(residual))][None, :])
    assert np.abs(y).max() <= 0.25 + 1e-3


def test_a_corridor_violation_always_shows_up_in_the_residual():
    """The guarantee is not "never violated", it is "never violated by more
    than the residual says". Alternating minimisation converges onto the
    constraint rather than landing on it: 20 iterations leaves 29 mm of
    overshoot on a 0.25 m corridor, 160 leaves none. The planner runs few
    iterations per pass and refuses anything whose residual exceeds its
    feasibility threshold, so what has to hold is that the residual is
    honest - a trajectory outside the band must never come back as zero.
    """
    basis = pp.TrajectoryBasis(degree=10, steps=30, horizon_s=15.0)
    projection = pp.Projection(basis, max_obstacles=2)
    s = np.linspace(0.0, 8.0, basis.steps)
    centres = np.stack([s, np.zeros_like(s)], axis=1)
    normals = np.tile(np.array([0.0, 1.0]), (basis.steps, 1))
    limit = np.full(basis.steps, 0.25)
    projection.set_corridor(centres, normals, limit, limit)
    samples = seeded_batch(basis, centres[0], centres[-1], count=60,
                           spread=1.5, seed=3)

    for iterations in (5, 20, 80):
        xi, residual = projection.solve(
            samples, [0, 0, 0, 8.0, 0, 0, 0, 0.0], [], iterations)
        _, y = basis.positions(xi)
        overshoot = np.clip(np.abs(y).max(axis=1) - 0.25, 0.0, None)
        assert (residual[overshoot > 1e-9] > 0.0).all(), (
            "a trajectory outside the band reported zero residual at %d "
            "iterations" % iterations)


def test_the_residual_counts_only_the_violated_side():
    """Summing a signed gap would let a wide berth on one side pay for a
    collision on the other, and the residual is what Algorithm 1 ranks on
    before it ever looks at cost."""
    basis = pp.TrajectoryBasis(degree=6, steps=20, horizon_s=10.0)
    projection = pp.Projection(basis, max_obstacles=1)
    line = np.linspace([0.0, 0.0], [4.0, 0.0], basis.n_c)
    xi = np.hstack([line[:, 0], line[:, 1]])[None, :]

    clear = projection.residual(xi, projection.pad_obstacles([[2.0, 9.0, 0.3]]))
    through = projection.residual(xi, projection.pad_obstacles([[2.0, 0.0, 0.5]]))

    assert clear[0] == pytest.approx(0.0)
    assert through[0] > 0.0


def test_everything_the_kkt_is_built_from_is_finite():
    """Some BLAS builds raise spurious divide and overflow flags out of
    matmul, so the module silences them. That is only defensible if the
    operands are actually finite, which is checked here rather than assumed
    at the point the warnings were switched off."""
    basis = pp.TrajectoryBasis(degree=10, steps=40, horizon_s=12.0)
    projection = pp.Projection(basis, max_obstacles=8)
    centres, normals, left, right = straight_corridor(basis)
    projection.set_corridor(centres, normals, left, right)

    for name in ("P", "Pdot", "Pddot"):
        assert np.isfinite(getattr(basis, name)).all(), name
    for name in ("F", "A", "M", "G", "tau"):
        assert np.isfinite(getattr(projection, name)).all(), name
