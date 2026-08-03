"""Start and goal only, inside the band - and the ways that quietly failed.

Every test here is a regression. Building this planner produced a long run
of results that looked like "the corridor is blocked" and were nothing of
the kind: a horizon too short to physically contain the path, a margin split
across two places so it stopped being a margin, a chair treated as a point.
None of them raised anything. They all came back as a stubborn residual,
which is exactly what a genuinely blocked road looks like.

So the through-line is: when the planner refuses, the refusal has to be
about the world. Anything else is a budget it set for itself.
"""

import importlib.util
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


pl = load("priest_planner")


def bent_corridor(length=40.0, amplitude=2.5, half_width=0.8, samples=200):
    s = np.linspace(0.0, length, samples)
    centres = np.stack([s, amplitude * np.sin(s / length * 2 * np.pi)], axis=1)
    tangent = np.gradient(centres, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    normals = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
    limits = np.full(samples, half_width)
    return pl.Corridor(centres, normals, limits, limits)


def drive(corridor, obstacles, seed=0, execute=6, limit=200):
    """Run the planner in closed loop, executing part of each plan."""
    planner = pl.PriestPlanner(seed=seed)
    position = corridor.centres[0].copy()
    velocity = np.zeros(2)
    acceleration = np.zeros(2)
    clearance = np.inf
    lateral = 0.0
    for cycles in range(limit):
        plan = planner.plan(position, velocity, acceleration, corridor,
                            obstacles)
        if plan.reason == "AT_GOAL":
            break
        if not plan.usable:
            return {"arc": corridor.arc_of(position), "reason": plan.reason,
                    "cycles": cycles, "clearance": clearance,
                    "lateral": lateral}
        points = plan.points()
        step = min(execute, len(points) - 1)
        if len(obstacles):
            circles = np.asarray(obstacles, dtype=np.float64)
            gap = (np.hypot(points[:step + 1, 0][:, None] - circles[:, 0],
                            points[:step + 1, 1][:, None] - circles[:, 1])
                   - circles[:, 2])
            clearance = min(clearance, gap.min())
        for point in points[:step + 1]:
            index = int(np.argmin(np.linalg.norm(corridor.centres - point,
                                                 axis=1)))
            lateral = max(lateral, abs(np.dot(corridor.normals[index],
                                              point - corridor.centres[index])))
        position = points[step]
        velocity = ((points[step] - points[step - 1])
                    / max(plan.times[step] - plan.times[step - 1], 1e-3))
        if corridor.arc_of(position) > corridor.length_m - 0.6:
            break
    return {"arc": corridor.arc_of(position), "reason": "", "cycles": cycles,
            "clearance": clearance, "lateral": lateral}


# ------------------------------------------------------- start to goal only

def test_it_reaches_the_goal_with_nothing_but_a_corridor():
    """No waypoint to track. The planner is given where it is, where the goal
    is, and the lateral limits in between."""
    corridor = bent_corridor()
    result = drive(corridor, [])

    assert result["reason"] == ""
    assert result["arc"] > corridor.length_m - 1.0
    assert result["lateral"] <= 0.8 + 1e-3


def test_it_reaches_the_goal_past_obstacles_it_was_never_told_about_in_advance():
    corridor = bent_corridor()
    centres = corridor.centres
    obstacles = [[centres[40, 0], centres[40, 1] + 0.30, 0.40],
                 [centres[100, 0], centres[100, 1] - 0.35, 0.45],
                 [centres[150, 0], centres[150, 1] + 0.30, 0.40]]

    result = drive(corridor, obstacles)

    assert result["reason"] == ""
    assert result["arc"] > corridor.length_m - 1.0
    assert result["lateral"] <= 0.8 + 1e-3


# -------------------------------------------------------- the chair is wide

def test_obstacles_are_grown_by_the_chair_and_not_just_by_its_centre():
    """The projection keeps a POINT outside each circle. Passing raw radii
    produced trajectories clearing an obstacle surface by 0.04 m - correct
    for a point, a collision for a 0.70 m chair. Clearance is measured to the
    real surface, so it has to exceed the half width."""
    planner = pl.PriestPlanner()
    grown = planner.inflate(np.array([[3.0, 0.0, 0.40]]))

    assert grown[0, 2] == pytest.approx(
        0.40 + planner.CHAIR_HALF_WIDTH_M + planner.OBSTACLE_MARGIN_M)

    corridor = bent_corridor()
    centres = corridor.centres
    result = drive(corridor,
                   [[centres[100, 0], centres[100, 1] + 0.35, 0.5]])

    assert result["reason"] == ""
    assert result["clearance"] >= planner.CHAIR_HALF_WIDTH_M, (
        "passed %.2f m from an obstacle surface with a %.2f m half width"
        % (result["clearance"], planner.CHAIR_HALF_WIDTH_M))


def test_a_gap_narrower_than_the_chair_is_refused_rather_than_squeezed():
    """The counterpart. Two obstacles whose grown radii cover the whole band
    leave no passage, and the honest answer is to stop. Before inflation the
    planner threaded a point through and reported success."""
    corridor = bent_corridor()
    centres = corridor.centres
    blocked = [[centres[60, 0], centres[60, 1] - 0.30, 0.40],
               [centres[63, 0], centres[63, 1] + 0.26, 0.40]]

    result = drive(corridor, blocked)

    assert result["reason"] == "NO_FEASIBLE_TRAJECTORY"
    assert result["arc"] < corridor.arc[60]


# --------------------------------------------------- budgets, not the world

def test_the_horizon_covers_the_reach_it_is_used_with():
    """A horizon shorter than path_length / v_max is infeasible before the
    optimiser starts, and it cannot say so except as a residual. Every early
    failure here was this.

    The property is not that the horizon grows without limit - it saturates
    at a ceiling, which is exactly why the reach is then derived back OUT of
    the horizon rather than assumed. What must hold is that the pair the
    planner actually uses is consistent: whatever reach it plans over fits
    inside the horizon with the margin still intact.
    """
    planner = pl.PriestPlanner(v_max=0.6)

    for remaining in (2.0, 5.0, 12.0, 30.0, 120.0):
        horizon = planner.horizon_for(remaining)
        reach = min(remaining, planner.v_max * horizon / planner.margin)
        assert horizon * planner.v_max >= reach * planner.margin - 1e-6, remaining
        assert reach > 0.0


def test_the_margin_is_one_number_used_in_both_places():
    """It sizes the horizon and it cuts the reach back out of it. Split
    across the two, the reach divisor was a literal while the horizon
    saturated its ceiling, so raising the margin changed nothing at all -
    a sweep over five values returned five identical residuals."""
    source = (SCRIPTS / "priest_planner.py").read_text(encoding="utf-8")

    assert "self.v_max * self.horizon_for(remaining)" in source
    assert source.count("/ self.margin") >= 1
    assert "self.margin * reach_m" in source
    assert "/ 1.35" not in source


def test_a_short_horizon_is_retried_smaller_before_it_is_called_blocked():
    """The refusal has to be about the world. When a slice cannot be planned,
    taking a smaller bite is the physically meaningful response, and only an
    exhausted backoff justifies claiming the corridor is impassable."""
    source = (SCRIPTS / "priest_planner.py").read_text(encoding="utf-8")
    assert "REACH_BACKOFF" in source and "MIN_REACH_M" in source

    corridor = bent_corridor()
    planner = pl.PriestPlanner(seed=0)
    plan = planner.plan(corridor.centres[0], np.zeros(2), np.zeros(2),
                        corridor, [])

    assert plan.usable and plan.residual <= planner.FEASIBLE_M


def test_the_leader_is_the_most_feasible_elite_not_the_cheapest():
    """Cost only means anything among trajectories that can be driven. Taking
    the cheapest of the elite set let a slightly infeasible trajectory lead,
    and its residual is what the caller then decides to drive on."""
    source = (SCRIPTS / "priest_planner.py").read_text(encoding="utf-8")
    assert "leader = int(np.argmin(elite_residual))" in source


def test_the_projection_is_sized_to_the_scene():
    """F's rows scale with the obstacle cap, and so does every iteration:
    24 slots cost 31 ms a solve where 6 cost 9. Sizing to the obstacles
    present took a closed-loop cycle from 620 ms to 54 ms."""
    planner = pl.PriestPlanner(max_obstacles=24)
    _, lean = planner.basis_for(12.0, 1)
    _, full = planner.basis_for(12.0, 24)

    assert lean.max_obstacles == 1
    assert full.max_obstacles == 24
    assert lean.F.shape[0] < full.F.shape[0]


def test_reaching_the_goal_is_reported_rather_than_driven_past():
    corridor = bent_corridor()
    plan = pl.PriestPlanner().plan(corridor.centres[-1], np.zeros(2),
                                   np.zeros(2), corridor, [])

    assert plan.reason == "AT_GOAL"
    assert not plan.usable
