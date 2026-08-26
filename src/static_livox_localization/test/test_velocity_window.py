"""The planner searches speeds it can actually reach, and pays to change one.

The 2026-08-23 run changed its target speed 1,035 times over the flat parts
of the route, about once a second, cycling 0.30 / 0.42 / 0.55 / 0.68 / 0.80
as five nearly-tied candidates reordered under tiny movements in path cost.
Two things were missing. The whole range was re-scored every cycle with no
regard for what the chair was doing, and nothing rewarded holding a speed -
the yaw axis had W_STEER for exactly that and the speed axis had nothing.

The ramp's asymmetry is what turned that into a lurch: it brakes at
0.60 m/s^2 and accelerates at 0.18, so every needless dip cost three times
longer to climb out of than to fall into.
"""

import importlib.util
from pathlib import Path

from pytest import approx


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(
        name, SCRIPTS / ("%s.py" % name))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


dwa_core = load("dwa_core")
samples = dwa_core.speed_samples
FLOOR = dwa_core.TURN_FLOOR_SPEED
TOP = dwa_core.MAX_SPEED


def moving(values):
    return [v for v in values if v > 0.0]


def test_without_a_current_speed_the_whole_range_is_offered():
    """The old behaviour, kept for callers that have no velocity to give."""
    v = moving(samples(TOP))
    assert min(v) == FLOOR
    assert max(v) == TOP


def test_the_window_is_drawn_around_where_the_chair_is():
    v = moving(samples(TOP, current=0.35))
    assert max(v) <= 0.35 + dwa_core.MAX_ACCEL * dwa_core.VELOCITY_WINDOW_S + 1e-9
    assert max(v) < TOP, "a standing start must not be able to pick top speed"


def test_the_chair_can_pull_away_from_rest():
    """The window bounds what the ramp reaches; the floor is what the wheels
    execute. Confusing the two stopped the chair dead on 2026-08-23: from
    rest the reachable ceiling is 0.18 m/s against a 0.35 floor, so no
    candidate existed and it reported SPEED_BELOW_FLOOR from a standstill
    for as long as it was asked to drive."""
    for current in (0.0, 0.05, 0.1, 0.2):
        offered = moving(samples(TOP, current=current))
        assert offered, "a standing chair must still be offered a speed"
        assert min(offered) == approx(FLOOR)


def test_braking_stays_available_from_any_speed():
    """The window is asymmetric because the chair is. Slowing down must never
    be the thing the window forbids."""
    v = moving(samples(TOP, current=TOP))
    assert min(v) == FLOOR


def test_the_window_never_offers_less_than_the_floor():
    for current in (0.35, 0.5, 0.8):
        assert min(moving(samples(TOP, current=current))) >= FLOOR - 1e-9


def test_a_cap_under_the_floor_still_reports_no_executable_speed():
    """SPEED_BELOW_FLOOR must survive the window: the caller distinguishes it
    from an obstacle, and that distinction cost a 2.4 minute stall to learn."""
    assert moving(samples(0.2, current=0.5)) == []


def test_a_reachable_cap_is_honoured_over_the_window():
    v = moving(samples(0.45, current=0.8))
    assert max(v) <= 0.45 + 1e-9


def test_holding_a_speed_is_cheaper_than_changing_one():
    """W_SPEED is what breaks the ties that were flipping every cycle."""
    assert dwa_core.W_SPEED > 0.0


def test_the_rollout_makes_every_speed_score_the_same_geometry():
    """Why a speed reward has to exist at all, pinned so it cannot be
    quietly removed. The rollout is sampled over a fixed distance, so two
    speeds going straight walk the same arc - and every geometric term in
    the cost is computed on that arc. Without W_VELOCITY the five straight
    candidates are not nearly tied, they are exactly tied, and the winner
    is whichever one argmin sees first."""
    state = (0.0, 0.0, 0.0, 0.0, 0.0)
    slow = dwa_core.rollout(state, 0.35, 0.0)
    fast = dwa_core.rollout(state, 0.80, 0.0)
    assert slow.shape == fast.shape
    assert abs(slow[-1][0] - fast[-1][0]) < 1e-6
    assert abs(slow[-1][1] - fast[-1][1]) < 1e-6


def test_a_turn_is_the_one_place_speed_shows_in_the_geometry():
    """Turning, the radius is v / w, so the arcs do differ and the geometric
    terms have something to say about speed. Straight, they have nothing -
    and straight flat running is where the target was changing once a
    second."""
    state = (0.0, 0.0, 0.0, 0.0, 0.0)
    slow = dwa_core.rollout(state, 0.35, 0.3)
    fast = dwa_core.rollout(state, 0.80, 0.3)
    assert abs(slow[-1][1] - fast[-1][1]) > 0.01


def test_the_preview_does_not_collapse_as_the_chair_speeds_up():
    """A fixed preview distance is a shrinking preview time, and the
    actuation lag does not shrink with it: 1.05 m is 3.0 s at 0.35 m/s and
    1.3 s at 0.80, so the 0.55 s lag goes from a fifth of the preview to
    nearly half. That is what the weave was made of on 2026-08-23 - amplitude
    0.15 m, then 0.24, then 0.53, with the yaw rate saturating three times in
    thirteen seconds."""
    planner = dwa_core.DwaPlanner.__new__(dwa_core.DwaPlanner)
    planner.distance_m = dwa_core.SIM_DISTANCE_M
    for speed in (0.5, 0.65, 0.8):
        preview_s = planner.preview_distance(speed) / speed
        assert preview_s >= min(
            dwa_core.SIM_MIN_PREVIEW_S,
            dwa_core.SIM_DISTANCE_M / speed) - 1e-9
    assert planner.preview_distance(0.8) / 0.8 >= 1.9


def test_a_slow_chair_keeps_the_preview_it_had():
    """The time is a floor, not a replacement. Creeping, the fixed distance
    already buys three seconds and stretching it further would have the
    planner steering for a place it will not reach for ages."""
    planner = dwa_core.DwaPlanner.__new__(dwa_core.DwaPlanner)
    planner.distance_m = dwa_core.SIM_DISTANCE_M
    assert planner.preview_distance(0.35) == approx(dwa_core.SIM_DISTANCE_M)


def test_speed_cannot_buy_more_than_a_hands_width_off_the_line():
    """The reward has to be small next to the terms that keep the chair on
    its line, or it pays for speed with position. At 2.0 it bought 0.3 m and
    the chair wove with a growing amplitude - 0.15 m, 0.24, 0.53 - saturating
    the yaw rate three times in thirteen seconds before the gate stopped it.
    The 2026-08-24 tracking pass tightened that trade to 0.11 m so the
    recorded route wins before the old weave can build again."""
    span = dwa_core.W_VELOCITY * (dwa_core.MAX_SPEED - FLOOR)
    tolerated_error_m = span / dwa_core.W_PATH
    assert tolerated_error_m <= 0.11


def test_going_faster_beats_holding_when_the_geometry_is_tied():
    """With the arc identical, the only terms left are the speed reward and
    the change penalty. The reward has to win or the chair never leaves the
    floor - which is exactly what it did on the first run after the window
    went in."""
    assert dwa_core.W_VELOCITY > dwa_core.W_SPEED
    for current, reachable in ((0.35, 0.53), (0.53, 0.71)):
        gain = dwa_core.W_VELOCITY * (reachable - current)
        penalty = dwa_core.W_SPEED * (reachable - current)
        assert gain > penalty
