"""The DWA profile: what it may command, and what it must refuse.

The band is enforced as a rollout critic rather than a costmap layer, so
these run against the band's own geometry - there is no grid here to be
wrong about, which is the point of the design.
"""

import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"

sys.path.insert(0, str(SCRIPTS))
try:
    import dwa_core
    from safety_band import SafetyBand
finally:
    sys.path.pop(0)


def shipped(kind):
    text = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8")
    m = re.search(r'^%s="\$\{%s:-.*?/routes/(\S+?)\}"' % (kind, kind),
                  text, re.M)
    assert m, "cannot tell which %s the bringup ships" % kind
    return ROOT / "routes" / m.group(1)


@pytest.fixture(scope="module")
def scene():
    band = SafetyBand(str(shipped("BAND")))
    import json
    route = np.array([[w["x"], w["y"]] for w in
                      json.load(open(shipped("ROUTE")))["waypoints"]])
    return band, route, dwa_core.DwaPlanner(band, route)


def on_route(route, k):
    heading = math.atan2(*(route[k + 3] - route[k])[::-1])
    return np.array([route[k][0], route[k][1], heading, 0.0, 0.0])


# ------------------------------------------------- the executable envelope

def test_no_candidate_lands_in_the_actuation_deadband():
    """The loaded base was measured not to move below about 0.30 m/s.
    Sampling inside that gap produces candidates that score well, get
    commanded, and do nothing - which is the standstill the MPC node sat in
    on 2026-08-05."""
    for v in dwa_core.speed_samples():
        assert v == 0.0 or v >= dwa_core.TURN_FLOOR_SPEED


def test_speeds_and_yaw_rates_stay_inside_the_chair_limits():
    assert max(dwa_core.speed_samples()) <= dwa_core.MAX_SPEED
    assert max(abs(w) for w in dwa_core.yaw_samples()) <= dwa_core.MAX_YAW_RATE


def test_a_speed_cap_below_the_floor_leaves_only_a_stop():
    """When the speed policy says less than the chair can execute, the only
    honest candidate is zero - not a crawl it will ignore."""
    assert dwa_core.speed_samples(max_speed=0.2) == (0.0,)


def test_turning_on_the_spot_is_never_a_candidate(scene):
    """Rotating in place below the rotation floor is the manoeuvre that put
    the chair at a wall three times on 2026-08-04."""
    band, route, planner = scene
    for k in (40, 200, 900):
        v, w, status = planner.plan(on_route(route, k))
        assert status != "OK" or v >= dwa_core.TURN_FLOOR_SPEED


def test_standing_still_is_a_refusal_and_never_a_candidate(scene):
    """A stationary rollout is one point, so on the line its path cost is
    exactly zero and it outscores every arc that actually goes somewhere.
    On 2026-08-08 that held the chair for 180 s in one run and 77 s in the
    other while it reported a healthy fix and an admissible corridor."""
    band, route, planner = scene
    for k in (40, 200, 900, 1500):
        v, w, status = planner.plan(on_route(route, k))
        assert status == "OK"
        assert v > 0.0


def test_a_chair_pointed_well_off_the_corridor_still_turns_back(scene):
    """The 2026-08-08 deadlocks were at 51 and 76 degrees of heading error.
    Both runs sat there until a person took the joystick."""
    band, route, planner = scene
    for degrees in (30, 51, 76):
        state = on_route(route, 200)
        state[2] += math.radians(degrees)
        v, w, status = planner.plan(state)
        if status == "OK":
            assert v > 0.0
            # turning back towards the corridor, not away from it
            assert w < 0.0


# ----------------------------------------------------------- the band veto

def test_a_rollout_that_leaves_the_band_is_rejected(scene):
    """Not scored badly - rejected. The corridor is not a preference."""
    band, route, planner = scene
    state = on_route(route, 40)
    v, w, status = planner.plan(state)
    assert status == "OK"
    path = dwa_core.rollout(state, v, w, planner.sim_time_s)
    assert dwa_core.stays_in_band(band, path)


def test_the_whole_rollout_is_tested_not_just_where_it_ends(scene):
    """An arc that ends back inside the corridor having crossed out of it
    partway is not a candidate."""
    band, route, planner = scene
    state = on_route(route, 40)
    v, w, _ = planner.plan(state)
    path = dwa_core.rollout(state, v, w, planner.sim_time_s)
    assert len(path) == planner.steps
    inside = band.contains_many(path[:, :2])
    assert inside.all()


def test_a_chair_pointed_out_of_the_corridor_stops(scene):
    """Facing the wall with no admissible arc, the answer is a stop with a
    reason, never a best-effort command."""
    band, route, planner = scene
    state = on_route(route, 40)
    state[2] += math.pi / 2          # square across the corridor
    v, w, status = planner.plan(state)
    if status == "OK":
        path = dwa_core.rollout(state, v, w, planner.sim_time_s)
        assert dwa_core.stays_in_band(band, path)
    else:
        assert (v, w) == (0.0, 0.0)
        assert status in ("OFF_BAND", "OBSTACLE", "NO_CANDIDATE")


# -------------------------------------------------------------- obstacles

def test_an_object_in_the_corridor_is_cleared_or_refused(scene):
    band, route, planner = scene
    state = on_route(route, 40)
    heading = np.array([math.cos(state[2]), math.sin(state[2])])
    blocker = state[:2] + heading * 1.0
    v, w, status = planner.plan(state, obstacles=(blocker,))
    if status == "OK":
        path = dwa_core.rollout(state, v, w, planner.sim_time_s)
        assert dwa_core.obstacle_clearance(path, (blocker,)) >= \
            dwa_core.OBSTACLE_FLOOR_M
    else:
        assert (v, w) == (0.0, 0.0)


def test_clearance_is_infinite_with_nothing_to_clear():
    path = dwa_core.rollout(np.array([0.0, 0.0, 0.0]), 0.5, 0.0)
    assert not np.isfinite(dwa_core.obstacle_clearance(path, ()))


# ------------------------------------------------------- speed and refusal

def test_the_speed_policy_still_caps_the_choice(scene):
    band, route, planner = scene
    state = on_route(route, 40)
    v, _w, status = planner.plan(state, speed_cap=dwa_core.TURN_FLOOR_SPEED)
    assert status == "OK"
    assert v <= dwa_core.TURN_FLOOR_SPEED + 1e-9


def test_a_refusal_says_which_kind_it_was(scene):
    """A corridor with no admissible arc and one with an object standing in
    it are different faults and the operator has to be able to tell."""
    band, route, planner = scene
    state = on_route(route, 40)
    heading = np.array([math.cos(state[2]), math.sin(state[2])])
    wall = [state[:2] + heading * d for d in np.arange(0.4, 2.0, 0.1)]
    v, w, status = planner.plan(state, obstacles=wall)
    assert (v, w) == (0.0, 0.0)
    assert status == "OBSTACLE"


# ------------------------------------------------- what the score looks at

def test_the_score_reads_heading_and_not_only_position(scene):
    """A position-only cost driving a saturating actuator is a bang-bang
    regulator. On 2026-08-08 it commanded +-MAX_YAW_RATE for half of every
    sample and reversed sign every 1.8 s."""
    band, route, planner = scene
    saturated = 0
    for k in range(100, 1900, 60):
        v, w, status = planner.plan(on_route(route, k))
        saturated += status == "OK" and abs(abs(w) - dwa_core.MAX_YAW_RATE) < 1e-9
    assert saturated == 0


def test_reversing_the_steer_costs_something(scene):
    """Chatter between adjacent yaw samples was free before this term."""
    band, route, planner = scene
    state = on_route(route, 200)
    held = planner.plan(state, last_yaw_rate=0.0)[1]
    against = planner.plan(state, last_yaw_rate=-dwa_core.MAX_YAW_RATE)[1]
    assert against <= held


# --------------------------------------------------- node wiring, as source

def follower():
    return (SCRIPTS / "dwa_follower.py").read_text(encoding="utf-8")


def test_the_node_runs_the_inherited_guards():
    src = follower()
    assert "handled_before_driving" in src
    assert "advance_progress" in src


def test_the_node_does_not_reimplement_the_hold_ladder():
    src = follower()
    for copied in ("hold_candidates", "evaluate_holds", "WOULD_HOLD"):
        assert copied not in src


def test_the_node_shares_the_command_ramp():
    """Both non-pursuit profiles hit the same standstill on 2026-08-05, so
    both go through the ramp that fixed it rather than each rolling their
    own conversion."""
    assert "advance_command(" in follower()
    assert re.search(r"state\[3\]\s*\+", follower()) is None


def test_the_node_declares_its_control_law():
    assert 'CONTROL_LAW = "dwa"' in follower()


def test_the_bringup_offers_the_profile_and_defaults_elsewhere():
    text = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8")
    assert "dwa)     FOLLOWER_NODE=dwa_follower.py" in text
    assert 'PROFILE="${PROFILE:-pursuit}"' in text
    assert "PROFILE must be pursuit, mpc or dwa" in text
