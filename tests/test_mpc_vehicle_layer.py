"""The parts of the MPC profile that decide whether it may drive.

mpc_core has its own suite; this covers the layer that puts it on a chair -
the anchor, the reference floor, and the structural promise that the second
control law did not bring its own copy of the guards.

rospy is not importable off the vehicle, so mpc_follower is read as source
here rather than imported. That is a real limit and the assertions are
written to respect it: they check the call is present, not that it fires.
The runbook carries the on-vehicle checks that only a running node can give.
"""

import ast
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"

sys.path.insert(0, str(SCRIPTS))
try:
    import mpc_speed
    from mpc_anchor import StateAnchor, blend_angle, wrap_angle
    from safety_band import SafetyBand
finally:
    sys.path.pop(0)

BAND = ROOT / "routes" / "20260802_route_v4_safety_band.json"


@pytest.fixture(scope="module")
def band():
    return SafetyBand(str(BAND))


# ---------------------------------------------------------------- anchor

def test_anchor_snaps_on_first_sample():
    a = StateAnchor()
    s = a.update((1.0, 2.0), 0.5, 0.3, 0.1, 0.0)
    assert s[:3] == pytest.approx([1.0, 2.0, 0.5])


def test_anchor_blends_position_but_not_velocity():
    a = StateAnchor(gain=0.4)
    a.update((0.0, 0.0), 0.0, 0.0, 0.0, 0.0)
    s = a.update((0.1, 0.0), 0.0, 0.55, -0.2, 0.1)
    # position low-passed at the gain...
    assert s[0] == pytest.approx(0.04)
    # ...velocities taken outright, because wheel odometry is already the
    # smooth local truth and lagging it would only add delay.
    assert s[3] == pytest.approx(0.55)
    assert s[4] == pytest.approx(-0.2)


def test_anchor_snaps_through_a_jump_rather_than_crawling_to_it():
    a = StateAnchor(gain=0.4)
    a.update((0.0, 0.0), 0.0, 0.0, 0.0, 0.0)
    s = a.update((2.0, 0.0), 0.0, 0.0, 0.0, 0.1)
    assert s[0] == pytest.approx(2.0)
    assert a.jumps == 1


def test_anchor_snaps_after_a_stale_gap():
    a = StateAnchor(gain=0.4)
    a.update((0.0, 0.0), 0.0, 0.0, 0.0, 0.0)
    s = a.update((0.1, 0.0), 0.0, 0.0, 0.0, 5.0)
    assert s[0] == pytest.approx(0.1)


def test_anchor_heading_blend_crosses_the_pi_seam():
    # A linear blend of +179 deg and -179 deg gives 0 - pointing backwards.
    blended = blend_angle(math.radians(179), math.radians(-179), 0.5)
    assert abs(wrap_angle(blended)) > math.radians(179)


def test_anchor_reset_forgets_the_previous_pose():
    a = StateAnchor(gain=0.4)
    a.update((0.0, 0.0), 0.0, 0.0, 0.0, 0.0)
    a.reset("held")
    s = a.update((5.0, 5.0), 1.0, 0.0, 0.0, 0.1)
    assert s[:2] == pytest.approx([5.0, 5.0])


# ----------------------------------------------------------- speed floor

# The measurement this whole module exists to respect: sweeping a constant
# reference from a standing start over 20 s, the chair settles at exactly
# zero for any reference at or below 0.22 m/s, while the solve reports OK.
SILENT_STANDSTILL_CEILING = 0.22


def test_the_floor_clears_the_measured_standstill_zone():
    assert mpc_speed.TURN_FLOOR_SPEED > SILENT_STANDSTILL_CEILING


def test_a_reference_is_never_issued_inside_the_dead_zone(band):
    """Whatever the policy asks for, what reaches the solver is either a
    speed it can actually hold or a stop - never a creep it will answer by
    standing still and calling it OK."""
    for k in range(0, len(band.xy), 7):
        v_ref, stop = mpc_speed.shaped_reference(band, band.xy[k], 25)
        if stop is None:
            assert v_ref.min() >= mpc_speed.TURN_FLOOR_SPEED
        else:
            assert np.all(v_ref == 0.0)


def test_policy_below_the_floor_becomes_a_stop_not_a_creep(band):
    v_ref, stop = mpc_speed.shaped_reference(
        band, band.xy[10], 25, obstacle_speed=mpc_speed.CREEP_SPEED)
    assert stop == mpc_speed.STOP
    assert np.all(v_ref == 0.0)


def test_slope_and_degraded_land_exactly_on_the_floor(band):
    """SLOPE_SPEED is 0.30 and the floor is 0.30, so a slope slows the chair
    to the floor rather than stopping it. If either constant moves, a hill
    silently becomes a hold - this is the tripwire for that."""
    for kwargs in ({"pitch_rad": math.radians(10.0)}, {"degraded": True}):
        v_ref, stop = mpc_speed.shaped_reference(band, band.xy[10], 25,
                                                 **kwargs)
        assert stop is None, kwargs
        assert v_ref[0] == pytest.approx(mpc_speed.TURN_FLOOR_SPEED)


def test_reference_is_flat_across_the_horizon(band):
    v_ref, _ = mpc_speed.shaped_reference(band, band.xy[10], 25)
    assert len(set(np.round(v_ref, 9))) == 1


def test_lookahead_reports_the_worst_station_it_reaches(band):
    """The point verdict may not see what the horizon will arrive at."""
    worst = min(mpc_speed.horizon_speed(band, p) for p in band.xy[::5])
    point = min(mpc_speed.policy_speed(band, p) for p in band.xy[::5])
    assert worst <= point


def test_hazard_ramp_is_continuous_at_its_ends():
    assert mpc_speed.hazard_speed(float("inf")) == mpc_speed.MAX_SPEED
    assert mpc_speed.hazard_speed(mpc_speed.SLACK_FULL_SPEED_M) == \
        pytest.approx(mpc_speed.MAX_SPEED)
    assert mpc_speed.hazard_speed(mpc_speed.SLACK_CREEP_M) == \
        pytest.approx(mpc_speed.CREEP_SPEED)


def test_the_hazard_ramp_is_inert_on_the_shipped_band(band):
    """Not a requirement - a record. hazard_clearance is finite at 0 of 758
    stations on v4, so the ramp above cannot fire on the route we drive. It
    is faithful to the follower and currently dead. When the band is
    re-measured with real drop semantics this test should start failing,
    and that failure is the good news."""
    finite = sum(1 for p in band.xy if np.isfinite(band.hazard_clearance(p)))
    assert finite == 0, (
        "hazard_clearance is now finite at %d stations - the band carries "
        "drop semantics at last; re-check the speed policy against it and "
        "update this test" % finite)


# ------------------------------------------------- guards are not copied

def follower_ast():
    return ast.parse((SCRIPTS / "mpc_follower.py").read_text("utf-8"))


def method(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


def calls_in(node):
    return {n.func.attr for n in ast.walk(node)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}


def test_mpc_step_runs_the_inherited_guards():
    """The one thing that must never be true of this file is that it decides
    for itself whether the chair may move."""
    assert "handled_before_driving" in calls_in(method(follower_ast(), "step"))


def test_mpc_step_advances_progress_so_the_geofence_arms():
    """route_locked is set in advance_progress, and OFF_ROUTE and OFF_BAND
    are both written to apply only once locked. A follower that skipped it
    would drive with those two guards inert and nothing would say so."""
    assert "advance_progress" in calls_in(method(follower_ast(), "step"))


def test_mpc_follower_does_not_reimplement_the_hold_ladder():
    source = (SCRIPTS / "mpc_follower.py").read_text("utf-8")
    for copied in ("hold_candidates", "evaluate_holds", "WOULD_HOLD"):
        assert copied not in source, (
            "%s appears in mpc_follower - the guards are meant to be "
            "inherited, not restated" % copied)


def test_mpc_follower_subclasses_the_validated_follower():
    tree = follower_ast()
    cls = next(n for n in ast.walk(tree)
               if isinstance(n, ast.ClassDef) and n.name == "MpcFollower")
    assert [b.id for b in cls.bases] == ["WaypointFollower"]


def test_mpc_follower_recovers_sys_path_for_the_devel_relay():
    source = (SCRIPTS / "mpc_follower.py").read_text("utf-8")
    assert ("sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))"
            in source)


# ------------------------------------------------------- bringup switch

def bringup():
    return (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        "utf-8")


def test_profile_defaults_to_the_validated_control_law():
    assert 'PROFILE="${PROFILE:-pursuit}"' in bringup()


def test_profile_rejects_anything_but_the_two_literals():
    text = bringup()
    assert '"$PROFILE" != "pursuit" ]' in text and '"$PROFILE" != "mpc" ]' \
        in text
    assert "PROFILE must be pursuit or mpc" in text


def test_bringup_launches_the_selected_follower():
    text = bringup()
    assert 'rosrun static_livox_localization "$FOLLOWER_NODE"' in text
    assert "FOLLOWER_NODE=mpc_follower.py" in text
    assert "FOLLOWER_NODE=waypoint_follower.py" in text


# --------------------------------------------------- the pursuit refactor

def test_pursuit_step_still_runs_the_guards_through_the_shared_path():
    tree = ast.parse((SCRIPTS / "waypoint_follower.py").read_text("utf-8"))
    assert "handled_before_driving" in calls_in(method(tree, "step"))
    assert "advance_progress" in calls_in(method(tree, "pure_pursuit_target"))


def test_shared_guard_path_still_stops_and_reports():
    tree = ast.parse((SCRIPTS / "waypoint_follower.py").read_text("utf-8"))
    shared = method(tree, "handled_before_driving")
    assert {"send_stop", "publish"} <= calls_in(shared)
