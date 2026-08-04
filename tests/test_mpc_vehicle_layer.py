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
    import mpc_core
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


def test_corridor_shaping_slows_for_the_narrowest_metre(band):
    """The route's tightest station is infeasible at 0.5 m/s and solves at
    0.4 (measured). The chair must arrive there at or below that."""
    widths = np.array([band.lateral_limits(q)[2] - band.lateral_limits(q)[1]
                       for q in band.xy])
    tightest = band.xy[int(np.argmin(widths))]
    assert mpc_speed.corridor_speed(band, tightest) <= 0.4


def test_corridor_shaping_slows_before_arriving_not_on_arrival(band):
    """A pinch must be seen from far enough back that the chair is already
    slow when the pinch enters the horizon."""
    widths = np.array([band.lateral_limits(q)[2] - band.lateral_limits(q)[1]
                       for q in band.xy])
    k = int(np.argmin(widths))
    approach = band.xy[max(0, k - 20)]          # 10 m back
    assert mpc_speed.corridor_speed(band, approach) < mpc_speed.MAX_SPEED


def test_corridor_shaping_leaves_open_road_alone(band):
    """It must not be a blanket slowdown: the wide majority of the route
    still runs at full speed, or this is just a slower chair."""
    full = sum(1 for q in band.xy[::5]
               if mpc_speed.corridor_speed(band, q) >= mpc_speed.MAX_SPEED)
    assert full > 0.5 * len(band.xy[::5])


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


# ------------------------------------- the reference the solver is given

def test_heading_reference_never_demands_more_yaw_than_the_chair_has(band):
    """The regression that cost the route.

    polyline_refs used to snap the heading to whichever polyline segment a
    step landed on. Stations are 0.5 m apart and a step covers 0.06 m, so
    eight steps shared a segment and the ninth inherited the whole
    inter-station heading change - up to 26.7 degrees, read by the cost as
    a demand for 4.7 rad/s against a 0.5 rad/s cap. The chair spent its yaw
    authority chasing a corner that was not there, drifted, and met the
    hard band rows as an INFEASIBLE_STOP at 350 m that looked for all the
    world like a corridor too narrow to drive.
    """
    params = mpc_core.MpcParams()
    worst = 0.0
    for k in range(0, len(band.xy), 5):
        # at the speed the policy would actually be running here, since the
        # curvature cap is the half of the fix that lives in mpc_speed
        v_ref, stop = mpc_speed.shaped_reference(band, band.xy[k],
                                                 params.horizon)
        if stop:
            continue
        _v, th = mpc_core.polyline_refs(band, band.xy[k], params.horizon,
                                        params.dt, float(v_ref[0]))
        if len(th) > 1:
            worst = max(worst, float(np.abs(np.diff(th)).max()) / params.dt)
    assert worst <= params.w_max + 1e-9, (
        "heading reference demands %.3f rad/s, cap is %.2f" % (worst,
                                                               params.w_max))


def test_curvature_cap_is_rare_rather_than_a_blanket_slowdown(band):
    """Only two stations of 756 demand more yaw than the chair has. If this
    starts biting everywhere, the curvature estimate has gone noisy and the
    chair is being slowed for nothing."""
    capped = sum(1 for q in band.xy[::5]
                 if mpc_speed.curvature_speed(band, q) < mpc_speed.MAX_SPEED)
    assert capped < 0.25 * len(band.xy[::5])


def test_heading_reference_still_turns_the_real_corner(band):
    """Smoothing would also have passed the test above, by rounding off the
    genuine 71-degree turn near 372 m. Interpolation must not: over the
    corner the reference has to actually sweep through it."""
    seg = np.diff(band.xy, axis=0)
    heading = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(seg, axis=1))])
    k = int(np.argmax(np.abs(np.diff(heading))))
    params = mpc_core.MpcParams()
    # from the corner itself: at 0.6 m/s the horizon reaches 1.5 m, so
    # starting further back would simply not arrive at the bend
    _v, th = mpc_core.polyline_refs(band, band.xy[k], params.horizon,
                                    params.dt, 0.6)
    swept = abs(float(th[-1] - th[0]))
    assert swept > math.radians(15.0), (
        "reference sweeps only %.1f degrees through the corner at %.0f m"
        % (math.degrees(swept), arc[k]))


def test_near_horizon_reserve_cannot_exclude_where_the_chair_already_is():
    """The 335 m stop, pinned.

    The reserve used to be flat across the horizon. Step 1 is 0.03 m ahead
    with 0.0 mm of measured error, and a flat 52 mm reserve there fenced off
    ground the chair was standing on - while being the step it has the least
    time to maneuver out of. Relaxing steps 0-4 restored feasibility;
    relaxing any later group did not.
    """
    p = mpc_core.MpcParams()
    near = min(max(p.band_inset * 1 / p.horizon, p.band_inset_min),
               p.band_inset)
    far = min(max(p.band_inset * p.horizon / p.horizon, p.band_inset_min),
              p.band_inset)
    assert near <= 0.015, "near-horizon reserve is %.3f m" % near
    assert far >= 0.05, "far-horizon reserve collapsed to %.3f m" % far
    assert near < far


def test_reserve_never_falls_under_the_measured_one_step_error():
    """It is a reserve against a measured quantity, not a free parameter:
    4.5 mm was the worst one-step lateral error over a full run."""
    assert mpc_core.MpcParams().band_inset_min >= 0.0045


def test_linearisation_reserve_leaves_the_tightest_station_drivable(band):
    """The reserve is a numerical allowance, not a safety margin, and at
    25 % per side it took half of the 0.13 m pinch and made it infeasible
    by 3.5 mm. Measured error at the speed the chair passes it is 4.5 mm."""
    params = mpc_core.MpcParams()
    widths = np.array([band.lateral_limits(q)[2] - band.lateral_limits(q)[1]
                       for q in band.xy])
    tightest = float(widths.min())
    inset = min(params.band_inset, tightest * params.band_inset_fraction)
    assert tightest - 2 * inset >= 0.09, (
        "reserve leaves only %.3f m of the %.3f m pinch"
        % (tightest - 2 * inset, tightest))
    assert inset >= 0.0045 * 2, "reserve is under 2x the measured error"


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
