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
import re
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

def shipped_band():
    """The band the bringup actually launches with, read from the bringup.

    This used to be a hard-coded v4 path, and on 2026-08-04 the bringup
    moved to v5 while the path here did not. Nothing failed - which is the
    problem. Every corridor assertion below would have gone on passing
    against a band the chair had stopped driving, and the speed policy they
    check is tuned to corridor widths that differ between the two by a
    factor of three at the pinch. A test that pins the wrong artifact is
    worse than no test: it reports a guard that is not guarding anything.
    """
    text = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8")
    match = re.search(r'^BAND="\$\{BAND:-.*?/routes/(\S+?)\}"', text, re.M)
    assert match, "cannot tell which band the bringup ships"
    path = ROOT / "routes" / match.group(1)
    assert path.exists(), "bringup ships a band that is not in the repo: %s" \
        % match.group(1)
    return path


BAND = shipped_band()


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


def test_a_thirteen_centimetre_corridor_is_entered_at_the_measured_speed():
    """The measurement is about width, not about a route: on the v4 band's
    0.13 m pinch, 0.5 m/s was infeasible and 0.4 solved. v5 has no corridor
    that tight, but the ramp must still honour the number if one appears -
    a band is a deployment choice and has already changed once."""
    assert mpc_speed.speed_for_width(0.13) <= 0.4


def test_the_shipped_band_is_entered_slower_where_it_is_tightest(band):
    """Whatever ships, the tightest place on it must not be taken at cruise."""
    widths = np.array([band.lateral_limits(q)[2] - band.lateral_limits(q)[1]
                       for q in band.xy])
    tightest = band.xy[int(np.argmin(widths))]
    assert mpc_speed.corridor_speed(band, tightest) < mpc_speed.MAX_SPEED


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


def test_latency_is_settable_from_the_bringup_and_defaults_to_zero():
    """The runbook tells the operator to set LATENCY_S once L is measured.
    That instruction is only true while the bringup actually forwards it -
    the node reads ~latency_s, which nothing else sets."""
    text = bringup()
    assert 'LATENCY_S="${LATENCY_S:-0}"' in text
    assert '_latency_s:="$LATENCY_S"' in text
    assert "LATENCY_S must be a non-negative number" in text


def runbook():
    return (ROOT / "docs" / "runbooks" / "mpc-profile-ko.md").read_text(
        "utf-8")


def test_runbook_calls_the_bringup_where_it_is_actually_installed():
    """push_to_nuc installs the bringup to $HOME on the NUC, not into a
    checkout. A runbook that says ./tools/... sends the operator to a path
    that does not exist on the machine they are typing at."""
    assert "~/start_wheelchair_localization.sh" in runbook()
    assert "./tools/start_wheelchair_localization.sh" not in runbook()


def test_runbook_passes_the_map_directory_push_to_nuc_requires():
    """push_to_nuc.sh takes the map directory as a required argument."""
    for line in runbook().splitlines():
        if "push_to_nuc.sh" in line and not line.startswith(("#", ">")):
            assert re.search(r"push_to_nuc\.sh\s+\S", line), (
                "runbook calls push_to_nuc.sh with no map directory: %s"
                % line.strip())


def test_runbook_does_not_tell_anyone_to_rosrun_the_node_directly():
    """The node needs ~route, ~safety_band and ~body_frame_profile and
    raises without them, so a bare rosrun just dies - and takes none of the
    rest of the stack up with it."""
    assert "rosrun static_livox_localization mpc_follower.py" not in runbook()


def test_runbook_warns_off_the_policies_off_trial_script():
    """trial_0727.sh brings the stack up with SAFETY_POLICIES=false. That is
    the right configuration for measuring localisation and the wrong one for
    the first run of an unvalidated controller."""
    text = runbook()
    assert "trial_0727.sh" in text and "SAFETY_POLICIES=false" in text


def test_bringup_launches_the_selected_follower():
    text = bringup()
    assert 'rosrun static_livox_localization "$FOLLOWER_NODE"' in text
    assert "FOLLOWER_NODE=mpc_follower.py" in text
    assert "FOLLOWER_NODE=waypoint_follower.py" in text


# ------------------------------------------------ which law is driving

def pursuit_source():
    return (SCRIPTS / "waypoint_follower.py").read_text("utf-8")


def class_attribute(source, class_name, attribute):
    """The literal a class assigns to a bare class-level name."""
    tree = ast.parse(source)
    cls = next(n for n in ast.walk(tree)
               if isinstance(n, ast.ClassDef) and n.name == class_name)
    for node in cls.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == attribute
                for t in node.targets):
            return ast.literal_eval(node.value)
    return None


def test_each_control_law_declares_which_one_it_is():
    assert class_attribute(
        pursuit_source(), "WaypointFollower", "CONTROL_LAW") == "pursuit"
    assert class_attribute(
        (SCRIPTS / "mpc_follower.py").read_text("utf-8"),
        "MpcFollower", "CONTROL_LAW") == "mpc"


def test_the_two_laws_do_not_answer_to_the_same_name():
    """The whole point is telling them apart. If a refactor ever made these
    equal, every check downstream would pass on the wrong controller and
    report success."""
    pursuit = class_attribute(
        pursuit_source(), "WaypointFollower", "CONTROL_LAW")
    mpc = class_attribute((SCRIPTS / "mpc_follower.py").read_text("utf-8"),
                          "MpcFollower", "CONTROL_LAW")
    assert pursuit != mpc


def test_the_identity_comes_from_the_class_not_the_launcher():
    """set_param must read the attribute, not a parameter or a literal.

    A launcher-asserted identity is what PLANNER=priest was: the preflight
    compared a shell variable against itself and kept passing after the
    planner it named had been reverted. Reading self.CONTROL_LAW means the
    class that implements the law is the one that names it, so a subclass
    cannot inherit the wrong answer.
    """
    assert 'rospy.set_param("~control_law", self.CONTROL_LAW)' \
        in pursuit_source()


def test_the_mpc_follower_does_not_publish_its_identity_separately():
    """It overrides the attribute and inherits the publishing. A second
    set_param call would be a second place to forget."""
    assert "set_param" not in (SCRIPTS / "mpc_follower.py").read_text("utf-8")


def go_mpc():
    return (ROOT / "tools" / "go_mpc.sh").read_text("utf-8")


def test_go_mpc_refuses_a_law_that_is_not_mpc():
    text = go_mpc()
    assert "rosparam get /waypoint_follower/control_law" in text
    assert '[ "$LAW" = "mpc" ] || fail' in text


def test_go_mpc_refuses_rather_than_defaulting_when_the_param_is_absent():
    """An older follower publishes no identity. Reading that as 'probably
    fine' would defeat the check on exactly the stack it exists to catch."""
    text = go_mpc()
    assert re.search(r'rosparam get /waypoint_follower/control_law[^\n]*\)"'
                     r'\s*\|\|\s*fail', text), \
        "go_mpc.sh must fail, not default, when the identity is missing"


def test_go_mpc_does_not_restate_the_checks_go_sh_already_owns():
    """Same rule the MPC follower follows for the hold ladder: a second copy
    of a guard drifts from the first, always toward the one that starts."""
    text = go_mpc()
    for owned in ("objects_summary", "localization_diagnostics",
                  "/waypoint_follower/start", "mode_cmd"):
        assert owned not in text, (
            "%s is go.sh's check - go_mpc.sh must delegate, not copy" % owned)
    assert 'exec "$SCRIPT_DIR/go.sh"' in text


def test_go_mpc_is_installed_onto_the_nuc():
    """It is typed at $HOME on the NUC like the rest of the bringup, so
    push_to_nuc has to carry it or the operator finds an empty path."""
    text = (ROOT / "tools" / "push_to_nuc.sh").read_text("utf-8")
    assert "go_mpc.sh" in text


# --------------------------------------------------- the pursuit refactor

def test_the_handoff_doc_names_tests_that_exist():
    """docs/mpc_handoff.md hands the next person a table of invariants and,
    for each one, the test that pins it. That table is only worth anything
    while the names in it resolve - a renamed test would leave a reader
    (or an agent) chasing a guard that appears to be enforced and is not.
    """
    doc = (ROOT / "docs" / "mpc_handoff.md").read_text(encoding="utf-8")
    cited = set(re.findall(r"`(test_[a-z0-9_]+)`", doc))
    assert cited, "the handoff doc cites no tests at all - did it move?"
    # a citation resolves either to a test function in this file or to a
    # test module - the doc names both, and both can be renamed away
    known = set(re.findall(r"^def (test_[a-z0-9_]+)",
                           Path(__file__).read_text(encoding="utf-8"), re.M))
    known |= {p.stem for p in (ROOT / "tests").glob("test_*.py")}
    missing = sorted(cited - known)
    assert not missing, (
        "docs/mpc_handoff.md cites tests that no longer exist: %s" % missing)


def test_pursuit_step_still_runs_the_guards_through_the_shared_path():
    tree = ast.parse((SCRIPTS / "waypoint_follower.py").read_text("utf-8"))
    assert "handled_before_driving" in calls_in(method(tree, "step"))
    assert "advance_progress" in calls_in(method(tree, "pure_pursuit_target"))


def test_shared_guard_path_still_stops_and_reports():
    tree = ast.parse((SCRIPTS / "waypoint_follower.py").read_text("utf-8"))
    shared = method(tree, "handled_before_driving")
    assert {"send_stop", "publish"} <= calls_in(shared)
