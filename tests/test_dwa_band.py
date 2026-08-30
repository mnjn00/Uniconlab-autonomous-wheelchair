"""The DWA profile: what it may command, and what it must refuse.

The band is enforced as a rollout critic rather than a costmap layer, so
these run against the band's own geometry - there is no grid here to be
wrong about, which is the point of the design.
"""

import json
import math
import re
import sys
from dataclasses import FrozenInstanceError
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"

sys.path.insert(0, str(SCRIPTS))
try:
    import cluster_guard
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


class OpenDrivableMask:
    """A test mask that declares every sampled point physically drivable."""

    def contains_many(self, points):
        return np.ones(len(points), dtype=bool)

    def paths_are_contained(self, paths):
        return np.ones(len(paths), dtype=bool)

    def boundary_cost_many(self, points):
        return np.zeros(len(points), dtype=float)


class BandBoundDrivableMask(OpenDrivableMask):
    """A hard mask whose physical boundary matches the preferred band."""

    def __init__(self, band):
        self.band = band

    def contains_many(self, points):
        return self.band.contains_many(points)

    def paths_are_contained(self, paths):
        return np.array([
            self.band.contains_many(path[:, :2]).all()
            for path in paths
        ])


def narrow_straight_scene(tmp_path, route_mask):
    route = np.array([[index * 0.25, 0.0] for index in range(33)])
    stations = [{
        "x": float(x),
        "y": float(y),
        "heading_deg": 0.0,
        "left_m": 0.22,
        "right_m": 0.22,
    } for x, y in route]
    path = tmp_path / "narrow-band.json"
    path.write_text(json.dumps({"stations": stations}))
    band = SafetyBand(str(path))
    mask = route_mask(band) if callable(route_mask) else route_mask
    return band, route, dwa_core.DwaPlanner(
        band, route, route_mask=mask)


def test_route_centre_requires_clearance_on_each_side(tmp_path):
    stations = []
    for index, (left, right) in enumerate((
        (1.0, 1.0),
        (1.2, 0.3),
        (0.4, 1.2),
        (1.0, 1.0),
    )):
        stations.append({
            "x": float(index),
            "y": 0.0,
            "heading_deg": 0.0,
            "left_m": left,
            "right_m": right,
            "left_kind": "drop",
            "right_kind": "drop",
            "left_drop_m": 0.15,
            "right_drop_m": 0.15,
        })
    path = tmp_path / "band.json"
    path.write_text(json.dumps({"stations": stations}))
    band = SafetyBand(str(path))

    assert band.route_centre_clearance_violations(
        required_side_m=0.45,
        endpoint_guard=1,
    ) == [1, 2]


def test_route_centre_chords_must_stay_inside_band(tmp_path):
    stations = [
        {
            "x": 0.0,
            "y": 0.0,
            "heading_deg": 90.0,
            "left_m": 0.5,
            "right_m": 0.5,
        },
        {
            "x": 1.0,
            "y": 1.0,
            "heading_deg": 0.0,
            "left_m": 0.5,
            "right_m": 0.5,
        },
        {
            "x": 2.0,
            "y": 1.0,
            "heading_deg": 0.0,
            "left_m": 0.5,
            "right_m": 0.5,
        },
    ]
    path = tmp_path / "band.json"
    path.write_text(json.dumps({"stations": stations}))
    band = SafetyBand(str(path))

    assert 0 in band.route_centre_chord_violations(
        endpoint_guard=0,
        spacing=0.1,
    )


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


def test_fixed_speed_cap_yields_one_executable_candidate():
    assert dwa_core.speed_samples(
        max_speed=dwa_core.TURN_FLOOR_SPEED,
    ) == (0.0, dwa_core.TURN_FLOOR_SPEED)


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
    path = dwa_core.rollout(state, v, w, planner.distance_m, planner.steps)
    assert dwa_core.stays_in_band(band, path)


def test_the_whole_rollout_is_tested_not_just_where_it_ends(scene):
    """An arc that ends back inside the corridor having crossed out of it
    partway is not a candidate."""
    band, route, planner = scene
    state = on_route(route, 40)
    v, w, _ = planner.plan(state)
    path = dwa_core.rollout(state, v, w, planner.distance_m, planner.steps)
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
        path = dwa_core.rollout(state, v, w, planner.distance_m, planner.steps)
        assert dwa_core.stays_in_band(band, path)
    else:
        assert (v, w) == (0.0, 0.0)
        assert status in ("OFF_BAND", "OBSTACLE", "NO_CANDIDATE")


def test_an_obstacle_can_force_escape_from_the_preferred_band(tmp_path):
    """The band is preferred, not lethal, when the hard mask says drivable."""
    band, route, planner = narrow_straight_scene(
        tmp_path, OpenDrivableMask())
    state = on_route(route, 4)
    heading = np.array([math.cos(state[2]), math.sin(state[2])])
    blocker = state[:2] + heading * 1.2

    v, w, status = planner.plan(
        state, obstacles=(blocker,), last_speed=0.35)

    assert status == "OK" and v > 0.0
    path = dwa_core.rollout(
        state, v, w, planner.preview_distance(0.35), planner.steps)
    assert not dwa_core.stays_in_band(band, path)


def test_escape_never_overrules_the_hard_drivable_mask(tmp_path):
    """The identical obstacle remains a refusal across a physical boundary."""
    band, route, planner = narrow_straight_scene(
        tmp_path, BandBoundDrivableMask)
    state = on_route(route, 4)
    heading = np.array([math.cos(state[2]), math.sin(state[2])])
    blocker = state[:2] + heading * 1.2

    v, w, status = planner.plan(
        state, obstacles=(blocker,), last_speed=0.35)

    assert (v, w) == (0.0, 0.0)
    assert status in ("OFF_BAND", "OBSTACLE")
    assert band.contains(state[:2])


def test_clear_road_stays_in_the_preferred_band_when_escape_is_open(tmp_path):
    """Finite escape cost must never make ordinary off-band driving normal."""
    band, route, planner = narrow_straight_scene(
        tmp_path, OpenDrivableMask())
    state = on_route(route, 4)

    v, w, status = planner.plan(state, last_speed=0.35)

    assert status == "OK" and v > 0.0
    path = dwa_core.rollout(
        state, v, w, planner.preview_distance(0.35), planner.steps)
    assert dwa_core.stays_in_band(band, path)


# -------------------------------------------------------------- obstacles

def test_an_object_in_the_corridor_is_cleared_or_refused(scene):
    band, route, planner = scene
    state = on_route(route, 40)
    heading = np.array([math.cos(state[2]), math.sin(state[2])])
    blocker = state[:2] + heading * 1.0
    v, w, status = planner.plan(state, obstacles=(blocker,))
    if status == "OK":
        path = dwa_core.rollout(
            state, v, w, planner.distance_m, planner.steps)
        assert dwa_core.obstacle_clearance(path, (blocker,)) >= \
            dwa_core.OBSTACLE_FLOOR_M
    else:
        assert (v, w) == (0.0, 0.0)


def test_clearance_is_infinite_with_nothing_to_clear():
    path = dwa_core.rollout(np.array([0.0, 0.0, 0.0]), 0.5, 0.0)
    assert not np.isfinite(dwa_core.obstacle_clearance(path, ()))


def test_actuator_state_and_proposal_are_immutable_json_contracts(scene):
    _band, route, planner = scene
    state = on_route(route, 200)
    actuator = dwa_core.ActuatorState(
        speed_mps=0.0, yaw_rate_rps=0.0, acceleration_mps2=0.0)
    assert actuator == dwa_core.ActuatorState.from_json(actuator.to_json())

    result = planner.plan(
        state,
        speed_cap=dwa_core.TURN_FLOOR_SPEED,
        actuator_state=actuator,
        committed_side="LEFT",
        proposal_seq=41,
        stamp_s=123.5,
        permit_track_id=8,
        latency_s=0.55,
        return_proposal=True,
    )

    assert result[2] == "OK"
    proposal = result[3]
    assert proposal.frame_id == "current_body"
    assert proposal.actuator_state == actuator
    assert proposal.distance_m == planner.preview_distance(None)
    assert proposal.latency_s == 0.55
    assert sum(proposal.time_steps_s[:6]) == 0.55
    assert proposal.time_steps_s[5] == 0.05
    assert proposal.horizon_s == pytest.approx(sum(proposal.time_steps_s))
    assert proposal.target_yaw_rate_rps > 0.0
    assert proposal.first_applied_speed_mps == proposal.speeds_mps[6]
    assert proposal.first_applied_yaw_rate_rps == proposal.yaw_rates_rps[6]
    assert proposal == dwa_core.TrajectoryProposal.from_json(
        proposal.to_json())
    wrong_frame = json.loads(proposal.to_json())
    wrong_frame["frame_id"] = "map"
    with pytest.raises(dwa_core.ProposalValidationError):
        dwa_core.TrajectoryProposal.from_json(json.dumps(wrong_frame))
    with pytest.raises(FrozenInstanceError):
        proposal.target_speed_mps = 0.0


def test_proposal_json_rejects_non_finite_and_mismatched_series():
    payload = {
        "schema": "wheelchair.trajectory_proposal/v1",
        "proposal_seq": 1,
        "stamp_s": 2.0,
        "permit_track_id": 3,
        "committed_side": "LEFT",
        "frame_id": "body",
        "horizon_s": 0.1,
        "actuator_state": {
            "schema": "wheelchair.actuator_state/v1",
            "speed_mps": 0.0,
            "yaw_rate_rps": 0.0,
            "acceleration_mps2": 0.0,
            "control_step_s": 0.1,
        },
        "target_speed_mps": 0.35,
        "target_yaw_rate_rps": 0.2,
        "first_applied_speed_mps": float("nan"),
        "first_applied_yaw_rate_rps": 0.1,
        "step_s": 0.1,
        "poses": [[0.0, 0.0, 0.0]],
        "speeds_mps": [float("nan")],
        "yaw_rates_rps": [],
    }

    with pytest.raises(dwa_core.ProposalValidationError):
        dwa_core.TrajectoryProposal.from_json(json.dumps(payload))
    with pytest.raises(dwa_core.ProposalValidationError):
        dwa_core.ActuatorState.from_json(json.dumps({
            "schema": "wheelchair.actuator_state/v1",
            "speed_mps": float("nan"),
            "yaw_rate_rps": 0.0,
            "acceleration_mps2": 0.0,
            "control_step_s": 0.1,
        }))


def test_proposal_mode_requires_identity_metadata(scene):
    _band, route, planner = scene

    result = planner.plan(
        on_route(route, 200),
        actuator_state=dwa_core.ActuatorState(0.0, 0.0, 0.0),
        return_proposal=True,
    )

    assert result == (0.0, 0.0, "ACTUATOR_STATE_INVALID", None)


@pytest.mark.parametrize("series_name,index,delta", [
    ("poses", (3, 1), 1e-4),
    ("speeds_mps", (3,), 1e-4),
    ("yaw_rates_rps", (3,), 1e-4),
    ("time_steps_s", (3,), 1e-4),
])
def test_proposal_rejects_any_finite_rollout_sample_change(
        scene, series_name, index, delta):
    _band, route, planner = scene
    result = planner.plan(
        on_route(route, 200),
        actuator_state=dwa_core.ActuatorState(0.2, 0.15, 0.0),
        committed_side="LEFT",
        proposal_seq=42,
        stamp_s=124.0,
        permit_track_id=8,
        latency_s=0.55,
        return_proposal=True,
    )
    assert result[2] == "OK"
    payload = json.loads(result[3].to_json())
    target = payload[series_name]
    if len(index) == 2:
        target[index[0]][index[1]] += delta
    else:
        target[index[0]] += delta
        if series_name == "time_steps_s":
            payload["horizon_s"] += delta

    with pytest.raises(dwa_core.ProposalValidationError):
        dwa_core.TrajectoryProposal.from_json(json.dumps(payload))


def test_latency_lead_path_blocks_before_candidate_ramp(scene):
    band, route, _planner = scene
    planner = dwa_core.DwaPlanner(
        band, route, route_mask=OpenDrivableMask())
    state = on_route(route, 200)
    actuator = dwa_core.ActuatorState(0.35, 0.5, 0.0)
    lead, _speeds, _yaw, _steps = dwa_core.rollout_actuation_timed(
        dwa_core.RolloutSpec(
            pose=tuple(state[:3]),
            target_speed_mps=0.35,
            target_yaw_rate_rps=0.5,
            actuator_state=actuator,
            distance_m=planner.distance_m,
            latency_s=0.55,
        ))
    blocker = lead[5][:2]

    result = planner.plan(
        state, obstacles=(blocker,),
        obstacle_floor_m=0.04, actuator_state=actuator,
        proposal_seq=43, stamp_s=125.0, permit_track_id=9,
        latency_s=0.55, return_proposal=True)

    assert result == (0.0, 0.0, "OBSTACLE", None)


def test_stopped_latency_keeps_first_yaw_zero_and_target_side(scene):
    _band, route, planner = scene

    result = planner.plan(
        on_route(route, 200),
        actuator_state=dwa_core.ActuatorState(0.0, 0.0, 0.0),
        committed_side="LEFT", minimum_turn_rps=0.08,
        proposal_seq=44, stamp_s=126.0, permit_track_id=10,
        latency_s=0.55, return_proposal=True)

    assert result[2] == "OK"
    assert result[3].yaw_rates_rps[0] == 0.0
    assert result[3].target_yaw_rate_rps >= 0.08


def test_stopped_latency_exposes_first_executable_command(scene):
    _band, route, planner = scene

    # Given a stopped chair whose validated proposal includes latency carry.
    result = planner.plan(
        on_route(route, 200),
        actuator_state=dwa_core.ActuatorState(0.0, 0.0, 0.0),
        committed_side="LEFT", minimum_turn_rps=0.08,
        proposal_seq=45, stamp_s=127.0, permit_track_id=10,
        latency_s=0.55, return_proposal=True)

    # Then the published command is the first ramp sample after the six carry
    # samples, while those carry samples remain in the validated trajectory.
    assert result[2] == "OK"
    proposal = result[3]
    assert proposal.speeds_mps[:6] == (0.0,) * 6
    assert proposal.first_applied_speed_mps == proposal.speeds_mps[6]
    assert proposal.first_applied_speed_mps > 0.0


def test_actuated_rollout_scores_the_applied_not_instant_target(scene):
    _band, route, planner = scene
    state = on_route(route, 200)
    actuator = dwa_core.ActuatorState(0.0, 0.0, 0.0)

    v, w, status, proposal = planner.plan(
        state,
        speed_cap=dwa_core.TURN_FLOOR_SPEED,
        actuator_state=actuator,
        proposal_seq=9,
        stamp_s=10.0,
        permit_track_id=4,
        return_proposal=True,
    )

    assert status == "OK"
    assert (v, w) == (proposal.target_speed_mps,
                      proposal.target_yaw_rate_rps)
    assert proposal.first_applied_speed_mps < proposal.target_speed_mps
    poses, speeds, yaw_rates = dwa_core.rollout_actuation(
        dwa_core.RolloutSpec(
            pose=tuple(state[:3]),
            target_speed_mps=v,
            target_yaw_rate_rps=w,
            actuator_state=actuator,
            distance_m=planner.preview_distance(None),
        ))
    heading = state[2]
    cosine, sine = math.cos(heading), math.sin(heading)
    relative = tuple((
        cosine * (x - state[0]) + sine * (y - state[1]),
        -sine * (x - state[0]) + cosine * (y - state[1]),
        math.atan2(math.sin(yaw - heading), math.cos(yaw - heading)),
    ) for x, y, yaw in poses)
    assert np.allclose(
        np.asarray(proposal.poses), np.asarray(relative), atol=1e-9)
    assert proposal.speeds_mps == speeds
    assert proposal.yaw_rates_rps == yaw_rates


def test_actuator_rollout_slews_before_reaching_target_yaw():
    poses, speeds, yaw_rates = dwa_core.rollout_actuation(
        dwa_core.RolloutSpec(
            pose=(0.0, 0.0, 0.0),
            target_speed_mps=0.35,
            target_yaw_rate_rps=0.5,
            actuator_state=dwa_core.ActuatorState(0.0, 0.0, 0.0),
            distance_m=0.1,
        ))

    assert len(poses) == len(speeds) == len(yaw_rates)
    assert speeds[0] == pytest.approx(0.008)
    assert yaw_rates[0] == 0.0
    assert yaw_rates[0] < 0.5
    assert max(yaw_rates) == pytest.approx(0.5)


def test_identical_applied_candidate_trajectories_are_deduplicated(scene):
    _band, route, planner = scene
    state = on_route(route, 200)
    pairs = [
        (dwa_core.TURN_FLOOR_SPEED, yaw)
        for _duplicate in range(5)
        for yaw in dwa_core.yaw_samples()
    ]

    unique, paths, full_paths, body_paths, speeds, yaw_rates, steps, travelled = \
        planner._candidate_rollouts(
            state, pairs, planner.distance_m,
            dwa_core.ActuatorState(0.0, 0.0, 0.0))

    assert len(unique) == len(paths) == len(full_paths) == len(body_paths) == \
        len(speeds) == len(yaw_rates) == len(steps) == len(travelled) == \
        len(dwa_core.yaw_samples())


@pytest.mark.parametrize("actuator", [
    dwa_core.ActuatorState(0.0, 0.0, 0.0),
    dwa_core.ActuatorState(0.8, 0.0, 0.18),
])
def test_actuated_rollout_reaches_scoring_span_from_rest_or_deceleration(
        actuator):
    span = 1.05
    poses, _speeds, _yaw_rates = dwa_core.rollout_actuation(
        dwa_core.RolloutSpec(
            pose=(0.0, 0.0, 0.0),
            target_speed_mps=0.35,
            target_yaw_rate_rps=0.2,
            actuator_state=actuator,
            distance_m=span,
        ))
    points = np.vstack((np.zeros((1, 2)), np.asarray(poses)[:, :2]))
    travelled = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()

    assert travelled >= span
    assert travelled < span + dwa_core.MAX_SPEED * actuator.control_step_s


def test_body_proposal_reconstructs_nonzero_world_start(scene):
    _band, route, planner = scene
    state = on_route(route, 300)
    actuator = dwa_core.ActuatorState(0.2, 0.1, 0.0)
    _v, _w, status, proposal = planner.plan(
        state,
        speed_cap=0.35,
        actuator_state=actuator,
        proposal_seq=80,
        stamp_s=90.0,
        permit_track_id=12,
        latency_s=0.55,
        return_proposal=True,
    )

    assert status == "OK"
    cosine, sine = math.cos(state[2]), math.sin(state[2])
    world = np.asarray([(
        state[0] + cosine * x - sine * y,
        state[1] + sine * x + cosine * y,
        state[2] + yaw,
    ) for x, y, yaw in proposal.poses])
    direct, _speeds, _yaw_rates = dwa_core.rollout_actuation(
        dwa_core.RolloutSpec(
            pose=tuple(state[:3]),
            target_speed_mps=proposal.target_speed_mps,
            target_yaw_rate_rps=proposal.target_yaw_rate_rps,
            actuator_state=actuator,
            distance_m=planner.preview_distance(None),
            latency_s=0.55,
        ))

    assert np.allclose(world, np.asarray(direct), atol=1e-9)


def test_obstacle_preview_extends_from_actual_travelled_distance(scene):
    _band, _route, planner = scene
    pairs, _paths, full_paths, _body, _speeds, _yaw_rates, _steps, travelled = \
        planner._candidate_rollouts(
            np.zeros(3), [(0.35, 0.0)], planner.distance_m,
            dwa_core.ActuatorState(0.0, 0.0, 0.0))

    watched = planner._obstacle_paths(
        full_paths, travelled, dwa_core.OBSTACLE_PREVIEW_M)

    assert pairs == [(0.35, 0.0)]
    assert watched[0, -1, 0] >= dwa_core.OBSTACLE_PREVIEW_M - 1e-6


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


# ------------------------------------------- keeping the middle of the band

def test_the_margins_match_the_containment_they_are_taken_from(scene):
    """One copy of the band rules. The planner's centring term reads the
    same station lookup and normals that decide containment."""
    band, route, planner = scene
    pts = np.array([route[k] for k in range(50, 1500, 137)])
    lateral, lo, hi = band.margins_many(pts)
    inside = band.contains_many(pts)
    assert np.array_equal(inside, (lateral >= lo - 1e-6) & (lateral <= hi + 1e-6))
    for point, lat, low, high in zip(pts, lateral, lo, hi):
        s_lat, s_lo, s_hi = band.lateral_limits(point)
        assert abs(s_lat - lat) < 1e-9
        assert abs(s_lo - low) < 1e-9 and abs(s_hi - high) < 1e-9


def test_margins_of_nothing_is_nothing(scene):
    band, _route, _planner = scene
    lateral, lo, hi = band.margins_many(np.zeros((0, 2)))
    assert len(lateral) == len(lo) == len(hi) == 0
    assert len(band.contains_many(np.zeros((0, 2)))) == 0


def edge_fraction(band, path):
    lateral, lo, hi = band.margins_many(path[:, :2])
    return np.abs(lateral - (hi + lo) / 2.0) / np.maximum((hi - lo) / 2.0, 1e-6)


def test_the_middle_of_the_corridor_is_cheaper_than_its_edge(scene):
    """Containment is a hard reject, so without a price on the margin the
    middle and a hair inside the edge score identically. On 2026-08-09 the
    chair settled at -0.12 m and a bend put it 6 mm outside 0.58 m of room."""
    band, route, planner = scene
    for k in (200, 700, 1400):
        state = on_route(route, k)
        v, w, status = planner.plan(state)
        assert status == "OK"
        chosen = edge_fraction(
            band, dwa_core.rollout(
                state, v, w, planner.distance_m, planner.steps))
        assert chosen.max() < 0.9


def test_a_chair_riding_the_edge_is_steered_back_towards_the_middle(scene):
    """Not merely kept legal - actively recentred, which is the whole point
    of pricing the margin rather than only rejecting the excursion."""
    band, route, planner = scene
    k = 700
    lateral, lo, hi = band.margins_many(route[k:k + 1])
    normal = np.array([-(route[k + 3] - route[k])[1],
                       (route[k + 3] - route[k])[0]])
    normal = normal / np.linalg.norm(normal)
    state = on_route(route, k)
    state[:2] = route[k] + normal * (hi[0] * 0.8)      # 80 % of the way out
    before = edge_fraction(band, state[None, :2])[0]
    v, w, status = planner.plan(state)
    assert status == "OK" and v > 0.0
    after = edge_fraction(
        band, dwa_core.rollout(
            state, v, w, planner.distance_m, planner.steps))
    assert after[-1] < before


def test_the_centring_term_is_priced_superlinearly():
    """Linear would bias the whole drive without ever making the last few
    centimetres unaffordable, which is the excursion that matters."""
    src = (SCRIPTS / "dwa_core.py").read_text(encoding="utf-8")
    assert "W_CENTRE" in src
    assert "np.square" in src


# ------------------------------------------ where the threat actually is

def test_a_threat_beside_the_chair_is_not_placed_in_front_of_it():
    """cluster_guard hands the follower a distance; without the lateral
    offset the only thing it can do is put the object on the heading. On
    2026-08-09 that turned a wall 0.70 m away at the side into a phantom in
    the corridor and held the profile for 211 consecutive cycles."""
    threat = cluster_guard.Threat(0.7, cluster_guard.MOVING, "wall",
                                  lateral_m=0.6)
    assert threat.lateral_m == 0.6

    state = np.array([10.0, 5.0, 0.0, 0.0, 0.0])       # facing +x
    heading = np.array([math.cos(state[2]), math.sin(state[2])])
    left = np.array([-heading[1], heading[0]])
    placed = state[:2] + heading * threat.distance_m + left * threat.lateral_m
    assert placed[0] == pytest.approx(10.7)
    assert placed[1] == pytest.approx(5.6)
    # and it no longer sits on the chair's own line of travel
    assert abs(placed[1] - state[1]) > dwa_core.OBSTACLE_FLOOR_M


def test_an_unparseable_threat_keeps_the_conservative_frontal_placement():
    threat = cluster_guard.Threat(0.7, cluster_guard.MOVING, "?")
    assert threat.lateral_m is None


def test_the_follower_places_every_return_where_it_is():
    """The 2026-08-09 fix, now taken from the profile rather than the box.

    It used to read threat.lateral_m - one object, one place. cluster_guard
    publishes the lateral slice each return falls in, so the follower places
    all of them, and a wall beside the chair arrives beside the chair along
    its whole length instead of at one point on it.
    """
    src = (SCRIPTS / "dwa_follower.py").read_text(encoding="utf-8")
    assert "corridor_obstacle_points(" in src
    assert "heading * forward + left * lateral" in src

    wall = {"class": "obstacle", "x": 1.0, "y": 0.9, "size": [2.0, 1.0, 1.2],
            "points": 80, "motion": cluster_guard.STATIC}
    summary = cluster_guard.parse_summary(json.dumps(
        {"stamp": 100.0, "status": "OK", "objects": [wall]}))
    blocks, points = cluster_guard.corridor_obstacle_points(summary, 1.0)

    assert blocks and len(points) > 1
    # Beside the chair, not on its line of travel, and along its length.
    assert all(lateral > 0.0 for _forward, lateral in points)
    assert max(f for f, _ in points) - min(f for f, _ in points) \
        <= 1e-9, "a box's near face is one distance across its width"


# ------------------------------------------- what the cycle spends itself on

def test_the_band_geometry_is_walked_once_per_plan(scene):
    """The control period is 100 ms and one pass over the band cost 24.4 ms
    of it on the target NUC, for 1,785 rollout points against 802 stations.
    plan() needs the same margins twice - to reject the arcs that leave the
    corridor, and to score how near its edge the rest of them run - and it
    used to ask for them twice, which was 96 % of the cycle computing one
    answer two ways.

    Counted rather than timed: a timing assertion on a loaded NUC is a flaky
    test, and the property that matters is not how fast the search is but
    how many times it is run.
    """
    band, route, planner = scene
    calls = []
    real = band.margins_many

    def counting(points):
        calls.append(len(points))
        return real(points)

    band.margins_many = counting
    try:
        v, w, status = planner.plan(on_route(route, 400))
    finally:
        band.margins_many = real

    assert status == "OK" and v > 0.0
    assert len(calls) == 1, \
        "the band was searched %d times for one command" % len(calls)


def test_straight_corridor_does_not_alternate_steering_sign(scene):
    _band, route, planner = scene
    commands = []
    for index in range(0, len(route) - 3, 5):
        _v, yaw_rate, status = planner.plan(on_route(route, index))
        assert status == "OK"
        if abs(yaw_rate) >= 0.05:
            commands.append(int(math.copysign(1, yaw_rate)))
    reversals = sum(
        previous != current
        for previous, current in zip(commands, commands[1:])
    )
    route_length_m = np.linalg.norm(np.diff(route, axis=0), axis=1).sum()
    assert reversals / route_length_m <= 0.02


def test_the_shared_verdict_is_the_same_one_contains_many_gives(scene):
    """contained() is the threshold contains_many applies, lifted out so a
    caller that also wants the margins does not pay for them twice. If the
    two ever disagree, the planner is rejecting arcs on a different rule
    from the one perception and the follower use."""
    band, route, _planner = scene
    points = np.array([
        band.xy[k] + band.normals[k] * offset
        for k in range(0, len(band.xy), 7)
        for offset in (-3.0, -0.5, -0.05, 0.0, 0.05, 0.5, 3.0)])

    for grace in (0.0, 0.10, 0.5):
        shared = band.contained(*band.margins_many(points), grace=grace)
        assert np.array_equal(shared, band.contains_many(points, grace=grace))
    # and it really is discriminating, not vacuously all-True
    verdict = band.contains_many(points)
    assert verdict.any() and not verdict.all()
