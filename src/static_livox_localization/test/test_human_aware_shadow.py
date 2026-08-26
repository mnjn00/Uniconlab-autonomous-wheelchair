"""Human-aware shadow policy over bag-derived tracked-person sequences."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).parents[1] / "scripts"


def load_shadow():
    """Load the shadow policy without requiring a ROS environment."""
    path = SCRIPTS / "human_aware_shadow.py"
    assert path.exists(), \
        "no human-aware shadow planner exists for the recorded person"
    spec = importlib.util.spec_from_file_location("human_aware_shadow", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def person(module, stamp, motion="static", speed_mps=0.0):
    return module.PersonObservation(
        track_id=1641,
        observed_stamp_s=stamp,
        motion=motion,
        speed_mps=speed_mps,
        forward_m=2.42,
        lateral_m=0.0,
        half_length_m=0.35,
        half_width_m=0.35,
        directly_observed=True,
        geometry_valid=True,
    )


def test_recorded_stationary_person_shadow_plan_stays_committed_through_motion_noise():
    # Given: id=1641 stayed STATIC for 28.2 s in the 220341 bag.
    shadow = load_shadow()
    conditioner = shadow.HumanAwareConditioner()
    snapshots = [
        conditioner.update(stamp, [person(shadow, stamp)])
        for stamp in [100.0 + 0.2 * index for index in range(51)]
    ]

    # When: one bounded tracker frame calls the same standing person MOVING.
    snapshots.append(conditioner.update(
        110.2, [person(shadow, 110.2, motion="moving", speed_mps=0.22)]))
    snapshots.append(conditioner.update(
        110.4, [person(shadow, 110.4)]))

    # Then: the advisory bypass remains committed instead of STOP-GO.
    assert [
        snapshot.decision.value for snapshot in snapshots[-3:]
    ] == ["BYPASS_COMMITTED"] * 3
    assert [snapshot.track_id for snapshot in snapshots[-3:]] == [1641] * 3


def committed(module):
    conditioner = module.HumanAwareConditioner()
    for index in range(51):
        stamp = 100.0 + 0.2 * index
        snapshot = conditioner.update(stamp, [person(module, stamp)])
    assert snapshot.decision.value == "BYPASS_COMMITTED"
    return conditioner


def test_dropout_revokes_shadow_commitment_immediately():
    # Given: a stationary person has earned a shadow bypass commitment.
    shadow = load_shadow()
    conditioner = committed(shadow)

    # When: the detector publishes a fresh empty cycle.
    snapshot = conditioner.update(110.2, [])

    # Then: absence is not treated as a clear person.
    assert snapshot.decision.value == "STOP_REQUIRED"


def test_producer_stamp_gap_revokes_shadow_commitment():
    # Given: a committed same-ID stationary person.
    shadow = load_shadow()
    conditioner = committed(shadow)

    # When: their next producer stamp has a discontinuity.
    snapshot = conditioner.update(110.6, [person(shadow, 110.6)])

    # Then: old commitment cannot bridge missing evidence.
    assert snapshot.decision.value == "STOP_REQUIRED"


def test_producer_stamp_regression_revokes_shadow_commitment():
    # Given: a committed same-ID stationary person.
    shadow = load_shadow()
    conditioner = committed(shadow)

    # When: the producer clock goes backwards.
    snapshot = conditioner.update(109.8, [person(shadow, 109.8)])

    # Then: reordered evidence fails closed.
    assert snapshot.decision.value == "STOP_REQUIRED"


def test_identity_replacement_revokes_shadow_commitment():
    # Given: track 1641 has a committed shadow bypass.
    shadow = load_shadow()
    conditioner = committed(shadow)
    replacement = person(shadow, 110.2)
    replacement = shadow.PersonObservation(
        track_id=1689,
        observed_stamp_s=replacement.observed_stamp_s,
        motion=replacement.motion,
        speed_mps=replacement.speed_mps,
        forward_m=replacement.forward_m,
        lateral_m=replacement.lateral_m,
        half_length_m=replacement.half_length_m,
        half_width_m=replacement.half_width_m,
        directly_observed=True,
        geometry_valid=True,
    )

    # When: a different person occupies the corridor.
    snapshot = conditioner.update(110.2, [replacement])

    # Then: identity continuity is required again.
    assert snapshot.decision.value == "STOP_REQUIRED"


def test_malformed_geometry_revokes_shadow_commitment():
    # Given: track 1641 has a committed shadow bypass.
    shadow = load_shadow()
    conditioner = committed(shadow)
    observed = person(shadow, 110.2)
    malformed = shadow.PersonObservation(
        track_id=observed.track_id,
        observed_stamp_s=observed.observed_stamp_s,
        motion=observed.motion,
        speed_mps=observed.speed_mps,
        forward_m=observed.forward_m,
        lateral_m=observed.lateral_m,
        half_length_m=observed.half_length_m,
        half_width_m=observed.half_width_m,
        directly_observed=True,
        geometry_valid=False,
    )

    # When: current geometry is unusable.
    snapshot = conditioner.update(110.2, [malformed])

    # Then: remembered geometry cannot authorize a maneuver.
    assert snapshot.decision.value == "STOP_REQUIRED"


def test_malformed_speed_never_becomes_stationary_shadow_evidence():
    # Given: a producer person whose speed field is absent.
    shadow = load_shadow()
    observed = person(shadow, 100.0)
    malformed = shadow.PersonObservation(
        track_id=observed.track_id,
        observed_stamp_s=observed.observed_stamp_s,
        motion=observed.motion,
        speed_mps=shadow.finite_or_nan(None),
        forward_m=observed.forward_m,
        lateral_m=observed.lateral_m,
        half_length_m=observed.half_length_m,
        half_width_m=observed.half_width_m,
        directly_observed=True,
        geometry_valid=True,
    )

    # When: it reaches the temporal trust boundary.
    snapshot = shadow.HumanAwareConditioner().update(100.0, [malformed])

    # Then: missing speed cannot silently become zero/static evidence.
    assert snapshot.decision.value == "STOP_REQUIRED"


def test_second_moving_person_revokes_shadow_commitment():
    # Given: one stationary person has earned a commitment.
    shadow = load_shadow()
    conditioner = committed(shadow)
    second = shadow.PersonObservation(
        track_id=1689,
        observed_stamp_s=110.2,
        motion="moving",
        speed_mps=0.8,
        forward_m=3.0,
        lateral_m=0.4,
        half_length_m=0.35,
        half_width_m=0.35,
        directly_observed=True,
        geometry_valid=True,
    )

    # When: another moving person enters the planning scene.
    snapshot = conditioner.update(
        110.2, [person(shadow, 110.2), second])

    # Then: the single-person commitment cannot survive.
    assert snapshot.decision.value == "STOP_REQUIRED"


def test_sustained_moving_person_revokes_shadow_commitment():
    # Given: one noise frame is tolerated after commitment.
    shadow = load_shadow()
    conditioner = committed(shadow)

    # When: the same person keeps moving across four producer cycles.
    snapshots = [
        conditioner.update(
            stamp,
            [person(shadow, stamp, motion="moving", speed_mps=0.6)],
        )
        for stamp in (110.2, 110.4, 110.6, 110.8)
    ]

    # Then: sustained motion is stop-required, not latched forever.
    assert snapshots[-1].decision.value == "STOP_REQUIRED"


def test_moving_person_commitment_revokes_when_localization_lost():
    # Given: a stationary person has earned a shadow commitment.
    shadow = load_shadow()
    conditioner = committed(shadow)

    # When: localization ceases to be TRACKING.
    snapshot = conditioner.update(
        110.2, [person(shadow, 110.2)], localization_tracking=False)

    # Then: map-frame human planning fails closed.
    assert snapshot.decision.value == "STOP_REQUIRED"


class OpenBand:
    def contains(self, _point):
        return True

    def chord_is_contained(self, _start, _end):
        return True


class ClosedMask:
    def contains(self, point):
        return point[0] < 1.0

    def segment_is_contained(self, start, end):
        return start[0] < 1.0 and end[0] < 1.0


def test_shadow_trajectory_never_overrules_hard_mask():
    # Given: a candidate starts in bounds but crosses the physical mask.
    shadow = load_shadow()
    validator = shadow.ShadowTrajectoryValidator(
        safety_band=OpenBand(), drivable_mask=ClosedMask())

    # When: HATEB proposes a local plan through forbidden ground.
    result = validator.validate(((0.0, 0.0), (0.8, 0.0), (1.2, 0.0)))

    # Then: the candidate is rejected rather than merely penalized.
    assert result.value == "HARD_MASK_REJECTED"


def test_stationary_person_maps_to_official_cohan_torso_contract():
    # Given: a local person observation and the corrected robot map pose.
    shadow = load_shadow()
    observed = person(shadow, 110.0)
    robot = shadow.RobotPose2D(x_m=10.0, y_m=5.0, yaw_rad=math.pi / 2.0)

    # When: the observation crosses the CoHAN adapter boundary.
    agent = shadow.to_cohan_agent(observed, robot, frame_id="map")

    # Then: stable identity, official constants, and map geometry survive.
    assert agent.track_id == 1641
    assert agent.state == 0
    assert agent.agent_type == 1
    assert agent.segment_type == 1
    assert agent.frame_id == "map"
    assert agent.stamp_s == 110.0
    assert agent.x_m == pytest.approx(10.0)
    assert agent.y_m == pytest.approx(7.42)
    assert agent.vx_mps == 0.0
    assert agent.vy_mps == 0.0
