import dataclasses
import importlib.util
import math
import sys
from pathlib import Path
import threading
import types

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "collision_supervisor.py"
POLICY = Path(__file__).parents[1] / "config" / "collision_policy.yaml"
spec = importlib.util.spec_from_file_location("collision_supervisor", SCRIPT)
cs = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = cs
spec.loader.exec_module(cs)


def policy():
    return cs.CollisionPolicy.load(str(POLICY))


def inputs(sequence=1, now=1.0, points=(), speed=0.5, **changes):
    base = cs.CollisionInputs(
        now_s=now,
        sequence=sequence,
        cloud_stamp_s=now - 0.01,
        odom_stamp_s=now - 0.01,
        nav_stamp_s=now - 0.01,
        safe_stamp_s=now - 0.01,
        points=points,
        odom_linear_mps=speed,
        nav_linear_mps=speed,
        safe_linear_mps=speed,
        coverage_fraction=1.0,
        expected_coverage_bins=20,
        observed_coverage_bins=20,
        policy_id=policy().policy_id,
        policy_sha256=policy().policy_sha256,
    )
    return dataclasses.replace(base, **changes)


def static_point(x, y=0.0):
    return cs.PointObservation(x, y, vx=0.0, vy=0.0, observation_count=3, covariance_valid=True)


def release(core, first_sequence=1, first_time=1.0):
    decision = None
    # Five frames are insufficient unless the first-to-last interval also reaches 0.50 s.
    for offset, elapsed in enumerate((0.0, 0.1, 0.2, 0.3, 0.5)):
        decision = core.evaluate(inputs(first_sequence + offset, first_time + elapsed, (static_point(10.0),), speed=0.0))
    return decision

def _slope_message(source, evaluation, policy_sha256, state):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: source)),
        evaluation_stamp=types.SimpleNamespace(to_sec=lambda: evaluation),
        pitch_rad=0.0,
        policy_sha256=policy_sha256,
        state=state,
    )


def _slope_callback_node(policy_sha256):
    node = object.__new__(cs.CollisionSupervisorRosNode)
    node._input_lock = threading.RLock()
    node.slope = None
    node.slope_high_water = None
    node.slope_evidence = cs.deque()
    node.slope_policy_sha256 = policy_sha256
    node.odom = node.nav = node.safe = node.intent = None
    node.odom_stamp = node.odom_receipt = node.odom_high_water = None
    node.odom_valid = False
    node.nav_stamp = node.safe_stamp = None
    return node

def _odom_message(source):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: source)),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0),
                angular=types.SimpleNamespace(z=0.0),
            )
        ),
    )


@pytest.mark.parametrize(
    "offset,expected",
    (
        (cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, True),
        (cs.CLOCK_FUTURE_TOLERANCE_S, True),
        (cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, False),
    ),
)
def test_odom_callback_allows_only_future_tolerance_source_receipt_skew(
    monkeypatch, offset, expected
):
    node = _slope_callback_node("a" * 64)
    monkeypatch.setitem(
        sys.modules,
        "rospy",
        types.SimpleNamespace(
            Time=types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0)
            )
        ),
    )

    source = 1.0 + offset
    node._odom_cb(_odom_message(source))

    assert node.odom_valid is expected
    assert node.odom_high_water == (pytest.approx(source) if expected else None)


def test_odom_callback_invalidates_duplicate_and_regressing_source(monkeypatch):
    node = _slope_callback_node("a" * 64)
    monkeypatch.setitem(
        sys.modules,
        "rospy",
        types.SimpleNamespace(
            Time=types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0)
            )
        ),
    )

    node._odom_cb(_odom_message(0.99))
    assert node.odom_valid
    node._odom_cb(_odom_message(0.99))
    assert not node.odom_valid
    node._odom_cb(_odom_message(0.98))
    assert not node.odom_valid
    assert node.odom_high_water == pytest.approx(0.99)


@pytest.mark.parametrize(
    "source,evaluation,receipt,expected",
    (
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, 1.0, 1.0, True),
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S, 1.0, 1.0, True),
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, 1.0, 1.0, False),
        (0.5, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, 1.0, True),
        (0.5, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S, 1.0, True),
        (0.5, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, 1.0, False),
    ),
)
def test_slope_chronology_allows_only_50ms_future_skew(source, evaluation, receipt, expected):
    assert cs._slope_chronology_valid(source, evaluation, receipt, None) is expected


@pytest.mark.parametrize(
    "source,evaluation,cloud_stamp,now,expected",
    (
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, 0.5, 1.0, 1.0, True),
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S, 0.5, 1.0, 1.0, True),
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, 0.5, 1.0, 1.0, False),
        (0.5, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, 1.0, 1.0, True),
        (0.5, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S, 1.0, 1.0, True),
        (0.5, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, 1.0, 1.0, False),
    ),
)
def test_slope_cloud_revalidation_allows_only_50ms_future_skew(
    source, evaluation, cloud_stamp, now, expected
):
    assert cs._slope_cloud_time_valid(source, evaluation, cloud_stamp, now) is expected


def test_slope_callback_rejects_source_regression(monkeypatch):
    policy_sha256 = "a" * 64
    node = _slope_callback_node(policy_sha256)
    monkeypatch.setitem(
        sys.modules,
        "rospy",
        types.SimpleNamespace(
            Time=types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0)
            )
        ),
    )

    node._slope_cb(_slope_message(1.0, 1.0, policy_sha256, cs.CLEAR))
    assert node.slope[-1] is True
    node._slope_cb(_slope_message(0.9, 1.0, policy_sha256, cs.CLEAR))

    assert node.slope[-1] is False
    assert not node.slope_evidence
    assert cs._select_slope_evidence(node.slope_evidence, 1.0, 1.0) is None

    node._slope_cb(_slope_message(1.01, 1.01, policy_sha256, cs.CLEAR))

    assert node.slope_evidence[-1].valid is True


@pytest.mark.parametrize(
    "policy_sha256,state",
    (
        ("b" * 64, cs.CLEAR),
        ("a" * 64, cs.STOP),
        ("a" * 64, cs.UNKNOWN),
    ),
)
def test_slope_callback_rejects_wrong_policy_and_unsafe_states(monkeypatch, policy_sha256, state):
    expected_policy_sha256 = "a" * 64
    node = _slope_callback_node(expected_policy_sha256)
    monkeypatch.setitem(
        sys.modules,
        "rospy",
        types.SimpleNamespace(
            Time=types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0)
            )
        ),
    )

    node._slope_cb(_slope_message(1.0, 1.0, policy_sha256, state))

    assert node.slope[-1] is False
    assert node.slope_high_water == 1.0


def _evidence(source, evaluation=None, receipt=None, valid=True):
    return cs.SlopeEvidence(
        source,
        source if evaluation is None else evaluation,
        source if receipt is None else receipt,
        0.0,
        "a" * 64,
        valid,
    )


def test_slope_buffer_selects_async_cloud_pair_and_restrictive_entry():
    evidence = cs.deque()
    for source in (0.86, 0.88, 0.90, 0.92, 0.94, 0.96, 0.98):
        cs._buffer_slope_evidence(evidence, _evidence(source))
    cs._buffer_slope_evidence(evidence, _evidence(0.95, valid=False))

    selected = cs._select_slope_evidence(evidence, 0.90, 1.0)

    assert selected is not None
    assert selected.source_s == pytest.approx(0.95)
    assert selected.valid is False


def test_slope_buffer_fresh_restrictive_evidence_dominates_newer_permissive():
    evidence = cs.deque((
        _evidence(0.94, receipt=0.94, valid=False),
        _evidence(0.95, receipt=0.95, valid=True),
    ))

    selected = cs._select_slope_evidence(evidence, 0.90, 1.00)

    assert selected is not None
    assert selected.source_s == pytest.approx(0.94)
    assert selected.valid is False


def test_slope_buffer_stale_restrictive_evidence_does_not_mask_exactly_fresh_permissive():
    evidence = cs.deque((
        _evidence(0.89, receipt=0.89, valid=False),
        _evidence(0.90, receipt=0.90, valid=True),
    ))

    selected = cs._select_slope_evidence(evidence, 0.90, 1.00)

    assert selected is not None
    assert selected.source_s == pytest.approx(0.90)
    assert selected.valid is True


def test_slope_buffer_has_no_candidate_when_all_samples_are_too_new():
    evidence = cs.deque((_evidence(0.96), _evidence(0.98)))

    assert cs._select_slope_evidence(evidence, 0.90, 1.0) is None


def test_slope_buffer_is_source_ordered_and_bounded():
    evidence = cs.deque()
    for source in (1.00, 0.99, 1.01, 1.30):
        cs._buffer_slope_evidence(evidence, _evidence(source))

    assert [entry.source_s for entry in evidence] == [1.30]


@pytest.mark.parametrize(
    "source,receipt,cloud_stamp,now,expected",
    (
        (0.95 - 0.0001, 1.0, 0.90, 1.0, True),
        (0.95, 1.0, 0.90, 1.0, True),
        (0.95 + 0.0001, 1.0, 0.90, 1.0, False),
        (0.90, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S, 0.90, 1.0, True),
        (0.90, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, 0.90, 1.0, False),
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, 1.0, 1.0, 1.0, True),
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S, 1.0, 1.0, 1.0, True),
        (1.0 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, 1.0, 1.0, 1.0, False),
        (0.90, 0.90, 0.90, 1.0, True),
        (0.90 - 0.1001, 1.0, 0.90, 1.0, False),
    ),
)
def test_slope_core_timing_keeps_exact_future_and_ttl_bounds(
    source, receipt, cloud_stamp, now, expected
):
    assert cs._slope_timing_valid(source, receipt, cloud_stamp, now) is expected

def test_core_accepts_async_cloud_matched_slope_with_fresh_receipt():
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        cloud_stamp_s=0.90,
        slope_stamp_s=0.90,
        slope_receipt_s=1.0,
    ))

    assert decision.reason != "stale_or_mismatched_slope_odom"


@pytest.mark.parametrize(
    "slope_stamp_s,slope_receipt_s",
    (
        (0.90 - 0.1001, 1.0),
        (0.90 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, 1.0),
        (0.90, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001),
    ),
)
def test_core_rejects_stale_or_future_async_slope_evidence(
    slope_stamp_s, slope_receipt_s
):
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        cloud_stamp_s=0.90,
        slope_stamp_s=slope_stamp_s,
        slope_receipt_s=slope_receipt_s,
    ))

    assert decision.state == cs.STOP
    assert decision.reason == "stale_or_mismatched_slope_odom"



def test_slope_callback_clears_prior_evidence_after_malformed_callback(monkeypatch):
    node = _slope_callback_node("a" * 64)
    monkeypatch.setitem(
        sys.modules,
        "rospy",
        types.SimpleNamespace(
            Time=types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0)
            )
        ),
    )
    node._slope_cb(_slope_message(0.95, 0.95, "a" * 64, cs.CLEAR))
    malformed = _slope_message(1.0, float("nan"), "a" * 64, cs.CLEAR)

    node._slope_cb(malformed)

    assert node.slope[-1] is False
    assert not node.slope_evidence
    assert cs._select_slope_evidence(node.slope_evidence, 1.0, 1.0) is None

    node._slope_cb(_slope_message(1.01, 1.01, "a" * 64, cs.CLEAR))

    assert node.slope_evidence[-1].valid is True


def test_slope_callback_receipt_is_sampled_after_input_lock_wait(monkeypatch):
    node = _slope_callback_node("a" * 64)
    clock = {"now": 1.0}
    monkeypatch.setitem(
        sys.modules,
        "rospy",
        types.SimpleNamespace(
            Time=types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_sec=lambda: clock["now"])
            )
        ),
    )
    node._input_lock.acquire()
    worker = threading.Thread(
        target=node._slope_cb,
        args=(_slope_message(1.0, 1.0, "a" * 64, cs.CLEAR),),
    )
    worker.start()
    clock["now"] = 2.0
    node._input_lock.release()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert node.slope[2] == 2.0


def test_input_callback_does_not_wait_for_cloud_decision_lock(monkeypatch):
    node = _slope_callback_node("a" * 64)
    node._decision_lock = threading.RLock()
    monkeypatch.setitem(
        sys.modules,
        "rospy",
        types.SimpleNamespace(
            Time=types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0)
            )
        ),
    )
    completed = threading.Event()
    worker = threading.Thread(
        target=lambda: (
            node._slope_cb(_slope_message(1.0, 1.0, "a" * 64, cs.CLEAR)),
            completed.set(),
        ),
    )
    with node._decision_lock:
        worker.start()
        assert completed.wait(1.0)
    worker.join(timeout=1.0)

    assert node.slope_evidence[-1].valid is True


def test_snapshot_is_immutable_and_uses_only_copied_slope_evidence(monkeypatch):
    node = _slope_callback_node("a" * 64)
    node.slope_evidence.append(_evidence(0.90, valid=True))
    monkeypatch.setitem(
        sys.modules,
        "rospy",
        types.SimpleNamespace(
            Time=types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0)
            )
        ),
    )

    snapshot = node._take_input_snapshot()
    node.slope_evidence.append(_evidence(0.95, valid=False))

    assert snapshot.now_s == 1.0
    assert isinstance(snapshot.slope_evidence, tuple)
    assert cs._select_slope_evidence(snapshot.slope_evidence, 0.90, snapshot.now_s).valid
    assert not cs._select_slope_evidence(
        tuple(node.slope_evidence), 0.90, snapshot.now_s
    ).valid
    with pytest.raises(dataclasses.FrozenInstanceError):
        snapshot.now_s = 2.0


def test_postprocess_fake_clock_uses_later_time_for_decision_ages():
    fake_clock = iter((1.00, 1.15))
    snapshot_now = next(fake_clock)
    evaluation_now = cs._postprocess_evaluation_time(snapshot_now, next(fake_clock))
    core = cs.CollisionSupervisorCore(policy())

    decision = core.evaluate(inputs(
        now=evaluation_now,
        cloud_stamp_s=1.14,
        odom_stamp_s=1.00,
        odom_receipt_s=1.00,
        nav_stamp_s=1.00,
        safe_stamp_s=1.00,
        intent_stamp_s=1.00,
    ))

    assert decision.evaluation_stamp == pytest.approx(1.15)
    assert decision.odom_age_s == pytest.approx(0.15)


@pytest.mark.parametrize("postprocess_now", (0.99, math.nan, math.inf))
def test_postprocess_clock_regression_or_nonfinite_is_fail_closed(postprocess_now):
    evaluation_now = cs._postprocess_evaluation_time(1.00, postprocess_now)

    assert math.isnan(evaluation_now)
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        now=evaluation_now,
        cloud_stamp_s=1.0,
        odom_stamp_s=1.0,
        odom_receipt_s=1.0,
        nav_stamp_s=1.0,
        safe_stamp_s=1.0,
        intent_stamp_s=1.0,
    ))
    assert decision.state == cs.STOP
    assert decision.reason == "nonfinite_input"

@pytest.mark.parametrize(
    "field",
    ("cloud_stamp_s", "odom_stamp_s", "nav_stamp_s", "safe_stamp_s", "intent_stamp_s"),
)
@pytest.mark.parametrize(
    "offset,accepted",
    (
        (cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, True),
        (cs.CLOCK_FUTURE_TOLERANCE_S, True),
        (cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, False),
    ),
)
def test_core_timestamp_sources_allow_only_future_tolerance(field, offset, accepted):
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        **{field: 1.0 + offset},
    ))

    if accepted:
        assert decision.reason != "invalid_timestamp"
    else:
        assert decision.state == cs.STOP
        assert decision.reason == "invalid_timestamp"


@pytest.mark.parametrize(
    "source,receipt,accepted",
    (
        (1.0, 1.0 - cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, True),
        (1.0, 1.0 - cs.CLOCK_FUTURE_TOLERANCE_S, True),
        (1.0, 1.0 - cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, False),
        (0.99, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S - 0.0001, True),
        (0.99, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S, True),
        (0.99, 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S + 0.0001, False),
    ),
)
def test_core_odom_receipt_allows_only_future_tolerance(source, receipt, accepted):
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        odom_stamp_s=source,
        odom_receipt_s=receipt,
    ))

    if accepted:
        assert decision.reason != "stale_or_mismatched_slope_odom"
    else:
        assert decision.state == cs.STOP
        assert decision.reason == "stale_or_mismatched_slope_odom"


def test_core_clamps_valid_future_timestamp_ages_to_zero():
    future = 1.0 + cs.CLOCK_FUTURE_TOLERANCE_S
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        cloud_stamp_s=future,
        odom_stamp_s=future,
        odom_receipt_s=future,
        nav_stamp_s=future,
        safe_stamp_s=future,
        intent_stamp_s=future,
    ))

    assert decision.reason != "invalid_timestamp"
    assert (decision.input_age_s, decision.odom_age_s, decision.command_age_s) == (0.0, 0.0, 0.0)


def test_slope_buffer_selects_equal_source_restrictive_latest_entry():
    evidence = cs.deque((
        _evidence(0.95, evaluation=0.95, receipt=0.95, valid=True),
        _evidence(0.95, evaluation=0.96, receipt=0.96, valid=False),
    ))

    selected = cs._select_slope_evidence(evidence, 0.90, 1.0)

    assert selected is not None
    assert selected.valid is False

def test_policy_is_hash_checked_and_simulation_only():
    loaded = policy()
    assert loaded.qualification == "simulation_only"
    assert loaded.hardware_motion_authorized is False
    assert loaded.passenger_operation_authorized is False
    assert loaded.policy_sha256 == "5850bb0cd84bc04f4f9cdc78cd347640a3f60f66241ad3f37c196ad63cbeba18"
    assert loaded.coverage_max_frames == 4
    assert loaded.coverage_motion_linear_tolerance_mps == 0.05
    assert cs.IMMUTABLE_HARD_LINEAR_SPEED_MPS == 0.55
    raw = {field.name: getattr(loaded, field.name) for field in dataclasses.fields(loaded)}
    raw["footprint_width_m"] += 0.01
    with pytest.raises(ValueError, match="SHA-256"):
        cs.CollisionPolicy.from_mapping(raw)


def test_v1_ttc_margin_is_exact_and_cannot_be_resealed_lower():
    loaded = policy()
    assert loaded.stop_ttc_margin_s == 0.50
    raw = {field.name: getattr(loaded, field.name) for field in dataclasses.fields(loaded)}
    raw["stop_ttc_margin_s"] = 0.499999
    raw["policy_sha256"] = cs.CollisionPolicy.hash_mapping(raw)
    with pytest.raises(ValueError, match="exactly 0.50"):
        cs.CollisionPolicy.from_mapping(raw)


def test_direction_disagreement_stops_material_opposition_but_not_tiny_noise():
    opposed = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        odom_linear_mps=-0.19,
        nav_linear_mps=0.10,
        safe_linear_mps=0.10,
    ))
    assert opposed.state == cs.STOP
    assert opposed.reason == "motion_direction_disagreement"

    opposed_reverse = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        odom_linear_mps=0.10,
        nav_linear_mps=-0.19,
        safe_linear_mps=-0.19,
    ))
    assert opposed_reverse.state == cs.STOP
    assert opposed_reverse.reason == "motion_direction_disagreement"

    noise = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        odom_linear_mps=-0.049,
        nav_linear_mps=0.049,
        safe_linear_mps=0.0,
    ))
    assert noise.reason != "motion_direction_disagreement"

    core = cs.CollisionSupervisorCore(policy())
    stale_slope = core.evaluate(inputs(
        points=(static_point(10.0),),
        slope_stamp_s=0.7,
        slope_receipt_s=0.7,
    ))
    assert stale_slope.state == cs.STOP
    assert stale_slope.reason == "stale_or_mismatched_slope_odom"

    core = cs.CollisionSupervisorCore(policy())
    stale_receipt = core.evaluate(inputs(
        points=(static_point(10.0),),
        odom_receipt_s=0.7,
    ))
    assert stale_receipt.state == cs.STOP
    assert stale_receipt.reason == "stale_odom"

    core = cs.CollisionSupervisorCore(policy())
    missing_slope = core.evaluate(inputs(
        points=(static_point(10.0),),
        slope_valid=False,
    ))
    assert missing_slope.state == cs.STOP
    assert missing_slope.reason == "invalid_slope_evidence"


def test_watchdog_retains_last_real_cloud_source_stamp():
    watchdog = cs.CollisionWatchdogState(policy().cloud_ttl_s)
    assert watchdog.observe_cloud(2.0, 2.01)
    assert watchdog.last_cloud_stamp_s == 2.0
    assert watchdog.stale_age(2.0 + policy().cloud_ttl_s + 1.0e-9) is not None


def test_cloud_transform_applies_rotation_translation_and_velocity():
    half_sqrt = math.sqrt(0.5)
    point = cs.PointObservation(
        1.0, 0.0, 2.0, vx=1.0, vy=0.0, track_id="track",
        observation_count=3, covariance_valid=True,
    )
    result = cs.prepare_transformed_cloud(
        (point,), "lidar_link", "base_footprint", 10.0, 9.98,
        (2.0, 3.0, -1.0), (0.0, 0.0, half_sqrt, half_sqrt), 0.1,
        "lidar_link", "base_footprint",
    )
    assert result.ok
    assert result.transform_age_s == pytest.approx(0.02)
    assert (result.points[0].x, result.points[0].y, result.points[0].z) == pytest.approx((2.0, 4.0, 1.0))
    assert (result.points[0].vx, result.points[0].vy) == pytest.approx((0.0, 1.0))
    assert result.points[0].track_id == "track"
    assert result.points[0].observation_count == 3
    assert result.points[0].covariance_valid is True


def test_identity_transform_does_not_merely_relabel_nonidentity_data():
    point = cs.PointObservation(1.0, -2.0, 0.5)
    identity = cs.prepare_transformed_cloud(
        (point,), "lidar_link", "base_footprint", 4.0, 4.0,
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.1,
    )
    translated = cs.prepare_transformed_cloud(
        (point,), "lidar_link", "base_footprint", 4.0, 4.0,
        (5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.1,
    )
    assert identity.ok and translated.ok
    assert identity.points[0] == point
    assert translated.points[0].x == pytest.approx(6.0)
    assert translated.points[0].x != point.x
    assert translated.frame_id == "base_footprint"


def test_zero_stamp_static_transform_is_valid_only_when_explicit_and_still_transforms():
    point = cs.PointObservation(1.0, 2.0, 0.5)
    result = cs.prepare_transformed_cloud(
        (point,), "lidar_link", "base_footprint", 10.0, 0.0,
        (3.0, -1.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.1,
        "lidar_link", "base_footprint", static_transform=True,
    )
    assert result.ok
    assert result.transform_age_s == 0.0
    assert (result.points[0].x, result.points[0].y, result.points[0].z) == pytest.approx(
        (4.0, 1.0, 0.5)
    )
    rejected = cs.prepare_transformed_cloud(
        (point,), "lidar_link", "base_footprint", 10.0, 0.0,
        (3.0, -1.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.1,
    )
    assert not rejected.ok
    assert rejected.reason == "stale_transform"


@pytest.mark.parametrize(
    "changes, reason",
    [
        ({"transform_stamp_s": 9.8}, "stale_transform"),
        ({"transform_stamp_s": 10.01}, "future_transform"),
        ({"source_frame": "base_footprint"}, "frame_mismatch"),
        ({"target_frame": "odom"}, "frame_mismatch"),
        ({"returned_source_frame": "camera_link"}, "frame_mismatch"),
        ({"translation": (math.nan, 0.0, 0.0)}, "nonfinite_transform"),
        ({"points": (cs.PointObservation(math.inf, 0.0, 0.0),)}, "nonfinite_point"),
        ({"points": ((1.0, 0.0, 0.0),)}, "malformed_point"),
        ({"rotation_xyzw": (0.0, 0.0, 0.0, 0.0)}, "invalid_rotation"),
    ],
)
def test_untrusted_cloud_transform_fails_closed(changes, reason):
    arguments = {
        "points": (cs.PointObservation(1.0, 0.0, 0.0),),
        "source_frame": "lidar_link",
        "target_frame": "base_footprint",
        "cloud_stamp_s": 10.0,
        "transform_stamp_s": 10.0,
        "translation": (0.0, 0.0, 0.0),
        "rotation_xyzw": (0.0, 0.0, 0.0, 1.0),
        "max_transform_age_s": 0.1,
        "returned_source_frame": "lidar_link",
        "returned_target_frame": "base_footprint",
    }
    arguments.update(changes)
    result = cs.prepare_transformed_cloud(**arguments)
    assert not result.ok
    assert result.points == ()
    assert result.reason == reason
    decision = cs.CollisionSupervisorCore(policy()).evaluate(
        inputs(points=result.points, frame_id=result.frame_id, transform_ok=result.ok,
               transform_age_s=result.transform_age_s)
    )
    assert decision.state == cs.STOP
    assert decision.reason_mask & cs.TF
    assert decision.signal_state == cs.SIGNAL_STOP


def test_transform_age_is_measured_at_cloud_source_timestamp():
    result = cs.prepare_transformed_cloud(
        (cs.PointObservation(1.0, 0.0, 0.0),),
        "lidar_link", "base_footprint",
        cloud_stamp_s=100.0,
        transform_stamp_s=99.96,
        translation=(0.0, 0.0, 0.0),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
        max_transform_age_s=0.05,
    )
    assert result.ok
    assert result.transform_age_s == pytest.approx(0.04)
    decision = cs.CollisionSupervisorCore(policy()).evaluate(
        inputs(points=(static_point(10.0),), transform_age_s=result.transform_age_s)
    )
    assert decision.transform_age_s == pytest.approx(0.04)


def test_static_obstacle_inside_required_stopping_distance_stops_in_one_frame():
    core = cs.CollisionSupervisorCore(policy())
    decision = core.evaluate(inputs(points=(static_point(1.0),)))
    speed = cs.IMMUTABLE_HARD_LINEAR_SPEED_MPS
    expected = speed * (0.01 + 0.05 + 0.09) + speed ** 2 / (2.0 * 0.5) + 0.20
    assert decision.required_stop_distance_m == pytest.approx(expected)
    assert decision.state == cs.STOP
    assert decision.obstacle_motion == cs.MOTION_STATIC
    assert decision.reason_mask & cs.COLLISION_DISTANCE
    assert decision.signal_state == cs.SIGNAL_STOP


def test_crossing_dynamic_track_uses_relative_ttc():
    point = cs.PointObservation(0.70, 2.0, vx=0.0, vy=-2.0, observation_count=3, covariance_valid=True)
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(points=(point,), speed=0.2))
    assert decision.obstacle_motion == cs.MOTION_DYNAMIC
    assert 0.0 < decision.time_to_collision_s < 1.0
    assert decision.closing_speed_mps > 0.0
    assert decision.state == cs.STOP
    assert decision.reason_mask & (cs.COLLISION_TTC | cs.COLLISION_DISTANCE)


def test_receding_dynamic_object_has_no_finite_ttc_and_can_clear():
    core = cs.CollisionSupervisorCore(policy())
    release(core)
    point = cs.PointObservation(2.0, 0.0, vx=2.0, vy=0.0, observation_count=3, covariance_valid=True)
    decision = core.evaluate(inputs(6, 1.6, (point,), speed=0.1))
    assert decision.obstacle_motion == cs.MOTION_DYNAMIC
    assert decision.time_to_collision_s == -1.0
    assert decision.closing_speed_mps == 0.0
    assert decision.state == cs.CLEAR
    assert decision.signal_state == cs.SIGNAL_CLEAR
    assert decision.recommended_max_linear_mps == -1.0


def test_unqualified_track_inside_dynamic_stop_ttc_is_ambiguous_and_stops():
    point = cs.PointObservation(1.2, 0.0, vx=0.0, vy=0.0, observation_count=2)
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(points=(point,), speed=0.1))
    assert decision.obstacle_motion == cs.MOTION_AMBIGUOUS
    assert decision.state == cs.STOP
    assert decision.reason == "ambiguous_obstacle"
def test_far_ambiguous_track_caps_speed_without_false_emergency_stop():
    core = cs.CollisionSupervisorCore(policy())
    release(core)
    point = cs.PointObservation(
        10.0,
        0.0,
        vx=0.0,
        vy=0.0,
        observation_count=2,
    )

    decision = core.evaluate(inputs(6, 1.6, (point,), speed=0.1))

    assert decision.obstacle_motion == cs.MOTION_AMBIGUOUS
    assert decision.time_to_collision_s == -1.0
    assert decision.state == cs.CAUTION
    assert decision.reason == "ambiguous_caution"
    assert decision.reason_mask == 0
    assert decision.signal_state == cs.SIGNAL_CLEAR
    assert decision.recommended_max_linear_mps == pytest.approx(
        policy().caution_max_linear_mps
    )


def test_lateral_ambiguous_wall_does_not_invent_radial_collision_path():
    core = cs.CollisionSupervisorCore(policy())
    release(core)
    wall = cs.PointObservation(
        1.165696,
        -2.834842,
        1.188661,
        vx=-1.979188099,
        vy=-0.296642116,
        track_id="wall",
        observation_count=policy().minimum_observations,
        covariance_valid=False,
    )

    decision = core.evaluate(inputs(
        6,
        1.6,
        (wall,),
        speed=0.030351,
        odom_angular_rps=-0.04516,
        nav_angular_rps=-0.04516,
        safe_angular_rps=-0.04516,
    ))

    assert decision.state == cs.CLEAR
    assert decision.reason == "clear"
    assert decision.time_to_collision_s == -1.0
    assert decision.reason_mask == 0


def test_unrelated_ambiguity_cannot_borrow_static_candidate_intersection():
    core = cs.CollisionSupervisorCore(policy())
    release(core)
    static_hazard = static_point(0.8)
    unrelated_ambiguous = cs.PointObservation(
        1.165696,
        -2.834842,
        1.188661,
        vx=-1.979188099,
        vy=-0.296642116,
        observation_count=policy().minimum_observations,
        covariance_valid=False,
    )

    decision = core.evaluate(inputs(
        6,
        1.6,
        (static_hazard, unrelated_ambiguous),
        speed=0.1,
    ))

    assert decision.state == cs.STOP
    assert decision.reason == "collision_distance"
    assert decision.reason_mask & cs.COLLISION_DISTANCE
    assert decision.reason != "ambiguous_obstacle"


def test_constant_twist_sweep_distinguishes_straight_and_turning_paths():
    point = static_point(1.5, 1.0)
    expansion = (
        policy().localization_uncertainty_m
        + policy().transform_uncertainty_m
        + policy().point_noise_m
        + policy().fixed_expansion_m
    )
    hx = policy().footprint_length_m / 2.0 + expansion
    hy = policy().footprint_width_m / 2.0 + expansion
    straight = cs.CollisionSupervisorCore._swept_ttc(
        point, 0.0, 0.0, 0.5, 0.0, hx, hy, policy().max_horizon_s
    )
    turning = cs.CollisionSupervisorCore._swept_ttc(
        point, 0.0, 0.0, 0.5, 0.5, hx, hy, policy().max_horizon_s
    )
    assert straight is None
    assert turning is not None


def test_measured_requested_and_prior_safe_angular_motion_is_conservative():
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        odom_linear_mps=0.2, nav_linear_mps=0.4, safe_linear_mps=0.3,
        odom_angular_rps=0.6, nav_angular_rps=0.2, safe_angular_rps=0.4,
    ))
    assert decision.forward_speed_mps == pytest.approx(0.55)
    assert decision.angular_speed_rps == pytest.approx(0.6)

    negative = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        odom_angular_rps=-0.3, nav_angular_rps=-0.5, safe_angular_rps=-0.4,
    ))
    assert negative.angular_speed_rps == pytest.approx(-0.5)
    prior_safe = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),),
        odom_angular_rps=0.3, nav_angular_rps=0.5, safe_angular_rps=0.7,
    ))
    assert prior_safe.angular_speed_rps == pytest.approx(0.7)
def test_swept_envelope_uses_immutable_hard_cap_when_nav_can_accelerate():
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(1.28),),
        odom_linear_mps=0.10,
        nav_linear_mps=0.10,
        safe_linear_mps=0.10,
    ))

    assert decision.forward_speed_mps == pytest.approx(0.55)
    assert decision.state == cs.STOP
    assert decision.reason == "collision_distance"




def test_empty_and_undercovered_clouds_are_blind_stop():
    empty = cs.CollisionSupervisorCore(policy()).evaluate(inputs(points=()))
    assert empty.visibility == cs.VIS_BLIND
    assert empty.reason_mask & cs.COLLISION_BLIND
    partial = cs.CollisionSupervisorCore(policy()).evaluate(
        inputs(points=(static_point(10.0),), coverage_fraction=0.9, observed_coverage_bins=18)
    )
    assert partial.state == cs.STOP
    assert partial.visibility == cs.VIS_BLIND
    assert partial.reason_mask & cs.COLLISION_BLIND


def test_occlusion_inside_required_corridor_fail_stops():
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(points=(static_point(1.2),), occluded=True))
    assert decision.state == cs.STOP
    assert decision.visibility == cs.VIS_PARTIAL
    assert decision.reason_mask & cs.COLLISION_OCCLUDED


@pytest.mark.parametrize(
    "changes, reason_bit",
    [
        ({"cloud_stamp_s": 0.0}, cs.LIDAR_STALE),
        ({"odom_linear_mps": math.nan}, cs.CORRUPT_DATA),
        ({"cloud_stamp_s": 2.0}, cs.CORRUPT_DATA),
        ({"transform_ok": False}, cs.TF),
    ],
)
def test_stale_nan_future_and_transform_fail_closed(changes, reason_bit):
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(points=(static_point(10.0),), **changes))
    assert decision.state == cs.STOP
    assert decision.reason_mask & reason_bit


def test_downhill_can_make_policy_deceleration_ineffective():
    decision = cs.CollisionSupervisorCore(policy()).evaluate(
        inputs(points=(static_point(10.0),), pitch_downhill_rad=-0.10)
    )
    assert decision.state == cs.STOP
    assert decision.reason == "nonpositive_effective_deceleration"
    assert decision.required_stop_distance_m == -1.0


def test_stop_entry_is_one_frame_and_clear_needs_frames_and_half_second():
    core = cs.CollisionSupervisorCore(policy())
    stopped = core.evaluate(inputs(points=(static_point(1.0),)))
    assert stopped.state == cs.STOP
    for sequence, elapsed in enumerate((0.1, 0.2, 0.3, 0.4), start=2):
        held = core.evaluate(inputs(sequence, 1.0 + elapsed, (static_point(10.0),), speed=0.0))
        assert held.state == cs.STOP
    cleared = core.evaluate(inputs(6, 1.6, (static_point(10.0),), speed=0.0))
    assert cleared.state == cs.CLEAR
    assert cleared.consecutive_clear_frames == 5


def test_reverse_is_prohibited_until_rear_coverage_is_qualified():
    core = cs.CollisionSupervisorCore(policy())
    denied = core.evaluate(inputs(points=(static_point(-10.0),), speed=-0.1))
    assert denied.state == cs.STOP
    assert denied.reason == "reverse_coverage_not_qualified"
    assert denied.reason_mask & cs.COLLISION_BLIND
    core = cs.CollisionSupervisorCore(policy())
    decision = None
    for sequence, elapsed in enumerate((0.0, 0.1, 0.2, 0.3, 0.5), start=1):
        decision = core.evaluate(inputs(sequence, 1.0 + elapsed, (static_point(-10.0),), speed=-0.1,
                                        rear_coverage_qualified=True))
    assert decision.state == cs.CLEAR


@pytest.mark.parametrize("speed", [-0.009999, 0.009999])
def test_stationary_linear_deadband_normalizes_subcentimeter_drift(speed):
    core = cs.CollisionSupervisorCore(policy())
    decision = None
    for sequence, elapsed in enumerate((0.0, 0.1, 0.2, 0.3, 0.5), start=1):
        decision = core.evaluate(inputs(
            sequence, 1.0 + elapsed, (static_point(10.0),), speed=speed,
        ))

    assert decision.state == cs.CLEAR
    assert decision.reason != "reverse_coverage_not_qualified"
    assert decision.forward_speed_mps == 0.0


@pytest.mark.parametrize("speed", [-0.01, -0.010001])
def test_stationary_linear_deadband_preserves_true_reverse_boundary(speed):
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(-10.0),), speed=speed,
    ))

    assert decision.state == cs.STOP
    assert decision.reason == "reverse_coverage_not_qualified"
    assert decision.reason_mask & cs.COLLISION_BLIND
    assert decision.forward_speed_mps == speed


def test_stationary_linear_deadband_does_not_normalize_nonfinite_speed():
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),), odom_linear_mps=math.nan,
    ))

    assert decision.state == cs.STOP
    assert decision.reason_mask & cs.CORRUPT_DATA
    assert decision.forward_speed_mps == -1.0


def test_sequence_and_policy_identity_must_remain_consistent():
    core = cs.CollisionSupervisorCore(policy())
    core.evaluate(inputs(points=(static_point(10.0),)))
    repeated = core.evaluate(inputs(points=(static_point(10.0),)))
    assert repeated.reason_mask & cs.CORRUPT_DATA
    mismatched = cs.CollisionSupervisorCore(policy()).evaluate(
        inputs(points=(static_point(10.0),), policy_sha256="0" * 64)
    )
    assert mismatched.reason_mask & cs.POLICY_MISMATCH


def full_ground_cloud():
    result = []
    count = policy().coverage_bins
    elevations = (
        policy().coverage_min_elevation_rad
        + (index + 0.5)
        * (policy().coverage_max_elevation_rad - policy().coverage_min_elevation_rad)
        / policy().coverage_elevation_bins
        for index in range(policy().coverage_elevation_bins)
    )
    for elevation in elevations:
        for index in range(count):
            azimuth = -math.pi / 2.0 + (index + 0.5) * math.pi / count
            radius = 4.0
            result.append(cs.PointObservation(
                radius * math.cos(azimuth),
                radius * math.sin(azimuth),
                radius * math.tan(elevation),
            ))
    return tuple(result)

def flat_forward_cloud():
    count = policy().coverage_bins
    return tuple(
        cs.PointObservation(
            4.0 * math.cos(-math.pi / 2.0 + (index + 0.5) * math.pi / count),
            4.0 * math.sin(-math.pi / 2.0 + (index + 0.5) * math.pi / count),
            0.0,
        )
        for index in range(count)
    )


def gazebo_144x4_cloud(sensor_origin):
    elevations = (-0.10, -0.07333333333, -0.04666666667, -0.02)
    radius = 4.0
    return tuple(
        cs.PointObservation(
            sensor_origin[0] + radius * math.cos(azimuth) * math.cos(elevation),
            sensor_origin[1] + radius * math.sin(azimuth) * math.cos(elevation),
            sensor_origin[2] + radius * math.sin(elevation),
        )
        for elevation in elevations
        for azimuth in (
            -math.pi + index * 2.0 * math.pi / 143
            for index in range(144)
        )
    )


def gazebo_144x4_ground_returns(sensor_origin):
    return tuple(
        cs.PointObservation(
            sensor_origin[0] + distance * math.cos(azimuth) * math.cos(elevation),
            sensor_origin[1] + distance * math.sin(azimuth) * math.cos(elevation),
            0.0,
        )
        for elevation in (-0.10, -0.07333333333, -0.04666666667, -0.02)
        for azimuth in (
            -math.pi + index * 2.0 * math.pi / 143
            for index in range(144)
        )
        for distance in (sensor_origin[2] / -math.sin(elevation),)
    )


def test_visibility_is_sensor_origin_relative_for_translated_144x4_gazebo_cloud():
    processor = cs.CloudPreprocessorTracker(policy())
    origin = (0.22, 0.0, 0.621)
    sensor_frame = gazebo_144x4_cloud((0.0, 0.0, 0.0))
    translated = gazebo_144x4_cloud(origin)

    sensor_cells = processor._visibility_cells(sensor_frame, (0.0, 0.0, 0.0))
    translated_cells = processor._visibility_cells(translated, origin)

    assert translated_cells == sensor_cells
    assert len(translated_cells) == 72
    assert {elevation for _, elevation in translated_cells} == {0, 1}
def test_empty_world_ground_returns_qualify_after_window_and_clear_hysteresis():
    processor = cs.CloudPreprocessorTracker(policy())
    core = cs.CollisionSupervisorCore(policy())
    origin = (0.22, 0.0, 0.621)
    ground_returns = gazebo_144x4_ground_returns(origin)

    for stamp in (1.0, 1.1):
        result = processor.process(ground_returns, stamp, sensor_origin=origin)
        assert result.coverage_fraction == 0.0

    decisions = []
    for sequence, stamp in enumerate((1.2, 1.3, 1.4, 1.5), 1):
        result = processor.process(ground_returns, stamp, sensor_origin=origin)
        assert result.coverage_fraction == 1.0
        decisions.append(core.evaluate(inputs(
            sequence, stamp, result.points, speed=0.0,
            raw_point_count=result.raw_point_count,
            expected_coverage_bins=result.expected_coverage_bins,
            observed_coverage_bins=result.observed_coverage_bins,
            coverage_fraction=result.coverage_fraction,
        )))
    result = processor.process(ground_returns, 1.6, sensor_origin=origin)
    decisions.append(core.evaluate(inputs(
        5, 1.7, result.points, speed=0.0,
        raw_point_count=result.raw_point_count,
        expected_coverage_bins=result.expected_coverage_bins,
        observed_coverage_bins=result.observed_coverage_bins,
        coverage_fraction=result.coverage_fraction,
    )))

    assert all(decision.reason == "clear_hysteresis" for decision in decisions[:-1])
    assert decisions[-1].state == cs.CLEAR

def test_sensor_origin_changes_only_visibility_and_is_validated_transactionally():
    processor = cs.CloudPreprocessorTracker(policy())
    obstacle = (
        cs.PointObservation(1.001, 0.001, 0.20),
        cs.PointObservation(1.020, 0.020, 0.20),
        cs.PointObservation(2.0, 0.0, 0.20),
    )
    accepted = processor.process(obstacle, 1.0, sensor_origin=(0.22, 0.0, 0.621))
    prior_tracks = dict(processor._tracks)
    prior_next_track_id = processor._next_track_id
    prior_coverage_frames = tuple(processor._coverage_frames)
    rejected = processor.process(obstacle, 1.1, sensor_origin=(math.nan, 0.0, 0.621))

    actual_points = [(point.x, point.y, point.z) for point in accepted.points]
    assert len(actual_points) == 2
    assert [point[0] for point in actual_points] == pytest.approx([1.001, 2.0])
    assert [point[1] for point in actual_points] == pytest.approx([0.001, 0.0])
    assert [point[2] for point in actual_points] == pytest.approx([0.20, 0.20])
    assert not rejected.ok
    assert rejected.reason == "malformed_sensor_origin"
    assert rejected.points == ()
    assert processor._tracks == prior_tracks
    assert processor._next_track_id == prior_next_track_id
    assert tuple(processor._coverage_frames) == prior_coverage_frames

    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=rejected.points,
        raw_point_count=rejected.raw_point_count,
        coverage_fraction=rejected.coverage_fraction,
        expected_coverage_bins=rejected.expected_coverage_bins,
        observed_coverage_bins=rejected.observed_coverage_bins,
        transform_ok=rejected.ok,
    ))
    assert decision.state == cs.STOP




def test_preprocessor_requires_bounded_multiframe_evidence_before_clear():
    processor = cs.CloudPreprocessorTracker(policy())
    empty = processor.process((), 1.0)
    first = processor.process(full_ground_cloud(), 1.1)
    processor.process(full_ground_cloud(), 1.2)
    ground = processor.process(full_ground_cloud(), 1.3)

    assert empty.ok and empty.raw_point_count == 0 and not empty.points
    assert first.coverage_fraction == 0.0
    assert ground.raw_point_count == policy().coverage_bins * policy().coverage_elevation_bins
    assert ground.observed_coverage_bins == ground.expected_coverage_bins
    blind = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=empty.points, raw_point_count=empty.raw_point_count,
        expected_coverage_bins=empty.expected_coverage_bins,
        observed_coverage_bins=empty.observed_coverage_bins,
        coverage_fraction=empty.coverage_fraction,
    ))
    assert blind.reason == "empty_cloud_blind"

    core = cs.CollisionSupervisorCore(policy())
    decision = None
    for sequence, elapsed in enumerate((0.0, 0.1, 0.2, 0.3, 0.5), 1):
        decision = core.evaluate(inputs(
            sequence, 2.0 + elapsed, points=ground.points, speed=0.0,
            raw_point_count=ground.raw_point_count,
            expected_coverage_bins=ground.expected_coverage_bins,
            observed_coverage_bins=ground.observed_coverage_bins,
            coverage_fraction=ground.coverage_fraction,
        ))
    assert decision.state == cs.CLEAR
def test_coverage_holes_and_single_snapshot_remain_blind():
    full = full_ground_cloud()
    low_elevation_only = tuple(
        point for point in full
        if math.atan2(point.z, math.hypot(point.x, point.y)) < -0.06
    )
    corridor_hole = tuple(point for point in full if abs(math.atan2(point.y, point.x)) > 0.4)

    for cloud in (low_elevation_only, corridor_hole):
        processor = cs.CloudPreprocessorTracker(policy())
        result = None
        for stamp in (1.0, 1.1, 1.2):
            result = processor.process(cloud, stamp, 0.5, 0.0)
        assert result.coverage_fraction < policy().required_forward_coverage_fraction
        decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
            points=result.points,
            raw_point_count=result.raw_point_count,
            expected_coverage_bins=result.expected_coverage_bins,
            observed_coverage_bins=result.observed_coverage_bins,
            coverage_fraction=result.coverage_fraction,
        ))
        assert decision.state == cs.STOP
        assert decision.visibility == cs.VIS_BLIND

    processor = cs.CloudPreprocessorTracker(policy())
    first = processor.process(full, 2.0, 0.5, 0.0)
    assert first.coverage_fraction == 0.0
    assert first.observed_coverage_bins == 0


def test_coverage_regression_and_turn_geometry_change_reset_evidence():
    processor = cs.CloudPreprocessorTracker(policy())
    full = full_ground_cloud()
    for stamp in (1.0, 1.1, 1.2):
        complete = processor.process(full, stamp, 0.5, 0.0)
    assert complete.coverage_fraction == 1.0

    regressed = processor.process(full, 1.15, 0.5, 0.0)
    assert regressed.coverage_fraction == 0.0
    changed_turn = processor.process(full, 1.25, 0.5, 0.2)
    assert changed_turn.coverage_fraction == 0.0
    gap_processor = cs.CloudPreprocessorTracker(policy())
    for stamp in (2.0, 2.1, 2.2):
        complete = gap_processor.process(full, stamp, 0.5, 0.0)
    assert complete.coverage_fraction == 1.0
    gapped = gap_processor.process(full, 2.4, 0.5, 0.0)
    assert gapped.coverage_fraction == 0.0
    assert len(gap_processor._coverage_frames) == 1
    assert gap_processor._coverage_frames.maxlen == policy().coverage_max_frames


def test_coverage_uses_bounded_latest_suffix_across_nominal_jitter():
    full = full_ground_cloud()

    slightly_slow = cs.CloudPreprocessorTracker(policy())
    for stamp in (1.000, 1.101, 1.201):
        result = slightly_slow.process(full, stamp, 0.5, 0.0)
    assert result.coverage_fraction == 1.0
    assert [frame[0] for frame in slightly_slow._coverage_suffix()] == pytest.approx(
        [1.000, 1.101, 1.201]
    )

    slightly_fast = cs.CloudPreprocessorTracker(policy())
    for stamp in (2.000, 2.099, 2.198):
        result = slightly_fast.process(full, stamp, 0.5, 0.0)
    assert result.coverage_fraction == 0.0
    result = slightly_fast.process(full, 2.297, 0.5, 0.0)
    selected = slightly_fast._coverage_suffix()
    assert result.coverage_fraction == 1.0
    assert len(selected) == 4
    assert selected[-1][0] - selected[0][0] == pytest.approx(0.297)
    assert selected[-1][0] - selected[0][0] < (
        policy().coverage_window_s + policy().coverage_max_frame_gap_s
    )


def test_coverage_gap_motion_and_missing_cell_boundaries_remain_stops():
    full = full_ground_cloud()

    gapped = cs.CloudPreprocessorTracker(policy())
    gapped.process(full, 3.000, 0.5, 0.0)
    gapped.process(full, 3.100, 0.5, 0.0)
    result = gapped.process(full, 3.211001, 0.5, 0.0)
    assert result.coverage_fraction == 0.0
    assert len(gapped._coverage_frames) == 1

    linear_tolerance = policy().coverage_motion_linear_tolerance_mps
    stable_linear = cs.CloudPreprocessorTracker(policy())
    stable_linear.process(full, 4.000, 0.0, 0.0)
    stable_linear.process(full, 4.100, linear_tolerance, 0.0)
    result = stable_linear.process(full, 4.200, linear_tolerance, 0.0)
    assert result.coverage_fraction == 1.0
    result = stable_linear.process(
        full, 4.300, 2.0 * linear_tolerance + 1e-6, 0.0
    )
    assert result.coverage_fraction == 0.0

    angular_tolerance = policy().coverage_motion_angular_tolerance_rps
    stable_angular = cs.CloudPreprocessorTracker(policy())
    stable_angular.process(full, 5.000, 0.0, 0.0)
    stable_angular.process(full, 5.100, 0.0, angular_tolerance)
    result = stable_angular.process(full, 5.200, 0.0, angular_tolerance)
    assert result.coverage_fraction == 1.0
    result = stable_angular.process(
        full, 5.300, 0.0, 2.0 * angular_tolerance + 1e-6
    )
    assert result.coverage_fraction == 0.0

    corridor_hole = tuple(
        point for point in full
        if abs(math.atan2(point.y, point.x)) > 0.4
    )
    missing = cs.CloudPreprocessorTracker(policy())
    for stamp in (6.000, 6.101, 6.201):
        result = missing.process(corridor_hole, stamp, 0.5, 0.0)
    assert result.observed_coverage_bins < result.expected_coverage_bins
    assert result.coverage_fraction < policy().required_forward_coverage_fraction


def test_adequate_temporal_coverage_clears_only_after_existing_hysteresis():
    processor = cs.CloudPreprocessorTracker(policy())
    core = cs.CollisionSupervisorCore(policy())
    full = full_ground_cloud()
    for stamp in (1.0, 1.1):
        processor.process(full, stamp, 0.0, 0.0)

    decision = None
    for sequence, stamp in enumerate((1.2, 1.3, 1.4, 1.5, 1.7), 1):
        result = processor.process(full, stamp, 0.0, 0.0)
        decision = core.evaluate(inputs(
            sequence, stamp, result.points, speed=0.0,
            raw_point_count=result.raw_point_count,
            expected_coverage_bins=result.expected_coverage_bins,
            observed_coverage_bins=result.observed_coverage_bins,
            coverage_fraction=result.coverage_fraction,
        ))
        if sequence < 5:
            assert decision.state == cs.STOP
        if stamp == 1.5:
            processor.process(full, 1.6, 0.0, 0.0)
    assert decision.state == cs.CLEAR
    assert decision.consecutive_clear_frames == 5



def test_ground_band_boundaries_do_not_remove_curb_or_overhang_returns():
    processor = cs.CloudPreprocessorTracker(policy())
    points = flat_forward_cloud() + (
        cs.PointObservation(1.0, 0.0, policy().ground_max_z_m),
        cs.PointObservation(1.2, 0.0, policy().ground_max_z_m + 0.001),
        cs.PointObservation(1.5, 0.0, policy().ground_min_z_m - 0.001),
    )
    result = processor.process(points, 1.0)
    assert result.ok
    assert [(point.x, point.z) for point in result.points] == [
        (1.2, pytest.approx(policy().ground_max_z_m + 0.001)),
        (1.5, pytest.approx(policy().ground_min_z_m - 0.001)),
    ]


def test_preprocessing_is_transactional_and_voxels_clusters_are_deterministic():
    near = cs.PointObservation(1.001, 0.001, 0.20)
    farther_same_voxel = cs.PointObservation(1.020, 0.020, 0.20)
    separate = cs.PointObservation(2.0, 0.0, 0.20)
    first = cs.CloudPreprocessorTracker(policy()).process(
        (farther_same_voxel, separate, near), 1.0
    )
    second = cs.CloudPreprocessorTracker(policy()).process(
        (near, separate, farther_same_voxel), 1.0
    )
    assert first.points == second.points
    assert [point.x for point in first.points] == pytest.approx([near.x, separate.x])

    processor = cs.CloudPreprocessorTracker(policy())
    before = processor.process((near,), 1.0)
    rejected = processor.process((near, cs.PointObservation(math.nan, 0.0, 0.2)), 1.1)
    after = processor.process((cs.PointObservation(1.01, 0.0, 0.2),), 1.2)
    assert not rejected.ok and rejected.points == ()
    assert before.points[0].track_id == after.points[0].track_id


def test_tracks_have_stable_ids_finite_velocity_covariance_and_bounded_drop():
    processor = cs.CloudPreprocessorTracker(policy())
    samples = []
    for stamp, x in ((1.0, 2.0), (1.1, 2.1), (1.2, 2.2)):
        samples.append(processor.process((cs.PointObservation(x, 0.0, 0.2),), stamp).points[0])
    assert len({point.track_id for point in samples}) == 1
    assert samples[-1].observation_count == 3
    assert samples[-1].vx == pytest.approx(1.0)
    assert samples[-1].vy == pytest.approx(0.0)
    assert samples[-1].covariance_valid

    dropped = processor.process((cs.PointObservation(2.21, 0.0, 0.2),), 1.6).points[0]
    assert dropped.track_id != samples[-1].track_id
    assert dropped.observation_count == 1
    assert dropped.vx is None and not dropped.covariance_valid
    assert cs.CollisionSupervisorCore(policy())._classify(dropped)[0] == cs.MOTION_AMBIGUOUS
def test_tracker_removes_ego_translation_and_rotation_from_static_obstacle_velocity():
    straight = cs.CloudPreprocessorTracker(policy())
    straight_samples = []
    for elapsed in (0.0, 0.1, 0.2):
        straight_samples.append(
            straight.process(
                (cs.PointObservation(2.0 - 0.1 * elapsed, 0.0, 0.2),),
                1.0 + elapsed,
                linear_speed_mps=0.1,
            ).points[0]
        )
    assert straight_samples[-1].vx == pytest.approx(0.0, abs=1e-9)
    assert straight_samples[-1].vy == pytest.approx(0.0, abs=1e-9)
    assert cs.CollisionSupervisorCore(policy())._classify(straight_samples[-1])[0] == cs.MOTION_STATIC

    turning = cs.CloudPreprocessorTracker(policy())
    turning_samples = []
    angular_speed = 0.1
    for elapsed in (0.0, 0.1, 0.2):
        yaw = angular_speed * elapsed
        turning_samples.append(
            turning.process(
                (
                    cs.PointObservation(
                        2.0 * math.cos(yaw),
                        -2.0 * math.sin(yaw),
                        0.2,
                    ),
                ),
                2.0 + elapsed,
                angular_speed_rps=angular_speed,
            ).points[0]
        )
    assert turning_samples[-1].vx == pytest.approx(0.0, abs=0.002)
    assert turning_samples[-1].vy == pytest.approx(0.0, abs=0.002)
    assert cs.CollisionSupervisorCore(policy())._classify(turning_samples[-1])[0] == cs.MOTION_STATIC


def test_tracker_uses_per_interval_compensation_for_changing_ego_twist():
    processor = cs.CloudPreprocessorTracker(policy())
    x, y = 2.0, 0.6
    stamp = 3.0
    samples = []

    for linear, angular in (
        (0.0, 0.0),
        (0.10, 0.05),
        (0.25, -0.08),
        (0.05, 0.12),
    ):
        if samples:
            dt = 0.1
            yaw_delta = angular * dt
            if abs(angular) < 1e-9:
                translation_x = linear * dt
                translation_y = 0.0
            else:
                radius = linear / angular
                translation_x = radius * math.sin(yaw_delta)
                translation_y = radius * (1.0 - math.cos(yaw_delta))
            relative_x = x - translation_x
            relative_y = y - translation_y
            x = (
                math.cos(yaw_delta) * relative_x
                + math.sin(yaw_delta) * relative_y
            )
            y = (
                -math.sin(yaw_delta) * relative_x
                + math.cos(yaw_delta) * relative_y
            )
            stamp += dt
        samples.append(processor.process(
            (cs.PointObservation(x, y, 0.2),),
            stamp,
            linear_speed_mps=linear,
            angular_speed_rps=angular,
        ).points[0])

    assert len({sample.track_id for sample in samples}) == 1
    assert samples[-1].observation_count == 4
    assert samples[-1].vx == pytest.approx(0.0, abs=1e-9)
    assert samples[-1].vy == pytest.approx(0.0, abs=1e-9)
    assert samples[-1].covariance_valid
    assert (
        cs.CollisionSupervisorCore(policy())._classify(samples[-1])[0]
        == cs.MOTION_STATIC
    )



def self_return_cloud():
    radius = min(policy().footprint_length_m, policy().footprint_width_m) / 4.0
    return tuple(
        cs.PointObservation(
            radius * math.cos(azimuth),
            radius * math.sin(azimuth),
            radius * math.tan(elevation),
        )
        for elevation in (
            policy().coverage_min_elevation_rad
            + (index + 0.5)
            * (policy().coverage_max_elevation_rad - policy().coverage_min_elevation_rad)
            / policy().coverage_elevation_bins
            for index in range(policy().coverage_elevation_bins)
        )
        for azimuth in (
            -math.pi / 2.0 + (index + 0.5) * math.pi / policy().coverage_bins
            for index in range(policy().coverage_bins)
        )
    )


def test_self_returns_are_masked_from_obstacles_but_preserve_raw_coverage():
    processor = cs.CloudPreprocessorTracker(policy())
    cloud = self_return_cloud()

    for stamp in (1.0, 1.1, 1.2):
        result = processor.process(cloud, stamp, 0.5, 0.0)

    assert result.ok
    assert result.points == ()
    assert result.raw_point_count == len(cloud)
    assert result.coverage_fraction == 1.0
    assert result.observed_coverage_bins == result.expected_coverage_bins


def test_self_mask_boundary_is_inclusive_but_external_approach_is_never_masked():
    processor = cs.CloudPreprocessorTracker(policy())
    half_length = policy().footprint_length_m / 2.0
    boundary = cs.PointObservation(half_length, 0.0, policy().ground_max_z_m + 0.01)
    outside = cs.PointObservation(
        half_length + 0.001, 0.0, policy().ground_max_z_m + 0.01,
    )

    masked = processor.process((boundary,), 1.0)
    retained = processor.process((outside,), 1.1)

    assert masked.points == ()
    assert len(retained.points) == 1
    assert retained.points[0].x == pytest.approx(outside.x)
    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=retained.points,
        raw_point_count=retained.raw_point_count,
        coverage_fraction=1.0,
    ))
    assert decision.state == cs.STOP
    assert decision.reason_mask & cs.COLLISION
    assert decision.reason_mask & cs.COLLISION_TTC
    assert decision.reason == "ambiguous_obstacle"


def test_self_mask_does_not_hide_obstacle_approaching_from_outside():
    processor = cs.CloudPreprocessorTracker(policy())
    half_length = policy().footprint_length_m / 2.0
    results = [
        processor.process((
            cs.PointObservation(x, 0.0, policy().ground_max_z_m + 0.01),
        ), stamp)
        for stamp, x in ((1.0, half_length + 0.2), (1.1, half_length + 0.1),
                         (1.2, half_length + 0.001))
    ]

    assert all(len(result.points) == 1 for result in results)
    assert len({result.points[0].track_id for result in results}) == 1
    assert results[-1].points[0].vx < 0.0


def hold_inputs(sequence, now, points):
    return inputs(
        sequence=sequence, now=now, points=points, speed=0.0,
        nav_available=False, intent_stamp_s=now, intent_behavior=cs.INTENT_HOLD,
        intent_max_linear_mps=0.0, intent_max_angular_rps=0.0,
    )


def test_isolated_ambiguous_wall_noise_under_hold_clears_after_hysteresis():
    core = cs.CollisionSupervisorCore(policy())
    wall_noise = tuple(
        cs.PointObservation(1.2, -0.8 + index * 0.2, 0.2)
        for index in range(9)
    )
    decisions = [
        core.evaluate(hold_inputs(index + 1, 1.0 + elapsed, wall_noise))
        for index, elapsed in enumerate((0.0, 0.1, 0.2, 0.3, 0.5))
    ]

    assert all(decision.reason != "ambiguous_obstacle" for decision in decisions)
    assert decisions[-1].state == cs.CLEAR


def test_ambiguous_overlap_under_hold_remains_stop():
    core = cs.CollisionSupervisorCore(policy())
    overlap = cs.PointObservation(
        policy().footprint_length_m / 2.0
        + policy().localization_uncertainty_m,
        0.0,
        0.2,
    )

    decision = core.evaluate(hold_inputs(1, 1.0, (overlap,)))

    assert decision.state == cs.STOP
    assert decision.reason == "collision_distance"
    assert decision.reason_mask & cs.COLLISION_DISTANCE


def test_ambiguous_point_under_proceed_preserves_conservative_stop():
    point = cs.PointObservation(1.2, 0.2, 0.2)

    decision = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(point,), speed=0.0, intent_behavior=cs.INTENT_PROCEED,
    ))

    assert decision.state == cs.STOP
    assert decision.reason == "ambiguous_obstacle"
    assert decision.reason_mask & cs.COLLISION_TTC


def test_tracked_dynamic_approach_under_hold_remains_stop():
    point = cs.PointObservation(
        1.0, 0.0, 0.2, vx=-1.0, vy=0.0, track_id="approach",
        observation_count=policy().minimum_observations, covariance_valid=True,
    )

    decision = cs.CollisionSupervisorCore(policy()).evaluate(
        hold_inputs(1, 1.0, (point,))
    )

    assert decision.state == cs.STOP
    assert decision.obstacle_motion == cs.MOTION_DYNAMIC
    assert decision.reason == "collision_distance"
    assert decision.reason_mask & cs.COLLISION
    assert decision.reason_mask & cs.COLLISION_DISTANCE


def test_classification_requires_history_and_covariance_for_dynamic_motion():
    core = cs.CollisionSupervisorCore(policy())
    static = cs.PointObservation(
        2.0, 0.0, 0.2, 0.01, 0.0, "1", policy().minimum_observations, False
    )
    dynamic = dataclasses.replace(static, vx=1.0, covariance_valid=True)
    uncertain = dataclasses.replace(static, vx=10.0, covariance_valid=False)
    assert core._classify(static)[0] == cs.MOTION_STATIC
    assert core._classify(dynamic)[0] == cs.MOTION_DYNAMIC
    motion, vx, vy = core._classify(uncertain)
    assert motion == cs.MOTION_AMBIGUOUS
    assert math.hypot(vx, vy) == pytest.approx(policy().ambiguous_approach_speed_mps)


def test_hold_allows_stationary_evidence_without_nav_and_rejects_nonzero_caps():
    core = cs.CollisionSupervisorCore(policy())
    held = core.evaluate(inputs(
        points=(static_point(10.0),), speed=0.0, nav_available=False,
        intent_stamp_s=0.99, intent_behavior=cs.INTENT_HOLD,
        intent_max_linear_mps=0.0, intent_max_angular_rps=0.0,
    ))
    assert held.reason != "missing_nav_command"
    malformed = cs.CollisionSupervisorCore(policy()).evaluate(inputs(
        points=(static_point(10.0),), speed=0.0, nav_available=False,
        intent_stamp_s=0.99, intent_behavior=cs.INTENT_HOLD,
        intent_max_linear_mps=0.01, intent_max_angular_rps=0.0,
    ))
    assert malformed.reason == "malformed_hold_intent"


@pytest.mark.parametrize(
    "elapsed_s,expected_pre_hold_rejection",
    (
        (policy().first_command_grace_s - 0.000001, False),
        (policy().first_command_grace_s, False),
        (policy().first_command_grace_s + 0.000001, True),
    ),
)
def test_pre_hold_nav_is_accepted_only_during_inclusive_first_command_grace(
    elapsed_s, expected_pre_hold_rejection
):
    core = cs.CollisionSupervisorCore(policy())
    core.evaluate(inputs(
        1, 1.0, (static_point(10.0),), speed=0.0, nav_available=False,
        intent_stamp_s=1.0, intent_behavior=cs.INTENT_HOLD,
        intent_max_linear_mps=0.0, intent_max_angular_rps=0.0,
    ))
    core.evaluate(inputs(
        2, 2.0, (static_point(10.0),), speed=0.0, nav_available=False,
        intent_stamp_s=2.0, intent_behavior=cs.INTENT_PROCEED,
    ))
    decision = core.evaluate(inputs(
        3, 2.0 + elapsed_s, (static_point(10.0),), speed=0.0,
        nav_available=True, nav_stamp_s=1.99, intent_stamp_s=2.0 + elapsed_s,
        intent_behavior=cs.INTENT_PROCEED,
    ))

    assert (decision.reason == "pre_hold_nav_command") is expected_pre_hold_rejection


def test_missing_nav_command_is_rejected_immediately_after_first_command_grace():
    core = cs.CollisionSupervisorCore(policy())
    core.evaluate(inputs(
        1, 1.0, (static_point(10.0),), speed=0.0, nav_available=False,
        intent_stamp_s=1.0, intent_behavior=cs.INTENT_HOLD,
        intent_max_linear_mps=0.0, intent_max_angular_rps=0.0,
    ))
    core.evaluate(inputs(
        2, 2.0, (static_point(10.0),), speed=0.0, nav_available=False,
        intent_stamp_s=2.0, intent_behavior=cs.INTENT_PROCEED,
    ))
    within_grace = core.evaluate(inputs(
        3, 2.0 + policy().first_command_grace_s, (static_point(10.0),),
        speed=0.0, nav_available=False,
        intent_stamp_s=2.0 + policy().first_command_grace_s,
        intent_behavior=cs.INTENT_PROCEED,
    ))
    expired = core.evaluate(inputs(
        4, 2.0 + policy().first_command_grace_s + 0.000001,
        (static_point(10.0),), speed=0.0, nav_available=False,
        intent_stamp_s=2.0 + policy().first_command_grace_s + 0.000001,
        intent_behavior=cs.INTENT_PROCEED,
    ))

    assert within_grace.reason != "missing_nav_command"
    assert expired.reason == "missing_nav_command"


def test_watchdog_initial_no_cloud_and_stale_cloud_are_canonical_stop_heartbeats():
    state = cs.CollisionWatchdogState(policy().cloud_ttl_s)
    core = cs.CollisionSupervisorCore(policy())

    initial_age = state.stale_age(10.0)
    initial = core.stale_cloud_decision(1, 10.0, initial_age)
    assert initial.state == cs.STOP
    assert initial.signal_state == cs.SIGNAL_STOP
    assert initial.reason == "stale_cloud"
    assert initial.reason_mask == cs.LIDAR_STALE | cs.SENSOR_STALE | cs.COLLISION
    assert initial.source == "collision_supervisor"
    assert initial.policy_id == policy().policy_id
    assert initial.policy_sha256 == policy().policy_sha256

    state.observe_cloud(10.0, 10.0)
    assert state.stale_age(10.0 + policy().cloud_ttl_s - 0.001) is None
    stale_age = state.stale_age(10.0 + policy().cloud_ttl_s)
    stale = core.stale_cloud_decision(2, 10.0 + policy().cloud_ttl_s, stale_age)
    assert stale.reason_mask == initial.reason_mask
    assert stale.sequence > initial.sequence
    assert stale.evaluation_stamp > initial.evaluation_stamp


def test_watchdog_never_publishes_while_fresh_cloud_decision_is_current():
    state = cs.CollisionWatchdogState(policy().cloud_ttl_s)
    state.observe_cloud(20.0, 20.0)
    assert state.stale_age(20.0) is None
    assert state.stale_age(20.0 + policy().cloud_ttl_s / 2.0) is None


def test_watchdog_sequences_are_monotonic_and_latch_fresh_core_evaluation():
    core = cs.CollisionSupervisorCore(policy())
    first = core.stale_cloud_decision(7, 30.0, 1.0)
    second = core.stale_cloud_decision(8, 30.1, 1.1)
    assert (first.sequence, second.sequence) == (7, 8)
    with pytest.raises(ValueError, match="monotonic"):
        core.stale_cloud_decision(8, 30.2, 1.2)

    fresh = core.evaluate(inputs(
        sequence=9, now=30.2, points=(static_point(10.0),), speed=0.0,
        cloud_stamp_s=30.19, odom_stamp_s=30.19, nav_stamp_s=30.19,
        safe_stamp_s=30.19,
    ))
    assert fresh.sequence == 9
    assert fresh.state == cs.STOP
    assert fresh.reason == "clear_hysteresis"


def test_ros_adapter_uses_queue_one_and_two_lock_ownership_contract():
    source = SCRIPT.read_text()
    assert source.count("queue_size=1") >= 7
    assert "rospy.Timer(" in source
    assert "self._input_lock = threading.RLock()" in source
    assert "self._decision_lock = threading.RLock()" in source
    assert "with self._input_lock:" in source
    assert "with self._decision_lock:" in source
    assert "with self._lock:" not in source
    cloud_callback = source[source.index("    def _cloud_cb"):source.index("    def _evaluate_cloud")]
    watchdog_callback = source[source.index("    def _watchdog_cb"):source.index("    def _next_sequence")]
    input_callbacks = source[source.index("    def _odom_cb"):source.index("    def _cloud_cb")]
    assert "with self._decision_lock:" in cloud_callback
    assert "with self._decision_lock:" in watchdog_callback
    assert "self._decision_lock" not in input_callbacks
    evaluation = source[source.index("    def _evaluate_cloud"):source.index("    def _schedule_cloud_deadline")]
    assert evaluation.index("self.preprocessor.process(") < evaluation.index(
        "_postprocess_evaluation_time("
    ) < evaluation.index("inputs = CollisionInputs(")

def reference_clusters(points, cluster_policy):
    tolerance_sq = cluster_policy.cluster_tolerance_m ** 2
    remaining = set(range(len(points)))
    clusters = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        pending, members = [seed], []
        while pending:
            index = pending.pop(0)
            members.append(index)
            neighbors = [
                other for other in sorted(remaining)
                if ((points[index].x - points[other].x) ** 2
                    + (points[index].y - points[other].y) ** 2
                    + (points[index].z - points[other].z) ** 2) <= tolerance_sq
            ]
            for other in neighbors:
                remaining.remove(other)
                pending.append(other)
        if len(members) >= cluster_policy.cluster_min_points:
            count = float(len(members))
            clusters.append((
                sum(points[i].x for i in members) / count,
                sum(points[i].y for i in members) / count,
                sum(points[i].z for i in members) / count,
            ))
    return tuple(sorted(clusters))


@pytest.mark.parametrize("layout", [
    "bucket_edges",
    "transitive_chain",
    "separated_clusters",
    "duplicates_and_negatives",
])
def test_spatial_bucket_clustering_matches_quadratic_reference(layout):
    cluster_policy = policy()
    tolerance = cluster_policy.cluster_tolerance_m
    layouts = {
        "bucket_edges": (
            (-0.01, 0.0, 0.0), (0.01, 0.0, 0.0),
            (tolerance - 0.01, tolerance - 0.01, 0.0),
            (tolerance + 0.01, tolerance + 0.01, 0.0),
        ),
        "transitive_chain": tuple(
            (index * tolerance * 0.75, 0.0, 0.0) for index in range(6)
        ),
        "separated_clusters": (
            (0.0, 0.0, 0.0), (tolerance * 0.5, 0.0, 0.0),
            (tolerance * 3.0, 1.0, 0.0), (tolerance * 3.5, 1.0, 0.0),
            (-tolerance * 4.0, -1.0, 0.2),
        ),
        "duplicates_and_negatives": (
            (-tolerance, -tolerance, -tolerance),
            (-tolerance, -tolerance, -tolerance),
            (-tolerance * 1.5, -tolerance, -tolerance),
            (tolerance * 2.0, tolerance * 2.0, tolerance * 2.0),
        ),
    }
    points = tuple(cs.PointObservation(*coordinates) for coordinates in layouts[layout])
    tracker = cs.CloudPreprocessorTracker(cluster_policy)

    expected = reference_clusters(points, cluster_policy)
    assert tracker._cluster(points) == expected
    assert tracker._cluster(points) == expected


def test_spatial_bucket_clustering_checks_only_local_candidates_for_bounded_cloud():
    cluster_policy = policy()
    tolerance = cluster_policy.cluster_tolerance_m
    points = tuple(
        cs.PointObservation(
            column * tolerance * 3.0 + member * tolerance * 0.5,
            row * tolerance * 3.0,
            0.0,
        )
        for row in range(12)
        for column in range(24)
        for member in range(2)
    )
    candidate_checks = [0]
    tracker = cs.CloudPreprocessorTracker(cluster_policy)

    actual = tracker._cluster(points, candidate_checks)

    assert actual == reference_clusters(points, cluster_policy)
    assert candidate_checks[0] <= len(points)
    assert candidate_checks[0] < len(points) * (len(points) - 1) // 20


def reference_associations(tracker, clusters, stamp_s):
    active = {
        track_id: track for track_id, track in tracker._tracks.items()
        if 0.0 < stamp_s - track.stamp_s <= tracker.policy.association_max_age_s
    }
    candidates = []
    for cluster_index, (x, y, z) in enumerate(clusters):
        for track_id, track in active.items():
            displacement = math.sqrt(
                (x - track.x) ** 2 + (y - track.y) ** 2 + (z - track.z) ** 2
            )
            if displacement <= tracker.policy.association_max_displacement_m:
                candidates.append((displacement, track_id, cluster_index))
    assigned_tracks, assigned_clusters, associations = set(), set(), {}
    for displacement, track_id, cluster_index in sorted(candidates):
        if track_id in assigned_tracks or cluster_index in assigned_clusters:
            continue
        ties = [
            item for item in candidates
            if abs(item[0] - displacement) <= 1e-12
            and (item[1] == track_id or item[2] == cluster_index)
        ]
        if len(ties) != 1:
            continue
        assigned_tracks.add(track_id)
        assigned_clusters.add(cluster_index)
        associations[cluster_index] = track_id
    return associations


@pytest.mark.parametrize("previous,current", [
    (
        ((-0.4, 0.0, 0.0), (0.4, 0.0, 0.0)),
        ((0.3, 0.0, 0.0), (-0.3, 0.0, 0.0)),
    ),
    (
        ((-0.2, 0.0, 0.0), (0.2, 0.0, 0.0)),
        ((0.0, 0.0, 0.0),),
    ),
    (
        ((0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.6, 0.0, 0.0)),
        ((0.1, 0.0, 0.0), (0.4, 0.0, 0.0), (0.7, 0.0, 0.0)),
    ),
    (
        ((-2.0, -1.0, -0.2), (-1.0, -2.0, -0.3)),
        ((-2.1, -1.0, -0.2), (-0.9, -2.0, -0.3)),
    ),
])
def test_spatial_track_association_matches_quadratic_reference(previous, current):
    tracker = cs.CloudPreprocessorTracker(policy())
    tracker._associate(previous, 1.0)
    expected = reference_associations(tracker, current, 1.1)
    next_track_id = tracker._next_track_id

    observations = tracker._associate(current, 1.1)

    expected_ids = []
    for index in range(len(current)):
        track_id = expected.get(index)
        if track_id is None:
            track_id = next_track_id
            next_track_id += 1
        expected_ids.append(str(track_id))
    assert tuple(point.track_id for point in observations) == tuple(expected_ids)


def test_spatial_track_association_checks_only_local_candidates_for_sparse_tracks():
    tracker = cs.CloudPreprocessorTracker(policy())
    limit = policy().association_max_displacement_m
    previous = tuple(
        (column * limit * 3.0, row * limit * 3.0, 0.0)
        for row in range(12)
        for column in range(24)
    )
    tracker._associate(previous, 1.0)
    current = tuple((x + limit * 0.1, y, z) for x, y, z in previous)
    candidate_checks = [0]

    expected = reference_associations(tracker, current, 1.1)
    observations = tracker._associate(current, 1.1, candidate_checks)

    assert tuple(int(point.track_id) for point in observations) == tuple(
        expected[index] for index in range(len(current))
    )
    assert candidate_checks[0] <= len(current)
    assert candidate_checks[0] < len(previous) * len(current) // 20
