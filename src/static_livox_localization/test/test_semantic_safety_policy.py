import math
import sys
from pathlib import Path

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from semantic_safety_policy import (  # noqa: E402
    PersonStopLatch,
    ThreatView,
    decide_semantic_stop,
    stopping_distance,
)


def person(distance, track_id=1):
    return ThreatView(distance, "moving", "person", track_id)


def test_stopping_distance_grows_with_speed_and_age():
    slow = stopping_distance(0.1, 0.1, 0.0)
    fast = stopping_distance(0.8, 0.8, 0.0)
    old = stopping_distance(0.8, 0.8, 0.4)
    assert slow < fast < old


def test_person_stop_remains_latched_when_radius_shrinks():
    latch = PersonStopLatch(0.30)
    assert latch.update(person(1.10), 1.20)
    # Stopped chair now computes a smaller radius, but the person did not move.
    assert latch.update(person(1.10), 0.90)
    assert latch.release_distance_m >= 1.50


def test_person_latch_releases_after_moving_away():
    latch = PersonStopLatch(0.30)
    assert latch.update(person(1.0), 1.2)
    assert not latch.update(person(1.6), 0.9)
    assert latch.release_distance_m is None


def test_identity_change_does_not_inherit_the_old_latch_when_clear():
    latch = PersonStopLatch(0.30)
    assert latch.update(person(1.0, 1), 1.2)
    assert not latch.update(person(2.0, 2), 0.9)


def test_unusable_perception_fails_closed():
    decision = decide_semantic_stop(
        False, 0.0, 0.0, 1.5, 0.6, 1.0,
        None, None, PersonStopLatch())
    assert decision.reason == "PERCEPTION_UNUSABLE"


def test_moving_object_inside_envelope_stops():
    threat = ThreatView(0.8, "unknown", "vehicle", 3)
    decision = decide_semantic_stop(
        True, 0.1, 0.1, 1.5, 0.6, 1.0,
        None, threat, PersonStopLatch())
    assert decision.reason == "MOVING_OBJECT"


def test_static_non_person_is_left_to_the_planner():
    threat = ThreatView(0.8, "static", "vehicle", 3)
    decision = decide_semantic_stop(
        True, 0.1, 0.1, 1.5, 0.6, 1.0,
        None, threat, PersonStopLatch())
    assert not decision.blocked


def test_nonfinite_inputs_fail_closed():
    assert math.isinf(stopping_distance(math.nan, 0.2, 0.1))
