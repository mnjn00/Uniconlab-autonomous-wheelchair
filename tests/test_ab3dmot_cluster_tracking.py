"""Regression tests for the lightweight AB3DMOT-style cluster tracker."""

import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "src" / \
    "static_livox_localization" / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    import cluster_tracking as tracking
finally:
    sys.path.pop(0)


def test_short_occlusion_keeps_identity_and_predicts_position():
    tracker = tracking.Tracker()
    first = tracker.update([(0.0, 0.0, "person")], 10.0)[0]
    tracker.update([(0.2, 0.0, "person")], 10.2)
    tracker.update([], 10.4)
    coast = tracker.coasting(10.4)

    assert len(coast) == 1
    assert coast[0].id == first.id
    assert coast[0].predicted_only
    assert coast[0].position[0] > 0.2

    reacquired = tracker.update([(0.6, 0.0, "person")], 10.6)[0]
    assert reacquired.id == first.id
    assert not reacquired.predicted_only


def test_tracks_expire_after_bounded_occlusion():
    tracker = tracking.Tracker(drop_after_s=1.0)
    original = tracker.update([(1.0, 0.0, "person")], 1.0)[0]
    tracker.update([], 1.5)
    replacement = tracker.update([(1.0, 0.0, "person")], 2.1)[0]

    assert replacement.id != original.id
    assert tracker.coasting(2.1) == []


def test_hungarian_assignment_preserves_global_best_pairing():
    tracker = tracking.Tracker(gate_m=3.0,
                               max_association_distance_m=3.0)
    left, right = tracker.update([
        (-0.5, 0.0, "person"),
        (0.5, 0.0, "person"),
    ], 1.0)
    # Reversed input order must not reverse identities.
    result = tracker.update([
        (0.45, 0.0, "person"),
        (-0.45, 0.0, "person"),
    ], 1.2)

    assert result[0].id == right.id
    assert result[1].id == left.id


def test_person_label_survives_single_footprint_flicker():
    tracker = tracking.Tracker()
    track = tracker.update([(2.0, 0.0, "person")], 1.0)[0]
    tracker.update([(2.0, 0.0, "person")], 1.2)
    tracker.update([(2.0, 0.0, "obstacle")], 1.4)

    assert track.label == "person"


def test_constant_velocity_filter_classifies_a_crossing_person_as_moving():
    tracker = tracking.Tracker()
    track = None
    for index in range(12):
        stamp = 20.0 + index * 0.2
        track = tracker.update([
            (3.0, -1.0 + index * 0.2, "person")
        ], stamp)[0]

    assert track is not None
    assert track.speed_mps() == pytest.approx(1.0, abs=0.15)
    assert track.motion(22.2) == tracking.MOVING


def test_size_change_is_soft_evidence_not_a_class_or_identity_gate():
    tracker = tracking.Tracker()
    original = tracker.update([(
        3.0, 0.0, "person", {"size": [0.5, 0.5, 1.5]}
    )], 1.0)[0]
    changed = tracker.update([(
        3.1, 0.0, "obstacle", {"size": [0.8, 0.6, 1.4]}
    )], 1.2)[0]

    assert changed.id == original.id
