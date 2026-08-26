"""Objective-level perception regressions.

The implementation lives with the ROS package; these imports keep the
operator-facing verification command stable from the repository root.
"""

from src.static_livox_localization.test.test_cluster_pipeline import (
    test_current_outside_band_verdict_remains_authoritative_with_tracking,
    test_mapped_wall_is_not_published_as_an_obstacle,
    test_novel_person_is_published_with_valid_box,
    test_short_detector_dropout_publishes_same_id_as_predicted_box,
)


__all__ = [
    "test_mapped_wall_is_not_published_as_an_obstacle",
    "test_novel_person_is_published_with_valid_box",
    "test_short_detector_dropout_publishes_same_id_as_predicted_box",
    "test_current_outside_band_verdict_remains_authoritative_with_tracking",
]
