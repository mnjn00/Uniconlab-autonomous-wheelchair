"""Validate one sensor-only perception/localization shadow snapshot."""

import math


DIAGNOSTIC_KEYS = (
    "tracking_state",
    "raw_scan_points",
    "dynamic_returns_dropped",
    "post_box_points",
    "map_filtered",
    "post_map_points",
    "rolling_submap_points",
)


def _positive_box(box):
    size = box.get("size")
    return (
        isinstance(size, list)
        and len(size) == 3
        and all(
            isinstance(value, (int, float))
            and math.isfinite(value)
            and value > 0.0
            for value in size
        )
    )


def validate_snapshot(summary, dynamic_boxes, diagnostics):
    """Return normalized counts, or raise on an unsafe/incomplete snapshot."""
    if summary.get("status") != "OK":
        raise ValueError("objects_summary is not OK")
    objects = summary.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects_summary has no object list")
    if not isinstance(dynamic_boxes, list):
        raise ValueError("dynamic boxes are not a list")
    if len(objects) != len(dynamic_boxes):
        raise ValueError(
            "object/box count mismatch: %d != %d"
            % (len(objects), len(dynamic_boxes))
        )
    if not all(_positive_box(box) for box in dynamic_boxes):
        raise ValueError("dynamic box sizes must be positive finite triples")
    missing = [key for key in DIAGNOSTIC_KEYS if key not in diagnostics]
    if missing:
        raise ValueError("missing diagnostics: %s" % ", ".join(missing))
    for key in DIAGNOSTIC_KEYS[1:]:
        value = float(diagnostics[key])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("%s must be finite and non-negative" % key)
    raw = int(diagnostics["raw_scan_points"])
    post_box = int(diagnostics["post_box_points"])
    post_map = int(diagnostics["post_map_points"])
    if post_box > raw or post_map > post_box:
        raise ValueError("registration filter stage counts are not monotonic")
    return {
        "object_count": len(objects),
        "dynamic_box_count": len(dynamic_boxes),
        "tracking_state": diagnostics["tracking_state"],
        "raw_scan_points": raw,
        "post_box_points": post_box,
        "post_map_points": post_map,
    }
