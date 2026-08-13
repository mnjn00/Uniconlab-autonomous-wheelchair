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
COMMAND_TOPICS = (
    "/cmd_vel_raw",
    "/cmd_vel_gated",
    "/cmd_vel",
    "/wheel_cmd",
    "/mode_cmd",
)
MOTION_NODE_TOKENS = (
    "wheel",
    "waypoint_follower",
    "dwa_follower",
    "mpc_follower",
    "safety_gate",
    "tip_guard",
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


def motion_surface_violations(publishers, subscribers):
    """Return connected motion topics/nodes that make shadow QA unsafe."""
    violations = []
    for role, graph in (
        ("publisher", publishers),
        ("subscriber", subscribers),
    ):
        for topic, nodes in graph.items():
            if topic in COMMAND_TOPICS and nodes:
                violations.append(
                    "%s %s: %s" % (role, topic, ", ".join(nodes))
                )
            for node in nodes:
                lowered = node.lower()
                if any(token in lowered for token in MOTION_NODE_TOKENS):
                    violations.append("%s node: %s" % (role, node))
    return sorted(set(violations))


def validate_snapshot(summary, dynamic_boxes, diagnostics,
                      diagnostics_stamp=None, boxes_source_stamp=None,
                      max_skew_s=2.0):
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
    summary_stamp = float(summary.get("stamp"))
    if not math.isfinite(summary_stamp):
        raise ValueError("objects_summary stamp is not finite")
    if boxes_source_stamp is not None:
        boxes_source_stamp = float(boxes_source_stamp)
        if not math.isfinite(boxes_source_stamp) or abs(
                boxes_source_stamp - summary_stamp) > 0.05:
            raise ValueError("summary and boxes are not from one source cycle")
    summary_ids = sorted(int(item["id"]) for item in objects)
    box_ids = sorted(int(box["id"]) for box in dynamic_boxes)
    if summary_ids != box_ids:
        raise ValueError("object/box IDs do not match")
    for box in dynamic_boxes:
        position = box.get("position")
        if not (
            isinstance(position, list)
            and len(position) == 3
            and all(
                isinstance(value, (int, float))
                and math.isfinite(value)
                for value in position
            )
        ):
            raise ValueError("dynamic box positions must be finite triples")
        stamp = float(box.get("stamp"))
        if not math.isfinite(stamp) or abs(stamp - summary_stamp) > max_skew_s:
            raise ValueError("summary/box snapshot stamps do not agree")
    missing = [key for key in DIAGNOSTIC_KEYS if key not in diagnostics]
    if missing:
        raise ValueError("missing diagnostics: %s" % ", ".join(missing))
    if diagnostics["tracking_state"] != "TRACKING":
        raise ValueError(
            "localization is not TRACKING: %s"
            % diagnostics["tracking_state"]
        )
    if diagnostics_stamp is not None:
        diagnostics_stamp = float(diagnostics_stamp)
        if not math.isfinite(diagnostics_stamp) or \
                abs(diagnostics_stamp - summary_stamp) > max_skew_s:
            raise ValueError(
                "perception/localization snapshot stamps do not agree"
            )
    for key in DIAGNOSTIC_KEYS[1:]:
        value = float(diagnostics[key])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("%s must be finite and non-negative" % key)
    raw = int(diagnostics["raw_scan_points"])
    box_dropped = int(diagnostics["dynamic_returns_dropped"])
    post_box = int(diagnostics["post_box_points"])
    map_filtered = int(diagnostics["map_filtered"])
    post_map = int(diagnostics["post_map_points"])
    rolling = int(diagnostics["rolling_submap_points"])
    if post_box > raw or post_map > post_box:
        raise ValueError("registration filter stage counts are not monotonic")
    if raw - box_dropped != post_box:
        raise ValueError("box-filter accounting is inconsistent")
    if post_box - map_filtered != post_map:
        raise ValueError("map-filter accounting is inconsistent")
    if raw <= 0 or rolling <= 0:
        raise ValueError("localization snapshot is empty")
    return {
        "object_count": len(objects),
        "dynamic_box_count": len(dynamic_boxes),
        "tracking_state": diagnostics["tracking_state"],
        "raw_scan_points": raw,
        "post_box_points": post_box,
        "post_map_points": post_map,
    }
