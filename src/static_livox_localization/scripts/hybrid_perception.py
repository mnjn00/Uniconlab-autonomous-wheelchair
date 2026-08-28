"""Fail-closed fusion of geometric and learned 3-D object summaries.

Geometry remains collision authority: learning may relabel a geometric box or
add a high-confidence box, but can never delete geometric evidence.  Inputs use
the JSON schema published by obstacle_clusters.py; output is chair-centred.
"""
from __future__ import annotations

import copy
import json
import math
from collections import Counter

import numpy as np

OK = "OK"
ALIASES = {
    "person": "person", "pedestrian": "person", "adult": "person",
    "child": "person", "cyclist": "two_wheeler", "bicycle": "two_wheeler",
    "bike": "two_wheeler", "motorcycle": "two_wheeler",
    "motorbike": "two_wheeler", "scooter": "two_wheeler",
    "car": "vehicle", "truck": "vehicle", "bus": "vehicle",
    "van": "vehicle", "vehicle": "vehicle", "two_wheeler": "two_wheeler",
    "outside_band": "outside_band", "obstacle": "obstacle",
}


def _payload(value):
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        parsed = json.loads(value)
        if isinstance(parsed, dict):
            return parsed
    raise ValueError("summary is not a JSON object")


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) \
        and math.isfinite(float(value))


def _label(value):
    text = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    return ALIASES.get(text, "obstacle")


def _box(item):
    try:
        size = item["size"]
        values = (float(item["x"]), float(item["y"]),
                  float(item.get("z", 0.0)), abs(float(size[0])),
                  abs(float(size[1])), abs(float(size[2]) if len(size) > 2 else 0.1))
    except (KeyError, IndexError, TypeError, ValueError):
        return None
    return values if all(math.isfinite(v) for v in values) \
        and 0.0 < min(values[3:]) and max(values[3:]) <= 20.0 else None


def _problem(value, now_s, expected_frame, max_age_s):
    try:
        data = _payload(value)
    except (TypeError, ValueError, json.JSONDecodeError):
        return "MALFORMED"
    if str(data.get("status", "")) != OK:
        return str(data.get("status") or "NOT_OK")
    if str(data.get("frame", "")) != expected_frame:
        return "FRAME_MISMATCH"
    stamp = data.get("stamp")
    if not _finite(stamp) or not _finite(now_s):
        return "STAMP_INVALID"
    age = float(now_s) - float(stamp)
    if age < -0.05:
        return "STAMP_FUTURE"
    if age > float(max_age_s):
        return "STALE"
    return "" if isinstance(data.get("objects"), list) else "OBJECTS_INVALID"


def _transform_profile(profile, rotation, translation):
    """Rebuild lateral slices after a rigid transform (conservative samples)."""
    if not isinstance(profile, dict):
        return profile
    try:
        step, y0, values = float(profile["bin_m"]), float(profile["y0"]), profile["min_x"]
        samples = [(float(x), y0 + (i + 0.5) * step, 0.0)
                   for i, x in enumerate(values) if x is not None and _finite(x)]
    except (KeyError, TypeError, ValueError):
        return profile
    if not samples or step <= 0.0:
        return profile
    points = np.asarray(samples) @ rotation.T + translation
    first = int(math.floor(float(points[:, 1].min()) / step))
    last = int(math.floor(float(points[:, 1].max()) / step))
    nearest = [None] * (last - first + 1)
    for x, y, _z in points:
        index = max(0, min(len(nearest) - 1, int(math.floor(y / step)) - first))
        nearest[index] = round(float(x), 2) if nearest[index] is None \
            else min(nearest[index], round(float(x), 2))
    return {"bin_m": round(step, 3), "y0": round(first * step, 3), "min_x": nearest}


def _transform(item, rotation, translation):
    box = _box(item)
    if box is None:
        return None
    x, y, z, sx, sy, sz = box
    centre = rotation @ np.asarray((x, y, z)) + translation
    size = np.abs(rotation) @ np.asarray((sx, sy, sz))
    result = copy.deepcopy(dict(item))
    result.update(x=round(float(centre[0]), 3), y=round(float(centre[1]), 3),
                  z=round(float(centre[2]), 3),
                  size=[round(float(v), 3) for v in size])
    if "profile" in result:
        result["profile"] = _transform_profile(result["profile"], rotation, translation)
    return result


def _score(item):
    try:
        value = float(item.get("score", item.get("confidence", 0.0)))
        return value if math.isfinite(value) else 0.0
    except (TypeError, ValueError):
        return 0.0


def _distance(first, second):
    a, b = _box(first), _box(second)
    return math.inf if a is None or b is None else math.hypot(a[0] - b[0], a[1] - b[1])


def _iou(first, second):
    a, b = _box(first), _box(second)
    if a is None or b is None:
        return 0.0
    ax0, ax1, ay0, ay1 = a[0]-a[3]/2, a[0]+a[3]/2, a[1]-a[4]/2, a[1]+a[4]/2
    bx0, bx1, by0, by1 = b[0]-b[3]/2, b[0]+b[3]/2, b[1]-b[4]/2, b[1]+b[4]/2
    intersection = max(0.0, min(ax1, bx1)-max(ax0, bx0)) * max(0.0, min(ay1, by1)-max(ay0, by0))
    union = a[3]*a[4] + b[3]*b[4] - intersection
    return intersection / union if union > 1e-9 else 0.0


def _matches(geometry, learned, gate_m):
    candidates = []
    for gi, geom in enumerate(geometry):
        for li, detection in enumerate(learned):
            distance, overlap = _distance(geom, detection), _iou(geom, detection)
            a, b = _box(geom), _box(detection)
            gate = gate_m if a is None or b is None else max(
                gate_m, min(2.0, 0.25*(math.hypot(a[3], a[4])+math.hypot(b[3], b[4]))))
            if distance <= gate or overlap > 0.01:
                candidates.append((-(2.0*overlap-distance+0.25*_score(detection)), gi, li))
    result, used_g, used_l = {}, set(), set()
    for _quality, gi, li in sorted(candidates):
        if gi not in used_g and li not in used_l:
            result[gi] = li; used_g.add(gi); used_l.add(li)
    return result


def _person_geometry(item, minimum):
    if _label(item.get("class")) == "person" and _box(item) is not None:
        size = list(item["size"])
        size[0], size[1] = max(float(size[0]), minimum), max(float(size[1]), minimum)
        item["size"] = size


def fuse_summaries(geometric_summary, learned_summary, now_s, rotation=None,
                   translation=None, geometric_frame="lidar", learned_frame="lidar",
                   output_frame="chair_centre", geometric_max_age_s=1.5,
                   learned_max_age_s=1.0, maximum_skew_s=0.40,
                   association_gate_m=0.85, person_score_threshold=0.35,
                   class_score_threshold=0.50, learned_only_score_threshold=0.65,
                   person_min_extent_m=0.70, require_learned=False):
    rotation = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    translation = np.zeros(3) if translation is None else np.asarray(translation, dtype=float)
    if rotation.shape != (3, 3) or translation.shape != (3,) or \
            not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError("output transform must be finite 3x3 + xyz")
    try:
        geometric = _payload(geometric_summary)
    except Exception:
        geometric = {}
    geometry_problem = _problem(geometric, now_s, geometric_frame, geometric_max_age_s)
    geometry = [_transform(item, rotation, translation)
                for item in geometric.get("objects", []) if isinstance(item, dict)]
    geometry = [item for item in geometry if item is not None]
    base = {"stamp": geometric.get("stamp", float(now_s)),
            "status": OK if not geometry_problem else "GEOMETRY_"+geometry_problem,
            "frame": output_frame, "mode": "blocked" if geometry_problem else "geometric_only",
            "objects": geometry, "counts": {},
            "sources": {"geometric": geometry_problem or OK, "learned": "UNAVAILABLE"}}
    if geometry_problem:
        base["counts"] = dict(Counter(_label(o.get("class")) for o in geometry)); return base

    try:
        learned = _payload(learned_summary) if learned_summary is not None else None
    except Exception:
        learned = None
    learned_problem = "UNAVAILABLE" if learned is None else _problem(
        learned, now_s, learned_frame, learned_max_age_s)
    if not learned_problem and abs(float(learned["stamp"])-float(geometric["stamp"])) > maximum_skew_s:
        learned_problem = "STAMP_SKEW"
    if learned_problem:
        base["sources"]["learned"] = learned_problem
        if require_learned:
            base["status"], base["mode"] = "LEARNED_"+learned_problem, "blocked"
        for item in geometry:
            item.setdefault("source", "geometric"); item.setdefault("semantic_confidence", 0.0)
        base["counts"] = dict(Counter(_label(o.get("class")) for o in geometry)); return base

    detections = [_transform(item, rotation, translation)
                  for item in learned.get("objects", []) if isinstance(item, dict)]
    detections = [item for item in detections if item is not None]
    matches, used = _matches(geometry, detections, association_gate_m), set()
    fused = []
    for gi, original in enumerate(geometry):
        item, original_label = copy.deepcopy(original), _label(original.get("class"))
        item["class"] = original_label; item.setdefault("source", "geometric")
        item.setdefault("semantic_confidence", 0.0)
        if gi in matches:
            detection = detections[matches[gi]]; used.add(matches[gi])
            confidence, learned_label = _score(detection), _label(detection.get("class"))
            if original_label == "person" or (learned_label == "person" and confidence >= person_score_threshold):
                item["class"] = "person"
            elif confidence >= class_score_threshold:
                item["class"] = learned_label
            item.update(source="geometric+learned", learned_class=learned_label,
                        semantic_confidence=round(confidence, 4),
                        association_iou=round(_iou(original, detection), 4),
                        association_distance_m=round(_distance(original, detection), 3))
        _person_geometry(item, person_min_extent_m); fused.append(item)
    next_id = -100000
    for li, detection in enumerate(detections):
        if li in used:
            continue
        confidence, label = _score(detection), _label(detection.get("class"))
        threshold = person_score_threshold if label == "person" else learned_only_score_threshold
        if confidence < threshold or any(_distance(detection, item) < 0.35 for item in fused):
            continue
        item = copy.deepcopy(detection); item.update(
            **{"class": label, "id": next_id, "motion": item.get("motion", "unknown"),
               "source": "learned_only", "semantic_confidence": round(confidence, 4)})
        next_id -= 1; _person_geometry(item, person_min_extent_m); fused.append(item)
    base.update(status=OK, mode="hybrid", objects=fused, model_id=str(learned.get("model_id", "unknown")))
    base["sources"]["learned"] = OK
    base["counts"] = dict(Counter(_label(o.get("class")) for o in fused))
    base["fusion"] = {"geometric_count": len(geometry), "learned_count": len(detections),
                      "matched_count": len(matches), "fused_count": len(fused)}
    return base
