import json
import sys
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from hybrid_perception import fuse_summaries  # noqa: E402


def geometric(objects, stamp=100.0, status="OK", frame="lidar"):
    return {
        "stamp": stamp,
        "status": status,
        "frame": frame,
        "objects": objects,
    }


def learned(objects, stamp=100.0, status="OK", frame="lidar"):
    return {
        "stamp": stamp,
        "status": status,
        "frame": frame,
        "model_id": "test-model",
        "objects": objects,
    }


def box(label="obstacle", x=2.0, y=0.0, size=(0.8, 0.8, 1.2), **extra):
    item = {
        "class": label,
        "x": x,
        "y": y,
        "z": 0.5,
        "size": list(size),
        "id": 7,
        "motion": "static",
        "profile": {"bin_m": 0.2, "y0": -0.4,
                    "min_x": [None, x - 0.3, x - 0.2, None]},
    }
    item.update(extra)
    return item


def detection(label="person", score=0.9, x=2.05, y=0.02,
              size=(0.5, 0.5, 1.7), **extra):
    item = {
        "class": label,
        "score": score,
        "x": x,
        "y": y,
        "z": 0.7,
        "size": list(size),
    }
    item.update(extra)
    return item


def test_geometry_survives_when_learning_is_absent():
    result = fuse_summaries(
        geometric([box()]), None, now_s=100.1,
        translation=(0.5, 0.2, 0.0),
    )

    assert result["status"] == "OK"
    assert result["mode"] == "geometric_only"
    assert len(result["objects"]) == 1
    assert result["objects"][0]["x"] == 2.5
    assert result["objects"][0]["y"] == 0.2


def test_learning_can_relabel_but_not_erase_geometry():
    result = fuse_summaries(
        geometric([box(label="obstacle")]),
        learned([detection(label="person", score=0.91)]),
        now_s=100.1,
    )

    assert result["mode"] == "hybrid"
    assert len(result["objects"]) == 1
    fused = result["objects"][0]
    assert fused["class"] == "person"
    assert fused["source"] == "geometric+learned"
    assert fused["size"][0] >= 0.70
    assert fused["size"][1] >= 0.70


def test_learned_obstacle_cannot_downgrade_a_geometric_person():
    result = fuse_summaries(
        geometric([box(label="person")]),
        learned([detection(label="vehicle", score=0.99)]),
        now_s=100.1,
    )

    assert result["objects"][0]["class"] == "person"


def test_high_confidence_learned_only_person_is_added_as_unknown_motion():
    result = fuse_summaries(
        geometric([]),
        learned([detection(label="pedestrian", score=0.8, x=4.0)]),
        now_s=100.1,
    )

    assert len(result["objects"]) == 1
    item = result["objects"][0]
    assert item["class"] == "person"
    assert item["motion"] == "unknown"
    assert item["source"] == "learned_only"


def test_low_confidence_non_person_learned_only_box_is_not_control_geometry():
    result = fuse_summaries(
        geometric([]),
        learned([detection(label="vehicle", score=0.40, x=4.0)]),
        now_s=100.1,
    )

    assert result["objects"] == []


def test_malformed_learning_never_clears_geometry():
    result = fuse_summaries(
        geometric([box()]), "not json", now_s=100.1,
    )

    assert result["status"] == "OK"
    assert result["mode"] == "geometric_only"
    assert len(result["objects"]) == 1


def test_required_learning_fails_closed_when_stale():
    result = fuse_summaries(
        geometric([box()], stamp=100.0),
        learned([], stamp=90.0),
        now_s=100.1,
        require_learned=True,
    )

    assert result["status"].startswith("LEARNED_")
    assert result["mode"] == "blocked"
    assert len(result["objects"]) == 1


def test_unusable_geometry_never_becomes_clear_from_learning():
    result = fuse_summaries(
        geometric([], status="NO_MAP_POSE"),
        learned([detection()]),
        now_s=100.1,
    )

    assert result["status"] == "GEOMETRY_NO_MAP_POSE"
    assert result["mode"] == "blocked"


def test_rotation_and_translation_apply_to_boxes_and_profiles():
    rotation = np.array([[0.0, -1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0]])
    result = fuse_summaries(
        geometric([box(x=2.0, y=0.0)]), None, now_s=100.1,
        rotation=rotation,
        translation=(1.0, 2.0, 0.0),
    )

    item = result["objects"][0]
    assert item["x"] == 1.0
    assert item["y"] == 4.0
    assert isinstance(item["profile"]["min_x"], list)


def test_json_strings_are_accepted_for_ros_std_msgs_payloads():
    result = fuse_summaries(
        json.dumps(geometric([box()])),
        json.dumps(learned([])),
        now_s=100.1,
    )

    assert result["status"] == "OK"


def test_builtin_lidar_geometry_is_moved_to_the_real_chair_centre():
    offset = np.asarray((-0.011, -0.02329, 0.04412))
    chair = np.asarray((-0.500, -0.200, 0.0))
    rotation = np.eye(3)
    translation = offset - chair
    result = fuse_summaries(
        geometric([box(x=1.0, y=0.0)]), None, now_s=100.1,
        rotation=rotation, translation=translation)

    item = result["objects"][0]
    # The lidar is about 0.49 m in front of and 0.18 m left of the axle
    # centre. An object 1 m ahead of the lidar is therefore ~1.49 m ahead of
    # the chair centre, not 1 m ahead of it.
    assert item["x"] > 1.45
    assert item["y"] > 0.15
