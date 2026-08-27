import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

PACKAGE = Path(__file__).parents[1]
ROOT = PACKAGE.parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))

from hybrid_perception import fuse_summaries  # noqa: E402
from localization_exclusion_policy import should_exclude  # noqa: E402
from route_mask import RouteMask  # noqa: E402
from semantic_safety_policy import (PersonStopLatch, ThreatView,
                                    decide_semantic_stop, stopping_distance)  # noqa: E402
from terrain_guard_policy import (evaluate_terrain_command, rollout_unicycle,
                                  stopping_horizon)  # noqa: E402


def summary(objects, stamp=100.0, status="OK", frame="lidar", model=False):
    data = {"stamp": stamp, "status": status, "frame": frame, "objects": objects}
    if model:
        data["model_id"] = "test-model"
    return data


def geom(label="obstacle", x=2.0, y=0.0):
    return {"class": label, "x": x, "y": y, "z": 0.5,
            "size": [0.8, 0.8, 1.2], "id": 7, "motion": "static",
            "profile": {"bin_m": 0.2, "y0": -0.4,
                        "min_x": [None, x - 0.3, x - 0.2, None]}}


def learned(label="person", score=0.9, x=2.05, y=0.02):
    return {"class": label, "score": score, "x": x, "y": y, "z": 0.7,
            "size": [0.5, 0.5, 1.7]}


def test_geometry_is_authoritative_and_shifted_to_chair_centre():
    result = fuse_summaries(summary([geom()]), None, 100.1,
                            translation=(0.5, 0.2, 0.0))
    assert result["status"] == "OK" and result["mode"] == "geometric_only"
    assert (result["objects"][0]["x"], result["objects"][0]["y"]) == (2.5, 0.2)


def test_learning_relabels_without_erasing_geometry_or_downgrading_people():
    result = fuse_summaries(summary([geom()]),
                            summary([learned()], model=True), 100.1)
    assert len(result["objects"]) == 1
    assert result["objects"][0]["class"] == "person"
    assert min(result["objects"][0]["size"][:2]) >= 0.70
    result = fuse_summaries(summary([geom("person")]),
                            summary([learned("vehicle", 0.99)], model=True), 100.1)
    assert result["objects"][0]["class"] == "person"


def test_learned_only_high_confidence_is_additive_and_unknown_motion():
    result = fuse_summaries(summary([]),
                            summary([learned(x=4.0)], model=True), 100.1)
    assert result["objects"][0]["source"] == "learned_only"
    assert result["objects"][0]["motion"] == "unknown"
    low = fuse_summaries(summary([]),
                         summary([learned("vehicle", 0.4, 4.0)], model=True), 100.1)
    assert low["objects"] == []


def test_bad_learning_keeps_geometry_and_required_learning_fails_closed():
    assert len(fuse_summaries(summary([geom()]), "bad", 100.1)["objects"]) == 1
    required = fuse_summaries(summary([geom()]),
                              summary([], 90.0, model=True), 100.1,
                              require_learned=True)
    assert required["status"].startswith("LEARNED_")
    assert len(required["objects"]) == 1


def test_bad_geometry_never_becomes_clear_from_learning():
    result = fuse_summaries(summary([], status="NO_MAP_POSE"),
                            summary([learned()], model=True), 100.1)
    assert result["status"] == "GEOMETRY_NO_MAP_POSE"


def test_json_payload_and_rotated_profile_are_supported():
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    result = fuse_summaries(json.dumps(summary([geom()])),
                            json.dumps(summary([], model=True)), 100.1,
                            rotation=rotation, translation=(1.0, 2.0, 0.0))
    assert (result["objects"][0]["x"], result["objects"][0]["y"]) == (1.0, 4.0)
    assert isinstance(result["objects"][0]["profile"]["min_x"], list)


def person(distance, track_id=1):
    return ThreatView(distance, "moving", "person", track_id)


def test_person_stop_hysteresis_survives_radius_shrink():
    latch = PersonStopLatch(0.30)
    assert latch.update(person(1.1), 1.2)
    assert latch.update(person(1.1), 0.9)
    assert latch.release_distance_m >= 1.5
    assert not latch.update(person(1.6), 0.9)


def test_unusable_perception_fails_closed_without_clearing_person_latch():
    latch = PersonStopLatch(); latch.update(person(1.0), 1.2)
    decision = decide_semantic_stop(False, 0.0, 0.0, 1.5, 0.6, 1.0,
                                    None, None, latch)
    assert decision.reason == "PERCEPTION_UNUSABLE"
    assert latch.release_distance_m is not None


def test_moving_object_stops_and_static_object_is_left_to_planner():
    moving = ThreatView(0.8, "unknown", "vehicle", 3)
    assert decide_semantic_stop(True, 0.1, 0.1, 1.5, 0.6, 1.0,
                                None, moving, PersonStopLatch()).reason == "MOVING_OBJECT"
    static = ThreatView(0.8, "static", "vehicle", 3)
    assert not decide_semantic_stop(True, 0.1, 0.1, 1.5, 0.6, 1.0,
                                    None, static, PersonStopLatch()).blocked


def test_stopping_distance_grows_and_nonfinite_fails_closed():
    assert stopping_distance(0.1, 0.1, 0.0) < stopping_distance(0.8, 0.8, 0.0)
    assert math.isinf(stopping_distance(math.nan, 0.2, 0.1))


def test_localization_exclusions_do_not_remove_static_mapped_walls():
    assert not should_exclude({
        "class": "vehicle", "motion": "static", "source": "geometric"})
    assert should_exclude({
        "class": "obstacle", "motion": "moving", "source": "geometric"})
    assert should_exclude({
        "class": "person", "motion": "unknown", "source": "geometric"})
    assert not should_exclude({
        "class": "vehicle", "motion": "unknown", "source": "learned_only"})


class Mask:
    def __init__(self, x_limit=10.0, clearance=1.0):
        self.x_limit, self.clearance = x_limit, clearance
    def contains_many(self, points):
        return np.asarray(points)[:, 0] <= self.x_limit
    def segment_is_contained(self, start, end):
        return max(start[0], end[0]) <= self.x_limit
    def clearance_many(self, points):
        return np.full(len(points), self.clearance)


class Band:
    def contains(self, point):
        return abs(point[1]) <= 2.0
    def chord_is_contained(self, start, end):
        return self.contains(start) and self.contains(end)


def test_terrain_rollout_boundary_clearance_and_edge_cap():
    straight = rollout_unicycle((1.0, 2.0, 0.0), 0.5, 0.0, 1.0)
    assert np.allclose(straight[-1, :2], (1.5, 2.0))
    assert evaluate_terrain_command(Mask(0.6), (0, 0, 0), 0.8, 0.0,
                                    0.1, 0.4, 0.35, Band(), 1.0).reason == "MASK_BOUNDARY"
    assert evaluate_terrain_command(Mask(clearance=0.05), (0, 0, 0), 0.2, 0.0,
                                    0.1, 0.4, 0.35, Band(), 1.0).reason == "MASK_CLEARANCE"
    edge = evaluate_terrain_command(Mask(clearance=0.25), (0, 0, 0), 0.8, 0.0,
                                    0.1, 0.4, 0.35, Band(), 1.0)
    assert not edge.blocked and edge.speed_cap_mps == 0.35
    assert stopping_horizon(0.2) < stopping_horizon(0.8)


def test_route_mask_clearance_api(tmp_path):
    image = np.zeros((9, 9), dtype=np.uint8); image[1:8, 1:8] = 254
    Image.fromarray(image).save(str(tmp_path / "mask.pgm"))
    (tmp_path / "mask.yaml").write_text(yaml.safe_dump({
        "image": "mask.pgm", "resolution": 0.1,
        "origin": [0.0, 0.0, 0.0]}), encoding="utf-8")
    mask = RouteMask(str(tmp_path / "mask.yaml"))
    assert mask.clearance_at((-1.0, -1.0)) == 0.0
    assert mask.clearance_at((0.4, 0.4)) > mask.clearance_at((0.1, 0.1))


def test_accumulator_uses_numpy_decoder():
    text = (SCRIPTS / "scan_accumulator.py").read_text(encoding="utf-8")
    assert "from cloud_points import points_xyz" in text
    assert "pts = points_xyz(message, read_points)" in text
    assert "np.array(list(read_points" not in text


def test_runtime_wiring_keeps_old_stack_as_rollback_and_checks_before_go():
    start = (ROOT / "tools" / "start_hybrid_avoidance.sh").read_text(encoding="utf-8")
    assert "PROFILE=dwa" in start and "start_wheelchair_localization.sh" in start
    assert "hybrid_geometric_objects.py" in start
    assert "geometric_exclusion_candidates" in start
    assert "localization_exclusion_boxes.py" in start
    assert "_cmd_topic:=/cmd_vel_planned" in start
    assert "/cmd_vel_gated:=/cmd_vel_terrain_safe" in start
    assert "/perception/objects_summary:=/perception/geometric_objects_summary" in start
    go = (ROOT / "tools" / "go_hybrid.sh").read_text(encoding="utf-8")
    assert go.index("hybrid_preflight.py") < go.index('exec "$GO"')


def test_catkin_installs_hybrid_nodes_and_policies():
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    for name in ("hybrid_geometric_objects.py", "hybrid_object_fusion.py",
                 "vision_detection_bridge.py", "localization_exclusion_boxes.py",
                 "semantic_safety_supervisor.py", "terrain_guard.py",
                 "hybrid_preflight.py", "hybrid_perception.py",
                 "localization_exclusion_policy.py",
                 "semantic_safety_policy.py", "terrain_guard_policy.py"):
        assert name in cmake