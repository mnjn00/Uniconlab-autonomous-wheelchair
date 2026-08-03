"""Adversarial boundaries for PRIEST's pure runtime obstacle policy."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    import priest_runtime as runtime
    from cluster_guard import Summary, parse_summary
finally:
    sys.path.remove(str(SCRIPTS))

TESTS = Path(__file__).parent
sys.path.insert(0, str(TESTS))
try:
    import test_cluster_pipeline as pipeline
finally:
    sys.path.remove(str(TESTS))


def obj(x: float, y: float, motion: str) -> dict:
    return {
        "class": "obstacle", "x": x, "y": y,
        "map_x": x, "map_y": y,
        "size": [0.4, 0.2, 1.2], "points": 40, "motion": motion,
    }


def summary_of(objects: list[dict]) -> Summary:
    return parse_summary(json.dumps(
        {"stamp": 100.0, "status": "OK", "objects": objects}))


def wait(
        objects: list[dict],
        trajectory: np.ndarray,
        current: np.ndarray | None = None,
        transform: np.ndarray | None = None,
        trajectory_start_index: int = 0) -> str | None:
    return runtime.wait_reason(
        summary_of(objects),
        np.eye(4) if transform is None else transform,
        np.zeros(3),
        np.eye(3),
        np.zeros(2) if current is None else current,
        trajectory,
        trajectory_start_index=trajectory_start_index,
    )


@pytest.mark.parametrize("trajectory", [
    np.array([[0.0, 0.0]]),
    np.array([[0.0, 0.0], [0.0, 0.0]]),
])
def test_degenerate_path_uses_conservative_radial_hold(
        trajectory: np.ndarray) -> None:
    assert wait([obj(2.0, 0.0, "moving")], trajectory) \
        == runtime.OBSTACLE_WAIT


def test_explicit_progress_index_preserves_immediate_self_crossing_branch() -> None:
    trajectory = np.array([
        [-1.0, 0.0], [1.0, 0.0], [2.0, 0.0],
        [0.0, 0.0], [0.0, 2.0],
    ])

    reason = wait(
        [obj(1.5, 0.0, "unknown")], trajectory,
        current=np.array([0.05, 0.0]), trajectory_start_index=1)

    assert reason == runtime.OBSTACLE_WAIT


def test_nonrigid_map_transform_cannot_move_conflict_out_of_view() -> None:
    scaled = np.eye(4)
    scaled[0, 0] = 10.0

    assert wait(
        [obj(2.0, 0.0, "moving")],
        np.array([[0.0, 0.0], [3.0, 0.0]]),
        transform=scaled) == runtime.OBSTACLE_WAIT


def test_fractional_planner_limit_cannot_erase_static_obstacles() -> None:
    with pytest.raises(ValueError, match="integral"):
        runtime.planner_obstacles(
            [obj(2.0, 0.0, "static")], np.eye(4), np.zeros(3), np.eye(3),
            limit=0.5)


def test_delayed_summary_keeps_obstacle_at_its_observation_map_pose() -> None:
    item = obj(2.0, 0.0, "static")
    item.update({"map_x": 2.0, "map_y": 0.0})
    current_pose = np.eye(4)
    current_pose[0, 3] = 0.6

    circles, dropped = runtime.planner_obstacles(
        [item], current_pose, np.zeros(3), np.eye(3))

    assert dropped == 0
    assert circles == [[2.0, 0.0, pytest.approx(0.22360679775)]]


def test_unbracketed_producer_pose_cannot_drop_confirmed_static_obstacle() \
        -> None:
    now_s = [100.0]
    module, node = pipeline.producer_at(now_s)
    node.map_poses.add(99.7, np.eye(4))
    for index in range(14):
        stamp_s = 100.0 + index * 0.2
        node.accumulator.scans = [(
            stamp_s, pipeline.box_of_points(2.0, 0.0))]
        node.accumulator.odoms = [(stamp_s, np.eye(4))]
        now_s[0] = stamp_s
        node.step()
    payload = json.loads(node.summary_pub.last.data)
    item = max(payload["objects"], key=lambda value: value["points"])

    assert payload["status"] == "OK" and item["motion"] == "static"
    assert item["map_x"] is None and item["map_y"] is None
    with pytest.raises(ValueError, match="static obstacle"):
        runtime.planner_obstacles(
            payload["objects"], np.eye(4), np.zeros(3), np.eye(3))


def test_static_clearance_uses_the_certified_chair_footprint() -> None:
    trajectory = np.array([[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]])

    assert not runtime.static_obstacles_clear(
        trajectory, [[0.5, 0.70, 0.10]])
    assert runtime.static_obstacles_clear(
        trajectory, [[0.5, 1.00, 0.10]])
    assert not runtime.static_obstacles_clear(trajectory, [[np.nan, 0, 0]])
