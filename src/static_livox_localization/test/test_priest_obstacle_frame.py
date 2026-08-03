"""Producer evidence for map-stable PRIEST obstacle coordinates."""

from __future__ import annotations

import json
import math
import sys
import threading
import types
from pathlib import Path

import numpy as np
import pytest


TESTS = Path(__file__).parent
sys.path.insert(0, str(TESTS))
try:
    import test_cluster_pipeline as pipeline
finally:
    sys.path.remove(str(TESTS))


def test_accumulator_window_matches_published_contract() -> None:
    module, node = pipeline.producer_at([100.0])
    accumulator_window = node.accumulator.merged.__func__.__globals__["WINDOW_S"]
    assert accumulator_window == module.WINDOW_S


def test_summary_uses_cloud_stamp_and_publishes_map_centres() -> None:
    now_s = [100.8]
    module, node = pipeline.producer_at(now_s)
    cloud_stamp = 100.0
    node.accumulator.scans = [(
        cloud_stamp, pipeline.box_of_points(2.0, 0.0))]
    node.accumulator.odoms = [(cloud_stamp, np.eye(4))]
    node.map_poses.add(cloud_stamp, np.eye(4))

    node.step()

    payload = json.loads(node.summary_pub.last.data)
    assert payload["stamp"] == cloud_stamp
    assert payload["published_at"] == now_s[0]
    assert payload["objects"]
    assert all("map_x" in item and "map_y" in item
               for item in payload["objects"])


def pose_at(x_m: float, yaw_rad: float) -> np.ndarray:
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    value = np.eye(4)
    value[:2, :2] = [[cosine, -sine], [sine, cosine]]
    value[0, 3] = x_m
    return value


def test_source_pose_is_interpolated_between_bracketing_samples() -> None:
    module = pipeline.load_producer([100.0])
    poses = module.MapPoseBuffer()
    poses.add(99.9, pose_at(0.0, math.radians(179.0)))
    poses.add(100.1, pose_at(0.2, math.radians(-179.0)))

    interpolated = poses.nearest(100.0)

    assert interpolated is not None
    assert interpolated[0, 3] == 0.1
    assert abs(abs(math.atan2(
        interpolated[1, 0], interpolated[0, 0])) - math.pi) < 1e-3


def test_unbracketed_pose_inside_old_tolerance_is_refused() -> None:
    module = pipeline.load_producer([100.0])
    poses = module.MapPoseBuffer()
    poses.add(99.71, np.eye(4))

    assert poses.nearest(100.0) is None


def _motion_pose(stamp_s: float) -> np.ndarray:
    offset_s = stamp_s - 100.0
    return pose_at(0.6 * offset_s, 0.5 * offset_s)


def _world_to_body(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    value = np.asarray(points, dtype=np.float64)
    return np.dot(value - pose[:3, 3], pose[:3, :3])


def test_asymmetric_odom_brackets_preserve_accumulated_map_centre() -> None:
    module, node = pipeline.producer_at([100.1])
    world = pipeline.box_of_points(10.0, 0.0)
    node.accumulator.scans = [
        (stamp, _world_to_body(world, _motion_pose(stamp)))
        for stamp in (100.0, 100.1)]
    node.accumulator.odoms = [
        (stamp, _motion_pose(stamp))
        for stamp in (99.851, 100.049, 100.249)]

    merged = node.accumulator.merged()

    assert merged is not None
    mapped = node._map_centre(merged.mean(axis=0), _motion_pose(100.1))
    np.testing.assert_allclose(mapped, world.mean(axis=0), atol=2e-3)


def test_unbracketed_odom_inside_old_tolerance_is_refused() -> None:
    module, node = pipeline.producer_at([100.0])
    node.accumulator.scans = [(100.0, pipeline.box_of_points(2.0, 0.0))]
    node.accumulator.odoms = [(99.90, np.eye(4))]

    assert node.accumulator.merged() is None


def _odom_message(
        stamp_s: float, frame_id: str,
        child_frame_id: str) -> types.SimpleNamespace:
    position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    quaternion = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=pipeline.Stamp(stamp_s), frame_id=frame_id),
        child_frame_id=child_frame_id,
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=position, orientation=quaternion)))


def _map_pose_message(
        stamp_s: float, frame_id: str) -> types.SimpleNamespace:
    message = _odom_message(stamp_s, frame_id, "body")
    return types.SimpleNamespace(header=message.header, pose=message.pose)


def _valid_source_state(
        node: types.SimpleNamespace, stamp_s: float) -> None:
    node.accumulator.scans = [(
        stamp_s, pipeline.box_of_points(2.0, 0.0))]
    node.accumulator.odoms = [(stamp_s, np.eye(4))]
    node.map_poses.add(stamp_s, np.eye(4))


def test_wrong_odom_frame_makes_summary_unusable(
        monkeypatch: pytest.MonkeyPatch) -> None:
    module, node = pipeline.producer_at([100.0])
    _valid_source_state(node, 100.0)
    transformations = node.add_map_pose.__func__.__globals__["tft"]
    tf_module = types.ModuleType("tf")
    tf_module.transformations = transformations
    monkeypatch.setitem(sys.modules, "tf", tf_module)
    monkeypatch.setitem(sys.modules, "tf.transformations", transformations)

    node.accumulator.add_odom(
        _odom_message(100.1, "wrong_odom", "body"))
    node.step()

    payload = json.loads(node.summary_pub.last.data)
    assert payload["status"] == "NO_CLOUD"


def test_wrong_cloud_frame_makes_summary_unusable(
        monkeypatch: pytest.MonkeyPatch) -> None:
    module, node = pipeline.producer_at([100.0])
    _valid_source_state(node, 100.0)
    cloud_api = node.accumulator.add_cloud.__func__.__globals__["pc2"]
    monkeypatch.setattr(
        cloud_api, "read_points",
        lambda *args, **kwargs: [(2.0, 0.0, 0.5)])
    cloud = types.SimpleNamespace(header=types.SimpleNamespace(
        stamp=pipeline.Stamp(100.1), frame_id="wrong_body"))

    node.accumulator.add_cloud(cloud)
    node.step()

    payload = json.loads(node.summary_pub.last.data)
    assert payload["status"] == "NO_CLOUD"


def test_wrong_map_frame_makes_summary_unusable(
        monkeypatch: pytest.MonkeyPatch) -> None:
    module, node = pipeline.producer_at([100.0])
    _valid_source_state(node, 100.0)
    monkeypatch.setattr(
        module.rospy, "logwarn_throttle", lambda *args: None, raising=False)

    node.add_map_pose(_map_pose_message(100.1, "wrong_map"))
    node.step()

    payload = json.loads(node.summary_pub.last.data)
    assert payload["status"] == "NO_MAP_POSE"


def test_invalid_odom_during_merge_cannot_restore_stale_authority(
        monkeypatch: pytest.MonkeyPatch) -> None:
    module, node = pipeline.producer_at([100.0])
    _valid_source_state(node, 100.0)
    namespace = node.accumulator.merged.__func__.__globals__
    interpolate = namespace["interpolate_rigid_pose"]
    invalidated = [False]

    def invalidate_after_snapshot(poses, stamp_s, max_span_s=0.20):
        result = interpolate(poses, stamp_s, max_span_s)
        if not invalidated[0]:
            invalidated[0] = True
            node.accumulator.add_odom(
                _odom_message(100.1, "wrong_odom", "body"))
        return result

    monkeypatch.setitem(
        namespace, "interpolate_rigid_pose", invalidate_after_snapshot)

    assert node.accumulator.merged() is None
    assert node.accumulator.reference is None


def test_replayed_map_pose_revokes_prior_map_authority(
        monkeypatch: pytest.MonkeyPatch) -> None:
    module, node = pipeline.producer_at([100.0])
    _valid_source_state(node, 100.0)
    monkeypatch.setattr(
        module.rospy, "logwarn_throttle", lambda *args: None, raising=False)

    node.add_map_pose(_map_pose_message(99.9, "map"))
    node.step()

    payload = json.loads(node.summary_pub.last.data)
    assert payload["status"] == "NO_MAP_POSE"


def _race_callbacks(valid_call, invalid_call, entered, release) -> bool:
    invalid_finished = threading.Event()
    valid_thread = threading.Thread(target=valid_call)
    valid_thread.start()
    assert entered.wait(1.0)

    def invalidate() -> None:
        invalid_call()
        invalid_finished.set()

    invalid_thread = threading.Thread(target=invalidate)
    invalid_thread.start()
    finished_before_release = invalid_finished.wait(0.1)
    release.set()
    valid_thread.join(1.0)
    invalid_thread.join(1.0)
    assert not valid_thread.is_alive() and not invalid_thread.is_alive()
    return finished_before_release


def test_newer_invalid_odom_wins_over_inflight_valid_commit(
        monkeypatch: pytest.MonkeyPatch) -> None:
    module, node = pipeline.producer_at([100.0])
    transformations = node.accumulator.add_odom.__func__.__globals__["tft"]
    entered, release = threading.Event(), threading.Event()

    def pause_matrix(values):
        entered.set()
        release.wait(1.0)
        return np.eye(4)

    monkeypatch.setattr(transformations, "quaternion_matrix", pause_matrix)
    raced = _race_callbacks(
        lambda: node.accumulator.add_odom(
            _odom_message(100.0, "camera_init", "body")),
        lambda: node.accumulator.add_odom(
            _odom_message(100.1, "wrong_odom", "body")), entered, release)

    assert not raced
    assert not node.accumulator.odom_valid and not node.accumulator.odoms


def test_newer_invalid_map_pose_wins_over_inflight_valid_commit(
        monkeypatch: pytest.MonkeyPatch) -> None:
    module = pipeline.load_producer([100.0])
    poses = module.MapPoseBuffer()
    namespace = poses.add.__func__.__globals__
    entered, release = threading.Event(), threading.Event()

    def pause_rigid_pose(value):
        entered.set()
        release.wait(1.0)
        return True

    monkeypatch.setitem(namespace, "_rigid_pose", pause_rigid_pose)
    raced = _race_callbacks(
        lambda: poses.add(100.0, np.eye(4)),
        lambda: poses.add(0.0, np.eye(4)), entered, release)

    assert not raced
    assert not poses.poses


@pytest.mark.parametrize("authority", ["odom", "map"])
def test_summary_publish_is_atomic_with_authority_callbacks(
        monkeypatch: pytest.MonkeyPatch, authority: str) -> None:
    module, node = pipeline.producer_at([100.0])
    _valid_source_state(node, 100.0)
    monkeypatch.setattr(
        module.rospy, "logwarn_throttle", lambda *args: None, raising=False)
    original_boxes = node._boxes
    entered, release = threading.Event(), threading.Event()

    def pause_boxes(clusters, map_pose):
        entered.set()
        release.wait(1.0)
        return original_boxes(clusters, map_pose)

    monkeypatch.setattr(node, "_boxes", pause_boxes)
    invalid_call = (
        lambda: node.accumulator.add_odom(
            _odom_message(100.1, "wrong_odom", "body"))) \
        if authority == "odom" else \
        lambda: node.add_map_pose(_map_pose_message(100.1, "wrong_map"))

    raced = _race_callbacks(node.step, invalid_call, entered, release)

    assert not raced
    assert json.loads(node.summary_pub.last.data)["status"] == "OK"
