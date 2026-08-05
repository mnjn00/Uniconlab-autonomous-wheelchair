"""Control-law tests for the optional GPU coarse-search path."""

import sys
import types
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    import initial_pose_global_search as search
finally:
    sys.path.pop(0)


def _scene():
    points = np.array([
        [-2.0, -2.0, 0.0], [2.0, -2.0, 0.0],
        [-2.0, 2.0, 1.0], [2.0, 2.0, 1.0],
    ], np.float32)
    return points, points.copy(), ((0.0, 0.0, 0.0, 0.0),)


def test_gpu_path_expands_laterally_and_scores_every_yaw(monkeypatch):
    seen = {}

    class FakeScorer(object):
        def __init__(self, *args, **kwargs):
            pass

        def score_poses(self, sample, poses):
            seen["poses"] = tuple(poses)
            return np.linspace(0.0, 1.0, len(poses))

    monkeypatch.setitem(
        sys.modules, "gpu_voxel_scorer",
        types.SimpleNamespace(GpuVoxelScorer=FakeScorer))
    points, sample, candidates = _scene()
    got = search.score_global_candidates(
        sample, points, candidates, 0.45,
        gpu_lateral_radius_m=2.0, gpu_lateral_step_m=1.0)

    expected = 5 * len(search.coarse_yaw_offsets())
    assert len(seen["poses"]) == expected
    assert len(got) == expected
    assert {round(pose[1], 6) for pose in seen["poses"]} == {
        -2.0, -1.0, 0.0, 1.0, 2.0}


def test_required_gpu_fails_closed_during_setup(monkeypatch):
    class BrokenScorer(object):
        def __init__(self, *args, **kwargs):
            raise RuntimeError("driver unavailable")

    monkeypatch.setitem(
        sys.modules, "gpu_voxel_scorer",
        types.SimpleNamespace(GpuVoxelScorer=BrokenScorer))
    points, sample, candidates = _scene()
    with pytest.raises(RuntimeError, match="required GPU.*driver unavailable"):
        search.score_global_candidates(
            sample, points, candidates, 0.45, require_gpu=True)


def test_optional_gpu_failure_returns_original_cpu_search(monkeypatch):
    class BrokenScorer(object):
        def __init__(self, *args, **kwargs):
            pass

        def score_poses(self, sample, poses):
            raise RuntimeError("out of memory")

    monkeypatch.setitem(
        sys.modules, "gpu_voxel_scorer",
        types.SimpleNamespace(GpuVoxelScorer=BrokenScorer))
    points, sample, candidates = _scene()
    messages = []
    got = search.score_global_candidates(
        sample, points, candidates, 0.45, log=messages.append)

    assert len(got) == len(search.coarse_yaw_offsets())
    assert any("CPU limited search" in message for message in messages)

