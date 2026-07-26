import importlib.util
from pathlib import Path

import numpy as np


COVERAGE_PATH = Path(__file__).parents[1] / "scripts" / "forward_coverage.py"
SPEC = importlib.util.spec_from_file_location("forward_coverage", COVERAGE_PATH)
COVERAGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COVERAGE)


def covered_corridor(y_center=0.0):
    points = []
    for x in (0.5, 1.5, 2.5):
        for y in (y_center - 0.3, y_center, y_center + 0.3):
            points.extend([(x, y, -0.3), (x + 0.03, y + 0.02, -0.28)])
    return np.asarray(points, dtype=np.float32)


def has_coverage(points, y_center=0.0):
    return COVERAGE.corridor_has_coverage(
        points, 0.25, 3.0, y_center, 0.5, -0.45, 1.6)


def test_distributed_forward_observations_are_covered():
    assert has_coverage(covered_corridor())


def test_rear_or_side_only_cloud_is_not_forward_coverage():
    rear = np.asarray([(-1.0, 0.0, 0.5)] * 100, dtype=np.float32)
    side = np.asarray([(1.0, 2.0, 0.5)] * 100, dtype=np.float32)
    assert not has_coverage(rear)
    assert not has_coverage(side)


def test_ceiling_only_cloud_is_not_forward_coverage():
    ceiling = covered_corridor()
    ceiling[:, 2] = 2.5
    assert not has_coverage(ceiling)


def test_dense_single_cluster_does_not_claim_coverage():
    cluster = np.asarray(
        [(0.5 + i * 0.001, 0.0, -0.3) for i in range(100)],
        dtype=np.float32)
    assert not has_coverage(cluster)


def test_partial_lateral_view_does_not_claim_coverage():
    one_side = np.asarray(
        [(x, -0.35, -0.3) for x in (0.5, 1.5, 2.5) for _ in range(20)],
        dtype=np.float32)
    assert not has_coverage(one_side)


def test_shifted_bypass_corridor_uses_its_own_coverage():
    shifted = covered_corridor(y_center=0.6)
    assert has_coverage(shifted, y_center=0.6)
    assert not has_coverage(shifted, y_center=-0.6)


def test_nonfinite_points_do_not_count_as_observations():
    invalid = np.asarray([(1.0, 0.0, np.nan)] * 100, dtype=np.float32)
    assert not has_coverage(invalid)
