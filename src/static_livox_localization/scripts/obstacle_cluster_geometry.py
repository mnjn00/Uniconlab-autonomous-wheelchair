"""Pure clustering, classification, and map-pose geometry."""

from __future__ import annotations

import math
import threading
from typing import Optional, Sequence, Tuple

import numpy as np

from body_frame import lidar_to_body


SENSOR_HEIGHT_M = 0.725
CELL_M = 0.20
MIN_CELL_POINTS = 2
MIN_CLUSTER_POINTS = 8
MAP_POSE_MAX_BRACKET_S = 0.20
STAMP_EPSILON_S = 1e-6
OBJECT_BAND_GRACE_M = 0.10
MAX_BAND_SAMPLE_POINTS = 96
OUTSIDE_MAX_INSIDE_FRACTION = 0.05
INSIDE_MIN_INSIDE_FRACTION = 0.95
OUTSIDE_BAND = "outside_band"
PROFILE_BIN_M = 0.2
MAX_PROFILE_BINS = 64
PERSON_MAX_FOOTPRINT_M = 0.9
PERSON_HEIGHT_M = (1.1, 2.0)
VEHICLE_MIN_FOOTPRINT_M = 1.5
VEHICLE_HEIGHT_M = (0.9, 2.5)


class MapPoseBuffer:
    """Small timestamped history of map_T_body localization poses."""

    def __init__(self):
        self.lock = threading.RLock()
        self.poses = []
        self.generation = 0

    def add(self, stamp_s, matrix):
        with self.lock:
            return self._add(stamp_s, matrix)

    def _add(self, stamp_s, matrix):
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            self.clear()
            return False
        value = np.asarray(matrix, dtype=np.float64)
        if not _rigid_pose(value):
            self.clear()
            return False
        if self.poses and stamp_s <= self.poses[-1][0]:
            self.clear()
            return False
        history = tuple(self.poses)
        self.poses = list((history + ((float(stamp_s), value.copy()),))[-80:])
        self.generation += 1
        return True

    def nearest(self, stamp_s, max_span_s=MAP_POSE_MAX_BRACKET_S):
        """Interpolate a bracketed pose; never extrapolate a stale pose."""
        with self.lock:
            generation = self.generation
            result = interpolate_rigid_pose(
                tuple(self.poses), stamp_s, max_span_s)
            return result if generation == self.generation else None

    def clear(self) -> None:
        with self.lock:
            self.poses = []
            self.generation += 1


def interpolate_rigid_pose(
        poses: Sequence[Tuple[float, np.ndarray]],
        stamp_s: float,
        max_span_s: float = MAP_POSE_MAX_BRACKET_S,
) -> Optional[np.ndarray]:
    """Interpolate one immutable, rigid bracket without extrapolation."""
    samples = tuple(poses)
    if not samples or not math.isfinite(stamp_s) \
            or not math.isfinite(max_span_s) or max_span_s <= 0.0:
        return None
    times = np.array([value for value, _ in samples])
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0.0):
        return None
    right = int(np.searchsorted(times, stamp_s, side="left"))
    for index in (right, right - 1):
        if 0 <= index < len(samples) \
                and abs(times[index] - stamp_s) <= STAMP_EPSILON_S:
            exact = np.asarray(samples[index][1], dtype=np.float64)
            return exact.copy() if _rigid_pose(exact) else None
    if right == 0 or right == len(samples):
        return None
    before_t, before_raw = samples[right - 1]
    after_t, after_raw = samples[right]
    before = np.asarray(before_raw, dtype=np.float64)
    after = np.asarray(after_raw, dtype=np.float64)
    span = after_t - before_t
    if span > max_span_s + STAMP_EPSILON_S \
            or not _rigid_pose(before) or not _rigid_pose(after):
        return None
    fraction = (stamp_s - before_t) / span
    blended = (1.0 - fraction) * before[:3, :3] \
        + fraction * after[:3, :3]
    left, _, right_basis = np.linalg.svd(blended)
    rotation = left @ right_basis
    if np.linalg.det(rotation) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right_basis
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = (1.0 - fraction) * before[:3, 3] \
        + fraction * after[:3, 3]
    return result


def _rigid_pose(value):
    return (
        value.shape == (4, 4)
        and np.isfinite(value).all()
        and np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6)
        and np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3),
                        atol=1e-6)
        and math.isclose(
            float(np.linalg.det(value[:3, :3])), 1.0, abs_tol=1e-6))


def cluster_band_relation(cluster, map_T_body, band, lidar_in_body,
                          lidar_to_body_rotation, grace_m):
    """Return route-band relation for sampled lidar-frame returns."""
    if map_T_body is None or band is None:
        return "unavailable", None
    count = min(len(cluster), MAX_BAND_SAMPLE_POINTS)
    if not count:
        return "unavailable", None
    indexes = np.linspace(0, len(cluster) - 1, count, dtype=int)
    sampled = np.asarray(cluster[indexes], dtype=np.float64)
    in_body = lidar_to_body(
        sampled, lidar_in_body, lidar_to_body_rotation)
    in_map = in_body @ map_T_body[:3, :3].T + map_T_body[:3, 3]
    inside = band.contains_many(in_map[:, :2], grace=grace_m)
    fraction = float(np.mean(inside))
    if fraction <= OUTSIDE_MAX_INSIDE_FRACTION:
        return "outside", fraction
    if fraction >= INSIDE_MIN_INSIDE_FRACTION:
        return "inside", fraction
    return "crossing", fraction


def lateral_profile(cluster, bin_m=PROFILE_BIN_M, max_bins=MAX_PROFILE_BINS):
    """Nearest forward return in each lateral slice of a cluster."""
    points = np.asarray(cluster, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] < 2:
        return None
    finite = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1])
    if not finite.any():
        return None
    x, y = points[finite, 0], points[finite, 1]
    span = float(y.max()) - float(y.min())
    while bin_m > 0.0 and (span / bin_m) + 1.0 > max_bins:
        bin_m *= 2.0
    first = int(math.floor(float(y.min()) / bin_m))
    count = int(math.floor(float(y.max()) / bin_m)) - first + 1
    index = np.clip(np.floor(y / bin_m).astype(int) - first, 0, count - 1)
    nearest = np.full(count, np.inf)
    np.minimum.at(nearest, index, x)
    return {
        "bin_m": round(float(bin_m), 3),
        "y0": round(float(first * bin_m), 3),
        "min_x": [None if not math.isfinite(value)
                  else round(float(value), 2) for value in nearest],
    }


def cluster_grid(points):
    """Connected components (8-neighbour) over a 2D cell grid."""
    cells = np.floor(points[:, :2] / CELL_M).astype(np.int64)
    order = np.lexsort((cells[:, 1], cells[:, 0]))
    cells, points = cells[order], points[order]
    keys, starts, counts = np.unique(
        cells, axis=0, return_index=True, return_counts=True)
    occupied = {tuple(key): index for index, key in enumerate(keys)
                if counts[index] >= MIN_CELL_POINTS}
    labels = {}
    clusters = []
    for cell in occupied:
        if cell in labels:
            continue
        member_cells, stack = [], [cell]
        labels[cell] = len(clusters)
        while stack:
            current = stack.pop()
            member_cells.append(current)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (current[0] + dx, current[1] + dy)
                    if neighbour in occupied and neighbour not in labels:
                        labels[neighbour] = len(clusters)
                        stack.append(neighbour)
        indexes = np.concatenate([
            np.arange(starts[occupied[item]],
                      starts[occupied[item]] + counts[occupied[item]])
            for item in member_cells])
        if len(indexes) >= MIN_CLUSTER_POINTS:
            clusters.append(points[indexes])
    return clusters


def classify(cluster):
    relative_height = cluster[:, 2] + SENSOR_HEIGHT_M
    height = float(relative_height.max())
    span = cluster[:, :2].max(axis=0) - cluster[:, :2].min(axis=0)
    footprint = float(np.hypot(span[0], span[1]))
    if footprint <= PERSON_MAX_FOOTPRINT_M and \
            PERSON_HEIGHT_M[0] <= height <= PERSON_HEIGHT_M[1]:
        return "person"
    if footprint >= VEHICLE_MIN_FOOTPRINT_M and \
            VEHICLE_HEIGHT_M[0] <= height <= VEHICLE_HEIGHT_M[1]:
        return "vehicle"
    return "obstacle"
