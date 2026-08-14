"""Remove returns already represented by the immutable localization map."""

import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def _load_binary_xyzi(path):
    path = Path(path)
    with path.open("rb") as source:
        header_lines = []
        for _ in range(64):
            line = source.readline()
            if not line:
                raise ValueError("unsupported PCD header: %s" % path)
            header_lines.append(line)
            if line.startswith(b"DATA "):
                break
        header = b"".join(header_lines)
        if b"FIELDS x y z intensity\n" not in header or \
                b"SIZE 4 4 4 4\n" not in header or \
                b"TYPE F F F F\n" not in header or \
                not header.endswith(b"DATA binary\n"):
            raise ValueError("fixed map must be binary XYZI PCD: %s" % path)
        point_line = next(
            (line for line in header_lines if line.startswith(b"POINTS ")),
            None)
        if point_line is None:
            raise ValueError("fixed map PCD has no POINTS field: %s" % path)
        point_count = int(point_line.split()[1])
        payload = source.read()
    if len(payload) != point_count * 16:
        raise ValueError("fixed map PCD payload size mismatch: %s" % path)
    records = np.frombuffer(payload, dtype=np.float32).reshape(point_count, 4)
    xyz = records[:, :3]
    return xyz[np.isfinite(xyz).all(axis=1)]


class FixedMapFilter(object):
    """KD-tree map membership used before obstacle clustering."""

    def __init__(self, map_path, expected_sha256, tolerance_m):
        path = Path(map_path)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != str(expected_sha256):
            raise ValueError(
                "fixed map SHA-256 mismatch: expected %s, got %s" %
                (expected_sha256, digest))
        tolerance_m = float(tolerance_m)
        if not np.isfinite(tolerance_m) or tolerance_m <= 0.0:
            raise ValueError("map match tolerance must be finite and positive")
        points = _load_binary_xyzi(path)
        if not len(points):
            raise ValueError("fixed map contains no finite points")
        self.tree = cKDTree(points)
        self.tolerance_m = tolerance_m

    def retain_novel(self, points_lidar, map_T_lidar):
        """Return only scan points farther than tolerance from mapped points."""
        points_lidar = np.asarray(points_lidar)
        map_T_lidar = np.asarray(map_T_lidar, dtype=np.float64)
        if points_lidar.ndim != 2 or points_lidar.shape[1] < 3:
            raise ValueError("scan points must be Nx3 or Nx4")
        if map_T_lidar.shape != (4, 4) or not np.isfinite(map_T_lidar).all():
            raise ValueError("map_T_lidar must be a finite 4x4 transform")
        map_xyz = (
            points_lidar[:, :3] @ map_T_lidar[:3, :3].T
            + map_T_lidar[:3, 3])
        distance, _ = self.tree.query(
            map_xyz, k=1, distance_upper_bound=self.tolerance_m)
        return points_lidar[~np.isfinite(distance)]
