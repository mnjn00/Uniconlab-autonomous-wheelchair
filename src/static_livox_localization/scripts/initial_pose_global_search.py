"""ROS-free global fallback for the automatic initial-pose node."""

import math
from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree

from initial_pose_candidates import InitializationCandidate


class BinaryPcdError(Exception):
    """A stable PCD boundary error."""

    def __init__(self, reason: str, path: Path):
        self.reason = reason
        self.path = path
        super().__init__(reason, str(path))

    def __str__(self) -> str:
        return "{}: {}".format(self.reason, self.path)


def load_pcd_xyz(path: Path) -> np.ndarray:
    """Read the exact binary XYZI layout required by the runtime map."""

    try:
        with path.open("rb") as source:
            header_lines = []
            for _ in range(64):
                line = source.readline()
                if not line:
                    raise BinaryPcdError("unsupported_header", path)
                header_lines.append(line)
                if line.startswith(b"DATA "):
                    break
            else:
                raise BinaryPcdError("unsupported_header", path)
            header = b"".join(header_lines)
            if b"FIELDS x y z intensity\n" not in header:
                raise BinaryPcdError("unsupported_fields", path)
            if b"SIZE 4 4 4 4\n" not in header or b"TYPE F F F F\n" not in header:
                raise BinaryPcdError("unsupported_fields", path)
            if not header.endswith(b"DATA binary\n"):
                raise BinaryPcdError("unsupported_encoding", path)
            point_line = next(
                (line for line in header_lines if line.startswith(b"POINTS ")),
                None,
            )
            if point_line is None:
                raise BinaryPcdError("missing_point_count", path)
            try:
                point_count = int(point_line.split()[1])
            except (IndexError, ValueError) as error:
                raise BinaryPcdError("invalid_point_count", path) from error
            payload = source.read()
    except OSError as error:
        raise BinaryPcdError("unreadable_pcd", path) from error

    expected_bytes = point_count * 16
    if len(payload) != expected_bytes:
        raise BinaryPcdError("payload_size_mismatch", path)
    records = np.frombuffer(payload, dtype=np.float32).reshape(point_count, 4)
    points = records[:, :3]
    return points[np.isfinite(points).all(axis=1)]


def load_trajectory_candidates(
    trajectory_path: Path,
    spacing_m: float,
) -> Tuple[Tuple[float, float, float, float], ...]:
    """Sample position and heading hypotheses along the mapping trajectory."""

    rows = np.loadtxt(str(trajectory_path), ndmin=2)
    positions = rows[:, 1:4]
    keep = [0]
    for index in range(1, len(positions)):
        if (
            np.linalg.norm(positions[index, :2] - positions[keep[-1], :2])
            >= spacing_m
        ):
            keep.append(index)
    candidates = []
    for index in keep:
        x, y, z = positions[index]
        nxt = positions[min(index + 5, len(positions) - 1)]
        heading = math.atan2(nxt[1] - y, nxt[0] - x)
        candidates.append((float(x), float(y), float(z), heading))
    return tuple(candidates)


def voxel_downsample(points: np.ndarray, size_m: float, cap: int) -> np.ndarray:
    """Select one deterministic representative per voxel and cap the sample."""

    keys = np.floor(points / size_m).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    sampled = points[np.sort(unique_idx)]
    if len(sampled) > cap:
        sampled = sampled[
            np.random.RandomState(0).choice(len(sampled), cap, False)
        ]
    return sampled


def score_global_candidates(
    sample: np.ndarray,
    map_points: np.ndarray,
    candidates: Sequence[Tuple[float, float, float, float]],
    inlier_radius_m: float,
) -> Tuple[InitializationCandidate, ...]:
    """Rank trajectory/yaw hypotheses by bounded nearest-map inlier fraction."""

    tree = cKDTree(map_points)
    yaw_offsets = (
        0.0,
        math.pi,
        math.pi / 4,
        -math.pi / 4,
        3 * math.pi / 4,
        -3 * math.pi / 4,
    )
    scored: List[InitializationCandidate] = []
    for x, y, z, heading in candidates:
        for offset in yaw_offsets:
            yaw = heading + offset
            cosine, sine = math.cos(yaw), math.sin(yaw)
            rotation = np.array(
                [[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]],
                np.float32,
            )
            world = sample @ rotation.T + np.array([x, y, z], np.float32)
            distances, _ = tree.query(
                world,
                k=1,
                distance_upper_bound=inlier_radius_m,
            )
            scored.append(
                InitializationCandidate(
                    x=x,
                    y=y,
                    z=z,
                    yaw_rad=yaw,
                    score=float(np.isfinite(distances).mean()),
                    source="global_search",
                )
            )
    return tuple(
        sorted(
            scored,
            key=lambda item: item.score if item.score is not None else -math.inf,
            reverse=True,
        )
    )
