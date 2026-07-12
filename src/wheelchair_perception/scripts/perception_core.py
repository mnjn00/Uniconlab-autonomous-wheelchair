#!/usr/bin/env python3
"""Deterministic, ROS-independent processing for canonical Livox clouds.

This module produces navigation evidence only.  In particular, ``Health`` has no
CLEAR state and must not be used as a safety permission; the collision supervisor
consumes the unfiltered canonical cloud independently.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from itertools import combinations
import math
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple


CANONICAL_FIELDS = (
    ("x", 0, "FLOAT32", 1),
    ("y", 4, "FLOAT32", 1),
    ("z", 8, "FLOAT32", 1),
    ("intensity", 12, "FLOAT32", 1),
    ("offset_time", 16, "UINT32", 1),
    ("line", 20, "UINT8", 1),
    ("tag", 21, "UINT8", 1),
    ("reflectivity", 22, "UINT8", 1),
    ("lidar_id", 23, "UINT8", 1),
)
_UINT32_MAX = (1 << 32) - 1
_INT64_MAX = (1 << 63) - 1


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float
    offset_time: int
    reflectivity: int
    tag: int
    line: int
    lidar_id: int

    @property
    def intensity(self) -> float:
        return float(self.reflectivity)


@dataclass(frozen=True)
class CloudInput:
    stamp_s: float
    frame_id: str
    points: Sequence[Point]
    source_id: str = "canonical_livox"
    fields: Tuple[Tuple[str, int, str, int], ...] = CANONICAL_FIELDS
    height: int = 1
    width: Optional[int] = None
    is_bigendian: bool = False
    point_step: int = 24
    row_step: Optional[int] = None
    is_dense: bool = True


@dataclass(frozen=True)
class ImuSample:
    stamp_s: float
    frame_id: str
    orientation_xyzw: Optional[Tuple[float, float, float, float]] = None
    linear_acceleration_xyz: Optional[Tuple[float, float, float]] = None
    angular_velocity_xyz: Optional[Tuple[float, float, float]] = None


@dataclass(frozen=True)
class Cluster:
    points: Tuple[Point, ...]
    centroid: Tuple[float, float, float]
    minimum: Tuple[float, float, float]
    maximum: Tuple[float, float, float]
    min_range_m: float


@dataclass(frozen=True)
class Health:
    ok: bool
    code: str
    reasons: Tuple[str, ...]
    stamp_s: float
    age_s: float
    input_points: int
    finite_points: int
    self_filtered_points: int
    roi_filtered_points: int
    voxel_points: int
    ground_points: int
    obstacle_points: int
    cluster_count: int
    observed_rate_hz: Optional[float]
    gap_s: Optional[float]
    sequence: int
    # Deliberately constant: perception is never a safety permission source.
    safety_clear: bool = False


@dataclass(frozen=True)
class PerceptionResult:
    obstacle_points: Tuple[Point, ...]
    clusters: Tuple[Cluster, ...]
    health: Health
    ground_points: Tuple[Point, ...] = ()


@dataclass(frozen=True)
class PerceptionConfig:
    schema_version: int
    policy_id: str
    qualification: str
    hardware_motion_authorized: bool
    passenger_operation_authorized: bool
    expected_cloud_frame: str
    expected_imu_frame: str
    expected_source_id: str
    cloud_ttl_s: float
    imu_ttl_s: float
    future_tolerance_s: float
    minimum_rate_hz: float
    maximum_gap_s: float
    roi_min_x_m: float
    roi_max_x_m: float
    roi_min_y_m: float
    roi_max_y_m: float
    roi_min_z_m: float
    roi_max_z_m: float
    self_min_x_m: float
    self_max_x_m: float
    self_min_y_m: float
    self_max_y_m: float
    self_min_z_m: float
    self_max_z_m: float
    voxel_size_m: float
    ground_cell_size_m: float
    ground_tolerance_m: float
    max_ground_slope_deg: float
    obstacle_min_height_m: float
    obstacle_max_height_m: float
    cluster_cell_size_m: float
    cluster_tolerance_m: float
    cluster_min_points: int
    cluster_max_points: int
    min_ground_points: int
    gravity_alignment_required: bool

    @classmethod
    def from_mapping(cls, mapping: Mapping[str, Any]) -> "PerceptionConfig":
        if not isinstance(mapping, Mapping):
            raise ValueError("configuration must be a mapping")
        expected = {item.name for item in fields(cls)}
        actual = set(mapping)
        unknown = sorted(actual - expected)
        missing = sorted(expected - actual)
        if unknown or missing:
            raise ValueError("closed configuration violation: unknown=%r missing=%r" % (unknown, missing))
        config = cls(**dict(mapping))
        config._validate()
        return config

    def _validate(self) -> None:
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("schema_version must equal 1")
        if not isinstance(self.policy_id, str) or not self.policy_id or len(self.policy_id) > 64:
            raise ValueError("policy_id must contain 1..64 characters")
        if not isinstance(self.qualification, str) or self.qualification not in ("simulation_only", "replay_only"):
            raise ValueError("qualification must be simulation_only or replay_only")
        if self.hardware_motion_authorized is not False or self.passenger_operation_authorized is not False:
            raise ValueError("perception configuration cannot authorize hardware or passengers")
        identities = (self.expected_cloud_frame, self.expected_imu_frame, self.expected_source_id)
        if any(not isinstance(value, str) or not value or len(value) > 128 for value in identities):
            raise ValueError("expected frames and source must contain 1..128 characters")
        positive = (
            "cloud_ttl_s", "imu_ttl_s", "future_tolerance_s", "minimum_rate_hz",
            "maximum_gap_s", "voxel_size_m", "ground_cell_size_m", "ground_tolerance_m",
            "cluster_cell_size_m", "cluster_tolerance_m",
        )
        for name in positive:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
                raise ValueError("%s must be finite and positive" % name)
        bounded_positive = {
            "cloud_ttl_s": 10.0, "imu_ttl_s": 10.0, "future_tolerance_s": 1.0,
            "minimum_rate_hz": 1000.0, "maximum_gap_s": 10.0, "voxel_size_m": 1.0,
            "ground_cell_size_m": 5.0, "ground_tolerance_m": 1.0,
            "cluster_cell_size_m": 5.0, "cluster_tolerance_m": 5.0,
        }
        if any(getattr(self, name) > upper for name, upper in bounded_positive.items()):
            raise ValueError("configuration exceeds a numeric upper bound")
        if self.voxel_size_m < 0.001 or self.cluster_tolerance_m < 0.01:
            raise ValueError("voxel/cluster bounds exceeded")
        for low, high, label in (
            (self.roi_min_x_m, self.roi_max_x_m, "roi x"),
            (self.roi_min_y_m, self.roi_max_y_m, "roi y"),
            (self.roi_min_z_m, self.roi_max_z_m, "roi z"),
            (self.self_min_x_m, self.self_max_x_m, "self x"),
            (self.self_min_y_m, self.self_max_y_m, "self y"),
            (self.self_min_z_m, self.self_max_z_m, "self z"),
            (self.obstacle_min_height_m, self.obstacle_max_height_m, "obstacle height"),
        ):
            if (any(isinstance(v, bool) or not isinstance(v, (int, float))
                    or not math.isfinite(v) for v in (low, high)) or low >= high):
                raise ValueError("invalid %s bounds" % label)
        if any(abs(float(value)) > 100.0 for value in (
                self.roi_min_x_m, self.roi_max_x_m, self.roi_min_y_m, self.roi_max_y_m,
                self.roi_min_z_m, self.roi_max_z_m)):
            raise ValueError("ROI bounds exceed 100 m")
        if any(abs(float(value)) > 10.0 for value in (
                self.self_min_x_m, self.self_max_x_m, self.self_min_y_m, self.self_max_y_m,
                self.self_min_z_m, self.self_max_z_m)):
            raise ValueError("self-filter bounds exceed 10 m")
        if self.obstacle_min_height_m < 0.0 or self.obstacle_max_height_m > 10.0:
            raise ValueError("obstacle heights must remain within [0, 10] m")
        if (isinstance(self.max_ground_slope_deg, bool)
                or not isinstance(self.max_ground_slope_deg, (int, float))
                or not math.isfinite(self.max_ground_slope_deg)
                or not 0.0 <= self.max_ground_slope_deg <= 30.0):
            raise ValueError("max_ground_slope_deg must be within [0, 30]")
        for name in ("cluster_min_points", "cluster_max_points", "min_ground_points"):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError("%s must be a positive integer" % name)
        if self.cluster_min_points > self.cluster_max_points:
            raise ValueError("cluster point bounds are reversed")
        if self.cluster_max_points > 1000000 or self.min_ground_points > 1000000:
            raise ValueError("point-count bounds exceeded")
        if type(self.gravity_alignment_required) is not bool:
            raise ValueError("gravity_alignment_required must be boolean")


class PerceptionCore:
    def __init__(self, config: PerceptionConfig):
        if not isinstance(config, PerceptionConfig):
            raise TypeError("config must be PerceptionConfig")
        config._validate()
        self.config = config
        self._last_stamp_s: Optional[float] = None
        self._sequence = 0

    def process(
        self, cloud: CloudInput, imu_or_none: Optional[ImuSample] = None,
        now_s: Optional[float] = None,
    ) -> PerceptionResult:
        self._sequence += 1
        reasons = list(self._validate_cloud(cloud))
        stamp = float(cloud.stamp_s) if _finite_number(cloud.stamp_s) else 0.0
        now = stamp if now_s is None else float(now_s)
        age = now - stamp
        if not math.isfinite(now):
            reasons.append("E_NOW_NONFINITE")
            age = math.inf
        elif age > self.config.cloud_ttl_s:
            reasons.append("E_CLOUD_STALE")
        elif age < -self.config.future_tolerance_s:
            reasons.append("E_CLOUD_FUTURE")

        gap = None if self._last_stamp_s is None else stamp - self._last_stamp_s
        rate = None if gap is None or gap <= 0.0 else 1.0 / gap
        if gap is not None:
            if gap <= 0.0:
                reasons.append("E_TIME_REGRESSION")
            elif gap > self.config.maximum_gap_s:
                reasons.append("E_RATE_GAP")
            elif rate is not None and rate < self.config.minimum_rate_hz:
                reasons.append("E_RATE_LOW")
        self._last_stamp_s = stamp

        roll_pitch = self._imu_roll_pitch(imu_or_none, stamp, reasons)
        if reasons:
            return self._failed(stamp, age, gap, rate, len(cloud.points), reasons)

        aligned = tuple(_align_point(p, roll_pitch[0], roll_pitch[1]) for p in cloud.points)
        self_filtered = tuple(p for p in aligned if not self._inside_self(p))
        roi = tuple(p for p in self_filtered if self._inside_roi(p))
        voxel = self._voxelize(roi)
        plane = self._ground_plane(voxel)
        if plane is None:
            reasons.append("E_GROUND_UNRESOLVED")
            return self._failed(
                stamp, age, gap, rate, len(cloud.points), reasons,
                finite=len(aligned), self_count=len(aligned) - len(self_filtered),
                roi_count=len(self_filtered) - len(roi), voxel_count=len(voxel),
            )

        a, b, c = plane
        slope = math.degrees(math.atan(math.hypot(a, b)))
        if slope > self.config.max_ground_slope_deg:
            reasons.append("E_GROUND_SLOPE")
        ground = []
        obstacles = []
        for point in voxel:
            height = point.z - (a * point.x + b * point.y + c)
            if abs(height) <= self.config.ground_tolerance_m:
                ground.append(point)
            elif self.config.obstacle_min_height_m <= height <= self.config.obstacle_max_height_m:
                obstacles.append(point)
        obstacle_points = tuple(sorted(obstacles, key=_point_key))
        clusters = self._clusters(obstacle_points)
        health = Health(
            ok=not reasons, code="OK" if not reasons else reasons[0], reasons=tuple(reasons),
            stamp_s=stamp, age_s=age, input_points=len(cloud.points), finite_points=len(aligned),
            self_filtered_points=len(aligned) - len(self_filtered),
            roi_filtered_points=len(self_filtered) - len(roi), voxel_points=len(voxel),
            ground_points=len(ground), obstacle_points=len(obstacle_points),
            cluster_count=len(clusters), observed_rate_hz=rate, gap_s=gap, sequence=self._sequence,
        )
        return PerceptionResult(obstacle_points, clusters, health, tuple(sorted(ground, key=_point_key)))

    def _validate_cloud(self, cloud: CloudInput) -> Tuple[str, ...]:
        errors = []
        if cloud.frame_id != self.config.expected_cloud_frame:
            errors.append("E_CLOUD_FRAME")
        if cloud.source_id != self.config.expected_source_id:
            errors.append("E_CLOUD_SOURCE")
        width = len(cloud.points) if cloud.width is None else cloud.width
        row_step = 24 * width if cloud.row_step is None else cloud.row_step
        if (cloud.fields != CANONICAL_FIELDS or cloud.height != 1 or width != len(cloud.points)
                or cloud.is_bigendian or cloud.point_step != 24 or row_step != 24 * width
                or not cloud.is_dense):
            errors.append("E_CANONICAL_LAYOUT")
        if not _finite_number(cloud.stamp_s) or cloud.stamp_s < 0.0:
            errors.append("E_CLOUD_TIME")
        stamp_ns = None
        if _finite_number(cloud.stamp_s) and cloud.stamp_s >= 0.0:
            scaled_stamp = cloud.stamp_s * 1e9
            if not math.isfinite(scaled_stamp) or scaled_stamp > _INT64_MAX:
                errors.append("E_POINT_TIME_OVERFLOW")
            else:
                stamp_ns = int(scaled_stamp)
        for point in cloud.points:
            if not isinstance(point, Point):
                errors.append("E_POINT_TYPE")
                break
            if not all(_finite_number(v) for v in (point.x, point.y, point.z)):
                errors.append("E_POINT_NONFINITE")
                break
            if type(point.offset_time) is not int or not 0 <= point.offset_time <= _UINT32_MAX:
                errors.append("E_POINT_TIME")
                break
            if stamp_ns is not None and stamp_ns + point.offset_time > _INT64_MAX:
                errors.append("E_POINT_TIME_OVERFLOW")
                break
            if any(type(v) is not int or not 0 <= v <= 255 for v in
                   (point.reflectivity, point.tag, point.line, point.lidar_id)):
                errors.append("E_POINT_METADATA")
                break
        return tuple(errors)

    def _imu_roll_pitch(self, imu: Optional[ImuSample], stamp: float, reasons: list) -> Tuple[float, float]:
        if imu is None:
            if self.config.gravity_alignment_required:
                reasons.append("E_IMU_MISSING")
            return (0.0, 0.0)
        if imu.frame_id != self.config.expected_imu_frame:
            reasons.append("E_IMU_FRAME")
        if not _finite_number(imu.stamp_s) or abs(stamp - imu.stamp_s) > self.config.imu_ttl_s:
            reasons.append("E_IMU_STALE")
        quaternion = imu.orientation_xyzw
        if quaternion is not None:
            if len(quaternion) != 4 or not all(_finite_number(v) for v in quaternion):
                reasons.append("E_IMU_ORIENTATION")
                return (0.0, 0.0)
            norm = math.sqrt(sum(v * v for v in quaternion))
            if abs(norm - 1.0) > 0.01:
                reasons.append("E_IMU_ORIENTATION")
                return (0.0, 0.0)
            x, y, z, w = (v / norm for v in quaternion)
            roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            pitch_arg = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
            return (roll, math.asin(pitch_arg))
        acceleration = imu.linear_acceleration_xyz
        if acceleration is None or len(acceleration) != 3 or not all(_finite_number(v) for v in acceleration):
            reasons.append("E_IMU_GRAVITY")
            return (0.0, 0.0)
        ax, ay, az = acceleration
        magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        if not 9.55665 <= magnitude <= 10.05665:
            reasons.append("E_IMU_DYNAMIC")
            return (0.0, 0.0)
        # A stationary ROS IMU reports +g upward in body axes.
        return (math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az)))

    def _inside_self(self, p: Point) -> bool:
        c = self.config
        return (c.self_min_x_m <= p.x <= c.self_max_x_m and
                c.self_min_y_m <= p.y <= c.self_max_y_m and
                c.self_min_z_m <= p.z <= c.self_max_z_m)

    def _inside_roi(self, p: Point) -> bool:
        c = self.config
        return (c.roi_min_x_m <= p.x <= c.roi_max_x_m and
                c.roi_min_y_m <= p.y <= c.roi_max_y_m and
                c.roi_min_z_m <= p.z <= c.roi_max_z_m)

    def _voxelize(self, points: Iterable[Point]) -> Tuple[Point, ...]:
        size = self.config.voxel_size_m
        representatives: Dict[Tuple[int, int, int], Point] = {}
        scores: Dict[Tuple[int, int, int], Tuple[Any, ...]] = {}
        for point in points:
            key = (math.floor(point.x / size), math.floor(point.y / size), math.floor(point.z / size))
            center = tuple((index + 0.5) * size for index in key)
            score = ((point.x - center[0]) ** 2 + (point.y - center[1]) ** 2 +
                     (point.z - center[2]) ** 2, _point_key(point))
            if key not in scores or score < scores[key]:
                scores[key] = score
                representatives[key] = point
        return tuple(representatives[key] for key in sorted(representatives))

    def _ground_plane(self, points: Sequence[Point]) -> Optional[Tuple[float, float, float]]:
        size = self.config.ground_cell_size_m
        lowest: Dict[Tuple[int, int], Point] = {}
        for point in points:
            key = (math.floor(point.x / size), math.floor(point.y / size))
            if key not in lowest or _point_key(point)[2:] < _point_key(lowest[key])[2:]:
                lowest[key] = point
        candidates = tuple(lowest[key] for key in sorted(lowest))
        if len(candidates) < self.config.min_ground_points:
            return None
        # Deterministic bounded RANSAC over cell minima.  Candidate models are
        # ranked by support in the nearest ground-seed cells before global
        # support, preventing a distant obstacle surface from outvoting the
        # traversable plane around the wheelchair.
        near_candidates = tuple(sorted(
            candidates,
            key=lambda p: (p.x * p.x + p.y * p.y, _point_key(p)),
        )[:self.config.min_ground_points])
        sample_count = min(12, len(candidates))
        if sample_count == len(candidates):
            sample = candidates
        else:
            indexes = tuple((i * (len(candidates) - 1)) // (sample_count - 1)
                            for i in range(sample_count))
            sample = tuple(candidates[i] for i in indexes)
        best = None
        tolerance = self.config.ground_tolerance_m
        for triplet in combinations(sample, 3):
            plane = _least_squares_plane(triplet)
            if plane is None or math.degrees(math.atan(math.hypot(plane[0], plane[1]))) > 45.0:
                continue
            inliers = tuple(
                p for p in candidates
                if abs(p.z - (plane[0] * p.x + plane[1] * p.y + plane[2])) <= tolerance
            )
            if len(inliers) < self.config.min_ground_points:
                continue
            near_inliers = sum(
                abs(p.z - (plane[0] * p.x + plane[1] * p.y + plane[2])) <= tolerance
                for p in near_candidates
            )
            residual = sum(abs(p.z - (plane[0] * p.x + plane[1] * p.y + plane[2]))
                           for p in inliers)
            score = (-near_inliers, -len(inliers), residual, plane)
            if best is None or score < best[0]:
                best = (score, inliers)
        if best is None:
            return None
        return _least_squares_plane(best[1])

    def _clusters(self, points: Sequence[Point]) -> Tuple[Cluster, ...]:
        if not points:
            return ()
        cell = self.config.cluster_cell_size_m
        buckets: Dict[Tuple[int, int, int], list] = {}
        for index, point in enumerate(points):
            key = (math.floor(point.x / cell), math.floor(point.y / cell), math.floor(point.z / cell))
            buckets.setdefault(key, []).append(index)
        visited = set()
        groups = []
        reach = int(math.ceil(self.config.cluster_tolerance_m / cell))
        tolerance2 = self.config.cluster_tolerance_m ** 2
        for seed in range(len(points)):
            if seed in visited:
                continue
            visited.add(seed)
            queue = [seed]
            group = []
            while queue:
                index = queue.pop(0)
                group.append(index)
                p = points[index]
                key = (math.floor(p.x / cell), math.floor(p.y / cell), math.floor(p.z / cell))
                neighbors = []
                for dx in range(-reach, reach + 1):
                    for dy in range(-reach, reach + 1):
                        for dz in range(-reach, reach + 1):
                            neighbors.extend(buckets.get((key[0] + dx, key[1] + dy, key[2] + dz), ()))
                for other in sorted(neighbors):
                    if other not in visited and _distance_squared(p, points[other]) <= tolerance2:
                        visited.add(other)
                        queue.append(other)
            if self.config.cluster_min_points <= len(group) <= self.config.cluster_max_points:
                groups.append(tuple(sorted((points[i] for i in group), key=_point_key)))
        result = []
        for group in groups:
            count = float(len(group))
            centroid = tuple(sum(getattr(p, axis) for p in group) / count for axis in ("x", "y", "z"))
            minimum = tuple(min(getattr(p, axis) for p in group) for axis in ("x", "y", "z"))
            maximum = tuple(max(getattr(p, axis) for p in group) for axis in ("x", "y", "z"))
            min_range = min(math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z) for p in group)
            result.append(Cluster(group, centroid, minimum, maximum, min_range))
        return tuple(sorted(result, key=lambda c: (_point_key(c.points[0]), len(c.points))))

    def _failed(self, stamp: float, age: float, gap: Optional[float], rate: Optional[float],
                input_count: int, reasons: Sequence[str], finite: int = 0, self_count: int = 0,
                roi_count: int = 0, voxel_count: int = 0) -> PerceptionResult:
        unique = tuple(dict.fromkeys(reasons))
        return PerceptionResult((), (), Health(
            False, unique[0], unique, stamp, age, input_count, finite, self_count, roi_count,
            voxel_count, 0, 0, 0, rate, gap, self._sequence,
        ))


def _finite_number(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def count_adjacent_offset_decreases(points: Iterable[Point]) -> int:
    """Count source-order offset decreases without treating interleaving as corruption."""
    decreases = 0
    previous = None
    for point in points:
        if previous is not None and point.offset_time < previous:
            decreases += 1
        previous = point.offset_time
    return decreases


def _point_key(point: Point) -> Tuple[Any, ...]:
    return (point.x, point.y, point.z, point.offset_time, point.line, point.tag,
            point.reflectivity, point.lidar_id)


def _align_point(point: Point, roll: float, pitch: float) -> Point:
    # body -> level-heading rotation R_y(pitch) R_x(roll), with yaw intentionally removed.
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    x1 = point.x
    y1 = cr * point.y - sr * point.z
    z1 = sr * point.y + cr * point.z
    return Point(cp * x1 + sp * z1, y1, -sp * x1 + cp * z1,
                 point.offset_time, point.reflectivity, point.tag, point.line, point.lidar_id)


def _least_squares_plane(points: Sequence[Point]) -> Optional[Tuple[float, float, float]]:
    sx = sy = sz = sxx = syy = sxy = sxz = syz = 0.0
    for p in points:
        sx += p.x; sy += p.y; sz += p.z
        sxx += p.x * p.x; syy += p.y * p.y; sxy += p.x * p.y
        sxz += p.x * p.z; syz += p.y * p.z
    matrix = [[sxx, sxy, sx, sxz], [sxy, syy, sy, syz], [sx, sy, float(len(points)), sz]]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: (abs(matrix[row][column]), -row))
        if abs(matrix[pivot][column]) < 1e-12:
            return None
        matrix[column], matrix[pivot] = matrix[pivot], matrix[column]
        divisor = matrix[column][column]
        matrix[column] = [value / divisor for value in matrix[column]]
        for row in range(3):
            if row == column:
                continue
            factor = matrix[row][column]
            matrix[row] = [matrix[row][i] - factor * matrix[column][i] for i in range(4)]
    return (matrix[0][3], matrix[1][3], matrix[2][3])


def _distance_squared(a: Point, b: Point) -> float:
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2
