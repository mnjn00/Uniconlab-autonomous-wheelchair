#!/usr/bin/env python3
"""Fail-closed collision evidence supervisor with a ROS-independent core.

The policy shipped with this node is simulation-only.  This process publishes
untrusted collision evidence; only ``safety_gate`` may publish ``/cmd_vel_safe``.
"""

from collections import deque
from dataclasses import dataclass, fields
import hashlib
import json
import math
import threading
import time
from typing import Any, Deque, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


IMMUTABLE_HARD_LINEAR_SPEED_MPS = 0.55
CLOCK_FUTURE_TOLERANCE_S = 0.05
SLOPE_EVIDENCE_BUFFER_HORIZON_S = 0.20
SLOPE_EVIDENCE_BUFFER_MAXLEN = 16
SLOPE_EVIDENCE_JOIN_TIMEOUT_S = 0.05

UNKNOWN, CLEAR, CAUTION, STOP = 0, 1, 2, 3
VIS_UNKNOWN, VIS_FULL, VIS_PARTIAL, VIS_BLIND = 0, 1, 2, 3
MOTION_NONE, MOTION_STATIC, MOTION_DYNAMIC, MOTION_AMBIGUOUS = 0, 1, 2, 3
INTENT_HOLD, INTENT_PROCEED, INTENT_SLOW, INTENT_STOP = 0, 1, 2, 3
SIGNAL_UNKNOWN, SIGNAL_CLEAR, SIGNAL_STOP = 0, 1, 2

COLLISION = 1 << 4
SENSOR_STALE = 1 << 12
COLLISION_BLIND = 1 << 13
COLLISION_TTC = 1 << 14
COLLISION_DISTANCE = 1 << 15
TF = 1 << 20
HARDWARE_UNVERIFIED = 1 << 24
COLLISION_OCCLUDED = 1 << 26
CORRUPT_DATA = 1 << 29
INPUT_UNKNOWN = 1 << 31
ODOM_STALE = 1 << 33
LIDAR_STALE = 1 << 35
POLICY_MISMATCH = 1 << 36


@dataclass(frozen=True)
class CollisionPolicy:
    schema_version: int
    policy_id: str
    qualification: str
    hardware_motion_authorized: bool
    passenger_operation_authorized: bool
    evaluation_frame: str
    cloud_ttl_s: float
    odom_ttl_s: float
    command_ttl_s: float
    footprint_length_m: float
    footprint_width_m: float
    localization_uncertainty_m: float
    transform_uncertainty_m: float
    point_noise_m: float
    fixed_expansion_m: float
    max_horizon_s: float
    static_speed_below_mps: float
    minimum_observations: int
    ambiguous_approach_speed_mps: float
    intent_ttl_s: float
    first_command_grace_s: float
    ground_min_z_m: float
    ground_max_z_m: float
    voxel_size_m: float
    cluster_tolerance_m: float
    cluster_min_points: int
    cluster_max_points: int
    association_max_age_s: float
    association_max_displacement_m: float
    dynamic_covariance_max_mps2: float
    coverage_bins: int
    coverage_window_s: float
    coverage_min_frames: int
    coverage_max_frames: int
    coverage_max_frame_gap_s: float
    coverage_elevation_bins: int
    coverage_min_elevation_rad: float
    coverage_max_elevation_rad: float
    coverage_motion_linear_tolerance_mps: float
    coverage_motion_angular_tolerance_rps: float
    gravity_mps2: float
    simulation_min_deceleration_mps2: float
    simulation_driver_latency_s: float
    compute_and_gate_budget_s: float
    fixed_uncertainty_margin_m: float
    required_forward_coverage_fraction: float
    caution_ttc_s: float
    caution_clearance_margin_m: float
    caution_max_linear_mps: float
    stop_ttc_margin_s: float
    clear_consecutive_frames: int
    clear_hold_s: float
    clearance_strictly_above_margin_m: float
    ttc_strictly_above_s: float
    policy_sha256: str

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "CollisionPolicy":
        names = {field.name for field in fields(cls)}
        if set(raw) != names:
            missing, extra = names - set(raw), set(raw) - names
            raise ValueError("policy keys mismatch; missing=%s extra=%s" % (sorted(missing), sorted(extra)))
        policy = cls(**{name: raw[name] for name in names})
        policy._validate(raw)
        return policy

    @classmethod
    def load(cls, path: str) -> "CollisionPolicy":
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError("PyYAML is required to load collision policy") from exc
        with open(path, "r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        if not isinstance(raw, dict):
            raise ValueError("collision policy must be a mapping")
        return cls.from_mapping(raw)

    @staticmethod
    def hash_mapping(raw: Mapping[str, Any]) -> str:
        content = {key: value for key, value in raw.items() if key != "policy_sha256"}
        canonical = json.dumps(content, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()

    def _validate(self, raw: Mapping[str, Any]) -> None:
        if self.schema_version != 1 or self.qualification != "simulation_only":
            raise ValueError("only schema v1 simulation-only collision policies are accepted")
        if self.hardware_motion_authorized or self.passenger_operation_authorized:
            raise ValueError("simulation collision policy cannot claim hardware/passenger authority")
        if self.evaluation_frame != "base_footprint":
            raise ValueError("collision observations must be transformed to base_footprint")
        numeric = [getattr(self, f.name) for f in fields(self) if f.name not in {
            "policy_id", "qualification", "evaluation_frame", "policy_sha256",
            "schema_version", "minimum_observations", "clear_consecutive_frames",
            "cluster_min_points", "cluster_max_points", "coverage_bins", "coverage_min_frames",
            "coverage_max_frames", "coverage_elevation_bins", "ground_min_z_m",
            "coverage_min_elevation_rad", "coverage_max_elevation_rad",
            "hardware_motion_authorized", "passenger_operation_authorized",
        }]
        if not all(_finite(value) and value >= 0.0 for value in numeric):
            raise ValueError("policy numeric values must be finite and nonnegative")
        if not self.policy_id or self.minimum_observations < 3 or self.clear_consecutive_frames < 1:
            raise ValueError("invalid collision policy identity/counts")
        if (self.cluster_min_points < 1 or self.cluster_max_points < self.cluster_min_points
                or self.coverage_bins < 1 or self.coverage_elevation_bins < 1
                or self.coverage_min_frames < 2
                or self.coverage_max_frames < self.coverage_min_frames):
            raise ValueError("invalid preprocessing counts")
        if not self.ground_min_z_m < self.ground_max_z_m:
            raise ValueError("ground band must be ordered")
        if not 0.0 < self.required_forward_coverage_fraction <= 1.0:
            raise ValueError("coverage threshold must be in (0, 1]")
        if not self.coverage_min_elevation_rad < self.coverage_max_elevation_rad:
            raise ValueError("coverage elevation band must be ordered")
        if self.stop_ttc_margin_s != 0.50:
            raise ValueError("schema v1 stop_ttc_margin_s must be exactly 0.50")
        if (self.simulation_min_deceleration_mps2 <= 0.0 or self.voxel_size_m <= 0.0
                or self.cluster_tolerance_m <= 0.0 or self.association_max_age_s <= 0.0
                or self.association_max_displacement_m <= 0.0 or self.intent_ttl_s <= 0.0
                or self.coverage_window_s <= 0.0 or self.coverage_max_frame_gap_s <= 0.0):
            raise ValueError("positive policy bounds must be positive")
        expected = self.hash_mapping(raw)
        if self.policy_sha256 != expected:
            raise ValueError("collision policy SHA-256 mismatch")


@dataclass(frozen=True)
class PointObservation:
    x: float
    y: float
    z: float = 0.0
    vx: Optional[float] = None
    vy: Optional[float] = None
    track_id: Optional[str] = None
    observation_count: int = 1
    covariance_valid: bool = False


@dataclass(frozen=True)
class CloudTransformResult:
    points: Tuple[PointObservation, ...]
    frame_id: str
    transform_age_s: float
    ok: bool
    reason: str


@dataclass(frozen=True)
class CloudProcessingResult:
    points: Tuple[PointObservation, ...]
    raw_point_count: int
    expected_coverage_bins: int
    observed_coverage_bins: int
    coverage_fraction: float
    ok: bool
    reason: str


@dataclass
class _Track:
    track_id: int
    stamp_s: float
    x: float
    y: float
    z: float
    compensated_velocities: List[Tuple[float, float]]
    observation_count: int


class CloudPreprocessorTracker:
    """Deterministic, ROS-independent simulation cloud preprocessor and tracker."""

    def __init__(self, policy: CollisionPolicy):
        self.policy = policy
        self._tracks: Dict[int, _Track] = {}
        self._next_track_id = 1
        self._coverage_frames: Deque[Tuple[float, float, float, frozenset]] = deque(
            maxlen=policy.coverage_max_frames
        )

    def process(self, points: Iterable[PointObservation], stamp_s: float,
                linear_speed_mps: float = 0.0, angular_speed_rps: float = 0.0,
                sensor_origin: Sequence[float] = (0.0, 0.0, 0.0),
                ego_linear_speed_mps: Optional[float] = None,
                ego_angular_speed_rps: Optional[float] = None) -> CloudProcessingResult:
        prior_tracks = self._tracks
        prior_next_track_id = self._next_track_id
        prior_coverage_frames = self._coverage_frames
        self._coverage_frames = deque(
            prior_coverage_frames, maxlen=self.policy.coverage_max_frames
        )
        try:
            if not _finite(stamp_s) or stamp_s < 0.0:
                raise ValueError("invalid_cloud_timestamp")
            raw = tuple(points)
            if len(raw) > self.policy.cluster_max_points:
                raise ValueError("cloud_exceeds_cluster_bound")
            for point in raw:
                if not isinstance(point, PointObservation):
                    raise ValueError("malformed_point")
                if not all(_finite(value) for value in (point.x, point.y, point.z)):
                    raise ValueError("nonfinite_point")
            if ego_linear_speed_mps is None:
                ego_linear_speed_mps = linear_speed_mps
            if ego_angular_speed_rps is None:
                ego_angular_speed_rps = angular_speed_rps
            if not all(
                _finite(value)
                for value in (
                    linear_speed_mps,
                    angular_speed_rps,
                    ego_linear_speed_mps,
                    ego_angular_speed_rps,
                )
            ):
                raise ValueError("nonfinite_coverage_motion")
            try:
                origin = tuple(sensor_origin)
            except TypeError as exc:
                raise ValueError("malformed_sensor_origin") from exc
            if len(origin) != 3 or not all(_finite(value) for value in origin):
                raise ValueError("malformed_sensor_origin")
            cells = self._visibility_cells(raw, origin)
            if self._coverage_frames:
                previous_stamp, _, _, _ = self._coverage_frames[-1]
                reset = (
                    stamp_s <= previous_stamp
                    or stamp_s - previous_stamp > self.policy.coverage_max_frame_gap_s
                )
                if reset:
                    self._coverage_frames.clear()
            self._coverage_frames.append(
                (stamp_s, linear_speed_mps, angular_speed_rps, frozenset(cells))
            )
            # Simulation-only self-return mask. Hardware use requires measured self
            # geometry rather than assuming the policy footprint is the sensor outline.
            half_length = self.policy.footprint_length_m / 2.0
            half_width = self.policy.footprint_width_m / 2.0
            nonground = tuple(
                point for point in raw
                if not self.policy.ground_min_z_m <= point.z <= self.policy.ground_max_z_m
                and not (-half_length <= point.x <= half_length
                         and -half_width <= point.y <= half_width)
            )
            voxels = self._voxelize(nonground)
            clusters = self._cluster(voxels)
            observations = self._associate(
                clusters,
                stamp_s,
                ego_linear_speed_mps=ego_linear_speed_mps,
                ego_angular_speed_rps=ego_angular_speed_rps,
            )
            expected_cells = self._required_coverage_cells(linear_speed_mps, angular_speed_rps)
            coverage_frames = self._coverage_suffix()
            observed_cells = (
                set.intersection(*(set(frame[3]) for frame in coverage_frames))
                if coverage_frames else set()
            )
            observed = len(expected_cells & observed_cells)
            expected = len(expected_cells)
            fraction = observed / float(expected) if expected and coverage_frames else 0.0
            if not coverage_frames:
                observed = 0
            return CloudProcessingResult(
                observations, len(raw), expected, observed, fraction, True, "ok",
            )
        except (AttributeError, OverflowError, TypeError, ValueError) as exc:
            # Processing is transactional: malformed input neither emits a partial cloud
            # nor mutates temporal tracks or coverage evidence.
            self._tracks = prior_tracks
            self._next_track_id = prior_next_track_id
            self._coverage_frames = prior_coverage_frames
            return CloudProcessingResult(
                (), 0, self.policy.coverage_bins * self.policy.coverage_elevation_bins,
                0, 0.0, False, str(exc),
            )

    def _coverage_suffix(self) -> Tuple[Tuple[float, float, float, frozenset], ...]:
        """Select the shortest latest frame suffix proving the minimum duration."""
        frames = tuple(self._coverage_frames)
        minimum = self.policy.coverage_min_frames
        if len(frames) < minimum:
            return ()
        latest_start = len(frames) - minimum
        for start in range(latest_start, -1, -1):
            candidate = frames[start:]
            if (
                candidate[-1][0] - candidate[0][0]
                >= self.policy.coverage_window_s - 1e-12
            ):
                return candidate
        return ()
    def _visibility_cells(self, points: Sequence[PointObservation],
                          sensor_origin: Sequence[float]) -> set:
        cells = set()
        elevation_span = (
            self.policy.coverage_max_elevation_rad - self.policy.coverage_min_elevation_rad
        )
        for point in points:
            relative_x = point.x - sensor_origin[0]
            relative_y = point.y - sensor_origin[1]
            relative_z = point.z - sensor_origin[2]
            azimuth = math.atan2(relative_y, relative_x)
            elevation = math.atan2(relative_z, math.hypot(relative_x, relative_y))
            if (relative_x >= 0.0 and abs(azimuth) <= math.pi / 2.0
                    and self.policy.coverage_min_elevation_rad <= elevation
                    <= self.policy.coverage_max_elevation_rad):
                azimuth_index = int(
                    (azimuth + math.pi / 2.0) / math.pi * self.policy.coverage_bins
                )
                elevation_index = int(
                    (elevation - self.policy.coverage_min_elevation_rad)
                    / elevation_span * self.policy.coverage_elevation_bins
                )
                cells.add((
                    min(self.policy.coverage_bins - 1, max(0, azimuth_index)),
                    min(self.policy.coverage_elevation_bins - 1, max(0, elevation_index)),
                ))
        return cells

    def _required_coverage_cells(self, linear: float, angular: float) -> set:
        expansion = (
            self.policy.footprint_width_m / 2.0
            + self.policy.localization_uncertainty_m
            + self.policy.transform_uncertainty_m
            + self.policy.point_noise_m
            + self.policy.fixed_expansion_m
        )
        horizon = self.policy.max_horizon_s
        samples = max(1, int(math.ceil(horizon / 0.02)))
        required_azimuth = set()
        for index in range(samples + 1):
            t = horizon * index / samples
            if abs(angular) < 1e-9:
                x, y = linear * t, 0.0
            else:
                yaw = angular * t
                radius = linear / angular
                x, y = radius * math.sin(yaw), radius * (1.0 - math.cos(yaw))
            distance = max(math.hypot(x, y), expansion)
            center = math.atan2(y, x) if x > 0.0 else 0.0
            half_angle = math.atan2(expansion, distance)
            low = max(-math.pi / 2.0, center - half_angle)
            high = min(math.pi / 2.0, center + half_angle)
            first = int((low + math.pi / 2.0) / math.pi * self.policy.coverage_bins)
            last = int((high + math.pi / 2.0) / math.pi * self.policy.coverage_bins)
            required_azimuth.update(range(max(0, first), min(self.policy.coverage_bins - 1, last) + 1))
        return {
            (azimuth, elevation)
            for azimuth in required_azimuth
            for elevation in range(self.policy.coverage_elevation_bins)
        }

    def _voxelize(self, points: Sequence[PointObservation]) -> Tuple[PointObservation, ...]:
        size = self.policy.voxel_size_m
        selected: Dict[Tuple[int, int, int], PointObservation] = {}
        for point in points:
            key = (math.floor(point.x / size), math.floor(point.y / size), math.floor(point.z / size))
            rank = (point.x * point.x + point.y * point.y + point.z * point.z,
                    point.x, point.y, point.z)
            current = selected.get(key)
            if current is None or rank < (
                    current.x * current.x + current.y * current.y + current.z * current.z,
                    current.x, current.y, current.z):
                selected[key] = point
        return tuple(selected[key] for key in sorted(selected))

    def _cluster(self, points: Sequence[PointObservation],
                 candidate_check_counter: Optional[List[int]] = None
                 ) -> Tuple[Tuple[float, float, float], ...]:
        if len(points) > self.policy.cluster_max_points:
            raise ValueError("cloud_exceeds_cluster_bound")
        tolerance = self.policy.cluster_tolerance_m
        tolerance_sq = tolerance ** 2
        buckets: Dict[Tuple[int, int, int], List[int]] = {}
        for index, point in enumerate(points):
            key = (
                math.floor(point.x / tolerance),
                math.floor(point.y / tolerance),
                math.floor(point.z / tolerance),
            )
            buckets.setdefault(key, []).append(index)

        remaining = set(range(len(points)))
        clusters = []
        while remaining:
            seed = min(remaining)
            remaining.remove(seed)
            pending, members = deque((seed,)), []
            while pending:
                index = pending.popleft()
                members.append(index)
                point = points[index]
                key = (
                    math.floor(point.x / tolerance),
                    math.floor(point.y / tolerance),
                    math.floor(point.z / tolerance),
                )
                candidates = []
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            candidates.extend(buckets.get(
                                (key[0] + dx, key[1] + dy, key[2] + dz), ()))
                neighbors = []
                for other in sorted(candidate for candidate in candidates if candidate in remaining):
                    if candidate_check_counter is not None:
                        candidate_check_counter[0] += 1
                    if ((point.x - points[other].x) ** 2
                            + (point.y - points[other].y) ** 2
                            + (point.z - points[other].z) ** 2) <= tolerance_sq:
                        neighbors.append(other)
                for other in neighbors:
                    remaining.remove(other)
                    pending.append(other)
            if len(members) >= self.policy.cluster_min_points:
                count = float(len(members))
                clusters.append((
                    sum(points[i].x for i in members) / count,
                    sum(points[i].y for i in members) / count,
                    sum(points[i].z for i in members) / count,
                ))
        return tuple(sorted(clusters))

    def _associate(self, clusters: Sequence[Tuple[float, float, float]],
                   stamp_s: float,
                   candidate_check_counter: Optional[List[int]] = None,
                   ego_linear_speed_mps: float = 0.0,
                   ego_angular_speed_rps: float = 0.0,
                   ) -> Tuple[PointObservation, ...]:
        active = {
            track_id: track for track_id, track in self._tracks.items()
            if 0.0 < stamp_s - track.stamp_s <= self.policy.association_max_age_s
        }
        if not _finite(ego_linear_speed_mps) or not _finite(ego_angular_speed_rps):
            raise ValueError("nonfinite_ego_motion")
        predicted = {}
        for track_id, track in active.items():
            dt = stamp_s - track.stamp_s
            yaw_delta = ego_angular_speed_rps * dt
            if abs(ego_angular_speed_rps) < 1e-9:
                translation_x = ego_linear_speed_mps * dt
                translation_y = 0.0
            else:
                radius = ego_linear_speed_mps / ego_angular_speed_rps
                translation_x = radius * math.sin(yaw_delta)
                translation_y = radius * (1.0 - math.cos(yaw_delta))
            relative_x = track.x - translation_x
            relative_y = track.y - translation_y
            cosine = math.cos(yaw_delta)
            sine = math.sin(yaw_delta)
            predicted[track_id] = (
                cosine * relative_x + sine * relative_y,
                -sine * relative_x + cosine * relative_y,
                track.z,
            )
        displacement_limit = self.policy.association_max_displacement_m
        displacement_limit_sq = displacement_limit ** 2
        track_buckets: Dict[Tuple[int, int, int], List[int]] = {}
        for track_id, track in active.items():
            key = (
                math.floor(predicted[track_id][0] / displacement_limit),
                math.floor(predicted[track_id][1] / displacement_limit),
                math.floor(predicted[track_id][2] / displacement_limit),
            )
            track_buckets.setdefault(key, []).append(track_id)

        candidates = []
        for cluster_index, (x, y, z) in enumerate(clusters):
            key = (
                math.floor(x / displacement_limit),
                math.floor(y / displacement_limit),
                math.floor(z / displacement_limit),
            )
            nearby_tracks = []
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        nearby_tracks.extend(track_buckets.get(
                            (key[0] + dx, key[1] + dy, key[2] + dz), ()))
            for track_id in nearby_tracks:
                if candidate_check_counter is not None:
                    candidate_check_counter[0] += 1
                predicted_x, predicted_y, predicted_z = predicted[track_id]
                displacement_sq = (
                    (x - predicted_x) ** 2
                    + (y - predicted_y) ** 2
                    + (z - predicted_z) ** 2
                )
                if displacement_sq <= displacement_limit_sq:
                    candidates.append((math.sqrt(displacement_sq), track_id, cluster_index))

        by_track: Dict[int, List[Tuple[float, int, int]]] = {}
        by_cluster: Dict[int, List[Tuple[float, int, int]]] = {}
        for candidate in candidates:
            by_track.setdefault(candidate[1], []).append(candidate)
            by_cluster.setdefault(candidate[2], []).append(candidate)
        ambiguous = set()
        for alternatives in list(by_track.values()) + list(by_cluster.values()):
            ordered = sorted(alternatives)
            for first, second in zip(ordered, ordered[1:]):
                if abs(first[0] - second[0]) <= 1e-12:
                    ambiguous.add(first)
                    ambiguous.add(second)

        assigned_tracks, assigned_clusters, associations = set(), set(), {}
        for candidate in sorted(candidates):
            displacement, track_id, cluster_index = candidate
            if track_id in assigned_tracks or cluster_index in assigned_clusters:
                continue
            # Equal-distance alternatives are ambiguous and deliberately left untracked.
            if candidate in ambiguous:
                continue
            assigned_tracks.add(track_id)
            assigned_clusters.add(cluster_index)
            associations[cluster_index] = track_id

        updated: Dict[int, _Track] = {}
        observations = []
        velocity_history_limit = max(2, self.policy.minimum_observations)
        for index, (x, y, z) in enumerate(clusters):
            track_id = associations.get(index)
            if track_id is None:
                track_id = self._next_track_id
                self._next_track_id += 1
                velocity_history: List[Tuple[float, float]] = []
                observation_count = 1
            else:
                previous = active[track_id]
                dt = stamp_s - previous.stamp_s
                if dt <= 0.0:
                    raise ValueError("nonpositive_track_interval")
                predicted_x, predicted_y, _ = predicted[track_id]
                residual_velocity = (
                    (x - predicted_x) / dt,
                    (y - predicted_y) / dt,
                )
                if not all(_finite(value) for value in residual_velocity):
                    raise ValueError("nonfinite_compensated_velocity")
                velocity_history = (
                    previous.compensated_velocities + [residual_velocity]
                )[-velocity_history_limit:]
                observation_count = min(previous.observation_count + 1, 0xFFFFFFFF)
            track = _Track(
                track_id,
                stamp_s,
                x,
                y,
                z,
                velocity_history,
                observation_count,
            )
            updated[track_id] = track
            vx = vy = None
            covariance_valid = False
            if velocity_history:
                mean_vx = sum(value[0] for value in velocity_history) / len(velocity_history)
                mean_vy = sum(value[1] for value in velocity_history) / len(velocity_history)
                if _finite(mean_vx) and _finite(mean_vy):
                    vx = mean_vx
                    vy = mean_vy
                    if observation_count >= self.policy.minimum_observations:
                        variance = sum(
                            (value[0] - mean_vx) ** 2
                            + (value[1] - mean_vy) ** 2
                            for value in velocity_history
                        ) / len(velocity_history)
                        covariance_valid = (
                            _finite(variance)
                            and variance <= self.policy.dynamic_covariance_max_mps2
                        )
            observations.append(PointObservation(
                x,
                y,
                z,
                vx,
                vy,
                str(track_id),
                observation_count,
                covariance_valid,
            ))
        self._tracks = updated
        return tuple(observations)


def transform_cloud_points(
        points: Iterable[PointObservation],
        translation: Sequence[float],
        rotation_xyzw: Sequence[float],
) -> Tuple[PointObservation, ...]:
    """Apply a rigid transform without depending on ROS or relabeling coordinates."""
    try:
        tx, ty, tz = (float(value) for value in translation)
        qx, qy, qz, qw = (float(value) for value in rotation_xyzw)
    except (TypeError, ValueError) as exc:
        raise ValueError("malformed_transform") from exc
    values = (tx, ty, tz, qx, qy, qz, qw)
    if not all(_finite(value) for value in values):
        raise ValueError("nonfinite_transform")
    norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
    if not _finite(norm_sq) or norm_sq <= 1e-12 or abs(norm_sq - 1.0) > 1e-3:
        raise ValueError("invalid_rotation")
    # Unit-quaternion rotation matrix.
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)
    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    transformed = []
    try:
        for point in points:
            coordinates = (point.x, point.y, point.z)
            if not all(_finite(value) for value in coordinates):
                raise ValueError("nonfinite_point")
            vx, vy = point.vx, point.vy
            if (vx is None) != (vy is None):
                raise ValueError("incomplete_velocity")
            if vx is not None and not all(_finite(value) for value in (vx, vy)):
                raise ValueError("nonfinite_velocity")
            transformed.append(PointObservation(
                r00 * point.x + r01 * point.y + r02 * point.z + tx,
                r10 * point.x + r11 * point.y + r12 * point.z + ty,
                r20 * point.x + r21 * point.y + r22 * point.z + tz,
                None if vx is None else r00 * vx + r01 * vy,
                None if vy is None else r10 * vx + r11 * vy,
                point.track_id,
                point.observation_count,
                point.covariance_valid,
            ))
    except (AttributeError, TypeError) as exc:
        raise ValueError("malformed_point") from exc
    return tuple(transformed)


def prepare_transformed_cloud(
        points: Iterable[PointObservation],
        source_frame: str,
        target_frame: str,
        cloud_stamp_s: float,
        transform_stamp_s: float,
        translation: Sequence[float],
        rotation_xyzw: Sequence[float],
        max_transform_age_s: float,
        returned_source_frame: Optional[str] = None,
        returned_target_frame: Optional[str] = None,
        static_transform: bool = False,
) -> CloudTransformResult:
    """Validate transform provenance/timing and route all failures fail-closed."""
    try:
        if source_frame != "lidar_link" or target_frame != "base_footprint":
            raise ValueError("frame_mismatch")
        if returned_source_frame is not None and returned_source_frame != source_frame:
            raise ValueError("frame_mismatch")
        if returned_target_frame is not None and returned_target_frame != target_frame:
            raise ValueError("frame_mismatch")
        if not isinstance(static_transform, bool):
            raise ValueError("invalid_transform_timestamp")
        timing = (cloud_stamp_s, transform_stamp_s, max_transform_age_s)
        if not all(_finite(value) and value >= 0.0 for value in timing):
            raise ValueError("invalid_transform_timestamp")
        if static_transform:
            if transform_stamp_s != 0.0:
                raise ValueError("invalid_transform_timestamp")
            transform_age = 0.0
        else:
            transform_age = cloud_stamp_s - transform_stamp_s
            if transform_age < 0.0:
                raise ValueError("future_transform")
            if transform_age > max_transform_age_s:
                raise ValueError("stale_transform")
        transformed = transform_cloud_points(points, translation, rotation_xyzw)
        return CloudTransformResult(transformed, target_frame, transform_age, True, "ok")
    except (TypeError, ValueError) as exc:
        return CloudTransformResult((), target_frame, -1.0, False, str(exc))




@dataclass(frozen=True)
class CollisionInputs:
    now_s: float
    sequence: int
    cloud_stamp_s: float
    odom_stamp_s: float
    nav_stamp_s: float
    safe_stamp_s: float
    points: Sequence[PointObservation]
    odom_linear_mps: float
    nav_linear_mps: float
    safe_linear_mps: float
    frame_id: str = "base_footprint"
    pitch_downhill_rad: float = 0.0
    coverage_fraction: float = 1.0
    expected_coverage_bins: int = 1
    observed_coverage_bins: int = 1
    rear_coverage_qualified: bool = False
    occluded: bool = False
    transform_ok: bool = True
    transform_age_s: float = 0.0
    policy_id: Optional[str] = None
    policy_sha256: Optional[str] = None
    raw_point_count: Optional[int] = None
    intent_stamp_s: Optional[float] = None
    intent_behavior: int = INTENT_PROCEED
    intent_max_linear_mps: float = 1.0
    intent_max_angular_rps: float = 1.0
    nav_available: bool = True
    odom_angular_rps: float = 0.0
    nav_angular_rps: float = 0.0
    safe_angular_rps: float = 0.0
    odom_receipt_s: Optional[float] = None
    slope_stamp_s: Optional[float] = None
    slope_receipt_s: Optional[float] = None
    slope_valid: bool = True


@dataclass(frozen=True)
class CollisionDecision:
    sequence: int
    state: int
    visibility: int
    obstacle_motion: int
    reason_mask: int
    source: str
    policy_id: str
    policy_sha256: str
    evaluation_stamp: float
    input_age_s: float
    transform_age_s: float
    odom_age_s: float
    command_age_s: float
    coverage_fraction: float
    forward_speed_mps: float
    angular_speed_rps: float
    closing_speed_mps: float
    nearest_x_m: float
    nearest_y_m: float
    nearest_distance_m: float
    time_to_collision_s: float
    reaction_distance_m: float
    braking_distance_m: float
    uncertainty_margin_m: float
    required_stop_distance_m: float
    clear_distance_m: float
    recommended_max_linear_mps: float
    obstacle_point_count: int
    consecutive_clear_frames: int
    signal_state: int
    reason: str

class CollisionWatchdogState:
    """ROS-independent lidar deadline state for fail-closed heartbeat publication."""

    def __init__(self, cloud_ttl_s: float):
        if not _finite(cloud_ttl_s) or cloud_ttl_s <= 0.0:
            raise ValueError("cloud TTL must be finite and positive")
        self.cloud_ttl_s = float(cloud_ttl_s)
        self.last_cloud_stamp_s: Optional[float] = None

    def observe_cloud(self, cloud_stamp_s: float, now_s: float) -> bool:
        if (_finite(cloud_stamp_s) and _finite(now_s)
                and 0.0 <= cloud_stamp_s <= now_s):
            self.last_cloud_stamp_s = float(cloud_stamp_s)
            return True
        return False

    def stale_age(self, now_s: float) -> Optional[float]:
        if not _finite(now_s) or now_s < 0.0:
            raise ValueError("watchdog time must be finite and nonnegative")
        if self.last_cloud_stamp_s is None:
            return float(now_s)
        age = float(now_s) - self.last_cloud_stamp_s
        if age < 0.0:
            return float(now_s)
        return age if age >= self.cloud_ttl_s else None



class CollisionSupervisorCore:
    """Stateful collision classifier and fail-closed stop/clear authority evidence."""

    def __init__(self, policy: CollisionPolicy):
        self.policy = policy
        self._last_sequence: Optional[int] = None
        self._stop_latched = True
        self._clear_frames = 0
        self._clear_since: Optional[float] = None
        self._last_intent_behavior: Optional[int] = None
        self._activation_started_s: Optional[float] = None
        self._activation_command_floor_s: Optional[float] = None
    def stale_cloud_decision(self, sequence: int, now_s: float,
                             cloud_age_s: float) -> CollisionDecision:
        """Latch and describe a current canonical STOP without inventing cloud evidence."""
        if (not isinstance(sequence, int) or isinstance(sequence, bool)
                or not 0 <= sequence <= 0xFFFFFFFF
                or (self._last_sequence is not None and sequence <= self._last_sequence)):
            raise ValueError("watchdog sequence must be monotonic uint32")
        if (not _finite(now_s) or now_s < 0.0
                or not _finite(cloud_age_s) or cloud_age_s < 0.0):
            raise ValueError("watchdog timestamps must be finite and nonnegative")
        self._last_sequence = sequence
        self._stop_latched = True
        self._clear_frames = 0
        self._clear_since = None
        reason_mask = LIDAR_STALE | SENSOR_STALE | COLLISION
        return CollisionDecision(
            sequence, STOP, VIS_BLIND, MOTION_NONE, reason_mask,
            "collision_supervisor", self.policy.policy_id, self.policy.policy_sha256,
            float(now_s), float(cloud_age_s), -1.0, -1.0, -1.0, 0.0,
            -1.0, -1.0, 0.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
            self.policy.fixed_uncertainty_margin_m, -1.0, -1.0, 0.0, 0, 0,
            SIGNAL_STOP, "stale_cloud",
        )


    def evaluate(self, inputs: CollisionInputs) -> CollisionDecision:
        invalid_reason = self._validate_inputs(inputs)
        ages = self._ages(inputs)
        coverage = self._coverage(inputs)
        speed = self._selected_speed(inputs)
        # Simulation-only stationary gate, aligned with localization: drift below
        # 1 cm/s travels < the existing 0.20 m uncertainty over the 4 s horizon.
        # This normalization does not authorize hardware operation.
        if _finite(speed) and abs(speed) < 0.01:
            speed = 0.0
        angular = self._selected_angular(inputs)
        reaction = braking = required = -1.0
        a_eff = self.policy.simulation_min_deceleration_mps2 - self.policy.gravity_mps2 * math.sin(
            max(0.0, -inputs.pitch_downhill_rad)
        ) if _finite(inputs.pitch_downhill_rad) else -math.inf
        if _finite(speed) and a_eff > 0.0 and all(_finite(age) and age >= 0.0 for age in ages):
            reaction = abs(speed) * (ages[0] + self.policy.simulation_driver_latency_s + self.policy.compute_and_gate_budget_s)
            braking = speed * speed / (2.0 * a_eff)
            required = reaction + braking + self.policy.fixed_uncertainty_margin_m

        visibility = VIS_FULL if coverage >= self.policy.required_forward_coverage_fraction else VIS_PARTIAL
        raw_state, mask, reason = CLEAR, 0, "clear"
        motion, closing, nearest_x, nearest_y, nearest, ttc = MOTION_NONE, 0.0, -1.0, -1.0, -1.0, -1.0

        if invalid_reason:
            raw_state, visibility, mask, reason = STOP, VIS_UNKNOWN, invalid_reason[0], invalid_reason[1]
        elif a_eff <= 0.0:
            raw_state, mask, reason = STOP, COLLISION | COLLISION_DISTANCE, "nonpositive_effective_deceleration"
        elif speed < 0.0 and not inputs.rear_coverage_qualified:
            raw_state, mask, reason = STOP, COLLISION | COLLISION_BLIND, "reverse_coverage_not_qualified"
            visibility = VIS_BLIND
        elif (inputs.raw_point_count if inputs.raw_point_count is not None else len(inputs.points)) == 0:
            raw_state, visibility, mask, reason = STOP, VIS_BLIND, COLLISION | COLLISION_BLIND, "empty_cloud_blind"
        elif coverage < self.policy.required_forward_coverage_fraction or inputs.observed_coverage_bins < inputs.expected_coverage_bins:
            raw_state, visibility, mask, reason = STOP, VIS_BLIND, COLLISION | COLLISION_BLIND, "insufficient_coverage"
        elif not inputs.points:
            # Visibility is computed before the frozen ground-only filter.
            nearest = math.inf
        else:
            hold = (
                inputs.intent_behavior == INTENT_HOLD
                and inputs.intent_max_linear_mps == 0.0
                and inputs.intent_max_angular_rps == 0.0
            )
            (
                motion,
                closing,
                nearest_x,
                nearest_y,
                nearest,
                ttc,
                intersects,
                ambiguous,
                ambiguous_ttc,
                ambiguous_intersects,
            ) = self._assess_obstacles(
                inputs.points, speed, angular, required, a_eff, hold
            )
            occluded = inputs.occluded or (coverage < 1.0 and nearest < required)
            if occluded:
                raw_state, visibility, mask, reason = STOP, VIS_PARTIAL, COLLISION | COLLISION_OCCLUDED, "path_occluded"
            else:
                dynamic_limit = ages[0] + self.policy.simulation_driver_latency_s + self.policy.compute_and_gate_budget_s + abs(speed) / a_eff + self.policy.stop_ttc_margin_s
                if (
                    ambiguous
                    and not hold
                    and (
                        ambiguous_intersects
                        or (
                            ambiguous_ttc >= 0.0
                            and ambiguous_ttc <= dynamic_limit
                        )
                    )
                ):
                    raw_state, mask, reason = STOP, COLLISION | COLLISION_TTC, "ambiguous_obstacle"
                elif intersects or (motion == MOTION_STATIC and nearest <= required):
                    raw_state, mask, reason = STOP, COLLISION | COLLISION_DISTANCE, "collision_distance"
                elif motion == MOTION_DYNAMIC and ttc >= 0.0 and ttc <= dynamic_limit:
                    raw_state, mask, reason = STOP, COLLISION | COLLISION_TTC, "collision_ttc"
                elif (
                    (ambiguous and not hold)
                    or (ttc >= 0.0 and ttc <= self.policy.caution_ttc_s)
                    or nearest <= required + self.policy.caution_clearance_margin_m
                ):
                    raw_state = CAUTION
                    reason = "ambiguous_caution" if ambiguous and not hold else "caution"

        clear_distance = (required + self.policy.clearance_strictly_above_margin_m
                          if required >= 0.0 else -1.0)
        strict_clear = (raw_state == CLEAR and nearest > clear_distance
                        and (ttc < 0.0 or ttc > self.policy.ttc_strictly_above_s))
        state = self._apply_hysteresis(raw_state, strict_clear, inputs.now_s)
        if state == STOP and raw_state != STOP:
            mask, reason = COLLISION, "clear_hysteresis"
        signal = SIGNAL_CLEAR if state in (CLEAR, CAUTION) and mask == 0 else SIGNAL_STOP
        recommended = self.policy.caution_max_linear_mps if state == CAUTION else (-1.0 if state == CLEAR else 0.0)
        if invalid_reason is None:
            self._last_sequence = inputs.sequence
        output_ages = tuple(age if _finite(age) and age >= 0.0 else -1.0 for age in ages)
        output_speed = speed if _finite(speed) else -1.0
        output_angular = angular if _finite(angular) else -1.0
        output_coverage = coverage if _finite(coverage) else -1.0
        output_stamp = inputs.now_s if _finite(inputs.now_s) and inputs.now_s >= 0.0 else -1.0
        output_sequence = (inputs.sequence if isinstance(inputs.sequence, int)
                           and not isinstance(inputs.sequence, bool)
                           and 0 <= inputs.sequence <= 0xFFFFFFFF else 0)
        return CollisionDecision(
            output_sequence, state, visibility, motion, mask, "collision_supervisor", self.policy.policy_id,
            self.policy.policy_sha256, output_stamp, output_ages[0], (
                inputs.transform_age_s if _finite(inputs.transform_age_s) and inputs.transform_age_s >= 0.0 else -1.0
            ), output_ages[1],
            output_ages[2], output_coverage, output_speed, output_angular, closing, nearest_x, nearest_y,
            (-1.0 if math.isinf(nearest) else nearest), ttc, reaction, braking,
            self.policy.fixed_uncertainty_margin_m, required,
            clear_distance, recommended, len(inputs.points), self._clear_frames, signal, reason,
        )

    def _validate_inputs(self, i: CollisionInputs) -> Optional[Tuple[int, str]]:
        if ((i.policy_id is not None and i.policy_id != self.policy.policy_id)
                or (i.policy_sha256 is not None and i.policy_sha256 != self.policy.policy_sha256)):
            return POLICY_MISMATCH | COLLISION, "policy_mismatch"
        if (not isinstance(i.sequence, int) or isinstance(i.sequence, bool) or not 0 <= i.sequence <= 0xFFFFFFFF
                or (self._last_sequence is not None and i.sequence <= self._last_sequence)):
            return CORRUPT_DATA | COLLISION, "sequence_nonmonotonic"
        if i.frame_id != self.policy.evaluation_frame:
            return TF | COLLISION_BLIND | COLLISION, "wrong_evaluation_frame"
        if not i.transform_ok:
            return TF | COLLISION_BLIND | COLLISION, "transform_failure"
        intent_stamp = i.nav_stamp_s if i.intent_stamp_s is None else i.intent_stamp_s
        if (not isinstance(i.intent_behavior, int) or isinstance(i.intent_behavior, bool)
                or i.intent_behavior not in (INTENT_HOLD, INTENT_PROCEED, INTENT_SLOW, INTENT_STOP)
                or not all(_finite(value) for value in
                           (intent_stamp, i.intent_max_linear_mps, i.intent_max_angular_rps))
                or i.intent_max_linear_mps < 0.0 or i.intent_max_angular_rps < 0.0):
            self._last_intent_behavior = None
            self._activation_started_s = None
            self._activation_command_floor_s = None
            return INPUT_UNKNOWN | COLLISION, "malformed_intent"
        scalar = (i.now_s, i.cloud_stamp_s, i.odom_stamp_s, i.safe_stamp_s, i.odom_linear_mps,
                  i.safe_linear_mps, i.odom_angular_rps, i.safe_angular_rps,
                  i.pitch_downhill_rad, i.coverage_fraction, i.transform_age_s, intent_stamp,
                  i.intent_max_linear_mps, i.intent_max_angular_rps)
        if i.nav_available:
            scalar += (i.nav_stamp_s, i.nav_linear_mps, i.nav_angular_rps)
        point_values = tuple(value for p in i.points for value in (p.x, p.y, p.z))
        optional_values = tuple(value for p in i.points for value in (p.vx, p.vy) if value is not None)
        if (not all(_finite(value) for value in scalar + point_values + optional_values)
                or i.transform_age_s < 0.0
                or not isinstance(i.expected_coverage_bins, int)
                or not isinstance(i.observed_coverage_bins, int)
                or isinstance(i.expected_coverage_bins, bool)
                or isinstance(i.observed_coverage_bins, bool)
                or any(not isinstance(p.observation_count, int) or p.observation_count < 1 for p in i.points)):
            return CORRUPT_DATA | COLLISION, "nonfinite_input"
        stamps = [i.cloud_stamp_s, i.odom_stamp_s, i.safe_stamp_s, intent_stamp]
        if i.nav_available:
            stamps.append(i.nav_stamp_s)
        if any(stamp < 0.0 or not _within_future_tolerance(stamp, i.now_s) for stamp in stamps):
            return CORRUPT_DATA | COLLISION, "invalid_timestamp"
        if i.now_s - intent_stamp > self.policy.intent_ttl_s:
            self._last_intent_behavior = None
            self._activation_started_s = None
            self._activation_command_floor_s = None
            return SENSOR_STALE | INPUT_UNKNOWN | COLLISION, "stale_intent"
        if i.intent_behavior == INTENT_STOP:
            self._last_intent_behavior = INTENT_STOP
            self._activation_started_s = None
            self._activation_command_floor_s = None
            return INPUT_UNKNOWN | COLLISION, "stop_intent"
        if i.intent_behavior == INTENT_HOLD:
            if i.intent_max_linear_mps != 0.0 or i.intent_max_angular_rps != 0.0:
                self._last_intent_behavior = None
                self._activation_started_s = None
                self._activation_command_floor_s = None
                return INPUT_UNKNOWN | COLLISION, "malformed_hold_intent"
            self._activation_started_s = None
            self._activation_command_floor_s = None
        else:
            if self._last_intent_behavior == INTENT_HOLD:
                self._activation_started_s = i.now_s
                self._activation_command_floor_s = intent_stamp
            fresh_after_hold = (self._activation_command_floor_s is None
                                or (i.nav_available and i.nav_stamp_s > self._activation_command_floor_s))
            grace = (self._activation_started_s is not None
                     and i.now_s - self._activation_started_s
                     <= self.policy.first_command_grace_s + 1e-12)
            if not i.nav_available and not grace:
                return SENSOR_STALE | COLLISION, "missing_nav_command"
            if i.nav_available and not fresh_after_hold and not grace:
                return SENSOR_STALE | COLLISION, "pre_hold_nav_command"
        cloud_age, odom_age, command_age = self._ages(i)
        if cloud_age > self.policy.cloud_ttl_s:
            return LIDAR_STALE | SENSOR_STALE | COLLISION, "stale_cloud"
        if odom_age > self.policy.odom_ttl_s:
            return ODOM_STALE | SENSOR_STALE | COLLISION, "stale_odom"
        if command_age > self.policy.command_ttl_s:
            return SENSOR_STALE | COLLISION, "stale_command"
        raw_count = len(i.points) if i.raw_point_count is None else i.raw_point_count
        if (not isinstance(raw_count, int) or isinstance(raw_count, bool) or raw_count < len(i.points)):
            return CORRUPT_DATA | COLLISION, "invalid_raw_point_count"
        if i.expected_coverage_bins <= 0 or not 0.0 <= i.coverage_fraction <= 1.0:
            return CORRUPT_DATA | COLLISION, "invalid_coverage"
        if not i.slope_valid:
            return INPUT_UNKNOWN | COLLISION, "invalid_slope_evidence"
        odom_receipt = i.odom_stamp_s if i.odom_receipt_s is None else i.odom_receipt_s
        slope_stamp = i.cloud_stamp_s if i.slope_stamp_s is None else i.slope_stamp_s
        slope_receipt = i.now_s if i.slope_receipt_s is None else i.slope_receipt_s
        if (not _finite(odom_receipt)
                or odom_receipt < 0.0
                or not _within_future_tolerance(i.odom_stamp_s, odom_receipt)
                or not _within_future_tolerance(odom_receipt, i.now_s)
                or not _slope_timing_valid(
                    slope_stamp, slope_receipt, i.cloud_stamp_s, i.now_s
                )):
            return SENSOR_STALE | INPUT_UNKNOWN | COLLISION, "stale_or_mismatched_slope_odom"
        if (_direction_disagreement(
                (i.odom_linear_mps, i.safe_linear_mps) +
                ((i.nav_linear_mps,) if i.nav_available else ()),
                self.policy.coverage_motion_linear_tolerance_mps)
                or _direction_disagreement(
                    (i.odom_angular_rps, i.safe_angular_rps) +
                    ((i.nav_angular_rps,) if i.nav_available else ()),
                    self.policy.coverage_motion_angular_tolerance_rps)):
            return CORRUPT_DATA | COLLISION, "motion_direction_disagreement"
        self._last_intent_behavior = i.intent_behavior
        if i.intent_behavior != INTENT_HOLD and i.nav_available:
            self._activation_started_s = None
            self._activation_command_floor_s = None
        return None

    @staticmethod
    def _ages(i: CollisionInputs) -> Tuple[float, float, float]:
        intent_stamp = i.nav_stamp_s if i.intent_stamp_s is None else i.intent_stamp_s
        if not all(_finite(value) for value in
                   (i.now_s, i.cloud_stamp_s, i.odom_stamp_s, i.safe_stamp_s, intent_stamp)):
            return -1.0, -1.0, -1.0
        command_age = _age_within_future_tolerance(i.now_s, i.safe_stamp_s)
        if i.intent_behavior != INTENT_HOLD and i.nav_available:
            command_age = max(
                command_age, _age_within_future_tolerance(i.now_s, i.nav_stamp_s)
            )
        odom_receipt = i.odom_stamp_s if i.odom_receipt_s is None else i.odom_receipt_s
        return (
            _age_within_future_tolerance(i.now_s, i.cloud_stamp_s),
            max(
                _age_within_future_tolerance(i.now_s, i.odom_stamp_s),
                _age_within_future_tolerance(i.now_s, odom_receipt),
            ),
            command_age,
        )

    @staticmethod
    def _coverage(i: CollisionInputs) -> float:
        if (not isinstance(i.expected_coverage_bins, int)
                or not isinstance(i.observed_coverage_bins, int)
                or i.expected_coverage_bins <= 0
                or not _finite(i.coverage_fraction)):
            return -1.0
        return min(i.coverage_fraction, max(0.0, i.observed_coverage_bins / float(i.expected_coverage_bins)))

    def _selected_speed(self, i: CollisionInputs) -> float:
        values = (i.odom_linear_mps, i.safe_linear_mps)
        if not all(_finite(value) for value in values):
            return math.nan
        if i.intent_behavior == INTENT_HOLD or not i.nav_available:
            return max(values, key=abs)
        if not _finite(i.nav_linear_mps):
            return math.nan
        values = values + (i.nav_linear_mps,)
        tolerance = self.policy.coverage_motion_linear_tolerance_mps
        if i.nav_linear_mps > tolerance:
            return max(IMMUTABLE_HARD_LINEAR_SPEED_MPS, *values)
        if i.nav_linear_mps < -tolerance:
            return min(-IMMUTABLE_HARD_LINEAR_SPEED_MPS, *values)
        return max(values, key=abs)
    @staticmethod
    def _selected_angular(i: CollisionInputs) -> float:
        values = (i.odom_angular_rps, i.safe_angular_rps)
        if not all(_finite(value) for value in values):
            return math.nan
        if i.intent_behavior == INTENT_HOLD or not i.nav_available:
            return max(values, key=abs)
        if not _finite(i.nav_angular_rps):
            return math.nan
        values = values + (i.nav_angular_rps,)
        if i.nav_angular_rps > 0.0:
            return max(0.0, *values)
        if i.nav_angular_rps < 0.0:
            return min(0.0, *values)
        return max(values, key=abs)

    def _assess_obstacles(
        self,
        points: Sequence[PointObservation],
        speed: float,
        angular: float,
        required: float,
        a_eff: float,
        hold: bool = False,
    ) -> Tuple[
        int,
        float,
        float,
        float,
        float,
        float,
        bool,
        bool,
        float,
        bool,
    ]:
        expansion = (
            self.policy.localization_uncertainty_m
            + self.policy.transform_uncertainty_m
            + self.policy.point_noise_m
            + self.policy.fixed_expansion_m
        )
        hx = self.policy.footprint_length_m / 2.0 + expansion
        hy = self.policy.footprint_width_m / 2.0 + expansion
        nearest_point = min(
            points,
            key=lambda point: math.hypot(
                max(0.0, abs(point.x) - hx),
                max(0.0, abs(point.y) - hy),
            ),
        )
        nearest = math.hypot(
            max(0.0, abs(nearest_point.x) - hx),
            max(0.0, abs(nearest_point.y) - hy),
        )
        best_ttc = math.inf
        best_closing = 0.0
        best_motion = MOTION_NONE
        relevant_ambiguous = False
        ambiguous_ttc = math.inf
        ambiguous_intersects = False
        intersects = False
        travel_time = min(
            self.policy.max_horizon_s,
            required / max(abs(speed), 1e-9),
        )

        for point in points:
            classified, classified_vx, classified_vy = self._classify(point)
            point_ambiguous = classified == MOTION_AMBIGUOUS
            point_ttc = None
            point_vx = classified_vx
            point_vy = classified_vy
            point_clearance = math.hypot(
                max(0.0, abs(point.x) - hx),
                max(0.0, abs(point.y) - hy),
            )

            if point_ambiguous:
                candidates: List[Tuple[float, float, float]] = []
                static_ttc = self._swept_ttc(
                    point,
                    0.0,
                    0.0,
                    speed,
                    angular,
                    hx,
                    hy,
                    self.policy.max_horizon_s,
                )
                if static_ttc is not None:
                    candidates.append((static_ttc, 0.0, 0.0))

                if point.vx is not None and point.vy is not None:
                    observed_vx = point.vx
                    observed_vy = point.vy
                    observed_speed = math.hypot(observed_vx, observed_vy)
                    if observed_speed > self.policy.ambiguous_approach_speed_mps:
                        scale = (
                            self.policy.ambiguous_approach_speed_mps
                            / observed_speed
                        )
                        observed_vx *= scale
                        observed_vy *= scale
                    observed_ttc = self._swept_ttc(
                        point,
                        observed_vx,
                        observed_vy,
                        speed,
                        angular,
                        hx,
                        hy,
                        self.policy.max_horizon_s,
                    )
                    if observed_ttc is not None:
                        candidates.append(
                            (observed_ttc, observed_vx, observed_vy)
                        )

                in_forward_corridor = (
                    point.x >= -hx
                    and abs(point.y)
                    <= hy + self.policy.caution_clearance_margin_m
                )
                if not hold and in_forward_corridor:
                    assumed_ttc = self._swept_ttc(
                        point,
                        classified_vx,
                        classified_vy,
                        speed,
                        angular,
                        hx,
                        hy,
                        self.policy.max_horizon_s,
                    )
                    if assumed_ttc is not None:
                        candidates.append(
                            (assumed_ttc, classified_vx, classified_vy)
                        )

                if candidates:
                    point_ttc, point_vx, point_vy = min(
                        candidates,
                        key=lambda candidate: candidate[0],
                    )
                point_relevant = (
                    in_forward_corridor
                    or point_ttc is not None
                    or point_clearance
                    <= required + self.policy.caution_clearance_margin_m
                )
                if not point_relevant:
                    continue
                relevant_ambiguous = True
                if point_ttc is not None:
                    ambiguous_ttc = min(ambiguous_ttc, point_ttc)
                    ambiguous_intersects = (
                        ambiguous_intersects
                        or point_ttc <= travel_time + 1e-12
                    )
            else:
                point_ttc = self._swept_ttc(
                    point,
                    point_vx,
                    point_vy,
                    speed,
                    angular,
                    hx,
                    hy,
                    self.policy.max_horizon_s,
                )

            rel_vx = point_vx - speed
            rel_vy = point_vy
            closing = max(
                0.0,
                -(point.x * rel_vx + point.y * rel_vy)
                / max(math.hypot(point.x, point.y), 1e-9),
            )
            if point_ttc is not None and point_ttc < best_ttc:
                best_ttc = point_ttc
                best_closing = closing
                best_motion = classified
            elif best_motion == MOTION_NONE or classified > best_motion:
                best_motion = classified
            intersects = (
                intersects
                or (
                    point_ttc is not None
                    and point_ttc <= travel_time + 1e-12
                )
            )

        return (
            best_motion,
            best_closing,
            nearest_point.x,
            nearest_point.y,
            nearest,
            -1.0 if math.isinf(best_ttc) else best_ttc,
            intersects,
            relevant_ambiguous,
            -1.0 if math.isinf(ambiguous_ttc) else ambiguous_ttc,
            ambiguous_intersects,
        )

    def _classify(self, point: PointObservation) -> Tuple[int, float, float]:
        if point.vx is None or point.vy is None or point.observation_count < self.policy.minimum_observations:
            radial = math.hypot(point.x, point.y)
            if radial == 0.0:
                return MOTION_AMBIGUOUS, 0.0, 0.0
            return (MOTION_AMBIGUOUS, -self.policy.ambiguous_approach_speed_mps * point.x / radial,
                    -self.policy.ambiguous_approach_speed_mps * point.y / radial)
        speed = math.hypot(point.vx, point.vy)
        if speed < self.policy.static_speed_below_mps:
            return MOTION_STATIC, point.vx, point.vy
        if point.covariance_valid:
            return MOTION_DYNAMIC, point.vx, point.vy
        radial = math.hypot(point.x, point.y)
        if radial == 0.0:
            return MOTION_AMBIGUOUS, 0.0, 0.0
        return (MOTION_AMBIGUOUS, -self.policy.ambiguous_approach_speed_mps * point.x / radial,
                -self.policy.ambiguous_approach_speed_mps * point.y / radial)

    @staticmethod
    def _swept_ttc(point: PointObservation, vx: float, vy: float, speed: float, angular: float,
                   hx: float, hy: float, horizon: float) -> Optional[float]:
        boundary_speed = abs(speed) + abs(angular) * math.hypot(hx, hy)
        step_s = min(0.02, 0.02 / max(boundary_speed, 1e-9))
        steps = max(1, int(math.ceil(horizon / step_s)))
        for index in range(steps + 1):
            t = horizon * index / steps
            if abs(angular) < 1e-9:
                cx, cy, yaw = speed * t, 0.0, 0.0
            else:
                yaw = angular * t
                radius = speed / angular
                cx, cy = radius * math.sin(yaw), radius * (1.0 - math.cos(yaw))
            px, py = point.x + vx * t - cx, point.y + vy * t - cy
            local_x = math.cos(yaw) * px + math.sin(yaw) * py
            local_y = -math.sin(yaw) * px + math.cos(yaw) * py
            if abs(local_x) <= hx and abs(local_y) <= hy:
                return t
        return None

    def _apply_hysteresis(self, raw_state: int, strict_clear: bool, now_s: float) -> int:
        if raw_state == STOP:
            self._stop_latched = True
            self._clear_frames = 0
            self._clear_since = None
            return STOP
        if not strict_clear:
            self._clear_frames = 0
            self._clear_since = None
            return STOP if self._stop_latched else raw_state
        if self._clear_since is None:
            self._clear_since = now_s
        self._clear_frames += 1
        if self._clear_frames >= self.policy.clear_consecutive_frames and now_s - self._clear_since >= self.policy.clear_hold_s:
            self._stop_latched = False
        return CLEAR if not self._stop_latched else STOP


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _within_future_tolerance(timestamp_s: float, reference_s: float) -> bool:
    return timestamp_s <= reference_s + CLOCK_FUTURE_TOLERANCE_S


def _age_within_future_tolerance(now_s: float, timestamp_s: float) -> float:
    age_s = now_s - timestamp_s
    return max(0.0, age_s) if _within_future_tolerance(timestamp_s, now_s) else age_s


def _slope_chronology_valid(
    source_s: float,
    evaluation_s: float,
    receipt_s: float,
    source_high_water_s: Optional[float],
) -> bool:
    return (
        all(_finite(value) for value in (source_s, evaluation_s, receipt_s))
        and 0.0 <= source_s <= evaluation_s + CLOCK_FUTURE_TOLERANCE_S
        and evaluation_s <= receipt_s + CLOCK_FUTURE_TOLERANCE_S
        and (source_high_water_s is None or source_s > source_high_water_s)
    )
@dataclass(frozen=True)
class SlopeEvidence:
    source_s: float
    evaluation_s: float
    receipt_s: float
    pitch_rad: float
    policy_sha256: str
    valid: bool


@dataclass(frozen=True)
class CollisionInputSnapshot:
    now_s: float
    odom: Optional[Tuple[float, float]]
    odom_stamp: Optional[float]
    odom_receipt: Optional[float]
    odom_valid: bool
    nav: Optional[Tuple[float, float]]
    nav_stamp: Optional[float]
    safe: Optional[Tuple[float, float]]
    safe_stamp: Optional[float]
    intent: Optional[Tuple[float, int, float, float]]
def _buffer_slope_evidence(
    evidence: Deque[SlopeEvidence], entry: SlopeEvidence
) -> None:
    if not _finite(entry.source_s):
        evidence.append(entry)
    else:
        position = len(evidence)
        for index, existing in enumerate(evidence):
            if not _finite(existing.source_s) or entry.source_s < existing.source_s:
                position = index
                break
        evidence.insert(position, entry)
    finite_sources = [item.source_s for item in evidence if _finite(item.source_s)]
    if finite_sources:
        minimum_source = max(finite_sources) - SLOPE_EVIDENCE_BUFFER_HORIZON_S
        while evidence and (
            not _finite(evidence[0].source_s)
            or evidence[0].source_s < minimum_source
        ):
            evidence.popleft()
    while len(evidence) > SLOPE_EVIDENCE_BUFFER_MAXLEN:
        evidence.popleft()


def _select_slope_evidence(
    evidence: Iterable[SlopeEvidence], cloud_stamp_s: float, now_s: float
) -> Optional[SlopeEvidence]:
    candidates = tuple(
        entry for entry in evidence
        if _finite(entry.source_s)
        and _slope_cloud_time_valid(
            entry.source_s, entry.evaluation_s, cloud_stamp_s, now_s
        )
    )
    fresh = tuple(
        entry for entry in candidates
        if _slope_timing_valid(entry.source_s, entry.receipt_s, cloud_stamp_s, now_s)
    )
    selected = fresh if fresh else candidates
    if fresh:
        restrictive = tuple(entry for entry in fresh if not entry.valid)
        if restrictive:
            selected = restrictive
    return max(
        selected,
        key=lambda entry: (entry.source_s, entry.evaluation_s, entry.receipt_s),
        default=None,
    )


def _slope_timing_valid(
    source_s: float, receipt_s: float, cloud_stamp_s: float, now_s: float
) -> bool:
    return (
        all(_finite(value) for value in (source_s, receipt_s, cloud_stamp_s, now_s))
        # Source stamps establish the slope/cloud evidence association.  Receipt
        # stamps establish freshness at the later collision evaluation; comparing
        # the source stamp directly to now rejects a valid asynchronous pair.
        and source_s <= cloud_stamp_s + CLOCK_FUTURE_TOLERANCE_S
        and cloud_stamp_s - source_s <= 0.10
        and source_s <= receipt_s + CLOCK_FUTURE_TOLERANCE_S
        and receipt_s <= now_s + CLOCK_FUTURE_TOLERANCE_S
        and now_s - receipt_s <= 0.10
    )


def _postprocess_evaluation_time(snapshot_now_s: float, postprocess_now_s: float) -> float:
    if (_finite(snapshot_now_s) and _finite(postprocess_now_s)
            and postprocess_now_s >= snapshot_now_s):
        return float(postprocess_now_s)
    return math.nan




def _slope_cloud_time_valid(
    source_s: float,
    evaluation_s: float,
    cloud_stamp_s: float,
    now_s: float,
) -> bool:
    return (
        source_s <= cloud_stamp_s + CLOCK_FUTURE_TOLERANCE_S
        and evaluation_s <= now_s + CLOCK_FUTURE_TOLERANCE_S
    )


def _direction_disagreement(values: Sequence[float], tolerance: float) -> bool:
    material = [float(value) for value in values if _finite(value) and abs(float(value)) >= tolerance]
    return bool(material) and min(material) < 0.0 < max(material)


class CollisionSupervisorRosNode:
    """Thin ROS1 adapter that transforms lidar_link clouds into the policy frame."""

    def __init__(self):
        import os
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from wheelchair_interfaces.msg import (
            CollisionStatus, MotionIntent, SafetySignal, SlopeStatus
        )

        policy_path = rospy.get_param(
            "~policy_file", os.path.join(os.path.dirname(__file__), "..", "config", "collision_policy.yaml")
        )
        self.core = CollisionSupervisorCore(CollisionPolicy.load(policy_path))
        self.preprocessor = CloudPreprocessorTracker(self.core.policy)
        self._input_lock = threading.RLock()
        self._slope_condition = threading.Condition(self._input_lock)
        self._decision_lock = threading.RLock()
        self.watchdog = CollisionWatchdogState(self.core.policy.cloud_ttl_s)
        self.cloud_deadline_timer = None
        self.sequence = 0
        self.odom = self.nav = self.safe = None
        self.odom_stamp = self.odom_receipt = None
        self.odom_high_water = None
        self.odom_valid = False
        self.nav_stamp = self.safe_stamp = None
        self.slope = None
        self.slope_high_water = None
        self.slope_evidence: Deque[SlopeEvidence] = deque()
        self._slope_stream_valid = False
        self.slope_policy_sha256 = str(rospy.get_param("~slope_policy_sha256", ""))
        self.intent = None
        self.transform_lookup_timeout_s = float(rospy.get_param("~transform_lookup_timeout_s", 0.05))
        self.max_transform_age_s = float(rospy.get_param("~max_transform_age_s", 0.10))
        self.watchdog_period_s = float(rospy.get_param(
            "~watchdog_period_s", min(0.10, self.core.policy.cloud_ttl_s / 3.0)
        ))
        if (not _finite(self.transform_lookup_timeout_s) or self.transform_lookup_timeout_s < 0.0
                or not _finite(self.max_transform_age_s) or self.max_transform_age_s < 0.0
                or not _finite(self.watchdog_period_s) or self.watchdog_period_s <= 0.0
                or self.watchdog_period_s > self.core.policy.cloud_ttl_s):
            raise ValueError("transform and watchdog timing bounds are invalid")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        status_topic = rospy.get_param("~status_topic", "/safety/collision_status")
        signal_topic = rospy.get_param("~signal_topic", "/safety/collision")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        nav_topic = rospy.get_param("~nav_topic", "/cmd_vel_nav")
        safe_topic = rospy.get_param("~safe_topic", "/cmd_vel_safe")
        cloud_topic = rospy.get_param("~cloud_topic", "/sensors/lidar/points")
        intent_topic = rospy.get_param("~intent_topic", "/decision/motion_intent")
        slope_topic = rospy.get_param("~slope_topic", "/safety/slope_status")
        if (len(self.slope_policy_sha256) != 64
                or any(character not in "0123456789abcdef" for character in self.slope_policy_sha256)):
            raise ValueError("collision supervisor requires exact slope policy SHA-256")
        self.status_pub = rospy.Publisher(status_topic, CollisionStatus, queue_size=1)
        self.signal_pub = rospy.Publisher(signal_topic, SafetySignal, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(nav_topic, Twist, self._nav_cb, queue_size=1)
        rospy.Subscriber(intent_topic, MotionIntent, self._intent_cb, queue_size=1)
        rospy.Subscriber(safe_topic, Twist, self._safe_cb, queue_size=1)
        rospy.Subscriber(slope_topic, SlopeStatus, self._slope_cb, queue_size=1)
        rospy.Subscriber(cloud_topic, PointCloud2, self._cloud_cb, queue_size=1)
        self._watchdog_cb(None)
        self.watchdog_timer = rospy.Timer(
            rospy.Duration.from_sec(self.watchdog_period_s), self._watchdog_cb
        )

    def _odom_cb(self, msg):
        import rospy
        try:
            source = float(msg.header.stamp.to_sec())
            values = (
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            )
        except (AttributeError, TypeError, ValueError):
            source, values = math.nan, (math.nan, math.nan)
        with self._input_lock:
            receipt = rospy.Time.now().to_sec()
            valid = (all(_finite(value) for value in values + (source, receipt,))
                     and 0.0 <= source
                     and _within_future_tolerance(source, receipt)
                     and (self.odom_high_water is None or source > self.odom_high_water))
            self.odom = values
            self.odom_stamp = source
            self.odom_receipt = receipt
            self.odom_valid = valid
            if valid:
                self.odom_high_water = source

    def _nav_cb(self, msg):
        import rospy
        values = (float(msg.linear.x), float(msg.angular.z))
        with self._input_lock:
            self.nav = values
            self.nav_stamp = rospy.Time.now().to_sec()

    def _intent_cb(self, msg):
        try:
            stamp = float(msg.header.stamp.to_sec())
            behavior = int(msg.behavior)
            linear_cap = float(msg.max_linear_mps)
            angular_cap = float(msg.max_angular_rps)
        except (AttributeError, TypeError, ValueError):
            stamp, behavior, linear_cap, angular_cap = math.nan, -1, math.nan, math.nan
        with self._input_lock:
            self.intent = (stamp, behavior, linear_cap, angular_cap)
            if behavior == INTENT_HOLD and linear_cap == 0.0 and angular_cap == 0.0:
                # A later PROCEED/SLOW must receive a command newer than this HOLD.
                self.nav = self.nav_stamp = None

    def _slope_cb(self, msg):
        import rospy
        try:
            source = float(msg.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            source = math.nan
        try:
            evaluation = float(msg.evaluation_stamp.to_sec())
            pitch = float(msg.pitch_rad)
            policy = str(msg.policy_sha256)
            state = int(msg.state)
        except (AttributeError, TypeError, ValueError):
            evaluation, pitch, policy, state = math.nan, math.nan, "", UNKNOWN
        with self._slope_condition:
            try:
                receipt = rospy.Time.now().to_sec()
            except (AttributeError, TypeError, ValueError):
                receipt = math.nan
            chronology_valid = _slope_chronology_valid(
                source, evaluation, receipt, self.slope_high_water
            )
            valid = bool(
                chronology_valid
                and _finite(pitch)
                and state in (CLEAR, CAUTION)
                and policy == self.slope_policy_sha256
            )
            self.slope = (source, evaluation, receipt, pitch, policy, valid)
            if not chronology_valid:
                self.slope_evidence.clear()
                self._slope_stream_valid = False
                self._slope_condition.notify_all()
                return
            entry = SlopeEvidence(source, evaluation, receipt, pitch, policy, valid)
            _buffer_slope_evidence(self.slope_evidence, entry)
            self._slope_stream_valid = True
            self.slope_high_water = source
            self._slope_condition.notify_all()
    def _safe_cb(self, msg):
        import rospy
        values = (float(msg.linear.x), float(msg.angular.z))
        with self._input_lock:
            self.safe = values
            self.safe_stamp = rospy.Time.now().to_sec()

    def _take_input_snapshot(self):
        import rospy
        with self._input_lock:
            return CollisionInputSnapshot(
                now_s=rospy.Time.now().to_sec(),
                odom=self.odom,
                odom_stamp=self.odom_stamp,
                odom_receipt=self.odom_receipt,
                odom_valid=self.odom_valid,
                nav=self.nav,
                nav_stamp=self.nav_stamp,
                safe=self.safe,
                safe_stamp=self.safe_stamp,
                intent=self.intent,
            )

    def _take_postprocess_slope_snapshot(self, snapshot_now_s, cloud_stamp_s):
        import rospy
        deadline = time.monotonic() + SLOPE_EVIDENCE_JOIN_TIMEOUT_S
        with self._slope_condition:
            while True:
                now_s = _postprocess_evaluation_time(
                    snapshot_now_s, rospy.Time.now().to_sec()
                )
                slope_evidence = tuple(self.slope_evidence)
                if (not _finite(now_s) or not self._slope_stream_valid):
                    return now_s, slope_evidence
                if any(
                    _slope_cloud_time_valid(
                        entry.source_s, entry.evaluation_s, cloud_stamp_s, now_s
                    )
                    and _slope_timing_valid(
                        entry.source_s, entry.receipt_s, cloud_stamp_s, now_s
                    )
                    for entry in slope_evidence
                ):
                    return now_s, slope_evidence
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return now_s, slope_evidence
                self._slope_condition.wait(remaining)

    def _cloud_cb(self, msg):
        with self._decision_lock:
            self._evaluate_cloud(msg)

    def _evaluate_cloud(self, msg):
        import rospy
        from sensor_msgs import point_cloud2
        stamp = msg.header.stamp.to_sec()
        result = CloudTransformResult((), self.core.policy.evaluation_frame, -1.0, False, "lookup_failure")
        try:
            transform = self.tf_buffer.lookup_transform(
                self.core.policy.evaluation_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration.from_sec(self.transform_lookup_timeout_s),
            )
            raw_points = (
                PointObservation(float(x), float(y), float(z))
                for x, y, z in point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=False
                )
            )
            vector = transform.transform.translation
            quaternion = transform.transform.rotation
            result = prepare_transformed_cloud(
                raw_points,
                msg.header.frame_id,
                self.core.policy.evaluation_frame,
                stamp,
                transform.header.stamp.to_sec(),
                (vector.x, vector.y, vector.z),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                self.max_transform_age_s,
                transform.child_frame_id,
                transform.header.frame_id,
                transform.header.stamp.to_sec() == 0.0,
            )
        except Exception as exc:  # tf2 and PointCloud2 decoding failures are safety evidence.
            rospy.logwarn_throttle(1.0, "Collision cloud transform failed: %s", exc)

        snapshot = self._take_input_snapshot()
        processing = CloudProcessingResult(
            (), 0, self.core.policy.coverage_bins * self.core.policy.coverage_elevation_bins,
            0, 0.0, False, result.reason
        )
        odom_linear, odom_angular = snapshot.odom or (math.nan, math.nan)
        nav_linear, nav_angular = snapshot.nav or (0.0, 0.0)
        safe_linear, safe_angular = snapshot.safe or (0.0, 0.0)
        intent_behavior = snapshot.intent[1] if snapshot.intent is not None else -1
        coverage_linear = self._conservative_component(
            odom_linear, nav_linear, safe_linear, snapshot.nav is not None, intent_behavior
        )
        coverage_angular = self._conservative_component(
            odom_angular, nav_angular, safe_angular, snapshot.nav is not None, intent_behavior
        )
        if result.ok:
            processing = self.preprocessor.process(
                result.points,
                stamp,
                coverage_linear,
                coverage_angular,
                (vector.x, vector.y, vector.z),
                ego_linear_speed_mps=odom_linear,
                ego_angular_speed_rps=odom_angular,
            )
        if not result.ok or not processing.ok:
            rospy.logwarn_throttle(
                1.0, "Collision cloud rejected: %s",
                result.reason if not result.ok else processing.reason,
            )
        now, slope_evidence = self._take_postprocess_slope_snapshot(snapshot.now_s, stamp)
        if self.watchdog.observe_cloud(stamp, now):
            self._schedule_cloud_deadline(now, stamp)
        slope_entry = _select_slope_evidence(slope_evidence, stamp, now)
        slope_source, slope_evaluation, slope_receipt, pitch, slope_policy, slope_valid = (
            (slope_entry.source_s, slope_entry.evaluation_s, slope_entry.receipt_s,
             slope_entry.pitch_rad, slope_entry.policy_sha256, slope_entry.valid)
            if slope_entry is not None
            else (math.nan, math.nan, math.nan, math.nan, "", False)
        )
        slope_valid = bool(
            slope_valid
            and slope_policy == self.slope_policy_sha256
        )
        points = processing.points
        self.sequence = self._next_sequence()
        intent_stamp, intent_behavior, linear_cap, angular_cap = (
            snapshot.intent if snapshot.intent is not None else (0.0, -1, math.nan, math.nan)
        )
        inputs = CollisionInputs(
            now, self.sequence, stamp,
            snapshot.odom_stamp if snapshot.odom_valid else math.nan,
            snapshot.nav_stamp or 0.0,
            snapshot.safe_stamp or 0.0, points, odom_linear, nav_linear,
            safe_linear, frame_id=result.frame_id,
            pitch_downhill_rad=pitch,
            coverage_fraction=processing.coverage_fraction,
            expected_coverage_bins=processing.expected_coverage_bins,
            observed_coverage_bins=processing.observed_coverage_bins,
            transform_ok=(result.ok and processing.ok),
            transform_age_s=result.transform_age_s,
            policy_id=self.core.policy.policy_id, policy_sha256=self.core.policy.policy_sha256,
            raw_point_count=processing.raw_point_count, intent_stamp_s=intent_stamp,
            intent_behavior=intent_behavior, intent_max_linear_mps=linear_cap,
            intent_max_angular_rps=angular_cap, nav_available=snapshot.nav is not None,
            odom_angular_rps=odom_angular, nav_angular_rps=nav_angular,
            safe_angular_rps=safe_angular,
            odom_receipt_s=snapshot.odom_receipt,
            slope_stamp_s=slope_source,
            slope_receipt_s=slope_receipt,
            slope_valid=slope_valid,
        )
        decision = self.core.evaluate(inputs)
        if decision.reason_mask:
            rospy.logwarn_throttle(
                1.0,
                "Collision decision STOP: %s (mask=%d)",
                decision.reason,
                decision.reason_mask,
            )
        self._publish(decision, msg.header)

    def _schedule_cloud_deadline(self, now_s, cloud_stamp_s):
        import rospy
        if self.cloud_deadline_timer is not None:
            self.cloud_deadline_timer.shutdown()
        age = max(0.0, now_s - cloud_stamp_s)
        remaining = self.core.policy.cloud_ttl_s - age
        if remaining <= 0.0:
            self.cloud_deadline_timer = None
            return
        self.cloud_deadline_timer = rospy.Timer(
            rospy.Duration.from_sec(remaining), self._watchdog_cb, oneshot=True
        )
    def _watchdog_cb(self, _event):
        import rospy
        from std_msgs.msg import Header
        with self._decision_lock:
            now = rospy.Time.now().to_sec()
            cloud_age = self.watchdog.stale_age(now)
            if cloud_age is None:
                return
            decision = self.core.stale_cloud_decision(
                self._next_sequence(), now, cloud_age
            )
            source_stamp = self.watchdog.last_cloud_stamp_s
            header = Header(
                stamp=(rospy.Time.from_sec(source_stamp)
                       if source_stamp is not None else rospy.Time())
            )
            self._publish(decision, header)


    def _next_sequence(self):
        if self.sequence >= 0xFFFFFFFF:
            raise RuntimeError("collision publication sequence exhausted")
        self.sequence += 1
        return self.sequence

    @staticmethod
    def _conservative_component(measured, requested, prior_safe, nav_available, intent_behavior):
        values = (measured, prior_safe)
        if intent_behavior == INTENT_HOLD or not nav_available:
            return max(values, key=abs)
        values += (requested,)
        if requested > 0.0:
            return max(0.0, *values)
        if requested < 0.0:
            return min(0.0, *values)
        return max(values, key=abs)

    def _publish(self, decision: CollisionDecision, header):
        import rospy
        from std_msgs.msg import Header
        from wheelchair_interfaces.msg import CollisionStatus, SafetySignal
        status = CollisionStatus()
        status.header = Header(
            seq=decision.sequence,
            stamp=header.stamp,
            frame_id=self.core.policy.evaluation_frame,
        )
        status.evaluation_stamp = rospy.Time.from_sec(max(0.0, decision.evaluation_stamp))
        for name in ("sequence", "state", "visibility", "obstacle_motion", "reason_mask", "source", "policy_id",
                     "policy_sha256", "input_age_s", "transform_age_s", "odom_age_s", "command_age_s",
                     "coverage_fraction", "forward_speed_mps", "angular_speed_rps", "closing_speed_mps",
                     "nearest_x_m", "nearest_y_m", "nearest_distance_m", "time_to_collision_s",
                     "reaction_distance_m", "braking_distance_m", "uncertainty_margin_m", "required_stop_distance_m",
                     "clear_distance_m", "recommended_max_linear_mps", "obstacle_point_count", "consecutive_clear_frames"):
            setattr(status, name, getattr(decision, name))
        signal_header = Header(seq=decision.sequence,
                               stamp=rospy.Time.from_sec(max(0.0, decision.evaluation_stamp)),
                               frame_id=self.core.policy.evaluation_frame)
        signal = SafetySignal(header=signal_header, sequence=decision.sequence, state=decision.signal_state,
                              reason_mask=decision.reason_mask, source=decision.source,
                              policy_sha256=decision.policy_sha256)
        self.status_pub.publish(status)
        self.signal_pub.publish(signal)


def run_ros_node() -> None:
    import rospy
    rospy.init_node("collision_supervisor")
    CollisionSupervisorRosNode()
    rospy.spin()


if __name__ == "__main__":
    run_ros_node()
