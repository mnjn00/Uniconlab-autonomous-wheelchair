#!/usr/bin/env python3
"""Immutable route-safety policy core and lazy ROS 1 adapter.

The core deliberately has no ROS imports.  Geometry is loaded exactly once from the
WP0 A06 manifest and converted to frozen values; runtime route messages contribute
identifiers and hashes only, never geometry or thresholds.
"""

from dataclasses import dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
from threading import Lock
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import jsonschema
import yaml

Point = Tuple[float, float]
Polygon = Tuple[Point, ...]
SHA256_LENGTH = 64
SOURCE = "wheelchair_route_safety"
FUTURE_TOLERANCE_S = 0.05  # Frozen A03 clock contract.
ACTIVE_ROUTE_TTL_S = 0.75  # Frozen mission route-authorization contract.
LOCALIZATION_POLICY_SHA256 = "5d84ea824c98a53639a480ed162a62f015600ca0a0460df7186d5839303d52e8"
LOCALIZATION_EVIDENCE_CACHE_SIZE = 8

UNKNOWN = 0
CLEAR = 1
STOP = 2
STATUS_UNKNOWN = 0
STATUS_INSIDE = 1
STATUS_MARGIN = 2
STATUS_OUTSIDE = 3
STATUS_MANIFEST_ERROR = 4

REASON_GEOFENCE = 1 << 3
REASON_INTERNAL_FAULT = 1 << 10
REASON_SENSOR_STALE = 1 << 12
REASON_ROUTE_MANIFEST = 1 << 18
REASON_TF = 1 << 20
REASON_MAP_MISMATCH = 1 << 25
REASON_INPUT_UNKNOWN = 1 << 31
REASON_ROUTE_STATE = 1 << 32
REASON_POLICY_MISMATCH = 1 << 36

class LocalizationEvidenceBuffer:
    """Bounded candidate/status evidence matched only by immutable identity fields."""

    def __init__(self, limit: int = LOCALIZATION_EVIDENCE_CACHE_SIZE) -> None:
        if limit < 1:
            raise ValueError("localization evidence cache limit must be positive")
        self._limit = limit
        self._lock = Lock()
        self.candidates: List[Any] = []
        self.statuses: List[Any] = []

    @staticmethod
    def _matches(candidate: Any, status: Any) -> bool:
        try:
            candidate_stamp = float(candidate.pose.header.stamp.to_sec())
            status_stamp = float(status.header.stamp.to_sec())
            return (
                math.isfinite(candidate_stamp)
                and math.isfinite(status_stamp)
                and abs(candidate_stamp - status_stamp) <= 1.0e-9
                and candidate.reset_count == status.reset_count
                and candidate.map_id == status.map_id
                and candidate.map_sha256 == status.map_sha256
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    @staticmethod
    def _append_bounded(cache: List[Any], value: Any, limit: int) -> None:
        cache.append(value)
        if len(cache) > limit:
            del cache[0:len(cache) - limit]

    def add_candidate(self, candidate: Any) -> None:
        with self._lock:
            self._append_bounded(self.candidates, candidate, self._limit)

    def add_status(self, status_evidence: Any) -> None:
        with self._lock:
            self._append_bounded(self.statuses, status_evidence, self._limit)

    def snapshot(self) -> Tuple[Optional[Any], Optional[Any]]:
        """Return the newest status and its exact matching candidate atomically."""
        with self._lock:
            status_evidence = self.statuses[-1] if self.statuses else None
            if status_evidence is None:
                return None, None
            status = status_evidence[0]
            candidate = next(
                (candidate for candidate in reversed(self.candidates)
                 if self._matches(candidate, status)),
                None,
            )
            return status_evidence, candidate
def status_chronology_is_newer(current: Tuple[int, float, float, float],
                               previous: Optional[Tuple[int, float, float, float]]) -> bool:
    """Accept a later status without treating an unchanged source identity as a regression."""
    if previous is None:
        return True
    current_sequence, current_source, current_evaluation, current_receipt = current
    previous_sequence, previous_source, previous_evaluation, previous_receipt = previous
    return (
        current_sequence > previous_sequence
        and current_source >= previous_source
        and current_evaluation >= previous_evaluation
        and current_receipt > previous_receipt
    )


def status_allows_pair_hold(status: Any, receipt_stamp: float, prior_reset_count: Any,
                            map_id: str, map_sha256: str, frame_id: str,
                            policy_sha256: str, ok_state: Any) -> bool:
    """Validate status-only evidence before retaining a prior exact pair."""
    try:
        source_stamp = float(status.header.stamp.to_sec())
        evaluation_stamp = float(status.evaluation_stamp.to_sec())
        transform_age = float(status.transform_age_s)
        return (
            int(status.sequence) >= 0
            and math.isfinite(source_stamp)
            and math.isfinite(evaluation_stamp)
            and math.isfinite(receipt_stamp)
            and math.isfinite(transform_age)
            and source_stamp <= evaluation_stamp <= receipt_stamp + FUTURE_TOLERANCE_S
            and transform_age >= 0.0
            and status.reset_count == prior_reset_count
            and status.map_id == map_id
            and status.map_sha256 == map_sha256
            and status.header.frame_id == frame_id
            and status.policy_sha256 == policy_sha256
            and status.state == ok_state
            and status.reason_mask == 0
            and status.independent_check_passed
        )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


class ManifestError(ValueError):
    """A manifest cannot safely become an immutable policy."""


@dataclass(frozen=True)
class RoutePolicy:
    route_id: str
    direction: str
    route_manifest_sha256: str
    corridor: Polygon
    corridor_margin_m: float
    segment_ids: Tuple[str, ...]
    zone_ids: Tuple[str, ...]
    centerline: Tuple[Point, ...] = ()
    tube_radius_m: float = 0.0
    segment_centerlines: Tuple[Tuple[Point, Point], ...] = ()


@dataclass(frozen=True)
class ZonePolicy:
    zone_id: str
    polygon: Polygon
    policy: str


@dataclass(frozen=True)
class RouteSafetyPolicy:
    manifest_id: str
    manifest_sha256: str
    map_id: str
    map_sha256: str
    frame_id: str
    geometry_sha256: str
    footprint_length_m: float
    footprint_width_m: float
    configured_uncertainty_margin_m: float
    fixed_boundary_margin_m: float
    pose_ttl_s: float
    status_ttl_s: float
    transform_ttl_s: float
    global_allowed: Polygon
    exclusions: Tuple[Polygon, ...]
    routes: Tuple[RoutePolicy, ...]
    zones: Tuple[ZonePolicy, ...]
    simulation_only: bool = False
    localization_policy_sha256: str = ""

    def route(self, route_id: str) -> Optional[RoutePolicy]:
        return next((route for route in self.routes if route.route_id == route_id), None)

    def zone(self, zone_id: str) -> Optional[ZonePolicy]:
        return next((zone for zone in self.zones if zone.zone_id == zone_id), None)


@dataclass(frozen=True)
class ActiveRouteSelection:
    """Untrusted identity-only input from navigation/decision."""

    route_id: str
    route_manifest_sha256: str
    safety_manifest_sha256: str
    map_id: str
    map_sha256: str
    segment_id: str
    zone_id: str
    mission_id: str = ""


@dataclass(frozen=True)
class PoseSample:
    """Candidate pose plus independent status/TF evidence, all in seconds."""

    x_m: float
    y_m: float
    yaw_rad: float
    pose_stamp_s: float
    status_stamp_s: float
    transform_stamp_s: float
    position_std_m: float
    localization_state: str
    pose_frame_id: str = "map"
    transform_valid: bool = True


@dataclass(frozen=True)
class GeofenceEvaluation:
    sequence: int
    evaluation_stamp_s: float
    pose_stamp_s: float
    frame_id: str
    state: int
    signal_state: int
    reason_mask: int
    source: str
    manifest_id: str
    manifest_sha256: str
    route_id: str
    segment_id: str
    zone_id: str
    pose_age_s: float
    transform_age_s: float
    position_uncertainty_m: float
    minimum_signed_clearance_m: float
    required_boundary_margin_m: float
    policy_sha256: str

    @property
    def clear(self) -> bool:
        return self.state == STATUS_INSIDE and self.signal_state == CLEAR and self.reason_mask == 0


def _schema_path(explicit: Optional[os.PathLike]) -> Path:
    if explicit is not None:
        path = Path(explicit)
        if path.is_file():
            return path
        raise ManifestError("A06 schema does not exist: {}".format(path))
    configured = os.environ.get("WHEELCHAIR_ROUTE_SAFETY_SCHEMA")
    if configured:
        return _schema_path(configured)
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for parent in (start,) + tuple(start.parents):
            path = parent / "contracts" / "wp0" / "A06-route-safety-schema.json"
            if path.is_file():
                return path
    raise ManifestError("A06 route-safety schema was not found")


def _sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _finite_number(value: Any, name: str, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ManifestError("{} must be finite".format(name))
    result = float(value)
    if positive and result <= 0.0:
        raise ManifestError("{} must be positive".format(name))
    return result


def _cross(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _area(poly: Polygon) -> float:
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))


def _on_segment(point: Point, a: Point, b: Point, epsilon: float = 1e-10) -> bool:
    return abs(_cross(a, b, point)) <= epsilon and min(a[0], b[0]) - epsilon <= point[0] <= max(a[0], b[0]) + epsilon and min(a[1], b[1]) - epsilon <= point[1] <= max(a[1], b[1]) + epsilon


def segments_intersect(a: Point, b: Point, c: Point, d: Point, include_boundary: bool = True) -> bool:
    values = (_cross(a, b, c), _cross(a, b, d), _cross(c, d, a), _cross(c, d, b))
    if ((values[0] > 0 > values[1]) or (values[1] > 0 > values[0])) and ((values[2] > 0 > values[3]) or (values[3] > 0 > values[2])):
        return True
    if not include_boundary:
        return False
    return any(abs(v) <= 1e-10 and _on_segment(p, x, y) for v, p, x, y in ((values[0], c, a, b), (values[1], d, a, b), (values[2], a, c, d), (values[3], b, c, d)))


def point_in_polygon(point: Point, polygon: Polygon, boundary_inside: bool = False) -> bool:
    """Ray-cast containment with explicit boundary semantics."""
    inside = False
    for index, a in enumerate(polygon):
        b = polygon[(index + 1) % len(polygon)]
        if _on_segment(point, a, b):
            return boundary_inside
        if (a[1] > point[1]) != (b[1] > point[1]):
            crossing_x = (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]
            if crossing_x > point[0]:
                inside = not inside
    return inside


def _polygon_edges(poly: Polygon):
    return tuple((poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


def _simple(poly: Polygon) -> bool:
    edges = _polygon_edges(poly)
    for i, edge_a in enumerate(edges):
        for j, edge_b in enumerate(edges):
            if i >= j or j in (i, (i + 1) % len(edges)) or i == (j + 1) % len(edges):
                continue
            if segments_intersect(edge_a[0], edge_a[1], edge_b[0], edge_b[1]):
                return False
    return True


def _polygon(value: Sequence[Sequence[float]], name: str, clockwise: bool = False) -> Polygon:
    try:
        poly = tuple((_finite_number(point[0], name), _finite_number(point[1], name)) for point in value)
    except (IndexError, TypeError) as exc:
        raise ManifestError("{} contains an invalid point".format(name)) from exc
    if len(poly) < 3 or len(set(poly)) < 3 or len(set(poly)) != len(poly):
        raise ManifestError("{} must have unique vertices".format(name))
    signed_area = _area(poly)
    if abs(signed_area) <= 1e-10 or (clockwise and signed_area >= 0) or (not clockwise and signed_area <= 0):
        raise ManifestError("{} has invalid orientation or area".format(name))
    if not _simple(poly):
        raise ManifestError("{} must be simple".format(name))
    return poly


def _polygon_strictly_inside(inner: Polygon, outer: Polygon) -> bool:
    if not all(point_in_polygon(point, outer, False) for point in inner):
        return False
    return not any(segments_intersect(a, b, c, d) for a, b in _polygon_edges(inner) for c, d in _polygon_edges(outer))


def _polygons_overlap(a: Polygon, b: Polygon) -> bool:
    if any(segments_intersect(x, y, u, v) for x, y in _polygon_edges(a) for u, v in _polygon_edges(b)):
        return True
    return point_in_polygon(a[0], b, True) or point_in_polygon(b[0], a, True)


def _distance_point_segment(point: Point, a: Point, b: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 == 0.0:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - (a[0] + t * dx), point[1] - (a[1] + t * dy))


def _nearest_centerline_segment(point: Point, centerline: Tuple[Point, ...]) -> Tuple[float, Point, Point]:
    candidates = tuple(
        (_distance_point_segment(point, centerline[index], centerline[index + 1]), centerline[index], centerline[index + 1])
        for index in range(len(centerline) - 1)
    )
    return min(candidates, key=lambda candidate: candidate[0])


def _boundary_distance(a: Polygon, b: Polygon) -> float:
    return min(_distance_point_segment(point, x, y) for point in a for x, y in _polygon_edges(b))


def transformed_footprint(x_m: float, y_m: float, yaw_rad: float, length_m: float, width_m: float, margin_m: float = 0.0) -> Polygon:
    """Transform a measured axis-aligned base rectangle into map coordinates."""
    values = (x_m, y_m, yaw_rad, length_m, width_m, margin_m)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError("footprint transform values must be finite")
    if length_m <= 0 or width_m <= 0 or margin_m < 0:
        raise ValueError("footprint dimensions must be positive and margin nonnegative")
    hx, hy = length_m / 2.0 + margin_m, width_m / 2.0 + margin_m
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return tuple((x_m + cosine * px - sine * py, y_m + sine * px + cosine * py) for px, py in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)))


def load_policy(
    manifest_path: os.PathLike,
    expected_manifest_sha256: str,
    expected_map_sha256: str,
    expected_route_hashes: Mapping[str, str],
    footprint_length_m: float,
    footprint_width_m: float,
    expected_geometry_sha256: str,
    pose_ttl_s: float = 0.25,
    status_ttl_s: float = 0.25,
    transform_ttl_s: float = 0.25,
    schema_path: Optional[os.PathLike] = None,
) -> RouteSafetyPolicy:
    """Load and bind A06 exactly once; returned policy exposes no mutation API."""
    try:
        raw = Path(manifest_path).read_bytes()
    except OSError as exc:
        raise ManifestError("manifest read failed: {}".format(exc)) from exc
    actual_hash = _sha256(raw)
    if expected_manifest_sha256 != actual_hash:
        raise ManifestError("manifest SHA-256 mismatch")
    try:
        manifest = yaml.safe_load(raw)
        schema = json.loads(_schema_path(schema_path).read_text(encoding="utf-8"))
        jsonschema.Draft7Validator.check_schema(schema)
        runtime_schema = dict(schema)
        runtime_schema.pop("$id", None)
        jsonschema.Draft7Validator(runtime_schema).validate(manifest)
    except (yaml.YAMLError, json.JSONDecodeError, OSError, jsonschema.ValidationError,
            jsonschema.SchemaError, jsonschema.exceptions.RefResolutionError) as exc:
        raise ManifestError("A06 validation failed: {}".format(exc)) from exc
    if not isinstance(manifest, dict):
        raise ManifestError("manifest root must be a mapping")
    def reject_nonfinite(value: Any, path: str = "$") -> None:
        if isinstance(value, Mapping):
            for key, child in value.items():
                reject_nonfinite(child, "{}.{}".format(path, key))
        elif isinstance(value, (list, tuple)):
            for index, child in enumerate(value):
                reject_nonfinite(child, "{}[{}]".format(path, index))
        elif isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
            raise ManifestError("{} must be finite".format(path))

    reject_nonfinite(manifest)
    ttl_values = (pose_ttl_s, status_ttl_s, transform_ttl_s)
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) or not 0.0 < float(value) <= 0.25
           for value in ttl_values):
        raise ManifestError("pose/status/transform TTLs must be in (0, 0.25]")
    if manifest["map"]["sha256"] != expected_map_sha256:
        raise ManifestError("map SHA-256 mismatch")
    if manifest["footprint"]["geometry_sha256"] != expected_geometry_sha256:
        raise ManifestError("footprint geometry SHA-256 mismatch")

    allowed = _polygon(manifest["global_allowed_polygon"], "global_allowed_polygon")
    exclusions = tuple(_polygon(value, "global_exclusion_polygons", True) for value in manifest["global_exclusion_polygons"])
    if any(not _polygon_strictly_inside(exclusion, allowed) for exclusion in exclusions):
        raise ManifestError("every exclusion must be strictly inside the global polygon")
    if any(_polygons_overlap(exclusions[i], exclusions[j]) for i in range(len(exclusions)) for j in range(i + 1, len(exclusions))):
        raise ManifestError("exclusions must not overlap")

    zones = tuple(ZonePolicy(item["zone_id"], _polygon(item["polygon"], "zone {}".format(item["zone_id"])), item["policy"]) for item in manifest["localization_zones"])
    zone_ids = [zone.zone_id for zone in zones]
    if len(set(zone_ids)) != len(zone_ids):
        raise ManifestError("duplicate zone_id")
    routes = []
    route_ids = []
    expected = dict(expected_route_hashes)
    for item in manifest["approved_routes"]:
        route_id = item["route_id"]
        route_ids.append(route_id)
        if expected.get(route_id) != item["route_manifest_sha256"]:
            raise ManifestError("route SHA-256 binding mismatch for {}".format(route_id))
        if any(zone_id not in zone_ids for zone_id in item["localization_zone_ids"]):
            raise ManifestError("route {} references an unknown zone".format(route_id))
        routes.append(RoutePolicy(route_id, item["direction"], item["route_manifest_sha256"], _polygon(item["corridor_polygon"], "route {} corridor".format(route_id)), float(item["corridor_margin_m"]), tuple(item["segment_ids"]), tuple(item["localization_zone_ids"])))
    if len(set(route_ids)) != len(route_ids) or set(expected) != set(route_ids):
        raise ManifestError("expected route bindings must exactly match approved routes")

    return RouteSafetyPolicy(
        manifest["manifest_id"], actual_hash, manifest["map"]["map_id"], manifest["map"]["sha256"], manifest["map"]["frame_id"],
        manifest["footprint"]["geometry_sha256"], _finite_number(footprint_length_m, "footprint_length_m", True),
        _finite_number(footprint_width_m, "footprint_width_m", True), float(manifest["footprint"]["localization_uncertainty_margin_m"]),
        float(manifest["footprint"]["fixed_boundary_margin_m"]), _finite_number(pose_ttl_s, "pose_ttl_s", True),
        _finite_number(status_ttl_s, "status_ttl_s", True), _finite_number(transform_ttl_s, "transform_ttl_s", True), allowed, exclusions,
        tuple(routes), zones,
    )


def _load_simulation_policy(config: Mapping[str, Any], config_path: Path) -> RouteSafetyPolicy:
    """Build a simulation tube only after validating all immutable bindings."""
    if (config.get("simulation_only") is not True
            or config.get("hardware_motion_authorized") is not False
            or config.get("passenger_operation_authorized") is not False):
        raise ManifestError("simulation policy must be explicitly simulation-only and non-authorizing")
    geometry, zone_values = config.get("simulation_geometry"), config.get("simulation_zones")
    if not isinstance(geometry, Mapping) or not isinstance(zone_values, list):
        raise ManifestError("simulation geometry and zones are required")
    if (geometry.get("immutable") is not True
            or geometry.get("runtime_mutation_allowed") is not False
            or geometry.get("widening_allowed") is not False
            or geometry.get("outside_corridor_action") != "STOP"
            or geometry.get("nonfinite_geometry_action") != "REJECT_AND_STOP"):
        raise ManifestError("simulation geometry must be immutable and fail closed")

    def finite_tree(value: Any, name: str) -> None:
        if isinstance(value, Mapping):
            for key, child in value.items():
                finite_tree(child, "{}.{}".format(name, key))
        elif isinstance(value, (list, tuple)):
            for index, child in enumerate(value):
                finite_tree(child, "{}[{}]".format(name, index))
        elif isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
            raise ManifestError("{} must be finite".format(name))

    finite_tree(geometry, "simulation_geometry")
    finite_tree(zone_values, "simulation_zones")

    def relative_path(value: Any, name: str) -> Path:
        if not isinstance(value, str) or not value:
            raise ManifestError("{} must be a path".format(name))
        path = Path(value)
        resolved = path if path.is_absolute() else (config_path.parent / path).resolve()
        if not resolved.is_file() and name in ("manifest_path", "schema_path"):
            installed = config_path.parent.parent / "contracts" / "wp0" / path.name
            resolved = installed if installed.is_file() else resolved
        return resolved

    route_path = relative_path(geometry.get("route_asset_path"), "route_asset_path")
    if not route_path.is_file():
        installed = config_path.parent.parent / "data" / route_path.name
        route_path = installed if installed.is_file() else route_path
    try:
        route_raw = route_path.read_bytes()
        route_asset = yaml.safe_load(route_raw)
    except (OSError, yaml.YAMLError) as exc:
        raise ManifestError("simulation route asset read failed: {}".format(exc)) from exc
    if not isinstance(route_asset, Mapping) or _sha256(route_raw) != geometry.get("route_asset_sha256"):
        raise ManifestError("simulation route asset SHA-256 mismatch")
    finite_tree(route_asset, "route_asset")
    if route_asset.get("immutable") is not True:
        raise ManifestError("simulation route asset is not immutable")
    navigation_path = relative_path(
        geometry.get("navigation_route_manifest_path"), "navigation_route_manifest_path",
    )
    if not navigation_path.is_file():
        installed = config_path.parent.parent / "data" / navigation_path.name
        navigation_path = installed if installed.is_file() else navigation_path
    try:
        navigation_raw = navigation_path.read_bytes()
        navigation_manifest = yaml.safe_load(navigation_raw)
    except (OSError, yaml.YAMLError) as exc:
        raise ManifestError("navigation route manifest read failed: {}".format(exc)) from exc
    if (not isinstance(navigation_manifest, Mapping)
            or _sha256(navigation_raw) != geometry.get("navigation_route_manifest_sha256")):
        raise ManifestError("navigation route manifest SHA-256 mismatch")
    finite_tree(navigation_manifest, "navigation_route_manifest")
    if navigation_manifest.get("immutable") is not True:
        raise ManifestError("navigation route manifest is not immutable")

    base = load_policy(
        relative_path(config.get("manifest_path"), "manifest_path"),
        config.get("expected_manifest_sha256"), config.get("expected_map_sha256"),
        config.get("expected_route_hashes"), config.get("measured_footprint_length_m"),
        config.get("measured_footprint_width_m"), config.get("expected_geometry_sha256"),
        config.get("pose_ttl_s", 0.25), config.get("status_ttl_s", 0.25),
        config.get("transform_ttl_s", 0.25), relative_path(config.get("schema_path"), "schema_path"),
    )
    asset_map = route_asset.get("map")
    if not isinstance(asset_map, Mapping) or (asset_map.get("map_id"), asset_map.get("sha256"), asset_map.get("frame_id")) != (base.map_id, base.map_sha256, base.frame_id):
        raise ManifestError("simulation route/map binding mismatch")
    navigation_map = navigation_manifest.get("map")
    waypoint_binding = navigation_manifest.get("waypoint_asset")
    if not isinstance(navigation_map, Mapping) or (
            navigation_map.get("map_id"), navigation_map.get("sha256"),
            navigation_map.get("frame_id")) != (base.map_id, base.map_sha256, base.frame_id):
        raise ManifestError("navigation route/map binding mismatch")
    if (not isinstance(waypoint_binding, Mapping)
            or waypoint_binding.get("sha256") != _sha256(route_raw)
            or navigation_manifest.get("safety_manifest_sha256") != base.manifest_sha256):
        raise ManifestError("navigation waypoint/safety binding mismatch")

    clearance = _finite_number(geometry.get("footprint_clearance_margin_m"), "footprint_clearance_margin_m")
    uncertainty = _finite_number(geometry.get("localization_uncertainty_margin_m"), "localization_uncertainty_margin_m")
    recorded_margin = _finite_number(geometry.get("recorded_corridor_margin_m"), "recorded_corridor_margin_m")
    if (clearance != base.fixed_boundary_margin_m
            or uncertainty != base.configured_uncertainty_margin_m
            or recorded_margin < 0.0):
        raise ManifestError("simulation margins must match the immutable safety manifest")

    if len(zone_values) != 1:
        raise ManifestError("exactly one simulation zone binding is required")
    zone_value = zone_values[0]
    base_zone = base.zone(zone_value.get("zone_id") if isinstance(zone_value, Mapping) else "")
    if not isinstance(zone_value, Mapping) or (
            base_zone is None or base_zone.policy != "normal"
            or zone_value.get("policy") != "normal"
            or zone_value.get("simulation_only") is not True
            or zone_value.get("hardware_authorized") is not False
            or zone_value.get("passenger_authorized") is not False):
        raise ManifestError("simulation zone must match one normal A06 zone and stay non-authorizing")
    localization_policy_sha256 = config.get("localization_policy_sha256")
    if localization_policy_sha256 != LOCALIZATION_POLICY_SHA256:
        raise ManifestError("simulation localization policy SHA-256 must match the frozen policy identity")
    if config.get("active_route_ttl_s") != ACTIVE_ROUTE_TTL_S:
        raise ManifestError("simulation ActiveRoute TTL must be frozen at 0.75 seconds")

    routes_by_direction = {}
    for route in base.routes:
        if route.direction in routes_by_direction:
            raise ManifestError("ambiguous approved route direction")
        routes_by_direction[route.direction] = route
    directions = tuple(zone_value.get("directions", ()))
    if set(directions) != set(routes_by_direction) or len(directions) != len(routes_by_direction):
        raise ManifestError("simulation zone directions do not exactly match approved routes")
    route_bindings = geometry.get("route_bindings")
    if not isinstance(route_bindings, Mapping) or set(route_bindings) != set(routes_by_direction):
        raise ManifestError("route bindings must exactly map every direction")

    simulation_routes = []
    tube_radius = (0.5 * math.hypot(base.footprint_length_m, base.footprint_width_m)
                   + clearance + uncertainty + recorded_margin)
    for direction in sorted(routes_by_direction):
        binding = route_bindings[direction]
        approved = routes_by_direction[direction]
        if not isinstance(binding, Mapping) or (
                binding.get("route_id") != approved.route_id
                or not isinstance(binding.get("asset_key"), str)):
            raise ManifestError("{} route binding mismatch".format(direction))
        source = route_asset.get(binding["asset_key"])
        if not isinstance(source, Mapping) or source.get("direction") != direction:
            raise ManifestError("missing or mismatched {} centerline".format(direction))
        navigation_source = navigation_manifest.get(binding["asset_key"])
        if not isinstance(navigation_source, Mapping) or (
                navigation_source.get("route_id") != approved.route_id
                or navigation_source.get("direction") != direction
                or navigation_source.get("route_manifest_sha256") != approved.route_manifest_sha256):
            raise ManifestError("{} navigation route binding mismatch".format(direction))
        waypoints = source.get("waypoints")
        if not isinstance(waypoints, list) or len(waypoints) < 2:
            raise ManifestError("{} centerline requires at least two points".format(direction))
        centerline = tuple((
            _finite_number(item.get("x_m") if isinstance(item, Mapping) else None, "{} x".format(direction)),
            _finite_number(item.get("y_m") if isinstance(item, Mapping) else None, "{} y".format(direction)),
        ) for item in waypoints)
        if any(centerline[index] == centerline[index + 1] for index in range(len(centerline) - 1)):
            raise ManifestError("{} centerline contains a zero-length segment".format(direction))
        navigation_waypoints = navigation_source.get("waypoints")
        if not isinstance(navigation_waypoints, list) or len(navigation_waypoints) != len(waypoints):
            raise ManifestError("{} navigation waypoint count mismatch".format(direction))
        for index, (navigation_waypoint, asset_waypoint) in enumerate(zip(navigation_waypoints, waypoints)):
            if not isinstance(navigation_waypoint, Mapping) or (
                    navigation_waypoint.get("x_m"), navigation_waypoint.get("y_m"),
                    navigation_waypoint.get("yaw_rad")) != (
                        asset_waypoint.get("x_m"), asset_waypoint.get("y_m"),
                        asset_waypoint.get("yaw_rad")):
                raise ManifestError("{} navigation waypoint {} mismatch".format(direction, index))
        segments = source.get("segments")
        if not isinstance(segments, list) or not segments or any(
                not isinstance(segment, Mapping)
                or segment.get("corridor_margin_m") != recorded_margin
                or segment.get("hardware_authorized") is not False
                or segment.get("zone_ids") != ["candidate-unsurveyed"]
                for segment in segments):
            raise ManifestError("{} route segment binding is unsafe".format(direction))
        navigation_segments = navigation_source.get("segments")
        if not isinstance(navigation_segments, list) or len(navigation_segments) != len(centerline) - 1:
            raise ManifestError("{} navigation segments do not cover its centerline".format(direction))
        segment_ids = []
        for index, segment in enumerate(navigation_segments):
            if not isinstance(segment, Mapping) or (
                    segment.get("start_waypoint_index") != index
                    or segment.get("end_waypoint_index") != index + 1
                    or segment.get("corridor_margin_m") != recorded_margin
                    or segment.get("corridor_width_m") != 2.0 * recorded_margin
                    or segment.get("zone_ids") != ["candidate-unsurveyed"]
                    or segment.get("hardware_authorized") is not False
                    or not isinstance(segment.get("segment_id"), str)
                    or not segment.get("segment_id")):
                raise ManifestError("{} navigation segment {} binding is unsafe".format(direction, index))
            segment_ids.append(segment["segment_id"])
        if len(set(segment_ids)) != len(segment_ids):
            raise ManifestError("{} navigation segment IDs are ambiguous".format(direction))
        simulation_routes.append(replace(
            approved, segment_ids=tuple(segment_ids),
            centerline=centerline,
            tube_radius_m=tube_radius,
            segment_centerlines=tuple(
                (centerline[index], centerline[index + 1])
                for index in range(len(centerline) - 1)
            ),
        ))

    return replace(
        base, routes=tuple(simulation_routes), simulation_only=True,
        localization_policy_sha256=localization_policy_sha256,
    )


def load_simulation_policy(
    config_path: os.PathLike,
    expected_config_sha256: str,
) -> RouteSafetyPolicy:
    """Load one byte-bound immutable simulation config; later changes cannot widen it."""
    resolved = Path(config_path).resolve()
    try:
        raw = resolved.read_bytes()
    except OSError as exc:
        raise ManifestError("simulation config read failed: {}".format(exc)) from exc
    if (not isinstance(expected_config_sha256, str)
            or len(expected_config_sha256) != SHA256_LENGTH
            or any(character not in "0123456789abcdef" for character in expected_config_sha256)):
        raise ManifestError("expected simulation config SHA-256 must be 64 lowercase hex characters")
    if _sha256(raw) != expected_config_sha256:
        raise ManifestError("simulation config SHA-256 mismatch")
    try:
        config = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise ManifestError("simulation config parse failed: {}".format(exc)) from exc
    if not isinstance(config, Mapping):
        raise ManifestError("simulation config root must be a mapping")
    return _load_simulation_policy(config, resolved)


def _derive_simulation_selection(route: RoutePolicy, pose: PoseSample,
                                 selection: ActiveRouteSelection) -> Optional[ActiveRouteSelection]:
    if (len(route.segment_centerlines) != len(route.segment_ids)
            or len(route.zone_ids) != 1):
        return None
    candidates = []
    for index, (first, second) in enumerate(route.segment_centerlines):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            return None
        alignment = (math.cos(pose.yaw_rad) * dx + math.sin(pose.yaw_rad) * dy) / length
        distance = _distance_point_segment((pose.x_m, pose.y_m), first, second)
        if alignment > 0.0 and distance <= route.corridor_margin_m + 1e-10:
            candidates.append((distance, -alignment, index))
    if not candidates:
        return None
    candidates.sort()
    best = candidates[0]
    if len(candidates) > 1:
        second = candidates[1]
        nonadjacent = abs(best[2] - second[2]) > 1
        indistinguishable = abs(best[0] - second[0]) <= 1e-8 and abs(best[1] - second[1]) <= 1e-6
        if nonadjacent and indistinguishable:
            return None
    index = best[2]
    return replace(
        selection,
        segment_id=route.segment_ids[index],
        zone_id=route.zone_ids[0],
    )


def _stop(policy: RouteSafetyPolicy, pose: Optional[PoseSample], selection: Optional[ActiveRouteSelection], now_s: float, sequence: int, state: int, reason: int, margin: float = 0.0, clearance: float = -1.0) -> GeofenceEvaluation:
    pose_stamp = pose.pose_stamp_s if pose else 0.0
    pose_age = now_s - pose_stamp if pose else -1.0
    tf_age = now_s - pose.transform_stamp_s if pose else -1.0
    uncertainty = pose.position_std_m if pose and math.isfinite(pose.position_std_m) else -1.0
    return GeofenceEvaluation(sequence, now_s, pose_stamp, policy.frame_id, state, STOP, reason, SOURCE, policy.manifest_id, policy.manifest_sha256,
                              selection.route_id if selection else "", selection.segment_id if selection else "", selection.zone_id if selection else "",
                              pose_age, tf_age, uncertainty, clearance, margin, policy.manifest_sha256)


def evaluate(policy: RouteSafetyPolicy, pose: Optional[PoseSample], selection: Optional[ActiveRouteSelection], now_s: float, sequence: int = 0) -> GeofenceEvaluation:
    """Evaluate the immutable approved intersection; every ambiguity returns STOP."""
    if not isinstance(policy, RouteSafetyPolicy) or not math.isfinite(now_s) or sequence < 0:
        raise ValueError("valid immutable policy, finite time, and nonnegative sequence required")
    if pose is None or selection is None:
        return _stop(policy, pose, selection, now_s, sequence, STATUS_UNKNOWN, REASON_INPUT_UNKNOWN | REASON_GEOFENCE)
    numeric = (pose.x_m, pose.y_m, pose.yaw_rad, pose.pose_stamp_s, pose.status_stamp_s, pose.transform_stamp_s, pose.position_std_m)
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) for value in numeric) or pose.position_std_m < 0:
        return _stop(policy, pose, selection, now_s, sequence, STATUS_UNKNOWN, REASON_INPUT_UNKNOWN | REASON_GEOFENCE)
    if pose.pose_frame_id != policy.frame_id or not pose.transform_valid:
        return _stop(policy, pose, selection, now_s, sequence, STATUS_UNKNOWN, REASON_TF | REASON_GEOFENCE)
    ages = (now_s - pose.pose_stamp_s, now_s - pose.status_stamp_s, now_s - pose.transform_stamp_s)
    if any(age < -FUTURE_TOLERANCE_S for age in ages):
        return _stop(policy, pose, selection, now_s, sequence, STATUS_UNKNOWN, REASON_SENSOR_STALE | REASON_GEOFENCE)
    if ages[0] > policy.pose_ttl_s or ages[1] > policy.status_ttl_s or ages[2] > policy.transform_ttl_s:
        return _stop(policy, pose, selection, now_s, sequence, STATUS_UNKNOWN, REASON_SENSOR_STALE | REASON_GEOFENCE)
    if pose.localization_state != "OK":
        return _stop(policy, pose, selection, now_s, sequence, STATUS_UNKNOWN, REASON_INPUT_UNKNOWN | REASON_GEOFENCE)
    if selection.safety_manifest_sha256 != policy.manifest_sha256:
        return _stop(policy, pose, selection, now_s, sequence, STATUS_MANIFEST_ERROR, REASON_POLICY_MISMATCH | REASON_ROUTE_MANIFEST)
    if selection.map_id != policy.map_id or selection.map_sha256 != policy.map_sha256:
        return _stop(policy, pose, selection, now_s, sequence, STATUS_MANIFEST_ERROR, REASON_MAP_MISMATCH | REASON_GEOFENCE)
    route = policy.route(selection.route_id)
    if route is None or selection.route_manifest_sha256 != route.route_manifest_sha256:
        return _stop(policy, pose, selection, now_s, sequence, STATUS_MANIFEST_ERROR, REASON_ROUTE_MANIFEST | REASON_ROUTE_STATE)
    simulation_clearance = None
    if policy.simulation_only:
        derived = _derive_simulation_selection(route, pose, selection)
        if derived is None:
            return _stop(policy, pose, selection, now_s, sequence, STATUS_OUTSIDE, REASON_ROUTE_STATE | REASON_GEOFENCE)
        selection = derived
    if selection.segment_id not in route.segment_ids:
        return _stop(policy, pose, selection, now_s, sequence, STATUS_OUTSIDE, REASON_ROUTE_STATE | REASON_GEOFENCE)
    if policy.simulation_only:
        zone = policy.zone(selection.zone_id)
        if (zone is None or zone.policy != "normal" or len(route.centerline) < 2
                or len(route.segment_centerlines) != len(route.segment_ids)):
            return _stop(policy, pose, selection, now_s, sequence, STATUS_OUTSIDE, REASON_ROUTE_STATE | REASON_GEOFENCE)
        segment_index = route.segment_ids.index(selection.segment_id)
        first, second = route.segment_centerlines[segment_index]
        center_distance = _distance_point_segment((pose.x_m, pose.y_m), first, second)
        required_tube_margin = policy.fixed_boundary_margin_m + max(
            policy.configured_uncertainty_margin_m, 3.0 * pose.position_std_m
        )
        raw_tube_footprint = transformed_footprint(
            pose.x_m, pose.y_m, pose.yaw_rad,
            policy.footprint_length_m, policy.footprint_width_m,
        )
        expanded_tube = transformed_footprint(
            pose.x_m, pose.y_m, pose.yaw_rad, policy.footprint_length_m,
            policy.footprint_width_m, required_tube_margin,
        )
        raw_footprint_distance = max(
            _nearest_centerline_segment(point, route.centerline)[0]
            for point in raw_tube_footprint
        )
        footprint_distance = max(
            _nearest_centerline_segment(point, route.centerline)[0]
            for point in expanded_tube
        )
        simulation_clearance = route.tube_radius_m - raw_footprint_distance
        if (center_distance > route.corridor_margin_m + 1e-10
                or footprint_distance > route.tube_radius_m + 1e-10):
            state = STATUS_MARGIN if center_distance <= route.corridor_margin_m + 1e-10 else STATUS_OUTSIDE
            return _stop(
                policy, pose, selection, now_s, sequence, state, REASON_GEOFENCE,
                required_tube_margin, simulation_clearance,
            )
    if selection.zone_id:
        if selection.zone_id not in route.zone_ids:
            return _stop(policy, pose, selection, now_s, sequence, STATUS_OUTSIDE, REASON_ROUTE_STATE | REASON_GEOFENCE)
        applicable_zones = (policy.zone(selection.zone_id),)
    else:
        applicable_zones = tuple(
            policy.zone(zone_id) for zone_id in route.zone_ids
            if policy.zone(zone_id) is not None and point_in_polygon((pose.x_m, pose.y_m), policy.zone(zone_id).polygon, True)
        )
    if len(applicable_zones) != 1 or applicable_zones[0] is None or applicable_zones[0].policy != "normal":
        return _stop(policy, pose, selection, now_s, sequence, STATUS_OUTSIDE, REASON_ROUTE_STATE | REASON_GEOFENCE)
    zone = applicable_zones[0]

    required_margin = policy.fixed_boundary_margin_m + max(policy.configured_uncertainty_margin_m, 3.0 * pose.position_std_m) + route.corridor_margin_m
    effective_zone_id = zone.zone_id
    raw_footprint = transformed_footprint(pose.x_m, pose.y_m, pose.yaw_rad, policy.footprint_length_m, policy.footprint_width_m)
    expanded = transformed_footprint(pose.x_m, pose.y_m, pose.yaw_rad, policy.footprint_length_m, policy.footprint_width_m, required_margin)
    forbidden_zone_overlap = any(
        candidate is not None and candidate.policy != "normal" and _polygons_overlap(expanded, candidate.polygon)
        for candidate in (policy.zone(zone_id) for zone_id in route.zone_ids)
    )
    containers = (policy.global_allowed, route.corridor, zone.polygon)
    contained = all(_polygon_strictly_inside(expanded, container) for container in containers)
    excluded = any(_polygons_overlap(expanded, exclusion) for exclusion in policy.exclusions)
    boundaries = containers + policy.exclusions
    clearance = min(_boundary_distance(raw_footprint, boundary) for boundary in boundaries) if boundaries else -1.0
    if simulation_clearance is not None:
        clearance = min(clearance, simulation_clearance)
    if not contained or excluded or forbidden_zone_overlap or clearance + 1e-10 < required_margin:
        state = STATUS_MARGIN if all(_polygon_strictly_inside(raw_footprint, container) for container in containers) and not any(_polygons_overlap(raw_footprint, exclusion) for exclusion in policy.exclusions) else STATUS_OUTSIDE
        return _stop(policy, pose, selection, now_s, sequence, state, REASON_GEOFENCE, required_margin, clearance)
    return GeofenceEvaluation(sequence, now_s, pose.pose_stamp_s, policy.frame_id, STATUS_INSIDE, CLEAR, 0, SOURCE, policy.manifest_id,
                              policy.manifest_sha256, selection.route_id, selection.segment_id, effective_zone_id, ages[0], ages[2],
                              pose.position_std_m, clearance, required_margin, policy.manifest_sha256)


def run_ros_node() -> None:
    """Lazy ROS adapter.  Parameters are snapshotted before immutable policy load."""
    import rospy
    from wheelchair_interfaces.msg import ActiveRoute, GeofenceStatus, LocalizationCandidate, LocalizationStatus, SafetySignal
    from std_msgs.msg import Header

    rospy.init_node("wheelchair_route_safety")
    config_path = Path(rospy.get_param("~config_path")).resolve()
    expected_config_sha256 = rospy.get_param("~expected_config_sha256")
    policy = load_simulation_policy(config_path, expected_config_sha256)
    status_pub = rospy.Publisher("/route_safety/geofence_status", GeofenceStatus, queue_size=1)
    signal_pub = rospy.Publisher("/safety/geofence", SafetySignal, queue_size=1)
    latest: Dict[str, Any] = {"route": None}
    high_water: Dict[str, Any] = {
        "pose": None, "route": None, "status": None, "status_message": None,
        "status_message_valid": False,
    }
    evidence = LocalizationEvidenceBuffer()
    sequence = [0]
    status_message = [0]
    held_pair = [None]


    def receive_status(msg: Any) -> None:
        status_message[0] += 1
        evidence.add_status((msg, rospy.Time.now().to_sec(), status_message[0]))

    rospy.Subscriber(
        "/localization/candidate", LocalizationCandidate, evidence.add_candidate, queue_size=1,
    )
    rospy.Subscriber("/localization/status", LocalizationStatus, receive_status, queue_size=1)
    rospy.Subscriber("/route/active", ActiveRoute, lambda msg: latest.__setitem__("route", msg), queue_size=1)

    def publish(_event: Any) -> None:
        route_msg = latest["route"]
        status_evidence, pose_msg = evidence.snapshot()
        now = rospy.Time.now()
        now_s = now.to_sec()
        selection = None
        if route_msg is not None:
            try:
                route_stamp = float(route_msg.header.stamp.to_sec())
                chronology_ok = high_water["route"] is None or route_stamp >= high_water["route"]
                fresh = -FUTURE_TOLERANCE_S <= now_s - route_stamp <= ACTIVE_ROUTE_TTL_S
                if chronology_ok and fresh:
                    selection = ActiveRouteSelection(
                        route_msg.route_id, route_msg.route_manifest_sha256,
                        route_msg.safety_manifest_sha256, route_msg.map_id,
                        route_msg.map_sha256, "", "", route_msg.mission_id,
                    )
                    high_water["route"] = route_stamp
            except (AttributeError, TypeError, ValueError):
                selection = None
        sample = None
        status_is_acceptable = False
        if status_evidence is not None:
            try:
                localization_msg, receipt_stamp, status_message_id = status_evidence
                status_source_stamp = float(localization_msg.header.stamp.to_sec())
                status_evaluation_stamp = float(localization_msg.evaluation_stamp.to_sec())
                receipt_stamp = float(receipt_stamp)
                status_sequence = int(localization_msg.sequence)
                status_high_water = (
                    status_sequence, status_source_stamp, status_evaluation_stamp, receipt_stamp,
                )
                previous_status = high_water["status"]
                is_new_status_message = status_message_id != high_water["status_message"]
                status_is_newer = status_chronology_is_newer(
                    status_high_water, previous_status,
                )
                if is_new_status_message:
                    if status_is_newer:
                        high_water["status"] = status_high_water
                    high_water["status_message"] = status_message_id
                    high_water["status_message_valid"] = status_is_newer
                status_is_acceptable = (
                    (not is_new_status_message and high_water["status_message_valid"])
                    or (is_new_status_message and status_is_newer)
                )
                if pose_msg is None:
                    if (
                            held_pair[0] is not None
                            and status_is_acceptable
                            and status_allows_pair_hold(
                                localization_msg, receipt_stamp, held_pair[0][1],
                                policy.map_id, policy.map_sha256, policy.frame_id,
                                policy.localization_policy_sha256, LocalizationStatus.OK,
                            )):
                        sample = held_pair[0][0]
                    else:
                        held_pair[0] = None
            except (AttributeError, TypeError, ValueError, OverflowError):
                try:
                    high_water["status_message"] = status_evidence[2]
                except (IndexError, TypeError):
                    pass
                high_water["status_message_valid"] = False
                held_pair[0] = None
        if pose_msg is not None and status_evidence is not None:
            try:
                pose_stamped = pose_msg.pose
                pose_stamp = float(pose_stamped.header.stamp.to_sec())
                covariance = pose_stamped.pose.covariance
                covariance_std = (
                    math.sqrt(max(0.0, max(covariance[0], covariance[7])))
                    if len(covariance) == 36 else float("nan")
                )
                status_std = float(localization_msg.position_std_m)
                position_std = max(covariance_std, status_std)
                q = pose_stamped.pose.pose.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                paired = (
                    status_sequence >= 0
                    and status_is_acceptable
                    and abs(pose_stamp - status_source_stamp) <= 1.0e-9
                    and pose_msg.reset_count == localization_msg.reset_count
                    and pose_msg.map_id == localization_msg.map_id == policy.map_id
                    and pose_msg.map_sha256 == localization_msg.map_sha256 == policy.map_sha256
                    and pose_stamped.header.frame_id == policy.frame_id
                    and localization_msg.header.frame_id == policy.frame_id
                    and localization_msg.policy_sha256 == policy.localization_policy_sha256
                    and localization_msg.state == LocalizationStatus.OK
                    and localization_msg.reason_mask == 0
                    and localization_msg.independent_check_passed
                    and pose_stamp <= status_evaluation_stamp <= receipt_stamp + FUTURE_TOLERANCE_S
                    and (high_water["pose"] is None or pose_stamp >= high_water["pose"])
                )
                transform_age = float(localization_msg.transform_age_s)
                transform_stamp = status_evaluation_stamp - transform_age
                sample = PoseSample(
                    float(pose_stamped.pose.pose.position.x),
                    float(pose_stamped.pose.pose.position.y), yaw,
                    pose_stamp, status_evaluation_stamp, transform_stamp,
                    position_std, "OK" if paired else "NOT_OK",
                    pose_stamped.header.frame_id,
                    paired and math.isfinite(transform_age) and transform_age >= 0.0,
                )
                if paired:
                    high_water["pose"] = pose_stamp
                    held_pair[0] = (sample, pose_msg.reset_count)
                else:
                    held_pair[0] = None
            except (AttributeError, TypeError, ValueError, OverflowError):
                sample = None
                held_pair[0] = None
        sequence[0] += 1
        result = evaluate(policy, sample, selection, now_s, sequence[0])
        status = GeofenceStatus()
        status.header.stamp = rospy.Time.from_sec(result.pose_stamp_s) if result.pose_stamp_s > 0.0 else rospy.Time()
        status.header.frame_id = result.frame_id
        status.evaluation_stamp = now
        status.sequence, status.state, status.reason_mask, status.source = result.sequence, result.state, result.reason_mask, result.source
        status.manifest_id, status.manifest_sha256 = result.manifest_id, result.manifest_sha256
        status.route_id, status.segment_id, status.zone_id = result.route_id, result.segment_id, result.zone_id
        status.pose_age_s, status.transform_age_s = result.pose_age_s, result.transform_age_s
        status.position_uncertainty_m = result.position_uncertainty_m
        status.minimum_signed_clearance_m, status.required_boundary_margin_m = result.minimum_signed_clearance_m, result.required_boundary_margin_m
        signal = SafetySignal()
        signal.header = Header(stamp=now, frame_id=result.frame_id)
        signal.sequence, signal.state, signal.reason_mask, signal.source, signal.policy_sha256 = result.sequence, result.signal_state, result.reason_mask, result.source, result.policy_sha256
        status_pub.publish(status)
        signal_pub.publish(signal)

    rospy.Timer(rospy.Duration(0.05), publish)
    rospy.spin()


if __name__ == "__main__":
    run_ros_node()
