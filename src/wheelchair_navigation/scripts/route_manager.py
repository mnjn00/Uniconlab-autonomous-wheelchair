#!/usr/bin/env python3
"""Strict, ROS-independent route loading and progress tracking.

Routes are frozen build-time products.  This module never reverses a route,
generates geometry, publishes velocity, or makes a geofence decision.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


SHA256_LEN = 64
DIRECTIONS = ("outbound", "return")
QUALIFICATIONS = ("candidate", "simulation_qualified", "closed_course_qualified", "campus_approved")
BEHAVIORS = ("proceed", "yield", "stop", "terminal_stop")
TAGS = ("none", "candidate", "unknown")
WHEELCHAIR_HALF_WIDTH_M = 0.30
LOCALIZATION_MARGIN_M = 0.25
FIXED_BOUNDARY_MARGIN_M = 0.20
REQUIRED_CORRIDOR_CLEARANCE_M = WHEELCHAIR_HALF_WIDTH_M + LOCALIZATION_MARGIN_M + FIXED_BOUNDARY_MARGIN_M


class RouteValidationError(ValueError):
    """The immutable route manifest or one of its bound assets is invalid."""


def _object(value: Any, name: str, required: Iterable[str], allowed: Iterable[str]) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise RouteValidationError("%s must be an object" % name)
    required_set, allowed_set = set(required), set(allowed)
    missing = required_set - set(value)
    unknown = set(value) - allowed_set
    if missing:
        raise RouteValidationError("%s missing fields: %s" % (name, sorted(missing)))
    if unknown:
        raise RouteValidationError("%s has unknown fields: %s" % (name, sorted(unknown)))
    return value


def _finite(value: Any, name: str, minimum: float = -1e6, maximum: float = 1e6) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RouteValidationError("%s must be numeric" % name)
    result = float(value)
    if not math.isfinite(result) or result < minimum or result > maximum:
        raise RouteValidationError("%s is outside its finite bounds" % name)
    return result


def _identifier(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value or len(value) > 64:
        raise RouteValidationError("%s is not an identifier" % name)
    if not value[0].isalnum() or any(not (c.isalnum() or c in "._-") for c in value):
        raise RouteValidationError("%s is not an identifier" % name)
    return value


def _sha(value: Any, name: str) -> str:
    if not isinstance(value, str) or len(value) != SHA256_LEN or any(c not in "0123456789abcdef" for c in value):
        raise RouteValidationError("%s must be a lowercase SHA-256" % name)
    return value


def canonical_sha256(value: Mapping[str, Any], omitted_key: str) -> str:
    payload = {key: item for key, item in value.items() if key != omitted_key}
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()

def canonical_route_sha256(route: Mapping[str, Any], map_id: str, map_sha256: str) -> str:
    """Bind one directional route's complete semantics to its runtime map."""
    payload = {
        "map_id": map_id,
        "map_sha256": map_sha256,
        "route": {key: item for key, item in route.items() if key != "route_manifest_sha256"},
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _segments_intersect(first: Waypoint, second: Waypoint, third: Waypoint, fourth: Waypoint) -> bool:
    """Return whether two non-adjacent closed centerline segments intersect."""
    def cross(origin: Waypoint, left: Waypoint, right: Waypoint) -> float:
        return ((left.x_m - origin.x_m) * (right.y_m - origin.y_m)
                - (left.y_m - origin.y_m) * (right.x_m - origin.x_m))

    def between(first_point: Waypoint, point: Waypoint, second_point: Waypoint) -> bool:
        return (min(first_point.x_m, second_point.x_m) <= point.x_m <= max(first_point.x_m, second_point.x_m)
                and min(first_point.y_m, second_point.y_m) <= point.y_m <= max(first_point.y_m, second_point.y_m))

    first_cross, second_cross = cross(first, second, third), cross(first, second, fourth)
    third_cross, fourth_cross = cross(third, fourth, first), cross(third, fourth, second)
    if first_cross == 0.0 and between(first, third, second):
        return True
    if second_cross == 0.0 and between(first, fourth, second):
        return True
    if third_cross == 0.0 and between(third, first, fourth):
        return True
    if fourth_cross == 0.0 and between(third, second, fourth):
        return True
    return ((first_cross > 0.0) != (second_cross > 0.0)
            and (third_cross > 0.0) != (fourth_cross > 0.0))


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
def _load_yaml_strict(path: Path, label: str) -> Any:
    """Load YAML without accepting duplicate-key resealing tricks."""
    try:
        import yaml
    except ImportError as exc:
        raise RouteValidationError("PyYAML is required to load route manifests") from exc

    class StrictLoader(yaml.SafeLoader):
        pass

    def construct_mapping(loader: Any, node: Any, deep: bool = False) -> Mapping[str, Any]:
        mapping = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in mapping:
                raise RouteValidationError("%s has a duplicate key: %r" % (label, key))
            mapping[key] = loader.construct_object(value_node, deep=deep)
        return mapping

    StrictLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping)
    try:
        return yaml.load(path.read_text(encoding="utf-8"), Loader=StrictLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise RouteValidationError("cannot read %s: %s" % (label, exc)) from exc


@dataclass(frozen=True)
class _OccupancyGrid:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    occupied_threshold: float
    free_threshold: float
    negate: int
    pixels: bytes

    def _cell(self, x_m: float, y_m: float) -> Tuple[int, int]:
        column = int(math.floor((x_m - self.origin_x_m) / self.resolution_m))
        row_from_bottom = int(math.floor((y_m - self.origin_y_m) / self.resolution_m))
        return column, self.height - 1 - row_from_bottom

    def is_free(self, x_m: float, y_m: float) -> bool:
        column, row = self._cell(x_m, y_m)
        if column < 0 or column >= self.width or row < 0 or row >= self.height:
            return False
        value = self.pixels[row * self.width + column]
        occupancy = (value if self.negate else 255 - value) / 255.0
        return occupancy < self.free_threshold


def _pgm_tokens(data: bytes) -> Tuple[List[bytes], int]:
    tokens: List[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if index < len(data) and data[index:index + 1] == b"#":
            while index < len(data) and data[index:index + 1] not in (b"\r", b"\n"):
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        if start == index:
            raise RouteValidationError("map PGM header is truncated")
        tokens.append(data[start:index])
    return tokens, index


def _load_occupancy_grid(map_yaml_path: Path, map_pgm_path: Path) -> _OccupancyGrid:
    map_value = _load_yaml_strict(map_yaml_path, "map YAML")
    fields = ("image", "mode", "resolution", "origin", "negate", "occupied_thresh", "free_thresh")
    value = _object(map_value, "map YAML", fields, fields)
    if value["mode"] != "trinary" or value["image"] != map_pgm_path.name:
        raise RouteValidationError("map YAML image or mode mismatch")
    origin = value["origin"]
    if not isinstance(origin, list) or len(origin) != 3:
        raise RouteValidationError("map YAML origin must be a three-element list")
    resolution = _finite(value["resolution"], "map YAML resolution", 1e-6, 100.0)
    origin_x, origin_y = _finite(origin[0], "map YAML origin.x"), _finite(origin[1], "map YAML origin.y")
    if _finite(origin[2], "map YAML origin.yaw", -1e-9, 1e-9) != 0.0:
        raise RouteValidationError("rotated map origins are unsupported")
    negate = value["negate"]
    if isinstance(negate, bool) or negate not in (0, 1):
        raise RouteValidationError("map YAML negate must be zero or one")
    occupied, free = _finite(value["occupied_thresh"], "map YAML occupied threshold", 0.0, 1.0), _finite(value["free_thresh"], "map YAML free threshold", 0.0, 1.0)
    if free >= occupied:
        raise RouteValidationError("map YAML thresholds are invalid")
    try:
        data = map_pgm_path.read_bytes()
    except OSError as exc:
        raise RouteValidationError("cannot read map PGM: %s" % exc) from exc
    tokens, offset = _pgm_tokens(data)
    if tokens[0] != b"P5":
        raise RouteValidationError("map PGM must be binary P5")
    try:
        width, height, maximum = (int(token) for token in tokens[1:])
    except ValueError as exc:
        raise RouteValidationError("map PGM header is invalid") from exc
    if width <= 0 or height <= 0 or maximum != 255:
        raise RouteValidationError("map PGM dimensions or depth are invalid")
    if offset >= len(data) or not data[offset:offset + 1].isspace():
        raise RouteValidationError("map PGM header is not terminated")
    offset += 1
    pixels = data[offset:]
    if len(pixels) != width * height:
        raise RouteValidationError("map PGM payload size mismatch")
    return _OccupancyGrid(width, height, resolution, origin_x, origin_y, occupied, free, negate, pixels)


def _validate_source_route(raw: Any, direction: str, route: Route, indexes: Sequence[int]) -> None:
    fields = ("direction", "route_id", "route_manifest_sha256", "segments", "waypoints")
    value = _object(raw, "waypoint source " + direction, fields, fields)
    if value["direction"] != direction or not isinstance(value["waypoints"], list) or len(value["waypoints"]) != len(indexes):
        raise RouteValidationError("%s waypoint source direction/index mismatch" % direction)
    for index, (source, waypoint) in enumerate(zip(value["waypoints"], route.waypoints)):
        source_value = _object(source, "waypoint source %s[%d]" % (direction, index),
                               ("x_m", "y_m", "yaw_rad"), ("x_m", "y_m", "yaw_rad"))
        if (_finite(source_value["x_m"], "source x"), _finite(source_value["y_m"], "source y"),
                _finite(source_value["yaw_rad"], "source yaw", -math.pi, math.pi)) != (waypoint.x_m, waypoint.y_m, waypoint.yaw_rad):
            raise RouteValidationError("%s waypoint %d disagrees with pinned source" % (direction, index))
    segments = value["segments"]
    if not isinstance(segments, list) or len(segments) != 1:
        raise RouteValidationError("%s source must define one complete directional segment" % direction)
    source_segment = _object(segments[0], "waypoint source " + direction + " segment",
                             ("corridor_margin_m", "end_waypoint_index", "hardware_authorized", "max_angular_rps", "max_linear_mps", "segment_id", "start_waypoint_index", "zone_ids"),
                             ("corridor_margin_m", "end_waypoint_index", "hardware_authorized", "max_angular_rps", "max_linear_mps", "segment_id", "start_waypoint_index", "zone_ids"))
    if (source_segment["start_waypoint_index"] != 0
            or source_segment["end_waypoint_index"] != len(value["waypoints"]) - 1):
        raise RouteValidationError("%s source segment does not cover its local route" % direction)
    source_margin = _finite(source_segment["corridor_margin_m"], "source corridor margin", 1e-6, 100.0)
    source_zones = source_segment["zone_ids"]
    if source_segment["hardware_authorized"] is not False or not isinstance(source_zones, list):
        raise RouteValidationError("%s source segment authority is invalid" % direction)
    for segment in route.segments:
        if segment.corridor_margin_m != source_margin or list(segment.zone_ids) != source_zones or segment.hardware_authorized:
            raise RouteValidationError("%s segment semantics disagree with pinned source" % direction)


def _validate_route_occupancy(route: Route, grid: _OccupancyGrid) -> None:
    """Require free centerline and the declared fixed corridor; unknown is never free."""
    for segment, first, second in zip(route.segments, route.waypoints, route.waypoints[1:]):
        length = math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
        samples = max(1, int(math.ceil(length / (grid.resolution_m / 2.0))))
        normal_x, normal_y = -(second.y_m - first.y_m) / length, (second.x_m - first.x_m) / length
        lateral_steps = int(math.ceil(max(segment.corridor_margin_m, REQUIRED_CORRIDOR_CLEARANCE_M) / (grid.resolution_m / 2.0)))
        for sample in range(samples + 1):
            fraction = sample / samples
            center_x = first.x_m + fraction * (second.x_m - first.x_m)
            center_y = first.y_m + fraction * (second.y_m - first.y_m)
            for lateral in range(-lateral_steps, lateral_steps + 1):
                offset = lateral * grid.resolution_m / 2.0
                if not grid.is_free(center_x + normal_x * offset, center_y + normal_y * offset):
                    raise RouteValidationError("%s crosses occupied, unknown, or off-map space" % segment.segment_id)


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    behavior: str
    goal_tolerance_m: float
    slope_tag: str
    crossing_tag: str
    visibility_tag: str


@dataclass(frozen=True)
class Segment:
    segment_id: str
    start_waypoint_index: int
    end_waypoint_index: int
    corridor_margin_m: float
    corridor_width_m: float
    zone_ids: Tuple[str, ...]
    max_linear_mps: float
    max_angular_rps: float
    hardware_authorized: bool
    behavior: str
    slope_tag: str
    crossing_tag: str


@dataclass(frozen=True)
class Route:
    route_id: str
    direction: str
    route_manifest_sha256: str
    qualification: str
    waypoints: Tuple[Waypoint, ...]
    segments: Tuple[Segment, ...]
    cumulative_m: Tuple[float, ...]

    @property
    def length_m(self) -> float:
        return self.cumulative_m[-1]


@dataclass(frozen=True)
class RouteManifest:
    manifest_id: str
    map_id: str
    map_sha256: str
    map_pgm_sha256: str
    waypoint_asset_sha256: str
    safety_manifest_sha256: str
    content_sha256: str
    status: str
    routes: Mapping[str, Route]

    def route(self, direction: str) -> Route:
        try:
            return self.routes[direction]
        except KeyError:
            raise RouteValidationError("unsupported direction %r" % direction)


def _validate_waypoint(raw: Any, name: str) -> Waypoint:
    fields = ("waypoint_id", "x_m", "y_m", "yaw_rad", "behavior", "goal_tolerance_m", "slope_tag", "crossing_tag", "visibility_tag")
    value = _object(raw, name, fields, fields)
    behavior = value["behavior"]
    tags = (value["slope_tag"], value["crossing_tag"], value["visibility_tag"])
    if behavior not in BEHAVIORS or any(tag not in TAGS for tag in tags):
        raise RouteValidationError("%s has unsupported behavior/tag" % name)
    return Waypoint(_identifier(value["waypoint_id"], name + ".waypoint_id"),
                    _finite(value["x_m"], name + ".x_m"),
                    _finite(value["y_m"], name + ".y_m"),
                    _finite(value["yaw_rad"], name + ".yaw_rad", -math.pi, math.pi),
                    behavior, _finite(value["goal_tolerance_m"], name + ".goal_tolerance_m", 0.05, 5.0),
                    tags[0], tags[1], tags[2])


def _validate_segment(raw: Any, name: str, waypoint_count: int) -> Segment:
    fields = ("segment_id", "start_waypoint_index", "end_waypoint_index", "corridor_margin_m", "corridor_width_m", "zone_ids", "max_linear_mps", "max_angular_rps", "hardware_authorized", "behavior", "slope_tag", "crossing_tag")
    value = _object(raw, name, fields, fields)
    start, end = value["start_waypoint_index"], value["end_waypoint_index"]
    if isinstance(start, bool) or isinstance(end, bool) or not isinstance(start, int) or not isinstance(end, int):
        raise RouteValidationError("%s waypoint indexes must be integers" % name)
    if start < 0 or end != start + 1 or end >= waypoint_count:
        raise RouteValidationError("%s must connect consecutive, in-range waypoints" % name)
    zones = value["zone_ids"]
    if not isinstance(zones, list) or not zones or len(zones) != len(set(zones)):
        raise RouteValidationError("%s.zone_ids must be a nonempty unique list" % name)
    margin = _finite(value["corridor_margin_m"], name + ".corridor_margin_m", 1e-6, 100.0)
    width = _finite(value["corridor_width_m"], name + ".corridor_width_m", 1e-6, 200.0)
    if abs(width - 2.0 * margin) > 1e-9:
        raise RouteValidationError("%s corridor width must equal twice its margin" % name)
    if value["hardware_authorized"] is not False:
        raise RouteValidationError("%s cannot authorize hardware" % name)
    behavior, slope, crossing = value["behavior"], value["slope_tag"], value["crossing_tag"]
    if behavior not in BEHAVIORS or slope not in TAGS or crossing not in TAGS:
        raise RouteValidationError("%s has unsupported behavior/tag" % name)
    linear = _finite(value["max_linear_mps"], name + ".max_linear_mps", 0.0, 0.70)
    angular = _finite(value["max_angular_rps"], name + ".max_angular_rps", 0.0, 1.0)
    if linear <= 0.0 or angular <= 0.0:
        raise RouteValidationError("%s speed policy must be active" % name)
    return Segment(_identifier(value["segment_id"], name + ".segment_id"), start, end, margin, width,
                   tuple(_identifier(z, name + ".zone_ids") for z in zones),
                   linear, angular, False, behavior, slope, crossing)


def _validate_route(raw: Any, direction: str, map_id: str, map_sha256: str) -> Route:
    fields = ("route_id", "direction", "route_manifest_sha256", "qualification", "waypoints", "segments")
    value = _object(raw, direction + "_route", fields, fields)
    if value["direction"] != direction:
        raise RouteValidationError("route direction mismatch")
    if value["qualification"] not in QUALIFICATIONS:
        raise RouteValidationError("unsupported route qualification")
    if _sha(value["route_manifest_sha256"], "route_manifest_sha256") != canonical_route_sha256(value, map_id, map_sha256):
        raise RouteValidationError("%s route semantic hash mismatch" % direction)
    raw_waypoints = value["waypoints"]
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
        raise RouteValidationError("%s needs at least two waypoints" % direction)
    waypoints = tuple(_validate_waypoint(w, "%s.waypoints[%d]" % (direction, i)) for i, w in enumerate(raw_waypoints))
    if len({w.waypoint_id for w in waypoints}) != len(waypoints):
        raise RouteValidationError("duplicate waypoint_id")
    if len({(w.x_m, w.y_m) for w in waypoints}) != len(waypoints):
        raise RouteValidationError("duplicate waypoint geometry")
    raw_segments = value["segments"]
    if not isinstance(raw_segments, list) or len(raw_segments) != len(waypoints) - 1:
        raise RouteValidationError("segments must cover every waypoint pair")
    segments = tuple(_validate_segment(s, "%s.segments[%d]" % (direction, i), len(waypoints)) for i, s in enumerate(raw_segments))
    if tuple((s.start_waypoint_index, s.end_waypoint_index) for s in segments) != tuple((i, i + 1) for i in range(len(segments))):
        raise RouteValidationError("segments are missing or out of order")
    if len({s.segment_id for s in segments}) != len(segments):
        raise RouteValidationError("duplicate segment_id")
    if (any(w.behavior == "terminal_stop" for w in waypoints[:-1])
            or any(s.behavior == "terminal_stop" for s in segments[:-1])
            or waypoints[-1].behavior != "terminal_stop"
            or segments[-1].behavior != "terminal_stop"):
        raise RouteValidationError("route must have one explicit terminal stop")
    if value["qualification"] != "candidate" and any(
            tag != "none" for waypoint in waypoints for tag in
            (waypoint.slope_tag, waypoint.crossing_tag, waypoint.visibility_tag)):
        raise RouteValidationError("unknown route evidence requires candidate qualification")
    if value["qualification"] != "candidate" and any(
            tag != "none" for segment in segments for tag in (segment.slope_tag, segment.crossing_tag)):
        raise RouteValidationError("unknown route evidence requires candidate qualification")
    cumulative = [0.0]
    for first, second in zip(waypoints, waypoints[1:]):
        distance = math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
        if distance <= 1e-6:
            raise RouteValidationError("zero-length route segment")
        cumulative.append(cumulative[-1] + distance)
    for first_index, (first, second) in enumerate(zip(waypoints, waypoints[1:])):
        for third, fourth in zip(waypoints[first_index + 2:], waypoints[first_index + 3:]):
            if _segments_intersect(first, second, third, fourth):
                raise RouteValidationError("route centerline self-intersects")
    for index, (first, second) in enumerate(zip(waypoints, waypoints[1:])):
        expected_yaw = math.atan2(second.y_m - first.y_m, second.x_m - first.x_m)
        yaw_error = math.atan2(math.sin(first.yaw_rad - expected_yaw), math.cos(first.yaw_rad - expected_yaw))
        if abs(yaw_error) > 1e-5:
            raise RouteValidationError("%s waypoint %d yaw was not recalculated from geometry" % (direction, index))
    terminal_expected = math.atan2(waypoints[-1].y_m - waypoints[-2].y_m, waypoints[-1].x_m - waypoints[-2].x_m)
    terminal_error = math.atan2(math.sin(waypoints[-1].yaw_rad - terminal_expected), math.cos(waypoints[-1].yaw_rad - terminal_expected))
    if abs(terminal_error) > 1e-5:
        raise RouteValidationError("%s terminal yaw was not recalculated from geometry" % direction)
    return Route(_identifier(value["route_id"], "route_id"), direction, value["route_manifest_sha256"], value["qualification"], waypoints, segments, tuple(cumulative))


def load_manifest(path: str, verify_assets: bool = True) -> RouteManifest:
    """Load a strict frozen manifest. ROS is intentionally not imported here."""
    manifest_path = Path(path).resolve()
    raw = _load_yaml_strict(manifest_path, "route manifest")
    fields = ("schema_version", "manifest_id", "owner", "reviewer", "status", "provenance", "immutable", "map", "waypoint_asset", "safety_manifest_sha256", "geometry_semantics", "generation", "outbound_route", "return_route", "content_sha256")
    value = _object(raw, "manifest", fields, fields)
    if value["schema_version"] != 1 or value["immutable"] is not True or value["status"] not in QUALIFICATIONS:
        raise RouteValidationError("unsupported schema, mutability, or status")
    provenance = _object(value["provenance"], "provenance", ("source_path", "source_sha256", "evidence_level", "surveyed"), ("source_path", "source_sha256", "evidence_level", "surveyed"))
    if provenance["evidence_level"] not in ("candidate", "simulation_only", "closed_course", "campus_survey") or not isinstance(provenance["surveyed"], bool):
        raise RouteValidationError("invalid provenance")
    _sha(provenance["source_sha256"], "provenance.source_sha256")
    map_value = _object(value["map"], "map", ("map_id", "sha256", "frame_id", "yaml_path", "yaml_sha256", "pgm_path", "pgm_sha256"), ("map_id", "sha256", "frame_id", "yaml_path", "yaml_sha256", "pgm_path", "pgm_sha256"))
    if map_value["frame_id"] != "map":
        raise RouteValidationError("map frame must be map")
    if _sha(map_value["sha256"], "map.sha256") != _sha(map_value["pgm_sha256"], "map.pgm_sha256"):
        raise RouteValidationError("runtime map binding must be the PGM hash")
    waypoint_asset = _object(value["waypoint_asset"], "waypoint_asset", ("path", "sha256"), ("path", "sha256"))
    geometry = _object(value["geometry_semantics"], "geometry_semantics", ("coordinate_frame", "linear_unit", "angular_unit", "point_order", "unknown_geometry_action", "nonfinite_geometry_action"), ("coordinate_frame", "linear_unit", "angular_unit", "point_order", "unknown_geometry_action", "nonfinite_geometry_action"))
    if tuple(geometry.values()) != ("map", "m", "rad", "ordered_centerline", "STOP", "REJECT_AND_STOP"):
        raise RouteValidationError("geometry semantics mismatch")
    generation_fields = ("method", "source_closed_loop", "split_source_index", "split_endpoint", "sampling_source_indexes", "yaw_method", "runtime_generation_allowed")
    generation = _object(value["generation"], "generation", generation_fields, generation_fields)
    if generation["method"] != "build_time_config_generation" or generation["source_closed_loop"] is not True or generation["runtime_generation_allowed"] is not False or generation["yaw_method"] != "atan2_to_next_terminal_from_previous":
        raise RouteValidationError("route generation provenance is not frozen")
    split = _object(generation["split_endpoint"], "generation.split_endpoint", ("x_m", "y_m"), ("x_m", "y_m"))
    split_x, split_y = _finite(split["x_m"], "generation.split_endpoint.x_m"), _finite(split["y_m"], "generation.split_endpoint.y_m")
    samples = _object(generation["sampling_source_indexes"], "generation.sampling_source_indexes", DIRECTIONS, DIRECTIONS)
    if isinstance(generation["split_source_index"], bool) or not isinstance(generation["split_source_index"], int) or generation["split_source_index"] < 1:
        raise RouteValidationError("split_source_index must be a positive integer")
    for direction in DIRECTIONS:
        indexes = samples[direction]
        if not isinstance(indexes, list) or len(indexes) < 2 or any(isinstance(i, bool) or not isinstance(i, int) or i < 0 for i in indexes) or indexes != sorted(indexes):
            raise RouteValidationError("%s source indexes must be ordered integers" % direction)
    if canonical_sha256(value, "content_sha256") != _sha(value["content_sha256"], "content_sha256"):
        raise RouteValidationError("manifest content hash mismatch")
    if verify_assets:
        for relative, expected, label in ((map_value["yaml_path"], map_value["yaml_sha256"], "map YAML"), (map_value["pgm_path"], map_value["pgm_sha256"], "map PGM"), (waypoint_asset["path"], waypoint_asset["sha256"], "waypoint")):
            asset = (manifest_path.parent / relative).resolve()
            if not asset.is_file() or _file_sha256(asset) != _sha(expected, label + " hash"):
                raise RouteValidationError(label + " asset hash mismatch")
    routes = {direction: _validate_route(value[direction + "_route"], direction,
                                         _identifier(map_value["map_id"], "map.map_id"),
                                         _sha(map_value["sha256"], "map.sha256"))
              for direction in DIRECTIONS}
    map_yaml_path = (manifest_path.parent / map_value["yaml_path"]).resolve()
    map_pgm_path = (manifest_path.parent / map_value["pgm_path"]).resolve()
    source_path = (manifest_path.parent / waypoint_asset["path"]).resolve()
    source = _load_yaml_strict(source_path, "waypoint source")
    source_fields = ("geometry_semantics", "immutable", "manifest_id", "map", "outbound_route",
                     "owner", "provenance", "return_route", "reviewer", "safety_manifest_sha256",
                     "schema_version", "status")
    source_value = _object(source, "waypoint source", source_fields, source_fields)
    source_geometry = _object(source_value["geometry_semantics"], "waypoint source geometry",
                              ("angular_unit", "coordinate_frame", "linear_unit",
                               "nonfinite_geometry_action", "point_order", "unknown_geometry_action"),
                              ("angular_unit", "coordinate_frame", "linear_unit",
                               "nonfinite_geometry_action", "point_order", "unknown_geometry_action"))
    if (source_value["schema_version"] != 1 or source_value["immutable"] is not True
            or source_value["status"] != "candidate"
            or tuple(source_geometry.values()) != ("rad", "map", "m", "REJECT_AND_STOP",
                                                   "ordered_centerline", "STOP")):
        raise RouteValidationError("waypoint source schema or geometry semantics mismatch")
    _identifier(source_value["manifest_id"], "waypoint source manifest_id")
    if not isinstance(source_value["owner"], str) or not source_value["owner"]:
        raise RouteValidationError("waypoint source owner is invalid")
    if not isinstance(source_value["reviewer"], str) or not source_value["reviewer"]:
        raise RouteValidationError("waypoint source reviewer is invalid")
    _sha(source_value["safety_manifest_sha256"], "waypoint source safety manifest hash")
    source_provenance = _object(source_value["provenance"], "waypoint source provenance",
                                ("evidence_level", "source_path", "source_sha256", "surveyed"),
                                ("evidence_level", "source_path", "source_sha256", "surveyed"))
    if (source_provenance["evidence_level"] != "candidate"
            or source_provenance["surveyed"] is not False
            or not isinstance(source_provenance["source_path"], str)
            or not source_provenance["source_path"]):
        raise RouteValidationError("waypoint source provenance is invalid")
    _sha(source_provenance["source_sha256"], "waypoint source provenance hash")
    source_map = _object(source_value["map"], "waypoint source map", ("frame_id", "map_id", "sha256"), ("frame_id", "map_id", "sha256"))
    if (source_map["frame_id"], source_map["map_id"], _sha(source_map["sha256"], "waypoint source map hash")) != (
            "map", map_value["map_id"], _sha(map_value["sha256"], "map.sha256")):
        raise RouteValidationError("waypoint source map binding mismatch")
    for direction in DIRECTIONS:
        _validate_source_route(source_value[direction + "_route"], direction, routes[direction], samples[direction])
    indexed_source_points = list(samples["outbound"]) + list(samples["return"])
    if indexed_source_points != list(range(sum(len(routes[direction].waypoints) for direction in DIRECTIONS))):
        raise RouteValidationError("global source waypoint indexes must be contiguous, unique, and in range")
    grid = _load_occupancy_grid(map_yaml_path, map_pgm_path)
    for direction in DIRECTIONS:
        _validate_route_occupancy(routes[direction], grid)
    if (not provenance["surveyed"] or provenance["evidence_level"] in ("candidate", "simulation_only")) and (
            value["status"] != "candidate" or any(route.qualification != "candidate" for route in routes.values())):
        raise RouteValidationError("unknown route qualification evidence requires candidate status")
    if routes["outbound"].route_id == routes["return"].route_id:
        raise RouteValidationError("outbound and return route IDs must differ")
    if len(samples["outbound"]) != len(routes["outbound"].waypoints) or len(samples["return"]) != len(routes["return"].waypoints):
        raise RouteValidationError("source index and frozen waypoint counts differ")
    outbound_terminal, return_start = routes["outbound"].waypoints[-1], routes["return"].waypoints[0]
    if (outbound_terminal.x_m, outbound_terminal.y_m) != (split_x, split_y) or (return_start.x_m, return_start.y_m) != (split_x, split_y):
        raise RouteValidationError("explicit split endpoint does not bind both directions")
    return RouteManifest(_identifier(value["manifest_id"], "manifest_id"), _identifier(map_value["map_id"], "map_id"),
                         _sha(map_value["sha256"], "map.sha256"), _sha(map_value["pgm_sha256"], "map.pgm_sha256"),
                         _sha(waypoint_asset["sha256"], "waypoint_asset.sha256"), _sha(value["safety_manifest_sha256"], "safety_manifest_sha256"),
                         value["content_sha256"], value["status"], routes)


@dataclass(frozen=True)
class Progress:
    state: str
    segment_id: str = ""
    waypoint_index: int = 0
    along_track_m: float = 0.0
    cross_track_error_m: float = 0.0
    distance_remaining_m: float = 0.0
    fault: str = ""


class ProgressTracker:
    """Geometric progress with bounded acquisition and no dead reckoning."""

    def __init__(self, route: Route, acquisition_limit_m: float = 2.0, loss_limit_m: float = 3.0,
                 regression_tolerance_m: float = 0.5, max_reacquire_segments: int = 2) -> None:
        self.route = route
        self.acquisition_limit_m = _finite(acquisition_limit_m, "acquisition_limit_m", 0.01, 100.0)
        self.loss_limit_m = _finite(loss_limit_m, "loss_limit_m", self.acquisition_limit_m, 100.0)
        self.regression_tolerance_m = _finite(regression_tolerance_m, "regression_tolerance_m", 0.0, 100.0)
        self.max_reacquire_segments = int(max_reacquire_segments)
        self._segment_index: Optional[int] = None
        self._along_m = 0.0
        self._waypoint_index = 0

    def inactive(self, fault: str) -> Progress:
        self._segment_index = None
        return Progress("INACTIVE", fault=fault)

    def update(self, x_m: float, y_m: float) -> Progress:
        x, y = _finite(x_m, "pose.x_m"), _finite(y_m, "pose.y_m")
        if self._segment_index is None:
            indexes = range(len(self.route.segments))
            limit = self.acquisition_limit_m
        else:
            low = max(0, self._segment_index - 1)
            high = min(len(self.route.segments), self._segment_index + self.max_reacquire_segments + 1)
            indexes = range(low, high)
            limit = self.loss_limit_m
        best = None
        for index in indexes:
            a, b = self.route.waypoints[index], self.route.waypoints[index + 1]
            dx, dy = b.x_m - a.x_m, b.y_m - a.y_m
            length_sq = dx * dx + dy * dy
            fraction = max(0.0, min(1.0, ((x - a.x_m) * dx + (y - a.y_m) * dy) / length_sq))
            px, py = a.x_m + fraction * dx, a.y_m + fraction * dy
            cross = math.hypot(x - px, y - py)
            along = self.route.cumulative_m[index] + fraction * math.sqrt(length_sq)
            candidate = (cross, -along, index, along)
            if best is None or candidate < best:
                best = candidate
        assert best is not None
        cross, _, index, along = best
        if cross > limit:
            return self.inactive("CROSS_TRACK_LOST")
        if self._segment_index is not None and along + self.regression_tolerance_m < self._along_m:
            return self.inactive("PROGRESS_REGRESSION")
        along = max(along, self._along_m)
        self._along_m, self._segment_index = along, index
        while self._waypoint_index + 1 < len(self.route.waypoints):
            next_index = self._waypoint_index + 1
            tolerance = self.route.waypoints[next_index].goal_tolerance_m
            if along + tolerance < self.route.cumulative_m[next_index]:
                break
            self._waypoint_index = next_index
        remaining = max(0.0, self.route.length_m - along)
        terminal = self.route.waypoints[-1]
        at_terminal = math.hypot(x - terminal.x_m, y - terminal.y_m) <= terminal.goal_tolerance_m
        state = "COMPLETE" if at_terminal and self._waypoint_index == len(self.route.waypoints) - 1 else "ACTIVE"
        return Progress(state, self.route.segments[index].segment_id, self._waypoint_index, along, cross, remaining)


class RouteManagerNode:
    """Thin lazy ROS adapter; publishes progress only and has no motion/safety publishers."""

    def __init__(self) -> None:
        import rospy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from wheelchair_interfaces.msg import ActiveRoute, RouteProgress
        self._rospy, self._RouteProgress = rospy, RouteProgress
        manifest_path = rospy.get_param("~manifest")
        self._manifest = load_manifest(manifest_path, verify_assets=True)
        self._tracker: Optional[ProgressTracker] = None
        self._mission_id = ""
        self._sequence = 0
        self._activation_sequence: Optional[int] = None
        self._activation_identity: Optional[Tuple[Any, ...]] = None
        self._publisher = rospy.Publisher("/route/progress", RouteProgress, queue_size=1)
        rospy.Subscriber("/route/active", ActiveRoute, self._active_callback, queue_size=1)
        rospy.Subscriber("/localization/pose", PoseWithCovarianceStamped, self._pose_callback, queue_size=1)

    def _publish(self, progress: Progress, route_id: str = "") -> None:
        msg = self._RouteProgress()
        msg.header.stamp = self._rospy.Time.now()
        msg.header.frame_id = "map"
        self._sequence += 1
        msg.sequence = self._sequence
        msg.state = getattr(self._RouteProgress, progress.state)
        msg.mission_id, msg.route_id, msg.map_id = self._mission_id, route_id, self._manifest.map_id
        msg.segment_id, msg.waypoint_index = progress.segment_id, progress.waypoint_index
        msg.along_track_m, msg.cross_track_error_m, msg.distance_remaining_m = progress.along_track_m, progress.cross_track_error_m, progress.distance_remaining_m
        self._publisher.publish(msg)

    def _active_callback(self, msg: Any) -> None:
        direction = {msg.DIRECTION_OUTBOUND: "outbound", msg.DIRECTION_RETURN: "return"}.get(msg.direction)
        identity = (
            msg.mission_id, msg.direction, msg.route_id, msg.map_id, msg.map_sha256,
            msg.route_manifest_sha256, msg.safety_manifest_sha256,
        )
        try:
            route = self._manifest.route(direction or "")
            if (msg.route_id, msg.map_id, msg.map_sha256, msg.route_manifest_sha256, msg.safety_manifest_sha256) != (route.route_id, self._manifest.map_id, self._manifest.map_sha256, route.route_manifest_sha256, self._manifest.safety_manifest_sha256):
                raise RouteValidationError("active route binding mismatch")
            activation_sequence = msg.activation_sequence
            if (isinstance(activation_sequence, bool)
                    or not isinstance(activation_sequence, int)
                    or activation_sequence <= 0):
                raise RouteValidationError("activation sequence must be a positive integer")
            if self._activation_sequence is not None:
                if activation_sequence < self._activation_sequence:
                    raise RouteValidationError("activation sequence regressed")
                if activation_sequence == self._activation_sequence:
                    if identity != self._activation_identity:
                        raise RouteValidationError("activation heartbeat identity changed")
                    return
        except RouteValidationError as exc:
            self._tracker = None
            self._mission_id = msg.mission_id
            self._rospy.logerr("route activation rejected: %s", exc)
            self._publish(Progress("INVALID", fault=str(exc)), msg.route_id)
            return
        self._mission_id = msg.mission_id
        self._activation_sequence = activation_sequence
        self._activation_identity = identity
        self._tracker = ProgressTracker(route)
        self._publish(Progress("INACTIVE"), route.route_id)
    def _pose_callback(self, msg: Any) -> None:
        if self._tracker is None:
            return
        position = msg.pose.pose.position
        progress = self._tracker.update(position.x, position.y)
        self._publish(progress, self._tracker.route.route_id)
        if progress.state == "INACTIVE":
            self._rospy.logerr("route progress lost: %s", progress.fault)
            self._tracker = None


def main() -> None:
    import rospy
    rospy.init_node("route_manager")
    RouteManagerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
