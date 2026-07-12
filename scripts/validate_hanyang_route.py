#!/usr/bin/env python3
"""Validate committed/generated Hanyang map and route candidate artifacts."""

import argparse
import hashlib
import json
import math
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

OCCUPIED_MAX = 50
FREE_MIN = 250


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def canonical_hash(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()



def load_yaml(path):
    if yaml is None:
        raise ValueError("PyYAML is required to validate YAML artifacts")
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def read_pgm(path):
    data = Path(path).read_bytes()
    offset = 0
    tokens = []
    while len(tokens) < 4:
        while offset < len(data) and chr(data[offset]).isspace():
            offset += 1
        if offset < len(data) and data[offset] == ord("#"):
            while offset < len(data) and data[offset] not in (10, 13):
                offset += 1
            continue
        start = offset
        while offset < len(data) and not chr(data[offset]).isspace():
            offset += 1
        if start == offset:
            raise ValueError("truncated PGM header")
        tokens.append(data[start:offset])
    magic, width, height, maximum = tokens
    if magic not in (b"P5", b"P2") or int(maximum) != 255:
        raise ValueError("PGM must be P5/P2 with max value 255")
    width, height = int(width), int(height)
    while offset < len(data) and chr(data[offset]).isspace():
        offset += 1
    if magic == b"P5":
        pixels = list(data[offset:])
    else:
        pixels = [int(item) for item in data[offset:].split()]
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise ValueError("PGM dimensions do not match pixel payload")
    return width, height, pixels


def extract_routes(document):
    if "outbound_route" in document and "return_route" in document:
        result = []
        for name in ("outbound_route", "return_route"):
            route = document[name]
            points = [(p["x_m"], p["y_m"], None) for p in route.get("waypoints", [])]
            margins = [s.get("corridor_margin_m") for s in route.get("segments", [])]
            result.append((name, points, margins))
        return result
    legacy = document.get("waypoints", {})
    points = [(p.get("x"), p.get("y"), p.get("z")) for p in legacy.get("poses", [])]
    return [("legacy_loop", points, [])]


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    rank = (len(values) - 1) * fraction
    low, high = int(math.floor(rank)), int(math.ceil(rank))
    if low == high:
        return values[low]
    return values[low] * (high - rank) + values[high] * (rank - low)


def grade_from_poses(points, minimum_distance=0.2):
    values = []
    for first, second in zip(points, points[1:]):
        if first[2] is None or second[2] is None:
            continue
        dx, dy, dz = second[0] - first[0], second[1] - first[1], second[2] - first[2]
        horizontal = math.hypot(dx, dy)
        if horizontal >= minimum_distance:
            values.append(100.0 * abs(dz) / horizontal)
    if not values:
        return None
    return {"max_grade_percent": max(values), "p95_grade_percent": percentile(values, 0.95),
            "mean_grade_percent": sum(values) / len(values), "sample_count": len(values)}


def validate(map_yaml_path, route_path, metadata_path=None, required_margin=0.0):
    errors, warnings = [], []
    map_document = load_yaml(map_yaml_path)
    route_document = load_yaml(route_path)
    pgm_path = Path(map_yaml_path).parent / map_document.get("image", "")
    if not pgm_path.is_file():
        raise ValueError("map YAML image does not exist")
    width, height, pixels = read_pgm(pgm_path)
    resolution = map_document.get("resolution")
    origin = map_document.get("origin")
    if not finite_number(resolution) or resolution <= 0 or not isinstance(origin, list) or len(origin) < 2 or not all(finite_number(v) for v in origin[:2]):
        errors.append("E_MAP_GEOMETRY: resolution/origin invalid")
        resolution, origin = 1.0, [0.0, 0.0]
    if map_document.get("mode", "trinary") != "trinary" or map_document.get("negate", 0) != 0:
        errors.append("E_MAP_SEMANTICS: expected trinary negate=0")
    routes = extract_routes(route_document)
    if len(routes) == 1:
        errors.append(
            "E_LEGACY_ROUTE_SCHEMA: regenerate with explicit outbound_route/return_route "
            "and corridor margins before candidate qualification")
    else:
        outbound_end = routes[0][1][-1] if routes[0][1] else None
        return_start = routes[1][1][0] if routes[1][1] else None
        if outbound_end is None or return_start is None or math.hypot(outbound_end[0] - return_start[0], outbound_end[1] - return_start[1]) > 1e-9:
            errors.append("E_SPLIT_ENDPOINT: outbound endpoint and return start differ")
        if route_document.get("status") != "candidate" or route_document.get("provenance", {}).get("surveyed") is not False:
            errors.append("E_QUALIFICATION: generated route must remain unsurveyed candidate")
        bound_hash = route_document.get("map", {}).get("sha256")
        if bound_hash != digest(pgm_path):
            errors.append("E_MAP_HASH: route map binding differs from PGM content")
        for key, direction in (("outbound_route", "outbound"), ("return_route", "return")):
            route = route_document.get(key, {})
            if route.get("direction") != direction:
                errors.append(f"E_DIRECTION: {key}")
            content = {name: route.get(name) for name in ("route_id", "direction", "waypoints", "segments")}
            if route.get("route_manifest_sha256") != canonical_hash(content):
                errors.append(f"E_ROUTE_HASH: {key}")
    all_points = []
    margin_failures = 0
    occupied_failures = 0
    unknown_failures = 0
    for route_name, points, margins in routes:
        if len(points) < 2:
            errors.append(f"E_ROUTE_EMPTY: {route_name}")
            continue
        if not all(finite_number(v) for point in points for v in point[:2]):
            errors.append(f"E_NONFINITE_POSE: {route_name}")
            continue
        if margins and (not all(finite_number(v) and v >= 0 for v in margins) or min(margins) < required_margin):
            errors.append(f"E_CORRIDOR_MARGIN: {route_name}")
        if not margins:
            warnings.append(f"W_NO_CORRIDOR_MARGIN: {route_name}")
        all_points.extend(points)
        radius = int(math.ceil(required_margin / resolution))
        for index, point in enumerate(points):
            x = int(math.floor((point[0] - origin[0]) / resolution))
            y = int(math.floor((point[1] - origin[1]) / resolution))
            row = height - 1 - y
            if x < 0 or row < 0 or x >= width or row >= height:
                errors.append(f"E_POSE_OUTSIDE_MAP: {route_name}[{index}]")
                continue
            pixel = pixels[row * width + x]
            if pixel <= OCCUPIED_MAX:
                occupied_failures += 1
            elif pixel < FREE_MIN:
                unknown_failures += 1
            margin_failed = False
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy > radius * radius:
                        continue
                    xx, yy = x + dx, row - dy
                    if (xx < 0 or yy < 0 or xx >= width or yy >= height
                            or pixels[yy * width + xx] < FREE_MIN):
                        margin_failed = True
                        break
                if margin_failed:
                    break
            if margin_failed:
                margin_failures += 1
    if occupied_failures:
        errors.append(f"E_POSE_OCCUPIED: {occupied_failures} poses")
    if unknown_failures:
        errors.append(
            f"E_POSE_NOT_FREE: {unknown_failures} poses lack recorded free-space evidence; "
            "regenerate the map with trajectory-corridor clearing")
    if margin_failures:
        errors.append(f"E_CORRIDOR_OCCUPANCY: {margin_failures} poses lack {required_margin:g} m free margin")
    closure = None
    if all_points and len(routes) == 1:
        closure = math.hypot(all_points[-1][0] - all_points[0][0], all_points[-1][1] - all_points[0][1])
        if route_document.get("waypoints", {}).get("closed_loop") and closure > 0.5:
            errors.append(
                f"E_LOOP_RESIDUAL: {closure:.6f} m exceeds candidate target 0.5 m; "
                "select and verify an explicit outbound/return split")
    computed_grade = grade_from_poses(all_points)
    metadata = None
    if metadata_path and Path(metadata_path).is_file():
        metadata = json.loads(Path(metadata_path).read_text(encoding="utf-8"))
        grid_meta = metadata.get("grid", metadata)
        expected_width = grid_meta.get("width", grid_meta.get("grid_width"))
        expected_height = grid_meta.get("height", grid_meta.get("grid_height"))
        if expected_width != width or expected_height != height:
            errors.append("E_METADATA_DIMENSIONS: metadata and PGM differ")
        hashes = metadata.get("hashes", {})
        if hashes and hashes.get("pgm_sha256") != digest(pgm_path):
            errors.append("E_METADATA_HASH: PGM hash differs")
        if hashes and hashes.get("map_yaml_sha256") != digest(map_yaml_path):
            errors.append("E_METADATA_HASH: map YAML hash differs")
        if hashes and hashes.get("route_sha256") != digest(route_path):
            errors.append("E_METADATA_HASH: route hash differs")
        if not hashes:
            errors.append(
                "E_MISSING_CONTENT_HASHES: regenerate metadata with PGM, map YAML, "
                "route, and source SHA-256 bindings")
        stated = metadata.get("grade")
        if stated and stated.get("formula") not in (None, "100*abs(dz)/sqrt(dx^2+dy^2)"):
            warnings.append("W_SUSPICIOUS_GRADE: unsupported grade formula")
        if stated and computed_grade:
            for key in ("max_grade_percent", "p95_grade_percent", "mean_grade_percent"):
                if key in stated and abs(stated[key] - computed_grade[key]) > max(0.05, 0.02 * computed_grade[key]):
                    warnings.append(f"W_SUSPICIOUS_GRADE: {key} metadata={stated[key]:.6g} computed={computed_grade[key]:.6g}")
    qualified = not errors and len(routes) == 2 and not warnings
    return {
        "valid": not errors,
        "candidate_qualified": qualified,
        "physically_qualified": False,
        "surveyed": False,
        "approved": False,
        "map": {"pgm": str(pgm_path), "width": width, "height": height,
                "resolution": resolution, "origin": origin[:2], "sha256": digest(pgm_path)},
        "route": {"path": str(route_path), "sha256": digest(route_path),
                  "pose_count": len(all_points), "loop_position_residual_m": closure},
        "grade_recomputed": computed_grade,
        "errors": errors,
        "warnings": warnings,
        "limitations": ["candidate evidence is not a survey or approval",
                        "no odometry, TF, command, mode, driver, or ground-truth evidence"],
    }


def parser():
    value = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1] / "data" / "hanyang_aegimun_loop"
    value.add_argument("--map-yaml", default=str(root / "map.yaml"))
    value.add_argument("--route", default=str(root / "hanyang_aegimun_loop.waypoints.yaml"))
    value.add_argument("--metadata", default=str(root / "map.metadata.json"))
    value.add_argument("--required-margin", type=float, default=0.0)
    return value


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        report = validate(args.map_yaml, args.route, args.metadata, args.required_margin)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise SystemExit("validation failed: " + str(exc))
    print(json.dumps(report, sort_keys=True))
    if not report["valid"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
