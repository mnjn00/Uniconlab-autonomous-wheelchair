#!/usr/bin/env python3
"""Deterministically export GLIM dump/array inputs as a ROS map and A05 route.

This ROS-free tool reads immutable numeric arrays or raw GLIM submaps. Outputs are
candidate evidence and never confer surveyed, hardware, or passenger authority.
"""

import argparse
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

import numpy as np

UNKNOWN, OCCUPIED, FREE = 205, 0, 254
MAX_GRID_CELLS = 100_000_000


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def canonical_hash(value):
    return sha256_bytes(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode())


def load_array(path):
    path = Path(path)
    if path.suffix == ".npy":
        result = np.load(str(path), allow_pickle=False)
    elif path.suffix == ".npz":
        archive = np.load(str(path), allow_pickle=False)
        if len(archive.files) != 1:
            raise ValueError("NPZ input must contain exactly one array")
        result = archive[archive.files[0]]
    else:
        result = np.loadtxt(str(path), dtype=np.float64, ndmin=2)
    return np.asarray(result, dtype=np.float64)


def parse_glim_transform(path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    try:
        start = next(index for index, line in enumerate(lines)
                     if line.strip().rstrip(":") == "T_world_origin") + 1
    except StopIteration as exc:
        raise ValueError(f"{path} has no T_world_origin transform") from exc
    if start + 4 > len(lines):
        raise ValueError(f"{path} has a truncated T_world_origin transform")
    try:
        transform = np.array([[float(value) for value in lines[index].split()]
                              for index in range(start, start + 4)], dtype=np.float64)
    except ValueError as exc:
        raise ValueError(f"{path} has a nonnumeric T_world_origin transform") from exc
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f"{path} T_world_origin must be a finite 4x4 matrix")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise ValueError(f"{path} T_world_origin is not homogeneous")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-5):
        raise ValueError(f"{path} T_world_origin rotation is invalid")
    return transform


def load_glim_dump(path):
    root = Path(path)
    if not root.is_dir():
        raise ValueError(f"GLIM dump directory does not exist: {root}")
    submaps = sorted(item for item in root.iterdir()
                     if item.is_dir() and (item / "points_compact.bin").is_file())
    if not submaps:
        raise ValueError(f"{root} contains no submaps with points_compact.bin")
    clouds = []
    for submap in submaps:
        data_path = submap / "data.txt"
        if not data_path.is_file():
            raise ValueError(f"{submap} has points_compact.bin but no data.txt")
        raw = np.fromfile(str(submap / "points_compact.bin"), dtype="<f4")
        if not len(raw) or len(raw) % 3:
            raise ValueError(f"{submap / 'points_compact.bin'} must contain float32 xyz triples")
        local = raw.reshape((-1, 3)).astype(np.float64)
        if not np.isfinite(local).all():
            raise ValueError(f"{submap / 'points_compact.bin'} contains nonfinite coordinates")
        transform = parse_glim_transform(data_path)
        clouds.append(local @ transform[:3, :3].T + transform[:3, 3])
    return np.vstack(clouds), submaps


def load_trajectory(path, trajectory_has_time=False):
    trajectory = load_array(path)
    if trajectory.ndim == 2 and trajectory.shape[1] == 8:
        if not np.isfinite(trajectory).all():
            raise ValueError("TUM trajectory values must be finite")
        quaternion_norm = np.linalg.norm(trajectory[:, 4:8], axis=1)
        if np.any(quaternion_norm < 1e-12) or not np.allclose(quaternion_norm, 1.0, atol=1e-5):
            raise ValueError("TUM trajectory quaternions must be finite unit quaternions")
        return trajectory[:, :4], True, "tum"
    return trajectory, trajectory_has_time, "time-xyz" if trajectory_has_time else "xyz"


def hash_files(paths):
    digest = hashlib.sha256()
    for path in sorted((Path(item) for item in paths), key=lambda item: (item.parent.name, item.name)):
        digest.update(f"{path.parent.name}/{path.name}".encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
    return digest.hexdigest()


def validate_inputs(cloud, trajectory, trajectory_has_time=False):
    if cloud.ndim != 2 or cloud.shape[1] < 3 or len(cloud) == 0:
        raise ValueError("point cloud must be a nonempty Nx3-or-wider array")
    expected = 4 if trajectory_has_time else 3
    if trajectory.ndim != 2 or trajectory.shape[1] < expected or len(trajectory) < 2:
        raise ValueError("trajectory must contain at least two poses")
    if not np.isfinite(cloud[:, :3]).all() or not np.isfinite(trajectory[:, :expected]).all():
        raise ValueError("coordinates and timestamps must be finite")
    xyz = trajectory[:, 1:4] if trajectory_has_time else trajectory[:, :3]
    if trajectory_has_time and np.any(np.diff(trajectory[:, 0]) <= 0):
        raise ValueError("trajectory timestamps must be strictly increasing in input order")
    if np.any(np.linalg.norm(np.diff(xyz, axis=0), axis=1) == 0):
        raise ValueError("trajectory contains consecutive duplicate poses")
    return cloud[:, :3].copy(), xyz.copy()


def gravity_basis(gravity):
    gravity = np.asarray(gravity, dtype=np.float64)
    if gravity.shape != (3,) or not np.isfinite(gravity).all() or np.linalg.norm(gravity) < 1e-9:
        raise ValueError("gravity must be a finite nonzero vector")
    up = -gravity / np.linalg.norm(gravity)
    reference = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(reference, up)) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    east = reference - np.dot(reference, up) * up
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    return np.stack((east, north, up), axis=1)


def project(points, basis):
    return np.asarray(points) @ basis


def nearest_ground_heights(points_xy, trajectory, chunk_size=4096):
    result = np.empty(len(points_xy), dtype=np.float64)
    ground_route = trajectory
    if len(trajectory) > 2000:
        total_length = float(np.sum(np.linalg.norm(np.diff(trajectory[:, :3], axis=0), axis=1)))
        ground_route = resample_polyline(trajectory, max(0.25, total_length / 1000.0))
    route_xy = ground_route[:, :2]
    for start in range(0, len(points_xy), chunk_size):
        query = points_xy[start:start + chunk_size]
        distances = np.sum((query[:, None, :] - route_xy[None, :, :]) ** 2, axis=2)
        result[start:start + len(query)] = ground_route[np.argmin(distances, axis=1), 2]
    return result


def resample_polyline(points, spacing):
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("route spacing must be finite and positive")
    lengths = np.linalg.norm(np.diff(points[:, :3], axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    targets = np.arange(0.0, cumulative[-1], spacing)
    targets = np.concatenate((targets, [cumulative[-1]]))
    output = np.column_stack([np.interp(targets, cumulative, points[:, column]) for column in range(3)])
    return output


def waypoint_records(points):
    delta = np.diff(points[:, :2], axis=0)
    yaw = np.arctan2(delta[:, 1], delta[:, 0])
    yaw = np.concatenate((yaw, [yaw[-1]]))
    return [{"x_m": float(p[0]), "y_m": float(p[1]), "yaw_rad": float(y)} for p, y in zip(points, yaw)]


def grade_statistics(points, minimum_distance=0.2):
    if not math.isfinite(minimum_distance) or minimum_distance <= 0.0:
        raise ValueError("grade sample minimum distance must be finite and positive")
    delta = np.diff(points[:, :3], axis=0)
    horizontal_steps = np.hypot(delta[:, 0], delta[:, 1])
    cumulative = np.concatenate(([0.0], np.cumsum(horizontal_steps)))
    full_intervals = int(math.floor(float(cumulative[-1]) / minimum_distance))
    if full_intervals < 1:
        raise ValueError("trajectory is too short for the grade sample minimum distance")
    targets = np.arange(full_intervals + 1, dtype=np.float64) * minimum_distance
    sampled_z = np.interp(targets, cumulative, points[:, 2])
    grades = 100.0 * np.abs(np.diff(sampled_z)) / minimum_distance
    return {
        "formula": "100*abs(dz)/sqrt(dx^2+dy^2)",
        "sample_min_horizontal_distance_m": float(minimum_distance),
        "sample_count": int(len(grades)),
        "z_min_m": float(np.min(points[:, 2])),
        "z_max_m": float(np.max(points[:, 2])),
        "max_grade_percent": float(np.max(grades)),
        "p95_grade_percent": float(np.percentile(grades, 95)),
        "mean_grade_percent": float(np.mean(grades)),
    }


def disk_offsets(radius_cells):
    values = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                values.append((dx, dy))
    return values


def mark_disks(grid, cells, radius, value, preserve_occupied=False):
    height, width = grid.shape
    for x, y in cells:
        for dx, dy in disk_offsets(radius):
            xx, yy = x + dx, y + dy
            if 0 <= xx < width and 0 <= yy < height and (not preserve_occupied or grid[yy, xx] != OCCUPIED):
                grid[yy, xx] = value


def build_grid(cloud, trajectory, resolution, padding, obstacle_min, obstacle_max,
               projection_mode, footprint_width, footprint_length, margin):
    numeric = (resolution, padding, obstacle_min, obstacle_max, footprint_width, footprint_length, margin)
    if not all(math.isfinite(v) for v in numeric) or resolution <= 0 or padding < 0:
        raise ValueError("grid parameters must be finite with positive resolution and nonnegative padding")
    if obstacle_min < 0 or obstacle_max <= obstacle_min:
        raise ValueError("obstacle height band must satisfy 0 <= min < max")
    if footprint_width <= 0 or footprint_length <= 0 or margin < 0:
        raise ValueError("footprint dimensions must be positive and clearance margin nonnegative")
    combined = np.vstack((cloud[:, :2], trajectory[:, :2]))
    low = np.floor((np.min(combined, axis=0) - padding) / resolution) * resolution
    high = np.ceil((np.max(combined, axis=0) + padding) / resolution) * resolution
    width, height = np.ceil((high - low) / resolution).astype(int) + 1
    if width < 1 or height < 1 or int(width) * int(height) > MAX_GRID_CELLS:
        raise ValueError("grid bounds exceed deterministic candidate limits")
    cells = np.floor((cloud[:, :2] - low) / resolution).astype(int)
    dense_route = resample_polyline(trajectory, resolution / 2.0)
    route_cells = np.unique(
        np.floor((dense_route[:, :2] - low) / resolution).astype(int), axis=0)
    relative_z = cloud[:, 2]
    if projection_mode == "ground-relative":
        relative_z = cloud[:, 2] - nearest_ground_heights(cloud[:, :2], dense_route)
    elif projection_mode != "z-band":
        raise ValueError("unsupported projection mode")
    obstacle = (relative_z >= obstacle_min) & (relative_z <= obstacle_max)
    ground = relative_z < obstacle_min
    grid = np.full((height, width), UNKNOWN, dtype=np.uint8)
    ground_cells = cells[ground]
    if len(ground_cells):
        ground_unique = np.unique(ground_cells, axis=0)
    else:
        ground_unique = np.empty((0, 2), dtype=cells.dtype)
    for x, y in ground_unique:
        if 0 <= x < width and 0 <= y < height:
            grid[y, x] = FREE
    obstacle_cells = cells[obstacle]
    if len(obstacle_cells):
        occupied_cells = np.unique(obstacle_cells, axis=0)
    else:
        occupied_cells = np.empty((0, 2), dtype=cells.dtype)
    for x, y in occupied_cells:
        if 0 <= x < width and 0 <= y < height:
            grid[y, x] = OCCUPIED
    inflation_radius = max(
        0, int(math.ceil((max(footprint_width, footprint_length) / 2.0 + margin) / resolution)))
    mark_disks(grid, occupied_cells, inflation_radius, OCCUPIED)
    occupied_before_clear = grid == OCCUPIED
    mark_disks(grid, route_cells, inflation_radius, FREE)
    cleared_occupied_count = int(np.count_nonzero(occupied_before_clear & (grid == FREE)))
    return (grid, low, route_cells, int(len(occupied_cells)), inflation_radius,
            cleared_occupied_count)


def pgm_bytes(grid):
    height, width = grid.shape
    return f"P5\n{width} {height}\n255\n".encode("ascii") + np.flipud(grid).tobytes(order="C")


def make_route_manifest(trajectory, split_index, spacing, map_hash, source_hash,
                        safety_hash, corridor_margin):
    if split_index <= 0 or split_index >= len(trajectory) - 1:
        raise ValueError("split index must identify an explicit interior endpoint")
    outbound_points = resample_polyline(trajectory[:split_index + 1], spacing)
    return_points = resample_polyline(trajectory[split_index:], spacing)
    if len(outbound_points) > 65536 or len(return_points) > 65536:
        raise ValueError("resampled route exceeds A05's 65,536-waypoint candidate limit")
    outbound = {"route_id": "hanyang-aegimun-outbound", "direction": "outbound",
                "waypoints": waypoint_records(outbound_points), "segments": []}
    returning = {"route_id": "hanyang-aegimun-return", "direction": "return",
                 "waypoints": waypoint_records(return_points), "segments": []}
    for route in (outbound, returning):
        route["segments"] = [{
            "segment_id": route["route_id"] + "-seg-000",
            "start_waypoint_index": 0,
            "end_waypoint_index": len(route["waypoints"]) - 1,
            "corridor_margin_m": float(corridor_margin),
            "zone_ids": ["candidate-unsurveyed"],
            "max_linear_mps": 0.0,
            "max_angular_rps": 0.0,
            "hardware_authorized": False,
        }]
        route["route_manifest_sha256"] = canonical_hash({k: route[k] for k in ("route_id", "direction", "waypoints", "segments")})
    return {
        "schema_version": 1,
        "manifest_id": "hanyang-aegimun-generated-candidate-v1",
        "owner": "WP2-offline-map-export",
        "reviewer": "UNREVIEWED",
        "status": "candidate",
        "provenance": {"source_path": "offline-trajectory-array", "source_sha256": source_hash,
                       "evidence_level": "candidate", "surveyed": False},
        "immutable": True,
        "map": {"map_id": "hanyang_aegimun_loop", "sha256": map_hash, "frame_id": "map"},
        "safety_manifest_sha256": safety_hash,
        "geometry_semantics": {"coordinate_frame": "map", "linear_unit": "m", "angular_unit": "rad",
                               "point_order": "ordered_centerline", "unknown_geometry_action": "STOP",
                               "nonfinite_geometry_action": "REJECT_AND_STOP"},
        "outbound_route": outbound,
        "return_route": returning,
    }


def check_route_alignment(manifest, grid, origin, resolution, required_margin):
    errors = []
    clearance_cells = int(math.ceil(required_margin / resolution))
    for route_name in ("outbound_route", "return_route"):
        for index, pose in enumerate(manifest[route_name]["waypoints"]):
            x = int(math.floor((pose["x_m"] - origin[0]) / resolution))
            y = int(math.floor((pose["y_m"] - origin[1]) / resolution))
            if x < 0 or y < 0 or y >= grid.shape[0] or x >= grid.shape[1]:
                errors.append(f"{route_name}[{index}] is outside map")
                continue
            for dx, dy in disk_offsets(clearance_cells):
                xx, yy = x + dx, y + dy
                if (xx < 0 or yy < 0 or yy >= grid.shape[0] or xx >= grid.shape[1]
                        or grid[yy, xx] != FREE):
                    errors.append(f"{route_name}[{index}] lacks {required_margin:.3f} m free corridor")
                    break
    return errors


def export(args):
    if args.footprint_source not in ("measured", "simulation"):
        raise ValueError("footprint source must be measured or simulation")
    dump_path = getattr(args, "glim_dump", None)
    cloud_path = getattr(args, "cloud", None)
    if bool(dump_path) == bool(cloud_path):
        raise ValueError("select exactly one of cloud array or GLIM dump directory")
    trajectory_path = getattr(args, "trajectory", None)
    if not trajectory_path and dump_path:
        trajectory_path = str(Path(dump_path) / "traj_lidar.txt")
    if not trajectory_path:
        raise ValueError("trajectory is required (or must exist as GLIM dump traj_lidar.txt)")
    if dump_path:
        cloud_raw, submaps = load_glim_dump(dump_path)
        cloud_sources = [path / name for path in submaps
                         for name in ("data.txt", "points_compact.bin")]
        cloud_source_hash = hash_files(cloud_sources)
        cloud_source_label = "glim-dump-directory"
    else:
        cloud_raw = load_array(cloud_path)
        cloud_source_hash = sha256_bytes(Path(cloud_path).read_bytes())
        cloud_source_label = "offline-cloud-array"
    trajectory_raw, trajectory_has_time, trajectory_format = load_trajectory(
        trajectory_path, getattr(args, "trajectory_has_time", False))
    cloud, trajectory = validate_inputs(cloud_raw, trajectory_raw, trajectory_has_time)
    basis = gravity_basis(args.gravity)
    cloud, trajectory = project(cloud, basis), project(trajectory, basis)
    (grid, origin, _, occupied_count, inflation_cells,
     cleared_occupied_count) = build_grid(
        cloud, trajectory, args.resolution, args.padding, args.obstacle_min_height,
        args.obstacle_max_height, args.projection_mode, args.footprint_width,
        args.footprint_length, args.clearance_margin)
    pgm = pgm_bytes(grid)
    map_hash = sha256_bytes(pgm)
    source_hash = sha256_bytes(Path(trajectory_path).read_bytes())
    grade = grade_statistics(trajectory, args.grade_min_distance)
    route = make_route_manifest(trajectory, args.split_index, args.route_spacing, map_hash,
                                source_hash, args.safety_manifest_sha256,
                                args.clearance_margin)
    route["provenance"]["source_path"] = (
        "glim-dump/traj_lidar.txt" if dump_path and not getattr(args, "trajectory", None)
        else "offline-trajectory-array")
    alignment_errors = check_route_alignment(route, grid, origin, args.resolution,
                                             args.clearance_margin)
    if alignment_errors:
        raise ValueError("route/map misalignment: " + "; ".join(alignment_errors[:8]))
    route_bytes = (json.dumps(route, sort_keys=True, indent=2, allow_nan=False) + "\n").encode()
    yaml_text = (f"image: {args.map_name}.pgm\nmode: trinary\nresolution: {args.resolution:.12g}\n"
                 f"origin: [{origin[0]:.12g}, {origin[1]:.12g}, 0.0]\nnegate: 0\n"
                 "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    yaml_bytes = yaml_text.encode()
    loop_residual = float(np.linalg.norm(trajectory[-1, :2] - trajectory[0, :2]))
    metadata = {
        "schema_version": 1, "qualification": "candidate", "surveyed": False,
        "hardware_motion_authorized": False, "passenger_operation_authorized": False,
        "source": {"cloud": cloud_source_label, "trajectory_format": trajectory_format},
        "projection": {"mode": args.projection_mode, "gravity": list(args.gravity),
                       "obstacle_height_band_m": [args.obstacle_min_height, args.obstacle_max_height]},
        "footprint": {"source": args.footprint_source, "width_m": args.footprint_width,
                      "length_m": args.footprint_length, "clearance_margin_m": args.clearance_margin,
                      "inflation_cells": inflation_cells},
        "grid": {"width": int(grid.shape[1]), "height": int(grid.shape[0]),
                 "resolution": args.resolution, "origin": [float(origin[0]), float(origin[1]), 0.0],
                 "occupied_source_cells": occupied_count,
                 "cleared_occupied_cells_in_recorded_corridor": cleared_occupied_count,
                 "semantics": {"occupied": 0, "unknown": 205, "free": 254}},
        "grade": grade,
        "loop_closure": {"position_residual_m": loop_residual,
                         "target_m": 0.5, "target_met": loop_residual <= 0.5,
                         "is_consistency_only": True},
        "candidate_limits": {"max_grid_cells": MAX_GRID_CELLS, "max_route_waypoints_per_direction": 65536,
                             "loop_position_residual_target_m": 0.5},
        "candidate_qualification": {"route_map_aligned": True, "loop_target_met": loop_residual <= 0.5,
                                    "physically_qualified": False, "surveyed": False, "approved": False},
        "hashes": {"pgm_sha256": map_hash, "map_yaml_sha256": sha256_bytes(yaml_bytes),
                   "route_sha256": sha256_bytes(route_bytes),
                   "cloud_source_sha256": cloud_source_hash,
                   "trajectory_source_sha256": source_hash},
        "limitations": ["recorded trajectory proves only a traversed candidate corridor",
                        "map clearing is navigation preprocessing, not survey or physical safety approval",
                        "no odometry, TF, command, mode, driver, or ground-truth evidence"],
    }
    metadata_bytes = (json.dumps(metadata, sort_keys=True, indent=2, allow_nan=False) + "\n").encode()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    artifacts = {args.map_name + ".pgm": pgm, args.map_name + ".yaml": yaml_bytes,
                 args.route_name + ".yaml": route_bytes, args.map_name + ".metadata.json": metadata_bytes}
    staged, backups, installed = [], [], []
    try:
        for name, content in artifacts.items():
            handle, temporary = tempfile.mkstemp(prefix="." + name + ".", dir=str(output))
            with os.fdopen(handle, "wb") as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            staged.append((Path(temporary), output / name))
        for _, final in staged:
            if final.exists():
                handle, backup = tempfile.mkstemp(prefix="." + final.name + ".backup.", dir=str(output))
                os.close(handle)
                Path(backup).unlink()
                os.replace(str(final), backup)
                backups.append((Path(backup), final))
        for temporary, final in staged:
            os.replace(str(temporary), str(final))
            installed.append(final)
        for backup, _ in backups:
            backup.unlink()
    except Exception:
        for final in installed:
            if final.exists():
                final.unlink()
        for backup, final in backups:
            if backup.exists():
                os.replace(str(backup), str(final))
        raise
    finally:
        for temporary, _ in staged:
            if temporary.exists():
                temporary.unlink()
        for backup, _ in backups:
            if backup.exists():
                backup.unlink()
    return metadata


def parser():
    value = argparse.ArgumentParser(description=__doc__)
    source = value.add_mutually_exclusive_group(required=True)
    source.add_argument("--cloud", help="numeric point-cloud array")
    source.add_argument("--glim-dump", help="GLIM dump containing sorted submap directories")
    value.add_argument("--trajectory", help="xyz/time-xyz/TUM trajectory; defaults to dump traj_lidar.txt")
    value.add_argument("--trajectory-has-time", action="store_true", help="trajectory columns are time,x,y,z")
    value.add_argument("--output-dir", required=True)
    value.add_argument("--map-name", default="map")
    value.add_argument("--route-name", default="hanyang_routes")
    value.add_argument("--resolution", type=float, default=0.10)
    value.add_argument("--padding", type=float, default=2.0)
    value.add_argument("--gravity", type=float, nargs=3, default=(0.0, 0.0, -1.0))
    value.add_argument("--projection-mode", choices=("z-band", "ground-relative"), default="ground-relative")
    value.add_argument("--obstacle-min-height", type=float, default=0.15)
    value.add_argument("--obstacle-max-height", type=float, default=2.0)
    value.add_argument("--footprint-source", choices=("measured", "simulation"), required=True)
    value.add_argument("--footprint-width", type=float, required=True)
    value.add_argument("--footprint-length", type=float, required=True)
    value.add_argument("--clearance-margin", type=float, default=0.0)
    value.add_argument("--route-spacing", type=float, default=1.0)
    value.add_argument("--split-index", type=int, required=True)
    value.add_argument("--grade-min-distance", type=float, default=0.2)
    value.add_argument("--safety-manifest-sha256", default="0" * 64)
    return value


def main(argv=None):
    args = parser().parse_args(argv)
    if len(args.safety_manifest_sha256) != 64 or any(c not in "0123456789abcdef" for c in args.safety_manifest_sha256):
        raise SystemExit("safety manifest hash must be 64 lowercase hexadecimal characters")
    try:
        report = export(args)
    except (OSError, ValueError) as exc:
        raise SystemExit("export failed: " + str(exc))
    print(json.dumps(report, sort_keys=True))


if __name__ == "__main__":
    main()
