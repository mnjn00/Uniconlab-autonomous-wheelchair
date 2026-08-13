#!/usr/bin/env python3
"""Plan a preferred route inside an authoritative drivable mask."""

from __future__ import annotations

import argparse
import hashlib
import heapq
import json
import math
from pathlib import Path
from typing import Final

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.signal import savgol_filter

FREE: Final = 254
CHAIR_HALF_WIDTH_M: Final = 0.35
PATH_STEP_M: Final = 0.2
BAND_STEP_M: Final = 0.5
PREFERENCE_WEIGHT: Final = 4.0
BOUNDARY_WEIGHT: Final = 2.0
BOUNDARY_SCALE_M: Final = 0.5
SMOOTH_WINDOW: Final = 15
SMOOTH_PASSES: Final = 2


def plan_preferred_path(
    drivable: np.ndarray,
    preferred: np.ndarray,
    start_rc: tuple[int, int],
    goal_rc: tuple[int, int],
    resolution_m: float,
) -> np.ndarray:
    """A* inside ``drivable`` with preference and boundary costs."""
    clearance = ndimage.distance_transform_edt(drivable) * resolution_m
    preference_distance = ndimage.distance_transform_edt(~preferred) * resolution_m
    cost = (
        1.0
        + PREFERENCE_WEIGHT * preference_distance
        + BOUNDARY_WEIGHT * np.exp(-clearance / BOUNDARY_SCALE_M)
    )
    height, width = drivable.shape
    goal = np.asarray(goal_rc, dtype=float)
    queue: list[tuple[float, float, int, int]] = [
        (float(np.linalg.norm(np.asarray(start_rc) - goal)), 0.0, *start_rc)
    ]
    best = np.full(drivable.shape, np.inf, dtype=float)
    best[start_rc] = 0.0
    parent_row = np.full(drivable.shape, -1, dtype=np.int32)
    parent_col = np.full(drivable.shape, -1, dtype=np.int32)
    directions = (
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    )
    reached = False
    while queue:
        _, current_cost, row, col = heapq.heappop(queue)
        if current_cost > best[row, col] + 1e-12:
            continue
        if (row, col) == goal_rc:
            reached = True
            break
        for d_row, d_col in directions:
            next_row, next_col = row + d_row, col + d_col
            if not (0 <= next_row < height and 0 <= next_col < width):
                continue
            if not drivable[next_row, next_col]:
                continue
            if d_row and d_col and (
                not drivable[row + d_row, col]
                or not drivable[row, col + d_col]
            ):
                continue
            step_m = resolution_m * math.hypot(d_row, d_col)
            next_cost = current_cost + step_m * (
                cost[row, col] + cost[next_row, next_col]
            ) / 2.0
            if next_cost >= best[next_row, next_col]:
                continue
            best[next_row, next_col] = next_cost
            parent_row[next_row, next_col] = row
            parent_col[next_row, next_col] = col
            heuristic = resolution_m * math.hypot(
                goal_rc[0] - next_row, goal_rc[1] - next_col
            )
            heapq.heappush(
                queue,
                (next_cost + heuristic, next_cost, next_row, next_col),
            )
    if not reached:
        raise RuntimeError("start and goal are not connected inside the drivable mask")
    path = [goal_rc]
    row, col = goal_rc
    while (row, col) != start_rc:
        row, col = int(parent_row[row, col]), int(parent_col[row, col])
        path.append((row, col))
    path.reverse()
    return np.asarray(path, dtype=int)


def build_mask_band_stations(
    path_rc: np.ndarray,
    drivable: np.ndarray,
    resolution_m: float,
    origin_xy: tuple[float, float],
    seed_stations: list[dict[str, float | str]] | None = None,
) -> list[dict[str, float | str]]:
    """Measure left/right reaches from a path to the hard-mask boundary."""
    height, width = drivable.shape
    stations: list[dict[str, float | str]] = []
    for index, (row, col) in enumerate(path_rc):
        before = path_rc[max(index - 1, 0)]
        after = path_rc[min(index + 1, len(path_rc) - 1)]
        direction = (after - before).astype(float)
        norm = max(float(np.linalg.norm(direction)), 1e-9)
        direction /= norm
        heading = math.atan2(-direction[0], direction[1])
        normal = np.asarray([-math.cos(heading), -math.sin(heading)])
        reaches: list[float] = []
        for sign in (1.0, -1.0):
            reach = 0.0
            for distance_px in np.arange(0.5, 100.5, 0.5):
                sample = np.rint(
                    np.asarray([row, col]) + sign * normal * distance_px
                ).astype(int)
                if not (
                    0 <= sample[0] < height
                    and 0 <= sample[1] < width
                    and drivable[sample[0], sample[1]]
                ):
                    break
                reach = distance_px * resolution_m
            reaches.append(max(reach, resolution_m / 2.0))
        x = origin_xy[0] + col * resolution_m
        y = origin_xy[1] + (height - 1 - row) * resolution_m
        semantics: dict[str, float | str] = {
            "left_drop_m": 0.0,
            "right_drop_m": 0.0,
            "left_kind": "unknown",
            "right_kind": "unknown",
            "left_rise_m": 0.0,
            "right_rise_m": 0.0,
        }
        if seed_stations:
            nearest = min(
                seed_stations,
                key=lambda station: (
                    (float(station["x"]) - x) ** 2
                    + (float(station["y"]) - y) ** 2
                ),
            )
            for key in semantics:
                semantics[key] = nearest[key]
        stations.append(
            {
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "heading_deg": round(math.degrees(heading), 3),
                "left_m": round(reaches[0], 3),
                "right_m": round(reaches[1], 3),
                **semantics,
            }
        )
    return stations


def _load_map(yaml_path: Path) -> tuple[np.ndarray, float, tuple[float, float]]:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image = np.asarray(Image.open(yaml_path.parent / metadata["image"]))
    return (
        image == FREE,
        float(metadata["resolution"]),
        (float(metadata["origin"][0]), float(metadata["origin"][1])),
    )


def _world_to_rc(
    point_xy: np.ndarray,
    shape: tuple[int, int],
    resolution_m: float,
    origin_xy: tuple[float, float],
) -> tuple[int, int]:
    col = int(round((point_xy[0] - origin_xy[0]) / resolution_m))
    row = int(round(shape[0] - 1 - (point_xy[1] - origin_xy[1]) / resolution_m))
    return row, col


def _segment_is_drivable(
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    drivable: np.ndarray,
    resolution_m: float,
    origin_xy: tuple[float, float],
) -> bool:
    """Conservatively inspect every raster interval crossed by a segment."""
    start = np.asarray(start_xy, dtype=float)
    end = np.asarray(end_xy, dtype=float)
    start_grid = np.array([
        (start[0] - origin_xy[0]) / resolution_m,
        drivable.shape[0] - 1
        - (start[1] - origin_xy[1]) / resolution_m,
    ])
    end_grid = np.array([
        (end[0] - origin_xy[0]) / resolution_m,
        drivable.shape[0] - 1
        - (end[1] - origin_xy[1]) / resolution_m,
    ])
    delta = end_grid - start_grid
    crossings = [0.0, 1.0]
    for axis in range(2):
        if abs(delta[axis]) < 1e-12:
            continue
        low, high = sorted((start_grid[axis], end_grid[axis]))
        first = int(np.floor(low - 0.5)) + 1
        last = int(np.ceil(high - 0.5))
        for cell in range(first, last + 1):
            t = (cell + 0.5 - start_grid[axis]) / delta[axis]
            if 0.0 < t < 1.0:
                crossings.append(float(t))
    crossings = np.unique(np.asarray(crossings))
    probes = np.concatenate((
        crossings,
        (crossings[:-1] + crossings[1:]) * 0.5,
    ))
    for point in start + probes[:, None] * (end - start):
        row, col = _world_to_rc(
            point, drivable.shape, resolution_m, origin_xy
        )
        if not (
            0 <= row < drivable.shape[0]
            and 0 <= col < drivable.shape[1]
            and drivable[row, col]
        ):
            return False
    return True


def _resample(points: np.ndarray, step_m: float) -> np.ndarray:
    legs = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(legs)))
    wanted = np.arange(0.0, arc[-1], step_m)
    if wanted.size == 0 or wanted[-1] < arc[-1] - 1e-9:
        wanted = np.append(wanted, arc[-1])
    return np.column_stack(
        (
            np.interp(wanted, arc, points[:, 0]),
            np.interp(wanted, arc, points[:, 1]),
        )
    )


def smooth_path(
    points: np.ndarray,
    drivable: np.ndarray,
    resolution_m: float,
    origin_xy: tuple[float, float],
) -> np.ndarray:
    """Remove grid stairs while retaining the authoritative hard mask."""
    points = np.asarray(points, dtype=float)
    if len(points) < SMOOTH_WINDOW:
        return points.copy()
    smoothed = points.copy()
    for _ in range(SMOOTH_PASSES):
        smoothed = np.column_stack([
            savgol_filter(
                smoothed[:, axis], SMOOTH_WINDOW, 3, mode="interp"
            )
            for axis in range(2)
        ])
    smoothed[0] = points[0]
    smoothed[-1] = points[-1]
    cells = np.asarray([
        _world_to_rc(point, drivable.shape, resolution_m, origin_xy)
        for point in smoothed
    ])
    valid = (
        (cells[:, 0] >= 0) & (cells[:, 0] < drivable.shape[0])
        & (cells[:, 1] >= 0) & (cells[:, 1] < drivable.shape[1])
    )
    clipped = cells.copy()
    clipped[:, 0] = np.clip(clipped[:, 0], 0, drivable.shape[0] - 1)
    clipped[:, 1] = np.clip(clipped[:, 1], 0, drivable.shape[1] - 1)
    valid &= drivable[clipped[:, 0], clipped[:, 1]]
    smoothed[~valid] = points[~valid]
    return smoothed


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--preferred-yaml", type=Path, required=True)
    parser.add_argument("--drivable-yaml", type=Path, required=True)
    parser.add_argument("--seed-route", type=Path, required=True)
    parser.add_argument("--seed-band", type=Path, required=True)
    parser.add_argument("--out-route", type=Path, required=True)
    parser.add_argument("--out-band", type=Path, required=True)
    args = parser.parse_args()
    preferred, preferred_resolution, preferred_origin = _load_map(
        args.preferred_yaml
    )
    drivable, resolution_m, origin_xy = _load_map(args.drivable_yaml)
    if (
        preferred.shape != drivable.shape
        or preferred_resolution != resolution_m
        or preferred_origin != origin_xy
    ):
        raise RuntimeError("preferred and drivable maps do not share one grid")
    seed = json.loads(args.seed_route.read_text(encoding="utf-8"))
    seed_band = json.loads(args.seed_band.read_text(encoding="utf-8"))
    seed_xy = np.asarray([[w["x"], w["y"]] for w in seed["waypoints"]])
    start = _world_to_rc(seed_xy[0], drivable.shape, resolution_m, origin_xy)
    goal = _world_to_rc(seed_xy[-1], drivable.shape, resolution_m, origin_xy)
    if not drivable[start] or not drivable[goal]:
        raise RuntimeError("seed route endpoints are outside the drivable mask")
    path_rc = plan_preferred_path(
        drivable, preferred, start, goal, resolution_m
    )
    path_xy = np.column_stack(
        (
            origin_xy[0] + path_rc[:, 1] * resolution_m,
            origin_xy[1] + (drivable.shape[0] - 1 - path_rc[:, 0]) * resolution_m,
        )
    )
    path_xy = smooth_path(path_xy, drivable, resolution_m, origin_xy)
    dense_xy = _resample(path_xy, PATH_STEP_M)
    dense_xy = smooth_path(dense_xy, drivable, resolution_m, origin_xy)
    if not all(
        _segment_is_drivable(
            start, end, drivable, resolution_m, origin_xy
        )
        for start, end in zip(dense_xy[:-1], dense_xy[1:])
    ):
        raise ValueError("smoothed route segment leaves drivable mask")
    tangent = np.gradient(dense_xy, axis=0)
    yaw = np.degrees(np.arctan2(tangent[:, 1], tangent[:, 0]))
    length_m = float(np.linalg.norm(np.diff(dense_xy, axis=0), axis=1).sum())
    route_doc = {
        "frame": "map",
        "source": "v6 preferred route optimized inside v8 drivable mask",
        "source_sha256": {
            "preferred_pgm": _sha256(args.preferred_yaml.parent / "route_2d_map_v6.pgm"),
            "drivable_pgm": _sha256(args.drivable_yaml.parent / "route_2d_map_v8.pgm"),
        },
        "body_frame_profile": str(seed["body_frame_profile"]),
        "count": len(dense_xy),
        "reference_point": "chair_centre",
        "chair_centre_in_body_xyz": seed["chair_centre_in_body_xyz"],
        "route_step_m": PATH_STEP_M,
        "path_length_m": round(length_m, 3),
        "operator_target_waypoint_index": len(dense_xy) - 1,
        "operator_target_xy_m": [
            round(float(dense_xy[-1, 0]), 3),
            round(float(dense_xy[-1, 1]), 3),
        ],
        "waypoints": [
            {
                "x": round(float(point[0]), 3),
                "y": round(float(point[1]), 3),
                "z": 0.0,
                "yaw_deg": round(float(angle), 6),
            }
            for point, angle in zip(dense_xy, yaw, strict=True)
        ],
    }
    band_xy = _resample(path_xy, BAND_STEP_M)
    band_rc = np.asarray(
        [
            _world_to_rc(point, drivable.shape, resolution_m, origin_xy)
            for point in band_xy
        ],
        dtype=int,
    )
    route_id = (
        "v6-v8:"
        + _sha256(args.preferred_yaml.parent / "route_2d_map_v6.pgm")[:12]
        + ":"
        + _sha256(args.drivable_yaml.parent / "route_2d_map_v8.pgm")[:12]
    )
    band_doc = {
        "frame": "map",
        "route_id": route_id,
        "drivable_mask_sha256": _sha256(
            args.drivable_yaml.parent / "route_2d_map_v8.pgm"),
        "drivable_mask_yaml_sha256": _sha256(args.drivable_yaml),
        "station_spacing_m": BAND_STEP_M,
        "stations": build_mask_band_stations(
            band_rc,
            drivable,
            resolution_m,
            origin_xy,
            seed_band["stations"],
        ),
        "corridor": {
            "source": args.drivable_yaml.name,
            "chair_half_width_m": CHAIR_HALF_WIDTH_M,
            "policy": "v8 is the authoritative chair-centre drivable mask",
            "stations_covered": len(band_rc),
            "stations_total": len(band_rc),
        },
        "physical_edge_semantics": {
            "source": args.seed_band.name,
            "status": "nearest v6 measured semantics over v8 hard boundary",
        },
    }
    args.out_band.write_text(json.dumps(band_doc, indent=1), encoding="utf-8")
    route_doc["asset_binding"] = {
        "route_id": route_id,
        "preferred_mask_sha256": _sha256(
            args.preferred_yaml.parent / "route_2d_map_v6.pgm"),
        "drivable_mask_sha256": _sha256(
            args.drivable_yaml.parent / "route_2d_map_v8.pgm"),
        "drivable_mask_yaml_sha256": _sha256(args.drivable_yaml),
        "safety_band_sha256": _sha256(args.out_band),
    }
    route_doc["asset_binding"]["route_content_sha256"] = hashlib.sha256(
        json.dumps(
            {k: v for k, v in route_doc.items() if k != "asset_binding"},
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    args.out_route.write_text(json.dumps(route_doc, indent=1), encoding="utf-8")
    print(
        f"route: {len(dense_xy)} waypoints, {length_m:.3f} m; "
        f"band: {len(band_rc)} stations"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
