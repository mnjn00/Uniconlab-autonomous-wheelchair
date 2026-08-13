#!/usr/bin/env python3

import argparse
import copy
import json
import math
import os
import sys
from dataclasses import asdict, dataclass

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.interpolate import PchipInterpolator
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import terrain_graph as tg
from curb_corridor import (
    audit_path_against_band,
    curb_bounded_mask,
    plan_curb_bounded_path,
)


@dataclass(frozen=True)
class PlannerConfig:
    cell_m: float = 0.20
    known_gap_m: float = 0.30
    ground_opening_m: float = 0.60
    body_low_m: float = 0.12
    body_high_m: float = 1.80
    body_min_returns: int = 48
    step_limit_m: float = 0.10
    slope_limit_deg: float = 14.0
    clearance_m: float = 0.40
    start_artifact_radius_m: float = 0.85
    output_step_m: float = 0.20


def clear_start_footprint(
    obstacle,
    start_xy,
    min_x,
    min_y,
    cell,
    radius_m,
):
    cleared = obstacle.copy()
    row = int(round((start_xy[1] - min_y) / cell - 0.5))
    col = int(round((start_xy[0] - min_x) / cell - 0.5))
    rows, cols = np.ogrid[: obstacle.shape[0], : obstacle.shape[1]]
    footprint = (
        ((rows - row) * cell) ** 2 + ((cols - col) * cell) ** 2
        <= radius_m**2
    )
    cleared[footprint] = False
    return cleared


def configuration_space(
    known,
    body_count,
    step_m,
    slope_deg,
    start_xy,
    min_x,
    min_y,
    cell,
    config,
):
    obstacle = ndimage.binary_closing(
        body_count >= config.body_min_returns,
        structure=tg.disk(1),
    )
    obstacle = clear_start_footprint(
        obstacle,
        start_xy,
        min_x,
        min_y,
        cell,
        config.start_artifact_radius_m,
    )
    stepped = step_m >= config.step_limit_m
    supported = ndimage.binary_erosion(
        known,
        structure=np.ones((3, 3), dtype=bool),
    )
    near_step = ndimage.binary_dilation(stepped, structure=tg.disk(1))
    steep = (
        (slope_deg > config.slope_limit_deg)
        & supported
        & np.logical_not(near_step)
    )
    hard = np.logical_not(known)
    for refusal in (obstacle, stepped, steep):
        hard = np.logical_or(hard, refusal)
    clearance = ndimage.distance_transform_edt(np.logical_not(hard)) * cell
    centre_free = known & (clearance >= config.clearance_m)
    return {
        "obstacle": obstacle,
        "stepped": stepped,
        "steep": steep,
        "hard": hard,
        "clearance": clearance,
        "centre_free": centre_free,
    }


def apply_exclusions(masks, exclusions, min_x, min_y, cell):
    hard = masks["hard"].copy()
    rows, cols = np.ogrid[: hard.shape[0], : hard.shape[1]]
    for x, y, radius in exclusions:
        row = int(round((y - min_y) / cell - 0.5))
        col = int(round((x - min_x) / cell - 0.5))
        hard |= (
            ((rows - row) * cell) ** 2 + ((cols - col) * cell) ** 2
            <= radius**2
        )
    clearance = ndimage.distance_transform_edt(np.logical_not(hard)) * cell
    updated = dict(masks)
    updated["hard"] = hard
    updated["clearance"] = clearance
    updated["centre_free"] = clearance >= 0.0
    updated["centre_free"] &= np.logical_not(hard)
    return updated


def band_clearance_violations(band, required_side_m):
    violations = []
    for index, station in enumerate(band["stations"]):
        heading = math.radians(station["heading_deg"])
        normal = np.array([-math.sin(heading), math.cos(heading)])
        centre = np.array([station["x"], station["y"]], dtype=float)
        for side, sign in (("left", 1.0), ("right", -1.0)):
            clearance = float(station[side + "_m"])
            if clearance >= required_side_m:
                continue
            edge = centre + sign * normal * clearance
            violations.append({
                "station": index,
                "side": side,
                "edge_xy": [
                    round(float(edge[0]), 6),
                    round(float(edge[1]), 6),
                ],
                "clearance_m": clearance,
            })
    return violations


def recenter_route_document(
    route,
    band,
    required_side_m,
    endpoint_guard=2,
    transition_stations=5,
):
    stations = band["stations"]
    route_xy = np.array([
        [item["x"], item["y"]]
        for item in route["waypoints"]
    ], dtype=float)
    route_arc = np.concatenate([
        [0.0],
        np.cumsum(np.hypot(*np.diff(route_xy, axis=0).T)),
    ])
    station_xy = np.array([
        [item["x"], item["y"]]
        for item in stations
    ], dtype=float)
    station_arc = np.concatenate([
        [0.0],
        np.cumsum(np.hypot(*np.diff(station_xy, axis=0).T)),
    ])
    station_offset = np.zeros(len(stations), dtype=float)
    violated = np.zeros(len(stations), dtype=bool)
    for index, station in enumerate(stations):
        if index < endpoint_guard or index >= len(stations) - endpoint_guard:
            continue
        left = float(station["left_m"])
        right = float(station["right_m"])
        if left >= required_side_m and right >= required_side_m:
            continue
        if left + right < 2.0 * required_side_m:
            raise RuntimeError(
                "band is physically narrower than the bilateral requirement "
                "at station %d" % index
            )
        if right < required_side_m:
            station_offset[index] = required_side_m - right
        elif left < required_side_m:
            station_offset[index] = -(required_side_m - left)
        violated[index] = True
    required_offset = station_offset.copy()
    station_offset.fill(0.0)
    for index in np.nonzero(violated)[0]:
        value = required_offset[index]
        start = max(endpoint_guard, index - transition_stations)
        end = min(
            len(stations) - endpoint_guard,
            index + transition_stations + 1,
        )
        for neighbor in range(start, end):
            weight = 1.0 - (
                abs(neighbor - index) / float(transition_stations + 1)
            )
            candidate = value * weight
            if candidate > 0.0:
                station_offset[neighbor] = max(
                    station_offset[neighbor],
                    candidate,
                )
            elif candidate < 0.0:
                station_offset[neighbor] = min(
                    station_offset[neighbor],
                    candidate,
                )
    station_heading = np.radians(np.array([
        item["heading_deg"]
        for item in stations
    ], dtype=float))
    station_normal = np.column_stack([
        -np.sin(station_heading),
        np.cos(station_heading),
    ])
    target_station_xy = (
        station_xy + station_normal * station_offset[:, None]
    )
    route_progress = route_arc / max(route_arc[-1], 1e-9)
    station_progress = station_arc / max(station_arc[-1], 1e-9)
    corrected_xy = np.column_stack([
        PchipInterpolator(
            station_progress,
            target_station_xy[:, axis],
        )(route_progress)
        for axis in (0, 1)
    ])
    corrected = copy.deepcopy(route)
    corrected_tangent = np.gradient(corrected_xy, axis=0)
    corrected_yaw = np.degrees(np.arctan2(
        corrected_tangent[:, 1],
        corrected_tangent[:, 0],
    ))
    for item, xy, yaw in zip(
        corrected["waypoints"],
        corrected_xy,
        corrected_yaw,
    ):
        item["x"] = round(float(xy[0]), 3)
        item["y"] = round(float(xy[1]), 3)
        item["yaw_deg"] = round(float(yaw), 2)
    corrected["source"] = corrected.get("source", "") + (
        "; recentered from measured bilateral safety band"
    )
    corrected["bilateral_clearance_requirement_m"] = required_side_m
    return corrected


def recenter_band_document(
    band,
    required_side_m,
    endpoint_guard=2,
    transition_stations=5,
):
    corrected = copy.deepcopy(band)
    stations = corrected["stations"]
    required_offset = np.zeros(len(stations), dtype=float)
    for index, station in enumerate(stations):
        if index < endpoint_guard or index >= len(stations) - endpoint_guard:
            continue
        left = float(station["left_m"])
        right = float(station["right_m"])
        if left + right < 2.0 * required_side_m:
            raise RuntimeError(
                "band is physically narrower than the bilateral requirement "
                "at station %d" % index
            )
        if right < required_side_m:
            required_offset[index] = required_side_m - right
        elif left < required_side_m:
            required_offset[index] = -(required_side_m - left)
    offset = np.zeros(len(stations), dtype=float)
    for index in np.nonzero(required_offset)[0]:
        start = max(endpoint_guard, index - transition_stations)
        end = min(
            len(stations) - endpoint_guard,
            index + transition_stations + 1,
        )
        for neighbor in range(start, end):
            if transition_stations:
                weight = 1.0 - (
                    abs(neighbor - index)
                    / float(transition_stations + 1)
                )
            else:
                weight = 1.0
            candidate = required_offset[index] * weight
            if candidate > 0.0:
                offset[neighbor] = max(offset[neighbor], candidate)
            elif candidate < 0.0:
                offset[neighbor] = min(offset[neighbor], candidate)
    for index, required in enumerate(required_offset):
        if required > 0.0:
            offset[index] = max(offset[index], required)
        elif required < 0.0:
            offset[index] = min(offset[index], required)
    for station, shift in zip(stations, offset):
        heading = math.radians(station["heading_deg"])
        normal = np.array([-math.sin(heading), math.cos(heading)])
        centre = np.array([station["x"], station["y"]], dtype=float)
        centre += normal * shift
        station["x"] = round(float(centre[0]), 6)
        station["y"] = round(float(centre[1]), 6)
        station["left_m"] = round(float(station["left_m"] - shift), 6)
        station["right_m"] = round(float(station["right_m"] + shift), 6)
    corrected["bilateral_clearance_requirement_m"] = required_side_m
    corrected["centreline_reexpressed_without_moving_measured_edges"] = True
    return corrected


def safe_component(centre_free, start_xy, min_x, min_y, cell):
    labels, _ = ndimage.label(
        centre_free,
        structure=np.ones((3, 3), dtype=np.int8),
    )
    row = int(np.clip(
        round((start_xy[1] - min_y) / cell - 0.5),
        0,
        centre_free.shape[0] - 1,
    ))
    col = int(np.clip(
        round((start_xy[0] - min_x) / cell - 0.5),
        0,
        centre_free.shape[1] - 1,
    ))
    label = int(labels[row, col])
    if label == 0:
        free_rows, free_cols = np.nonzero(centre_free)
        if not len(free_rows):
            return np.zeros_like(centre_free)
        nearest = int(np.argmin(
            (free_rows - row) ** 2 + (free_cols - col) ** 2
        ))
        label = int(labels[free_rows[nearest], free_cols[nearest]])
    return labels == label


def plan_safe_path(
    masks,
    slope_deg,
    start_xy,
    goal_xy,
    min_x,
    min_y,
    cell,
    config,
):
    reachable = safe_component(
        masks["centre_free"],
        start_xy,
        min_x,
        min_y,
        cell,
    )
    goal_row = int(np.clip(
        round((goal_xy[1] - min_y) / cell - 0.5),
        0,
        reachable.shape[0] - 1,
    ))
    goal_col = int(np.clip(
        round((goal_xy[0] - min_x) / cell - 0.5),
        0,
        reachable.shape[1] - 1,
    ))
    if not reachable[goal_row, goal_col]:
        raise RuntimeError(
            "goal is outside the start-connected safe component"
        )
    grid = {
        "cell": cell,
        "min_x": min_x,
        "min_y": min_y,
        "nx": reachable.shape[1],
        "ny": reachable.shape[0],
    }
    graph_land = {"gate_slope_deg": np.zeros_like(slope_deg)}
    graph_masks = {
        "reachable": reachable,
        "clearance": masks["clearance"],
    }
    graph, index, rows, cols, _ = tg.cell_graph(
        grid,
        graph_land,
        graph_masks,
    )
    cells, _ = tg.grid_plan(
        graph,
        index,
        rows,
        cols,
        grid,
        start_xy,
        goal_xy,
    )
    if cells is None:
        raise RuntimeError(
            "start and goal are not connected in the map-only safe space"
        )
    cells = np.asarray(cells, dtype=np.int64)
    return np.column_stack([
        min_x + (cells[:, 1] + 0.5) * cell,
        min_y + (cells[:, 0] + 0.5) * cell,
    ])


def _ply_header(path):
    with open(path, "rb") as handle:
        header = b""
        while not header.endswith(b"end_header\n"):
            byte = handle.read(1)
            if not byte:
                raise ValueError("invalid PLY: missing end_header")
            header += byte
    text = header.decode("ascii")
    count = None
    properties = []
    for line in text.splitlines():
        parts = line.split()
        if parts[:2] == ["element", "vertex"]:
            count = int(parts[2])
        elif parts[:1] == ["property"]:
            properties.append(parts[-1])
    if count is None or properties[:4] != ["x", "y", "z", "intensity"]:
        raise ValueError("expected binary PLY x y z intensity vertices")
    return len(header), count


def load_dense_ply(path):
    offset, count = _ply_header(path)
    dtype = np.dtype([
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
    ])
    return np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(count,))


def load_route_local_ply(
    path,
    route_xy,
    radius_m,
    chunk_size=1_000_000,
):
    vertices = load_dense_ply(path)
    route_tree = cKDTree(np.asarray(route_xy, dtype=float))
    selected = []
    for begin in range(0, len(vertices), chunk_size):
        chunk = vertices[begin:begin + chunk_size]
        xyz = np.column_stack([
            chunk["x"],
            chunk["y"],
            chunk["z"],
        ])
        finite = np.isfinite(xyz).all(axis=1)
        distance = np.full(len(xyz), np.inf, dtype=float)
        distance[finite] = route_tree.query(
            xyz[finite, :2],
            workers=-1,
        )[0]
        keep = np.logical_and(finite, distance <= radius_m)
        if np.any(keep):
            selected.append(np.column_stack([
                xyz[keep],
                np.asarray(chunk["intensity"])[keep],
            ]))
    if not selected:
        return np.empty((0, 4), dtype=np.float32)
    return np.vstack(selected).astype(np.float32, copy=False)


def dense_dem(path, start_xy, config, exclusions=()):
    vertices = load_dense_ply(path)
    cell = config.cell_m
    bounds = []
    for field in ("x", "y"):
        values = np.asarray(vertices[field])
        bounds.append((float(np.nanmin(values)), float(np.nanmax(values))))
    min_x = math.floor((bounds[0][0] - 1.0) / cell) * cell
    min_y = math.floor((bounds[1][0] - 1.0) / cell) * cell
    max_x = math.ceil((bounds[0][1] + 1.0) / cell) * cell
    max_y = math.ceil((bounds[1][1] + 1.0) / cell) * cell
    nx = int(round((max_x - min_x) / cell))
    ny = int(round((max_y - min_y) / cell))
    total = nx * ny
    lowest = np.full(total, np.inf, dtype=np.float32)
    count = np.zeros(total, dtype=np.int32)
    chunk = 1_000_000
    for start in range(0, len(vertices), chunk):
        part = vertices[start : start + chunk]
        x = np.asarray(part["x"])
        y = np.asarray(part["y"])
        z = np.asarray(part["z"])
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        col = np.floor((x[valid] - min_x) / cell).astype(np.int64)
        row = np.floor((y[valid] - min_y) / cell).astype(np.int64)
        inside = (col >= 0) & (col < nx) & (row >= 0) & (row < ny)
        flat = row[inside] * nx + col[inside]
        np.minimum.at(lowest, flat, z[valid][inside])
        np.add.at(count, flat, 1)
    lowest = lowest.reshape(ny, nx)
    count = count.reshape(ny, nx)
    measured = np.isfinite(lowest) & (count >= 2)
    distance, nearest = ndimage.distance_transform_edt(
        np.logical_not(measured),
        return_indices=True,
    )
    seed = lowest.copy()
    seed[np.logical_not(measured)] = lowest[
        tuple(nearest[:, np.logical_not(measured)])
    ]
    known = distance * cell <= config.known_gap_m
    radius = max(1, int(round(config.ground_opening_m / cell)))
    ground = ndimage.grey_opening(seed, footprint=tg.disk(radius))
    ground = ndimage.median_filter(ground, size=3)
    body = np.zeros(total, dtype=np.int32)
    flat_ground = ground.reshape(-1)
    for start in range(0, len(vertices), chunk):
        part = vertices[start : start + chunk]
        x = np.asarray(part["x"])
        y = np.asarray(part["y"])
        z = np.asarray(part["z"])
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        col = np.floor((x[valid] - min_x) / cell).astype(np.int64)
        row = np.floor((y[valid] - min_y) / cell).astype(np.int64)
        inside = (col >= 0) & (col < nx) & (row >= 0) & (row < ny)
        flat = row[inside] * nx + col[inside]
        above = z[valid][inside] - flat_ground[flat]
        body_flat = flat[
            (above >= config.body_low_m) & (above <= config.body_high_m)
        ]
        np.add.at(body, body_flat, 1)
    body = body.reshape(ny, nx)
    step = np.zeros((ny, nx), dtype=np.float32)
    for shift_row, shift_col in ((1, 0), (0, 1), (1, 1), (1, -1)):
        source_rows = slice(
            max(0, -shift_row),
            ny - max(0, shift_row),
        )
        source_cols = slice(
            max(0, -shift_col),
            nx - max(0, shift_col),
        )
        target_rows = slice(
            max(0, shift_row),
            ny - max(0, -shift_row),
        )
        target_cols = slice(
            max(0, shift_col),
            nx - max(0, -shift_col),
        )
        difference = np.abs(
            ground[source_rows, source_cols]
            - ground[target_rows, target_cols]
        )
        supported = (
            known[source_rows, source_cols]
            & known[target_rows, target_cols]
        )
        difference[np.logical_not(supported)] = 0.0
        target = step[source_rows, source_cols]
        np.maximum(target, difference, out=target)
    gradient_y, gradient_x = np.gradient(ground, cell)
    slope = np.degrees(np.arctan(np.hypot(gradient_x, gradient_y)))
    masks = configuration_space(
        known,
        body,
        step,
        slope,
        start_xy,
        min_x,
        min_y,
        cell,
        config,
    )
    if exclusions:
        masks = apply_exclusions(
            masks,
            exclusions,
            min_x,
            min_y,
            cell,
        )
        masks["centre_free"] = (
            known
            & (masks["clearance"] >= config.clearance_m)
        )
    return {
        "cell": cell,
        "min_x": min_x,
        "min_y": min_y,
        "ground": ground,
        "known": known,
        "body": body,
        "step": step,
        "slope": slope,
        **masks,
    }


def resample(polyline, step_m):
    segment = np.diff(polyline, axis=0)
    distance = np.hypot(segment[:, 0], segment[:, 1])
    arc = np.concatenate([[0.0], np.cumsum(distance)])
    wanted = np.arange(0.0, arc[-1] + step_m * 0.5, step_m)
    return np.column_stack([
        np.interp(wanted, arc, polyline[:, 0]),
        np.interp(wanted, arc, polyline[:, 1]),
    ])


def route_document(
    path_xy,
    dem,
    config,
    start_xy,
    goal_xy,
    exclusions=(),
):
    dense = resample(path_xy, config.output_step_m)
    cell = dem["cell"]
    rows = np.clip(
        np.rint((dense[:, 1] - dem["min_y"]) / cell - 0.5).astype(int),
        0,
        dem["ground"].shape[0] - 1,
    )
    cols = np.clip(
        np.rint((dense[:, 0] - dem["min_x"]) / cell - 0.5).astype(int),
        0,
        dem["ground"].shape[1] - 1,
    )
    tangent = np.gradient(dense, axis=0)
    yaw = np.degrees(np.arctan2(tangent[:, 1], tangent[:, 0]))
    waypoints = [
        {
            "x": round(float(x), 3),
            "y": round(float(y), 3),
            "z": round(float(dem["ground"][row, col]), 3),
            "yaw_deg": round(float(angle), 2),
        }
        for (x, y), row, col, angle in zip(dense, rows, cols, yaw)
    ]
    return {
        "frame": "map",
        "source": (
            "independent dense 3D map planner; no recorded route input"
        ),
        "planner_inputs": {
            "start_xy": list(start_xy),
            "goal_xy": list(goal_xy),
            "recorded_route_used": False,
            "excluded_hazard_discs": [list(item) for item in exclusions],
        },
        "body_frame_profile": "builtin",
        "reference_point": "chair_centre",
        "chair_centre_in_body_xyz": [-0.5, -0.2, 0.0],
        "route_step_m": config.output_step_m,
        "count": len(waypoints),
        "path_length_m": round(float(np.hypot(
            *np.diff(dense, axis=0).T
        ).sum()), 3),
        "waypoints": waypoints,
    }


def conservative_band_document(route, clearance):
    stations = []
    for waypoint, raw_clearance in zip(route["waypoints"], clearance):
        side = round(float(raw_clearance), 6)
        stations.append({
            "x": waypoint["x"],
            "y": waypoint["y"],
            "heading_deg": waypoint["yaw_deg"],
            "left_m": side,
            "right_m": side,
            "left_kind": "unscanned",
            "right_kind": "unscanned",
        })
    return {
        "frame": "map",
        "station_spacing_m": route["route_step_m"],
        "source": "conservative clearance transform of map-derived hard mask",
        "stations": stations,
    }


def maximum_connected_clearance(
    hard,
    start_xy,
    goal_xy,
    min_x,
    min_y,
    cell,
):
    clearance = ndimage.distance_transform_edt(
        np.logical_not(hard)) * cell

    def grid_cell(point):
        col = int(round((point[0] - min_x) / cell - 0.5))
        row = int(round((point[1] - min_y) / cell - 0.5))
        return row, col

    start = grid_cell(start_xy)
    goal = grid_cell(goal_xy)
    if not all((
        0 <= start[0] < hard.shape[0],
        0 <= start[1] < hard.shape[1],
        0 <= goal[0] < hard.shape[0],
        0 <= goal[1] < hard.shape[1],
    )):
        return 0.0
    candidates = np.unique(clearance)
    for threshold in candidates[::-1]:
        free = clearance + 1e-9 >= threshold
        labels, _ = ndimage.label(
            free,
            structure=ndimage.generate_binary_structure(2, 1),
        )
        start_label = int(labels[start])
        if start_label and start_label == int(labels[goal]):
            return float(threshold)
    return 0.0


def plan_and_audit_dem(dem, start_xy, goal_xy, config=None):
    config = config or PlannerConfig(clearance_m=0.45)
    cell = float(dem["cell"])
    hard = np.asarray(dem["hard"], dtype=bool)
    clearance = ndimage.distance_transform_edt(
        np.logical_not(hard)) * cell
    centre_free = clearance + 1e-9 >= config.clearance_m
    masks = {
        "hard": hard,
        "clearance": clearance,
        "centre_free": centre_free,
    }
    slope = np.asarray(
        dem.get("slope", dem.get("slope_deg", np.zeros(hard.shape))),
        dtype=float,
    )
    try:
        path_xy = plan_safe_path(
            masks,
            slope,
            start_xy,
            goal_xy,
            float(dem["min_x"]),
            float(dem["min_y"]),
            cell,
            config,
        )
    except RuntimeError as error:
        return {
            "status": "BLOCKED",
            "reason": str(error),
            "required_clearance_m": config.clearance_m,
            "maximum_connected_clearance_m": round(
                maximum_connected_clearance(
                    hard,
                    start_xy,
                    goal_xy,
                    float(dem["min_x"]),
                    float(dem["min_y"]),
                    cell,
                ),
                6,
            ),
        }
    planning_dem = dict(dem)
    planning_dem["clearance"] = clearance
    planning_dem["centre_free"] = centre_free
    route = route_document(
        path_xy,
        planning_dem,
        config,
        start_xy,
        goal_xy,
    )
    route_xy = np.array([
        [waypoint["x"], waypoint["y"]]
        for waypoint in route["waypoints"]
    ])
    rows = np.clip(
        np.rint(
            (route_xy[:, 1] - float(dem["min_y"])) / cell - 0.5
        ).astype(int),
        0,
        hard.shape[0] - 1,
    )
    cols = np.clip(
        np.rint(
            (route_xy[:, 0] - float(dem["min_x"])) / cell - 0.5
        ).astype(int),
        0,
        hard.shape[1] - 1,
    )
    sampled_clearance = clearance[rows, cols]
    violations = sampled_clearance + 1e-9 < config.clearance_m
    band = conservative_band_document(route, sampled_clearance)
    audit = {
        "bilateral_station_violations": int(np.count_nonzero(violations)),
        "continuous_clearance_violations": int(np.count_nonzero(violations)),
        "hard_hazard_hits": int(np.count_nonzero(hard[rows, cols])),
        "minimum_clearance_m": round(float(sampled_clearance.min()), 6),
        "required_clearance_m": config.clearance_m,
    }
    return {
        "status": "APPROVED" if not any((
            audit["bilateral_station_violations"],
            audit["continuous_clearance_violations"],
            audit["hard_hazard_hits"],
        )) else "BLOCKED",
        "route": route,
        "band": band,
        "audit": audit,
    }


def write_preview(path, dem, route):
    known = dem["known"]
    image = np.full(known.shape, 205, dtype=np.uint8)
    image[known] = 254
    image[dem["hard"]] = 0
    canvas = Image.fromarray(image[::-1]).convert("RGB")
    draw = ImageDraw.Draw(canvas)
    height = image.shape[0]

    def pixel(x, y):
        col = int((x - dem["min_x"]) / dem["cell"])
        row = int((y - dem["min_y"]) / dem["cell"])
        return col, height - 1 - row

    points = [pixel(item["x"], item["y"]) for item in route["waypoints"]]
    draw.line(points, fill=(0, 230, 210), width=3)
    draw.ellipse([
        points[0][0] - 6,
        points[0][1] - 6,
        points[0][0] + 6,
        points[0][1] + 6,
    ], fill=(80, 255, 100))
    draw.ellipse([
        points[-1][0] - 6,
        points[-1][1] - 6,
        points[-1][0] + 6,
        points[-1][1] + 6,
    ], fill=(255, 90, 90))
    canvas.save(path)


def parse_xy(value):
    x, y = value.split(",")
    return float(x), float(y)


def parse_disc(value):
    x, y, radius = value.split(",")
    return float(x), float(y), float(radius)


def load_dem_cache(path):
    with np.load(path) as cache:
        required = ("cell", "min_x", "min_y", "ground", "known", "hard")
        missing = [name for name in required if name not in cache]
        if missing:
            raise ValueError(
                "DEM cache is missing required fields: %s"
                % ", ".join(missing))
        slope_name = "slope" if "slope" in cache else "slope_deg"
        slope = (
            np.asarray(cache[slope_name])
            if slope_name in cache
            else np.zeros(cache["hard"].shape, dtype=np.float32)
        )
        return {
            "cell": float(cache["cell"]),
            "min_x": float(cache["min_x"]),
            "min_y": float(cache["min_y"]),
            "ground": np.asarray(cache["ground"]),
            "known": np.asarray(cache["known"], dtype=bool),
            "hard": np.asarray(cache["hard"], dtype=bool),
            "slope": slope,
        }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("map_geometry")
    parser.add_argument("--start", required=True, type=parse_xy)
    parser.add_argument("--goal", required=True, type=parse_xy)
    parser.add_argument("--out-prefix", required=True)
    parser.add_argument(
        "--required-clearance",
        type=float,
        default=0.45,
    )
    parser.add_argument(
        "--exclude-disc",
        action="append",
        default=[],
        type=parse_disc,
    )
    args = parser.parse_args(argv)
    config = PlannerConfig(clearance_m=args.required_clearance)
    if args.map_geometry.endswith(".npz"):
        dem = load_dem_cache(args.map_geometry)
        result = plan_and_audit_dem(
            dem,
            args.start,
            args.goal,
            config,
        )
        with open(args.out_prefix + "_audit.json", "w") as handle:
            json.dump(
                result["audit"] if result["status"] == "APPROVED" else result,
                handle,
                indent=1,
            )
        if result["status"] != "APPROVED":
            print(json.dumps(result, indent=1))
            return 2
        with open(args.out_prefix + "_route.json", "w") as handle:
            json.dump(result["route"], handle, indent=1)
        with open(args.out_prefix + "_band.json", "w") as handle:
            json.dump(result["band"], handle, indent=1)
        print(json.dumps({
            "status": result["status"],
            "route": args.out_prefix + "_route.json",
            "band": args.out_prefix + "_band.json",
            "audit": args.out_prefix + "_audit.json",
            **result["audit"],
        }, indent=1))
        return 0
    dem = dense_dem(
        args.map_geometry,
        args.start,
        config,
        args.exclude_disc,
    )
    path_xy = plan_safe_path(
        dem,
        dem["slope"],
        args.start,
        args.goal,
        dem["min_x"],
        dem["min_y"],
        dem["cell"],
        config,
    )
    route = route_document(
        path_xy,
        dem,
        config,
        args.start,
        args.goal,
        args.exclude_disc,
    )
    with open(args.out_prefix + "_route.json", "w") as handle:
        json.dump(route, handle, indent=1)
    np.savez_compressed(
        args.out_prefix + "_terrain.npz",
        cell=dem["cell"],
        min_x=dem["min_x"],
        min_y=dem["min_y"],
        ground=dem["ground"].astype(np.float32),
        known=dem["known"],
        obstacle=dem["obstacle"],
        stepped=dem["stepped"],
        steep=dem["steep"],
        hard=dem["hard"],
        clearance=dem["clearance"].astype(np.float32),
        centre_free=dem["centre_free"],
        config=json.dumps(asdict(config)),
    )
    write_preview(args.out_prefix + "_preview.png", dem, route)
    summary = {
        "recorded_route_used": False,
        "path_length_m": route["path_length_m"],
        "waypoints": route["count"],
        "min_clearance_m": round(float(np.min([
            dem["clearance"][
                int(round((item["y"] - dem["min_y"]) / dem["cell"] - 0.5)),
                int(round((item["x"] - dem["min_x"]) / dem["cell"] - 0.5)),
            ]
            for item in route["waypoints"]
        ])), 3),
        "config": asdict(config),
    }
    with open(args.out_prefix + "_summary.json", "w") as handle:
        json.dump(summary, handle, indent=1)
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
