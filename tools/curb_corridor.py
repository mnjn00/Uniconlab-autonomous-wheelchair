"""Curb-bounded raster planning from a dense-map measured safety band."""

import math

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

import terrain_graph as tg


def _band_arrays(band):
    stations = band["stations"]
    xy = np.array([[item["x"], item["y"]] for item in stations], dtype=float)
    heading = np.radians([item["heading_deg"] for item in stations])
    normals = np.column_stack([-np.sin(heading), np.cos(heading)])
    left = np.array([item["left_m"] for item in stations], dtype=float)
    right = np.array([item["right_m"] for item in stations], dtype=float)
    return xy, normals, left, right


def _raw_margins(points, band):
    xy, normals, left, right = _band_arrays(band)
    distances, order = cKDTree(xy).query(
        np.asarray(points, dtype=float),
        k=min(2, len(xy)),
    )
    if order.ndim == 1:
        order = order[:, None]
        distances = distances[:, None]
    nearest = order[:, 0]
    lateral = np.einsum(
        "ij,ij->i",
        np.asarray(points, dtype=float) - xy[nearest],
        normals[nearest],
    )
    return (
        np.min(left[order], axis=1) - lateral,
        np.min(right[order], axis=1) + lateral,
        distances[:, 0],
    )


def curb_bounded_mask(
    band,
    shape,
    min_x,
    min_y,
    cell,
    required_side_m,
    maximum_offset_m,
):
    """Rasterize only chair-centre cells proven inside both measured curbs."""
    rows, cols = np.indices(shape)
    points = np.column_stack([
        min_x + (cols.ravel() + 0.5) * cell,
        min_y + (rows.ravel() + 0.5) * cell,
    ])
    left, right, distance = _raw_margins(points, band)
    allowed = (
        (left + 1e-9 >= required_side_m)
        & (right + 1e-9 >= required_side_m)
        & (distance <= maximum_offset_m + 0.5 * cell)
    )
    return allowed.reshape(shape)


def _grid_cell(point, shape, min_x, min_y, cell):
    row = int(round((point[1] - min_y) / cell - 0.5))
    col = int(round((point[0] - min_x) / cell - 0.5))
    if not (0 <= row < shape[0] and 0 <= col < shape[1]):
        raise RuntimeError("endpoint is outside the curb-bounded raster")
    return row, col


def _segment_inside(mask, start, end, min_x, min_y, cell):
    spacing = cell * 0.25
    count = max(1, int(math.ceil(
        float(np.linalg.norm(end - start)) / spacing
    )))
    points = start + (
        end - start
    ) * np.arange(count + 1)[:, None] / count
    rows = np.rint((points[:, 1] - min_y) / cell - 0.5).astype(int)
    cols = np.rint((points[:, 0] - min_x) / cell - 0.5).astype(int)
    return bool(mask[rows, cols].all())


def _pull_inside_mask(path, mask, min_x, min_y, cell):
    """Pull a grid path taut without crossing a rasterized curb boundary."""
    pulled = [path[0]]
    start = 0
    while start < len(path) - 1:
        end = len(path) - 1
        while end > start + 1 and not _segment_inside(
            mask,
            path[start],
            path[end],
            min_x,
            min_y,
            cell,
        ):
            end -= 1
        pulled.append(path[end])
        start = end
    return np.asarray(pulled, dtype=float)


def plan_curb_bounded_path(
    mask,
    start_xy,
    goal_xy,
    min_x,
    min_y,
    cell,
):
    """Find a complete grid path without snapping either endpoint."""
    start = _grid_cell(start_xy, mask.shape, min_x, min_y, cell)
    goal = _grid_cell(goal_xy, mask.shape, min_x, min_y, cell)
    if not mask[start] or not mask[goal]:
        raise RuntimeError("endpoint is outside proven curb-bounded support")
    grid = {
        "cell": cell,
        "min_x": min_x,
        "min_y": min_y,
        "nx": mask.shape[1],
        "ny": mask.shape[0],
    }
    zero = np.zeros(mask.shape, dtype=float)
    graph, index, rows, cols, open_cell = tg.cell_graph(
        grid,
        {"gate_slope_deg": zero},
        {
            "reachable": mask,
            "clearance": ndimage.distance_transform_edt(mask) * cell,
        },
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
        raise RuntimeError("start and goal are disconnected by curb boundaries")
    path = np.column_stack([
        min_x + (cells[:, 1] + 0.5) * cell,
        min_y + (cells[:, 0] + 0.5) * cell,
    ])
    path[0] = start_xy
    path[-1] = goal_xy
    return _pull_inside_mask(path, open_cell, min_x, min_y, cell)


def _sample_path(path, spacing_m):
    samples = []
    for start, end in zip(path[:-1], path[1:]):
        count = max(1, int(math.ceil(
            float(np.linalg.norm(end - start)) / spacing_m
        )))
        samples.extend(
            start + (end - start) * index / count
            for index in range(count)
        )
    return np.asarray([*samples, path[-1]], dtype=float)


def audit_path_against_band(
    path,
    band,
    required_side_m,
    sample_spacing_m,
):
    """Audit every sampled chair centre and its bilateral wheel clearance."""
    samples = _sample_path(np.asarray(path, dtype=float), sample_spacing_m)
    left, right, _ = _raw_margins(samples, band)
    centre_violations = (left + 1e-9 < required_side_m) | (
        right + 1e-9 < required_side_m
    )
    tangent = np.gradient(samples, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normals = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    left_wheel = samples + 0.35 * normals
    right_wheel = samples - 0.35 * normals
    ll, lr, _ = _raw_margins(left_wheel, band)
    rl, rr, _ = _raw_margins(right_wheel, band)
    wheel_margin = np.minimum.reduce([ll, lr, rl, rr])
    wheel_violations = wheel_margin + 1e-9 < 0.10
    footprint_margins = []
    for along in (-0.485, 0.485):
        for lateral in (-0.38, 0.38):
            corner = samples + along * tangent + lateral * normals
            corner_left, corner_right, _ = _raw_margins(corner, band)
            footprint_margins.append(np.minimum(corner_left, corner_right))
    footprint_margin = np.min(np.stack(footprint_margins), axis=0)
    footprint_violations = footprint_margin + 1e-9 < 0.07
    violations = (
        centre_violations | wheel_violations | footprint_violations
    )
    return {
        "status": "BLOCKED" if np.any(violations) else "APPROVED",
        "samples": len(samples),
        "wheel_envelope_violations": int(np.count_nonzero(violations)),
        "minimum_left_clearance_m": float(np.min(left)),
        "minimum_right_clearance_m": float(np.min(right)),
        "minimum_wheel_boundary_margin_m": float(np.min(wheel_margin)),
        "minimum_padded_footprint_boundary_margin_m": float(
            np.min(footprint_margin)
        ),
    }
