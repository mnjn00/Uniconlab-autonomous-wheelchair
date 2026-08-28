#!/usr/bin/env python3
"""Start-and-goal global plan on the 3D map. No recorded line.

Refuses buildings/canopy (dense body-height returns), true kerbs (a
walkway edge or a drop larger than map quantization), and grades above
12°. A continuous descent — including the 0.20 m voxel stairs it becomes
in this cloud — is a slope, not a kerb. The kerb that matters is the
lip beside that descent.

    plan_start_goal.py <map.pcd> --start X,Y --goal X,Y --out-prefix PREFIX

Writes PREFIX_waypoints.json, PREFIX_summary.json, PREFIX_preview.png.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import terrain_graph as tg  # noqa: E402


def parse_xy(text):
    x, y = text.split(",")
    return float(x), float(y)


def raster_full(points, cell=tg.CELL, pad=2.0):
    """Raster the whole cloud. No route, no corridor seed."""
    pts = points[np.isfinite(points).all(axis=1)]
    min_x = float(pts[:, 0].min()) - pad
    min_y = float(pts[:, 1].min()) - pad
    nx = int((pts[:, 0].max() - min_x) / cell) + 2
    ny = int((pts[:, 1].max() - min_y) / cell) + 2
    col = ((pts[:, 0] - min_x) / cell).astype(np.int32)
    row = ((pts[:, 1] - min_y) / cell).astype(np.int32)
    valid = (col >= 0) & (col < nx) & (row >= 0) & (row < ny)
    col, row, xyz = col[valid], row[valid], pts[valid]
    flat = (row.astype(np.int64) * nx + col.astype(np.int64))
    count = np.bincount(flat, minlength=nx * ny)
    order = np.argsort(flat, kind="stable")
    sorted_flat, sorted_z = flat[order], xyz[order, 2]
    start = np.searchsorted(sorted_flat, np.arange(nx * ny))
    lowest = np.full(nx * ny, np.nan)
    occupied = np.where(count > 0)[0]
    lowest[occupied] = np.minimum.reduceat(sorted_z, start[occupied])
    known = (count > 0).reshape(ny, nx)
    lowest = lowest.reshape(ny, nx)
    filled = lowest[tuple(ndimage.distance_transform_edt(
        tg.keep_out(known), return_distances=False, return_indices=True))]
    return dict(
        cell=cell, min_x=min_x, min_y=min_y, nx=nx, ny=ny,
        known=known, filled=filled, inside_points=xyz,
        flat=flat, count=count.reshape(ny, nx),
        # Must not be 0: traversability treats to_route <= self_return_m as
        # the mapping vehicle. 1 m is inside CORRIDOR_M so the whole map
        # is "inside" and nothing is marked self-return.
        to_route=np.ones((ny, nx), np.float64),
        station=np.zeros((ny, nx), np.int64),
    )


def densify(xy, step=0.4):
    if len(xy) < 2:
        return xy.copy()
    out = [xy[0]]
    for a, b in zip(xy[:-1], xy[1:]):
        span = math.hypot(*(b - a))
        n = max(int(span / step), 1)
        for i in range(1, n + 1):
            t = i / n
            out.append(a * (1 - t) + b * t)
    return np.asarray(out, float)


def seal_aliasing_gaps(grid, masks):
    """Join measured pavement that is not a wall or a true kerb.

    Close at most ~1.2 m so a missed kerb cannot become a road crossing.
    """
    hard = np.logical_or(masks["obstruction"], masks["stepped"])
    candidate = np.logical_and(grid["known"], tg.keep_out(hard))
    candidate = np.logical_and(candidate, tg.keep_out(masks["steep"]))
    sealed = ndimage.binary_closing(candidate, structure=tg.disk(3))
    sealed = np.logical_or(sealed, candidate)
    bridged = ndimage.binary_closing(sealed, structure=tg.disk(4))
    return np.logical_and(bridged, tg.keep_out(hard))


def reachable_from(centre_free, grid, start_xy):
    labels, _ = ndimage.label(
        centre_free, structure=np.ones((3, 3), dtype=np.int8))
    row = int(np.clip(round((start_xy[1] - grid["min_y"]) / grid["cell"] - 0.5),
                      0, grid["ny"] - 1))
    col = int(np.clip(round((start_xy[0] - grid["min_x"]) / grid["cell"] - 0.5),
                      0, grid["nx"] - 1))
    label = int(labels[row, col])
    if label == 0:
        open_r, open_c = np.nonzero(centre_free)
        if not len(open_r):
            return np.zeros_like(centre_free)
        nearest = int(np.argmin((open_r - row) ** 2 + (open_c - col) ** 2))
        label = int(labels[open_r[nearest], open_c[nearest]])
    return labels == label


def at_cell(grid, xy):
    row = int(np.clip(round((xy[1] - grid["min_y"]) / grid["cell"] - 0.5),
                      0, grid["ny"] - 1))
    col = int(np.clip(round((xy[0] - grid["min_x"]) / grid["cell"] - 0.5),
                      0, grid["nx"] - 1))
    return row, col


def preview(grid, masks, sealed, planned, start, goal, recorded, path):
    cell = grid["cell"]
    ny, nx = grid["ny"], grid["nx"]
    image = np.zeros((ny, nx, 3), np.uint8)
    image[...] = (14, 18, 22)
    image[grid["known"]] = (42, 52, 58)
    image[sealed] = (48, 88, 78)
    image[masks["reachable"]] = (36, 90, 96)
    image[masks["steep"]] = (176, 72, 118)
    image[masks["stepped"]] = (214, 128, 36)
    image[masks["obstruction"]] = (176, 68, 58)

    pad_m = 12.0
    xs = [start[0], goal[0]]
    ys = [start[1], goal[1]]
    if planned is not None and len(planned):
        xs.extend(planned[:, 0].tolist())
        ys.extend(planned[:, 1].tolist())
    c0 = int(np.clip((min(xs) - pad_m - grid["min_x"]) / cell, 0, nx - 1))
    r0 = int(np.clip((min(ys) - pad_m - grid["min_y"]) / cell, 0, ny - 1))
    c1 = int(np.clip((max(xs) + pad_m - grid["min_x"]) / cell + 1, 1, nx))
    r1 = int(np.clip((max(ys) + pad_m - grid["min_y"]) / cell + 1, 1, ny))
    crop = image[r0:r1, c0:c1]
    scale = 3
    canvas = Image.fromarray(crop[::-1]).resize(
        ((c1 - c0) * scale, (r1 - r0) * scale), Image.NEAREST)
    pen = ImageDraw.Draw(canvas, "RGBA")

    def px(points):
        pts = np.asarray(points, float).reshape(-1, 2)
        x = ((pts[:, 0] - grid["min_x"]) / cell - c0) * scale
        y = ((r1 - r0) - ((pts[:, 1] - grid["min_y"]) / cell - r0)) * scale
        return [tuple(p) for p in np.column_stack([x, y])]

    if recorded is not None and len(recorded):
        pen.line(px(recorded), fill=(230, 220, 190, 90), width=1)
    if planned is not None and len(planned) > 1:
        pen.line(px(planned), fill=(80, 230, 210, 255), width=4)
    for xy, colour in ((start, (90, 220, 120)), (goal, (240, 90, 90))):
        x, y = px([xy])[0]
        pen.ellipse([x - 6, y - 6, x + 6, y + 6], fill=colour + (255,))
    canvas.save(path)


def sample_cells(grid, xy):
    cell = grid["cell"]
    row = np.clip(np.round((xy[:, 1] - grid["min_y"]) / cell - 0.5).astype(int),
                  0, grid["ny"] - 1)
    col = np.clip(np.round((xy[:, 0] - grid["min_x"]) / cell - 0.5).astype(int),
                  0, grid["nx"] - 1)
    return row, col


def write_waypoints(polyline, start, goal, out_path, source, grid=None,
                    ground=None):
    dense = densify(polyline, 0.5)
    if len(dense) < 2:
        dense = np.vstack([start, goal])
    heading = np.gradient(dense, axis=0)
    yaw = np.degrees(np.arctan2(heading[:, 1], heading[:, 0]))
    yaw[0] = math.degrees(math.atan2(dense[1, 1] - dense[0, 1],
                                     dense[1, 0] - dense[0, 0]))
    if grid is not None and ground is not None:
        row, col = sample_cells(grid, dense)
        height = ground[row, col]
    else:
        height = np.zeros(len(dense))
    length = float(np.hypot(*np.diff(dense, axis=0).T).sum())
    waypoints = []
    for (x, y), deg, z in zip(dense, yaw, height):
        waypoints.append({
            "x": round(float(x), 3),
            "y": round(float(y), 3),
            "z": round(float(z), 3),
            "yaw_deg": round(float(deg), 6),
        })
    doc = {
        "frame": "map",
        "source": source,
        "body_frame_profile": "builtin",
        "count": len(waypoints),
        "reference_point": "chair_centre",
        "chair_centre_in_body_xyz": [0.0, -0.173, 0.0],
        "route_step_m": 0.5,
        "path_length_m": round(length, 1),
        "operator_target_waypoint_index": len(waypoints) - 1,
        "operator_target_xy_m": [round(float(goal[0]), 2),
                                 round(float(goal[1]), 2)],
        "waypoints": waypoints,
    }
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(doc, handle, indent=2)
        handle.write("\n")
    return doc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_pcd")
    parser.add_argument("--start", required=True, type=parse_xy)
    parser.add_argument("--goal", required=True, type=parse_xy)
    parser.add_argument("--out-prefix", required=True)
    parser.add_argument("--min-body-returns", type=int, default=None,
                        help="legacy: every cell with this many body hits "
                             "is a wall. Default is dense-core buildings.")
    parser.add_argument("--dense-body-returns", type=int, default=8,
                        help="body-height hits that seed a building core")
    parser.add_argument("--attached-body-returns", type=int, default=3,
                        help="lighter body hits kept only if they touch a core")
    args = parser.parse_args(argv)

    start = np.array(args.start, float)
    goal = np.array(args.goal, float)
    print("start %s  goal %s  map %s" % (start, goal, args.map_pcd), flush=True)

    points = tg.load_cloud(args.map_pcd)
    print("cloud %d points" % len(points), flush=True)

    recorded = None
    grid = raster_full(points)
    print("grid %d x %d at %.2f m" % (grid["nx"], grid["ny"], grid["cell"]),
          flush=True)
    land = tg.terrain(grid)
    # Clear a small disc at the start so the parked chair in the map is
    # not an obstruction. No recorded line is consulted.
    if args.min_body_returns is not None:
        masks = tg.traversability(
            grid, land, trust_driven=False, seed_xy=tuple(start),
            self_return_m=0.0, min_body_returns=args.min_body_returns)
    else:
        masks = tg.traversability(
            grid, land, trust_driven=False, seed_xy=tuple(start),
            self_return_m=0.0,
            dense_body_returns=args.dense_body_returns,
            attached_body_returns=args.attached_body_returns)
    sr, sc = at_cell(grid, start)
    disc = tg.disk(int(round(0.85 / grid["cell"])))
    r0 = max(sr - disc.shape[0] // 2, 0)
    c0 = max(sc - disc.shape[1] // 2, 0)
    r1 = min(r0 + disc.shape[0], grid["ny"])
    c1 = min(c0 + disc.shape[1], grid["nx"])
    patch = disc[: r1 - r0, : c1 - c0]
    masks["obstruction"][r0:r1, c0:c1] = np.logical_and(
        masks["obstruction"][r0:r1, c0:c1], tg.keep_out(patch))
    cell = grid["cell"]
    area = cell * cell
    # Directional residual, not neighbour drop and not raw Laplacian: a
    # downhill ramp (even voxel-staircased) is a grade; the walkway edge
    # beside it is the kerb.
    kerb = tg.ramp_aware_kerb(
        land["ground"], land["slope_x"], land["slope_y"], grid["cell"])
    # The raster fills unknown cells from the nearest known height.
    # That invents cliffs across courtyards and map rims; they are not
    # walkway edges. Only measured ground can be a kerb.
    kerb = np.logical_and(kerb, grid["known"])
    masks["stepped"] = tg._drop_small(kerb, tg.MIN_STEP_CELLS)
    # Gate grade on the 2 m-smoothed surface so voxel stairs of a 12°
    # ramp are not 13.5°. Bleed only around true kerbs, not the descent.
    land["near_step"] = ndimage.binary_dilation(
        masks["stepped"], structure=tg.disk(max(int(round(tg.STEP_BLEED_M / grid["cell"])), 1)))
    smooth_slope = tg.smooth_slope_deg(land["ground"], grid["cell"])
    land["gate_slope_deg"] = np.where(land["near_step"], 0.0, smooth_slope)
    masks["steep"] = (land["gate_slope_deg"] > tg.SLOPE_BLOCK_DEG)
    masks["steep"] = np.logical_and(masks["steep"], grid["known"])
    masks["steep"] = tg._drop_small(masks["steep"], tg.MIN_STEP_CELLS)
    print("refusals  obstruction %.0f m2 | kerb/step %.0f m2 | steep %.0f m2"
          % (masks["obstruction"].sum() * area,
             masks["stepped"].sum() * area,
             masks["steep"].sum() * area), flush=True)

    sealed = seal_aliasing_gaps(grid, masks)
    # Chair-centre space: the footprint half-width plus the band margin.
    # One cell of erosion left the polyline on the lip of every kerb.
    radius = max(int(round((tg.CHAIR_HALF_WIDTH_M + tg.BAND_MARGIN_M)
                           / grid["cell"])), 1)
    centre = ndimage.binary_erosion(sealed, structure=tg.disk(radius))
    if not centre.any():
        preview(grid, masks, sealed, None, start, goal, recorded,
                args.out_prefix + "_preview.png")
        raise SystemExit(
            "chair-width erosion left no free cells. See %s_preview.png"
            % args.out_prefix)
    requested_start = start.copy()
    requested_goal = goal.copy()
    sr, sc = at_cell(grid, start)
    if not centre[sr, sc]:
        open_r, open_c = np.nonzero(centre)
        nearest = int(np.argmin((open_r - sr) ** 2 + (open_c - sc) ** 2))
        snap = np.array([
            grid["min_x"] + (open_c[nearest] + 0.5) * cell,
            grid["min_y"] + (open_r[nearest] + 0.5) * cell,
        ])
        snap_d = float(np.hypot(*(snap - start)))
        if snap_d > 4.0:
            preview(grid, masks, sealed, None, start, goal, recorded,
                    args.out_prefix + "_preview.png")
            raise SystemExit(
                "start is %.2f m from chair-width free space. "
                "See %s_preview.png" % (snap_d, args.out_prefix))
        print("start snapped by %.2f m onto sealed sidewalk" % snap_d,
              flush=True)
        start = snap
    reachable = reachable_from(centre, grid, start)
    print("sealed sidewalk %.0f m2; chair-centre reachable %.0f m2"
          % (sealed.sum() * area, reachable.sum() * area), flush=True)

    gr, gc = at_cell(grid, goal)
    if not reachable[gr, gc]:
        open_r, open_c = np.nonzero(reachable)
        if len(open_r):
            nearest = int(np.argmin((open_r - gr) ** 2 + (open_c - gc) ** 2))
            snap_d = math.hypot((open_r[nearest] - gr) * cell,
                                (open_c[nearest] - gc) * cell)
            if snap_d <= 4.0:
                goal_snap = np.array([
                    grid["min_x"] + (open_c[nearest] + 0.5) * cell,
                    grid["min_y"] + (open_r[nearest] + 0.5) * cell,
                ])
                print("goal snapped by %.2f m onto start-connected sidewalk"
                      % snap_d, flush=True)
                goal = goal_snap
                gr, gc = at_cell(grid, goal)
    if not reachable[gr, gc]:
        preview(grid, masks, sealed, None, start, goal, recorded,
                args.out_prefix + "_preview.png")
        raise SystemExit(
            "goal is not connected to start through drop-safe, "
            "building-safe ground. See %s_preview.png"
            % args.out_prefix)

    masks = dict(masks)
    masks["reachable"] = reachable
    masks["clearance"] = ndimage.distance_transform_edt(sealed) * cell
    net, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    staircase, cost = tg.grid_plan(net, index, rows, cols, grid,
                                   tuple(start), tuple(goal))
    if staircase is None:
        raise SystemExit("A* found no path inside the reachable component")
    pulled = tg.taut(staircase, open_cell)
    planned = np.column_stack([
        grid["min_x"] + (pulled[:, 1] + 0.5) * cell,
        grid["min_y"] + (pulled[:, 0] + 0.5) * cell,
    ])
    planned[0] = start
    planned[-1] = goal
    length = float(np.hypot(*np.diff(planned, axis=0).T).sum())
    straight = float(np.hypot(*(goal - start)))
    print("path %.1f m  (%d vertices)  straight line %.1f m  cost %.1f"
          % (length, len(planned), straight, cost), flush=True)

    preview(grid, masks, sealed, planned, start, goal, recorded,
            args.out_prefix + "_preview.png")
    route = write_waypoints(
        planned, start, goal, args.out_prefix + "_waypoints.json",
        source="A* on full 3D map; start/goal only; no recorded line. "
               "Kerbs are walkway edges / drops, not the downhill grade.",
        grid=grid, ground=land["ground"])
    row, col = sample_cells(grid, densify(planned, 0.5))
    to_kerb = float((ndimage.distance_transform_edt(
        tg.keep_out(masks["stepped"])) * cell)[row, col].min())
    to_body = float((ndimage.distance_transform_edt(
        tg.keep_out(masks["obstruction"])) * cell)[row, col].min())
    z = land["ground"][row, col]
    summary = {
        "start": [float(start[0]), float(start[1])],
        "goal": [float(goal[0]), float(goal[1])],
        "requested_start": [float(requested_start[0]),
                            float(requested_start[1])],
        "requested_goal": [float(requested_goal[0]),
                           float(requested_goal[1])],
        "path_length_m": route["path_length_m"],
        "straight_line_m": round(straight, 1),
        "vertices": len(planned),
        "cost": round(float(cost), 2),
        "refusals_m2": {
            "obstruction_buildings_canopy": round(float(masks["obstruction"].sum() * area), 1),
            "kerb_step": round(float(masks["stepped"].sum() * area), 1),
            "steep_grade": round(float(masks["steep"].sum() * area), 1),
        },
        "reachable_m2": round(float(reachable.sum() * area), 1),
        "net_drop_m": round(float(z[0] - z[-1]), 2),
        "height_range_m": [round(float(np.nanmin(z)), 2),
                           round(float(np.nanmax(z)), 2)],
        "min_clearance_to_kerb_m": round(to_kerb, 2),
        "min_clearance_to_building_m": round(to_body, 2),
        "chair_erosion_m": round(tg.CHAIR_HALF_WIDTH_M + tg.BAND_MARGIN_M, 2),
        "map": os.path.abspath(args.map_pcd),
        "recorded_route_used_as_path": False,
        "recorded_route_used_as_coverage_bound": False,
        "kerb_is_ramp_aware": True,
        "min_body_returns": args.min_body_returns,
        "dense_body_returns": None if args.min_body_returns is not None
        else args.dense_body_returns,
        "attached_body_returns": None if args.min_body_returns is not None
        else args.attached_body_returns,
        "waypoints": args.out_prefix + "_waypoints.json",
        "preview": args.out_prefix + "_preview.png",
    }
    with open(args.out_prefix + "_summary.json", "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")
    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
