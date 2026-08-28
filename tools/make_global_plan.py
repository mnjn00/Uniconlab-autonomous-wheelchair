#!/usr/bin/env python3
"""Plan a global route across the committed map, and show the terrain it used.

    make_global_plan.py <map.pcd> <route.json> <out-dir>

Reads the 3D map, recovers the ground surface, splits the corridor into the
three reasons a cell refuses the chair (a body-height obstruction, a ground
step, a grade above what this chair has demonstrated), then plans start to goal
over the traversable surface with a slope and clearance cost.

What the recorded 0727 line contributes, stated plainly, because it is not
small: it bounds which part of the map is analysed, it places the start and
goal, it suppresses the obstruction test within SELF_RETURN_M (those returns are
the chair and its rider), and - with trust_driven on - it is injected into the
free set as drivable ground on the grounds that the chair drove it. The last of
those constructs a corridor along the recorded line, so the planned path is NOT
independent evidence for that line.

The tool therefore plans twice and reports both. Measured on merged_0707_0725:
with the injection on, a 364.9 m route exists and sits a median 0.18 m from the
recorded line; with it withdrawn, the region the chair centre can occupy is 2 m2
out of 7137 m2 measured and the best route is 3.8 m. Only 66 percent of the cells
under the recorded line have the 0.45 m of free half width the chair needs to be
placed at all. That gap is the map's resolution, not the planner's: a 0.20 m
voxel cloud rasterised at 0.15 m with the ground taken from cell minima cannot
resolve a footway to the precision a 0.70 m chair needs.

Writes: terrain.npz, global_plan.json, plan-overview.png, plan-detail.png and
global_plan.html (self-contained, zoomable).

Offline analysis over a committed map. Grants no motion authority; nothing here
is loaded by the runtime graph.
"""

import base64
import json
import math
import os
import sys
import time

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import terrain_graph as tg  # noqa: E402


# Deliberately not a theme: the raster IS the data, and a plan view of LiDAR
# returns reads on a dark ground. Recolouring it per viewer preference would
# change what the colours mean, so the instrument commits to one world.
INK = {
    "ground": (11, 16, 20),
    "unmapped": (22, 29, 34),
    "free": (51, 66, 75),
    "reachable": (38, 92, 99),
    "step": (217, 131, 36),
    "obstruction": (179, 69, 60),
    "steep": (181, 71, 127),
    "driven": (242, 234, 211),
    "path": (78, 224, 208),
    "node": (110, 168, 199),
    "edge": (47, 93, 122),
}
SLOPE_RAMP = [(0.0, (18, 42, 50)), (3.0, (63, 143, 122)),
              (7.0, (212, 185, 66)), (12.0, (194, 80, 63)),
              (25.0, (107, 31, 58))]
RESILIENCE_STEP_M = 5.0
RESILIENCE_RADIUS_M = 1.5


def note(message):
    print(message, flush=True)


def to_cell(grid, x, y):
    return ((y - grid["min_y"]) / grid["cell"] - 0.5,
            (x - grid["min_x"]) / grid["cell"] - 0.5)


def sample_surface(grid, field, xy):
    rows, cols = to_cell(grid, xy[:, 0], xy[:, 1])
    rows = np.clip(np.rint(rows).astype(np.int64), 0, grid["ny"] - 1)
    cols = np.clip(np.rint(cols).astype(np.int64), 0, grid["nx"] - 1)
    return field[rows, cols]


def resample(polyline, step_m):
    """Even arc-length resampling of a polyline."""
    delta = np.diff(polyline, axis=0)
    leg = np.hypot(delta[:, 0], delta[:, 1])
    arc = np.concatenate([[0.0], np.cumsum(leg)])
    if arc[-1] <= 0.0:
        return polyline.copy(), arc
    wanted = np.arange(0.0, arc[-1] + step_m * 0.5, step_m)
    out = np.column_stack([np.interp(wanted, arc, polyline[:, 0]),
                           np.interp(wanted, arc, polyline[:, 1])])
    return out, wanted


def grade_profile(grid, land, polyline, baseline_m=tg.SLOPE_BASELINE_M):
    """Along-path grade and cross-path roll, in degrees, at 0.2 m spacing.

    One slope number per cell is not enough for a wheelchair: 6 degrees is a
    speed limit going up it and a tip risk going across it. The surface gradient
    is therefore projected onto the direction of travel and onto its normal.
    """
    dense, arc = resample(np.asarray(polyline, float), 0.2)
    if len(dense) < 3:
        return dense, arc, np.zeros(len(dense)), np.zeros(len(dense))
    heading = np.gradient(dense, axis=0)
    norm = np.hypot(heading[:, 0], heading[:, 1])
    norm[norm <= 0.0] = 1.0
    heading = heading / norm[:, None]
    slope_x = sample_surface(grid, land["slope_x"], dense)
    slope_y = sample_surface(grid, land["slope_y"], dense)
    along = slope_x * heading[:, 0] + slope_y * heading[:, 1]
    cross = -slope_x * heading[:, 1] + slope_y * heading[:, 0]
    return (dense, arc, np.degrees(np.arctan(np.abs(along))),
            np.degrees(np.arctan(np.abs(cross))))


def trace_grade(route_xy, route_z, baseline_m=1.0):
    """Grade straight from the recorded pose z - what the chair actually drove."""
    leg = np.hypot(*np.diff(route_xy, axis=0).T)
    arc = np.concatenate([[0.0], np.cumsum(leg)])
    ahead = np.clip(np.searchsorted(arc, arc + baseline_m), 0, len(arc) - 1)
    run = arc[ahead] - arc
    usable = run > baseline_m * 0.5
    rise = np.abs(route_z[ahead] - route_z)
    grade = np.zeros(len(arc))
    grade[usable] = np.degrees(np.arctan(rise[usable] / run[usable]))
    return grade, arc, usable


def graph_components(adjacency):
    """Sizes of the connected pieces of an adjacency list, largest first."""
    seen = [-1] * len(adjacency)
    sizes = []
    for start in range(len(adjacency)):
        if seen[start] >= 0:
            continue
        stack, size = [start], 0
        seen[start] = len(sizes)
        while stack:
            node = stack.pop()
            size += 1
            for other, _ in adjacency[node]:
                if seen[other] < 0:
                    seen[other] = len(sizes)
                    stack.append(other)
        sizes.append(size)
    return sorted(sizes, reverse=True)


def surface_grade_along(grid, land, polyline, baseline_m=1.0):
    """Grade along a path from the map's ground height, 1-D and step-immune.

    The projected gradient (grade_profile) uses a 1.95 m square window, so
    wherever the path runs within a metre of a kerb it measures the kerb. This
    walks the ground height along the path instead, exactly as trace_grade()
    walks the recorded pose z, which makes the two directly comparable and is
    the number to check against the 10.35 degrees the chair demonstrated.
    """
    dense, arc = resample(np.asarray(polyline, float), 0.2)
    height = sample_surface(grid, land["ground"], dense)
    ahead = np.clip(np.searchsorted(arc, arc + baseline_m), 0, len(arc) - 1)
    run = arc[ahead] - arc
    usable = run > baseline_m * 0.5
    grade = np.zeros(len(arc))
    grade[usable] = np.degrees(np.arctan(
        np.abs(height[ahead] - height)[usable] / run[usable]))
    return dense, arc, grade, usable


# --- rendering ------------------------------------------------------------
def class_raster(grid, land, masks):
    image = np.zeros((grid["ny"], grid["nx"], 3), np.uint8)
    image[...] = INK["ground"]
    inside = masks["inside"]
    image[inside & tg.keep_out(grid["known"])] = INK["unmapped"]
    image[inside & grid["known"]] = INK["free"]
    image[masks["reachable"]] = INK["reachable"]
    image[inside & masks["steep"]] = INK["steep"]
    image[inside & masks["stepped"]] = INK["step"]
    image[inside & masks["obstruction"]] = INK["obstruction"]
    return image


def slope_raster(grid, land, masks):
    stops = np.array([s for s, _ in SLOPE_RAMP])
    colours = np.array([c for _, c in SLOPE_RAMP], float)
    value = np.clip(land["slope_deg"], stops[0], stops[-1])
    image = np.zeros((grid["ny"], grid["nx"], 3), np.uint8)
    for channel in range(3):
        image[..., channel] = np.interp(value, stops,
                                        colours[:, channel]).astype(np.uint8)
    dark = tg.keep_out(masks["inside"] & grid["known"])
    image[dark] = INK["ground"]
    return image


def draw_overlays(image, grid, scale, recorded, planned, nodes_xy, edges_xy,
                  blockages):
    canvas = Image.fromarray(image[::-1]).resize(
        (grid["nx"] * scale, grid["ny"] * scale), Image.NEAREST)
    pen = ImageDraw.Draw(canvas, "RGBA")

    def to_px(points):
        points = np.asarray(points, float).reshape(-1, 2)
        px = (points[:, 0] - grid["min_x"]) / grid["cell"] * scale
        py = (grid["ny"] - (points[:, 1] - grid["min_y"]) / grid["cell"]) * scale
        return [tuple(v) for v in np.column_stack([px, py])]

    for a, b in edges_xy:
        pen.line(to_px([a, b]), fill=INK["edge"] + (70,), width=max(scale // 2, 1))
    if len(nodes_xy):
        for x, y in to_px(nodes_xy):
            radius = max(scale // 2, 1)
            pen.ellipse([x - radius, y - radius, x + radius, y + radius],
                        fill=INK["node"] + (200,))
    pen.line(to_px(recorded), fill=INK["driven"] + (210,), width=max(scale, 1))
    if planned is not None:
        pen.line(to_px(planned), fill=INK["path"] + (255,),
                 width=max(2 * scale, 2), joint="curve")
    for xy, ok in blockages:
        colour = (110, 200, 140, 220) if ok else (220, 90, 90, 230)
        radius = RESILIENCE_RADIUS_M / grid["cell"] * scale
        x, y = to_px([xy])[0]
        pen.ellipse([x - radius, y - radius, x + radius, y + radius],
                    outline=colour, width=max(scale, 1))
    return canvas


def png_bytes(image):
    from io import BytesIO
    buffer = BytesIO()
    Image.fromarray(image[::-1]).save(buffer, format="PNG", optimize=True)
    return buffer.getvalue()


def island_survey(grid, land, masks, route_xy, reach_m=0.30):
    """How the route breaks up when nothing is injected.

    Without the driven-ribbon injection the configuration space is not one
    corridor but a chain of disconnected islands. Walking the route and asking
    which island each station falls in turns "no path" into a measurement: how
    many islands, how much of the route they cover, and the longest run of route
    that stays inside one of them - the furthest a start-and-goal-only plan can
    get before the map stops being able to place the chair.
    """
    cell = grid["cell"]
    labels, total = ndimage.label(masks["centre_free"])
    if not total:
        return dict(islands=0, covered=0.0, longest_run_m=0.0, runs=0,
                    run_median_m=0.0, route_length_m=0.0), labels
    nearest = labels[tuple(ndimage.distance_transform_edt(
        labels == 0, return_distances=False, return_indices=True))]
    distance = ndimage.distance_transform_edt(labels == 0) * cell
    rows = np.clip(np.rint((route_xy[:, 1] - grid["min_y"]) / cell
                           - 0.5).astype(int), 0, grid["ny"] - 1)
    cols = np.clip(np.rint((route_xy[:, 0] - grid["min_x"]) / cell
                           - 0.5).astype(int), 0, grid["nx"] - 1)
    at_station = np.where(distance[rows, cols] <= reach_m,
                          nearest[rows, cols], 0)

    leg = np.hypot(*np.diff(route_xy, axis=0).T)
    arc = np.concatenate([[0.0], np.cumsum(leg)])
    runs, begin = [], None
    for i, label in enumerate(at_station):
        if label and (begin is None or at_station[begin] != label):
            if begin is not None:
                runs.append((arc[begin], arc[i - 1], int(at_station[begin])))
            begin = i
        elif not label and begin is not None:
            runs.append((arc[begin], arc[i - 1], int(at_station[begin])))
            begin = None
    if begin is not None:
        runs.append((arc[begin], arc[-1], int(at_station[begin])))
    spans = [b - a for a, b, _ in runs]
    return dict(islands=int(len({label for _, _, label in runs})),
                covered=float((at_station > 0).mean()),
                longest_run_m=float(max(spans) if spans else 0.0),
                runs=int(len(runs)),
                run_median_m=float(np.median(spans)) if spans else 0.0,
                route_length_m=float(arc[-1])), labels


def refusal_breakdown(masks, boundary):
    """Of the cells that stop the chair, which refusal names each one."""
    total = int(boundary.sum())
    if not total:
        return {}
    named = {key: float((boundary & masks[key]).sum() / total)
             for key in ("obstruction", "stepped", "steep")}
    any_named = np.logical_or(
        np.logical_or(masks["obstruction"], masks["stepped"]), masks["steep"])
    # refused by none of the three: the ground is free, just too narrow to
    # place the chair in once the half width and margin are eroded out
    named["too_narrow_only"] = float(
        (boundary & tg.keep_out(any_named)).sum() / total)
    return named


def unassisted(grid, land, start_xy, goal_xy, tree, route_xy, out_dir):
    """Plan again with the driven-ribbon injection withdrawn.

    The headline path depends on treating the recorded line as drivable. This
    measures how much of it survives without that, which is the only honest way
    to report what the map alone supports.
    """
    masks = tg.traversability(grid, land, trust_driven=False, seed_xy=start_xy)
    area = grid["cell"] ** 2
    driven = masks["driven"]
    result = dict(
        reachable_m2=float(masks["reachable"].sum() * area),
        free_m2=float(masks["free"].sum() * area),
        drive_inside_reachable=float((masks["reachable"] & driven).sum()
                                     / max(driven.sum(), 1)))
    half_width = ndimage.distance_transform_edt(masks["free"]) * grid["cell"]
    on_line = half_width[driven & grid["known"]]
    needed = tg.CHAIR_HALF_WIDTH_M + tg.BAND_MARGIN_M
    result["free_half_width_on_line_m"] = dict(
        median=float(np.median(on_line)),
        p25=float(np.percentile(on_line, 25)),
        fraction_wide_enough=float((on_line >= needed).mean()),
        needed=float(needed))

    net, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    result["path_m"] = 0.0
    plan_xy = None
    if net is not None:
        cells, _ = tg.grid_plan(net, index, rows, cols, grid, start_xy, goal_xy)
        if cells is not None:
            plan_xy = np.column_stack([
                grid["min_x"] + (cells[:, 1] + 0.5) * grid["cell"],
                grid["min_y"] + (cells[:, 0] + 0.5) * grid["cell"]])
            result["path_m"] = float(
                np.hypot(*np.diff(plan_xy, axis=0).T).sum())

    survey, labels = island_survey(grid, land, masks, route_xy)
    result["islands"] = survey

    boundary = np.logical_and(
        ndimage.binary_dilation(masks["centre_free"], structure=tg.disk(2)),
        tg.keep_out(masks["centre_free"]))
    boundary = np.logical_and(boundary, masks["inside"])
    result["refused_by"] = refusal_breakdown(masks, boundary)

    image = np.zeros((grid["ny"], grid["nx"], 3), np.uint8)
    image[...] = INK["ground"]
    image[masks["inside"] & tg.keep_out(grid["known"])] = INK["unmapped"]
    image[masks["inside"] & grid["known"]] = INK["free"]
    image[masks["inside"] & masks["stepped"]] = INK["step"]
    image[masks["inside"] & masks["obstruction"]] = INK["obstruction"]
    image[masks["inside"] & masks["steep"]] = INK["steep"]
    # each island gets its own hue so the fragmentation is countable by eye
    if labels.max():
        rng = np.random.default_rng(4)
        palette = (60 + rng.random((labels.max() + 1, 3)) * 195).astype(np.uint8)
        palette[0] = INK["ground"]
        speck = masks["centre_free"]
        image[speck] = palette[labels[speck]]
    canvas = draw_overlays(image, grid, 2, route_xy, plan_xy,
                           np.zeros((0, 2)), [], [])
    canvas.save(os.path.join(out_dir, "plan-unassisted.png"))
    return result


# --- main -----------------------------------------------------------------
def main(map_path, route_path, out_dir):
    started = time.time()
    os.makedirs(out_dir, exist_ok=True)
    route_doc = json.load(open(route_path, encoding="utf-8"))
    waypoints = route_doc["waypoints"]
    route_xy = np.array([[w["x"], w["y"]] for w in waypoints])
    route_z = np.array([w["z"] for w in waypoints])
    note("route: %d points, %s, reference %s"
         % (len(waypoints), route_doc.get("frame"),
            route_doc.get("reference_point")))

    points = tg.load_cloud(map_path)
    note("map: %d points" % len(points))
    tree = cKDTree(route_xy)
    grid = tg.raster(points, route_xy, route_z, tree)
    land = tg.terrain(grid)
    masks = tg.traversability(grid, land)
    area = grid["cell"] ** 2
    note("grid %d x %d at %.2f m; %d cells measured (%.0f m2)"
         % (grid["nx"], grid["ny"], grid["cell"], int(grid["known"].sum()),
            grid["known"].sum() * area))

    # --- what the terrain refuses, and why
    labels, objects = ndimage.label(masks["obstruction"])
    note("refusals: obstruction %.0f m2 in %d objects | step %.0f m2 | "
         "grade>%.1f deg %.0f m2"
         % (masks["obstruction"].sum() * area, objects,
            masks["stepped"].sum() * area, tg.SLOPE_BLOCK_DEG,
            masks["steep"].sum() * area))
    note("free ground %.0f m2; reachable by the chair centre %.0f m2"
         % (masks["free"].sum() * area, masks["reachable"].sum() * area))

    # --- the check that matters: the drive happened, so it must be reachable
    covered = float((masks["reachable"] & masks["driven"]).sum()
                    / max(masks["driven"].sum(), 1))
    note("the recorded drive lies %.2f%% inside the reachable region"
         % (100 * covered))
    driven_slope = land["slope_deg"][masks["driven"]]
    measured_grade, arc_recorded, usable = trace_grade(route_xy, route_z)
    note("grade the chair demonstrated (pose z, 1 m baseline): median %.2f "
         "p99 %.2f max %.2f deg"
         % (np.median(measured_grade[usable]),
            np.percentile(measured_grade[usable], 99),
            measured_grade[usable].max()))
    note("map grade under the driven ribbon: median %.2f p99 %.2f max %.2f deg"
         % (np.median(driven_slope), np.percentile(driven_slope, 99),
            driven_slope.max()))
    if covered < 0.999:
        note("WARNING: the classification refuses ground the chair drove - the "
             "classification is wrong, not the drive")
    if measured_grade[usable].max() > tg.SLOPE_BLOCK_DEG:
        note("WARNING: SLOPE_BLOCK_DEG is below the demonstrated envelope")

    # --- the visibility graph, and the measurement of why it is not enough
    nodes_rc = tg.corner_nodes(masks["reachable"], grid["cell"])
    nodes_xy, adjacency = tg.build_graph(nodes_rc, grid, land, masks)
    edges = tg.graph_edges(adjacency)
    pieces = graph_components(adjacency)
    note("visibility graph: %d corner nodes, %d edges, %d connected pieces "
         "(largest %d)"
         % (len(nodes_rc), len(edges[0]), len(pieces), max(pieces or [0])))
    ribbon = ndimage.distance_transform_edt(masks["reachable"]) * grid["cell"]
    on_line = ribbon[masks["driven"]]
    note("chair-centre freedom on the recorded line: median %.2f m, p10 %.2f m "
         "- a corner graph has nothing to bend at in a ribbon this narrow"
         % (np.median(on_line), np.percentile(on_line, 10)))

    # --- the complete search, then tautened by the same visibility test
    start_xy = tuple(float(v) for v in route_xy[0])
    goal_xy = tuple(float(v) for v in route_xy[-1])
    cell_net, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    staircase, cost = tg.grid_plan(cell_net, index, rows, cols, grid,
                                   start_xy, goal_xy)
    if staircase is None:
        note("NO PATH from start to goal over the mapped, traversable surface")
        planned = None
    else:
        pulled = tg.taut(staircase, open_cell)
        planned = np.column_stack([
            grid["min_x"] + (pulled[:, 1] + 0.5) * grid["cell"],
            grid["min_y"] + (pulled[:, 0] + 0.5) * grid["cell"]])
        walked = float(np.hypot(*np.diff(planned, axis=0).T).sum())
        staircase_m = float(np.hypot(*np.diff(staircase, axis=0).T).sum()) \
            * grid["cell"]
        straight = float(np.hypot(*(np.array(goal_xy) - np.array(start_xy))))
        note("global path: %d segments, %.1f m walked (grid staircase %.1f m), "
             "cost %.1f | straight line %.1f m, recorded drive %.1f m"
             % (len(planned) - 1, walked, staircase_m, cost, straight,
                arc_recorded[-1]))
        dense, arc_plan, along, cross = grade_profile(grid, land, planned)
        deviation = tree.query(dense, k=1)[0]
        note("planned vs recorded line: median %.2f m, p95 %.2f m, max %.2f m"
             % (np.median(deviation), np.percentile(deviation, 95),
                deviation.max()))
        note("planned path grade: along median %.2f max %.2f deg | "
             "cross median %.2f max %.2f deg (1.95 m window - reads the kerb "
             "wherever the path hugs one)"
             % (np.median(along), along.max(), np.median(cross), cross.max()))
        _, _, walk_grade, walk_ok = surface_grade_along(grid, land, planned)
        _, _, line_grade, line_ok = surface_grade_along(grid, land, route_xy)
        note("grade along the surface (1 m baseline, step-immune):")
        note("    planned path : median %.2f p99 %.2f max %.2f deg"
             % (np.median(walk_grade[walk_ok]),
                np.percentile(walk_grade[walk_ok], 99),
                walk_grade[walk_ok].max()))
        note("    recorded line: median %.2f p99 %.2f max %.2f deg "
             "(same measurement on ground the chair provably drove)"
             % (np.median(line_grade[line_ok]),
                np.percentile(line_grade[line_ok], 99),
                line_grade[line_ok].max()))
        note("    chair's own pose z on that line: median %.2f p99 %.2f max "
             "%.2f deg" % (np.median(measured_grade[usable]),
                           np.percentile(measured_grade[usable], 99),
                           measured_grade[usable].max()))
        clearance = sample_surface(grid, masks["clearance"], dense)
        needed = tg.CHAIR_HALF_WIDTH_M + tg.BAND_MARGIN_M
        leaning = float((clearance < needed).mean())
        note("planned path clearance to refused ground: min %.2f m, "
             "median %.2f m; %.0f%% of it is under the %.2f m the chair needs "
             "and is legal only because the chair already drove there"
             % (clearance.min(), np.median(clearance), 100 * leaning, needed))

    # --- resilience: if the recorded line is blocked, is there another way?
    blockages = []
    if staircase is not None:
        stations, _ = resample(route_xy, RESILIENCE_STEP_M)
        detoured = 0
        for point in stations:
            shut = tg.cells_within(rows, cols, grid, point, RESILIENCE_RADIUS_M)
            found, _ = tg.grid_plan(cell_net, index, rows, cols, grid,
                                    start_xy, goal_xy, blocked_cells=shut)
            blockages.append((tuple(float(v) for v in point),
                              found is not None))
            detoured += found is not None
        note("resilience: a %.1f m obstruction placed every %.1f m along the "
             "recorded line - %d of %d stations still had a route (%.0f%%)"
             % (2 * RESILIENCE_RADIUS_M, RESILIENCE_STEP_M, detoured,
                len(blockages), 100.0 * detoured / max(len(blockages), 1)))


    # --- how much of that answer was the assumption, not the map
    bare = unassisted(grid, land, start_xy, goal_xy, tree, route_xy,
                      out_dir)
    note("without the driven-ribbon injection: reachable %.0f m2 (was %.0f), "
         "the recorded drive is %.1f%% inside it (was 100.0), longest route "
         "%.1f m (was %.1f)"
         % (bare["reachable_m2"], masks["reachable"].sum() * area,
            100 * bare["drive_inside_reachable"], bare["path_m"],
            0.0 if planned is None else walked))
    isl = bare["islands"]
    note("    with nothing injected the route is not one corridor: %d islands "
         "touch it in %d runs, they cover %.1f%% of its length, the longest "
         "unbroken run is %.1f m of %.1f m (median run %.1f m)"
         % (isl["islands"], isl["runs"], 100 * isl["covered"],
            isl["longest_run_m"], isl["route_length_m"], isl["run_median_m"]))
    named = bare["refused_by"]
    note("    what refuses the chair at the island edges: obstruction %.0f%%, "
         "step %.0f%%, grade %.0f%%, and %.0f%% is free ground that is simply "
         "too narrow to place the chair in"
         % (100 * named.get("obstruction", 0), 100 * named.get("stepped", 0),
            100 * named.get("steep", 0), 100 * named.get("too_narrow_only", 0)))
    width = bare["free_half_width_on_line_m"]
    note("    free half width under the recorded line: median %.2f m, p25 "
         "%.2f m; only %.1f%% of it reaches the %.2f m the chair needs to be "
         "placed at all - the map cannot certify ground the chair drove"
         % (width["median"], width["p25"], 100 * width["fraction_wide_enough"],
            width["needed"]))

    # --- artifacts
    classes = class_raster(grid, land, masks)
    slopes = slope_raster(grid, land, masks)

    keep = np.linspace(0, len(edges[0]) - 1, min(len(edges[0]), 3000)).astype(int)
    edges_xy = [(nodes_xy[edges[0][k]], nodes_xy[edges[1][k]]) for k in keep] \
        if len(edges[0]) else []

    overview = draw_overlays(classes, grid, 2, route_xy, planned, nodes_xy,
                             edges_xy, blockages)
    overview.save(os.path.join(out_dir, "plan-overview.png"))
    detail_box = (80.0, -28.0, 104.0, -6.0)
    detail = draw_overlays(classes, grid, 8, route_xy, planned, nodes_xy,
                           edges_xy, blockages)
    left = int((detail_box[0] - grid["min_x"]) / grid["cell"] * 8)
    right = int((detail_box[2] - grid["min_x"]) / grid["cell"] * 8)
    top = int((grid["ny"] - (detail_box[3] - grid["min_y"]) / grid["cell"]) * 8)
    bottom = int((grid["ny"] - (detail_box[1] - grid["min_y"]) / grid["cell"]) * 8)
    detail.crop((max(left, 0), max(top, 0), right, bottom)).save(
        os.path.join(out_dir, "plan-detail.png"))

    report = dict(
        map=os.path.basename(map_path), route=os.path.basename(route_path),
        cell=grid["cell"], corridor_m=tg.CORRIDOR_M,
        thresholds=dict(step_m=tg.STEP_M, slope_slow_deg=tg.SLOPE_SLOW_DEG,
                        slope_block_deg=tg.SLOPE_BLOCK_DEG,
                        slope_baseline_m=tg.SLOPE_BASELINE_M,
                        chair_half_width_m=tg.CHAIR_HALF_WIDTH_M,
                        band_margin_m=tg.BAND_MARGIN_M),
        areas_m2=dict(
            measured=float(grid["known"].sum() * area),
            obstruction=float(masks["obstruction"].sum() * area),
            stepped=float(masks["stepped"].sum() * area),
            steep=float(masks["steep"].sum() * area),
            free=float(masks["free"].sum() * area),
            reachable=float(masks["reachable"].sum() * area)),
        drive_inside_reachable=covered,
        without_driven_injection=bare,
        demonstrated_grade_deg=dict(
            median=float(np.median(measured_grade[usable])),
            p99=float(np.percentile(measured_grade[usable], 99)),
            max=float(measured_grade[usable].max())),
        recorded_length_m=float(arc_recorded[-1]),
        graph=dict(nodes=int(len(nodes_xy)), edges=int(len(edges[0])),
                   components=pieces[:8],
                   centre_freedom_median_m=float(np.median(on_line)),
                   centre_freedom_p10_m=float(np.percentile(on_line, 10))))
    if planned is not None:
        report["path"] = dict(
            segments=int(len(planned) - 1), length_m=walked,
            grid_staircase_m=staircase_m, cost=float(cost),
            straight_line_m=straight,
            deviation_m=dict(median=float(np.median(deviation)),
                             p95=float(np.percentile(deviation, 95)),
                             max=float(deviation.max())),
            grade_deg=dict(along_median=float(np.median(along)),
                           along_max=float(along.max()),
                           cross_median=float(np.median(cross)),
                           cross_max=float(cross.max()),
                           surface_along_median=float(
                               np.median(walk_grade[walk_ok])),
                           surface_along_p99=float(
                               np.percentile(walk_grade[walk_ok], 99)),
                           surface_along_max=float(walk_grade[walk_ok].max()),
                           recorded_surface_along_p99=float(
                               np.percentile(line_grade[line_ok], 99)),
                           recorded_surface_along_max=float(
                               line_grade[line_ok].max())),
            clearance_m=dict(min=float(clearance.min()),
                             median=float(np.median(clearance)),
                             needed=float(needed),
                             fraction_below_needed=leaning),
            vertices=[[float(x), float(y)] for x, y in planned])
        report["resilience"] = dict(
            radius_m=RESILIENCE_RADIUS_M, step_m=RESILIENCE_STEP_M,
            stations=len(blockages),
            with_a_route=int(sum(1 for _, ok in blockages if ok)),
            blocked=[list(xy) for xy, ok in blockages if not ok])
    json.dump(report, open(os.path.join(out_dir, "global_plan.json"), "w",
                           encoding="utf-8"), indent=2, ensure_ascii=False)

    np.savez_compressed(
        os.path.join(out_dir, "terrain.npz"), cell=grid["cell"],
        min_x=grid["min_x"], min_y=grid["min_y"],
        ground=land["ground"].astype(np.float32),
        slope_deg=land["slope_deg"].astype(np.float32),
        step_m=land["step"].astype(np.float32),
        **{key: masks[key] for key in ("driven", "obstruction", "stepped",
                                       "steep", "free", "reachable")})

    write_viewer(os.path.join(out_dir, "global_plan.html"), grid, classes,
                 slopes, route_xy, planned, nodes_xy, edges_xy, blockages,
                 report,
                 recorded_profile=(arc_recorded, measured_grade, usable),
                 planned_profile=(arc_plan, along, cross)
                 if planned is not None else None)
    note("wrote %s (%.1f s total)" % (out_dir, time.time() - started))


def write_viewer(path, grid, classes, slopes, recorded, planned, nodes_xy,
                 edges_xy, blockages, report, recorded_profile,
                 planned_profile):
    def encode(image):
        return base64.b64encode(png_bytes(image)).decode("ascii")

    def thin(points, step):
        points = np.asarray(points, float)
        return [[round(float(x), 3), round(float(y), 3)]
                for x, y in points[::step]]

    arc_r, grade_r, usable_r = recorded_profile
    payload = dict(
        cell=grid["cell"], minX=grid["min_x"], minY=grid["min_y"],
        nx=grid["nx"], ny=grid["ny"],
        recorded=thin(recorded, 2),
        planned=None if planned is None else thin(planned, 1),
        nodes=thin(nodes_xy, max(len(nodes_xy) // 2500, 1)),
        edges=[[round(float(a[0]), 2), round(float(a[1]), 2),
                round(float(b[0]), 2), round(float(b[1]), 2)]
               for a, b in edges_xy],
        blockages=[[round(xy[0], 2), round(xy[1], 2), 1 if ok else 0]
                   for xy, ok in blockages],
        recordedProfile=[[round(float(s), 2), round(float(g), 2)]
                         for s, g, u in zip(arc_r, grade_r, usable_r) if u][::4],
        plannedProfile=None if planned_profile is None else
        [[round(float(s), 2), round(float(a), 2), round(float(c), 2)]
         for s, a, c in zip(*planned_profile)][::4],
        report=report,
        slopeRamp=[[s, list(c)] for s, c in SLOPE_RAMP],
        ink={k: list(v) for k, v in INK.items()},
        blockRadius=RESILIENCE_RADIUS_M)

    html = VIEWER_HTML.replace("__CLASSES__", encode(classes)) \
                      .replace("__SLOPES__", encode(slopes)) \
                      .replace("__DATA__", json.dumps(payload,
                                                      ensure_ascii=False))
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(html)


VIEWER_HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>전역 경로 · 지형 통행성 · UNICON</title>
<style>
:root {
  color-scheme: dark;
  --ground: #0b1014;
  --panel: #121a20;
  --panel-edge: #1e2a33;
  --ink: #dce8ef;
  --ink-dim: #8ba3b1;
  --ink-faint: #5c7382;
  --accent: #4ee0d0;
  --warn: #d98324;
  --stop: #b3453c;
  --steep: #b5477f;
  --driven: #f2ead3;
  --mono: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace;
  --sans: "Helvetica Neue", Helvetica, Arial, system-ui, sans-serif;
}
* { box-sizing: border-box; }
html, body { margin: 0; height: 100%; }
body {
  background: var(--ground); color: var(--ink); font-family: var(--sans);
  display: grid; grid-template-columns: minmax(0, 1fr) 336px; height: 100vh;
}
@media (max-width: 900px) { body { grid-template-columns: 1fr; grid-template-rows: 1fr auto; } }

#stage { position: relative; overflow: hidden; background: var(--ground); }
#stage canvas { position: absolute; inset: 0; width: 100%; height: 100%; display: block; }
#hint {
  position: absolute; left: 14px; bottom: 12px; font-family: var(--mono);
  font-size: 11px; letter-spacing: .08em; color: var(--ink-faint);
  text-transform: uppercase; pointer-events: none;
}
#readout {
  position: absolute; right: 14px; top: 12px; font-family: var(--mono);
  font-size: 12px; color: var(--ink-dim); background: rgba(11,16,20,.78);
  border: 1px solid var(--panel-edge); padding: 7px 10px; border-radius: 2px;
  font-variant-numeric: tabular-nums; pointer-events: none; min-width: 176px;
}

aside {
  background: var(--panel); border-left: 1px solid var(--panel-edge);
  overflow-y: auto; padding: 20px 20px 32px; display: flex;
  flex-direction: column; gap: 22px;
}
h1 {
  font-size: 15px; font-weight: 600; margin: 0; letter-spacing: -.01em;
  text-wrap: balance;
}
h1 span { display: block; font-family: var(--mono); font-size: 10.5px;
  letter-spacing: .14em; text-transform: uppercase; color: var(--ink-faint);
  margin-top: 6px; font-weight: 400; }
h2 {
  font-family: var(--mono); font-size: 10.5px; letter-spacing: .14em;
  text-transform: uppercase; color: var(--ink-faint); margin: 0 0 9px;
  font-weight: 400;
}
section { border-top: 1px solid var(--panel-edge); padding-top: 16px; }
section:first-of-type { border-top: 0; padding-top: 0; }

.row { display: flex; align-items: baseline; justify-content: space-between;
  gap: 10px; padding: 3px 0; font-size: 12.5px; }
.row dt { color: var(--ink-dim); }
.row dd { margin: 0; font-family: var(--mono); font-variant-numeric: tabular-nums; }
dl { margin: 0; }

.legend { display: flex; flex-direction: column; gap: 5px; }
.legend label { display: flex; align-items: center; gap: 9px; font-size: 12.5px;
  cursor: pointer; user-select: none; }
.legend input { appearance: none; width: 13px; height: 13px; border-radius: 2px;
  border: 1px solid var(--ink-faint); background: transparent; cursor: pointer;
  flex: none; }
.legend input:checked { background: var(--ink-dim); border-color: var(--ink-dim); }
.legend input:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
.swatch { width: 22px; height: 10px; border-radius: 1px; flex: none; }
.swatch.line { height: 3px; }

.tabs { display: flex; gap: 0; border: 1px solid var(--panel-edge);
  border-radius: 2px; overflow: hidden; }
.tabs button { flex: 1; background: transparent; border: 0; color: var(--ink-dim);
  font-family: var(--mono); font-size: 11px; letter-spacing: .1em; padding: 7px 0;
  cursor: pointer; text-transform: uppercase; }
.tabs button[aria-pressed="true"] { background: var(--panel-edge); color: var(--ink); }
.tabs button:focus-visible { outline: 2px solid var(--accent); outline-offset: -2px; }

.ramp { height: 9px; border-radius: 1px; margin: 8px 0 4px; }
.ramp-scale { display: flex; justify-content: space-between;
  font-family: var(--mono); font-size: 10px; color: var(--ink-faint);
  font-variant-numeric: tabular-nums; }

#profile { width: 100%; height: 112px; display: block; }
.caption { font-size: 11.5px; line-height: 1.5; color: var(--ink-faint);
  margin: 8px 0 0; }
.flag { font-size: 11.5px; line-height: 1.55; color: var(--ink-dim);
  border-left: 2px solid var(--warn); padding: 2px 0 2px 10px; margin-top: 4px; }
button.reset { background: transparent; border: 1px solid var(--panel-edge);
  color: var(--ink-dim); font-family: var(--mono); font-size: 11px;
  letter-spacing: .1em; padding: 7px 10px; border-radius: 2px; cursor: pointer;
  text-transform: uppercase; }
button.reset:hover { color: var(--ink); border-color: var(--ink-faint); }
</style>
</head>
<body>
<main id="stage">
  <canvas id="view"></canvas>
  <div id="readout">커서를 지도 위에 두세요</div>
  <div id="hint">드래그 이동 · 휠 확대</div>
</main>
<aside>
  <h1>지형 통행성과 전역 경로<span>terrain traversability · visibility-graph plan</span></h1>

  <section>
    <h2>바탕</h2>
    <div class="tabs" role="group" aria-label="바탕 레이어">
      <button id="tab-class" aria-pressed="true">분류</button>
      <button id="tab-slope" aria-pressed="false">경사</button>
    </div>
    <div id="ramp-wrap" hidden>
      <div class="ramp" id="ramp"></div>
      <div class="ramp-scale"><span>0°</span><span>3°</span><span>7°</span><span>12°</span><span>25°+</span></div>
      <p class="caption">지면 표면의 국소 평면 기울기. 3°는 주행기가 이미 감속하는 지점, 12°는 거부 임계입니다.</p>
    </div>
  </section>

  <section>
    <h2>레이어</h2>
    <div class="legend" id="legend"></div>
  </section>

  <section>
    <h2>전역 경로</h2>
    <dl id="stats"></dl>
  </section>

  <section>
    <h2>경사 프로파일</h2>
    <canvas id="profile"></canvas>
    <p class="caption" id="profile-caption"></p>
  </section>

  <section>
    <h2>막혔을 때</h2>
    <dl id="resilience"></dl>
    <p class="caption">기록선을 따라 일정 간격으로 지름 3 m 장애를 놓고 매번 다시 계획했습니다. 초록 원은 우회로가 있던 지점, 빨간 원은 지도 범위 안에 우회로가 없던 지점입니다.</p>
  </section>

  <section>
    <h2>기록선을 빼면</h2>
    <dl id="bare"></dl>
    <p class="caption">위 경로는 <strong>기록된 0727 선을 통행 가능으로 주입한 상태</strong>에서 나온 것입니다. 그 주입을 빼면 지도만으로 확보되는 면적과 경로가 아래와 같습니다. 차이가 곧 가정의 크기입니다.</p>
  </section>

  <section>
    <h2>이 그림이 허가하지 않는 것</h2>
    <p class="flag">커밋된 맵에 대한 오프라인 해석입니다. 실주행 권한, 캠퍼스 운행 권한, 탑승 권한을 부여하지 않습니다. 경로는 측량되지 않았고, 임계값은 0727 주행이 실제로 보여준 범위에서 역산한 것입니다.</p>
    <button class="reset" id="reset">시야 초기화</button>
  </section>
</aside>

<script>
const DATA = __DATA__;
const layers = {
  base:      { label: "바탕 래스터", on: true, swatch: "#33424b" },
  edges:     { label: "가시성 그래프 (표본)", on: false, swatch: "#2f5d7a", line: true },
  nodes:     { label: "코너 노드", on: false, swatch: "#6ea8c7" },
  recorded:  { label: "기록 주행선 (0727)", on: true, swatch: "#f2ead3", line: true },
  planned:   { label: "전역 경로", on: true, swatch: "#4ee0d0", line: true },
  blockages: { label: "차단 시험 지점", on: false, swatch: "#6ec88c" },
};
let baseMode = "class";

const images = { class: new Image(), slope: new Image() };
images.class.src = "data:image/png;base64,__CLASSES__";
images.slope.src = "data:image/png;base64,__SLOPES__";

const canvas = document.getElementById("view");
const ctx = canvas.getContext("2d");
const view = { scale: 1, x: 0, y: 0, fitted: false };

function toPx(x, y) {
  return [(x - DATA.minX) / DATA.cell, DATA.ny - (y - DATA.minY) / DATA.cell];
}
function toWorld(px, py) {
  return [px * DATA.cell + DATA.minX, (DATA.ny - py) * DATA.cell + DATA.minY];
}

function resize() {
  const rect = canvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  if (!view.fitted) fit();
  draw();
}
function fit() {
  const pad = 24 * (window.devicePixelRatio || 1);
  view.scale = Math.min((canvas.width - pad) / DATA.nx,
                        (canvas.height - pad) / DATA.ny);
  view.x = (canvas.width - DATA.nx * view.scale) / 2;
  view.y = (canvas.height - DATA.ny * view.scale) / 2;
  view.fitted = true;
}

function polyline(points, width, colour, alpha) {
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.globalAlpha = alpha === undefined ? 1 : alpha;
  ctx.strokeStyle = colour;
  ctx.lineWidth = width;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  points.forEach(([x, y], i) => {
    const [px, py] = toPx(x, y);
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  });
  ctx.stroke();
  ctx.restore();
}

function draw() {
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = "#0b1014";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.setTransform(view.scale, 0, 0, view.scale, view.x, view.y);
  ctx.imageSmoothingEnabled = view.scale < 1;

  if (layers.base.on) {
    const image = images[baseMode];
    if (image.complete) ctx.drawImage(image, 0, 0, DATA.nx, DATA.ny);
  }
  const unit = 1 / view.scale;

  if (layers.edges.on) {
    ctx.save();
    ctx.globalAlpha = 0.34;
    ctx.strokeStyle = "#2f5d7a";
    ctx.lineWidth = Math.max(unit, 0.5);
    ctx.beginPath();
    DATA.edges.forEach(([ax, ay, bx, by]) => {
      const a = toPx(ax, ay), b = toPx(bx, by);
      ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
    });
    ctx.stroke();
    ctx.restore();
  }
  if (layers.nodes.on) {
    ctx.fillStyle = "#6ea8c7";
    const r = Math.max(unit * 1.6, 0.8);
    DATA.nodes.forEach(([x, y]) => {
      const [px, py] = toPx(x, y);
      ctx.beginPath(); ctx.arc(px, py, r, 0, 6.2832); ctx.fill();
    });
  }
  if (layers.blockages.on) {
    const r = DATA.blockRadius / DATA.cell;
    ctx.lineWidth = Math.max(unit, 0.6);
    DATA.blockages.forEach(([x, y, ok]) => {
      const [px, py] = toPx(x, y);
      ctx.strokeStyle = ok ? "#6ec88c" : "#d95c5c";
      ctx.beginPath(); ctx.arc(px, py, r, 0, 6.2832); ctx.stroke();
    });
  }
  if (layers.recorded.on) polyline(DATA.recorded, Math.max(unit * 1.4, 0.9), "#f2ead3", 0.85);
  if (layers.planned.on) polyline(DATA.planned, Math.max(unit * 2.6, 1.6), "#4ee0d0", 1);
}

// --- interaction
let drag = null;
canvas.addEventListener("pointerdown", (event) => {
  drag = { x: event.clientX, y: event.clientY };
  canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener("pointerup", () => { drag = null; });
canvas.addEventListener("pointermove", (event) => {
  const dpr = window.devicePixelRatio || 1;
  if (drag) {
    view.x += (event.clientX - drag.x) * dpr;
    view.y += (event.clientY - drag.y) * dpr;
    drag = { x: event.clientX, y: event.clientY };
    draw();
  }
  const rect = canvas.getBoundingClientRect();
  const px = ((event.clientX - rect.left) * dpr - view.x) / view.scale;
  const py = ((event.clientY - rect.top) * dpr - view.y) / view.scale;
  const [wx, wy] = toWorld(px, py);
  document.getElementById("readout").textContent =
    "x " + wx.toFixed(2) + " m\ny " + wy.toFixed(2) + " m";
});
canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const cx = (event.clientX - rect.left) * dpr;
  const cy = (event.clientY - rect.top) * dpr;
  const factor = Math.exp(-event.deltaY * 0.0016);
  const next = Math.min(Math.max(view.scale * factor, 0.15), 40);
  view.x = cx - (cx - view.x) * (next / view.scale);
  view.y = cy - (cy - view.y) * (next / view.scale);
  view.scale = next;
  draw();
}, { passive: false });
document.getElementById("reset").addEventListener("click", () => {
  view.fitted = false; fit(); draw();
});

// --- panel
const legend = document.getElementById("legend");
Object.entries(layers).forEach(([key, layer]) => {
  const label = document.createElement("label");
  const box = document.createElement("input");
  box.type = "checkbox"; box.checked = layer.on;
  box.addEventListener("change", () => { layer.on = box.checked; draw(); });
  const swatch = document.createElement("span");
  swatch.className = "swatch" + (layer.line ? " line" : "");
  swatch.style.background = layer.swatch;
  const text = document.createElement("span");
  text.textContent = layer.label;
  label.append(box, swatch, text);
  legend.append(label);
});

function setBase(mode) {
  baseMode = mode;
  document.getElementById("tab-class").setAttribute("aria-pressed", mode === "class");
  document.getElementById("tab-slope").setAttribute("aria-pressed", mode === "slope");
  document.getElementById("ramp-wrap").hidden = mode !== "slope";
  draw();
}
document.getElementById("tab-class").addEventListener("click", () => setBase("class"));
document.getElementById("tab-slope").addEventListener("click", () => setBase("slope"));

document.getElementById("ramp").style.background =
  "linear-gradient(90deg," + DATA.slopeRamp.map(([stop, rgb]) =>
    "rgb(" + rgb.join(",") + ") " + (stop / 25 * 100).toFixed(1) + "%").join(",") + ")";

function rows(target, pairs) {
  const node = document.getElementById(target);
  node.innerHTML = "";
  pairs.forEach(([term, value]) => {
    const row = document.createElement("div");
    row.className = "row";
    const dt = document.createElement("dt"); dt.textContent = term;
    const dd = document.createElement("dd"); dd.textContent = value;
    row.append(dt, dd); node.append(row);
  });
}
const R = DATA.report;
if (R.path) {
  rows("stats", [
    ["계획 길이", R.path.length_m.toFixed(1) + " m"],
    ["기록 주행 길이", R.recorded_length_m.toFixed(1) + " m"],
    ["직선 거리", R.path.straight_line_m.toFixed(1) + " m"],
    ["구간 수", String(R.path.segments)],
    ["기록선과 차이 (중앙/최대) — 기록선 주입 상태",
     R.path.deviation_m.median.toFixed(2) + " / " + R.path.deviation_m.max.toFixed(2) + " m"],
    ["진행방향 경사 (중앙/최대)",
     R.path.grade_deg.along_median.toFixed(1) + " / " + R.path.grade_deg.along_max.toFixed(1) + "°"],
    ["횡경사 (중앙/최대)",
     R.path.grade_deg.cross_median.toFixed(1) + " / " + R.path.grade_deg.cross_max.toFixed(1) + "°"],
    ["거부지형까지 최소 여유", R.path.clearance_m.min.toFixed(2) + " m"],
    ["그래프", R.graph.nodes + " 노드 / " + R.graph.edges + " 엣지"],
  ]);
} else {
  rows("stats", [["결과", "경로 없음"]]);
}
const B = R.without_driven_injection;
if (B) {
  rows("bare", [
    ["중심 이동가능 면적", B.reachable_m2.toFixed(0) + " m² (주입 시 " +
      R.areas_m2.reachable.toFixed(0) + " m²)"],
    ["기록선이 그 안에 든 비율", (100 * B.drive_inside_reachable).toFixed(1) + "%"],
    ["지도만으로 나온 최장 경로", B.path_m.toFixed(1) + " m"],
    ["기록선 아래 통행가능 반폭", B.free_half_width_on_line_m.median.toFixed(2) +
      " m (중앙), p25 " + B.free_half_width_on_line_m.p25.toFixed(2) + " m"],
    ["휠체어에 필요한 " + B.free_half_width_on_line_m.needed.toFixed(2) + " m 확보",
     (100 * B.free_half_width_on_line_m.fraction_wide_enough).toFixed(1) + "%"],
  ]);
}
if (R.resilience) {
  rows("resilience", [
    ["시험 지점", String(R.resilience.stations)],
    ["우회로 있음", R.resilience.with_a_route + " (" +
      (100 * R.resilience.with_a_route / R.resilience.stations).toFixed(0) + "%)"],
    ["우회 불가", String(R.resilience.stations - R.resilience.with_a_route)],
    ["장애 지름", (2 * R.resilience.radius_m).toFixed(1) + " m"],
  ]);
}

// --- slope profile
const profile = document.getElementById("profile");
function drawProfile() {
  const dpr = window.devicePixelRatio || 1;
  const rect = profile.getBoundingClientRect();
  profile.width = rect.width * dpr;
  profile.height = 112 * dpr;
  const g = profile.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w = rect.width, h = 112, pad = { l: 26, r: 4, t: 8, b: 16 };
  g.clearRect(0, 0, w, h);
  const maxS = Math.max(
    DATA.recordedProfile.length ? DATA.recordedProfile[DATA.recordedProfile.length - 1][0] : 1,
    DATA.plannedProfile && DATA.plannedProfile.length ? DATA.plannedProfile[DATA.plannedProfile.length - 1][0] : 1);
  const maxG = 14;
  const X = (s) => pad.l + (s / maxS) * (w - pad.l - pad.r);
  const Y = (v) => h - pad.b - (Math.min(v, maxG) / maxG) * (h - pad.t - pad.b);

  g.strokeStyle = "#1e2a33"; g.lineWidth = 1;
  g.font = "9px ui-monospace, monospace"; g.fillStyle = "#5c7382";
  [0, 3, 7, 12].forEach((v) => {
    g.beginPath(); g.moveTo(pad.l, Y(v)); g.lineTo(w - pad.r, Y(v)); g.stroke();
    g.fillText(v + "°", 4, Y(v) + 3);
  });
  g.strokeStyle = "#b5477f"; g.setLineDash([3, 3]);
  g.beginPath(); g.moveTo(pad.l, Y(12)); g.lineTo(w - pad.r, Y(12)); g.stroke();
  g.setLineDash([]);

  function line(series, ix, colour, width, alpha) {
    if (!series || !series.length) return;
    g.save(); g.globalAlpha = alpha; g.strokeStyle = colour; g.lineWidth = width;
    g.beginPath();
    series.forEach((row, i) => {
      const x = X(row[0]), y = Y(row[ix]);
      if (i === 0) g.moveTo(x, y); else g.lineTo(x, y);
    });
    g.stroke(); g.restore();
  }
  line(DATA.recordedProfile, 1, "#f2ead3", 1, 0.75);
  if (DATA.plannedProfile) {
    line(DATA.plannedProfile, 2, "#b5477f", 1, 0.6);
    line(DATA.plannedProfile, 1, "#4ee0d0", 1.6, 1);
  }
}
document.getElementById("profile-caption").innerHTML =
  '<span style="color:#f2ead3">기록 주행 실측 경사</span> · ' +
  '<span style="color:#4ee0d0">계획 경로 진행방향</span> · ' +
  '<span style="color:#b5477f">계획 경로 횡경사</span>. 점선은 12° 거부선.';

images.class.onload = () => { resize(); };
images.slope.onload = () => { draw(); };
window.addEventListener("resize", () => { resize(); drawProfile(); });
resize(); drawProfile();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
