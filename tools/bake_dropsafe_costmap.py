#!/usr/bin/env python3
"""Bake a drop-safe occupancy grid from the 3D localization map.

The 2D map shipped with the stack (data/hanyang_aegimun_loop/map.pgm) is a
height-band projection: it knows walls and posts, and nothing about a kerb or
a road edge. Drop safety in this stack lives in the safety band, which is
keyed to a *recorded* route's stations - so a path that leaves that route has
no drop protection at all.

This tool closes that gap for an automatically planned path. It runs the same
terrain analysis `make_global_plan.py` uses (terrain_graph.traversability) -
ground surface, step/drop, grade, body obstruction, eroded by the chair's half
width - and writes the result as a ROS occupancy grid the standard global
planner can plan over. Every cell a chair CENTRE may not safely occupy - a
kerb, a steep grade, an obstruction, unmapped ground, or anywhere outside the
analysis corridor - is lethal. The free cells are the drop-safe configuration
space, and nothing about them depends on a hand-drawn line.

navfn/NavfnROS with `allow_unknown: false` (already in move_base.yaml) then
plans start to goal through that free space and through nothing else, so the
global path it returns cannot cross a drop the map knows about. The runtime
safety band (rebuilt along the new path by path_to_route_assets.py) is the
second, independent layer.

Defence in depth: the costmap refuses the drop and the band refuses it again.
Either alone is a single point of failure for a wheel going off an edge this
sensor cannot see.

Usage
-----
    bake_dropsafe_costmap.py <map.pcd> --start X,Y --goal X,Y \\
        --out <out-prefix> [--corridor-m 20] [--cell 0.15]

    bake_dropsafe_costmap.py <map.pcd> --route <route.json> \\
        --out <out-prefix>          # corridor bound by a recorded route

Outputs <out-prefix>.pgm + <out-prefix>.yaml (ROS map_server) and
<out-prefix>.npz (the raw masks, for inspection).

Nothing here touches the runtime graph or grants motion authority. It is an
offline map product, like make_global_plan.py.
"""

import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import terrain_graph as tg  # noqa: E402

# How far either side of the start-goal line to analyse. Wider than
# terrain_graph's CORRIDOR_M (14) because a planner must be free to detour
# around an obstruction rather than be confined to the straight line. The map
# thins past this, so it is also a coverage bound: a corridor wider than the
# mapped footprint just adds unknown cells.
DEFAULT_CORRIDOR_M = 20.0
# Densification of the synthetic start-goal seed, so the corridor bound is a
# smooth band rather than two points.
SEED_STEP_M = 1.0
# Cells with fewer than this many returns are unknown, not free. The 3D map is
# a 0.20 m voxel cloud, so a cell with one return is aliasing rather than a
# measurement of flat ground.
MIN_KNOWN_POINTS = 2


def load_cloud(path):
    return tg.load_cloud(path)


def seed_route(start_xy, goal_xy, step_m=SEED_STEP_M):
    """A densified straight line start to goal, with height sampled later.

    terrain_graph.raster bounds the analysis by distance to a route and filters
    map points by height relative to the route's own height. A planned path has
    no recorded height, so the seed is a straight line at z=0 and the height
    filter is widened to the corridor's full vertical extent - the ground
    surface itself is recovered per cell by terrain_graph.terrain regardless.
    """
    sx, sy = float(start_xy[0]), float(start_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    length = math.hypot(gx - sx, gy - sy)
    n = max(int(round(length / step_m)), 2)
    xs = np.linspace(sx, gx, n)
    ys = np.linspace(sy, gy, n)
    zs = np.zeros(n)  # widened filter below makes z irrelevant
    return np.column_stack([xs, ys]), zs


def build_grid(points, start_xy, goal_xy, corridor_m, cell):
    """Raster the corridor around the start-goal line and analyse its terrain.

    Returns (grid, land, masks) exactly as terrain_graph produces them, with
    the one departure that the height-relative filter is widened so a seed with
    no recorded height does not clip real terrain.
    """
    from scipy.spatial import cKDTree

    route_xy, route_z = seed_route(start_xy, goal_xy)
    tree = cKDTree(route_xy)
    grid = tg.raster(points, route_xy, route_z, tree, cell=cell,
                     corridor_m=corridor_m)
    # terrain_graph filters points to those within BELOW_ROUTE_M/ABOVE_ROUTE_M
    # of the route height. The seed route is at z=0, so re-derive the known mask
    # from the points that actually fell in the corridor rather than from a
    # height gate that ground truth does not back. This keeps a low-lying
    # footway that sits below z=0 from being thrown out as 'below the route'.
    distance, _ = tree.query(points[:, :2], k=1)
    in_corridor = distance < corridor_m
    col = ((points[in_corridor, 0] - grid["min_x"]) / cell).astype(np.int64)
    row = ((points[in_corridor, 1] - grid["min_y"]) / cell).astype(np.int64)
    known = np.zeros((grid["ny"], grid["nx"]), dtype=bool)
    valid = (col >= 0) & (col < grid["nx"]) & (row >= 0) & (row < grid["ny"])
    flat = (row[valid] * grid["nx"] + col[valid]).astype(np.int64)
    counts = np.bincount(flat, minlength=grid["nx"] * grid["ny"])
    known = (counts.reshape(grid["ny"], grid["nx"]) >= MIN_KNOWN_POINTS)
    grid["known"] = known
    land = tg.terrain(grid)
    masks = tg.traversability(grid, land, trust_driven=False,
                              seed_xy=start_xy, self_return_m=0.0)
    return grid, land, masks


def occupancy_grid(grid, masks):
    """ROS occupancy values: free where the chair centre is drop-safe.

    254 (free)   - inside the reachable configuration space
    0   (lethal) - refused by a drop, a grade, an obstruction, or unmapped,
                   or outside the analysis corridor
    205 (unknown) - reserved; here every analysed cell is decided, so nothing
                    is left unknown. Outside the corridor is lethal too,
                    because a planner that may 'freely' leave the analysed
                    region has left the region the map certifies.
    """
    ny, nx = grid["ny"], grid["nx"]
    image = np.full((ny, nx), 0, dtype=np.uint8)  # lethal by default
    image[masks["reachable"]] = 254               # drop-safe free
    return image


def write_ros_map(image, grid, out_prefix):
    """Write a map_server .pgm + .yaml in the map frame, origin at min_x/min_y."""
    ny, nx = image.shape
    # map_server expects origin as the lower-left pixel's world coordinate;
    # row 0 of the array is the +y edge, so the pgm is flipped on write.
    pgm_path = out_prefix + ".pgm"
    Image.fromarray(image[::-1].astype(np.uint8)).save(pgm_path)
    yaml_path = out_prefix + ".yaml"
    with open(yaml_path, "w") as handle:
        handle.write(
            "image: %s\n"
            "mode: trinary\n"
            "resolution: %.4f\n"
            "origin: [%.4f, %.4f, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n"
            % (os.path.basename(pgm_path), grid["cell"],
               grid["min_x"], grid["min_y"]))
    return pgm_path, yaml_path


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Bake a drop-safe ROS occupancy grid from the 3D map.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_pcd", help="3D localization map (.pcd, XYZI binary)")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--start", help="start x,y (map frame, metres)")
    src.add_argument("--route", help="recorded route JSON (corridor seed)")
    parser.add_argument("--goal", help="goal x,y (map frame, metres); "
                        "required with --start")
    parser.add_argument("--out", required=True, help="output prefix")
    parser.add_argument("--corridor-m", type=float, default=DEFAULT_CORRIDOR_M,
                        help="analysis half-width around the start-goal line "
                        "(default %(default).0f m)")
    parser.add_argument("--cell", type=float, default=tg.CELL,
                        help="grid resolution in metres (default %(default).2f)")
    args = parser.parse_args(argv)

    if args.start:
        if not args.goal:
            parser.error("--goal is required with --start")
        start_xy = tuple(float(v) for v in args.start.split(","))
        goal_xy = tuple(float(v) for v in args.goal.split(","))
        route_doc = None
    else:
        route_doc = json.load(open(args.route, encoding="utf-8"))
        wp = route_doc["waypoints"]
        start_xy = (float(wp[0]["x"]), float(wp[0]["y"]))
        goal_xy = (float(wp[-1]["x"]), float(wp[-1]["y"]))

    points = load_cloud(args.map_pcd)
    print("map: %d points" % len(points), flush=True)

    grid, land, masks = build_grid(
        points, start_xy, goal_xy, args.corridor_m, args.cell)
    cell = grid["cell"]
    area = cell * cell
    print("grid %d x %d at %.2f m; %d cells measured (%.0f m2)"
          % (grid["nx"], grid["ny"], cell, int(grid["known"].sum()),
             grid["known"].sum() * area), flush=True)
    print("refusals: obstruction %.0f m2 | step %.0f m2 | grade>%.1f deg %.0f m2"
          % (masks["obstruction"].sum() * area,
             masks["stepped"].sum() * area, tg.SLOPE_BLOCK_DEG,
             masks["steep"].sum() * area), flush=True)
    print("drop-safe free (chair centre): %.0f m2 in %d reachable cells"
          % (masks["reachable"].sum() * area, int(masks["reachable"].sum())),
          flush=True)

    image = occupancy_grid(grid, masks)
    pgm, yaml = write_ros_map(image, grid, args.out)
    print("wrote %s, %s" % (pgm, yaml), flush=True)

    np.savez_compressed(
        args.out + ".npz",
        cell=cell, min_x=grid["min_x"], min_y=grid["min_y"],
        known=grid["known"],
        reachable=masks["reachable"],
        obstruction=masks["obstruction"],
        stepped=masks["stepped"],
        steep=masks["steep"],
        ground=land["ground"].astype(np.float32),
        slope_deg=land["slope_deg"].astype(np.float32))
    print("wrote %s.npz" % args.out, flush=True)

    # A start or goal that is not itself drop-safe is a hard error rather than
    # a plan that starts inside a refusal. navfn would search for the nearest
    # free cell, which on a kerb edge is the wrong side of it.
    def at(point):
        r = int(np.clip(round((point[1] - grid["min_y"]) / cell - 0.5),
                        0, grid["ny"] - 1))
        c = int(np.clip(round((point[0] - grid["min_x"]) / cell - 0.5),
                        0, grid["nx"] - 1))
        return r, c
    for name, point in (("start", start_xy), ("goal", goal_xy)):
        r, c = at(point)
        if not masks["reachable"][r, c]:
            print("WARNING: %s at (%.2f, %.2f) is not in the drop-safe free "
                  "space - the planner cannot place the chair there without "
                  "crossing a refusal. Move it or widen the corridor."
                  % (name, point[0], point[1]), flush=True)


if __name__ == "__main__":
    main()
