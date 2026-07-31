#!/usr/bin/env python3
"""Where the chair can actually go, from the map itself rather than from rays.

The safety band answers one question per station per side - how far out does
the ground break - by walking a single lateral ray. That cannot describe a
pillar beside the path, an obstruction in the middle of the corridor, or a
kerb that runs at an angle to the route, and its per-station answers are
independent, so the boundary it produces is ragged where the real one is not.

This works on the cloud directly:

  1. Raster the corridor at CELL and take each cell's minimum height.
  2. Recover the ground surface by grey-opening that raster - erode to the
     local minimum, dilate back - which lifts off anything narrower than the
     structuring element (posts, bollards, benches, parked cars) while
     following the route's real slope, 11 m of it on this route.
  3. Call a cell obstructed if it holds returns in the volume the chair must
     pass through, and stepped if the GROUND surface breaks across it.
  4. Keep what the chair can reach: erode the free ground by the chair's half
     width plus its margin, then take the connected component containing the
     drive. Everything else is no-go, and the boundary comes out coherent
     because it is a region boundary rather than 380 independent ray answers.

The map contains the chair and its rider. Measured on merged_0707_0725, 64.7
percent of the cells the chair physically occupied hold returns at chair
height - more than the 47.5 percent just outside it - and the height spread
within 0.5 m of the driven line is 1.0 to 1.35 m against 0.10 m at 0.75 m out.
Those are self-returns, so the swept footprint is taken as proven drivable and
the obstruction test is not applied inside SELF_RETURN_M of the line. The
ground surface is built from cell minima and is unaffected either way.

Usage: make_traversability.py <map.pcd> <route.json> <out-prefix>
Writes <out-prefix>.npz (masks + ground) and <out-prefix>.png (a plan view).
"""

import json
import sys

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree


CELL = 0.15
CORRIDOR_M = 9.0
# Route-relative, so an 11 m climb does not turn into a global height gate.
BELOW_ROUTE_M = 3.0
ABOVE_ROUTE_M = 8.0
# Radius of the ground-opening element. Anything narrower than twice this is
# treated as an object standing ON the ground, not as ground.
GROUND_RADIUS_M = 1.5
# The volume the chair and rider actually sweep. Returns above this are canopy
# and overhead signage - a separate question from whether the wheels can pass.
BODY_LOW_M = 0.08
BODY_HIGH_M = 1.60
MIN_BODY_RETURNS = 2
# A ground break of this much across one cell stops the chair: kerbs, but also
# rough ground, which is equally undrivable and equally worth stopping for.
STEP_M = 0.06
CHAIR_HALF_WIDTH_M = 0.35
BAND_MARGIN_M = 0.10
SELF_RETURN_M = 0.70
# Specks are removed by component SIZE, never by an opening. An opening erodes
# with a 3x3 element, which deletes anything thinner than about 0.45 m - and a
# bollard is exactly that. At this cell size an object needs roughly 0.3 m
# across to register at all, which is also about where the 0.20 m voxel map
# stops resolving one.
MIN_OBJECT_CELLS = 2
# A kerb is one cell wide by construction - it is the boundary between two flat
# surfaces - so it must NOT be cleaned up by an opening, which erodes any
# one-cell line out of existence. Removing lone cells by component size keeps
# the line and drops the noise; the previous opening did the exact opposite,
# deleting clean kerbs and keeping thick patches of rough ground.
MIN_STEP_CELLS = 3


def keep_out(mask):
    """Logical NOT that cannot scribble on its operand.

    numpy reuses an operand's buffer when it decides the operand is a
    temporary, and on this build (numpy 2.2.6, Python 3.14) a large
    function-local array looks like one to the `~` operator: evaluating
    `other & ~mask` negates `mask` IN PLACE. Measured here, the swept-footprint
    mask came back as its own complement - 11569 cells became 1560300 - and
    every downstream class inverted with it, silently, with no error.

    np.logical_not() does not take that path. The rule for this file is that
    `~` is never applied to a named array that is still needed.
    """
    return np.logical_not(mask)


def disk(radius_cells):
    span = np.ogrid[-radius_cells:radius_cells + 1,
                    -radius_cells:radius_cells + 1]
    return span[1] ** 2 + span[0] ** 2 <= radius_cells ** 2


def load_cloud(path):
    with open(path, "rb") as handle:
        header = b""
        while not header.endswith(b"DATA binary\n"):
            header += handle.read(1)
        cloud = np.frombuffer(handle.read(), dtype=np.float32).reshape(-1, 4)
    points = cloud[:, :3]
    return points[np.isfinite(points).all(axis=1)]


def build(points, route_xy, route_z):
    tree = cKDTree(route_xy)
    distance, nearest = tree.query(points[:, :2], k=1)
    relative = points[:, 2] - route_z[nearest]
    keep = ((distance < CORRIDOR_M)
            & (relative > -BELOW_ROUTE_M) & (relative < ABOVE_ROUTE_M))
    inside = points[keep]

    min_x = inside[:, 0].min() - 1.0
    min_y = inside[:, 1].min() - 1.0
    nx = int((inside[:, 0].max() - min_x) / CELL) + 2
    ny = int((inside[:, 1].max() - min_y) / CELL) + 2
    col = ((inside[:, 0] - min_x) / CELL).astype(np.int32)
    row = ((inside[:, 1] - min_y) / CELL).astype(np.int32)
    flat = (row * nx + col).astype(np.int64)

    count = np.bincount(flat, minlength=nx * ny)
    order = np.argsort(flat, kind="stable")
    sorted_flat, sorted_z = flat[order], inside[order, 2]
    start = np.searchsorted(sorted_flat, np.arange(nx * ny))
    lowest = np.full(nx * ny, np.nan)
    occupied = np.where(count > 0)[0]
    lowest[occupied] = np.minimum.reduceat(sorted_z, start[occupied])

    known = (count > 0).reshape(ny, nx)
    lowest = lowest.reshape(ny, nx)
    filled = lowest[tuple(ndimage.distance_transform_edt(
        ~known, return_distances=False, return_indices=True))]
    element = disk(int(round(GROUND_RADIUS_M / CELL)))
    ground = ndimage.grey_dilation(
        ndimage.grey_erosion(filled, footprint=element), footprint=element)

    above = inside[:, 2] - ground.ravel()[flat]
    in_body = (above > BODY_LOW_M) & (above < BODY_HIGH_M)
    body = np.bincount(flat, weights=in_body.astype(float),
                       minlength=nx * ny).reshape(ny, nx)

    step = np.zeros_like(ground)
    for shift_y, shift_x in ((0, 1), (1, 0), (1, 1), (1, -1)):
        step = np.maximum(step, np.abs(
            ground - np.roll(np.roll(ground, shift_y, 0), shift_x, 1)))

    grid_y, grid_x = np.mgrid[0:ny, 0:nx]
    centres = np.column_stack([(min_x + (grid_x.ravel() + 0.5) * CELL),
                               (min_y + (grid_y.ravel() + 0.5) * CELL)])
    to_route = tree.query(centres, k=1)[0].reshape(ny, nx)
    return dict(ground=ground, body=body, step=step, known=known,
                count=count.reshape(ny, nx), to_route=to_route,
                min_x=min_x, min_y=min_y, nx=nx, ny=ny)


def classify(grid):
    inside = grid["to_route"] < CORRIDOR_M
    driven = grid["to_route"] <= CHAIR_HALF_WIDTH_M
    self_return = grid["to_route"] <= SELF_RETURN_M

    obstruction = (grid["body"] >= MIN_BODY_RETURNS) & keep_out(self_return)
    # Size filter BEFORE closing: closing first would fuse neighbouring specks
    # into blobs large enough to survive the filter that is meant to remove them.
    labels, total = ndimage.label(obstruction)
    if total:
        sizes = ndimage.sum(obstruction, labels, range(1, total + 1))
        specks = np.isin(labels, 1 + np.where(sizes < MIN_OBJECT_CELLS)[0])
        obstruction = np.logical_and(obstruction, keep_out(specks))
    obstruction = ndimage.binary_closing(obstruction, structure=disk(2))

    stepped = (grid["step"] > STEP_M) & keep_out(driven)
    labels, total = ndimage.label(stepped)
    if total:
        sizes = ndimage.sum(stepped, labels, range(1, total + 1))
        specks = np.isin(labels, 1 + np.where(sizes < MIN_STEP_CELLS)[0])
        stepped = np.logical_and(stepped, keep_out(specks))

    free = np.logical_and(inside, grid["known"])
    free = np.logical_and(free, grid["count"] >= 2)
    free = np.logical_and(free, keep_out(obstruction))
    free = np.logical_and(free, keep_out(stepped))
    free = np.logical_or(free, driven)

    radius = int(round((CHAIR_HALF_WIDTH_M + BAND_MARGIN_M) / CELL))
    centre_free = ndimage.binary_erosion(free, structure=disk(radius)) | driven
    labels, total = ndimage.label(centre_free)
    seeds = labels[driven & (labels > 0)]
    reachable = (labels == np.bincount(seeds).argmax()) if seeds.size else centre_free
    swept = ndimage.binary_dilation(
        reachable, structure=disk(int(round(CHAIR_HALF_WIDTH_M / CELL)))) & free
    return dict(inside=inside, driven=driven, obstruction=obstruction,
                stepped=stepped, free=free, reachable=reachable, swept=swept)


def render(grid, masks, path):
    image = np.zeros((grid["ny"], grid["nx"], 3), np.uint8)
    image[...] = (14, 18, 21)
    image[np.logical_and(masks["inside"], keep_out(grid["known"]))] = (24, 29, 33)
    image[masks["inside"] & grid["known"]] = (46, 56, 63)
    image[masks["swept"]] = (32, 86, 92)
    image[masks["reachable"]] = (59, 183, 192)
    image[masks["inside"] & masks["stepped"]] = (233, 160, 58)
    image[masks["inside"] & masks["obstruction"]] = (233, 100, 90)
    image[masks["driven"]] = (250, 246, 180)
    Image.fromarray(image[::-1]).save(path)


def main(map_path, route_path, out_prefix):
    route = json.load(open(route_path, encoding="utf-8"))["waypoints"]
    route_xy = np.array([[w["x"], w["y"]] for w in route])
    route_z = np.array([w["z"] for w in route])
    grid = build(load_cloud(map_path), route_xy, route_z)
    masks = classify(grid)

    area = CELL * CELL
    labels, objects = ndimage.label(masks["obstruction"])
    sizes = ndimage.sum(masks["obstruction"], labels,
                        range(1, objects + 1)) * area if objects else np.array([])
    print("grid %d x %d at %.2f m, %d cells measured"
          % (grid["nx"], grid["ny"], CELL, int(grid["known"].sum())))
    print("obstructions: %d objects, %.0f m2 (post-sized %d, building-sized %d)"
          % (objects, masks["obstruction"].sum() * area,
             int(((sizes > 0.05) & (sizes < 1.5)).sum()), int((sizes > 10).sum())))
    print("ground steps: %.0f m2" % (masks["stepped"].sum() * area))
    print("free ground: %.0f m2" % (masks["free"].sum() * area))
    print("reachable by the chair centre: %.0f m2" % (masks["reachable"].sum() * area))
    covered = (masks["reachable"] & masks["driven"]).sum() / max(
        masks["driven"].sum(), 1)
    print("the drive lies %.1f%% inside the reachable region" % (100 * covered))
    if covered < 0.999:
        print("WARNING: the chair went where this says it cannot - the "
              "classification is wrong, not the drive")

    np.savez_compressed(
        out_prefix + ".npz", cell=CELL, min_x=grid["min_x"], min_y=grid["min_y"],
        ground=grid["ground"].astype(np.float32), **{
            k: masks[k] for k in
            ("driven", "obstruction", "stepped", "free", "reachable", "swept")})
    render(grid, masks, out_prefix + ".png")
    print("wrote %s.npz and %s.png" % (out_prefix, out_prefix))


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
