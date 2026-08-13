#!/usr/bin/env python3
"""Terrain analysis and a visibility-graph global planner, from the map alone.

This is the CMU ground-autonomy shape - terrain analysis, then a global planner
over the traversable surface - rebuilt against this map and this chair:

    3D map cloud
      -> ground surface        (grey opening: lift objects off, keep the grade)
      -> slope / step / body   (three independent reasons a cell is refused)
      -> configuration space   (erode by the chair's half width + margin)
      -> visibility graph      (nodes at convex corners of the refused region)
      -> A* with a slope and clearance cost
      -> global path

Two deliberate departures from a literal FAR Planner port:

  * Nodes are convex corners of the *configuration* space, not of raw obstacle
    polygons. A taut path in a polygonal world bends only at convex obstacle
    corners, so those cells are the whole search space; taking them after
    erosion means every graph edge is already chair-width feasible and no
    separate footprint check is needed at plan time.

  * Slope is not one number per cell. A 6 degree grade is a speed limit when
    the chair climbs it and a tip risk when it crosses it, so the cost carries
    the grade and the planner refuses only above what this chair has actually
    demonstrated - measured from the 0727 trace, not assumed.

Nothing here touches the runtime graph. It is an offline planner over a
committed map, and it grants no motion authority of any kind.
"""

import heapq
import math

import numpy as np
from scipy import ndimage


# --- raster ---------------------------------------------------------------
CELL = 0.15
# How far from the recorded line to analyse. The map only covers what was
# driven, so this is a coverage bound, not a policy: past roughly 12 m the
# cloud thins out to building faces and the far pavement.
CORRIDOR_M = 14.0
BELOW_ROUTE_M = 3.0
ABOVE_ROUTE_M = 8.0

# --- ground surface -------------------------------------------------------
# Radius of the opening element. Anything narrower than twice this is an object
# standing ON the ground rather than ground. Measured: without the opening the
# surface follows the 0.20 m voxel aliasing and reports a median 16.5 degrees
# along cells the chair demonstrably drove at 2.7.
GROUND_RADIUS_M = 1.5

# --- the three refusals ---------------------------------------------------
# Body volume the chair and rider sweep. Above this is canopy and signage.
BODY_LOW_M = 0.08
BODY_HIGH_M = 1.60
MIN_BODY_RETURNS = 2
MIN_OBJECT_CELLS = 2
# A ground break across one cell. Kerbs are one cell wide by construction.
STEP_M = 0.06
MIN_STEP_CELLS = 3
# Slope baseline: the chair's own scale. Shorter reads voxel noise, longer
# smooths a real ramp shoulder away.
SLOPE_BASELINE_M = 1.95
# 3.0 degrees is where the running follower already drops to SLOPE_SPEED, so
# the cost starts biting at the same place the chair starts slowing.
SLOPE_SLOW_DEG = 3.0
# The hard refusal. The 0727 trace climbed 10.35 degrees over a 1 m baseline
# (p99 7.96), so a gate at or below that would refuse ground the chair has
# already driven. 12.0 clears the demonstrated envelope with margin and still
# sits far under the 20-30 degrees a kerb face or planting bed reads.
SLOPE_BLOCK_DEG = 12.0
# A slope window that straddles a kerb measures the kerb, not the pavement the
# wheels are on. Those cells are already refused by the step mask, so the slope
# gate is not applied within half a baseline of one - otherwise every kerb
# sheds a 1 m skirt of false steep ground onto the footway beside it.
STEP_BLEED_M = SLOPE_BASELINE_M / 2.0

# --- the chair ------------------------------------------------------------
CHAIR_HALF_WIDTH_M = 0.35
BAND_MARGIN_M = 0.10
# The map contains the chair and its rider; returns this close to the recorded
# line are self-returns, and the line is proven drivable by having been driven.
SELF_RETURN_M = 0.70
HOLE_FILL_CELLS = 2

# --- graph and cost -------------------------------------------------------
NODE_MIN_SEPARATION_M = 1.2
MAX_EDGE_M = 30.0
# Clearance the planner prefers but will trade away rather than fail.
PREFERRED_CLEARANCE_M = 0.45
SLOPE_WEIGHT = 1.5
CLEARANCE_WEIGHT = 0.8


def keep_out(mask):
    """Logical NOT that cannot scribble on its operand.

    numpy reuses an operand's buffer when it decides the operand is a
    temporary, and a large function-local array looks like one to `~`:
    `other & ~mask` has been measured negating `mask` in place in this project,
    silently inverting every downstream class. `~` is never applied to a named
    array that is still needed.
    """
    return np.logical_not(mask)


def disk(radius_cells):
    span = np.ogrid[-radius_cells:radius_cells + 1,
                    -radius_cells:radius_cells + 1]
    return span[1] ** 2 + span[0] ** 2 <= radius_cells ** 2


def load_cloud(path):
    """Read a binary XYZI .pcd."""
    with open(path, "rb") as handle:
        header = b""
        while not header.endswith(b"DATA binary\n"):
            chunk = handle.read(1)
            if not chunk:
                raise ValueError("%s: no binary DATA section" % path)
            header += chunk
        cloud = np.frombuffer(handle.read(), dtype=np.float32).reshape(-1, 4)
    points = cloud[:, :3]
    return points[np.isfinite(points).all(axis=1)]


def curvature_kerb(ground, cell, thresh_m=0.10):
    """True kerbs are curvature, ramps are not.

    The discrete Laplacian of a plane (any grade) is zero. A 12-15 cm
    kerb is a spike. Neighbour-drop tests cannot tell a descent from a
    drop; this can.

    This is exact on a continuous plane. A 0.20 m voxel cloud turns a
    ramp into stairs; use ramp_aware_kerb on those maps.
    """
    lap = np.abs(ndimage.laplace(ground.astype(np.float64)))
    return lap > thresh_m


# A 0.20 m voxelized grade produces ~20 cm along-slope risers. Side kerbs
# on this campus are 12-15 cm. The two only separate by direction: the
# walkway edge is across the grade, the voxel stairs are along it.
KERB_SIDE_M = 0.10
KERB_DROP_M = 0.25
_FLAT_GRADE = math.tan(math.radians(2.0))


def smooth_slope_deg(ground, cell, smooth_m=2.0):
    """Grade of a 2 m-smoothed surface, in degrees.

    Voxel stairs of a real ramp become the ramp again. A 12° descent
    must not read 13.5° and trip SLOPE_BLOCK_DEG.
    """
    sigma = max(smooth_m / cell, 2.0)
    smooth = ndimage.gaussian_filter(ground.astype(np.float64), sigma=sigma)
    grade_x, grade_y = surface_gradient(smooth, cell, max(3.0, SLOPE_BASELINE_M))
    return np.degrees(np.arctan(np.hypot(grade_x, grade_y)))


def ramp_aware_kerb(ground, slope_x, slope_y, cell,
                    side_m=KERB_SIDE_M, drop_m=KERB_DROP_M):
    """Kerbs on a ramp, not the ramp itself.

    Grade direction is taken from a 2 m smooth so a 12-15 cm lip cannot
    impersonate a ramp. After that plane is removed:
      * a jump across the grade above `side_m` is the walkway edge
      * a jump along the grade above `drop_m` is a real stair or drop
      * smaller along-grade jumps are voxel stairs of the descent
    On flat ground every direction is a potential kerb.

    Every ufunc names its destination. Under Python 3.14 / numpy 2.2 a
    temporary like `jump * weight` is elided into `jump` or `weight` on
    rasters past ~200x200, and the detector goes dark on a real map.

    `slope_x` / `slope_y` are unused; grade is recomputed from `ground`.
    """
    del slope_x, slope_y
    sigma = max(2.0 / cell, 2.0)
    smooth = ndimage.gaussian_filter(ground.astype(np.float64), sigma=sigma)
    grade_x, grade_y = surface_gradient(smooth, cell, max(3.0, SLOPE_BASELINE_M))
    mag = np.empty_like(ground, dtype=np.float64)
    np.hypot(grade_x, grade_y, out=mag)
    flat = mag < _FLAT_GRADE
    mag_safe = np.empty_like(mag)
    np.maximum(mag, 1e-6, out=mag_safe)
    side = np.zeros_like(ground, dtype=np.float64)
    along = np.zeros_like(ground, dtype=np.float64)
    jump = np.empty_like(ground, dtype=np.float64)
    pred = np.empty_like(ground, dtype=np.float64)
    weight = np.empty_like(ground, dtype=np.float64)
    tmp = np.empty_like(ground, dtype=np.float64)
    for shift_y, shift_x in ((0, 1), (1, 0), (1, 1), (1, -1)):
        shifted = np.roll(np.roll(ground, shift_y, 0), shift_x, 1)
        np.multiply(grade_x, shift_x * cell, out=pred)
        np.multiply(grade_y, shift_y * cell, out=tmp)
        np.add(pred, tmp, out=pred)
        np.subtract(ground, shifted, out=jump)
        np.subtract(jump, pred, out=jump)
        np.abs(jump, out=jump)
        off_len = math.hypot(shift_x, shift_y) * cell
        np.abs(pred, out=weight)
        np.multiply(mag_safe, off_len, out=tmp)
        np.divide(weight, tmp, out=weight)
        np.clip(weight, 0.0, 1.0, out=weight)
        np.multiply(jump, weight, out=tmp)
        np.maximum(along, tmp, out=along)
        np.subtract(1.0, weight, out=weight)
        np.multiply(jump, weight, out=tmp)
        np.maximum(side, tmp, out=side)
        np.copyto(tmp, jump)
        tmp[keep_out(flat)] = 0.0
        np.maximum(side, tmp, out=side)
    return (side > side_m) | (along > drop_m)


def detrended_step(ground, slope_x, slope_y, cell):
    """Local height jump leftover after subtracting the fitted plane.

    A continuous ramp has a large neighbour-to-neighbour drop and a matching
    gradient, so the residual is near zero. A kerb is a break the plane
    cannot explain. Neighbour offsets must be named destinations: under
    Python 3.14 a chained `ground - shifted` can scribble on `ground`.
    """
    residual = np.zeros_like(ground, dtype=np.float64)
    jump = np.empty_like(ground, dtype=np.float64)
    for shift_y, shift_x in ((0, 1), (1, 0), (1, 1), (1, -1)):
        shifted = np.roll(np.roll(ground, shift_y, 0), shift_x, 1)
        pred = (slope_x * (shift_x * cell)) + (slope_y * (shift_y * cell))
        np.subtract(ground, shifted, out=jump)
        np.subtract(jump, pred, out=jump)
        np.abs(jump, out=jump)
        np.maximum(residual, jump, out=residual)
    return residual


def surface_gradient(surface, cell, baseline_m):
    """Least-squares plane gradient of a height raster, in metres per metre.

    On a regular grid the LS plane fit collapses to a ramp correlation,

        dz/dx = sum(i * z) / (cell * sum(i^2))

    summed over the window. The moment form E[x^2] - E[x]^2 must not be used
    here: at x ~ 200 m the variance of a 1 m window is ~1e-2 against a mean
    square of ~4e4, which is 6 significant digits of cancellation - it was
    measured returning 90 degrees for every cell on this map.
    """
    half = max(int(round(baseline_m / cell)) // 2, 1)
    ramp = np.arange(-half, half + 1, dtype=np.float64)
    width = 2 * half + 1
    sum_i2 = float((ramp ** 2).sum()) * width
    height = surface.astype(np.float64)
    along_x = ndimage.correlate1d(height, ramp, axis=1, mode="nearest")
    along_x = ndimage.uniform_filter1d(along_x, width, axis=0,
                                       mode="nearest") * width
    along_y = ndimage.correlate1d(height, ramp, axis=0, mode="nearest")
    along_y = ndimage.uniform_filter1d(along_y, width, axis=1,
                                       mode="nearest") * width
    scale = cell * sum_i2
    return along_x / scale, along_y / scale


def raster(points, route_xy, route_z, tree, cell=CELL, corridor_m=CORRIDOR_M):
    """Corridor raster: per-cell minimum height, coverage, distance to route."""
    distance, nearest = tree.query(points[:, :2], k=1)
    relative = points[:, 2] - route_z[nearest]
    inside = points[(distance < corridor_m)
                    & (relative > -BELOW_ROUTE_M)
                    & (relative < ABOVE_ROUTE_M)]
    if not inside.size:
        raise ValueError("no map points inside the corridor")

    min_x = float(inside[:, 0].min()) - 1.0
    min_y = float(inside[:, 1].min()) - 1.0
    nx = int((inside[:, 0].max() - min_x) / cell) + 2
    ny = int((inside[:, 1].max() - min_y) / cell) + 2
    col = ((inside[:, 0] - min_x) / cell).astype(np.int32)
    row = ((inside[:, 1] - min_y) / cell).astype(np.int32)
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
        keep_out(known), return_distances=False, return_indices=True))]

    grid_y, grid_x = np.mgrid[0:ny, 0:nx]
    centres = np.column_stack([(min_x + (grid_x.ravel() + 0.5) * cell),
                               (min_y + (grid_y.ravel() + 0.5) * cell)])
    to_route, station = tree.query(centres, k=1)
    return dict(cell=cell, min_x=min_x, min_y=min_y, nx=nx, ny=ny,
                known=known, filled=filled, inside_points=inside,
                flat=flat, count=count.reshape(ny, nx),
                to_route=to_route.reshape(ny, nx),
                station=station.reshape(ny, nx))


def terrain(grid):
    """Ground surface, slope, step and body occupancy - the three refusals."""
    cell = grid["cell"]
    element = disk(int(round(GROUND_RADIUS_M / cell)))
    ground = ndimage.grey_dilation(
        ndimage.grey_erosion(grid["filled"], footprint=element),
        footprint=element)

    inside = grid["inside_points"]
    above = inside[:, 2] - ground.ravel()[grid["flat"]]
    in_body = (above > BODY_LOW_M) & (above < BODY_HIGH_M)
    body = np.bincount(grid["flat"], weights=in_body.astype(float),
                       minlength=grid["nx"] * grid["ny"]).reshape(
                           grid["ny"], grid["nx"])

    step = np.zeros_like(ground)
    for shift_y, shift_x in ((0, 1), (1, 0), (1, 1), (1, -1)):
        # Every operation names its destination. Written as
        #   step = np.maximum(step, np.abs(ground - np.roll(...)))
        # this loop overwrote `ground` in place on all four iterations: under
        # Python 3.14 the interpreter borrows the stack reference, numpy sees
        # refcount 1 on a named local and elides the subtraction into it.
        # Measured on this map, the ground surface went from -10.66..10.53 m to
        # -16.09..16.09 m, and every slope and step number after it - including
        # the ones the planner costs - came from that wreckage. `ground` is
        # never a destination here, and test_terrain_graph checks the surface
        # against an independent opening after terrain() returns.
        shifted = np.roll(np.roll(ground, shift_y, 0), shift_x, 1)
        np.subtract(ground, shifted, out=shifted)
        np.abs(shifted, out=shifted)
        np.maximum(step, shifted, out=step)

    slope_x, slope_y = surface_gradient(ground, cell, SLOPE_BASELINE_M)
    slope_deg = np.degrees(np.arctan(np.hypot(slope_x, slope_y)))
    # Adjacent-cell drop on a ramp is the grade, not a kerb. A 10° ramp
    # already drops 2.6 cm per 0.15 m cell (3.7 cm on the diagonal); voxel
    # aliasing pushes that over STEP_M and paints the whole descent as a
    # kerb. The kerb is the residual AFTER the local plane is removed.
    residual = detrended_step(ground, slope_x, slope_y, cell)

    # A slope window straddling a step measures the step, not the pavement the
    # wheels are on. Along the line the chair actually drove, that reads a p99
    # of 22 and a max of 34 degrees where the trace proves it drove 10.35 - so a
    # raw slope gate refuses the proven route. Where a step is within half a
    # baseline, the step mask is the authority and the grade is set aside; the
    # cost then falls back to clearance, which is the right thing to be
    # sensitive to next to a kerb. This is the ONE field anything downstream may
    # gate on, so legality is decided once.
    near_step = ndimage.binary_dilation(
        residual > STEP_M, structure=disk(max(int(round(STEP_BLEED_M / cell)), 1)))
    gate_slope_deg = np.where(near_step, 0.0, slope_deg)
    return dict(ground=ground, body=body, step=step, residual_step=residual,
                slope_deg=slope_deg,
                gate_slope_deg=gate_slope_deg, near_step=near_step,
                slope_x=slope_x, slope_y=slope_y)


def body_obstruction(body, dense_returns=8, attached_returns=3,
                     min_cells=MIN_OBJECT_CELLS):
    """Buildings have a dense vertical core. Isolated light hits are canopy.

    A cell is a wall when it has at least `attached_returns` body-height
    hits AND it belongs to a connected component that contains at least
    one cell with `dense_returns` hits. Scattered 1-4 return specks from
    leaves do not qualify; a facade does, including its thinner fringe.
    """
    attached = body >= attached_returns
    labels, total = ndimage.label(attached)
    if not total:
        return np.zeros(body.shape, dtype=bool)
    cores = np.unique(labels[body >= dense_returns])
    cores = cores[cores > 0]
    mask = np.isin(labels, cores) if cores.size else np.zeros(body.shape, dtype=bool)
    mask = _drop_small(mask, min_cells)
    return ndimage.binary_closing(mask, structure=disk(2))


def _drop_small(mask, min_cells):
    labels, total = ndimage.label(mask)
    if not total:
        return mask
    sizes = ndimage.sum(mask, labels, range(1, total + 1))
    specks = np.isin(labels, 1 + np.where(sizes < min_cells)[0])
    return np.logical_and(mask, keep_out(specks))


def traversability(grid, land, trust_driven=True, seed_xy=None,
                    self_return_m=SELF_RETURN_M,
                    min_body_returns=MIN_BODY_RETURNS,
                    dense_body_returns=None,
                    attached_body_returns=3):
    """Split the corridor into what refuses the chair and why.

    `trust_driven` controls a load-bearing assumption. With it on, the cells
    under the recorded line are taken as drivable because the chair drove them:
    the step and slope refusals are waived there, the ribbon is forced into the
    free set both before and after the configuration-space erosion, and the
    reachable component is seeded from it. That is defensible - the drive
    happened - but it means the reachable region is CONSTRUCTED along the
    recorded line, so a path planned inside it cannot be called independent
    evidence for that line.

    With it off, only the recorded route's role as a corridor bound and as the
    start and goal remains, and the terrain has to earn every cell. Compare the
    two to see how much of the answer came from the map and how much came from
    the assumption; make_global_plan reports both.

    The self-return exemption within `self_return_m` is NOT part of this switch.
    The map contains the chair and its rider, so returns that close to the line
    are the vehicle itself; suppressing the obstruction test there is a
    statement about the data, not a concession to the route. It defaults to
    SELF_RETURN_M for the recorded mapping trajectory; a fresh start-goal plan
    whose corridor bound is not the mapping drive passes 0 so an obstacle the
    straight line happens to cross is not silently exempted as 'self'.
    """
    cell = grid["cell"]
    inside = grid["to_route"] < CORRIDOR_M
    driven = grid["to_route"] <= CHAIR_HALF_WIDTH_M
    self_return = grid["to_route"] <= self_return_m
    trusted = driven if trust_driven else np.zeros_like(driven)

    if dense_body_returns is not None:
        obstruction = body_obstruction(
            land["body"], dense_returns=dense_body_returns,
            attached_returns=attached_body_returns)
        obstruction = np.logical_and(obstruction, keep_out(self_return))
    else:
        obstruction = (land["body"] >= min_body_returns) & keep_out(self_return)
        obstruction = _drop_small(obstruction, MIN_OBJECT_CELLS)
        obstruction = ndimage.binary_closing(obstruction, structure=disk(2))

    # Residual after removing the local plane: a ramp is not a kerb.
    step_field = land["residual_step"] if "residual_step" in land else land["step"]
    stepped = (step_field > STEP_M) & keep_out(trusted)
    stepped = _drop_small(stepped, MIN_STEP_CELLS)

    # The slope refusal reads the one field terrain() published for it.
    near_step = land["near_step"]
    steep = (land["gate_slope_deg"] > SLOPE_BLOCK_DEG) & keep_out(trusted)
    steep = _drop_small(steep, MIN_STEP_CELLS)

    free = np.logical_and(inside, grid["known"])
    for refusal in (obstruction, stepped, steep):
        free = np.logical_and(free, keep_out(refusal))
    free = np.logical_or(free, trusted)
    # Seal aliasing holes before eroding, then re-apply the refusals so a hole
    # that really does contain something stays refused.
    free = ndimage.binary_closing(free, structure=disk(HOLE_FILL_CELLS))
    for refusal in (obstruction, stepped, steep):
        free = np.logical_and(free, keep_out(refusal))

    radius = int(round((CHAIR_HALF_WIDTH_M + BAND_MARGIN_M) / cell))
    centre_free = np.logical_or(
        ndimage.binary_erosion(free, structure=disk(radius)), trusted)
    labels, total = ndimage.label(centre_free)
    if trust_driven:
        seeds = labels[driven & (labels > 0)]
        reachable = ((labels == np.bincount(seeds).argmax()) if seeds.size
                     else centre_free)
    else:
        # Seeded from the start pose alone, which the mission legitimately
        # knows. If the start cell earns nothing, the region is empty rather
        # than quietly falling back to the recorded line.
        point = seed_xy if seed_xy is not None else (0.0, 0.0)
        row = int(np.clip(round((point[1] - grid["min_y"]) / cell - 0.5),
                          0, grid["ny"] - 1))
        col = int(np.clip(round((point[0] - grid["min_x"]) / cell - 0.5),
                          0, grid["nx"] - 1))
        label_here = labels[row, col]
        if label_here == 0:
            open_rows, open_cols = np.nonzero(centre_free)
            if len(open_rows):
                nearest = np.argmin((open_rows - row) ** 2
                                    + (open_cols - col) ** 2)
                label_here = labels[open_rows[nearest], open_cols[nearest]]
        reachable = (labels == label_here) if label_here else centre_free
    clearance = ndimage.distance_transform_edt(free) * cell
    return dict(inside=inside, driven=driven, obstruction=obstruction,
                stepped=stepped, steep=steep, near_step=near_step,
                free=free, centre_free=centre_free, reachable=reachable,
                clearance=clearance, trust_driven=trust_driven)


# --- visibility graph -----------------------------------------------------
def corner_nodes(passable, cell, min_separation_m=NODE_MIN_SEPARATION_M):
    """Convex corners of the refused region, thinned to a minimum spacing.

    A taut path bends only where it wraps a convex corner. On a grid that is a
    passable cell with a diagonal neighbour refused and both shared orthogonal
    neighbours passable - the cell a string would press against. Everything
    else in the free space is interior and can never be a turn in a shortest
    path, which is what keeps the graph small enough to search in one pass.
    """
    blocked = keep_out(passable)
    corner = np.zeros_like(passable)
    for shift_y, shift_x in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        diagonal = np.roll(np.roll(blocked, shift_y, 0), shift_x, 1)
        side_y = np.roll(passable, shift_y, 0)
        side_x = np.roll(passable, shift_x, 1)
        corner = np.logical_or(
            corner, passable & diagonal & side_y & side_x)

    rows, cols = np.nonzero(corner)
    if not rows.size:
        return np.zeros((0, 2), np.int64)
    # Keep the best-cleared candidate in each min_separation_m tile, which
    # thins a 300 m corridor from tens of thousands of corner cells to
    # hundreds without dropping any distinct corner.
    tile = max(int(round(min_separation_m / cell)), 1)
    key = (rows // tile).astype(np.int64) * (10 ** 6) + (cols // tile)
    order = np.lexsort((rows, cols, key))
    key, rows, cols = key[order], rows[order], cols[order]
    first = np.concatenate([[True], key[1:] != key[:-1]])
    return np.column_stack([rows[first], cols[first]])


def _line_cells(r0, c0, r1, c1):
    """Half-cell sampling of the segment between two cell coordinates.

    The span is rounded, not truncated. A node's cell index recovered from
    metres comes back as 49.9999999 rather than 50 once the raster origin is a
    number like 204.9, and truncating that dropped a sample pair - which made
    the scalar test accept 14 edges the bulk test rejected on the same world.
    """
    span = max(abs(r1 - r0), abs(c1 - c0))
    steps = int(round(span)) * 2 + 2
    rows = np.rint(np.linspace(r0, r1, steps)).astype(np.int64)
    cols = np.rint(np.linspace(c0, c1, steps)).astype(np.int64)
    return rows, cols


def visible(passable, r0, c0, r1, c1):
    """Is the straight segment between two cells entirely passable?

    Sampled at half-cell spacing. The test runs in configuration space, where
    the chair's half width and margin are already eroded out, so a sample that
    lands passable means the whole footprint fits there.
    """
    rows, cols = _line_cells(r0, c0, r1, c1)
    return bool(passable[rows, cols].all())


def slope_penalty(grade_deg):
    """0 below the speed-limit grade, 1 at the refusal grade, saturating there.

    Saturating matters. An unbounded penalty lets one cell whose window happens
    to straddle something make an otherwise legal corridor arbitrarily
    expensive, and the planner abandons a route it should take.
    """
    span = SLOPE_BLOCK_DEG - SLOPE_SLOW_DEG
    return np.clip((np.asarray(grade_deg, float) - SLOPE_SLOW_DEG) / span,
                   0.0, 1.0)


def clearance_penalty(clearance_m):
    """0 at or beyond the preferred clearance, 1 with none at all."""
    return np.clip((PREFERRED_CLEARANCE_M - np.asarray(clearance_m, float))
                   / PREFERRED_CLEARANCE_M, 0.0, 1.0)


def edge_cost(nodes_xy, i, j, gate_slope_deg, clearance, passable, cell,
              min_x, min_y):
    """Length inflated by grade and by tight clearance, or None if refused.

    The slope argument is terrain()'s published gate field, never the raw
    slope: legality is decided once, in traversability(). Re-deciding it here
    with a cruder rule was measured severing the recorded route, because the
    raw grade reads 22-34 degrees wherever the 1.95 m window straddles a kerb.

    The multiplier stays in [1, 1 + SLOPE_WEIGHT + CLEARANCE_WEIGHT], so
    straight-line distance remains an admissible A* heuristic.
    """
    (x0, y0), (x1, y1) = nodes_xy[i], nodes_xy[j]
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= 0.0:
        return None
    r0 = (y0 - min_y) / cell - 0.5
    c0 = (x0 - min_x) / cell - 0.5
    r1 = (y1 - min_y) / cell - 0.5
    c1 = (x1 - min_x) / cell - 0.5
    rows, cols = _line_cells(r0, c0, r1, c1)
    if not passable[rows, cols].all():
        return None
    grade = gate_slope_deg[rows, cols]
    if float(grade.max()) > SLOPE_BLOCK_DEG:
        return None
    multiplier = (1.0
                  + SLOPE_WEIGHT * float(slope_penalty(grade.mean()))
                  + CLEARANCE_WEIGHT
                  * float(clearance_penalty(clearance[rows, cols].min())))
    return length * multiplier


def build_graph(nodes_rc, grid, land, masks, max_edge_m=MAX_EDGE_M,
                block_elements=4_000_000):
    """Connect every mutually visible pair within max_edge_m.

    Same decision as edge_cost() for every pair, computed in bulk. A 300 m
    corridor yields on the order of a million candidate pairs, and testing them
    one segment at a time is minutes of interpreter overhead. Pairs are grouped
    by their exact sample count - integral for graph nodes, since a node sits at
    a cell centre - so the vectorised test walks precisely the cells the scalar
    test would, and the two cannot drift apart. test_terrain_graph asserts that
    equality directly.
    """
    from scipy.spatial import cKDTree

    cell, min_x, min_y = grid["cell"], grid["min_x"], grid["min_y"]
    nodes_rc = np.asarray(nodes_rc, np.int64).reshape(-1, 2)
    nodes_xy = np.column_stack([min_x + (nodes_rc[:, 1] + 0.5) * cell,
                                min_y + (nodes_rc[:, 0] + 0.5) * cell])
    adjacency = [[] for _ in range(len(nodes_xy))]
    if len(nodes_xy) < 2:
        return nodes_xy, adjacency

    passable = masks["reachable"]
    gate_slope = land["gate_slope_deg"]
    clearance = masks["clearance"]

    pairs = cKDTree(nodes_xy).query_pairs(max_edge_m, output_type="ndarray")
    if not len(pairs):
        return nodes_xy, adjacency
    left, right = pairs[:, 0], pairs[:, 1]
    row0, col0 = nodes_rc[left, 0], nodes_rc[left, 1]
    delta_row = nodes_rc[right, 0] - row0
    delta_col = nodes_rc[right, 1] - col0
    span = np.maximum(np.abs(delta_row), np.abs(delta_col))
    length = np.hypot(nodes_xy[right, 0] - nodes_xy[left, 0],
                      nodes_xy[right, 1] - nodes_xy[left, 1])

    for width in np.unique(span):
        group = np.nonzero(span == width)[0]
        samples = int(width) * 2 + 2
        step = max(int(block_elements // samples), 1)
        walk = np.linspace(0.0, 1.0, samples)
        for begin in range(0, len(group), step):
            block = group[begin:begin + step]
            rows = np.rint(row0[block][:, None]
                           + delta_row[block][:, None] * walk).astype(np.int64)
            cols = np.rint(col0[block][:, None]
                           + delta_col[block][:, None] * walk).astype(np.int64)
            clear = passable[rows, cols].all(axis=1)
            if not clear.any():
                continue
            grade = gate_slope[rows, cols]
            tight = clearance[rows, cols].min(axis=1)
            ok = clear & (grade.max(axis=1) <= SLOPE_BLOCK_DEG)
            if not ok.any():
                continue
            cost = length[block] * (
                1.0 + SLOPE_WEIGHT * slope_penalty(grade.mean(axis=1))
                + CLEARANCE_WEIGHT * clearance_penalty(tight))
            for index in np.nonzero(ok)[0]:
                a = int(left[block[index]])
                b = int(right[block[index]])
                weight = float(cost[index])
                adjacency[a].append((b, weight))
                adjacency[b].append((a, weight))
    return nodes_xy, adjacency


def astar(nodes_xy, adjacency, start, goal):
    """Straight-line heuristic, admissible because every edge multiplier >= 1."""
    def heuristic(index):
        return float(np.hypot(*(nodes_xy[index] - nodes_xy[goal])))

    best = {start: 0.0}
    came_from = {}
    frontier = [(heuristic(start), 0.0, start)]
    closed = set()
    while frontier:
        _, cost_here, current = heapq.heappop(frontier)
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return path[::-1], cost_here
        if current in closed:
            continue
        closed.add(current)
        for neighbour, step in adjacency[current]:
            if neighbour in closed:
                continue
            candidate = cost_here + step
            if candidate < best.get(neighbour, math.inf) - 1e-12:
                best[neighbour] = candidate
                came_from[neighbour] = current
                heapq.heappush(
                    frontier, (candidate + heuristic(neighbour), candidate,
                               neighbour))
    return None, math.inf


def graph_edges(adjacency):
    """Flatten an adjacency list to unique (i, j, cost) arrays."""
    i, j, cost = [], [], []
    for a, neighbours in enumerate(adjacency):
        for b, weight in neighbours:
            if a < b:
                i.append(a)
                j.append(b)
                cost.append(weight)
    return (np.asarray(i, np.int64), np.asarray(j, np.int64),
            np.asarray(cost, float))


def segment_distance(nodes_xy, i, j, point):
    """Distance from a point to each segment (nodes_xy[i], nodes_xy[j])."""
    a = nodes_xy[i]
    b = nodes_xy[j]
    along = b - a
    length2 = (along ** 2).sum(axis=1)
    to_point = np.asarray(point, float)[None, :] - a
    with np.errstate(invalid="ignore", divide="ignore"):
        t = np.where(length2 > 0.0,
                     (to_point * along).sum(axis=1) / np.where(
                         length2 > 0.0, length2, 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    closest = a + t[:, None] * along
    return np.hypot(*(np.asarray(point, float)[None, :] - closest).T)


def adjacency_without(nodes_xy, edges, blockage_xy, radius_m):
    """The graph with every edge that a disc obstruction would cut removed.

    An unexpected obstacle does not change the terrain, only which edges are
    still usable, so re-planning around one is exact: an edge survives if and
    only if its segment stays clear of the disc. That is a point-to-segment
    test, so a blockage costs one vectorised pass and one search rather than a
    rebuild of the whole graph - which is what makes the resilience scan below
    affordable at every station.
    """
    i, j, cost = edges
    keep = segment_distance(nodes_xy, i, j, blockage_xy) > radius_m
    adjacency = [[] for _ in range(len(nodes_xy))]
    for a, b, weight in zip(i[keep], j[keep], cost[keep]):
        adjacency[int(a)].append((int(b), float(weight)))
        adjacency[int(b)].append((int(a), float(weight)))
    return adjacency


def cell_costs(grid, land, masks):
    """Per-cell cost multiplier and the refusal mask, shared by both searches.

    Identical penalty algebra to edge_cost(), so a grid path and a visibility
    path are priced on the same scale and can be compared directly. Refusal is
    membership in the reachable region plus the published slope gate - nothing
    is re-decided here.
    """
    grade = land["gate_slope_deg"]
    multiplier = (1.0 + SLOPE_WEIGHT * slope_penalty(grade)
                  + CLEARANCE_WEIGHT * clearance_penalty(masks["clearance"]))
    refused = np.logical_or(keep_out(masks["reachable"]),
                            grade > SLOPE_BLOCK_DEG)
    return multiplier, refused


def cell_graph(grid, land, masks):
    """8-connected cost graph over the cells a chair centre may occupy."""
    from scipy.sparse import coo_matrix

    cell, nx, ny = grid["cell"], grid["nx"], grid["ny"]
    multiplier, refused = cell_costs(grid, land, masks)
    open_cell = keep_out(refused).copy()
    rows, cols = np.nonzero(open_cell)
    count = len(rows)
    index = -np.ones((ny, nx), np.int64)
    index[rows, cols] = np.arange(count)
    if not count:
        return None, index, rows, cols, open_cell
    per_node = multiplier[open_cell]

    from_index, to_index, weight = [], [], []
    for shift_y, shift_x, span in ((0, 1, 1.0), (1, 0, 1.0),
                                   (1, 1, math.sqrt(2.0)),
                                   (1, -1, math.sqrt(2.0))):
        shifted = np.roll(np.roll(index, -shift_y, 0), -shift_x, 1)
        if shift_y:
            shifted[-shift_y:, :] = -1
        if shift_x > 0:
            shifted[:, -shift_x:] = -1
        elif shift_x < 0:
            shifted[:, :(-shift_x)] = -1
        pair = np.logical_and(open_cell, shifted >= 0)
        if shift_y and shift_x:
            vertical = np.roll(open_cell, -shift_y, axis=0)
            horizontal = np.roll(open_cell, -shift_x, axis=1)
            vertical[-shift_y:, :] = False
            if shift_x > 0:
                horizontal[:, -shift_x:] = False
            else:
                horizontal[:, :(-shift_x)] = False
            pair &= vertical & horizontal
        here = index[pair]
        there = shifted[pair]
        from_index.append(here)
        to_index.append(there)
        weight.append(span * cell * 0.5 * (per_node[here] + per_node[there]))
    graph = coo_matrix((np.concatenate(weight),
                        (np.concatenate(from_index), np.concatenate(to_index))),
                       shape=(count, count)).tocsr()
    return graph, index, rows, cols, open_cell


def grid_plan(graph, index, rows, cols, grid, start_xy, goal_xy,
              blocked_cells=None):
    """Least-cost cell path. Complete: finds a route whenever one exists.

    A corner-only visibility graph is the right structure for a polygonal world
    and the wrong one here. Measured on this map, the chair-centre reachable
    region is a ribbon about 0.6 m wide over most of its length: almost no
    convex corners to bend at, and no sight line longer than the ribbon's own
    curvature. The graph came out in two disconnected halves. So the search runs
    on cells, where connectivity is guaranteed, and the visibility test is used
    afterwards to pull the answer taut. The visibility graph is still built and
    reported - that it fragments IS the finding about how much room this route
    leaves a global planner.
    """
    from scipy.sparse.csgraph import dijkstra

    if graph is None:
        return None, math.inf
    cell = grid["cell"]

    def nearest_node(point):
        row = int(np.clip(round((point[1] - grid["min_y"]) / cell - 0.5),
                          0, grid["ny"] - 1))
        col = int(np.clip(round((point[0] - grid["min_x"]) / cell - 0.5),
                          0, grid["nx"] - 1))
        if index[row, col] >= 0:
            return int(index[row, col])
        return int(np.argmin((rows - row) ** 2 + (cols - col) ** 2))

    source, target = nearest_node(start_xy), nearest_node(goal_xy)
    if blocked_cells is not None and len(blocked_cells):
        keep = np.ones(graph.shape[0], bool)
        keep[blocked_cells] = False
        if not keep[source] or not keep[target]:
            return None, math.inf
        live = np.nonzero(keep)[0]
        remap = -np.ones(graph.shape[0], np.int64)
        remap[live] = np.arange(len(live))
        sub = graph[live][:, live]
        distance, predecessor = dijkstra(sub, directed=False,
                                        indices=int(remap[source]),
                                        return_predecessors=True)
        end = int(remap[target])
        if not np.isfinite(distance[end]):
            return None, math.inf
        chain = [end]
        while chain[-1] != int(remap[source]):
            chain.append(int(predecessor[chain[-1]]))
        chain = live[chain[::-1]]
    else:
        distance, predecessor = dijkstra(graph, directed=False, indices=source,
                                        return_predecessors=True)
        if not np.isfinite(distance[target]):
            return None, math.inf
        chain = [target]
        while chain[-1] != source:
            chain.append(int(predecessor[chain[-1]]))
        chain = np.array(chain[::-1])
        return (np.column_stack([rows[chain], cols[chain]]),
                float(distance[target]))
    return (np.column_stack([rows[chain], cols[chain]]),
            float(distance[end]))


def taut(path_rc, passable):
    """String-pull a cell path into the fewest straight segments that stay clear.

    The grid answer is an 8-connected staircase; this is the polyline a rope
    between the same ends would take - what the visibility graph would have
    produced had it been connected. Greedy: a segment is only ever kept if the
    visibility test passes, so tautening can shorten the path but never make it
    cut a refused cell.
    """
    path_rc = np.asarray(path_rc, np.int64)
    if len(path_rc) < 3:
        return path_rc.copy()
    kept = [0]
    anchor = 0
    while anchor < len(path_rc) - 1:
        furthest = anchor + 1
        for ahead in range(anchor + 2, len(path_rc)):
            if visible(passable, path_rc[anchor, 0], path_rc[anchor, 1],
                       path_rc[ahead, 0], path_rc[ahead, 1]):
                furthest = ahead
            else:
                break
        kept.append(furthest)
        anchor = furthest
    return path_rc[kept]


def cells_within(rows, cols, grid, centre_xy, radius_m):
    """Node indices whose cell centre lies inside a disc."""
    x = grid["min_x"] + (cols + 0.5) * grid["cell"]
    y = grid["min_y"] + (rows + 0.5) * grid["cell"]
    return np.nonzero(np.hypot(x - centre_xy[0], y - centre_xy[1])
                      <= radius_m)[0]


def attach(nodes_xy, adjacency, grid, land, masks, point_xy):
    """Add a terminal to the graph, wired to every node it can see."""
    cell, min_x, min_y = grid["cell"], grid["min_x"], grid["min_y"]
    index = len(nodes_xy)
    nodes_xy = np.vstack([nodes_xy, np.asarray(point_xy, float)[None, :]])
    adjacency = adjacency + [[]]
    for other in range(index):
        if np.hypot(*(nodes_xy[other] - nodes_xy[index])) > MAX_EDGE_M:
            continue
        cost = edge_cost(nodes_xy, index, other, land["slope_deg"],
                         masks["clearance"], masks["reachable"], cell,
                         min_x, min_y)
        if cost is None:
            continue
        adjacency[index].append((other, cost))
        adjacency[other].append((index, cost))
    return nodes_xy, adjacency, index
