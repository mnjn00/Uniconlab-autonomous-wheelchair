"""The terrain analysis and the global planner, on worlds with known answers.

Every check here is one the real map cannot make for us: on the map we can only
ask whether an answer looks plausible, so the arithmetic is pinned against
synthetic ground where the right answer is known in closed form.
"""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
from scipy import ndimage

ROOT = Path(__file__).resolve().parents[1]


def load_module():
    path = ROOT / "tools" / "terrain_graph.py"
    spec = importlib.util.spec_from_file_location("terrain_graph", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


tg = load_module()


# --- the gradient ---------------------------------------------------------
@pytest.mark.parametrize("want_a,want_b", [(0.10, 0.0), (0.0, -0.25),
                                           (0.05, 0.12), (-0.30, 0.30)])
def test_gradient_is_exact_on_a_plane(want_a, want_b):
    grid_y, grid_x = np.mgrid[0:60, 0:80]
    surface = want_a * (grid_x * tg.CELL) + want_b * (grid_y * tg.CELL)
    got_a, got_b = tg.surface_gradient(surface, tg.CELL, 1.95)
    core = (slice(12, -12), slice(12, -12))
    assert abs(got_a[core].mean() - want_a) < 1e-9
    assert abs(got_b[core].mean() - want_b) < 1e-9


def test_gradient_survives_a_large_coordinate_offset():
    """The moment form E[x^2]-E[x]^2 loses six digits at x ~ 200 m.

    It was measured returning 90 degrees for every cell of the real map, so the
    offset is part of the contract, not an incidental detail.
    """
    grid_y, grid_x = np.mgrid[0:60, 0:80]
    ramp = 0.07 * (grid_x * tg.CELL) - 0.04 * (grid_y * tg.CELL)
    flat = tg.surface_gradient(ramp, tg.CELL, 1.95)
    offset = tg.surface_gradient(ramp + 250.0, tg.CELL, 1.95)
    core = (slice(12, -12), slice(12, -12))
    assert np.allclose(flat[0][core], offset[0][core], atol=1e-9)
    assert np.allclose(flat[1][core], offset[1][core], atol=1e-9)
    slope = np.degrees(np.arctan(np.hypot(offset[0][core], offset[1][core])))
    assert abs(slope.mean() - math.degrees(math.atan(math.hypot(0.07, 0.04)))) \
        < 1e-6


def test_gradient_reports_a_known_grade_in_degrees():
    grid_y, grid_x = np.mgrid[0:50, 0:50]
    for degrees in (3.0, 6.0, 12.0, 20.0):
        surface = math.tan(math.radians(degrees)) * (grid_x * tg.CELL) + 100.0
        a, b = tg.surface_gradient(surface, tg.CELL, 1.95)
        core = (slice(12, -12), slice(12, -12))
        got = np.degrees(np.arctan(np.hypot(a[core], b[core])))
        assert abs(got.mean() - degrees) < 1e-6


# --- ramp vs kerb ---------------------------------------------------------
def test_a_uniform_ramp_is_not_a_kerb():
    grid_y, grid_x = np.mgrid[0:80, 0:80]
    ground = math.tan(math.radians(24.0)) * (grid_x * tg.CELL) + 8.0
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    residual = tg.detrended_step(ground, slope_x, slope_y, tg.CELL)
    core = residual[12:-12, 12:-12]
    assert float(core.max()) < 0.02, core.max()
    raw = np.zeros_like(ground)
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        shifted = np.roll(np.roll(ground, dy, 0), dx, 1)
        raw = np.maximum(raw, np.abs(ground - shifted))
    assert float(raw[12:-12, 12:-12].max()) > tg.STEP_M, \
        "the old neighbour drop must fire on this ramp, or the test is vacuous"


def test_laplacian_ignores_a_ramp_and_keeps_a_kerb():
    grid_y, grid_x = np.mgrid[0:80, 0:80]
    ramp = math.tan(math.radians(12.0)) * (grid_x * tg.CELL) + 5.0
    assert not tg.curvature_kerb(ramp, tg.CELL, 0.10)[15:-15, 15:-15].any()
    mixed = ramp.copy()
    mixed[:, 40:] = mixed[:, 40:] + 0.14
    kerb = tg.curvature_kerb(mixed, tg.CELL, 0.10)
    assert kerb[20:60, 39:42].any()
    assert not kerb[20:60, 10:25].any()


def test_a_kerb_across_a_ramp_is_still_a_kerb():
    grid_y, grid_x = np.mgrid[0:80, 0:80]
    ground = math.tan(math.radians(8.0)) * (grid_x * tg.CELL) + 8.0
    ground[:, 40:] = ground[:, 40:] + 0.15
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    residual = tg.detrended_step(ground, slope_x, slope_y, tg.CELL)
    assert float(residual[20:60, 39:42].max()) > 0.10
    assert float(residual[20:60, 8:20].max()) < 0.03


def _voxelized_ramp(degrees=12.0, voxel_m=0.20, ny=80, nx=80):
    """A downhill grade quantized like the 0.20 m merged cloud."""
    grid_y, grid_x = np.mgrid[0:ny, 0:nx]
    continuous = math.tan(math.radians(degrees)) * (grid_x * tg.CELL) + 5.0
    return np.floor(continuous / voxel_m) * voxel_m


def test_ramp_aware_kerb_does_not_scribble_on_ground():
    ground = _voxelized_ramp(ny=80, nx=80)
    before = ground.copy()
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    tg.ramp_aware_kerb(ground, slope_x, slope_y, tg.CELL)
    assert np.array_equal(ground, before)


def test_a_flat_kerb_survives_a_planning_scale_raster():
    """Python 3.14 / numpy elides ufuncs into locals past ~200x200."""
    ny = nx = 400
    ground = np.zeros((ny, nx), np.float64)
    ground[:, 200:] = 0.14
    before = ground.copy()
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    kerb = tg.ramp_aware_kerb(ground, slope_x, slope_y, tg.CELL)
    assert np.array_equal(ground, before)
    band = kerb[80:320, 198:203]
    assert band.mean() > 0.3, "a 14 cm lip must be a line, not a speck"
    assert not kerb[80:320, 20:80].any()


def test_a_12cm_side_lip_on_a_large_voxelized_ramp_is_a_line():
    ny = nx = 400
    ground = _voxelized_ramp(ny=ny, nx=nx)
    ground[200:, :] = ground[200:, :] - 0.12
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    kerb = tg.ramp_aware_kerb(ground, slope_x, slope_y, tg.CELL)
    lip = kerb[197:204, 50:350]
    row_frac = lip.mean(axis=1)
    assert (row_frac > 0.8).any(), (
        "a 12 cm walkway edge on the ramp must be a connected line; "
        "row fractions %s" % np.round(row_frac, 3))
    on_ramp = kerb[20:80, 50:350]
    assert on_ramp.mean() < 0.05, (
        "the descent itself must stay clear; fired on %.1f%%"
        % (100.0 * on_ramp.mean()))


def test_an_along_slope_drop_survives_a_planning_scale_raster():
    ny = nx = 400
    grid_y, grid_x = np.mgrid[0:ny, 0:nx]
    ground = math.tan(math.radians(8.0)) * (grid_x * tg.CELL) + 5.0
    ground[:, 200:] = ground[:, 200:] - 0.30
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    kerb = tg.ramp_aware_kerb(ground, slope_x, slope_y, tg.CELL)
    col_frac = kerb[80:320, 196:205].mean(axis=0)
    assert (col_frac > 0.8).any(), (
        "a 0.30 m along-grade drop must be a line; col fractions %s"
        % np.round(col_frac, 3))


def test_a_smoothed_12deg_voxel_ramp_is_not_half_steep():
    ground = _voxelized_ramp(degrees=12.0, ny=80, nx=80)
    slope = tg.smooth_slope_deg(ground, tg.CELL)
    core = slope[18:-18, 18:-18]
    assert abs(float(core.mean()) - 12.0) < 1.0
    assert float((core > tg.SLOPE_BLOCK_DEG).mean()) < 0.05


def test_a_voxelized_ramp_is_not_a_kerb():
    """The 0.20 m map turns a ramp into stairs. Those stairs are the grade."""
    ground = _voxelized_ramp()
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    kerb = tg.ramp_aware_kerb(ground, slope_x, slope_y, tg.CELL)
    core = kerb[18:-18, 18:-18]
    assert not core.any(), (
        "voxel stairs of a 12 deg ramp must not be kerbs; fired on %.1f%%"
        % (100.0 * core.mean()))


def test_a_side_kerb_on_a_voxelized_ramp_is_a_kerb():
    """The walkway edge while descending, not the descent itself."""
    ground = _voxelized_ramp()
    ground[40:, :] = ground[40:, :] - 0.15
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    kerb = tg.ramp_aware_kerb(ground, slope_x, slope_y, tg.CELL)
    assert kerb[38:43, 20:60].any(), "the side lip on the ramp must fire"
    on_ramp = kerb[8:28, 20:60]
    assert on_ramp.mean() < 0.05, (
        "the descending surface itself must stay clear; fired on %.1f%%"
        % (100.0 * on_ramp.mean()))


def test_an_along_slope_drop_larger_than_a_voxel_is_a_kerb():
    """A real stair or drop across the path, bigger than map quantization."""
    ground = _voxelized_ramp(degrees=8.0)
    ground[:, 50:] = ground[:, 50:] - 0.30
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    kerb = tg.ramp_aware_kerb(ground, slope_x, slope_y, tg.CELL)
    assert kerb[20:60, 48:53].any()


def test_a_flat_kerb_is_still_a_kerb():
    ground = np.zeros((80, 80), np.float64)
    ground[:, 40:] = 0.14
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, 1.95)
    kerb = tg.ramp_aware_kerb(ground, slope_x, slope_y, tg.CELL)
    assert kerb[20:60, 39:42].any()
    assert not kerb[20:60, 8:25].any()


def test_isolated_canopy_is_not_a_building_but_a_dense_core_is():
    ny = nx = 40
    body = np.zeros((ny, nx))
    body[6:9, 6:9] = 4          # isolated canopy, no dense core
    body[20:28, 20] = 10        # wall: dense column
    body[20:28, 21] = 4         # wall fringe
    mask = tg.body_obstruction(body, dense_returns=8, attached_returns=3)
    assert not mask[6:9, 6:9].any(), "canopy without a dense core is not a wall"
    assert mask[20:28, 20].all(), "the dense wall core must stay"
    assert mask[20:28, 21].any(), "the sparse fringe of a wall stays attached"


def test_sparse_body_returns_are_not_a_wall_at_raised_threshold():
    ny = nx = 24
    body = np.zeros((ny, nx))
    body[11:14, 11:14] = 3
    body[5:8, 5:8] = 10
    ground = np.zeros((ny, nx), np.float64)
    grid = dict(cell=tg.CELL, min_x=0.0, min_y=0.0, nx=nx, ny=ny,
                known=np.ones((ny, nx), bool),
                to_route=np.ones((ny, nx)),
                filled=ground)
    land = dict(ground=ground, body=body, step=np.zeros((ny, nx)),
                residual_step=np.zeros((ny, nx)),
                slope_deg=np.zeros((ny, nx)),
                gate_slope_deg=np.zeros((ny, nx)),
                near_step=np.zeros((ny, nx), bool),
                slope_x=np.zeros((ny, nx)), slope_y=np.zeros((ny, nx)))
    strict = tg.traversability(grid, land, trust_driven=False,
                               min_body_returns=2)
    raised = tg.traversability(grid, land, trust_driven=False,
                               min_body_returns=8)
    assert strict["obstruction"][12, 12]
    assert not raised["obstruction"][12, 12]
    assert raised["obstruction"][6, 6]
    core = tg.traversability(grid, land, trust_driven=False,
                             dense_body_returns=8, attached_body_returns=3)
    assert not core["obstruction"][11:14, 11:14].any()
    assert core["obstruction"][5:8, 5:8].any()


# --- speck removal keeps the thin things that matter ----------------------
def test_a_one_cell_kerb_line_survives_and_a_speck_does_not():
    mask = np.zeros((20, 20), bool)
    mask[5, 2:12] = True          # a kerb: one cell wide, ten long
    mask[15, 15] = True           # a speck
    mask[17, 3:5] = True          # two cells: below MIN_STEP_CELLS
    kept = tg._drop_small(mask, tg.MIN_STEP_CELLS)
    assert kept[5, 2:12].all(), "a one-cell-wide kerb must not be cleaned up"
    assert not kept[15, 15]
    assert not kept[17, 3:5].any()


# --- corners --------------------------------------------------------------
def build_world(shape, blocks):
    passable = np.ones(shape, bool)
    for rows, cols in blocks:
        passable[rows, cols] = False
    passable[0, :] = passable[-1, :] = False
    passable[:, 0] = passable[:, -1] = False
    return passable


def test_corners_land_on_a_box_and_nowhere_else():
    passable = build_world((60, 60), [(slice(25, 36), slice(25, 36))])
    nodes = tg.corner_nodes(passable, tg.CELL, min_separation_m=0.15)
    assert len(nodes) == 4, nodes
    want = {(24, 24), (24, 36), (36, 24), (36, 36)}
    assert {tuple(n) for n in nodes} == want


def test_corner_thinning_keeps_distinct_corners_far_apart():
    passable = build_world((120, 120), [(slice(20, 41), slice(20, 41)),
                                        (slice(20, 41), slice(80, 101))])
    dense = tg.corner_nodes(passable, tg.CELL, min_separation_m=0.15)
    thinned = tg.corner_nodes(passable, tg.CELL, min_separation_m=1.2)
    assert len(dense) == 8
    assert len(thinned) == 8, "thinning must not merge separate obstacles"


def test_an_interior_free_cell_is_never_a_corner():
    passable = np.ones((40, 40), bool)
    passable[0, :] = passable[-1, :] = passable[:, 0] = passable[:, -1] = False
    nodes = tg.corner_nodes(passable, tg.CELL, min_separation_m=0.15)
    assert len(nodes) == 0, "a convex room has no corners a taut path bends at"


# --- visibility -----------------------------------------------------------
def test_a_wall_blocks_the_line_and_its_gap_does_not():
    passable = np.ones((40, 40), bool)
    passable[20, 0:30] = False
    assert not tg.visible(passable, 10, 10, 30, 10)
    assert tg.visible(passable, 10, 35, 30, 35)
    assert tg.visible(passable, 10, 10, 10, 30)


def test_visibility_is_symmetric():
    rng = np.random.default_rng(7)
    passable = rng.random((50, 50)) > 0.15
    for _ in range(200):
        r0, c0, r1, c1 = rng.integers(1, 49, 4)
        assert tg.visible(passable, r0, c0, r1, c1) == \
            tg.visible(passable, r1, c1, r0, c0)


# --- cost -----------------------------------------------------------------
def cost_world(slope_value, clearance_value, shape=(60, 60)):
    grid = dict(cell=tg.CELL, min_x=0.0, min_y=0.0, nx=shape[1], ny=shape[0])
    field = np.full(shape, slope_value, float)
    land = dict(slope_deg=field, gate_slope_deg=field)
    masks = dict(reachable=np.ones(shape, bool),
                 clearance=np.full(shape, clearance_value, float))
    return grid, land, masks


def test_cost_is_pure_length_on_flat_well_cleared_ground():
    grid, land, masks = cost_world(0.0, 2.0)
    nodes = np.array([[1.0, 1.0], [4.0, 5.0]])
    cost = tg.edge_cost(nodes, 0, 1, land["slope_deg"], masks["clearance"],
                        masks["reachable"], tg.CELL, 0.0, 0.0)
    assert cost == pytest.approx(5.0, abs=1e-9)


def test_cost_never_falls_below_length_so_the_heuristic_stays_admissible():
    nodes = np.array([[1.0, 1.0], [4.0, 5.0]])
    for slope in (0.0, 2.0, 3.0, 6.0, 11.9):
        for clear in (0.05, 0.45, 3.0):
            grid, land, masks = cost_world(slope, clear)
            cost = tg.edge_cost(nodes, 0, 1, land["slope_deg"],
                                masks["clearance"], masks["reachable"],
                                tg.CELL, 0.0, 0.0)
            assert cost >= 5.0 - 1e-9, (slope, clear, cost)


def test_grade_above_the_demonstrated_envelope_is_refused_outright():
    nodes = np.array([[1.0, 1.0], [4.0, 5.0]])
    grid, land, masks = cost_world(tg.SLOPE_BLOCK_DEG + 0.5, 2.0)
    assert tg.edge_cost(nodes, 0, 1, land["slope_deg"], masks["clearance"],
                        masks["reachable"], tg.CELL, 0.0, 0.0) is None


def test_cost_rises_with_grade_and_with_tightness():
    nodes = np.array([[1.0, 1.0], [4.0, 5.0]])

    def cost(slope, clear):
        grid, land, masks = cost_world(slope, clear)
        return tg.edge_cost(nodes, 0, 1, land["slope_deg"], masks["clearance"],
                            masks["reachable"], tg.CELL, 0.0, 0.0)

    assert cost(0.0, 2.0) < cost(6.0, 2.0) < cost(11.0, 2.0)
    assert cost(0.0, 2.0) < cost(0.0, 0.30) < cost(0.0, 0.05)


def test_an_edge_crossing_refused_ground_has_no_cost():
    grid, land, masks = cost_world(0.0, 2.0)
    masks["reachable"][20, :] = False
    nodes = np.array([[1.0, 1.0], [1.0, 5.0]])
    assert tg.edge_cost(nodes, 0, 1, land["slope_deg"], masks["clearance"],
                        masks["reachable"], tg.CELL, 0.0, 0.0) is None


# --- the planner ----------------------------------------------------------
def plan_between(passable, slope_deg, start_xy, goal_xy, cell=tg.CELL):
    ny, nx = passable.shape
    grid = dict(cell=cell, min_x=0.0, min_y=0.0, nx=nx, ny=ny)
    land = dict(slope_deg=slope_deg, gate_slope_deg=slope_deg)
    masks = dict(reachable=passable,
                 clearance=np.full(passable.shape, 5.0, float))
    nodes_rc = tg.corner_nodes(passable, cell, min_separation_m=cell)
    nodes_xy, adjacency = tg.build_graph(nodes_rc, grid, land, masks)
    nodes_xy, adjacency, start = tg.attach(nodes_xy, adjacency, grid, land,
                                           masks, start_xy)
    nodes_xy, adjacency, goal = tg.attach(nodes_xy, adjacency, grid, land,
                                          masks, goal_xy)
    path, cost = tg.astar(nodes_xy, adjacency, start, goal)
    return None if path is None else nodes_xy[path], cost


def test_a_clear_room_is_crossed_in_one_straight_edge():
    passable = np.ones((80, 80), bool)
    passable[0, :] = passable[-1, :] = passable[:, 0] = passable[:, -1] = False
    start = (2.0, 2.0)
    goal = (9.0, 9.0)
    path, cost = plan_between(passable, np.zeros((80, 80)), start, goal)
    assert len(path) == 2, path
    assert cost == pytest.approx(math.hypot(7.0, 7.0), abs=1e-9)


def test_the_path_around_a_wall_matches_the_taut_length():
    """A wall from the left edge to x = 6 m, with a gap beyond it.

    The shortest route is the two-segment bend around the wall tip, so the
    planner's answer has a closed form and any smoothing error shows up.
    """
    ny = nx = 100
    passable = np.ones((ny, nx), bool)
    passable[0, :] = passable[-1, :] = passable[:, 0] = passable[:, -1] = False
    wall_row = 50
    wall_end = 40
    passable[wall_row, 0:wall_end] = False
    start = ((10 + 0.5) * tg.CELL, (20 + 0.5) * tg.CELL)
    goal = ((10 + 0.5) * tg.CELL, (80 + 0.5) * tg.CELL)
    path, cost = plan_between(passable, np.zeros((ny, nx)), start, goal)
    assert path is not None
    tip = np.array([(wall_end + 0.5) * tg.CELL, (wall_row + 0.5) * tg.CELL])
    taut = (np.hypot(*(np.array(start) - tip))
            + np.hypot(*(np.array(goal) - tip)))
    assert cost == pytest.approx(taut, rel=0.02), (cost, taut)
    assert 3 <= len(path) <= 4
    # and it really does pass the wall tip rather than cut through
    bend = path[1:-1]
    assert np.hypot(*(bend[0] - tip)) < 0.5


def test_a_sealed_room_has_no_path():
    ny = nx = 60
    passable = np.ones((ny, nx), bool)
    passable[0, :] = passable[-1, :] = passable[:, 0] = passable[:, -1] = False
    passable[30, :] = False
    # x comes from the column, y from the row: the wall spans a row, so the
    # two points have to differ in y for it to separate them.
    start = ((10 + 0.5) * tg.CELL, (10 + 0.5) * tg.CELL)
    goal = ((10 + 0.5) * tg.CELL, (50 + 0.5) * tg.CELL)
    path, cost = plan_between(passable, np.zeros((ny, nx)), start, goal)
    assert path is None
    assert cost == math.inf


def test_cell_graph_forbids_diagonal_corner_cut():
    reachable = np.zeros((3, 3), dtype=bool)
    reachable[0, 0] = True
    reachable[1, 1] = True
    grid = {
        "cell": 0.2,
        "min_x": 0.0,
        "min_y": 0.0,
        "nx": 3,
        "ny": 3,
    }
    land = {"gate_slope_deg": np.zeros((3, 3), dtype=float)}
    masks = {
        "reachable": reachable,
        "clearance": np.ones((3, 3), dtype=float),
    }

    graph, index, _, _, _ = tg.cell_graph(grid, land, masks)

    assert graph[index[0, 0], index[1, 1]] == 0


def test_the_planner_takes_the_longer_way_round_to_avoid_a_steep_ramp():
    """Two corridors to the same goal: the short one is steep, the long one flat.

    This is the point of costing grade rather than only gating it. The cost is a
    line integral, so the trade is explicit: at SLOPE_WEIGHT = 1.5 an edge at
    the steepest allowed grade costs at most 1 + 1.5 * (12 - 3) / 12 = 2.1x its
    length, so the planner will spend up to roughly twice the distance to stay
    off it and no more. A 3 m sidestep buys its way out here; a 12 m one would
    not, and should not.
    """
    ny, nx = 120, 120
    passable = np.zeros((ny, nx), bool)
    passable[55:65, 5:115] = True          # short, direct corridor
    passable[75:85, 5:115] = True          # parallel corridor, 3 m south
    passable[55:85, 5:15] = True           # west link
    passable[55:85, 105:115] = True        # east link
    slope = np.zeros((ny, nx))
    slope[55:65, 20:100] = 11.0            # ramp on the direct route only

    start = ((10 + 0.5) * tg.CELL, (60 + 0.5) * tg.CELL)
    goal = ((110 + 0.5) * tg.CELL, (60 + 0.5) * tg.CELL)
    direct_m = float(np.hypot(goal[0] - start[0], goal[1] - start[1]))

    flat_path, flat_cost = plan_between(passable, np.zeros((ny, nx)), start,
                                        goal)
    steep_path, steep_cost = plan_between(passable, slope, start, goal)
    assert flat_path is not None and steep_path is not None

    # flat: the direct corridor wins and the path never leaves it
    assert flat_cost == pytest.approx(direct_m, rel=1e-6)
    assert flat_path[:, 1].max() < (70 * tg.CELL)

    # costed ramp: the planner leaves the direct corridor for the south one
    assert steep_path[:, 1].max() > (74 * tg.CELL), steep_path
    # and the detour it accepts is longer in metres but cheaper in cost
    walked = float(np.hypot(*np.diff(steep_path, axis=0).T).sum())
    assert walked > direct_m
    assert steep_cost < direct_m * (1.0 + tg.SLOPE_WEIGHT
                                    * (11.0 - tg.SLOPE_SLOW_DEG)
                                    * (80 * tg.CELL / direct_m)
                                    / tg.SLOPE_BLOCK_DEG)


def test_a_short_ramp_is_not_worth_a_long_detour():
    """The same machinery must also decline a bad trade, or the cost is just a
    gate with extra steps.
    """
    ny, nx = 120, 120
    passable = np.zeros((ny, nx), bool)
    passable[15:25, 5:115] = True           # direct
    passable[95:105, 5:115] = True          # detour, 12 m south
    passable[15:105, 5:15] = True
    passable[15:105, 105:115] = True
    slope = np.zeros((ny, nx))
    slope[15:25, 55:65] = 11.0              # only 1.5 m of ramp

    start = ((10 + 0.5) * tg.CELL, (20 + 0.5) * tg.CELL)
    goal = ((110 + 0.5) * tg.CELL, (20 + 0.5) * tg.CELL)
    path, _ = plan_between(passable, slope, start, goal)
    assert path is not None
    assert path[:, 1].max() < (40 * tg.CELL), \
        "a 1.5 m ramp must not buy a 24 m detour"


def test_a_blocked_corridor_forces_the_detour_even_with_no_slope():
    ny, nx = 120, 120
    passable = np.zeros((ny, nx), bool)
    passable[55:65, 5:115] = True
    passable[95:105, 5:115] = True
    passable[55:105, 5:15] = True
    passable[55:105, 105:115] = True
    passable[55:65, 58:62] = False        # the direct corridor is closed
    start = ((10 + 0.5) * tg.CELL, (60 + 0.5) * tg.CELL)
    goal = ((110 + 0.5) * tg.CELL, (60 + 0.5) * tg.CELL)
    path, cost = plan_between(passable, np.zeros((ny, nx)), start, goal)
    assert path is not None, "a detour exists and must be found"
    assert path[:, 1].max() > (90 * tg.CELL)


def test_astar_agrees_with_dijkstra_on_a_random_graph():
    """A* must not trade optimality for the heuristic."""
    rng = np.random.default_rng(11)
    count = 120
    nodes_xy = rng.random((count, 2)) * 40.0
    adjacency = [[] for _ in range(count)]
    for i in range(count):
        for j in range(i + 1, count):
            d = float(np.hypot(*(nodes_xy[i] - nodes_xy[j])))
            if d < 9.0:
                weight = d * (1.0 + rng.random())
                adjacency[i].append((j, weight))
                adjacency[j].append((i, weight))

    def dijkstra(start, goal):
        import heapq
        best = {start: 0.0}
        frontier = [(0.0, start)]
        seen = set()
        while frontier:
            cost, node = heapq.heappop(frontier)
            if node == goal:
                return cost
            if node in seen:
                continue
            seen.add(node)
            for other, weight in adjacency[node]:
                if cost + weight < best.get(other, math.inf):
                    best[other] = cost + weight
                    heapq.heappush(frontier, (cost + weight, other))
        return math.inf

    for start, goal in ((0, 50), (3, 90), (17, 4), (60, 61)):
        _, got = tg.astar(nodes_xy, adjacency, start, goal)
        assert got == pytest.approx(dijkstra(start, goal), rel=1e-9)


# --- the terrain split ----------------------------------------------------
def test_a_synthetic_kerb_reads_as_a_step_and_not_as_a_slope():
    """A 0.12 m kerb must be refused by the step mask, and the footway beside
    it must NOT be refused for slope - the slope window straddles the kerb and
    would otherwise shed a 1 m skirt of false steep ground onto flat pavement.
    """
    ny, nx = 80, 200
    ground = np.zeros((ny, nx), np.float32)
    ground[:, 100:] = 0.12                     # kerb at column 100
    grid = dict(cell=tg.CELL, min_x=0.0, min_y=0.0, nx=nx, ny=ny,
                known=np.ones((ny, nx), bool),
                to_route=np.full((ny, nx), 3.0),
                filled=ground)
    slope_x, slope_y = tg.surface_gradient(ground, tg.CELL, tg.SLOPE_BASELINE_M)
    slope_deg = np.degrees(np.arctan(np.hypot(slope_x, slope_y)))
    step = np.zeros_like(ground)
    for shift_y, shift_x in ((0, 1), (1, 0), (1, 1), (1, -1)):
        step = np.maximum(step, np.abs(
            ground - np.roll(np.roll(ground, shift_y, 0), shift_x, 1)))
    near_step = ndimage.binary_dilation(
        step > tg.STEP_M,
        structure=tg.disk(max(int(round(tg.STEP_BLEED_M / tg.CELL)), 1)))
    land = dict(ground=ground, body=np.zeros((ny, nx)), step=step,
                slope_deg=slope_deg, near_step=near_step,
                gate_slope_deg=np.where(near_step, 0.0, slope_deg),
                slope_x=slope_x, slope_y=slope_y)
    masks = tg.traversability(grid, land)

    assert masks["stepped"][:, 100].any(), "the kerb itself must be refused"
    far = masks["steep"][:, 60:85]
    assert not far.any(), "flat pavement beside a kerb must not read as steep"
    # the pavement well away from the kerb stays free
    assert masks["free"][:, 20:60].mean() > 0.9


# --- the bulk graph must agree with the scalar cost exactly ---------------
def test_bulk_graph_matches_the_scalar_cost_pair_for_pair():
    """build_graph() is a vectorised rewrite of edge_cost(). If the two ever
    disagree, every number downstream is unaudited, so the equality is a test
    rather than a comment.
    """
    rng = np.random.default_rng(5)
    ny = nx = 90
    passable = rng.random((ny, nx)) > 0.12
    passable = np.logical_or(passable, np.zeros((ny, nx), bool))
    slope = rng.random((ny, nx)) * 16.0
    clearance = rng.random((ny, nx)) * 1.2
    grid = dict(cell=tg.CELL, min_x=-13.7, min_y=204.9, nx=nx, ny=ny)
    land = dict(slope_deg=slope, gate_slope_deg=slope)
    masks = dict(reachable=passable, clearance=clearance)

    nodes_rc = tg.corner_nodes(passable, tg.CELL, min_separation_m=0.45)
    assert len(nodes_rc) > 40, len(nodes_rc)
    nodes_xy, adjacency = tg.build_graph(nodes_rc, grid, land, masks,
                                        max_edge_m=4.0, block_elements=997)
    got = {}
    for a, neighbours in enumerate(adjacency):
        for b, weight in neighbours:
            got[(min(a, b), max(a, b))] = weight

    want = {}
    for a in range(len(nodes_xy)):
        for b in range(a + 1, len(nodes_xy)):
            if np.hypot(*(nodes_xy[a] - nodes_xy[b])) > 4.0:
                continue
            cost = tg.edge_cost(nodes_xy, a, b, slope, clearance, passable,
                                tg.CELL, grid["min_x"], grid["min_y"])
            if cost is not None:
                want[(a, b)] = cost

    assert set(got) == set(want), (len(got), len(want),
                                   sorted(set(got) ^ set(want))[:8])
    for key in want:
        assert got[key] == pytest.approx(want[key], rel=1e-12)


# --- the complete grid search and tautening -------------------------------
def world(shape, blocks, slope_value=0.0):
    ny, nx = shape
    passable = np.ones(shape, bool)
    for rows, cols in blocks:
        passable[rows, cols] = False
    passable[0, :] = passable[-1, :] = False
    passable[:, 0] = passable[:, -1] = False
    grid = dict(cell=tg.CELL, min_x=0.0, min_y=0.0, nx=nx, ny=ny)
    field = np.full(shape, slope_value, float)
    land = dict(slope_deg=field, gate_slope_deg=field)
    masks = dict(reachable=passable,
                 clearance=np.full(shape, 5.0, float))
    return grid, land, masks, passable


def cell_xy(row, col):
    return ((col + 0.5) * tg.CELL, (row + 0.5) * tg.CELL)


def test_the_grid_search_finds_a_path_where_the_corner_graph_cannot():
    """A 0.6 m ribbon that snakes: the real corridor's shape.

    This is the case that broke the visibility graph on the map, reproduced
    small. The grid search must still connect the ends.
    """
    ny, nx = 60, 120
    passable = np.zeros((ny, nx), bool)
    passable[10:14, 5:60] = True            # 0.6 m wide leg
    passable[10:46, 56:60] = True           # turn
    passable[42:46, 56:115] = True          # return leg
    grid = dict(cell=tg.CELL, min_x=0.0, min_y=0.0, nx=nx, ny=ny)
    zero = np.zeros((ny, nx))
    land = dict(slope_deg=zero, gate_slope_deg=zero)
    masks = dict(reachable=passable, clearance=np.full((ny, nx), 0.3))
    start, goal = cell_xy(12, 7), cell_xy(44, 112)

    corner_only = tg.corner_nodes(passable, tg.CELL)
    nodes_xy, adjacency = tg.build_graph(corner_only, grid, land, masks)
    nodes_xy, adjacency, s = tg.attach(nodes_xy, adjacency, grid, land, masks,
                                       start)
    nodes_xy, adjacency, g = tg.attach(nodes_xy, adjacency, grid, land, masks,
                                       goal)
    graph_path, _ = tg.astar(nodes_xy, adjacency, s, g)

    graph_, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    cells, cost = tg.grid_plan(graph_, index, rows, cols, grid, start, goal)
    assert cells is not None, "the grid search must connect a snaking ribbon"
    assert cost > 0
    # the path stays inside the ribbon
    assert passable[cells[:, 0], cells[:, 1]].all()
    # and it actually goes round the turn rather than cutting across
    assert cells[:, 1].max() >= 56
    del graph_path


def test_tautening_shortens_without_ever_cutting_a_corner():
    grid, land, masks, passable = world((80, 80), [])
    graph, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    cells, _ = tg.grid_plan(graph, index, rows, cols, grid,
                            cell_xy(10, 10), cell_xy(70, 70))
    pulled = tg.taut(cells, open_cell)
    assert len(pulled) == 2, pulled          # an empty room is one segment
    assert tuple(pulled[0]) == tuple(cells[0])
    assert tuple(pulled[-1]) == tuple(cells[-1])


def test_tautening_keeps_every_segment_clear_of_refused_cells():
    ny, nx = 90, 90
    blocks = [(slice(30, 60), slice(30, 40)), (slice(20, 40), slice(60, 70)),
              (slice(60, 75), slice(50, 58))]
    grid, land, masks, passable = world((ny, nx), blocks)
    graph, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    cells, _ = tg.grid_plan(graph, index, rows, cols, grid,
                            cell_xy(10, 10), cell_xy(80, 80))
    assert cells is not None
    pulled = tg.taut(cells, open_cell)
    assert len(pulled) < len(cells)
    for a, b in zip(pulled[:-1], pulled[1:]):
        assert tg.visible(open_cell, a[0], a[1], b[0], b[1]), (a, b)
    walked_before = float(np.hypot(*np.diff(cells, axis=0).T).sum())
    walked_after = float(np.hypot(*np.diff(pulled, axis=0).T).sum())
    assert walked_after <= walked_before + 1e-9


def test_the_grid_search_refuses_ground_above_the_slope_gate():
    ny, nx = 60, 60
    slope = np.zeros((ny, nx))
    slope[:, 28:32] = tg.SLOPE_BLOCK_DEG + 1.0   # an impassable ramp wall
    grid, land, masks, passable = world((ny, nx), [])
    land["slope_deg"] = slope
    land["gate_slope_deg"] = slope
    graph, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    cells, cost = tg.grid_plan(graph, index, rows, cols, grid,
                               cell_xy(30, 10), cell_xy(30, 50))
    assert cells is None and cost == math.inf


def island_world():
    """An island with two 1.65 m channels round it - narrow enough that a
    single 2.4 m obstruction really does close one.
    """
    ny, nx = 80, 80
    grid, land, masks, passable = world((ny, nx), [(slice(12, 68),
                                                   slice(20, 60))])
    graph, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    return grid, land, masks, graph, index, rows, cols, open_cell


def test_a_blockage_forces_the_grid_search_onto_the_other_branch():
    grid, land, masks, graph, index, rows, cols, open_cell = island_world()
    start, goal = cell_xy(40, 10), cell_xy(40, 70)
    cells, cost = tg.grid_plan(graph, index, rows, cols, grid, start, goal)
    assert cells is not None
    took_north = cells[:, 0].mean() > 40.0
    channel = cell_xy(73 if took_north else 5, 40)
    blocked = tg.cells_within(rows, cols, grid, channel, 1.2)
    assert len(blocked) > 0

    again, again_cost = tg.grid_plan(graph, index, rows, cols, grid, start,
                                     goal, blocked_cells=blocked)
    assert again is not None, "the other channel is still open"
    assert (again[:, 0].mean() > 40.0) != took_north, \
        "the re-plan must use the other side of the island"
    assert again_cost >= cost - 1e-9
    assert masks["reachable"][again[:, 0], again[:, 1]].all()


def test_blocking_both_channels_leaves_no_route():
    grid, land, masks, graph, index, rows, cols, open_cell = island_world()
    start, goal = cell_xy(40, 10), cell_xy(40, 70)
    assert tg.grid_plan(graph, index, rows, cols, grid, start, goal)[0] \
        is not None
    both = np.unique(np.concatenate([
        tg.cells_within(rows, cols, grid, cell_xy(73, 40), 1.2),
        tg.cells_within(rows, cols, grid, cell_xy(5, 40), 1.2)]))
    again, cost = tg.grid_plan(graph, index, rows, cols, grid, start, goal,
                              blocked_cells=both)
    assert again is None and cost == math.inf


def test_cells_within_selects_exactly_the_disc():
    grid, land, masks, passable = world((40, 40), [])
    graph, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    centre = cell_xy(20, 20)
    picked = tg.cells_within(rows, cols, grid, centre, 0.5)
    x = grid["min_x"] + (cols[picked] + 0.5) * tg.CELL
    y = grid["min_y"] + (rows[picked] + 0.5) * tg.CELL
    assert len(picked) > 0
    assert np.hypot(x - centre[0], y - centre[1]).max() <= 0.5 + 1e-12
    outside = np.setdiff1d(np.arange(len(rows)), picked)
    ox = grid["min_x"] + (cols[outside] + 0.5) * tg.CELL
    oy = grid["min_y"] + (rows[outside] + 0.5) * tg.CELL
    assert np.hypot(ox - centre[0], oy - centre[1]).min() > 0.5


def test_grid_cost_and_visibility_cost_agree_on_a_straight_flat_run():
    """The two searches must be priced on the same scale or their numbers are
    not comparable, which is the whole reason cell_costs() is shared.
    """
    ny, nx = 40, 200
    grid, land, masks, passable = world((ny, nx), [])
    graph, index, rows, cols, open_cell = tg.cell_graph(grid, land, masks)
    start, goal = cell_xy(20, 10), cell_xy(20, 180)
    _, grid_cost = tg.grid_plan(graph, index, rows, cols, grid, start, goal)
    nodes = np.array([start, goal])
    straight = tg.edge_cost(nodes, 0, 1, land["slope_deg"], masks["clearance"],
                            passable, tg.CELL, 0.0, 0.0)
    assert grid_cost == pytest.approx(straight, rel=1e-9)


def test_cell_graph_preserves_large_reachable_mask():
    size = 512
    reachable = np.zeros((size, size), dtype=bool)
    reachable[2:-2, 2:-2] = True
    grid = {
        "cell": 0.2,
        "nx": size,
        "ny": size,
        "min_x": 0.0,
        "min_y": 0.0,
    }
    land = {"gate_slope_deg": np.zeros((size, size), dtype=float)}
    masks = {
        "reachable": reachable,
        "clearance": np.ones((size, size), dtype=float),
    }

    _, _, _, _, open_cell = tg.cell_graph(grid, land, masks)

    assert np.array_equal(open_cell, reachable)


# --- the surface must survive its own analysis ----------------------------
def synthetic_grid(ny=70, nx=160):
    """A raster with a real grade, a kerb and a post, built by hand."""
    from scipy.spatial import cKDTree
    filled = np.zeros((ny, nx), np.float64)
    grid_y, grid_x = np.mgrid[0:ny, 0:nx]
    filled += 0.07 * (grid_x * tg.CELL)          # 4 degree grade along x
    filled[:, 90:] += 0.12                       # kerb
    filled[30:34, 40:44] = 1.2                   # post
    route_xy = np.column_stack([
        tg.CELL * (np.arange(nx) + 0.5),
        np.full(nx, tg.CELL * (ny // 2 + 0.5))])
    tree = cKDTree(route_xy)
    centres = np.column_stack([
        (grid_x.ravel() + 0.5) * tg.CELL, (grid_y.ravel() + 0.5) * tg.CELL])
    to_route, station = tree.query(centres, k=1)
    grid = dict(cell=tg.CELL, min_x=0.0, min_y=0.0, nx=nx, ny=ny,
                known=np.ones((ny, nx), bool), filled=filled,
                inside_points=np.zeros((0, 3)), flat=np.zeros(0, np.int64),
                count=np.ones((ny, nx)),
                to_route=to_route.reshape(ny, nx),
                station=station.reshape(ny, nx))
    return grid


def test_terrain_does_not_overwrite_its_own_ground_surface():
    """The step loop used to elide into `ground` and destroy it.

    Under Python 3.14 the interpreter borrows the stack reference, so numpy sees
    refcount 1 on a named local array and writes the subtraction into it. On the
    real map this turned a -10.66..10.53 m surface into -16.09..16.09 m and
    every slope and step figure downstream was computed from the wreckage. The
    surface is therefore checked against an independent opening.
    """
    from scipy import ndimage
    grid = synthetic_grid()
    before = grid["filled"].copy()
    element = tg.disk(int(round(tg.GROUND_RADIUS_M / tg.CELL)))
    expected = ndimage.grey_dilation(
        ndimage.grey_erosion(before, footprint=element), footprint=element)

    land = tg.terrain(grid)
    assert np.array_equal(grid["filled"], before), "terrain() mutated its input"
    assert np.array_equal(land["ground"], expected), \
        "the ground surface was modified after it was built"


def test_terrain_step_and_slope_are_measured_from_the_intact_surface():
    from scipy import ndimage
    grid = synthetic_grid()
    element = tg.disk(int(round(tg.GROUND_RADIUS_M / tg.CELL)))
    ground = ndimage.grey_dilation(
        ndimage.grey_erosion(grid["filled"].copy(), footprint=element),
        footprint=element)
    want_step = np.zeros_like(ground)
    for shift_y, shift_x in ((0, 1), (1, 0), (1, 1), (1, -1)):
        shifted = np.roll(np.roll(ground, shift_y, 0), shift_x, 1)
        want_step = np.maximum(want_step, np.abs(ground - shifted))
    want_x, want_y = tg.surface_gradient(ground, tg.CELL, tg.SLOPE_BASELINE_M)

    land = tg.terrain(grid)
    assert np.array_equal(land["step"], want_step)
    assert np.allclose(land["slope_x"], want_x, atol=1e-12)
    assert np.allclose(land["slope_y"], want_y, atol=1e-12)
    # and the numbers are the ones the synthetic ground was built to have
    core = (slice(10, -10), slice(20, 80))
    grade = np.degrees(np.arctan(np.hypot(land["slope_x"], land["slope_y"])))
    assert abs(grade[core].mean() - math.degrees(math.atan(0.07))) < 0.05
    # the kerb reads as its own height plus one cell of the grade it sits on
    assert land["step"][:, 89:91].max() == pytest.approx(
        0.12 + 0.07 * tg.CELL, abs=1e-9)


def test_ground_opening_lifts_a_post_off_the_surface():
    """A bollard must not become a hill: the opened surface has to pass under
    it, or every post in the map turns into unwalkable terrain instead of an
    obstacle to go round.
    """
    ny = nx = 120
    filled = np.zeros((ny, nx), np.float32)
    filled[58:62, 58:62] = 1.1                 # a 0.6 m wide, 1.1 m tall post
    element = tg.disk(int(round(tg.GROUND_RADIUS_M / tg.CELL)))
    from scipy import ndimage
    opened = ndimage.grey_dilation(
        ndimage.grey_erosion(filled, footprint=element), footprint=element)
    assert opened[58:62, 58:62].max() < 0.01, "the post stayed in the ground"
    a, b = tg.surface_gradient(opened, tg.CELL, tg.SLOPE_BASELINE_M)
    assert np.degrees(np.arctan(np.hypot(a, b))).max() < 0.5


# --- blockage and re-planning --------------------------------------------
def test_segment_distance_matches_hand_computed_cases():
    nodes = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 5.0]])
    i = np.array([0, 0])
    j = np.array([1, 2])
    got = tg.segment_distance(nodes, i, j, (5.0, 3.0))
    # both feet land inside their segment: (5,0) and (0,3)
    assert got[0] == pytest.approx(3.0)
    assert got[1] == pytest.approx(5.0)
    # past the end of a segment clamps to the endpoint
    beyond = tg.segment_distance(nodes, np.array([0]), np.array([1]),
                                 (14.0, 0.0))
    assert beyond[0] == pytest.approx(4.0)
    corner = tg.segment_distance(nodes, np.array([0]), np.array([2]),
                                 (5.0, 9.0))
    assert corner[0] == pytest.approx(math.hypot(5.0, 4.0))
    # a degenerate segment is a point
    same = tg.segment_distance(nodes, np.array([1]), np.array([1]), (10.0, 3.0))
    assert same[0] == pytest.approx(3.0)


def test_a_blockage_removes_exactly_the_edges_it_touches():
    nodes = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 10.0], [10.0, 10.0]])
    adjacency = [[] for _ in range(4)]
    for a, b in ((0, 1), (0, 2), (1, 3), (2, 3)):
        adjacency[a].append((b, 10.0))
        adjacency[b].append((a, 10.0))
    edges = tg.graph_edges(adjacency)
    assert len(edges[0]) == 4
    # a disc on the middle of the bottom edge cuts that edge and no other
    cut = tg.adjacency_without(nodes, edges, (5.0, 0.0), 1.0)
    assert [n for n, _ in cut[0]] == [2]
    assert [n for n, _ in cut[1]] == [3]
    assert len(tg.graph_edges(cut)[0]) == 3


def test_replanning_round_a_blockage_finds_the_other_side_of_the_loop():
    nodes = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 10.0], [10.0, 10.0]])
    adjacency = [[] for _ in range(4)]
    for a, b, w in ((0, 1, 10.0), (1, 3, 10.0), (0, 2, 10.0), (2, 3, 10.0)):
        adjacency[a].append((b, w))
        adjacency[b].append((a, w))
    edges = tg.graph_edges(adjacency)
    path, cost = tg.astar(nodes, adjacency, 0, 3)
    assert cost == pytest.approx(20.0)
    blocked = tg.adjacency_without(nodes, edges, (5.0, 0.0), 1.0)
    detour, detour_cost = tg.astar(nodes, blocked, 0, 3)
    assert detour == [0, 2, 3]
    assert detour_cost == pytest.approx(20.0)
    # cutting both ways out isolates the goal
    both = tg.adjacency_without(nodes, edges, (5.0, 0.0), 1.0)
    both = tg.adjacency_without(nodes, tg.graph_edges(both), (0.0, 5.0), 1.0)
    assert tg.astar(nodes, both, 0, 3)[0] is None
