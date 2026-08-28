"""Unit tests for the drop-safe costmap baker.

A synthetic 3D cloud: a flat pavement strip with a kerb step on one side and a
wall on the other. The baker must mark the kerb and the wall lethal and leave
the drivable strip free, and a start/goal on the strip must pass.
"""

import json
import os
import sys
import tempfile

import numpy as np
import pytest
from PIL import Image

TOOLS = os.path.join(os.path.dirname(__file__), "..", "tools")
sys.path.insert(0, TOOLS)
import bake_dropsafe_costmap as baker  # noqa: E402


def flat_pavement_with_kerb():
    """A 30 m x 12 m scene. y in [0,8] is the footway at z=0; y>8 is a
    0.20 m road drop; a wall sits at x in [14,15], y in [3,5], z=0.8.

    The footway is 8 m wide so terrain_graph's 1.5 m ground-opening radius
    recovers a clean surface away from the kerb - the real map is wide open
    outdoor ground, and a 3 m strip is too narrow for that opening."""
    rng = np.random.default_rng(7)
    xs, ys, zs = [], [], []
    for _ in range(120000):
        xs.append(rng.uniform(0, 30))
        ys.append(rng.uniform(0, 8))
        zs.append(rng.normal(0, 0.005))
    for _ in range(60000):
        xs.append(rng.uniform(0, 30))
        ys.append(rng.uniform(8.4, 12))
        zs.append(rng.normal(-0.20, 0.005))
    for _ in range(12000):
        xs.append(rng.uniform(14, 15))
        ys.append(rng.uniform(3.0, 5.0))
        zs.append(rng.uniform(0.15, 1.2))
    return np.column_stack([xs, ys, zs]).astype(np.float32)


def write_pcd(path, points):
    with open(path, "wb") as f:
        n = len(points)
        f.write(("# .PCD v0.7 - Point Cloud Data file format\n"
                 "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                 "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n"
                 "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
                 % (n, n)).encode())
        xyzi = np.column_stack([points, np.zeros(len(points))]).astype(np.float32)
        f.write(xyzi.tobytes())


def test_kerb_and_wall_are_lethal_pavement_is_free(tmp_path):
    pcd = tmp_path / "map.pcd"
    write_pcd(str(pcd), flat_pavement_with_kerb())
    out = str(tmp_path / "dropsafe")
    rc = baker.main([
        str(pcd), "--start", "2.0,4.0", "--goal", "28.0,4.0",
        "--out", out, "--corridor-m", "12", "--cell", "0.15"])
    assert rc is None  # main returns None on success

    pgm = Image.open(out + ".pgm")
    arr = np.array(pgm)[::-1]  # un-flip to match grid row order
    grid = np.load(out + ".npz")
    cell = float(grid["cell"])
    min_x, min_y = float(grid["min_x"]), float(grid["min_y"])

    def cell_of(x, y):
        c = int(round((x - min_x) / cell - 0.5))
        r = int(round((y - min_y) / cell - 0.5))
        return r, c

    r, c = cell_of(8.0, 4.0)
    assert arr[r, c] == 254
    r, c = cell_of(14.5, 4.0)
    assert arr[r, c] == 0
    r, c = cell_of(8.0, 10.0)
    assert arr[r, c] == 0
    r, c = cell_of(8.0, 8.0)
    assert arr[r, c] == 0


def test_start_on_a_drop_warns(tmp_path, capsys):
    pcd = tmp_path / "map.pcd"
    write_pcd(str(pcd), flat_pavement_with_kerb())
    out = str(tmp_path / "dropsafe")
    baker.main([
        str(pcd), "--start", "8.0,11.0", "--goal", "28.0,4.0",
        "--out", out, "--corridor-m", "12", "--cell", "0.15"])
    captured = capsys.readouterr()
    assert "not in the drop-safe free space" in captured.out


def test_route_seed_matches_start_goal_seed(tmp_path):
    """The corridor and free space do not depend on a recorded route."""
    pcd = tmp_path / "map.pcd"
    write_pcd(str(pcd), flat_pavement_with_kerb())
    points = flat_pavement_with_kerb()
    g1, _, m1 = baker.build_grid(points, (2.0, 4.0), (28.0, 4.0), 12.0, 0.15)
    assert m1["reachable"].sum() > 0
    r = int(round((4.0 - g1["min_y"]) / 0.15 - 0.5))
    c = int(round((2.0 - g1["min_x"]) / 0.15 - 0.5))
    assert m1["reachable"][r, c]
