"""Unit tests for path_to_route_assets conversion."""

import json
import os
import sys

import numpy as np
import pytest

TOOLS = os.path.join(os.path.dirname(__file__), "..", "tools")
sys.path.insert(0, TOOLS)
import path_to_route_assets as p2r  # noqa: E402


def write_path(tmp_path, points, shaped="poses"):
    if shaped == "poses":
        doc = [{"pose": {"position": {"x": float(x), "y": float(y), "z": 0}}}
               for x, y in points]
    else:
        doc = [{"x": float(x), "y": float(y)} for x, y in points]
    p = tmp_path / "path.json"
    p.write_text(json.dumps(doc))
    return str(p)


def test_resample_even_spacing():
    poly = np.array([[0, 0], [1, 0], [2, 0], [5, 0]], dtype=float)
    dense = p2r.resample(poly, 0.5)
    seg = np.hypot(*np.diff(dense, axis=0).T)
    assert np.allclose(seg, 0.5, atol=1e-9)
    assert abs(dense[-1, 0] - 5.0) < 1e-6


def test_tangent_yaw_along_x():
    poly = np.array([[0, 0], [1, 0], [2, 0]], dtype=float)
    yaw = p2r.tangent_yaw_deg(poly)
    assert np.allclose(yaw, 0.0)
    poly = np.array([[0, 0], [0, 1]], dtype=float)
    yaw = p2r.tangent_yaw_deg(poly)
    assert np.allclose(yaw, 90.0)


def test_load_path_accepts_both_shapes(tmp_path):
    pts = [(0, 0), (2, 0), (4, 0)]
    for shape in ("poses", "flat"):
        p = write_path(tmp_path, pts, shape)
        arr = p2r.load_path(p)
        assert arr.shape == (3, 2)
        assert np.allclose(arr[-1], [4, 0])


def test_build_route_doc_schema(tmp_path):
    p = write_path(tmp_path, [(0, 0), (1, 0), (2, 0)])
    out_route = str(tmp_path / "route.json")
    rc = p2r.main([p, "--out-route", out_route, "--step", "0.5"])
    assert rc is None
    route = json.load(open(out_route))
    assert route["frame"] == "map"
    assert route["reference_point"] == "chair_centre"
    assert route["body_frame_profile"] == "builtin"
    assert route["chair_centre_in_body_xyz"] == [-0.5, -0.2, 0.0]
    assert route["count"] == len(route["waypoints"])
    assert route["count"] >= 4
    wp0 = route["waypoints"][0]
    assert set(wp0.keys()) == {"x", "y", "z", "yaw_deg"}
    assert abs(route["path_length_m"] - 2.0) < 1e-6


def test_ground_height_from_costmap(tmp_path):
    dense = np.array([[1.0, 1.0], [2.0, 2.0]])
    npz = str(tmp_path / "cm.npz")
    ground = np.full((20, 20), -0.5)
    ground[5, 5] = 0.3
    np.savez(npz, cell=0.15, min_x=0.0, min_y=0.0, ground=ground)
    z = p2r.sample_ground_height(dense, npz)
    assert z.shape == (2,)
    assert np.isfinite(z).all()


def test_band_generation_requires_pcd(tmp_path):
    p = write_path(tmp_path, [(0, 0), (1, 0)])
    out_route = str(tmp_path / "route.json")
    with pytest.raises(SystemExit):
        p2r.main([p, "--out-route", out_route, "--out-band-prefix",
                  str(tmp_path / "band")])
