import sys
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import independent_route as ir


def test_planner_finds_bilateral_safe_route_on_curved_corridor():
    cell = 0.2
    shape = (90, 150)
    rows, cols = np.indices(shape)
    hard = np.ones(shape, dtype=bool)
    curve_cols = np.arange(10, 140)
    curve_rows = np.rint(
        45.0 + 16.0 * np.sin((curve_cols - 10.0) / 130.0 * np.pi)
    ).astype(int)
    for row, col in zip(curve_rows, curve_cols):
        hard |= False
        hard[(rows - row) ** 2 + (cols - col) ** 2 <= 7**2] = False
    dem = {
        "cell": cell,
        "min_x": 0.0,
        "min_y": 0.0,
        "ground": np.zeros(shape, dtype=np.float32),
        "known": np.logical_not(hard),
        "hard": hard,
        "slope": np.zeros(shape, dtype=np.float32),
    }
    start = ((curve_cols[0] + 0.5) * cell,
             (curve_rows[0] + 0.5) * cell)
    goal = ((curve_cols[-1] + 0.5) * cell,
            (curve_rows[-1] + 0.5) * cell)

    result = ir.plan_and_audit_dem(
        dem,
        start_xy=start,
        goal_xy=goal,
        config=ir.PlannerConfig(clearance_m=0.45),
    )

    assert result["status"] == "APPROVED"
    assert result["audit"]["bilateral_station_violations"] == 0
    assert result["audit"]["continuous_clearance_violations"] == 0
    assert result["audit"]["minimum_clearance_m"] >= 0.45
    assert len(result["route"]["waypoints"]) == len(
        result["band"]["stations"])


def test_cached_dem_cli_blocks_disconnected_corridor_without_route(
    tmp_path,
    capsys,
):
    shape = (40, 80)
    hard = np.zeros(shape, dtype=bool)
    hard[:, 38:42] = True
    cache = tmp_path / "blocked.npz"
    np.savez_compressed(
        cache,
        cell=0.2,
        min_x=0.0,
        min_y=0.0,
        ground=np.zeros(shape, dtype=np.float32),
        known=np.ones(shape, dtype=bool),
        hard=hard,
        slope=np.zeros(shape, dtype=np.float32),
    )
    prefix = tmp_path / "approved"

    exit_code = ir.main([
        str(cache),
        "--start", "2.1,4.1",
        "--goal", "14.1,4.1",
        "--out-prefix", str(prefix),
        "--required-clearance", "0.45",
    ])

    output = json.loads(capsys.readouterr().out)
    assert exit_code == 2
    assert output["status"] == "BLOCKED"
    assert output["maximum_connected_clearance_m"] < 0.45
    assert (tmp_path / "approved_audit.json").exists()
    assert not (tmp_path / "approved_route.json").exists()
    assert not (tmp_path / "approved_band.json").exists()


def test_load_route_local_ply_keeps_only_nearby_points(tmp_path):
    vertices = np.array([
        (0.0, 0.0, 0.0, 1.0),
        (1.0, 0.2, 0.1, 2.0),
        (10.0, 10.0, 0.0, 3.0),
    ], dtype=[
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
    ])
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex 3\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float intensity\n"
        "end_header\n"
    ).encode()
    path = tmp_path / "map.ply"
    path.write_bytes(header + vertices.tobytes())

    local = ir.load_route_local_ply(
        path,
        route_xy=np.array([[0.0, 0.0], [2.0, 0.0]]),
        radius_m=1.1,
        chunk_size=2,
    )

    assert local.shape == (2, 4)
    np.testing.assert_allclose(local[:, :3], [
        [0.0, 0.0, 0.0],
        [1.0, 0.2, 0.1],
    ])


def test_start_clearance_removes_only_current_chair_footprint():
    obstacle = np.ones((30, 30), dtype=bool)
    cleared = ir.clear_start_footprint(
        obstacle,
        start_xy=(2.0, 2.0),
        min_x=0.0,
        min_y=0.0,
        cell=0.2,
        radius_m=0.8,
    )

    assert not cleared[10, 10]
    assert cleared[10, 20]
    assert obstacle.all()


def test_hazard_masks_refuse_unknown_body_step_and_steep_cells():
    shape = (40, 60)
    known = np.ones(shape, dtype=bool)
    body = np.zeros(shape, dtype=np.int32)
    step = np.zeros(shape, dtype=np.float32)
    slope = np.zeros(shape, dtype=np.float32)
    body[20, 20] = 48
    step[20, 30] = 0.10
    slope[20, 40] = 14.01
    known[20, 50] = False

    masks = ir.configuration_space(
        known=known,
        body_count=body,
        step_m=step,
        slope_deg=slope,
        start_xy=(1.0, 1.0),
        min_x=0.0,
        min_y=0.0,
        cell=0.2,
        config=ir.PlannerConfig(),
    )

    for col in (20, 30, 40, 50):
        assert masks["hard"][20, col]


def test_excluded_hazard_disc_becomes_lethal():
    masks = {
        "hard": np.zeros((30, 30), dtype=bool),
        "clearance": np.ones((30, 30), dtype=float),
        "centre_free": np.ones((30, 30), dtype=bool),
    }

    updated = ir.apply_exclusions(
        masks,
        exclusions=[(2.0, 2.0, 0.8)],
        min_x=0.0,
        min_y=0.0,
        cell=0.2,
    )

    assert updated["hard"][10, 10]
    assert not updated["hard"][10, 20]
    assert not masks["hard"].any()


def test_band_feedback_rejects_each_side_independently():
    band = {
        "stations": [
            {
                "x": 0.0,
                "y": 0.0,
                "heading_deg": 0.0,
                "left_m": 0.8,
                "right_m": 0.1,
            },
            {
                "x": 1.0,
                "y": 0.0,
                "heading_deg": 0.0,
                "left_m": 0.5,
                "right_m": 0.5,
            },
        ]
    }

    violations = ir.band_clearance_violations(
        band,
        required_side_m=0.45,
    )

    assert violations == [
        {
            "station": 0,
            "side": "right",
            "edge_xy": [0.0, -0.1],
            "clearance_m": 0.1,
        }
    ]


def test_recenter_route_moves_to_bilateral_band_midpoint():
    route = {
        "waypoints": [
            {"x": float(x), "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
            for x in np.linspace(0.0, 4.0, 21)
        ]
    }
    band = {
        "stations": [
            {
                "x": float(x),
                "y": 0.0,
                "heading_deg": 0.0,
                "left_m": left,
                "right_m": right,
            }
            for x, left, right in (
                (0.0, 0.8, 0.8),
                (1.0, 0.8, 0.8),
                (2.0, 1.2, 0.4),
                (3.0, 0.8, 0.8),
                (4.0, 0.8, 0.8),
            )
        ]
    }

    corrected = ir.recenter_route_document(
        route,
        band,
        required_side_m=0.45,
        endpoint_guard=0,
    )

    points = np.array([
        [item["x"], item["y"]]
        for item in corrected["waypoints"]
    ])
    centre_index = int(np.argmin(np.abs(points[:, 0] - 2.0)))
    np.testing.assert_allclose(
        points[centre_index, 1],
        0.05,
        atol=0.03,
    )


def test_recenter_route_refuses_physically_narrow_band():
    route = {
        "waypoints": [
            {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
            {"x": 1.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
        ]
    }
    band = {
        "stations": [
            {
                "x": 0.5,
                "y": 0.0,
                "heading_deg": 0.0,
                "left_m": 0.4,
                "right_m": 0.4,
            }
        ]
    }

    with np.testing.assert_raises(RuntimeError):
        ir.recenter_route_document(
            route,
            band,
            required_side_m=0.45,
            endpoint_guard=0,
        )


def test_recenter_band_preserves_measured_edge_coordinates():
    band = {
        "frame": "map",
        "station_spacing_m": 1.0,
        "stations": [
            {
                "x": float(index),
                "y": 0.0,
                "heading_deg": 0.0,
                "left_m": left,
                "right_m": right,
                "left_kind": "drop",
                "right_kind": "drop",
            }
            for index, (left, right) in enumerate((
                (1.0, 1.0),
                (1.2, 0.3),
                (1.0, 1.0),
            ))
        ],
    }

    centred = ir.recenter_band_document(
        band,
        required_side_m=0.45,
        endpoint_guard=0,
        transition_stations=0,
    )

    old = band["stations"][1]
    new = centred["stations"][1]
    old_left_edge = np.array([old["x"], old["y"]]) + [0.0, old["left_m"]]
    old_right_edge = np.array([old["x"], old["y"]]) - [0.0, old["right_m"]]
    new_left_edge = np.array([new["x"], new["y"]]) + [0.0, new["left_m"]]
    new_right_edge = np.array([new["x"], new["y"]]) - [0.0, new["right_m"]]

    np.testing.assert_allclose(new_left_edge, old_left_edge)
    np.testing.assert_allclose(new_right_edge, old_right_edge)
    assert new["left_m"] >= 0.45
    assert new["right_m"] >= 0.45


def test_plan_uses_only_safe_component_without_route_field():
    shape = (70, 120)
    known = np.ones(shape, dtype=bool)
    body = np.zeros(shape, dtype=np.int32)
    step = np.zeros(shape, dtype=np.float32)
    slope = np.zeros(shape, dtype=np.float32)
    body[5:60, 55:60] = 80
    body[40:45, 55:60] = 0
    config = ir.PlannerConfig(clearance_m=0.3)
    masks = ir.configuration_space(
        known=known,
        body_count=body,
        step_m=step,
        slope_deg=slope,
        start_xy=(2.0, 4.0),
        min_x=0.0,
        min_y=0.0,
        cell=0.2,
        config=config,
    )

    path = ir.plan_safe_path(
        masks=masks,
        slope_deg=slope,
        start_xy=(2.0, 4.0),
        goal_xy=(20.0, 4.0),
        min_x=0.0,
        min_y=0.0,
        cell=0.2,
        config=config,
    )

    assert len(path) > 2
    assert path[:, 1].min() < 1.0 or path[:, 1].max() > 12.0
    rows = np.rint(path[:, 1] / 0.2 - 0.5).astype(int)
    cols = np.rint(path[:, 0] / 0.2 - 0.5).astype(int)
    assert not masks["hard"][rows, cols].any()


def test_plan_refuses_to_snap_disconnected_goal():
    shape = (40, 80)
    masks = {
        "centre_free": np.ones(shape, dtype=bool),
        "clearance": np.ones(shape, dtype=float),
    }
    masks["centre_free"][:, 38:42] = False
    slope = np.zeros(shape, dtype=float)

    with np.testing.assert_raises(RuntimeError):
        ir.plan_safe_path(
            masks=masks,
            slope_deg=slope,
            start_xy=(2.0, 4.0),
            goal_xy=(14.0, 4.0),
            min_x=0.0,
            min_y=0.0,
            cell=0.2,
            config=ir.PlannerConfig(),
        )


def test_corridor_mask_excludes_flat_ground_beyond_measured_curbs():
    band = {
        "stations": [
            {
                "x": float(x),
                "y": 2.0,
                "heading_deg": 0.0,
                "left_m": 0.9,
                "right_m": 0.7,
            }
            for x in np.arange(1.0, 7.1, 0.2)
        ]
    }

    mask = ir.curb_bounded_mask(
        band,
        shape=(30, 45),
        min_x=0.0,
        min_y=0.0,
        cell=0.2,
        required_side_m=0.45,
        maximum_offset_m=0.5,
    )

    assert mask[10, 20]
    assert not mask[16, 20]
    assert not mask[4, 20]


def test_plan_on_curb_bounded_mask_keeps_wheel_envelope_inside():
    band = {
        "stations": [
            {
                "x": float(x),
                "y": 2.0 + 0.4 * np.sin(x / 2.0),
                "heading_deg": 0.0,
                "left_m": 0.9,
                "right_m": 0.9,
            }
            for x in np.arange(1.0, 9.1, 0.2)
        ]
    }
    mask = ir.curb_bounded_mask(
        band,
        shape=(35, 55),
        min_x=0.0,
        min_y=0.0,
        cell=0.2,
        required_side_m=0.45,
        maximum_offset_m=0.5,
    )

    path = ir.plan_curb_bounded_path(
        mask,
        start_xy=(1.0, 2.2),
        goal_xy=(9.0, 1.6),
        min_x=0.0,
        min_y=0.0,
        cell=0.2,
    )

    audit = ir.audit_path_against_band(
        path,
        band,
        required_side_m=0.45,
        sample_spacing_m=0.1,
    )
    assert audit["status"] == "APPROVED"
    assert audit["wheel_envelope_violations"] == 0
    assert audit["minimum_left_clearance_m"] >= 0.45
    assert audit["minimum_right_clearance_m"] >= 0.45
    assert audit["minimum_wheel_boundary_margin_m"] >= 0.10
    assert audit["minimum_padded_footprint_boundary_margin_m"] >= 0.07
