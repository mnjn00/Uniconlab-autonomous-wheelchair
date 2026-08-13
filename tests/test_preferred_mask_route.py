"""Preferred-route planning inside an authoritative drivable mask."""

import sys
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
try:
    import build_preferred_mask_route as route_builder
finally:
    sys.path.pop(0)


def test_preferred_cells_win_when_both_routes_are_drivable():
    # Given: two equally short lanes, with only the upper lane preferred.
    drivable = np.zeros((9, 11), dtype=bool)
    drivable[2:7, 1:10] = True
    preferred = np.zeros_like(drivable)
    preferred[3, 1:10] = True

    # When: a route is planned between the lane endpoints.
    path = route_builder.plan_preferred_path(
        drivable, preferred, (3, 1), (3, 9), resolution_m=0.1
    )

    # Then: every path cell stays on the preferred lane.
    assert drivable[path[:, 0], path[:, 1]].all()
    assert preferred[path[:, 0], path[:, 1]].all()


def test_boundary_cost_keeps_an_unbiased_route_in_the_interior():
    # Given: an open area with no preferred pixels.
    drivable = np.zeros((9, 11), dtype=bool)
    drivable[1:8, 1:10] = True
    preferred = np.zeros_like(drivable)

    # When: the endpoints are on the interior centre row.
    path = route_builder.plan_preferred_path(
        drivable, preferred, (4, 1), (4, 9), resolution_m=0.1
    )

    # Then: the route never buys a shorter-looking excursion along an edge.
    assert np.all(path[:, 0] == 4)
    assert drivable[path[:, 0], path[:, 1]].all()


def test_outside_drivable_mask_is_never_bought_by_preference():
    # Given: a preferred shortcut that crosses forbidden cells.
    drivable = np.zeros((9, 11), dtype=bool)
    drivable[1:8, 1:4] = True
    drivable[1:8, 7:10] = True
    drivable[6:8, 3:8] = True
    preferred = np.zeros_like(drivable)
    preferred[4, 1:10] = True

    # When: preference points straight through the forbidden gap.
    path = route_builder.plan_preferred_path(
        drivable, preferred, (4, 1), (4, 9), resolution_m=0.1
    )

    # Then: the hard mask wins and the route takes the legal detour.
    assert drivable[path[:, 0], path[:, 1]].all()
    assert np.any(path[:, 0] >= 6)


def test_runtime_band_uses_the_same_hard_mask_boundary():
    # Given: a straight route through an asymmetric drivable region.
    drivable = np.zeros((11, 15), dtype=bool)
    drivable[2:9, 1:14] = True
    path_rc = np.column_stack(
        [np.full(11, 5, dtype=int), np.arange(2, 13, dtype=int)]
    )

    # When: runtime safety-band stations are generated from that region.
    stations = route_builder.build_mask_band_stations(
        path_rc, drivable, resolution_m=0.1, origin_xy=(0.0, 0.0)
    )

    # Then: the measured reaches point to the mask edge, not a fixed width.
    assert len(stations) == len(path_rc)
    assert all(station["left_m"] > 0.2 for station in stations)
    assert all(station["right_m"] > 0.2 for station in stations)
    assert all("left_corridor_m" not in station for station in stations)


def test_runtime_band_preserves_nearest_seed_edge_semantics():
    path_rc = np.asarray([[2, 2], [2, 3], [2, 4]], dtype=int)
    drivable = np.ones((7, 7), dtype=bool)
    seed = [
        {
            "x": 2.0, "y": 4.0,
            "left_kind": "narrow", "right_kind": "open",
            "left_drop_m": 0.2, "right_drop_m": 0.0,
            "left_rise_m": 0.0, "right_rise_m": 0.1,
        },
        {
            "x": 4.0, "y": 4.0,
            "left_kind": "open", "right_kind": "narrow",
            "left_drop_m": 0.0, "right_drop_m": 0.3,
            "left_rise_m": 0.1, "right_rise_m": 0.0,
        },
    ]

    stations = route_builder.build_mask_band_stations(
        path_rc, drivable, 1.0, (0.0, 0.0), seed)

    assert stations[0]["left_kind"] == "narrow"
    assert stations[0]["left_drop_m"] == 0.2
    assert stations[-1]["right_kind"] == "narrow"
    assert stations[-1]["right_drop_m"] == 0.3


def test_smooth_path_stays_in_the_hard_mask():
    drivable = np.zeros((40, 60), dtype=bool)
    drivable[2:38, 2:58] = True
    points = np.asarray([
        [0.2, 1.5],
        [0.8, 1.5],
        [1.0, 1.4],
        [1.6, 1.4],
        [1.8, 1.3],
        [2.4, 1.3],
        [2.6, 1.2],
        [3.2, 1.2],
        [3.4, 1.1],
        [4.0, 1.1],
        [4.2, 1.0],
        [4.8, 1.0],
        [5.0, 0.9],
        [5.4, 0.9],
        [5.6, 0.8],
    ])
    smoothed = route_builder.smooth_path(
        points, drivable, resolution_m=0.1, origin_xy=(0.0, -2.0)
    )
    cells = [
        route_builder._world_to_rc(
            point, drivable.shape, 0.1, (0.0, -2.0)
        )
        for point in smoothed
    ]
    assert all(drivable[cell] for cell in cells)
    assert np.linalg.norm(smoothed - points, axis=1).max() <= 0.20


def test_smoothed_route_removes_raster_heading_stairs():
    route_path = ROOT / "routes" / "20260812_route_v6_v8_waypoints.json"
    route = json.loads(route_path.read_text(encoding="utf-8"))
    points = np.asarray(
        [[waypoint["x"], waypoint["y"]] for waypoint in route["waypoints"]]
    )
    heading = np.unwrap(np.arctan2(
        np.gradient(points[:, 1]), np.gradient(points[:, 0])
    ))
    heading_step_deg = np.degrees(np.abs(np.diff(heading)))
    assert heading_step_deg.max() <= 10.0
    assert np.percentile(heading_step_deg, 99) <= 6.0


def test_generator_rejects_short_forbidden_corner_clip():
    drivable = np.ones((5, 5), dtype=bool)
    drivable[2, 2] = False
    assert not route_builder._segment_is_drivable(
        np.array([0.0, 0.2504]),
        np.array([0.4, 0.1496]),
        drivable,
        0.1,
        (0.0, 0.0),
    )


def test_generator_rejects_forbidden_corner_touch():
    drivable = np.ones((5, 5), dtype=bool)
    drivable[1, 2] = False
    assert not route_builder._segment_is_drivable(
        np.array([0.1, 0.1]),
        np.array([0.3, 0.3]),
        drivable,
        0.1,
        (0.0, 0.0),
    )
