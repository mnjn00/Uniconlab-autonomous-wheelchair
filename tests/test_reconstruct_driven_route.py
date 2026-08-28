import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import reconstruct_driven_route as rdr


def pose(x, y, yaw_deg, timestamp):
    yaw = np.radians(yaw_deg)
    return np.array([x, y, 0.0, yaw, timestamp], dtype=float)


def test_stitch_driven_poses_uses_heading_consistent_monotone_overlap():
    first = np.array([
        pose(0.0, 0.0, 0.0, 0.0),
        pose(1.0, 0.0, 0.0, 1.0),
        pose(2.0, 0.0, 0.0, 2.0),
        pose(3.0, 0.0, 0.0, 3.0),
        pose(4.0, 0.0, 0.0, 4.0),
    ])
    second = np.array([
        pose(2.05, 0.02, 180.0, 10.0),
        pose(3.02, 0.01, 0.5, 11.0),
        pose(4.01, 0.02, 0.5, 12.0),
        pose(5.0, 0.0, 0.0, 13.0),
        pose(6.0, 0.0, 0.0, 14.0),
    ])

    stitched, audit = rdr.stitch_driven_poses(
        first,
        second,
        start_xy=(0.0, 0.0),
        goal_xy=(6.0, 0.0),
        max_position_gap_m=0.1,
        max_heading_gap_deg=5.0,
        resample_m=0.2,
    )

    assert audit["first_stitch_index"] == 4
    assert audit["second_stitch_index"] == 2
    assert audit["stitch_position_gap_m"] < 0.1
    assert audit["stitch_heading_gap_deg"] < 5.0
    np.testing.assert_allclose(stitched[0, :2], [0.0, 0.0])
    np.testing.assert_allclose(stitched[-1, :2], [6.0, 0.0], atol=0.1)
    assert np.linalg.norm(np.diff(stitched[:, :2], axis=0), axis=1).max() <= 0.21


def test_body_pose_to_chair_centre_rotates_offset():
    body = np.array([
        pose(10.0, 20.0, 90.0, 0.0),
    ])

    chair = rdr.body_to_chair_centre(
        body,
        chair_centre_in_body_xy=(-0.5, -0.2),
    )

    np.testing.assert_allclose(chair[0, :2], [10.2, 19.5], atol=1e-9)


def test_default_stitch_rejects_nearby_opposite_heading_tail():
    first = np.array([
        pose(0.0, 0.0, 0.0, 0.0),
        pose(1.0, 0.0, 0.0, 1.0),
        pose(2.0, 0.0, 0.0, 2.0),
        pose(3.0, 0.0, 0.0, 3.0),
        pose(4.0, 0.0, 14.0, 4.0),
    ])
    second = np.array([
        pose(3.02, 0.01, 0.5, 10.0),
        pose(4.02, 0.01, 0.0, 11.0),
        pose(5.0, 0.0, 0.0, 12.0),
    ])

    _, audit = rdr.stitch_driven_poses(
        first,
        second,
        start_xy=(0.0, 0.0),
        goal_xy=(5.0, 0.0),
    )

    assert audit["first_stitch_index"] == 3
    assert audit["stitch_heading_gap_deg"] < 5.0


def test_erase_spatial_loops_removes_driven_out_and_back_excursion():
    driven = np.array([
        pose(0.0, 0.0, 0.0, 0.0),
        pose(1.0, 0.0, 0.0, 1.0),
        pose(2.0, 0.0, 0.0, 2.0),
        pose(3.0, 0.0, 0.0, 3.0),
        pose(2.0, 0.02, 180.0, 4.0),
        pose(1.02, 0.01, 180.0, 5.0),
        pose(2.0, -0.02, 0.0, 6.0),
        pose(3.0, -0.02, 0.0, 7.0),
        pose(4.0, 0.0, 0.0, 8.0),
    ])

    simple, removed = rdr.erase_spatial_loops(
        driven,
        revisit_radius_m=0.1,
        minimum_index_separation=2,
    )

    assert removed >= 3
    np.testing.assert_allclose(simple[0, :2], [0.0, 0.0])
    np.testing.assert_allclose(simple[-1, :2], [4.0, 0.0])
    assert np.linalg.norm(np.diff(simple[:, :2], axis=0), axis=1).sum() < 5.0


def test_smooth_driven_path_limits_turn_without_large_lateral_shift():
    driven = np.array([
        pose(0.0, 0.0, 0.0, 0.0),
        pose(1.0, 0.0, 0.0, 1.0),
        pose(2.0, 0.0, 0.0, 2.0),
        pose(2.2, 0.2, 45.0, 3.0),
        pose(2.4, 1.0, 90.0, 4.0),
        pose(2.4, 2.0, 90.0, 5.0),
    ])

    smooth, audit = rdr.smooth_driven_path(
        driven,
        sample_m=0.1,
        sigma_m=0.5,
        output_spacing_m=0.2,
    )

    heading = np.unwrap(np.arctan2(
        np.gradient(smooth[:, 1]),
        np.gradient(smooth[:, 0]),
    ))
    assert np.degrees(np.abs(np.diff(heading))).max() < 15.0
    assert audit["maximum_smoothing_shift_m"] < 0.3
    np.testing.assert_allclose(smooth[0, :2], driven[0, :2])
    np.testing.assert_allclose(smooth[-1, :2], driven[-1, :2])


def test_shortest_driven_path_uses_safe_forward_revisit_edges():
    outbound = [
        pose(0.0, 0.0, 0.0, 0.0),
        pose(1.0, 0.0, 0.0, 1.0),
        pose(2.0, 0.0, 0.0, 2.0),
        pose(3.0, 0.0, 0.0, 3.0),
    ]
    loop = [
        pose(3.0, 1.0, 90.0, 4.0),
        pose(2.0, 1.0, 180.0, 5.0),
        pose(2.05, 0.02, 0.0, 6.0),
    ]
    tail = [
        pose(3.0, 0.0, 0.0, 7.0),
        pose(4.0, 0.0, 0.0, 8.0),
    ]

    simple, removed = rdr.shortest_driven_path(
        np.array(outbound + loop + tail),
        revisit_radius_m=0.1,
        minimum_index_separation=2,
    )

    assert removed >= 2
    assert np.linalg.norm(np.diff(simple[:, :2], axis=0), axis=1).sum() < 5.0
    np.testing.assert_allclose(simple[0, :2], [0.0, 0.0])
    np.testing.assert_allclose(simple[-1, :2], [4.0, 0.0])
