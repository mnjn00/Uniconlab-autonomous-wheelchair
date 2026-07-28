"""Numeric tests for the body->lidar correction used by the motion nodes.

Unlike the surface tests beside them, these EXECUTE the code. The rest of
this suite asserts that substrings appear in the source, which cannot catch
a sign error or a stale constant - and a sign error here rotates the
obstacle corridor away from the direction the chair is travelling.

body_frame.py is deliberately ROS-free so this runs without a ROS install.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import body_frame  # noqa: E402

from body_frame import (  # noqa: E402
    LIDAR_IN_BODY_XYZ,
    LIDAR_IN_BODY_XYZ_BUILTIN,
    LIDAR_TO_BODY_ROTATION,
    LIDAR_TO_BODY_ROTATION_BUILTIN,
    body_to_lidar,
    lidar_extrinsics,
)

SENSOR_HEIGHT_M = 0.30


def _to_body(lidar_pt, xyz, rotation):
    """Forward transform, i.e. what FAST-LIO's extrinsic does."""
    return (
        np.asarray([lidar_pt], dtype=np.float64)
        @ np.asarray(rotation, dtype=np.float64).T
        + np.asarray(xyz, dtype=np.float64)
    )


def test_round_trip_recovers_the_lidar_frame_exactly():
    lidar_pt = [3.0, 0.0, -SENSOR_HEIGHT_M]
    body = _to_body(
        lidar_pt, LIDAR_IN_BODY_XYZ, LIDAR_TO_BODY_ROTATION)
    got = body_to_lidar(body)
    assert np.allclose(got, np.array([lidar_pt]), atol=1e-9), got


def test_correction_removes_the_yaw_skew_not_just_the_offset():
    """A translation-only fix would leave the 2.8 deg corridor skew, which
    is the part that displaces the corridor by 0.17 m at 3.4 m."""
    body = _to_body(
        [10.0, 0.0, 0.0], LIDAR_IN_BODY_XYZ, LIDAR_TO_BODY_ROTATION)
    got = body_to_lidar(body)
    assert abs(body[0, 1] - LIDAR_IN_BODY_XYZ[1]) > 0.4
    assert np.allclose(got, np.array([[10.0, 0.0, 0.0]]), atol=1e-9)


def test_ground_lands_at_minus_sensor_height_after_correction():
    """The classifier computes rel = z + sensor_height, so the ground must
    read exactly -sensor_height once the frame is corrected."""
    body = _to_body(
        [2.0, 0.5, -SENSOR_HEIGHT_M],
        LIDAR_IN_BODY_XYZ,
        LIDAR_TO_BODY_ROTATION,
    )
    got = body_to_lidar(body)
    assert abs(got[0, 2] + SENSOR_HEIGHT_M) < 1e-9


def test_forward_range_is_measured_from_the_lidar_not_the_body():
    """Uncorrected, an obstacle reads 14.5 cm farther than it is, which
    shortens every speed-scaled stop distance by the same amount."""
    body = _to_body(
        [1.00, 0.0, 0.0], LIDAR_IN_BODY_XYZ, LIDAR_TO_BODY_ROTATION)
    assert body[0, 0] - 1.00 > 0.14, "offset should be ~14.5 cm"
    assert abs(body_to_lidar(body)[0, 0] - 1.00) < 1e-9


def test_builtin_imu_constants_are_a_much_smaller_correction():
    """VN_IMU=0 must still be correctable, and by a smaller amount."""
    body = _to_body([3.0, 0.0, -SENSOR_HEIGHT_M],
                    LIDAR_IN_BODY_XYZ_BUILTIN,
                    LIDAR_TO_BODY_ROTATION_BUILTIN)
    got = body_to_lidar(
        body,
        LIDAR_IN_BODY_XYZ_BUILTIN,
        LIDAR_TO_BODY_ROTATION_BUILTIN,
    )
    assert np.allclose(got, np.array([[3.0, 0.0, -SENSOR_HEIGHT_M]]),
                       atol=1e-9)


def test_profile_selection_is_exact_and_unknown_values_fail_closed():
    assert lidar_extrinsics("vn100") == (
        LIDAR_IN_BODY_XYZ, LIDAR_TO_BODY_ROTATION)
    assert lidar_extrinsics("builtin") == (
        LIDAR_IN_BODY_XYZ_BUILTIN, LIDAR_TO_BODY_ROTATION_BUILTIN)
    try:
        lidar_extrinsics("vn-100")
    except ValueError:
        pass
    else:
        raise AssertionError("unknown body-frame profile was accepted")


def test_empty_and_none_clouds_pass_through_untouched():
    assert body_to_lidar(None) is None
    empty = np.zeros((0, 3), dtype=np.float32)
    assert len(body_to_lidar(empty)) == 0


def test_transform_is_its_own_inverse_over_many_points():
    rng = np.random.default_rng(0)
    lidar_pts = rng.uniform(-20.0, 20.0, size=(500, 3))
    body = (
        lidar_pts @ np.asarray(LIDAR_TO_BODY_ROTATION).T
        + np.asarray(LIDAR_IN_BODY_XYZ)
    )
    assert np.allclose(body_to_lidar(body), lidar_pts, atol=1e-9)


def test_pose_correction_is_identity_when_the_frames_already_match():
    for profile in ("vn100", "builtin"):
        np.testing.assert_allclose(
            body_frame.pose_correction(profile, profile), np.eye(4),
            atol=1e-12)


def test_pose_correction_moves_the_origin_by_the_measured_offset():
    """The two body origins are 15.5 cm apart along the chair's forward axis
    and 2.80 deg apart in heading. Simulating the follower's own steering
    loop over the 2026-07-27 route, ignoring that costs 7 cm of mean
    cross-track against a 0.45 m kerb clearance budget."""
    C = body_frame.pose_correction("vn100", "builtin")
    np.testing.assert_allclose(C[:3, 3], [0.1548, 0.0089, 0.0241], atol=1e-3)
    yaw = np.degrees(np.arctan2(C[1, 0], C[0, 0]))
    np.testing.assert_allclose(yaw, 2.796, atol=1e-2)


def test_pose_correction_round_trips():
    a = body_frame.pose_correction("vn100", "builtin")
    b = body_frame.pose_correction("builtin", "vn100")
    np.testing.assert_allclose(a @ b, np.eye(4), atol=1e-12)


def test_every_shipped_route_declares_the_frame_it_was_captured_in():
    """An unlabelled route is of unknown provenance, and reading it in the
    wrong body frame spends kerb clearance silently instead of failing."""
    import json

    routes = sorted((ROOT.parents[1] / "routes").glob("*waypoints.json"))
    assert routes
    for path in routes:
        data = json.load(open(path))
        assert "body_frame_profile" in data, path.name
        body_frame.lidar_extrinsics(data["body_frame_profile"])
