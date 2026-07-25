"""Numeric tests for the body->lidar correction used by the motion nodes.

Unlike the surface tests beside them, these EXECUTE the code. The rest of
this suite asserts that substrings appear in the source, which cannot catch
a sign error or a stale constant - and a sign error here rotates the
obstacle corridor away from the direction the chair is travelling.

body_frame.py is deliberately ROS-free so this runs without a ROS install.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from body_frame import (  # noqa: E402
    LIDAR_IN_BODY_XYZ, LIDAR_IN_BODY_XYZ_BUILTIN, LIDAR_IN_BODY_YAW_RAD,
    LIDAR_IN_BODY_YAW_RAD_BUILTIN, body_to_lidar)

SENSOR_HEIGHT_M = 0.30


def _to_body(lidar_pt, xyz, yaw):
    """Forward transform, i.e. what FAST-LIO's extrinsic does."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[
        c * lidar_pt[0] - s * lidar_pt[1] + xyz[0],
        s * lidar_pt[0] + c * lidar_pt[1] + xyz[1],
        lidar_pt[2] + xyz[2]]])


def test_round_trip_recovers_the_lidar_frame_exactly():
    lidar_pt = [3.0, 0.0, -SENSOR_HEIGHT_M]
    body = _to_body(lidar_pt, LIDAR_IN_BODY_XYZ, LIDAR_IN_BODY_YAW_RAD)
    got = body_to_lidar(body)
    assert np.allclose(got, np.array([lidar_pt]), atol=1e-9), got


def test_correction_removes_the_yaw_skew_not_just_the_offset():
    """A translation-only fix would leave the 2.8 deg corridor skew, which
    is the part that displaces the corridor by 0.17 m at 3.4 m."""
    assert abs(LIDAR_IN_BODY_YAW_RAD) > math.radians(1.0), \
        "yaw term must not be silently zero"
    body = np.array([[10.0 + LIDAR_IN_BODY_XYZ[0],
                      LIDAR_IN_BODY_XYZ[1], LIDAR_IN_BODY_XYZ[2]]])
    got = body_to_lidar(body)
    expected_y = -10.0 * math.sin(LIDAR_IN_BODY_YAW_RAD)
    assert abs(got[0, 1] - expected_y) < 1e-9
    assert abs(got[0, 1]) > 0.4, "skew at 10 m should exceed 0.4 m"


def test_ground_lands_at_minus_sensor_height_after_correction():
    """The classifier computes rel = z + sensor_height, so the ground must
    read exactly -sensor_height once the frame is corrected."""
    body = np.array([[2.0, 0.5,
                      -SENSOR_HEIGHT_M + LIDAR_IN_BODY_XYZ[2]]])
    got = body_to_lidar(body)
    assert abs(got[0, 2] + SENSOR_HEIGHT_M) < 1e-9


def test_forward_range_is_measured_from_the_lidar_not_the_body():
    """Uncorrected, an obstacle reads 14.5 cm farther than it is, which
    shortens every speed-scaled stop distance by the same amount."""
    body = _to_body([1.00, 0.0, 0.0], LIDAR_IN_BODY_XYZ,
                    LIDAR_IN_BODY_YAW_RAD)
    assert body[0, 0] - 1.00 > 0.14, "offset should be ~14.5 cm"
    assert abs(body_to_lidar(body)[0, 0] - 1.00) < 1e-9


def test_builtin_imu_constants_are_a_much_smaller_correction():
    """VN_IMU=0 must still be correctable, and by a smaller amount."""
    assert LIDAR_IN_BODY_YAW_RAD_BUILTIN == 0.0
    body = _to_body([3.0, 0.0, -SENSOR_HEIGHT_M],
                    LIDAR_IN_BODY_XYZ_BUILTIN, 0.0)
    got = body_to_lidar(body, LIDAR_IN_BODY_XYZ_BUILTIN, 0.0)
    assert np.allclose(got, np.array([[3.0, 0.0, -SENSOR_HEIGHT_M]]),
                       atol=1e-9)


def test_empty_and_none_clouds_pass_through_untouched():
    assert body_to_lidar(None) is None
    empty = np.zeros((0, 3), dtype=np.float32)
    assert len(body_to_lidar(empty)) == 0


def test_transform_is_its_own_inverse_over_many_points():
    rng = np.random.default_rng(0)
    lidar_pts = rng.uniform(-20.0, 20.0, size=(500, 3))
    c, s = math.cos(LIDAR_IN_BODY_YAW_RAD), math.sin(LIDAR_IN_BODY_YAW_RAD)
    body = np.column_stack([
        c * lidar_pts[:, 0] - s * lidar_pts[:, 1] + LIDAR_IN_BODY_XYZ[0],
        s * lidar_pts[:, 0] + c * lidar_pts[:, 1] + LIDAR_IN_BODY_XYZ[1],
        lidar_pts[:, 2] + LIDAR_IN_BODY_XYZ[2]])
    assert np.allclose(body_to_lidar(body), lidar_pts, atol=1e-9)
