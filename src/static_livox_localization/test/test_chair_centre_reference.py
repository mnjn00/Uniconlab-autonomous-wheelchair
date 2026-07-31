"""The route is a path of the sensor, not of the chair, until it is moved.

FAST-LIO reports the pose of its own body frame, and that frame sits wherever
the lidar was bolted on - here, the front of the LEFT armrest. Every geometry
constant downstream then treats that point as the middle of the chair and lays
the chair out symmetrically around it: CHAIR_HALF_WIDTH either side in the
band, |y| < 0.40 for the rider box, a forward cone centred on x.

Measured from the 2026-07-27 route recording itself, that point is 0.517 m
forward and 0.173 m LEFT of the centre the chair actually turns about. Both
spin-in-place bookends of that drive were fitted jointly for p = c + R(yaw)r,
215 poses over 371 deg and 798 deg, residual 22.5 mm; the 798 deg spin alone
fits to 7.5 mm. Four clean spins in the 0707/0725 mapping runs bracket the
same value, so the mount did not move between mapping and recording.

The consequence is not symmetric. With the model centred 0.173 m left of the
chair, the left side is over-protected by that much and the RIGHT side is
under-protected by the same: at a station whose right edge is a fall hazard,
the band lets the chair sit where the code believes the wheel is 0.10 m clear
while it is really 0.065 m past the edge.

These cases pin the correction and, just as importantly, that a route
expressed about one point cannot be driven as if it were about the other.
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import body_frame  # noqa: E402


def test_the_measured_offset_is_the_operators_measurement():
    """Guards the constant against a silent edit: it is a measurement, and
    changing it changes where the chair believes its wheels are.

    0.500/0.200 measured physically on 2026-07-31, superseding the
    0.517/0.173 fitted from six in-place rotations. The two agree to within
    that fit's own +-0.03 m between-spin spread, so this is a sharper
    instrument rather than a contradiction - a tape reaches the wheel axle
    directly, where the fit had to infer it from how the chair turned."""
    forward, left, up = body_frame.CHAIR_CENTRE_IN_BODY_XYZ

    assert forward == pytest.approx(-0.500, abs=5e-4)
    assert left == pytest.approx(-0.200, abs=5e-4)
    assert up == 0.0


def test_the_correction_moves_the_pose_to_the_chair_centre():
    """Facing along +x, the chair centre is 0.500 m behind the sensor and
    0.200 m to its right."""
    correction = body_frame.reference_correction(
        body_frame.REFERENCE_CHAIR_CENTRE)
    pose = np.eye(4)
    pose[:3, 3] = (10.0, 5.0, 0.0)

    centred = pose @ correction

    assert centred[0, 3] == pytest.approx(10.0 - 0.500)
    assert centred[1, 3] == pytest.approx(5.0 - 0.200)


def test_the_correction_rotates_with_the_chair():
    """Heading north, 'right of the sensor' is +x in the map, not -y. A
    correction applied in the map frame instead of the body frame would put
    the chair centre in the wrong place everywhere but due east."""
    yaw = math.pi / 2
    pose = np.eye(4)
    pose[:3, :3] = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    pose[:3, 3] = (0.0, 0.0, 0.0)

    centred = pose @ body_frame.reference_correction(
        body_frame.REFERENCE_CHAIR_CENTRE)

    # forward is +y here, left is -x
    assert centred[0, 3] == pytest.approx(0.200)
    assert centred[1, 3] == pytest.approx(-0.500)


def test_a_body_referenced_route_is_left_exactly_as_it_was():
    """Inert on the old reference, so a route that was never re-expressed
    keeps the behaviour it was validated with."""
    correction = body_frame.reference_correction(body_frame.REFERENCE_BODY)

    assert np.allclose(correction, np.eye(4))


def test_an_unknown_reference_is_refused_rather_than_assumed():
    """Guessing here silently shifts the whole route sideways."""
    with pytest.raises(ValueError):
        body_frame.reference_correction("middle_of_the_seat")


def test_the_offset_is_large_enough_to_matter_against_the_band_margin():
    """Sanity on scale rather than on the number: the correction has to be
    worth making. BAND_MARGIN is 0.10 m and the lateral error is larger, so
    ignoring it consumes the entire margin on the right."""
    _, left, _ = body_frame.CHAIR_CENTRE_IN_BODY_XYZ

    assert abs(left) > 0.10


def test_the_route_declares_which_point_it_is_about():
    """A chair-centred route driven as a body-centred one, or the reverse,
    is displaced 0.173 m sideways with nothing to reveal it. The route file
    has to say, and the follower has to read it."""
    follower = (ROOT / "scripts" / "waypoint_follower.py").read_text(
        encoding="utf-8")

    assert "reference_point" in follower
    assert "reference_correction" in follower


def test_the_rider_box_is_centred_on_the_rider_not_on_the_sensor():
    """The rider sits on the chair, so their body is centred 0.173 m right of
    the sensor. A box centred on the sensor leaves that much of their right
    side outside it, and what falls outside is reported as an obstacle - the
    rider's own body braking the chair."""
    sys.path.insert(0, str(ROOT / "scripts"))
    from motion_safety import filter_obstacle_points

    # a point on the rider's right flank, 0.50 m right of the sensor
    cloud = np.array([[0.0, -0.50, -0.10]], dtype=float)
    common = dict(sensor_height_m=0.30, min_height_m=0.15, max_height_m=2.4,
                  self_x_min_m=-1.0, self_x_max_m=0.55, self_half_width_m=0.40)

    centred_on_sensor = filter_obstacle_points(cloud, **common)
    centred_on_rider = filter_obstacle_points(
        cloud, self_y_centre_m=-0.173, **common)

    assert len(centred_on_sensor) == 1, "the rider used to read as an obstacle"
    assert len(centred_on_rider) == 0, "and must not any more"


def test_recentring_the_rider_box_does_not_swallow_a_real_obstacle():
    """The box moved, it did not grow: something 0.50 m to the LEFT is still
    outside it and still stops the chair."""
    sys.path.insert(0, str(ROOT / "scripts"))
    from motion_safety import filter_obstacle_points

    cloud = np.array([[0.0, 0.50, -0.10]], dtype=float)

    kept = filter_obstacle_points(
        cloud, sensor_height_m=0.30, min_height_m=0.15, max_height_m=2.4,
        self_x_min_m=-1.0, self_x_max_m=0.55, self_half_width_m=0.40,
        self_y_centre_m=-0.173)

    assert len(kept) == 1


def test_both_consumers_pass_the_measured_lateral_centre():
    """A re-centred filter that nobody calls with the offset is no fix."""
    for name in ("safety_gate.py", "obstacle_clusters.py"):
        text = (ROOT / "scripts" / name).read_text(encoding="utf-8")
        assert "CHAIR_CENTRE_IN_BODY_XYZ" in text, name


@pytest.mark.parametrize("name", [
    "20260727_new_route_waypoints.json",
    "20260727_chair_centred_waypoints.json",
])
def test_shipped_routes_declare_a_reference_point(name):
    path = ROOT.parents[1] / "routes" / name
    if not path.exists():
        pytest.skip("%s is not shipped" % name)
    route = json.loads(path.read_text(encoding="utf-8"))

    assert route.get("reference_point") in (
        body_frame.REFERENCE_BODY, body_frame.REFERENCE_CHAIR_CENTRE)
