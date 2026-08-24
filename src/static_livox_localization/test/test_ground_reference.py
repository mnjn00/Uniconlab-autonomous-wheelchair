"""Obstacle heights are measured from the ground, not from the chair.

Reproduces the 2026-08-23 crest stop. Cresting the hill at 0.8 m/s the
chair was still nose-up while the road ahead had levelled; the flat
body-frame height band cut through the road, and the road at 3.30 m stood
0.33 m proud of it - 5.7 degrees over that range. safety_gate stopped for
it twice.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from cloud_points import COLLISION_MAX_HEIGHT_M, COLLISION_MIN_HEIGHT_M
from motion_safety import filter_obstacle_points, ground_reference

SENSOR_HEIGHT_M = 0.725
# The attitude measured at the crest, from the object it invented: 0.33 m
# of apparent height at 3.30 m of range.
CREST_NOSE_UP_DEG = math.degrees(math.asin(0.33 / 3.30))


def ground(x_from=0.6, x_to=10.0, slope=0.0, crest_at=None):
    """A sheet of returns off the road, in world coordinates."""
    xs = np.arange(x_from, x_to, 0.1)
    ys = np.arange(-3.0, 3.01, 0.2)
    out = []
    for x in xs:
        rise = x * slope
        if crest_at is not None and x > crest_at:
            rise = crest_at * slope
        for y in ys:
            out.append((x, y, rise - SENSOR_HEIGHT_M))
    return np.array(out)


def box(centre_x, centre_y, height_m, base_m=0.0, width=0.6, depth=0.6):
    xs = np.arange(centre_x - depth / 2, centre_x + depth / 2, 0.05)
    ys = np.arange(centre_y - width / 2, centre_y + width / 2, 0.05)
    zs = np.arange(base_m, base_m + height_m, 0.05)
    return np.array([(x, y, z - SENSOR_HEIGHT_M)
                     for x in xs for y in ys for z in zs])


def nose_up(points, degrees):
    """The same scene as the chair sees it when pitched nose-up."""
    a = math.radians(degrees)
    rotation = np.array([[math.cos(a), 0.0, -math.sin(a)],
                         [0.0, 1.0, 0.0],
                         [math.sin(a), 0.0, math.cos(a)]])
    return points @ rotation.T


def obstacles(cloud, ground_referenced=True, **kwargs):
    """Note the default: these tests opt IN.

    filter_obstacle_points ships with the reference off - see the constant
    block in motion_safety for why - so a test that wants the mechanism has
    to ask for it, and the tests at the bottom of this file check what the
    shipped default actually does.
    """
    return filter_obstacle_points(
        cloud,
        sensor_height_m=SENSOR_HEIGHT_M,
        min_height_m=COLLISION_MIN_HEIGHT_M,
        max_height_m=COLLISION_MAX_HEIGHT_M,
        self_x_min_m=-1.0, self_x_max_m=0.55,
        self_half_width_m=0.40,
        ground_referenced=ground_referenced, **kwargs)


def in_front(points, near=1.0, far=6.0):
    if not len(points):
        return points
    keep = ((points[:, 0] > near) & (points[:, 0] < far) &
            (np.abs(points[:, 1]) < 1.0))
    return points[keep]


def test_level_chair_on_flat_road_sees_nothing():
    assert len(in_front(obstacles(ground()))) == 0


def test_the_crest_attitude_no_longer_invents_a_road():
    """The stop this file is named for."""
    cloud = nose_up(ground(), CREST_NOSE_UP_DEG)
    assert len(in_front(obstacles(cloud))) == 0


def test_the_crest_attitude_did_invent_one_before():
    """Red for the test above.

    Without the ground reference the same cloud turns the road into an
    obstacle, and it does so at the range the wedge between the chair plane
    and the road opens past the threshold: min_height / sin(pitch), which
    is 1.50 m here. Asserting the range and not merely the count is what
    makes this a statement about the mechanism rather than about a count.
    """
    cloud = nose_up(ground(), CREST_NOSE_UP_DEG)
    found = in_front(obstacles(cloud, ground_referenced=False))
    assert len(found) > 0
    onset = COLLISION_MIN_HEIGHT_M / math.sin(math.radians(CREST_NOSE_UP_DEG))
    assert abs(float(found[:, 0].min()) - onset) < 0.3, (
        "road first read as obstacle at %.2f m, wedge predicts %.2f m"
        % (float(found[:, 0].min()), onset))


def test_a_real_obstacle_on_flat_road_survives():
    cloud = np.vstack([ground(), box(3.0, 0.0, height_m=0.5)])
    found = in_front(obstacles(cloud))
    assert len(found) > 0
    assert abs(float(np.median(found[:, 0])) - 3.0) < 0.4


def test_a_real_obstacle_survives_the_crest_attitude_too():
    """The fix must not buy its quiet by going blind."""
    cloud = nose_up(np.vstack([ground(), box(3.0, 0.0, height_m=0.5)]),
                    CREST_NOSE_UP_DEG)
    assert len(in_front(obstacles(cloud))) > 0


def test_a_constant_slope_is_road_and_what_stands_on_it_is_not():
    slope = math.tan(math.radians(8.0))
    bare = ground(slope=slope)
    assert len(in_front(obstacles(bare))) == 0
    standing = np.vstack([bare, box(3.0, 0.0, height_m=0.5,
                                    base_m=3.0 * slope)])
    assert len(in_front(obstacles(standing))) > 0


def test_a_crest_in_the_road_is_road():
    cloud = nose_up(ground(slope=math.tan(math.radians(6.0)), crest_at=3.0),
                    CREST_NOSE_UP_DEG)
    assert len(in_front(obstacles(cloud))) == 0


def test_a_wall_cannot_lift_the_reference_to_its_own_roofline():
    """The clamp earns its keep here: a bin that is nothing but obstacle
    would otherwise take the obstacle's height as the ground."""
    cloud = np.vstack([ground(x_to=3.0), box(3.2, 0.0, height_m=2.0,
                                             width=6.0, depth=0.4)])
    found = in_front(obstacles(cloud), near=2.5, far=4.0)
    assert len(found) > 0


def test_an_overhead_branch_is_still_ignored():
    cloud = np.vstack([ground(), box(3.0, 0.0, height_m=0.4, base_m=2.0)])
    assert len(in_front(obstacles(cloud))) == 0


def test_the_reference_follows_the_road_uphill():
    slope = math.tan(math.radians(8.0))
    cloud = ground(slope=slope)
    reference = ground_reference(cloud[:, :3], SENSOR_HEIGHT_M)
    far = cloud[:, 0] > 5.0
    assert float(np.median(reference[far])) > 0.4


def test_an_empty_cloud_is_not_an_error():
    assert len(obstacles(np.zeros((0, 3)))) == 0


# What the field said an hour after the reference went in, at station ~1218
# on a -2.2 degree descent. The reference climbed +0.1625 per range bin -
# the slope clamp exactly, not the road - because the near field held no
# ground returns to anchor it, and by 4 m it read +0.90 m.
FIELD_REFERENCE_AT_2M = 0.84
FIELD_BRANCH_HEIGHTS_M = [2.14, 2.31, 2.25, 2.29, 2.30, 2.33, 2.10, 2.18]


def test_the_shipped_default_is_the_flat_plane():
    """Off, deliberately. A missed obstacle is not the same kind of
    mistake as an unnecessary stop, and a saturating reference makes the
    first kind."""
    cloud = nose_up(ground(), CREST_NOSE_UP_DEG)
    assert len(in_front(obstacles(cloud, ground_referenced=False))) > 0
    assert len(in_front(obstacles(cloud, ground_referenced=None))) > 0


def test_overhead_branches_stay_overhead_under_the_default():
    """The regression that took the reference back out.

    Lifting the reference lifts the ceiling with it: at +0.84 m the band
    runs 0.99 to 2.34 m, and branches the flat plane had always ignored
    became obstacles in the forward corridor. Thirteen of them stopped the
    chair.
    """
    for height in FIELD_BRANCH_HEIGHTS_M:
        assert height > COLLISION_MAX_HEIGHT_M, height
        lifted_ceiling = FIELD_REFERENCE_AT_2M + COLLISION_MAX_HEIGHT_M
        assert height < lifted_ceiling, (
            "%.2f m would have been inside the lifted band" % height)
    cloud = np.vstack([ground(), box(2.0, 0.0, height_m=0.3, base_m=2.1)])
    assert len(in_front(obstacles(cloud, ground_referenced=False))) == 0


def test_a_reference_that_climbs_would_hide_a_real_obstacle():
    """The serious half, stated as an assertion rather than a comment.

    A 0.5 m object at 4 m sits under reference + min_height once the
    reference has walked to 0.90, so it reads as road.
    """
    climbed = 0.90
    obstacle_top = 0.5
    assert obstacle_top < climbed + COLLISION_MIN_HEIGHT_M, (
        "an obstacle this size is invisible under a reference that high")


# What 40 real clouds from aejimum_to_gongsen.bag said about the estimator,
# outdoors, this sensor on this chair. Per-frame peak reference:
#
#     with the sheet test    median 0.63   90th 0.74   max 0.79
#     without it             median 0.65   90th 0.91   max 0.95
FIELD_PEAK_WITH_SHEET_TEST_M = 0.63
FIELD_PEAK_WITHOUT_SHEET_TEST_M = 0.65


def test_the_ceiling_does_not_move_with_the_reference():
    """The structural half of the fix, and the one worth keeping.

    Whatever the reference does - right, wrong, saturated - it cannot admit
    something overhead, because the ceiling answers a different question.
    How high the rider is does not change because the road tilted.
    """
    steep = nose_up(ground(), 12.0)
    cloud = np.vstack([steep, nose_up(box(2.5, 0.0, height_m=0.4,
                                          base_m=2.1), 12.0)])
    found = obstacles(cloud, ground_referenced=True)
    overhead = in_front(found, near=1.5, far=4.0)
    assert len(overhead) == 0, (
        "the reference lifted the ceiling and let an overhead object in")


def test_the_sheet_test_did_not_rescue_the_estimator_in_the_field():
    """Kept as a number so the next attempt starts from the measurement.

    The premise - lowest returns in a range bin are the ground - does not
    hold for a MID360 on the armrest: there is not enough near-field road
    in a scan for a low percentile to land on it. Filtering by density
    moved the peak reference by 0.02 m.
    """
    improvement = (FIELD_PEAK_WITHOUT_SHEET_TEST_M
                   - FIELD_PEAK_WITH_SHEET_TEST_M)
    assert improvement < 0.05, (
        "if a later change makes the sheet test actually discriminate, "
        "this number is the baseline it has to beat")
    assert FIELD_PEAK_WITH_SHEET_TEST_M > 0.5, (
        "0.63 m of reference on flat outdoor ground is the premise "
        "failing, not the constants being wrong")
