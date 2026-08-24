"""ROS-independent motion and swept-footprint safety calculations."""

import math
from typing import NamedTuple, Optional, Sequence

import numpy as np


class MotionEstimate(NamedTuple):
    """Pose-derived motion; FAST-LIO's published twist is always zero."""

    valid: bool
    source_stamp_s: float
    receipt_stamp_s: float
    linear_speed_mps: float
    angular_speed_rps: float
    reason: str


# The current 1.0 m/s command cap stays below this 1.2 m/s plausibility
# ceiling. A reported step above it is a localization correction, not chair
# motion, and the planner has no way to tell the two apart.
POSE_STEP_LIMIT_MPS = 1.2
# Below this, arguing with the fix costs more than believing it.
POSE_STEP_FLOOR_M = 0.05


def clamp_pose_step(previous_xy, candidate_xy, elapsed_s,
                    limit_mps=POSE_STEP_LIMIT_MPS,
                    floor_m=POSE_STEP_FLOOR_M):
    """Let the reported pose move no faster than the chair itself can.

    On 2026-08-09 the map correction swung 0.35 m one way and 0.52 m back
    inside four seconds, twice, giving an apparent speed of 2.64 m/s on a
    chair whose limit is 0.6. Over the whole excursion the correction
    returned to within 0.074 m of where it started, so nothing had actually
    moved - but the follower saw the chair jump to the edge of a narrowing
    corridor and steered hard to recover, which is the one input that turns
    a localization glitch into a real excursion.

    Clamped rather than frozen, and rather than rejected. Freezing stops
    tracking the motion that IS real; rejecting outright cannot converge if
    the localizer has legitimately re-seeded. Clamping keeps following the
    chair at the fastest rate the chair could be moving, so a genuine
    re-seed is absorbed over a few cycles instead of arriving in one, and
    nothing has to decide which kind of jump it was.

    Returns ``(xy, withheld_m)`` - the second is how much of the step was
    not believed, which is worth reporting rather than swallowing.
    """
    candidate = np.asarray(candidate_xy, dtype=float)
    if previous_xy is None or not elapsed_s or float(elapsed_s) <= 0.0:
        return candidate, 0.0
    previous = np.asarray(previous_xy, dtype=float)
    delta = candidate - previous
    step = float(np.hypot(delta[0], delta[1]))
    allowed = max(float(floor_m), float(limit_mps) * float(elapsed_s))
    if step <= allowed:
        return candidate, 0.0
    return previous + delta * (allowed / step), step - allowed


class StoppingEnvelope(NamedTuple):
    speed_mps: float
    yaw_rate_rps: float
    reaction_s: float
    distance_m: float
    horizon_s: float


class MotionSafetyInputError(ValueError):
    pass


class _PoseSample(NamedTuple):
    stamp_s: float
    receipt_s: float
    x: float
    y: float
    yaw: float


def _invalid(
        reason: str,
        stamp_s: float = 0.0,
        receipt_s: float = 0.0) -> MotionEstimate:
    return MotionEstimate(False, stamp_s, receipt_s, 0.0, 0.0, reason)


def _wrapped_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PoseMotionEstimator:
    """Derive planar speed from pose deltas.

    The previous accepted pose is intentionally mutated on each update. A
    discontinuity replaces that baseline, allowing recovery on the next sample
    while the current sample remains fail-closed.
    """

    def __init__(
            self,
            expected_frame: str,
            expected_child: str,
            min_dt_s: float = 0.005,
            max_dt_s: float = 0.5,
            max_speed_mps: float = 3.0,
            max_yaw_rate_rps: float = 5.0) -> None:
        self.expected_frame = expected_frame
        self.expected_child = expected_child
        self.min_dt_s = min_dt_s
        self.max_dt_s = max_dt_s
        self.max_speed_mps = max_speed_mps
        self.max_yaw_rate_rps = max_yaw_rate_rps
        self._previous: Optional[_PoseSample] = None

    def update(
            self,
            source_stamp_s: float,
            receipt_stamp_s: float,
            frame_id: str,
            child_frame_id: str,
            x: float,
            y: float,
            quaternion_xyzw: Sequence[float]) -> MotionEstimate:
        values = (source_stamp_s, receipt_stamp_s, x, y) + \
            tuple(quaternion_xyzw)
        if not all(math.isfinite(value) for value in values) or \
                source_stamp_s <= 0.0 or receipt_stamp_s < 0.0:
            return _invalid("ODOM_INVALID", source_stamp_s, receipt_stamp_s)
        if frame_id != self.expected_frame or \
                child_frame_id != self.expected_child:
            return _invalid("ODOM_FRAME", source_stamp_s, receipt_stamp_s)

        qx, qy, qz, qw = quaternion_xyzw
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if abs(norm - 1.0) > 0.02:
            return _invalid(
                "ODOM_QUATERNION", source_stamp_s, receipt_stamp_s)
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        sample = _PoseSample(source_stamp_s, receipt_stamp_s, x, y, yaw)
        previous = self._previous
        if previous is None:
            self._previous = sample
            return _invalid("ODOM_INITIALIZING", source_stamp_s,
                            receipt_stamp_s)

        dt = source_stamp_s - previous.stamp_s
        if dt <= 0.0 or dt > self.max_dt_s:
            self._previous = sample
            reason = "ODOM_TIME" if dt <= 0.0 else "ODOM_GAP"
            return _invalid(reason, source_stamp_s, receipt_stamp_s)
        if dt < self.min_dt_s:
            return _invalid("ODOM_INTERVAL", source_stamp_s, receipt_stamp_s)

        speed = math.hypot(x - previous.x, y - previous.y) / dt
        yaw_rate = _wrapped_angle(yaw - previous.yaw) / dt
        self._previous = sample
        if speed > self.max_speed_mps or \
                abs(yaw_rate) > self.max_yaw_rate_rps:
            return _invalid("ODOM_JUMP", source_stamp_s, receipt_stamp_s)
        return MotionEstimate(True, source_stamp_s, receipt_stamp_s,
                              speed, yaw_rate, "")


def motion_hold_reason(
        estimate: MotionEstimate,
        now_s: float,
        max_age_s: float,
        future_tolerance_s: float = 0.05) -> str:
    if not estimate.valid:
        return estimate.reason or "ODOM_INVALID"
    if not math.isfinite(now_s):
        return "ODOM_TIME"
    ages = (now_s - estimate.source_stamp_s,
            now_s - estimate.receipt_stamp_s)
    if min(ages) < -future_tolerance_s:
        return "ODOM_TIME"
    if max(ages) > max_age_s:
        return "ODOM_STALE"
    return ""


def stopping_envelope(
        measured_speed_mps: float,
        requested_speed_mps: float,
        measured_yaw_rate_rps: float,
        requested_yaw_rate_rps: float,
        cloud_age_s: float,
        accumulation_s: float,
        pipeline_s: float,
        min_linear_decel_mps2: float,
        min_angular_decel_rps2: float,
        geometry_margin_m: float) -> StoppingEnvelope:
    values = (measured_speed_mps, requested_speed_mps,
              measured_yaw_rate_rps, requested_yaw_rate_rps, cloud_age_s,
              accumulation_s, pipeline_s, min_linear_decel_mps2,
              min_angular_decel_rps2, geometry_margin_m)
    if not all(math.isfinite(value) for value in values):
        raise MotionSafetyInputError(
            "stopping envelope inputs must be finite")
    if min(cloud_age_s, accumulation_s, pipeline_s, geometry_margin_m) < 0.0 \
            or min_linear_decel_mps2 <= 0.0 \
            or min_angular_decel_rps2 <= 0.0:
        raise MotionSafetyInputError(
            "stopping envelope limits must be positive")

    speed = max(abs(measured_speed_mps), max(0.0, requested_speed_mps))
    yaw_rate = max(abs(measured_yaw_rate_rps),
                   abs(requested_yaw_rate_rps))
    reaction = cloud_age_s + accumulation_s + pipeline_s
    distance = geometry_margin_m + speed * reaction + \
        speed * speed / (2.0 * min_linear_decel_mps2)
    linear_stop = speed / min_linear_decel_mps2
    angular_stop = yaw_rate / min_angular_decel_rps2
    return StoppingEnvelope(
        speed, yaw_rate, reaction, distance,
        reaction + max(linear_stop, angular_stop))


# The ground reference, and why heights are not measured from the chair.
#
# relative_height used to be points[:, 2] + sensor_height_m, which is a flat
# plane rigidly attached to the chair. The chair pitches. Wherever its
# attitude differs from the slope of the ground in front of it, that plane
# cuts through the road, and the road becomes an obstacle at the range where
# the wedge between them opens past min_height_m.
#
# Measured on 2026-08-23 (blackbox_20260823_200204, t+2253): cresting the
# hill at 0.8 m/s the chair was still nose-up while the ground ahead had
# levelled, and an object appeared at 3.30 m, 0.33 m tall - 5.7 degrees of
# nose-up over that range, exactly. It then wandered from y -0.70 to +0.41
# to -1.13 in four seconds with its height flickering between 0.22 and 0.49,
# and vanished when the pitch settled. Nothing physical moves like that. It
# was the road. The gate stopped the chair twice for 1.3 s each.
#
# So the band is taken against the ground the sensor can actually see, in
# range bins, at a low percentile of each bin. Bins are walked outward from
# the chair, which is standing on the ground and therefore anchors the
# nearest one, and each is allowed to differ from the last by no more than a
# drivable gradient. That clamp is what stops a wall or a parked car from
# lifting the reference up to its own roofline and erasing itself: a bin
# full of obstacle can only ever raise the reference by one bin's worth of
# slope, and everything above that is still an obstacle.
#
# Known limit, stated rather than hidden: a step that fills a whole bin and
# rises less than one bin's slope allowance above the last reads as terrain.
# A 0.20 m kerb square across the path is near that line. The flat-plane
# test it replaces caught such a step only when the chair happened to be
# pitched the right way, and false-stopped on every crest for it.
GROUND_BIN_M = 0.5
GROUND_MIN_POINTS = 12
GROUND_PERCENTILE = 10.0
# tan(18 degrees), and it has to cover the SLOPE THE SENSOR SEES, not the
# slope of the road: the reference is built in the chair's own frame, so
# the chair's attitude adds to the terrain. Route v9 climbs about 6 degrees
# and the crest transient was 5.7, so 12 is the working number and 18 is the
# margin over it. Any tighter and the clamp lags the road it is meant to
# follow, which puts the false obstacle straight back.
GROUND_SLOPE_LIMIT = 0.325
GROUND_MAX_RANGE_M = 12.0
# Only points within this of the centreline shape the reference. Beyond it
# lie kerbs, planters and the bank the route runs along, none of which are
# the surface the chair is about to drive over.
GROUND_CORRIDOR_HALF_WIDTH_M = 2.0
# A bin has found the ground when a slab this thin above its low percentile
# holds this many returns. Ground is a dense sheet; the lowest points off a
# bank, a trunk or a parked car are the bottom of something vertical and
# are spread out, not stacked in 0.15 m. Without this test a bin that can
# see no road at all still produced a candidate, and the clamp below turned
# a sequence of them into a staircase - the 2026-08-23 runaway, +0.1625 per
# bin all the way to +0.90 m, which is the clamp and not the road.
GROUND_SHEET_SLAB_M = 0.15
GROUND_SHEET_POINTS = 6
# An absolute cap was tried here and removed. It cannot tell a staircase
# from a real climb: 0.35 m is 5.7 degrees of attitude at 3.5 m and also an
# 8 degree road at 2.5 m, and capping the second to stop the first turns
# every genuine slope back into an obstacle.

# STILL OFF, and now for a reason that is about the premise rather than
# the tuning.
#
# The second attempt closed both field failures by construction - the
# ceiling no longer moves with the reference, so nothing overhead can be
# admitted however wrong it is, and a bin must look like a ground sheet
# before the reference will follow it. Then the sheet test was measured
# against 40 real clouds from aejimum_to_gongsen.bag, outdoors, this
# sensor on this chair:
#
#     per-frame peak reference   median   90th    max
#     with the sheet test         0.63    0.74    0.79
#     without it                  0.65    0.91    0.95
#
# It filters almost nothing, and the per-bin values are +0.163, +0.325,
# +0.488 - the clamp, one step at a time, in bins that pass the density
# test. The MID360 on the armrest does not return enough near-field road
# for a low percentile to be the ground, so "lowest returns in a range bin"
# is not a ground estimator on this vehicle. That is a wrong premise, not a
# wrong constant, and no amount of retuning reaches it.
#
# What would: /cloud_registered_body and the pose pitch recorded through
# the crest, which nothing captures today - the black box carries neither.
# With those, the transient part of the attitude is measurable directly
# (current pitch against the pitch the chair has been holding), and that is
# the quantity the crest false-stop is actually made of.
#
# The ceiling change and the sheet test are kept because they are right
# whatever replaces the estimator. The record of the first attempt follows:
#
# What follows is the record of why it went off; the constants above are
# the answer to it. The ceiling no longer moves, so no reference however
# wrong can admit something overhead. The reference cannot rise without a
# bin that looks like a ground sheet, so it no longer climbs the clamp on
# nothing. And it is capped at GROUND_MAX_OFFSET_M, so what a wrong
# reference can hide is bounded at 0.50 m rather than open-ended.
#
# The original note, kept because the numbers in it are the test data:
#
# Measured in the field on 2026-08-23, an hour after it went in: on a
# -2.2 degree descent at station ~1218 the reference climbed +0.1625 per
# bin - exactly the slope clamp, not the road - and reached +0.90 m by 4 m
# of range. It saturates whenever a bin holds no real ground returns, and
# at this pose the near field has almost none: the 5th percentile of the
# 0.5-1.0 m bin was already +0.33 m above the chair plane.
#
# Two things then go wrong, and the second is the serious one.
#
# The band is measured from the reference at BOTH ends, so lifting it by
# 0.7 m lifts the ceiling with it. Overhead clutter the flat plane
# correctly ignored comes into range: 13 points at a true 2.10-2.33 m -
# branches, well above the rider - were called obstacles in the forward
# corridor where the flat plane found none, and the gate stopped the chair
# on them. That is the branch problem this stack already solved once.
#
# Worse, a reference that high SUPPRESSES real low obstacles at range. A
# 0.5 m object at 4 m sits under reference + 0.15 and reads as road. An
# over-eager stop is a nuisance; a missed obstacle is not, and that is why
# this is off rather than merely retuned.
#
# The crest false-stop it was written for is real and still unfixed - see
# test_ground_reference.py, which keeps the mechanism and its numbers. What
# is missing is a reference that knows when it has actually found the
# ground rather than walking upward at the clamp when it has not.


def ground_reference(points: np.ndarray,
                     sensor_height_m: float,
                     bin_m: float = GROUND_BIN_M,
                     min_points: int = GROUND_MIN_POINTS,
                     percentile: float = GROUND_PERCENTILE,
                     slope_limit: float = GROUND_SLOPE_LIMIT,
                     max_range_m: float = GROUND_MAX_RANGE_M,
                     corridor_half_width_m: float =
                     GROUND_CORRIDOR_HALF_WIDTH_M) -> np.ndarray:
    """Height of the ground under each point, relative to the chair plane.

    Returns one value per point. A point's own height above the ground is
    its relative height minus this.

    Binned by forward distance rather than by radial range. The road rises
    along the direction of travel, so a radial bin mixes ground 5 m ahead
    with ground 4 m ahead and 3 m to the side, and its low percentile lands
    on the near-side floor - 0.14 m under the surface straight ahead on an
    8 degree climb, which is most of a 0.15 m threshold spent before any
    obstacle exists.
    """
    if len(points) == 0:
        return np.zeros(0, dtype=float)
    heights = points[:, 2] + float(sensor_height_m)
    forward = np.maximum(points[:, 0], 0.0)
    bins = np.minimum(
        (forward / float(bin_m)).astype(int),
        max(int(float(max_range_m) / float(bin_m)), 1))
    near = np.abs(points[:, 1]) <= float(corridor_half_width_m)
    count = int(bins.max()) + 1
    # The chair stands on the ground, so the bin it stands in is level with
    # it by definition. Everything else is reached outward from there.
    reference = np.zeros(count, dtype=float)
    allowance = float(slope_limit) * float(bin_m)
    previous = 0.0
    for index in range(count):
        selected = heights[(bins == index) & near]
        candidate = previous
        if len(selected) >= int(min_points):
            low = float(np.percentile(selected, float(percentile)))
            # Only believe it if it looks like a sheet. A bin that can see
            # no road has a low percentile too, and taking it is how the
            # reference walks upward on nothing.
            sheet = np.count_nonzero(
                (selected >= low) & (selected <= low + GROUND_SHEET_SLAB_M))
            if sheet >= GROUND_SHEET_POINTS:
                candidate = low
        previous = float(np.clip(candidate,
                                 previous - allowance,
                                 previous + allowance))
        reference[index] = previous
    return reference[bins]


def filter_obstacle_points(
        cloud: np.ndarray,
        sensor_height_m: float,
        min_height_m: float,
        max_height_m: float,
        self_x_min_m: float,
        self_x_max_m: float,
        self_half_width_m: float,
        self_y_centre_m: float = 0.0,
        ground_referenced: bool = False) -> np.ndarray:
    points = np.asarray(cloud)
    if points.ndim != 2 or points.shape[1] < 3:
        raise MotionSafetyInputError("cloud must have shape (N, 3+)")
    finite = np.all(np.isfinite(points[:, :3]), axis=1)
    relative_height = points[:, 2] + sensor_height_m
    floor = float(min_height_m)
    if ground_referenced and len(points):
        # The FLOOR moves with the ground; the ceiling never does.
        #
        # Measuring both ends from the reference is what put branches at a
        # true 2.10-2.33 m inside the band on 2026-08-23 and stopped the
        # chair on them: lifting the reference 0.84 m lifted the ceiling to
        # 2.34. The ceiling answers a different question - how high the
        # rider is - and the answer does not change because the road tilted.
        floor = float(min_height_m) + ground_reference(
            points[:, :3], sensor_height_m)
    # Centred on the rider, not on the sensor. The sensor is mounted on the
    # left armrest, so the body it is trying to exclude sits 0.173 m to its
    # right; a box centred on the sensor left that much of the rider's right
    # side outside it, and everything outside is an obstacle. Defaults to 0.0
    # so callers that have not been told the offset keep their old behaviour.
    self_return = ((points[:, 0] >= self_x_min_m) &
                   (points[:, 0] <= self_x_max_m) &
                   (np.abs(points[:, 1] - self_y_centre_m)
                    <= self_half_width_m))
    keep = (finite & ~self_return &
            (relative_height >= floor) &
            (relative_height <= max_height_m))
    return points[keep, :2]


def swept_footprint_collision(
        points_xy: np.ndarray,
        linear_speed_mps: float,
        angular_speed_rps: float,
        horizon_s: float,
        front_m: float,
        rear_m: float,
        half_width_m: float,
        margin_m: float,
        min_points: int = 5,
        max_step_s: float = 0.02,
        max_boundary_step_m: float = 0.02) -> bool:
    points = np.asarray(points_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise MotionSafetyInputError("points_xy must have shape (N, 2)")
    limits = (linear_speed_mps, angular_speed_rps, horizon_s, front_m,
              rear_m, half_width_m, margin_m, max_step_s,
              max_boundary_step_m)
    if not all(math.isfinite(value) for value in limits) or \
            min(horizon_s, front_m, rear_m, half_width_m, margin_m) < 0.0 \
            or max_step_s <= 0.0 or max_boundary_step_m <= 0.0 or \
            min_points < 1:
        raise MotionSafetyInputError("invalid swept-footprint limits")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < min_points:
        return False

    radius = math.hypot(max(front_m, rear_m) + margin_m,
                        half_width_m + margin_m)
    reach = abs(linear_speed_mps) * horizon_s + radius
    points = points[np.hypot(points[:, 0], points[:, 1]) <= reach]
    if len(points) < min_points:
        return False
    boundary_speed = abs(linear_speed_mps) + \
        abs(angular_speed_rps) * radius
    steps = max(
        1,
        int(math.ceil(horizon_s / max_step_s)),
        int(math.ceil(horizon_s * boundary_speed / max_boundary_step_m)))
    for elapsed in np.linspace(0.0, horizon_s, steps + 1):
        yaw = angular_speed_rps * elapsed
        if abs(angular_speed_rps) < 1e-9:
            x = linear_speed_mps * elapsed
            y = 0.0
        else:
            radius_of_turn = linear_speed_mps / angular_speed_rps
            x = radius_of_turn * math.sin(yaw)
            y = radius_of_turn * (1.0 - math.cos(yaw))
        dx = points[:, 0] - x
        dy = points[:, 1] - y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_x = np.add(
            np.multiply(dx, cosine), np.multiply(dy, sine))
        local_y = np.subtract(
            np.multiply(dy, cosine), np.multiply(dx, sine))
        inside = ((local_x >= -rear_m - margin_m) &
                  (local_x <= front_m + margin_m) &
                  (np.abs(local_y) <= half_width_m + margin_m))
        if np.count_nonzero(inside) >= min_points:
            return True
    return False
