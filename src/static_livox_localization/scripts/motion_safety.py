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


def filter_obstacle_points(
        cloud: np.ndarray,
        sensor_height_m: float,
        min_height_m: float,
        max_height_m: float,
        self_x_min_m: float,
        self_x_max_m: float,
        self_half_width_m: float,
        self_y_centre_m: float = 0.0) -> np.ndarray:
    points = np.asarray(cloud)
    if points.ndim != 2 or points.shape[1] < 3:
        raise MotionSafetyInputError("cloud must have shape (N, 3+)")
    finite = np.all(np.isfinite(points[:, :3]), axis=1)
    relative_height = points[:, 2] + sensor_height_m
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
            (relative_height >= min_height_m) &
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
