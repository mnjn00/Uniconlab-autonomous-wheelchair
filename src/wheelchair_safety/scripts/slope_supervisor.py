#!/usr/bin/env python3
"""Fail-closed slope evidence supervisor with a ROS-independent core.

Quaternion inputs use ROS ``(x, y, z, w)`` order.  ``imu_to_base_quaternion``
rotates vectors reported in ``imu_link`` into ``base_link``.  Its provenance
must be explicitly labelled ``measured`` or ``simulation``; neither label
confers hardware or passenger authority.
"""

from dataclasses import dataclass
import hashlib
import json
import math
import re
from typing import Any, Iterable, Optional, Sequence, Tuple


UNKNOWN = 0
CLEAR = 1
SLOW = 2
STOP = 3
SAFETY_UNKNOWN = 0
SAFETY_CLEAR = 1
SAFETY_STOP = 2
CAL_UNCALIBRATED = 0
CAL_CALIBRATING = 1
CAL_VALID = 2
CAL_INVALID = 3
CLOCK = 1 << 8

SENSOR_STALE = 1 << 12
SLOPE = 1 << 16
IMU_UNCALIBRATED = 1 << 17
TF = 1 << 20
HARDWARE_UNVERIFIED = 1 << 24
CORRUPT_DATA = 1 << 29
INPUT_UNKNOWN = 1 << 31
GEOFENCE = 1 << 3
ROUTE_STATE = 1 << 32
IMU_STALE = 1 << 34
FUTURE_TOLERANCE_S = 0.05

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class SlopePolicy:
    policy_id: str = "slope-sim-v1"
    policy_sha256: str = "0000000000000000000000000000000000000000000000000000000000000000"
    source_ttl_s: float = 0.10
    receipt_ttl_s: float = 0.10
    transform_ttl_s: float = 0.10
    route_zone_ttl_s: float = 0.10
    gravity_mps2: float = 9.80665
    gravity_tolerance_mps2: float = 0.15
    fallback_gravity_tolerance_mps2: float = 0.25
    fallback_low_pass_time_constant_s: float = 1.0
    calibration_duration_s: float = 10.0
    calibration_sample_fraction: float = 0.95
    calibration_rate_hz: float = 200.0
    calibration_gyro_p95_max_rps: float = 0.02
    calibration_angle_stddev_max_deg: float = 0.25
    fallback_gyro_max_rps: float = 0.05
    orientation_disagreement_max_deg: float = 3.0
    dynamic_residual_max_mps2: float = 0.75
    dynamic_residual_duration_s: float = 0.10
    fallback_hold_max_s: float = 0.50
    downhill_stop_deg: float = -7.0
    downhill_clear_deg: float = -5.0
    uphill_clear_deg: float = 7.0
    uphill_stop_deg: float = 10.0
    roll_clear_deg: float = 4.0
    roll_stop_deg: float = 6.0
    slow_max_linear_mps: float = 0.10
    hysteresis_hold_s: float = 1.0
    hysteresis_tighten_deg: float = 1.0
    quaternion_norm_tolerance: float = 1.0e-3
    hardware_motion_authorized: bool = False
    passenger_operation_authorized: bool = False
    route_safety_source: str = "wheelchair_route_safety"
    route_safety_manifest_id: str = "hanyang-route-safety-candidate-v1"
    route_safety_manifest_sha256: str = "93ca862dac1fbdd5914d93b2d2c325fe2742aef2a05289d44d0d4fe45989de57"
    route_safety_map_id: str = "hanyang_aegimun_loop"
    route_safety_map_sha256: str = "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278"
    route_zone_policies: Tuple[Tuple[str, str], ...] = (("zone-simulation-candidate", "normal"),)

    def __post_init__(self) -> None:
        if self.hardware_motion_authorized or self.passenger_operation_authorized:
            raise ValueError("simulation slope policy cannot grant hardware/passenger authority")
        if not (self.downhill_stop_deg < self.downhill_clear_deg < self.uphill_clear_deg < self.uphill_stop_deg):
            raise ValueError("pitch thresholds are not ordered")
        if not (0.0 <= self.roll_clear_deg < self.roll_stop_deg):
            raise ValueError("roll thresholds are not ordered")
        if not self.policy_sha256 or not re.fullmatch(r"[0-9a-f]{64}", self.policy_sha256):
            raise ValueError("slope policy requires a supplied file SHA-256")
        if (not self.route_safety_source or not self.route_safety_manifest_id
                or not self.route_safety_map_id
                or not re.fullmatch(r"[0-9a-f]{64}", self.route_safety_manifest_sha256)
                or not re.fullmatch(r"[0-9a-f]{64}", self.route_safety_map_sha256)
                or not self.route_zone_policies
                or any(not zone or policy not in ("normal", "manual_only", "degraded_stop")
                       for zone, policy in self.route_zone_policies)):
            raise ValueError("route safety identity or zone policy is invalid")


@dataclass(frozen=True)
class SlopeDecision:
    sequence: int
    state: int
    safety_signal_state: int
    reason_mask: int
    calibration_state: int
    policy_id: str
    policy_sha256: str
    calibration_sha256: str
    source: str
    input_age_s: float
    transform_age_s: float
    gravity_norm_mps2: float
    pitch_rad: float
    roll_rad: float
    pitch_rate_rps: float
    roll_rate_rps: float
    acceleration_residual_mps2: float
    orientation_disagreement_rad: float
    recommended_max_linear_mps: float
    downhill_factor: float
    hardware_motion_authorized: bool = False
    passenger_operation_authorized: bool = False


@dataclass(frozen=True)
class CalibrationSample:
    quaternion: Quaternion
    acceleration: Vector3
    angular_velocity: Vector3
    source_stamp_s: Optional[float] = None
    imu_to_base_quaternion: Optional[Quaternion] = None
    imu_to_base_translation: Optional[Vector3] = None


def _components(value: Sequence[float], count: int) -> Tuple[float, ...]:
    if hasattr(value, "x"):
        names = ("x", "y", "z", "w")[:count]
        result = tuple(float(getattr(value, name)) for name in names)
    else:
        result = tuple(float(x) for x in value)
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise ValueError("non-finite or incorrectly sized value")
    return result


def normalize_quaternion(quaternion: Sequence[float], norm_tolerance: float = 1.0e-3) -> Quaternion:
    x, y, z, w = _components(quaternion, 4)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0 or abs(norm - 1.0) > norm_tolerance:
        raise ValueError("invalid quaternion norm")
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_to_pitch_roll(quaternion: Sequence[float], norm_tolerance: float = 1.0e-3) -> Tuple[float, float]:
    """Return REP-103 ``(pitch, roll)`` in radians from a ROS quaternion."""
    x, y, z, w = normalize_quaternion(quaternion, norm_tolerance)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return pitch, roll


def _quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quaternion_conjugate(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(vector: Sequence[float], quaternion: Sequence[float]) -> Vector3:
    """Rotate a vector by a normalized ROS quaternion."""
    v = _components(vector, 3)
    q = normalize_quaternion(quaternion)
    rotated = _quaternion_multiply(_quaternion_multiply(q, (v[0], v[1], v[2], 0.0)), _quaternion_conjugate(q))
    return rotated[:3]


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(x * x for x in vector))


def _percentile95(values: Sequence[float]) -> float:
    ordered = sorted(values)
    return ordered[int(math.ceil(0.95 * len(ordered))) - 1]




def _angle_stddev(values: Sequence[float]) -> float:
    mean = sum(values) / len(values)
    return math.sqrt(sum((x - mean) ** 2 for x in values) / len(values))


def _gravity_from_angles(pitch: float, roll: float, magnitude: float) -> Vector3:
    return (
        -magnitude * math.sin(pitch),
        magnitude * math.sin(roll) * math.cos(pitch),
        magnitude * math.cos(roll) * math.cos(pitch),
    )


def normalize_initial_zone_policy(zone: str, operation_mode: str) -> str:
    """Normalize only the explicit simulation bootstrap route-zone exception."""
    value = str(zone).strip()
    if value == "simulation_allow" and operation_mode == "simulation":
        return "normal"
    if value in ("manual_only", "degraded_stop"):
        return value
    return "unknown"



def _require_mapping(value: Any, name: str, keys: Sequence[str]) -> dict:
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError("%s keys are unsupported or incomplete" % name)
    return value


def _number(value: Any, name: str, minimum: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("%s must be numeric" % name)
    value = float(value)
    if not math.isfinite(value) or value < minimum:
        raise ValueError("%s is outside supported bounds" % name)
    return value


def _load_slope_policy(path: str, expected_file_sha256: Optional[str]) -> SlopePolicy:
    """Load the immutable simulation policy and bind it to the launch-supplied hash."""
    if not isinstance(expected_file_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_file_sha256):
        raise ValueError("slope policy SHA-256 must be supplied by launch")
    with open(path, "rb") as stream:
        raw = stream.read()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != expected_file_sha256:
        raise ValueError("slope policy file hash mismatch")
    import yaml
    document = yaml.safe_load(raw)
    root = _require_mapping(document, "policy", ("schema_version", "policy_id", "policy_sha256", "qualification", "operation_mode", "authority", "frames", "input", "calibration", "gravity_estimation", "thresholds_deg", "hysteresis", "route_zones", "route_safety_evidence", "downhill_stopping_coupling", "fail_closed"))
    if root["schema_version"] != 1 or root["qualification"] != "simulation_only" or root["operation_mode"] != "simulation":
        raise ValueError("policy is not qualified for simulation only")
    if not isinstance(root["policy_id"], str) or not root["policy_id"] or not re.fullmatch(r"[0-9a-f]{64}", root["policy_sha256"]):
        raise ValueError("invalid policy identity")
    authority = _require_mapping(root["authority"], "authority", ("hardware_motion_authorized", "passenger_operation_authorized", "transferable_to_hardware"))
    if any(authority[key] is not False for key in authority):
        raise ValueError("policy cannot authorize hardware or passengers")
    frames = _require_mapping(root["frames"], "frames", ("output", "input", "transform", "transform_provenance_allowed", "measured_transform_required_for_hardware", "convention"))
    if (frames["output"], frames["input"], frames["transform"], frames["convention"]) != ("base_link", "imu_link", "base_link_to_imu_link", "REP-103") or frames["transform_provenance_allowed"] != ["measured", "simulation"] or frames["measured_transform_required_for_hardware"] is not True:
        raise ValueError("unsupported frame policy")
    inputs = _require_mapping(root["input"], "input", ("source_ttl_s", "receipt_ttl_s", "transform_ttl_s", "route_zone_ttl_s", "verified_time_required"))
    if inputs["verified_time_required"] is not True:
        raise ValueError("verified time is required")
    calibration = _require_mapping(root["calibration"], "calibration", ("output_during_calibration", "input_provenance_required", "stationary_duration_s", "nominal_rate_hz", "minimum_expected_sample_fraction", "p95_gyro_norm_max_radps", "mean_gravity_mps2", "mean_acceleration_tolerance_mps2", "pitch_roll_stddev_max_deg", "verified_extrinsic_required", "verified_time_alignment_required", "zero_stamp_transform_allowed_only_when_explicitly_static", "hash_fields", "replay_hardware_unverified_result"))
    if (calibration["output_during_calibration"], calibration["replay_hardware_unverified_result"]) != ("unknown_stop", "uncalibrated_stop") or any(calibration[key] is not True for key in ("input_provenance_required", "verified_extrinsic_required", "verified_time_alignment_required", "zero_stamp_transform_allowed_only_when_explicitly_static")) or calibration["hash_fields"] != ["derived_acceleration_bias", "derived_gyro_bias", "verified_extrinsic", "input_provenance", "sample_count", "window"]:
        raise ValueError("unsupported calibration policy")
    gravity = _require_mapping(root["gravity_estimation"], "gravity_estimation", ("fallback", "low_pass_time_constant_s", "fallback_acceleration_magnitude_tolerance_mps2", "fallback_gyro_norm_max_radps", "orientation_gravity_agreement_max_deg", "dynamic_residual_max_mps2", "dynamic_residual_duration_s", "fallback_hold_max_s"))
    thresholds = _require_mapping(root["thresholds_deg"], "thresholds_deg", ("clear", "slow", "slow_max_linear_mps"))
    clear = _require_mapping(thresholds["clear"], "thresholds_deg.clear", ("pitch_min", "pitch_max", "absolute_roll_max"))
    slow = _require_mapping(thresholds["slow"], "thresholds_deg.slow", ("pitch_min", "pitch_max", "absolute_roll_max"))
    hysteresis = _require_mapping(root["hysteresis"], "hysteresis", ("release_hold_s", "tighten_each_boundary_deg"))
    zones = _require_mapping(root["route_zones"], "route_zones", ("normal", "manual_only", "degraded_stop", "unknown"))
    route_evidence = _require_mapping(root["route_safety_evidence"], "route safety evidence", ("source", "manifest_id", "manifest_sha256", "map_id", "map_sha256", "zones"))
    if (not isinstance(route_evidence["source"], str) or not route_evidence["source"]
            or not isinstance(route_evidence["manifest_id"], str) or not route_evidence["manifest_id"]
            or not isinstance(route_evidence["map_id"], str) or not route_evidence["map_id"]
            or not isinstance(route_evidence["manifest_sha256"], str)
            or not re.fullmatch(r"[0-9a-f]{64}", route_evidence["manifest_sha256"])
            or not isinstance(route_evidence["map_sha256"], str)
            or not re.fullmatch(r"[0-9a-f]{64}", route_evidence["map_sha256"])
            or not isinstance(route_evidence["zones"], dict)
            or not route_evidence["zones"]
            or any(not isinstance(zone_id, str) or not zone_id
                   or zone_policy not in ("normal", "manual_only", "degraded_stop")
                   for zone_id, zone_policy in route_evidence["zones"].items())):
        raise ValueError("unsupported route safety evidence policy")
    coupling = _require_mapping(root["downhill_stopping_coupling"], "downhill_stopping_coupling", ("downhill_factor", "equation", "nonpositive_effective_deceleration"))
    fail_closed = _require_mapping(root["fail_closed"], "fail_closed", ("stale_uncalibrated_bad_tf_bad_time_dynamic_unknown_zone", "slow_only_lowers_speed"))
    if gravity["fallback"] != "first_order_low_pass_acceleration_vector" or zones != {"normal": "evaluate_slope_policy", "manual_only": "stop_autonomy", "degraded_stop": "stop", "unknown": "stop"} or coupling != {"downhill_factor": "sin(max(0, -pitch_rad))", "equation": "a_eff = a_policy - g * downhill_factor", "nonpositive_effective_deceleration": "stop"} or fail_closed != {"stale_uncalibrated_bad_tf_bad_time_dynamic_unknown_zone": "stop", "slow_only_lowers_speed": True}:
        raise ValueError("unsupported fail-closed policy")
    values = dict(policy_id=root["policy_id"], policy_sha256=digest, source_ttl_s=_number(inputs["source_ttl_s"], "source_ttl_s"), receipt_ttl_s=_number(inputs["receipt_ttl_s"], "receipt_ttl_s"), transform_ttl_s=_number(inputs["transform_ttl_s"], "transform_ttl_s"), route_zone_ttl_s=_number(inputs["route_zone_ttl_s"], "route_zone_ttl_s"), gravity_mps2=_number(calibration["mean_gravity_mps2"], "mean_gravity_mps2"), gravity_tolerance_mps2=_number(calibration["mean_acceleration_tolerance_mps2"], "mean_acceleration_tolerance_mps2"), fallback_gravity_tolerance_mps2=_number(gravity["fallback_acceleration_magnitude_tolerance_mps2"], "fallback tolerance"), fallback_low_pass_time_constant_s=_number(gravity["low_pass_time_constant_s"], "low pass"), calibration_duration_s=_number(calibration["stationary_duration_s"], "duration"), calibration_sample_fraction=_number(calibration["minimum_expected_sample_fraction"], "fraction"), calibration_rate_hz=_number(calibration["nominal_rate_hz"], "rate"), calibration_gyro_p95_max_rps=_number(calibration["p95_gyro_norm_max_radps"], "gyro"), calibration_angle_stddev_max_deg=_number(calibration["pitch_roll_stddev_max_deg"], "angle"), fallback_gyro_max_rps=_number(gravity["fallback_gyro_norm_max_radps"], "fallback gyro"), orientation_disagreement_max_deg=_number(gravity["orientation_gravity_agreement_max_deg"], "agreement"), dynamic_residual_max_mps2=_number(gravity["dynamic_residual_max_mps2"], "residual"), dynamic_residual_duration_s=_number(gravity["dynamic_residual_duration_s"], "residual duration"), fallback_hold_max_s=_number(gravity["fallback_hold_max_s"], "fallback hold"), downhill_stop_deg=_number(slow["pitch_min"], "slow pitch minimum", -90.0), downhill_clear_deg=_number(clear["pitch_min"], "clear pitch minimum", -90.0), uphill_clear_deg=_number(clear["pitch_max"], "clear pitch maximum", -90.0), uphill_stop_deg=_number(slow["pitch_max"], "slow pitch maximum", -90.0), roll_clear_deg=_number(clear["absolute_roll_max"], "clear roll"), roll_stop_deg=_number(slow["absolute_roll_max"], "slow roll"), slow_max_linear_mps=_number(thresholds["slow_max_linear_mps"], "slow speed"), hysteresis_hold_s=_number(hysteresis["release_hold_s"], "hold"), hysteresis_tighten_deg=_number(hysteresis["tighten_each_boundary_deg"], "tighten"))
    values["route_safety_source"] = route_evidence["source"]
    values["route_safety_manifest_id"] = route_evidence["manifest_id"]
    values["route_safety_manifest_sha256"] = route_evidence["manifest_sha256"]
    values["route_safety_map_id"] = route_evidence["map_id"]
    values["route_safety_map_sha256"] = route_evidence["map_sha256"]
    values["route_zone_policies"] = tuple(sorted(route_evidence["zones"].items()))
    return SlopePolicy(**values)
def load_slope_policy(path: str, expected_file_sha256: Optional[str]) -> SlopePolicy:
    """Return only a fully parsed, hash-bound simulation policy."""
    try:
        return _load_slope_policy(path, expected_file_sha256)
    except Exception as exc:
        raise ValueError("invalid slope policy") from exc



def publication_due(last_stamp_s: Optional[float], stamp_s: float,
                    period_s: float) -> bool:
    """Bound publications to the safety-gate consumption rate without hiding bad time."""
    if not math.isfinite(stamp_s) or not math.isfinite(period_s) or period_s <= 0.0:
        return True
    if last_stamp_s is None or not math.isfinite(last_stamp_s) or stamp_s <= last_stamp_s:
        return True
    return stamp_s - last_stamp_s + 1.0e-12 >= period_s

class SlopeSupervisorCore:
    """Stateful fail-closed classifier; it never commands motion."""

    def __init__(self, policy: Optional[SlopePolicy] = None, source: str = "slope_supervisor"):
        self.policy = policy or SlopePolicy()
        self.source = source
        self.sequence = 0
        self.calibration_state = CAL_UNCALIBRATED
        self.calibration_sha256 = ""
        self._restricted_since: Optional[float] = None
        self._last_state = UNKNOWN
        self._dynamic_since: Optional[float] = None
        self._fallback_since: Optional[float] = None
        self._filtered_gravity: Optional[Vector3] = None
        self._filter_stamp: Optional[float] = None
        self._last_now: Optional[float] = None
        self._last_source_stamp: Optional[float] = None
        self._sealed_transform: Optional[Tuple[Vector3, Quaternion]] = None
        self._sealed_transform_label = ""
        self._sealed_input_provenance = ""

    def calibrate(
        self,
        samples: Iterable[CalibrationSample],
        imu_to_base_quaternion: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        imu_to_base_translation: Sequence[float] = (0.0, 0.0, 0.0),
        transform_verified: bool = False,
        time_verified: bool = False,
        transform_label: str = "unknown",
        *,
        operation_mode: str = "unverified",
        input_provenance: str = "",
        transform_is_static: bool = False,
        transform_stamp_s: Optional[float] = None,
    ) -> bool:
        """Validate and seal one deterministic, simulation-only stationary window."""
        self.calibration_state = CAL_CALIBRATING
        self.calibration_sha256 = ""
        try:
            values = list(samples)
            minimum = math.ceil(
                self.policy.calibration_duration_s
                * self.policy.calibration_rate_hz
                * self.policy.calibration_sample_fraction
            )
            transform = normalize_quaternion(imu_to_base_quaternion, self.policy.quaternion_norm_tolerance)
            translation = _components(imu_to_base_translation, 3)
            if operation_mode != "simulation":
                raise ValueError("calibration is restricted to simulation")
            if not isinstance(input_provenance, str) or not input_provenance.strip():
                raise ValueError("input provenance is required")
            if transform_verified is not True or time_verified is not True or transform_label != "simulation":
                raise ValueError("unverified calibration evidence")
            if (
                transform_stamp_s is None
                or not math.isfinite(float(transform_stamp_s))
                or float(transform_stamp_s) < 0.0
            ):
                raise ValueError("transform stamp evidence is invalid")
            if float(transform_stamp_s) == 0.0 and transform_is_static is not True:
                raise ValueError("zero-stamp transform was not explicitly identified as static")
            if len(values) < minimum:
                raise ValueError("insufficient calibration sample coverage")

            stamps = [float(sample.source_stamp_s) for sample in values]
            if not all(math.isfinite(stamp) and stamp > 0.0 for stamp in stamps):
                raise ValueError("invalid calibration timestamp")
            if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
                raise ValueError("calibration timestamps are not strictly increasing")
            window_duration = stamps[-1] - stamps[0]
            maximum_window = self.policy.calibration_duration_s + 1.0 / self.policy.calibration_rate_hz
            if (
                window_duration < self.policy.calibration_duration_s
                or window_duration > maximum_window + 1.0e-12
            ):
                raise ValueError("calibration window duration is invalid")

            gravity_norms, gyro_norms, residuals, pitches, rolls = [], [], [], [], []
            accelerations, gyros = [], []
            for sample in values:
                observed_transform = normalize_quaternion(
                    sample.imu_to_base_quaternion, self.policy.quaternion_norm_tolerance
                )
                equivalent_inverse_sign = tuple(-component for component in transform)
                observed_translation = _components(sample.imu_to_base_translation or (0.0, 0.0, 0.0), 3)
                if (observed_transform != transform and observed_transform != equivalent_inverse_sign) or observed_translation != translation:
                    raise ValueError("sample transform does not exactly match extrinsic evidence")
                acceleration = rotate_vector(sample.acceleration, transform)
                gyro = rotate_vector(sample.angular_velocity, transform)
                orientation_imu = normalize_quaternion(sample.quaternion, self.policy.quaternion_norm_tolerance)
                orientation = _quaternion_multiply(orientation_imu, _quaternion_conjugate(transform))
                pitch, roll = quaternion_to_pitch_roll(orientation, self.policy.quaternion_norm_tolerance)
                expected = _gravity_from_angles(pitch, roll, self.policy.gravity_mps2)
                residuals.append(_norm(tuple(acceleration[i] - expected[i] for i in range(3))))
                gravity_norms.append(_norm(acceleration))
                gyro_norms.append(_norm(gyro))
                pitches.append(pitch)
                rolls.append(roll)
                accelerations.append(acceleration)
                gyros.append(gyro)

            gravity_mean = sum(gravity_norms) / len(gravity_norms)
            angle_limit = math.radians(self.policy.calibration_angle_stddev_max_deg)
            if abs(gravity_mean - self.policy.gravity_mps2) > self.policy.gravity_tolerance_mps2 + 1.0e-12:
                raise ValueError("gravity calibration outside tolerance")
            if _percentile95(gyro_norms) > self.policy.calibration_gyro_p95_max_rps + 1.0e-12:
                raise ValueError("gyro calibration outside tolerance")
            if _percentile95(residuals) > self.policy.dynamic_residual_max_mps2:
                raise ValueError("stationary acceleration residual outside tolerance")
            if (
                _angle_stddev(pitches) > angle_limit + 1.0e-12
                or _angle_stddev(rolls) > angle_limit + 1.0e-12
            ):
                raise ValueError("stationary angle variation outside tolerance")

            mean_acceleration = tuple(sum(vector[i] for vector in accelerations) / len(values) for i in range(3))
            mean_gyro = tuple(sum(vector[i] for vector in gyros) / len(values) for i in range(3))
            mean_pitch = sum(pitches) / len(values)
            mean_roll = sum(rolls) / len(values)
            expected_mean = _gravity_from_angles(mean_pitch, mean_roll, self.policy.gravity_mps2)
            acceleration_bias = tuple(mean_acceleration[i] - expected_mean[i] for i in range(3))
            evidence = json.dumps(
                {
                    "bias": {"acceleration": acceleration_bias, "gyro": mean_gyro},
                    "extrinsic": {
                        "imu_to_base_quaternion": transform,
                        "imu_to_base_translation": translation,
                        "label": transform_label,
                        "static": bool(transform_is_static),
                        "stamp_s": float(transform_stamp_s),
                        "verified": True,
                    },
                    "input_provenance": input_provenance.strip(),
                    "sample_count": len(values),
                    "window": [stamps[0], stamps[-1]],
                },
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            self.calibration_sha256 = hashlib.sha256(evidence.encode("utf-8")).hexdigest()
            self._sealed_transform = (translation, transform)
            self._sealed_transform_label = transform_label
            self._sealed_input_provenance = input_provenance.strip()
            self.calibration_state = CAL_VALID
            return True
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.calibration_state = CAL_INVALID
            self.calibration_sha256 = ""
            return False

    def decide(self, **kwargs) -> SlopeDecision:
        """Alias for :meth:`evaluate`, suitable as the pure decision API."""
        return self.evaluate(**kwargs)

    def evaluate(
        self,
        quaternion: Optional[Sequence[float]],
        acceleration: Sequence[float],
        angular_velocity: Sequence[float],
        source_stamp_s: float,
        receipt_stamp_s: float,
        now_s: float,
        *,
        transform_age_s: float = 0.0,
        transform_valid: bool = True,
        time_valid: bool = True,
        transform_verified: bool = True,
        transform_label: str = "simulation",
        imu_to_base_quaternion: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        imu_to_base_translation: Sequence[float] = (0.0, 0.0, 0.0),
        input_provenance: str = "",
        zone: str = "normal",
        zone_age_s: float = math.inf,
        orientation_available: bool = True,
        calibrated: Optional[bool] = None,
    ) -> SlopeDecision:
        self.sequence += 1
        policy = self.policy
        reason = 0
        diagnostics = {
            "input_age_s": -1.0,
            "transform_age_s": -1.0,
            "gravity_norm_mps2": -1.0,
            "pitch_rad": 0.0,
            "roll_rad": 0.0,
            "pitch_rate_rps": 0.0,
            "roll_rate_rps": 0.0,
            "acceleration_residual_mps2": -1.0,
            "orientation_disagreement_rad": -1.0,
        }
        effective_calibrated = self.calibration_state == CAL_VALID and calibrated is not False
        calibration_state = self.calibration_state

        try:
            source_stamp = float(source_stamp_s)
            receipt_stamp = float(receipt_stamp_s)
            now = float(now_s)
            transform_age = float(transform_age_s)
            if not all(math.isfinite(x) for x in (source_stamp, receipt_stamp, now, transform_age)):
                raise ValueError("non-finite time")
            diagnostics["transform_age_s"] = transform_age
            source_age = now - source_stamp
            receipt_age = now - receipt_stamp
            diagnostics["input_age_s"] = max(0.0, source_age, receipt_age)
            if (
                not time_valid
                or source_age < -FUTURE_TOLERANCE_S - 1.0e-12
                or receipt_age < 0.0
                or receipt_stamp - source_stamp < -FUTURE_TOLERANCE_S - 1.0e-12
                or (self._last_now is not None and now < self._last_now)
                or (self._last_source_stamp is not None and source_stamp <= self._last_source_stamp)
            ):
                reason |= CLOCK | INPUT_UNKNOWN
            self._last_now = now
            if self._last_source_stamp is None or source_stamp > self._last_source_stamp:
                self._last_source_stamp = source_stamp
            if source_age > policy.source_ttl_s or receipt_age > policy.receipt_ttl_s:
                reason |= SENSOR_STALE | IMU_STALE
            if (
                not transform_valid
                or not transform_verified
                or transform_label not in ("measured", "simulation")
                or transform_age < 0.0
                or transform_age > policy.transform_ttl_s
            ):
                reason |= TF

            transform = normalize_quaternion(imu_to_base_quaternion, policy.quaternion_norm_tolerance)
            translation = _components(imu_to_base_translation, 3)
            if self._sealed_transform is not None:
                sealed_translation, sealed_rotation = self._sealed_transform
                sign_equivalent = tuple(-component for component in sealed_rotation)
                if (translation != sealed_translation or
                        (transform != sealed_rotation and transform != sign_equivalent) or
                        transform_label != self._sealed_transform_label or
                        input_provenance.strip() != self._sealed_input_provenance):
                    self.calibration_state = CAL_INVALID
                    self.calibration_sha256 = ""
                    effective_calibrated = False
                    calibration_state = CAL_INVALID
                    reason |= TF | IMU_UNCALIBRATED
            acceleration_base = rotate_vector(acceleration, transform)
            gyro_base = rotate_vector(angular_velocity, transform)
            gravity_norm = _norm(acceleration_base)
            gyro_norm = _norm(gyro_base)
            diagnostics["gravity_norm_mps2"] = gravity_norm
            diagnostics["pitch_rate_rps"] = gyro_base[1]
            diagnostics["roll_rate_rps"] = gyro_base[0]

            if orientation_available:
                if quaternion is None:
                    raise ValueError("orientation marked available but absent")
                orientation_imu = normalize_quaternion(quaternion, policy.quaternion_norm_tolerance)
                orientation = _quaternion_multiply(orientation_imu, _quaternion_conjugate(transform))
                pitch, roll = quaternion_to_pitch_roll(orientation, policy.quaternion_norm_tolerance)
                expected = _gravity_from_angles(pitch, roll, policy.gravity_mps2)
                residual = _norm(tuple(acceleration_base[i] - expected[i] for i in range(3)))
                if gravity_norm == 0.0:
                    raise ValueError("zero acceleration")
                cosine = sum(acceleration_base[i] * expected[i] for i in range(3)) / (gravity_norm * policy.gravity_mps2)
                disagreement = math.acos(max(-1.0, min(1.0, cosine)))
                if disagreement > math.radians(policy.orientation_disagreement_max_deg):
                    reason |= CORRUPT_DATA
                self._fallback_since = None
                self._filtered_gravity = acceleration_base
                self._filter_stamp = now
            else:
                fallback_valid = (
                    abs(gravity_norm - policy.gravity_mps2) <= policy.fallback_gravity_tolerance_mps2
                    and gyro_norm <= policy.fallback_gyro_max_rps
                )
                if not fallback_valid:
                    reason |= INPUT_UNKNOWN
                    if self._fallback_since is None:
                        self._fallback_since = now
                else:
                    self._fallback_since = None
                    if self._filtered_gravity is None or self._filter_stamp is None:
                        self._filtered_gravity = acceleration_base
                    else:
                        elapsed = max(0.0, now - self._filter_stamp)
                        alpha = elapsed / (policy.fallback_low_pass_time_constant_s + elapsed)
                        self._filtered_gravity = tuple(
                            old + alpha * (new - old)
                            for old, new in zip(self._filtered_gravity, acceleration_base)
                        )
                    self._filter_stamp = now
                gravity_estimate = self._filtered_gravity if self._filtered_gravity is not None else acceleration_base
                pitch = math.atan2(-gravity_estimate[0], math.hypot(gravity_estimate[1], gravity_estimate[2]))
                roll = math.atan2(gravity_estimate[1], gravity_estimate[2])
                expected = _gravity_from_angles(pitch, roll, policy.gravity_mps2)
                residual = _norm(tuple(acceleration_base[i] - expected[i] for i in range(3)))
                disagreement = -1.0

            diagnostics.update(
                pitch_rad=pitch,
                roll_rad=roll,
                acceleration_residual_mps2=residual,
                orientation_disagreement_rad=disagreement,
            )
            if residual > policy.dynamic_residual_max_mps2:
                if self._dynamic_since is None:
                    self._dynamic_since = now
                if now - self._dynamic_since >= policy.dynamic_residual_duration_s:
                    reason |= INPUT_UNKNOWN
            else:
                self._dynamic_since = None
        except (TypeError, ValueError, OverflowError):
            reason |= CORRUPT_DATA | INPUT_UNKNOWN

        if not effective_calibrated:
            reason |= IMU_UNCALIBRATED
        if (not math.isfinite(float(zone_age_s)) or float(zone_age_s) < 0.0 or
                float(zone_age_s) > policy.route_zone_ttl_s or
                zone not in ("normal", "manual_only", "degraded_stop")):
            reason |= ROUTE_STATE
        elif zone in ("manual_only", "degraded_stop"):
            reason |= ROUTE_STATE

        pitch_deg = math.degrees(diagnostics["pitch_rad"])
        roll_deg = abs(math.degrees(diagnostics["roll_rad"]))
        desired = self._angle_state(pitch_deg, roll_deg)
        if desired == STOP:
            reason |= SLOPE
        fail_closed = reason != 0
        state = UNKNOWN if fail_closed and not (reason & (SLOPE | ROUTE_STATE)) else STOP if fail_closed else desired
        if not fail_closed:
            state = self._apply_hysteresis(desired, pitch_deg, roll_deg, now)
        else:
            self._restricted_since = None
        self._last_state = state

        recommended = policy.slow_max_linear_mps if state == SLOW else (0.0 if state in (UNKNOWN, STOP) else -1.0)
        signal = SAFETY_UNKNOWN if state == UNKNOWN else SAFETY_STOP if state == STOP else SAFETY_CLEAR
        downhill_factor = max(0.0, math.sin(max(0.0, -diagnostics["pitch_rad"])))
        return SlopeDecision(
            sequence=self.sequence,
            state=state,
            safety_signal_state=signal,
            reason_mask=reason,
            calibration_state=calibration_state,
            policy_id=policy.policy_id,
            policy_sha256=policy.policy_sha256,
            calibration_sha256=self.calibration_sha256,
            source=self.source,
            recommended_max_linear_mps=recommended,
            downhill_factor=downhill_factor,
            **diagnostics,
        )

    def _angle_state(self, pitch_deg: float, absolute_roll_deg: float) -> int:
        p = self.policy
        epsilon = 1.0e-9
        if (
            pitch_deg < p.downhill_stop_deg - epsilon
            or pitch_deg > p.uphill_stop_deg + epsilon
            or absolute_roll_deg > p.roll_stop_deg + epsilon
        ):
            return STOP
        if (
            pitch_deg < p.downhill_clear_deg - epsilon
            or pitch_deg > p.uphill_clear_deg + epsilon
            or absolute_roll_deg > p.roll_clear_deg + epsilon
        ):
            return SLOW
        return CLEAR

    def _apply_hysteresis(self, desired: int, pitch_deg: float, roll_deg: float, now_s: float) -> int:
        if self._last_state not in (SLOW, STOP) or desired >= self._last_state:
            self._restricted_since = None
            return desired
        p = self.policy
        tight_clear = (
            pitch_deg >= p.downhill_clear_deg + p.hysteresis_tighten_deg
            and pitch_deg <= p.uphill_clear_deg - p.hysteresis_tighten_deg
            and roll_deg <= p.roll_clear_deg - p.hysteresis_tighten_deg
        )
        if not tight_clear:
            self._restricted_since = None
            return self._last_state
        if self._restricted_since is None:
            self._restricted_since = now_s
            return self._last_state
        if now_s - self._restricted_since < p.hysteresis_hold_s:
            return self._last_state
        self._restricted_since = None
        return desired


def _published_diagnostic(value: float) -> float:
    """Return the ABI sentinel when a numeric diagnostic is unavailable."""
    value = float(value)
    return value if math.isfinite(value) else -1.0


def _initial_zone_bootstrap_configuration(
    initial_policy, operation_mode, calibration_enabled, timeout_s, construction_stamp_s
):
    """Validate the bounded, simulation-only initial route-zone exception."""
    try:
        timeout_s = float(timeout_s)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("initial route-zone timeout must be finite") from None
    if not math.isfinite(timeout_s):
        raise ValueError("initial route-zone timeout must be finite")
    normalized_policy = normalize_initial_zone_policy(initial_policy, operation_mode)
    if timeout_s <= 0.0:
        return "unknown", False, None
    if (
        normalized_policy != "normal"
        or operation_mode != "simulation"
        or not calibration_enabled
    ):
        raise ValueError(
            "positive initial route-zone timeout requires simulation_allow simulation bootstrap"
        )
    try:
        construction_stamp_s = float(construction_stamp_s)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("initial route-zone bootstrap construction time must be finite") from None
    deadline_s = construction_stamp_s + timeout_s
    if not math.isfinite(construction_stamp_s) or not math.isfinite(deadline_s):
        raise ValueError("initial route-zone bootstrap construction time must be finite")
    return normalized_policy, True, deadline_s


class SlopeSupervisorRosNode:
    """Thin ROS1 adapter.  All ROS imports remain behind construction."""

    def __init__(self):
        import rospy
        import tf2_ros
        from sensor_msgs.msg import Imu
        from wheelchair_interfaces.msg import GeofenceStatus, SafetySignal, SlopeStatus

        self.rospy = rospy
        construction_stamp_s = rospy.Time.now().to_sec()
        self.SlopeStatus = SlopeStatus
        self.SafetySignal = SafetySignal
        self.core = SlopeSupervisorCore(load_slope_policy(
            str(rospy.get_param("~policy")),
            str(rospy.get_param("~expected_policy_file_sha256")),
        ))
        self.operation_mode = str(rospy.get_param("~operation_mode", "unverified"))
        self.calibration_enabled = bool(rospy.get_param("~stationary_calibration_enabled", False))
        self.transform_label = str(rospy.get_param("~transform_label", "unknown"))
        self.transform_verified = bool(rospy.get_param("~transform_verified", False))
        self.transform_is_static = bool(rospy.get_param("~transform_is_static", False))
        self.input_provenance = str(rospy.get_param("~input_provenance", ""))
        self.imu_frame = str(rospy.get_param("~imu_frame", "imu_link"))
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        if not math.isfinite(publish_rate_hz) or not 1.0 <= publish_rate_hz <= 50.0:
            raise ValueError("publish_rate_hz must be finite and in [1, 50]")
        self.publish_period_s = 1.0 / publish_rate_hz
        self.last_published_source_stamp_s = None
        self._source_high_water_stamp_s = None
        self._invalid_source_stop_emitted = False
        self._calibration_samples = []
        if self.calibration_enabled and self.operation_mode == "simulation":
            self.core.calibration_state = CAL_CALIBRATING
        else:
            # Precomputed hashes cannot authorize replay, hardware, or unverified input.
            self.core.calibration_state = CAL_UNCALIBRATED
            self.core.calibration_sha256 = ""
        self.zone, self._initial_zone_bootstrap_active, self._initial_zone_bootstrap_deadline_s = (
            _initial_zone_bootstrap_configuration(
                rospy.get_param("~initial_route_zone_policy", "unknown"),
                self.operation_mode,
                self.calibration_enabled,
                rospy.get_param("~initial_route_zone_timeout_s", 0.0),
                construction_stamp_s,
            )
        )
        self.zone_receipt_stamp_s = (
            construction_stamp_s if self._initial_zone_bootstrap_active else -math.inf
        )
        self._bootstrap_zone_sequence_high_water = None
        self._bootstrap_zone_evaluation_high_water_stamp_s = None
        self._bootstrap_zone_source_high_water_stamp_s = None
        self._zone_sequence_high_water = None
        self._zone_source_high_water_stamp_s = None
        self._zone_evaluation_high_water_stamp_s = None
        self._static_imu_transform = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.status_pub = rospy.Publisher("/safety/slope_status", SlopeStatus, queue_size=1)
        self.signal_pub = rospy.Publisher("/safety/slope", SafetySignal, queue_size=1)
        rospy.Subscriber("/sensors/imu/data", Imu, self._imu_cb, queue_size=1)
        rospy.Subscriber("/route_safety/geofence_status", GeofenceStatus, self._zone_cb, queue_size=1)

    def _zone_cb(self, msg):
        """Accept only fresh, monotonic, hash-bound GeofenceStatus evidence."""
        receipt_stamp = self.rospy.Time.now().to_sec()
        policy = self.core.policy
        try:
            source_stamp = float(msg.header.stamp.to_sec())
            evaluation_stamp = float(msg.evaluation_stamp.to_sec())
            sequence = msg.sequence
            state = msg.state
            reason_mask = msg.reason_mask
            route_id = msg.route_id
            segment_id = msg.segment_id
            zone_id = msg.zone_id
            if (
                not isinstance(sequence, int)
                or isinstance(sequence, bool)
                or not isinstance(state, int)
                or isinstance(state, bool)
                or not isinstance(reason_mask, int)
                or isinstance(reason_mask, bool)
                or sequence < 0
                or state < 0
                or reason_mask < 0
                or not all(math.isfinite(value) for value in (source_stamp, evaluation_stamp, receipt_stamp))
                or msg.source != policy.route_safety_source
                or msg.manifest_id != policy.route_safety_manifest_id
                or msg.manifest_sha256 != policy.route_safety_manifest_sha256
                or not all(isinstance(value, str) for value in (route_id, segment_id, zone_id))
            ):
                raise ValueError("invalid route safety identity")
            if (
                source_stamp == 0.0
                and evaluation_stamp == 0.0
                and receipt_stamp == 0.0
                and state == 0
                and reason_mask in (INPUT_UNKNOWN | GEOFENCE, TF | GEOFENCE)
                and segment_id == ""
                and zone_id == ""
                and self._initial_zone_bootstrap_active
                and self.operation_mode == "simulation"
            ):
                return
            if (
                source_stamp >= 0.0
                and source_stamp <= evaluation_stamp
                and state == 0
                and reason_mask in (INPUT_UNKNOWN | GEOFENCE, TF | GEOFENCE)
                and evaluation_stamp > 0.0
                and receipt_stamp - evaluation_stamp >= -FUTURE_TOLERANCE_S - 1.0e-12
                and receipt_stamp - evaluation_stamp <= policy.route_zone_ttl_s
                and segment_id == ""
                and zone_id == ""
                and self._initial_zone_bootstrap_active
                and self.operation_mode == "simulation"
                and self._initial_zone_bootstrap_deadline_s is not None
                and receipt_stamp < self._initial_zone_bootstrap_deadline_s
                and (self._bootstrap_zone_sequence_high_water is None
                     or sequence > self._bootstrap_zone_sequence_high_water)
                and (self._bootstrap_zone_evaluation_high_water_stamp_s is None
                     or evaluation_stamp > self._bootstrap_zone_evaluation_high_water_stamp_s)
                and (
                    self._bootstrap_zone_source_high_water_stamp_s is None
                    and source_stamp == 0.0
                    or source_stamp > 0.0
                    and (
                        self._bootstrap_zone_source_high_water_stamp_s is None
                        or source_stamp >= self._bootstrap_zone_source_high_water_stamp_s
                    )
                )
            ):
                self._bootstrap_zone_sequence_high_water = sequence
                self._bootstrap_zone_evaluation_high_water_stamp_s = evaluation_stamp
                if source_stamp > 0.0:
                    self._bootstrap_zone_source_high_water_stamp_s = source_stamp
                self.zone_receipt_stamp_s = receipt_stamp
                return
            if (
                source_stamp <= 0.0
                or source_stamp > evaluation_stamp
                or receipt_stamp - evaluation_stamp < -FUTURE_TOLERANCE_S - 1.0e-12
                or receipt_stamp - evaluation_stamp > policy.route_zone_ttl_s
                or self._zone_sequence_high_water is not None
                and sequence <= self._zone_sequence_high_water
                or self._zone_source_high_water_stamp_s is not None
                and source_stamp < self._zone_source_high_water_stamp_s
                or self._zone_evaluation_high_water_stamp_s is not None
                and evaluation_stamp <= self._zone_evaluation_high_water_stamp_s
                or self._bootstrap_zone_sequence_high_water is not None
                and sequence <= self._bootstrap_zone_sequence_high_water
                or self._bootstrap_zone_evaluation_high_water_stamp_s is not None
                and evaluation_stamp <= self._bootstrap_zone_evaluation_high_water_stamp_s
                or self._bootstrap_zone_source_high_water_stamp_s is not None
                and source_stamp < self._bootstrap_zone_source_high_water_stamp_s
                or state != 1
                or reason_mask != 0
                or not route_id
                or not segment_id
            ):
                raise ValueError("invalid route safety chronology or clear evidence")
            zone_policy = dict(policy.route_zone_policies).get(zone_id)
            if zone_policy not in ("normal", "manual_only", "degraded_stop"):
                raise ValueError("unknown route zone")
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._revoke_initial_zone_bootstrap(receipt_stamp)
            return
        self._zone_sequence_high_water = sequence
        self._zone_source_high_water_stamp_s = source_stamp
        self._zone_evaluation_high_water_stamp_s = evaluation_stamp
        self._initial_zone_bootstrap_active = False
        self.zone = zone_policy
        self.zone_receipt_stamp_s = receipt_stamp

    def _revoke_initial_zone_bootstrap(self, receipt_stamp):
        self._initial_zone_bootstrap_active = False
        self.zone = "unknown"
        self.zone_receipt_stamp_s = receipt_stamp

    def _lookup_imu_transform(self, source_stamp, receipt):
        if self.transform_is_static and self._static_imu_transform is not None:
            return self._static_imu_transform + (0.0,)
        tf = self.tf_buffer.lookup_transform(
            "base_link", self.imu_frame, source_stamp, self.rospy.Duration(0.01)
        )
        q = tf.transform.rotation
        transform_q = (q.x, q.y, q.z, q.w)
        translation = tf.transform.translation
        transform_translation = (translation.x, translation.y, translation.z)
        transform_stamp = tf.header.stamp.to_sec()
        if self.transform_is_static:
            if (
                transform_stamp < 0.0
                or not math.isfinite(transform_stamp)
                or tf.header.frame_id != "base_link"
                or tf.child_frame_id != self.imu_frame
            ):
                raise ValueError("static TF evidence is invalid")
            normalize_quaternion(transform_q, self.core.policy.quaternion_norm_tolerance)
            _components(transform_translation, 3)
            if not all(math.isfinite(value) for value in transform_translation):
                raise ValueError("static TF translation is invalid")
            if self.transform_verified:
                self._static_imu_transform = (
                    transform_q, transform_translation, transform_stamp,
                )
                return self._static_imu_transform + (0.0,)
            return transform_q, transform_translation, transform_stamp, 0.0
        if transform_stamp == 0.0:
            raise ValueError("zero-stamp TF was not identified as static")
        return transform_q, transform_translation, transform_stamp, (
            receipt - tf.header.stamp
        ).to_sec()

    def _refresh_initial_zone_bootstrap(self, receipt_stamp):
        if not self._initial_zone_bootstrap_active:
            return
        if (
            self.operation_mode != "simulation"
            or self._initial_zone_bootstrap_deadline_s is None
            or receipt_stamp >= self._initial_zone_bootstrap_deadline_s
        ):
            self._revoke_initial_zone_bootstrap(receipt_stamp)

    def _reset_calibration_candidate(self):
        self._calibration_samples = []
        if self.calibration_enabled and self.operation_mode == "simulation":
            self.core.calibration_state = CAL_CALIBRATING
            self.core.calibration_sha256 = ""


    def _observe_calibration_sample(
        self, sample, transform_valid, transform_q, transform_translation, transform_stamp
    ):
        if self.core.calibration_state != CAL_CALIBRATING:
            return False
        stamp = sample.source_stamp_s
        try:
            normalize_quaternion(sample.quaternion, self.core.policy.quaternion_norm_tolerance)
            _components(sample.acceleration, 3)
            _components(sample.angular_velocity, 3)
            _components(sample.imu_to_base_translation, 3)
            _components(transform_translation, 3)
            normalize_quaternion(
                sample.imu_to_base_quaternion,
                self.core.policy.quaternion_norm_tolerance,
            )
            normalize_quaternion(transform_q, self.core.policy.quaternion_norm_tolerance)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._reset_calibration_candidate()
            return False
        if (
            not transform_valid
            or stamp is None
            or not math.isfinite(float(stamp))
            or float(stamp) <= 0.0
            or transform_stamp is None
            or not math.isfinite(float(transform_stamp))
        ):
            self._reset_calibration_candidate()
            return False

        stamp = float(stamp)
        calibration_sample_valid = True
        if self._calibration_samples:
            previous_stamp = self._calibration_samples[-1].source_stamp_s
            if stamp <= previous_stamp:
                self._reset_calibration_candidate()
        self._calibration_samples.append(sample)
        maximum_window = (
            self.core.policy.calibration_duration_s
            + 1.0 / self.core.policy.calibration_rate_hz
        )
        while (
            len(self._calibration_samples) > 1
            and stamp - self._calibration_samples[0].source_stamp_s
            > maximum_window + 1.0e-12
        ):
            self._calibration_samples.pop(0)

        minimum_samples = math.ceil(
            self.core.policy.calibration_duration_s
            * self.core.policy.calibration_rate_hz
            * self.core.policy.calibration_sample_fraction
        )
        if (
            len(self._calibration_samples) >= minimum_samples
            and stamp - self._calibration_samples[0].source_stamp_s
            >= self.core.policy.calibration_duration_s
        ):
            calibrated = self.core.calibrate(
                self._calibration_samples,
                imu_to_base_quaternion=transform_q,
                imu_to_base_translation=transform_translation,
                transform_verified=self.transform_verified,
                time_verified=True,
                transform_label=self.transform_label,
                operation_mode=self.operation_mode,
                input_provenance=self.input_provenance,
                transform_is_static=self.transform_is_static,
                transform_stamp_s=transform_stamp,
            )
            self._calibration_samples = []
            if not calibrated:
                self.core.calibration_state = CAL_CALIBRATING
                self.core.calibration_sha256 = ""
        return calibration_sample_valid

    def _imu_cb(self, msg):
        rospy = self.rospy
        receipt = rospy.Time.now()
        transform_valid = msg.header.frame_id == self.imu_frame
        transform_age = math.inf
        transform_stamp = None
        transform_q = (0.0, 0.0, 0.0, 1.0)
        transform_translation = (0.0, 0.0, 0.0)
        try:
            if not transform_valid:
                raise ValueError("unexpected IMU frame")
            transform_q, transform_translation, transform_stamp, transform_age = (
                self._lookup_imu_transform(msg.header.stamp, receipt)
            )
        except Exception:
            transform_valid = False
        sample = CalibrationSample(
            quaternion=(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            acceleration=(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
            angular_velocity=(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            source_stamp_s=msg.header.stamp.to_sec(),
            imu_to_base_quaternion=transform_q,
            imu_to_base_translation=transform_translation,
        )
        if self.calibration_enabled and self.operation_mode == "simulation":
            self._observe_calibration_sample(
                sample, transform_valid, transform_q, transform_translation, transform_stamp
            )
        source_stamp_s = msg.header.stamp.to_sec()
        self._refresh_initial_zone_bootstrap(receipt.to_sec())
        try:
            source_stamp_s = float(source_stamp_s)
            new_source = (
                math.isfinite(source_stamp_s)
                and (self._source_high_water_stamp_s is None
                     or source_stamp_s > self._source_high_water_stamp_s)
            )
        except (TypeError, ValueError, OverflowError):
            new_source = False
        if not new_source:
            if self._invalid_source_stop_emitted:
                return
            self._invalid_source_stop_emitted = True
        else:
            self._source_high_water_stamp_s = source_stamp_s
            self._invalid_source_stop_emitted = False
            if not publication_due(
                    self.last_published_source_stamp_s,
                    source_stamp_s,
                    self.publish_period_s):
                return
            self.last_published_source_stamp_s = source_stamp_s
        orientation_available = not (len(msg.orientation_covariance) and msg.orientation_covariance[0] == -1.0)
        decision = self.core.evaluate(
            quaternion=(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            acceleration=(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
            angular_velocity=(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            source_stamp_s=source_stamp_s,
            receipt_stamp_s=receipt.to_sec(),
            now_s=receipt.to_sec(),
            transform_age_s=transform_age,
            transform_valid=transform_valid,
            transform_verified=self.transform_verified,
            transform_label=self.transform_label,
            imu_to_base_quaternion=transform_q,
            imu_to_base_translation=transform_translation,
            input_provenance=self.input_provenance,
            zone=self.zone,
            zone_age_s=receipt.to_sec() - self.zone_receipt_stamp_s,
            orientation_available=orientation_available,
        )
        self._publish_decision(decision, msg.header.stamp, receipt)

    def _publish_decision(self, decision, source_stamp, evaluation_stamp):
        status = self.SlopeStatus()
        status.header.stamp = source_stamp
        status.header.frame_id = "base_link"
        status.evaluation_stamp = evaluation_stamp
        for name in (
            "sequence", "state", "calibration_state", "reason_mask", "source", "policy_id",
            "policy_sha256", "calibration_sha256",
        ):
            setattr(status, name, getattr(decision, name))
        for name in (
            "gravity_norm_mps2", "pitch_rad", "roll_rad", "pitch_rate_rps",
            "roll_rate_rps", "acceleration_residual_mps2",
            "orientation_disagreement_rad", "recommended_max_linear_mps",
        ):
            setattr(status, name, _published_diagnostic(getattr(decision, name)))
        status.input_age_s = _published_diagnostic(decision.input_age_s)
        status.transform_age_s = _published_diagnostic(decision.transform_age_s)
        signal = self.SafetySignal()
        signal.header.stamp = status.evaluation_stamp
        signal.header.frame_id = status.header.frame_id
        signal.sequence = status.sequence
        signal.state = decision.safety_signal_state
        signal.reason_mask = status.reason_mask
        signal.source = status.source
        signal.policy_sha256 = status.policy_sha256
        self.status_pub.publish(status)
        self.signal_pub.publish(signal)


def run_ros_node() -> None:
    import rospy

    rospy.init_node("slope_supervisor")
    SlopeSupervisorRosNode()
    rospy.spin()


if __name__ == "__main__":
    run_ros_node()
