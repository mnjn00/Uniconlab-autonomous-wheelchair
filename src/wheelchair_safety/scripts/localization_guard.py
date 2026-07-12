#!/usr/bin/env python3
"""ROS-independent localization confidence guard.

A localizer candidate is untrusted evidence.  Only this stateful guard may turn a
complete set of independent checks into localization permission.  The module has
no ROS imports so the same decisions can be replayed in ordinary Python tests.
"""

from dataclasses import dataclass, replace
from bisect import bisect_left
import hashlib
import math
import re
from types import MappingProxyType
from typing import Any, Mapping, Optional, Tuple
import os
import struct
from typing import Iterable, List, Sequence
import time
import threading


# LocalizationStatus states.
UNINITIALIZED = 0
INITIALIZING = 1
OK = 2
DEGRADED = 3
LOST = 4
RELOCALIZING = 5

# SafetySignal states.
SAFETY_UNKNOWN = 0
SAFETY_CLEAR = 1
SAFETY_STOP = 2

# SafetyReason registry bits used by this guard.
REASON_LOCALIZATION = 32
REASON_CLOCK = 256
REASON_STARTUP = 2048
REASON_SENSOR_STALE = 4096
REASON_TF = 1048576
REASON_MAP_MISMATCH = 33554432
REASON_LOCALIZATION_INCONSISTENT = 134217728
REASON_CORRUPT_DATA = 536870912
REASON_RESET_REJECTED = 1073741824
REASON_INPUT_UNKNOWN = 2147483648
REASON_ODOM_STALE = 8589934592
REASON_POLICY_MISMATCH = 68719476736

_SHA256 = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class LocalizationPolicy:
    """Runtime-immutable, software-RC qualification limits (not ground truth)."""

    map_id: str
    map_sha256: str
    policy_sha256: str
    expected_frame: str = "map"
    max_candidate_age_s: float = 0.25
    max_future_skew_s: float = 0.05
    max_transform_age_s: float = 0.25
    min_position_std_m: float = 0.01
    max_position_std_m: float = 0.20
    min_yaw_std_rad: float = math.radians(0.5)
    max_yaw_std_rad: float = math.radians(5.0)
    max_innovation_nis: float = 12.84
    max_scan_residual_m: float = 0.20
    min_inlier_ratio: float = 0.65
    min_ambiguity_ratio: float = 1.50
    scan_candidate_samples: int = 1024
    scan_minimum_selected_points: int = 16
    scan_maximum_selected_points: int = 128
    scan_planar_range_min_m: float = 0.5
    scan_planar_range_max_m: float = 20.0
    scan_base_z_min_m: float = 0.15
    scan_base_z_max_m: float = 2.0
    max_position_jump_m: float = 0.25
    max_yaw_jump_rad: float = math.radians(10.0)
    max_odom_age_s: float = 0.25
    stationary_linear_mps: float = 0.01
    stationary_angular_rps: float = 0.02
    stationary_duration_s: float = 2.0
    initialization_samples: int = 20
    initialization_span_s: float = 2.0
    relocalization_samples: int = 30
    relocalization_span_s: float = 3.0
    degraded_zones: Tuple[str, ...] = ()
    manual_only_zones: Tuple[str, ...] = ()
    calibration_fraction: float = 0.70
    holdout_fraction: float = 0.30
    holdout_windows: int = 10000
    holdout_false_ok_windows: int = 0
    scenario_disjoint: bool = True
    thresholds_frozen_before_holdout: bool = True
    absolute_truth_claimed: bool = False
    calibration_qualified: bool = True

    def __post_init__(self) -> None:
        if not self.map_id or not self.expected_frame:
            raise ValueError("map_id and expected_frame are required")
        for name in ("map_sha256", "policy_sha256"):
            value = getattr(self, name)
            if not _SHA256.fullmatch(value):
                raise ValueError("%s must be a lowercase SHA-256" % name)
        numeric = (
            "max_candidate_age_s", "max_transform_age_s", "min_position_std_m",
            "max_position_std_m", "min_yaw_std_rad", "max_yaw_std_rad",
            "max_innovation_nis", "max_scan_residual_m", "min_inlier_ratio",
            "min_ambiguity_ratio", "max_position_jump_m", "max_yaw_jump_rad",
            "max_odom_age_s", "stationary_duration_s", "initialization_span_s",
            "relocalization_span_s",
        )
        if any(not math.isfinite(float(getattr(self, n))) or float(getattr(self, n)) <= 0 for n in numeric):
            raise ValueError("policy limits must be finite and positive")
        if self.max_candidate_age_s > 0.25 or self.max_transform_age_s > 0.25:
            raise ValueError("candidate/TF age exceeds software-RC ceiling")
        if self.max_position_std_m > 0.20 or self.max_yaw_std_rad > math.radians(5.0) + 1e-12:
            raise ValueError("reported uncertainty exceeds software-RC ceiling")
        if self.max_innovation_nis > 12.84 or self.max_scan_residual_m > 0.20:
            raise ValueError("consistency threshold exceeds software-RC ceiling")
        if self.min_inlier_ratio < 0.65 or self.min_ambiguity_ratio < 1.50:
            raise ValueError("qualification threshold is weaker than software-RC floor")
        if (
                self.scan_candidate_samples != 1024
                or self.scan_minimum_selected_points != 16
                or self.scan_maximum_selected_points != 128
                or self.scan_planar_range_min_m != 0.5
                or self.scan_planar_range_max_m != 20.0
                or self.scan_base_z_min_m != 0.15
                or self.scan_base_z_max_m != 2.0):
            raise ValueError("scan selection must match the frozen A09 geometry contract")
        if self.max_position_jump_m > 0.25 or self.max_yaw_jump_rad > math.radians(10.0) + 1e-12:
            raise ValueError("continuity threshold exceeds software-RC ceiling")
        if self.min_position_std_m >= self.max_position_std_m or self.min_yaw_std_rad >= self.max_yaw_std_rad:
            raise ValueError("invalid covariance calibration range")
        if self.initialization_samples < 20 or self.initialization_span_s < 2.0:
            raise ValueError("initialization qualification is too weak")
        if self.relocalization_samples < 30 or self.relocalization_span_s < 3.0:
            raise ValueError("relocalization qualification is too weak")
        if abs(self.calibration_fraction - 0.70) > 1e-9 or abs(self.holdout_fraction - 0.30) > 1e-9:
            raise ValueError("calibration metadata must specify a 70/30 split")
        if self.holdout_windows < 10000 or self.holdout_false_ok_windows != 0:
            raise ValueError("holdout evidence does not meet false-OK contract")
        if not self.scenario_disjoint or not self.thresholds_frozen_before_holdout:
            raise ValueError("holdout must be scenario-disjoint and evaluated after threshold freeze")
        if self.absolute_truth_claimed:
            raise ValueError("localization qualification may not claim absolute truth")


@dataclass(frozen=True)
class CandidateEvidence:
    stamp_s: float
    source: str
    raw_state: int
    reset_count: int
    map_id: str
    map_sha256: str
    raw_score: float
    position_std_m: float
    yaw_std_rad: float
    scan_residual_m: float
    inlier_ratio: float
    innovation_nis: float
    ambiguity_ratio: float
    frame_id: str = "map"
    policy_sha256: str = ""
    transform_age_s: float = -1.0
    odom_age_s: float = -1.0
    position_jump_m: float = 0.0
    yaw_jump_rad: float = 0.0
    covariance_planar: Optional[Tuple[float, ...]] = None
    zone_id: str = ""
    linear_speed_mps: float = math.inf
    angular_speed_rps: float = math.inf
    stationary_duration_s: float = 0.0
    tf_authority_count: int = 1
    odom_delta_m: float = 0.0
    initial_pose_fresh: bool = False
    mission_canceled: bool = False
    relocalization_requested: bool = False
    relocalization_jump_evidence: bool = False


@dataclass(frozen=True)
class GuardResult:
    state: int
    reason_mask: int
    independent_check_passed: bool
    safety_state: int
    safety_reason_mask: int
    sequence: int
    evaluation_stamp_s: float
    pose_age_s: float
    transform_age_s: float
    position_std_m: float
    yaw_std_rad: float
    scan_residual_m: float
    inlier_ratio: float
    innovation_nis: float
    ambiguity_ratio: float
    position_jump_m: float
    yaw_jump_rad: float
    consecutive_good_samples: int
    reset_count: int
    source: str
    map_id: str
    map_sha256: str
    policy_sha256: str
    zone_id: str


class LocalizationGuardCore:
    """Deterministic independent guard with latched loss and explicit recovery."""

    UNINITIALIZED = UNINITIALIZED
    INITIALIZING = INITIALIZING
    OK = OK
    DEGRADED = DEGRADED
    LOST = LOST
    RELOCALIZING = RELOCALIZING

    def __init__(self, policy: LocalizationPolicy):
        self.policy = policy
        self.state = UNINITIALIZED
        self.sequence = 0
        self._good_count = 0
        self._window_start_stamp: Optional[float] = None
        self._source_stamp_high_water: Optional[float] = None
        self._last_reset_count: Optional[int] = None
        self._loss_reset_count: Optional[int] = None
        self._recovery_epoch_reset_count: Optional[int] = None

    def evaluate(self, candidate: CandidateEvidence, now_s: float) -> GuardResult:
        self.sequence += 1
        # An adapter/watchdog loss has no candidate reset count.  The first
        # subsequent candidate establishes a STOP-only baseline; only a later
        # explicit increment can start relocalization.
        self.adopt_loss_reset_baseline(candidate.reset_count)
        reasons, pose_age = self._qualification_reasons(candidate, now_s)
        stationary = (
            candidate.mission_canceled
            and abs(candidate.linear_speed_mps) < self.policy.stationary_linear_mps
            and abs(candidate.angular_speed_rps) < self.policy.stationary_angular_rps
            and candidate.stationary_duration_s >= self.policy.stationary_duration_s
        )
        degraded = (
            candidate.zone_id in (self.policy.degraded_zones + self.policy.manual_only_zones)
            or candidate.zone_id in ("unknown", "unknown_zone")
        )
        reset_changed = (
            self._last_reset_count is not None and
            candidate.reset_count != self._last_reset_count
        )
        reset_regressed = (
            self._last_reset_count is not None and
            candidate.reset_count < self._last_reset_count
        )
        if reset_regressed:
            reasons |= REASON_LOCALIZATION | REASON_RESET_REJECTED
        raw_ok = candidate.raw_state == OK
        if not raw_ok:
            reasons |= REASON_LOCALIZATION
            reasons |= (REASON_LOCALIZATION_INCONSISTENT
                        if candidate.raw_state in (INITIALIZING, DEGRADED, LOST, RELOCALIZING)
                        else REASON_INPUT_UNKNOWN)

        # A reset is recovery evidence only after loss.  Everywhere else it is a
        # discontinuity, never an opportunity for an already-clear guard to stay clear.
        if reset_changed and self.state not in (LOST, RELOCALIZING):
            reasons |= REASON_LOCALIZATION | REASON_RESET_REJECTED
        if self._last_reset_count is None or candidate.reset_count > self._last_reset_count:
            self._last_reset_count = candidate.reset_count

        if reasons:
            self._transition_loss(candidate.reset_count)
            return self._result(LOST, reasons, False, candidate, now_s, pose_age)

        if degraded:
            self._reset_window()
            self.state = DEGRADED
            return self._result(DEGRADED, REASON_LOCALIZATION, False, candidate, now_s, pose_age)
        if self.state == DEGRADED:
            self._transition_loss(candidate.reset_count)
            return self._result(
                LOST, REASON_LOCALIZATION | REASON_RESET_REJECTED,
                False, candidate, now_s, pose_age,
            )

        if self.state in (LOST, RELOCALIZING):
            reset_advanced = self._loss_reset_count is not None and candidate.reset_count > self._loss_reset_count
            recovery_ready = (
                candidate.relocalization_requested and candidate.initial_pose_fresh
                and candidate.relocalization_jump_evidence and stationary and reset_advanced
            )
            if not recovery_ready:
                self._reset_window()
                self.state = LOST
                return self._result(
                    LOST, REASON_LOCALIZATION | REASON_RESET_REJECTED,
                    False, candidate, now_s, pose_age,
                )
            self.state = RELOCALIZING
            self._advance_window(candidate.stamp_s)
            if (self._good_count >= self.policy.relocalization_samples and
                    self._window_span(candidate.stamp_s) >= self.policy.relocalization_span_s):
                self.state = OK
                return self._result(OK, 0, True, candidate, now_s, pose_age)
            return self._result(
                RELOCALIZING, REASON_LOCALIZATION | REASON_STARTUP,
                False, candidate, now_s, pose_age,
            )

        if self.state in (UNINITIALIZED, INITIALIZING):
            initialization_prerequisites = candidate.initial_pose_fresh and stationary
            if not initialization_prerequisites:
                self._reset_window()
                self.state = INITIALIZING
                return self._result(
                    INITIALIZING, REASON_LOCALIZATION | REASON_STARTUP,
                    False, candidate, now_s, pose_age,
                )
            self.state = INITIALIZING
            self._advance_window(candidate.stamp_s)
            if (self._good_count >= self.policy.initialization_samples and
                    self._window_span(candidate.stamp_s) >= self.policy.initialization_span_s):
                self.state = OK
                return self._result(OK, 0, True, candidate, now_s, pose_age)
            return self._result(
                INITIALIZING, REASON_LOCALIZATION | REASON_STARTUP,
                False, candidate, now_s, pose_age,
            )

        self._advance_window(candidate.stamp_s)
        self.state = OK
        return self._result(OK, 0, True, candidate, now_s, pose_age)

    def _qualification_reasons(self, c: CandidateEvidence, now_s: float) -> Tuple[int, float]:
        reason = 0
        if not math.isfinite(now_s) or not math.isfinite(c.stamp_s):
            return REASON_LOCALIZATION | REASON_CLOCK | REASON_CORRUPT_DATA, math.inf
        pose_age = now_s - c.stamp_s
        if pose_age < -self.policy.max_future_skew_s:
            reason |= REASON_LOCALIZATION | REASON_CLOCK
        elif pose_age < 0.0:
            pose_age = 0.0
        elif pose_age > self.policy.max_candidate_age_s:
            reason |= REASON_LOCALIZATION | REASON_SENSOR_STALE
        if self._source_stamp_high_water is not None and c.stamp_s <= self._source_stamp_high_water:
            reason |= REASON_LOCALIZATION | REASON_CLOCK
        if (self._source_stamp_high_water is None or
                c.stamp_s > self._source_stamp_high_water):
            self._source_stamp_high_water = c.stamp_s
        if c.frame_id != self.policy.expected_frame:
            reason |= REASON_LOCALIZATION | REASON_TF
        if c.map_id != self.policy.map_id or c.map_sha256 != self.policy.map_sha256:
            reason |= REASON_LOCALIZATION | REASON_MAP_MISMATCH
        if c.policy_sha256 and c.policy_sha256 != self.policy.policy_sha256:
            reason |= REASON_LOCALIZATION | REASON_POLICY_MISMATCH
        if not self.policy.calibration_qualified:
            reason |= REASON_LOCALIZATION | REASON_POLICY_MISMATCH
        if c.tf_authority_count != 1 or not _finite_between(c.transform_age_s, 0.0, self.policy.max_transform_age_s):
            reason |= REASON_LOCALIZATION | REASON_TF
        if not _finite_between(c.odom_age_s, 0.0, self.policy.max_odom_age_s):
            reason |= REASON_LOCALIZATION | REASON_ODOM_STALE
        if not c.source or not math.isfinite(c.raw_score) or c.raw_score < 0.0:
            reason |= REASON_LOCALIZATION | REASON_INPUT_UNKNOWN
        if not self._valid_covariance(c):
            reason |= REASON_LOCALIZATION | REASON_CORRUPT_DATA
        metrics = (c.scan_residual_m, c.inlier_ratio, c.innovation_nis, c.ambiguity_ratio)
        if any(not math.isfinite(v) or v < 0.0 for v in metrics):
            reason |= REASON_LOCALIZATION | REASON_INPUT_UNKNOWN
        elif (c.scan_residual_m > self.policy.max_scan_residual_m
              or c.inlier_ratio < self.policy.min_inlier_ratio
              or c.innovation_nis > self.policy.max_innovation_nis
              or c.ambiguity_ratio < self.policy.min_ambiguity_ratio):
            reason |= REASON_LOCALIZATION | REASON_LOCALIZATION_INCONSISTENT
        jump = c.position_jump_m > self.policy.max_position_jump_m or abs(c.yaw_jump_rad) > self.policy.max_yaw_jump_rad
        explicit_jump = self.state in (LOST, RELOCALIZING) and c.relocalization_requested and c.relocalization_jump_evidence
        if jump and not explicit_jump:
            reason |= REASON_LOCALIZATION | REASON_LOCALIZATION_INCONSISTENT
        if c.odom_delta_m > self.policy.max_position_jump_m and c.position_jump_m <= self.policy.min_position_std_m:
            reason |= REASON_LOCALIZATION | REASON_LOCALIZATION_INCONSISTENT
        return reason, pose_age

    def _valid_covariance(self, c: CandidateEvidence) -> bool:
        if not (_finite_between(c.position_std_m, self.policy.min_position_std_m, self.policy.max_position_std_m)
                and _finite_between(c.yaw_std_rad, self.policy.min_yaw_std_rad, self.policy.max_yaw_std_rad)):
            return False
        if c.covariance_planar is None:
            return True
        values = c.covariance_planar
        if len(values) != 9 or any(not math.isfinite(v) for v in values):
            return False
        # Sylvester criterion for a symmetric positive-definite 3x3 covariance.
        if abs(values[1] - values[3]) > 1e-9 or abs(values[2] - values[6]) > 1e-9 or abs(values[5] - values[7]) > 1e-9:
            return False
        d1 = values[0]
        d2 = values[0] * values[4] - values[1] * values[3]
        det = (values[0] * (values[4] * values[8] - values[5] * values[7])
               - values[1] * (values[3] * values[8] - values[5] * values[6])
               + values[2] * (values[3] * values[7] - values[4] * values[6]))
        return d1 > 0.0 and d2 > 0.0 and det > 0.0

    def _advance_window(self, stamp_s: float) -> None:
        if self._window_start_stamp is None:
            self._window_start_stamp = stamp_s
            self._good_count = 1
        else:
            self._good_count += 1

    def _window_span(self, stamp_s: float) -> float:
        return 0.0 if self._window_start_stamp is None else stamp_s - self._window_start_stamp

    def _reset_window(self) -> None:
        self._good_count = 0
        self._window_start_stamp = None

    def _transition_loss(self, reset_count: Optional[int] = None) -> None:
        """Enter LOST once and preserve the reset baseline required for recovery."""
        if self.state not in (LOST, RELOCALIZING):
            self._loss_reset_count = (
                self._last_reset_count if reset_count is None else reset_count
            )
            self._recovery_epoch_reset_count = None
        self.state = LOST
        self._reset_window()

    def adopt_loss_reset_baseline(self, reset_count: int) -> None:
        """Record the first candidate reset count observed after an input-only loss."""
        if self.state in (LOST, RELOCALIZING) and self._loss_reset_count is None:
            self._loss_reset_count = reset_count

    def consume_recovery_epoch(self, reset_count: int, requested: bool) -> bool:
        """Consume one explicit reset transition to begin a new input chronology."""
        if (self.state not in (LOST, RELOCALIZING) or not requested or
                self._loss_reset_count is None or
                reset_count <= self._loss_reset_count or
                self._recovery_epoch_reset_count == reset_count):
            return False
        self._recovery_epoch_reset_count = reset_count
        return True

    def force_loss(self) -> None:
        """Fail closed for adapter failures that cannot yield CandidateEvidence."""
        self._transition_loss()

    def _result(self, state: int, reason: int, passed: bool, c: CandidateEvidence,
                now_s: float, pose_age: float) -> GuardResult:
        clear = state == OK and passed and reason == 0
        return GuardResult(
            state=state, reason_mask=reason, independent_check_passed=clear,
            safety_state=SAFETY_CLEAR if clear else SAFETY_STOP,
            safety_reason_mask=0 if clear else (reason | REASON_LOCALIZATION),
            sequence=self.sequence, evaluation_stamp_s=now_s,
            pose_age_s=pose_age, transform_age_s=c.transform_age_s,
            position_std_m=c.position_std_m, yaw_std_rad=c.yaw_std_rad,
            scan_residual_m=c.scan_residual_m, inlier_ratio=c.inlier_ratio,
            innovation_nis=c.innovation_nis, ambiguity_ratio=c.ambiguity_ratio,
            position_jump_m=c.position_jump_m, yaw_jump_rad=c.yaw_jump_rad,
            consecutive_good_samples=self._good_count, reset_count=c.reset_count,
            source=c.source, map_id=c.map_id, map_sha256=c.map_sha256,
            policy_sha256=self.policy.policy_sha256, zone_id=c.zone_id,
        )


def load_localization_policy(path: str, expected_file_sha256: Optional[str] = None) -> LocalizationPolicy:
    """Load and validate a policy once; returned policy and nested zones are immutable."""
    import yaml  # Lazy dependency: importing the pure core does not require PyYAML.

    with open(path, "rb") as stream:
        raw = stream.read()
    digest = hashlib.sha256(raw).hexdigest()
    if expected_file_sha256 is not None and digest != expected_file_sha256:
        raise ValueError("localization policy file hash mismatch")
    document = yaml.safe_load(raw)
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("unsupported localization policy schema")
    thresholds = _mapping(document, "thresholds")
    calibration = _mapping(document, "calibration")
    holdout = _mapping(document, "holdout")
    hashes = _mapping(document, "hashes")
    initialization = _mapping(document, "initialization")
    relocalization = _mapping(document, "relocalization")
    ok_gate = _mapping(document, "ok_gate")
    runtime = _mapping(document, "runtime_boundary")
    authority = _mapping(document, "authority")

    required_hashes = (
        "schema_sha256", "policy_sha256", "map_sha256", "localizer_sha256",
        "localizer_config_sha256", "calibration_dataset_sha256",
        "holdout_dataset_sha256", "threshold_tool_sha256",
    )
    hash_values = {}
    all_hashes_verified = True
    for key in required_hashes:
        record = hashes.get(key)
        if not isinstance(record, dict) or record.get("status") not in ("verified", "candidate", "blocked_unknown"):
            raise ValueError("invalid hashes.%s record" % key)
        value = record.get("value")
        if value is not None and not _SHA256.fullmatch(value):
            raise ValueError("hashes.%s.value must be a lowercase SHA-256 or null" % key)
        hash_values[key] = value
        all_hashes_verified = all_hashes_verified and record.get("status") == "verified"

    if runtime.get("candidate_is_untrusted") is not True or runtime.get("candidate_may_publish_safety") is not False:
        raise ValueError("candidate authority boundary is invalid")
    if runtime.get("independent_guard_is_sole_safety_publisher") is not True:
        raise ValueError("independent guard must be sole localization safety publisher")
    if runtime.get("single_map_to_odom_authority") is not True or runtime.get("guard_may_broadcast_map_to_odom") is not False:
        raise ValueError("TF authority boundary is invalid")
    if any(authority.get(key) is not False for key in (
            "hardware_motion_authorized", "passenger_operation_authorized", "transferable_to_hardware")):
        raise ValueError("software policy cannot grant hardware or passenger authority")

    scan = _mapping(thresholds, "scan_map_residual")
    inliers = _mapping(thresholds, "inlier_fraction")
    selection = _mapping(thresholds, "scan_selection")
    policy_hash = hash_values["policy_sha256"]
    map_hash = hash_values["map_sha256"]
    if policy_hash is None or map_hash is None:
        raise ValueError("runtime policy and map identities must be present")
    values = {
        "map_id": document.get("policy_id"),
        "map_sha256": map_hash,
        "policy_sha256": policy_hash,
        "expected_frame": runtime.get("map_frame"),
        "max_candidate_age_s": thresholds.get("candidate_age_max_s"),
        "max_transform_age_s": thresholds.get("tf_age_max_s"),
        "max_position_std_m": thresholds.get("reported_planar_stddev_max_m"),
        "max_yaw_std_rad": math.radians(thresholds.get("reported_yaw_stddev_max_deg")),
        "max_innovation_nis": thresholds.get("odom_innovation_nis_max"),
        "max_scan_residual_m": scan.get("software_rc_ceiling_m"),
        "min_inlier_ratio": inliers.get("software_rc_floor"),
        "min_ambiguity_ratio": thresholds.get("ambiguity_ratio_min"),
        "scan_candidate_samples": selection.get("candidate_samples"),
        "scan_minimum_selected_points": selection.get("minimum_selected_points"),
        "scan_maximum_selected_points": selection.get("maximum_selected_points"),
        "scan_planar_range_min_m": selection.get("planar_range_min_m"),
        "scan_planar_range_max_m": selection.get("planar_range_max_m"),
        "scan_base_z_min_m": selection.get("base_z_min_m"),
        "scan_base_z_max_m": selection.get("base_z_max_m"),
        "max_position_jump_m": thresholds.get("continuity_planar_max_m"),
        "max_yaw_jump_rad": math.radians(thresholds.get("continuity_yaw_max_deg")),
        "stationary_linear_mps": initialization.get("stationary_linear_speed_below_mps"),
        "stationary_angular_rps": initialization.get("stationary_angular_speed_below_radps"),
        "stationary_duration_s": initialization.get("stationary_hold_s"),
        "initialization_samples": ok_gate.get("consecutive_samples"),
        "initialization_span_s": ok_gate.get("minimum_span_s"),
        "relocalization_samples": relocalization.get("passing_samples"),
        "relocalization_span_s": relocalization.get("minimum_span_s"),
        "degraded_zones": ("DEGRADED", "degraded_stop"),
        "manual_only_zones": ("manual_only",),
        "calibration_fraction": calibration.get("calibration_fraction"),
        "holdout_fraction": calibration.get("holdout_fraction"),
        "holdout_windows": holdout.get("minimum_decorrelated_windows"),
        "holdout_false_ok_windows": holdout.get("maximum_false_ok_windows"),
        "scenario_disjoint": calibration.get("scenario_disjoint"),
        "thresholds_frozen_before_holdout": calibration.get("thresholds_frozen_before_holdout"),
        "absolute_truth_claimed": holdout.get("claim_limit") != "software_evidence_not_hardware_probability",
        "calibration_qualified": (
            document.get("qualification") == "software_rc"
            and all_hashes_verified
            and document.get("status") == "approved_software_rc"
        ),
    }
    return LocalizationPolicy(**values)


def load_policy(path: str, expected_file_sha256: Optional[str] = None) -> LocalizationPolicy:
    """Compatibility alias used by runtime adapters."""
    return load_localization_policy(path, expected_file_sha256)


def _mapping(document: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = document.get(key)
    if not isinstance(value, dict):
        raise ValueError("%s must be a mapping" % key)
    return MappingProxyType(value)


def _finite_between(value: float, low: float, high: float) -> bool:
    return math.isfinite(value) and low <= value <= high


@dataclass(frozen=True)
class OccupancyMap:
    """Immutable occupancy map used by the independent scan scorer."""

    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    occupied: Tuple[Tuple[float, float], ...]
    file_sha256: str
    image_sha256: str
    occupied_cells: frozenset = frozenset()
    occupied_columns_by_row: Tuple[Tuple[int, ...], ...] = ()

    def __post_init__(self) -> None:
        if self.occupied_columns_by_row:
            return
        rows: List[List[int]] = [[] for _ in range(self.height)]
        for row, column in self.occupied_cells:
            if 0 <= row < self.height and 0 <= column < self.width:
                rows[row].append(column)
        object.__setattr__(
            self, "occupied_columns_by_row",
            tuple(tuple(sorted(columns)) for columns in rows),
        )

    def nearest_distance(self, x: float, y: float) -> float:
        if not self.occupied:
            return math.inf
        if not self.occupied_cells:
            return min(math.hypot(x - ox, y - oy) for ox, oy in self.occupied)
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        relative_x, relative_y = x - self.origin_x, y - self.origin_y
        column = int(math.floor((cosine * relative_x + sine * relative_y) / self.resolution))
        map_y = (-sine * relative_x + cosine * relative_y) / self.resolution
        row = self.height - 1 - int(math.floor(map_y))
        maximum_cells = max(1, int(math.ceil(1.0 / self.resolution)))
        minimum_column = column - maximum_cells
        maximum_column = column + maximum_cells
        best = math.inf
        first_row = max(0, row - maximum_cells)
        last_row = min(self.height - 1, row + maximum_cells)
        for candidate_row in range(first_row, last_row + 1):
            columns = self.occupied_columns_by_row[candidate_row]
            insertion = bisect_left(columns, column)
            for index in (insertion - 1, insertion):
                if index < 0 or index >= len(columns):
                    continue
                candidate_column = columns[index]
                if not minimum_column <= candidate_column <= maximum_column:
                    continue
                ox, oy = self._cell_center(candidate_row, candidate_column)
                best = min(best, math.hypot(x - ox, y - oy))
        return best if math.isfinite(best) else 1.0 + self.resolution

    def _cell_center(self, row: int, column: int) -> Tuple[float, float]:
        local_x = (column + 0.5) * self.resolution
        local_y = (self.height - row - 0.5) * self.resolution
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )


@dataclass(frozen=True)
class ScanMetrics:
    residual_m: float
    inlier_ratio: float
    ambiguity_ratio: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def load_occupancy_map(path: str, expected_file_sha256: str,
                       expected_image_sha256: Optional[str] = None) -> OccupancyMap:
    """Load a ROS map YAML and PGM after verifying the pinned byte identities."""

    import yaml

    if not _SHA256.fullmatch(expected_file_sha256 or ""):
        raise ValueError("expected map file SHA-256 is required")
    with open(path, "rb") as stream:
        raw = stream.read()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != expected_file_sha256:
        raise ValueError("occupancy map file hash mismatch")
    document = yaml.safe_load(raw)
    if not isinstance(document, dict):
        raise ValueError("occupancy map metadata must be a mapping")
    resolution = float(document.get("resolution", 0.0))
    origin = document.get("origin")
    if (not math.isfinite(resolution) or resolution <= 0.0 or
            not isinstance(origin, list) or len(origin) != 3 or
            any(not math.isfinite(float(value)) for value in origin)):
        raise ValueError("invalid occupancy map geometry")
    image_path = str(document.get("image", ""))
    if not image_path or os.path.isabs(image_path):
        raise ValueError("occupancy map image must be a relative pinned path")
    image_path = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(path)), image_path))
    map_directory = os.path.realpath(os.path.dirname(path))
    if os.path.commonpath((map_directory, image_path)) != map_directory:
        raise ValueError("occupancy map image escapes map directory")
    with open(image_path, "rb") as stream:
        image_raw = stream.read()
    image_digest = hashlib.sha256(image_raw).hexdigest()
    if expected_image_sha256 is not None and image_digest != expected_image_sha256:
        raise ValueError("occupancy map image hash mismatch")
    width, height, maximum, pixels = _read_pgm(image_raw)
    negate = int(document.get("negate", 0))
    occupied_threshold = float(document.get("occupied_thresh", 0.65))
    if negate not in (0, 1) or not 0.0 <= occupied_threshold <= 1.0:
        raise ValueError("invalid occupancy map thresholds")
    occupied = []
    occupied_cells = set()
    yaw = float(origin[2])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    for row, pixel_row in enumerate(pixels):
        for column, pixel in enumerate(pixel_row):
            probability = pixel / float(maximum) if negate else (maximum - pixel) / float(maximum)
            if probability >= occupied_threshold:
                occupied_cells.add((row, column))
                local_x = (column + 0.5) * resolution
                local_y = (height - row - 0.5) * resolution
                occupied.append((
                    float(origin[0]) + cosine * local_x - sine * local_y,
                    float(origin[1]) + sine * local_x + cosine * local_y,
                ))
    if not occupied:
        raise ValueError("occupancy map contains no occupied cells")
    return OccupancyMap(
        resolution, float(origin[0]), float(origin[1]), yaw, width, height,
        tuple(occupied), digest, image_digest, frozenset(occupied_cells),
    )


def _read_pgm(raw: bytes) -> Tuple[int, int, int, List[List[int]]]:
    """Strictly decode binary/ascii greyscale PGM without image-library state."""

    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(raw) and chr(raw[index]).isspace():
            index += 1
        if index < len(raw) and raw[index:index + 1] == b"#":
            newline = raw.find(b"\n", index)
            if newline < 0:
                raise ValueError("truncated PGM comment")
            index = newline + 1
            continue
        end = index
        while end < len(raw) and not chr(raw[end]).isspace() and raw[end:end + 1] != b"#":
            end += 1
        if end == index:
            raise ValueError("truncated PGM header")
        tokens.append(raw[index:end])
        index = end
    magic = tokens[0]
    try:
        width, height, maximum = (int(value) for value in tokens[1:])
    except ValueError as exc:
        raise ValueError("invalid PGM header") from exc
    if width <= 0 or height <= 0 or not 0 < maximum <= 65535:
        raise ValueError("invalid PGM dimensions")
    count = width * height
    if magic == b"P2":
        body = raw[index:].split()
        try:
            flat = [int(value) for value in body]
        except ValueError as exc:
            raise ValueError("invalid PGM pixels") from exc
    elif magic == b"P5":
        if index >= len(raw) or not chr(raw[index]).isspace():
            raise ValueError("binary PGM header requires a separator")
        index += 1
        if index < len(raw) and raw[index - 1:index + 1] == b"\r\n":
            index += 1
        sample_bytes = 1 if maximum < 256 else 2
        payload = raw[index:]
        if len(payload) != count * sample_bytes:
            raise ValueError("invalid binary PGM length")
        if sample_bytes == 1:
            flat = list(payload)
        else:
            flat = list(struct.unpack(">%dH" % count, payload))
    else:
        raise ValueError("only P2/P5 occupancy maps are supported")
    if len(flat) != count or any(value < 0 or value > maximum for value in flat):
        raise ValueError("invalid PGM pixel range")
    return width, height, maximum, [
        flat[row * width:(row + 1) * width] for row in range(height)
    ]


def uniform_sample_uvs(width: int, height: int,
                       maximum_points: int = 1024) -> Tuple[Tuple[int, int], ...]:
    """Return deterministic, evenly distributed PointCloud2 sample coordinates."""

    if (
        isinstance(width, bool)
        or isinstance(height, bool)
        or isinstance(maximum_points, bool)
        or not isinstance(width, int)
        or not isinstance(height, int)
        or not isinstance(maximum_points, int)
        or width <= 0
        or height <= 0
        or maximum_points <= 0
        or maximum_points > 1024
    ):
        raise ValueError("cloud dimensions and sample bound are invalid")
    point_count = width * height
    sample_count = min(maximum_points, point_count)
    if sample_count == 1:
        indexes = (0,)
    else:
        indexes = tuple(
            int(round(index * (point_count - 1) / float(sample_count - 1)))
            for index in range(sample_count)
        )
    return tuple((index % width, index // width) for index in indexes)


def select_scan_points(points: Sequence[Tuple[float, float, float]],
                       translation: Tuple[float, float, float],
                       quaternion: Tuple[float, float, float, float],
                       policy: LocalizationPolicy) -> Tuple[Tuple[float, float], ...]:
    """Apply the frozen sensor-to-base transform and geometry-only selection."""

    values = tuple(float(value) for point in points for value in point)
    tf_values = tuple(float(value) for value in translation + quaternion)
    if len(values) != len(points) * 3 or not all(math.isfinite(value) for value in values + tf_values):
        raise ValueError("scan or sensor transform contains malformed/non-finite values")
    qx, qy, qz, qw = tf_values[3:]
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("sensor transform quaternion is invalid")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    tx, ty, tz = tf_values[:3]
    qualifying = []
    for point in points:
        x, y, z = (float(value) for value in point)
        # Quaternion rotation matrix, including roll and pitch.
        bx = tx + (1 - 2 * (qy * qy + qz * qz)) * x + 2 * (qx * qy - qz * qw) * y + 2 * (qx * qz + qy * qw) * z
        by = ty + 2 * (qx * qy + qz * qw) * x + (1 - 2 * (qx * qx + qz * qz)) * y + 2 * (qy * qz - qx * qw) * z
        bz = tz + 2 * (qx * qz - qy * qw) * x + 2 * (qy * qz + qx * qw) * y + (1 - 2 * (qx * qx + qy * qy)) * z
        planar_range = math.hypot(bx, by)
        if (policy.scan_planar_range_min_m <= planar_range <= policy.scan_planar_range_max_m
                and policy.scan_base_z_min_m <= bz <= policy.scan_base_z_max_m):
            qualifying.append((bx, by))
    if len(qualifying) < policy.scan_minimum_selected_points:
        raise ValueError("insufficient geometrically selected scan evidence")
    indexes = uniform_sample_uvs(
        len(qualifying), 1, min(policy.scan_maximum_selected_points, len(qualifying))
    )
    return tuple(qualifying[u] for u, _ in indexes)


def transform_points(points: Iterable[Tuple[float, float]], pose: Pose2D) -> Tuple[Tuple[float, float], ...]:
    cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
    return tuple((
        pose.x + cosine * float(x) - sine * float(y),
        pose.y + sine * float(x) + cosine * float(y),
    ) for x, y in points if math.isfinite(float(x)) and math.isfinite(float(y)))


def _hypothesis_score(residual_m: float, inlier_ratio: float,
                      max_scan_residual_m: float, min_inlier_ratio: float) -> float:
    return (
        residual_m / max_scan_residual_m
        + (1.0 - inlier_ratio) / (1.0 - min_inlier_ratio)
    )


def compute_scan_metrics(occupancy_map: OccupancyMap,
                         scan_points: Sequence[Tuple[float, float]],
                         candidate_pose: Pose2D,
                         max_scan_residual_m: float,
                         min_inlier_ratio: float) -> ScanMetrics:
    """Score candidate pose against map; viable alternatives expose ambiguity."""

    if (not scan_points
            or not math.isfinite(max_scan_residual_m) or max_scan_residual_m <= 0.0
            or not math.isfinite(min_inlier_ratio)
            or not 0.0 <= min_inlier_ratio < 1.0):
        return ScanMetrics(math.inf, 0.0, 0.0)

    def score(pose: Pose2D) -> Tuple[float, float]:
        distances = [
            occupancy_map.nearest_distance(x, y)
            for x, y in transform_points(scan_points, pose)
        ]
        ordered = sorted(distances)
        middle = len(ordered) // 2
        residual_m = (ordered[middle] if len(ordered) % 2
                      else (ordered[middle - 1] + ordered[middle]) / 2.0)
        inlier_ratio = (
            sum(value <= max_scan_residual_m for value in distances) / float(len(distances))
        )
        return residual_m, inlier_ratio

    residual, inliers = score(candidate_pose)
    candidate_score = _hypothesis_score(
        residual, inliers, max_scan_residual_m, min_inlier_ratio
    )
    alternatives = []
    for dx, dy, dyaw in (
            (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0), (0.0, 0.0, math.radians(20.0)),
            (0.0, 0.0, math.radians(-20.0))):
        alt_residual, alt_inliers = score(Pose2D(
            candidate_pose.x + dx, candidate_pose.y + dy, candidate_pose.yaw + dyaw
        ))
        if alt_residual <= max_scan_residual_m and alt_inliers >= min_inlier_ratio:
            alternatives.append(_hypothesis_score(
                alt_residual, alt_inliers,
                max_scan_residual_m, min_inlier_ratio,
            ))

    ambiguity = 1_000_000.0
    if alternatives:
        second_best_score = min(alternatives)
        if second_best_score <= candidate_score:
            ambiguity = 0.0
        else:
            ambiguity = min(
                second_best_score / max(candidate_score, 1e-12),
                1_000_000.0,
            )
    return ScanMetrics(residual, inliers, ambiguity)


def planar_covariance(values: Sequence[float]) -> Tuple[float, ...]:
    """Extract x/y/yaw covariance from a ROS 6x6 row-major covariance."""

    if len(values) != 36:
        raise ValueError("pose covariance must have 36 elements")
    indices = (0, 1, 5, 6, 7, 11, 30, 31, 35)
    result = tuple(float(values[index]) for index in indices)
    if any(not math.isfinite(value) for value in result):
        raise ValueError("pose covariance must be finite")
    return result


def add_planar_covariances(first: Sequence[float],
                           second: Sequence[float]) -> Tuple[float, ...]:
    if len(first) != 9 or len(second) != 9:
        raise ValueError("planar covariances must have nine elements")
    result = tuple(float(a) + float(b) for a, b in zip(first, second))
    if any(not math.isfinite(value) for value in result):
        raise ValueError("planar covariance must be finite")
    return result


def covariance_nis(dx: float, dy: float, dyaw: float,
                   covariance: Sequence[float]) -> float:
    """Mahalanobis distance for a planar innovation, rejecting non-PD matrices."""

    if len(covariance) != 9:
        return math.inf
    a, b, c, d, e, f, g, h, i = (float(value) for value in covariance)
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if (any(not math.isfinite(value) for value in covariance) or
            abs(b - d) > 1e-9 or abs(c - g) > 1e-9 or abs(f - h) > 1e-9 or
            a <= 0.0 or a * e - b * d <= 0.0 or determinant <= 0.0):
        return math.inf
    inverse = (
        (e * i - f * h) / determinant, (c * h - b * i) / determinant, (b * f - c * e) / determinant,
        (f * g - d * i) / determinant, (a * i - c * g) / determinant, (c * d - a * f) / determinant,
        (d * h - e * g) / determinant, (b * g - a * h) / determinant, (a * e - b * d) / determinant,
    )
    vector = (dx, dy, dyaw)
    return sum(vector[row] * inverse[row * 3 + column] * vector[column]
               for row in range(3) for column in range(3))


def angle_delta(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))
def transform_planar_pose(transform, pose: Pose2D) -> Pose2D:
    """Apply a map<-odom transform to an odometry-frame planar pose."""
    translation = transform.transform.translation
    yaw = quaternion_yaw(transform.transform.rotation)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return Pose2D(
        translation.x + cosine * pose.x - sine * pose.y,
        translation.y + sine * pose.x + cosine * pose.y,
        angle_delta(yaw + pose.yaw, 0.0),
    )


def rotate_planar_covariance(covariance: Sequence[float], yaw: float) -> Tuple[float, ...]:
    """Express an odometry planar covariance in the map frame."""
    if len(covariance) != 9:
        raise ValueError("planar covariance must have nine elements")
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = (
        cosine, -sine, 0.0,
        sine, cosine, 0.0,
        0.0, 0.0, 1.0,
    )
    first = tuple(
        sum(rotation[row * 3 + inner] * covariance[inner * 3 + column]
            for inner in range(3))
        for row in range(3) for column in range(3)
    )
    return tuple(
        sum(first[row * 3 + inner] * rotation[column * 3 + inner]
            for inner in range(3))
        for row in range(3) for column in range(3)
    )


class IndependentEvidenceTracker:
    """Derive temporal/odom evidence without consuming localizer diagnostics."""

    def __init__(self):
        self.last_candidate: Optional[Pose2D] = None
        self.last_odom: Optional[Pose2D] = None

    def derive(self, candidate: Pose2D, odom: Pose2D,
               covariance: Sequence[float]) -> Tuple[float, float, float, float]:
        if self.last_candidate is None or self.last_odom is None:
            self.last_candidate, self.last_odom = candidate, odom
            return 0.0, 0.0, 0.0, 0.0
        candidate_delta = Pose2D(
            candidate.x - self.last_candidate.x,
            candidate.y - self.last_candidate.y,
            angle_delta(candidate.yaw, self.last_candidate.yaw),
        )
        odom_delta = Pose2D(
            odom.x - self.last_odom.x,
            odom.y - self.last_odom.y,
            angle_delta(odom.yaw, self.last_odom.yaw),
        )
        self.last_candidate, self.last_odom = candidate, odom
        position_jump = math.hypot(candidate_delta.x, candidate_delta.y)
        odom_distance = math.hypot(odom_delta.x, odom_delta.y)
        nis = covariance_nis(
            candidate_delta.x - odom_delta.x,
            candidate_delta.y - odom_delta.y,
            angle_delta(candidate_delta.yaw, odom_delta.yaw),
            covariance,
        )
        return position_jump, abs(candidate_delta.yaw), odom_distance, nis


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def has_explicit_synthetic_qualification(path: str) -> bool:
    """Recognize the only qualification allowed to clear in this software RC."""

    import yaml

    with open(path, "rb") as stream:
        document = yaml.safe_load(stream.read())
    if not isinstance(document, dict):
        return False
    provenance = document.get("provenance")
    simulation = document.get("simulation_guard")
    return (
        document.get("qualification_scope") ==
        "gazebo_synthetic_ground_truth_calibration_only"
        and isinstance(provenance, dict)
        and provenance.get("evidence_level") == "simulation_only"
        and isinstance(simulation, dict)
        and simulation.get("may_report_ok") is True
        and simulation.get("scope") == "gazebo_simulation_only"
    )


class CandidateSequenceTracker:
    """Keep publication identity coupled to candidates, never watchdog ticks."""

    def __init__(self):
        self.last_observed = 0
        self.last_accepted = 0

    def observe(self, sequence: int) -> bool:
        sequence = int(sequence)
        self.last_observed = sequence
        if sequence == 0 or sequence <= self.last_accepted:
            return False
        self.last_accepted = sequence
        return True

    def watchdog_sequence(self) -> int:
        return self.last_observed


def _published_diagnostic(value: float) -> float:
    """Return the ABI sentinel when a numeric diagnostic is unavailable."""
    value = float(value)
    return value if math.isfinite(value) else -1.0


class LocalizationGuardNode:
    """ROS1 I/O boundary. It publishes confidence only and never TF or commands."""

    STATUS_TOPIC = "/localization/status"
    SIGNAL_TOPIC = "/safety/localization"
    CANDIDATE_TOPIC = "/localization/candidate"
    CLOUD_TOPIC = "/sensors/lidar/points"
    ODOM_TOPIC = "/odom"
    TF_AUTHORITY_WAIT_S = 0.05
    MAP_TO_ODOM_AUTHORITY = "/localization_adapter"

    def __init__(self):
        import rospy
        import tf2_ros
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Bool
        from tf2_msgs.msg import TFMessage
        from wheelchair_interfaces.msg import LocalizationCandidate, LocalizationStatus, SafetySignal

        self.rospy = rospy
        self.LocalizationStatus = LocalizationStatus
        self.SafetySignal = SafetySignal
        policy_path = str(rospy.get_param("~policy_file"))
        policy_hash = str(rospy.get_param("~expected_policy_file_sha256"))
        map_path = str(rospy.get_param("~map_file"))
        map_hash = str(rospy.get_param("~expected_map_file_sha256"))
        image_hash = str(rospy.get_param("~expected_map_image_sha256", "")) or None
        self.initialization_attempt_timeout_s = float(
            rospy.get_param("~initialization_attempt_timeout_s")
        )
        if self.initialization_attempt_timeout_s != 30.0:
            raise ValueError("~initialization_attempt_timeout_s must be the fixed 30 seconds")
        self._monotonic = time.monotonic
        self.policy = load_localization_policy(policy_path, policy_hash)
        if (self.policy.calibration_qualified and
                not has_explicit_synthetic_qualification(policy_path)):
            self.policy = replace(self.policy, calibration_qualified=False)
        self.map = load_occupancy_map(map_path, map_hash, image_hash)
        if self.policy.map_sha256 not in (self.map.file_sha256, self.map.image_sha256):
            raise ValueError("policy map identity does not match pinned occupancy map")
        self.core = LocalizationGuardCore(self.policy)
        self.tracker = IndependentEvidenceTracker()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.status_pub = rospy.Publisher(self.STATUS_TOPIC, LocalizationStatus, queue_size=1)
        self.signal_pub = rospy.Publisher(self.SIGNAL_TOPIC, SafetySignal, queue_size=1)
        self.cloud = self.odom = self.initial_pose = None
        self.initialization_attempt_pose = None
        self.initialization_attempt_deadline = None
        self.initialization_request_consumed = False
        self.mission_canceled = False
        self.relocalization_requested = False
        self.tf_authorities = {}
        self.tf_authority_condition = threading.Condition()
        self.stationary_since = None
        self.last_candidate_receipt = None
        self.last_candidate_source_stamp = None
        self.candidate_sequences = CandidateSequenceTracker()
        self.last_cloud_stamp = None
        self.last_odom_stamp = None
        self.output_sequence = 0
        rospy.Subscriber(self.CANDIDATE_TOPIC, LocalizationCandidate,
                         self._candidate_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(self.CLOUD_TOPIC, PointCloud2,
                         self._cloud_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(self.ODOM_TOPIC, Odometry,
                         self._odom_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                         self._initial_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/safety/mission_cancelled", Bool,
                         self._mission_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/localization/relocalize", Bool,
                         self._relocalization_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/tf", TFMessage, self._tf_callback, queue_size=1, tcp_nodelay=True)
        self.watchdog = rospy.Timer(rospy.Duration(0.05), self._watchdog_callback)

    def _cloud_callback(self, message):
        stamp = message.header.stamp.to_sec()
        if (not math.isfinite(stamp) or
                (self.last_cloud_stamp is not None and stamp <= self.last_cloud_stamp)):
            self._publish_stop(
                self.rospy.Time.now().to_sec(),
                REASON_LOCALIZATION | REASON_CLOCK | REASON_CORRUPT_DATA,
            )
            return
        self.last_cloud_stamp = stamp
        self.cloud = (message, self.rospy.Time.now().to_sec())

    def _odom_callback(self, message):
        now = self.rospy.Time.now().to_sec()
        stamp = message.header.stamp.to_sec()
        if (not math.isfinite(stamp) or
                (self.last_odom_stamp is not None and stamp <= self.last_odom_stamp)):
            self._publish_stop(
                now, REASON_LOCALIZATION | REASON_CLOCK | REASON_CORRUPT_DATA,
            )
            return
        self.last_odom_stamp = stamp
        self.odom = (message, now)
        moving = (abs(message.twist.twist.linear.x) >= self.policy.stationary_linear_mps or
                  abs(message.twist.twist.angular.z) >= self.policy.stationary_angular_rps)
        if moving:
            self.stationary_since = None
            self._clear_initialization_attempt()
        elif self.stationary_since is None:
            self.stationary_since = now

    def _clear_initialization_attempt(self):
        self.initialization_attempt_pose = None
        self.initialization_attempt_deadline = None

    def _initialization_attempt_active(self):
        if self.initialization_attempt_deadline is None:
            return False
        if self._monotonic() >= self.initialization_attempt_deadline:
            self._clear_initialization_attempt()
            return False
        return True

    def _clear_initialization_attempt_on_state_exit(self, state):
        if state not in (UNINITIALIZED, INITIALIZING):
            self._clear_initialization_attempt()

    def _initial_pose_fresh(self, now):
        if self.core.state in (UNINITIALIZED, INITIALIZING):
            return self._initialization_attempt_active()
        return bool(
            self.initial_pose and
            now - self.initial_pose[1] <= (
                self.policy.relocalization_span_s + self.policy.max_candidate_age_s
            ) and
            self.initial_pose[0].header.frame_id == self.policy.expected_frame
        )


    def _stationary_hold_satisfied(self, now):
        return (
            self.stationary_since is not None and
            now - self.stationary_since >= self.policy.stationary_duration_s
        )

    def _initial_pose_callback(self, message):
        now = self.rospy.Time.now().to_sec()
        age = now - message.header.stamp.to_sec()
        valid = (
            message.header.frame_id == self.policy.expected_frame and
            -self.policy.max_future_skew_s <= age <= self.policy.max_candidate_age_s
        )
        if not valid:
            return
        if self.core.state in (UNINITIALIZED, INITIALIZING):
            if self.initialization_request_consumed:
                return
            self.initial_pose = (message, now)
            if self.mission_canceled and self._stationary_hold_satisfied(now):
                self.initialization_request_consumed = True
                self.initialization_attempt_pose = (message, self._monotonic())
                self.initialization_attempt_deadline = (
                    self.initialization_attempt_pose[1] +
                    self.initialization_attempt_timeout_s
                )
            return
        self.initial_pose = (message, now)

    def _mission_callback(self, message):
        self.mission_canceled = bool(message.data)

    def _relocalization_callback(self, message):
        self.relocalization_requested = bool(message.data)

    def _tf_callback(self, message):
        caller = getattr(message, "_connection_header", {}).get("callerid", "")
        if not caller:
            return
        with self.tf_authority_condition:
            for transform in message.transforms:
                if (transform.header.frame_id == "map" and
                        transform.child_frame_id == "odom"):
                    self.tf_authorities[caller] = True
            self.tf_authority_condition.notify_all()

    def _active_tf_authorities(self, _now):
        # Authority identity is latched: a duplicate must not disappear merely
        # because one conflicting broadcaster goes quiet.
        with self.tf_authority_condition:
            return self._tf_authority_count()

    def _tf_authority_count(self):
        return 1 if (len(self.tf_authorities) == 1 and
                     self.MAP_TO_ODOM_AUTHORITY in self.tf_authorities) else 0

    def _await_single_tf_authority(self):
        """Bound startup callback skew without accepting absent or duplicate TF."""
        deadline = self._monotonic() + self.TF_AUTHORITY_WAIT_S
        with self.tf_authority_condition:
            while self.MAP_TO_ODOM_AUTHORITY not in self.tf_authorities:
                remaining = deadline - self._monotonic()
                if remaining <= 0.0:
                    break
                self.tf_authority_condition.wait(remaining)
            return self._tf_authority_count()

    def _lookup(self, target, source, stamp):
        transform = self.tf_buffer.lookup_transform(
            target, source, stamp, self.rospy.Duration(0.05)
        )
        age = self.rospy.Time.now().to_sec() - transform.header.stamp.to_sec()
        return transform, age

    def _candidate_callback(self, message):
        now = self.rospy.Time.now().to_sec()
        self.last_candidate_receipt = now
        self.last_candidate_source_stamp = message.pose.header.stamp.to_sec()
        candidate_sequence = int(message.pose.header.seq)
        if not self.candidate_sequences.observe(candidate_sequence):
            self._publish_stop(
                now,
                REASON_LOCALIZATION | REASON_CLOCK | REASON_CORRUPT_DATA,
                candidate_sequence,
            )
            return
        self.core.adopt_loss_reset_baseline(message.reset_count)
        if self.core.consume_recovery_epoch(
                message.reset_count, self.relocalization_requested):
            self._begin_input_epoch()
        try:
            evidence = self._derive_evidence(message, now)
        except Exception as exc:
            self.rospy.logwarn_throttle(1.0, "localization evidence unavailable: %s", exc)
            self._publish_stop(
                now, REASON_LOCALIZATION | REASON_INPUT_UNKNOWN, candidate_sequence
            )
            return
        evaluation_now = self.rospy.Time.now().to_sec()
        if evaluation_now - self.last_candidate_receipt > self.policy.max_candidate_age_s:
            self._publish_stop(
                evaluation_now,
                REASON_LOCALIZATION | REASON_SENSOR_STALE,
                candidate_sequence,
            )
            return
        result = self.core.evaluate(evidence, evaluation_now)
        self._clear_initialization_attempt_on_state_exit(result.state)
        self._publish_result(result, evidence.stamp_s)

    def _begin_input_epoch(self):
        """Forget only per-epoch chronology after an explicit recovery transition."""
        self.last_cloud_stamp = None
        self.last_odom_stamp = None
        self.cloud = None
        self.odom = None

    def _derive_evidence(self, message, now):
        from sensor_msgs import point_cloud2

        if self.cloud is None or self.odom is None:
            raise ValueError("cloud/odom missing")
        cloud, cloud_receipt = self.cloud
        odom, odom_receipt = self.odom
        stamp = message.pose.header.stamp
        stamp_s = stamp.to_sec()
        cloud_stamp_s = cloud.header.stamp.to_sec()
        odom_stamp_s = odom.header.stamp.to_sec()
        cloud_age = now - cloud_stamp_s
        odom_source_age = now - odom_stamp_s
        if (not -self.policy.max_future_skew_s <= cloud_age <= self.policy.max_candidate_age_s or
                not -self.policy.max_future_skew_s <= odom_source_age <= self.policy.max_odom_age_s or
                now - cloud_receipt > self.policy.max_candidate_age_s or
                now - odom_receipt > self.policy.max_odom_age_s or
                abs(cloud_stamp_s - stamp_s) > self.policy.max_candidate_age_s or
                abs(odom_stamp_s - stamp_s) > self.policy.max_odom_age_s):
            raise ValueError("cloud/odom stale, future, or unsynchronized")
        if odom.header.frame_id != "odom" or odom.child_frame_id != "base_footprint":
            raise ValueError("odometry frame contract is invalid")
        sensor_tf, _ = self._lookup("base_link", cloud.header.frame_id, cloud.header.stamp)
        map_to_odom, _ = self._lookup("map", "odom", stamp)
        tf_authority_count = self._await_single_tf_authority()
        now = self.rospy.Time.now().to_sec()
        cloud_age = now - cloud_stamp_s
        odom_source_age = now - odom_stamp_s
        if (not -self.policy.max_future_skew_s <= cloud_age <= self.policy.max_candidate_age_s or
                not -self.policy.max_future_skew_s <= odom_source_age <= self.policy.max_odom_age_s or
                now - cloud_receipt > self.policy.max_candidate_age_s or
                now - odom_receipt > self.policy.max_odom_age_s):
            raise ValueError("cloud/odom stale after TF authority wait")
        transform_age = now - map_to_odom.header.stamp.to_sec()
        translation = sensor_tf.transform.translation
        rotation = sensor_tf.transform.rotation
        candidate_pose = Pose2D(
            message.pose.pose.pose.position.x,
            message.pose.pose.pose.position.y,
            quaternion_yaw(message.pose.pose.pose.orientation),
        )
        uvs = uniform_sample_uvs(
            int(cloud.width), int(cloud.height), self.policy.scan_candidate_samples
        )
        sample_count = len(uvs)
        raw_points = tuple(point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=False, uvs=uvs
        ))
        if len(raw_points) != sample_count:
            raise ValueError("sampled cloud is malformed")
        points = select_scan_points(
            raw_points,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            self.policy,
        )
        scan = compute_scan_metrics(
            self.map, points, candidate_pose,
            self.policy.max_scan_residual_m, self.policy.min_inlier_ratio,
        )
        covariance = planar_covariance(message.pose.pose.covariance)
        odom_covariance = rotate_planar_covariance(
            planar_covariance(odom.pose.covariance),
            quaternion_yaw(map_to_odom.transform.rotation),
        )
        innovation_covariance = add_planar_covariances(covariance, odom_covariance)
        map_odom_pose = transform_planar_pose(
            map_to_odom,
            Pose2D(
                odom.pose.pose.position.x, odom.pose.pose.position.y,
                quaternion_yaw(odom.pose.pose.orientation),
            ),
        )
        position_jump, yaw_jump, odom_delta, nis = self.tracker.derive(
            candidate_pose, map_odom_pose, innovation_covariance,
        )
        position_std = math.sqrt(max(covariance[0], covariance[4]))
        yaw_std = math.sqrt(covariance[8])
        initial_fresh = self._initial_pose_fresh(now)
        stationary_duration = 0.0 if self.stationary_since is None else now - self.stationary_since
        return CandidateEvidence(
            stamp_s=stamp_s, source=message.source, raw_state=message.raw_state,
            reset_count=message.reset_count, map_id=message.map_id,
            map_sha256=message.map_sha256, policy_sha256=self.policy.policy_sha256,
            raw_score=message.raw_score, position_std_m=position_std,
            yaw_std_rad=yaw_std, scan_residual_m=scan.residual_m,
            inlier_ratio=scan.inlier_ratio, innovation_nis=nis,
            ambiguity_ratio=scan.ambiguity_ratio,
            frame_id=message.pose.header.frame_id,
            transform_age_s=transform_age, odom_age_s=now - odom_receipt,
            position_jump_m=position_jump, yaw_jump_rad=yaw_jump,
            covariance_planar=covariance,
            linear_speed_mps=odom.twist.twist.linear.x,
            angular_speed_rps=odom.twist.twist.angular.z,
            stationary_duration_s=stationary_duration,
            tf_authority_count=tf_authority_count,
            odom_delta_m=odom_delta, initial_pose_fresh=initial_fresh,
            mission_canceled=self.mission_canceled,
            relocalization_requested=self.relocalization_requested,
            relocalization_jump_evidence=(
                self.relocalization_requested and
                (position_jump > self.policy.max_position_jump_m or
                 yaw_jump > self.policy.max_yaw_jump_rad)
            ),
        )

    def _watchdog_callback(self, _event):
        self._initialization_attempt_active()
        now = self.rospy.Time.now().to_sec()
        if (self.last_candidate_receipt is None or
                now - self.last_candidate_receipt > self.policy.max_candidate_age_s):
            self._publish_stop(
                now,
                REASON_LOCALIZATION | REASON_SENSOR_STALE,
                self.candidate_sequences.watchdog_sequence(),
            )

    def _next_output_sequence(self):
        self.output_sequence = getattr(self, "output_sequence", 0) + 1
        return self.output_sequence

    def _publish_result(self, result, candidate_stamp_s):
        status = self.LocalizationStatus()
        status.header.stamp = self.rospy.Time.from_sec(candidate_stamp_s)
        status.header.frame_id = self.policy.expected_frame
        status.evaluation_stamp = self.rospy.Time.from_sec(result.evaluation_stamp_s)
        for field in (
                "state", "reset_count", "map_id", "map_sha256", "zone_id",
                "consecutive_good_samples", "independent_check_passed"):
            setattr(status, field, getattr(result, field))
        for field in (
                "position_std_m", "yaw_std_rad", "scan_residual_m", "inlier_ratio",
                "innovation_nis", "ambiguity_ratio", "position_jump_m", "yaw_jump_rad"):
            setattr(status, field, _published_diagnostic(getattr(result, field)))
        status.sequence = self._next_output_sequence()
        status.reason_mask = result.safety_reason_mask
        status.source = "localization_guard"
        status.policy_sha256 = result.policy_sha256
        status.pose_age_s = _published_diagnostic(result.pose_age_s)
        status.transform_age_s = _published_diagnostic(result.transform_age_s)
        signal = self.SafetySignal()
        signal.header.stamp = status.evaluation_stamp
        signal.header.frame_id = status.header.frame_id
        signal.sequence = status.sequence
        signal.state = result.safety_state
        signal.reason_mask = status.reason_mask
        signal.source = status.source
        signal.policy_sha256 = status.policy_sha256
        self.status_pub.publish(status)
        self.signal_pub.publish(signal)

    def _publish_stop(self, now, reason, _candidate_sequence=None):
        reason |= REASON_LOCALIZATION
        cold_start = (
            self.core.sequence == 0 and
            self.core.state in (UNINITIALIZED, INITIALIZING)
        )
        if not cold_start:
            self.core.force_loss()
        source_stamp_s = getattr(self, "last_candidate_source_stamp", None)
        if source_stamp_s is None or not math.isfinite(source_stamp_s):
            source_stamp_s = 0.0
        source_stamp = self.rospy.Time.from_sec(source_stamp_s)
        evaluation_stamp = self.rospy.Time.from_sec(now)
        status = self.LocalizationStatus()
        status.header.stamp = source_stamp
        status.header.frame_id = self.policy.expected_frame
        status.evaluation_stamp = evaluation_stamp
        status.sequence = self._next_output_sequence()
        status.state = self.core.state
        status.reason_mask = reason
        status.source = "localization_guard"
        status.policy_sha256 = self.policy.policy_sha256
        status.pose_age_s = -1.0
        status.transform_age_s = -1.0
        status.independent_check_passed = False
        signal = self.SafetySignal()
        signal.header.stamp = status.evaluation_stamp
        signal.header.frame_id = status.header.frame_id
        signal.sequence = status.sequence
        signal.state = SAFETY_STOP
        signal.reason_mask = status.reason_mask
        signal.source = status.source
        signal.policy_sha256 = status.policy_sha256
        self.status_pub.publish(status)
        self.signal_pub.publish(signal)


def main() -> None:
    import rospy

    rospy.init_node("localization_guard")
    LocalizationGuardNode()
    rospy.spin()


if __name__ == "__main__":
    main()
