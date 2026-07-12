"""Pure, fail-closed speed-cap policy for simulation.

The core consumes values already extracted from the frozen ROS interfaces.  It
has no ROS or wall-clock dependency; callers supply a monotonic evaluation
stamp so replaying the same sequence produces the same result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Mapping, Optional


class SpeedPolicyError(ValueError):
    """The closed policy configuration is invalid."""


def _number(value: Any, name: str, minimum: float, strictly_positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SpeedPolicyError("%s must be numeric" % name)
    result = float(value)
    if not math.isfinite(result) or result < minimum or (strictly_positive and result <= minimum):
        raise SpeedPolicyError("%s is outside its finite bounds" % name)
    return result


def _sha256(value: Any, name: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise SpeedPolicyError("%s must be a lowercase SHA-256" % name)
    return value


@dataclass(frozen=True)
class SpeedPolicyConfig:
    schema_version: int
    policy_id: str
    qualification: str
    hardware_motion_authorized: bool
    passenger_operation_authorized: bool
    max_authority_cap_mps: float
    sidewalk_cap_mps: float
    road_cap_mps: float
    simulation_unsurveyed_cap_mps: float
    localization_uncertain_confidence: float
    localization_stop_confidence: float
    localization_uncertain_cap_mps: float
    lateral_acceleration_max_mps2: float
    curvature_epsilon_inv_m: float
    slope_slow_cap_mps: float
    collision_caution_cap_mps: float
    collision_caution_ttc_s: float
    collision_stop_ttc_margin_s: float
    minimum_deceleration_mps2: float
    driver_latency_s: float
    pipeline_budget_s: float
    acceleration_limit_mps2: float
    deceleration_limit_mps2: float
    jerk_limit_mps3: float
    evidence_ttl_s: float
    slope_policy_sha256: str
    collision_policy_sha256: str
    localization_policy_sha256: str

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "SpeedPolicyConfig":
        """Build a config while rejecting missing, unknown, or unsafe fields."""
        if not isinstance(raw, Mapping):
            raise SpeedPolicyError("policy must be a mapping")
        expected = {item.name for item in fields(cls)}
        missing, unknown = expected - set(raw), set(raw) - expected
        if missing or unknown:
            raise SpeedPolicyError("closed policy fields mismatch; missing=%s unknown=%s" %
                                   (sorted(missing), sorted(unknown)))
        if (isinstance(raw["schema_version"], bool) or not isinstance(raw["schema_version"], int) or
                raw["schema_version"] != 1 or raw["qualification"] != "simulation_only"):
            raise SpeedPolicyError("only schema version 1 simulation policy is supported")
        if not isinstance(raw["policy_id"], str) or not raw["policy_id"]:
            raise SpeedPolicyError("policy_id must be nonempty")
        if raw["hardware_motion_authorized"] is not False or raw["passenger_operation_authorized"] is not False:
            raise SpeedPolicyError("speed policy cannot grant hardware or passenger authority")

        values = dict(raw)
        positive = (
            "max_authority_cap_mps", "sidewalk_cap_mps", "road_cap_mps",
            "simulation_unsurveyed_cap_mps", "localization_uncertain_cap_mps",
            "lateral_acceleration_max_mps2",
            "curvature_epsilon_inv_m", "slope_slow_cap_mps", "collision_caution_cap_mps",
            "collision_caution_ttc_s", "minimum_deceleration_mps2", "acceleration_limit_mps2",
            "deceleration_limit_mps2", "jerk_limit_mps3", "evidence_ttl_s",
        )
        for name in positive:
            values[name] = _number(raw[name], name, 0.0, strictly_positive=True)
        for name in ("collision_stop_ttc_margin_s", "driver_latency_s", "pipeline_budget_s"):
            values[name] = _number(raw[name], name, 0.0)
        for name in ("localization_uncertain_confidence", "localization_stop_confidence"):
            values[name] = _number(raw[name], name, 0.0)
            if values[name] > 1.0:
                raise SpeedPolicyError("%s must not exceed one" % name)
        if values["localization_stop_confidence"] >= values["localization_uncertain_confidence"]:
            raise SpeedPolicyError("localization stop confidence must be below uncertain confidence")
        cap_names = ("sidewalk_cap_mps", "road_cap_mps", "simulation_unsurveyed_cap_mps",
                     "localization_uncertain_cap_mps", "slope_slow_cap_mps",
                     "collision_caution_cap_mps")
        if any(values[name] > values["max_authority_cap_mps"] for name in cap_names):
            raise SpeedPolicyError("a policy cap cannot exceed authority")
        if values["simulation_unsurveyed_cap_mps"] > 0.20:
            raise SpeedPolicyError("simulation unsurveyed cap must not exceed 0.20 m/s")
        for name in ("slope_policy_sha256", "collision_policy_sha256", "localization_policy_sha256"):
            values[name] = _sha256(raw[name], name)
        return cls(**values)


@dataclass(frozen=True)
class SpeedEvidence:
    segment_cap_mps: float
    zone_cap_mps: float
    hard_cap_mps: float
    curvature_inv_m: float
    zone: str
    localization_state: int
    localization_confidence: float
    localization_policy_sha256: str
    slope_state: int
    pitch_rad: float
    slope_recommended_cap_mps: float
    slope_policy_sha256: str
    collision_state: int
    collision_ttc_s: float
    collision_recommended_cap_mps: float
    collision_policy_sha256: str
    odometry_speed_mps: float
    evidence_stamp_monotonic_s: float
    now_monotonic_s: float


class SpeedPolicyCore:
    """Stateful nominal cap trajectory generator.

    Message constants are used directly: LocalizationStatus OK=2/DEGRADED=3,
    SlopeStatus CLEAR=1/SLOW=2/STOP=3, and CollisionStatus
    CLEAR=1/CAUTION=2/STOP=3.  Every unknown state fails closed.
    """

    def __init__(self, config: SpeedPolicyConfig):
        if not isinstance(config, SpeedPolicyConfig):
            raise TypeError("config must be SpeedPolicyConfig")
        self.config = config
        self._last_time: Optional[float] = None
        self._last_cap = 0.0
        self._last_acceleration = 0.0

    def _stop(self, now: float) -> float:
        self._last_time = now if math.isfinite(now) else None
        self._last_cap = 0.0
        self._last_acceleration = 0.0
        return 0.0

    def evaluate(self, evidence: SpeedEvidence) -> float:
        """Return a finite nonnegative cap; invalid or hazardous input is zero."""
        if not isinstance(evidence, SpeedEvidence):
            return self._stop(float("nan"))
        numeric = (
            evidence.segment_cap_mps, evidence.zone_cap_mps, evidence.hard_cap_mps,
            evidence.curvature_inv_m, evidence.localization_confidence, evidence.pitch_rad,
            evidence.slope_recommended_cap_mps, evidence.collision_ttc_s,
            evidence.collision_recommended_cap_mps, evidence.odometry_speed_mps,
            evidence.evidence_stamp_monotonic_s, evidence.now_monotonic_s,
        )
        if not all(math.isfinite(value) for value in numeric):
            return self._stop(evidence.now_monotonic_s)
        if (evidence.segment_cap_mps < 0.0 or evidence.zone_cap_mps < 0.0 or
                evidence.hard_cap_mps < 0.0 or evidence.localization_confidence < 0.0 or
                evidence.localization_confidence > 1.0 or
                evidence.slope_recommended_cap_mps < 0.0 and
                evidence.slope_recommended_cap_mps != -1.0 or
                evidence.collision_recommended_cap_mps < 0.0 and
                evidence.collision_recommended_cap_mps != -1.0 or
                evidence.odometry_speed_mps < 0.0):
            return self._stop(evidence.now_monotonic_s)
        if max(evidence.segment_cap_mps, evidence.zone_cap_mps,
               evidence.hard_cap_mps) > self.config.max_authority_cap_mps:
            return self._stop(evidence.now_monotonic_s)
        if (evidence.slope_policy_sha256 != self.config.slope_policy_sha256 or
                evidence.collision_policy_sha256 != self.config.collision_policy_sha256 or
                evidence.localization_policy_sha256 != self.config.localization_policy_sha256):
            return self._stop(evidence.now_monotonic_s)
        age = evidence.now_monotonic_s - evidence.evidence_stamp_monotonic_s
        if age < 0.0 or age > self.config.evidence_ttl_s:
            return self._stop(evidence.now_monotonic_s)
        if self._last_time is not None and evidence.now_monotonic_s <= self._last_time:
            return self._stop(evidence.now_monotonic_s)

        # UNKNOWN, LOST, relocalizing, explicit hazard STOP, and reverse motion
        # bypass all comfort shaping.
        if evidence.localization_state not in (2, 3) or evidence.slope_state not in (1, 2, 3):
            return self._stop(evidence.now_monotonic_s)
        if evidence.collision_state not in (1, 2, 3):
            return self._stop(evidence.now_monotonic_s)
        if evidence.slope_state == 3 or evidence.collision_state == 3:
            return self._stop(evidence.now_monotonic_s)
        if ((evidence.slope_state == 2 and evidence.slope_recommended_cap_mps < 0.0) or
                (evidence.collision_state == 2 and
                 evidence.collision_recommended_cap_mps < 0.0)):
            return self._stop(evidence.now_monotonic_s)

        authority_limit = min(self.config.max_authority_cap_mps, evidence.hard_cap_mps,
                              evidence.segment_cap_mps, evidence.zone_cap_mps)
        if evidence.zone == "sidewalk":
            zone_type_cap = self.config.sidewalk_cap_mps
        elif evidence.zone == "road":
            zone_type_cap = self.config.road_cap_mps
        elif evidence.zone == "simulation_unsurveyed":
            zone_type_cap = self.config.simulation_unsurveyed_cap_mps
        else:
            return self._stop(evidence.now_monotonic_s)
        curvature_cap = math.sqrt(
            self.config.lateral_acceleration_max_mps2 /
            max(abs(evidence.curvature_inv_m), self.config.curvature_epsilon_inv_m)
        )
        caps = [authority_limit, zone_type_cap, curvature_cap]
        if evidence.slope_recommended_cap_mps >= 0.0:
            caps.append(evidence.slope_recommended_cap_mps)
        if evidence.collision_recommended_cap_mps >= 0.0:
            caps.append(evidence.collision_recommended_cap_mps)

        if evidence.localization_state == 3 or evidence.localization_confidence < self.config.localization_uncertain_confidence:
            if evidence.localization_confidence <= self.config.localization_stop_confidence:
                return self._stop(evidence.now_monotonic_s)
            caps.append(self.config.localization_uncertain_cap_mps)

        pitch_deg = math.degrees(evidence.pitch_rad)
        if pitch_deg < -7.0 or pitch_deg > 10.0:
            return self._stop(evidence.now_monotonic_s)
        if evidence.slope_state == 2 or pitch_deg < -5.0 or pitch_deg > 7.0:
            caps.append(self.config.slope_slow_cap_mps)

        a_effective = (self.config.minimum_deceleration_mps2 -
                       9.80665 * math.sin(max(0.0, -evidence.pitch_rad)))
        if a_effective <= 0.0:
            return self._stop(evidence.now_monotonic_s)
        if evidence.collision_ttc_s < 0.0:
            if evidence.collision_ttc_s != -1.0:
                return self._stop(evidence.now_monotonic_s)
            if evidence.collision_state == 2:
                caps.append(self.config.collision_caution_cap_mps)
        else:
            stop_ttc = (age + self.config.driver_latency_s + self.config.pipeline_budget_s +
                        evidence.odometry_speed_mps / a_effective +
                        self.config.collision_stop_ttc_margin_s)
            if evidence.collision_ttc_s <= stop_ttc:
                return self._stop(evidence.now_monotonic_s)
            if (evidence.collision_state == 2 or
                    evidence.collision_ttc_s <= self.config.collision_caution_ttc_s):
                caps.append(self.config.collision_caution_cap_mps)
        if evidence.collision_state == 2:
            caps.append(self.config.collision_caution_cap_mps)

        target = min(caps)
        if not math.isfinite(target) or target < 0.0:
            return self._stop(evidence.now_monotonic_s)
        if self._last_time is None:
            self._last_time = evidence.now_monotonic_s
            self._last_cap = target
            self._last_acceleration = 0.0
            return target

        dt = evidence.now_monotonic_s - self._last_time
        desired_acceleration = (target - self._last_cap) / dt
        desired_acceleration = min(self.config.acceleration_limit_mps2,
                                   max(-self.config.deceleration_limit_mps2, desired_acceleration))
        jerk_delta = self.config.jerk_limit_mps3 * dt
        acceleration = min(self._last_acceleration + jerk_delta,
                           max(self._last_acceleration - jerk_delta, desired_acceleration))
        cap = self._last_cap + acceleration * dt
        if (target - self._last_cap) * (target - cap) <= 0.0:
            cap = target
            acceleration = (cap - self._last_cap) / dt
        cap = min(authority_limit, max(0.0, cap))
        self._last_time = evidence.now_monotonic_s
        self._last_cap = cap
        self._last_acceleration = acceleration
        return cap
