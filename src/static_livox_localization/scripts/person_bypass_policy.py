"""ROS-independent qualification and gate policy for static-threat bypass.

People and non-person objects use one authorization lifecycle.  Motion must
be STATIC, the same directly observed track must remain geometrically valid
for two continuous seconds, and the permit expires before perception can go
stale.  A single bounded producer dropout may preserve an already committed
permit; it can never create one.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Optional, Sequence, Tuple


STATIC = "static"
PERSON = "person"
PERMIT_SCHEMA = "static-threat-bypass/v2"
STATIC_THREAT_BYPASS = "STATIC_THREAT_BYPASS"
STATIC_THREAT_DROPOUT_GRACE = "STATIC_THREAT_DROPOUT_GRACE"
QUALIFYING_STATIC_DROPOUT = "QUALIFYING_STATIC_DROPOUT"
PERMIT_FIELDS = frozenset((
    "schema", "capable", "active", "stamp", "expires", "track_id",
    "target_x_m", "target_y_m", "threat_label", "static_for_s",
    "max_speed_mps", "min_clearance_m", "reason",
))
PASS_SIDES = frozenset(("left", "right"))


def commit_pass_side(committed_side, proposed_side):
    proposed = _normal_label(proposed_side)
    if proposed not in PASS_SIDES:
        raise ValueError("pass side must be left or right")
    if committed_side is None:
        return proposed
    committed = _normal_label(committed_side)
    if committed not in PASS_SIDES:
        raise ValueError("committed pass side is invalid")
    return committed


def tail_clear_release(clear_frames: int, tail_clear: bool,
                       required_frames: int = 3):
    if isinstance(clear_frames, bool) or not isinstance(clear_frames, int) or \
            clear_frames < 0:
        raise ValueError("clear frame count must be a nonnegative integer")
    if type(tail_clear) is not bool:
        raise ValueError("tail clear verdict must be boolean")
    if isinstance(required_frames, bool) or not isinstance(required_frames, int) or \
            required_frames <= 0:
        raise ValueError("required clear frames must be a positive integer")
    next_frames = clear_frames + 1 if tail_clear else 0
    return next_frames, next_frames >= required_frames


@dataclass(frozen=True)
class StaticThreatObservation:
    track_id: int
    stamp_s: float
    x_m: float
    y_m: float
    size_x_m: float
    size_y_m: float
    label: str
    motion: str
    source: str
    directly_observed: bool = True
    geometry_valid: bool = True

    @property
    def near_distance_m(self) -> float:
        return max(0.0, self.x_m - 0.5 * self.size_x_m)

    @property
    def geometrically_backed(self) -> bool:
        source = _normal_label(self.source)
        return bool(source) and source not in ("learned_only", "learned")

    @property
    def eligible_static(self) -> bool:
        values = (
            self.stamp_s, self.x_m, self.y_m,
            self.size_x_m, self.size_y_m,
        )
        return (
            isinstance(self.track_id, int)
            and not isinstance(self.track_id, bool)
            and self.track_id >= 0
            and all(_finite(value) for value in values)
            and self.size_x_m > 0.0
            and self.size_y_m > 0.0
            and _normal_label(self.motion) == STATIC
            and self.directly_observed is True
            and self.geometry_valid is True
            and self.geometrically_backed
            and _valid_normal_label(_normal_label(self.label))
        )

    @property
    def is_person(self) -> bool:
        return _normal_label(self.label) == PERSON


@dataclass(frozen=True)
class BypassPermit:
    capable: bool
    active: bool
    stamp_s: float
    expires_s: float
    track_id: Optional[int]
    target_x_m: Optional[float]
    target_y_m: Optional[float]
    threat_label: Optional[str]
    static_for_s: float
    max_speed_mps: float
    min_clearance_m: float
    reason: str

    def as_dict(self):
        return {
            "schema": PERMIT_SCHEMA,
            "capable": bool(self.capable),
            "active": bool(self.active),
            "stamp": float(self.stamp_s),
            "expires": float(self.expires_s),
            "track_id": self.track_id,
            "target_x_m": self.target_x_m,
            "target_y_m": self.target_y_m,
            "threat_label": self.threat_label,
            "static_for_s": round(float(self.static_for_s), 3),
            "max_speed_mps": round(float(self.max_speed_mps), 3),
            "min_clearance_m": round(float(self.min_clearance_m), 3),
            "reason": str(self.reason),
        }

    def to_json(self) -> str:
        return json.dumps(self.as_dict(), separators=(",", ":"), sort_keys=True)


@dataclass(frozen=True)
class GateOverrideDecision:
    allowed: bool
    reason: str
    speed_cap_mps: Optional[float]


def _payload(value):
    if isinstance(value, dict):
        return value
    if hasattr(value, "objects") and hasattr(value, "stamp_s"):
        return {
            "status": getattr(value, "status", ""),
            "stamp": getattr(value, "stamp_s"),
            "objects": getattr(value, "objects"),
        }
    if isinstance(value, str):
        parsed = json.loads(value)
        if isinstance(parsed, dict):
            return parsed
    raise ValueError("summary is not a JSON object")


def _finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) \
        and math.isfinite(float(value))


def _normal_label(value) -> str:
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_")


def _valid_normal_label(value) -> bool:
    return (
        isinstance(value, str)
        and bool(value)
        and value == _normal_label(value)
        and all(character in "abcdefghijklmnopqrstuvwxyz0123456789_"
                for character in value)
    )


def threat_observations(summary, *, maximum_forward_m: float = 10.0,
                        maximum_lateral_m: float = 1.25,
                        labels=None,
                        retained_track_id=None) -> Tuple[StaticThreatObservation, ...]:
    """Return tracked threats in the maneuver region.

    The fused summary may contain learned-only boxes with negative synthetic
    IDs. They remain useful for stopping, but cannot authorize motion; the
    qualifier rejects them through :attr:`geometrically_backed` and track ID.
    """
    try:
        data = _payload(summary)
    except (TypeError, ValueError, json.JSONDecodeError):
        return ()
    if str(data.get("status", "")) != "OK" or not _finite(data.get("stamp")):
        return ()
    objects = data.get("objects")
    if not isinstance(objects, list):
        return ()
    stamp_s = float(data["stamp"])
    found = []
    for item in objects:
        if not isinstance(item, dict):
            if labels is None:
                found.append(StaticThreatObservation(
                    track_id=-1, stamp_s=stamp_s, x_m=0.0, y_m=0.0,
                    size_x_m=1.0, size_y_m=1.0, label="unknown",
                    motion="unknown", source="invalid",
                    directly_observed=False, geometry_valid=False))
            continue
        label = _normal_label(item.get("class"))
        if labels is not None and label not in labels:
            continue
        try:
            track_id = item.get("id")
            if isinstance(track_id, bool) or not isinstance(track_id, int):
                raise ValueError("invalid track id")
            size = item["size"]
            x_m = float(item["x"])
            y_m = float(item["y"])
            size_x_m = abs(float(size[0]))
            size_y_m = abs(float(size[1]))
        except (KeyError, IndexError, TypeError, ValueError):
            found.append(StaticThreatObservation(
                track_id=-1, stamp_s=stamp_s, x_m=0.0, y_m=0.0,
                size_x_m=1.0, size_y_m=1.0, label=label,
                motion="unknown", source="invalid",
                directly_observed=False, geometry_valid=False))
            continue
        values = (x_m, y_m, size_x_m, size_y_m)
        if not all(math.isfinite(value) for value in values) or \
                size_x_m <= 0.0 or size_y_m <= 0.0:
            found.append(StaticThreatObservation(
                track_id=-1, stamp_s=stamp_s, x_m=0.0, y_m=0.0,
                size_x_m=1.0, size_y_m=1.0, label=label,
                motion="unknown", source="invalid",
                directly_observed=False, geometry_valid=False))
            continue
        retained = track_id == retained_track_id
        if not retained and (
                x_m + 0.5 * size_x_m < 0.0
                or x_m - 0.5 * size_x_m > float(maximum_forward_m)
                or abs(y_m) - 0.5 * size_y_m > float(maximum_lateral_m)):
            continue
        found.append(StaticThreatObservation(
            track_id=track_id,
            stamp_s=stamp_s,
            x_m=x_m,
            y_m=y_m,
            size_x_m=size_x_m,
            size_y_m=size_y_m,
            label=label,
            motion=str(item.get("motion", "unknown")),
            source=str(item.get("source", "geometric")),
            directly_observed=item.get("directly_observed", True) is True,
            geometry_valid=item.get("geometry_valid", True) is True,
        ))
    return tuple(sorted(found, key=lambda value: value.near_distance_m))


def person_observations(summary, *, maximum_forward_m: float = 10.0,
                        maximum_lateral_m: float = 1.25):
    return threat_observations(
        summary, maximum_forward_m=maximum_forward_m,
        maximum_lateral_m=maximum_lateral_m, labels={PERSON})


class StaticThreatBypassManager:
    def __init__(self, confirmation_s: float = 2.0,
                 maximum_gap_s: float = 0.45,
                 maximum_position_jump_m: float = 0.35,
                 permit_lifetime_s: float = 0.45,
                 maximum_forward_m: float = 8.0,
                 maximum_lateral_m: float = 1.0,
                 lateral_hysteresis_m: float = 0.25,
                 minimum_near_distance_m: float = 0.60,
                 max_speed_mps: float = 0.35,
                 min_clearance_m: float = 0.50):
        values = (
            confirmation_s, maximum_gap_s, maximum_position_jump_m,
            permit_lifetime_s, maximum_forward_m, maximum_lateral_m,
            lateral_hysteresis_m, minimum_near_distance_m, max_speed_mps,
            min_clearance_m,
        )
        if not all(math.isfinite(float(value)) and float(value) > 0.0
                   for value in values):
            raise ValueError("static-threat qualification values must be positive")
        self.confirmation_s = float(confirmation_s)
        self.maximum_gap_s = float(maximum_gap_s)
        self.maximum_position_jump_m = float(maximum_position_jump_m)
        self.permit_lifetime_s = float(permit_lifetime_s)
        self.maximum_forward_m = float(maximum_forward_m)
        self.maximum_lateral_m = float(maximum_lateral_m)
        self.lateral_hysteresis_m = float(lateral_hysteresis_m)
        self.minimum_near_distance_m = float(minimum_near_distance_m)
        self.max_speed_mps = float(max_speed_mps)
        self.min_clearance_m = float(min_clearance_m)
        self.reset()

    def reset(self) -> None:
        self.track_id = None
        self.first_stamp_s = None
        self.last_stamp_s = None
        self.last_xy = None
        self.committed = False
        self.dropout_used = False
        self.last_observation = None
        self.lifecycle = "IDLE"
        self.pass_side = None
        self.clear_frames = 0

    def _committed_recent(self, reference_stamp_s: float) -> bool:
        return (
            self.committed
            and self.last_stamp_s is not None
            and -0.05 <= reference_stamp_s - float(self.last_stamp_s)
            <= self.maximum_gap_s
        )

    def _inactive_conflict(self, now_s: float, reason: str,
                           evidence_stamp_s=None) -> BypassPermit:
        reference_stamp_s = (
            float(evidence_stamp_s)
            if _finite(evidence_stamp_s) else now_s)
        if self._committed_recent(reference_stamp_s):
            self.lifecycle = "BYPASS_HOLD"
            return self.inactive(now_s, reason)
        self.reset()
        return self.inactive(now_s, reason)

    def commit_pass_side(self, proposed_side: str) -> str:
        if not self.committed:
            raise ValueError("bypass is not committed")
        self.pass_side = commit_pass_side(self.pass_side, proposed_side)
        self.lifecycle = "PASSING"
        self.clear_frames = 0
        return self.pass_side

    def observe_tail_clear(self, tail_clear: bool) -> bool:
        if not self.committed or self.pass_side is None:
            return False
        if self.lifecycle not in ("PASSING", "CLEARING"):
            self.clear_frames = 0
            return False
        self.clear_frames, released = tail_clear_release(
            self.clear_frames, tail_clear)
        self.lifecycle = "CLEARING" if tail_clear else "PASSING"
        if released:
            self.reset()
        return released

    def inactive(self, now_s: float, reason: str) -> BypassPermit:
        now_s = float(now_s)
        return BypassPermit(
            capable=True, active=False, stamp_s=now_s,
            expires_s=now_s + self.permit_lifetime_s,
            track_id=None, target_x_m=None, target_y_m=None,
            threat_label=None,
            static_for_s=0.0, max_speed_mps=self.max_speed_mps,
            min_clearance_m=self.min_clearance_m, reason=reason,
        )

    def _permit(self, now_s: float, observation: StaticThreatObservation,
                static_for_s: float, active: bool, reason: str) -> BypassPermit:
        return BypassPermit(
            capable=True, active=active, stamp_s=now_s,
            expires_s=now_s + self.permit_lifetime_s,
            track_id=observation.track_id,
            target_x_m=observation.x_m, target_y_m=observation.y_m,
            threat_label=_normal_label(observation.label),
            static_for_s=static_for_s,
            max_speed_mps=self.max_speed_mps,
            min_clearance_m=self.min_clearance_m, reason=reason,
        )

    def update(self, observations: Sequence[StaticThreatObservation], now_s: float,
               localization_tracking: bool, *, summary_healthy: bool = True,
               dynamic_conflict: bool = False,
               summary_stamp_s=None) -> BypassPermit:
        now_s = float(now_s)
        if not math.isfinite(now_s):
            self.reset()
            return self.inactive(0.0, "CLOCK_INVALID")
        if not localization_tracking:
            self.reset()
            return self.inactive(now_s, "LOCALIZATION_NOT_TRACKING")
        if not summary_healthy:
            self.reset()
            return self.inactive(now_s, "SUMMARY_UNHEALTHY")
        if dynamic_conflict:
            evidence_stamp_s = max(
                (observation.stamp_s for observation in observations
                 if _finite(observation.stamp_s)),
                default=summary_stamp_s)
            return self._inactive_conflict(
                now_s, "DYNAMIC_CONFLICT", evidence_stamp_s)
        if not observations:
            reference_stamp_s = (
                float(summary_stamp_s)
                if _finite(summary_stamp_s) else now_s)
            if (
                    self.committed
                    and not self.dropout_used
                    and self.last_stamp_s is not None
                    and reference_stamp_s - float(self.last_stamp_s)
                    <= self.maximum_gap_s
                    and self.last_observation is not None):
                self.dropout_used = True
                static_for_s = max(
                    0.0, float(self.last_stamp_s) - float(self.first_stamp_s))
                return self._permit(
                    now_s, self.last_observation, static_for_s, True,
                    STATIC_THREAT_DROPOUT_GRACE)
            if (
                    not self.committed
                    and self.track_id is not None
                    and not self.dropout_used
                    and self.last_stamp_s is not None
                    and reference_stamp_s - float(self.last_stamp_s)
                    <= self.maximum_gap_s
                    and self.last_observation is not None):
                self.dropout_used = True
                static_for_s = max(
                    0.0, float(self.last_stamp_s) - float(self.first_stamp_s))
                return self._permit(
                    now_s, self.last_observation, static_for_s, False,
                    QUALIFYING_STATIC_DROPOUT)
            return self._inactive_conflict(
                now_s, "NO_THREAT", summary_stamp_s)
        if len(observations) != 1:
            return self._inactive_conflict(
                now_s, "MULTIPLE_THREATS", summary_stamp_s)
        threat = observations[0]
        same_track = self.track_id == threat.track_id
        if not threat.eligible_static:
            if same_track and _normal_label(threat.motion) in (
                    "moving", "unknown"):
                return self._inactive_conflict(
                    now_s, "THREAT_NOT_CONFIRMED_STATIC", threat.stamp_s)
            self.reset()
            return self.inactive(now_s, "THREAT_NOT_CONFIRMED_STATIC")
        stamp_s = float(threat.stamp_s)
        if stamp_s > now_s + 0.05 or now_s - stamp_s > self.maximum_gap_s:
            self.reset()
            return self.inactive(now_s, "THREAT_OBSERVATION_STALE")

        committed_same_track = self.committed and same_track
        if not committed_same_track:
            if threat.near_distance_m < self.minimum_near_distance_m:
                self.reset()
                return self.inactive(now_s, "THREAT_TOO_CLOSE")
            lateral_limit_m = self.maximum_lateral_m + (
                self.lateral_hysteresis_m if same_track else 0.0)
            if threat.near_distance_m > self.maximum_forward_m or \
                    abs(threat.y_m) - 0.5 * threat.size_y_m > lateral_limit_m:
                self.reset()
                return self.inactive(
                    now_s, "THREAT_OUTSIDE_MANEUVER_REGION")

        if not same_track or self.last_stamp_s is None:
            self.reset()
            self.track_id = threat.track_id
            self.first_stamp_s = stamp_s
            self.last_stamp_s = stamp_s
            self.last_xy = (threat.x_m, threat.y_m)
            self.last_observation = threat
            self.lifecycle = "QUALIFYING_STATIC"
            return self._permit(
                now_s, threat, 0.0, False, "QUALIFYING_STATIC_THREAT")

        gap_s = stamp_s - float(self.last_stamp_s)
        if gap_s < -1e-6 or gap_s > self.maximum_gap_s:
            self.reset()
            return self.update((threat,), now_s, localization_tracking)
        if self.last_xy is not None:
            jump = math.hypot(
                threat.x_m - self.last_xy[0], threat.y_m - self.last_xy[1])
            if jump > self.maximum_position_jump_m:
                self.reset()
                return self.update((threat,), now_s, localization_tracking)
        if gap_s > 1e-6:
            self.last_stamp_s = stamp_s
            self.last_xy = (threat.x_m, threat.y_m)
        self.last_observation = threat
        self.dropout_used = False

        if self.committed:
            self.lifecycle = "PASSING" if self.pass_side else "BYPASS_COMMITTED"
            return self._permit(
                now_s, threat,
                max(0.0, stamp_s - float(self.first_stamp_s)), True,
                STATIC_THREAT_BYPASS)

        static_for_s = max(0.0, stamp_s - float(self.first_stamp_s))
        if static_for_s + 1e-6 < self.confirmation_s:
            return self._permit(
                now_s, threat, static_for_s, False,
                "QUALIFYING_STATIC_THREAT")
        self.committed = True
        self.lifecycle = "BYPASS_COMMITTED"
        return self._permit(
            now_s, threat, static_for_s, True, STATIC_THREAT_BYPASS,
        )


def permit_from_payload(value) -> Optional[BypassPermit]:
    try:
        data = _payload(value)
        if frozenset(data) != PERMIT_FIELDS or data["schema"] != PERMIT_SCHEMA:
            return None
        if type(data["capable"]) is not bool or type(data["active"]) is not bool:
            return None
        numeric_keys = (
            "stamp", "expires", "static_for_s", "max_speed_mps",
            "min_clearance_m")
        if any(not _finite(data[key]) for key in numeric_keys):
            return None
        stamp_s = float(data["stamp"])
        expires_s = float(data["expires"])
        max_speed = float(data["max_speed_mps"])
        clearance = float(data["min_clearance_m"])
        static_for = float(data["static_for_s"])
        track_id = data.get("track_id")
        if track_id is not None and (isinstance(track_id, bool)
                                     or not isinstance(track_id, int)
                                     or track_id < 0):
            return None
        x_m = data.get("target_x_m")
        y_m = data.get("target_y_m")
        if x_m is not None and not _finite(x_m):
            return None
        if y_m is not None and not _finite(y_m):
            return None
        x_m = None if x_m is None else float(x_m)
        y_m = None if y_m is None else float(y_m)
        threat_label = data.get("threat_label")
        if threat_label is not None and not _valid_normal_label(threat_label):
            return None
        reason = data["reason"]
        if not isinstance(reason, str) or not reason:
            return None
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if stamp_s < 0.0 or static_for < 0.0 or max_speed <= 0.0 or \
            clearance <= 0.0 or expires_s <= stamp_s:
        return None
    active = data["active"]
    if active and (
            not data["capable"]
            or track_id is None or x_m is None or y_m is None
            or threat_label is None
            or static_for < 2.0
            or reason not in (
                STATIC_THREAT_BYPASS, STATIC_THREAT_DROPOUT_GRACE)):
        return None
    return BypassPermit(
        capable=bool(data.get("capable", False)), active=active,
        stamp_s=stamp_s, expires_s=expires_s, track_id=track_id,
        target_x_m=x_m, target_y_m=y_m, static_for_s=static_for,
        threat_label=threat_label,
        max_speed_mps=max_speed, min_clearance_m=clearance,
        reason=reason,
    )


def permit_is_fresh(permit: Optional[BypassPermit], now_s: float,
                    maximum_age_s: float = 0.45) -> bool:
    if permit is None or not permit.capable:
        return False
    try:
        now_s = float(now_s)
        maximum_age_s = float(maximum_age_s)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(now_s) or not math.isfinite(maximum_age_s) or \
            maximum_age_s <= 0.0:
        return False
    age = now_s - permit.stamp_s
    return -0.05 <= age <= maximum_age_s and now_s <= permit.expires_s


def permit_matches_observation(permit: Optional[BypassPermit],
                               observation: StaticThreatObservation,
                               maximum_position_error_m: float = 0.45) -> bool:
    if permit is None or not permit.active or not observation.eligible_static:
        return False
    if permit.reason not in (
            STATIC_THREAT_BYPASS, STATIC_THREAT_DROPOUT_GRACE):
        return False
    if permit.track_id != observation.track_id or \
            permit.target_x_m is None or permit.target_y_m is None:
        return False
    if permit.threat_label != _normal_label(observation.label):
        return False
    error = math.hypot(
        permit.target_x_m - observation.x_m,
        permit.target_y_m - observation.y_m,
    )
    return error <= float(maximum_position_error_m)


def permit_matches_threat(permit: Optional[BypassPermit], threat,
                          now_s: float,
                          maximum_position_error_m: float = 0.45) -> bool:
    if permit is None or threat is None or not permit.active or \
            not permit_is_fresh(permit, now_s):
        return False
    if permit.reason not in (
            STATIC_THREAT_BYPASS, STATIC_THREAT_DROPOUT_GRACE):
        return False
    if not getattr(threat, "parked", False) or \
            getattr(threat, "geometry_valid", False) is not True or \
            getattr(threat, "track_id", None) != permit.track_id:
        return False
    if permit.reason == STATIC_THREAT_BYPASS and \
            getattr(threat, "directly_observed", False) is not True:
        return False
    observed_stamp_s = getattr(threat, "observed_stamp_s", None)
    if not _finite(observed_stamp_s) or \
            not -0.05 <= float(now_s) - float(observed_stamp_s) <= 0.45:
        return False
    label = _normal_label(getattr(threat, "label", ""))
    if not label or label != permit.threat_label:
        return False
    coordinates = (
        getattr(threat, "center_x_m", None),
        getattr(threat, "center_y_m", None))
    if permit.target_x_m is None or permit.target_y_m is None or \
            not all(_finite(value) for value in coordinates):
        return False
    error = math.hypot(
        permit.target_x_m - float(coordinates[0]),
        permit.target_y_m - float(coordinates[1]))
    return error <= float(maximum_position_error_m)


def evaluate_gate_override(*, permit: Optional[BypassPermit], now_s: float,
                           requested_v_mps: float, requested_w_rps: float,
                           immediate_collision: bool,
                           requested_path_collision: bool,
                           carried_path_collision: bool,
                           minimum_turn_rps: float = 0.08,
                           maximum_permit_age_s: float = 0.45) -> GateOverrideDecision:
    """Allow only a fresh, genuinely curved, collision-free static-person arc."""
    if permit is None or not permit.active:
        return GateOverrideDecision(False, "NO_ACTIVE_PERMIT", None)
    if not permit_is_fresh(permit, now_s, maximum_permit_age_s):
        return GateOverrideDecision(False, "PERMIT_STALE", None)
    values = (requested_v_mps, requested_w_rps, minimum_turn_rps)
    if not all(math.isfinite(float(value)) for value in values):
        return GateOverrideDecision(False, "COMMAND_INVALID", None)
    if requested_v_mps <= 0.0:
        return GateOverrideDecision(False, "NO_FORWARD_MOTION", None)
    if abs(requested_w_rps) < float(minimum_turn_rps):
        return GateOverrideDecision(False, "TURN_TOO_SMALL", None)
    if immediate_collision:
        return GateOverrideDecision(False, "IMMEDIATE_FOOTPRINT", None)
    if carried_path_collision:
        return GateOverrideDecision(False, "CARRIED_PATH_COLLISION", None)
    if requested_path_collision:
        return GateOverrideDecision(False, "REQUESTED_PATH_COLLISION", None)
    return GateOverrideDecision(
        True, "STATIC_PERSON_TRAJECTORY_CLEAR", float(permit.max_speed_mps))
