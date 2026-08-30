"""ROS-independent qualification and gate policy for stationary-person bypass.

A person is never bypassed from a class label alone. Motion must be STATIC,
the same geometrically-backed track must be observed continuously, and the
permit expires faster than one perception cycle can go stale. Moving,
unknown, learned-only, malformed, or multiple people reset authorization.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Optional, Sequence, Tuple


STATIC = "static"
PERSON = "person"


@dataclass(frozen=True)
class PersonObservation:
    track_id: int
    stamp_s: float
    x_m: float
    y_m: float
    size_x_m: float
    size_y_m: float
    motion: str
    source: str

    @property
    def near_distance_m(self) -> float:
        return max(0.0, self.x_m - 0.5 * self.size_x_m)

    @property
    def geometrically_backed(self) -> bool:
        source = self.source.strip().lower()
        return bool(source) and source not in ("learned_only", "learned")

    @property
    def eligible_static(self) -> bool:
        values = (
            self.stamp_s, self.x_m, self.y_m,
            self.size_x_m, self.size_y_m,
        )
        return (
            self.track_id >= 0
            and all(math.isfinite(float(value)) for value in values)
            and self.size_x_m > 0.0
            and self.size_y_m > 0.0
            and self.motion.strip().lower() == STATIC
            and self.geometrically_backed
        )


@dataclass(frozen=True)
class BypassPermit:
    capable: bool
    active: bool
    stamp_s: float
    expires_s: float
    track_id: Optional[int]
    target_x_m: Optional[float]
    target_y_m: Optional[float]
    static_for_s: float
    max_speed_mps: float
    min_clearance_m: float
    reason: str

    def as_dict(self):
        return {
            "schema": "person-bypass/v1",
            "capable": bool(self.capable),
            "active": bool(self.active),
            "stamp": float(self.stamp_s),
            "expires": float(self.expires_s),
            "track_id": self.track_id,
            "target_x_m": self.target_x_m,
            "target_y_m": self.target_y_m,
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


def person_observations(summary, *, maximum_forward_m: float = 10.0,
                        maximum_lateral_m: float = 1.25) -> Tuple[PersonObservation, ...]:
    """Return directly observed persons in the maneuver region.

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
        if not isinstance(item, dict) or _normal_label(item.get("class")) != PERSON:
            continue
        try:
            track_id = item.get("id")
            if isinstance(track_id, bool) or not isinstance(track_id, int):
                continue
            size = item["size"]
            x_m = float(item["x"])
            y_m = float(item["y"])
            size_x_m = abs(float(size[0]))
            size_y_m = abs(float(size[1]))
        except (KeyError, IndexError, TypeError, ValueError):
            continue
        values = (x_m, y_m, size_x_m, size_y_m)
        if not all(math.isfinite(value) for value in values):
            continue
        if size_x_m <= 0.0 or size_y_m <= 0.0:
            continue
        if x_m + 0.5 * size_x_m < 0.0 or \
                x_m - 0.5 * size_x_m > float(maximum_forward_m) or \
                abs(y_m) - 0.5 * size_y_m > float(maximum_lateral_m):
            continue
        found.append(PersonObservation(
            track_id=track_id,
            stamp_s=stamp_s,
            x_m=x_m,
            y_m=y_m,
            size_x_m=size_x_m,
            size_y_m=size_y_m,
            motion=str(item.get("motion", "unknown")),
            source=str(item.get("source", "geometric")),
        ))
    return tuple(sorted(found, key=lambda value: value.near_distance_m))


class StaticPersonQualifier:
    """Continuous same-track evidence before a DWA person pass is allowed."""

    def __init__(self, confirmation_s: float = 3.0,
                 maximum_gap_s: float = 0.45,
                 maximum_position_jump_m: float = 0.35,
                 permit_lifetime_s: float = 0.45,
                 maximum_forward_m: float = 8.0,
                 observation_forward_m: float = 10.0,
                 maximum_lateral_m: float = 1.0,
                 lateral_hysteresis_m: float = 0.25,
                 minimum_near_distance_m: float = 0.60,
                 max_speed_mps: float = 0.35,
                 min_clearance_m: float = 0.50):
        values = (
            confirmation_s, maximum_gap_s, maximum_position_jump_m,
            permit_lifetime_s, maximum_forward_m, observation_forward_m,
            maximum_lateral_m,
            lateral_hysteresis_m, minimum_near_distance_m, max_speed_mps,
            min_clearance_m,
        )
        if not all(math.isfinite(float(value)) and float(value) > 0.0
                   for value in values):
            raise ValueError("static-person qualification values must be positive")
        self.confirmation_s = float(confirmation_s)
        self.maximum_gap_s = float(maximum_gap_s)
        self.maximum_position_jump_m = float(maximum_position_jump_m)
        self.permit_lifetime_s = float(permit_lifetime_s)
        self.maximum_forward_m = float(maximum_forward_m)
        self.observation_forward_m = float(observation_forward_m)
        if self.observation_forward_m < self.maximum_forward_m:
            raise ValueError(
                "static-person observation range must cover maneuver range")
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

    def inactive(self, now_s: float, reason: str) -> BypassPermit:
        now_s = float(now_s)
        return BypassPermit(
            capable=True, active=False, stamp_s=now_s,
            expires_s=now_s + self.permit_lifetime_s,
            track_id=None, target_x_m=None, target_y_m=None,
            static_for_s=0.0, max_speed_mps=self.max_speed_mps,
            min_clearance_m=self.min_clearance_m, reason=reason,
        )

    def update(self, observations: Sequence[PersonObservation], now_s: float,
               localization_tracking: bool) -> BypassPermit:
        now_s = float(now_s)
        if not math.isfinite(now_s):
            self.reset()
            return self.inactive(0.0, "CLOCK_INVALID")
        if not localization_tracking:
            self.reset()
            return self.inactive(now_s, "LOCALIZATION_NOT_TRACKING")
        if len(observations) != 1:
            self.reset()
            return self.inactive(
                now_s, "NO_PERSON" if not observations else "MULTIPLE_PEOPLE")
        person = observations[0]
        same_track = self.track_id == person.track_id
        if not person.eligible_static:
            self.reset()
            return self.inactive(now_s, "PERSON_NOT_CONFIRMED_STATIC")
        if person.near_distance_m < self.minimum_near_distance_m:
            self.reset()
            return self.inactive(now_s, "PERSON_TOO_CLOSE")
        lateral_limit_m = self.maximum_lateral_m + (
            self.lateral_hysteresis_m if same_track else 0.0)
        if person.near_distance_m > self.observation_forward_m or \
                abs(person.y_m) - 0.5 * person.size_y_m > lateral_limit_m:
            self.reset()
            return self.inactive(now_s, "PERSON_OUTSIDE_MANEUVER_REGION")

        stamp_s = float(person.stamp_s)
        if stamp_s > now_s + 0.05 or now_s - stamp_s > self.maximum_gap_s:
            self.reset()
            return self.inactive(now_s, "PERSON_OBSERVATION_STALE")

        if not same_track or self.last_stamp_s is None:
            self.track_id = person.track_id
            self.first_stamp_s = stamp_s
            self.last_stamp_s = stamp_s
            self.last_xy = (person.x_m, person.y_m)
            return self.inactive(now_s, "QUALIFYING_STATIC_PERSON")

        gap_s = stamp_s - float(self.last_stamp_s)
        if gap_s < -1e-6 or gap_s > self.maximum_gap_s:
            self.reset()
            return self.update((person,), now_s, localization_tracking)
        if self.last_xy is not None:
            jump = math.hypot(
                person.x_m - self.last_xy[0], person.y_m - self.last_xy[1])
            if jump > self.maximum_position_jump_m:
                self.reset()
                return self.update((person,), now_s, localization_tracking)
        if gap_s > 1e-6:
            self.last_stamp_s = stamp_s
            self.last_xy = (person.x_m, person.y_m)

        static_for_s = max(0.0, stamp_s - float(self.first_stamp_s))
        if static_for_s + 1e-6 < self.confirmation_s:
            return BypassPermit(
                capable=True, active=False, stamp_s=now_s,
                expires_s=now_s + self.permit_lifetime_s,
                track_id=person.track_id,
                target_x_m=person.x_m, target_y_m=person.y_m,
                static_for_s=static_for_s,
                max_speed_mps=self.max_speed_mps,
                min_clearance_m=self.min_clearance_m,
                reason="QUALIFYING_STATIC_PERSON",
            )
        if person.near_distance_m > self.maximum_forward_m:
            # Keep the same-track timer warm while the person is visible in
            # the approach region, but do not authorize a trajectory until
            # they enter the bounded maneuver region.  This avoids the field
            # deadlock where DWA saw a person at 8.3 m while the 8.0 m
            # qualifier repeatedly returned NO_PERSON and forced an early
            # stop that prevented the range from ever closing.
            return BypassPermit(
                capable=True, active=False, stamp_s=now_s,
                expires_s=now_s + self.permit_lifetime_s,
                track_id=person.track_id,
                target_x_m=person.x_m, target_y_m=person.y_m,
                static_for_s=static_for_s,
                max_speed_mps=self.max_speed_mps,
                min_clearance_m=self.min_clearance_m,
                reason="STATIC_PERSON_READY_OUTSIDE_MANEUVER_REGION",
            )
        return BypassPermit(
            capable=True, active=True, stamp_s=now_s,
            expires_s=now_s + self.permit_lifetime_s,
            track_id=person.track_id,
            target_x_m=person.x_m, target_y_m=person.y_m,
            static_for_s=static_for_s,
            max_speed_mps=self.max_speed_mps,
            min_clearance_m=self.min_clearance_m,
            reason="STATIC_PERSON_BYPASS",
        )


def static_obstacle_permit(*, now_s: float, observed_stamp_s: float,
                           track_id: int, target_x_m: float,
                           target_y_m: float, motion: str,
                           directly_observed: bool, geometry_valid: bool,
                           maximum_observation_age_s: float = 0.45,
                           permit_lifetime_s: float = 0.45,
                           max_speed_mps: float = 0.35,
                           min_clearance_m: float = 0.50) -> BypassPermit:
    now_s = float(now_s)

    def inactive(reason: str) -> BypassPermit:
        return BypassPermit(
            capable=True, active=False, stamp_s=now_s,
            expires_s=now_s + float(permit_lifetime_s),
            track_id=None, target_x_m=None, target_y_m=None,
            static_for_s=0.0, max_speed_mps=float(max_speed_mps),
            min_clearance_m=float(min_clearance_m), reason=reason,
        )

    values = (
        now_s, observed_stamp_s, target_x_m, target_y_m,
        maximum_observation_age_s, permit_lifetime_s,
        max_speed_mps, min_clearance_m,
    )
    try:
        finite = all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        finite = False
    if not finite or not math.isfinite(now_s):
        return inactive("OBJECT_OBSERVATION_INVALID")
    if float(maximum_observation_age_s) <= 0.0 or \
            float(permit_lifetime_s) <= 0.0 or \
            float(max_speed_mps) <= 0.0 or float(min_clearance_m) <= 0.0:
        return inactive("OBJECT_POLICY_INVALID")
    if isinstance(track_id, bool) or not isinstance(track_id, int) or track_id < 0:
        return inactive("OBJECT_TRACK_INVALID")
    if not directly_observed:
        return inactive("OBJECT_NOT_DIRECTLY_OBSERVED")
    if not geometry_valid:
        return inactive("OBJECT_GEOMETRY_INVALID")
    if _normal_label(motion) != STATIC:
        return inactive("OBJECT_NOT_CONFIRMED_STATIC")
    age_s = now_s - float(observed_stamp_s)
    if age_s < -0.05 or age_s > float(maximum_observation_age_s):
        return inactive("OBJECT_OBSERVATION_STALE")
    return BypassPermit(
        capable=True, active=True, stamp_s=now_s,
        expires_s=now_s + float(permit_lifetime_s),
        track_id=track_id, target_x_m=float(target_x_m),
        target_y_m=float(target_y_m), static_for_s=0.0,
        max_speed_mps=float(max_speed_mps),
        min_clearance_m=float(min_clearance_m),
        reason="STATIC_OBJECT_BYPASS",
    )


def permit_from_payload(value) -> Optional[BypassPermit]:
    try:
        data = _payload(value)
        if data.get("schema") != "person-bypass/v1":
            return None
        stamp_s = float(data["stamp"])
        expires_s = float(data["expires"])
        max_speed = float(data["max_speed_mps"])
        clearance = float(data["min_clearance_m"])
        static_for = float(data.get("static_for_s", 0.0))
        track_id = data.get("track_id")
        if track_id is not None and (isinstance(track_id, bool)
                                     or not isinstance(track_id, int)):
            return None
        x_m = data.get("target_x_m")
        y_m = data.get("target_y_m")
        x_m = None if x_m is None else float(x_m)
        y_m = None if y_m is None else float(y_m)
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    finite_values = [stamp_s, expires_s, max_speed, clearance, static_for]
    if x_m is not None:
        finite_values.append(x_m)
    if y_m is not None:
        finite_values.append(y_m)
    if not all(math.isfinite(value) for value in finite_values):
        return None
    if max_speed <= 0.0 or clearance <= 0.0 or expires_s < stamp_s:
        return None
    active = bool(data.get("active", False))
    if active and (track_id is None or x_m is None or y_m is None):
        return None
    return BypassPermit(
        capable=bool(data.get("capable", False)), active=active,
        stamp_s=stamp_s, expires_s=expires_s, track_id=track_id,
        target_x_m=x_m, target_y_m=y_m, static_for_s=static_for,
        max_speed_mps=max_speed, min_clearance_m=clearance,
        reason=str(data.get("reason", "")),
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
                               observation: PersonObservation,
                               maximum_position_error_m: float = 0.45) -> bool:
    if permit is None or not permit.active or \
            permit.reason != "STATIC_PERSON_BYPASS" or \
            not observation.eligible_static:
        return False
    if permit.track_id != observation.track_id or \
            permit.target_x_m is None or permit.target_y_m is None:
        return False
    error = math.hypot(
        permit.target_x_m - observation.x_m,
        permit.target_y_m - observation.y_m,
    )
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
