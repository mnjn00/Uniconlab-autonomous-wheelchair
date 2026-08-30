"""ROS-independent semantic stop policy used ahead of the raw safety gate."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class ThreatView:
    distance_m: float
    motion: str
    label: str
    track_id: Optional[int] = None

    @property
    def is_person(self) -> bool:
        return str(self.label).strip().lower() == "person"

    @property
    def moving_or_unknown(self) -> bool:
        return str(self.motion).strip().lower() != "static"


@dataclass(frozen=True)
class SemanticDecision:
    reason: str
    stop_distance_m: float
    release_distance_m: Optional[float]

    @property
    def blocked(self) -> bool:
        return bool(self.reason)


def stopping_distance(
        measured_speed_mps: float,
        requested_speed_mps: float,
        cloud_age_s: float,
        accumulation_s: float = 0.6,
        pipeline_s: float = 0.2,
        minimum_deceleration_mps2: float = 0.5,
        geometry_margin_m: float = 0.9) -> float:
    values = (measured_speed_mps, requested_speed_mps, cloud_age_s,
              accumulation_s, pipeline_s, minimum_deceleration_mps2,
              geometry_margin_m)
    if not all(math.isfinite(float(value)) for value in values):
        return math.inf
    if minimum_deceleration_mps2 <= 0.0 or \
            min(cloud_age_s, accumulation_s, pipeline_s,
                geometry_margin_m) < 0.0:
        return math.inf
    speed = max(abs(float(measured_speed_mps)),
                max(0.0, float(requested_speed_mps)))
    reaction = float(cloud_age_s) + float(accumulation_s) + float(pipeline_s)
    return (float(geometry_margin_m) + speed * reaction
            + speed * speed / (2.0 * float(minimum_deceleration_mps2)))


class PersonStopLatch:
    """Hold a person stop through the stopped-chair envelope shrinking.

    The same person can otherwise alternate between STOP and GO because the
    dynamic braking radius becomes smaller immediately after the stop.  The
    latch releases only after the person moves beyond the largest stop radius
    seen plus a fixed margin, or disappears for longer than the caller's
    bounded memory.
    """

    def __init__(self, release_margin_m: float = 0.30):
        self.release_margin_m = float(release_margin_m)
        self.track_id = None
        self.release_distance_m = None

    def reset(self) -> None:
        self.track_id = None
        self.release_distance_m = None

    def release_track(self, track_id: int) -> bool:
        if self.track_id != track_id:
            return False
        self.reset()
        return True

    def update(self, threat: Optional[ThreatView], stop_distance_m: float) -> bool:
        if threat is None or not threat.is_person:
            self.reset()
            return False
        if not math.isfinite(threat.distance_m) or \
                not math.isfinite(stop_distance_m):
            self.track_id = threat.track_id
            self.release_distance_m = math.inf
            return True

        same_identity = (
            self.release_distance_m is not None
            and (self.track_id is None or threat.track_id is None
                 or self.track_id == threat.track_id)
        )
        if same_identity:
            self.release_distance_m = max(
                float(self.release_distance_m),
                float(stop_distance_m) + self.release_margin_m)
            if threat.distance_m <= self.release_distance_m:
                return True
            self.reset()

        if threat.distance_m < stop_distance_m:
            self.track_id = threat.track_id
            self.release_distance_m = \
                float(stop_distance_m) + self.release_margin_m
            return True
        return False


def decide_semantic_stop(
        summary_usable: bool,
        summary_age_s: float,
        command_age_s: float,
        maximum_summary_age_s: float,
        maximum_command_age_s: float,
        stop_distance_m: float,
        person: Optional[ThreatView],
        nearest: Optional[ThreatView],
        person_latch: PersonStopLatch) -> SemanticDecision:
    # Upstream outages are already stop conditions. Keep any latched person
    # identity through them so a one-frame perception or command gap cannot
    # erase the larger release radius and immediately authorize motion when
    # the stream returns.
    if not summary_usable:
        return SemanticDecision("PERCEPTION_UNUSABLE", stop_distance_m, None)
    if not math.isfinite(summary_age_s) or summary_age_s > maximum_summary_age_s:
        return SemanticDecision("PERCEPTION_STALE", stop_distance_m, None)
    if not math.isfinite(command_age_s) or command_age_s > maximum_command_age_s:
        return SemanticDecision("INPUT_STALE", stop_distance_m, None)
    if person_latch.update(person, stop_distance_m):
        return SemanticDecision(
            "PERSON", stop_distance_m, person_latch.release_distance_m)
    if nearest is not None and nearest.moving_or_unknown \
            and nearest.distance_m < stop_distance_m:
        return SemanticDecision(
            "MOVING_OBJECT", stop_distance_m, None)
    return SemanticDecision("", stop_distance_m, None)
