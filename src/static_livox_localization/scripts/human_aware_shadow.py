"""Temporal human evidence for advisory CoHAN/HATEB shadow planning."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

STATIC_CONFIRM_S = 10.0
MAX_PRODUCER_GAP_S = 0.35
MOVING_REVOKE_S = 0.60
COHAN_STATIC = 0
COHAN_HUMAN = 1
COHAN_TORSO = 1


def finite_or_nan(value: str | float | None) -> float:
    """Parse an external scalar without turning missing evidence into zero."""
    if value is None:
        return math.nan
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


class ShadowDecision(Enum):
    """Advisory states; none of them carries motion authority."""

    STOP_REQUIRED = "STOP_REQUIRED"
    OBSERVING = "OBSERVING"
    BYPASS_COMMITTED = "BYPASS_COMMITTED"


class TrajectoryDecision(Enum):
    """Why a shadow HATEB path is reportable or refused."""

    ACCEPTED = "ACCEPTED"
    INVALID = "INVALID"
    HARD_MASK_REJECTED = "HARD_MASK_REJECTED"
    SAFETY_BAND_REJECTED = "SAFETY_BAND_REJECTED"


class SafetyBandGeometry(Protocol):
    def contains(self, point: Sequence[float]) -> bool:
        ...

    def chord_is_contained(
            self,
            start: Sequence[float],
            end: Sequence[float]) -> bool:
        ...


class DrivableMaskGeometry(Protocol):
    def contains(self, point: Sequence[float]) -> bool:
        ...

    def segment_is_contained(
            self,
            start: Sequence[float],
            end: Sequence[float]) -> bool:
        ...


@dataclass(frozen=True)
class PersonObservation:
    """One directly observed person at a producer timestamp."""

    track_id: int
    observed_stamp_s: float
    motion: str
    speed_mps: float
    forward_m: float
    lateral_m: float
    half_length_m: float
    half_width_m: float
    directly_observed: bool
    geometry_valid: bool

    @property
    def usable(self) -> bool:
        values = (
            self.observed_stamp_s,
            self.speed_mps,
            self.forward_m,
            self.lateral_m,
            self.half_length_m,
            self.half_width_m,
        )
        return (
            self.directly_observed
            and self.geometry_valid
            and self.track_id >= 0
            and all(math.isfinite(value) for value in values)
            and self.half_length_m > 0.0
            and self.half_width_m > 0.0
        )


@dataclass(frozen=True)
class AdvisorySnapshot:
    """Current shadow-only decision and the identity it concerns."""

    decision: ShadowDecision
    track_id: int | None
    evidence_s: float


@dataclass(frozen=True)
class RobotPose2D:
    """Corrected chair/body pose in the CoHAN global frame."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class CohanAgentPayload:
    """ROS-independent values required by one CoHAN torso segment."""

    track_id: int
    state: int
    agent_type: int
    segment_type: int
    frame_id: str
    stamp_s: float
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    position_variance: float


def to_cohan_agent(
        person: PersonObservation,
        robot: RobotPose2D,
        frame_id: str = "map") -> CohanAgentPayload:
    """Transform one committed stationary torso into CoHAN's global frame."""
    cosine = math.cos(robot.yaw_rad)
    sine = math.sin(robot.yaw_rad)
    x_m = robot.x_m + cosine * person.forward_m \
        - sine * person.lateral_m
    y_m = robot.y_m + sine * person.forward_m \
        + cosine * person.lateral_m
    position_variance = max(
        person.half_length_m ** 2,
        person.half_width_m ** 2,
    )
    return CohanAgentPayload(
        track_id=person.track_id,
        state=COHAN_STATIC,
        agent_type=COHAN_HUMAN,
        segment_type=COHAN_TORSO,
        frame_id=frame_id,
        stamp_s=person.observed_stamp_s,
        x_m=x_m,
        y_m=y_m,
        vx_mps=0.0,
        vy_mps=0.0,
        position_variance=position_variance,
    )


class HumanAwareConditioner:
    """Mutable evidence accumulator for one-person shadow planning.

    Mutation is the purpose of this object: continuity across producer
    cycles is the evidence a frame-by-frame classifier cannot represent.
    """

    def __init__(
            self,
            static_confirm_s: float = STATIC_CONFIRM_S,
            max_producer_gap_s: float = MAX_PRODUCER_GAP_S,
            moving_revoke_s: float = MOVING_REVOKE_S):
        self.static_confirm_s: float = float(static_confirm_s)
        self.max_producer_gap_s: float = float(max_producer_gap_s)
        self.moving_revoke_s: float = float(moving_revoke_s)
        self.track_id: int | None = None
        self.static_since_s: float | None = None
        self.last_stamp_s: float | None = None
        self.moving_since_s: float | None = None
        self.is_committed: bool = False

    def reset(self) -> None:
        self.track_id = None
        self.static_since_s = None
        self.last_stamp_s = None
        self.moving_since_s = None
        self.is_committed = False

    def snapshot(self, decision: ShadowDecision) -> AdvisorySnapshot:
        evidence_s = 0.0
        if self.static_since_s is not None and self.last_stamp_s is not None:
            evidence_s = max(0.0, self.last_stamp_s - self.static_since_s)
        return AdvisorySnapshot(decision, self.track_id, evidence_s)

    def stop_and_reset(self) -> AdvisorySnapshot:
        self.reset()
        return self.snapshot(ShadowDecision.STOP_REQUIRED)

    def update(
            self,
            stamp_s: float,
            people: Sequence[PersonObservation],
            localization_tracking: bool = True) -> AdvisorySnapshot:
        """Consume one coherent producer cycle and return its advisory state."""
        if not localization_tracking or len(people) != 1:
            return self.stop_and_reset()
        person = people[0]
        if (
                not person.usable
                or not math.isfinite(stamp_s)
                or abs(person.observed_stamp_s - stamp_s) > 1e-6):
            return self.stop_and_reset()

        if self.track_id is None:
            if person.motion != "static":
                return self.stop_and_reset()
            self.track_id = person.track_id
            self.static_since_s = stamp_s
            self.last_stamp_s = stamp_s
            return self.snapshot(ShadowDecision.OBSERVING)

        if person.track_id != self.track_id or self.last_stamp_s is None:
            return self.stop_and_reset()
        gap_s = stamp_s - self.last_stamp_s
        if gap_s < 0.0 or gap_s > self.max_producer_gap_s:
            return self.stop_and_reset()
        if gap_s > 0.0:
            self.last_stamp_s = stamp_s

        if person.motion == "moving":
            if not self.is_committed:
                return self.stop_and_reset()
            if self.moving_since_s is None:
                self.moving_since_s = stamp_s
            moving_s = stamp_s - self.moving_since_s
            if moving_s >= self.moving_revoke_s - 1e-9:
                return self.stop_and_reset()
            return self.snapshot(ShadowDecision.BYPASS_COMMITTED)

        if person.motion != "static":
            return self.stop_and_reset()

        self.moving_since_s = None
        if self.static_since_s is None:
            self.static_since_s = stamp_s
        evidence_s = stamp_s - self.static_since_s
        if evidence_s >= self.static_confirm_s - 1e-9:
            self.is_committed = True
            return self.snapshot(ShadowDecision.BYPASS_COMMITTED)
        return self.snapshot(ShadowDecision.OBSERVING)


class ShadowTrajectoryValidator:
    """Hard validation for an advisory local plan; never a cost function."""

    def __init__(
            self,
            safety_band: SafetyBandGeometry,
            drivable_mask: DrivableMaskGeometry):
        self.safety_band: SafetyBandGeometry = safety_band
        self.drivable_mask: DrivableMaskGeometry = drivable_mask

    def validate(
            self,
            points: Sequence[tuple[float, float]]) -> TrajectoryDecision:
        if len(points) < 2 or any(
                len(point) != 2
                or not all(math.isfinite(value) for value in point)
                for point in points):
            return TrajectoryDecision.INVALID
        if (
                not all(self.drivable_mask.contains(point) for point in points)
                or not all(
                    self.drivable_mask.segment_is_contained(start, end)
                    for start, end in zip(points[:-1], points[1:]))):
            return TrajectoryDecision.HARD_MASK_REJECTED
        if (
                not all(self.safety_band.contains(point) for point in points)
                or not all(
                    self.safety_band.chord_is_contained(start, end)
                    for start, end in zip(points[:-1], points[1:]))):
            return TrajectoryDecision.SAFETY_BAND_REJECTED
        return TrajectoryDecision.ACCEPTED
