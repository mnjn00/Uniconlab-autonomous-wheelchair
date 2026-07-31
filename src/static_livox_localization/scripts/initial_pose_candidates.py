"""ROS-free candidate policy for known-start and global localization."""

import json
import math
import os
import sys
from pathlib import Path
from typing import Mapping, NamedTuple, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Baseline for the start-heading check; mirrors the route generator's own.
GUARD_BASELINE_M = 1.0

from body_frame import (  # noqa: E402
    CHAIR_CENTRE_IN_BODY_XYZ,
    REFERENCE_BODY,
    REFERENCE_CHAIR_CENTRE,
)


class InitializationCandidate(NamedTuple):
    """One map-frame pose for the downstream ICP verifier."""

    x: float
    y: float
    z: float
    yaw_rad: float
    score: Optional[float]
    source: str


class KnownStartRouteError(Exception):
    """A stable, machine-readable route-prior contract error."""

    def __init__(self, reason: str, path: Path):
        self.reason = reason
        self.path = path
        super().__init__(reason, str(path))

    def __str__(self) -> str:
        return "{}: {}".format(self.reason, self.path)


def _finite_number(value, field: str, path: Path) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise KnownStartRouteError("invalid_{}".format(field), path)
    parsed = float(value)
    if not math.isfinite(parsed):
        raise KnownStartRouteError("invalid_{}".format(field), path)
    return parsed


def load_known_start(
    route_path: Path,
    expected_frame: str,
    expected_body_frame_profile: str,
) -> InitializationCandidate:
    """Parse the first route waypoint into a trusted map-frame pose prior."""

    try:
        document = json.loads(route_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise KnownStartRouteError("unreadable_route", route_path) from error
    if not isinstance(document, dict):
        raise KnownStartRouteError("invalid_document", route_path)
    if document.get("frame") != expected_frame:
        raise KnownStartRouteError("frame_mismatch", route_path)
    if document.get("body_frame_profile") != expected_body_frame_profile:
        raise KnownStartRouteError("body_frame_profile_mismatch", route_path)

    waypoints = document.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise KnownStartRouteError("empty_waypoints", route_path)
    if document.get("count") != len(waypoints):
        raise KnownStartRouteError("count_mismatch", route_path)
    first = waypoints[0]
    if not isinstance(first, dict):
        raise KnownStartRouteError("invalid_waypoint", route_path)

    # A backwards start heading is not a small error: seeded as the prior it
    # points ICP away from every surface it should match, so verification
    # cannot converge and initialization falls through to the global search
    # every time. A route generated from a spin-in-place start did exactly
    # that, storing -175 deg where the chair actually faced +4 deg. The
    # chair drives forwards, so the recorded heading and the direction the
    # route immediately travels cannot be more than a right angle apart.
    if len(waypoints) >= 2 and isinstance(waypoints[1], dict):
        # Over one resampling step the travel direction is noise - the route
        # carries the trace itself, so consecutive points are 0.2 m apart and
        # the raw path turns by up to 27 deg between them. Measure it over a
        # baseline long enough to mean something.
        first_x = _finite_number(first.get("x"), "x", route_path)
        first_y = _finite_number(first.get("y"), "y", route_path)
        second = waypoints[1]
        for candidate in waypoints[1:]:
            if not isinstance(candidate, dict):
                continue
            gap = math.hypot(
                _finite_number(candidate.get("x"), "x", route_path) - first_x,
                _finite_number(candidate.get("y"), "y", route_path) - first_y)
            second = candidate
            if gap >= GUARD_BASELINE_M:
                break
        travel = math.degrees(math.atan2(
            _finite_number(second.get("y"), "y", route_path) - first_y,
            _finite_number(second.get("x"), "x", route_path) - first_x,
        ))
        heading = _finite_number(first.get("yaw_deg"), "yaw_deg", route_path)
        if abs((heading - travel + 180.0) % 360.0 - 180.0) > 90.0:
            raise KnownStartRouteError("start_heading_reversed", route_path)

    x = _finite_number(first.get("x"), "x", route_path)
    y = _finite_number(first.get("y"), "y", route_path)
    yaw = math.radians(
        _finite_number(first.get("yaw_deg"), "yaw_deg", route_path)
    )

    # The seed published downstream is a pose of FAST-LIO's BODY frame, which
    # is where the sensor is. A chair-centred route stores the chair's centre
    # instead, 0.545 m away, so it has to be converted back or the prior is
    # seeded half a metre from where the sensor actually is - far enough that
    # verification either fails or, worse, converges somewhere wrong.
    reference = document.get("reference_point", REFERENCE_BODY)
    if reference == REFERENCE_CHAIR_CENTRE:
        forward, left, _ = CHAIR_CENTRE_IN_BODY_XYZ
        x -= math.cos(yaw) * forward - math.sin(yaw) * left
        y -= math.sin(yaw) * forward + math.cos(yaw) * left
    elif reference != REFERENCE_BODY:
        raise KnownStartRouteError("unknown_reference_point", route_path)

    return InitializationCandidate(
        x=x,
        y=y,
        z=_finite_number(first.get("z", 0.0), "z", route_path),
        yaw_rad=yaw,
        score=None,
        source="known_start_route",
    )


def initialization_attempts(
    known_start: Optional[InitializationCandidate],
    globally_scored: Sequence[InitializationCandidate],
    minimum_global_score: float,
    global_limit: int,
) -> Tuple[InitializationCandidate, ...]:
    """Prioritize the known start; score-gate only global-search fallbacks."""

    selected = []
    if known_start is not None:
        selected.append(known_start)
    selected.extend(
        candidate
        for candidate in globally_scored
        if candidate.source == "global_search"
        and candidate.score is not None
        and candidate.score >= minimum_global_score
    )
    return tuple(selected[: global_limit + (1 if known_start is not None else 0)])


def seed_was_acknowledged(
    state: Mapping[str, object],
    baseline_sequence: int,
    baseline_reset_count: int,
) -> bool:
    """Return true only for the MANUAL_ALIGN receipt created by this seed."""

    sequence = state.get("sequence")
    reset_count = state.get("reset_count")
    return (
        isinstance(sequence, int)
        and not isinstance(sequence, bool)
        and isinstance(reset_count, int)
        and not isinstance(reset_count, bool)
        and sequence > baseline_sequence
        and reset_count > baseline_reset_count
        and state.get("message") == "MANUAL_ALIGN"
    )


def tracking_was_verified(
    state: Mapping[str, object],
    enable_sequence: int,
    candidate_reset_count: int,
    saw_verifying: bool,
) -> bool:
    """Reject stale TRACKING receipts and generations that skipped VERIFYING."""

    sequence = state.get("sequence")
    reset_count = state.get("reset_count")
    return (
        saw_verifying
        and isinstance(sequence, int)
        and not isinstance(sequence, bool)
        and isinstance(reset_count, int)
        and not isinstance(reset_count, bool)
        and sequence > enable_sequence
        and reset_count == candidate_reset_count
        and state.get("message") == "TRACKING"
    )
