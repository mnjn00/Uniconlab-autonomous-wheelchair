#!/usr/bin/env python3
"""ROS-independent, fail-closed mission state machine."""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Mapping, Optional, Tuple


class MissionState(str, Enum):
    DISARMED = "DISARMED"
    LOCALIZING = "LOCALIZING"
    READY = "READY"
    NAVIGATING = "NAVIGATING"
    PAUSED_OBSTACLE = "PAUSED_OBSTACLE"
    GOAL_REACHED = "GOAL_REACHED"
    ABORTED = "ABORTED"
    FAULT = "FAULT"


class MotionIntent(str, Enum):
    HOLD = "HOLD"
    PROCEED = "PROCEED"
    SLOW = "SLOW"
    STOP = "STOP"


class EventType(str, Enum):
    ARM = "ARM"
    DISARM = "DISARM"
    RESET = "RESET"
    RESUME = "RESUME"
    ROUTE_STATUS = "ROUTE_STATUS"
    LOCALIZATION = "LOCALIZATION"
    GEOFENCE = "GEOFENCE"
    COLLISION = "COLLISION"
    SLOPE = "SLOPE"
    MODE = "MODE"
    DRIVER = "DRIVER"
    PROGRESS = "PROGRESS"
    PROCESS_LOST = "PROCESS_LOST"
    MOVE_BASE_ACTIVE = "MOVE_BASE_ACTIVE"
    MOVE_BASE_SUCCEEDED = "MOVE_BASE_SUCCEEDED"
    MOVE_BASE_ABORTED = "MOVE_BASE_ABORTED"
    MOVE_BASE_LOST = "MOVE_BASE_LOST"
    TICK = "TICK"


# Stable string constants for adapters which do not import Enum values.
DISARMED = MissionState.DISARMED.value
LOCALIZING = MissionState.LOCALIZING.value
READY = MissionState.READY.value
NAVIGATING = MissionState.NAVIGATING.value
PAUSED_OBSTACLE = MissionState.PAUSED_OBSTACLE.value
GOAL_REACHED = MissionState.GOAL_REACHED.value
ABORTED = MissionState.ABORTED.value
FAULT = MissionState.FAULT.value
HOLD = MotionIntent.HOLD.value
PROCEED = MotionIntent.PROCEED.value
SLOW = MotionIntent.SLOW.value
STOP = MotionIntent.STOP.value


@dataclass(frozen=True)
class MissionConfig:
    max_linear_mps: float = 0.35
    max_angular_rps: float = 0.65
    slow_linear_mps: float = 0.12
    slow_angular_rps: float = 0.25
    evidence_timeout_s: float = 0.75
    progress_timeout_s: float = 4.0
    action_timeout_s: float = 1.0
    obstacle_stop_entry_s: float = 0.20
    obstacle_clear_s: float = 1.00

    def __post_init__(self) -> None:
        values = (
            self.max_linear_mps,
            self.max_angular_rps,
            self.slow_linear_mps,
            self.slow_angular_rps,
            self.evidence_timeout_s,
            self.progress_timeout_s,
            self.action_timeout_s,
            self.obstacle_stop_entry_s,
            self.obstacle_clear_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("mission configuration values must be finite and positive")
        if self.slow_linear_mps >= self.max_linear_mps:
            raise ValueError("slow linear cap must be below normal cap")
        if self.slow_angular_rps >= self.max_angular_rps:
            raise ValueError("slow angular cap must be below normal cap")


@dataclass(frozen=True)
class MissionEvent:
    kind: EventType
    value: Any = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", EventType(self.kind))


@dataclass(frozen=True)
class MissionOutput:
    state: MissionState
    intent: MotionIntent
    reason: str
    cancel_goal: bool
    send_waypoint_index: Optional[int]
    terminal_status: str
    progress: int
    max_linear_mps: float
    max_angular_rps: float


class MissionFSM:
    """Deterministic FSM. All time enters through method arguments or ``clock``."""

    TRANSITION_TABLE: Dict[Tuple[MissionState, EventType], str] = {
        (state, event): "_dispatch" for state in MissionState for event in EventType
    }

    _EVIDENCE = ("localization", "geofence", "collision", "slope", "mode", "driver")

    def __init__(self, config: MissionConfig, clock: Callable[[], float]) -> None:
        self.config = config
        self.clock = clock
        self.state = MissionState.DISARMED
        self.reason = "disarmed"
        self._route: Optional[Dict[str, Any]] = None
        self._waypoint_count = 0
        self._progress = 0
        self._values: Dict[str, Any] = {
            "localization": False,
            "geofence": False,
            "collision": "unknown",
            "slope": "unknown",
            "mode": False,
            "driver": False,
        }
        self._updated: Dict[str, float] = {}
        self._action_updated: Optional[float] = None
        self._progress_updated: Optional[float] = None
        self._blocked_since: Optional[float] = None
        self._clear_since: Optional[float] = None
        self._resume_requested = False
        self._goal_active = False
        self._cancel_goal = False
        self._send_waypoint: Optional[int] = None
        self._terminal_status = ""
        self.validate_transition_table()

    @classmethod
    def validate_transition_table(cls) -> bool:
        expected = {(state, event) for state in MissionState for event in EventType}
        actual = set(cls.TRANSITION_TABLE)
        if actual != expected:
            missing = expected - actual
            extra = actual - expected
            raise ValueError("incomplete transition table: missing=%r extra=%r" % (missing, extra))
        return True

    def arm(self, route: Any, now: Optional[float] = None) -> MissionOutput:
        return self.update(MissionEvent(EventType.ARM, route), now)

    def reset(self, now: Optional[float] = None) -> MissionOutput:
        return self.update(MissionEvent(EventType.RESET), now)

    def disarm(self, now: Optional[float] = None) -> MissionOutput:
        return self.update(MissionEvent(EventType.DISARM), now)

    def resume(self, now: Optional[float] = None) -> MissionOutput:
        return self.update(MissionEvent(EventType.RESUME), now)

    def tick(self, now: Optional[float] = None) -> MissionOutput:
        return self.update(MissionEvent(EventType.TICK), now)

    def update(self, event: MissionEvent, now: Optional[float] = None) -> MissionOutput:
        if not isinstance(event, MissionEvent):
            raise TypeError("event must be MissionEvent")
        timestamp = self._time(now)
        self._cancel_goal = False
        self._send_waypoint = None
        handler_name = self.TRANSITION_TABLE[(self.state, event.kind)]
        getattr(self, handler_name)(event, timestamp)
        self._evaluate(timestamp)
        return self.output()

    def output(self) -> MissionOutput:
        intent = MotionIntent.HOLD
        linear = 0.0
        angular = 0.0
        if self.state == MissionState.NAVIGATING:
            if self._values["slope"] == "slow":
                intent = MotionIntent.SLOW
                linear = self.config.slow_linear_mps
                angular = self.config.slow_angular_rps
            else:
                intent = MotionIntent.PROCEED
                linear = self.config.max_linear_mps
                angular = self.config.max_angular_rps
        return MissionOutput(
            state=self.state,
            intent=intent,
            reason=self.reason,
            cancel_goal=self._cancel_goal,
            send_waypoint_index=self._send_waypoint,
            terminal_status=self._terminal_status,
            progress=self._progress,
            max_linear_mps=linear,
            max_angular_rps=angular,
        )

    def _time(self, now: Optional[float]) -> float:
        value = self.clock() if now is None else now
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("time must be finite and non-negative")
        return value

    @staticmethod
    def _field(route: Any, key: str, default: Any = None) -> Any:
        if isinstance(route, Mapping):
            return route.get(key, default)
        return getattr(route, key, default)

    def _valid_route(self, route: Any) -> bool:
        return bool(
            route is not None
            and self._field(route, "valid", False)
            and self._field(route, "graph_valid", False)
            and self._field(route, "map_valid", False)
            and self._field(route, "hash_valid", False)
            and self._field(route, "route_id", "")
            and self._field(route, "map_id", "")
            and self._field(route, "route_hash", "")
            and isinstance(self._field(route, "waypoint_count", 0), int)
            and not isinstance(self._field(route, "waypoint_count", 0), bool)
            and self._field(route, "waypoint_count", 0) > 0
        )

    def _dispatch(self, event: MissionEvent, now: float) -> None:
        kind = event.kind
        if kind == EventType.DISARM:
            if self.state in (MissionState.FAULT, MissionState.ABORTED):
                self._cancel_goal = True
            else:
                self._to_disarmed("operator_disarm")
            return
        if kind == EventType.RESET:
            if self.state in (MissionState.FAULT, MissionState.ABORTED, MissionState.GOAL_REACHED):
                self._to_disarmed("operator_reset")
            return
        if kind == EventType.ARM:
            self._on_arm(event.value, now)
            return
        if kind == EventType.RESUME:
            if self.state != MissionState.PAUSED_OBSTACLE:
                self._impossible("resume_out_of_order")
            else:
                self._resume_requested = True
                self.reason = "resume_requested"
            return
        if kind == EventType.ROUTE_STATUS:
            if self.state == MissionState.DISARMED:
                return
            if not self._route_status_valid(event.value):
                self._fault("route_map_hash_invalid")
            return
        if kind in (EventType.LOCALIZATION, EventType.GEOFENCE, EventType.COLLISION,
                    EventType.SLOPE, EventType.MODE, EventType.DRIVER):
            self._on_evidence(kind, event.value, now)
            return
        if kind == EventType.PROGRESS:
            self._on_progress(event.value, now)
            return
        if kind == EventType.MOVE_BASE_ACTIVE:
            self._on_action_active(now)
            return
        if kind == EventType.MOVE_BASE_SUCCEEDED:
            self._on_action_succeeded(now)
            return
        if kind == EventType.MOVE_BASE_ABORTED:
            self._on_action_aborted()
            return
        if kind == EventType.PROCESS_LOST:
            if self.state not in (MissionState.DISARMED, MissionState.FAULT):
                self._fault("required_process_lost")
            return
        if kind == EventType.MOVE_BASE_LOST:
            if self.state == MissionState.NAVIGATING:
                self._fault("move_base_lost")
            elif self.state not in (MissionState.DISARMED, MissionState.FAULT):
                self._impossible("move_base_lost_out_of_order")

    def _on_arm(self, route: Any, now: float) -> None:
        if self.state != MissionState.DISARMED:
            self._impossible("arm_out_of_order")
            return
        if not self._valid_route(route):
            self._fault("route_validation_failed")
            return
        self._route = dict(route) if isinstance(route, Mapping) else {
            key: self._field(route, key) for key in (
                "valid", "graph_valid", "map_valid", "hash_valid", "route_id",
                "map_id", "route_hash", "waypoint_count"
            )
        }
        self._waypoint_count = int(self._field(route, "waypoint_count"))
        self._progress = 0
        self._progress_updated = None
        self._terminal_status = ""
        self.state = MissionState.LOCALIZING
        self.reason = "awaiting_fresh_readiness"

    def _route_status_valid(self, value: Any) -> bool:
        if isinstance(value, Mapping):
            return all(bool(value.get(key, False)) for key in ("route_valid", "map_valid", "hash_valid", "graph_valid"))
        return bool(value)

    def _on_evidence(self, kind: EventType, value: Any, now: float) -> None:
        key = kind.value.lower()
        if key in ("localization", "geofence", "mode", "driver"):
            normalized: Any = bool(value)
        elif key == "collision":
            normalized = str(value).lower()
            if normalized not in ("clear", "blocked", "estop"):
                self._fault("invalid_collision_evidence")
                return
        else:
            normalized = str(value).lower()
            if normalized not in ("safe", "slow", "unsafe"):
                self._fault("invalid_slope_evidence")
                return
        self._values[key] = normalized
        self._updated[key] = now
        if key == "collision":
            if normalized == "blocked":
                self._blocked_since = now if self._blocked_since is None else self._blocked_since
                self._clear_since = None
            elif normalized == "clear":
                self._blocked_since = None
                self._clear_since = now if self._clear_since is None else self._clear_since
            else:
                self._fault("emergency_stop")
                return
        if self.state == MissionState.NAVIGATING:
            critical = {
                "localization": normalized is False,
                "geofence": normalized is False,
                "slope": normalized == "unsafe",
                "mode": normalized is False,
                "driver": normalized is False,
            }
            if critical.get(key, False):
                self._fault(key + "_lost")

    def _on_progress(self, value: Any, now: float) -> None:
        if isinstance(value, bool) or not isinstance(value, int):
            self._fault("invalid_progress")
            return
        if self.state == MissionState.LOCALIZING:
            if value < self._progress or value >= self._waypoint_count:
                self._fault("invalid_initial_progress")
                return
            self._progress = value
            self._progress_updated = now
            return
        if self.state != MissionState.NAVIGATING:
            self._impossible("progress_out_of_order")
            return
        if value < self._progress or value > self._waypoint_count:
            self._fault("invalid_progress_order")
            return
        self._progress = value
        self._progress_updated = now

    def _on_action_active(self, now: float) -> None:
        if self.state == MissionState.READY:
            self.state = MissionState.NAVIGATING
            self.reason = "move_base_active"
            self._goal_active = True
            self._action_updated = now
            self._progress_updated = now
        elif self.state == MissionState.NAVIGATING:
            self._action_updated = now
        else:
            self._impossible("move_base_active_out_of_order")

    def _on_action_succeeded(self, now: float) -> None:
        if self.state != MissionState.NAVIGATING or not self._goal_active:
            self._impossible("move_base_success_out_of_order")
            return
        self._goal_active = False
        self._action_updated = now
        self._progress += 1
        self._progress_updated = now
        if self._progress >= self._waypoint_count:
            self.state = MissionState.GOAL_REACHED
            self.reason = "mission_complete"
            self._terminal_status = "SUCCEEDED"
        else:
            self.state = MissionState.READY
            self.reason = "waypoint_reached"
            self._send_waypoint = self._progress

    def _on_action_aborted(self) -> None:
        if self.state != MissionState.NAVIGATING or not self._goal_active:
            self._impossible("move_base_abort_out_of_order")
            return
        self._goal_active = False
        self._cancel_goal = True
        self.state = MissionState.ABORTED
        self.reason = "move_base_aborted"
        self._terminal_status = "ABORTED"

    def _fresh(self, key: str, now: float, timeout: Optional[float] = None) -> bool:
        updated = self._updated.get(key)
        limit = self.config.evidence_timeout_s if timeout is None else timeout
        return updated is not None and 0.0 <= now - updated <= limit

    def _readiness_ok(self, now: float) -> bool:
        return bool(
            self._route is not None
            and self._values["localization"] is True
            and self._values["geofence"] is True
            and self._values["collision"] == "clear"
            and self._values["slope"] in ("safe", "slow")
            and self._values["mode"] is True
            and self._values["driver"] is True
            and all(self._fresh(key, now) for key in self._EVIDENCE)
        )

    def _evaluate(self, now: float) -> None:
        if self.state == MissionState.LOCALIZING and self._readiness_ok(now):
            self.state = MissionState.READY
            self.reason = "ready"
            self._send_waypoint = self._progress
        elif self.state == MissionState.READY:
            if not self._fresh("localization", now) or not self._values["localization"]:
                self.state = MissionState.LOCALIZING
                self.reason = "localization_not_ready"
            elif not self._readiness_ok(now):
                self.reason = "readiness_not_satisfied"
        elif self.state == MissionState.NAVIGATING:
            stale = [key for key in self._EVIDENCE if not self._fresh(key, now)]
            if stale:
                self._fault("stale_" + stale[0])
                return
            if self._action_updated is None or now - self._action_updated > self.config.action_timeout_s:
                self._fault("stale_move_base")
                return
            if self._progress_updated is None or now - self._progress_updated > self.config.progress_timeout_s:
                self._fault("stale_progress")
                return
            if self._values["collision"] == "blocked" and self._blocked_since is not None:
                if now - self._blocked_since >= self.config.obstacle_stop_entry_s:
                    self._cancel_active_goal()
                    self.state = MissionState.PAUSED_OBSTACLE
                    self.reason = "obstacle_persisted"
                    self._resume_requested = False
                    self._clear_since = None
        elif self.state == MissionState.PAUSED_OBSTACLE:
            if self._values["collision"] == "estop":
                self._fault("emergency_stop")
                return
            stale = [key for key in self._EVIDENCE if not self._fresh(key, now)]
            if stale and stale[0] != "collision":
                self._fault("stale_" + stale[0])
                return
            clear_stable = bool(
                self._values["collision"] == "clear"
                and self._clear_since is not None
                and now - self._clear_since >= self.config.obstacle_clear_s
            )
            if self._resume_requested and clear_stable and self._readiness_ok(now):
                self.state = MissionState.READY
                self.reason = "obstacle_cleared_operator_resume"
                self._resume_requested = False
                self._send_waypoint = self._progress

    def _cancel_active_goal(self) -> None:
        if self._goal_active:
            self._cancel_goal = True
        self._goal_active = False

    def _fault(self, reason: str) -> None:
        if self.state == MissionState.FAULT:
            return
        self._cancel_active_goal()
        # Fault handling is an unconditional cancellation command. This remains
        # safe when no goal is active and covers races with action activation.
        self._cancel_goal = True
        self.state = MissionState.FAULT
        self.reason = reason
        self._terminal_status = "FAULT"

    def _impossible(self, reason: str) -> None:
        if self.state not in (MissionState.DISARMED, MissionState.FAULT):
            self._fault(reason)

    def _to_disarmed(self, reason: str) -> None:
        self._cancel_active_goal()
        self.state = MissionState.DISARMED
        self.reason = reason
        self._route = None
        self._waypoint_count = 0
        self._progress = 0
        self._values.update({
            "localization": False,
            "geofence": False,
            "collision": "unknown",
            "slope": "unknown",
            "mode": False,
            "driver": False,
        })
        self._updated.clear()
        self._action_updated = None
        self._progress_updated = None
        self._blocked_since = None
        self._clear_since = None
        self._resume_requested = False
        self._terminal_status = ""
