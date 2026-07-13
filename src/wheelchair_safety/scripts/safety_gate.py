#!/usr/bin/env python3
"""Fail-closed ROS1 velocity gate with a ROS-independent decision core."""

from dataclasses import dataclass, field
import math
import re
import threading
import time
from typing import Dict, Optional, Tuple

UNKNOWN, CLEAR, STOP, HOLD = 0, 1, 2, 3
DISARMED, STATE_CLEAR, STOPPED, LATCHED, FAULT = range(5)

_REASON_NAMES = (
    "ESTOP", "STALE_CMD", "MODE", "GEOFENCE", "COLLISION", "LOCALIZATION",
    "DRIVER", "INVALID_CMD", "CLOCK", "STALE_INTENT", "INTERNAL_FAULT",
    "STARTUP", "SENSOR_STALE", "COLLISION_BLIND", "COLLISION_TTC",
    "COLLISION_DISTANCE", "SLOPE", "IMU_UNCALIBRATED", "ROUTE_MANIFEST",
    "GRAPH_TOPOLOGY", "TF", "BACKPRESSURE", "DEADLINE_MISS", "MANUAL_OVERRIDE",
    "HARDWARE_UNVERIFIED", "MAP_MISMATCH", "COLLISION_OCCLUDED",
    "LOCALIZATION_INCONSISTENT", "RESOURCE", "CORRUPT_DATA", "RESET_REJECTED",
    "INPUT_UNKNOWN", "ROUTE_STATE", "ODOM_STALE", "IMU_STALE", "LIDAR_STALE",
    "POLICY_MISMATCH",
)
REASONS = {name: 1 << bit for bit, name in enumerate(_REASON_NAMES)}
DEFINED_REASON_MASK = (1 << 37) - 1
for _name, _value in REASONS.items():
    globals()[_name] = _value

_HASH_RE = re.compile(r"^[0-9a-f]{64}$")
TOPOLOGY_TTL_S = 0.75
# An arm edge may wait only for a current pair join; this matches the 0.10 s
# slope evidence TTL and never extends any evidence authority window.
ARM_PENDING_TTL_S = 0.10
_REQUIRED_POLICY_KEYS = frozenset(
    ("geofence", "collision", "slope", "localization", "mode", "driver", "topology"))
_DEFAULT_POLICY_SHA256 = {
    "geofence": "93ca862dac1fbdd5914d93b2d2c325fe2742aef2a05289d44d0d4fe45989de57",
    "collision": "5850bb0cd84bc04f4f9cdc78cd347640a3f60f66241ad3f37c196ad63cbeba18",
    "slope": "171d0febf5f3a691d1500d7b7839ef8f4a04637545b79dcb95d825bead7f6d0d",
    "localization": "5d84ea824c98a53639a480ed162a62f015600ca0a0460df7186d5839303d52e8",
    "mode": "5d84ea824c98a53639a480ed162a62f015600ca0a0460df7186d5839303d52e8",
    "driver": "5d84ea824c98a53639a480ed162a62f015600ca0a0460df7186d5839303d52e8",
    "topology": "93941ad3312c3f3c26da99863c785a1034aa283cd89d1db65d675cc4dbfb6f80",
}



@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0

    def is_zero(self) -> bool:
        return all(value == 0.0 for value in self.values())

    def values(self) -> Tuple[float, ...]:
        return (self.linear_x, self.linear_y, self.linear_z,
                self.angular_x, self.angular_y, self.angular_z)


@dataclass(frozen=True)
class SignalEvidence:
    """Untrusted permission with independently checked source and receipt times."""

    state: int = UNKNOWN
    source_stamp_s: Optional[float] = None
    receipt_stamp_s: Optional[float] = None
    reason_mask: int = 0
    source: str = ""
    policy_sha256: str = ""
    sequence: Optional[int] = None
    max_linear_mps: Optional[float] = None
    max_angular_rps: Optional[float] = None


@dataclass(frozen=True)
class StructuredEvidence:
    """ROS-independent subset of an authoritative structured status."""

    sequence: int
    clear: bool
    source_stamp_s: float
    evaluation_stamp_s: float
    receipt_stamp_s: float
    reason_mask: int
    source: str
    policy_sha256: str
    max_linear_mps: Optional[float] = None
    max_angular_rps: Optional[float] = None


@dataclass(frozen=True)
class GenericEvidence:
    """ROS-independent subset of a generic SafetySignal."""

    sequence: int
    state: int
    stamp_s: float
    receipt_stamp_s: float
    reason_mask: int
    source: str
    policy_sha256: str


class EvidencePairBuffer:
    """Bounded latest-sequence join with a fail-closed committed CLEAR hold."""

    def __init__(self, signal_names, ttl_s, future_tolerance_s, stamp_semantics):
        self.signal_names = tuple(signal_names)
        self.ttl_s = ttl_s
        self.future_tolerance_s = future_tolerance_s
        self.stamp_semantics = stamp_semantics
        self.sequence = None
        self.status = None
        self.signals = {name: None for name in self.signal_names}
        self._last_sequences = {"status": None}
        self._last_sequences.update({name: None for name in self.signal_names})
        self.poisoned = False
        self.committed_status = None
        self.committed_signals = {name: None for name in self.signal_names}
        self.hold_latched = False
        self._lock = threading.RLock()
        self._last_stamps = {
            "status_source": None,
            "status_evaluation": None,
            "status_receipt": None,
        }
        self._last_stamps.update({
            "%s_stamp" % name: None for name in self.signal_names
        })
        self._last_stamps.update({
            "%s_receipt" % name: None for name in self.signal_names
        })

    @staticmethod
    def _sequence(value):
        return (isinstance(value, int) and not isinstance(value, bool) and
                0 <= value <= 0xffffffff)

    def _accept_sequence(self, stream, value):
        sequence = value.sequence
        if not self._sequence(sequence):
            self.reject()
            return False
        previous_sequence = self._last_sequences[stream]
        previous_value = self.status if stream == "status" else self.signals[stream]
        if previous_sequence is not None and sequence < previous_sequence:
            self.reject()
            return False
        if previous_sequence == sequence:
            if previous_value != value:
                self.reject()
                return False
            return not self.poisoned
        incomplete = self.sequence is not None and (
            self.status is None or self.status.sequence != self.sequence or
            any(signal is None or signal.sequence != self.sequence
                for signal in self.signals.values()))
        if ((self.committed_status is not None and
             sequence > self.committed_status.sequence + 1) or
                (self.sequence is not None and sequence > self.sequence and incomplete)):
            self.hold_latched = True
        self._last_sequences[stream] = sequence
        if stream == "status":
            self.status = value
        else:
            self.signals[stream] = value
        if self.sequence is None or sequence > self.sequence:
            self.poisoned = False
            self.sequence = sequence
        return not self.poisoned

    def reject(self):
        with self._lock:
            self.poisoned = True
            self.status = None
            self.signals = {name: None for name in self.signal_names}
            self.committed_status = None
            self.committed_signals = {name: None for name in self.signal_names}

    def update_status(self, status):
        with self._lock:
            try:
                if self._accept_sequence("status", status):
                    self._commit_complete_clear_unlocked()
            except Exception:
                self.reject()

    def update_signal(self, name, signal):
        with self._lock:
            try:
                if name not in self.signals:
                    self.reject()
                    return
                if self._accept_sequence(name, signal):
                    self._commit_complete_clear_unlocked()
            except Exception:
                self.reject()

    def _commit_complete_clear_unlocked(self):
        try:
            status = self.status
            signals = tuple(self.signals[name] for name in self.signal_names)
            if (self.poisoned or status is None or
                    status.sequence != self.sequence or
                    any(signal is None or signal.sequence != self.sequence
                        for signal in signals)):
                return
            if (status.clear is not True or
                    not isinstance(status.reason_mask, int) or
                    isinstance(status.reason_mask, bool) or
                    status.reason_mask != 0):
                return
            if any(not isinstance(signal.state, int) or
                   isinstance(signal.state, bool) or
                   signal.state != CLEAR or
                   not isinstance(signal.reason_mask, int) or
                   isinstance(signal.reason_mask, bool) or
                   signal.reason_mask != 0
                   for signal in signals):
                return
            receipts = (status.receipt_stamp_s,) + tuple(
                signal.receipt_stamp_s for signal in signals)
            result = self._pair_evidence(
                status, signals, max(receipts), update_stamps=True)
            if result.state != CLEAR or result.reason_mask != 0:
                return
            self.committed_status = status
            self.committed_signals = {
                name: signal for name, signal in zip(self.signal_names, signals)
            }
            self.hold_latched = False
        except Exception:
            self.reject()

    @staticmethod
    def _text_valid(value):
        return (isinstance(value, str) and bool(value) and
                len(value.encode("utf-8")) <= 64 and
                not any(ord(char) < 32 or ord(char) == 127 for char in value))

    def _unknown(self, receipt=None):
        return SignalEvidence(
            UNKNOWN, None, receipt, CORRUPT_DATA | INPUT_UNKNOWN, "pairing_invalid",
            "0" * 64, self.sequence)

    def evidence(self, now_s):
        with self._lock:
            return self._evidence_unlocked(now_s)

    def diagnostic_snapshot(self):
        with self._lock:
            return {
                "sequence": self.sequence,
                "status": None if self.status is None else self.status.sequence,
                "signals": {
                    name: None if signal is None else signal.sequence
                    for name, signal in self.signals.items()
                },
                "committed_status": (
                    None if self.committed_status is None
                    else self.committed_status.sequence
                ),
                "committed_signals": {
                    name: None if signal is None else signal.sequence
                    for name, signal in self.committed_signals.items()
                },
                "poisoned": self.poisoned,
                "hold_latched": self.hold_latched,
            }

    def _pair_evidence(self, status, signals, now_s, update_stamps):
        numeric = (now_s, status.source_stamp_s, status.evaluation_stamp_s,
                   status.receipt_stamp_s, status.reason_mask)
        if (not all(_finite(value) for value in numeric[:4]) or
                any(value <= 0.0 for value in numeric[:4]) or
                not isinstance(status.reason_mask, int) or
                isinstance(status.reason_mask, bool) or
                status.reason_mask < 0 or status.reason_mask & ~DEFINED_REASON_MASK or
                not isinstance(status.clear, bool) or
                not self._text_valid(status.source) or
                not _HASH_RE.fullmatch(status.policy_sha256)):
            raise ValueError("invalid status")
        expected_stamp = (status.evaluation_stamp_s if self.stamp_semantics == "evaluation"
                          else status.source_stamp_s)
        expected_state = CLEAR if status.clear else STOP
        timestamps = {
            "status_source": status.source_stamp_s,
            "status_evaluation": status.evaluation_stamp_s,
            "status_receipt": status.receipt_stamp_s,
        }
        for name, signal in zip(self.signal_names, signals):
            if (not isinstance(signal.state, int) or isinstance(signal.state, bool) or
                    not isinstance(signal.reason_mask, int) or
                    isinstance(signal.reason_mask, bool) or
                    signal.reason_mask < 0 or signal.reason_mask & ~DEFINED_REASON_MASK or
                    signal.sequence != status.sequence or
                    signal.state != expected_state or
                    signal.reason_mask != status.reason_mask or
                    signal.source != status.source or
                    signal.policy_sha256 != status.policy_sha256 or
                    signal.stamp_s != expected_stamp or
                    not all(_finite(value) for value in
                            (signal.stamp_s, signal.receipt_stamp_s)) or
                    signal.stamp_s <= 0.0 or signal.receipt_stamp_s <= 0.0):
                raise ValueError("pair mismatch")
            timestamps["%s_stamp" % name] = signal.stamp_s
            timestamps["%s_receipt" % name] = signal.receipt_stamp_s
        if not _finite(now_s):
            raise ValueError("invalid clock")
        for key, stamp in timestamps.items():
            if (now_s - stamp > self.ttl_s + 1e-12 or
                    stamp - now_s > self.future_tolerance_s + 1e-12 or
                    (self._last_stamps[key] is not None and
                     stamp < self._last_stamps[key])):
                raise ValueError("invalid timestamp")
        for cap in (status.max_linear_mps, status.max_angular_rps):
            if cap is not None and (not _finite(cap) or cap < 0.0):
                raise ValueError("invalid cap")
        if update_stamps:
            self._last_stamps.update(timestamps)
        return SignalEvidence(
            expected_state, expected_stamp, min(timestamps[key] for key in timestamps
                                                if key.endswith("receipt")),
            status.reason_mask, status.source, status.policy_sha256,
            status.sequence, status.max_linear_mps, status.max_angular_rps)

    def _partial_member_permissive(self, value, is_status, name, now_s,
                                   committed_status):
        if (not self._sequence(value.sequence) or value.sequence != self.sequence or
                not isinstance(value.reason_mask, int) or
                isinstance(value.reason_mask, bool) or value.reason_mask != 0 or
                value.source != committed_status.source or
                value.policy_sha256 != committed_status.policy_sha256 or
                not self._text_valid(value.source) or
                not _HASH_RE.fullmatch(value.policy_sha256) or not _finite(now_s)):
            return False
        if is_status:
            if (value.clear is not True or
                    not all(_finite(stamp) and stamp > 0.0 for stamp in
                            (value.source_stamp_s, value.evaluation_stamp_s,
                             value.receipt_stamp_s)) or
                    any(cap is not None and (not _finite(cap) or cap < 0.0)
                        for cap in (value.max_linear_mps, value.max_angular_rps)) or
                    any(new_cap is not None and
                        (new_cap <= 0.0 or old_cap is None or new_cap < old_cap)
                        for new_cap, old_cap in zip(
                            (value.max_linear_mps, value.max_angular_rps),
                            (committed_status.max_linear_mps,
                             committed_status.max_angular_rps)))):
                return False
            timestamps = {
                "status_source": value.source_stamp_s,
                "status_evaluation": value.evaluation_stamp_s,
                "status_receipt": value.receipt_stamp_s,
            }
        else:
            if (not isinstance(value.state, int) or isinstance(value.state, bool) or
                    value.state != CLEAR or
                    not all(_finite(stamp) and stamp > 0.0 for stamp in
                            (value.stamp_s, value.receipt_stamp_s))):
                return False
            timestamps = {
                "%s_stamp" % name: value.stamp_s,
                "%s_receipt" % name: value.receipt_stamp_s,
            }
        return all(
            now_s - stamp <= self.ttl_s + 1e-12 and
            stamp - now_s <= self.future_tolerance_s + 1e-12 and
            (self._last_stamps[key] is None or stamp >= self._last_stamps[key])
            for key, stamp in timestamps.items())

    def _held_committed_evidence(self, now_s):
        committed_status = self.committed_status
        committed_signals = tuple(self.committed_signals.values())
        if (committed_status is None or
                any(signal is None for signal in committed_signals) or
                self.sequence is None or self.sequence <= committed_status.sequence or
                self.hold_latched):
            return None
        members = [(self.status, True, "status")]
        members.extend((signal, False, name)
                       for name, signal in self.signals.items())
        for value, is_status, name in members:
            if value is not None and value.sequence > committed_status.sequence:
                if not self._partial_member_permissive(
                        value, is_status, name, now_s, committed_status):
                    raise ValueError("unsafe partial generation")
        result = self._pair_evidence(
            committed_status, committed_signals, now_s, update_stamps=False)
        if result.state != CLEAR or result.reason_mask != 0:
            return None
        return result

    def _evidence_unlocked(self, now_s):
        status = self.status
        signals = tuple(self.signals.values())
        complete = (
            status is not None
            and status.sequence == self.sequence
            and all(
                signal is not None and signal.sequence == self.sequence
                for signal in signals
            )
        )
        if self.poisoned:
            return self._unknown()
        try:
            if not complete:
                held = self._held_committed_evidence(now_s)
                return self._unknown() if held is None else held
            result = self._pair_evidence(status, signals, now_s, update_stamps=True)
            self.committed_status = status
            self.committed_signals = {
                name: signal for name, signal in zip(self.signal_names, signals)
            }
            self.hold_latched = False
            return result
        except Exception:
            self.reject()
            return self._unknown()
    def evidence_and_pending_arm_state(self, now_s):
        """Capture evidence and arm classification under one pair lock."""
        with self._lock:
            evidence = self._evidence_unlocked(now_s)
            return evidence, self._pending_arm_state_unlocked(now_s, evidence)

    def pending_arm_state(self, now_s, evidence=None):
        """Classify this pair for an already edge-qualified pending arm request."""
        with self._lock:
            if evidence is None:
                evidence = self._evidence_unlocked(now_s)
            return self._pending_arm_state_unlocked(now_s, evidence)

    def _pending_arm_state_unlocked(self, now_s, evidence):
        if self.poisoned or not _finite(now_s):
            return "drop"
        status = self.status
        signals = tuple(self.signals.values())
        complete = (
            status is not None
            and status.sequence == self.sequence
            and all(signal is not None and signal.sequence == self.sequence
                    for signal in signals)
        )
        if complete:
            return ("complete" if evidence.state == CLEAR and evidence.reason_mask == 0
                    else "drop")
        members = [(status, True, "status")]
        members.extend((signal, False, name)
                       for name, signal in self.signals.items())
        for value, is_status, name in members:
            if value is None or value.sequence != self.sequence:
                continue
            if not self._pending_arm_value_permissive(
                    value, is_status, name, now_s):
                return "drop"
        return "wait"

    def _pending_arm_value_permissive(self, value, is_status, name, now_s):
        if (not self._sequence(value.sequence) or
                not isinstance(value.reason_mask, int) or
                isinstance(value.reason_mask, bool) or value.reason_mask != 0 or
                not self._text_valid(value.source) or
                not _HASH_RE.fullmatch(value.policy_sha256)):
            return False
        if is_status:
            if (value.clear is not True or
                    not all(_finite(stamp) and stamp > 0.0 for stamp in
                            (value.source_stamp_s, value.evaluation_stamp_s,
                             value.receipt_stamp_s))):
                return False
            timestamps = {
                "status_source": value.source_stamp_s,
                "status_evaluation": value.evaluation_stamp_s,
                "status_receipt": value.receipt_stamp_s,
            }
            caps = (value.max_linear_mps, value.max_angular_rps)
        else:
            if (not isinstance(value.state, int) or isinstance(value.state, bool) or
                    value.state != CLEAR or
                    not all(_finite(stamp) and stamp > 0.0 for stamp in
                            (value.stamp_s, value.receipt_stamp_s))):
                return False
            timestamps = {
                "%s_stamp" % name: value.stamp_s,
                "%s_receipt" % name: value.receipt_stamp_s,
            }
            caps = ()
        if any(cap is not None and (not _finite(cap) or cap < 0.0) for cap in caps):
            return False
        return all(
            now_s - stamp <= self.ttl_s + 1e-12 and
            stamp - now_s <= self.future_tolerance_s + 1e-12 and
            (self._last_stamps[key] is None or stamp >= self._last_stamps[key])
            for key, stamp in timestamps.items()
        )


@dataclass(frozen=True)
class SafetyConfig:
    stale_timeout_s: float = 0.30
    max_linear_speed: float = 0.55
    max_angular_speed: float = 0.85
    command_ttl_s: Optional[float] = None
    intent_ttl_s: float = 0.30
    geofence_ttl_s: float = 0.25
    collision_ttl_s: float = 0.30
    slope_ttl_s: float = 0.10
    localization_ttl_s: float = 0.25
    mode_ttl_s: float = 0.15
    driver_ttl_s: float = 0.15
    future_tolerance_s: float = 0.05
    publication_period_s: float = 0.02
    deadline_limit_s: float = 0.05
    stationary_linear_mps: float = 0.01
    stationary_angular_rps: float = 0.02
    expected_policy_sha256: Dict[str, str] = field(
        default_factory=lambda: dict(_DEFAULT_POLICY_SHA256))
    release_manifest_sha256: str = ""

    def __post_init__(self):
        if not (0.0 < self.max_linear_speed <= 0.55 and
                0.0 < self.max_angular_speed <= 0.85):
            raise ValueError("configured caps may not exceed immutable software-RC caps")
        values = (self.stale_timeout_s,
                  self.stale_timeout_s if self.command_ttl_s is None else self.command_ttl_s,
                  self.intent_ttl_s, self.geofence_ttl_s, self.collision_ttl_s,
                  self.slope_ttl_s, self.localization_ttl_s, self.mode_ttl_s,
                  self.driver_ttl_s, self.future_tolerance_s,
                  self.publication_period_s, self.deadline_limit_s)
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError("timeouts and periods must be finite and positive")
        keys = set(self.expected_policy_sha256)
        if keys != _REQUIRED_POLICY_KEYS:
            missing = sorted(_REQUIRED_POLICY_KEYS - keys)
            unknown = sorted(keys - _REQUIRED_POLICY_KEYS)
            raise ValueError("expected policy hashes require exact keys; missing=%s unknown=%s" %
                             (missing, unknown))
        if any(not _HASH_RE.fullmatch(value)
               for value in self.expected_policy_sha256.values()):
            raise ValueError("expected policy hashes must be lowercase SHA-256")

    @property
    def cmd_ttl_s(self) -> float:
        return self.stale_timeout_s if self.command_ttl_s is None else self.command_ttl_s

    @property
    def activation_grace_s(self) -> float:
        """Bounded first-command grace after HOLD releases motion authority."""
        return 0.10

    @property
    def topology_ttl_s(self) -> float:
        """Topology evidence lifetime is an immutable software-RC boundary."""
        return TOPOLOGY_TTL_S


@dataclass(frozen=True)
class GateInputs:
    cmd: Optional[VelocityCommand] = None
    cmd_age_s: Optional[float] = None  # compatibility; source and receipt use this age
    now_s: Optional[float] = None
    cmd_source_stamp_s: Optional[float] = None
    cmd_receipt_stamp_s: Optional[float] = None
    motion_intent: Optional[SignalEvidence] = None
    geofence: Optional[SignalEvidence] = None
    collision: Optional[SignalEvidence] = None
    slope: Optional[SignalEvidence] = None
    localization: Optional[SignalEvidence] = None
    mode: Optional[SignalEvidence] = None
    driver: Optional[SignalEvidence] = None
    topology: Optional[SignalEvidence] = None
    e_stop: Optional[bool] = None
    e_stop_reset: bool = False
    arm_request: bool = False
    reset_driver_healthy: bool = False
    manual_or_disarmed: bool = False
    stationary: bool = False
    mission_cancelled: bool = False
    graph_valid: bool = False  # inert compatibility field; only fresh topology evidence grants
    deadline_missed: bool = False
    backpressure: bool = False
    internal_fault: bool = False
    clock_fault: bool = False
    # Removed permissive Bool API names remain accepted only as explicit STOP evidence.
    geofence_ok: Optional[bool] = None
    mode_allowed: Optional[bool] = None
    collision_stop: Optional[bool] = None


@dataclass(frozen=True)
class GateDecision:
    command: VelocityCommand
    reason: str
    e_stop_latched: bool
    reason_mask: int = 0
    state: int = DISARMED
    armed: bool = False
    ages: Dict[str, float] = field(default_factory=dict)
    deadline_miss_count: int = 0
    dropped_input_count: int = 0


class SafetyGateCore:
    """Stateful fail-closed authority. CLEAR never overrides a concurrently observed STOP."""

    SIGNALS = (
        ("motion_intent", STALE_INTENT), ("geofence", GEOFENCE),
        ("collision", COLLISION), ("slope", SLOPE),
        ("localization", LOCALIZATION), ("mode", MODE), ("driver", DRIVER),
        ("topology", GRAPH_TOPOLOGY),
    )

    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        self.e_stop_latched = False
        self.armed = False
        self.last_now_s = None
        self.last_source_stamps = {}
        self.last_receipt_stamps = {}
        self.last_sequences = {}
        self.deadline_miss_count = 0
        self.dropped_input_count = 0
        self.last_backpressure_input = ""
        self.last_backpressure_previous = None
        self.last_backpressure_current = None
        self.last_motion_intent_state = None
        self.activation_started_s = None
        self.activation_command_floor_s = None
        self.awaiting_first_command = False
        self._reset_level = False
        self._state_lock = threading.RLock()
        self.last_evidence = {}
        self._arm_level = False

    def reset(self) -> None:
        """Administrative test reset; runtime reset must use guarded GateInputs."""
        self.e_stop_latched = False
        self.armed = False
        self.last_now_s = None
        self.last_source_stamps.clear()
        self.last_receipt_stamps.clear()
        self.last_sequences.clear()
        self.last_backpressure_input = ""
        self.last_backpressure_previous = None
        self.last_backpressure_current = None
        self.last_motion_intent_state = None
        self.activation_started_s = None
        self.activation_command_floor_s = None
        self.awaiting_first_command = False
        self._reset_level = False
        self.last_evidence.clear()
        self._arm_level = False

    def latch_estop(self) -> None:
        """Callback-safe latch; only guarded reset in _evaluate clears it."""
        with self._state_lock:
            self.e_stop_latched = True
            self.armed = False

    def evaluate(self, inputs: GateInputs) -> GateDecision:
        with self._state_lock:
            try:
                return self._evaluate(inputs)
            except Exception:
                self.armed = False
                return self._stop(INTERNAL_FAULT | CORRUPT_DATA | STARTUP, {})

    def _evaluate(self, inputs: GateInputs) -> GateDecision:
        now = inputs.now_s
        mask = 0
        ages = {}
        if now is not None and (not _finite(now) or
                                (self.last_now_s is not None and now < self.last_now_s)):
            mask |= CLOCK
        elif now is not None:
            self.last_now_s = now

        if inputs.e_stop is True:
            self.e_stop_latched = True
            self.armed = False
        elif inputs.e_stop is None:
            mask |= ESTOP | INPUT_UNKNOWN
        if inputs.clock_fault:
            mask |= CLOCK

        if inputs.internal_fault:
            mask |= INTERNAL_FAULT
        if inputs.deadline_missed:
            self.deadline_miss_count += 1
            mask |= DEADLINE_MISS
        if inputs.backpressure:
            mask |= BACKPRESSURE

        intent_faults = 0
        for name, base_reason in self.SIGNALS:
            evidence = getattr(inputs, name)
            faults = self._evidence_faults(name, evidence, base_reason, now, ages)
            mask |= faults
            if name == "motion_intent":
                intent_faults = faults

        intent = inputs.motion_intent
        intent_state = (intent.state if intent is not None and intent_faults == 0
                        else None)
        if intent_state == HOLD:
            self.activation_started_s = None
            self.activation_command_floor_s = None
            self.awaiting_first_command = False
        elif intent_state == CLEAR and self.last_motion_intent_state == HOLD:
            self.activation_started_s = now
            self.activation_command_floor_s = max(intent.source_stamp_s,
                                                  intent.receipt_stamp_s)
            self.awaiting_first_command = True
        elif intent_state != CLEAR:
            self.activation_started_s = None
            self.activation_command_floor_s = None
            self.awaiting_first_command = False
        self.last_motion_intent_state = intent_state

        activation_grace = False
        if intent_state == HOLD:
            if inputs.cmd is None:
                ages["command"] = -1.0
                command_faults = 0
            else:
                command_faults = self._command_faults(inputs, now, ages)
                command_faults &= ~(STALE_CMD | INPUT_UNKNOWN)
        else:
            command_faults = self._command_faults(inputs, now, ages)
            if self.awaiting_first_command:
                if self._command_precedes_activation(inputs):
                    command_faults |= STALE_CMD
                only_unavailable = not command_faults & ~(STALE_CMD | INPUT_UNKNOWN)
                within_grace = (
                    _finite(now) and _finite(self.activation_started_s) and
                    now - self.activation_started_s <=
                    self.config.activation_grace_s + 1e-12
                )
                if command_faults == 0:
                    self.awaiting_first_command = False
                    self.activation_started_s = None
                    self.activation_command_floor_s = None
                elif only_unavailable and within_grace:
                    command_faults = 0
                    activation_grace = True
        mask |= command_faults

        # Legacy boolean fields can only add stops; absence never grants permission.
        if inputs.geofence_ok is False:
            mask |= GEOFENCE
        if inputs.mode_allowed is False:
            mask |= MODE
        if inputs.collision_stop is True:
            mask |= COLLISION

        if mask & ~DEFINED_REASON_MASK:
            mask = (mask & DEFINED_REASON_MASK) | INTERNAL_FAULT

        reset_requested = bool(inputs.e_stop_reset) and not self._reset_level
        arm_requested = bool(inputs.arm_request) and not self._arm_level
        self._reset_level = bool(inputs.e_stop_reset)
        self._arm_level = bool(inputs.arm_request)
        if reset_requested:
            # A healthy manual/AUTO_DISABLED driver pair is reset evidence, not
            # motion-ready authority.  Its intentional STOP reason is excluded
            # only for reset eligibility; AUTO_READY remains required to arm.
            reset_faults = mask
            if inputs.reset_driver_healthy:
                # Only the expected manual/disarmed mode is excluded.  The
                # independent reset-health proof rejects malformed/faulted
                # DriverStatus evidence before this branch is reachable.
                reset_faults &= ~(MODE | DRIVER)
            reset_ok = (self.e_stop_latched and inputs.e_stop is False and
                        inputs.reset_driver_healthy and inputs.manual_or_disarmed and inputs.stationary and
                        inputs.mission_cancelled and reset_faults == 0)
            if reset_ok:
                self.e_stop_latched = False
                self.armed = False
            else:
                mask |= RESET_REJECTED

        if self.e_stop_latched:
            mask |= ESTOP

        # Reset and arm are intentionally distinct evaluations and requests are
        # one-shot rising edges, so a latched publisher cannot replay either.
        if arm_requested and not reset_requested and mask == 0:
            self.armed = True
        elif mask != 0:
            self.armed = False

        if not self.armed:
            if mask == 0:
                return GateDecision(VelocityCommand(), "startup", False, STARTUP,
                                    DISARMED, False, ages, self.deadline_miss_count,
                                    self.dropped_input_count)
            return self._stop(mask | STARTUP, ages)

        if intent_state == HOLD:
            return GateDecision(VelocityCommand(), "hold", self.e_stop_latched, 0,
                                STATE_CLEAR, True, ages, self.deadline_miss_count,
                                self.dropped_input_count)
        if activation_grace:
            return GateDecision(VelocityCommand(), "activation_grace",
                                self.e_stop_latched, 0, STATE_CLEAR, True, ages,
                                self.deadline_miss_count, self.dropped_input_count)

        capped = self._cap(inputs.cmd, inputs)
        reason = "speed_cap" if capped != inputs.cmd else "nominal"
        return GateDecision(capped, reason, self.e_stop_latched, 0, STATE_CLEAR,
                            True, ages, self.deadline_miss_count,
                            self.dropped_input_count)

    @staticmethod
    def _command_shape_faults(command):
        if command is None:
            return 0
        if not all(_finite(value) for value in command.values()):
            return INVALID_CMD | CORRUPT_DATA
        if any(value != 0.0 for value in (command.linear_y, command.linear_z,
                                          command.angular_x, command.angular_y)):
            return INVALID_CMD
        return 0

    def _command_precedes_activation(self, inputs):
        if inputs.cmd is None or self.activation_command_floor_s is None:
            return True
        if inputs.cmd_age_s is not None:
            return False
        return (inputs.cmd_source_stamp_s is None or
                inputs.cmd_receipt_stamp_s is None or
                inputs.cmd_source_stamp_s < self.activation_command_floor_s or
                inputs.cmd_receipt_stamp_s < self.activation_command_floor_s)

    def _command_faults(self, inputs, now, ages):
        if inputs.cmd is None:
            ages["command"] = -1.0
            return STALE_CMD | INPUT_UNKNOWN
        shape_faults = self._command_shape_faults(inputs.cmd)
        if shape_faults:
            return shape_faults
        if inputs.cmd_age_s is not None:
            if not _finite(inputs.cmd_age_s) or inputs.cmd_age_s < 0.0:
                return CLOCK | STALE_CMD
            ages["command"] = inputs.cmd_age_s
            return STALE_CMD if inputs.cmd_age_s > self.config.cmd_ttl_s + 1e-12 else 0
        return self._timestamps("command", inputs.cmd_source_stamp_s,
                                inputs.cmd_receipt_stamp_s, now,
                                self.config.cmd_ttl_s, STALE_CMD, ages)

    def _evidence_faults(self, name, evidence, base_reason, now, ages):
        if evidence is None:
            ages[name] = -1.0
            return base_reason | INPUT_UNKNOWN
        mask = self._timestamps(name, evidence.source_stamp_s,
                                evidence.receipt_stamp_s, now,
                                getattr(self.config, name.replace("motion_intent", "intent") + "_ttl_s"),
                                (base_reason if name in ("motion_intent", "topology")
                                 else SENSOR_STALE),
                                ages)
        if evidence.state not in (UNKNOWN, CLEAR, STOP, HOLD):
            mask |= CORRUPT_DATA | base_reason
        elif evidence.state == UNKNOWN:
            mask |= INPUT_UNKNOWN | base_reason | evidence.reason_mask
        elif evidence.state == STOP:
            mask |= evidence.reason_mask or base_reason
        elif evidence.reason_mask != 0:
            mask |= CORRUPT_DATA | evidence.reason_mask
        if evidence.reason_mask & ~DEFINED_REASON_MASK:
            mask |= INTERNAL_FAULT
        if (not evidence.source or len(evidence.source.encode("utf-8")) > 64 or
                any(ord(char) < 32 or ord(char) == 127 for char in evidence.source)):
            mask |= CORRUPT_DATA | base_reason
        if name == "topology" and evidence.source != "topology_guard":
            mask |= CORRUPT_DATA | GRAPH_TOPOLOGY
        expected = self.config.expected_policy_sha256.get(name)
        if name != "motion_intent" and (
                expected is None or not _HASH_RE.fullmatch(evidence.policy_sha256) or
                evidence.policy_sha256 != expected):
            mask |= POLICY_MISMATCH
        for cap in (evidence.max_linear_mps, evidence.max_angular_rps):
            if cap is not None and (not _finite(cap) or cap < 0.0):
                mask |= CORRUPT_DATA | base_reason
        if name == "motion_intent" and (
                evidence.max_linear_mps is None or evidence.max_angular_rps is None):
            mask |= CORRUPT_DATA | STALE_INTENT
        if (name == "motion_intent" and evidence.state == HOLD and
                (evidence.max_linear_mps != 0.0 or
                 evidence.max_angular_rps != 0.0)):
            mask |= CORRUPT_DATA | STALE_INTENT
        if name != "motion_intent" and evidence.state == HOLD:
            mask |= CORRUPT_DATA | base_reason
        if name in ("motion_intent", "topology") and evidence.sequence is not None:
            previous = self.last_sequences.get(name)
            fingerprint = (
                evidence.state, evidence.source_stamp_s, evidence.receipt_stamp_s,
                evidence.reason_mask, evidence.source, evidence.policy_sha256,
                evidence.max_linear_mps, evidence.max_angular_rps,
            )
            if previous is not None:
                if evidence.sequence < previous:
                    mask |= CLOCK
                elif evidence.sequence == previous:
                    if self.last_evidence.get(name) != fingerprint:
                        mask |= CLOCK | CORRUPT_DATA
                elif evidence.sequence > previous + 1:
                    self.dropped_input_count += evidence.sequence - previous - 1
                    self.last_backpressure_input = name
                    self.last_backpressure_previous = previous
                    self.last_backpressure_current = evidence.sequence
            if previous is None or evidence.sequence > previous:
                self.last_sequences[name] = evidence.sequence
                self.last_evidence[name] = fingerprint
        return mask

    def _timestamps(self, name, source, receipt, now, ttl, stale_reason, ages):
        if now is None or source is None or receipt is None:
            ages[name] = -1.0
            return stale_reason | INPUT_UNKNOWN
        if not all(_finite(v) for v in (source, receipt, now)):
            ages[name] = -1.0
            return CLOCK | stale_reason
        if source <= 0.0 or receipt <= 0.0:
            ages[name] = -1.0
            return CLOCK | stale_reason
        source_age, receipt_age = now - source, now - receipt
        ages[name] = max(source_age, receipt_age)
        mask = 0
        if source_age < -self.config.future_tolerance_s - 1e-12 or receipt_age < -self.config.future_tolerance_s - 1e-12:
            mask |= CLOCK
        if source_age > ttl + 1e-12 or receipt_age > ttl + 1e-12:
            mask |= stale_reason
        previous_source = self.last_source_stamps.get(name)
        previous_receipt = self.last_receipt_stamps.get(name)
        if name in ("motion_intent", "topology"):
            if previous_source is not None and source < previous_source:
                mask |= CLOCK
            if previous_receipt is not None and receipt < previous_receipt:
                mask |= CLOCK
            if previous_source is None or source > previous_source:
                self.last_source_stamps[name] = source
            if previous_receipt is None or receipt > previous_receipt:
                self.last_receipt_stamps[name] = receipt
        return mask

    def _cap(self, cmd, inputs):
        linear, angular = self.config.max_linear_speed, self.config.max_angular_speed
        for evidence in (inputs.motion_intent, inputs.collision, inputs.slope):
            if evidence.max_linear_mps is not None:
                if not _finite(evidence.max_linear_mps) or evidence.max_linear_mps < 0.0:
                    return VelocityCommand()
                linear = min(linear, evidence.max_linear_mps)
            if evidence.max_angular_rps is not None:
                if not _finite(evidence.max_angular_rps) or evidence.max_angular_rps < 0.0:
                    return VelocityCommand()
                angular = min(angular, evidence.max_angular_rps)
        return VelocityCommand(_clamp(cmd.linear_x, -linear, linear),
                               _clamp(cmd.angular_z, -angular, angular))

    def _stop(self, mask, ages):
        fault_bits = INTERNAL_FAULT | CLOCK | DEADLINE_MISS | GRAPH_TOPOLOGY
        state = LATCHED if self.e_stop_latched else (FAULT if mask & fault_bits else STOPPED)
        return GateDecision(VelocityCommand(), _reason_string(mask), self.e_stop_latched,
                            mask, state, False, ages, self.deadline_miss_count,
                            self.dropped_input_count)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _reason_string(mask):
    names = [name.lower() for name in _REASON_NAMES if mask & REASONS[name]]
    return "|".join(names) if names else "nominal"


def _twist_to_command(msg):
    return VelocityCommand(float(msg.linear.x), float(msg.angular.z),
                           float(msg.linear.y), float(msg.linear.z),
                           float(msg.angular.x), float(msg.angular.y))


def _command_to_twist(command):
    from geometry_msgs.msg import Twist
    msg = Twist()
    msg.linear.x, msg.angular.z = command.linear_x, command.angular_z
    return msg


class SafetyGateRosNode:
    """Thin queue-one ROS adapter; callbacks never publish authority directly."""

    def __init__(self):
        import rospy
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool
        from wheelchair_interfaces.msg import (CollisionStatus, DriverStatus, GeofenceStatus,
                                                LocalizationStatus, MotionIntent, SafetySignal,
                                                SafetyState, SlopeStatus)

        p = rospy.get_param
        hashes = p("~expected_policy_sha256", {})
        cfg = SafetyConfig(
            stale_timeout_s=float(p("~command_ttl_s", 0.30)),
            max_linear_speed=float(p("~max_linear_speed", 0.55)),
            max_angular_speed=float(p("~max_angular_speed", 0.85)),
            intent_ttl_s=float(p("~intent_ttl_s", 0.30)),
            geofence_ttl_s=float(p("~geofence_ttl_s", 0.25)),
            collision_ttl_s=float(p("~collision_ttl_s", 0.15)),
            slope_ttl_s=float(p("~slope_ttl_s", 0.10)),
            localization_ttl_s=float(p("~localization_ttl_s", 0.25)),
            mode_ttl_s=float(p("~mode_ttl_s", 0.15)),
            driver_ttl_s=float(p("~driver_ttl_s", 0.15)),
            expected_policy_sha256=dict(hashes),
            release_manifest_sha256=str(p("~release_manifest_sha256", "")),
        )
        self.core, self.cfg = SafetyGateCore(cfg), cfg
        self.evidence = {
            name: None for name, _ in self.core.SIGNALS if name != "topology"
        }
        self.pairs = {
            "geofence": EvidencePairBuffer(("geofence",), cfg.geofence_ttl_s,
                                           cfg.future_tolerance_s, "evaluation"),
            "collision": EvidencePairBuffer(("collision",), cfg.collision_ttl_s,
                                            cfg.future_tolerance_s, "evaluation"),
            "slope": EvidencePairBuffer(("slope",), cfg.slope_ttl_s,
                                        cfg.future_tolerance_s, "evaluation"),
            "localization": EvidencePairBuffer(("localization",), cfg.localization_ttl_s,
                                               cfg.future_tolerance_s, "evaluation"),
            "driver": EvidencePairBuffer(("mode", "driver"), cfg.driver_ttl_s,
                                         cfg.future_tolerance_s, "source"),
        }
        self.last_cmd = self.cmd_source = self.cmd_receipt = None
        self.driver_estop = self.external_estop = None
        self.reset_requested = self.arm_requested = False
        self.reset_level = self.arm_level = None
        self.reset_low_observed = self.arm_low_observed = False
        self.arm_pending_wall = None
        self.manual_or_disarmed = self.reset_driver_healthy = self.stationary = False
        self.mission_cancelled = False
        self.topology_evidence = None
        self.internal_fault = self.backpressure = False
        self.last_tick_wall = self.last_ros_time = self.last_ros_wall = None
        self.sequence = 0
        self._input_lock = threading.RLock()
        self._observability_lock = threading.Lock()
        self._observability_snapshot = None
        self._observability_stop = threading.Event()

        self.pub = rospy.Publisher(p("~output_cmd_topic", "/cmd_vel_safe"), Twist, queue_size=1)
        self.state_pub = rospy.Publisher(p("~state_topic", "/safety/state"), SafetyState, queue_size=1, latch=True)
        self.diag_pub = rospy.Publisher(p("~diagnostics_topic", "/diagnostics"), DiagnosticArray, queue_size=1)
        rospy.Subscriber(p("~input_cmd_topic", "/cmd_vel_nav"), Twist, self._cmd_cb, queue_size=1)
        rospy.Subscriber(p("~intent_topic", "/decision/motion_intent"), MotionIntent, self._intent_cb, queue_size=1)
        for name in ("geofence", "collision", "slope", "localization", "mode"):
            rospy.Subscriber(p("~%s_topic" % name, "/safety/%s" % name), SafetySignal,
                             lambda msg, n=name: self._signal_cb(n, msg), queue_size=1)
        rospy.Subscriber(p("~driver_signal_topic", "/safety/driver"), SafetySignal,
                         lambda msg: self._signal_cb("driver", msg), queue_size=1)
        rospy.Subscriber(p("~geofence_status_topic", "/route_safety/geofence_status"),
                         GeofenceStatus, self._geofence_status_cb, queue_size=1)
        rospy.Subscriber(p("~collision_status_topic", "/safety/collision_status"),
                         CollisionStatus, self._collision_status_cb, queue_size=1)
        rospy.Subscriber(p("~slope_status_topic", "/safety/slope_status"),
                         SlopeStatus, self._slope_status_cb, queue_size=1)
        rospy.Subscriber(p("~localization_status_topic", "/localization/status"),
                         LocalizationStatus, self._localization_status_cb, queue_size=1)
        rospy.Subscriber(p("~driver_topic", "/hardware/driver_status"),
                         DriverStatus, self._driver_cb, queue_size=1)
        rospy.Subscriber(p("~estop_topic", "/safety/estop"), Bool, self._estop_cb, queue_size=1)
        rospy.Subscriber(p("~estop_reset_topic", "/safety/estop_reset"), Bool, self._reset_cb, queue_size=1)
        rospy.Subscriber(p("~arm_topic", "/safety/arm"), Bool, self._arm_cb, queue_size=1)
        rospy.Subscriber(p("~mission_cancelled_topic", "/safety/mission_cancelled"), Bool, self._mission_cb, queue_size=1)
        rospy.Subscriber(p("~topology_topic", "/safety/topology"), SafetySignal,
                         self._topology_cb, queue_size=1)
        rate = float(p("~publish_rate_hz", 50.0))
        if rate < 50.0:
            raise ValueError("safety gate publication must be at least 50 Hz")
        self._wall_stop = threading.Event()
        self._wall_period_s = 1.0 / rate
        rospy.on_shutdown(self._wall_stop.set)
        rospy.on_shutdown(self._observability_stop.set)
        self._wall_thread = threading.Thread(target=self._wall_publish_loop,
                                             name="safety-gate-publisher", daemon=True)
        self._observability_thread = threading.Thread(
            target=self._observability_loop, name="safety-gate-observability", daemon=True)
        self._wall_thread.start()
        self._observability_thread.start()

    @staticmethod
    def _now():
        import rospy
        return rospy.Time.now().to_sec()

    def _cmd_cb(self, msg):
        try:
            now = self._now()
            command = _twist_to_command(msg)
            with self._input_lock:
                self.last_cmd = command
                self.cmd_source = self.cmd_receipt = now
        except Exception:
            with self._input_lock:
                self.internal_fault = True

    def _from_signal(self, msg, receipt):
        return SignalEvidence(msg.state, msg.header.stamp.to_sec(), receipt,
                              int(msg.reason_mask), str(msg.source),
                              str(msg.policy_sha256), int(msg.sequence))

    def _signal_cb(self, name, msg):
        pair_name = "driver" if name in ("mode", "driver") else name
        try:
            signal = GenericEvidence(
                int(msg.sequence), int(msg.state), msg.header.stamp.to_sec(), self._now(),
                int(msg.reason_mask), str(msg.source), str(msg.policy_sha256))
            self.pairs[pair_name].update_signal(name, signal)
        except Exception:
            self.pairs[pair_name].reject()

    def _status(self, msg, clear, policy, receipt, max_linear_mps=None):
        return StructuredEvidence(
            int(msg.sequence), bool(clear), msg.header.stamp.to_sec(),
            msg.evaluation_stamp.to_sec(), receipt, int(msg.reason_mask),
            str(msg.source), str(policy), max_linear_mps)

    def _geofence_status_cb(self, msg):
        try:
            status = self._status(
                msg, msg.state == msg.INSIDE, msg.manifest_sha256, self._now())
            self.pairs["geofence"].update_status(status)
        except Exception:
            self.pairs["geofence"].reject()

    def _collision_status_cb(self, msg):
        try:
            cap = float(msg.recommended_max_linear_mps)
            status = self._status(
                msg, msg.state in (msg.STATE_CLEAR, msg.STATE_CAUTION),
                msg.policy_sha256, self._now(),
                None if cap < 0.0 else cap)
            self.pairs["collision"].update_status(status)
        except Exception:
            self.pairs["collision"].reject()

    def _slope_status_cb(self, msg):
        try:
            cap = float(msg.recommended_max_linear_mps)
            status = self._status(
                msg, msg.state in (msg.STATE_CLEAR, msg.STATE_SLOW),
                msg.policy_sha256, self._now(), None if cap < 0.0 else cap)
            self.pairs["slope"].update_status(status)
        except Exception:
            self.pairs["slope"].reject()

    def _localization_status_cb(self, msg):
        try:
            status = self._status(
                msg, msg.state == msg.OK, msg.policy_sha256, self._now())
            self.pairs["localization"].update_status(status)
        except Exception:
            self.pairs["localization"].reject()

    def _intent_cb(self, msg):
        try:
            if msg.behavior in (msg.PROCEED, msg.SLOW):
                state = CLEAR
            elif msg.behavior == msg.HOLD:
                state = HOLD
            else:
                state = STOP
            self.evidence["motion_intent"] = SignalEvidence(
                state, msg.header.stamp.to_sec(), self._now(), int(msg.reason_mask),
                "motion_intent", "", int(msg.sequence), float(msg.max_linear_mps),
                float(msg.max_angular_rps))
        except Exception:
            self.internal_fault = True

    def _driver_cb(self, msg):
        try:
            physical_estop = bool(msg.physical_estop_asserted)
            state, reason_mask = msg.state, int(msg.reason_mask)
            clear = (state == msg.AUTO_READY and msg.enabled and
                     not msg.manual_override_active and not physical_estop and
                     msg.watchdog_verified)
            reset_driver_healthy = (
                state in (msg.MANUAL, msg.AUTO_DISABLED) and bool(msg.enabled) and
                bool(msg.watchdog_verified) and not bool(msg.manual_override_active) and
                not physical_estop and reason_mask in (0, MODE | DRIVER) and
                _finite(msg.measured_linear_mps) and _finite(msg.measured_angular_rps) and
                isinstance(msg.sequence, int) and not isinstance(msg.sequence, bool) and
                0 <= msg.sequence <= 0xffffffff and isinstance(msg.source, str) and
                bool(msg.source) and _HASH_RE.fullmatch(str(msg.contract_sha256)) is not None)
            status = StructuredEvidence(
                int(msg.sequence), bool(clear), msg.header.stamp.to_sec(),
                msg.header.stamp.to_sec(), self._now(), reason_mask,
                str(msg.source), str(msg.contract_sha256))
            with self._input_lock:
                self.driver_estop = physical_estop
                if physical_estop:
                    self.core.latch_estop()
                self.pairs["driver"].update_status(status)
                self.manual_or_disarmed = state in (msg.MANUAL, msg.AUTO_DISABLED)
                self.reset_driver_healthy = reset_driver_healthy
                self.stationary = (
                    _finite(msg.measured_linear_mps) and _finite(msg.measured_angular_rps) and
                    abs(msg.measured_linear_mps) < self.cfg.stationary_linear_mps and
                    abs(msg.measured_angular_rps) < self.cfg.stationary_angular_rps)
        except Exception:
            with self._input_lock:
                self.pairs["driver"].reject()
                self.internal_fault = True

    def _estop_cb(self, msg):
        try:
            asserted = bool(msg.data)
            with self._input_lock:
                self.external_estop = asserted
                if asserted:
                    self.core.latch_estop()
        except Exception:
            with self._input_lock:
                self.internal_fault = True

    def _combined_estop(self):
        if self.driver_estop is True or self.external_estop is True:
            return True
        if self.driver_estop is False and self.external_estop is False:
            return False
        return None

    def _request_edge(self, request, level, low_observed, msg):
        value = bool(msg.data)
        with self._input_lock:
            if not value:
                setattr(self, level, False)
                setattr(self, low_observed, True)
                return False
            if getattr(self, low_observed) and not getattr(self, level):
                setattr(self, request, True)
                setattr(self, level, True)
                return True
            return False

    def _reset_cb(self, msg):
        self._request_edge("reset_requested", "reset_level", "reset_low_observed", msg)

    def _arm_cb(self, msg):
        if self._request_edge("arm_requested", "arm_level", "arm_low_observed", msg):
            with self._input_lock:
                if getattr(self, "arm_pending_wall", None) is None:
                    self.arm_pending_wall = time.monotonic()

    def _mission_cb(self, msg):
        self.mission_cancelled = bool(msg.data)

    def _topology_cb(self, msg):
        try:
            self.topology_evidence = self._from_signal(msg, self._now())
        except Exception:
            self.topology_evidence = None
            self.internal_fault = True

    def _wall_publish_loop(self):
        deadline = time.monotonic()
        while not self._wall_stop.is_set():
            try:
                self._timer_cb(None, time.monotonic())
            except Exception:
                # The sole command domain retries a fail-closed exact zero.
                with self._input_lock:
                    self.internal_fault = True
                self.core.latch_estop()
                try:
                    self.pub.publish(_command_to_twist(VelocityCommand()))
                except Exception:
                    pass
            deadline += self._wall_period_s
            self._wall_stop.wait(max(0.0, deadline - time.monotonic()))

    def _consume_pending_arm_request(self, wall_now, pair_states):
        """Return the one stored arm edge only after every pair has joined."""
        pending_wall = self.arm_pending_wall
        if pending_wall is None:
            return False
        if (not _finite(wall_now) or not _finite(pending_wall) or
                wall_now < pending_wall or
                wall_now - pending_wall > ARM_PENDING_TTL_S + 1e-12):
            self.arm_pending_wall = None
            return False
        states = [snapshot[1] for snapshot in pair_states.values()]
        if any(state == "drop" for state in states):
            self.arm_pending_wall = None
            return False
        if all(state == "complete" for state in states):
            self.arm_pending_wall = None
            return True
        return False

    def _timer_cb(self, _event, wall_now=None):
        wall_now = time.monotonic() if wall_now is None else wall_now
        now = self._now()
        with self._input_lock:
            deadline = (self.last_tick_wall is not None and
                        wall_now - self.last_tick_wall > self.cfg.deadline_limit_s)
            self.last_tick_wall = wall_now
            clock_fault = not _finite(now)
            if self.last_ros_time is not None:
                if now < self.last_ros_time:
                    clock_fault = True
                elif now == self.last_ros_time:
                    clock_fault |= wall_now - self.last_ros_wall >= self._wall_period_s
            if _finite(now) and (self.last_ros_time is None or now > self.last_ros_time):
                self.last_ros_time, self.last_ros_wall = now, wall_now
            reset = self.reset_requested
            self.reset_requested = False
            snapshots = {
                name: pair.evidence_and_pending_arm_state(now)
                for name, pair in self.pairs.items()
            }
            paired = {name: snapshot[0] for name, snapshot in snapshots.items()}
            arm = self._consume_pending_arm_request(wall_now, snapshots)
            self.arm_requested = False
            self.evidence.update({
                "geofence": paired["geofence"], "collision": paired["collision"],
                "slope": paired["slope"], "localization": paired["localization"],
                "mode": paired["driver"], "driver": paired["driver"],
            })
            inputs = GateInputs(
                cmd=self.last_cmd, now_s=now, cmd_source_stamp_s=self.cmd_source,
                cmd_receipt_stamp_s=self.cmd_receipt, e_stop=self._combined_estop(),
                e_stop_reset=reset, arm_request=arm,
                reset_driver_healthy=self.reset_driver_healthy,
                manual_or_disarmed=self.manual_or_disarmed, stationary=self.stationary,
                mission_cancelled=self.mission_cancelled, topology=self.topology_evidence,
                deadline_missed=deadline, backpressure=self.backpressure,
                internal_fault=self.internal_fault, clock_fault=clock_fault, **self.evidence)
            requested_command = self.last_cmd or VelocityCommand()
        decision = self.core.evaluate(inputs)
        self.pub.publish(_command_to_twist(decision.command))
        with self._observability_lock:
            self.sequence += 1
            self._observability_snapshot = (
                now, self.sequence, decision, requested_command,
                self.pairs["collision"].diagnostic_snapshot())
            state = self._build_safety_state(self._observability_snapshot)
            self.state_pub.publish(state)

    def _observability_loop(self):
        while not self._observability_stop.wait(1.0):
            try:
                self._publish_observability()
            except Exception:
                with self._input_lock:
                    self.internal_fault = True

    def _build_safety_state(self, snapshot):
        import rospy
        from wheelchair_interfaces.msg import SafetyState

        now, sequence, decision, requested, _collision_pair = snapshot
        state = SafetyState()
        state.header.stamp = rospy.Time.from_sec(now)
        state.header.frame_id = "base_footprint"
        state.sequence, state.state, state.reason_mask = sequence, decision.state, decision.reason_mask
        state.armed, state.estop_latched = decision.armed, decision.e_stop_latched
        state.requested_command = _command_to_twist(requested)
        state.output_command = _command_to_twist(decision.command)
        for attr, key in (("command_age_s", "command"), ("intent_age_s", "motion_intent"),
                          ("geofence_age_s", "geofence"), ("collision_age_s", "collision"),
                          ("localization_age_s", "localization"), ("slope_age_s", "slope"),
                          ("mode_age_s", "mode"), ("driver_age_s", "driver")):
            setattr(state, attr, float(decision.ages.get(key, -1.0)))
        state.deadline_miss_count = decision.deadline_miss_count
        state.dropped_input_count = decision.dropped_input_count
        state.release_manifest_sha256 = self.cfg.release_manifest_sha256
        return state

    def _publish_observability(self):
        import rospy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        with self._observability_lock:
            snapshot = self._observability_snapshot
        if snapshot is None:
            return
        now, _sequence, decision, _requested, collision_pair = snapshot
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if decision.armed else DiagnosticStatus.ERROR,
            name="wheelchair_safety/gate", message=decision.reason,
            hardware_id="software_rc_inert")
        status.values = [
            KeyValue("reason_mask", str(decision.reason_mask)),
            KeyValue("armed", str(decision.armed).lower()),
            KeyValue("collision_pair_sequence", str(collision_pair["sequence"])),
            KeyValue("collision_pair_poisoned", str(collision_pair["poisoned"]).lower()),
        ]
        diag = DiagnosticArray()
        diag.header.stamp = rospy.Time.from_sec(now)
        diag.status = [status]
        self.diag_pub.publish(diag)


def run_ros_node():
    import rospy
    rospy.init_node("safety_gate")
    SafetyGateRosNode()
    rospy.spin()


if __name__ == "__main__":
    run_ros_node()
