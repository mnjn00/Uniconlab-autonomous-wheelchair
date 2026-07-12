#!/usr/bin/env python3
"""Bounded, ROS-independent incident recorder with an optional lazy ROS1 adapter.

The callback-facing API only sanitizes and appends to bounded memory.  Incident
serialization and atomic filesystem writes are performed by worker threads.
This process observes commands and state; it has no publisher or service API.
"""

import argparse
import collections
import dataclasses
import hashlib
import json
import math
import os
import queue
import re
import tempfile
import threading
import time
from pathlib import Path

SCHEMA = "wheelchair-incident-evidence/v1"
_HASH = re.compile(r"^[0-9a-f]{64}$")
_SECRET = re.compile(r"(authorization|cookie|credential|passwd|password|private.?key|secret|token)", re.I)
_SECRET_VALUE = re.compile(r"(?i)(bearer\s+[a-z0-9._~+/-]+=*|(?:token|password|secret)\s*[:=]\s*\S+)")
REDACTED = "[REDACTED]"
REASON_NAMES = (
    "ESTOP", "STALE_CMD", "MODE", "GEOFENCE", "COLLISION", "LOCALIZATION",
    "DRIVER", "INVALID_CMD", "CLOCK", "STALE_INTENT", "INTERNAL_FAULT",
    "STARTUP", "SENSOR_STALE", "COLLISION_BLIND", "COLLISION_TTC",
    "COLLISION_DISTANCE", "SLOPE", "IMU_UNCALIBRATED", "ROUTE_MANIFEST",
    "GRAPH_TOPOLOGY", "TF", "BACKPRESSURE", "DEADLINE_MISS",
    "MANUAL_OVERRIDE", "HARDWARE_UNVERIFIED", "MAP_MISMATCH",
    "COLLISION_OCCLUDED", "LOCALIZATION_INCONSISTENT", "RESOURCE",
    "CORRUPT_DATA", "RESET_REJECTED", "INPUT_UNKNOWN", "ROUTE_STATE",
    "ODOM_STALE", "IMU_STALE", "LIDAR_STALE", "POLICY_MISMATCH",
)


@dataclasses.dataclass(frozen=True)
class RecorderConfig:
    pre_event_s: float = 10.0
    post_event_s: float = 10.0
    max_memory_bytes: int = 64 * 1024 * 1024
    max_record_bytes: int = 64 * 1024
    max_evidence_bytes: int = 16 * 1024 * 1024
    max_write_queue: int = 4
    rate_limit_s: float = 1.0
    poll_s: float = 0.05
    max_string_chars: int = 2048
    max_collection_items: int = 256
    max_gap_s: float = 1.0

    def __post_init__(self):
        finite_positive = (self.pre_event_s, self.post_event_s, self.rate_limit_s,
                           self.poll_s, self.max_gap_s)
        if not all(math.isfinite(value) and value > 0 for value in finite_positive):
            raise ValueError("time bounds must be finite and positive")
        if not (0 < self.max_memory_bytes <= 64 * 1024 * 1024):
            raise ValueError("max_memory_bytes must be in (0, 64 MiB]")
        if not (0 < self.max_record_bytes <= self.max_memory_bytes):
            raise ValueError("max_record_bytes must not exceed memory bound")
        if not (0 < self.max_evidence_bytes <= self.max_memory_bytes):
            raise ValueError("max_evidence_bytes must not exceed memory bound")
        if self.max_write_queue <= 0 or self.max_string_chars <= 0 or self.max_collection_items <= 0:
            raise ValueError("queue and sanitization bounds must be positive")


@dataclasses.dataclass(frozen=True)
class BufferedRecord:
    timestamp_ns: int
    received_monotonic_ns: int
    sequence: int
    topic: str
    payload: dict
    encoded_bytes: int

    def document(self):
        return {
            "timestamp_ns": self.timestamp_ns,
            "received_monotonic_ns": self.received_monotonic_ns,
            "sequence": self.sequence,
            "topic": self.topic,
            "payload": self.payload,
        }


@dataclasses.dataclass(frozen=True)
class PendingIncident:
    event: str
    reason: str
    state: str
    context: dict
    trigger_timestamp_ns: int
    trigger_monotonic_ns: int
    ready_monotonic_ns: int
    trigger_sequence: int


def _sanitize(value, config, key="", depth=0):
    """Return a bounded JSON value while removing credential-shaped content."""
    if _SECRET.search(str(key)):
        return REDACTED
    if depth > 12:
        return "[DEPTH_LIMIT]"
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {"bytes": len(value), "sha256": hashlib.sha256(bytes(value)).hexdigest()}
    if isinstance(value, str):
        text = _SECRET_VALUE.sub(REDACTED, value)
        if len(text) > config.max_string_chars:
            text = text[:config.max_string_chars] + "[TRUNCATED]"
        return text
    if dataclasses.is_dataclass(value):
        value = dataclasses.asdict(value)
    if isinstance(value, dict):
        result = {}
        items = sorted(value.items(), key=lambda item: str(item[0]))
        for child_key, child in items[:config.max_collection_items]:
            name = str(child_key)[:128]
            result[name] = _sanitize(child, config, name, depth + 1)
        if len(items) > config.max_collection_items:
            result["_truncated_items"] = len(items) - config.max_collection_items
        return result
    if isinstance(value, (list, tuple)):
        result = [_sanitize(child, config, key, depth + 1)
                  for child in value[:config.max_collection_items]]
        if len(value) > config.max_collection_items:
            result.append({"_truncated_items": len(value) - config.max_collection_items})
        return result
    return _sanitize(str(value), config, key, depth + 1)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def identity(path=None, expected_sha256=None):
    """Build a hash identity outside callbacks, explicitly marking unavailable data."""
    if path:
        candidate = Path(path)
        if candidate.is_file():
            observed = sha256_file(candidate)
            if expected_sha256 and observed != expected_sha256:
                raise ValueError("identity hash mismatch: %s" % candidate)
            return {"available": True, "sha256": observed, "source": candidate.name}
    if expected_sha256:
        if not _HASH.fullmatch(str(expected_sha256)):
            raise ValueError("identity must be a lowercase SHA-256")
        return {"available": True, "sha256": str(expected_sha256), "source": "declared"}
    return {"available": False, "sha256": None, "source": "unavailable"}


def atomic_write_json(path, document, maximum_bytes):
    """Durably replace one evidence file; no partial final filename is exposed."""
    target = Path(path)
    payload = (json.dumps(document, sort_keys=True, separators=(",", ":"),
                          ensure_ascii=True) + "\n").encode("utf-8")
    if len(payload) > maximum_bytes:
        raise ValueError("evidence exceeds max_evidence_bytes")
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".%s." % target.name,
                                              suffix=".tmp", dir=str(target.parent))
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, str(target))
        directory_fd = os.open(str(target.parent), os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise
    return len(payload)


class IncidentRecorder:
    """Non-authoritative bounded ring and asynchronous incident writer."""

    publishes_motion = False
    grants_permission = False

    def __init__(self, output_dir, identities, event_names, config=None,
                 wall_time_ns=None, monotonic_ns=None, start_workers=True):
        self.config = config or RecorderConfig()
        self.output_dir = Path(output_dir)
        self.identities = self._validate_identities(identities)
        self.event_names = frozenset(str(name) for name in event_names)
        if not self.event_names:
            raise ValueError("at least one named safety event is required")
        self._wall_time_ns = wall_time_ns or time.time_ns
        self._monotonic_ns = monotonic_ns or time.monotonic_ns
        self._records = collections.deque()
        self._memory_bytes = 0
        self._sequence = 0
        self._pending = []
        self._last_trigger_ns = {}
        self._lock = threading.Lock()
        self._jobs = queue.Queue(maxsize=self.config.max_write_queue)
        self._stop = threading.Event()
        self._drops = {
            "lock_contention": 0, "memory_overflow": 0, "oversize_record": 0,
            "rate_limited_event": 0, "pending_overflow": 0,
            "write_queue_overflow": 0, "write_failure": 0,
        }
        self._written = []
        self._scheduler = None
        self._writer = None
        if start_workers:
            self._scheduler = threading.Thread(target=self._schedule_loop,
                                               name="incident-scheduler", daemon=True)
            self._writer = threading.Thread(target=self._write_loop,
                                            name="incident-writer", daemon=True)
            self._scheduler.start()
            self._writer.start()

    @staticmethod
    def _validate_identities(identities):
        result = {}
        for name in ("config", "map", "route", "release"):
            value = dict((identities or {}).get(name, {}))
            available = value.get("available") is True
            digest = value.get("sha256")
            if available and not isinstance(digest, str):
                raise ValueError("available identity requires sha256: %s" % name)
            if available and not _HASH.fullmatch(digest):
                raise ValueError("invalid identity sha256: %s" % name)
            result[name] = {
                "available": available,
                "sha256": digest if available else None,
                "source": str(value.get("source", "unavailable")),
            }
        return result

    def counters(self):
        with self._lock:
            result = dict(self._drops)
            result.update({"ring_records": len(self._records),
                           "ring_bytes": self._memory_bytes,
                           "pending_incidents": len(self._pending),
                           "write_queue_depth": self._jobs.qsize(),
                           "written_incidents": len(self._written)})
            return result

    def capture(self, topic, payload, timestamp_ns=None, received_monotonic_ns=None):
        """Append without waiting for worker I/O; returns False on bounded drop."""
        wall_ns = int(self._wall_time_ns() if timestamp_ns is None else timestamp_ns)
        mono_ns = int(self._monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns)
        clean = _sanitize(payload, self.config)
        if not isinstance(clean, dict):
            clean = {"value": clean}
        estimate = len(json.dumps(clean, sort_keys=True, separators=(",", ":")).encode("utf-8")) + 128
        if estimate > self.config.max_record_bytes:
            self._drops["oversize_record"] += 1
            return False
        if not self._lock.acquire(False):
            self._drops["lock_contention"] += 1
            return False
        try:
            self._sequence += 1
            record = BufferedRecord(wall_ns, mono_ns, self._sequence, str(topic), clean, estimate)
            self._records.append(record)
            self._memory_bytes += estimate
            horizon_ns = mono_ns - int((self.config.pre_event_s + self.config.post_event_s) * 1e9)
            while self._records and (self._records[0].received_monotonic_ns < horizon_ns or
                                     self._memory_bytes > self.config.max_memory_bytes):
                over_capacity = self._memory_bytes > self.config.max_memory_bytes
                removed = self._records.popleft()
                self._memory_bytes -= removed.encoded_bytes
                if over_capacity:
                    self._drops["memory_overflow"] += 1
            return True
        finally:
            self._lock.release()

    def trigger(self, event, reason, state, context=None, timestamp_ns=None,
                received_monotonic_ns=None):
        """Arm a named capture. Unknown/repeated/full events are dropped, never blocked."""
        name = str(event)
        if name not in self.event_names:
            return False
        wall_ns = int(self._wall_time_ns() if timestamp_ns is None else timestamp_ns)
        mono_ns = int(self._monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns)
        clean_context = _sanitize(context or {}, self.config)
        if not self._lock.acquire(False):
            self._drops["lock_contention"] += 1
            return False
        try:
            previous = self._last_trigger_ns.get(name)
            if previous is not None and mono_ns - previous < int(self.config.rate_limit_s * 1e9):
                self._drops["rate_limited_event"] += 1
                return False
            if len(self._pending) >= self.config.max_write_queue:
                self._drops["pending_overflow"] += 1
                return False
            self._last_trigger_ns[name] = mono_ns
            self._pending.append(PendingIncident(
                event=name, reason=str(reason), state=str(state), context=clean_context,
                trigger_timestamp_ns=wall_ns, trigger_monotonic_ns=mono_ns,
                ready_monotonic_ns=mono_ns + int(self.config.post_event_s * 1e9),
                trigger_sequence=self._sequence))
            return True
        finally:
            self._lock.release()

    def flush_ready(self, now_monotonic_ns=None, force=False):
        """Move completed windows to the writer queue; useful for deterministic tests."""
        now_ns = int(self._monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns)
        if not self._lock.acquire(False):
            self._drops["lock_contention"] += 1
            return 0
        try:
            ready = [item for item in self._pending if force or item.ready_monotonic_ns <= now_ns]
            self._pending = [item for item in self._pending if item not in ready]
            jobs = []
            for incident in ready:
                start_ns = incident.trigger_monotonic_ns - int(self.config.pre_event_s * 1e9)
                end_ns = incident.trigger_monotonic_ns + int(self.config.post_event_s * 1e9)
                records = [record for record in self._records
                           if start_ns <= record.received_monotonic_ns <= end_ns]
                jobs.append((incident, records, dict(self._drops)))
        finally:
            self._lock.release()
        accepted = 0
        for job in jobs:
            try:
                self._jobs.put_nowait(job)
                accepted += 1
            except queue.Full:
                self._drops["write_queue_overflow"] += 1
        return accepted

    def _document(self, incident, records, drops):
        first = records[0].received_monotonic_ns if records else None
        last = records[-1].received_monotonic_ns if records else None
        expected_start = incident.trigger_monotonic_ns - int(self.config.pre_event_s * 1e9)
        expected_end = incident.trigger_monotonic_ns + int(self.config.post_event_s * 1e9)
        incomplete_reasons = []
        if not records:
            incomplete_reasons.append("no_records")
        else:
            if first > expected_start + int(self.config.max_gap_s * 1e9):
                incomplete_reasons.append("pre_window_gap")
            if last < expected_end - int(self.config.max_gap_s * 1e9):
                incomplete_reasons.append("post_window_gap")
            if any(right.received_monotonic_ns - left.received_monotonic_ns >
                   int(self.config.max_gap_s * 1e9)
                   for left, right in zip(records, records[1:])):
                incomplete_reasons.append("record_gap")
        if any(value for value in drops.values()):
            incomplete_reasons.append("recorder_drop")
        if any(not item["available"] for item in self.identities.values()):
            incomplete_reasons.append("identity_unavailable")
        document = {
            "schema": SCHEMA,
            "created_timestamp_ns": int(self._wall_time_ns()),
            "event": {
                "name": incident.event, "reason": incident.reason,
                "state": incident.state, "context": incident.context,
                "timestamp_ns": incident.trigger_timestamp_ns,
                "received_monotonic_ns": incident.trigger_monotonic_ns,
                "sequence": incident.trigger_sequence,
            },
            "identities": self.identities,
            "limits": dataclasses.asdict(self.config),
            "authority": {
                "hardware_motion_authorized": False,
                "passenger_operation_authorized": False,
                "publishes_motion": False,
                "grants_permission": False,
            },
            "completeness": {
                "complete": not incomplete_reasons,
                "reasons": sorted(set(incomplete_reasons)),
                "drop_counters": drops,
            },
            "records": [record.document() for record in records],
        }
        return document

    def _write_job(self, job):
        incident, records, drops = job
        document = self._document(incident, records, drops)
        stamp = incident.trigger_timestamp_ns
        safe_event = re.sub(r"[^A-Za-z0-9_.-]+", "_", incident.event)[:80]
        filename = "incident-%d-%s-%06d.json" % (stamp, safe_event, incident.trigger_sequence)
        target = self.output_dir / filename
        try:
            atomic_write_json(target, document, self.config.max_evidence_bytes)
        except ValueError:
            self._drops["write_failure"] += 1
            failure = self._document(incident, [], dict(self._drops))
            failure["completeness"] = {
                "complete": False,
                "reasons": ["evidence_limit", "persistence_failure"],
                "drop_counters": dict(self._drops),
            }
            try:
                atomic_write_json(target, failure, self.config.max_evidence_bytes)
            except Exception:
                return None
        except Exception:
            self._drops["write_failure"] += 1
            return None
        with self._lock:
            self._written.append(str(target))
        return target

    def write_one(self):
        """Synchronously consume one already-queued job (test/operator helper)."""
        try:
            job = self._jobs.get_nowait()
        except queue.Empty:
            return None
        try:
            return self._write_job(job)
        finally:
            self._jobs.task_done()

    def _schedule_loop(self):
        while not self._stop.wait(self.config.poll_s):
            self.flush_ready()

    def _write_loop(self):
        while not self._stop.is_set() or not self._jobs.empty():
            try:
                job = self._jobs.get(timeout=self.config.poll_s)
            except queue.Empty:
                continue
            try:
                self._write_job(job)
            finally:
                self._jobs.task_done()

    def close(self, flush=False, timeout=2.0):
        if flush:
            self.flush_ready(force=True)
        self._stop.set()
        if self._scheduler:
            self._scheduler.join(timeout)
        if self._writer:
            self._writer.join(timeout)


class SafetyEventDetector:
    """Translate SafetyState transitions/reason bits into named event triggers."""

    def __init__(self):
        self._previous_mask = 0
        self._previous_state = None

    def events(self, state, reason_mask):
        state = int(state)
        mask = int(reason_mask)
        newly_set = mask & ~self._previous_mask
        result = [(name, name) for bit, name in enumerate(REASON_NAMES)
                  if newly_set & (1 << bit)]
        state_names = {2: "SAFETY_STOPPED", 3: "SAFETY_LATCHED", 4: "SAFETY_FAULT"}
        if state != self._previous_state and state in state_names:
            result.append((state_names[state], "STATE_TRANSITION"))
        self._previous_mask = mask
        self._previous_state = state
        return result


def _ros_value(value, config, depth=0):
    """Convert a ROS message without importing serialization/graph helpers."""
    if hasattr(value, "to_nsec") and callable(value.to_nsec):
        return int(value.to_nsec())
    slots = getattr(value, "__slots__", None)
    if slots is not None:
        return {slot: _ros_value(getattr(value, slot), config, depth + 1)
                for slot in slots[:config.max_collection_items]}
    return _sanitize(value, config, depth=depth)


def run_ros_node(config_path):
    """Lazy ROS entry point. All params and files are resolved before subscribing."""
    import rospy
    import yaml
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan, PointCloud2
    from wheelchair_interfaces.msg import (DriverStatus, LocalizationStatus,
                                            MissionState, RouteProgress, SafetyState)

    rospy.init_node("incident_recorder", anonymous=False, disable_signals=False)
    config_document = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    bounds = config_document["bounds"]
    recorder_config = RecorderConfig(**bounds)
    configured_hashes = rospy.get_param("~identity_hashes", {})
    configured_paths = rospy.get_param("~identity_paths", {})
    identities = {"config": identity(config_path)}
    for name in ("map", "route", "release"):
        identities[name] = identity(configured_paths.get(name), configured_hashes.get(name))
    output_dir = rospy.get_param("~output_dir", config_document["output_dir"])
    event_names = config_document["safety_events"]
    recorder = IncidentRecorder(output_dir, identities, event_names, recorder_config)
    detector = SafetyEventDetector()
    topics = config_document["topics"]

    def callback(topic):
        def receive(message):
            mono_ns = time.monotonic_ns()
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            timestamp_ns = int(stamp.to_nsec()) if stamp is not None and stamp.to_nsec() else time.time_ns()
            recorder.capture(topic, _ros_value(message, recorder_config), timestamp_ns, mono_ns)
        return receive

    def safety_callback(message):
        mono_ns = time.monotonic_ns()
        stamp = message.header.stamp.to_nsec()
        wall_ns = int(stamp) if stamp else time.time_ns()
        payload = _ros_value(message, recorder_config)
        recorder.capture(topics["safety_state"], payload, wall_ns, mono_ns)
        state_name = {0: "DISARMED", 1: "CLEAR", 2: "STOPPED", 3: "LATCHED", 4: "FAULT"}.get(
            int(message.state), "UNKNOWN")
        for event_name, reason in detector.events(message.state, message.reason_mask):
            recorder.trigger(event_name, reason, state_name,
                             {"reason_mask": int(message.reason_mask),
                              "armed": bool(message.armed),
                              "sequence": int(message.sequence)}, wall_ns, mono_ns)

    _subscribers = [
        rospy.Subscriber(topics["diagnostics"], DiagnosticArray, callback(topics["diagnostics"]), queue_size=1),
        rospy.Subscriber(topics["lidar"], PointCloud2 if topics["lidar"].endswith("points") else LaserScan,
                         callback(topics["lidar"]), queue_size=1),
        rospy.Subscriber(topics["imu"], Imu, callback(topics["imu"]), queue_size=1),
        rospy.Subscriber(topics["odom"], Odometry, callback(topics["odom"]), queue_size=1),
        rospy.Subscriber(topics["localization"], LocalizationStatus, callback(topics["localization"]), queue_size=1),
        rospy.Subscriber(topics["route"], RouteProgress, callback(topics["route"]), queue_size=1),
        rospy.Subscriber(topics["mission"], MissionState, callback(topics["mission"]), queue_size=1),
        rospy.Subscriber(topics["driver"], DriverStatus, callback(topics["driver"]), queue_size=1),
        rospy.Subscriber(topics["nav_command"], Twist, callback(topics["nav_command"]), queue_size=1),
        rospy.Subscriber(topics["safe_command"], Twist, callback(topics["safe_command"]), queue_size=1),
        rospy.Subscriber(topics["safety_state"], SafetyState, safety_callback, queue_size=1),
    ]
    rospy.on_shutdown(lambda: recorder.close(flush=True))
    rospy.spin()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Bounded non-authoritative ROS incident recorder")
    parser.add_argument("--config", required=True, help="observability YAML")
    args, unknown = parser.parse_known_args(argv)
    invalid = [value for value in unknown if ":=" not in value]
    if invalid:
        parser.error("unrecognized arguments: {}".format(" ".join(invalid)))
    run_ros_node(args.config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
