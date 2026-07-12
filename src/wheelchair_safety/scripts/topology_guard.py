#!/usr/bin/env python3
"""Fail-closed, profile-aware ROS graph, TF, and deadline auditor."""

from dataclasses import dataclass, field, replace
import hashlib
import math
import threading
import time
from typing import Callable, Dict, Iterable, Mapping, Optional, Sequence, Tuple, Union

GRAPH_TOPOLOGY = 1 << 19
TF = 1 << 20
BACKPRESSURE = 1 << 21
DEADLINE_MISS = 1 << 22
INPUT_UNKNOWN = 1 << 31
TOPOLOGY_POLICY_SHA256 = hashlib.sha256(b"wheelchair-topology-authority-v2").hexdigest()


def _caller_id(value: str) -> str:
    """Return the canonical ROS caller ID without discarding its namespace."""
    caller_id = str(value).strip().rstrip("/")
    return "/" + caller_id.lstrip("/") if caller_id else ""


def _nodes(values: Iterable[str]) -> Tuple[str, ...]:
    return tuple(sorted(_caller_id(value) for value in values if str(value).strip()))


def _is_finite(value) -> bool:
    try:
        return value is not None and math.isfinite(value)
    except (TypeError, ValueError):
        return False


@dataclass(frozen=True)
class TopicObservation:
    queue_size: Optional[int]
    latest_only: Optional[bool]
    last_receipt_s: Optional[float]
    deadline_s: Optional[float]


@dataclass(frozen=True)
class TransformObservation:
    owner: str
    last_receipt_s: Optional[float]
    source_stamp_s: Optional[float]
    static: bool = False
    source_age_s: Optional[float] = None
    translation_m: Optional[Tuple[float, float, float]] = None
    yaw_rad: Optional[float] = None


TransformEvidence = Union[str, TransformObservation]


@dataclass(frozen=True)
class GraphSnapshot:
    publishers: Mapping[str, Sequence[str]] = field(default_factory=dict)
    subscribers: Mapping[str, Sequence[str]] = field(default_factory=dict)
    transforms: Mapping[Tuple[str, str], Sequence[TransformEvidence]] = field(default_factory=dict)
    observations: Mapping[str, TopicObservation] = field(default_factory=dict)
    captured_at_s: Optional[float] = None
    master_evidence_complete: bool = False
    tf_evidence_complete: bool = False
    timing_evidence_complete: bool = False
    motion_active: Optional[bool] = None
    relocalizing: Optional[bool] = None


@dataclass(frozen=True)
class TopicAuthority:
    publishers: Tuple[str, ...]
    subscribers: Tuple[str, ...]
    subscriber_alternatives: Tuple[str, ...] = ()
    publisher_optional: bool = False
    required_when_motion_active: bool = False
    subscriber_alternatives_optional: bool = False
    allowed_subscribers: Tuple[str, ...] = ()
    def __post_init__(self) -> None:
        object.__setattr__(self, "publishers", _nodes(self.publishers))
        object.__setattr__(self, "subscribers", _nodes(self.subscribers))
        object.__setattr__(self, "subscriber_alternatives",
                           _nodes(self.subscriber_alternatives))
        object.__setattr__(self, "allowed_subscribers",
                           _nodes(self.allowed_subscribers))


@dataclass(frozen=True)
class TransformAuthority:
    owners: Tuple[str, ...]
    maximum_age_s: Optional[float]
    static: bool = False
    def __post_init__(self) -> None:
        object.__setattr__(self, "owners", _nodes(self.owners))


# Conservative explicit-profile boundary for importers which do not select one.
DEFAULT_AUTHORITIES: Mapping[str, TopicAuthority] = {
    "/cmd_vel_nav": TopicAuthority(("move_base",), ("safety_gate", "collision_supervisor")),
    "/cmd_vel_safe": TopicAuthority(("safety_gate",), ("collision_supervisor",),
                                            ("hardware_adapter", "hardware_shadow_adapter", "hardware_enabled_adapter")),
    "/safety/localization": TopicAuthority(
        ("localization_guard", "independent_localization_guard"), ("safety_gate",)),
    "/safety/geofence": TopicAuthority(("wheelchair_route_safety", "route_safety"), ("safety_gate",)),
    "/safety/collision": TopicAuthority(("collision_supervisor",), ("safety_gate",)),
    "/safety/collision_status": TopicAuthority(("collision_supervisor",), ("safety_gate",)),
    "/safety/slope": TopicAuthority(("slope_supervisor",), ("safety_gate",)),
    "/safety/slope_status": TopicAuthority(("slope_supervisor",), ("safety_gate",)),
    "/safety/mode": TopicAuthority(("hardware_adapter", "hardware_enabled_adapter"), ("safety_gate",)),
    "/safety/driver": TopicAuthority(("hardware_adapter", "hardware_enabled_adapter"), ("safety_gate",)),
    "/safety/estop": TopicAuthority(("operator_io", "verified_io", "hardware_adapter"), ("safety_gate",), publisher_optional=True),
    "/safety/estop_reset": TopicAuthority(("operator_request", "guarded_operator_request"), ("safety_gate",), publisher_optional=True),
    "/safety/topology": TopicAuthority(("topology_guard",), ("safety_gate",)),
}
DEFAULT_TIMED_TOPICS = tuple(topic for topic, authority in DEFAULT_AUTHORITIES.items()
                             if not authority.publisher_optional)

FORBIDDEN_ACTIVE_NODES = (
    "relay", "mux", "twist_mux", "yocs_cmd_vel", "velocity_smoother", "plugin",
)
EVENT_TOPICS = (
    "/safety/estop", "/safety/estop_reset", "/safety/arm",
    "/safety/mission_cancelled", "/initialpose", "/localization/relocalize",
)
SIM_OBSERVER_TOPIC_GRANTS = {
    "control_monitor": (
        "/cmd_vel_nav",
        "/cmd_vel_safe",
        "/decision/motion_intent",
        "/route/active",
        "/route/progress",
        "/odom",
        "/safety/state",
    ),
    "incident_recorder": (
        "/cmd_vel_nav",
        "/cmd_vel_safe",
        "/route/progress",
        "/sensors/lidar/points",
        "/sensors/imu/data",
        "/odom",
        "/localization/status",
        "/hardware/driver_status",
        "/safety/state",
    ),
    "localization_adapter": (
        "/odom",
        "/initialpose",
    ),
    "move_base": (
        "/odom",
    ),
    "collision_supervisor": (
        "/safety/slope_status",
    ),
    "rc_fault_injector": (
        "/safety/state",
    ),
    "rc_scenario_driver": (
        "/route/progress",
        "/localization/status",
        "/route_safety/geofence_status",
        "/safety/collision_status",
        "/safety/slope_status",
        "/safety/state",
    ),
    "rc_metrics_collector": (
        "/route/progress",
        "/localization/candidate",
        "/localization/status",
        "/route_safety/geofence_status",
        "/safety/collision_status",
        "/safety/slope_status",
        "/safety/state",
        "/cmd_vel_nav",
        "/cmd_vel_safe",
        "/wheelchair_base_controller/cmd_vel",
    ),
}


def sim_observer_topic_grants(input_cmd_topic: str, output_cmd_topic: str):
    """Bind collector command observation to launch-selected command topics."""
    return {
        observer: tuple(
            input_cmd_topic if topic == "/cmd_vel_nav" else
            output_cmd_topic if topic == "/cmd_vel_safe" else topic
            for topic in topics
        )
        for observer, topics in SIM_OBSERVER_TOPIC_GRANTS.items()
    }


@dataclass(frozen=True)
class ExpectedGraph:
    authorities: Mapping[str, TopicAuthority] = field(default_factory=lambda: dict(DEFAULT_AUTHORITIES))
    timed_topics: Tuple[str, ...] = DEFAULT_TIMED_TOPICS
    transforms: Mapping[Tuple[str, str], TransformAuthority] = field(default_factory=lambda: {
        ("map", "odom"): TransformAuthority(("localization_adapter", "selected_localization_adapter", "amcl", "slam_toolbox"), 0.25)
    })
    command_topics: Tuple[str, ...] = ("/cmd_vel_nav", "/cmd_vel_safe")
    passive_subscribers: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)
    profile: str = "sim"


def expected_graph(profile: str, input_cmd_topic: str, output_cmd_topic: str,
                   hardware_authority_proven: bool = False,
                   hardware_sink_topic: Optional[str] = None) -> ExpectedGraph:
    """Build an exact profile boundary from launch-selected command topics."""
    profile = str(profile).strip()
    if profile not in ("sim", "replay", "hardware_shadow", "hardware_enabled"):
        raise ValueError("unknown topology profile: %s" % profile)
    if profile == "hardware_enabled" and hardware_authority_proven is not True:
        raise ValueError("hardware_enabled requires proven hardware boundary authority")
    input_topic, output_topic = str(input_cmd_topic), str(output_cmd_topic)
    if not input_topic.startswith("/") or not output_topic.startswith("/") or input_topic == output_topic:
        raise ValueError("command topics must be distinct absolute ROS topic names")
    sink_contract = None if hardware_sink_topic is None else str(hardware_sink_topic)
    if profile == "hardware_enabled":
        if not sink_contract or not sink_contract.startswith("/") or sink_contract in (input_topic, output_topic):
            raise ValueError("hardware_enabled requires a distinct absolute manifest-selected sink topic")
    elif sink_contract:
        raise ValueError("hardware sink contract is only valid for hardware_enabled")
    if profile == "replay" and output_topic != "/shadow/cmd_vel_safe":
        raise ValueError("replay safe output must be /shadow/cmd_vel_safe")
    if profile == "sim":
        evidence_owner = ("sim_evidence_bridge",)
        output_consumers = ("collision_supervisor", "simulation_controller_adapter")
        map_owners = ("localization_adapter", "selected_localization_adapter",
                      "base_model_localization_adapter")
        odom_owners = ("gazebo", "gazebo_ros_control", "wheelchair_base_controller",
                       "sim_evidence_bridge")
        sensor_owners = ("sim_sensor_canonicalizer",)
        initialization_owners = ("operator_request", "guarded_operator_request", "rc_scenario_driver")
        candidate_owners = ("localization_adapter", "selected_localization_adapter",
                            "base_model_localization_adapter")
        sink_topic = "/wheelchair_base_controller/cmd_vel"
        sink_authority = TopicAuthority(
            ("simulation_controller_adapter",), ("gazebo",)
        )
    elif profile == "replay":
        evidence_owner = ("play", "rosbag", "replay_evidence_bridge")
        output_consumers = ("collision_supervisor",)
        map_owners = ("play", "rosbag", "replay_localization_adapter", "localization_adapter")
        odom_owners = ("play", "rosbag", "replay_base_adapter")
        sensor_owners = ("play", "rosbag", "replay_evidence_bridge")
        initialization_owners = ("operator_request", "guarded_operator_request")
        candidate_owners = ("replay_localization_adapter", "localization_adapter")
        sink_topic = None
        sink_authority = None
    elif profile == "hardware_shadow":
        evidence_owner = ("hardware_shadow_adapter",)
        output_consumers = ("collision_supervisor", "hardware_shadow_adapter")
        map_owners = ("localization_adapter", "selected_localization_adapter", "amcl")
        odom_owners = ("hardware_shadow_adapter", "verified_base_adapter")
        sensor_owners = ("hardware_shadow_adapter",)
        initialization_owners = ("operator_request", "guarded_operator_request")
        candidate_owners = ("localization_adapter", "selected_localization_adapter", "amcl")
        sink_topic = None
        sink_authority = None
    else:
        evidence_owner = ("hardware_enabled_adapter",)
        output_consumers = ("collision_supervisor", "hardware_enabled_adapter")
        map_owners = ("localization_adapter", "selected_localization_adapter", "amcl")
        odom_owners = ("verified_base_adapter", "hardware_enabled_adapter")
        sensor_owners = ("hardware_enabled_adapter",)
        initialization_owners = ("operator_request", "guarded_operator_request")
        candidate_owners = ("localization_adapter", "selected_localization_adapter", "amcl")
        sink_topic = sink_contract
        sink_authority = TopicAuthority(("hardware_enabled_adapter",), ("physical_driver",))

    guard_owners = ("localization_guard", "independent_localization_guard")
    authorities = {
        input_topic: TopicAuthority(("move_base",), ("safety_gate", "collision_supervisor"),
                                    publisher_optional=True, required_when_motion_active=True),
        output_topic: TopicAuthority(("safety_gate",), output_consumers),
        "/decision/motion_intent": TopicAuthority(
            ("wheelchair_mission",), ("safety_gate", "collision_supervisor")),
        "/route/active": TopicAuthority(("wheelchair_mission",),
                                        ("route_manager", "wheelchair_route_safety")),
        "/route/progress": TopicAuthority(("route_manager",),
                                          ("wheelchair_mission",)),
        "/sensors/lidar/points": TopicAuthority(
            sensor_owners, ("collision_supervisor", "localization_guard", "perception_node")),
        "/sensors/imu/data": TopicAuthority(
            sensor_owners, ("slope_supervisor", "perception_node")),
        "/odom": TopicAuthority(
            odom_owners, ("collision_supervisor", "localization_guard",
                          "wheelchair_mission")),
        "/initialpose": TopicAuthority(
            initialization_owners,
            ("localization_guard",), publisher_optional=True),
        "/localization/relocalize": TopicAuthority(
            ("operator_request", "guarded_operator_request"),
            ("localization_guard",), publisher_optional=True),
        "/localization/candidate": TopicAuthority(
            candidate_owners, ("wheelchair_route_safety",),
            ("localization_guard", "independent_localization_guard"),
            subscriber_alternatives_optional=False),
        "/localization/status": TopicAuthority(
            guard_owners,
            ("safety_gate", "wheelchair_route_safety", "wheelchair_mission",
             "localization_adapter")),
        "/safety/localization": TopicAuthority(guard_owners, ("safety_gate",)),
        "/route_safety/geofence_status": TopicAuthority(
            ("wheelchair_route_safety",),
            ("safety_gate", "wheelchair_mission", "slope_supervisor")),
        "/safety/geofence": TopicAuthority(("wheelchair_route_safety",), ("safety_gate",)),
        "/safety/collision_status": TopicAuthority(
            ("collision_supervisor",), ("safety_gate", "wheelchair_mission")),
        "/safety/collision": TopicAuthority(("collision_supervisor",), ("safety_gate",)),
        "/safety/slope_status": TopicAuthority(
            ("slope_supervisor",), ("safety_gate", "wheelchair_mission")),
        "/safety/slope": TopicAuthority(("slope_supervisor",), ("safety_gate",)),
        "/hardware/driver_status": TopicAuthority(evidence_owner, ("safety_gate",)),
        "/safety/mode": TopicAuthority(evidence_owner, ("safety_gate",)),
        "/safety/driver": TopicAuthority(evidence_owner, ("safety_gate",)),
        "/safety/state": TopicAuthority(("safety_gate",), ("wheelchair_mission",)),
        "/safety/topology": TopicAuthority(("topology_guard",), ("safety_gate",)),
    }
    if sink_topic is not None:
        authorities[sink_topic] = sink_authority
    if profile == "sim":
        for observer, topics in sim_observer_topic_grants(input_topic, output_topic).items():
            for topic in topics:
                authority = authorities[topic]
                authorities[topic] = TopicAuthority(
                    authority.publishers,
                    authority.subscribers,
                    authority.subscriber_alternatives,
                    authority.publisher_optional,
                    authority.required_when_motion_active,
                    authority.subscriber_alternatives_optional,
                    allowed_subscribers=authority.allowed_subscribers + (observer,),
                )
    event_owners = {
        "/safety/estop": ("operator_io", "verified_io", "hardware_shadow_adapter"),
        "/safety/estop_reset": ("operator_request", "guarded_operator_request"),
        "/safety/arm": ("operator_request", "guarded_operator_request"),
        "/safety/mission_cancelled": ("wheelchair_mission",),
    }
    if profile == "sim":
        event_owners["/safety/estop"] += ("rc_fault_injector",)
        event_owners["/safety/estop_reset"] += ("rc_fault_injector",)
        event_owners["/safety/arm"] += ("rc_scenario_driver",)
    for topic, owners in event_owners.items():
        subscribers = ("safety_gate", "localization_guard") \
            if topic == "/safety/mission_cancelled" else ("safety_gate",)
        authorities[topic] = TopicAuthority(
            owners, subscribers, publisher_optional=True)

    deadlines = {
        input_topic: 0.30, output_topic: 0.10, "/decision/motion_intent": 0.30,
        "/sensors/lidar/points": 0.30, "/sensors/imu/data": 0.10, "/odom": 0.20,
        "/route/active": 0.75, "/route/progress": 0.50,
        "/localization/candidate": 0.25, "/localization/status": 0.25,
        "/safety/localization": 0.25, "/route_safety/geofence_status": 0.10,
        "/safety/geofence": 0.25, "/safety/collision_status": 0.30,
        "/safety/collision": 0.30, "/safety/slope_status": 0.10,
        "/safety/slope": 0.10, "/hardware/driver_status": 0.15,
        "/safety/mode": 0.15, "/safety/driver": 0.15,
    }
    transforms = {
        ("map", "odom"): TransformAuthority(map_owners, 0.25),
        ("odom", "base_footprint"): TransformAuthority(odom_owners, 0.25),
        ("base_footprint", "base_link"): TransformAuthority(("robot_state_publisher",), None, True),
        ("base_link", "lidar_link"): TransformAuthority(("robot_state_publisher",), None, True),
        ("base_link", "imu_link"): TransformAuthority(("robot_state_publisher",), None, True),
    }
    command_topics = (input_topic, output_topic) + (
        (sink_topic,) if sink_topic is not None else ()
    )
    passive_grants = {topic: ("topology_guard",) for topic in deadlines}
    return ExpectedGraph(authorities, tuple(deadlines), transforms, command_topics,
                         passive_grants, profile)


def profile_deadlines(expected: ExpectedGraph) -> Dict[str, float]:
    defaults = {topic: 0.30 for topic in expected.timed_topics}
    for topic, value in ((expected.command_topics[1], 0.10), ("/sensors/imu/data", 0.10),
                         ("/odom", 0.20), ("/route_safety/geofence_status", 0.10),
                         ("/safety/slope", 0.10), ("/safety/slope_status", 0.10),
                         ("/hardware/driver_status", 0.15), ("/safety/mode", 0.15),
                         ("/safety/driver", 0.15)):
        if topic in defaults:
            defaults[topic] = value
    for topic, value in (("/route/active", 0.75), ("/route/progress", 0.50)):
        if topic in defaults:
            defaults[topic] = value
    for topic in ("/localization/candidate", "/localization/status", "/safety/localization",
                  "/safety/geofence"):
        if topic in defaults:
            defaults[topic] = 0.25
    return defaults


@dataclass(frozen=True)
class AuditResult:
    ok: bool
    reason_mask: int
    violations: Tuple[str, ...]
    passive_nodes: Tuple[str, ...]


class DeadlineObserver:
    def __init__(self, deadlines_s: Mapping[str, float], queue_size: int = 1):
        if queue_size != 1:
            raise ValueError("safety observation queue_size must be exactly one")
        self._deadlines = {str(k): float(v) for k, v in deadlines_s.items()}
        if any(not _is_finite(v) or v <= 0.0 for v in self._deadlines.values()):
            raise ValueError("deadlines must be positive and finite")
        self._queue_size = queue_size
        self._last_receipt = {}  # type: Dict[str, float]
        self._lock = threading.Lock()
        self._motion_active = None  # type: Optional[bool]
        self._relocalizing = None  # type: Optional[bool]

    def observe(self, topic: str, receipt_s: Optional[float] = None) -> None:
        if topic not in self._deadlines:
            raise KeyError(topic)
        value = time.monotonic() if receipt_s is None else float(receipt_s)
        if not math.isfinite(value):
            raise ValueError("receipt time must be finite")
        with self._lock:
            self._last_receipt[topic] = value

    def observe_motion_intent(self, message, receipt_s: Optional[float] = None) -> None:
        self.observe("/decision/motion_intent", receipt_s)
        behavior = getattr(message, "behavior", None)
        valid = isinstance(behavior, int) and not isinstance(behavior, bool) and behavior in (0, 1, 2, 3)
        with self._lock:
            self._motion_active = behavior in (1, 2) if valid else None
    def observe_localization_status(self, message, receipt_s: Optional[float] = None) -> None:
        self.observe("/localization/status", receipt_s)
        state = getattr(message, "state", None)
        states = tuple(
            getattr(message.__class__, name, None)
            for name in ("UNINITIALIZED", "INITIALIZING", "OK", "DEGRADED", "LOST", "RELOCALIZING")
        )
        valid = (isinstance(state, int) and not isinstance(state, bool) and
                 None not in states and state in states)
        with self._lock:
            self._relocalizing = state == getattr(message.__class__, "RELOCALIZING") if valid else None

    def relocalizing(self) -> Optional[bool]:
        with self._lock:
            return self._relocalizing


    def motion_active(self) -> Optional[bool]:
        with self._lock:
            return self._motion_active

    def evidence(self) -> Dict[str, TopicObservation]:
        with self._lock:
            receipts = dict(self._last_receipt)
        return {topic: TopicObservation(self._queue_size, True, receipts.get(topic), deadline)
                for topic, deadline in self._deadlines.items()}


class TransformObserver:
    def __init__(self, clock: Callable[[], float] = time.monotonic,
                 source_clock: Optional[Callable[[], float]] = None):
        self._clock = clock
        self._source_clock = source_clock
        self._edges = {}  # type: Dict[Tuple[str, str], Dict[str, TransformObservation]]
        self._seen_tf = False
        self._seen_static = False
        self._lock = threading.Lock()

    def callback(self, message, static: bool = False) -> None:
        receipt = float(self._clock())
        owner = str(getattr(message, "_connection_header", {}).get("callerid", "")).rstrip("/")
        if owner:
            owner = "/" + owner.lstrip("/")
        with self._lock:
            self._seen_static = self._seen_static or static
            self._seen_tf = self._seen_tf or not static
            for transform in getattr(message, "transforms", ()):
                parent = str(transform.header.frame_id).lstrip("/")
                child = str(transform.child_frame_id).lstrip("/")
                stamp = float(transform.header.stamp.to_sec())
                source_age = None
                if not static and self._source_clock is not None:
                    source_age = float(self._source_clock()) - stamp
                if parent and child and owner:
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    yaw = math.atan2(
                        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
                    )
                    self._edges.setdefault((parent, child), {})[owner] = TransformObservation(
                        owner, receipt, stamp, static, source_age,
                        (float(translation.x), float(translation.y), float(translation.z)), yaw)

    def evidence(self):
        with self._lock:
            return ({edge: tuple(owners.values()) for edge, owners in self._edges.items()},
                    self._seen_tf and self._seen_static)


class TopologyAuditor:
    def __init__(self, expected: Optional[ExpectedGraph] = None):
        self.expected = expected or ExpectedGraph()
        self._tf_source_high_water = {}
        self._map_to_odom_pose = {}

    def audit(self, snapshot: GraphSnapshot) -> AuditResult:
        violations, mask = [], 0
        passive = self._passive_nodes(snapshot)
        motion_active = self._effective_motion_active(snapshot)
        if not snapshot.master_evidence_complete:
            violations.append("missing ROS master graph evidence"); mask |= GRAPH_TOPOLOGY | INPUT_UNKNOWN
        if not snapshot.tf_evidence_complete:
            violations.append("missing TF ownership evidence"); mask |= TF | INPUT_UNKNOWN
        if not snapshot.timing_evidence_complete or snapshot.captured_at_s is None:
            violations.append("missing timing/deadline evidence"); mask |= DEADLINE_MISS | INPUT_UNKNOWN

        for topic, authority in self.expected.authorities.items():
            publisher_entries = tuple(value for value in snapshot.publishers.get(topic, ())
                                      if str(value).strip())
            publishers = _nodes(publisher_entries)
            subscribers = tuple(
                n for n in _nodes(snapshot.subscribers.get(topic, ()))
                if n not in _nodes(self.expected.passive_subscribers.get(topic, ())))
            publisher_required = not authority.publisher_optional or (
                authority.required_when_motion_active and motion_active is True)
            publisher_ok = (not publisher_entries and not publisher_required) or \
                           (len(publisher_entries) == 1 and len(publishers) == 1 and
                            publishers[0] in authority.publishers)
            if authority.required_when_motion_active and type(motion_active) is not bool:
                violations.append("%s motion activity evidence is missing or invalid" % topic)
                mask |= GRAPH_TOPOLOGY | INPUT_UNKNOWN
            if not publisher_ok:
                violations.append("%s publisher authority: expected %sone of %s, got %s" %
                                  (topic, "zero or " if not publisher_required else "",
                                   authority.publishers, publishers))
                mask |= GRAPH_TOPOLOGY
            missing = tuple(n for n in authority.subscribers if n not in subscribers)
            if missing:
                violations.append("%s missing required subscribers: %s" % (topic, missing)); mask |= GRAPH_TOPOLOGY
            allowed = (authority.subscribers + authority.subscriber_alternatives +
                       authority.allowed_subscribers)
            unknown = tuple(n for n in subscribers if n not in allowed)
            if unknown:
                violations.append("%s unauthorized subscribers: %s" % (topic, unknown)); mask |= GRAPH_TOPOLOGY
            if authority.subscriber_alternatives:
                selected = tuple(n for n in subscribers if n in authority.subscriber_alternatives)
                valid_count = len(selected) <= 1 if authority.subscriber_alternatives_optional \
                    else len(selected) == 1
                if not valid_count:
                    qualifier = "at most" if authority.subscriber_alternatives_optional else "exactly"
                    violations.append("%s requires %s one profile-selected subscriber from %s, got %s" %
                                      (topic, qualifier, authority.subscriber_alternatives, selected))
                    mask |= GRAPH_TOPOLOGY

        mask |= self._audit_active_edges(snapshot, passive, violations)
        mask |= self._audit_tf(snapshot, violations)
        mask |= self._audit_timing(snapshot, violations, motion_active)
        passive_grants = set()
        for grants in self.expected.passive_subscribers.values():
            passive_grants.update(_nodes(grants))
        return AuditResult(not violations, mask, tuple(violations), tuple(sorted(passive_grants)))

    def _effective_motion_active(self, snapshot: GraphSnapshot) -> Optional[bool]:
        if type(snapshot.motion_active) is bool:
            return snapshot.motion_active
        if snapshot.motion_active is not None:
            return None
        conditional = tuple(
            (topic, authority) for topic, authority in self.expected.authorities.items()
            if authority.required_when_motion_active)
        if conditional and all(
                len(tuple(value for value in snapshot.publishers.get(topic, ())
                          if str(value).strip())) == 1 and
                topic in snapshot.observations
                for topic, _authority in conditional):
            return True
        return None

    def _passive_nodes(self, snapshot: GraphSnapshot) -> set:
        passive = set()
        for topic, grants in self.expected.passive_subscribers.items():
            passive.update(set(_nodes(snapshot.subscribers.get(topic, ()))) & set(_nodes(grants)))
        return passive

    def _audit_active_edges(self, snapshot, passive, violations):
        mask = 0
        protected = set(self.expected.authorities)
        for topic in sorted(set(snapshot.publishers) | set(snapshot.subscribers)):
            subscribers = set(_nodes(snapshot.subscribers.get(topic, ()))) - \
                set(_nodes(self.expected.passive_subscribers.get(topic, ())))
            active = set(_nodes(snapshot.publishers.get(topic, ()))) | subscribers
            protected_surface = (topic in self.expected.command_topics or
                                 topic.startswith("/safety/") or self._is_command_topic(topic))
            if protected_surface:
                forbidden = tuple(sorted(
                    node for node in active
                    if node.rsplit("/", 1)[-1] in FORBIDDEN_ACTIVE_NODES))
                if forbidden:
                    violations.append("%s forbidden relay/mux/plugin nodes: %s" % (topic, forbidden))
                    mask |= GRAPH_TOPOLOGY
            if protected_surface and topic not in protected and active:
                violations.append("unknown active authority edge on %s" % topic)
                mask |= GRAPH_TOPOLOGY
        return mask

    def _audit_tf(self, snapshot, violations):
        mask, now = 0, snapshot.captured_at_s
        normalized = {(str(p).lstrip("/"), str(c).lstrip("/")): values
                      for (p, c), values in snapshot.transforms.items()}
        for edge, authority in self.expected.transforms.items():
            evidence = normalized.get(edge, ())
            converted = [
                replace(item, owner=_caller_id(item.owner))
                if isinstance(item, TransformObservation)
                else TransformObservation(
                    _caller_id(str(item)), now, None, authority.static)
                for item in evidence
            ]
            identities = tuple(sorted(item.owner for item in converted))
            if len(identities) != 1 or len(set(identities)) != 1 or \
                    identities[0] not in authority.owners:
                violations.append("%s->%s authority: expected exactly one of %s, got %s" %
                                  (edge[0], edge[1], authority.owners, identities)); mask |= TF | GRAPH_TOPOLOGY
                continue
            item, identity = converted[0], identities[0]
            if item.static != authority.static:
                violations.append("%s->%s static/dynamic class mismatch" % edge); mask |= TF
                continue
            if authority.static:
                if not all(_is_finite(value) for value in
                           (now, item.last_receipt_s, item.source_stamp_s)) or \
                        item.last_receipt_s < 0.0 or item.last_receipt_s > now or \
                        item.source_stamp_s < 0.0:
                    violations.append("%s->%s invalid static TF evidence" % edge); mask |= TF | INPUT_UNKNOWN
                continue
            if authority.maximum_age_s is None or not all(
                    _is_finite(v) for v in
                    (now, item.last_receipt_s, item.source_stamp_s, item.source_age_s)):
                violations.append("%s->%s stale or invalid TF evidence" % edge); mask |= TF
                continue
            if item.source_stamp_s <= 0.0 or item.last_receipt_s > now or \
                    now - item.last_receipt_s > authority.maximum_age_s or \
                    item.source_age_s < -0.05 or item.source_age_s > authority.maximum_age_s:
                violations.append("%s->%s stale or invalid TF evidence" % edge); mask |= TF
                continue
            key = (edge, identity)
            high_water = self._tf_source_high_water.get(key)
            if high_water is not None and item.source_stamp_s < high_water:
                violations.append("%s->%s source stamp regression" % edge); mask |= TF
                continue
            if edge == ("map", "odom"):
                pose = item.translation_m, item.yaw_rad
                if pose[0] is None or not _is_finite(pose[1]) or \
                        len(pose[0]) != 3 or not all(_is_finite(value) for value in pose[0]):
                    violations.append("map->odom missing transform continuity evidence"); mask |= TF | INPUT_UNKNOWN
                    continue
                prior = self._map_to_odom_pose.get(edge)
                if prior is not None:
                    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(pose[0], prior[0])))
                    yaw_delta = abs((pose[1] - prior[1] + math.pi) % (2.0 * math.pi) - math.pi)
                    if distance > 0.50 or yaw_delta > math.radians(15.0):
                        if snapshot.relocalizing is not True:
                            violations.append("map->odom unapproved relocalization jump"); mask |= TF
                            continue
                self._map_to_odom_pose[edge] = pose
            self._tf_source_high_water[key] = item.source_stamp_s
        return mask

    def _audit_timing(self, snapshot, violations, motion_active):
        mask, now = 0, snapshot.captured_at_s
        for topic in self.expected.timed_topics:
            authority = self.expected.authorities.get(topic)
            if authority is not None and authority.required_when_motion_active and \
                    motion_active is False:
                continue
            observation = snapshot.observations.get(topic)
            if observation is None:
                violations.append("%s missing queue/deadline observation" % topic); mask |= DEADLINE_MISS | INPUT_UNKNOWN
                continue
            if observation.queue_size != 1 or observation.latest_only is not True:
                violations.append("%s violates queue_size=1 latest-only contract" % topic); mask |= BACKPRESSURE
            values = (now, observation.last_receipt_s, observation.deadline_s)
            if any(not _is_finite(value) for value in values):
                violations.append("%s has invalid or missing deadline timestamps" % topic); mask |= DEADLINE_MISS | INPUT_UNKNOWN
            elif observation.deadline_s <= 0.0 or observation.last_receipt_s > now or \
                    now - observation.last_receipt_s > observation.deadline_s + 1e-9:
                violations.append("%s deadline missed" % topic); mask |= DEADLINE_MISS
        return mask

    @staticmethod
    def _is_command_topic(topic):
        segments = tuple(part for part in str(topic).lower().split("/") if part)
        exact_commands = (
            "motor_command", "wheel_command", "drive_command", "actuator_command",
        )
        return any(
            segment == "cmd_vel" or segment.startswith("cmd_vel_") or
            segment.endswith("_cmd_vel") or segment in exact_commands
            for segment in segments
        )


class RosMasterAdapter:
    def __init__(self, caller_id="/topology_guard", transform_provider=None,
                 observation_provider=None, motion_active_provider=None,
                 relocalizing_provider=None, clock=time.monotonic):
        self.caller_id, self.transform_provider = caller_id, transform_provider
        self.observation_provider = observation_provider
        self.motion_active_provider = motion_active_provider
        self.relocalizing_provider, self.clock = relocalizing_provider, clock

    def snapshot(self):
        try:
            import rosgraph
            publishers, subscribers, _ = rosgraph.Master(self.caller_id).getSystemState()
            master_complete = True
        except Exception:
            publishers, subscribers, master_complete = (), (), False
        try:
            transforms, tf_complete = self.transform_provider() if self.transform_provider else ({}, False)
        except Exception:
            transforms, tf_complete = {}, False
        try:
            observations = self.observation_provider() if self.observation_provider else {}
            timing_complete = self.observation_provider is not None
        except Exception:
            observations, timing_complete = {}, False
        try:
            motion_active = self.motion_active_provider() if self.motion_active_provider else None
        except Exception:
            motion_active = None
        try:
            relocalizing = self.relocalizing_provider() if self.relocalizing_provider else None
        except Exception:
            relocalizing = None
        return GraphSnapshot({t: tuple(n) for t, n in publishers}, {t: tuple(n) for t, n in subscribers},
                             dict(transforms), dict(observations), float(self.clock()), master_complete,
                             bool(tf_complete), timing_complete, motion_active, relocalizing)


def main() -> None:
    import rospy
    from tf2_msgs.msg import TFMessage
    from wheelchair_interfaces.msg import LocalizationStatus, MotionIntent, SafetySignal

    rospy.init_node("topology_guard")
    profile = rospy.get_param("~profile", rospy.get_param("/wheelchair_bringup/profile", ""))
    input_topic = rospy.get_param("~input_cmd_topic", "")
    output_topic = rospy.get_param("~output_cmd_topic", "")
    hardware_sink_topic = rospy.get_param("~hardware_sink_topic", "")
    try:
        boundary_proven = (
            rospy.get_param("~hardware_boundary_authority_proven", False) is True and
            rospy.get_param("/hardware_motion_authorized", False) is True and
            rospy.get_param("/passenger_operation_authorized", False) is False)
        expected = expected_graph(
            profile, input_topic, output_topic, boundary_proven, hardware_sink_topic)
    except ValueError as exc:
        rospy.logfatal(str(exc))
        raise
    deadline_observer = DeadlineObserver(profile_deadlines(expected))
    subscriptions = []
    for topic in expected.timed_topics:
        if topic == "/decision/motion_intent":
            callback = deadline_observer.observe_motion_intent
            message_type = MotionIntent
        elif topic == "/localization/status":
            callback = deadline_observer.observe_localization_status
            message_type = LocalizationStatus
        else:
            callback = lambda _msg, name=topic: deadline_observer.observe(name)
            message_type = rospy.AnyMsg
        subscriptions.append(rospy.Subscriber(topic, message_type, callback, queue_size=1))
    tf_observer = TransformObserver(source_clock=lambda: rospy.Time.now().to_sec())
    subscriptions.extend((
        rospy.Subscriber("/tf", TFMessage, lambda msg: tf_observer.callback(msg, False), queue_size=1),
        rospy.Subscriber("/tf_static", TFMessage, lambda msg: tf_observer.callback(msg, True), queue_size=1),
    ))
    signal_topic = rospy.get_param("~signal_topic", "/safety/topology")
    signal_pub = rospy.Publisher(signal_topic, SafetySignal, queue_size=1, latch=False)
    adapter = RosMasterAdapter(transform_provider=tf_observer.evidence,
                               observation_provider=deadline_observer.evidence,
                               motion_active_provider=deadline_observer.motion_active,
                               relocalizing_provider=deadline_observer.relocalizing)
    auditor = TopologyAuditor(expected)
    sequence = 0
    rate = rospy.Rate(float(rospy.get_param("~audit_rate_hz", 2.0)))
    while not rospy.is_shutdown():
        result = auditor.audit(adapter.snapshot())
        sequence += 1
        signal = SafetySignal()
        signal.header.stamp = rospy.Time.now()
        signal.sequence = sequence
        signal.state = SafetySignal.CLEAR if result.ok else SafetySignal.STOP
        signal.reason_mask = result.reason_mask
        signal.source = "topology_guard"
        signal.policy_sha256 = TOPOLOGY_POLICY_SHA256
        signal_pub.publish(signal)
        if result.violations:
            rospy.logerr_throttle(5.0, "topology guard stop: %s", "; ".join(result.violations))
        rate.sleep()


if __name__ == "__main__":
    main()
