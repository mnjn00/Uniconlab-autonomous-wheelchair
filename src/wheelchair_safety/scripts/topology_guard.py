#!/usr/bin/env python3
"""Fail-closed, profile-aware ROS graph, TF, and deadline auditor."""

from dataclasses import dataclass, field
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


def _node_name(name: str) -> str:
    return str(name).rstrip("/").rsplit("/", 1)[-1]


def _nodes(values: Iterable[str]) -> Tuple[str, ...]:
    return tuple(sorted({_node_name(value) for value in values if str(value).strip()}))


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


@dataclass(frozen=True)
class TopicAuthority:
    publishers: Tuple[str, ...]
    subscribers: Tuple[str, ...]
    subscriber_alternatives: Tuple[str, ...] = ()
    publisher_optional: bool = False
    required_when_motion_active: bool = False
    subscriber_alternatives_optional: bool = False
    allowed_subscribers: Tuple[str, ...] = ()


@dataclass(frozen=True)
class TransformAuthority:
    owners: Tuple[str, ...]
    maximum_age_s: Optional[float]
    static: bool = False


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

PASSIVE_NODE_TOKENS = ("diagnostic", "monitor", "observer", "recorder", "rosbag", "rostopic", "rqt", "rviz")
FORBIDDEN_ACTIVE_NODE_TOKENS = ("relay", "mux", "twist_mux", "yocs_cmd_vel", "velocity_smoother", "plugin")
COMMAND_TOPIC_TOKENS = ("cmd_vel", "motor_command", "wheel_command", "drive_command", "actuator_command")
EVENT_TOPICS = ("/safety/estop", "/safety/estop_reset", "/safety/arm", "/safety/mission_cancelled")
SIM_OBSERVER_TOPIC_GRANTS = {
    "rc_scenario_driver": (
        "/route/progress",
        "/localization/status",
        "/route_safety/geofence_status",
        "/safety/collision_status",
        "/safety/slope_status",
    ),
    "rc_metrics_collector": (
        "/route/progress",
        "/localization/status",
        "/route_safety/geofence_status",
        "/safety/collision_status",
        "/safety/slope_status",
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
        ("map", "odom"): TransformAuthority(("localization_adapter", "selected_localization_adapter", "amcl", "slam_toolbox"), None)
    })
    command_topics: Tuple[str, ...] = ("/cmd_vel_nav", "/cmd_vel_safe")
    passive_subscribers: Tuple[str, ...] = ("topology_guard",)
    profile: str = "sim"


def expected_graph(profile: str, input_cmd_topic: str, output_cmd_topic: str,
                   hardware_authority_proven: bool = False) -> ExpectedGraph:
    """Build an exact profile boundary from launch-selected command topics."""
    profile = str(profile).strip()
    if profile not in ("sim", "replay", "hardware_shadow", "hardware_enabled"):
        raise ValueError("unknown topology profile: %s" % profile)
    if profile == "hardware_enabled" and hardware_authority_proven is not True:
        raise ValueError("hardware_enabled requires proven hardware boundary authority")
    input_topic, output_topic = str(input_cmd_topic), str(output_cmd_topic)
    if not input_topic.startswith("/") or not output_topic.startswith("/") or input_topic == output_topic:
        raise ValueError("command topics must be distinct absolute ROS topic names")
    if profile == "replay" and output_topic != "/shadow/cmd_vel_safe":
        raise ValueError("replay safe output must be /shadow/cmd_vel_safe")

    if profile == "sim":
        evidence_owner = ("sim_evidence_bridge",)
        output_consumers = ("collision_supervisor", "simulation_controller_adapter")
        map_owners = ("localization_adapter", "selected_localization_adapter",
                      "base_model_localization_adapter")
        odom_owners = ("gazebo", "gazebo_ros_control", "wheelchair_base_controller",
                       "sim_evidence_bridge")
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
        candidate_owners = ("replay_localization_adapter", "localization_adapter")
        sink_topic = None
        sink_authority = None
    elif profile == "hardware_shadow":
        evidence_owner = ("hardware_shadow_adapter",)
        output_consumers = ("collision_supervisor", "hardware_shadow_adapter")
        map_owners = ("localization_adapter", "selected_localization_adapter", "amcl")
        odom_owners = ("hardware_shadow_adapter", "verified_base_adapter")
        candidate_owners = ("localization_adapter", "selected_localization_adapter", "amcl")
        sink_topic = None
        sink_authority = None
    else:
        evidence_owner = ("hardware_enabled_adapter",)
        output_consumers = ("collision_supervisor", "hardware_enabled_adapter")
        map_owners = ("localization_adapter", "selected_localization_adapter", "amcl")
        odom_owners = ("verified_base_adapter", "hardware_enabled_adapter")
        candidate_owners = ("localization_adapter", "selected_localization_adapter", "amcl")
        sink_topic = None
        sink_authority = None

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
                                          ("wheelchair_mission", "wheelchair_route_safety")),
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
            ("wheelchair_route_safety",), ("safety_gate", "wheelchair_mission")),
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
        "/route/active": 0.75, "/route/progress": 0.50,
        "/localization/candidate": 0.25, "/localization/status": 0.25,
        "/safety/localization": 0.25, "/route_safety/geofence_status": 0.25,
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
    return ExpectedGraph(authorities, tuple(deadlines), transforms, command_topics,
                         ("topology_guard",), profile)


def profile_deadlines(expected: ExpectedGraph) -> Dict[str, float]:
    defaults = {topic: 0.30 for topic in expected.timed_topics}
    for topic, value in ((expected.command_topics[1], 0.10), ("/safety/slope", 0.10),
                         ("/safety/slope_status", 0.10), ("/hardware/driver_status", 0.15),
                         ("/safety/mode", 0.15), ("/safety/driver", 0.15)):
        if topic in defaults:
            defaults[topic] = value
    for topic, value in (("/route/active", 0.75), ("/route/progress", 0.50)):
        if topic in defaults:
            defaults[topic] = value
    for topic in ("/localization/candidate", "/localization/status", "/safety/localization",
                  "/safety/geofence", "/route_safety/geofence_status"):
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
        owner = _node_name(getattr(message, "_connection_header", {}).get("callerid", ""))
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
                    self._edges.setdefault((parent, child), {})[owner] = TransformObservation(
                        owner, receipt, stamp, static, source_age)

    def evidence(self):
        with self._lock:
            return ({edge: tuple(owners.values()) for edge, owners in self._edges.items()},
                    self._seen_tf and self._seen_static)


class TopologyAuditor:
    def __init__(self, expected: Optional[ExpectedGraph] = None):
        self.expected = expected or ExpectedGraph()

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
            subscribers = tuple(n for n in _nodes(snapshot.subscribers.get(topic, ()))
                                if n not in passive and n not in self.expected.passive_subscribers)
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
        return AuditResult(not violations, mask, tuple(violations), tuple(sorted(passive | set(self.expected.passive_subscribers))))

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
        nodes = set()
        for endpoints in tuple(snapshot.publishers.values()) + tuple(snapshot.subscribers.values()):
            nodes.update(_nodes(endpoints))
        return {node for node in nodes if any(token in node.lower() for token in PASSIVE_NODE_TOKENS)}

    def _audit_active_edges(self, snapshot, passive, violations):
        mask = 0
        topics = set(snapshot.publishers) | set(snapshot.subscribers)
        for topic in sorted(topics):
            publishers = set(_nodes(snapshot.publishers.get(topic, ())))
            subscribers = set(_nodes(snapshot.subscribers.get(topic, ()))) - passive - set(self.expected.passive_subscribers)
            active = publishers | subscribers
            forbidden = sorted(node for node in active if any(token in node.lower() for token in FORBIDDEN_ACTIVE_NODE_TOKENS))
            if forbidden and (topic.startswith("/safety/") or self._is_command_topic(topic)):
                violations.append("%s forbidden relay/mux/plugin nodes: %s" % (topic, tuple(forbidden))); mask |= GRAPH_TOPOLOGY
            if topic.startswith("/safety/") and topic not in self.expected.authorities and "safety_gate" in subscribers:
                violations.append("unknown safety authority edge on %s" % topic); mask |= GRAPH_TOPOLOGY
            if self._is_command_topic(topic) and topic not in self.expected.command_topics and active:
                violations.append("unknown active command edge on %s" % topic); mask |= GRAPH_TOPOLOGY
        return mask

    def _audit_tf(self, snapshot, violations):
        mask, now = 0, snapshot.captured_at_s
        normalized = {(str(p).lstrip("/"), str(c).lstrip("/")): values
                      for (p, c), values in snapshot.transforms.items()}
        for edge, authority in self.expected.transforms.items():
            evidence = normalized.get(edge, ())
            converted = [item if isinstance(item, TransformObservation)
                         else TransformObservation(str(item), now, None, authority.static) for item in evidence]
            owners = tuple(sorted(_node_name(item.owner) for item in converted))
            if len(owners) != 1 or owners[0] not in authority.owners:
                violations.append("%s->%s authority: expected exactly one of %s, got %s" %
                                  (edge[0], edge[1], authority.owners, owners)); mask |= TF | GRAPH_TOPOLOGY
                continue
            item = converted[0]
            if item.static != authority.static:
                violations.append("%s->%s static/dynamic class mismatch" % edge); mask |= TF
            if authority.static:
                if not _is_finite(item.last_receipt_s):
                    violations.append("%s->%s missing static TF receipt" % edge); mask |= TF | INPUT_UNKNOWN
            elif authority.maximum_age_s is not None:
                if not all(_is_finite(v) for v in
                           (now, item.last_receipt_s, item.source_stamp_s, item.source_age_s)) or \
                        item.source_stamp_s <= 0.0 or item.last_receipt_s > now or \
                        now - item.last_receipt_s > authority.maximum_age_s or \
                        item.source_age_s < -0.05 or item.source_age_s > authority.maximum_age_s:
                    violations.append("%s->%s stale or invalid TF evidence" % edge); mask |= TF
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
        return any(token in topic.lower() for token in COMMAND_TOPIC_TOKENS)


class RosMasterAdapter:
    def __init__(self, caller_id="/topology_guard", transform_provider=None,
                 observation_provider=None, motion_active_provider=None, clock=time.monotonic):
        self.caller_id, self.transform_provider = caller_id, transform_provider
        self.observation_provider = observation_provider
        self.motion_active_provider, self.clock = motion_active_provider, clock

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
        return GraphSnapshot({t: tuple(n) for t, n in publishers}, {t: tuple(n) for t, n in subscribers},
                             dict(transforms), dict(observations), float(self.clock()), master_complete,
                             bool(tf_complete), timing_complete, motion_active)


def main() -> None:
    import rospy
    from tf2_msgs.msg import TFMessage
    from wheelchair_interfaces.msg import MotionIntent, SafetySignal

    rospy.init_node("topology_guard")
    profile = rospy.get_param("~profile", rospy.get_param("/wheelchair_bringup/profile", ""))
    input_topic = rospy.get_param("~input_cmd_topic", "")
    output_topic = rospy.get_param("~output_cmd_topic", "")
    try:
        boundary_proven = (
            rospy.get_param("~hardware_boundary_authority_proven", False) is True and
            rospy.get_param("/hardware_motion_authorized", False) is True and
            rospy.get_param("/passenger_operation_authorized", False) is False)
        expected = expected_graph(
            profile, input_topic, output_topic, boundary_proven)
    except ValueError as exc:
        rospy.logfatal(str(exc))
        raise
    deadline_observer = DeadlineObserver(profile_deadlines(expected))
    subscriptions = []
    for topic in expected.timed_topics:
        if topic == "/decision/motion_intent":
            callback = deadline_observer.observe_motion_intent
            message_type = MotionIntent
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
                               motion_active_provider=deadline_observer.motion_active)
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
