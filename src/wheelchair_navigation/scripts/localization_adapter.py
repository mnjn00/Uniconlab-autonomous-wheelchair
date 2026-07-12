#!/usr/bin/env python3
"""Untrusted native-localizer candidate and sole map->odom TF adapter.

The module deliberately imports no ROS packages at import time. Native localizer
output remains evidence only; an independent guard owns localization status and
safety authority.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Sequence

VALID_SOURCES = ("base_model", "amcl", "cartographer_noetic")
STATUS_MAX_AGE_S = 0.25
TF_FUTURE_TOLERANCE_S = 0.05
TF_VALIDITY_HORIZON_S = 0.04


class ConfigurationError(ValueError):
    """A localization ownership/configuration invariant was violated."""


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CandidateIdentity:
    """Identity used to bind one guard decision to one current candidate."""

    sequence: int
    stamp_s: float
    receipt_s: float
    reset_count: int
    source: str
    map_id: str
    map_sha256: str
    policy_sha256: str


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(first: Pose2D, second: Pose2D) -> Pose2D:
    """Compose planar transforms ``first * second``."""
    c, s = math.cos(first.yaw), math.sin(first.yaw)
    return Pose2D(
        first.x + c * second.x - s * second.y,
        first.y + s * second.x + c * second.y,
        _wrap(first.yaw + second.yaw),
    )


def inverse(transform: Pose2D) -> Pose2D:
    """Invert a planar transform."""
    c, s = math.cos(transform.yaw), math.sin(transform.yaw)
    return Pose2D(
        -c * transform.x - s * transform.y,
        s * transform.x - c * transform.y,
        _wrap(-transform.yaw),
    )


def map_to_odom(map_to_base: Pose2D, odom_to_base: Pose2D) -> Pose2D:
    """Return map->odom from map->base and odom->base evidence."""
    return compose(map_to_base, inverse(odom_to_base))


def select_native_source(configured: str, enabled_sources: Iterable[str]) -> Optional[str]:
    """Validate source arbitration and return the sole enabled source.

    An empty configured source is the only disabled/uninitialized state.
    """
    configured = configured.strip()
    enabled = tuple(enabled_sources)
    invalid = set(enabled) - set(VALID_SOURCES)
    if configured and configured not in VALID_SOURCES:
        invalid.add(configured)
    if invalid:
        raise ConfigurationError("unsupported native localization source: %s" % sorted(invalid))
    if not configured:
        if enabled:
            raise ConfigurationError("native source enabled while adapter is disabled")
        return None
    if len(enabled) != 1 or enabled[0] != configured:
        raise ConfigurationError("exactly the configured native source must be enabled")
    return configured


def validate_tf_authority(adapter_name: str, declared_authorities: Sequence[str]) -> None:
    """Require the adapter to be the sole declared map->odom authority."""
    authorities = tuple(name for name in declared_authorities if name)
    if authorities != (adapter_name,):
        raise ConfigurationError(
            "map->odom requires exactly one authority (%s), got %s"
            % (adapter_name, authorities)
        )


def guard_status_allows_tf(
    candidate: CandidateIdentity,
    *,
    now_s: float,
    status_receipt_s: float,
    status_stamp_s: float,
    status_frame_id: str,
    evaluation_stamp_s: float,
    status_sequence: int,
    status_state: int,
    ok_state: int,
    independent_check_passed: bool,
    reset_count: int,
    source: str,
    map_id: str,
    map_sha256: str,
    policy_sha256: str,
    external_tf_authority: bool = False,
    max_age_s: float = STATUS_MAX_AGE_S,
) -> bool:
    """Fail closed unless a fresh guard decision exactly binds the candidate."""
    times = (now_s, status_receipt_s, status_stamp_s, evaluation_stamp_s, candidate.stamp_s, candidate.receipt_s)
    if not all(math.isfinite(value) for value in times):
        return False
    if max_age_s <= 0.0 or max_age_s > STATUS_MAX_AGE_S:
        return False
    if status_frame_id != "map":
        return False
    if external_tf_authority or status_state != ok_state or not independent_check_passed:
        return False
    if status_sequence != candidate.sequence or status_stamp_s != candidate.stamp_s:
        return False
    if reset_count != candidate.reset_count:
        return False
    if (source, map_id, map_sha256, policy_sha256) != (
        candidate.source,
        candidate.map_id,
        candidate.map_sha256,
        candidate.policy_sha256,
    ):
        return False
    ages = (
        now_s - candidate.receipt_s,
        now_s - status_receipt_s,
        now_s - status_stamp_s,
        now_s - evaluation_stamp_s,
    )
    return all(0.0 <= age <= max_age_s for age in ages)


class EvidenceTracker:
    """Reject time/reset discontinuities before publishing source evidence."""

    def __init__(self, future_tolerance_s: float = 0.05) -> None:
        self.future_tolerance_s = future_tolerance_s
        self._last_now: Optional[float] = None
        self._last_stamp: Optional[float] = None
        self._reset_count: Optional[int] = None
        self._relocalization_authorized = False

    def authorize_relocalization(self) -> None:
        self._relocalization_authorized = True

    def accept(self, now: float, stamp: float, reset_count: int) -> bool:
        values = (now, stamp, float(reset_count))
        if not all(math.isfinite(value) for value in values):
            return False
        if stamp <= 0.0 or stamp > now + self.future_tolerance_s:
            return False
        if self._last_now is not None and now < self._last_now:
            return False
        if self._last_stamp is not None and stamp < self._last_stamp:
            return False
        if self._reset_count is not None:
            if reset_count < self._reset_count:
                return False
            if reset_count > self._reset_count and not self._relocalization_authorized:
                return False
        self._last_now = now
        self._last_stamp = stamp
        self._reset_count = reset_count
        self._relocalization_authorized = False
        return True


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pose2d(pose) -> Pose2D:
    return Pose2D(pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation))


def pose_evidence_is_finite(pose, covariance: Sequence[float] = ()) -> bool:
    """Return whether all source pose components and supplied covariance are finite."""
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) + tuple(covariance)
    quaternion_norm = sum(value * value for value in values[3:7])
    return all(math.isfinite(value) for value in values) and abs(quaternion_norm - 1.0) <= 1e-3


def parse_reset_count(value: float) -> Optional[int]:
    """Accept only an exact non-negative uint32 reset counter."""
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(numeric) or numeric < 0.0 or numeric > 0xFFFFFFFF:
        return None
    parsed = int(numeric)
    return parsed if numeric == parsed else None


class LocalizationAdapterNode:
    """ROS wrapper for candidate publication and sole untrusted TF ownership."""

    def __init__(self) -> None:
        import rospy
        import tf2_ros
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        from wheelchair_interfaces.msg import LocalizationCandidate, LocalizationStatus, MissionState

        self.rospy = rospy
        self.LocalizationCandidate = LocalizationCandidate
        self.LocalizationStatus = LocalizationStatus
        self.enabled = bool(rospy.get_param("~enabled", False))
        configured = str(rospy.get_param("~source", ""))
        enabled = [name for name in VALID_SOURCES if bool(rospy.get_param("~sources/%s/enabled" % name, False))]
        self.source = select_native_source(configured, enabled)
        if not self.enabled and (configured.strip() or enabled):
            raise ConfigurationError("disabled adapter must have no configured native source")
        self.node_name = rospy.get_name()
        validate_tf_authority(self.node_name, rospy.get_param("~map_to_odom_authorities", [self.node_name]))

        self.map_id = str(rospy.get_param("~map_id", ""))
        self.map_sha256 = str(rospy.get_param("~map_sha256", ""))
        self.policy_sha256 = str(rospy.get_param("~policy_sha256", ""))
        self.status_max_age_s = float(rospy.get_param("~status_max_age_s", STATUS_MAX_AGE_S))
        self.tf_future_tolerance_s = float(
            rospy.get_param("~tf_future_tolerance_s", TF_VALIDITY_HORIZON_S)
        )
        if (
            not math.isfinite(self.tf_future_tolerance_s)
            or not 0.0 <= self.tf_future_tolerance_s <= TF_FUTURE_TOLERANCE_S
        ):
            raise ConfigurationError("tf_future_tolerance_s must be in [0, 0.05]")
        if not math.isfinite(self.status_max_age_s) or not 0.0 < self.status_max_age_s <= STATUS_MAX_AGE_S:
            raise ConfigurationError("status_max_age_s must be in (0, 0.25]")
        if self.enabled and (not self.source or not self.map_id or not self.map_sha256 or not self.policy_sha256):
            raise ConfigurationError("enabled localization requires source and exact map/policy identity")

        self.candidate_pub = rospy.Publisher("/localization/candidate", LocalizationCandidate, queue_size=1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tracker = EvidenceTracker()
        self.odom = None
        self.metrics: Dict[str, float] = {}
        self.metadata: Dict[str, str] = {}
        self.external_tf_authority = False
        self.candidate_sequence = 0
        self.pending_candidate = None
        self.pending_pose = None
        self.pending_odom_tf = None
        self.map_to_odom_estimate = None
        self.map_to_odom_receipt_s = None
        self.last_tf_stamp_s = None
        self.mission_canceled = False

        if self.enabled:
            pose_topic = str(rospy.get_param("~sources/%s/pose_topic" % self.source, ""))
            if not pose_topic:
                raise ConfigurationError("selected native source has no pose_topic")
            rospy.Subscriber(pose_topic, PoseWithCovarianceStamped, self._pose_callback, queue_size=1, tcp_nodelay=True)
            rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1, tcp_nodelay=True)
            rospy.Subscriber("/tf", TFMessage, self._tf_callback, queue_size=1, tcp_nodelay=True)
            rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._initial_pose_callback, queue_size=1, tcp_nodelay=True)
            rospy.Subscriber("/decision/state", MissionState, self._mission_callback, queue_size=1, tcp_nodelay=True)
            rospy.Subscriber("/localization/status", LocalizationStatus, self._status_callback, queue_size=1, tcp_nodelay=True)
            diagnostic_topic = str(rospy.get_param("~sources/%s/diagnostic_topic" % self.source, ""))
            if diagnostic_topic:
                rospy.Subscriber(diagnostic_topic, DiagnosticArray, self._diagnostic_callback, queue_size=1, tcp_nodelay=True)

    def _odom_callback(self, message) -> None:
        self.odom = message
        if (
            self.map_to_odom_estimate is None
            or self.map_to_odom_receipt_s is None
            or self.external_tf_authority
        ):
            return
        now = self.rospy.Time.now()
        age_s = now.to_sec() - self.map_to_odom_receipt_s
        if not 0.0 <= age_s <= self.status_max_age_s:
            return
        validity_stamp = message.header.stamp + self.rospy.Duration(
            self.tf_future_tolerance_s
        )
        self._send_map_to_odom(self.map_to_odom_estimate, validity_stamp)

    def _mission_callback(self, message) -> None:
        self.mission_canceled = message.state in (message.DISARMED, message.LOCALIZING)

    def _initial_pose_callback(self, message) -> None:
        now = self.rospy.Time.now().to_sec()
        stamp = message.header.stamp.to_sec()
        stopped = (
            self.odom is not None
            and abs(self.odom.twist.twist.linear.x) < 0.01
            and abs(self.odom.twist.twist.angular.z) < 0.02
        )
        if (
            message.header.frame_id.lstrip("/") == "map"
            and self.mission_canceled
            and stopped
            and stamp > 0.0
            and -0.05 <= now - stamp <= STATUS_MAX_AGE_S
        ):
            self.tracker.authorize_relocalization()

    def _tf_callback(self, message) -> None:
        caller = str(
            getattr(message, "_connection_header", {}).get("callerid", "")
        ).strip().lstrip("/")
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if (
                parent == "map"
                and child == "odom"
                and caller != self.node_name.lstrip("/")
            ):
                self.external_tf_authority = True
                self._discard_pending_candidate()

    def _diagnostic_callback(self, message) -> None:
        for status in message.status:
            for item in status.values:
                self.metadata[item.key] = item.value
                try:
                    self.metrics[item.key] = float(item.value)
                except (TypeError, ValueError):
                    continue

    def _pose_callback(self, pose) -> None:
        now = self.rospy.Time.now()
        if self.external_tf_authority:
            return
        now_s = now.to_sec()
        stamp = pose.header.stamp
        reset_count = parse_reset_count(self.metrics.get("reset_count", 0.0))
        source_map_id = self.metadata.get("map_id", self.map_id)
        source_map_sha256 = self.metadata.get("map_sha256", self.map_sha256)
        if pose.header.frame_id.lstrip("/") != "map":
            return
        if reset_count is None or not pose_evidence_is_finite(pose.pose.pose, pose.pose.covariance):
            return
        if (source_map_id, source_map_sha256) != (self.map_id, self.map_sha256):
            return
        if not self.tracker.accept(now_s, stamp.to_sec(), reset_count):
            return
        if self.odom is None or not 0.0 <= now_s - self.odom.header.stamp.to_sec() <= 0.20:
            return
        if not pose_evidence_is_finite(self.odom.pose.pose):
            return
        if not all(
            math.isfinite(value)
            for value in (self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z)
        ):
            return
        try:
            odom_tf = self.tf_buffer.lookup_transform("odom", "base_footprint", stamp, self.rospy.Duration(0.02))
        except Exception:
            return
        transform_values = (
            odom_tf.transform.translation.x,
            odom_tf.transform.translation.y,
            odom_tf.transform.translation.z,
            odom_tf.transform.rotation.x,
            odom_tf.transform.rotation.y,
            odom_tf.transform.rotation.z,
            odom_tf.transform.rotation.w,
        )
        if (
            not 0.0 <= now_s - odom_tf.header.stamp.to_sec() <= STATUS_MAX_AGE_S
            or not all(math.isfinite(value) for value in transform_values)
            or abs(sum(value * value for value in transform_values[3:]) - 1.0) > 1e-3
        ):
            return

        candidate = self.LocalizationCandidate()
        candidate.pose = copy.deepcopy(pose)
        candidate.pose.header.frame_id = "map"
        candidate.raw_state = int(self.metrics.get("raw_state", candidate.RAW_UNINITIALIZED))
        candidate.reset_count = reset_count
        candidate.map_id = source_map_id
        candidate.map_sha256 = source_map_sha256
        candidate.source = self.source
        candidate.raw_score = float(self.metrics.get("raw_score", -1.0))

        self.candidate_sequence += 1
        candidate.pose.header.seq = self.candidate_sequence
        self.pending_candidate = CandidateIdentity(
            sequence=self.candidate_sequence,
            stamp_s=stamp.to_sec(),
            receipt_s=now_s,
            reset_count=reset_count,
            source=self.source,
            map_id=self.map_id,
            map_sha256=self.map_sha256,
            policy_sha256=self.policy_sha256,
        )
        self.pending_pose = candidate.pose
        self.pending_odom_tf = odom_tf
        # The selected adapter owns map->odom; the independent guard audits this
        # untrusted candidate and alone grants motion permission.  Publishing TF
        # before the candidate removes a bootstrap cycle without bypassing safety.
        self._broadcast_map_to_odom(self.pending_pose, self.pending_odom_tf)
        self.candidate_pub.publish(candidate)

    def _status_callback(self, status) -> None:
        candidate = self.pending_candidate
        if candidate is None:
            return
        now_s = self.rospy.Time.now().to_sec()
        matching_sequence = status.sequence == candidate.sequence
        allowed = guard_status_allows_tf(
            candidate,
            now_s=now_s,
            status_receipt_s=now_s,
            status_stamp_s=status.header.stamp.to_sec(),
            status_frame_id=status.header.frame_id,
            evaluation_stamp_s=status.evaluation_stamp.to_sec(),
            status_sequence=status.sequence,
            status_state=status.state,
            ok_state=status.OK,
            independent_check_passed=status.independent_check_passed,
            reset_count=status.reset_count,
            source=status.source,
            map_id=status.map_id,
            map_sha256=status.map_sha256,
            policy_sha256=status.policy_sha256,
            external_tf_authority=self.external_tf_authority,
            max_age_s=self.status_max_age_s,
        )
        # Status binding is still evaluated to reject stale/mismatched decisions,
        # but the guard never broadcasts TF and cannot become a second authority.
        _ = allowed
        if matching_sequence:
            self._discard_pending_candidate()

    def _discard_pending_candidate(self) -> None:
        self.pending_candidate = None
        self.pending_pose = None
        self.pending_odom_tf = None

    def _broadcast_map_to_odom(self, pose, odom_tf) -> None:
        odom_to_base = Pose2D(
            odom_tf.transform.translation.x,
            odom_tf.transform.translation.y,
            _yaw_from_quaternion(odom_tf.transform.rotation),
        )
        transform = map_to_odom(_pose2d(pose.pose.pose), odom_to_base)
        self.map_to_odom_estimate = transform
        now = self.rospy.Time.now()
        self.map_to_odom_receipt_s = now.to_sec()

        # Preserve the source-time sample for guard evaluation, then extend the
        # same estimate only to A03's bounded validity horizon.  Odom callbacks
        # refresh that horizon while the localization evidence remains fresh.
        self._send_map_to_odom(transform, pose.header.stamp)
        validity_stamp = now + self.rospy.Duration(self.tf_future_tolerance_s)
        self._send_map_to_odom(transform, validity_stamp)

    def _send_map_to_odom(self, transform: Pose2D, stamp) -> None:
        stamp_s = stamp.to_sec()
        if (
            not math.isfinite(stamp_s)
            or stamp_s <= 0.0
            or (
                self.last_tf_stamp_s is not None
                and stamp_s <= self.last_tf_stamp_s
            )
        ):
            return
        self.last_tf_stamp_s = stamp_s

        from geometry_msgs.msg import TransformStamped

        output = TransformStamped()
        output.header.stamp = stamp
        output.header.frame_id = "map"
        output.child_frame_id = "odom"
        output.transform.translation.x = transform.x
        output.transform.translation.y = transform.y
        output.transform.rotation.z = math.sin(transform.yaw * 0.5)
        output.transform.rotation.w = math.cos(transform.yaw * 0.5)
        self.tf_broadcaster.sendTransform(output)


def main() -> None:
    import rospy

    rospy.init_node("localization_adapter")
    try:
        LocalizationAdapterNode()
        rospy.spin()
    except ConfigurationError as error:
        rospy.logfatal(str(error))
        raise SystemExit(2)


if __name__ == "__main__":
    main()
