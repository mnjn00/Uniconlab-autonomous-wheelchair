#!/usr/bin/env python3
"""Simulation-only Gazebo evidence bridge with a ROS-independent core.

The bridge observes only clock, odometry, and controller-manager state.  It has
no command, motor, e-stop, or reset surface and grants no hardware authority.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

MAP_ID = "hanyang_aegimun_loop"
MAP_SHA256 = "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278"
POLICY_SHA256 = "5d84ea824c98a53639a480ed162a62f015600ca0a0460df7186d5839303d52e8"
SOURCE = "sim_evidence_bridge:sim_only:authority_false"
CONTRACT_ID = "gazebo_sim_evidence_v1"

CLOCK_STALE = 1 << 0
ODOM_STALE = 1 << 1
CONTROLLER_STALE = 1 << 2
TIME_INVALID = 1 << 3

POSE_COVARIANCE = (
    0.0025, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0025, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0012184696791468343,
)

DIAGNOSTIC_METADATA = (
    ("raw_state", "2"),
    ("raw_score", "0.98"),
    ("reset_count", "0"),
    ("scan_residual_m", "0.03"),
    ("inlier_ratio", "0.98"),
    ("innovation_nis", "0.50"),
    ("ambiguity_ratio", "2.50"),
    ("map_id", MAP_ID),
    ("map_sha256", MAP_SHA256),
    ("policy_sha256", POLICY_SHA256),
    ("qualification", "gazebo_synthetic_ground_truth_only"),
    ("transferable_to_replay", "false"),
    ("transferable_to_hardware", "false"),
    ("hardware_motion_authorized", "false"),
    ("passenger_operation_authorized", "false"),
)

OBSERVED_TOPICS = ("/clock", "/wheelchair_base_controller/odom")
PUBLISHED_TOPICS = (
    "/base_model/localization_pose",
    "/base_model/localization_diagnostics",
    "/hardware/driver_status",
    "/safety/mode",
    "/safety/driver",
    "/odom",
)


def compose_planar_pose(
    map_origin_x: float,
    map_origin_y: float,
    map_origin_yaw: float,
    odom_x: float,
    odom_y: float,
    odom_qz: float,
    odom_qw: float,
):
    """Compose an immutable map->odom origin with a planar odom pose."""
    values = tuple(
        float(value)
        for value in (
            map_origin_x,
            map_origin_y,
            map_origin_yaw,
            odom_x,
            odom_y,
            odom_qz,
            odom_qw,
        )
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("planar pose inputs must be finite")
    (
        map_origin_x,
        map_origin_y,
        map_origin_yaw,
        odom_x,
        odom_y,
        odom_qz,
        odom_qw,
    ) = values
    quaternion_norm = math.hypot(odom_qz, odom_qw)
    if quaternion_norm <= 0.0:
        raise ValueError("odom planar quaternion must have nonzero norm")
    odom_qz /= quaternion_norm
    odom_qw /= quaternion_norm
    odom_yaw = math.atan2(
        2.0 * odom_qw * odom_qz,
        odom_qw * odom_qw - odom_qz * odom_qz,
    )
    cosine = math.cos(map_origin_yaw)
    sine = math.sin(map_origin_yaw)
    map_x = map_origin_x + cosine * odom_x - sine * odom_y
    map_y = map_origin_y + sine * odom_x + cosine * odom_y
    map_yaw = math.atan2(
        math.sin(map_origin_yaw + odom_yaw),
        math.cos(map_origin_yaw + odom_yaw),
    )
    result = (map_x, map_y, math.sin(0.5 * map_yaw), math.cos(0.5 * map_yaw))
    if not all(math.isfinite(value) for value in result):
        raise ValueError("composed map pose must be finite")
    return result


def _required_finite_parameter(rospy, name: str) -> float:
    value = float(rospy.get_param(name))
    if not math.isfinite(value):
        raise ValueError("%s must be finite" % name)
    return value


@dataclass(frozen=True)
class FreshnessLimits:
    clock_s: float = 0.15
    odom_s: float = 0.20
    controller_s: float = 0.50
    stamp_skew_s: float = 0.20
    future_tolerance_s: float = 0.05


@dataclass(frozen=True)
class EvidenceState:
    healthy: bool
    reason_mask: int
    clock_stamp_s: Optional[float]
    odom_stamp_s: Optional[float]
    odom_generation: int


class EvidenceCore:
    """Track receipt freshness using monotonic wall time, independent of ROS."""

    def __init__(self, limits: FreshnessLimits = FreshnessLimits()) -> None:
        self.limits = limits
        self._lock = threading.Lock()
        self._clock_arrival: Optional[float] = None
        self._clock_stamp: Optional[float] = None
        self._clock_valid = False
        self._odom_arrival: Optional[float] = None
        self._odom_stamp: Optional[float] = None
        self._odom_generation = 0
        self._controller_arrival: Optional[float] = None

    @staticmethod
    def _valid_time(value: float) -> bool:
        return math.isfinite(value) and value >= 0.0

    def record_clock(self, arrival_s: float, stamp_s: float) -> None:
        with self._lock:
            valid = self._valid_time(arrival_s) and self._valid_time(stamp_s)
            if valid and self._clock_stamp is not None and stamp_s < self._clock_stamp:
                valid = False
            self._clock_arrival = arrival_s if self._valid_time(arrival_s) else None
            self._clock_stamp = stamp_s if self._valid_time(stamp_s) else None
            self._clock_valid = valid

    def record_odom(self, arrival_s: float, stamp_s: float) -> None:
        with self._lock:
            if not self._valid_time(arrival_s) or not self._valid_time(stamp_s):
                self._odom_arrival = None
                self._odom_stamp = None
                return
            self._odom_arrival = arrival_s
            self._odom_stamp = stamp_s
            self._odom_generation += 1

    def record_controller(self, arrival_s: float, running: bool) -> None:
        with self._lock:
            self._controller_arrival = arrival_s if running and self._valid_time(arrival_s) else None

    def evaluate(self, now_s: float) -> EvidenceState:
        with self._lock:
            reason = 0
            if (
                not self._clock_valid
                or self._clock_arrival is None
                or now_s < self._clock_arrival
                or now_s - self._clock_arrival > self.limits.clock_s
            ):
                reason |= CLOCK_STALE
            if (
                self._odom_arrival is None
                or now_s < self._odom_arrival
                or now_s - self._odom_arrival > self.limits.odom_s
            ):
                reason |= ODOM_STALE
            if (
                self._controller_arrival is None
                or now_s < self._controller_arrival
                or now_s - self._controller_arrival > self.limits.controller_s
            ):
                reason |= CONTROLLER_STALE
            if self._clock_stamp is None or self._odom_stamp is None:
                reason |= TIME_INVALID
            elif (
                self._odom_stamp > self._clock_stamp + self.limits.future_tolerance_s
                or self._clock_stamp - self._odom_stamp > self.limits.stamp_skew_s
            ):
                reason |= TIME_INVALID
            return EvidenceState(
                healthy=reason == 0,
                reason_mask=reason,
                clock_stamp_s=self._clock_stamp,
                odom_stamp_s=self._odom_stamp,
                odom_generation=self._odom_generation,
            )


def _valid_controller_odom(message) -> bool:
    try:
        if message.header.frame_id.lstrip("/") != "odom":
            return False
        if message.child_frame_id.lstrip("/") != "base_footprint":
            return False
        values = tuple(
            float(value)
            for value in (
                message.header.stamp.to_sec(),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
                message.twist.twist.angular.x,
                message.twist.twist.angular.y,
                message.twist.twist.angular.z,
            )
        )
        covariance = tuple(
            float(value)
            for value in tuple(message.pose.covariance) + tuple(message.twist.covariance)
        )
    except (AttributeError, TypeError, ValueError):
        return False
    return (
        len(covariance) == 72
        and all(math.isfinite(float(value)) for value in values + covariance)
        and values[0] >= 0.0
        and math.hypot(*values[4:8]) > 0.0
    )


class SimEvidenceBridge:
    """Thin ROS wrapper; imports and graph effects are intentionally lazy."""

    def __init__(self) -> None:
        import rospy
        from controller_manager_msgs.srv import ListControllers
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from wheelchair_interfaces.msg import DriverStatus, SafetySignal

        self.rospy = rospy
        self.DiagnosticArray = DiagnosticArray
        self.DiagnosticStatus = DiagnosticStatus
        self.KeyValue = KeyValue
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.DriverStatus = DriverStatus
        self.SafetySignal = SafetySignal
        self.core = EvidenceCore(
            FreshnessLimits(
                clock_s=float(rospy.get_param("~clock_ttl_s", 0.15)),
                odom_s=float(rospy.get_param("~odom_ttl_s", 0.20)),
                controller_s=float(rospy.get_param("~controller_ttl_s", 0.50)),
            )
        )
        self.controller_name = str(rospy.get_param("~controller_name", "wheelchair_base_controller"))
        self.map_origin_x = _required_finite_parameter(rospy, "~map_origin_x")
        self.map_origin_y = _required_finite_parameter(rospy, "~map_origin_y")
        self.map_origin_yaw = _required_finite_parameter(rospy, "~map_origin_yaw")
        self._latest_odom = None
        self._odom_lock = threading.Lock()
        self._last_pose_generation = 0
        self._sequence = 0
        self._last_controller_poll = -math.inf

        self.pose_pub = rospy.Publisher(PUBLISHED_TOPICS[0], PoseWithCovarianceStamped, queue_size=1)
        self.diag_pub = rospy.Publisher(PUBLISHED_TOPICS[1], DiagnosticArray, queue_size=1)
        self.driver_pub = rospy.Publisher(PUBLISHED_TOPICS[2], DriverStatus, queue_size=1)
        self.mode_pub = rospy.Publisher(PUBLISHED_TOPICS[3], SafetySignal, queue_size=1)
        self.driver_signal_pub = rospy.Publisher(PUBLISHED_TOPICS[4], SafetySignal, queue_size=1)
        self.odom_pub = rospy.Publisher(PUBLISHED_TOPICS[5], Odometry, queue_size=1)
        rospy.Subscriber(OBSERVED_TOPICS[0], Clock, self._clock_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(OBSERVED_TOPICS[1], Odometry, self._odom_callback, queue_size=1, tcp_nodelay=True)
        self._list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)

    def _clock_callback(self, message) -> None:
        self.core.record_clock(time.monotonic(), message.clock.to_sec())

    def _odom_callback(self, message) -> None:
        arrival_s = time.monotonic()
        if not _valid_controller_odom(message):
            with self._odom_lock:
                self._latest_odom = None
            self.core.record_odom(math.nan, math.nan)
            return
        with self._odom_lock:
            self._latest_odom = message
        self.core.record_odom(arrival_s, message.header.stamp.to_sec())
        self.odom_pub.publish(message)

    def _poll_controller(self, now_s: float) -> None:
        if now_s - self._last_controller_poll < 0.20:
            return
        self._last_controller_poll = now_s
        running = False
        try:
            self.rospy.wait_for_service("/controller_manager/list_controllers", timeout=0.02)
            response = self._list_controllers()
            running = any(
                controller.name == self.controller_name and controller.state == "running"
                for controller in response.controller
            )
        except (self.rospy.ROSException, self.rospy.ServiceException):
            pass
        self.core.record_controller(time.monotonic(), running)

    def _publish_pose_and_diagnostics(self, state: EvidenceState, stamp) -> None:
        if state.odom_generation == self._last_pose_generation:
            return
        with self._odom_lock:
            odom = self._latest_odom
        if odom is None:
            return
        try:
            map_x, map_y, map_qz, map_qw = compose_planar_pose(
                self.map_origin_x,
                self.map_origin_y,
                self.map_origin_yaw,
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            )
        except (TypeError, ValueError, OverflowError):
            return
        pose = self.PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = map_x
        pose.pose.pose.position.y = map_y
        pose.pose.pose.position.z = odom.pose.pose.position.z
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = map_qz
        pose.pose.pose.orientation.w = map_qw
        pose.pose.covariance = POSE_COVARIANCE

        item = self.DiagnosticStatus()
        item.level = self.DiagnosticStatus.OK
        item.name = "sim_evidence_bridge/localization"
        item.hardware_id = "gazebo_simulation_only"
        item.message = "CALIBRATED_SYNTHETIC_GAZEBO_EVIDENCE_AUTHORITY_FALSE"
        item.values = [self.KeyValue(key=key, value=value) for key, value in DIAGNOSTIC_METADATA]
        diagnostics = self.DiagnosticArray()
        diagnostics.header = pose.header
        diagnostics.status = [item]
        self.pose_pub.publish(pose)
        self.diag_pub.publish(diagnostics)
        self._last_pose_generation = state.odom_generation

    def _publish_status_pair(self, state: EvidenceState, stamp) -> None:
        self._sequence += 1
        driver = self.DriverStatus()
        driver.header.stamp = stamp
        driver.header.frame_id = "simulation_only"
        driver.sequence = self._sequence
        driver.state = self.DriverStatus.AUTO_READY if state.healthy else self.DriverStatus.AUTO_DISABLED
        driver.reason_mask = state.reason_mask
        driver.source = SOURCE
        driver.contract_id = CONTRACT_ID
        driver.contract_sha256 = POLICY_SHA256
        driver.enabled = state.healthy
        driver.manual_override_active = False
        driver.physical_estop_asserted = False
        driver.watchdog_verified = state.healthy
        driver.heartbeat_age_s = 0.0 if state.healthy else self.core.limits.controller_s + 1.0
        driver.command_timeout_s = 0.0
        with self._odom_lock:
            odom = self._latest_odom
        if odom is not None:
            driver.measured_linear_mps = odom.twist.twist.linear.x
            driver.measured_angular_rps = odom.twist.twist.angular.z

        for publisher in (self.mode_pub, self.driver_signal_pub):
            signal = self.SafetySignal()
            signal.header = driver.header
            signal.sequence = driver.sequence
            signal.state = self.SafetySignal.CLEAR if state.healthy else self.SafetySignal.STOP
            signal.reason_mask = state.reason_mask
            signal.source = SOURCE
            signal.policy_sha256 = POLICY_SHA256
            publisher.publish(signal)
        self.driver_pub.publish(driver)

    def spin(self) -> None:
        while not self.rospy.is_shutdown():
            now_s = time.monotonic()
            self._poll_controller(now_s)
            state = self.core.evaluate(time.monotonic())
            status_stamp = self.rospy.Time.from_sec(state.clock_stamp_s or 0.0)
            if state.healthy:
                pose_stamp = self.rospy.Time.from_sec(state.odom_stamp_s or 0.0)
                self._publish_pose_and_diagnostics(state, pose_stamp)
            self._publish_status_pair(state, status_stamp)
            time.sleep(0.05)


def main() -> None:
    import rospy

    rospy.init_node("sim_evidence_bridge", anonymous=False)
    SimEvidenceBridge().spin()


if __name__ == "__main__":
    main()
