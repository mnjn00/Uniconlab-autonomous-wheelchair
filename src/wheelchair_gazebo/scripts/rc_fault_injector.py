#!/usr/bin/env python3
"""Bounded, simulation-only ROS graph fault injector for RC qualification.

This node has no hardware authority.  The only direct publication to the
simulated actuator sink is the exact-zero graph-bypass probe.
"""

import hashlib
import json
import math
import time

SCHEMA = "wheelchair.sim_fault/v1"
EVENT_TOPIC = "/simulation/fault_event"
ACTUATOR_SINK = "/wheelchair_base_controller/cmd_vel"
PHASES = frozenset(("ready", "triggered", "reset_attempted", "completed", "failed"))
FAULT_IDS = frozenset((
    "lidar_loss", "imu_loss", "odom_loss", "tf_loss", "localizer_loss",
    "decision_loss", "safety_loss", "driver_loss", "generic_process_loss",
    "stale_lidar", "future_imu", "out_of_order_odom", "nan_command",
    "duplicate_cmd_publisher", "duplicate_tf_authority", "clock_reset",
    "cpu_pressure", "queue_pressure", "estop_asserted", "reset_while_asserted",
    "reset_while_moving", "reset_in_auto", "graph_bypass",
))
NORMAL_IDS = frozenset((
    "", "default", "normal", "nominal", "full_rc_matrix",
    "wheelchair_rc_scenarios", "wheelchair_rc_scenarios.world",
))
PROCESS_NODES = {
    "lidar_loss": "/perception_node",
    "imu_loss": "/slope_supervisor",
    "tf_loss": "/robot_state_publisher",
    "localizer_loss": "/localization_adapter",
    "decision_loss": "/wheelchair_mission",
    "safety_loss": "/safety_gate",
    "driver_loss": "/simulation_controller_adapter",
    "generic_process_loss": "/route_manager",
}
MOTION_REQUIRED = frozenset(tuple(PROCESS_NODES) + (
    "odom_loss", "nan_command", "reset_while_moving",
))
ARMED_REQUIRED = frozenset((
    "estop_asserted", "reset_while_asserted", "reset_while_moving", "reset_in_auto",
))
MAX_EFFECT_S = 5.0
MAX_PRESSURE_MESSAGES = 128
MAX_PRESSURE_BYTES = 4 * 1024 * 1024


class FaultError(RuntimeError):
    """A fail-closed configuration or live injection failure."""


def classify_fault(fault_id):
    value = str(fault_id).strip()
    if value in NORMAL_IDS:
        return "normal"
    if value in FAULT_IDS:
        return "fault"
    raise FaultError("unknown non-normal fault_id: %s" % value)


def validate_preflight(values):
    """Validate exact booleans; truthy strings and integers are rejected."""
    expected = {
        "/simulation_only": True,
        "/use_sim_time": True,
        "/hardware_motion_authorized": False,
        "/passenger_operation_authorized": False,
    }
    for name, required in expected.items():
        value = values.get(name)
        if type(value) is not bool or value is not required:
            raise FaultError("preflight parameter %s must be exactly %r" % (name, required))
    profile = values.get("/wheelchair_bringup/profile")
    if profile != "sim":
        raise FaultError("fault injection requires the sim profile")
    return True


def bounded_effect_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0 or value > MAX_EFFECT_S:
        raise FaultError("effect_duration_s must be finite and in (0, %.1f]" % MAX_EFFECT_S)
    return value


def pressure_plan(messages, payload_bytes):
    messages, payload_bytes = int(messages), int(payload_bytes)
    if messages <= 0 or messages > MAX_PRESSURE_MESSAGES:
        raise FaultError("pressure message count exceeds bounded limit")
    if payload_bytes <= 0 or messages * payload_bytes > MAX_PRESSURE_BYTES:
        raise FaultError("pressure byte budget exceeds bounded limit")
    return messages, payload_bytes


def event_json(fault_id, phase, stamp_s, detail):
    if phase not in PHASES:
        raise FaultError("invalid event phase")
    stamp_s = float(stamp_s)
    if not math.isfinite(stamp_s) or stamp_s < 0.0:
        raise FaultError("event stamp must be finite and nonnegative")
    event = {
        "schema": SCHEMA,
        "fault_id": str(fault_id),
        "phase": phase,
        "stamp_s": stamp_s,
        "detail": str(detail),
    }
    return json.dumps(event, sort_keys=True, separators=(",", ":"), allow_nan=False)


def is_exact_zero_twist(message):
    values = (message.linear.x, message.linear.y, message.linear.z,
              message.angular.x, message.angular.y, message.angular.z)
    return all(type(value) in (int, float) and math.isfinite(value) and value == 0.0
               for value in values)


class FaultInjector:
    def __init__(self, rospy, fault_id, timeout_s=30.0, effect_duration_s=1.0):
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        from wheelchair_interfaces.msg import SafetyState

        self.rospy = rospy
        self.fault_id = str(fault_id).strip()
        self.timeout_s = float(timeout_s)
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0:
            raise FaultError("timeout_s must be finite and positive")
        self.effect_s = bounded_effect_seconds(effect_duration_s)
        self._armed = False
        self._moving = False
        self._String = String
        self._event_pub = rospy.Publisher(EVENT_TOPIC, String, queue_size=10, latch=True)
        self._observers = (
            rospy.Subscriber("/safety/state", SafetyState, self._safety_cb, queue_size=1),
            rospy.Subscriber("/cmd_vel_safe", Twist, self._command_cb, queue_size=1),
        )

    def _safety_cb(self, message):
        self._armed = bool(message.armed)

    def _command_cb(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if all(math.isfinite(value) for value in values):
            self._moving = any(value != 0.0 for value in values)

    def _stamp(self):
        return float(self.rospy.Time.now().to_sec())

    def emit(self, phase, detail):
        self._event_pub.publish(self._String(data=event_json(
            self.fault_id, phase, max(0.0, self._stamp()), detail)))

    def _wait(self, predicate, description):
        deadline = time.monotonic() + self.timeout_s
        while not self.rospy.is_shutdown() and time.monotonic() <= deadline:
            if predicate():
                return
            time.sleep(0.02)
        raise FaultError("bounded timeout waiting for %s" % description)

    def _wait_preconditions(self):
        self._wait(lambda: math.isfinite(self._stamp()) and self._stamp() > 0.0,
                   "live simulated time")
        if self.fault_id in ARMED_REQUIRED:
            self._wait(lambda: self._armed, "armed simulation state")
        if self.fault_id in MOTION_REQUIRED:
            self._wait(lambda: self._moving, "observed nonzero safe command")

    @staticmethod
    def _sleep(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            time.sleep(min(0.02, deadline - time.monotonic()))

    def _kill_process(self):
        import rosnode
        node = PROCESS_NODES[self.fault_id]
        if node not in rosnode.get_node_names():
            raise FaultError("selected simulation node is absent: %s" % node)
        _success, failed = rosnode.kill_nodes([node])
        if failed:
            raise FaultError("failed to stop simulation node: %s" % node)
        return "stopped simulation node %s" % node

    def _controller_loss(self):
        from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
        service = "/controller_manager/switch_controller"
        self.rospy.wait_for_service(service, timeout=self.timeout_s)
        switch = self.rospy.ServiceProxy(service, SwitchController)
        stop = SwitchControllerRequest()
        stop.stop_controllers = ["wheelchair_base_controller"]
        stop.strictness = SwitchControllerRequest.STRICT
        if not switch(stop).ok:
            raise FaultError("controller manager rejected simulated odometry loss")
        try:
            self._sleep(self.effect_s)
        finally:
            start = SwitchControllerRequest()
            start.start_controllers = ["wheelchair_base_controller"]
            start.strictness = SwitchControllerRequest.STRICT
            if not switch(start).ok:
                raise FaultError("failed to restore simulated base controller")
        return "stopped and restored simulated base controller"

    def _publish_bounded(self, topic, msg_type, factory, count=5, latch=False):
        publisher = self.rospy.Publisher(topic, msg_type, queue_size=1, latch=latch)
        try:
            deadline = time.monotonic() + self.effect_s
            sent = 0
            while sent < count and time.monotonic() <= deadline:
                publisher.publish(factory())
                sent += 1
                time.sleep(min(0.05, self.effect_s / max(1, count)))
            if sent == 0:
                raise FaultError("bounded publisher produced no messages")
            return sent
        finally:
            publisher.unregister()

    def _timestamp_fault(self):
        now = self.rospy.Time.now()
        if self.fault_id == "stale_lidar":
            from sensor_msgs.msg import PointCloud2
            def message():
                value = PointCloud2()
                value.header.stamp = self.rospy.Time(0)
                value.header.frame_id = "lidar_link"
                return value
            topic, msg_type = "/sensors/lidar/points", PointCloud2
        elif self.fault_id == "future_imu":
            from sensor_msgs.msg import Imu
            def message():
                value = Imu()
                value.header.stamp = now + self.rospy.Duration(60.0)
                value.header.frame_id = "imu_link"
                return value
            topic, msg_type = "/sensors/imu/data", Imu
        else:
            from nav_msgs.msg import Odometry
            def message():
                value = Odometry()
                value.header.stamp = self.rospy.Time(max(0.0, now.to_sec() - 60.0))
                value.header.frame_id = "odom"
                value.child_frame_id = "base_link"
                return value
            topic, msg_type = "/wheelchair/odometry", Odometry
        count = self._publish_bounded(topic, msg_type, message)
        return "published %d malformed timestamp messages on %s" % (count, topic)

    def _nan_command(self):
        from geometry_msgs.msg import Twist
        def message():
            value = Twist()
            value.linear.x = float("nan")
            return value
        count = self._publish_bounded("/cmd_vel_nav", Twist, message, count=1)
        return "published %d NaN navigation command (not actuator sink)" % count

    def _duplicate_command(self):
        from geometry_msgs.msg import Twist
        count = self._publish_bounded("/cmd_vel_safe", Twist, Twist, count=10)
        return "created bounded duplicate safe-command publisher with %d exact-zero messages" % count

    def _duplicate_tf(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        def message():
            transform = TransformStamped()
            transform.header.stamp = self.rospy.Time.now()
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.transform.rotation.w = 1.0
            return TFMessage(transforms=[transform])
        count = self._publish_bounded("/tf", TFMessage, message, count=10)
        return "created bounded duplicate TF authority with %d transforms" % count

    def _clock_reset(self):
        from rosgraph_msgs.msg import Clock
        def message():
            return Clock(clock=self.rospy.Time(0))
        count = self._publish_bounded("/clock", Clock, message, count=1)
        return "published %d simulated clock reset" % count

    def _cpu_pressure(self):
        deadline = time.monotonic() + self.effect_s
        rounds = 0
        value = b"wheelchair-simulation-pressure"
        while time.monotonic() < deadline:
            value = hashlib.sha256(value).digest()
            rounds += 1
        return "completed bounded CPU pressure (%d rounds, %.3fs)" % (rounds, self.effect_s)

    def _queue_pressure(self):
        from sensor_msgs.msg import PointCloud2, PointField
        messages, payload_bytes = pressure_plan(32, 64 * 1024)
        def message():
            value = PointCloud2()
            value.header.stamp = self.rospy.Time.now()
            value.header.frame_id = "lidar_link"
            value.height = 1
            value.width = payload_bytes // 4
            value.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1)]
            value.point_step = 4
            value.row_step = payload_bytes
            value.data = bytes(payload_bytes)
            value.is_dense = False
            return value
        count = self._publish_bounded("/sensors/lidar/points", PointCloud2, message,
                                      count=messages)
        return "published bounded queue pressure (%d messages, %d bytes each)" % (count, payload_bytes)

    def _bool_events(self):
        from std_msgs.msg import Bool
        estop = self.rospy.Publisher("/safety/estop", Bool, queue_size=1)
        reset = self.rospy.Publisher("/safety/estop_reset", Bool, queue_size=1)
        try:
            if self.fault_id == "estop_asserted":
                estop.publish(Bool(data=True))
                detail = "asserted simulated emergency stop"
            elif self.fault_id == "reset_while_asserted":
                estop.publish(Bool(data=True))
                time.sleep(0.05)
                reset.publish(Bool(data=True))
                self.emit("reset_attempted", "reset requested while simulated estop remained asserted")
                detail = "asserted estop and attempted guarded reset"
            else:
                reset.publish(Bool(data=True))
                self.emit("reset_attempted", "guarded reset requested in disallowed live state")
                detail = "attempted guarded reset in %s" % self.fault_id
            self._sleep(min(self.effect_s, 0.25))
            return detail
        finally:
            # Deassertion is not a reset and cannot re-arm the system.
            estop.publish(Bool(data=False))
            reset.publish(Bool(data=False))
            estop.unregister()
            reset.unregister()

    def _graph_bypass(self):
        from geometry_msgs.msg import Twist
        zero = Twist()
        if not is_exact_zero_twist(zero):
            raise FaultError("internal graph bypass command was not exact zero")
        count = self._publish_bounded(ACTUATOR_SINK, Twist, lambda: zero, count=10)
        return "created bounded direct sink publisher with %d exact-zero commands" % count

    def inject(self):
        if self.fault_id in PROCESS_NODES:
            return self._kill_process()
        effects = {
            "odom_loss": self._controller_loss,
            "stale_lidar": self._timestamp_fault,
            "future_imu": self._timestamp_fault,
            "out_of_order_odom": self._timestamp_fault,
            "nan_command": self._nan_command,
            "duplicate_cmd_publisher": self._duplicate_command,
            "duplicate_tf_authority": self._duplicate_tf,
            "clock_reset": self._clock_reset,
            "cpu_pressure": self._cpu_pressure,
            "queue_pressure": self._queue_pressure,
            "estop_asserted": self._bool_events,
            "reset_while_asserted": self._bool_events,
            "reset_while_moving": self._bool_events,
            "reset_in_auto": self._bool_events,
            "graph_bypass": self._graph_bypass,
        }
        return effects[self.fault_id]()

    def run(self):
        mode = classify_fault(self.fault_id)
        self.emit("ready", "simulation-only fault injector preflight passed")
        if mode == "normal":
            self.emit("completed", "normal scenario: no injection requested")
            return
        self._wait_preconditions()
        detail = self.inject()
        self.emit("triggered", detail)
        self.emit("completed", "bounded simulation fault effect completed")


def main():
    import rospy

    rospy.init_node("rc_fault_injector", anonymous=False)
    fault_id = str(rospy.get_param("~fault_id", ""))
    injector = None
    try:
        values = {name: rospy.get_param(name, None) for name in (
            "/simulation_only", "/use_sim_time", "/hardware_motion_authorized",
            "/passenger_operation_authorized", "/wheelchair_bringup/profile",
        )}
        validate_preflight(values)
        injector = FaultInjector(
            rospy, fault_id,
            timeout_s=rospy.get_param("~timeout_s", 30.0),
            effect_duration_s=rospy.get_param("~effect_duration_s", 1.0),
        )
        injector.run()
    except Exception as exc:
        if injector is not None:
            try:
                injector.emit("failed", str(exc))
            except Exception:
                pass
        rospy.logfatal("simulation fault injector fail-closed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
