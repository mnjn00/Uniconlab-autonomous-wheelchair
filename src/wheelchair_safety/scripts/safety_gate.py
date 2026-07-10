#!/usr/bin/env python3
"""ROS1 safety gate for wheelchair cmd_vel arbitration.

The pure-Python SafetyGateCore is intentionally independent of rospy so unit tests can run
on hosts without ROS Noetic installed.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0

    def is_zero(self) -> bool:
        return self.linear_x == 0.0 and self.angular_z == 0.0


@dataclass(frozen=True)
class SafetyConfig:
    stale_timeout_s: float = 0.30
    max_linear_speed: float = 0.55
    max_angular_speed: float = 0.85


@dataclass(frozen=True)
class GateInputs:
    cmd: Optional[VelocityCommand]
    cmd_age_s: Optional[float]
    e_stop: bool = False
    e_stop_reset: bool = False
    geofence_ok: bool = True
    mode_allowed: bool = True
    collision_stop: bool = False


@dataclass(frozen=True)
class GateDecision:
    command: VelocityCommand
    reason: str
    e_stop_latched: bool


class SafetyGateCore:
    """Priority gate: e-stop latch > stale watchdog > geofence/mode > collision > speed cap > nominal."""

    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        self.e_stop_latched = False

    def reset(self) -> None:
        self.e_stop_latched = False

    def evaluate(self, inputs: GateInputs) -> GateDecision:
        if inputs.e_stop:
            self.e_stop_latched = True
        elif inputs.e_stop_reset:
            self.e_stop_latched = False

        if self.e_stop_latched:
            return self._stop("e_stop_latched")

        if inputs.cmd is None or inputs.cmd_age_s is None or inputs.cmd_age_s > self.config.stale_timeout_s:
            return self._stop("stale_watchdog")

        if not inputs.geofence_ok or not inputs.mode_allowed:
            return self._stop("geofence_or_mode_violation")

        if inputs.collision_stop:
            return self._stop("collision_stop")

        capped = self._cap(inputs.cmd)
        if capped != inputs.cmd:
            return GateDecision(capped, "speed_cap", self.e_stop_latched)
        return GateDecision(capped, "nominal", self.e_stop_latched)

    def _stop(self, reason: str) -> GateDecision:
        return GateDecision(VelocityCommand(), reason, self.e_stop_latched)

    def _cap(self, cmd: VelocityCommand) -> VelocityCommand:
        return VelocityCommand(
            linear_x=_clamp(cmd.linear_x, -self.config.max_linear_speed, self.config.max_linear_speed),
            angular_z=_clamp(cmd.angular_z, -self.config.max_angular_speed, self.config.max_angular_speed),
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _twist_to_command(msg) -> VelocityCommand:
    return VelocityCommand(linear_x=float(msg.linear.x), angular_z=float(msg.angular.z))


def _command_to_twist(command: VelocityCommand):
    from geometry_msgs.msg import Twist

    msg = Twist()
    msg.linear.x = command.linear_x
    msg.angular.z = command.angular_z
    return msg


class SafetyGateRosNode:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool, String

        cfg = SafetyConfig(
            stale_timeout_s=float(rospy.get_param("~stale_timeout_s", 0.30)),
            max_linear_speed=float(rospy.get_param("~max_linear_speed", 0.55)),
            max_angular_speed=float(rospy.get_param("~max_angular_speed", 0.85)),
        )
        self.core = SafetyGateCore(cfg)
        self.last_cmd = None
        self.last_cmd_time = None
        self.e_stop = False
        self.e_stop_reset_requested = False
        self.geofence_ok = True
        self.mode_allowed = True
        self.collision_stop = False

        input_cmd_topic = rospy.get_param("~input_cmd_topic", "/cmd_vel_nav")
        output_cmd_topic = rospy.get_param("~output_cmd_topic", "/cmd_vel_safe")
        estop_topic = rospy.get_param("~estop_topic", "/safety/estop")
        estop_reset_topic = rospy.get_param("~estop_reset_topic", "/safety/estop_reset")
        geofence_ok_topic = rospy.get_param("~geofence_ok_topic", "/safety/geofence_ok")
        mode_allowed_topic = rospy.get_param("~mode_allowed_topic", "/safety/mode_allowed")
        collision_stop_topic = rospy.get_param("~collision_stop_topic", "/safety/collision_stop")
        status_topic = rospy.get_param("~status_topic", "/safety/status")
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))

        self.pub = rospy.Publisher(output_cmd_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(input_cmd_topic, Twist, self._cmd_cb, queue_size=1)
        rospy.Subscriber(estop_topic, Bool, self._estop_cb, queue_size=1)
        rospy.Subscriber(estop_reset_topic, Bool, self._estop_reset_cb, queue_size=1)
        rospy.Subscriber(geofence_ok_topic, Bool, self._geofence_ok_cb, queue_size=1)
        rospy.Subscriber(mode_allowed_topic, Bool, self._mode_allowed_cb, queue_size=1)
        rospy.Subscriber(collision_stop_topic, Bool, self._collision_stop_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / publish_rate_hz), self._timer_cb)

        rospy.loginfo(
            "safety_gate routing %s -> %s with stale_timeout_s=%.3f max_linear=%.3f max_angular=%.3f",
            input_cmd_topic,
            output_cmd_topic,
            cfg.stale_timeout_s,
            cfg.max_linear_speed,
            cfg.max_angular_speed,
        )

    def _cmd_cb(self, msg):
        import rospy

        self.last_cmd = _twist_to_command(msg)
        self.last_cmd_time = rospy.Time.now()

    def _estop_cb(self, msg):
        self.e_stop = bool(msg.data)

    def _estop_reset_cb(self, msg):
        if bool(msg.data):
            self.e_stop_reset_requested = True

    def _geofence_ok_cb(self, msg):
        self.geofence_ok = bool(msg.data)

    def _mode_allowed_cb(self, msg):
        self.mode_allowed = bool(msg.data)

    def _collision_stop_cb(self, msg):
        self.collision_stop = bool(msg.data)

    def _timer_cb(self, _event):
        import rospy
        from std_msgs.msg import String

        now = rospy.Time.now()
        if self.last_cmd_time is None:
            cmd_age_s = None
        else:
            cmd_age_s = (now - self.last_cmd_time).to_sec()

        reset_requested = self.e_stop_reset_requested
        self.e_stop_reset_requested = False
        decision = self.core.evaluate(
            GateInputs(
                cmd=self.last_cmd,
                cmd_age_s=cmd_age_s,
                e_stop=self.e_stop,
                e_stop_reset=reset_requested,
                geofence_ok=self.geofence_ok,
                mode_allowed=self.mode_allowed,
                collision_stop=self.collision_stop,
            )
        )
        self.pub.publish(_command_to_twist(decision.command))
        self.status_pub.publish(String(data=decision.reason))


def run_ros_node() -> None:
    import rospy

    rospy.init_node("safety_gate")
    SafetyGateRosNode()
    rospy.spin()


if __name__ == "__main__":
    run_ros_node()
