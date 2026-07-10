#!/usr/bin/env python3
"""Config-backed runtime mode policy publisher for the ROS1 wheelchair scaffold."""

from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class ModeLimits:
    max_linear_speed: float
    max_angular_speed: float
    requires_geofence: bool = True


class ModePolicy:
    def __init__(self, modes: Dict[str, ModeLimits], default_mode: str = "sidewalk"):
        if default_mode not in modes:
            raise ValueError(f"default mode {default_mode!r} is not configured")
        self.modes = dict(modes)
        self.current_mode = default_mode

    def set_mode(self, mode: str) -> bool:
        if mode not in self.modes:
            return False
        self.current_mode = mode
        return True

    def is_allowed(self, geofence_ok: bool = True) -> bool:
        limits = self.modes[self.current_mode]
        return geofence_ok or not limits.requires_geofence

    def current_limits(self) -> ModeLimits:
        return self.modes[self.current_mode]


DEFAULT_POLICY = ModePolicy(
    modes={
        "sidewalk": ModeLimits(max_linear_speed=0.55, max_angular_speed=0.85, requires_geofence=True),
        "road_free_space": ModeLimits(max_linear_speed=0.70, max_angular_speed=1.00, requires_geofence=True),
    },
    default_mode="sidewalk",
)


class ModeManagerRosNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import Bool, String

        modes_param = rospy.get_param("~modes", {})
        default_mode = rospy.get_param("~default_mode", "sidewalk")
        if modes_param:
            modes = {
                name: ModeLimits(
                    max_linear_speed=float(cfg.get("max_linear_speed", 0.55)),
                    max_angular_speed=float(cfg.get("max_angular_speed", 0.85)),
                    requires_geofence=bool(cfg.get("requires_geofence", True)),
                )
                for name, cfg in modes_param.items()
            }
            self.policy = ModePolicy(modes, default_mode)
        else:
            self.policy = DEFAULT_POLICY
            self.policy.set_mode(default_mode)

        self.geofence_ok = True
        self.mode_pub = rospy.Publisher("/runtime/mode", String, queue_size=1, latch=True)
        self.allowed_pub = rospy.Publisher("/safety/mode_allowed", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/runtime/mode_request", String, self._mode_request_cb, queue_size=1)
        rospy.Subscriber("/safety/geofence_ok", Bool, self._geofence_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self._timer_cb)
        rospy.loginfo("mode_manager started with mode=%s", self.policy.current_mode)

    def _mode_request_cb(self, msg):
        import rospy

        requested = msg.data.strip()
        if not self.policy.set_mode(requested):
            rospy.logwarn("ignoring unknown wheelchair runtime mode: %s", requested)

    def _geofence_cb(self, msg):
        self.geofence_ok = bool(msg.data)

    def _timer_cb(self, _event):
        from std_msgs.msg import Bool, String

        self.mode_pub.publish(String(data=self.policy.current_mode))
        self.allowed_pub.publish(Bool(data=self.policy.is_allowed(self.geofence_ok)))


def run_ros_node() -> None:
    import rospy

    rospy.init_node("mode_manager")
    ModeManagerRosNode()
    rospy.spin()


if __name__ == "__main__":
    run_ros_node()
