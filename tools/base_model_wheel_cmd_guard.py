#!/usr/bin/env python3

from pathlib import Path
import importlib.util
import sys

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray


MODEL_DIRS = (
    Path(__file__).resolve().parent,
    Path(__file__).resolve().parents[1] / "static_livox_localization",
    Path(__file__).resolve().parents[1]
    / "src" / "static_livox_localization" / "scripts",
    Path(__file__).resolve().parents[2]
    / "static_livox_localization" / "scripts",
)
MODEL_PATH = None
for model_dir in MODEL_DIRS:
    candidate = model_dir / "wheel_command_model.py"
    if candidate.is_file():
        MODEL_PATH = candidate
        break
if MODEL_PATH is None:
    raise ImportError("wheel_command_model.py must accompany this guard")
MODEL_SPEC = importlib.util.spec_from_file_location(
    "wheel_command_model", MODEL_PATH)
if MODEL_SPEC is None or MODEL_SPEC.loader is None:
    raise ImportError("wheel_command_model.py has no import loader")
MODEL = importlib.util.module_from_spec(MODEL_SPEC)
sys.modules["wheel_command_model"] = MODEL
MODEL_SPEC.loader.exec_module(MODEL)

from wheel_command_model import (  # noqa: E402
    COUNTS_PER_KMH,
    MAGNITUDE_OFFSET,
    MAX_ANGULAR_RAD_S,
    MAX_LINEAR_MPS,
    STOP_COMMAND,
    TURN_AUTHORITY_KMH,
    TURN_AUTHORITY_MAX_LINEAR_MPS,
    WHEEL_SEPARATION_M,
    YAW_DEADBAND_RAD_S,
    encode_wheel_command,
)


AUTO_MODE = 65
EXPECTED_CMD_CALLER = "/tip_guard"
EXPECTED_STATUS_CALLER = "/uart"


def message_caller_id(message):
    header = getattr(message, "_connection_header", None)
    if not isinstance(header, dict):
        return ""
    return str(header.get("callerid", "")).strip()


compute_wheel_command = encode_wheel_command


class WheelCommandGuard:
    def __init__(self):
        rospy.init_node("wheel_cmd")
        self.publisher = rospy.Publisher(
            "wheel_cmd", Int16MultiArray, queue_size=1)
        self.mode = None
        self.fault_latched = False
        rospy.Subscriber(
            "cmd_vel", Twist, self.on_velocity, queue_size=1)
        rospy.Subscriber(
            "wheel_status",
            Int16MultiArray,
            self.on_wheel_status,
            queue_size=5,
        )

    def publish(self, values):
        message = Int16MultiArray()
        message.data = list(values)
        self.publisher.publish(message)

    def publish_stop(self):
        self.publish(STOP_COMMAND)

    def on_velocity(self, message):
        if self.mode != AUTO_MODE:
            self.publish_stop()
            return
        if message_caller_id(message) != EXPECTED_CMD_CALLER:
            self.fault_latched = True
        command = compute_wheel_command(
            message.linear.x, message.angular.z)
        if command is None:
            self.fault_latched = True
        if self.fault_latched:
            self.publish_stop()
            return
        self.publish(command)

    def on_wheel_status(self, message):
        if message_caller_id(message) != EXPECTED_STATUS_CALLER or \
                len(message.data) <= 1:
            self.mode = None
            self.fault_latched = True
            self.publish_stop()
            return
        next_mode = int(message.data[1])
        if next_mode != AUTO_MODE:
            self.fault_latched = False
        elif self.mode != AUTO_MODE:
            self.fault_latched = False
        self.mode = next_mode


if __name__ == "__main__":
    WheelCommandGuard()
    rospy.spin()
