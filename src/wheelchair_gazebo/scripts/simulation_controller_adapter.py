#!/usr/bin/env python3
"""Fail-closed, simulation-only adapter for the approved Gazebo command sink."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

SOURCE_TOPIC = "/cmd_vel_safe"
SINK_TOPIC = "/wheelchair_base_controller/cmd_vel"
READY_TOPIC = "/wheelchair_base_controller/odom"
# Cold-loading the exact Hanyang occupancy mesh is bounded but can exceed 15 s.
CONTROLLER_READY_TIMEOUT_S = 60.0
COMMAND_TIMEOUT_S = 0.10
PUBLISH_HZ = 50.0
ZERO_COMMAND = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class PlanarCommand:
    """ROS-independent representation of all six Twist axes."""

    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float

    def axes(self) -> Tuple[float, float, float, float, float, float]:
        return (
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        )


ZERO = PlanarCommand(*ZERO_COMMAND)


class SimulationCommandCore:
    """Validate one command stream and return zero after any timing/data fault."""

    def __init__(self, timeout_s: float = COMMAND_TIMEOUT_S) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0 or timeout_s > COMMAND_TIMEOUT_S:
            raise ValueError("timeout_s must be finite, positive, and no greater than 0.10")
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._command = ZERO
        self._source_time_s: Optional[float] = None
        self._receipt_time_s: Optional[float] = None
        self._last_source_now_s: Optional[float] = None
        self._last_receipt_now_s: Optional[float] = None

    @staticmethod
    def _finite_nonnegative(value: float) -> bool:
        try:
            return math.isfinite(value) and value >= 0.0
        except TypeError:
            return False
    @staticmethod
    def _finite(value: float) -> bool:
        try:
            return math.isfinite(value)
        except TypeError:
            return False

    @classmethod
    def _valid_command(cls, command: PlanarCommand) -> bool:
        axes: Iterable[float] = command.axes()
        return (
            all(cls._finite(value) for value in axes)
            and command.linear_y == 0.0
            and command.linear_z == 0.0
            and command.angular_x == 0.0
            and command.angular_y == 0.0
        )

    def _clear(self) -> None:
        self._command = ZERO
        self._source_time_s = None
        self._receipt_time_s = None

    def clear(self) -> None:
        """Discard buffered command and timing state."""
        with self._lock:
            self._clear()
            self._last_source_now_s = None
            self._last_receipt_now_s = None

    def record(
        self,
        command: PlanarCommand,
        source_time_s: float,
        receipt_time_s: float,
        source_now_s: float,
        receipt_now_s: float,
    ) -> bool:
        """Record a command only when its data and both time domains are valid."""
        with self._lock:
            times = (source_time_s, receipt_time_s, source_now_s, receipt_now_s)
            if not all(self._finite_nonnegative(value) for value in times):
                self._clear()
                return False

            time_regressed = (
                (self._last_source_now_s is not None and source_now_s < self._last_source_now_s)
                or (self._last_receipt_now_s is not None and receipt_now_s < self._last_receipt_now_s)
                or (self._source_time_s is not None and source_time_s < self._source_time_s)
                or (self._receipt_time_s is not None and receipt_time_s < self._receipt_time_s)
            )
            self._last_source_now_s = source_now_s
            self._last_receipt_now_s = receipt_now_s
            if (
                time_regressed
                or source_time_s > source_now_s
                or receipt_time_s > receipt_now_s
                or not self._valid_command(command)
            ):
                self._clear()
                return False

            self._command = command
            self._source_time_s = source_time_s
            self._receipt_time_s = receipt_time_s
            return True

    def output(self, source_now_s: float, receipt_now_s: float) -> PlanarCommand:
        """Return the current command, or exact zero while missing/stale/faulted."""
        with self._lock:
            if not self._finite_nonnegative(source_now_s) or not self._finite_nonnegative(receipt_now_s):
                self._clear()
                return ZERO

            clock_reset = (
                (self._last_source_now_s is not None and source_now_s < self._last_source_now_s)
                or (self._last_receipt_now_s is not None and receipt_now_s < self._last_receipt_now_s)
            )
            self._last_source_now_s = source_now_s
            self._last_receipt_now_s = receipt_now_s
            if clock_reset or self._source_time_s is None or self._receipt_time_s is None:
                self._clear()
                return ZERO

            source_age = source_now_s - self._source_time_s
            receipt_age = receipt_now_s - self._receipt_time_s
            if (
                source_age < 0.0
                or receipt_age < 0.0
                or source_age > self.timeout_s + 1e-12
                or receipt_age > self.timeout_s + 1e-12
            ):
                self._clear()
                return ZERO
            return self._command


class SimulationControllerAdapter:
    """Thin ROS boundary that waits for controller runtime evidence."""

    def __init__(self) -> None:
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        self.rospy = rospy
        self.Twist = Twist
        self.core = SimulationCommandCore()
        self._ready = threading.Event()
        self._accept_commands = False
        ready_timeout_s = rospy.get_param(
            "~controller_ready_timeout_s", CONTROLLER_READY_TIMEOUT_S
        )
        if (
            isinstance(ready_timeout_s, bool)
            or not isinstance(ready_timeout_s, (int, float))
            or not math.isfinite(ready_timeout_s)
            or ready_timeout_s <= 0.0
        ):
            raise ValueError(
                "~controller_ready_timeout_s must be a finite positive number"
            )
        self.ready_timeout_s = float(ready_timeout_s)

        required = (
            ("/simulation_only", True),
            ("/use_sim_time", True),
            ("/hardware_motion_authorized", False),
            ("/passenger_operation_authorized", False),
        )
        for parameter, expected in required:
            actual = rospy.get_param(parameter, None)
            if actual is not expected:
                raise RuntimeError("{} must be exactly {!r}".format(parameter, expected))

        self.publisher = rospy.Publisher(SINK_TOPIC, Twist, queue_size=1)
        self.subscriber = rospy.Subscriber(
            SOURCE_TOPIC,
            Twist,
            self._command_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.ready_subscriber = rospy.Subscriber(
            READY_TOPIC,
            Odometry,
            self._ready_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

    def _to_twist(self, command: PlanarCommand):
        message = self.Twist()
        message.linear.x = command.linear_x
        message.linear.y = command.linear_y
        message.linear.z = command.linear_z
        message.angular.x = command.angular_x
        message.angular.y = command.angular_y
        message.angular.z = command.angular_z
        return message

    def _ready_callback(self, _message) -> None:
        self._ready.set()

    def _command_callback(self, message) -> None:
        if not self._accept_commands:
            return
        source_now_s = self.rospy.get_rostime().to_sec()
        receipt_now_s = time.monotonic()
        command = PlanarCommand(
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        self.core.record(
            command,
            source_now_s,
            receipt_now_s,
            source_now_s,
            receipt_now_s,
        )

    def _wait_until_ready(self) -> None:
        deadline_s = time.monotonic() + self.ready_timeout_s
        while not self._ready.is_set():
            remaining_s = deadline_s - time.monotonic()
            if remaining_s <= 0.0:
                raise RuntimeError("timed out waiting for {}".format(READY_TOPIC))
            if self.rospy.is_shutdown():
                raise RuntimeError("shutdown before wheelchair controller became ready")
            self._ready.wait(min(remaining_s, 0.05))

    def run(self) -> None:
        self._wait_until_ready()
        self.core.clear()
        self.publisher.publish(self._to_twist(ZERO))
        self._accept_commands = True
        period_s = 1.0 / PUBLISH_HZ
        next_tick_s = time.monotonic()
        try:
            while not self.rospy.is_shutdown():
                receipt_now_s = time.monotonic()
                source_now_s = self.rospy.get_rostime().to_sec()
                command = self.core.output(source_now_s, receipt_now_s)
                self.publisher.publish(self._to_twist(command))
                next_tick_s += period_s
                delay_s = next_tick_s - time.monotonic()
                if delay_s > 0.0:
                    time.sleep(delay_s)
                else:
                    next_tick_s = time.monotonic()
        finally:
            self._accept_commands = False
            self.publisher.publish(self._to_twist(ZERO))


def main() -> None:
    import rospy

    rospy.init_node("simulation_controller_adapter")
    SimulationControllerAdapter().run()


if __name__ == "__main__":
    main()
