#!/usr/bin/env python3
"""ROS-independent tests for the fail-closed simulation command adapter."""

import importlib.util
import math
import pathlib
import sys
import unittest
import types
from unittest import mock


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "simulation_controller_adapter.py"


def load_adapter():
    spec = importlib.util.spec_from_file_location(
        "simulation_controller_adapter_under_test", str(SCRIPT)
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


adapter = load_adapter()


class SimulationCommandCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = adapter.SimulationCommandCore()
        self.command = adapter.PlanarCommand(0.4, 0.0, 0.0, 0.0, 0.0, -0.2)

    def record(self, command=None, source=10.0, receipt=20.0,
               source_now=10.0, receipt_now=20.0):
        return self.core.record(
            self.command if command is None else command,
            source,
            receipt,
            source_now,
            receipt_now,
        )

    def test_starts_at_exact_finite_zero(self):
        result = self.core.output(0.0, 0.0)
        self.assertEqual(result, adapter.ZERO)
        self.assertEqual(result.axes(), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(all(math.isfinite(value) for value in result.axes()))

    def test_valid_planar_command_survives_only_within_both_timeouts(self):
        self.assertTrue(self.record())
        self.assertEqual(self.core.output(10.10, 20.10), self.command)

        source_stale = adapter.SimulationCommandCore()
        self.assertTrue(source_stale.record(self.command, 10.0, 20.0, 10.0, 20.0))
        self.assertEqual(source_stale.output(10.100001, 20.05), adapter.ZERO)

        receipt_stale = adapter.SimulationCommandCore()
        self.assertTrue(receipt_stale.record(self.command, 10.0, 20.0, 10.0, 20.0))
        self.assertEqual(receipt_stale.output(10.05, 20.100001), adapter.ZERO)

    def test_nonfinite_or_unsupported_axis_clears_prior_nonzero(self):
        malformed = (
            adapter.PlanarCommand(math.nan, 0.0, 0.0, 0.0, 0.0, 0.1),
            adapter.PlanarCommand(0.1, 1.0, 0.0, 0.0, 0.0, 0.1),
            adapter.PlanarCommand(0.1, 0.0, 1.0, 0.0, 0.0, 0.1),
            adapter.PlanarCommand(0.1, 0.0, 0.0, 1.0, 0.0, 0.1),
            adapter.PlanarCommand(0.1, 0.0, 0.0, 0.0, 1.0, 0.1),
        )
        for index, command in enumerate(malformed):
            core = adapter.SimulationCommandCore()
            self.assertTrue(core.record(self.command, 10.0, 20.0, 10.0, 20.0))
            now = 10.01 + index * 0.001
            receipt = 20.01 + index * 0.001
            self.assertFalse(core.record(command, now, receipt, now, receipt))
            self.assertEqual(core.output(now, receipt), adapter.ZERO)

    def test_future_and_regressing_timestamps_are_rejected_fail_closed(self):
        self.assertTrue(self.record())
        self.assertFalse(self.record(source=10.2, receipt=20.01,
                                     source_now=10.1, receipt_now=20.01))
        self.assertEqual(self.core.output(10.1, 20.01), adapter.ZERO)

        core = adapter.SimulationCommandCore()
        self.assertTrue(core.record(self.command, 5.0, 8.0, 5.0, 8.0))
        self.assertFalse(core.record(self.command, 4.9, 8.01, 5.01, 8.01))
        self.assertEqual(core.output(5.01, 8.01), adapter.ZERO)

    def test_clock_reset_immediately_discards_nonzero_and_can_recover(self):
        self.assertTrue(self.record())
        self.assertEqual(self.core.output(9.0, 20.01), adapter.ZERO)
        self.assertEqual(self.core.output(9.01, 20.02), adapter.ZERO)
        self.assertTrue(self.core.record(self.command, 9.02, 20.03, 9.02, 20.03))
        self.assertEqual(self.core.output(9.03, 20.04), self.command)

    def test_timeout_configuration_cannot_exceed_contract(self):
        with self.assertRaises(ValueError):
            adapter.SimulationCommandCore(0.100001)
        with self.assertRaises(ValueError):
            adapter.SimulationCommandCore(0.0)

    def test_clear_discards_a_buffered_command(self):
        self.assertTrue(self.record())
        self.core.clear()
        self.assertEqual(self.core.output(10.01, 20.01), adapter.ZERO)


class _Vector:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vector()
        self.angular = _Vector()


class _Odometry:
    pass


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FakeRospy:
    def __init__(self, timeout=adapter.CONTROLLER_READY_TIMEOUT_S):
        self.params = {
            "/simulation_only": True,
            "/use_sim_time": True,
            "/hardware_motion_authorized": False,
            "/passenger_operation_authorized": False,
            "~controller_ready_timeout_s": timeout,
        }
        self.publisher = _Publisher()
        self.subscribers = []
        self.shutdown_checks = 0

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def Publisher(self, *_args, **_kwargs):
        return self.publisher

    def Subscriber(self, topic, message_type, callback, **kwargs):
        self.subscribers.append((topic, message_type, callback, kwargs))
        return object()

    def get_rostime(self):
        return types.SimpleNamespace(to_sec=lambda: 10.0)

    def is_shutdown(self):
        self.shutdown_checks += 1
        return self.shutdown_checks > 1


def make_ros_adapter(timeout=adapter.CONTROLLER_READY_TIMEOUT_S):
    rospy = _FakeRospy(timeout)
    modules = {
        "rospy": rospy,
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.SimpleNamespace(Twist=_Twist),
        "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": types.SimpleNamespace(Odometry=_Odometry),
    }
    with mock.patch.dict(sys.modules, modules):
        instance = adapter.SimulationControllerAdapter()
    return instance, rospy


class ControllerReadinessBarrierTest(unittest.TestCase):
    def test_publishes_nothing_before_readiness(self):
        instance, rospy = make_ros_adapter()
        self.assertEqual(rospy.publisher.messages, [])
        self.assertFalse(instance._accept_commands)

    def test_first_odom_unlocks_with_exact_zero_first(self):
        instance, rospy = make_ros_adapter()
        instance._ready_callback(_Odometry())
        with mock.patch.object(adapter.time, "sleep", return_value=None):
            instance.run()
        self.assertGreaterEqual(len(rospy.publisher.messages), 2)
        first = rospy.publisher.messages[0]
        self.assertEqual(
            (first.linear.x, first.linear.y, first.linear.z,
             first.angular.x, first.angular.y, first.angular.z),
            adapter.ZERO_COMMAND,
        )

    def test_timeout_is_bounded_and_fails_closed(self):
        instance, rospy = make_ros_adapter(timeout=0.001)
        with self.assertRaisesRegex(RuntimeError, "timed out waiting"):
            instance.run()
        self.assertEqual(rospy.publisher.messages, [])

    def test_malformed_timeout_is_rejected(self):
        for malformed in (0.0, -1.0, math.inf, math.nan, True, "15"):
            with self.subTest(malformed=malformed):
                with self.assertRaises(ValueError):
                    make_ros_adapter(timeout=malformed)

    def test_pre_readiness_command_is_not_replayed(self):
        instance, rospy = make_ros_adapter()
        command = _Twist()
        command.linear.x = 0.4
        command.angular.z = -0.2
        instance._command_callback(command)
        instance._ready_callback(_Odometry())
        with mock.patch.object(adapter.time, "sleep", return_value=None):
            instance.run()
        for message in rospy.publisher.messages:
            self.assertEqual(
                (message.linear.x, message.linear.y, message.linear.z,
                 message.angular.x, message.angular.y, message.angular.z),
                adapter.ZERO_COMMAND,
            )


class AdapterSurfaceTest(unittest.TestCase):
    def test_topics_rate_and_node_identity_are_fixed(self):
        self.assertEqual(adapter.SOURCE_TOPIC, "/cmd_vel_safe")
        self.assertEqual(adapter.SINK_TOPIC, "/wheelchair_base_controller/cmd_vel")
        self.assertEqual(adapter.READY_TOPIC, "/wheelchair_base_controller/odom")
        self.assertEqual(adapter.CONTROLLER_READY_TIMEOUT_S, 60.0)
        self.assertLessEqual(adapter.COMMAND_TIMEOUT_S, 0.10)
        self.assertGreaterEqual(adapter.PUBLISH_HZ, 50.0)
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn('rospy.init_node("simulation_controller_adapter")', source)
        self.assertNotIn("wheelchair_hardware", source)
        self.assertNotIn("motor_topic", source)

    def test_authority_gates_precede_the_only_publisher(self):
        source = SCRIPT.read_text(encoding="utf-8")
        publisher_offset = source.index("rospy.Publisher(")
        for parameter, expected in (
            ("/simulation_only", "True"),
            ("/use_sim_time", "True"),
            ("/hardware_motion_authorized", "False"),
            ("/passenger_operation_authorized", "False"),
        ):
            gate = '("{}", {})'.format(parameter, expected)
            self.assertIn(gate, source)
            self.assertLess(source.index(gate), publisher_offset)
        self.assertEqual(source.count("rospy.Publisher("), 1)
        self.assertEqual(source.count("rospy.Subscriber("), 2)
        self.assertIn("queue_size=1", source)
        self.assertIn("tcp_nodelay=True", source)


if __name__ == "__main__":
    unittest.main()
