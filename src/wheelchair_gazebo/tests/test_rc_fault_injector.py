#!/usr/bin/env python3
"""Pure and static safety contract tests for the simulation fault injector."""

import importlib.util
import json
import os
import xml.etree.ElementTree as ET
import sys
from types import ModuleType

import pytest

PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPT = os.path.join(PACKAGE, "scripts", "rc_fault_injector.py")
CMAKE = os.path.join(PACKAGE, "CMakeLists.txt")
PACKAGE_XML = os.path.join(PACKAGE, "package.xml")
BRINGUP = os.path.abspath(os.path.join(PACKAGE, "..", "wheelchair_bringup",
                                        "launch", "sim_bringup.launch"))
RC_SIM = os.path.join(PACKAGE, "launch", "rc_sim.launch")
SPEC = importlib.util.spec_from_file_location("rc_fault_injector", SCRIPT)
injector = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(injector)

EXPECTED_FAULTS = {
    "lidar_loss", "imu_loss", "odom_loss", "tf_loss", "localizer_loss",
    "decision_loss", "safety_loss", "driver_loss", "generic_process_loss",
    "stale_lidar", "future_imu", "out_of_order_odom", "nan_command",
    "duplicate_cmd_publisher", "duplicate_tf_authority", "clock_reset",
    "cpu_pressure", "queue_pressure", "estop_asserted", "reset_while_asserted",
    "reset_while_moving", "reset_in_auto", "graph_bypass",
}
class _FakeBool:
    def __init__(self, data=False):
        self.data = data


class _FakeString:
    def __init__(self, data=""):
        self.data = data

class _FakeTwist:
    pass


class _FakeSafetyState:
    pass



class _FakePublisher:
    def __init__(self, rospy, topic, fail_publish):
        self.rospy = rospy
        self.topic = topic
        self.fail_publish = fail_publish
        self.unregistered = False

    def get_num_connections(self):
        return self.rospy.estop_connections if self.topic == injector.ESTOP_TOPIC else 0

    def publish(self, message):
        if self.fail_publish and self.topic == injector.ESTOP_TOPIC:
            raise RuntimeError("simulated publish failure")
        self.rospy.messages.append((self.topic, message))

    def unregister(self):
        self.unregistered = True


class _FakeRospy:
    class Time:
        @staticmethod
        def now():
            return type("Stamp", (), {"to_sec": lambda self: 1.0})()

    def __init__(self, estop_connections=1, fail_publish=False):
        self.estop_connections = estop_connections
        self.fail_publish = fail_publish
        self.messages = []
        self.publishers = []

    def Publisher(self, topic, _message_type, queue_size, latch=False):
        publisher = _FakePublisher(self, topic, self.fail_publish)
        self.publishers.append(publisher)
        return publisher

    @staticmethod
    def Subscriber(*_args, **_kwargs):
        return object()

    @staticmethod
    def is_shutdown():
        return False


def _new_fault_injector(monkeypatch, fault_id, **rospy_options):
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = _FakeBool
    std_msgs_msg.String = _FakeString
    std_msgs.msg = std_msgs_msg
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = _FakeTwist
    geometry_msgs.msg = geometry_msgs_msg
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)

    wheelchair_interfaces = ModuleType("wheelchair_interfaces")
    wheelchair_interfaces_msg = ModuleType("wheelchair_interfaces.msg")
    wheelchair_interfaces_msg.SafetyState = _FakeSafetyState
    wheelchair_interfaces.msg = wheelchair_interfaces_msg
    monkeypatch.setitem(sys.modules, "wheelchair_interfaces", wheelchair_interfaces)
    monkeypatch.setitem(sys.modules, "wheelchair_interfaces.msg", wheelchair_interfaces_msg)
    rospy = _FakeRospy(**rospy_options)
    return injector.FaultInjector(rospy, fault_id, timeout_s=1.0, effect_duration_s=0.1), rospy


def _event_phases(rospy):
    return [json.loads(message.data)["phase"] for topic, message in rospy.messages
            if topic == injector.EVENT_TOPIC]


def _estop_values(rospy):
    return [message.data for topic, message in rospy.messages
            if topic == injector.ESTOP_TOPIC]


def test_normal_run_publishes_external_estop_clear_before_ready_without_reset_or_arm(monkeypatch):
    machine, rospy = _new_fault_injector(monkeypatch, "normal")
    monkeypatch.setattr(injector.time, "sleep", lambda _duration: None)

    machine.run()

    assert _estop_values(rospy) == [False] * injector.BASELINE_ESTOP_PUBLISH_COUNT
    assert _event_phases(rospy) == ["ready", "completed"]
    assert not [message for topic, message in rospy.messages
                if topic == "/safety/estop_reset"]
    assert machine._armed is False
    first_ready = next(index for index, (topic, message) in enumerate(rospy.messages)
                       if topic == injector.EVENT_TOPIC
                       and json.loads(message.data)["phase"] == "ready")
    assert all(index < first_ready for index, (topic, _message) in enumerate(rospy.messages)
               if topic == injector.ESTOP_TOPIC)


def test_estop_baseline_wait_and_publish_failures_are_bounded_and_fail_closed(monkeypatch):
    machine, rospy = _new_fault_injector(monkeypatch, "normal", estop_connections=0)
    ticks = iter((0.0, 0.0, 1.1))
    monkeypatch.setattr(injector.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(injector.time, "sleep", lambda _duration: None)

    with pytest.raises(injector.FaultError, match="external-estop subscriber"):
        machine._establish_estop_baseline()

    assert _estop_values(rospy) == []
    assert rospy.publishers[-1].unregistered
    monkeypatch.setattr(injector.time, "monotonic", lambda: 2.0)

    machine, rospy = _new_fault_injector(monkeypatch, "normal", fail_publish=True)
    with pytest.raises(injector.FaultError, match="clear evidence"):
        machine._establish_estop_baseline()

    assert _estop_values(rospy) == []
    assert rospy.publishers[-1].unregistered


def test_every_fault_run_establishes_the_same_initial_estop_clear(monkeypatch):
    monkeypatch.setattr(injector.time, "sleep", lambda _duration: None)

    for fault_id in EXPECTED_FAULTS:
        machine, rospy = _new_fault_injector(monkeypatch, fault_id)
        machine._wait_preconditions = lambda: None
        machine.inject = lambda: "simulated effect"

        machine.run()

        assert _estop_values(rospy) == [False] * injector.BASELINE_ESTOP_PUBLISH_COUNT
        assert _event_phases(rospy)[0] == "ready"


@pytest.mark.parametrize("fault_id", ("estop_asserted", "reset_while_asserted"))
def test_estop_faults_assert_after_baseline_then_deassert(monkeypatch, fault_id):
    machine, rospy = _new_fault_injector(monkeypatch, fault_id)
    monkeypatch.setattr(injector.time, "sleep", lambda _duration: None)
    machine._sleep = lambda _duration: None

    machine._establish_estop_baseline()
    machine._bool_events()

    assert _estop_values(rospy) == (
        [False] * injector.BASELINE_ESTOP_PUBLISH_COUNT + [True, False])


def test_allowlist_is_exact_and_unknown_faults_fail_closed():
    assert injector.FAULT_IDS == EXPECTED_FAULTS
    assert all(injector.classify_fault(value) == "fault" for value in EXPECTED_FAULTS)
    with pytest.raises(injector.FaultError, match="unknown non-normal"):
        injector.classify_fault("hardware_driver_fault")


def test_documented_normal_scenario_and_world_names_are_idle():
    for value in ("", "default", "normal", "nominal", "full_rc_matrix",
                  "wheelchair_rc_scenarios", "wheelchair_rc_scenarios.world"):
        assert injector.classify_fault(value) == "normal"
        assert value not in injector.FAULT_IDS


def test_preflight_requires_exact_simulation_authority_values():
    valid = {
        "/simulation_only": True,
        "/use_sim_time": True,
        "/hardware_motion_authorized": False,
        "/passenger_operation_authorized": False,
        "/wheelchair_bringup/profile": "sim",
    }
    assert injector.validate_preflight(valid)
    for key in valid:
        invalid = dict(valid)
        invalid[key] = ({"sim": "replay"}.get(valid[key], int(valid[key]))
                        if isinstance(valid[key], bool) else "hardware_shadow")
        with pytest.raises(injector.FaultError):
            injector.validate_preflight(invalid)


def test_event_is_canonical_exact_schema_and_rejects_bad_values():
    raw = injector.event_json("lidar_loss", "triggered", 12.5, "node stopped")
    assert raw == ('{"detail":"node stopped","fault_id":"lidar_loss",'
                   '"phase":"triggered","schema":"wheelchair.sim_fault/v1",'
                   '"stamp_s":12.5}')
    assert set(json.loads(raw)) == {"schema", "fault_id", "phase", "stamp_s", "detail"}
    with pytest.raises(injector.FaultError):
        injector.event_json("lidar_loss", "injected", 1.0, "bad phase")
    with pytest.raises(injector.FaultError):
        injector.event_json("lidar_loss", "failed", float("nan"), "bad stamp")


def test_pressure_is_strictly_bounded():
    assert injector.pressure_plan(32, 64 * 1024) == (32, 64 * 1024)
    assert 32 * 64 * 1024 <= injector.MAX_PRESSURE_BYTES
    for values in ((0, 1), (injector.MAX_PRESSURE_MESSAGES + 1, 1),
                   (1, injector.MAX_PRESSURE_BYTES + 1)):
        with pytest.raises(injector.FaultError):
            injector.pressure_plan(*values)
    with pytest.raises(injector.FaultError):
        injector.bounded_effect_seconds(injector.MAX_EFFECT_S + 0.01)


def test_actuator_sink_bypass_is_exact_zero_only():
    source = open(SCRIPT, encoding="utf-8").read()
    assert source.count('ACTUATOR_SINK = "/wheelchair_base_controller/cmd_vel"') == 1
    assert source.count("self._publish_bounded(ACTUATOR_SINK") == 1
    assert "is_exact_zero_twist(zero)" in source
    graph_body = source.split("def _graph_bypass(self):", 1)[1].split("\n    def ", 1)[0]
    assert "Twist()" in graph_body
    assert "linear.x =" not in graph_body and "angular.z =" not in graph_body


def test_each_fault_has_a_real_bounded_dispatch_path():
    source = open(SCRIPT, encoding="utf-8").read()
    assert set(injector.PROCESS_NODES).issubset(EXPECTED_FAULTS)
    for fault_id in EXPECTED_FAULTS - set(injector.PROCESS_NODES):
        assert ('"%s": self.' % fault_id) in source
    assert "rosnode.kill_nodes" in source
    assert "SwitchControllerRequest" in source
    assert 'self.rospy.Publisher("/clock"' in source or '"/clock", Clock' in source
    assert "ESTOP_TOPIC = \"/safety/estop\"" in source
    assert "self.rospy.Publisher(ESTOP_TOPIC, Bool, queue_size=1)" in source
    assert 'self.rospy.Publisher("/safety/estop_reset"' in source


def test_launch_is_opt_in_and_keeps_scenario_and_fault_selection_independent():
    root = ET.parse(BRINGUP).getroot()
    rc_sim_includes = [
        include for include in root.findall("include")
        if include.attrib.get("file") == "$(find wheelchair_gazebo)/launch/rc_sim.launch"
    ]
    assert len(rc_sim_includes) == 1
    rc_sim_args = {
        item.attrib["name"]: item.attrib.get("value")
        for item in rc_sim_includes[0].findall("arg")
    }
    assert rc_sim_args["scenario"] == "$(arg scenario)"

    rc_sim_root = ET.parse(RC_SIM).getroot()
    rc_sim_params = {
        item.attrib["name"]: item.attrib.get("value")
        for item in rc_sim_root.findall("param")
    }
    assert rc_sim_params["/wheelchair_gazebo/scenario"] == "$(arg scenario)"

    nodes = [node for node in root.findall("node")
             if node.attrib.get("type") == "rc_fault_injector.py"]
    assert len(nodes) == 1
    node = nodes[0]
    assert node.attrib.get("if") == "$(arg auto_start)"
    assert node.attrib.get("pkg") == "wheelchair_gazebo"
    params = {item.attrib["name"]: item.attrib.get("value") for item in node.findall("param")}
    assert params == {"fault_id": "$(arg fault_id)"}


def test_script_and_test_are_registered_and_dependencies_declared():
    cmake = open(CMAKE, encoding="utf-8").read()
    assert "scripts/rc_fault_injector.py" in cmake
    assert "tests/test_rc_fault_injector.py" in cmake
    dependencies = {element.text for element in ET.parse(PACKAGE_XML).getroot().findall("exec_depend")}
    assert {"controller_manager_msgs", "geometry_msgs", "nav_msgs", "rosgraph_msgs",
            "rosnode", "rospy", "sensor_msgs", "std_msgs", "tf2_msgs",
            "wheelchair_interfaces"}.issubset(dependencies)
