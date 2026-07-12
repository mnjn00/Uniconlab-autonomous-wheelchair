"""Static launch-topology checks for the inert hardware boundary."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src" / "wheelchair_bringup" / "launch"
ADAPTER = ROOT / "src" / "wheelchair_hardware" / "scripts" / "hardware_adapter.py"


def _root(name):
    return ET.parse(str(LAUNCH / name)).getroot()


def _arg(root, name):
    return next(element for element in root.findall("arg") if element.get("name") == name)


def test_generic_bringup_defaults_to_shadow_without_unconditional_relay():
    root = _root("bringup.launch")
    assert _arg(root, "hardware_profile").get("default") == "hardware_shadow"
    assert not root.findall("node")

    groups = root.findall("group")
    conditions = {group.get("if") for group in groups}
    assert any("hardware_shadow" in condition for condition in conditions)
    assert any("hardware_enabled" in condition for condition in conditions)
    text = (LAUNCH / "bringup.launch").read_text()
    assert "topic_tools" not in text
    assert "/base_controller/cmd_vel" not in text


def test_enabled_profile_is_explicit_and_fail_closed_by_default():
    root = _root("hardware_enabled.launch")
    assert _arg(root, "hardware_enable").get("default") == "false"
    assert _arg(root, "driver_manifest").get("default").endswith(
        "/config/driver-unverified.yaml"
    )
    assert _arg(root, "release_authority").get("default").endswith(
        "/contracts/wp0/A16-release-authority.yaml"
    )
    assert _arg(root, "bundle_root").get("default").startswith("/nonexistent/")
    assert _arg(root, "runtime_evidence").get("default") == "UNSET"
    assert _arg(root, "runtime_evidence_sha256").get("default") == "0" * 64

    text = (LAUNCH / "hardware_enabled.launch").read_text()
    assert "/base_controller/cmd_vel" not in text
    assert "driver_topic" not in text
    assert "--hardware-enable $(arg hardware_enable)" in text
    assert "--release-authority $(arg release_authority)" in text
    assert "--bundle-root $(arg bundle_root)" in text
    assert "--runtime-evidence $(arg runtime_evidence)" in text
    assert "--runtime-evidence-sha256 $(arg runtime_evidence_sha256)" in text


def test_shadow_has_no_motor_publisher_or_real_topic_configuration():
    root = _root("hardware_shadow.launch")
    nodes = root.findall("node")
    assert len(nodes) == 1
    assert nodes[0].get("type") == "hardware_adapter.py"
    assert "--profile hardware_shadow" in nodes[0].get("args")
    text = (LAUNCH / "hardware_shadow.launch").read_text()
    assert "topic_tools" not in text
    assert "driver_topic" not in text
    assert "/base_controller/cmd_vel" not in text


def test_adapter_preflights_before_publishers_and_uses_latest_only_queues():
    source = ADAPTER.read_text()
    tree = ast.parse(source)
    main = next(
        node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main"
    )
    calls = [
        node
        for node in ast.walk(main)
        if isinstance(node, ast.Call)
    ]
    preflight_line = min(
        node.lineno
        for node in calls
        if isinstance(node.func, ast.Name) and node.func.id == "_adapter_preflight"
    )
    publisher_lines = [
        node.lineno
        for node in calls
        if isinstance(node.func, ast.Attribute) and node.func.attr == "Publisher"
    ]
    assert publisher_lines and preflight_line < min(publisher_lines)
    assert "queue_size=1" in source
    assert 'authority.get("release_scope")' in source
    assert 'get("hardware_motion_authorized")' in source
    assert 'get("passenger_operation_authorized")' in source
    assert "math.isfinite" in source
    assert "from driver_contract import" in source
    assert "import rospy" not in "\n".join(source.splitlines()[:20])


def test_only_safe_command_topic_is_subscribed_and_sim_has_no_hardware_path():
    adapter = ADAPTER.read_text()
    assert 'safe_topic != "/cmd_vel_safe"' in adapter
    tree = ast.parse(adapter)
    safe_subscribers = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "Subscriber"
        and node.args
        and isinstance(node.args[0], ast.Name)
        and node.args[0].id == "safe_topic"
    ]
    assert len(safe_subscribers) == 1
    assert "rospy.Publisher(driver_topic" not in adapter

    sim = (LAUNCH / "sim_bringup.launch").read_text()
    rc_sim = (ROOT / "src" / "wheelchair_gazebo" / "launch" / "rc_sim.launch").read_text()
    sim_adapter = (
        ROOT / "src" / "wheelchair_gazebo" / "scripts" / "simulation_controller_adapter.py"
    ).read_text()
    assert "wheelchair_hardware" not in sim
    assert "hardware_adapter" not in sim
    assert 'type="simulation_controller_adapter.py"' in rc_sim
    assert 'SOURCE_TOPIC = "/cmd_vel_safe"' in sim_adapter
    assert 'SINK_TOPIC = "/wheelchair_base_controller/cmd_vel"' in sim_adapter

def test_sim_fault_selection_is_explicit_and_defaults_to_normal():
    root = _root("sim_bringup.launch")
    assert _arg(root, "fault_id").get("default") == "normal"

    injector = next(
        node for node in root.findall("node")
        if node.get("type") == "rc_fault_injector.py"
    )
    assert injector.get("if") == "$(arg auto_start)"
    fault_param = next(
        param for param in injector.findall("param")
        if param.get("name") == "fault_id"
    )
    assert fault_param.get("value") == "$(arg fault_id)"
