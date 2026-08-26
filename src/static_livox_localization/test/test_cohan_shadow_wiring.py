"""Machine contracts for the isolated CoHAN/HATEB shadow graph."""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PACKAGE = Path(__file__).parents[1]
SCRIPTS = PACKAGE / "scripts"


def publisher_topics(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    topics = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if not (
            isinstance(function, ast.Attribute)
            and function.attr == "Publisher"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
        ):
            continue
        topics.append(node.args[0].value)
    return topics


def test_shadow_adapter_has_no_motion_authority_publisher():
    # Given: the concrete ROS adapter source that will run beside HATEB.
    node = SCRIPTS / "human_aware_shadow_node.py"
    assert node.exists(), "no shadow ROS adapter exists"

    # When: every statically declared publisher topic is extracted.
    topics = publisher_topics(node)

    # Then: only advisory human/status surfaces exist.
    assert set(topics) == {
        "/human_aware_shadow/tracked_agents",
        "/human_aware_shadow/status",
    }
    forbidden = {
        "/cmd_vel_raw",
        "/cmd_vel_gated",
        "/cmd_vel",
        "/wheel_cmd",
        "/mode_cmd",
    }
    assert forbidden.isdisjoint(topics)


def test_shadow_metadata_pins_official_cohan_contract():
    # Given: the machine-consumed external integration manifest.
    path = PACKAGE / "config" / "cohan_shadow.yaml"
    assert path.exists(), "no pinned CoHAN shadow metadata exists"

    # When: its upstream and topic contracts are parsed.
    config = yaml.safe_load(path.read_text(encoding="utf-8"))

    # Then: the audited revision and advisory boundary are immutable.
    assert config["upstream"] == {
        "repository": "https://github.com/LAAS-HRI/CoHAN2.0.git",
        "commit": "bdfc5240b2128347bf6fe501d392d14768754a9d",
    }
    assert config["tracked_agents_topic"] == \
        "/human_aware_shadow/tracked_agents"
    assert config["velocity_sink_topic"] == \
        "/human_aware_shadow/velocity_proposal"
    assert config["planner_plugin"] == \
        "hateb_local_planner/HATebLocalPlannerROS"


def test_shadow_launch_sinks_every_hateb_velocity_command():
    # Given: the isolated launch graph.
    path = PACKAGE / "launch" / "cohan_shadow.launch"
    assert path.exists(), "no isolated CoHAN shadow launch exists"

    # When: launch nodes, params, and remaps are parsed.
    root = ET.fromstring(path.read_text(encoding="utf-8"))
    nodes = root.findall(".//node")
    remaps = root.findall(".//remap")
    params = root.findall(".//param")
    rosparams = root.findall(".//rosparam")

    # Then: HATEB is namespaced and its velocity reaches only the sink.
    assert any(
        node.get("pkg") == "static_livox_localization"
        and node.get("type") == "human_aware_shadow_node.py"
        for node in nodes
    )
    assert any(
        node.get("pkg") == "agent_path_prediction"
        and node.get("type") == "agent_path_predict"
        for node in nodes
    )
    assert any(
        node.get("pkg") == "move_base"
        and node.get("type") == "move_base"
        for node in nodes
    )
    assert any(
        param.get("name") == "base_local_planner"
        and param.get("value")
        == "hateb_local_planner/HATebLocalPlannerROS"
        for param in params
    )
    assert any(
        remap.get("from") == "cmd_vel"
        and remap.get("to")
        == "/human_aware_shadow/velocity_proposal"
        for remap in remaps
    )
    assert not any(
        remap.get("to") in {
            "/cmd_vel_raw",
            "/cmd_vel_gated",
            "/cmd_vel",
            "/wheel_cmd",
            "/mode_cmd",
        }
        for remap in remaps
    )
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "scripts/human_aware_shadow_node.py" in cmake
    assert "scripts/human_aware_shadow.py" in cmake
    common = [
        item for item in rosparams
        if item.get("file", "").endswith("/config/costmap_common.yaml")
    ]
    assert {item.get("ns") for item in common} == {
        "global_costmap",
        "local_costmap",
    }
