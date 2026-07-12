"""Static contracts for complete software-only release-candidate profiles."""

import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[3]
LAUNCH = ROOT / "src" / "wheelchair_bringup" / "launch"
SHA256 = re.compile(r"^[0-9a-f]{64}$")
LOCALIZATION_POLICY = (
    ROOT / "src" / "wheelchair_navigation" / "config" / "localization_confidence_sim.yaml"
)
ROUTE_SAFETY_CONFIG_SHA256 = "471bc90f8d52e341d2d6d287992fd26bf4224b776c57a057c82796bb0506eb60"
FORBIDDEN = ("ros2", "ament_", "GLIM", "glim", "/hardware/cmd_vel", "/motor/cmd")
WAYPOINTS = (
    ROOT / "data" / "hanyang_aegimun_loop" / "hanyang_aegimun_loop.waypoints.yaml"
)
NAVIGATION_MANIFEST = (
    ROOT / "src" / "wheelchair_navigation" / "config" / "hanyang_routes.yaml"
)


def _root(name):
    return ET.parse(str(LAUNCH / name)).getroot()


def _args(root):
    return {item.get("name"): item for item in root.findall("arg")}


def _include_files(root):
    return [item.get("file", "") for item in root.findall(".//include")]


def _node_types(root):
    return [(item.get("pkg"), item.get("type"), item.get("name")) for item in root.findall(".//node")]


def _param(root, name):
    return next(item for item in root.findall(".//param") if item.get("name") == name)


def _include(root, suffix):
    return next(item for item in root.findall(".//include") if item.get("file", "").endswith(suffix))


def _include_args(include):
    return {item.get("name"): item.get("value") for item in include.findall("arg")}


def test_sim_profile_has_complete_fail_closed_evidence_graph():
    root = _root("sim_bringup.launch")
    includes = _include_files(root)
    for suffix in (
        "wheelchair_gazebo)/launch/rc_sim.launch",
        "wheelchair_perception)/launch/perception.launch",
        "wheelchair_navigation)/launch/localization.launch",
        "wheelchair_navigation)/launch/route_manager.launch",
        "wheelchair_decision)/launch/decision.launch",
        "wheelchair_navigation)/launch/navigation.launch",
        "wheelchair_safety)/launch/safety.launch",
        "wheelchair_navigation)/launch/control_monitor.launch",
    ):
        assert sum(path.endswith(suffix) for path in includes) == 1

    nodes = _node_types(root)
    assert ("wheelchair_gazebo", "sim_evidence_bridge.py", "sim_evidence_bridge") in nodes
    assert ("wheelchair_route_safety", "route_safety.py", "wheelchair_route_safety") in nodes
    assert ("wheelchair_bringup", "incident_recorder.py", "incident_recorder") in nodes
    assert _param(root, "/use_sim_time").get("value") == "true"
    assert _param(root, "/hardware_motion_authorized").get("value") == "false"
    assert _param(root, "/passenger_operation_authorized").get("value") == "false"
    assert _param(root, "/wheelchair_bringup/startup_mode").get("value") == "DISARMED"

    localization = _include_args(_include(root, "wheelchair_navigation)/launch/localization.launch"))
    assert localization["enabled"] == "true"
    assert localization["source"] == "$(arg localization_source)"
    assert localization["base_model_enabled"] == "true"
    assert localization["base_model_pose_topic"] == "/base_model/localization_pose"


def test_sim_spawn_is_vetted_second_outbound_waypoint():
    root = _root("sim_bringup.launch")
    args = _args(root)
    route = json.loads(WAYPOINTS.read_text(encoding="utf-8"))
    first, second = route["outbound_route"]["waypoints"][:2]
    spawn = {
        "x_m": float(args["spawn_x"].get("default")),
        "y_m": float(args["spawn_y"].get("default")),
        "yaw_rad": float(args["spawn_yaw"].get("default")),
    }
    expected = {
        key: second[key]
        for key in ("x_m", "y_m", "yaw_rad")
    }
    assert args["spawn_x"].get("default") == "0.44629311316521386"
    assert args["spawn_y"].get("default") == "0.2644224486717838"
    assert args["spawn_yaw"].get("default") == "0.21401823367762954"

    assert args["spawn_z"].get("default") == "0.10"
    for key in expected:
        assert math.isclose(spawn[key], expected[key], abs_tol=1e-6)

    segment_x = second["x_m"] - first["x_m"]
    segment_y = second["y_m"] - first["y_m"]
    segment_length_squared = segment_x**2 + segment_y**2
    along_segment = (
        (spawn["x_m"] - first["x_m"]) * segment_x
        + (spawn["y_m"] - first["y_m"]) * segment_y
    ) / segment_length_squared
    projected_x = first["x_m"] + along_segment * segment_x
    projected_y = first["y_m"] + along_segment * segment_y
    assert 0.0 <= along_segment <= 1.0
    assert math.hypot(spawn["x_m"] - projected_x, spawn["y_m"] - projected_y) < 1e-6

    manifest = yaml.safe_load(NAVIGATION_MANIFEST.read_text(encoding="utf-8"))
    first_goal = manifest["outbound_route"]["waypoints"][0]
    assert math.hypot(
        spawn["x_m"] - first_goal["x_m"],
        spawn["y_m"] - first_goal["y_m"],
    ) <= first_goal["goal_tolerance_m"]

    simulation = _include_args(_include(root, "wheelchair_gazebo)/launch/rc_sim.launch"))
    assert simulation["spawn_x"] == "$(arg spawn_x)"
    assert simulation["spawn_y"] == "$(arg spawn_y)"
    assert simulation["spawn_z"] == "$(arg spawn_z)"
    assert simulation["spawn_yaw"] == "$(arg spawn_yaw)"

    driver = next(
        node for node in root.findall(".//node")
        if node.get("pkg") == "wheelchair_gazebo"
        and node.get("type") == "rc_scenario_driver.py"
    )
    driver_params = {
        item.get("name"): item.get("value")
        for item in driver.findall("param")
    }
    assert driver_params["initial_pose_x"] == "$(arg spawn_x)"
    assert driver_params["initial_pose_y"] == "$(arg spawn_y)"
    assert driver_params["initial_pose_yaw"] == "$(arg spawn_yaw)"

def test_sim_safety_binds_localization_policy_file_bytes():
    root = _root("sim_bringup.launch")
    safety = _include_args(_include(root, "wheelchair_safety)/launch/safety.launch"))
    expected = hashlib.sha256(LOCALIZATION_POLICY.read_bytes()).hexdigest()

    assert safety["localization_policy_file_sha256"] == expected
    assert _args(root)["localization_policy_sha256"].get("default") == (
        "5d84ea824c98a53639a480ed162a62f015600ca0a0460df7186d5839303d52e8"
    )


def test_replay_consumes_canonical_topics_and_blocks_localization_candidates():
    root = _root("replay_bringup.launch")
    perception = _include_args(_include(root, "wheelchair_perception)/launch/perception.launch"))
    assert perception == {
        "source_profile": "replay",
        "replay_cloud_topic": "/sensors/lidar/points",
        "replay_imu_topic": "/sensors/imu/data",
    }
    localization = _include_args(_include(root, "wheelchair_navigation)/launch/localization.launch"))
    assert localization["enabled"] == "false"
    assert localization["source"] == ""
    for source in ("base_model_enabled", "amcl_enabled", "cartographer_noetic_enabled"):
        assert localization[source] == "false"
    assert _param(root, "/wheelchair_bringup/localization_candidate_policy").get("value") == "BLOCKED_UNLESS_QUALIFIED"
    safety = _include_args(_include(root, "wheelchair_safety)/launch/safety.launch"))
    assert safety["input_cmd_topic"] == "/cmd_vel_nav"
    assert safety["output_cmd_topic"] == "/shadow/cmd_vel_safe"
    assert ("wheelchair_bringup", "incident_recorder.py", "incident_recorder") in _node_types(root)


def test_profiles_bind_every_policy_and_identity_to_sha256():
    required_paths = ("route_policy", "route_safety_policy", "localization_policy", "collision_policy", "slope_policy")
    for name in ("sim_bringup.launch", "replay_bringup.launch"):
        root = _root(name)
        args = _args(root)
        assert SHA256.fullmatch(args["map_sha256"].get("default", ""))
        assert SHA256.fullmatch(args["route_sha256"].get("default", ""))
        for policy in required_paths:
            assert "$(find " in args[policy].get("default", "")
            assert SHA256.fullmatch(args[policy + "_sha256"].get("default", ""))
        assert args["route_safety_policy_sha256"].get("default") == ROUTE_SAFETY_CONFIG_SHA256
        route_safety_node = next(
            node for node in root.findall(".//node")
            if node.get("pkg") == "wheelchair_route_safety"
            and node.get("type") == "route_safety.py"
        )
        route_safety_params = {
            item.get("name"): item.get("value")
            for item in route_safety_node.findall("param")
        }
        assert route_safety_params == {
            "config_path": "$(arg route_safety_policy)",
            "expected_config_sha256": "$(arg route_safety_policy_sha256)",
        }
        assert _param(root, "/hardware_motion_authorized").get("value") == "false"
        assert _param(root, "/passenger_operation_authorized").get("value") == "false"
        params = {item.get("name") for item in root.findall(".//param")}
        for policy in required_paths:
            assert "/wheelchair_bringup/policies/%s_sha256" % policy.replace("_policy", "") in params


def test_authority_publishers_are_singular_and_replay_has_no_sink():
    sim = _root("sim_bringup.launch")
    replay = _root("replay_bringup.launch")
    gazebo = ET.parse(str(ROOT / "src" / "wheelchair_gazebo" / "launch" / "rc_sim.launch")).getroot()
    navigation = ET.parse(str(ROOT / "src" / "wheelchair_navigation" / "launch" / "navigation.launch")).getroot()
    safety = ET.parse(str(ROOT / "src" / "wheelchair_safety" / "launch" / "safety.launch")).getroot()

    assert sum(node.get("type") == "move_base" for node in navigation.findall(".//node")) == 1
    assert sum(node.get("type") == "safety_gate.py" for node in safety.findall(".//node")) == 1
    adapters = [
        node for node in gazebo.findall(".//node")
        if node.get("type") == "simulation_controller_adapter.py"
    ]
    assert len(adapters) == 1
    assert not any(node.get("pkg") == "topic_tools" for node in gazebo.findall(".//node"))
    assert sum(path.endswith("navigation.launch") for path in _include_files(sim)) == 1
    assert sum(path.endswith("safety.launch") for path in _include_files(sim)) == 1
    assert sum(path.endswith("navigation.launch") for path in _include_files(replay)) == 1
    assert sum(path.endswith("safety.launch") for path in _include_files(replay)) == 1
    assert not any(node.get("pkg") in ("topic_tools", "wheelchair_hardware") for node in replay.findall(".//node"))


def test_generic_rc_allow_lists_profiles_and_rejects_hardware_enabled():
    root = _root("rc_bringup.launch")
    profile = _args(root)["profile"]
    assert profile.get("default") == "sim"
    assert "sim, replay, hardware_shadow" in profile.get("doc", "")
    selector = _param(root, "/wheelchair_bringup/profile").get("value", "")
    assert "'sim':'sim'" in selector
    assert "'replay':'replay'" in selector
    assert "'hardware_shadow':'hardware_shadow'" in selector
    assert "hardware_enabled" not in selector
    assert sorted(path.rsplit("/", 1)[-1] for path in _include_files(root)) == [
        "hardware_shadow.launch",
        "replay_bringup.launch",
        "sim_bringup.launch",
    ]


def test_profiles_contain_no_ros2_glim_or_real_motor_surface():
    for name in ("sim_bringup.launch", "replay_bringup.launch", "rc_bringup.launch"):
        text = (LAUNCH / name).read_text(encoding="utf-8")
        for token in FORBIDDEN:
            assert token not in text
