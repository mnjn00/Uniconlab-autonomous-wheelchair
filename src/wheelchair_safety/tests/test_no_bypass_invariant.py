from dataclasses import replace
import importlib.util
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
TOPOLOGY_GUARD = ROOT / "src" / "wheelchair_safety" / "scripts" / "topology_guard.py"
spec = importlib.util.spec_from_file_location("topology_guard_no_bypass", TOPOLOGY_GUARD)
topology_guard = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = topology_guard
spec.loader.exec_module(topology_guard)


def _authority_evidence_paths():
    paths = set(ROOT.glob("src/**/launch/*.launch"))
    paths.update(ROOT.glob("src/**/config/*.yaml"))
    paths.update(ROOT.glob("src/**/config/*.yml"))
    paths.update(ROOT.glob("src/**/config/*.json"))
    paths.update(ROOT.glob("src/**/package.xml"))
    paths.update(path for path in ROOT.glob("src/**/*") if path.is_file() and "profile" in path.name.lower())
    return tuple(sorted(paths))


# Included in assertion output so a release artifact identifies exactly what was inspected.
STATIC_AUTHORITY_EVIDENCE = _authority_evidence_paths()
MOTOR_TOPICS = ("/wheelchair_base_controller/cmd_vel", "/base_controller/cmd_vel", "/motor_command")
UNSAFE_COMMAND_TOPICS = ("/cmd_vel_nav", "/cmd_vel_raw", "/cmd_vel_shadow")
SIMULATION_CONTROLLERS = ROOT / "src" / "wheelchair_gazebo" / "config" / "controllers.yaml"


def test_authority_evidence_path_list_covers_every_launch_profile_and_package_file():
    expected = set(ROOT.glob("src/**/launch/*.launch"))
    expected.update(ROOT.glob("src/**/package.xml"))
    expected.update(path for path in ROOT.glob("src/**/*") if path.is_file() and "profile" in path.name.lower())
    assert expected.issubset(STATIC_AUTHORITY_EVIDENCE), [str(path.relative_to(ROOT)) for path in expected]
    assert STATIC_AUTHORITY_EVIDENCE, "no launch/profile/package evidence was discovered"


def test_no_static_alternate_command_or_geofence_authority_path():
    offenders = []
    for path in STATIC_AUTHORITY_EVIDENCE:
        text = path.read_text()
        lowered = text.lower()
        relative = path.relative_to(ROOT)

        for index, line in enumerate(text.splitlines(), start=1):
            if any(motor in line for motor in MOTOR_TOPICS) and any(source in line for source in UNSAFE_COMMAND_TOPICS):
                offenders.append(f"{relative}:{index}: unsafe command reaches motor topic: {line.strip()}")
            if "/cmd_vel_safe" in line and "/cmd_vel_nav" in line and "safety_gate" not in lowered:
                offenders.append(f"{relative}:{index}: command topics coupled outside safety gate: {line.strip()}")

        if re.search(r"<(?:node|plugin)\b[^>]*(?:relay|mux|velocity_smoother)[^>]*>.*?/cmd_vel_nav", text,
                     flags=re.IGNORECASE | re.DOTALL):
            offenders.append(f"{relative}: relay/mux/plugin consumes /cmd_vel_nav")
        if ("wheelchair_navigation" in path.parts or "wheelchair_decision" in path.parts) and \
                ("/safety/geofence" in text or "/route_safety/geofence_status" in text):
            offenders.append(f"{relative}: navigation/decision package claims geofence publication")

    assert offenders == [], "inspected evidence:\n%s\nviolations:\n%s" % (
        "\n".join(str(path.relative_to(ROOT)) for path in STATIC_AUTHORITY_EVIDENCE),
        "\n".join(offenders),
    )


def test_expected_nav_to_safety_to_base_command_chain_is_present():
    nav_launch = ROOT / "src" / "wheelchair_navigation" / "launch" / "navigation.launch"
    safety_launch = ROOT / "src" / "wheelchair_safety" / "launch" / "safety.launch"
    sim_bringup = ROOT / "src" / "wheelchair_bringup" / "launch" / "sim_bringup.launch"
    sim_adapter = ROOT / "src" / "wheelchair_gazebo" / "scripts" / "simulation_controller_adapter.py"

    nav_root = ET.parse(nav_launch).getroot()
    move_base = [node for node in nav_root.findall("node") if node.attrib.get("pkg") == "move_base"]
    assert len(move_base) == 1
    remaps = {(item.attrib.get("from"), item.attrib.get("to")) for item in move_base[0].findall("remap")}
    assert ("cmd_vel", "$(arg cmd_vel_nav_topic)") in remaps
    assert 'default="/cmd_vel_nav"' in nav_launch.read_text()

    safety_text = safety_launch.read_text()
    assert 'default="/cmd_vel_nav"' in safety_text
    assert 'default="/cmd_vel_safe"' in safety_text

    sim_text = sim_bringup.read_text()
    adapter_text = sim_adapter.read_text()
    assert "wheelchair_gazebo)/launch/rc_sim.launch" in sim_text
    assert 'SOURCE_TOPIC = "/cmd_vel_safe"' in adapter_text
    assert 'SINK_TOPIC = "/wheelchair_base_controller/cmd_vel"' in adapter_text
    for unsafe_topic in UNSAFE_COMMAND_TOPICS:
        assert f'SOURCE_TOPIC = "{unsafe_topic}"' not in adapter_text

def test_simulation_sink_rejects_multiple_command_publishers():
    controllers_text = SIMULATION_CONTROLLERS.read_text()
    setting_values = re.findall(
        r"(?im)^  allow_multiple_cmd_vel_publishers:\s*([^\s#]+)\s*(?:#.*)?$",
        controllers_text,
    )
    assert setting_values == ["false"]
    assert not re.search(
        r"(?im)^  allow_multiple_cmd_vel_publishers:\s*(?:true|1|yes|on)\s*(?:#.*)?$",
        controllers_text,
    )


def _minimal_valid_graph():
    publishers = {
        topic: (authority.publishers[0],)
        for topic, authority in topology_guard.DEFAULT_AUTHORITIES.items()
    }
    subscribers = {
        topic: authority.subscribers + (
            (authority.subscriber_alternatives[0],) if authority.subscriber_alternatives else ()
        )
        for topic, authority in topology_guard.DEFAULT_AUTHORITIES.items()
    }
    observations = {
        topic: topology_guard.TopicObservation(1, True, 4.9, 0.3)
        for topic in topology_guard.DEFAULT_TIMED_TOPICS
    }
    return topology_guard.GraphSnapshot(
        publishers=publishers,
        subscribers=subscribers,
        transforms={("map", "odom"): (topology_guard.TransformObservation(
            "selected_localization_adapter", 4.9, 4.9, False, 0.1,
            (0.0, 0.0, 0.0), 0.0),)},
        observations=observations,
        captured_at_s=5.0,
        master_evidence_complete=True,
        tf_evidence_complete=True,
        timing_evidence_complete=True,
    )


def test_synthetic_forbidden_edge_mutations_all_fail_closed_without_ros():
    baseline = _minimal_valid_graph()
    auditor = topology_guard.TopologyAuditor()
    assert auditor.audit(baseline).ok

    mutations = []
    publishers = dict(baseline.publishers)
    publishers["/cmd_vel_safe"] += ("decision",)
    mutations.append(replace(baseline, publishers=publishers))

    publishers = dict(baseline.publishers)
    subscribers = dict(baseline.subscribers)
    publishers["/base_controller/cmd_vel"] = ("move_base",)
    subscribers["/base_controller/cmd_vel"] = ("motor_plugin",)
    mutations.append(replace(baseline, publishers=publishers, subscribers=subscribers))

    publishers = dict(baseline.publishers)
    publishers["/safety/geofence"] = ("navigation_route_manager",)
    mutations.append(replace(baseline, publishers=publishers))

    mutations.append(replace(
        baseline,
        transforms={("map", "odom"): ("amcl", "localization_adapter")},
    ))
    mutations.append(replace(baseline, master_evidence_complete=False))

    results = [auditor.audit(mutation) for mutation in mutations]
    assert all(not result.ok for result in results)
    assert all(result.reason_mask for result in results)
