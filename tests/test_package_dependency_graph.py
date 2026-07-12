"""Package dependency checks for the explicit release-candidate launch graph."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
FIND_PACKAGE = re.compile(r"\$\(find ([A-Za-z0-9_]+)\)")
DEPENDENCY_TAGS = {"depend", "build_depend", "build_export_depend", "exec_depend"}


def _manifest(package):
    return ET.parse(str(SRC / package / "package.xml")).getroot()


def _dependencies(package):
    root = _manifest(package)
    return {item.text.strip() for item in root if item.tag in DEPENDENCY_TAGS and item.text}


def _workspace_packages():
    return {path.parent.name for path in SRC.glob("*/package.xml")}


def _launch_references(path):
    root = ET.parse(str(path)).getroot()
    packages = set(FIND_PACKAGE.findall(path.read_text(encoding="utf-8")))
    packages.update(node.get("pkg") for node in root.findall(".//node") if node.get("pkg"))
    return packages


def test_bringup_declares_every_profile_and_incident_runtime_dependency():
    dependencies = _dependencies("wheelchair_bringup")
    assert {
        "wheelchair_decision",
        "wheelchair_gazebo",
        "wheelchair_hardware",
        "wheelchair_interfaces",
        "wheelchair_navigation",
        "wheelchair_perception",
        "wheelchair_route_safety",
        "wheelchair_safety",
        "diagnostic_msgs",
        "geometry_msgs",
        "nav_msgs",
        "rospy",
        "sensor_msgs",
        "topic_tools",
        "python3-yaml",
    } <= dependencies


def test_launch_workspace_references_are_declared_by_owning_package():
    workspace = _workspace_packages()
    for launch in SRC.glob("*/launch/*.launch"):
        owner = launch.parents[1].name
        references = (_launch_references(launch) & workspace) - {owner}
        missing = references - _dependencies(owner)
        assert not missing, "%s is missing package dependencies: %s" % (
            launch.relative_to(ROOT), sorted(missing)
        )


def test_profile_dependency_graph_has_no_self_edges_or_cycles():
    workspace = _workspace_packages()
    graph = {
        package: (_dependencies(package) & workspace) - {package}
        for package in workspace
    }
    assert all(package not in dependencies for package, dependencies in graph.items())

    visiting = set()
    visited = set()

    def visit(package, chain):
        if package in visiting:
            raise AssertionError("workspace package dependency cycle: " + " -> ".join(chain + [package]))
        if package in visited:
            return
        visiting.add(package)
        for dependency in sorted(graph[package]):
            visit(dependency, chain + [package])
        visiting.remove(package)
        visited.add(package)

    for package in sorted(graph):
        visit(package, [])


def test_profile_launches_use_ros1_only_and_reference_existing_workspace_packages():
    workspace = _workspace_packages()
    for name in ("rc_bringup.launch", "sim_bringup.launch", "replay_bringup.launch"):
        path = SRC / "wheelchair_bringup" / "launch" / name
        text = path.read_text(encoding="utf-8")
        assert "ros2" not in text.lower()
        assert "ament_" not in text
        for package in FIND_PACKAGE.findall(text):
            if package.startswith("wheelchair_"):
                assert package in workspace
