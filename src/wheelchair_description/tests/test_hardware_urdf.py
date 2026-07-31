"""Contracts for the measured-hardware description kept separate from simulation."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT / "src" / "wheelchair_description"
HARDWARE_URDF = DESCRIPTION / "urdf" / "wheelchair_hardware.urdf.xacro"
HARDWARE_LAUNCH = DESCRIPTION / "launch" / "hardware_description.launch"
SIMULATION_URDF = DESCRIPTION / "urdf" / "wheelchair.urdf.xacro"


def _joint(root: ET.Element, name: str) -> ET.Element:
    joint = root.find("./joint[@name='{}']".format(name))
    assert joint is not None
    return joint


def _joint_transform(root: ET.Element, name: str) -> tuple[str, str, str, str]:
    joint = _joint(root, name)
    parent = joint.find("parent")
    child = joint.find("child")
    origin = joint.find("origin")
    assert parent is not None and child is not None and origin is not None
    return (
        parent.attrib["link"],
        child.attrib["link"],
        origin.attrib["xyz"],
        origin.attrib["rpy"],
    )


def test_hardware_copy_has_distinct_identity_and_no_simulation_plugins() -> None:
    # Given: the hardware model copied from the active NUC base_model package.
    # When: its XML and declared links are inspected.
    root = ET.parse(HARDWARE_URDF).getroot()
    links = {node.attrib["name"] for node in root.findall("link")}
    text = HARDWARE_URDF.read_text(encoding="utf-8")

    # Then: it is an explicitly hardware-only model with the required frames.
    assert root.attrib["name"] == "wheelchair_hardware"
    assert {
        "base_footprint",
        "base_link",
        "lidar_link",
        "imu_link",
        "body",
        "link_left_wheel",
        "link_right_wheel",
        "link_support_left_wheel",
        "link_support_right_wheel",
    }.issubset(links)
    assert "<gazebo" not in text
    assert "/simulation/" not in text
    assert "libgazebo" not in text


def test_hardware_frames_use_the_drive_axle_as_base_footprint() -> None:
    # Given: REP-103 coordinates and the NUC model's 0.26 m axle setback.
    root = ET.parse(HARDWARE_URDF).getroot()

    # When/Then: base_footprint is the axle midpoint and left is positive Y.
    assert _joint_transform(root, "base_footprint_joint") == (
        "base_footprint",
        "base_link",
        "0.260 0 0",
        "0 0 0",
    )
    assert _joint_transform(root, "joint_left_wheel") == (
        "base_link",
        "link_left_wheel",
        "-0.260 0.220 0.300",
        "0 0 0",
    )
    assert _joint_transform(root, "joint_right_wheel") == (
        "base_link",
        "link_right_wheel",
        "-0.260 -0.220 0.300",
        "0 0 0",
    )


def test_hardware_sensor_frames_encode_the_measured_left_armrest_mount() -> None:
    # Given: the 7/27 spin-fit body offset and the built-in IMU extrinsic.
    root = ET.parse(HARDWARE_URDF).getroot()

    # When/Then: the chair-to-sensor transforms preserve both calibrations.
    assert _joint_transform(root, "lidar_joint") == (
        "base_link",
        "lidar_link",
        "0.246 0.14971 0.300",
        "0 0 0",
    )
    assert _joint_transform(root, "imu_joint") == (
        "base_link",
        "imu_link",
        "0.257 0.173 0.25588",
        "0 0 0",
    )
    assert _joint_transform(root, "fast_lio_body_joint") == (
        "imu_link",
        "body",
        "0 0 0",
        "0 0 0",
    )


def test_hardware_frame_graph_is_one_connected_tree() -> None:
    # Given: every link and joint in the expanded hardware description source.
    root = ET.parse(HARDWARE_URDF).getroot()
    links = {node.attrib["name"] for node in root.findall("link")}
    parents = {}

    # When: child ownership and ancestry are reconstructed from all joints.
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        assert parent is not None and child is not None
        parent_name = parent.attrib["link"]
        child_name = child.attrib["link"]
        assert parent_name in links and child_name in links
        assert child_name not in parents
        parents[child_name] = parent_name

    # Then: base_footprint is the sole root and every ancestry chain reaches it.
    assert links - set(parents) == {"base_footprint"}
    for link in links - {"base_footprint"}:
        visited = set()
        current = link
        while current in parents:
            assert current not in visited
            visited.add(current)
            current = parents[current]
        assert current == "base_footprint"


def test_hardware_and_simulation_have_separate_launch_surfaces() -> None:
    # Given: field and Gazebo descriptions have different calibration authority.
    hardware_root = ET.parse(HARDWARE_LAUNCH).getroot()
    hardware_text = HARDWARE_LAUNCH.read_text(encoding="utf-8")
    simulation_launch = ET.parse(DESCRIPTION / "launch" / "display.launch").getroot()
    simulation_text = SIMULATION_URDF.read_text(encoding="utf-8")

    # When/Then: neither launch surface exposes a model override across profiles.
    assert hardware_root.tag == "launch"
    assert hardware_root.find("./arg[@name='model']") is None
    assert simulation_launch.find("./arg[@name='model']") is None
    hardware_description = hardware_root.find("./param[@name='robot_description']")
    simulation_description = simulation_launch.find("./param[@name='robot_description']")
    assert hardware_description is not None and simulation_description is not None
    assert hardware_description.attrib["command"] == (
        "$(find xacro)/xacro "
        "$(find wheelchair_description)/urdf/wheelchair_hardware.urdf.xacro"
    )
    assert simulation_description.attrib["command"] == (
        "$(find xacro)/xacro "
        "$(find wheelchair_description)/urdf/wheelchair.urdf.xacro"
    )
    assert "robot_state_publisher" in hardware_text
    assert "/simulation/sensors/" in simulation_text
    assert "/simulation/sensors/" not in HARDWARE_URDF.read_text(encoding="utf-8")


def test_hardware_launch_dependencies_are_declared() -> None:
    # Given: every executable named by the hardware description launch.
    package_root = ET.parse(DESCRIPTION / "package.xml").getroot()

    # When/Then: a deployed package declares all three ROS runtime providers.
    dependencies = {node.text for node in package_root.findall("exec_depend")}
    assert {"xacro", "robot_state_publisher", "joint_state_publisher"}.issubset(
        dependencies
    )
