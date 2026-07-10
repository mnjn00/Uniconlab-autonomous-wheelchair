from pathlib import Path
import math
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
DESC = ROOT / "src" / "wheelchair_description"
XACRO_NS = "{http://www.ros.org/wiki/xacro}"


def _numeric_props():
    tree = ET.parse(DESC / "urdf" / "dimensions.xacro")
    props = {}
    for node in tree.getroot().iter(f"{XACRO_NS}property"):
        value = node.attrib.get("value", "")
        try:
            props[node.attrib["name"]] = float(value)
        except ValueError:
            pass
    return props


def test_image_dimension_constants_are_transcribed_exactly():
    props = _numeric_props()
    expected = {
        "wheelchair_length": 0.970,
        "wheelchair_height": 0.910,
        "wheelchair_width": 0.600,
        "seat_width": 0.445,
        "seat_height": 0.475,
        "front_wheel_radius": 0.0635,
        "rear_wheel_radius": 0.254,
        "wheelchair_mass": 13.5,
        "folded_length": 0.730,
        "folded_width": 0.300,
    }
    for key, value in expected.items():
        assert key in props
        assert math.isclose(props[key], value, rel_tol=0.0, abs_tol=1e-9)


def test_derived_footprint_and_wheel_extents_are_consistent():
    p = _numeric_props()
    half_length = p["wheelchair_length"] / 2.0
    half_width = p["wheelchair_width"] / 2.0
    footprint = [
        (-half_length, -half_width),
        (-half_length, half_width),
        (half_length, half_width),
        (half_length, -half_width),
    ]
    assert min(x for x, _ in footprint) == -0.485
    assert max(x for x, _ in footprint) == 0.485
    assert min(y for _, y in footprint) == -0.300
    assert max(y for _, y in footprint) == 0.300

    rear_min_x = p["rear_wheel_x"] - p["rear_wheel_radius"]
    rear_max_x = p["rear_wheel_x"] + p["rear_wheel_radius"]
    front_max_x = p["front_caster_x"] + p["front_wheel_radius"]
    rear_total_width = p["rear_wheel_track"] + p["rear_wheel_thickness"]
    front_total_width = p["front_caster_track"] + p["front_wheel_thickness"]

    assert rear_min_x >= -half_length - 1e-9
    assert rear_max_x <= half_length + 1e-9
    assert front_max_x <= half_length + 1e-9
    assert rear_total_width <= p["wheelchair_width"] + 1e-9
    assert front_total_width <= p["wheelchair_width"] + 1e-9
    assert p["front_wheel_radius"] < p["rear_wheel_radius"]
    assert p["seat_width"] < p["wheelchair_width"]
    assert p["folded_width"] < p["wheelchair_width"]


def test_urdf_xacro_xml_and_required_links_are_present():
    urdf = DESC / "urdf" / "wheelchair.urdf.xacro"
    tree = ET.parse(urdf)
    root = tree.getroot()
    assert root.attrib["name"] == "wheelchair"

    links = {node.attrib["name"] for node in root.findall("link")}
    joints = {node.attrib["name"] for node in root.findall("joint")}
    required_links = {
        "base_footprint",
        "base_link",
        "seat_link",
        "backrest_link",
        "nuc_link",
        "lidar_link",
        "rear_left_wheel_link",
        "rear_right_wheel_link",
        "front_left_caster_link",
        "front_right_caster_link",
    }
    required_joints = {
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
        "front_left_caster_joint",
        "front_right_caster_joint",
        "lidar_joint",
    }
    assert required_links.issubset(links)
    assert required_joints.issubset(joints)

    text = urdf.read_text()
    assert "gazebo_ros_control" in text
    assert "hardware_interface/VelocityJointInterface" in text
    assert re.search(r"<axis\s+xyz=\"0 1 0\"", text)


def test_description_sources_do_not_reference_ros2_or_gazebo_sim_stack():
    forbidden = ["ros2_control", "gz_ros2_control", "ros_gz", "Nav2", "launch_testing", "Gazebo Harmonic"]
    for path in (DESC / "urdf").glob("*.xacro"):
        text = path.read_text()
        for token in forbidden:
            assert token not in text, f"{token} found in {path}"
