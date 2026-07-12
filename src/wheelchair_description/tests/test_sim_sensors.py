from pathlib import Path
import math
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
DESC = ROOT / "src" / "wheelchair_description"
URDF = DESC / "urdf" / "wheelchair.urdf.xacro"
DIMENSIONS = DESC / "urdf" / "dimensions.xacro"
XACRO_NS = "{http://www.ros.org/wiki/xacro}"


def _properties():
    root = ET.parse(DIMENSIONS).getroot()
    return {
        node.attrib["name"]: float(node.attrib["value"])
        for node in root.iter(f"{XACRO_NS}property")
    }


def _sensor(reference, sensor_type):
    root = ET.parse(URDF).getroot()
    gazebo = next(
        node for node in root.findall("gazebo")
        if node.attrib.get("reference") == reference
    )
    sensor = gazebo.find("sensor")
    assert sensor is not None
    assert sensor.attrib["type"] == sensor_type
    return sensor


def test_block_laser_is_a_multilayer_pointcloud2_source():
    sensor = _sensor("lidar_link", "ray")
    plugin = sensor.find("plugin")
    horizontal = sensor.find("./ray/scan/horizontal")
    vertical = sensor.find("./ray/scan/vertical")
    props = _properties()

    assert sensor.attrib["name"] == "simulation_mid360_like_block_laser"
    assert plugin is not None
    assert plugin.attrib["filename"] == "libgazebo_ros_block_laser.so"
    assert plugin.findtext("topicName") == "/simulation/sensors/lidar/raw"
    assert plugin.findtext("frameName") == "lidar_link"
    assert horizontal is not None and vertical is not None
    horizontal_samples = int(horizontal.findtext("samples"))
    vertical_samples = int(vertical.findtext("samples"))
    assert horizontal_samples == 144
    assert vertical_samples == 4
    assert horizontal_samples % 36 == 0
    assert vertical_samples % 2 == 0
    assert horizontal_samples * vertical_samples <= 576
    assert math.isclose(
        float(horizontal.findtext("max_angle"))
        - float(horizontal.findtext("min_angle")),
        2.0 * math.pi,
        abs_tol=1e-9,
    )
    assert math.isclose(float(vertical.findtext("min_angle")), -0.10, abs_tol=1e-12)
    assert math.isclose(float(vertical.findtext("max_angle")), -0.02, abs_tol=1e-12)
    assert float(vertical.findtext("max_angle")) < 0.0
    horizontal_min = float(horizontal.findtext("min_angle"))
    horizontal_max = float(horizontal.findtext("max_angle"))
    vertical_min = float(vertical.findtext("min_angle"))
    vertical_max = float(vertical.findtext("max_angle"))
    horizontal_angles = [
        horizontal_min + index * (horizontal_max - horizontal_min) / (horizontal_samples - 1)
        for index in range(horizontal_samples)
    ]
    vertical_angles = [
        vertical_min + index * (vertical_max - vertical_min) / (vertical_samples - 1)
        for index in range(vertical_samples)
    ]
    forward_bin_ray_counts = [0] * 36
    for azimuth in horizontal_angles:
        if abs(azimuth) <= math.pi / 2.0:
            azimuth_index = min(
                35,
                max(0, int((azimuth + math.pi / 2.0) / math.pi * 36)),
            )
            forward_bin_ray_counts[azimuth_index] += 1
    assert min(forward_bin_ray_counts) >= 2
    assert len(vertical_angles) == 4
    assert len(set(vertical_angles)) == 4
    covered_cells = {
        (
            min(35, max(0, int((azimuth + math.pi / 2.0) / math.pi * 36))),
            min(1, max(0, int((elevation + 0.105) / 0.090 * 2))),
        )
        for azimuth in horizontal_angles
        for elevation in vertical_angles
        if abs(azimuth) <= math.pi / 2.0 and -0.105 <= elevation <= -0.015
    }
    assert covered_cells == {
        (azimuth_index, elevation_index)
        for azimuth_index in range(36)
        for elevation_index in range(2)
    }
    assert props["simulation_lidar_rate"] == 10.0
    assert props["simulation_lidar_min_range"] == 0.10
    assert props["simulation_lidar_max_range"] == 40.0


def test_simulation_imu_uses_canonical_topic_and_rep103_frame():
    root = ET.parse(URDF).getroot()
    sensor = _sensor("imu_link", "imu")
    plugin = sensor.find("plugin")
    imu_joint = root.find("./joint[@name='imu_joint']")
    props = _properties()

    assert sensor.attrib["name"] == "simulation_rep103_imu"
    assert plugin is not None
    assert plugin.attrib["filename"] == "libgazebo_ros_imu_sensor.so"
    assert plugin.findtext("topicName") == "/simulation/sensors/imu/raw"
    assert plugin.findtext("frameName") == "imu_link"
    assert plugin.findtext("updateRateHZ") == "${simulation_imu_rate}"
    assert plugin.findtext("gaussianNoise") == "${simulation_imu_noise}"
    assert props["simulation_imu_rate"] == 200.0
    assert props["simulation_imu_noise"] > 0.0
    assert imu_joint is not None and imu_joint.attrib["type"] == "fixed"
    assert imu_joint.find("origin").attrib["rpy"] == "0 0 0"


def test_sensor_sources_are_explicitly_simulation_only():
    text = URDF.read_text()
    dimensions = DIMENSIONS.read_text()
    normalized = " ".join(text.split())

    assert "/simulation/sensors/lidar/raw" in text
    assert "/simulation/sensors/imu/raw" in text
    assert "canonical perception topic" in text
    assert "REP-103" in text
    assert "Simulation-only" in text
    assert "simulation evidence" in text.lower()
    assert "not measured hardware calibration" in text
    assert "does not reproduce Livox scan pattern, timing" in normalized
    assert "Explicitly non-physical simulation-only ray grid" in text
    assert "two nominal rays per each of the 36 forward azimuth bins" in normalized
    assert "at all four vertical layers" in normalized
    assert "reducing bin-boundary jitter" in normalized
    assert "this is not Livox fidelity" in normalized
    assert "not a hardware IMU claim" in text
    assert "not measured calibration" in dimensions
    assert "physical sensor fidelity" in dimensions
    assert "<topicName>/scan</topicName>" not in text
    assert "lidar_3d_placeholder" not in text
    assert "libgazebo_ros_laser.so" not in text

def test_chassis_contact_sensor_proves_collision_metric_without_command_authority():
    sensor = _sensor("base_link", "contact")
    plugin = sensor.find("plugin")
    root = ET.parse(URDF).getroot()
    collision = root.find("./link[@name='base_link']/collision")
    assert sensor.attrib["name"] == "simulation_wheelchair_contact"
    assert sensor.findtext("contact/collision") == "base_link_collision"
    assert plugin is not None
    assert plugin.attrib["filename"] == "libgazebo_ros_bumper.so"
    assert plugin.findtext("bumperTopicName") == "/simulation/contacts"
    assert plugin.findtext("frameName") == "base_footprint"
    assert collision is not None and collision.attrib["name"] == "base_link_collision"


def test_package_declares_classic_gazebo_sensor_runtime_dependencies():
    root = ET.parse(DESC / "package.xml").getroot()
    dependencies = {node.text for node in root.findall("exec_depend")}
    assert {"gazebo_plugins", "gazebo_ros", "sensor_msgs"}.issubset(dependencies)
