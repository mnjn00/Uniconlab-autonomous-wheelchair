from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "config" / "moving_localization.yaml"
RVIZ = ROOT / "config" / "moving_localization.rviz"
LAUNCH = ROOT / "launch" / "moving_localization.launch"


def test_rviz_uses_only_bounded_live_preview():
    rviz = RVIZ.read_text(encoding="utf-8")

    assert "Frame Rate: 10" in rviz
    assert "Topic: /fast_lio_icp/live_preview" in rviz
    assert "Decay Time: 0" in rviz
    current_cloud = rviz.split("Name: Current Livox Cloud", 1)[1].split(
        "Name: Estimated Wheelchair Pose", 1
    )[0]
    assert "/cloud_registered_body" not in current_cloud


def test_localizer_keeps_original_cloud_and_preview_is_bounded():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    assert config["cloud_topic"] == "/cloud_registered_body"
    assert config["preview_input_topic"] == "/cloud_registered_body"
    assert config["preview_output_topic"] == "/fast_lio_icp/live_preview"
    assert config["preview_voxel_resolution"] == 0.30
    assert config["preview_max_rate_hz"] == 5.0


def test_visualization_launch_starts_bounded_preview_with_queue_safe_node():
    tree = ET.parse(LAUNCH)
    nodes = tree.findall(".//node")
    preview_nodes = [
        node for node in nodes if node.attrib.get("type") == "bounded_cloud_preview_node"
    ]

    assert len(preview_nodes) == 1
    preview = preview_nodes[0]
    rosparam = preview.find("rosparam")
    assert rosparam is not None and rosparam.attrib.get("command") == "load"
