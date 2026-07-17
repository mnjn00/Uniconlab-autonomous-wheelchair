from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / "launch" / "localization.launch"


def test_fast_lio_icp_has_explicit_pose_diagnostic_and_enable_arguments():
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'name="fast_lio_icp_pose_topic" default="/fast_lio_icp/pose"' in text
    assert (
        'name="fast_lio_icp_diagnostic_topic" '
        'default="/fast_lio_icp/localization_diagnostics"'
    ) in text
    assert 'name="fast_lio_icp_enabled" default="false"' in text
    assert "One of: base_model, amcl, cartographer_noetic, fast_lio_icp" in text


def test_fast_lio_icp_arguments_are_bound_to_adapter_parameters():
    tree = ET.parse(LAUNCH)
    parameters = {
        node.attrib["name"]: node.attrib.get("value", "")
        for node in tree.findall(".//param")
        if "name" in node.attrib
    }

    assert parameters["sources/fast_lio_icp/enabled"] == "$(arg fast_lio_icp_enabled)"
    assert parameters["sources/fast_lio_icp/pose_topic"] == "$(arg fast_lio_icp_pose_topic)"
    assert (
        parameters["sources/fast_lio_icp/diagnostic_topic"]
        == "$(arg fast_lio_icp_diagnostic_topic)"
    )


def test_adapter_launch_does_not_start_driver_navigation_or_commands():
    text = LAUNCH.read_text(encoding="utf-8")

    assert "livox_ros_driver" not in text
    assert "fast_lio" not in text.replace("fast_lio_icp", "")
    assert "move_base" not in text
    assert "/cmd_vel" not in text

