from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]


def test_rviz_uses_fixed_map_and_only_bounded_visualization_topics():
    text = (ROOT / "config" / "moving_localization.rviz").read_text(
        encoding="utf-8"
    )

    assert "Fixed Frame: map" in text
    assert "/fast_lio_icp/map_preview" in text
    assert "/fast_lio_icp/pose" in text
    assert "/fast_lio_icp/path" in text
    assert "/fast_lio_icp/live_preview" in text
    assert "/fast_lio_icp/reference_marker" in text
    assert "/fast_lio_icp/state_marker" in text
    assert "Frame Rate: 10" in text
    assert "Decay Time: 0" in text


def test_map_preview_is_voxelized_latched_and_never_rewrites_map():
    text = (ROOT / "src" / "map_preview_publisher.cpp").read_text(
        encoding="utf-8"
    )

    assert "pcl::VoxelGrid" in text
    assert "map_preview_voxel_resolution" in text
    assert "advertise<sensor_msgs::PointCloud2>" in text
    assert "true" in text
    assert "savePCDFile" not in text


def test_reference_marker_is_visual_only_and_cannot_reset_localizer():
    marker = (ROOT / "scripts" / "reference_marker.py").read_text(
        encoding="utf-8"
    )
    localizer = (ROOT / "src" / "moving_icp_localizer.cpp").read_text(
        encoding="utf-8"
    )

    assert '"/clicked_point"' in marker
    assert '"/fast_lio_icp/reference_marker"' in marker
    assert "/clicked_point" not in localizer


def test_state_marker_has_tracking_degraded_and_lost_colors():
    text = (ROOT / "scripts" / "localization_state_marker.py").read_text(
        encoding="utf-8"
    )

    assert '"TRACKING": (0.0, 1.0, 0.0)' in text
    assert '"DEGRADED": (1.0, 1.0, 0.0)' in text
    assert '"LOST": (1.0, 0.0, 0.0)' in text


def test_moving_launch_starts_visual_helpers_without_rviz_by_default():
    launch_path = ROOT / "launch" / "moving_localization.launch"
    tree = ET.parse(launch_path)
    node_types = {node.attrib.get("type") for node in tree.findall(".//node")}
    launch_text = launch_path.read_text(encoding="utf-8")

    assert "map_preview_publisher" in node_types
    assert "reference_marker.py" in node_types
    assert "localization_state_marker.py" in node_types
    assert '<arg name="rviz" default="false"' in launch_text

