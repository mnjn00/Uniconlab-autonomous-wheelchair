from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


ROOT = Path(__file__).parents[1]


def test_moving_launch_is_isolated_from_static_localizer_and_motion_control():
    launch_path = ROOT / "launch" / "moving_localization.launch"
    launch_text = launch_path.read_text(encoding="utf-8")
    tree = ET.parse(launch_path)
    nodes = tree.findall(".//node")

    assert any(node.attrib.get("type") == "moving_icp_localizer" for node in nodes)
    assert "static_localization.launch" not in launch_text
    assert "static_icp_localizer" not in launch_text
    assert "/cmd_vel" not in launch_text
    assert "move_base" not in launch_text
    assert "hardware_adapter" not in launch_text


def test_moving_config_declares_fixed_map_and_tracking_contract():
    config_text = (ROOT / "config" / "moving_localization.yaml").read_text(
        encoding="utf-8"
    )

    for key in (
        "map_path:",
        "map_sha256:",
        "cloud_topic:",
        "odom_topic:",
        "seed_topic:",
        "correction_period_s:",
        "rolling_window_s:",
        "min_inlier_ratio:",
    ):
        assert key in config_text

    assert "livox_raw_20260707_0p20m_xyzi.pcd" in config_text
    assert "b985cd2b49c796809c3dfe8ae79e39717454e27e725cd4495d695ea95c6628dc" in config_text

def test_moving_config_accepts_observed_stationary_calibration_band():
    config = yaml.safe_load(
        (ROOT / "config" / "moving_localization.yaml").read_text(encoding="utf-8")
    )

    assert config["max_fitness"] >= 0.263
    assert config["min_inlier_ratio"] <= 0.225



def test_static_localizer_remains_tf_and_motion_command_free():
    static_text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            ROOT / "src" / "static_icp_localizer.cpp",
            ROOT / "launch" / "static_localization.launch",
            ROOT / "config" / "static_localization.yaml",
        )
    )

    assert "TransformBroadcaster" not in static_text
    assert "/cmd_vel" not in static_text
    assert "move_base" not in static_text

