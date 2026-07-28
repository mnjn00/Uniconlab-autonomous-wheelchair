from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]


def test_launch_is_localization_only_and_uses_builtin_imu_config():
    text = (ROOT / "launch" / "mid360_localization.launch").read_text()
    assert 'file="$(find fast_lio)/config/mid360.yaml"' in text
    assert "fastlio_mid360_vn100" not in text
    assert "/vectornav" not in text
    assert "/cmd_vel" not in text
    assert "move_base" not in text
    assert 'name="pcd_save/pcd_save_en" type="bool" value="false"' in text
    assert 'name="publish/tf_en" type="bool" value="false"' in text


def test_launch_requires_exact_map_identity():
    tree = ET.parse(ROOT / "launch" / "mid360_localization.launch")
    arguments = {item.attrib["name"]: item.attrib for item in tree.findall("arg")}
    for required in ("map_path", "map_id", "map_sha256"):
        assert required in arguments
        assert "default" not in arguments[required]


def test_no_guessed_body_to_base_static_transform():
    text = (ROOT / "launch" / "mid360_localization.launch").read_text()
    assert "static_transform_publisher" not in text
    node = (ROOT / "src" / "map_localizer_node.cpp").read_text()
    assert "lookupBodyToBase" in node
    assert "map_T_body * body_T_base" in node


def test_relocalization_uses_the_same_initialpose_seen_by_safety_adapter():
    node = (ROOT / "src" / "map_localizer_node.cpp").read_text()
    assert '"/initialpose"' in node
    assert "initialPoseCallback" in node
    assert "advertiseService" not in node


def test_source_time_regression_fails_closed():
    node = (ROOT / "src" / "map_localizer_node.cpp").read_text()
    assert "ODOMETRY_TIME_REGRESSION" in node
    assert "CLOUD_TIME_REGRESSION" in node
    assert "message->header.stamp.isZero()" in node
    assert "snapshot.state_epoch != state_epoch_" in node
    assert "pose.header.seq = snapshot.reset_count" in node


def test_global_engine_has_no_weak_runtime_fallback():
    config = (ROOT / "config" / "localization.yaml").read_text()
    node = (ROOT / "src" / "map_localizer_node.cpp").read_text()
    assert "global_engine: FPFH_TEASER" in config
    assert "no silent weak fallback" in node
    assert 'global_engine_ != "FPFH_TEASER"' in node


def test_global_and_tracking_registration_reject_gravity_axis_aliases():
    config = (ROOT / "config" / "localization.yaml").read_text()
    node = (ROOT / "src" / "map_localizer_node.cpp").read_text()
    core = (ROOT / "src" / "localization_core.cpp").read_text()
    assert "max_map_to_odom_tilt_deg: 5.0" in config
    assert "GLOBAL_GRAVITY_ALIGNMENT_REJECTED" in node
    assert "TRACKING_GRAVITY_ALIGNMENT_REJECTED" in node
    assert "rotationDistance(reference, map_T_odom)" in core


def test_registration_delta_is_independent_of_map_origin():
    node = (ROOT / "src" / "map_localizer_node.cpp").read_text()
    assert "initial_guess.inverse() * result.map_T_body" in node
    assert "result.map_T_body * initial_guess.inverse()" not in node
