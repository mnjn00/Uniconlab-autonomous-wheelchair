from pathlib import Path


ROOT = Path(__file__).parents[1]
NODE = ROOT / "src" / "moving_icp_localizer.cpp"


def node_text():
    return NODE.read_text(encoding="utf-8")


def test_node_consumes_seed_body_cloud_and_fast_lio_odometry():
    text = node_text()

    assert "seed_topic_" in text
    assert "cloud_topic_" in text
    assert "odom_topic_" in text
    assert "geometry_msgs::PoseWithCovarianceStamped" in text
    assert "sensor_msgs::PointCloud2" in text
    assert "nav_msgs::Odometry" in text


def test_node_publishes_pose_path_diagnostics_and_only_map_to_odom_tf():
    text = node_text()

    assert '"/fast_lio_icp/pose"' in text
    assert '"/fast_lio_icp/path"' in text
    assert '"/fast_lio_icp/localization_diagnostics"' in text
    assert text.count("tf2_ros::TransformBroadcaster") == 1
    assert "map_frame_" in text
    assert "odom_frame_" in text
    assert "base_frame_" in text


def test_node_uses_rolling_submap_and_map_to_odom_correction_math():
    text = node_text()

    assert "RollingSubmap" in text
    assert "compute_map_T_odom" in text
    assert "evaluate_correction" in text
    assert "limit_map_T_odom_step" in text
    assert "map_T_odom_ * odom_T_base" in text


def test_node_never_controls_motion_or_rewrites_fixed_map():
    text = node_text()

    for forbidden in (
        "/cmd_vel",
        "move_base",
        "savePCDFile",
        "setGlobalMapOrigin",
    ):
        assert forbidden not in text


def test_cmake_builds_moving_node_separately_from_static_node():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "add_executable(moving_icp_localizer" in cmake
    assert "add_executable(static_icp_localizer" in cmake

