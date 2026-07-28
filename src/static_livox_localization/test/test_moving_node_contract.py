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



def test_pose_tf_and_path_stamps_never_repeat_or_go_backwards():
    """A correction is matched to a HISTORICAL odometry sample that the
    odometry callback has usually already published. Re-publishing against
    it re-sent map->odom at a stamp tf2 already held, so tf2 dropped the
    corrected transform as TF_REPEATED_DATA and it only reached the tree on
    the next odometry message. Measured over full_debug_20260727_214306.bag
    before the fix: 425 of 5079 map->camera_init transforms were repeats
    and 425 of 5078 pose stamps went backwards, by up to 1.50 s, while
    every other transform in the system was clean."""
    text = node_text()
    guard = "if (has_published_ && odom.stamp <= last_published_stamp_) return;"
    assert guard in text
    assert "last_published_stamp_ = odom.stamp;" in text
    # the guard has to sit ahead of every publish in that function
    start = text.index("void publish_pose_tf_path_locked")
    assert text.index(guard, start) < text.index("pose_pub_.publish", start)
    assert text.index(guard, start) < text.index("path_pub_.publish", start)
    assert text.index(guard, start) < text.index("sendTransform", start)


def test_a_correction_is_published_against_the_freshest_odometry():
    """The correction is the same either way; applying it to the newest
    base pose is what keeps the stamp moving forward."""
    text = node_text()
    assert "void publish_correction_locked" in text
    assert ("publish_pose_tf_path_locked(has_latest_odom_ ? latest_odom_ "
            ": matched);") in text
    # both accepted-correction branches must go through it, not straight
    # to the raw publisher
    assert text.count("publish_correction_locked(odom);") == 2
