from pathlib import Path


ROOT = Path(__file__).parents[1]

def test_every_gtest_is_inside_catkin_testing_guard():
    cmake_lines = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8").splitlines()
    depth = 0
    inside_catkin_testing = False
    for line in cmake_lines:
        stripped = line.strip()
        if stripped == "if(CATKIN_ENABLE_TESTING)":
            depth += 1
            inside_catkin_testing = True
        elif stripped.startswith("if("):
            depth += 1
        elif stripped == "endif()":
            depth -= 1
            if depth == 0:
                inside_catkin_testing = False
        elif "catkin_add_gtest(" in stripped:
            assert inside_catkin_testing and depth >= 1
    assert depth == 0


def test_every_catkin_gtest_defines_an_entry_point():
    cmake_lines = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8").splitlines()
    test_sources = []
    for line in cmake_lines:
        stripped = line.strip()
        if stripped.startswith("catkin_add_gtest("):
            test_sources.append(stripped.rstrip(")").split()[-1])

    assert test_sources
    for source in test_sources:
        assert "RUN_ALL_TESTS()" in (ROOT / source).read_text(encoding="utf-8")


def test_bounded_preview_core_node_and_gtest_are_built():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    for relative in (
        "include/static_livox_localization/bounded_cloud_preview.hpp",
        "src/bounded_cloud_preview.cpp",
        "src/bounded_cloud_preview_node.cpp",
        "test/test_bounded_cloud_preview.cpp",
    ):
        assert (ROOT / relative).is_file()

    assert "add_library(bounded_cloud_preview" in cmake
    assert "add_executable(bounded_cloud_preview_node" in cmake
    assert "catkin_add_gtest(test_bounded_cloud_preview" in cmake
    assert "install(" in cmake and "bounded_cloud_preview_node" in cmake


def test_preview_node_has_bounded_queues_rate_and_empty_cloud_guard():
    node = (ROOT / "src" / "bounded_cloud_preview_node.cpp").read_text(
        encoding="utf-8"
    )
    assert "subscribe(input_topic_, 1" in node
    assert "advertise<sensor_msgs::PointCloud2>(output_topic_, 1" in node
    assert "should_publish" in node
    assert "downsample" in node
    assert "output->empty()" in node
