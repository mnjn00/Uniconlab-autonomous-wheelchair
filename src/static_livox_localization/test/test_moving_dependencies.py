from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_package_declares_required_moving_ros_dependencies():
    package_text = (ROOT / "package.xml").read_text(encoding="utf-8")
    cmake_text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    for dependency in (
        "nav_msgs",
        "tf2",
        "tf2_ros",
        "visualization_msgs",
        "message_filters",
    ):
        assert dependency in package_text
        assert dependency in cmake_text

