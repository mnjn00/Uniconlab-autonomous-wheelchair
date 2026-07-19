from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_preflight_declares_tf_message_runtime_dependency():
    package_text = (ROOT / "package.xml").read_text(encoding="utf-8")
    cmake_text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "tf2_msgs" in package_text
    assert "tf2_msgs" in cmake_text

