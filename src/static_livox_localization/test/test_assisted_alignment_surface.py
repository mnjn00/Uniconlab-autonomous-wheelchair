from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_assisted_alignment_core_is_built_and_tested():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert (ROOT / "src" / "assisted_alignment.cpp").is_file()
    assert "add_library(assisted_alignment src/assisted_alignment.cpp)" in cmake
    assert "catkin_add_gtest(test_assisted_alignment" in cmake
    assert "target_link_libraries(test_assisted_alignment assisted_alignment" in cmake
