from pathlib import Path


def test_shared_accumulator_uses_the_numpy_pointcloud_decoder():
    script = (Path(__file__).parents[1] / "scripts" /
              "scan_accumulator.py").read_text(encoding="utf-8")
    assert "from cloud_points import points_xyz" in script
    assert "pts = points_xyz(message, read_points)" in script
    assert "np.array(list(read_points" not in script
