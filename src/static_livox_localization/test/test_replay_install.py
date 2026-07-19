from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_replay_metric_gate_is_installed_as_catkin_python_program():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "scripts/replay_metrics.py" in cmake
