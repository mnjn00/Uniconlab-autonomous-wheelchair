from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_moving_report_is_installed_as_catkin_python_program():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "scripts/report_moving_trial.py" in cmake

