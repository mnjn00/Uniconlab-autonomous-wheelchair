"""Static packaging contract for this package.

A `catkin_make install` space, and therefore a fresh NUC deploy, contains only
what CMakeLists installs and only the dependencies package.xml declares. A
source/devel workspace hides gaps in both, because the source tree stays on the
path and the NUC already carries stray system packages -- so the field command
path is checked statically here instead.
"""

import re
from pathlib import Path


ROOT = Path(__file__).parents[1]
REPOSITORY = ROOT.parents[1]
SCRIPTS = ROOT / "scripts"

# Python module name -> rosdep key that must appear in package.xml.
DEPENDENCY_KEYS = {
    "rospy": "rospy",
    "tf": "tf",
    "std_msgs": "std_msgs",
    "std_srvs": "std_srvs",
    "sensor_msgs": "sensor_msgs",
    "geometry_msgs": "geometry_msgs",
    "nav_msgs": "nav_msgs",
    "diagnostic_msgs": "diagnostic_msgs",
    "visualization_msgs": "visualization_msgs",
    "numpy": "python3-numpy",
    "scipy": "python3-scipy",
}


def cmake_text():
    """CMakeLists with comment lines removed, so a mention in prose never counts."""
    lines = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8").splitlines()
    return "\n".join(line for line in lines if not line.lstrip().startswith("#"))


def package_text():
    return (ROOT / "package.xml").read_text(encoding="utf-8")


def script_paths():
    return sorted(SCRIPTS.glob("*.py"))


def top_level_imports(path):
    names = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r"^\s*(?:from|import)\s+([A-Za-z_][A-Za-z0-9_]*)", line)
        if match:
            names.add(match.group(1))
    return names


def test_package_declares_required_moving_ros_dependencies():
    package = package_text()
    cmake = cmake_text()
    for dependency in (
        "nav_msgs",
        "tf2",
        "tf2_ros",
        "visualization_msgs",
        "message_filters",
    ):
        assert dependency in package
        assert dependency in cmake


def test_every_executable_node_is_installed():
    """Anything with a shebang is a node someone can rosrun, so it must install."""
    cmake = cmake_text()
    for path in script_paths():
        first_line = path.read_text(encoding="utf-8").split("\n", 1)[0]
        if not first_line.startswith("#!"):
            continue
        assert f"scripts/{path.name}" in cmake, (
            f"{path.name} is an executable node but is missing from "
            "catkin_install_python in CMakeLists.txt"
        )


def test_sibling_modules_imported_by_nodes_are_installed():
    """`from body_frame import ...` only resolves if the module installs alongside."""
    cmake = cmake_text()
    module_names = {path.stem for path in script_paths()}
    for path in script_paths():
        for name in top_level_imports(path):
            if name not in module_names or name == path.stem:
                continue
            assert f"scripts/{name}.py" in cmake, (
                f"{path.name} imports sibling module {name}, but "
                f"scripts/{name}.py is not installed in CMakeLists.txt"
            )


def test_third_party_and_ros_python_dependencies_are_declared():
    package = package_text()
    for path in script_paths():
        for name in top_level_imports(path):
            key = DEPENDENCY_KEYS.get(name)
            if key is None:
                continue
            assert f">{key}<" in package, (
                f"{path.name} imports {name}, but package.xml does not declare {key}"
            )


def test_nodes_on_the_field_command_path_are_installed():
    """Whatever the launch files and the startup script invoke must be installed."""
    cmake = cmake_text()
    sources = list((ROOT / "launch").glob("*.launch"))
    sources.append(REPOSITORY / "tools" / "start_wheelchair_localization.sh")

    referenced = set()
    for source in sources:
        if not source.is_file():
            continue
        text = source.read_text(encoding="utf-8")
        for name in re.findall(r"([A-Za-z_][A-Za-z0-9_]*\.py)", text):
            if (SCRIPTS / name).is_file():
                referenced.add(name)

    assert referenced, "no package nodes were found on the field command path"
    for name in sorted(referenced):
        assert f"scripts/{name}" in cmake, (
            f"{name} is invoked by a launch file or the startup script but is "
            "not installed in CMakeLists.txt"
        )
