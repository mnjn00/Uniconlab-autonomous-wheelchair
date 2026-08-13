"""Every installed node must still find the modules sitting beside it.

catkin_install_python does not copy a script into devel/lib - it writes a
relay there that open()s the source file and exec()s it. That makes
sys.path[0] the relay's directory, so a node's sibling policy modules stop
being importable the moment the node is added to catkin_install_python,
while every offline test keeps passing because those import the modules
directly.

That is exactly how it failed on the vehicle: tip_guard and
waypoint_follower died at import with ModuleNotFoundError for
tip_guard_policy and body_frame, with the localization stack otherwise
healthy and TRACKING. safety_gate survived only because it had been left
out of catkin_install_python and so ran from the source tree.

The script list and the module list are both derived here rather than
written down, so a new node or a new policy module is covered without
anyone remembering this file exists.
"""

import re
from pathlib import Path


ROOT = Path(__file__).parents[1]
PKG = ROOT / "src" / "static_livox_localization"
SCRIPTS = PKG / "scripts"
CMAKE = PKG / "CMakeLists.txt"

SYS_PATH_RECOVERY = (
    "sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))")


def installed_scripts():
    text = CMAKE.read_text(encoding="utf-8")
    block = re.search(r"catkin_install_python\(\s*PROGRAMS(.*?)DESTINATION",
                      text, re.S)
    assert block, "catkin_install_python(PROGRAMS ...) block not found"
    return sorted(Path(p).name for p in re.findall(r"scripts/(\S+\.py)",
                                                   block.group(1)))


def sibling_modules(script=None):
    """Everything a node could import from its own installed directory.

    Both lists count, not just the policy modules: a node installed through
    catkin_install_python lands in the same directory as the modules, so one
    node importing another - mpc_follower subclassing waypoint_follower to
    inherit its guards - has exactly the hazard this file is about. Only the
    importing script itself is excluded, since it cannot import itself.
    """
    everything = {p.stem for p in SCRIPTS.glob("*.py")}
    return everything - ({Path(script).stem} if script else set())


def sibling_imports(script):
    text = (SCRIPTS / script).read_text(encoding="utf-8")
    siblings = sibling_modules(script)
    found = set()
    for match in re.finditer(r"^\s*(?:from|import)\s+([A-Za-z_][A-Za-z0-9_]*)",
                             text, re.M):
        if match.group(1) in siblings:
            found.add(match.group(1))
    return found


def first_import_at(text, module):
    """Where a module is first imported, in either spelling.

    Looking only for "from x " skips "import x" entirely and, because the
    detector above accepts both, raised ValueError on the first node to use
    the plain form rather than reporting anything.
    """
    match = re.search(r"^\s*(?:from|import)\s+%s\b" % re.escape(module),
                      text, re.M)
    assert match, "%s is imported but cannot be located" % module
    return match.start()


def test_every_installed_node_importing_a_sibling_recovers_its_own_directory():
    offenders = []
    for script in installed_scripts():
        imports = sibling_imports(script)
        if not imports:
            continue
        text = (SCRIPTS / script).read_text(encoding="utf-8")
        if SYS_PATH_RECOVERY not in text:
            offenders.append("%s imports %s" % (script, sorted(imports)))
            continue
        # and it has to happen BEFORE the first sibling import
        guard = text.index(SYS_PATH_RECOVERY)
        first = min(first_import_at(text, m) for m in sorted(imports))
        if guard > first:
            offenders.append("%s recovers sys.path after importing" % script)
    assert not offenders, (
        "installed nodes that will die at import on the vehicle: %s"
        % offenders)


def test_the_recovery_uses_file_which_the_relay_actually_sets():
    """The relay sets __file__ to the SOURCE path, which is what makes this
    work. Deriving the directory from sys.argv[0] or sys.path[0] instead
    would resolve back to the relay and fix nothing."""
    for script in installed_scripts():
        text = (SCRIPTS / script).read_text(encoding="utf-8")
        if SYS_PATH_RECOVERY in text:
            assert "sys.argv[0]" not in text.split(SYS_PATH_RECOVERY)[0][-400:]


def test_every_node_the_field_startup_runs_is_installed():
    """safety_gate ran from the source tree because it was missing from
    catkin_install_python. That hid this bug for it and means the vehicle
    was running an unbuilt copy."""
    startup = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8")
    launched = set(re.findall(
        r"rosrun\s+static_livox_localization\s+(\S+\.py)", startup))
    missing = sorted(launched - set(installed_scripts()))
    assert not missing, (
        "started in the field but not installed: %s" % missing)


def test_every_installed_node_sibling_import_is_installed():
    text = CMAKE.read_text(encoding="utf-8")
    installed = set(re.findall(r"scripts/(\S+\.py)", text))
    missing = []
    for script in installed_scripts():
        for module in sibling_imports(script):
            if f"{module}.py" not in installed:
                missing.append(f"{script} imports {module}.py")
    assert not missing, "missing installed sibling modules: %s" % missing


def test_declared_runtime_dependencies_cover_route_assets():
    package = (PKG / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>python3-yaml</exec_depend>" in package
    assert "<exec_depend>python3-pil</exec_depend>" in package
