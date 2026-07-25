"""Make the repository's catkin Python packages importable from a clean checkout.

README documents a plain ``python3 -m pytest -q`` from the repository root, but a
catkin Python package such as ``wheelchair_navigation`` only reaches ``sys.path``
after ``catkin_make`` followed by ``source devel/setup.bash``. Without that, the
whole run dies during collection on ``ModuleNotFoundError: wheelchair_navigation``.

CI still builds the workspace and sources ``devel/setup.bash`` so the real catkin
install path stays under test. These entries are appended rather than inserted,
so a sourced devel/install space always wins and this only acts as the
clean-checkout fallback.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def catkin_python_source_dirs():
    """Yield the ``src`` dir of every package that calls ``catkin_python_setup()``."""
    for cmakelists in sorted(ROOT.glob("src/*/CMakeLists.txt")):
        if "catkin_python_setup()" not in cmakelists.read_text(encoding="utf-8"):
            continue
        source_dir = cmakelists.parent / "src"
        if source_dir.is_dir():
            yield source_dir


for source_dir in catkin_python_source_dirs():
    entry = str(source_dir)
    if entry not in sys.path:
        sys.path.append(entry)
