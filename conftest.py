"""Repository-wide pytest setup for the ROS Noetic interpreter."""

from pathlib import Path
import sys

# A static command-authority test intentionally scans text files under src/.
# Prevent imports during the same pytest process from creating __pycache__
# binaries whose names contain words such as "profile" and would otherwise be
# mistaken for source evidence.
sys.dont_write_bytecode = True

SCRIPTS = (Path(__file__).resolve().parent / "src" /
           "static_livox_localization" / "scripts")
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

# The pinned Noetic image ships an older SciPy cKDTree query signature. Tests
# exercise both the legacy CPU planner and the RTX wrapper, so normalise it
# before test modules import dwa_core.
from scipy_ckdtree_compat import install as _install_ckdtree_compat
_install_ckdtree_compat()
