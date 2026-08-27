"""Early interpreter compatibility for the ROS Noetic Python environment.

Python imports ``sitecustomize`` after normal site initialisation and before
pytest or a top-level application module. This makes the old SciPy cKDTree
query signature safe even for tests that import ``dwa_core`` before pytest has
loaded the repository ``conftest.py``.
"""

from pathlib import Path
import sys

SCRIPTS = (Path(__file__).resolve().parent / "src" /
           "static_livox_localization" / "scripts")
if SCRIPTS.is_dir() and str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

try:
    from scipy_ckdtree_compat import install
    install()
except Exception:
    # Some tooling imports the repository without SciPy installed. Runtime
    # nodes and tests that require SciPy will report that dependency normally.
    pass
