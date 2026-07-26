"""Guard the documented clean-checkout developer command.

README tells a developer to run ``python3 -m pytest -q`` from the repository
root. That aborted during collection, because ``wheelchair_navigation`` lives in
a catkin ``src/`` layout that only reaches ``sys.path`` after ``catkin_make`` and
``source devel/setup.bash``. One un-importable module turns into
``Interrupted: 1 error during collection`` and the entire suite stops.

These tests pin the documented command so the regression cannot come back
silently, and they deliberately scrub ``PYTHONPATH`` so a developer's ambient
environment cannot make a broken checkout look healthy.
"""

import os
import subprocess
import sys
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]


def clean_environment():
    """The ambient shell must not be what makes collection succeed."""
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("ROS_PACKAGE_PATH", None)
    return environment


class CleanCheckoutCollectionTests(unittest.TestCase):
    def test_documented_pytest_command_collects_without_pythonpath(self):
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "pytest",
                "-q",
                "--collect-only",
                "-p",
                "no:cacheprovider",
            ],
            cwd=str(REPOSITORY),
            env=clean_environment(),
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=600,
        )
        self.assertEqual(
            result.returncode,
            0,
            "documented `python3 -m pytest -q` cannot collect a clean checkout:\n"
            + result.stdout[-4000:],
        )
        self.assertNotIn("error during collection", result.stdout)

    def test_every_catkin_python_package_has_an_importable_layout(self):
        """The layout `catkin_python_setup()` promises must actually be there.

        This imports each package with only its own declared `src` directory on
        the path -- the same thing a devel/install space provides, and the same
        thing the root conftest falls back to. A bare `python3 -c "import ..."`
        from the repository root is deliberately not asserted: without a sourced
        workspace that is not expected to work, and CI covers the real catkin
        path by sourcing devel/setup.bash.
        """
        packages = []
        for cmakelists in sorted(REPOSITORY.glob("src/*/CMakeLists.txt")):
            if "catkin_python_setup()" not in cmakelists.read_text(encoding="utf-8"):
                continue
            package = cmakelists.parent.name
            source_dir = cmakelists.parent / "src"
            self.assertTrue(
                (source_dir / package / "__init__.py").is_file(),
                f"{package} declares catkin_python_setup() but has no "
                f"src/{package}/__init__.py",
            )
            self.assertTrue(
                (cmakelists.parent / "setup.py").is_file(),
                f"{package} declares catkin_python_setup() but has no setup.py",
            )
            packages.append((package, source_dir))

        self.assertTrue(packages, "no catkin Python packages were discovered")

        for package, source_dir in packages:
            result = subprocess.run(
                [
                    sys.executable,
                    "-c",
                    f"import sys; sys.path.insert(0, {str(source_dir)!r}); "
                    f"import {package}",
                ],
                cwd=str(REPOSITORY),
                env=clean_environment(),
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=120,
            )
            self.assertEqual(
                result.returncode, 0, f"{package} is not importable:\n{result.stdout}"
            )


if __name__ == "__main__":
    unittest.main()
