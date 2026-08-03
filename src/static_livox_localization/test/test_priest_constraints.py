"""Physical-unit contracts shared by PRIEST planning and runtime safety."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
PACKAGE_ROOT = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
try:
    import priest_planner as pl
finally:
    sys.path.remove(str(SCRIPTS))


def test_canonical_footprint_inflates_past_the_rectangular_corner() -> None:
    """A circular proxy must enclose the 1.00 m by 0.60 m chair rectangle."""
    planner = pl.PriestPlanner()
    obstacle_radius_m = 0.40

    grown = planner.inflate(np.array([[3.0, 0.0, obstacle_radius_m]]))

    expected_m = obstacle_radius_m + math.hypot(0.50, 0.30) + 0.10
    assert grown[0, 2] == pytest.approx(expected_m), (
        "PRIEST's obstacle proxy does not enclose the chair's front corner")


def test_independent_tolerance_rejects_each_exceeded_physical_unit() -> None:
    """Metres, m/s, m/s^2 and rad/s cannot share one summed threshold."""
    tolerances = pl.ConstraintTolerances(
        obstacle_m=0.01,
        corridor_m=0.01,
        speed_mps=0.02,
        acceleration_mps2=0.03,
        yaw_rate_rps=0.04,
    )
    violation_type = pl.ConstraintViolations
    cases = (
        violation_type(0.011, 0.0, 0.0, 0.0, 0.0),
        violation_type(0.0, 0.011, 0.0, 0.0, 0.0),
        violation_type(0.0, 0.0, 0.021, 0.0, 0.0),
        violation_type(0.0, 0.0, 0.0, 0.031, 0.0),
        violation_type(0.0, 0.0, 0.0, 0.0, 0.041),
    )

    assert all(not violation.is_within(tolerances) for violation in cases), (
        "an exceeded physical-unit limit was hidden by a combined residual")


def test_priest_support_modules_are_installed_for_relay_imports() -> None:
    """catkin relays must install every sibling imported by the node."""
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    for module in (
            "priest_constraints.py", "priest_feasibility.py",
            "priest_sampling.py", "priest_types.py"):
        assert "scripts/%s" % module in cmake, (
            "%s is imported by installed PRIEST code but is not installed"
            % module)
