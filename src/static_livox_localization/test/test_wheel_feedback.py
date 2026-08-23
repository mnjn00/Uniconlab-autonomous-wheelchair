"""The only velocity measurement on the bus is the base's own report.

/Odometry carries no twist: FAST-LIO leaves it at zero, and all 13,395
samples of the 2026-08-23 run are exactly 0.000. The planner had been
folding that into its anchor, so every rollout started from a chair that was
never moving. The base reports both wheels at 100 Hz and nothing decoded it.
"""

import importlib.util
from pathlib import Path

from pytest import approx


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load_follower():
    from test_waypoint_follower_geometry import load_follower_module
    return load_follower_module()


def frame(left_dir, left, right_dir, right, mode=65):
    return [72, mode, ord(left_dir), left, ord(right_dir), right,
            ord("O"), 88, 0, 13, 10]


def test_forward_pair_decodes_to_metres_per_second():
    module = load_follower()
    read = module.WaypointFollower.reported_wheel_speeds
    left, right = read(frame("C", 0x3B, "C", 0x3B))
    assert left == approx(0.722, abs=0.002)
    assert right == approx(0.722, abs=0.002)


def test_a_stopped_wheel_reads_zero():
    module = load_follower()
    read = module.WaypointFollower.reported_wheel_speeds
    left, right = read(frame("S", 0x21, "S", 0x21))
    assert left == 0.0 and right == 0.0


def test_reverse_is_signed():
    module = load_follower()
    read = module.WaypointFollower.reported_wheel_speeds
    left, right = read(frame("W", 0x3B, "W", 0x3B))
    assert left < 0 and right < 0


def test_a_differential_reads_as_a_turn():
    """The 08-19 spin: left driving, right stopped. Nothing was watching the
    yaw rate this makes, and the chair turned 2.9 times on the spot."""
    module = load_follower()
    read = module.WaypointFollower.reported_wheel_speeds
    left, right = read(frame("C", 0x3B, "S", 0x21))
    speed = (left + right) / 2.0
    yaw_rate = (right - left) / module.WHEEL_SEPARATION_M
    assert speed == approx(0.361, abs=0.002)
    assert yaw_rate == approx(-1.337, abs=0.01)
