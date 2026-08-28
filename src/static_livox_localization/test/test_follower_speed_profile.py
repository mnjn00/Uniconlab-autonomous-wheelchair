"""Jerk-limited S-curve speed profile of the follower, pinned as behaviour.

The operator set the cruise cap to 0.8 m/s on 2026-08-16 and asked for a
jerk limit so speed changes stop lurching the chair. The profile is a pure
function (advance_speed) so the ramp can be pinned without ROS: speed must
stay inside the cap, acceleration must never change faster than the jerk
limit (that is the S-curve), acceleration must ease back toward zero at the
target (not clipped mid-air), and a hard obstacle brake must not be delayed
by any of that comfort logic.
"""

from test_waypoint_follower_geometry import load_follower_module

import pytest


@pytest.fixture(scope="module")
def follower():
    return load_follower_module()


def run_profile(follower, start, target, seconds, brake_hard=False):
    dt = 1.0 / follower.CONTROL_HZ
    speed, accel = start, 0.0
    trace = [(speed, accel)]
    for _ in range(int(seconds * follower.CONTROL_HZ)):
        speed, accel = follower.advance_speed(
            speed, accel, target, dt, brake_hard=brake_hard)
        trace.append((speed, accel))
    return dt, trace


def test_speed_never_exceeds_target_or_zero(follower):
    dt, trace = run_profile(follower, 0.0, follower.MAX_SPEED, 30.0)
    assert all(0.0 <= v <= follower.MAX_SPEED for v, _ in trace)


def test_acceleration_changes_no_faster_than_the_jerk_limit(follower):
    dt, trace = run_profile(follower, 0.0, follower.MAX_SPEED, 30.0)
    jerks = [abs((a1 - a0) / dt) for (_, a0), (_, a1) in zip(trace, trace[1:])]
    assert max(jerks) <= follower.MAX_JERK + 1e-9


def test_profile_is_an_s_curve_not_a_step(follower):
    _, trace = run_profile(follower, 0.0, follower.MAX_SPEED, 30.0)
    accels = [a for _, a in trace]
    peak = max(accels)
    assert peak <= follower.MAX_ACCEL + 1e-9
    # An S-curve rises into its peak acceleration and eases back out of it,
    # instead of jumping to the accel cap in one tick like the old trapezoid
    first_peak = accels.index(peak)
    assert accels[1] < 0.5 * peak
    assert any(a < peak * 0.9 for a in accels[first_peak:])
    assert trace[-1][0] == pytest.approx(follower.MAX_SPEED)
    assert accels[-1] < peak * 0.5


def test_deceleration_also_respects_the_jerk_limit(follower):
    dt, trace = run_profile(follower, follower.MAX_SPEED, 0.3, 30.0)
    jerks = [abs((a1 - a0) / dt) for (_, a0), (_, a1) in zip(trace, trace[1:])]
    assert max(jerks) <= follower.MAX_JERK + 1e-9
    assert trace[-1][0] == pytest.approx(0.3)


def test_hard_brake_is_not_delayed_by_the_jerk_limit(follower):
    dt, trace = run_profile(follower, follower.MAX_SPEED, 0.0, 3.0,
                            brake_hard=True)
    for (v0, _), (v1, a1) in zip(trace, trace[1:]):
        assert v1 == pytest.approx(max(0.0, v0 - follower.MAX_DECEL * dt))
        assert a1 == 0.0


def test_cruise_cap_is_the_operator_directed_point_eight(follower):
    assert follower.MAX_SPEED == pytest.approx(0.8)
