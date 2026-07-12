#!/usr/bin/env python3
"""Deterministic fake-series tests for the diagnostics-only control monitor."""

import importlib.util
import math
import sys
from pathlib import Path


MODULE = Path(__file__).resolve().parents[1] / "scripts" / "control_monitor.py"
SPEC = importlib.util.spec_from_file_location("control_monitor", str(MODULE))
control_monitor = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = control_monitor
SPEC.loader.exec_module(control_monitor)


def velocity(value, stamp, angular=0.0):
    return control_monitor.TimedVelocity(value, angular, stamp, stamp)


def complete_inputs(now, nav=0.2, safe=0.2, actual=0.2, angular=0.0,
                    cross_track=0.0, behavior=control_monitor.PROCEED,
                    linear_cap=0.4, angular_cap=0.6, safety_state=control_monitor.CLEAR,
                    deadline_count=0, stamp=None):
    source = now if stamp is None else stamp
    return control_monitor.MonitorInputs(
        now_s=now,
        route=control_monitor.RouteObservation(now, cross_track, 10.0 - now, source, now),
        odom=control_monitor.TimedVelocity(actual, angular, source, now),
        nav_command=velocity(nav, now, angular),
        safe_command=velocity(safe, now, angular),
        intent=control_monitor.IntentObservation(behavior, linear_cap, angular_cap, source, now),
        safety=control_monitor.SafetyObservation(
            safety_state, deadline_count, source, now, nav, angular, safe, angular),
    )


def test_nominal_series_reports_tracking_and_derivatives_without_faults():
    core = control_monitor.ControlMonitorCore()
    first = core.observe(complete_inputs(1.00, actual=0.10))
    second = core.observe(complete_inputs(1.05, actual=0.20))
    third = core.observe(complete_inputs(1.10, actual=0.25))
    assert first.faults == ()
    assert second.faults == ()
    assert math.isclose(second.linear_acceleration_mps2, 2.0)
    assert math.isclose(third.linear_tracking_error_mps, 0.05)
    assert math.isclose(third.linear_jerk_mps3, -20.0)


def test_curve_series_tracks_lateral_window_and_angular_error():
    core = control_monitor.ControlMonitorCore()
    values = (-0.20, 0.10, 0.30)
    result = None
    for index, cross_track in enumerate(values):
        now = 2.0 + 0.05 * index
        result = core.observe(complete_inputs(now, angular=0.2, cross_track=cross_track))
    assert math.isclose(result.cross_track_mean_m, sum(values) / len(values))
    assert math.isclose(result.cross_track_rms_m,
                        math.sqrt(sum(value * value for value in values) / len(values)))
    assert result.cross_track_max_abs_m == 0.30
    assert result.angular_tracking_error_rps == 0.0


def test_stop_counts_transition_and_flags_persistent_nonzero_safe_command():
    core = control_monitor.ControlMonitorCore()
    core.observe(complete_inputs(3.00))
    immediate = core.observe(complete_inputs(3.05, behavior=control_monitor.STOP,
                                              linear_cap=0.0, angular_cap=0.0))
    persistent = core.observe(complete_inputs(3.10, behavior=control_monitor.STOP,
                                               linear_cap=0.0, angular_cap=0.0))
    cleared = core.observe(complete_inputs(3.15, nav=0.0, safe=0.0, actual=0.0,
                                            behavior=control_monitor.STOP,
                                            linear_cap=0.0, angular_cap=0.0))
    assert immediate.stop_count == 1
    assert "command_after_stop" in immediate.faults
    assert "safe_exceeds_intent" in immediate.faults
    assert "unsafe_command_persistence" in persistent.faults
    assert "command_after_stop" not in cleared.faults
    assert cleared.stop_count == 1


def test_stale_source_and_receipt_ages_are_independent_and_deterministic():
    core = control_monitor.ControlMonitorCore()
    result = core.observe(complete_inputs(4.0, stamp=3.0))
    assert "stale_route" in result.faults
    assert "stale_odom" in result.faults
    assert "stale_intent" in result.faults
    assert "stale_safety" in result.faults
    assert result.ages_s["odom_source"] == 1.0
    assert result.ages_s["odom_receipt"] == 0.0


def test_cap_wrong_sign_reverse_and_nonfinite_contract_faults():
    core = control_monitor.ControlMonitorCore()
    capped = core.observe(complete_inputs(5.0, nav=0.4, safe=0.3, linear_cap=0.2))
    wrong = core.observe(complete_inputs(5.05, nav=0.2, safe=-0.1, linear_cap=0.2))
    nonfinite = core.observe(complete_inputs(5.10, safe=float("nan")))
    assert "safe_exceeds_intent" in capped.faults
    assert capped.cap_event_count == 1
    assert "wrong_sign" in wrong.faults
    assert "reverse_autonomy" in wrong.faults
    assert "nonfinite" in nonfinite.faults


def test_safety_state_command_echo_mismatches_are_flagged():
    core = control_monitor.ControlMonitorCore()
    inputs = complete_inputs(5.15)
    mismatch = control_monitor.SafetyObservation(
        control_monitor.CLEAR, 0, 5.15, 5.15, 0.1, 0.0, 0.1, 0.0)
    result = core.observe(control_monitor.MonitorInputs(
        now_s=inputs.now_s, route=inputs.route, odom=inputs.odom,
        nav_command=inputs.nav_command, safe_command=inputs.safe_command,
        intent=inputs.intent, safety=mismatch))
    assert "safety_requested_mismatch" in result.faults
    assert "safety_output_mismatch" in result.faults

def test_jitter_deadlines_and_safety_counter_regression_are_observed():
    core = control_monitor.ControlMonitorCore()
    core.observe(complete_inputs(6.00, deadline_count=2))
    late = core.observe(complete_inputs(6.08, deadline_count=4))
    regressed = core.observe(complete_inputs(6.10, deadline_count=3))
    clock = core.observe(complete_inputs(6.09, deadline_count=4))
    assert "deadline_miss" in late.faults
    assert late.deadline_miss_count == 5
    assert "time_regression" in regressed.faults
    assert "time_regression" in clock.faults


def test_statistics_and_event_memory_remain_bounded():
    config = control_monitor.MonitorConfig(statistics_window=8, event_history=5)
    core = control_monitor.ControlMonitorCore(config)
    result = None
    for index in range(1000):
        now = 10.0 + 0.05 * index
        result = core.observe(complete_inputs(now, nav=0.3, safe=0.2,
                                              cross_track=float(index)))
    assert core.retained_sample_count <= 13
    assert result.cross_track_mean_m == sum(range(992, 1000)) / 8.0
    assert result.cross_track_max_abs_m == 999.0


def test_safety_stop_and_nonfinite_time_never_create_an_authority_output():
    core = control_monitor.ControlMonitorCore()
    stopped = core.observe(complete_inputs(7.0, safety_state=control_monitor.STOPPED))
    invalid_time = core.observe(complete_inputs(float("nan")))
    assert "command_after_stop" in stopped.faults
    assert "nonfinite" in invalid_time.faults
    assert not hasattr(core, "publish_command")
    assert not hasattr(stopped, "command")
