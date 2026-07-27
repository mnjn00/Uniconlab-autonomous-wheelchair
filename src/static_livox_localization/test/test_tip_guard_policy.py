import importlib.util
from pathlib import Path


POLICY_PATH = Path(__file__).parents[1] / "scripts" / "tip_guard_policy.py"
SPEC = importlib.util.spec_from_file_location("tip_guard_policy", POLICY_PATH)
POLICY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(POLICY)


def test_zero_request_is_an_immediate_stop_authority():
    assert POLICY.next_linear_speed(1.2, 0.0, 0.3, 0.02, False) == 0.0
    assert POLICY.next_linear_speed(-0.2, 0.0, 0.3, 0.02, False) == 0.0


def test_internal_trip_or_staleness_stops_immediately():
    assert POLICY.next_linear_speed(1.2, 1.2, 0.3, 0.02, True) == 0.0


def test_output_cannot_exceed_the_final_stage_limit():
    limit = POLICY.ABSOLUTE_V_LIMIT
    assert POLICY.next_linear_speed(limit, 99.0, 99.0, 1.0, False) == limit
    assert POLICY.next_linear_speed(-limit, -99.0, 99.0, 1.0, False) == -limit


def test_nonzero_acceleration_remains_rate_limited():
    assert POLICY.next_linear_speed(0.0, 1.0, 0.3, 0.1, False) == 0.03
