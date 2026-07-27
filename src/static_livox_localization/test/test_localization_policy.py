import importlib.util
from pathlib import Path

import pytest


POLICY_PATH = Path(__file__).parents[1] / "scripts" / "localization_policy.py"
SPEC = importlib.util.spec_from_file_location("localization_policy", POLICY_PATH)
POLICY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(POLICY)


@pytest.mark.parametrize("state", [
    "",
    "MANUAL_ALIGN",
    "VERIFYING",
    "WAITING_INITIALIZATION",
    "UNRECOGNIZED_STATE",
])
def test_non_tracking_alignment_startup_and_unknown_states_fail_closed(state):
    assert POLICY.localization_hold_reason(state, None, 3.0) == \
        "LOCALIZATION_NOT_TRACKING"


def test_tracking_is_the_only_immediately_driveable_state():
    assert POLICY.localization_hold_reason("TRACKING", None, 3.0) is None
    assert POLICY.localization_hold_reason("LOST", None, 3.0) == \
        "LOCALIZATION_LOST"


def test_degraded_keeps_bounded_grace_then_holds():
    assert POLICY.localization_hold_reason("DEGRADED", 0.0, 3.0) is None
    assert POLICY.localization_hold_reason("DEGRADED", 3.0, 3.0) is None
    assert POLICY.localization_hold_reason("DEGRADED", 3.001, 3.0) == \
        "LOCALIZATION_DEGRADED_TIMEOUT"
