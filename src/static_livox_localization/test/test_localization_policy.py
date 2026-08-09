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


# ------------------------------------- the hold that could not clear itself

SUPPRESSED = "STATIONARY_CORRECTION_SUPPRESSED"


def test_a_parked_and_blind_chair_may_creep_far_enough_to_re_register():
    """Suppression means no registration ran, not that one failed. Holding
    on it is self-locking: the hold stops the chair, a stopped chair
    suppresses corrections, and DEGRADED can never clear. On 2026-08-09 that
    held the chair for 36, 46, 137 and 266 s, each ended by hand."""
    assert POLICY.localization_hold_reason(
        "DEGRADED", 10.0, 3.0, reason=SUPPRESSED, reacquire_m=0.0) is None
    assert POLICY.localization_hold_reason(
        "DEGRADED", 10.0, 3.0, reason=SUPPRESSED, reacquire_m=0.49) is None


def test_the_creep_is_bounded_and_says_which_fault_it_was():
    assert POLICY.localization_hold_reason(
        "DEGRADED", 10.0, 3.0, reason=SUPPRESSED, reacquire_m=0.51) == \
        "LOCALIZATION_REACQUIRE_FAILED"


def test_no_pose_to_bound_the_creep_fails_closed():
    assert POLICY.localization_hold_reason(
        "DEGRADED", 10.0, 3.0, reason=SUPPRESSED, reacquire_m=None) == \
        "LOCALIZATION_DEGRADED_TIMEOUT"


def test_a_real_registration_failure_still_stops_the_chair():
    """Only the suppressed reason is unmeasured. A convergence failure is
    evidence, and the gate it never came near stays where it is."""
    for reason in ("NOT_CONVERGED", "OK", "", None):
        assert POLICY.localization_hold_reason(
            "DEGRADED", 10.0, 3.0, reason=reason, reacquire_m=0.0) == \
            "LOCALIZATION_DEGRADED_TIMEOUT"


def test_the_grace_still_comes_first():
    """Inside the grace nothing holds, whatever the reason - 21 of 30
    episodes cleared there on their own."""
    assert POLICY.localization_hold_reason(
        "DEGRADED", 1.0, 3.0, reason=SUPPRESSED, reacquire_m=9.0) is None
