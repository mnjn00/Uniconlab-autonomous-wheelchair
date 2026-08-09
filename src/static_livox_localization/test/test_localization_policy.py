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


# --------------------------------------- a pose that moved faster than the chair

import importlib.util as _ilu
from pathlib import Path as _P
_MS = _P(__file__).parents[1] / "scripts" / "motion_safety.py"
_S = _ilu.spec_from_file_location("motion_safety", _MS)
MOTION = _ilu.module_from_spec(_S)
_S.loader.exec_module(MOTION)


def test_a_believable_step_is_passed_through_untouched():
    xy, withheld = MOTION.clamp_pose_step((0.0, 0.0), (0.05, 0.0), 0.1)
    assert withheld == 0.0
    assert tuple(xy) == (0.05, 0.0)


def test_a_step_the_chair_could_not_have_made_is_clamped_not_dropped():
    """On 2026-08-09 the map correction swung 0.52 m in one sample - an
    apparent 2.64 m/s on a chair limited to 0.6. Clamped rather than
    rejected, so a genuine re-seed still converges within a few cycles."""
    xy, withheld = MOTION.clamp_pose_step((0.0, 0.0), (0.52, 0.0), 0.1)
    assert withheld > 0.0
    assert 0.0 < xy[0] < 0.52
    assert abs(xy[0] - MOTION.POSE_STEP_LIMIT_MPS * 0.1) < 1e-9


def test_the_clamp_keeps_the_direction_it_was_given():
    xy, _ = MOTION.clamp_pose_step((1.0, 1.0), (1.0, 5.0), 0.1)
    assert xy[0] == 1.0 and 1.0 < xy[1] < 5.0


def test_the_first_pose_and_a_zero_interval_are_believed():
    xy, withheld = MOTION.clamp_pose_step(None, (9.0, 9.0), 0.1)
    assert withheld == 0.0 and tuple(xy) == (9.0, 9.0)
    xy, withheld = MOTION.clamp_pose_step((0.0, 0.0), (9.0, 9.0), 0.0)
    assert withheld == 0.0 and tuple(xy) == (9.0, 9.0)


def test_jitter_below_the_floor_is_not_argued_with():
    xy, withheld = MOTION.clamp_pose_step((0.0, 0.0), (0.04, 0.0), 0.001)
    assert withheld == 0.0
