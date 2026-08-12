"""ROS-free localization-state policy for the field waypoint follower."""

# What the localizer reports when registration did not run at all because
# the chair has not moved far enough to earn a correction.
SUPPRESSED_WHILE_PARKED = "STATIONARY_CORRECTION_SUPPRESSED"

# These are not ordinary transient registration failures. The chair has
# evidence that the map pose itself jumped or that the scan was dominated by
# returns we could not safely mask, so the follower must stop immediately
# rather than spend its degraded grace period steering from a bad pose.
IMMEDIATE_HOLD_REASONS = frozenset((
    "PREDICTION_TRANSLATION_JUMP",
    "PREDICTION_ROTATION_JUMP",
    "DYNAMIC_FILTER_OVERLOADED",
))

# How far the chair may creep to earn one before the hold becomes final.
REACQUIRE_LIMIT_M = 0.5


def localization_hold_reason(state, degraded_age_s, degraded_stop_s,
                             reason=None, reacquire_m=None,
                             reacquire_limit_m=REACQUIRE_LIMIT_M):
    """Return a hold reason unless localization is safe enough to drive.

    TRACKING is the only immediately driveable state.  DEGRADED retains the
    existing bounded grace period so a single rejected correction does not
    create a stop/start oscillation.  Every alignment, startup, unknown, or
    malformed state fails closed.

    Past that grace there is one state that waiting cannot resolve. A parked
    chair suppresses corrections by design - the thresholds are there
    because a motionless chair correcting continuously is how a good fix
    gets talked out of itself, measured at the goal after the 2026-07-31
    runs - so once a DEGRADED hold has brought the chair to a stop, no
    registration runs and DEGRADED can never clear by waiting. Measured over
    1413 s on 2026-08-09: 30 DEGRADED episodes, 21 cleared inside the grace,
    and every one of the four that did not was parked and suppressed. They
    lasted 36, 46, 137 and 266 s and each one ended with a person pushing
    the chair by hand.

    That state is unmeasured, not bad, so the answer is to measure it rather
    than to widen the gate it never came near: over the same run the worst
    successful registration scored fitness 0.034 against a 0.28 limit and
    inlier 0.994 against 0.20. Creep far enough to earn a correction and
    judge on the result. The chair is stationary, so the pose cannot have
    drifted since the last fix; the distance is bounded here; the speed is
    already capped by the degraded branch of the speed policy; and the band
    critic still refuses any rollout that leaves the corridor. If the creep
    does not restore TRACKING the hold becomes final and says which one it
    was, because a chair that cannot re-register is a different fault from
    one that is merely parked.
    """
    if state == "TRACKING":
        return None
    if state == "DEGRADED":
        if reason in IMMEDIATE_HOLD_REASONS:
            return "LOCALIZATION_JUMP" if reason != \
                "DYNAMIC_FILTER_OVERLOADED" else "LOCALIZATION_DYNAMIC_FILTER"
        if degraded_age_s is None or degraded_age_s <= degraded_stop_s:
            return None
        if reason == SUPPRESSED_WHILE_PARKED:
            if reacquire_m is None:
                # No pose to bound the creep with. Fail closed.
                return "LOCALIZATION_DEGRADED_TIMEOUT"
            if reacquire_m > reacquire_limit_m:
                return "LOCALIZATION_REACQUIRE_FAILED"
            return None
        return "LOCALIZATION_DEGRADED_TIMEOUT"
    if state == "LOST":
        return "LOCALIZATION_LOST"
    return "LOCALIZATION_NOT_TRACKING"
