"""ROS-free localization-state policy for the field waypoint follower."""


def localization_hold_reason(state, degraded_age_s, degraded_stop_s):
    """Return a hold reason unless localization is safe enough to drive.

    TRACKING is the only immediately driveable state.  DEGRADED retains the
    existing bounded grace period so a single rejected correction does not
    create a stop/start oscillation.  Every alignment, startup, unknown, or
    malformed state fails closed.
    """
    if state == "TRACKING":
        return None
    if state == "DEGRADED":
        if degraded_age_s is None:
            # How long it has been degraded is unknown, so the grace period
            # cannot be evaluated. The caller sets degraded_since before asking,
            # but tracking_state is written from the diagnostic callback thread,
            # so a state change landing between those two points reaches here
            # with no age. Holding for that one cycle is the fail-closed answer.
            return "LOCALIZATION_DEGRADED_AGE_UNKNOWN"
        if degraded_age_s > degraded_stop_s:
            return "LOCALIZATION_DEGRADED_TIMEOUT"
        return None
    if state == "LOST":
        return "LOCALIZATION_LOST"
    return "LOCALIZATION_NOT_TRACKING"
