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
        if degraded_age_s is not None and degraded_age_s > degraded_stop_s:
            return "LOCALIZATION_DEGRADED_TIMEOUT"
        return None
    if state == "LOST":
        return "LOCALIZATION_LOST"
    return "LOCALIZATION_NOT_TRACKING"
