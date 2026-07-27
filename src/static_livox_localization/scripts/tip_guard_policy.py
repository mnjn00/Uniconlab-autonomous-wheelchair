"""ROS-free final-stage linear-speed policy for the field tip guard."""

ABSOLUTE_V_LIMIT = 1.6
NORMAL_DECEL_MPS2 = 2.0


def next_linear_speed(current, requested, accel_limit, dt, stop_required):
    """Apply the governor without delaying an upstream stop.

    A zero request is the common stop authority used by obstacle, localization,
    route-band, tilt, and manual-mode holds. The final stage must never turn
    that zero into residual motion through its own slew limiter.
    """
    if stop_required or requested == 0.0:
        return 0.0

    desired = max(-ABSOLUTE_V_LIMIT, min(ABSOLUTE_V_LIMIT, requested))
    if desired > current:
        step = min(desired - current, max(0.0, accel_limit) * max(0.0, dt))
    else:
        step = max(desired - current, -NORMAL_DECEL_MPS2 * max(0.0, dt))
    speed = current + step
    return max(-ABSOLUTE_V_LIMIT, min(ABSOLUTE_V_LIMIT, speed))
