"""Turning the solver's accelerations into the velocity the base is given.

Small, and separate from the node, because the bug it exists to fix was
invisible in simulation and only arithmetic can be argued with at a desk.

WHAT WENT WRONG ON 2026-08-05
-----------------------------
The first version of this conversion was one line in the node:

    speed = measured_velocity + a * (1 / CONTROL_HZ)

Two things are wrong with it, and the field run showed both.

**It cannot start the chair.** From rest the measured velocity is 0 and
a_max is 0.18, so the command is 0.018 m/s. The loaded base was measured
not to move below roughly 0.3 m/s - the same measurement TURN_FLOOR_SPEED
comes from. So the wheels ignore 0.018, the measured velocity stays 0, and
the next cycle computes 0.018 again. Driven, it sat at v=0.00-0.02 and
covered one 0.2 m waypoint in five seconds. The pursuit follower never had
this: it has always ramped its own COMMANDED speed, which climbs out of the
deadband whether or not the wheels have answered yet.

**The period was assumed, not measured.** The same run had the control loop
at a median 0.194 s, p90 0.813 s, p99 2.14 s against a nominal 0.1 s -
72 % of cycles over period - because FAST-LIO, RViz, a remote-desktop
session and the CUDA localizer were sharing four cores. A fixed dt turns
that into a ramp two to twenty times slower than intended, silently,
because the arithmetic still looks correct.

Neither could have been caught in simulation. The unicycle plant has no
actuation deadband, so it follows 0.018 m/s perfectly, and the simulated
loop never misses its period.
"""

import numpy as np

# A cycle longer than this is not a control period, it is a gap. The caller
# re-syncs to the measured velocity across one rather than carrying a stale
# command forward: after two seconds of silence the chair has done whatever
# it did, and our idea of what we last asked for is fiction.
MAX_COMMAND_GAP_S = 0.5
# ...and a cycle shorter than this contributes no meaningful ramp. It also
# covers the first cycle, where there is no previous stamp to subtract.
MIN_COMMAND_STEP_S = 0.02
# Ceiling on how fast the acceleration itself may change. The ramp above is
# built from a velocity error divided by a step, so it can jump from nothing
# to MAX_ACCEL between two cycles: the speed curve is trapezoidal and the
# corners are what the rider feels. Bounding the slope turns those corners
# into an S. Sized against the chair, not comfort alone - MAX_ACCEL is
# 0.18 m/s^2, so 0.8 m/s^3 reaches full acceleration in 0.23 s and costs
# nothing at the speeds this route runs.
MAX_JERK = 0.8


def jerk_limited(desired_accel, previous_accel, step_s, max_jerk=MAX_JERK):
    """The acceleration to use this cycle, given the one used last cycle.

    Pure, because the alternative is a follower that cannot be tested without
    a chair under it. Hard braking does not come through here: a stop resets
    the carried acceleration instead, so nothing about ride quality can
    delay one.
    """
    room = abs(float(max_jerk)) * max(float(step_s), 0.0)
    low = float(previous_accel) - room
    high = float(previous_accel) + room
    return min(max(float(desired_accel), low), high)


def advance_command(speed, yaw_rate, u0, elapsed_s, v_max, yaw_max):
    """Integrate one solver step onto the previous COMMAND.

    Onto the command, not onto the measured velocity - that is the fix, not
    a detail. Returns the (speed, yaw_rate) to put on the wire, clamped:
    an inaccurate solve must not carry the base past a cap.
    """
    step = min(max(float(elapsed_s), MIN_COMMAND_STEP_S), MAX_COMMAND_GAP_S)
    return (float(np.clip(speed + u0[0] * step, 0.0, v_max)),
            float(np.clip(yaw_rate + u0[1] * step, -yaw_max, yaw_max)))
