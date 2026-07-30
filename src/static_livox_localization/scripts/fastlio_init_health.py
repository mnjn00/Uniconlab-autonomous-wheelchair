#!/usr/bin/env python3
"""Verify FAST-LIO initialized cleanly before anything is chained to it.

FAST-LIO estimates gravity direction and IMU bias from the first seconds of
data on the assumption the vehicle is stationary. Starting anywhere but the
usual spot quietly breaks that assumption: the chair was just wheeled into
position and the rider is still settling, so the window that is supposed to
be still is not. A tilted gravity estimate leaks a component of g into
measured acceleration, which integrates into velocity, and the odometry runs
away in its own frame.

No initial pose can repair that. The seed sets map-to-odom only, and the odom
it is chained to is the thing that is wrong - which is why a bad init looks
exactly like a bad seed in RViz while being a completely different fault.

The distinction is free to make before driving: parked and stationary, a
healthy FAST-LIO holds its pose in camera_init. This node measures that and
fails closed, so a startup can restart FAST-LIO instead of driving on an
estimate that is already diverging.
"""

import math
import sys
from typing import NamedTuple, Sequence, Tuple


# Stationary FAST-LIO holds position to a couple of centimetres, and a real
# divergence produces metres. These sit between the two: loose enough that
# sensor noise or a rider shifting their weight does not cost a drive, tight
# enough that anything actually running away is caught before it is trusted.
MAX_STATIONARY_DRIFT_M = 0.15
MAX_STATIONARY_ATTITUDE_DRIFT_DEG = 2.0
# Drift needs time to show. A shorter window can be perfectly still and prove
# nothing about an init that ramps over seconds.
MIN_WINDOW_S = 5.0
MIN_SAMPLES = 25


class InitHealth(NamedTuple):
    """One verdict on whether FAST-LIO's own estimate is trustworthy."""

    healthy: bool
    reason: str
    translation_drift_m: float
    attitude_drift_deg: float
    window_s: float
    samples: int


def stationary_verdict(
    samples: Sequence[Tuple[float, float, float, float, float, float]],
    max_drift_m: float = MAX_STATIONARY_DRIFT_M,
    max_attitude_drift_deg: float = MAX_STATIONARY_ATTITUDE_DRIFT_DEG,
    min_window_s: float = MIN_WINDOW_S,
    min_samples: int = MIN_SAMPLES,
) -> InitHealth:
    """Judge a parked odometry log: (t, x, y, z, roll_deg, pitch_deg) rows.

    Fails closed. A window too short or too sparse to show drift returns
    unhealthy, because a silent or dead FAST-LIO must never read as a clean
    bill of health.
    """

    rows = tuple(samples)
    if len(rows) < min_samples:
        return InitHealth(
            healthy=False,
            reason="insufficient_samples",
            translation_drift_m=0.0,
            attitude_drift_deg=0.0,
            window_s=0.0,
            samples=len(rows),
        )

    window_s = float(rows[-1][0] - rows[0][0])
    origin = rows[0]
    translation_drift = max(
        math.sqrt(
            (row[1] - origin[1]) ** 2
            + (row[2] - origin[2]) ** 2
            + (row[3] - origin[3]) ** 2
        )
        for row in rows
    )
    attitude_drift = max(
        max(abs(row[4] - origin[4]), abs(row[5] - origin[5])) for row in rows
    )

    if window_s < min_window_s:
        reason = "window_too_short"
    elif translation_drift > max_drift_m:
        reason = "translation_drift"
    elif attitude_drift > max_attitude_drift_deg:
        reason = "attitude_drift"
    else:
        reason = "stationary"

    return InitHealth(
        healthy=reason == "stationary",
        reason=reason,
        translation_drift_m=float(translation_drift),
        attitude_drift_deg=float(attitude_drift),
        window_s=window_s,
        samples=len(rows),
    )


def main() -> int:
    import rospy
    import tf.transformations as tft
    from nav_msgs.msg import Odometry

    rospy.init_node("fastlio_init_health")
    duration_s = float(rospy.get_param("~duration_s", 8.0))
    collected = []

    def on_odom(message):
        pose = message.pose.pose
        orientation = pose.orientation
        roll, pitch, _ = tft.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        collected.append((
            message.header.stamp.to_sec(),
            pose.position.x,
            pose.position.y,
            pose.position.z,
            math.degrees(roll),
            math.degrees(pitch),
        ))

    rospy.Subscriber("/Odometry", Odometry, on_odom, queue_size=200)
    rospy.loginfo(
        "measuring FAST-LIO init health for %.1f s - keep the chair still",
        duration_s,
    )
    deadline = rospy.Time.now() + rospy.Duration(duration_s)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        rospy.sleep(0.1)

    verdict = stationary_verdict(collected)
    rospy.loginfo(
        "init health: %s (drift %.3f m, attitude %.2f deg, %.1f s, %d samples)",
        verdict.reason,
        verdict.translation_drift_m,
        verdict.attitude_drift_deg,
        verdict.window_s,
        verdict.samples,
    )
    if verdict.healthy:
        return 0
    rospy.logerr(
        "FAST-LIO did not initialize cleanly (%s). Its odometry is already "
        "moving while parked, so no initial pose can correct it.",
        verdict.reason,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
