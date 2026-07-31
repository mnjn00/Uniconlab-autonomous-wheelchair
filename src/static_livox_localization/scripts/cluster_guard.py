"""Obstacle avoidance from classified, tracked clusters instead of raw returns.

obstacle_clusters.py was written as an observer - it draws boxes for RViz and
the black box, and nothing in the motion chain read it. This makes it a
control input, which changes what its output has to be trusted for.

Why clusters rather than the raw corridor check the follower already has:
that check stops on any five returns in the corridor above 0.18 m, so it
fires on sensor noise, on ground returns near the height clip, and on
whatever the rider's coat is doing. The cluster node requires eight points
across at least two occupied 0.20 m cells, excludes the rider by an explicit
box, and classifies what is left by footprint and height. Same sensor, far
fewer reasons to stop for nothing.

More importantly it carries a motion verdict, which is what lets the two
situations be answered differently. Something confirmed parked is gone
around; anything moving - or not yet watched long enough to say - is waited
out where it stands. A raw return cannot support that distinction at all,
because it has no identity from one scan to the next.

The geometry uses each object's BOX, not its centre. A vehicle whose centre
sits 3 m to the side can still have a corner in the corridor, and a guard
comparing centres would drive into it.

Everything unreadable fails closed, and fails closed as MOVING. A malformed
object is reported at zero distance rather than skipped, because skipping it
means not seeing an obstacle; and it is never reported as parked, because
the one thing that must not follow from a producer bug is a manoeuvre around
something whose position is not actually known.
"""

import json
import math

from cluster_tracking import MOVING, STATIC, UNKNOWN

# Must match obstacle_clusters.WINDOW_S; test_cluster_guard pins the pair.
# The consumer needs it to size the stopping envelope: an object's reported
# position is already this old before the message is even published.
ACCUMULATION_S = 0.6
# The producer runs at 5 Hz and publishes every cycle, including when it has
# no cloud, so silence for this long means the node itself is gone.
STALE_S = 1.5

BLOCKED = 0.0


class Threat(object):
    """The nearest thing in the corridor, and whether it is going to move."""

    def __init__(self, distance_m, motion, label=""):
        self.distance_m = distance_m
        self.motion = motion
        self.label = label

    @property
    def parked(self):
        """True only when the producer has watched it stand still.

        UNKNOWN is deliberately not parked: it is what a track looks like
        before it has been seen long enough, and before that the honest
        answer is that this could be someone about to step out.
        """
        return self.motion == STATIC


class Summary(object):
    """One cycle of the producer's output, already validated."""

    def __init__(self, stamp_s, status, objects):
        self.stamp_s = stamp_s
        self.status = status
        self.objects = objects

    @property
    def usable(self):
        return self.status == "OK"


def parse_summary(payload):
    """Parse /perception/objects_summary, or raise ValueError.

    Raising is deliberate: the caller has to distinguish "no objects" from
    "no idea", and a parser returning an empty list for both would make a
    crashed producer look like clear road.
    """
    try:
        data = json.loads(payload)
    except (TypeError, ValueError) as error:
        raise ValueError("objects_summary is not JSON: %s" % error)
    if not isinstance(data, dict):
        raise ValueError("objects_summary is not an object")
    stamp = data.get("stamp")
    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise ValueError("objects_summary has no usable stamp")
    objects = data.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects_summary has no object list")
    return Summary(float(stamp), str(data.get("status", "")), objects)


def object_reach(item, lateral_shift_m):
    """(lateral gap, forward distance, motion) for one object's box.

    The lateral gap is how far the near side of the box sits from the
    chair's centre line; negative means it straddles the line. Forward
    distance is the near face, clamped at zero - an object whose box already
    contains the chair is not at a negative distance, it is here.
    """
    try:
        x = float(item["x"])
        y = float(item["y"])
        size = item["size"]
        half_x = abs(float(size[0])) / 2.0
        half_y = abs(float(size[1])) / 2.0
    except (KeyError, IndexError, TypeError, ValueError):
        return -1.0, BLOCKED, MOVING
    if not all(math.isfinite(v) for v in (x, y, half_x, half_y)):
        return -1.0, BLOCKED, MOVING
    motion = item.get("motion", UNKNOWN)
    if motion not in (STATIC, MOVING, UNKNOWN):
        motion = MOVING
    return abs(y - lateral_shift_m) - half_y, max(0.0, x - half_x), motion


def nearest_threat(summary, half_width_m, lateral_shift_m=0.0):
    """The nearest object overlapping the corridor, or None.

    None means nothing is in the way. It never means "could not tell": an
    unusable summary reports a blocking threat, so a caller treating None
    as clear road is right to.
    """
    if not summary.usable:
        return Threat(BLOCKED, MOVING, summary.status or "unusable")
    nearest = None
    for item in summary.objects:
        gap, distance, motion = object_reach(item, lateral_shift_m)
        if gap > half_width_m:
            continue
        if nearest is None or distance < nearest.distance_m:
            nearest = Threat(distance, motion, str(item.get("class", "")))
    return nearest


GO_ROUND = "go_round"
WAIT = "wait"
CLEAR = "clear"


def avoidance_decision(threat, blocking, blocked_for_s, plan_ahead_m,
                       bypass_after_s):
    """What to do about the nearest thing in the corridor.

    GO_ROUND for something the tracker has watched stand still, and taken
    while it is still plan_ahead_m away so the chair drifts past rather than
    driving up to it and stopping first.

    WAIT for anything moving, or not yet watched long enough to say.
    Stepping around a person is a manoeuvre into where they are about to be.
    Nothing here resumes the chair explicitly: once they leave the corridor
    the threat is gone, the answer becomes CLEAR, and it drives on.

    blocked_for_s is the fallback for sources that carry no identity. A
    raw-scan return is UNKNOWN forever, so standing in the way is the only
    evidence of parkedness it can ever offer - but it never overrules a
    tracker that says the thing is moving.
    """
    if threat is None:
        return CLEAR
    if threat.parked and threat.distance_m < plan_ahead_m:
        return GO_ROUND
    if blocking and blocked_for_s is not None and \
            blocked_for_s > bypass_after_s and threat.motion != MOVING:
        return GO_ROUND
    return WAIT if blocking else CLEAR


def is_stale(summary_stamp_s, now_s, stale_s=STALE_S):
    """True when the producer has gone quiet - or has never spoken.

    A never-seen producer is stale, not clear: the node not being launched
    must not read as an empty road.
    """
    if summary_stamp_s is None:
        return True
    if not math.isfinite(now_s) or not math.isfinite(summary_stamp_s):
        return True
    return (now_s - summary_stamp_s) > stale_s
