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

The geometry uses each object's EXTENT, not its centre. A vehicle whose
centre sits 3 m to the side can still have a corner in the corridor, and a
guard comparing centres would drive into it.

Extent means the object's own returns, sliced laterally, wherever the
producer publishes them. An axis-aligned box was the first answer and is
still the fallback, but its near face is a corner of the bounding volume
rather than anything the sensor saw.

For a wall crossing the scan diagonally the two answers are far apart, and
on 2026-07-31 the gap cost a 16-minute hold at waypoint 349. The box put
the wall 0.69 m dead ahead; its nearest return inside the corridor was
2.13 m. 0.69 m is inside the 0.9 m floor on the stop radius, so the chair
stopped - and a stopped chair's stopping envelope stays at that floor, so
the phantom near face stayed inside it and no amount of waiting could
change anything. The tracker had it right the whole time: static, and
therefore never going to move out of the way.

Sidestepping was not the missing answer and is not the fix. That wall
really does span the corridor, and the route curves past it; what the chair
needed was to not be hard-stopped by a face with no returns behind it.
Measured from the returns it reads 1.72 m - outside the stop radius, inside
the slow radius - so the chair creeps and the route does the rest. See
test/test_object_profile.py, which reconstructs the wall from the recorded
box and pins both numbers.

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


def object_box(item):
    """(x, y, half_x, half_y) for one object, or None if it does not parse."""
    try:
        x = float(item["x"])
        y = float(item["y"])
        size = item["size"]
        half_x = abs(float(size[0])) / 2.0
        half_y = abs(float(size[1])) / 2.0
    except (KeyError, IndexError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, half_x, half_y)):
        return None
    return x, y, half_x, half_y


def object_motion(item):
    motion = item.get("motion", UNKNOWN)
    return motion if motion in (STATIC, MOVING, UNKNOWN) else MOVING


def object_reach(item, lateral_shift_m):
    """(lateral gap, forward distance, motion) for one object's box.

    The lateral gap is how far the near side of the box sits from the
    chair's centre line; negative means it straddles the line. Forward
    distance is the near face, clamped at zero - an object whose box already
    contains the chair is not at a negative distance, it is here.

    This is the fallback. The near face of an AXIS-ALIGNED box is a corner
    of the bounding volume, and for anything not aligned with the chair
    there are no returns there: a wall crossing the scan diagonally reported
    0.69 m on 2026-07-31 with its nearest return in the corridor at 2.13 m.
    Where the producer supplies a lateral profile, corridor_reach measures
    from that instead.
    """
    box = object_box(item)
    if box is None:
        return -1.0, BLOCKED, MOVING
    x, y, half_x, half_y = box
    return (abs(y - lateral_shift_m) - half_y, max(0.0, x - half_x),
            object_motion(item))


def profile_reach(item, lateral_shift_m, half_width_m):
    """(blocks, distance) from the object's own returns, or None.

    None means this object carries no profile and the caller must fall back
    to its box. A profile that is PRESENT but unreadable is not a fallback:
    it blocks at zero, because a producer emitting a broken profile is one
    whose box is no more trustworthy, and the fallback would quietly restore
    the very over-approximation this exists to remove.

    A slice counts when it overlaps the corridor at all, so a return sitting
    just inside a slice that straddles the corridor edge is never missed.
    """
    profile = item.get("profile")
    if profile is None:
        return None
    if not isinstance(profile, dict):
        return True, BLOCKED
    try:
        bin_m = float(profile["bin_m"])
        y0 = float(profile["y0"])
        slices = profile["min_x"]
    except (KeyError, TypeError, ValueError):
        return True, BLOCKED
    if not isinstance(slices, list) or not slices or \
            not math.isfinite(bin_m) or bin_m <= 0.0 or not math.isfinite(y0):
        return True, BLOCKED
    low = lateral_shift_m - half_width_m
    high = lateral_shift_m + half_width_m
    nearest = None
    for index, value in enumerate(slices):
        if value is None:
            continue
        if y0 + (index + 1) * bin_m <= low or y0 + index * bin_m >= high:
            continue
        try:
            x = float(value)
        except (TypeError, ValueError):
            return True, BLOCKED
        if not math.isfinite(x):
            return True, BLOCKED
        x = max(0.0, x)
        if nearest is None or x < nearest:
            nearest = x
    if nearest is None:
        return False, None
    return True, nearest


def corridor_reach(item, lateral_shift_m, half_width_m):
    """(blocks, distance, motion) for one object against one corridor.

    The single place the question "is this in the way, and how far" is
    answered. Measures from the object's returns where they are published
    and from its box otherwise; anything that does not parse blocks at zero
    and reads as MOVING, so a producer bug can neither hide an obstacle nor
    make one look parked enough to drive around.
    """
    if object_box(item) is None:
        return True, BLOCKED, MOVING
    motion = object_motion(item)
    measured = profile_reach(item, lateral_shift_m, half_width_m)
    if measured is not None:
        blocks, distance = measured
        if not blocks:
            return False, None, motion
        return True, BLOCKED if distance is None else distance, motion
    gap, distance, _ = object_reach(item, lateral_shift_m)
    if gap > half_width_m:
        return False, None, motion
    return True, distance, motion


def profile_points(profile):
    """(x, y) of each lateral slice's nearest return, or None if unreadable.

    The slice at `index` spans [y0 + index * bin_m, y0 + (index + 1) * bin_m)
    - the same arithmetic profile_reach selects with - so its return is
    placed at the slice centre. Half a bin of lateral uncertainty is what
    the producer's own quantisation already carries.
    """
    if not isinstance(profile, dict):
        return None
    try:
        bin_m = float(profile["bin_m"])
        y0 = float(profile["y0"])
        slices = profile["min_x"]
    except (KeyError, TypeError, ValueError):
        return None
    if not isinstance(slices, list) or not slices or \
            not math.isfinite(bin_m) or bin_m <= 0.0 or not math.isfinite(y0):
        return None
    points = []
    for index, value in enumerate(slices):
        if value is None:
            continue
        try:
            x = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(x):
            return None
        points.append((max(0.0, x), y0 + (index + 0.5) * bin_m))
    return points or None


def object_points(item):
    """Where one object is, as (x, y) in the chair's frame.

    The same measurement corridor_reach already trusts for distance, kept
    two-dimensional instead of collapsed to one number. A planner choosing
    between arcs has to know which SIDE the thing is on, and a scalar
    distance cannot say - dwa_follower.obstacle_points carries what that
    cost on 2026-08-09.

    Falls back to the near face of the box, sampled across its width rather
    than at the single corner object_reach measures, where the producer
    publishes no profile. Anything unreadable returns the chair's own
    position, which rejects every arc - the same fail-closed answer
    corridor_reach gives by reporting BLOCKED.
    """
    here = [(0.0, 0.0)]
    profile = item.get("profile")
    if profile is not None:
        return profile_points(profile) or here
    box = object_box(item)
    if box is None:
        return here
    x, y, half_x, half_y = box
    near = max(0.0, x - half_x)
    return [(near, y - half_y), (near, y), (near, y + half_y)]


def corridor_obstacle_points(summary, half_width_m, lateral_shift_m=0.0,
                             max_distance_m=None):
    """Every corridor obstacle's own returns, in the chair's frame.

    WHICH objects count is unchanged - exactly what corridor_reach calls
    blocking - so a wall alongside that the guard ignores today is still
    ignored here, and a planner reading this cannot be stopped by something
    the corridor test never minded. What changes is that each of them
    arrives as geometry instead of as a distance.

    An unusable summary reports one point at the chair itself. That rejects
    every rollout, which is what the follower already did with a threat
    reported at zero distance, and it keeps a crashed producer from reading
    as clear road here as everywhere else.
    """
    if summary is None or not summary.usable:
        return [(0.0, 0.0)]
    points = []
    for item in summary.objects:
        blocks, distance, _motion = corridor_reach(
            item, lateral_shift_m, half_width_m)
        if not blocks:
            continue
        if max_distance_m is not None and distance > max_distance_m:
            continue
        points.extend(object_points(item))
    return points


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
        blocks, distance, motion = corridor_reach(
            item, lateral_shift_m, half_width_m)
        if not blocks:
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
