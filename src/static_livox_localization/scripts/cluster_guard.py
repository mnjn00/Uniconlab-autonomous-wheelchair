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

# A profile that is present but unreadable. Distinct from None, which is an
# object that never carried one, because the two have opposite answers: no
# profile falls back to the box, a broken profile blocks.
BROKEN = object()

# Spacing at which a box's near face is sampled when the producer published
# no profile. obstacle_clusters.PROFILE_BIN_M, so the fallback has the same
# resolution as the measurement it stands in for - test_object_profile pins
# the pair, and a fallback finer than the real thing would only be inventing
# detail the box never had.
BOX_SAMPLE_M = 0.2

# Objects handed to a planner as geometry, nearest first. A corridor this
# width does not hold many, and the rollout scoring is O(candidates x steps
# x points) inside a 0.1 s control period - the producer's forty-cluster
# ceiling arriving whole would be a different node.
MAX_OBSTACLE_OBJECTS = 4
# The producer's label for a person. Compared rather than enumerated,
# because a label this code does not recognise must not silently become
# something it is willing to drive around.
PERSON_LABEL = "person"


class Threat(object):
    """The nearest thing in the corridor, and whether it is going to move."""

    def __init__(self, distance_m, motion, label="", lateral_m=None):
        self.distance_m = distance_m
        self.motion = motion
        self.label = label
        # Signed offset from the corridor centreline, chair frame, left
        # positive. None when the producer did not give a parseable box.
        #
        # Kept because dropping it makes every threat frontal. A planner
        # given only a distance can do nothing but place the object dead
        # ahead, so a wall 0.70 m away on the inside of a bend became a
        # phantom in the middle of the corridor and killed every candidate
        # rollout - 211 consecutive DWA_OBSTACLE holds on 2026-08-09 at a
        # station 0.3 m wide, where passing it on the far side was the whole
        # manoeuvre available.
        self.lateral_m = lateral_m

    @property
    def parked(self):
        """True only when the producer has watched it stand still.

        UNKNOWN is deliberately not parked: it is what a track looks like
        before it has been seen long enough, and before that the honest
        answer is that this could be someone about to step out.
        """
        return self.motion == STATIC

    @property
    def is_person(self):
        """A person, however still they are standing.

        The label is the producer's, and anything it cannot name is not
        given the benefit of the doubt in the other direction either: this
        only ever adds caution, never removes it.
        """
        return str(self.label).strip().lower() == PERSON_LABEL


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


def parsed_profile(item):
    """(bin_m, y0, slices) for a present, structurally sound profile.

    None when the object carries no profile at all - that is the fallback
    to its box. BROKEN when a profile is present but its envelope does not
    parse, which is NOT a fallback: a producer emitting a broken profile is
    one whose box is no more trustworthy.

    Individual slice values are left unread here on purpose. profile_reach
    never looked at a slice outside the corridor it was asked about, so a
    broken value out there has never blocked, and parsing eagerly to share
    this would quietly make it start.
    """
    profile = item.get("profile")
    if profile is None:
        return None
    if not isinstance(profile, dict):
        return BROKEN
    try:
        bin_m = float(profile["bin_m"])
        y0 = float(profile["y0"])
        slices = profile["min_x"]
    except (KeyError, TypeError, ValueError):
        return BROKEN
    if not isinstance(slices, list) or not slices or \
            not math.isfinite(bin_m) or bin_m <= 0.0 or not math.isfinite(y0):
        return BROKEN
    return bin_m, y0, slices


def slices_in(profile, low, high):
    """(lateral centre, forward distance) per slice overlapping [low, high].

    Raises ValueError for a slice the producer wrote unreadably, which every
    caller turns into a block at zero. A slice counts when it overlaps the
    window at all, so a return sitting just inside a slice that straddles
    the corridor edge is never missed.
    """
    bin_m, y0, slices = profile
    for index, value in enumerate(slices):
        if value is None:
            continue
        if y0 + (index + 1) * bin_m <= low or y0 + index * bin_m >= high:
            continue
        x = float(value)
        if not math.isfinite(x):
            raise ValueError("profile slice %d is not finite" % index)
        yield y0 + (index + 0.5) * bin_m, max(0.0, x)


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
    profile = parsed_profile(item)
    if profile is None or profile is BROKEN:
        return None if profile is None else (True, BLOCKED)
    try:
        reach = [x for _, x in slices_in(
            profile, lateral_shift_m - half_width_m,
            lateral_shift_m + half_width_m)]
    except (TypeError, ValueError):
        return True, BLOCKED
    if not reach:
        return False, None
    return True, min(reach)


def object_points(item, lateral_shift_m, half_width_m):
    """(blocks, points) for one object: where its returns actually are.

    points are ``(forward, left)`` in the chair frame, one per lateral slice
    the object occupies - the same measurement profile_reach takes the
    minimum of, handed over whole instead. Absolute chair-frame offsets, not
    the corridor-relative ``Threat.lateral_m`` beside them, because a
    consumer placing them in the map needs where they are and not where they
    are relative to a lane.

    A planner given one distance can only put the object dead ahead, and a
    planner given one point can only put it at one place. Neither is what a
    wall is. On 2026-07-31 a wall crossing the scan diagonally spanned the
    corridor for metres while its box reported a single near face 0.69 m
    away that had no returns behind it; measuring from the returns fixed the
    distance, and this fixes the shape, which is the half that was left. An
    arc scored against one point can pass 0.41 m from it and be admitted
    while driving through the rest of the wall.

    Fails closed exactly as corridor_reach does: an object whose box does not
    parse, or whose profile is present but unreadable, blocks at zero range
    on the centreline - a point no candidate can clear - rather than being
    skipped. Where there is no profile the box's near face is sampled across
    its own width, which is everything the box actually asserts.
    """
    box = object_box(item)
    if box is None:
        return True, [(BLOCKED, lateral_shift_m)]
    low = lateral_shift_m - half_width_m
    high = lateral_shift_m + half_width_m
    profile = parsed_profile(item)
    if profile is BROKEN:
        return True, [(BLOCKED, lateral_shift_m)]
    if profile is not None:
        try:
            points = [(x, y) for y, x in slices_in(profile, low, high)]
        except (TypeError, ValueError):
            return True, [(BLOCKED, lateral_shift_m)]
        return (True, points) if points else (False, [])
    x, y, half_x, half_y = box
    if abs(y - lateral_shift_m) - half_y > half_width_m:
        return False, []
    near = max(0.0, x - half_x)
    edges = (max(y - half_y, low), min(y + half_y, high))
    span = edges[1] - edges[0]
    count = max(int(math.ceil(span / BOX_SAMPLE_M)), 1)
    return True, [(near, edges[0] + span * (k + 0.5) / count)
                  for k in range(count)]


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


def nearest_threat(summary, half_width_m, lateral_shift_m=0.0,
                   labels=None):
    """The nearest object overlapping the corridor, or None.

    None means nothing is in the way. It never means "could not tell": an
    unusable summary reports a blocking threat, so a caller treating None
    as clear road is right to.
    """
    if not summary.usable:
        return Threat(BLOCKED, MOVING, summary.status or "unusable")
    wanted = None if labels is None else {
        str(label).strip().lower() for label in labels}
    nearest = None
    for item in summary.objects:
        label = str(item.get("class", "")).strip().lower()
        if wanted is not None and label not in wanted:
            continue
        blocks, distance, motion = corridor_reach(
            item, lateral_shift_m, half_width_m)
        if not blocks:
            continue
        if nearest is None or distance < nearest.distance_m:
            box = object_box(item)
            lateral = None if box is None else float(box[1]) - lateral_shift_m
            nearest = Threat(distance, motion, str(item.get("class", "")),
                             lateral_m=lateral)
    return nearest


def corridor_obstacle_points(summary, half_width_m, lateral_shift_m=0.0,
                             max_distance_m=None,
                             max_objects=MAX_OBSTACLE_OBJECTS):
    """(blocks, points) for everything in the way, as geometry.

    The planner-facing companion to nearest_threat. That one answers "is
    there something and how far", which is what a stopping radius needs;
    this answers "and what shape is it", which is what a planner choosing
    between arcs around it needs, and the two are not the same question.

    Several objects, not just the nearest. The single-object argument holds
    for a distance - a second object behind the first constrains no
    candidate the first already kills - and stops holding for a shape: an
    object off to one side kills the arcs on that side only, and what
    forbids the other side is a different object the nearest one never
    covered.

    Never more than max_objects of them, nearest first. Scoring is O(
    candidates x steps x points) in a 0.1 s period, and the producer will
    publish up to forty clusters.

    An unusable summary blocks at zero on the centreline, so a caller that
    treats an empty point list as clear road is right to.
    """
    if not summary.usable:
        return True, [(BLOCKED, lateral_shift_m)]
    found = []
    for item in summary.objects:
        blocks, points = object_points(item, lateral_shift_m, half_width_m)
        if not blocks or not points:
            continue
        nearest = min(x for x, _ in points)
        if max_distance_m is not None and nearest > float(max_distance_m):
            continue
        found.append((nearest, points))
    found.sort(key=lambda entry: entry[0])
    if max_objects is not None:
        found = found[:int(max_objects)]
    if not found:
        return False, []
    return True, [point for _, points in found for point in points]


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

    A person is treated like any other object only after the tracker has
    positively watched them stand still. MOVING and UNKNOWN people are
    still waited out; a predicted-only track is UNKNOWN by construction.
    This permits a path around a stationary pedestrian without ever turning
    a detector dropout into permission to move.

    blocked_for_s is the fallback for sources that carry no identity. A
    raw-scan return is UNKNOWN forever, so standing in the way is the only
    evidence of parkedness it can ever offer - but it never overrules a
    tracker that says the thing is moving.
    """
    if threat is None:
        return CLEAR
    if threat.is_person:
        if threat.parked and threat.distance_m < plan_ahead_m:
            return GO_ROUND
        return WAIT if blocking else CLEAR
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


# Preference order for a step around a parked object. Sizes are what the
# corridor is asked for; bypass_offsets_for_room decides what it can give.
BYPASS_OFFSETS = (0.6, -0.6, 1.0, -1.0)
# The largest step this will ever take, and the room it keeps between the
# offset chair's outer edge and the band edge.
#
# The ladder above is a preference order, not a set of achievable offsets.
# On 2026-08-16 the chair took a 0.60 m step in a corridor whose band was
# 1.70 m wide - 0.85 m either side of the line - and the operator stopped
# it. The step was legal: band.contains() tests the centre point, and an
# edge with no drop behind it only insets EDGE_MARGIN, by design, because
# that is what buys room to pass obstacles at all. But the chair is 0.70 m
# wide, so 0.60 m off the line put its outer edge 0.275 m past the band.
# Legal and drivable are not the same number, and the ladder could not tell
# the difference because it never asked how much room was there.
BYPASS_OFFSET_MAX_M = 1.0
BYPASS_EDGE_KEEP_M = 0.25
# Below this the chair has not gone round anything, it has only drifted,
# and it still pays the whole cost of leaving the line.
BYPASS_OFFSET_MIN_M = 0.30
# Where the step is measured and checked. One list so the offset that gets
# sized and the offset that gets approved are talking about the same ground.
BYPASS_PROBE_AHEAD_M = (0.5, 1.5, 2.5, 3.5)


def bypass_offsets_for_room(left_room, right_room):
    """Offsets worth trying, given the room each side actually has.

    BYPASS_OFFSETS is the preference order; this is what the corridor can
    give. An entry survives at its own size when it fits, and is cut down
    to the side's room when it does not, so a narrow corridor is offered a
    step sized to it instead of a 0.60 m step admitted because the centre
    point happened to land inside the band. A side with less room than
    BYPASS_OFFSET_MIN_M is dropped: there is no step there worth the cost
    of leaving the line.

    ROS-free so it can be tested without a running follower.
    """
    room = {1.0: min(left_room, BYPASS_OFFSET_MAX_M),
            -1.0: min(right_room, BYPASS_OFFSET_MAX_M)}
    offsets = []
    for wanted in BYPASS_OFFSETS:
        side = 1.0 if wanted > 0 else -1.0
        allowed = room[side]
        if allowed < BYPASS_OFFSET_MIN_M:
            continue
        value = side * min(abs(wanted), allowed)
        if not any(abs(value - seen) < 1e-6 for seen in offsets):
            offsets.append(value)
    return offsets
