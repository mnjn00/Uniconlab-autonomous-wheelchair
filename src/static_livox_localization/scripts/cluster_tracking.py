"""Static or moving, decided by watching a cluster across frames.

A single cycle cannot tell a parked bicycle from someone about to step out:
both are a box in the corridor. The difference only exists over time, and
only in a frame that does not move with the chair - in the lidar frame a
parked car approaches at driving speed, so everything is "moving" there.
Tracking therefore runs on odom-frame positions, which is why it lives with
the producer: that node already holds the poses it motion-compensates with.

Association is nearest neighbour inside a gate, which suffices because the
objects that matter are metres apart at 5 Hz. There is no attempt at identity
through occlusion - a track that vanishes and returns is a new track, a new
track is UNKNOWN, and UNKNOWN is handled as moving. Every uncertainty here
resolves toward stopping, because the failure being guarded is driving around
something that then steps into the chair.

Displacement is measured against the oldest position still in the window, not
against the previous frame. Frame to frame it is mostly jitter - a wall's
centroid slides sideways as more of the wall becomes visible, and a vehicle's
does the same as it comes out from behind a hedge - and differencing noise
yields noise. Over a window a real walker separates from that cleanly.
"""

import math

MOVING = "moving"
STATIC = "static"
UNKNOWN = "unknown"

# A person at 1.5 m/s covers 0.3 m per 5 Hz cycle; a gate this size holds the
# association through that without reaching the next object over.
ASSOCIATION_GATE_M = 1.2
# How long an object must be watched before its stillness is believed. Under
# this it is UNKNOWN, which is handled as moving.
CONFIRM_S = 1.5
# Apparent speed below which an object is parked. Not zero: the centroid of a
# partially observed object drifts as the view opens up, and at 15 m that
# drift alone runs to a few cm per cycle.
STATIC_SPEED_MPS = 0.20
# Positions older than this leave the window. Long enough that CONFIRM_S of
# history is always available, short enough that an object which stops after
# moving is not called moving forever.
HISTORY_S = 3.0
# A track not matched for this long is gone. One missed cycle at 5 Hz is
# 0.2 s, so this tolerates three.
DROP_AFTER_S = 0.8


class Track(object):
    """One object's history in the odom frame."""

    def __init__(self, track_id, label, x, y, stamp_s):
        self.id = track_id
        self.label = label
        self.history = [(stamp_s, x, y)]

    @property
    def first_seen(self):
        return self.history[0][0]

    @property
    def last_seen(self):
        return self.history[-1][0]

    @property
    def position(self):
        return self.history[-1][1], self.history[-1][2]

    def observe(self, label, x, y, stamp_s):
        self.label = label
        self.history.append((stamp_s, x, y))
        self.history = [h for h in self.history
                        if stamp_s - h[0] <= HISTORY_S] or [(stamp_s, x, y)]

    def age_s(self, now_s):
        return now_s - self.first_seen

    def speed_mps(self):
        """Displacement from the oldest position in the window over its span.

        Straight-line, not path length: an object jittering in place has a
        long path and no displacement, and it is displacement that decides
        whether the chair can drive past where the object is now.
        """
        if len(self.history) < 2:
            return 0.0
        span = self.history[-1][0] - self.history[0][0]
        if span <= 1e-6:
            return 0.0
        moved = math.hypot(self.history[-1][1] - self.history[0][1],
                           self.history[-1][2] - self.history[0][2])
        return moved / span

    def motion(self, now_s, confirm_s=CONFIRM_S,
               static_speed_mps=STATIC_SPEED_MPS):
        if self.age_s(now_s) < confirm_s:
            return UNKNOWN
        return STATIC if self.speed_mps() < static_speed_mps else MOVING


class Tracker(object):
    """Associates detections to tracks and ages out what stops appearing."""

    def __init__(self, gate_m=ASSOCIATION_GATE_M, drop_after_s=DROP_AFTER_S):
        self.gate_m = gate_m
        self.drop_after_s = drop_after_s
        self.tracks = []
        self.next_id = 1

    def update(self, detections, stamp_s):
        """Take (x, y, label) detections in the odom frame at stamp_s.

        Returns one Track per detection, in the same order, so the caller can
        attach identity and motion to the object it already built.
        """
        self.tracks = [t for t in self.tracks
                       if stamp_s - t.last_seen <= self.drop_after_s]

        # Greedy by increasing distance rather than in detection order: a
        # detection early in the list would otherwise claim a track that sits
        # much closer to a later one, and the ordering here is by cluster
        # size, which has nothing to do with which object is which.
        pairs = []
        for d_index, (x, y, _label) in enumerate(detections):
            for t_index, track in enumerate(self.tracks):
                tx, ty = track.position
                distance = math.hypot(x - tx, y - ty)
                if distance <= self.gate_m:
                    pairs.append((distance, d_index, t_index))
        pairs.sort()

        assigned = {}
        claimed = set()
        for _distance, d_index, t_index in pairs:
            if d_index in assigned or t_index in claimed:
                continue
            assigned[d_index] = t_index
            claimed.add(t_index)

        result = []
        for d_index, (x, y, label) in enumerate(detections):
            if d_index in assigned:
                track = self.tracks[assigned[d_index]]
                track.observe(label, x, y, stamp_s)
            else:
                track = Track(self.next_id, label, x, y, stamp_s)
                self.next_id += 1
                self.tracks.append(track)
            result.append(track)
        return result
