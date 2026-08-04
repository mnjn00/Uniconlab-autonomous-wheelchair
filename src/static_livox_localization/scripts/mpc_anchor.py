"""Where the MPC starts each cycle from, and why it is not the raw pose.

Section 7 of docs/mpc_follower_design.md makes this mandatory rather than a
later tune, on a measurement: re-anchoring the horizon on the raw localiser
pose produces 5.0 yaw-command reversals per metre with 2 cm of lateral
jitter, against 1.3/m through an EMA at gain 0.4, with no tracking
regression either way (lateral RMS 13 mm). At 5 cm of jitter the raw anchor
stalls the run outright. Nothing oscillates in the classical sense - the
limit-cycle screen and the lateral spectrum stay clean - but a controller
that reverses its steering five times a metre is one that feels like
hunting to whoever is sitting in the chair.

Two separate things are blended, and the split matters:

  position and heading  come from localisation, low-passed, because that is
                        the only source that knows where the MAP is;
  v and w               come from wheel odometry, which is smooth, local and
                        exactly what the actuator just did - integrating the
                        localiser to get them is what injects the jitter
                        into the derivative in the first place.

Everything here is ROS-free arithmetic so it can be argued with at a desk.
"""

import math

import numpy as np

# Validated in the 2026-08-04 noise sweep. Higher trusts the new pose more
# (jitterier), lower lags the map (slower to correct a real offset).
DEFAULT_GAIN = 0.4
# A pose this far from the anchor is not jitter, it is a jump - a relocalise,
# a seed, or a shove. Blending through it would crawl toward the truth over
# several cycles while planning from a place the chair is not.
JUMP_M = 0.5
JUMP_RAD = math.radians(25.0)
# Beyond this the anchor is stale and blending it forward is invention.
MAX_GAP_S = 0.5


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def blend_angle(previous, new, gain):
    """EMA on the circle - a linear blend flips at the +-pi seam."""
    return wrap_angle(previous + gain * wrap_angle(new - previous))


class StateAnchor:
    """The smoothed (X, Y, theta, v, w) the solver plans from."""

    def __init__(self, gain=DEFAULT_GAIN, jump_m=JUMP_M, jump_rad=JUMP_RAD,
                 max_gap_s=MAX_GAP_S):
        self.gain = float(gain)
        self.jump_m = float(jump_m)
        self.jump_rad = float(jump_rad)
        self.max_gap_s = float(max_gap_s)
        self.state = None
        self.stamp_s = None
        self.jumps = 0

    def reset(self, reason=""):
        self.state = None
        self.stamp_s = None

    def update(self, pose_xy, pose_yaw, odom_v, odom_w, stamp_s):
        """Fold one localisation pose and one odometry sample into the anchor.

        Returns the anchor state (5,). The first sample, a jump, and a stale
        gap all snap rather than blend: in each case the previous anchor is
        not evidence about where the chair is now, and averaging it in would
        plan from a place that never existed.
        """
        pose_xy = np.asarray(pose_xy, dtype=float)
        fresh = np.array([pose_xy[0], pose_xy[1], wrap_angle(float(pose_yaw)),
                          float(odom_v), float(odom_w)])
        snap = self.state is None
        if not snap and self.stamp_s is not None \
                and stamp_s - self.stamp_s > self.max_gap_s:
            snap = True
        if not snap:
            moved = float(np.linalg.norm(pose_xy - self.state[:2]))
            turned = abs(wrap_angle(fresh[2] - self.state[2]))
            if moved > self.jump_m or turned > self.jump_rad:
                snap = True
                self.jumps += 1
        if snap:
            self.state = fresh
        else:
            g = self.gain
            self.state = np.array([
                self.state[0] + g * (fresh[0] - self.state[0]),
                self.state[1] + g * (fresh[1] - self.state[1]),
                blend_angle(self.state[2], fresh[2], g),
                # v and w are taken outright: wheel odometry is already the
                # smooth, local truth about what the actuator is doing, and
                # low-passing it would only add lag to the one signal that
                # does not need it.
                fresh[3],
                fresh[4],
            ])
        self.stamp_s = float(stamp_s)
        return self.state.copy()
