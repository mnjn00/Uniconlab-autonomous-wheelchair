#!/usr/bin/env python3
"""Estimate the rotation between the VN-100 and the Livox internal IMU.

Both units are bolted to the same rigid frame, so their angular-rate
vectors are the same physical vector expressed in two frames:

    w_livox = R * w_vn

R is recovered with Kabsch/SVD over the samples where the chair is
actually rotating (still samples carry no directional information and
only add noise). Because FAST-LIO's Livox extrinsic_R is identity, the
R solved here IS the lidar<-VN rotation.

Gravity from the still segment gives an independent tilt cross-check:
it constrains 2 of the 3 axes, so a large disagreement means the gyro
fit is wrong rather than merely noisy.
"""

import sys

import numpy as np
import rosbag

VN_TOPIC = "/vectornav/IMU"
LIVOX_TOPIC = "/livox/imu"
MOVING_RATE_RAD_S = 0.15   # below this the sample tells us nothing
STILL_RATE_RAD_S = 0.02
MAX_PAIR_DT_S = 0.006      # 200 Hz on both -> half a sample period


def read_bag(path):
    vn, livox = [], []
    with rosbag.Bag(path) as bag:
        for topic, msg, _ in bag.read_messages(topics=[VN_TOPIC, LIVOX_TOPIC]):
            t = msg.header.stamp.to_sec()
            w = (msg.angular_velocity.x, msg.angular_velocity.y,
                 msg.angular_velocity.z)
            a = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                 msg.linear_acceleration.z)
            (vn if topic == VN_TOPIC else livox).append((t, w, a))
    return (np.array([r[0] for r in vn]),
            np.array([r[1] for r in vn]),
            np.array([r[2] for r in vn]),
            np.array([r[0] for r in livox]),
            np.array([r[1] for r in livox]),
            np.array([r[2] for r in livox]))


def pair_nearest(t_ref, t_other):
    """Nearest-neighbour time association, keeping only tight pairs."""
    idx = np.searchsorted(t_other, t_ref)
    idx = np.clip(idx, 1, len(t_other) - 1)
    left, right = t_other[idx - 1], t_other[idx]
    take_left = (t_ref - left) < (right - t_ref)
    matched = np.where(take_left, idx - 1, idx)
    dt = np.abs(t_other[matched] - t_ref)
    good = dt < MAX_PAIR_DT_S
    return matched, good


def kabsch(src, dst):
    """Rotation R minimising |dst - R @ src|, no scaling, det(R) = +1."""
    H = src.T @ dst
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1.0, 1.0, d]) @ U.T


def rot_to_rpy_deg(R):
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def main(path):
    t_vn, w_vn, a_vn, t_lx, w_lx, a_lx = read_bag(path)
    if not len(t_vn) or not len(t_lx):
        print("ERROR: one of the IMU topics is empty")
        return 1
    print("samples: vn=%d livox=%d  span=%.1fs"
          % (len(t_vn), len(t_lx), t_vn[-1] - t_vn[0]))

    matched, good = pair_nearest(t_vn, t_lx)
    w_vn_p, w_lx_p = w_vn[good], w_lx[matched[good]]
    a_vn_p, a_lx_p = a_vn[good], a_lx[matched[good]]
    print("time-aligned pairs: %d" % len(w_vn_p))

    speed = np.linalg.norm(w_vn_p, axis=1)
    moving = speed > MOVING_RATE_RAD_S
    print("rotating samples: %d (max rate %.2f rad/s)"
          % (moving.sum(), speed.max()))
    if moving.sum() < 200:
        print("ERROR: not enough rotation - excite all three axes and rerun")
        return 2

    # Axis coverage: without motion about all three axes the fit is
    # under-determined and would silently return a plausible-looking R.
    cover = np.abs(w_vn_p[moving]).max(axis=0)
    print("per-axis peak rate (vn): x=%.2f y=%.2f z=%.2f rad/s" % tuple(cover))
    if cover.min() < 0.20:
        print("WARNING: one axis barely moved - yaw/roll may be unreliable")

    R = kabsch(w_vn_p[moving], w_lx_p[moving])
    resid = w_lx_p[moving] - w_vn_p[moving] @ R.T
    rms = np.sqrt((resid ** 2).sum(axis=1).mean())
    scale = np.linalg.norm(w_lx_p[moving]) / np.linalg.norm(w_vn_p[moving])
    print("\ngyro fit: rms residual %.4f rad/s, magnitude ratio %.4f"
          % (rms, scale))

    still = speed < STILL_RATE_RAD_S
    if still.sum() > 100:
        g_vn = a_vn_p[still].mean(axis=0)
        g_lx = a_lx_p[still].mean(axis=0)
        g_vn /= np.linalg.norm(g_vn)
        g_lx /= np.linalg.norm(g_lx)
        err = np.degrees(np.arccos(np.clip((R @ g_vn) @ g_lx, -1, 1)))
        print("gravity cross-check: %.2f deg disagreement (%d still samples)"
              % (err, still.sum()))
    else:
        print("gravity cross-check: skipped (no still segment)")

    rpy = rot_to_rpy_deg(R)
    print("\nR (lidar <- vectornav), roll/pitch/yaw = "
          "%.2f / %.2f / %.2f deg" % tuple(rpy))
    print("extrinsic_R: [ %.6f, %.6f, %.6f,\n"
          "               %.6f, %.6f, %.6f,\n"
          "               %.6f, %.6f, %.6f ]"
          % tuple(R.flatten()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
