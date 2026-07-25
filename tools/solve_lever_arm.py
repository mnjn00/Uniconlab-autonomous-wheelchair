#!/usr/bin/env python3
"""Estimate the lever arm between the VN-100 and the Livox internal IMU.

Two accelerometers on one rigid body see the same motion plus the
rotational terms of their offset r (Livox position relative to VN):

    a_livox - R @ a_vn = (skew(w)^2 + skew(alpha)) @ r

so r falls out of a linear least squares over the rotating samples.
alpha comes from differentiating the gyro, which is noisy, so the rate
is smoothed first and the conditioning of the normal matrix is reported
- a poorly excited recording yields a large r with no warning otherwise.

Units differ between the two: the Livox driver reports acceleration in
g, the VN-100 in m/s^2. Both are rescaled by their own still-segment
magnitude, which removes the unit difference and any scale-factor error
at the same time.
"""

import sys

import numpy as np
import rosbag

VN_TOPIC = "/vectornav/IMU"
LIVOX_TOPIC = "/livox/imu"
MAX_PAIR_DT_S = 0.006
STILL_RATE_RAD_S = 0.02
MOVING_RATE_RAD_S = 0.30    # lever-arm terms scale with w^2, need real motion
SMOOTH_N = 9                # ~45 ms at 200 Hz
G = 9.80665
# FAST-LIO's extrinsic_T is the LIDAR origin expressed in the IMU frame
# (Lidar_T_wrt_IMU), not the IMU position. The value below is the working
# Livox entry; it equals the negated MID360 datasheet IMU offset of
# (11.0, 23.29, -44.12) mm, which confirms the direction.
T_LIVOXIMU_LIDAR = np.array([-0.011, -0.02329, 0.04412])


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def moving_average(x, n):
    kernel = np.ones(n) / n
    return np.stack([np.convolve(x[:, i], kernel, mode="same")
                     for i in range(x.shape[1])], axis=1)


def read_bag(path):
    vn, livox = [], []
    with rosbag.Bag(path) as bag:
        for topic, msg, _ in bag.read_messages(topics=[VN_TOPIC, LIVOX_TOPIC]):
            row = (msg.header.stamp.to_sec(),
                   (msg.angular_velocity.x, msg.angular_velocity.y,
                    msg.angular_velocity.z),
                   (msg.linear_acceleration.x, msg.linear_acceleration.y,
                    msg.linear_acceleration.z))
            (vn if topic == VN_TOPIC else livox).append(row)
    unpack = lambda rows: (np.array([r[0] for r in rows]),
                           np.array([r[1] for r in rows]),
                           np.array([r[2] for r in rows]))
    return unpack(vn) + unpack(livox)


def pair_nearest(t_ref, t_other):
    idx = np.clip(np.searchsorted(t_other, t_ref), 1, len(t_other) - 1)
    take_left = (t_ref - t_other[idx - 1]) < (t_other[idx] - t_ref)
    matched = np.where(take_left, idx - 1, idx)
    return matched, np.abs(t_other[matched] - t_ref) < MAX_PAIR_DT_S


def main(bag_path, R_flat):
    R = np.array(R_flat, dtype=float).reshape(3, 3)
    t_vn, w_vn, a_vn, t_lx, w_lx, a_lx = read_bag(bag_path)
    matched, good = pair_nearest(t_vn, t_lx)
    t = t_vn[good]
    w = w_vn[good]
    a_vn_p, a_lx_p = a_vn[good], a_lx[matched[good]]

    still = np.linalg.norm(w, axis=1) < STILL_RATE_RAD_S
    if still.sum() < 100:
        print("ERROR: no still segment to normalise units against")
        return 2
    vn_scale = G / np.linalg.norm(a_vn_p[still].mean(axis=0))
    lx_scale = G / np.linalg.norm(a_lx_p[still].mean(axis=0))
    print("unit normalisation: vn x%.4f  livox x%.4f" % (vn_scale, lx_scale))

    w_s = moving_average(w, SMOOTH_N)
    dt = np.gradient(t)
    alpha = np.stack([np.gradient(w_s[:, i]) / dt for i in range(3)], axis=1)

    delta = a_lx_p * lx_scale - (R @ (a_vn_p * vn_scale).T).T

    use = np.linalg.norm(w, axis=1) > MOVING_RATE_RAD_S
    print("samples used: %d of %d" % (use.sum(), len(t)))
    if use.sum() < 300:
        print("ERROR: too little rotation for lever-arm observability")
        return 3

    rows, rhs = [], []
    for i in np.flatnonzero(use):
        Om = skew(w_s[i])
        rows.append(Om @ Om + skew(alpha[i]))
        rhs.append(delta[i])
    M = np.vstack(rows)
    b = np.concatenate(rhs)

    r, *_ = np.linalg.lstsq(M, b, rcond=None)
    resid = b - M @ r
    rms = np.sqrt((resid ** 2).mean())
    sv = np.linalg.svd(M, compute_uv=False)
    cond = sv[0] / sv[-1]

    print("\nr = livox IMU minus VN position, lidar axes:"
          " [%.4f, %.4f, %.4f] m" % tuple(r))
    print("residual rms %.3f m/s^2, condition number %.1f" % (rms, cond))

    # lidar - VN = (lidar - livoxIMU) + (livoxIMU - VN), in lidar axes,
    # then rotated into VN axes because FAST-LIO wants it in the IMU frame.
    t_lidar_in_vn = R.T @ (T_LIVOXIMU_LIDAR + r)
    vn_in_lidar = -(T_LIVOXIMU_LIDAR + r)
    print("\nphysical layout: VN sits %.1f cm behind, %.1f cm to the side, "
          "%.1f cm below the lidar"
          % (-vn_in_lidar[0] * 100, vn_in_lidar[1] * 100,
             -vn_in_lidar[2] * 100))

    print("\n--- FAST-LIO entries (Lidar_wrt_IMU) ---")
    print("extrinsic_T: [ %.5f, %.5f, %.5f ]" % tuple(t_lidar_in_vn))
    Rt = R.T
    print("extrinsic_R: [ %.6f, %.6f, %.6f,\n"
          "               %.6f, %.6f, %.6f,\n"
          "               %.6f, %.6f, %.6f ]" % tuple(Rt.flatten()))
    if cond > 200 or rms > 1.0:
        print("\nWARNING: weak fit - treat this as a rough number and "
              "cross-check against a tape measure")
    return 0


if __name__ == "__main__":
    R_SOLVED = [0.998785, 0.048776, 0.006981,
                -0.048796, 0.998805, 0.002716,
                -0.006840, -0.003054, 0.999972]
    sys.exit(main(sys.argv[1], R_SOLVED))
