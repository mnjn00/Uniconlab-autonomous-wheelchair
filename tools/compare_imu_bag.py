#!/usr/bin/env python3
"""Compare two co-mounted IMUs sample-for-sample from a debug bag.

The 2026-07-27 rollback (999028a) was performed so a drive could be recorded
with the MID360's internal IMU driving FAST-LIO while the VN-100 runs beside
it, influencing nothing. Both sensors then see the exact same motion, which
is the only honest way to ask whether the VN-100 is the better sensor for
this chair. This tool is the repeatable readout of that question, so the
next drive is one command rather than a re-derivation.

The report covers, in order:
  1. topic health       counts, rates, header-stamp offset between the IMUs
  2. accel units        mean |accel| while stationary; the Livox IMU reports
                        in g and the VN-100 in m/s^2, and feeding one to a
                        config tuned for the other silently breaks tilt
  3. stationary blocks  gyro bias/noise, integrated raw drift, gravity tilt
                        per block, per IMU; blocks come from the wheel
                        odometry speed so they are physical, not assumed
  4. bias drift         bias change between the first and last block, with a
                        flag when the later block is noisy on BOTH sensors,
                        which means real vibration rather than drift
  5. moving agreement   gyro difference after yaw alignment, and each IMU's
                        yaw rate checked against wheel odometry as an
                        independent third source - when both IMUs disagree
                        with the wheels by the same amount, the wheels are
                        slipping and the IMUs are not the problem
  6. saturation         max |gyro| and |accel| seen, to rule out clipping

Read-only. Pure python; install the reader offline, never on the NUC:
    python3 -m pip install rosbags numpy

Usage:
    python3 tools/compare_imu_bag.py /path/to/debug.bag
    python3 tools/compare_imu_bag.py debug.bag --yaw-deg -2.80

The default yaw is the rotation measured between these two units (SVD over
150 s of shared gyro, residual 0.0144 rad/s); pass 0 to compare unaligned.
"""

import argparse
import sys

import numpy as np

DEG_PER_RAD = 180.0 / np.pi


def yaw_rotation(yaw_deg):
    """Rotation that expresses IMU-B vectors in IMU-A's frame for a pure yaw."""
    a = yaw_deg / DEG_PER_RAD
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def stationary_blocks(t, speed, threshold, min_duration_s):
    """Index ranges [start, end) where |speed| stays below threshold long enough."""
    mask = np.abs(speed) < threshold
    if mask.size == 0:
        return []
    edges = np.diff(mask.astype(int))
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)
    if mask[0]:
        starts.insert(0, 0)
    if mask[-1]:
        ends.append(len(mask))
    return [(s, e) for s, e in zip(starts, ends) if t[e - 1] - t[s] >= min_duration_s]


def gyro_stats(t, gyro, start, end):
    """Bias, noise and integrated rotation of one gyro over t[start:end]."""
    seg_t = t[start:end]
    seg_g = gyro[start:end]
    bias = seg_g.mean(axis=0)
    noise = seg_g.std(axis=0)
    integrated = np.sum(seg_g[:-1] * np.diff(seg_t)[:, None], axis=0)
    return {"bias": bias, "noise": noise, "integrated": integrated}


def gravity_and_tilt(accel_mean):
    """Gravity magnitude and the pitch/roll implied by the mean accel vector."""
    norm = float(np.linalg.norm(accel_mean))
    pitch = np.arctan2(-accel_mean[0], np.hypot(accel_mean[1], accel_mean[2]))
    roll = np.arctan2(accel_mean[1], accel_mean[2])
    return norm, pitch * DEG_PER_RAD, roll * DEG_PER_RAD


def accel_scale_label(mean_norm):
    """What unit an accelerometer is reporting in, from its stationary |g|."""
    if 0.8 < mean_norm < 1.2:
        return "g"
    if 9.0 < mean_norm < 10.6:
        return "m/s^2"
    return "unknown"


def align_onto(t_ref, t_src, values_src):
    """Resample values_src (N,3) onto t_ref by per-axis linear interpolation."""
    return np.column_stack([np.interp(t_ref, t_src, values_src[:, i]) for i in range(3)])


def rms_per_axis(diff):
    return np.sqrt((diff ** 2).mean(axis=0))


def correlation(a, b):
    if a.std() == 0.0 or b.std() == 0.0:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def read_bag(path, imu_a_topic, imu_b_topic, odom_topic):
    """Load the three series needed for the comparison. Lazy-imports rosbags."""
    from pathlib import Path

    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS1_NOETIC)

    def series(reader, topic, kind):
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            present = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                "error: topic %s not in the bag.\n  topics present: %s"
                % (topic, ", ".join(present))
            )
        t, first, second = [], [], []
        for conn, _stamp, raw in reader.messages(connections=conns):
            msg = typestore.deserialize_ros1(raw, conn.msgtype)
            t.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            if kind == "imu":
                first.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                second.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            else:
                first.append(msg.twist.twist.linear.x)
                second.append(msg.twist.twist.angular.z)
        arr1 = np.array(first)
        arr2 = np.array(second)
        return np.array(t), arr1, arr2

    with Reader(Path(path)) as reader:
        ta, gyro_a, accel_a = series(reader, imu_a_topic, "imu")
        tb, gyro_b, accel_b = series(reader, imu_b_topic, "imu")
        to, vx, wz = series(reader, odom_topic, "odom")
    return {
        "a": (ta, gyro_a, accel_a),
        "b": (tb, gyro_b, accel_b),
        "odom": (to, vx, wz),
    }


def report(data, imu_a_topic, imu_b_topic, yaw_deg, speed_threshold, min_block_s):
    ta, gyro_a, accel_a = data["a"]
    tb, gyro_b, accel_b = data["b"]
    to, vx, wz = data["odom"]

    print("=" * 78)
    print("1. TOPIC HEALTH")
    print("  %-18s %7d msgs  %6.1f Hz" % (imu_a_topic, len(ta), len(ta) / (ta[-1] - ta[0])))
    print("  %-18s %7d msgs  %6.1f Hz" % (imu_b_topic, len(tb), len(tb) / (tb[-1] - tb[0])))
    lo, hi = max(ta[0], tb[0]), min(ta[-1], tb[-1])
    if hi <= lo:
        raise SystemExit("error: the two IMU topics do not overlap in time.")
    sel = (ta >= lo) & (ta <= hi)
    t, g_a, a_a = ta[sel], gyro_a[sel], accel_a[sel]
    g_b = align_onto(t, tb, gyro_b)
    a_b = align_onto(t, tb, accel_b)
    rot = yaw_rotation(yaw_deg)
    g_b_al = g_b @ rot.T
    a_b_al = a_b @ rot.T
    nb = np.searchsorted(tb, t).clip(1, len(tb) - 1)
    print("  header-stamp offset (%s minus %s), median: %+.2f ms"
          % (imu_a_topic, imu_b_topic, np.median(t - tb[nb]) * 1e3))
    print("  yaw alignment applied to %s: %+.2f deg" % (imu_b_topic, yaw_deg))

    vx_i = np.interp(t, to, vx)
    wz_i = np.interp(t, to, wz)
    blocks = stationary_blocks(t, vx_i, speed_threshold, min_block_s)
    moving = np.abs(vx_i) >= speed_threshold
    print("  stationary: %.1f%% of samples, moving: %.1f%%"
          % (100.0 * (~moving).mean(), 100.0 * moving.mean()))
    print("  stationary blocks >= %.0f s (bag-relative seconds): %s"
          % (min_block_s, [(round(float(t[s] - t[0]), 1), round(float(t[e - 1] - t[0]), 1)) for s, e in blocks]))

    print("=" * 78)
    print("2. ACCEL UNITS (stationary mean |accel|)")
    stat = ~moving
    if stat.any():
        for name, a in ((imu_a_topic, a_a), (imu_b_topic, a_b_al)):
            norm = float(np.linalg.norm(a[stat].mean(axis=0)))
            print("  %-18s |g| = %8.4f  ->  reporting in %s" % (name, norm, accel_scale_label(norm)))
    else:
        print("  no stationary samples; cannot tell the units apart")

    print("=" * 78)
    print("3. STATIONARY BLOCKS")
    block_noise_z = {}
    for i, (s, e) in enumerate(blocks):
        print("  block %d: %.1f s" % (i + 1, t[e - 1] - t[s]))
        for name, g, a in ((imu_a_topic, g_a, a_a), (imu_b_topic, g_b_al, a_b_al)):
            st = gyro_stats(t, g, s, e)
            bias, noise = st["bias"], st["noise"]
            dt = float(np.median(np.diff(t[s:e])))
            arw_z = noise[2] * DEG_PER_RAD * np.sqrt(dt) * 60.0
            block_noise_z[(i, name)] = noise[2]
            print("    %-18s bias %s deg/s  (z = %+.1f deg/h)"
                  % (name, np.array2string(bias * DEG_PER_RAD, precision=4, suppress_small=True),
                     bias[2] * DEG_PER_RAD * 3600.0))
            print("    %-18s noise %s deg/s  (ARW z ~ %.2f deg/sqrt(h))"
                  % ("", np.array2string(noise * DEG_PER_RAD, precision=4, suppress_small=True), arw_z))
            print("    %-18s integrated over block %s deg"
                  % ("", np.array2string(st["integrated"] * DEG_PER_RAD, precision=3, suppress_small=True)))
            norm, pitch, roll = gravity_and_tilt(a[s:e].mean(axis=0))
            print("    %-18s gravity |g| %.4f, tilt pitch %+.3f roll %+.3f deg" % ("", norm, pitch, roll))

    if len(blocks) >= 2:
        print("=" * 78)
        print("4. BIAS DRIFT (first block -> last block)")
        s0, e0 = blocks[0]
        s1, e1 = blocks[-1]
        span = t[s1] - t[e0 - 1]
        for name, g in ((imu_a_topic, g_a), (imu_b_topic, g_b_al)):
            d0 = gyro_stats(t, g, s0, e0)["bias"]
            d1 = gyro_stats(t, g, s1, e1)["bias"]
            drift = (d1 - d0) * DEG_PER_RAD
            print("  %-18s %s deg/s  (z = %+.1f deg/h over %.0f s)"
                  % (name, np.array2string(drift, precision=4, suppress_small=True),
                     drift[2] * 3600.0, span))
        ratio_a = block_noise_z[(len(blocks) - 1, imu_a_topic)] / max(block_noise_z[(0, imu_a_topic)], 1e-12)
        ratio_b = block_noise_z[(len(blocks) - 1, imu_b_topic)] / max(block_noise_z[(0, imu_b_topic)], 1e-12)
        if ratio_a > 3.0 and ratio_b > 3.0:
            print("  NOTE: the last block is %.1fx/%.1fx noisier on BOTH sensors." % (ratio_a, ratio_b))
            print("        That is physical vibration, not bias drift; the numbers")
            print("        above are contaminated. Re-record with the chair settled")
            print("        and undisturbed for the final bookend.")

    print("=" * 78)
    print("5. MOVING AGREEMENT (after yaw alignment)")
    if moving.any():
        dg = g_a[moving] - g_b_al[moving]
        print("  gyro diff RMS      %s deg/s"
              % np.array2string(rms_per_axis(dg) * DEG_PER_RAD, precision=4, suppress_small=True))
        for name, g in ((imu_a_topic, g_a), (imu_b_topic, g_b_al)):
            err = g[moving, 2] - wz_i[moving]
            print("  %-18s gz vs wheel wz  bias %+.4f  RMS %7.4f deg/s  corr %.4f"
                  % (name, err.mean() * DEG_PER_RAD,
                     np.sqrt((err ** 2).mean()) * DEG_PER_RAD, correlation(g[moving, 2], wz_i[moving])))
        da = a_a[moving] - a_b_al[moving]
        lever = float(wz_i[moving].max() ** 2 * 0.144)
        print("  accel diff RMS     %s m/s^2"
              % np.array2string(rms_per_axis(da), precision=4, suppress_small=True))
        print("  (lever-arm term omega^2*r alone can reach ~%.2f m/s^2; accel" % lever)
        print("   differences are not sensor error until that is compensated)")
    else:
        print("  no moving samples; drive the chair between the bookends")

    print("=" * 78)
    print("6. SATURATION / RANGE")
    print("  %-18s max |gyro| %8.1f deg/s   max |accel| %8.2f (raw units)"
          % (imu_a_topic, np.abs(g_a).max() * DEG_PER_RAD, np.abs(a_a).max()))
    print("  %-18s max |gyro| %8.1f deg/s   max |accel| %8.2f (raw units)"
          % (imu_b_topic, np.abs(g_b).max() * DEG_PER_RAD, np.abs(a_b).max()))
    print("=" * 78)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", help="ROS 1 bag recorded with both IMUs and the wheel odometry")
    parser.add_argument("--imu-a", default="/livox/imu", help="reference IMU topic (default %(default)s)")
    parser.add_argument("--imu-b", default="/vectornav/IMU", help="second IMU topic (default %(default)s)")
    parser.add_argument("--odom", default="/odom", help="wheel odometry topic (default %(default)s)")
    parser.add_argument("--yaw-deg", type=float, default=-2.80,
                        help="yaw of IMU B in IMU A's frame, degrees (default %(default)s, "
                             "the measured VN-100 <- MID360 extrinsic; 0 compares unaligned)")
    parser.add_argument("--speed-threshold", type=float, default=0.02,
                        help="wheel-speed below which the chair counts as stationary (default %(default)s m/s)")
    parser.add_argument("--min-block-s", type=float, default=20.0,
                        help="shortest stationary block worth reporting (default %(default)s s)")
    args = parser.parse_args(argv)

    data = read_bag(args.bag, args.imu_a, args.imu_b, args.odom)
    report(data, args.imu_a, args.imu_b, args.yaw_deg, args.speed_threshold, args.min_block_s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
