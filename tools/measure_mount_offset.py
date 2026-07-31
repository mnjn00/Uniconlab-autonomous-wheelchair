#!/usr/bin/env python3
"""Measure where the sensor sits on the chair, from spin-in-place in a bag.

body_frame.CHAIR_CENTRE_IN_BODY_XYZ is a measurement, not a nominal figure
copied off a drawing, so this is how it was obtained and how to obtain it
again if the mount is ever moved.

The chair's wheels are symmetric about its centre, so a true spin in place
turns about that centre and the sensor traces a circle around it:

    p(t) = centre + R(yaw(t)) . r

which is linear in (centre, r) once yaw is known, so a least-squares fit over
a spin recovers r directly - the sensor's offset in the body frame. A spin
with unequal wheel speeds turns about a point displaced along the axle
instead, and shows up as a poor residual rather than a plausible wrong answer,
which is why the residual is reported and why short or dirty spins are
rejected.

Run it on a drive that begins and ends with the operator turning on the spot;
the 2026-07-27 route recording does both. Reads ROS1 bags directly, so it
needs no ROS install.

Usage: measure_mount_offset.py <bag> [topic]
       topic defaults to /fast_lio_icp/pose
"""

import math
import struct
import sys
import bz2

import numpy as np

try:
    import lz4.frame as _LZ4_FRAME
except ImportError:  # pragma: no cover - depends on the host
    _LZ4_FRAME = None

U32 = struct.Struct("<I")

# A spin has to turn far enough that the circle is well determined, and hold
# still enough that the fit is about the mount rather than about drifting.
MIN_TURN_RAD = 1.8
MAX_RESIDUAL_M = 0.030
MIN_POSES = 35


def _fields(buf):
    out = {}
    offset = 0
    while offset < len(buf):
        size = U32.unpack_from(buf, offset)[0]
        offset += 4
        item = buf[offset:offset + size]
        offset += size
        split = item.index(b"=")
        out[item[:split].decode("ascii", "replace")] = item[split + 1:]
    return out


def _records(buf, offset=0):
    while offset + 8 <= len(buf):
        size = U32.unpack_from(buf, offset)[0]
        offset += 4
        header = buf[offset:offset + size]
        offset += size
        size = U32.unpack_from(buf, offset)[0]
        offset += 4
        yield _fields(header), buf[offset:offset + size]
        offset += size


def _decompress(data, compression):
    if compression == "none":
        return data
    if compression == "bz2":
        return bz2.decompress(data)
    if compression == "lz4":
        if _LZ4_FRAME is None:
            raise SystemExit("bag is lz4 compressed; pip install lz4")
        return _LZ4_FRAME.decompress(data)
    raise SystemExit("unsupported bag compression: %s" % compression)


def _pose_rows(data, is_odometry):
    """(t, x, y, yaw) from a PoseWithCovarianceStamped or Odometry payload."""

    offset = 4
    secs, nsecs = struct.unpack_from("<II", data, offset)
    offset += 8
    frame_len = U32.unpack_from(data, offset)[0]
    offset += 4 + frame_len
    if is_odometry:
        child_len = U32.unpack_from(data, offset)[0]
        offset += 4 + child_len
    x, y, _z, qx, qy, qz, qw = struct.unpack_from("<7d", data, offset)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return secs + nsecs * 1e-9, x, y, yaw


def read_poses(path, topic):
    blob = open(path, "rb").read()
    if not blob.startswith(b"#ROSBAG V2.0"):
        raise SystemExit("not a ROS1 v2.0 bag: %s" % path)
    wanted = {}
    kind = {}
    rows = []

    def consume(header, data):
        op = header.get("op", b"\x00")[0]
        if op == 0x07:
            connection = U32.unpack(header["conn"])[0]
            name = header["topic"].decode()
            if name == topic:
                wanted[connection] = name
                kind[connection] = b"Odometry" in _fields(data).get("type", b"")
        elif op == 0x02:
            connection = U32.unpack(header["conn"])[0]
            if connection in wanted:
                rows.append(_pose_rows(data, kind[connection]))

    for header, data in _records(blob, blob.index(b"\n") + 1):
        if header.get("op", b"\x00")[0] == 0x05:
            inner = _decompress(
                data, header.get("compression", b"none").decode())
            for sub_header, sub_data in _records(inner):
                consume(sub_header, sub_data)
        else:
            consume(header, data)

    if not rows:
        raise SystemExit("no messages on %s" % topic)
    table = np.array(sorted(rows), dtype=np.float64)
    # Drop duplicate or backward stamps before differencing.
    keep = np.concatenate([[True], np.diff(table[:, 0]) > 0.02])
    table = table[keep]
    table[:, 3] = np.unwrap(table[:, 3])
    return table


def fit_spins(table):
    """Least squares for a shared r over every clean spin in the drive."""

    t, x, y, yaw = table[:, 0], table[:, 1], table[:, 2], table[:, 3]
    step = np.hypot(np.diff(x), np.diff(y))
    rate = np.diff(yaw) / np.maximum(np.diff(t), 1e-3)
    speed = step / np.maximum(np.diff(t), 1e-3)

    spans = []
    start = None
    for index in range(len(rate)):
        spinning = abs(rate[index]) > 0.25 and speed[index] < 1.0
        if spinning and start is None:
            start = index
        elif not spinning and start is not None:
            if index - start >= MIN_POSES:
                spans.append((start, index))
            start = None
    if start is not None and len(rate) - start >= MIN_POSES:
        spans.append((start, len(rate)))

    results = []
    for a, b in spans:
        turn = float(abs(yaw[b] - yaw[a]))
        if turn < MIN_TURN_RAD:
            continue
        count = b - a
        design = np.zeros((2 * count, 4))
        rhs = np.zeros(2 * count)
        angles = yaw[a:b]
        design[0::2, 0] = 1
        design[0::2, 2] = np.cos(angles)
        design[0::2, 3] = -np.sin(angles)
        rhs[0::2] = x[a:b]
        design[1::2, 1] = 1
        design[1::2, 2] = np.sin(angles)
        design[1::2, 3] = np.cos(angles)
        rhs[1::2] = y[a:b]
        with np.errstate(all="ignore"):
            solution, _, _, _ = np.linalg.lstsq(design, rhs, rcond=None)
        residual = float(np.sqrt(np.mean((design @ solution - rhs) ** 2)))
        results.append({
            "t0": float(t[a] - t[0]),
            "turn_rad": turn,
            "forward_m": float(solution[2]),
            "left_m": float(solution[3]),
            "residual_m": residual,
        })
    return results


def main(path, topic):
    table = read_poses(path, topic)
    print("%s: %d poses over %.0f s" % (
        topic, len(table), table[-1, 0] - table[0, 0]))
    spins = fit_spins(table)
    if not spins:
        raise SystemExit(
            "no spin-in-place found; this needs a drive where the operator "
            "turns on the spot")

    print("\n%-8s %8s %10s %10s %10s" % (
        "t0 (s)", "turn", "forward", "LEFT", "residual"))
    clean = []
    for spin in spins:
        flag = "" if spin["residual_m"] <= MAX_RESIDUAL_M else "  rejected"
        print("%8.1f %7.2f  %+9.3f %+10.3f %9.1f mm%s" % (
            spin["t0"], spin["turn_rad"], spin["forward_m"], spin["left_m"],
            spin["residual_m"] * 1000, flag))
        if not flag:
            clean.append(spin)

    if not clean:
        raise SystemExit(
            "every spin fitted worse than %.0f mm - none was a true spin in "
            "place" % (MAX_RESIDUAL_M * 1000))

    weights = np.array([s["turn_rad"] for s in clean])
    forward = np.average([s["forward_m"] for s in clean], weights=weights)
    left = np.average([s["left_m"] for s in clean], weights=weights)
    print("\nturn-weighted over %d clean spin(s):" % len(clean))
    print("   sensor is %+.3f m forward and %+.3f m LEFT of the chair centre"
          % (forward, left))
    print("   body_frame.CHAIR_CENTRE_IN_BODY_XYZ = (%.3f, %.3f, 0.0)"
          % (-forward, -left))
    if len(clean) > 1:
        spread = max(s["left_m"] for s in clean) - min(
            s["left_m"] for s in clean)
        print("   lateral spread between spins: %.3f m" % spread)


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    main(sys.argv[1],
         sys.argv[2] if len(sys.argv) == 3 else "/fast_lio_icp/pose")
