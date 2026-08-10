"""Read x, y, z out of a PointCloud2 without visiting each point in Python.

sensor_msgs.point_cloud2.read_points is a generator that, per point, calls
struct.unpack_from, loops over the returned values looking for NaN, and
yields a tuple. Wrapping that in np.array(list(...)) allocates one tuple per
point and then rebuilds the whole thing as an array. On a MID360 sweep at
10 Hz that is the dominant cost of every node that looks at the cloud, and
on 2026-08-05 three of them were doing it independently on the same message:
waypoint_follower (via scan_accumulator), safety_gate (the same), and
obstacle_clusters (its own copy). Measured on the NUC during a run that
never left the start, with the follower PAUSED and solving nothing:

  mpc_follower.py     276% CPU        obstacle_clusters.py  110% CPU
  safety_gate.py      226% CPU        fastlio (C++)          53% CPU

Load average 18.3 on 8 threads, 71.7% of it system time. The MPC solver was
not running for any of it; the parsing was.

A PointCloud2 is already a packed array of fixed-stride records. numpy can
describe that layout as a dtype and read the buffer in one C-level call, so
the work per point drops to nothing the interpreter has to see. Same bytes,
same numbers - this is a change of decoding, not of geometry.

The fast path needs the layout to be one numpy can express: x, y and z each
a single FLOAT32 inside the point stride. That is what FAST-LIO publishes
and what the Livox driver publishes, but neither is promised by the message
definition, so a cloud that does not match falls back to read_points rather
than guessing. The fallback is logged once per process, never silent - a
node that quietly went back to the slow path would look exactly like this
bug did before it was found.
"""

import numpy as np

# sensor_msgs/PointField. Only FLOAT32 is accepted below: the conversion the
# other types would need is per-point work again, and nothing in this stack
# publishes them.
FLOAT32 = 7

_warned = False
_warned_xyzi = False


def point_dtype(message):
    """A structured dtype over one point, or None if it cannot be expressed.

    Offsets come from the message rather than being assumed to be 0/4/8 -
    a cloud carrying intensity or ring between the coordinates is still
    readable this way, and assuming the packing is where a decoder that
    looks right silently returns other fields' bytes as coordinates.
    """
    offsets = {}
    for field in message.fields:
        if field.name in ("x", "y", "z"):
            if field.datatype != FLOAT32 or field.count != 1:
                return None
            offsets[field.name] = field.offset
    if len(offsets) != 3:
        return None
    if max(offsets.values()) + 4 > message.point_step:
        return None
    order = ">f4" if message.is_bigendian else "<f4"
    return np.dtype({
        "names": ["x", "y", "z"],
        "formats": [order] * 3,
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": message.point_step,
    })


def _fallback(message, read_points, why):
    global _warned
    if not _warned:
        _warned = True
        try:
            import rospy
            rospy.logwarn(
                "cloud_points: reading the cloud point-by-point (%s). This "
                "is the slow path and costs whole cores; see "
                "cloud_points.py", why)
        except Exception:
            pass
    if read_points is None:
        import sensor_msgs.point_cloud2 as pc2
        read_points = pc2.read_points
    return np.array(list(read_points(
        message, field_names=("x", "y", "z"), skip_nans=True)),
        dtype=np.float32)


def points_xyz(message, read_points=None):
    """(N, 3) float32 of finite-coordinate points, in the message's frame.

    Drops any point with a NaN coordinate and keeps everything else,
    including infinities - which is what read_points(skip_nans=True) did,
    and this is a decoding change that must not also change what survives
    the filter.
    """
    dtype = point_dtype(message)
    if dtype is None:
        return _fallback(message, read_points, "unexpected field layout")

    width, height = int(message.width), int(message.height)
    point_step, row_step = int(message.point_step), int(message.row_step)
    packed = width * point_step
    # row_step may pad past the points; height * row_step is what the buffer
    # is required to hold. Anything short of that is a malformed message and
    # the point-by-point reader is the one that gets to decide what to do
    # about it, since it is what every consumer here used until now.
    if row_step < packed or len(message.data) < height * row_step:
        return _fallback(message, read_points, "row stride does not fit")
    if width == 0 or height == 0:
        return np.zeros((0, 3), dtype=np.float32)

    raw = np.frombuffer(message.data, dtype=np.uint8, count=height * row_step)
    rows = raw.reshape(height, row_step)[:, :packed]
    records = np.ascontiguousarray(rows).view(dtype).reshape(-1)
    points = np.stack((records["x"], records["y"], records["z"]),
                      axis=-1).astype(np.float32, copy=False)
    return points[~np.isnan(points).any(axis=1)]


def point_dtype_xyzi(message):
    """A structured dtype over one point carrying x, y, z, intensity.

    Like point_dtype but also locates a FLOAT32 ``intensity`` field.
    Returns None when x, y, z are not all FLOAT32 or when intensity is
    absent — the caller falls back to the xyz-only path in that case.
    """
    offsets = {}
    for field in message.fields:
        if field.name in ("x", "y", "z", "intensity"):
            if field.datatype != FLOAT32 or field.count != 1:
                if field.name in ("x", "y", "z"):
                    return None
                continue
            offsets[field.name] = field.offset
    if len({k: v for k, v in offsets.items() if k in ("x", "y", "z")}) != 3:
        return None
    if "intensity" not in offsets:
        return None
    if max(offsets.values()) + 4 > message.point_step:
        return None
    order = ">f4" if message.is_bigendian else "<f4"
    return np.dtype({
        "names": ["x", "y", "z", "intensity"],
        "formats": [order] * 4,
        "offsets": [offsets["x"], offsets["y"], offsets["z"],
                    offsets["intensity"]],
        "itemsize": message.point_step,
    })


def _fallback_xyzi(message, read_points, why):
    global _warned_xyzi
    if not _warned_xyzi:
        _warned_xyzi = True
        try:
            import rospy
            rospy.logwarn(
                "cloud_points: reading intensity the slow path (%s). "
                "This costs whole cores; see cloud_points.py", why)
        except Exception:
            pass
    if read_points is None:
        import sensor_msgs.point_cloud2 as pc2
        read_points = pc2.read_points
    pts = np.array(list(read_points(
        message, field_names=("x", "y", "z", "intensity"),
        skip_nans=True)), dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 4:
        return np.zeros((0, 4), dtype=np.float32)
    return pts[:, :4]


def points_xyzi(message, read_points=None):
    """(N, 4) float32 of x, y, z, intensity — finite coordinates only.

    Falls back to points_xyz with a zero intensity column when the cloud
    does not carry a FLOAT32 intensity field.  This keeps the caller's
    shape contract (N, 4) without silently dropping points that a
    non-intensity-publishing source still produces.
    """
    dtype = point_dtype_xyzi(message)
    if dtype is None:
        xyz = points_xyz(message, read_points)
        out = np.zeros((len(xyz), 4), dtype=np.float32)
        out[:, :3] = xyz
        return out

    width, height = int(message.width), int(message.height)
    point_step, row_step = int(message.point_step), int(message.row_step)
    packed = width * point_step
    if row_step < packed or len(message.data) < height * row_step:
        return _fallback_xyzi(message, read_points, "row stride does not fit")
    if width == 0 or height == 0:
        return np.zeros((0, 4), dtype=np.float32)

    raw = np.frombuffer(message.data, dtype=np.uint8, count=height * row_step)
    rows = raw.reshape(height, row_step)[:, :packed]
    records = np.ascontiguousarray(rows).view(dtype).reshape(-1)
    points = np.stack(
        (records["x"], records["y"], records["z"], records["intensity"]),
        axis=-1).astype(np.float32, copy=False)
    return points[~np.isnan(points[:, :3]).any(axis=1)]
