"""The buffer decoder, checked against a point-by-point reference.

Every assertion here compares the fast path to an independent struct-based
implementation of what read_points did, rather than to a recorded expected
array. A decoder that reads the wrong bytes produces perfectly plausible
coordinates - slightly shifted obstacles, a corridor that looks clear - so
the property worth pinning is agreement with the thing it replaced, on
layouts chosen to break wrong assumptions about packing.
"""

import os
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

from cloud_points import point_dtype, points_xyz  # noqa: E402

FLOAT32 = 7
FLOAT64 = 8


class Field:
    def __init__(self, name, offset, datatype=FLOAT32, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


class Cloud:
    """Just the attributes the decoder reads."""

    def __init__(self, fields, data, width, height=1, point_step=None,
                 row_step=None, is_bigendian=False):
        self.fields = fields
        self.data = data
        self.width = width
        self.height = height
        self.point_step = point_step
        self.row_step = row_step if row_step is not None \
            else width * point_step
        self.is_bigendian = is_bigendian


def reference(cloud):
    """What read_points(skip_nans=True) yielded, done here with struct."""
    order = ">" if cloud.is_bigendian else "<"
    offsets = {f.name: f.offset for f in cloud.fields}
    out = []
    for v in range(cloud.height):
        base = v * cloud.row_step
        for u in range(cloud.width):
            at = base + u * cloud.point_step
            point = tuple(
                struct.unpack_from(order + "f", cloud.data, at + offsets[n])[0]
                for n in ("x", "y", "z"))
            if not any(value != value for value in point):   # NaN check
                out.append(point)
    return np.array(out, dtype=np.float32).reshape(-1, 3)


def pack(points, offsets, point_step, is_bigendian=False, row_pad=0,
         height=1):
    """Lay points out at the given field offsets inside the stride."""
    order = ">" if is_bigendian else "<"
    per_row = len(points) // height
    blob = b""
    for v in range(height):
        row = b""
        for point in points[v * per_row:(v + 1) * per_row]:
            record = bytearray(b"\x00" * point_step)
            for name, value in zip(("x", "y", "z"), point):
                struct.pack_into(order + "f", record, offsets[name], value)
            row += bytes(record)
        blob += row + b"\x00" * row_pad
    return blob


SAMPLE = [(1.0, 2.0, 3.0), (-0.5, 0.25, 12.75), (0.0, 0.0, 0.0),
          (7.5, -3.25, 0.125), (100.0, 0.5, -8.0), (2.0, 2.0, 2.0)]


def build(points, offsets=None, point_step=16, **kwargs):
    offsets = offsets or {"x": 0, "y": 4, "z": 8}
    height = kwargs.pop("height", 1)
    row_pad = kwargs.pop("row_pad", 0)
    is_bigendian = kwargs.pop("is_bigendian", False)
    data = pack(points, offsets, point_step, is_bigendian, row_pad, height)
    fields = [Field(n, offsets[n]) for n in ("x", "y", "z")]
    fields.append(Field("intensity", 12))
    return Cloud(fields, data, width=len(points) // height, height=height,
                 point_step=point_step,
                 row_step=len(points) // height * point_step + row_pad,
                 is_bigendian=is_bigendian, **kwargs)


def boom(*a, **k):
    raise AssertionError("the slow reader was called")


def test_matches_the_point_by_point_reader():
    cloud = build(SAMPLE)
    np.testing.assert_array_equal(points_xyz(cloud, boom), reference(cloud))


def test_does_not_assume_x_y_z_are_the_first_three_fields():
    """A cloud with intensity ahead of the coordinates decodes the same.
    Assuming 0/4/8 would return intensity bytes as an x coordinate and look
    entirely reasonable doing it."""
    cloud = build(SAMPLE, offsets={"x": 4, "y": 8, "z": 12}, point_step=20)
    np.testing.assert_array_equal(points_xyz(cloud, boom), reference(cloud))


def test_stride_wider_than_the_coordinates():
    cloud = build(SAMPLE, point_step=32)
    np.testing.assert_array_equal(points_xyz(cloud, boom), reference(cloud))


def test_rows_padded_past_their_points():
    """row_step may exceed width * point_step. Reading straight through
    would walk the padding into the next row's coordinates."""
    cloud = build(SAMPLE, height=2, row_pad=7)
    np.testing.assert_array_equal(points_xyz(cloud, boom), reference(cloud))


def test_big_endian():
    cloud = build(SAMPLE, is_bigendian=True)
    np.testing.assert_array_equal(points_xyz(cloud, boom), reference(cloud))


def test_nan_points_are_dropped():
    nan = float("nan")
    points = [(1.0, 2.0, 3.0), (nan, 1.0, 1.0), (4.0, 5.0, 6.0),
              (1.0, nan, 1.0), (7.0, 8.0, 9.0), (1.0, 1.0, nan)]
    cloud = build(points)
    got = points_xyz(cloud, boom)
    np.testing.assert_array_equal(got, reference(cloud))
    assert len(got) == 3


def test_infinities_survive_exactly_as_they_did():
    """read_points(skip_nans=True) tested isnan, not isfinite, so an
    infinite coordinate reached the consumers. This is a change of decoding
    and must not quietly also change what gets filtered."""
    inf = float("inf")
    points = [(1.0, 2.0, 3.0), (inf, 1.0, 1.0), (4.0, 5.0, -inf)]
    cloud = build(points)
    got = points_xyz(cloud, boom)
    np.testing.assert_array_equal(got, reference(cloud))
    assert len(got) == 3


def test_empty_cloud_is_an_empty_frame_not_a_crash():
    cloud = Cloud([Field(n, o) for n, o in (("x", 0), ("y", 4), ("z", 8))],
                  b"", width=0, height=1, point_step=16, row_step=0)
    got = points_xyz(cloud, boom)
    assert got.shape == (0, 3)
    assert not len(got)


def test_output_is_float32():
    assert points_xyz(build(SAMPLE), boom).dtype == np.float32


def test_a_layout_it_cannot_express_falls_back_rather_than_guessing():
    """Coordinates as FLOAT64 are not something this decoder handles. The
    wrong answer here is to read them as pairs of float32 and return
    nonsense that still has three columns."""
    cloud = build(SAMPLE)
    cloud.fields = [Field("x", 0, FLOAT64), Field("y", 8, FLOAT64),
                    Field("z", 16, FLOAT64)]
    assert point_dtype(cloud) is None
    called = []

    def slow(message, field_names=None, skip_nans=False):
        called.append(True)
        return [(1.0, 2.0, 3.0)]

    np.testing.assert_array_equal(points_xyz(cloud, slow),
                                  np.array([[1.0, 2.0, 3.0]], np.float32))
    assert called


def test_a_missing_coordinate_falls_back():
    cloud = build(SAMPLE)
    cloud.fields = [Field("x", 0), Field("y", 4)]
    assert point_dtype(cloud) is None


def test_a_field_reaching_past_the_stride_falls_back():
    """point_step is the record size; a coordinate that does not fit inside
    it means the header disagrees with itself."""
    cloud = build(SAMPLE)
    cloud.point_step = 8
    assert point_dtype(cloud) is None


def test_a_truncated_buffer_falls_back_instead_of_reading_past_it():
    cloud = build(SAMPLE)
    cloud.data = cloud.data[:-5]
    called = []

    def slow(message, field_names=None, skip_nans=False):
        called.append(True)
        return []

    points_xyz(cloud, slow)
    assert called, "a short buffer must not be decoded as if it were whole"


@pytest.mark.parametrize("count", [1, 7, 64, 999])
def test_agreement_holds_at_several_sizes(count):
    rng = np.random.default_rng(count)
    points = [tuple(float(v) for v in row)
              for row in rng.uniform(-40, 40, size=(count, 3))]
    cloud = build(points)
    np.testing.assert_array_equal(points_xyz(cloud, boom), reference(cloud))
