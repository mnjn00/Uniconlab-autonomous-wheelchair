"""What the chair can reach, and the numpy trap that inverted it once.

The classification is a few lines of mask algebra over a raster, and the one
property that makes it trustworthy is cheap to state: the chair drove the
route, so every cell it physically occupied has to come out reachable. That
check is what caught the failure below, because nothing else did.

`other & ~mask` negates `mask` IN PLACE when numpy decides the operand is a
temporary, which a large function-local array can look like (numpy 2.2.6,
Python 3.14). The swept-footprint mask came back as its own complement -
11569 cells became 1560300 - every downstream class inverted, and the tool
printed a full set of confident, wrong numbers with no error anywhere.
"""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


TOOL = Path(__file__).parents[2].parent / "tools" / "make_traversability.py"


def load():
    spec = importlib.util.spec_from_file_location("traversability", TOOL)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tv = load()

# Above numpy's elision threshold; the bug does not appear on small arrays,
# which is exactly why it survived a prototype and reached the tool.
BIG = 1_600_000


def test_negation_does_not_scribble_on_its_operand():
    def inside_a_function():
        mask = np.zeros(BIG, bool)
        mask[:10_000] = True
        other = np.zeros(BIG, bool)
        other[::3] = True
        before = int(mask.sum())
        other & tv.keep_out(mask)
        return before, int(mask.sum())

    before, after = inside_a_function()
    assert before == after == 10_000


def test_the_bare_operator_is_what_this_avoids():
    """Pins the reason keep_out exists. If a numpy release fixes the elision
    this starts failing, and the helper can go - but not before."""
    def inside_a_function():
        mask = np.zeros(BIG, bool)
        mask[:10_000] = True
        other = np.zeros(BIG, bool)
        other[::3] = True
        before = int(mask.sum())
        other & ~mask
        return before, int(mask.sum())

    before, after = inside_a_function()
    if before == after:
        pytest.skip("numpy no longer elides here; keep_out may be retired")
    assert after != before


def synthetic_corridor():
    """A 30 m straight of pavement, a kerb down one side, a 0.3 m post on it.

    The raised side runs out to +6 m on purpose. Ground recovery opens the
    height raster with a 1.5 m element, so a raised strip narrower than about
    3 m is lifted off as an object standing ON the ground rather than kept as
    ground - which is the right call for a bollard and the wrong one for a
    verge, and is why this scene is wider than it first needs to be.
    """
    rows = []
    for along in np.arange(0.0, 30.0, 0.04):
        for across in np.arange(-3.0, 6.0, 0.04):
            height = 0.0 if across < 1.2 else 0.14      # kerb up at +1.2 m
            rows.append((along, across, height))
    post = []
    for angle in np.linspace(0, 2 * np.pi, 60):
        for up in np.arange(0.1, 1.2, 0.05):
            post.append((15.0 + 0.15 * np.cos(angle), -2.2 + 0.15 * np.sin(angle), up))
    return np.array(rows + post, dtype=np.float32)


def test_the_drive_always_comes_out_reachable():
    """The property the whole thing rests on. A classification that refuses
    ground the chair measurably crossed is wrong about the ground, not about
    the chair."""
    points = synthetic_corridor()
    route_xy = np.stack([np.arange(0.0, 30.0, 0.2), np.zeros(150)], axis=1)
    route_z = np.zeros(len(route_xy))

    grid = tv.build(points, route_xy, route_z)
    masks = tv.classify(grid)

    driven = masks["driven"]
    assert driven.sum() > 0
    assert (masks["reachable"] & driven).sum() == driven.sum()


def test_a_post_beside_the_path_is_found_and_a_kerb_is_not_called_a_post():
    points = synthetic_corridor()
    route_xy = np.stack([np.arange(0.0, 30.0, 0.2), np.zeros(150)], axis=1)

    grid = tv.build(points, route_xy, np.zeros(len(route_xy)))
    masks = tv.classify(grid)

    # the post stands at chair height; the kerb is a step in the ground
    assert masks["obstruction"].sum() > 0
    assert masks["stepped"].sum() > 0
    # and they are different places
    assert (masks["obstruction"] & masks["stepped"]).sum() < masks["obstruction"].sum()


def test_the_swept_footprint_is_never_called_an_obstruction():
    """The map holds the chair and its rider along the driven line. Reading
    those returns as structure is what makes 64.7 percent of the cells the
    chair occupied look blocked."""
    points = synthetic_corridor()
    route_xy = np.stack([np.arange(0.0, 30.0, 0.2), np.zeros(150)], axis=1)

    grid = tv.build(points, route_xy, np.zeros(len(route_xy)))
    masks = tv.classify(grid)

    assert (masks["obstruction"] & masks["driven"]).sum() == 0
