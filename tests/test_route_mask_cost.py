"""Runtime hard-mask and boundary-cost behavior."""

import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from route_mask import RouteMask
finally:
    sys.path.pop(0)


def write_mask(tmp_path: Path, image=None) -> Path:
    if image is None:
        image = np.full((9, 11), 205, dtype=np.uint8)
        image[1:8, 1:10] = 254
    Image.fromarray(image).save(tmp_path / "mask.pgm")
    yaml_path = tmp_path / "mask.yaml"
    yaml_path.write_text(
        "image: mask.pgm\nresolution: 0.1\n"
        "origin: [0.0, 0.0, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
    )
    return yaml_path


def test_outside_mask_is_a_hard_reject(tmp_path):
    # Given: a rectangular authoritative drivable mask.
    mask = RouteMask(str(write_mask(tmp_path)))
    points = np.array([[0.5, 0.4], [0.0, 0.4], [1.0, 0.4]])

    # When: runtime points are checked.
    inside = mask.contains_many(points)

    # Then: no cost can buy a point outside the mask.
    assert inside.tolist() == [True, False, False]


def test_boundary_cost_rises_toward_the_mask_edge(tmp_path):
    # Given: a rectangular authoritative drivable mask.
    mask = RouteMask(str(write_mask(tmp_path)))
    points = np.array([[0.5, 0.4], [0.2, 0.4], [0.1, 0.4]])

    # When: boundary costs are evaluated from centre to edge.
    costs = mask.boundary_cost_many(points)

    # Then: the edge is progressively less selectable.
    assert costs[0] < costs[1] < costs[2]
    assert costs[0] < 0.05
    assert costs[2] > 0.6


def test_segment_containment_catches_forbidden_cell_between_endpoints(tmp_path):
    image = np.full((3, 5), 254, dtype=np.uint8)
    image[1, 2] = 0
    mask = RouteMask(str(write_mask(tmp_path, image)))
    start = np.array([0.0, 0.1])
    end = np.array([0.4, 0.1])
    assert mask.contains(start)
    assert mask.contains(end)
    assert not mask.segment_is_contained(start, end)
    assert not mask.paths_are_contained([[start, end]])[0]


def test_segment_containment_catches_short_corner_clip(tmp_path):
    image = np.full((5, 5), 254, dtype=np.uint8)
    image[2, 2] = 0
    mask = RouteMask(str(write_mask(tmp_path, image)))
    start = np.array([0.0, 0.2504])
    end = np.array([0.4, 0.1496])
    assert mask.contains(start)
    assert mask.contains(end)
    assert not mask.segment_is_contained(start, end)


def test_segment_containment_rejects_forbidden_corner_touch(tmp_path):
    image = np.full((5, 5), 254, dtype=np.uint8)
    image[1, 2] = 0
    mask = RouteMask(str(write_mask(tmp_path, image)))
    assert not mask.segment_is_contained([0.1, 0.1], [0.3, 0.3])
