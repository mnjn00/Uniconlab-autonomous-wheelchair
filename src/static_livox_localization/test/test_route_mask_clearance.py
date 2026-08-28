import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from route_mask import RouteMask  # noqa: E402


def make_mask(tmp_path):
    image = np.zeros((9, 9), dtype=np.uint8)
    image[1:8, 1:8] = 254
    Image.fromarray(image).save(str(tmp_path / "mask.pgm"))
    metadata = {
        "image": "mask.pgm",
        "resolution": 0.1,
        "origin": [0.0, 0.0, 0.0],
    }
    (tmp_path / "mask.yaml").write_text(
        yaml.safe_dump(metadata), encoding="utf-8")
    return RouteMask(str(tmp_path / "mask.yaml"))


def test_clearance_is_zero_outside_and_positive_inside(tmp_path):
    mask = make_mask(tmp_path)
    values = mask.clearance_many([
        (0.4, 0.4),
        (-1.0, -1.0),
    ])
    assert values[0] > 0.0
    assert values[1] == 0.0


def test_clearance_grows_towards_the_middle(tmp_path):
    mask = make_mask(tmp_path)
    near = mask.clearance_at((0.1, 0.1))
    middle = mask.clearance_at((0.4, 0.4))
    assert middle > near


def test_single_point_input_is_accepted(tmp_path):
    mask = make_mask(tmp_path)
    assert isinstance(mask.contains((0.4, 0.4)), bool)
