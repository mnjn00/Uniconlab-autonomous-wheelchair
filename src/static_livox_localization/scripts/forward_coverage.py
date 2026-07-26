"""ROS-free forward-corridor observability checks for obstacle guards."""

import numpy as np


MIN_FORWARD_POINTS = 12
FORWARD_X_BINS = 3
MIN_FORWARD_X_BINS = 2
FORWARD_Y_BINS = 3
MIN_FORWARD_Y_BINS = 2


def corridor_has_coverage(
        points, x_min, x_max, y_center, half_width, z_min, z_max):
    """Return whether a forward corridor is observed, not merely obstacle-free.

    Coverage requires enough finite points spread across both range and lateral
    bins.  A cloud containing only rear, side, or one narrow cluster therefore
    cannot be interpreted as evidence that the forward path is clear.
    """
    if points is None or x_max <= x_min or half_width <= 0.0 or z_max <= z_min:
        return False
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] < 3:
        return False
    finite = np.all(np.isfinite(points[:, :3]), axis=1)
    corridor = (
        finite &
        (points[:, 0] > x_min) &
        (points[:, 0] < x_max) &
        (np.abs(points[:, 1] - y_center) < half_width) &
        (points[:, 2] > z_min) &
        (points[:, 2] < z_max)
    )
    observed = points[corridor]
    if len(observed) < MIN_FORWARD_POINTS:
        return False

    x_index = np.floor(
        (observed[:, 0] - x_min) / (x_max - x_min) * FORWARD_X_BINS
    ).astype(np.int64)
    y_index = np.floor(
        (observed[:, 1] - (y_center - half_width)) /
        (2.0 * half_width) * FORWARD_Y_BINS
    ).astype(np.int64)
    x_index = np.clip(x_index, 0, FORWARD_X_BINS - 1)
    y_index = np.clip(y_index, 0, FORWARD_Y_BINS - 1)
    return (
        len(np.unique(x_index)) >= MIN_FORWARD_X_BINS and
        len(np.unique(y_index)) >= MIN_FORWARD_Y_BINS
    )
