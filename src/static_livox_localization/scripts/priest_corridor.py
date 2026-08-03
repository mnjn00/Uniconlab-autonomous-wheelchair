"""Read the runtime SafetyBand as the PRIEST proposal corridor.

The band and the planner mean different things by "lateral limit", and the
gap between them is exactly where a kerb would slip through. SafetyBand is the
only implementation of measured hazards, drawings, negative limits and crossed
stations. This adapter copies its already-computed arrays; it never rebuilds or
floors them.

On the v4 band every edge kind is "open" and every drop is zero - the ZIP
carried no drop measurement, and the band says so itself under
physical_edge_semantics. usable_limit() therefore subtracts nothing there,
and the whole constraint is the operator's drawing. That is the shipped
reality, not a choice made here; it is recorded in the README's warning and
re-measuring it is an open item. This module simply refuses to make it look
better than it is.

Normals point LEFT of the station heading (heading rotated +90 degrees), so
a positive lateral offset is a move to the left - the same convention the
band's left_m/right_m already use.
"""

from __future__ import annotations

import numpy as np

from safety_band import SafetyBand


def corridor_arrays(
        band_path: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(centres, normals, left, right) from a shipped band file.

    Arrays in station order, ready for priest_planner.Corridor. Raises on a
    band too short to define a direction of travel - a one-station corridor
    has no arc length, and arc length is how the planner knows which way the
    goal lies.
    """
    band = SafetyBand(band_path)
    if len(band.xy) < 2:
        raise ValueError(
            "band %s has %d stations; a corridor needs at least two to have "
            "a direction" % (band_path, len(band.xy)))
    return (
        band.xy.astype(np.float64, copy=True),
        band.normals.astype(np.float64, copy=True),
        band.left.astype(np.float64, copy=True),
        band.right.astype(np.float64, copy=True),
    )
