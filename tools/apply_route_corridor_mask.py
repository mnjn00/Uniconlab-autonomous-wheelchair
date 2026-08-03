#!/usr/bin/env python3
"""Carry a hand-drawn corridor into the band as a second, weaker limit.

The map-derived band answers "how far out does the ground break". On this
route that answer is often very permissive on one side - the shipped band's
median left_m is 2.45 m - because the ground genuinely does not break for
2.45 m. It is a plaza, a forecourt, a road surface flush with the pavement.
Nothing the map can measure says the chair should not be out there, and the
follower will happily step 2 m off the line to pass a pedestrian.

The operator drawing says it should not. That is a different kind of claim -
judgement about where the chair belongs, not a measurement of where the
ground is - and it is recorded separately for that reason:

  left_m / right_m           unchanged, still the measured edge
  left_kind / *_drop_m       unchanged, still the measured hazard
  left_corridor_m / right_*  how far the drawing extends, or absent

Only SafetyBand.usable_limit is clamped by the corridor. hazard_clearance
and safe_offset keep reading the physical fields, so speed pacing and the
lean-away-from-the-kerb bias still react to real falls at their real
distances. Overwriting left_m with the corridor extent instead would have
told the speed policy a 2.45 m kerb was 0.80 m away and slowed the chair
for the whole route; erasing left_kind to avoid that would have told it a
real kerb was open pavement. Neither is what the drawing means.

Three rules keep the drawing from making the route undrivable:

  - It may only NARROW. A corridor wider than the measured band is ignored,
    because a person drawing over a map cannot authorise ground the map says
    breaks.
  - Where the drawing does not cover the station at all - 72 of 381 on the
    0727 route, including the last 8.7 m into the goal - nothing is clamped
    and the station is listed in the audit. Failing closed there would
    refuse a route that two complete runs on 2026-07-31 drove.
  - The clamp is floored at zero, never negative. The driven line is the one
    path known to have been driven; a corridor that excludes it is a drawing
    error, and the audit says so rather than the chair holding at that
    station.

Usage: apply_route_corridor_mask.py <corridor.yaml> <band.json> <route.json>
                                    <out-band.json> <out-audit.json>
Writes <out-band.json> (band plus corridor fields), <out-audit.json>, and
<out-audit>.png.
"""

import json
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw

Image.MAX_IMAGE_PIXELS = None

# Lateral walk resolution. Finer than the 0.1 m mask cell, so the reported
# extent lands on the cell boundary rather than a cell centre.
STEP = 0.05
# Matches make_route_safety_band.MAX_LAT: beyond this the band does not
# measure either, so a corridor answer out there could never bind.
MAX_LAT = 6.0
# safety_band.CHAIR_HALF_WIDTH. The drawing is a corridor for the CHAIR, so
# the whole chair belongs inside it, not just its centre. Kept here so the
# recorded extent and the consumer's inset cannot drift apart.
CHAIR_HALF_WIDTH = 0.35
FREE = 254


def load_mask(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                            meta["image"])
    img = np.array(Image.open(img_path))
    if img.ndim != 2:
        img = img[..., 0]
    return img == FREE, float(meta["resolution"]), np.array(meta["origin"][:2])


def main():
    if len(sys.argv) != 6:
        sys.exit(__doc__)
    corridor_yaml, band_path, route_path, out_band, out_audit = sys.argv[1:6]

    free, res, origin = load_mask(corridor_yaml)
    height, width = free.shape

    def inside(pts):
        col = np.rint((pts[:, 0] - origin[0]) / res).astype(int)
        row = np.rint(height - 1 - (pts[:, 1] - origin[1]) / res).astype(int)
        ok = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        out = np.zeros(len(pts), bool)
        out[ok] = free[row[ok], col[ok]]
        return out

    band = json.load(open(band_path))
    stations = band["stations"]
    xy = np.array([[s["x"], s["y"]] for s in stations], dtype=float)
    heading = np.radians([s["heading_deg"] for s in stations])
    normals = np.stack([-np.sin(heading), np.cos(heading)], axis=1)

    offsets = np.arange(int(MAX_LAT / STEP) + 1) * STEP
    covered = inside(xy)

    uncovered, narrowed = [], []
    for i, station in enumerate(stations):
        station.pop("left_corridor_m", None)
        station.pop("right_corridor_m", None)
        if not covered[i]:
            uncovered.append(i)
            continue
        extent = {}
        for side, sign in (("left", +1.0), ("right", -1.0)):
            pts = xy[i] + sign * np.outer(offsets, normals[i])
            broke = np.nonzero(~inside(pts))[0]
            reach = offsets[broke[0] - 1] if len(broke) else MAX_LAT
            extent[side] = round(float(reach), 3)
            station[f"{side}_corridor_m"] = extent[side]
        for side in ("left", "right"):
            if extent[side] < station[f"{side}_m"]:
                narrowed.append(i)
                break

    # The audit has to report the decision the CHAIR will make, so it runs
    # the shipped consumer rather than reimplementing it. safety_band is
    # kept free of ROS imports for exactly this.
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "src", "static_livox_localization", "scripts"))
    import safety_band  # noqa: E402

    measured_l, measured_r, drawn_l, drawn_r, yielded = [], [], [], [], []
    for i, s in enumerate(stations):
        ml = safety_band.usable_limit(
            s["left_m"], s.get("left_drop_m"), s.get("left_kind"))
        mr = safety_band.usable_limit(
            s["right_m"], s.get("right_drop_m"), s.get("right_kind"))
        cl = safety_band.corridor_limit(ml, s.get("left_corridor_m"))
        cr = safety_band.corridor_limit(mr, s.get("right_corridor_m"))
        if cl + cr < 0.0 <= ml + mr:
            yielded.append(i)
            cl, cr = ml, mr
        measured_l.append(ml)
        measured_r.append(mr)
        drawn_l.append(cl)
        drawn_r.append(cr)

    before = np.array(measured_l + measured_r)
    after = np.array(drawn_l + drawn_r)
    width_before = np.array(measured_l) + np.array(measured_r)
    width_after = np.array(drawn_l) + np.array(drawn_r)

    # where the driven route leaves the drawing entirely
    route = json.load(open(route_path))
    rxy = np.array([[w["x"], w["y"]] for w in route["waypoints"]], dtype=float)
    r_in = inside(rxy)
    arc = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(rxy, axis=0), axis=1))])
    excursions = []
    flips = np.diff(np.concatenate([[1], r_in.astype(int), [1]]))
    for a, b in zip(np.nonzero(flips == -1)[0], np.nonzero(flips == 1)[0]):
        excursions.append({
            "start_m": round(float(arc[a]), 1),
            "end_m": round(float(arc[b - 1]), 1),
            "length_m": round(float(arc[b - 1] - arc[a]), 1),
            "waypoints": int(b - a),
            "start_xy": [round(float(rxy[a, 0]), 1), round(float(rxy[a, 1]), 1)],
            "end_xy": [round(float(rxy[b - 1, 0]), 1), round(float(rxy[b - 1, 1]), 1)],
        })
    excursions.sort(key=lambda e: -e["length_m"])

    band["corridor"] = {
        "source": corridor_yaml.replace("\\", "/").rsplit("/", 1)[-1],
        "chair_half_width_m": CHAIR_HALF_WIDTH,
        "policy": "operator judgement may narrow the measured band, never widen it; "
                  "stations the drawing does not cover keep the measured band",
        "stations_covered": int(covered.sum()),
        "stations_total": len(stations),
    }
    json.dump(band, open(out_band, "w"), indent=1)

    audit = {
        "band": band_path.replace("\\", "/").rsplit("/", 1)[-1],
        "route": route_path.replace("\\", "/").rsplit("/", 1)[-1],
        "corridor": corridor_yaml.replace("\\", "/").rsplit("/", 1)[-1],
        "stations_total": len(stations),
        "stations_covered": int(covered.sum()),
        "stations_uncovered": uncovered,
        "stations_narrowed": len(narrowed),
        "stations_corridor_yielded": yielded,
        "usable_limit_m": {
            "median_before": round(float(np.median(before)), 3),
            "median_after": round(float(np.median(after)), 3),
            "total_before": round(float(before.sum()), 1),
            "total_after": round(float(after.sum()), 1),
            "never_widened": bool(np.all(after <= before + 1e-9)),
        },
        "admissible_width_m": {
            "median_before": round(float(np.median(width_before)), 3),
            "median_after": round(float(np.median(width_after)), 3),
            "empty_before": int((width_before < 0.0).sum()),
            "empty_after": int((width_after < 0.0).sum()),
            "no_new_empty_stations":
                int((width_after < 0.0).sum()) <= int((width_before < 0.0).sum()),
        },
        "route_outside_corridor": {
            "waypoints": int((~r_in).sum()),
            "of": len(rxy),
            "excursions": excursions,
        },
    }
    json.dump(audit, open(out_audit, "w"), indent=1)

    # preview: corridor in grey, band edges, clamped stations marked
    span = xy.max(axis=0) - xy.min(axis=0)
    scale = 1600.0 / max(span)
    lo = xy.min(axis=0) - 6.0

    def px(p):
        return ((p[0] - lo[0]) * scale, (span[1] + 12.0 - (p[1] - lo[1])) * scale)

    img = Image.new("RGB", (int((span[0] + 12) * scale), int((span[1] + 12) * scale)),
                    (255, 255, 255))
    draw = ImageDraw.Draw(img)
    rows, cols = np.nonzero(free)
    cx = origin[0] + cols * res
    cy = origin[1] + (height - 1 - rows) * res
    keep = ((cx > lo[0]) & (cx < lo[0] + span[0] + 12) &
            (cy > lo[1]) & (cy < lo[1] + span[1] + 12))
    for x, y in zip(cx[keep], cy[keep]):
        draw.point(px((x, y)), fill=(200, 225, 205))
    for i, s in enumerate(stations):
        n = normals[i]
        clamped = "left_corridor_m" in s
        draw.line([px(xy[i] + n * s["left_m"]), px(xy[i] - n * s["right_m"])],
                  fill=(215, 215, 215), width=1)
        if clamped:
            draw.line([px(xy[i] + n * drawn_l[i]), px(xy[i] - n * drawn_r[i])],
                      fill=(30, 130, 60), width=2)
        else:
            draw.ellipse([px(xy[i])[0] - 3, px(xy[i])[1] - 3,
                          px(xy[i])[0] + 3, px(xy[i])[1] + 3], outline=(200, 40, 40))
    draw.line([px(p) for p in rxy], fill=(20, 20, 20), width=2)
    img.save(out_audit.rsplit(".", 1)[0] + ".png")

    aw = audit["admissible_width_m"]
    print(f"stations covered   : {covered.sum()}/{len(stations)} by the drawing")
    print(f"narrowed           : {len(narrowed)}   yielded: {len(yielded)}")
    print(f"usable limit median: {audit['usable_limit_m']['median_before']} -> "
          f"{audit['usable_limit_m']['median_after']} m")
    print(f"never widened      : {audit['usable_limit_m']['never_widened']}")
    print(f"empty stations     : {aw['empty_before']} -> {aw['empty_after']}"
          f"   no new: {aw['no_new_empty_stations']}")
    print(f"route outside      : {(~r_in).sum()}/{len(rxy)} wp in "
          f"{len(excursions)} excursions")


if __name__ == "__main__":
    main()
