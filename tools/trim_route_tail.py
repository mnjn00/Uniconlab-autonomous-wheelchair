#!/usr/bin/env python3
"""Cut a route short of a corner the chair cannot physically take.

build_preferred_mask_route.py smooths with Savitzky-Golay and never checks the
curvature it leaves behind, so a mask that is drivable everywhere can still
produce a route with a corner tighter than the base can turn. Those corners are
a full stop: curvature_speed asks for less than TURN_FLOOR_SPEED, mpc_speed
returns STOP, and because the condition depends on position and the position
cannot change, the follower never recovers. This was the 08-15 drive.

When such a corner is at the *end* of the route, trimming is the honest fix and
the cheap one - the chair drives the whole route bar the last few metres. This
tool does only that. A blocked corner in the middle is not something trimming
can fix, so it refuses rather than silently truncating most of the route.

The route, band and mask are bound to each other by SHA-256, so both documents
are rewritten together and the binding is recomputed and then re-validated.
"""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "src" / "static_livox_localization"
                       / "scripts"))

import mpc_speed                                        # noqa: E402
from route_assets import (route_content_sha256, sha256,  # noqa: E402
                          validate_asset_binding)
from safety_band import SafetyBand                      # noqa: E402


def _write(path, text):
    """Always LF.

    These files are hashed byte for byte and the hash travels to the NUC, so
    a run of this tool on Windows has to produce the same bytes as a run on
    Linux. Python's text mode would otherwise turn every newline into CRLF
    and the binding would only validate on the machine that wrote it.
    """
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def blocked_runs(band_path):
    """Contiguous stations whose curvature demands less than the turn floor."""
    band = SafetyBand(str(band_path))
    floor = mpc_speed.TURN_FLOOR_SPEED
    speeds = []
    for point in band.xy:
        try:
            speeds.append(float(mpc_speed.curvature_speed(band, point)))
        except Exception:
            speeds.append(float("nan"))
    speeds = np.array(speeds)
    blocked = speeds < floor

    runs, start = [], None
    for i, bad in enumerate(blocked):
        if bad and start is None:
            start = i
        elif not bad and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(blocked) - 1))
    return runs, speeds, len(band.xy)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", type=Path, required=True)
    parser.add_argument("--band", type=Path, required=True)
    parser.add_argument("--out-route", type=Path, required=True)
    parser.add_argument("--out-band", type=Path, required=True)
    parser.add_argument("--mask-yaml", type=Path, default=None,
                        help="drivable mask yaml, to validate the binding")
    parser.add_argument("--id-suffix", default=None,
                        help="appended to route_id; defaults to -tN")
    parser.add_argument("--keep-stations", type=int, default=None,
                        help="override the automatic cut point")
    parser.add_argument("--max-trim-m", type=float, default=15.0,
                        help="refuse to discard more than this much route")
    args = parser.parse_args()

    runs, speeds, n_stations = blocked_runs(args.band)
    band = json.loads(args.band.read_text(encoding="utf-8"))
    route = json.loads(args.route.read_text(encoding="utf-8"))
    spacing = float(band.get("station_spacing_m", 0.5))

    if args.keep_stations is not None:
        keep = int(args.keep_stations)
    else:
        if not runs:
            print("막힌 코너 없음 - 자를 것이 없습니다.")
            return 0
        tail_start = runs[-1][0]
        # Only a run that reaches the end can be trimmed away. Anything the
        # chair would meet before that has to be re-drawn, not cut.
        earlier = [r for r in runs if r[0] < tail_start]
        if earlier:
            print("꼬리 아닌 곳에 막힌 구간이 %d개 있습니다. 절단으로 해결되지 "
                  "않습니다:" % len(earlier), file=sys.stderr)
            for a, b in earlier:
                print("  스테이션 %d-%d  최소 %.3f m/s"
                      % (a, b, float(np.nanmin(speeds[a:b + 1]))),
                      file=sys.stderr)
            return 2
        keep = tail_start

    discarded_m = (n_stations - keep) * spacing
    if discarded_m > args.max_trim_m:
        print("버리는 길이 %.1f m 가 한도 %.1f m 를 넘습니다."
              % (discarded_m, args.max_trim_m), file=sys.stderr)
        return 2
    if keep < 2:
        print("남는 스테이션이 없습니다.", file=sys.stderr)
        return 2

    # Band first: the stations are the authority on where the route now ends.
    band["stations"] = band["stations"][:keep]
    if isinstance(band.get("corridor"), dict):
        for field in ("stations_covered", "stations_total"):
            if field in band["corridor"]:
                band["corridor"][field] = keep
    last = band["stations"][-1]
    end_xy = np.array([float(last["x"]), float(last["y"])])

    # Then the dense waypoints, cut at the one nearest the new last station
    # rather than at a computed index: the two files have different spacings
    # and nothing guarantees the ratio is exact.
    pts = np.array([[float(w["x"]), float(w["y"])] for w in route["waypoints"]])
    cut = int(np.argmin(np.linalg.norm(pts - end_xy, axis=1)))
    route["waypoints"] = route["waypoints"][:cut + 1]
    kept = np.array([[float(w["x"]), float(w["y"])]
                     for w in route["waypoints"]])
    route["count"] = len(route["waypoints"])
    route["path_length_m"] = round(
        float(np.sum(np.linalg.norm(np.diff(kept, axis=0), axis=1))), 3)
    if "operator_target_waypoint_index" in route:
        route["operator_target_waypoint_index"] = route["count"] - 1
    if "operator_target_xy_m" in route:
        route["operator_target_xy_m"] = [round(float(kept[-1][0]), 3),
                                         round(float(kept[-1][1]), 3)]

    # A trimmed route is a different route, and the id is what the follower
    # checks the band against, so it has to say so.
    suffix = args.id_suffix or ("-t%d" % keep)
    old_id = str(route.get("asset_binding", {}).get("route_id",
                                                    band.get("route_id", "")))
    if suffix not in old_id:
        head, sep, rest = old_id.partition(":")
        new_id = head + suffix + sep + rest
    else:
        new_id = old_id
    band["route_id"] = new_id
    route.setdefault("asset_binding", {})["route_id"] = new_id
    route["trimmed_from"] = {
        "source_route": args.route.name,
        "reason": "trailing corner below TURN_FLOOR_SPEED",
        "stations_before": n_stations,
        "stations_after": keep,
        "discarded_m": round(discarded_m, 2),
    }

    _write(args.out_band, json.dumps(band, indent=1))
    route["asset_binding"]["safety_band_sha256"] = sha256(args.out_band)
    route["asset_binding"].pop("route_content_sha256", None)
    route["asset_binding"]["route_content_sha256"] = route_content_sha256(route)
    _write(args.out_route, json.dumps(route, indent=1))

    validate_asset_binding(args.out_route, args.out_band, args.mask_yaml)

    print("스테이션 %d -> %d,  버린 길이 %.2f m" % (n_stations, keep, discarded_m))
    print("경유점  %d -> %d,  경로 길이 %.1f m"
          % (len(pts), route["count"], route["path_length_m"]))
    print("route_id  %s" % new_id)
    print("바인딩 검증 통과")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
