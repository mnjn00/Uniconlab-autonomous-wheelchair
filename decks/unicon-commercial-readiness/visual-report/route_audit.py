from __future__ import annotations

# ruff: noqa: I001

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from pydantic import BaseModel


REPO_ROOT: Final = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "src/static_livox_localization/scripts"))
from safety_band import SafetyBand

GRACE_M: Final = 0.10


class RuntimeSourceMismatchError(ValueError):
    pass


class Station(BaseModel, frozen=True):
    x: float
    y: float
    heading_deg: float
    left_m: float
    right_m: float


class BandDocument(BaseModel, frozen=True):
    stations: tuple[Station, ...]


class Waypoint(BaseModel, frozen=True):
    x: float
    y: float


class RouteDocument(BaseModel, frozen=True):
    waypoints: tuple[Waypoint, ...]


@dataclass(frozen=True, slots=True)
class RouteAudit:
    station_count: int
    route_waypoint_count: int
    grace_m: float
    rejected_station_indices: tuple[int, ...]
    inverted_station_indices: tuple[int, ...]
    failed_station_chord_indices: tuple[int, ...]
    rejected_route_waypoint_indices: tuple[int, ...]
    failed_route_chord_indices: tuple[int, ...]


def load_documents(
    band_path: Path, route_path: Path
) -> tuple[BandDocument, RouteDocument]:
    band = BandDocument.model_validate_json(band_path.read_text(encoding="utf-8"))
    route = RouteDocument.model_validate_json(route_path.read_text(encoding="utf-8"))
    return band, route


def audit_route_bundle(
    band_path: Path, route_path: Path, safety_band_path: Path
) -> RouteAudit:
    if not safety_band_path.is_file():
        raise FileNotFoundError(safety_band_path)
    imported_source = REPO_ROOT / "src/static_livox_localization/scripts/safety_band.py"
    if safety_band_path.resolve() != imported_source.resolve():
        raise RuntimeSourceMismatchError(
            f"Expected runtime source {imported_source}, got {safety_band_path}"
        )
    band_doc, route_doc = load_documents(band_path, route_path)
    runtime = SafetyBand(str(band_path))
    station_xy = tuple((item.x, item.y) for item in band_doc.stations)
    route_xy = tuple((item.x, item.y) for item in route_doc.waypoints)
    rejected_stations = tuple(
        index
        for index, point in enumerate(station_xy)
        if not runtime.contains(point, grace=GRACE_M)
    )
    inverted = tuple(
        index
        for index, point in enumerate(station_xy)
        if runtime.lateral_limits(point)[1] > runtime.lateral_limits(point)[2]
    )
    failed_station_chords = tuple(
        index
        for index in range(len(station_xy) - 1)
        if not runtime.chord_is_contained(
            station_xy[index], station_xy[index + 1], grace=GRACE_M
        )
    )
    rejected_waypoints = tuple(
        index
        for index, point in enumerate(route_xy)
        if not runtime.contains(point, grace=GRACE_M)
    )
    failed_route_chords = tuple(
        index
        for index in range(len(route_xy) - 1)
        if not runtime.chord_is_contained(
            route_xy[index], route_xy[index + 1], grace=GRACE_M
        )
    )
    return RouteAudit(
        len(station_xy),
        len(route_xy),
        GRACE_M,
        rejected_stations,
        inverted,
        failed_station_chords,
        rejected_waypoints,
        failed_route_chords,
    )
