from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Final

import numpy as np
from numpy.typing import NDArray
from PIL import Image, ImageDraw, ImageFont
from route_audit import RouteAudit, Station, Waypoint

BLUE: Final = "#0B5FFF"
INK: Final = "#1A1A1A"
BORDER: Final = "#D1D5DB"
MUTED: Final = "#6B7280"
WHITE: Final = "#FFFFFF"
CLUSTERS: Final = (
    ("A", 96, 114),
    ("B", 124, 152),
    ("C", 201, 213),
    ("D", 285, 288),
    ("E", 368, 370),
)


class PcdFormatError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class PlotData:
    points: NDArray[np.float32]
    stations: tuple[Station, ...]
    waypoints: tuple[Waypoint, ...]
    audit: RouteAudit


def load_xyzi(path: Path) -> NDArray[np.float32]:
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise PcdFormatError("PCD DATA header not found")
            if line.startswith(b"DATA binary"):
                offset = stream.tell()
                break
    values = np.fromfile(path, dtype="<f4", offset=offset)
    if values.size % 4:
        raise PcdFormatError("PCD payload is not XYZI float32")
    return values.reshape((-1, 4))[:, :2]


def plot_bounds(
    stations: tuple[Station, ...], start: int, end: int, padding: float
) -> tuple[float, float, float, float]:
    subset = stations[start : end + 1]
    xs = [item.x for item in subset]
    ys = [item.y for item in subset]
    return min(xs) - padding, max(xs) + padding, min(ys) - padding, max(ys) + padding


def _pixel(
    x: float,
    y: float,
    bounds: tuple[float, float, float, float],
    size: tuple[int, int],
) -> tuple[int, int]:
    x0, x1, y0, y1 = bounds
    width, height = size
    return (
        round((x - x0) / (x1 - x0) * (width - 1)),
        round((y1 - y) / (y1 - y0) * (height - 1)),
    )


def _base_map(
    points: NDArray[np.float32],
    bounds: tuple[float, float, float, float],
    size: tuple[int, int],
) -> Image.Image:
    x0, x1, y0, y1 = bounds
    mask = (
        (points[:, 0] >= x0)
        & (points[:, 0] <= x1)
        & (points[:, 1] >= y0)
        & (points[:, 1] <= y1)
    )
    width, height = size
    density, _, _ = np.histogram2d(
        points[mask, 1],
        points[mask, 0],
        bins=(height, width),
        range=((y0, y1), (x0, x1)),
    )
    scaled = np.log1p(density)
    ceiling = max(float(np.percentile(scaled, 99.5)), 1.0)
    gray = 244 - np.clip(scaled / ceiling, 0.0, 1.0) * 125
    raster = np.flipud(gray.astype(np.uint8))
    return Image.fromarray(np.stack((raster, raster, raster), axis=2), mode="RGB")


def _draw_dashed(
    draw: ImageDraw.ImageDraw,
    first: tuple[int, int],
    second: tuple[int, int],
) -> None:
    dx, dy = second[0] - first[0], second[1] - first[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return
    for offset in np.arange(0.0, length, 16.0):
        end = min(offset + 9.0, length)
        start_xy = (first[0] + dx * offset / length, first[1] + dy * offset / length)
        end_xy = (first[0] + dx * end / length, first[1] + dy * end / length)
        draw.line((start_xy, end_xy), fill=BLUE, width=6)


def draw_plot(
    data: PlotData,
    bounds: tuple[float, float, float, float],
    size: tuple[int, int],
) -> Image.Image:
    image = _base_map(data.points, bounds, size)
    draw = ImageDraw.Draw(image)
    for side in ("left", "right"):
        edge: list[tuple[int, int]] = []
        for station in data.stations:
            heading = math.radians(station.heading_deg)
            normal = (-math.sin(heading), math.cos(heading))
            distance = station.left_m if side == "left" else -station.right_m
            edge.append(
                _pixel(
                    station.x + normal[0] * distance,
                    station.y + normal[1] * distance,
                    bounds,
                    size,
                )
            )
        draw.line(edge, fill=MUTED, width=2)
    station_pixels = [_pixel(item.x, item.y, bounds, size) for item in data.stations]
    route_pixels = [_pixel(item.x, item.y, bounds, size) for item in data.waypoints]
    draw.line(station_pixels, fill=INK, width=3)
    for index in data.audit.failed_station_chord_indices:
        draw.line(
            (station_pixels[index], station_pixels[index + 1]), fill=BLUE, width=9
        )
    for index in data.audit.failed_route_chord_indices:
        _draw_dashed(draw, route_pixels[index], route_pixels[index + 1])
    for index in data.audit.rejected_station_indices:
        x, y = station_pixels[index]
        draw.ellipse((x - 7, y - 7, x + 7, y + 7), fill=BLUE, outline=WHITE, width=2)
    for index in data.audit.rejected_route_waypoint_indices:
        x, y = route_pixels[index]
        draw.polygon(((x, y - 9), (x + 9, y), (x, y + 9), (x - 9, y)), fill=BLUE)
    return image


def render_assets(data: PlotData, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    overview_bounds = plot_bounds(data.stations, 0, len(data.stations) - 1, 12.0)
    overview = draw_plot(data, overview_bounds, (1800, 920))
    overview_draw = ImageDraw.Draw(overview)
    label_font = ImageFont.truetype(
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf", 30
    )
    for label, start, end in CLUSTERS:
        midpoint = data.stations[(start + end) // 2]
        x, y = _pixel(midpoint.x, midpoint.y, overview_bounds, overview.size)
        overview_draw.rectangle((x - 20, y - 20, x + 20, y + 20), fill=BLUE)
        overview_draw.text((x - 10, y - 17), label, fill=WHITE, font=label_font)
    overview.save(output_dir / "route-band-overview.png", optimize=True)

    canvas = Image.new("RGB", (1800, 1120), WHITE)
    canvas_draw = ImageDraw.Draw(canvas)
    for position, (label, start, end) in enumerate(CLUSTERS):
        col, row = position % 3, position // 3
        left, top = 32 + col * 590, 32 + row * 535
        bounds = plot_bounds(data.stations, start, end, 8.0)
        plot = draw_plot(data, bounds, (550, 450))
        canvas.paste(plot, (left, top + 52))
        canvas_draw.rectangle(
            (left, top, left + 550, top + 502), outline=BORDER, width=2
        )
        canvas_draw.text(
            (left + 16, top + 10),
            f"{label}  stations {start}-{end}",
            font=label_font,
            fill=INK,
        )
    canvas.save(output_dir / "route-band-hotspots.png", optimize=True)
