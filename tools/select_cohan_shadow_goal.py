#!/usr/bin/env python3
"""Choose a nearby route goal from the first pose in a replay bag."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import cast


def waypoint_xy(waypoint: Mapping[str, object]) -> tuple[float, float]:
    try:
        raw_x = waypoint["x"]
        raw_y = waypoint["y"]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("route contains an invalid waypoint") from error
    if (
            isinstance(raw_x, bool)
            or isinstance(raw_y, bool)
            or not isinstance(raw_x, (str, int, float))
            or not isinstance(raw_y, (str, int, float))):
        raise TypeError("route contains a non-numeric waypoint")
    x_m = float(raw_x)
    y_m = float(raw_y)
    if not math.isfinite(x_m) or not math.isfinite(y_m):
        raise ValueError("route contains a non-finite waypoint")
    return x_m, y_m


def select_goal(
        waypoints: Sequence[Mapping[str, object]],
        start_x_m: float,
        start_y_m: float,
        lookahead_m: float = 5.0,
        max_straight_distance_m: float = 10.0) -> tuple[float, float]:
    """Return the first route point one lookahead ahead of the nearest point."""
    parsed = [waypoint_xy(waypoint) for waypoint in waypoints]
    if not parsed:
        raise ValueError("route contains no waypoints")
    nearest = min(
        range(len(parsed)),
        key=lambda index: math.hypot(
            parsed[index][0] - start_x_m,
            parsed[index][1] - start_y_m,
        ),
    )
    selected = nearest
    arc_m = 0.0
    for index in range(nearest + 1, len(parsed)):
        arc_m += math.hypot(
            parsed[index][0] - parsed[index - 1][0],
            parsed[index][1] - parsed[index - 1][1],
        )
        if math.hypot(
                parsed[index][0] - start_x_m,
                parsed[index][1] - start_y_m) <= max_straight_distance_m:
            selected = index
        if arc_m >= lookahead_m:
            break
    return parsed[selected]


def pose_from_csv(path: Path) -> tuple[float, float]:
    with path.open(newline="", encoding="utf-8") as stream:
        row = next(csv.DictReader(stream), None)
    if row is None:
        raise ValueError("pose CSV contains no message")

    def field(suffix: str) -> float:
        values = [
            value for key, value in row.items()
            if key.endswith(suffix) and value is not None
        ]
        if len(values) != 1:
            raise ValueError(f"pose CSV has no unique {suffix}")
        parsed = float(values[0])
        if not math.isfinite(parsed):
            raise ValueError(f"pose CSV has non-finite {suffix}")
        return parsed

    return field(".pose.pose.position.x"), field(".pose.pose.position.y")


def main():
    parser = argparse.ArgumentParser()
    _ = parser.add_argument("--pose-csv", required=True)
    _ = parser.add_argument("--route", required=True)
    args = parser.parse_args()
    route_path = Path(cast(str, args.route))
    route_data = cast(
        object,
        json.loads(route_path.read_text(encoding="utf-8")),
    )
    if not isinstance(route_data, dict):
        raise TypeError("route is not an object")
    route_object = cast("dict[str, object]", route_data)
    raw_waypoints = route_object.get("waypoints")
    if not isinstance(raw_waypoints, list):
        raise TypeError("route has no waypoint list")
    raw_items = cast("list[object]", raw_waypoints)
    if not all(isinstance(item, dict) for item in raw_items):
        raise TypeError("route has a non-object waypoint")
    waypoints = cast(
        "list[dict[str, object]]",
        raw_items,
    )
    start_x_m, start_y_m = pose_from_csv(
        Path(cast(str, args.pose_csv)))
    goal_x_m, goal_y_m = select_goal(waypoints, start_x_m, start_y_m)
    print(goal_x_m, goal_y_m)


if __name__ == "__main__":
    main()
