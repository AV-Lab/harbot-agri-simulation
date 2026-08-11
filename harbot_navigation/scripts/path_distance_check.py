#!/usr/bin/env python3

import json
import math
import sys
from pathlib import Path

import rospkg


PATHS_DIR = Path(rospkg.RosPack().get_path("harbot_navigation")) / "paths"


def build_route_name(args):
    if len(args) == 1:
        route = args[0]
    elif len(args) == 2:
        route = f"{args[0]}-{args[1]}"
    else:
        raise ValueError("Use either one route like a29-a30 or two point names like a29 a30.")

    return route if route.endswith(".json") else f"{route}.json"


def find_route_file(route_name):
    matches = sorted(PATHS_DIR.rglob(route_name))
    if not matches:
        raise FileNotFoundError(f"No path file found for {route_name}")
    if len(matches) > 1:
        raise RuntimeError(
            "Multiple path files matched:\n" + "\n".join(str(match) for match in matches)
        )
    return matches[0]


def load_points(path_file):
    with path_file.open("r", encoding="utf-8") as handle:
        points = json.load(handle)
    if len(points) < 2:
        raise ValueError(f"{path_file} does not contain enough points to measure a distance.")
    return points


def compute_distance(points):
    total = 0.0
    for previous, current in zip(points, points[1:]):
        total += math.hypot(
            float(current[0]) - float(previous[0]),
            float(current[1]) - float(previous[1]),
        )
    return total


def main():
    try:
        route_name = build_route_name(sys.argv[1:])
        path_file = find_route_file(route_name)
        points = load_points(path_file)
        euclidean_total = compute_distance(points)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        print(
            "Usage: rosrun harbot_navigation path_distance_check.py a29-a30\n"
            "   or: rosrun harbot_navigation path_distance_check.py a29 a30",
            file=sys.stderr,
        )
        return 1

    print(f"Route file: {path_file}")
    print(f"Point count: {len(points)}")
    print(f"Euclidean distance: {euclidean_total:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
