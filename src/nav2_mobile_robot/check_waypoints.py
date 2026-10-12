#!/usr/bin/env python3
import math
import os
import sys

import yaml

WALL_EDGE = 0.975
BOXES = [
    (0.30, 0.50, 0.20, 0.40),
    (-0.50, -0.30, -0.40, -0.20),
]
ROBOT_RADIUS = 0.18
SAFE_CLEARANCE = 0.30


def clearance(x, y):
    distance = WALL_EDGE - max(abs(x), abs(y))

    for x_min, x_max, y_min, y_max in BOXES:
        dx = max(x_min - x, 0, x - x_max)
        dy = max(y_min - y, 0, y - y_max)
        distance = min(distance, math.hypot(dx, dy))

    return distance


def verdict(distance):
    if distance < ROBOT_RADIUS:
        return "UNREACHABLE - Nav2 treats this as a collision"
    if distance < SAFE_CLEARANCE:
        return "RISKY - reaches it, but retries and can wedge"
    return "ok"


def check_one_point(x, y):
    distance = clearance(x, y)
    print(f"({x:+.2f}, {y:+.2f})  clearance {distance:.3f} m  ->  {verdict(distance)}")


def check_fleet_file():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    fleet_file = os.path.join(this_folder, "config", "fleet.yaml")

    with open(fleet_file) as f:
        config = yaml.safe_load(f)
    waypoints = config["fleet_manager"]["ros__parameters"]["platforms"]

    worst = None
    for name, (x, y, yaw) in sorted(waypoints.items()):
        distance = clearance(x, y)
        print(f"  {name}  ({x:+.2f}, {y:+.2f})  clearance {distance:.3f} m  ->  {verdict(distance)}")
        if worst is None or distance < worst:
            worst = distance

    print(f"\nworst: {worst:.3f} m   (want >= {SAFE_CLEARANCE})")


def main():
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        check_one_point(x, y)
    else:
        check_fleet_file()


if __name__ == "__main__":
    main()
