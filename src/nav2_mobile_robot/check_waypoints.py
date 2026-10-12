#!/usr/bin/env python3
"""Check waypoint clearance against the small_world geometry.

    python3 check_waypoints.py                # check every waypoint in fleet.yaml
    python3 check_waypoints.py 0.65 0.0       # check one candidate x y
"""
import math
import os
import sys

WALL = 0.975                                             # interior wall face
BOXES = [(0.30, 0.50, 0.20, 0.40),                       # box1: x0,x1,y0,y1
         (-0.50, -0.30, -0.40, -0.20)]                   # box2
ROBOT_RADIUS = 0.18
RELIABLE = 0.30


def clearance(x, y):
    """Distance from (x, y) to the nearest wall or box."""
    d = WALL - max(abs(x), abs(y))
    for x0, x1, y0, y1 in BOXES:
        d = min(d, math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)))
    return d


def verdict(c):
    if c < ROBOT_RADIUS:
        return "UNREACHABLE - Nav2 treats this as a collision"
    if c < RELIABLE:
        return "RISKY - reaches it, but retries and can wedge"
    return "ok"


def main():
    if len(sys.argv) == 3:
        x, y = float(sys.argv[1]), float(sys.argv[2])
        c = clearance(x, y)
        print(f"({x:+.2f}, {y:+.2f})  clearance {c:.3f} m  ->  {verdict(c)}")
        return

    import yaml
    cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "fleet.yaml")
    wp = yaml.safe_load(open(cfg))["fleet_manager"]["ros__parameters"]["platforms"]
    worst = min(clearance(v[0], v[1]) for v in wp.values())
    for name, v in sorted(wp.items()):
        c = clearance(v[0], v[1])
        print(f"  {name}  ({v[0]:+.2f}, {v[1]:+.2f})  clearance {c:.3f} m  ->  {verdict(c)}")
    print(f"\nworst: {worst:.3f} m   (want >= {RELIABLE})")


main()
