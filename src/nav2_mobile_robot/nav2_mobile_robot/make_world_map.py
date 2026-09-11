"""Generate an exact occupancy grid for worlds/small_world.sdf.

The SLAM maps of this room (map_2156 et al.) stop 1-2 cm short of the north and
east walls, so lidar rays aimed at those walls land outside the grid. Every such
ray logs a worldToMap failure -- 2.68 million of them in one two-robot run --
and the planner has no wall to plan against on two sides.

This builds the grid from the world's own geometry instead, padded past the
walls, at 0.01 m/px rather than the SLAM maps' 0.003 (9x fewer costmap cells,
which matters with one costmap pair per robot).

    ros2 run nav2_mobile_robot make_world_map
"""

import os

import numpy as np
from PIL import Image

# Straight from worlds/small_world.sdf.
WALL_CENTRE = 1.00      # wall centreline offset from origin
WALL_THICK = 0.05
BOXES = [(0.40, 0.30), (-0.40, -0.30)]
BOX_SIDE = 0.20

RESOLUTION = 0.01
PAD = 0.10              # map extends this far beyond the wall centrelines

FREE, OCCUPIED = 255, 0


def build():
    extent = WALL_CENTRE + PAD
    n = int(round(2 * extent / RESOLUTION))
    # Cell centres, in world coordinates.
    coords = -extent + (np.arange(n) + 0.5) * RESOLUTION
    X, Y = np.meshgrid(coords, coords, indexing="xy")

    grid = np.full((n, n), FREE, dtype=np.uint8)

    half = WALL_THICK / 2.0
    outer = WALL_CENTRE + half
    for mask in (
        (np.abs(X - WALL_CENTRE) <= half),   # east
        (np.abs(X + WALL_CENTRE) <= half),   # west
        (np.abs(Y - WALL_CENTRE) <= half),   # north
        (np.abs(Y + WALL_CENTRE) <= half),   # south
    ):
        grid[mask & (np.abs(X) <= outer) & (np.abs(Y) <= outer)] = OCCUPIED

    for bx, by in BOXES:
        h = BOX_SIDE / 2.0
        grid[(np.abs(X - bx) <= h) & (np.abs(Y - by) <= h)] = OCCUPIED

    # PGM row 0 is the top of the image (max y); the map origin is bottom-left.
    return np.flipud(grid), n, -extent


def main():
    grid, n, origin = build()
    out = os.path.join(os.getcwd(), "small_world")
    Image.fromarray(grid, mode="L").save(out + ".pgm")
    with open(out + ".yaml", "w") as fh:
        fh.write(
            f"image: small_world.pgm\n"
            f"mode: trinary\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{origin:.3f}, {origin:.3f}, 0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )
    span = origin + n * RESOLUTION
    print(f"wrote {out}.pgm  {n}x{n} @ {RESOLUTION} m/px")
    print(f"covers x,y [{origin:+.3f}, {span:+.3f}] -- walls at +-1.000 are inside")
    print(f"free cells: {(grid == FREE).sum()} / {grid.size}")


if __name__ == "__main__":
    main()
