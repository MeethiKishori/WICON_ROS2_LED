# ros2_led_floor

Two namespaced robots (`r1`, `r2`) run their own waypoint routes at the same
time under Nav2, in a 2 × 2 m Gazebo world.

Nav2 plans and avoids; `peer_obstacles` makes each robot an obstacle to the
other; `fleet_manager` hands out waypoints and never sends two robots to the
same one.

## Build

```bash
cd ~/shubham/ros2_led_floor
colcon build --packages-select nav2_mobile_robot --symlink-install
```

Never run `colcon build` from inside `src/`.

## Run

Every terminal starts with:

```bash
cd ~/shubham/ros2_led_floor && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

**Terminal 1 — Gazebo** (add `headless:=-s` for no GUI). Wait ~10 s.
```bash
ros2 launch nav2_mobile_robot multi_gazebo.launch.py robots:=r1,r2
```

**Terminal 2 — Nav2, one stack per robot.** Wait for `Managed nodes are active`.
```bash
ros2 launch nav2_mobile_robot multi_nav2.launch.py robots:=r1,r2
```

**Terminal 3 — RViz** (optional)
```bash
ros2 launch nav2_mobile_robot multi_rviz.launch.py robots:=r1,r2
```

**Terminal 4 — peer_obstacles + fleet_manager**
```bash
ros2 launch nav2_mobile_robot fleet.launch.py robots:=r1,r2
```

Ctrl-C in terminal 4 stops both robots and prints a summary.

## Checks (terminal 5)

```bash
# exactly one of each, or they fight over the robots
ros2 node list | grep -c '^/fleet_manager$'

# the map server often loses its activation race
ros2 lifecycle get /map_server          # want: active
ros2 lifecycle set /map_server activate
```

## Stop

```bash
pkill -f fleet.launch.py
pkill -TERM -f multi_nav2.launch.py
pkill -TERM -f multi_gazebo.launch.py
```

Use `-TERM`, not `-9`. A `kill -9` leaves FastDDS shared memory locked and the
next run hangs on `waiting for Nav2 servers`; if that happens:
`rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`

## Waypoints

Edit `src/nav2_mobile_robot/config/fleet.yaml`, then rebuild with
`--symlink-install`.

```yaml
platforms:
  P1: [0.00, 0.75, -1.571]    # x, y, yaw(radians)
  P3: [0.75, 0.00,  3.142]    # a name used by both routes is shared
routes:
  r1: [P1, P2, P3, P4]
  r2: [P5, P6, P3, P7]
```

### How many?

- **Waypoints: any number.** Routes cycle with `index % len(route)`, so add as
  many as you like. The two routes need not be the same length, and they may
  share any, all or none of their waypoints.
- **Robots: 4 maximum** — `multi_gazebo.launch.py` has four validated spawn
  poses and raises if you ask for more.

### Allowable x, y

The world is 1.95 × 1.95 m with two 0.20 m boxes. The robot is 0.36 m across,
so a waypoint needs room around it, not just to be inside the walls.

| clearance to nearest wall/box | behaviour |
|---|---|
| < 0.18 m | unreachable — Nav2 calls it a collision |
| 0.18 – 0.30 m | reaches it, but retries and occasionally gets stuck |
| **≥ 0.30 m** | **reliable** |

Rule of thumb for the reliable band:

```
|x| <= 0.675  and  |y| <= 0.675
and >= 0.30 m from box1  (x 0.30..0.50, y  0.20..0.40)
and >= 0.30 m from box2  (x -0.50..-0.30, y -0.40..-0.20)
```

Which leaves two areas — the boxes sit on the NE–SW diagonal, so the open
ground is the NW–SE one:

```
   y=+0.60  ##.....######
   y=+0.30  ##.....######      '.' = reliable
   y=+0.00  ######.######      '#' = too close
   y=-0.30  ######.....##
   y=-0.60  ######.....##
            x=-0.90 ... +0.90
```

Check a candidate before adding it:

```python
import math
WALL = 0.975
BOXES = [(0.30, 0.50, 0.20, 0.40), (-0.50, -0.30, -0.40, -0.20)]
def clearance(x, y):
    d = WALL - max(abs(x), abs(y))
    for x0, x1, y0, y1 in BOXES:
        d = min(d, math.hypot(max(x0-x, 0, x-x1), max(y0-y, 0, y-y1)))
    return d
```

> The waypoints shipped in `fleet.yaml` sit at 0.22 m clearance — inside the
> middle band. They work, but that is why you occasionally see
> `could not reach P3, retrying`. Move them inside 0.675 to stop it.

`yaw` is the heading the robot finishes in, in radians. It only affects the
final turn, so any value is valid.

## Other packages

`led_panel` drives the 640 × 640 LED floor; `m-explore-ros2` is vendored
frontier exploration. Neither is used by the fleet.
