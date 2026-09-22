"""Hand each robot its next waypoint, and keep two robots off the same spot.

Nav2 already does the hard parts: it plans, it replans, and because
peer_obstacles paints each robot into the other's costmap, it steers around a
peer or finds another way round. None of that needs repeating here.

So this node does two things and nothing else:

  1. walk each robot through its own list of waypoints, looping forever;
  2. refuse to send a robot to a spot another robot is sitting on, or already
     heading for -- whoever asked first keeps it, the other waits and tries
     again next tick.

Everything else (avoiding the peer en route, backing out, picking a different
path) is Nav2's job.

    ros2 launch nav2_mobile_robot fleet.launch.py robots:=r1,r2
"""

import math
import signal
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

# The global costmap is latched: it is published once and then only on change.
# A default (VOLATILE) subscription joins too late and receives nothing, which
# silently disables the escape check -- so ask for the latched copy.
COSTMAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

IDLE = "IDLE"              # ready for its next waypoint
DRIVING = "DRIVING"        # goal in flight
DWELL = "DWELL"            # parked at a waypoint, serving its pause
ESCAPING = "ESCAPING"      # backing out of the inflation band before carrying on


class Robot:
    def __init__(self, node, name, route):
        self.node = node
        self.name = name
        self.route = route

        self.state = IDLE
        self.index = 0
        self.pose = None        # (x, y) from odom
        self.target = None      # waypoint name it is heading for, if any
        self.handle = None
        self.since = 0.0
        self.waiting_since = None
        self.legs = 0
        self.laps = 0
        self.failed = 0

        self.costmap = None       # this robot's global costmap
        self.trapped_since = None  # when it first entered the inflation band
        self.escapes = 0           # consecutive failed attempts to get clear

        self.nav = ActionClient(node, NavigateToPose, f"/{name}/navigate_to_pose")
        node.create_subscription(Odometry, f"/{name}/odom", self._odom, 10)
        node.create_subscription(
            OccupancyGrid, f"/{name}/global_costmap/costmap", self._cm, COSTMAP_QOS
        )

    def _cm(self, msg):
        self.costmap = msg

    def cost(self, x, y):
        """Costmap value at a point: 0 free, 1-98 inflation, 99 inscribed, 100 lethal."""
        g = self.costmap
        if g is None:
            return None
        mx = int((x - g.info.origin.position.x) / g.info.resolution)
        my = int((y - g.info.origin.position.y) / g.info.resolution)
        if 0 <= mx < g.info.width and 0 <= my < g.info.height:
            return g.data[my * g.info.width + mx]
        return None

    def cost_here(self):
        return self.cost(*self.pose) if self.pose else None

    def way_out(self, search, step=0.05):
        """Nearest genuinely free spot to retreat to.

        Spirals outward and takes the first point that is free *and* whose
        immediate surroundings are free too, so the robot does not escape one
        obstacle by parking against another.
        """
        if self.pose is None or self.costmap is None:
            return None
        x0, y0 = self.pose
        rings = int(search / step)
        for i in range(2, rings + 1):
            r = i * step
            for k in range(int(2 * math.pi * r / step)):
                a = 2 * math.pi * k / max(1, int(2 * math.pi * r / step))
                x, y = x0 + r * math.cos(a), y0 + r * math.sin(a)
                if self.cost(x, y) != 0:
                    continue
                # NB: compare to 0 explicitly. `cost or 99` is wrong here --
                # a cost of 0 is falsy, so that test can never pass.
                if all(
                    self.cost(x + 0.12 * math.cos(b), y + 0.12 * math.sin(b)) == 0
                    for b in (0, math.pi / 2, math.pi, 3 * math.pi / 2)
                ):
                    return x, y
        return None

    def _odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    @property
    def next_waypoint(self):
        return self.route[self.index % len(self.route)]

    def enter(self, state):
        self.state = state
        self.since = self.node.now()


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager", automatically_declare_parameters_from_overrides=True)

        self.lock = threading.RLock()
        self.dwell = self._param("dwell", 1.5)
        self.loop = self._param("loop", True)
        # Two robots are 0.36 m across. Keep their destinations at least this
        # far apart so one never drives onto the other.
        self.min_separation = self._param("min_separation", 0.55)
        self.goal_timeout = self._param("goal_timeout", 120.0)
        self.retry_delay = self._param("retry_delay", 3.0)
        self.tick_period = self._param("tick_period", 1.0)
        # Hysteresis on the costmap's own inflation rings, acting on the OUTER
        # one. Once a robot reaches the inscribed ring (cost 99) nothing can
        # move it: a nav goal, a raw cmd_vel and Nav2's BackUp action all
        # collision-check first and refuse. So act earlier, at cost ~50, where
        # the robot is inside the inflation band but still outside its own
        # footprint radius and therefore still drivable. Clear only at cost 0,
        # fully outside the band -- that gap is what stops it chattering.
        self.escape_cost = int(self._param("escape_cost", 50))
        self.escape_after = self._param("escape_after", 5.0)
        self.escape_search = self._param("escape_search", 0.70)
        # After this many failed attempts, give up on the waypoint and move to
        # the next one. A robot that cannot be freed at least stops holding a
        # waypoint the other robot is waiting for.
        self.escape_attempts = int(self._param("escape_attempts", 3))

        self.waypoints = {
            name: tuple(float(v) for v in p.value)
            for name, p in self.get_parameters_by_prefix("platforms").items()
        }
        routes = {n: list(p.value) for n, p in self.get_parameters_by_prefix("routes").items()}
        if not self.waypoints or not routes:
            raise RuntimeError("no platforms/routes -- launch via fleet.launch.py")

        picked = [n for n in self._param("robots", []) if n]
        names = [n for n in sorted(routes) if not picked or n in picked]
        unknown = {w for n in names for w in routes[n] if w not in self.waypoints}
        if unknown:
            raise RuntimeError(f"routes use undefined waypoints: {sorted(unknown)}")

        self.robots = [Robot(self, n, routes[n]) for n in names]
        self.create_timer(self.tick_period, self._tick)
        self.started = False

        for r in self.robots:
            self.get_logger().info(f"{r.name}: {' -> '.join(r.route)}")
        self.get_logger().info(
            f"keeping destinations {self.min_separation:.2f} m apart; Nav2 does the rest"
        )

    def _param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------

    def _taken(self, robot, spot):
        """Is another robot sitting on `spot`, or already driving to it?"""
        for other in self.robots:
            if other is robot:
                continue
            if other.pose and math.dist(other.pose, spot) < self.min_separation:
                return f"{other.name} is there"
            if other.target:
                goal = self.waypoints[other.target][:2]
                if math.dist(goal, spot) < self.min_separation:
                    return f"{other.name} is heading there"
        return None

    def _tick(self):
        with self.lock:
            if not self.started:
                if any(r.pose is None for r in self.robots):
                    return
                if not all(r.nav.server_is_ready() for r in self.robots):
                    self.get_logger().info("waiting for Nav2 ...", throttle_duration_sec=10.0)
                    return
                self.started = True
                self.get_logger().info("all robots up -- running waypoints")
                for r in self.robots:
                    r.since = self.now()

            now = self.now()
            # Longest-waiting robot asks first, so nobody is starved.
            order = sorted(
                self.robots,
                key=lambda r: r.waiting_since if r.waiting_since is not None else now,
            )
            for robot in order:
                if self._check_trapped(robot, now):
                    continue
                if robot.state == DWELL and now - robot.since >= self.dwell:
                    robot.enter(IDLE)
                if robot.state == DRIVING and now - robot.since > self.goal_timeout:
                    self.get_logger().warn(f"{robot.name}: giving up on {robot.target}")
                    if robot.handle is not None:
                        robot.handle.cancel_goal_async()
                    self._done(robot, ok=False)
                if robot.state != IDLE:
                    continue

                spot = self.waypoints[robot.next_waypoint][:2]
                busy = self._taken(robot, spot)
                if busy:
                    if robot.waiting_since is None:
                        robot.waiting_since = now
                        self.get_logger().info(
                            f"{robot.name}: waiting for {robot.next_waypoint} -- {busy}"
                        )
                    continue
                robot.waiting_since = None
                self._send(robot)

    def _check_trapped(self, robot, now):
        """Two-ring hysteresis. Returns True if the robot is being rescued."""
        cost = robot.cost_here()
        if cost is None:
            return robot.state == ESCAPING

        if cost >= self.escape_cost:
            if robot.trapped_since is None:
                robot.trapped_since = now
        elif cost == 0:
            # Only a full exit from the inflation band counts as clear -- that
            # gap between the rings is the hysteresis.
            robot.trapped_since = None

        if robot.state == ESCAPING:
            if cost == 0:
                self.get_logger().info(f"{robot.name}: clear again, resuming")
                robot.escapes = 0
                if robot.handle is not None:
                    robot.handle.cancel_goal_async()
                    robot.handle = None
                robot.target = None
                robot.enter(IDLE)
                return False
            if now - robot.since > self.escape_after * 2:
                robot.escapes += 1
                if robot.handle is not None:
                    robot.handle.cancel_goal_async()
                    robot.handle = None
                if robot.escapes >= self.escape_attempts:
                    # Cannot be freed. Nav2 will not drive a robot this close to
                    # an obstacle -- goals, cmd_vel and BackUp all refuse -- so
                    # skip the waypoint rather than block the other robot too.
                    robot.escapes = 0
                    robot.trapped_since = None
                    robot.index += 1
                    self.get_logger().error(
                        f"{robot.name}: cannot get clear -- skipping to "
                        f"{robot.next_waypoint}. It may need moving by hand."
                    )
                robot.target = None
                robot.enter(IDLE)
                return False
            return True

        if robot.trapped_since is None or now - robot.trapped_since < self.escape_after:
            return False

        spot = robot.way_out(self.escape_search)
        if spot is None:
            self.get_logger().warn(
                f"{robot.name}: stuck at cost {cost} and nowhere free within "
                f"{self.escape_search:.2f} m"
            )
            robot.trapped_since = now      # back off, try again later
            return False

        self.get_logger().warn(
            f"{robot.name}: inside the inflation ring (cost {cost}) for "
            f"{now - robot.trapped_since:.0f}s -- backing out to "
            f"({spot[0]:+.2f}, {spot[1]:+.2f}) before carrying on"
        )
        if robot.handle is not None:
            robot.handle.cancel_goal_async()
            robot.handle = None
        robot.target = None
        self._send_pose(robot, spot[0], spot[1], 0.0, ESCAPING)
        return True

    def _send(self, robot):
        robot.target = robot.next_waypoint
        x, y, yaw = self.waypoints[robot.target]
        self.get_logger().info(f"{robot.name} -> {robot.target} ({x:+.2f}, {y:+.2f})")
        self._send_pose(robot, x, y, yaw, DRIVING)

    def _send_pose(self, robot, x, y, yaw, state):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        robot.enter(state)
        robot.nav.send_goal_async(goal).add_done_callback(
            lambda f, r=robot: self._accepted(f, r)
        )

    def _accepted(self, future, robot):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f"{robot.name}: goal rejected")
            with self.lock:
                self._done(robot, ok=False)
            return
        with self.lock:
            robot.handle = handle
        handle.get_result_async().add_done_callback(lambda f, r=robot: self._result(f, r))

    def _result(self, future, robot):
        result = future.result()
        with self.lock:
            robot.handle = None
            if robot.state == ESCAPING:
                # Arrived at the retreat spot (or failed to). Either way the
                # hysteresis check decides when it is safe to resume.
                robot.enter(ESCAPING)
                return
            if robot.state != DRIVING:
                return
            if getattr(result, "status", None) == GoalStatus.STATUS_SUCCEEDED:
                took = self.now() - robot.since
                self.get_logger().info(f"{robot.name}: reached {robot.target} in {took:.1f}s")
                self._done(robot, ok=True)
            else:
                # Nav2 could not get there this time -- almost always the peer is
                # in the way. Let go of the waypoint and try again shortly.
                self.get_logger().info(
                    f"{robot.name}: could not reach {robot.target}, retrying"
                )
                self._done(robot, ok=False)

    def _done(self, robot, ok):
        robot.target = None
        if ok:
            robot.legs += 1
            robot.index += 1
            if robot.index % len(robot.route) == 0:
                robot.laps += 1
                self.get_logger().info(f"{robot.name}: lap {robot.laps} complete")
            robot.enter(DWELL)
        else:
            robot.failed += 1
            robot.enter(DWELL)
            robot.since = self.now() - self.dwell + self.retry_delay

    # ------------------------------------------------------------------

    def stop(self):
        """Cancel anything in flight. Nav2 keeps driving otherwise."""
        with self.lock:
            for robot in self.robots:
                if robot.handle is not None:
                    robot.handle.cancel_goal_async()
                    robot.handle = None
        self.get_logger().info("stopped")

    def report(self):
        self.get_logger().info(f"{'robot':8s} {'legs':>5s} {'laps':>5s} {'retries':>8s}")
        for r in self.robots:
            self.get_logger().info(f"{r.name:8s} {r.legs:5d} {r.laps:5d} {r.failed:8d}")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = FleetManager()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    done = threading.Event()

    def bail(signum, frame):
        if done.is_set():
            return
        done.set()
        node.stop()
        node.report()
        executor.shutdown(timeout_sec=0.0)
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, bail)
    signal.signal(signal.SIGTERM, bail)
    try:
        executor.spin()
    except Exception:
        pass
    finally:
        if not done.is_set():
            node.stop()
            node.report()


if __name__ == "__main__":
    main()
